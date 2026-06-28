#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "bmi088.h"
#include <stdint.h>

/**
 * @brief 惯导实例。
 *
 * 该结构体是上层底盘、云台等模块读取姿态和 IMU 原始量的统一入口。
 * 数据遵循 REP-103 右手坐标系：x forward, y left, z up。
 * 角速度单位为 rad/s，加速度单位为 m/s^2；角度同时提供弧度制和角度制。
 */
typedef struct INS_Instance
{
    struct
    {
        float INS_gyro[3];  // 三轴角速度，单位 rad/s，顺序为 roll/pitch/yaw 对应轴
        float INS_accel[3]; // 三轴加速度，单位 m/s^2，顺序为 x/y/z，已完成安装矩阵旋转和零偏补偿
        float INS_mag[3];   // 三轴磁力计数据，当前 EKF 解算未使用，保留为 0
        float INS_quat[4];  // 姿态四元数，顺序为 w/x/y/z，坐标语义遵循 REP-103
    } INS_data;

    struct
    {
        float INS_angle[3];        // 欧拉角，单位 rad，顺序为 roll/pitch/yaw
        float Yaw_total_angle;     // 展开后的连续 yaw 角，单位 rad
        float INS_angle_deg[3];    // 欧拉角，单位 deg，顺序为 roll/pitch/yaw
        float Yaw_total_angle_deg; // 展开后的连续 yaw 角，单位 deg
    } output;

    BMI088Instance *BMI088; // 当前惯导绑定的 BMI088 实例
    float timing_time;      // 本次解算周期，单位 s
} INS_Instance;

typedef enum
{
    IMU_BIAS_CALIB_STATE_IDLE = 0,
    IMU_BIAS_CALIB_STATE_WAIT_TEMP = 1,
    IMU_BIAS_CALIB_STATE_WARMUP = 2,
    IMU_BIAS_CALIB_STATE_SAMPLE = 3,
    IMU_BIAS_CALIB_STATE_DONE = 4,
    IMU_BIAS_CALIB_STATE_ABORT = 5,
} ImuBiasCalibState_e;

typedef enum
{
    IMU_BIAS_CALIB_ABORT_NONE = 0,
    IMU_BIAS_CALIB_ABORT_DISABLED = 1,
    IMU_BIAS_CALIB_ABORT_USER = 2,
    IMU_BIAS_CALIB_ABORT_BAD_PARAM = 3,
} ImuBiasCalibAbortReason_e;

typedef struct
{
    uint8_t enable;       // 1 允许 Ozone 触发一次 IMU 静态偏置采集
    uint8_t start;        // 上升沿启动；保持为 1 不会重复启动
    uint8_t abort;        // 1 立即中止当前采集
    uint8_t reserved;
    uint32_t run_id;      // 手动设置，用于 CSV 分组
    uint32_t sample_target; // 目标有效样本数；0 使用默认 20000
    uint32_t warmup_ticks;  // SAMPLE 前静止预热 tick；0 使用默认 1000
} ImuBiasCalibCmd_s;

typedef struct
{
    uint32_t run_id;
    uint8_t state;        // ImuBiasCalibState_e
    uint8_t valid;        // 1 表示 done 行可用于拟合
    uint8_t done;         // 1 表示本次采集完成
    uint8_t abort_reason; // ImuBiasCalibAbortReason_e
    uint32_t sample_count;
    uint32_t invalid_count;
    uint32_t sample_target;
    uint32_t warmup_ticks;
    uint32_t warmup_count;
    float sample_time_s;
    float temperature_c;
    float gyro_mean_body_raw[3]; // 安装矩阵后的未扣偏置 gyro，单位 rad/s
    float acc_mean_body_raw[3];  // 安装矩阵后的未扣偏置 acc，单位 m/s^2
    float gyro_std_body_raw[3];
    float acc_std_body_raw[3];
    float gyro_norm_dps;         // 当前 body raw gyro 模长，单位 deg/s
    float acc_norm_mps2;         // 当前 body raw acc 模长，单位 m/s^2
    float old_gyro_offset[3];
    float old_acc_offset[3];
} ImuBiasCalibRecord_s;

extern volatile ImuBiasCalibCmd_s g_imu_bias_calib_cmd;
extern volatile ImuBiasCalibRecord_s g_imu_bias_calib_record;

/**
 * @brief 初始化惯导解算实例。
 * @param bmi088 已注册并完成基础初始化的 BMI088 实例
 * @return 初始化成功返回 INS 实例指针，失败返回 NULL
 */
INS_Instance *INS_Init(BMI088Instance *bmi088);

/**
 * @brief 执行一次 BMI088 + QuaternionEKF 姿态解算。
 *
 * 该函数应由实时任务周期调用。函数内部会：
 * 1. 读取 BMI088 原始数据；
 * 2. 完成安装方向旋转、零偏补偿和加速度低通；
 * 3. 执行 z 轴陀螺运行时零偏检测；
 * 4. 更新 EKF；
 */
void INS_EKF(void);

/**
 * @brief 从 PC 或其他外部入口设置 EKF 核心噪声参数。
 *
 * 参数写入后会自动限幅，避免异常串口数据把滤波器参数写到非法范围。
 *
 * @param q1_pc     姿态状态过程噪声 Q1
 * @param q2_pc     零偏状态过程噪声 Q2
 * @param r_pc      加速度观测噪声 R
 * @param lambda_pc 零偏协方差遗忘因子 lambda
 */
void INS_EKF_SetCoreParams(float q1_pc, float q2_pc, float r_pc, float lambda_pc);

#endif
