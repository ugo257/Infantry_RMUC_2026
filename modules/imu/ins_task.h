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
