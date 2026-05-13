#pragma once
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

/* Forward declarations to avoid depending on low-level headers here */
typedef struct INS_Instance INS_Instance;

typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

typedef enum {
    CHASSIS_ZERO_FORCE = 0,
    CHASSIS_ROTATE,
    CHASSIS_NO_FOLLOW,
    CHASSIS_FOLLOW_GIMBAL_YAW,
    CHASSIS_REVERSE_ROTATE,
    CHASSIS_MECANUM_FORCE,
} chassis_mode_e;

typedef enum {
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_GYRO_MODE,
    GIMBAL_MOTOR_MODE,
    GIMBAL_HOLE_MODE, // 狗洞模式
} gimbal_mode_e;

typedef enum {
    none_version_control = 0,
    version_control,
} nuc_mode_e;

typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;

typedef enum {
    FRICTION_OFF = 0,
    FRICTION_ON,
} friction_mode_e;

typedef enum {
    LOAD_STOP = 0,
    LOAD_REVERSE,
    LOAD_1_BULLET,
    LOAD_BURSTFIRE,
} loader_mode_e;

typedef enum {
    LOAD_UNINIT = 0,
    LOAD_REINIT,
    LOAD_INFRARED_INIT,
} loader_state_e;

typedef enum {
    SUPERCAP_UNUSE = 0,
    SUPERCAP_USE,
} SuperCap_Mode_e;

typedef struct {
    uint32_t send_ID;
    uint32_t receive_ID;
    uint8_t error_state;
    uint16_t p_int;
    uint16_t v_int;
    uint16_t t_int;
    float PMAX;
    float VMAX;
    float TMAX;
    float pos;
    float vel;
    float tor;
    uint8_t MOS_temperature;
    uint8_t temperature;
} DM4310_t;

extern DM4310_t DM4310;

#pragma pack(1)

typedef struct
{
    uint16_t power_buffer;
    float chassis_power;
    uint8_t level;
    uint16_t power_limit;
    uint8_t SuperCap_flag_from_user;
    float vx;
    float vy;
    float wz;
    float offset_angle;
    float gimbal_error_angle;
    chassis_mode_e chassis_mode;
    gimbal_mode_e gimbal_mode;
    uint8_t nuc_enable_flag;
    float vw_set;
    float wz_K;
    uint8_t mecanum_force_enable;
} Chassis_Ctrl_Cmd_s;

typedef struct
{
    uint16_t power_buffer;
    float chassis_power;
    uint8_t level;
    uint16_t power_limit;
    uint8_t SuperCap_flag_from_user;
    uint16_t vx;
    uint16_t vy;
    uint16_t wz;
    uint16_t offset_angle;
    uint16_t gimbal_error_angle;
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s_half_float;

typedef struct
{
    float yaw;
    float pitch;
    float yaw_version;
    float pitch_version;
    float yaw_speedFeed;
    float yaw_kp;
    float yaw_kd;
    float yaw_speedKp;
    float yaw_measure_angle;
    float yaw_measure_gyro;
    float yaw_ref_rad;       // RS485 vision yaw absolute reference, rad
    float yaw_ref_vel_rad_s; // RS485 vision yaw velocity reference, rad/s
    float yaw_ref_acc_rad_s2; // RS485 vision yaw acceleration reference, rad/s^2
    gimbal_mode_e gimbal_mode;
    nuc_mode_e nuc_mode;
    uint8_t ifSpeedFeed;
    uint8_t hole_yaw_ready;
    uint8_t hole_restore_active;
} Gimbal_Ctrl_Cmd_s;

typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    loader_mode_e load_mode_last;
    friction_mode_e friction_mode;
    uint8_t rest_heat;
    uint16_t shooter_heat_cooling_rate;
    uint16_t shooter_referee_heat;
    uint16_t shooter_cooling_limit;
    float shoot_rate;
    float bullet_speed;
    float shoot_aim_angle;
    uint8_t Shoot_Once_Flag;
    int32_t shoot_count;
} Shoot_Ctrl_Cmd_s;

typedef struct
{
    uint8_t ui_send_flag;
    chassis_mode_e chassis_mode;
    uint16_t chassis_attitude_angle;
    friction_mode_e friction_mode;
    uint8_t rune_mode;
    uint8_t SuperCap_mode;
    float supercap_voltage;
    float Chassis_Ctrl_power;
    uint8_t cap_online_flag;
    uint16_t Chassis_power_limit;
    uint16_t Shooter_heat;
    uint8_t robot_level;
    uint16_t Heat_Limit;
    uint16_t nuc_flag;
} UI_Cmd_s;

typedef struct
{
    float real_vx;
    float real_vy;
    float real_wz;
    uint8_t cap_online_flag;
    float cap_voltage;
    uint16_t capget_power_limit;
    float chassis_power_output;
    float chassis_voltage;
    float chassis_imu_data[3];
    float chassis_pitch;
} Chassis_Upload_Data_s;

#pragma pack()

typedef struct
{
    uint8_t enable;              // 1 表示本段数据用于 H 矩阵拟合，0 表示普通 debug 观察
    uint8_t reserved[3];
    uint32_t run_id;             // 每次开始新采集段时手动递增，用于 CSV 分组
} OmniHCalibCmd_s;

typedef struct
{
    uint8_t enable;              // 1 默认启用 LQI 电流接管，0 回退到原速度环
    uint8_t current_offset_enable; // 1 启用静摩擦电流补偿
    uint8_t eso_compensate_enable; // 1 启用 ESO 扰动补偿，0 仅观测
    uint8_t reserved;
    float current_limit;         // 四轮统一 DJI 电流命令限幅，单位 cmd
    float current_rate_limit;    // 四轮统一电流变化率限幅，单位 cmd/s
    float kp_vx;                 // LQI vx 比例增益，SI 单位反馈
    float kp_vy;                 // LQI vy_left 比例增益，SI 单位反馈
    float kp_wz;                 // LQI wz 比例增益，SI 单位反馈
    float ki_vx;                 // LQI vx 积分增益，第一版默认 0
    float ki_vy;                 // LQI vy_left 积分增益，第一版默认 0
    float ki_wz;                 // LQI wz 积分增益，第一版默认 0
    float eso_comp_vx;           // ESO vx 扰动补偿比例，1.0 表示全量补偿
    float eso_comp_vy;           // ESO vy_left 扰动补偿比例，1.0 表示全量补偿
    float eso_comp_wz;           // ESO wz 扰动补偿比例，1.0 表示全量补偿
} ChassisLqiCurrentCmd_s;

typedef enum
{
    OMNI_H_CALIB_WZ_SOURCE_ZERO = 0,       // 零力/电流标定/无跟随时强制 0
    OMNI_H_CALIB_WZ_SOURCE_RAW = 1,        // 直接使用 chassis_cmd_recv.wz 原始字段
    OMNI_H_CALIB_WZ_SOURCE_FOLLOW_YAW = 2, // 使用 offset_angle 经 Chassis_Follow_PID 算出的 wz
    OMNI_H_CALIB_WZ_SOURCE_ROTATE = 3,     // 小陀螺正向自旋内部生成 wz
    OMNI_H_CALIB_WZ_SOURCE_REVERSE = 4,    // 反向自旋内部生成 wz
} OmniHCalibWzSource_e;

typedef struct
{
    uint8_t current_limit_active; // 1 表示本周期触发电流幅值限幅
    uint8_t current_rate_limit_active; // 1 表示本周期触发电流变化率限幅
    uint8_t eso_compensate_enable; // 1 表示当前 ESO 扰动估计参与补偿
    uint8_t reserved;
    float cmd_vx;                // 当前送入 omni 控制器的底盘 x 方向命令
    float cmd_vy_left;           // 当前送入 omni 控制器的左向 y 命令
    float cmd_wz;                // 当前送入 omni 控制器的 yaw 角速度命令
    float eso_comp_vx;           // 当前 vx 轴 ESO 扰动补偿比例
    float eso_comp_vy;           // 当前 vy_left 轴 ESO 扰动补偿比例
    float eso_comp_wz;           // 当前 wz 轴 ESO 扰动补偿比例
    float omni_vx_hat;           // 编码器反解得到的 x 方向速度估计
    float omni_vy_left_hat;      // 编码器反解得到的左向 y 速度估计
    float omni_wz_hat;           // 编码器反解得到的 yaw 速度估计
    float eso_vx_hat;            // ESO x 速度估计，换算回当前底盘速度命令单位
    float eso_vy_left_hat;       // ESO 左向 y 速度估计，换算回当前底盘速度命令单位
    float eso_wz_hat;            // ESO yaw 速度估计，换算回当前底盘 yaw 命令单位
    float eso_dist_vx;           // ESO x 方向扰动估计，单位 m/s^2
    float eso_dist_vy;           // ESO y 方向扰动估计，单位 m/s^2
    float eso_dist_wz;           // ESO yaw 扰动估计，单位 rad/s^2
    float wheel_current_ref_fit_lf; // 乘 wheel_dir 后的左前电流参考
    float wheel_current_ref_fit_rf; // 乘 wheel_dir 后的右前电流参考
    float wheel_current_ref_fit_rb; // 乘 wheel_dir 后的右后电流参考
    float wheel_current_ref_fit_lb; // 乘 wheel_dir 后的左后电流参考
    float real_current_fit_lf;   // 乘 wheel_dir 后的左前实测电流
    float real_current_fit_rf;   // 乘 wheel_dir 后的右前实测电流
    float real_current_fit_rb;   // 乘 wheel_dir 后的右后实测电流
    float real_current_fit_lb;   // 乘 wheel_dir 后的左后实测电流
    float force_cmd_lf;          // 左前轮实际分配轮力，fit 坐标，单位 N
    float force_cmd_rf;          // 右前轮实际分配轮力，fit 坐标，单位 N
    float force_cmd_rb;          // 右后轮实际分配轮力，fit 坐标，单位 N
    float force_cmd_lb;          // 左后轮实际分配轮力，fit 坐标，单位 N
    float slip_prob_lf;          // 左前轮滑移概率
    float slip_prob_rf;          // 右前轮滑移概率
    float slip_prob_rb;          // 右后轮滑移概率
    float slip_prob_lb;          // 左后轮滑移概率
} OmniHCalibRecord_s;

extern volatile OmniHCalibCmd_s g_omni_h_calib_cmd;
extern volatile OmniHCalibRecord_s g_omni_h_calib_record;
extern volatile ChassisLqiCurrentCmd_s g_chassis_lqi_current_cmd;

typedef struct
{
    uint8_t enable;              // 1 进入电流响应标定，0 恢复正常底盘速度环
    uint8_t wheel_index;         // 0xFF 四轮同电流；0~3 单轮 LF/RF/RB/LB
    float current_cmd_fit;       // 标定电流命令，符号按 wheel_dir 修正前的拟合方向
    float current_limit;         // 电流命令硬限幅，默认 3000
} ChassisCurrentForceCalibCmd_s;

typedef struct
{
    uint32_t sample_count;       // 调试采样序号，每执行一次底盘任务递增一次
    uint8_t enable;              // 1 表示当前处于电流响应标定模式
    uint8_t wheel_index;         // 0xFF 四轮同电流；0~3 单轮 LF/RF/RB/LB
    uint8_t valid;               // 1 表示四个底盘轮对象都有效，可用于 CSV 记录
    uint8_t calib_active;        // 1 表示底盘已经切入电流响应标定开环入口
    uint8_t pulse_state;         // 0 idle, 1 wait trigger, 2 active, 3 done, 4 hard timeout
    uint8_t pulse_done;          // 1 表示本次自动脉冲已经结束
    uint8_t hard_timeout;        // 1 表示 1.25 s 硬超时保护曾触发
    uint8_t reserved;
    float current_cmd_fit;       // 标定电流命令，用于拟合
    float current_limit;         // 本次标定实际使用的电流命令限幅
    float current_cmd_motor_lf;  // 实际发给左前 DJI 的原始电流命令
    float current_cmd_motor_rf;  // 实际发给右前 DJI 的原始电流命令
    float current_cmd_motor_rb;  // 实际发给右后 DJI 的原始电流命令
    float current_cmd_motor_lb;  // 实际发给左后 DJI 的原始电流命令
    float real_current;          // 乘 wheel_dir 后四轮反馈电流的平均值
    float real_current_fit_lf;   // 乘 wheel_dir 后的左前反馈电流
    float real_current_fit_rf;   // 乘 wheel_dir 后的右前反馈电流
    float real_current_fit_rb;   // 乘 wheel_dir 后的右后反馈电流
    float real_current_fit_lb;   // 乘 wheel_dir 后的左后反馈电流
    float wheel_speed_fit;       // 四轮方向修正后轮端速度的最大绝对值，用于判断是否打滑
    float wheel_speed_fit_lf;    // 乘 wheel_dir 后的左前轮端速度
    float wheel_speed_fit_rf;    // 乘 wheel_dir 后的右前轮端速度
    float wheel_speed_fit_rb;    // 乘 wheel_dir 后的右后轮端速度
    float wheel_speed_fit_lb;    // 乘 wheel_dir 后的左后轮端速度
    float body_acc_x;            // REP-103 底盘 IMU x 加速度，单位 m/s^2
    float body_acc_y;            // REP-103 底盘 IMU y 加速度，单位 m/s^2
    float body_acc_z;            // REP-103 底盘 IMU z 加速度，单位 m/s^2
    float body_omega_z;          // REP-103 底盘 IMU yaw 轴角速度，单位 rad/s
    float body_roll;             // REP-103 EKF roll，单位 rad
    float body_pitch;            // REP-103 EKF pitch，单位 rad
    float body_yaw;              // REP-103 EKF yaw，单位 rad
    float body_yaw_total;        // EKF 展开 yaw，单位 rad
    float sample_time_s;         // 本帧 debug 的 DWT 时间戳，单位 s
    float trigger_time_s;        // current_cmd_fit 从 0 跳到非零时的 DWT 时间戳，单位 s
    float pulse_elapsed_s;       // 当前脉冲从触发开始经过的时间，单位 s
    float force_raw;             // x 方向力传感器未接入 MCU 时保留为 0
    float force_fit;             // x 方向力传感器未接入 MCU 时保留为 0
} ChassisCurrentForceCalibRecord_s;

extern volatile ChassisCurrentForceCalibCmd_s g_chassis_force_calib_cmd;
extern volatile ChassisCurrentForceCalibRecord_s g_chassis_force_calib_record;
extern const float CHASSIS_FORCE_CALIB_X_CMD_PER_N;
extern const float CHASSIS_FORCE_CALIB_Y_CMD_PER_N;
extern const float CHASSIS_FORCE_CALIB_X_N_PER_CMD;
extern const float CHASSIS_FORCE_CALIB_Y_N_PER_CMD;

#pragma pack(1)

typedef struct
{
    uint16_t yaw;
} Chassis_Upload_Data_s_half_float;

typedef struct
{
    INS_Instance *gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
    uint16_t yaw_ecd;
    uint16_t pitch_ecd;
    float yaw_total_angle;
    float pitch_total_angle;
    float yaw_angle_pidout;
    float yaw_angleP;
    float yaw_angleD;
    float yaw_speedP;
    float speedFeed;
    float pitch_up_motor_pos_rad;
    float pitch_up_motor_vel_rad_s;
    float pitch_down_motor_pos_rad;
    float pitch_down_motor_vel_rad_s;
    float pitch_up_imu_rad;
    float yaw_motor_vel;
} Gimbal_Upload_Data_s;

typedef struct
{
    int shooter_heat_control;
    float shooter_local_heat;
    float loader_angle;
} Shoot_Upload_Data_s;

typedef enum
{
    red = 0,
    blue,
} version_color;

typedef struct
{
    float vx;
    float vy;
    float wz;
    float yaw_control;
    float yaw_angle;
    float yaw_gyro;
    float offset_angle;
    float gimbal_error_angle;
    float gimbal_yaw_signal_sound_angle;
    int32_t shoot_count;
    float nuc_yaw;
    float yaw_vel;              // 视觉提供的yaw速度（用于自瞄前馈）
    float vision_yaw;           // USB视觉解析出的yaw绝对目标角，单位rad
    float vision_yaw_vel;       // USB视觉解析出的yaw目标角速度，单位rad/s
    float vision_yaw_acc;       // USB视觉解析出的yaw目标角加速度，单位rad/s^2
    gimbal_mode_e gimbal_mode;
    chassis_mode_e chassis_mode;
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    nuc_mode_e nuc_mode;
    uint8_t reset_flag;
    uint8_t UI_SendFlag;
    uint8_t superCap_flag;
} Chassis_Ctrl_Cmd_s_uart;

typedef struct
{
    float yaw_motor_single_round_angle;
    float yaw_total_angle;
    float yaw_ecd;
    float chassis_pitch_angle;
    float chassis_yaw_gyro;
    float initial_speed;
    float yaw_angle_pidout;
    float yaw_motor_vel;
    version_color color;
    uint8_t hole_yaw_ready;
} Chassis_Upload_Data_s_uart;

typedef struct
{
    uint8_t eee;
} UI_Upload_Data_s;

#pragma pack()

#define Chassis_Ctrl_Cmd_s_uart_size   sizeof(Chassis_Ctrl_Cmd_s_uart)
#define Chassis_Upload_Data_s_uart_size sizeof(Chassis_Upload_Data_s_uart)

extern uint8_t nuc_rx[8];

#endif // ROBOT_TYPES_H
