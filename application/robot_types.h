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
    CHASSIS_DIRECT,
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

#define VISION_TX_MODE_DEFAULT 1u
#define VISION_TX_MODE_KEY_B   2u
#define VISION_TX_MODE_KEY_F   3u

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
    uint16_t supercap_power_limit;
    uint8_t SuperCap_flag_from_user;
    float vx;
    float vy;
    float wz;
    float offset_angle;
    float gimbal_error_angle;
    float yaw_control;
    float gimbal_yaw_total_angle;
    float gimbal_yaw_signal_sound_angle;
    chassis_mode_e chassis_mode;
    gimbal_mode_e gimbal_mode;
    uint8_t nuc_enable_flag;
    float vw_set;
    float wz_K;
    uint8_t mecanum_force_enable;
    uint8_t hole_offset_yaw_takeover;
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
    float offset_angle;      // chassis offset angle, deg
    float yaw_ref_rad;       // RS485 vision yaw absolute reference, rad
    float yaw_ref_vel_rad_s; // RS485 vision yaw velocity reference, rad/s
    float yaw_ref_acc_rad_s2; // RS485 vision yaw acceleration reference, rad/s^2
    float pitch_ref_rad;       // RS485 vision pitch absolute reference, rad
    float pitch_ref_vel_rad_s; // RS485 vision pitch velocity reference, rad/s
    float pitch_ref_acc_rad_s2; // RS485 vision pitch acceleration reference, rad/s^2
    int32_t vision_frame_seq;  // 视觉有效帧序号，用于自瞄包间外推新包检测
    float vision_period_s;     // 视觉有效帧平均周期，单位 s
    gimbal_mode_e gimbal_mode;
    nuc_mode_e nuc_mode;
    uint8_t ifSpeedFeed;
    uint8_t hole_yaw_ready;
    uint8_t hole_restore_active;
    uint8_t hole_offset_yaw_takeover;
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
    float pitch_actual_angle_deg; // UI地面轨迹线使用的云台pitch角，单位deg
    friction_mode_e friction_mode;
    uint8_t rune_mode;
    uint8_t SuperCap_mode;
    float supercap_voltage;
    float Chassis_Ctrl_power;
    uint8_t cap_online_flag;
    uint8_t cap_ready_flag;
    uint8_t cap_active_flag;
    uint8_t hole_mode;
    uint8_t vision_tx_mode;
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
    uint8_t cap_ready_flag;
    uint8_t cap_active_flag;
    float cap_voltage;
    uint16_t capget_power_limit;
    float chassis_power_output;
    float chassis_voltage;
    float chassis_imu_data[3];
    float chassis_pitch;
    float speed[4];
    float current[4];
    float super_cap_power;
} Chassis_Upload_Data_s;

#pragma pack()

typedef struct
{
    uint8_t enable;              // 1 允许 H 矩阵自动标定入口接管底盘速度环
    uint8_t start;               // 上升沿启动一次 vx/vy_left/wz 自动激励序列
    uint8_t abort;               // 1 立即中止并清零四轮参考
    uint8_t reserved;
    uint32_t run_id;             // 每次开始新采集段时手动递增，用于 CSV 分组
    float amp_vx;                // vx 激励幅值，沿用底盘速度命令单位
    float amp_vy_left;           // vy_left 激励幅值，沿用底盘速度命令单位
    float amp_wz;                // wz 激励幅值，沿用底盘 yaw 命令单位
    uint32_t zero_ticks;         // 每段开始的静止采样 tick 数，1 tick = ChassisTask 周期
    uint32_t ramp_ticks;         // 斜坡上升/下降 tick 数
    uint32_t hold_ticks;         // 满幅保持 tick 数
    uint32_t repeat_count;       // 完整 vx/vy_left/wz 正反向序列重复次数
    float wheel_speed_limit;     // 预测轮端速度安全限幅，单位 deg/s，<=0 使用默认值
} OmniHCalibCmd_s;

typedef enum
{
    OMNI_H_CALIB_STATE_IDLE = 0,
    OMNI_H_CALIB_STATE_ZERO = 1,
    OMNI_H_CALIB_STATE_RAMP_UP = 2,
    OMNI_H_CALIB_STATE_HOLD = 3,
    OMNI_H_CALIB_STATE_RAMP_DOWN = 4,
    OMNI_H_CALIB_STATE_DONE = 5,
    OMNI_H_CALIB_STATE_ABORT = 6,
    OMNI_H_CALIB_STATE_FAULT = 7,
} OmniHCalibState_e;

typedef enum
{
    OMNI_H_CALIB_AXIS_VX = 0,
    OMNI_H_CALIB_AXIS_VY_LEFT = 1,
    OMNI_H_CALIB_AXIS_WZ = 2,
} OmniHCalibAxis_e;

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

typedef struct
{
    uint32_t sample_count;       // 调试采样序号，正常/失效路径都会递增
    uint8_t valid;               // 1 表示本帧来自正常 omni/LQI 输出，0 表示零力/标定等旁路
    uint8_t chassis_mode;        // chassis_mode_e
    uint8_t lqi_active;          // 1 表示当前 raw current LQI 接管生效
    uint8_t current_offset_enable; // 1 表示本帧 force->current 含静摩擦 offset
    uint8_t limit_active;        // omni 输出总限幅标志
    uint8_t current_limit_active; // 电流幅值限幅标志
    uint8_t power_limit_active;  // 功率限幅/功率缩放标志
    uint8_t reachable_vxy_valid; // 1 表示已缓存当前方向纯平移最大可达速度
    uint8_t reachable_wz_valid;  // 1 表示已缓存纯自旋最大可达 wz
    uint8_t rotate_strategy_active; // 1 表示本帧实际启用 rotate 60% vxy 分配
    uint8_t reserved;
    float dt_s;                  // 本周期 ChassisTask dt，单位 s
    float current_limit;         // 本周期统一电流限幅，单位 cmd
    float power_vxy_scale;       // 功率约束下 vx/vy 缩放
    float power_wz_scale;        // 功率约束下 wz 缩放
    float power_effective_limit_w; // 实际功率预算，单位 W
    float power_real_w;          // supercap 反馈真实底盘功率，单位 W
    float reachable_vxy_max;     // 当前缓存方向纯平移最大可达速度幅值
    float reachable_wz_max;      // 当前缓存纯自旋最大可达 wz 幅值
    float reachable_vxy_dir_x;   // vxy 最大可达缓存对应的 vx 单位方向
    float reachable_vxy_dir_y;   // vxy 最大可达缓存对应的 vy_left 单位方向
    float cmd_vx_pre_slew;       // 命令系 vx，斜率限制前
    float cmd_vy_pre_slew;       // 命令系 vy，斜率限制前，右向为正
    float cmd_wz_pre_slew;       // 命令系 wz，斜率限制前
    float cmd_vx;                // 命令系 vx，斜率限制后
    float cmd_vy;                // 命令系 vy，斜率限制后，右向为正
    float cmd_wz;                // 命令系 wz，斜率限制后
    float body_vx;               // 底盘系 vx
    float body_vy_right;         // 底盘系 vy，右向为正
    float omni_vy_left;          // 送入 omni 控制器的 vy，左向为正
    float nu_ref[3];             // vx, vy_left, wz
    float nu_hat[3];             // vx, vy_left, wz
    float lqi_error[3];          // vx, vy_left, wz
    float wrench_des[3];         // vx, vy_left, wz 期望合力/力矩
    float wrench_act[3];         // vx, vy_left, wz 实际可达合力/力矩
    float wrench_err[3];         // wrench_des - wrench_act
    float force_to_cmd_pos[4];   // LF, RF, RB, LB 正向 cmd/N
    float force_to_cmd_neg[4];   // LF, RF, RB, LB 负向 cmd/N
    float force_bound_pos[4];    // LF, RF, RB, LB 正向可用轮力，单位 N
    float force_bound_neg_abs[4]; // LF, RF, RB, LB 负向可用轮力绝对值，单位 N
    float force_cmd[4];          // LF, RF, RB, LB 实际分配轮力，单位 N
    float wheel_current_ref[4];  // LF, RF, RB, LB 发送给 DJI 的电流命令
    float real_current_fit[4];   // LF, RF, RB, LB 乘 wheel_dir 后的反馈电流
    float wheel_speed_fit[4];    // LF, RF, RB, LB 乘 wheel_dir 后的轮端速度
    float slip_prob[4];          // LF, RF, RB, LB 滑移概率
} ChassisOmniForceDebug_s;

typedef struct
{
    uint16_t power_limit;        // 最近一次用于匹配/保存的功率限制
    uint8_t load_valid;          // 1 表示本次 power_limit 命中了有效 flash slot
    uint8_t pending_save;        // 1 表示已有新搜索结果等待写回 flash
    uint8_t last_write_ok;       // 最近一次写 flash 是否成功
    uint8_t record_valid;        // 1 表示上电读取到的 flash 记录 CRC/version 有效
    uint8_t slot_index;          // 最近命中的 slot；0xFF 表示未命中
    uint8_t reserved;
    uint32_t write_count;        // 本次上电后成功写 flash 次数
    uint32_t crc_fail_count;     // 上电读取记录 CRC/version 失败次数
} ChassisReachableFlashDebug_s;

typedef struct
{
    uint32_t sample_count;       // Ozone/CSV 采样序号
    uint32_t run_id;             // 与 g_omni_h_calib_cmd.run_id 对齐
    uint32_t segment_index;      // 自动序列段序号
    uint32_t state_tick;         // 当前 state 内 tick 计数
    uint8_t enable;              // 1 表示 H 标定入口当前接管或保持接管
    uint8_t state;               // OmniHCalibState_e
    uint8_t axis;                // OmniHCalibAxis_e
    int8_t sign;                 // 当前轴激励方向：+1 / -1 / 0
    uint8_t valid;               // 1 表示四个轮电机对象都有效
    uint8_t wheel_valid_mask;    // bit0~3 对应 LF/RF/RB/LB 是否有反馈对象
    uint8_t repeat_index;        // 当前重复序号
    uint8_t reserved;
    float sample_time_s;         // DWT 时间戳，单位 s
    float dt_s;                  // 本周期 ChassisTask dt，单位 s
    float cmd_vx;                // 实际送入 H 激励/记录的 vx 命令
    float cmd_vy_left;           // 实际送入 H 激励/记录的 vy_left 命令
    float cmd_wz;                // 实际送入 H 激励/记录的 wz 命令
    float wheel_speed_fit_lf;    // wheel_dir 修正后的左前轮端速度，单位 deg/s
    float wheel_speed_fit_rf;    // wheel_dir 修正后的右前轮端速度，单位 deg/s
    float wheel_speed_fit_rb;    // wheel_dir 修正后的右后轮端速度，单位 deg/s
    float wheel_speed_fit_lb;    // wheel_dir 修正后的左后轮端速度，单位 deg/s
} OmniHCalibRecord_s;

extern volatile OmniHCalibCmd_s g_omni_h_calib_cmd;
extern volatile OmniHCalibRecord_s g_omni_h_calib_record;
extern volatile ChassisLqiCurrentCmd_s g_chassis_lqi_current_cmd;
extern volatile ChassisOmniForceDebug_s g_chassis_omni_force_debug;
extern volatile ChassisReachableFlashDebug_s g_chassis_reachable_flash_debug;

typedef enum
{
    CHASSIS_FORCE_CALIB_AUTO_IDLE = 0,
    CHASSIS_FORCE_CALIB_AUTO_ARM = 1,
    CHASSIS_FORCE_CALIB_AUTO_BASELINE = 2,
    CHASSIS_FORCE_CALIB_AUTO_PULSE_ACTIVE = 3,
    CHASSIS_FORCE_CALIB_AUTO_COAST_WAIT = 4,
    CHASSIS_FORCE_CALIB_AUTO_NEXT_TRIAL = 5,
    CHASSIS_FORCE_CALIB_AUTO_FINISHED = 6,
    CHASSIS_FORCE_CALIB_AUTO_ABORT = 7,
} ChassisForceCalibAutoState_e;

typedef enum
{
    CHASSIS_FORCE_CALIB_ABORT_NONE = 0,
    CHASSIS_FORCE_CALIB_ABORT_MOTOR_INVALID = 1,
    CHASSIS_FORCE_CALIB_ABORT_CURRENT_LIMIT = 2,
    CHASSIS_FORCE_CALIB_ABORT_HARD_TIMEOUT = 3,
    CHASSIS_FORCE_CALIB_ABORT_ILLEGAL_STATE = 4,
    CHASSIS_FORCE_CALIB_ABORT_INVALID_WHEEL = 5,
} ChassisForceCalibAbortReason_e;

typedef struct
{
    uint8_t enable;              // 1 进入电流响应标定，0 恢复正常底盘速度环
    uint8_t wheel_index;         // 0~3 单轮 LF/RF/RB/LB；0xFF 保留为四轮全流程
    float current_cmd_fit;       // 标定电流命令，符号按 wheel_dir 修正前的拟合方向
    float current_limit;         // 电流命令安全限幅，默认 5000，硬上限 6000
} ChassisCurrentForceCalibCmd_s;

typedef struct
{
    uint32_t sample_count;       // 调试采样序号，每执行一次底盘任务递增一次
    uint32_t trial_id;           // 自动标定 trial 序号：单轮 0~19，0xFF 全轮 0~79
    uint8_t enable;              // 1 表示当前处于电流响应标定模式
    uint8_t valid;               // 1 表示四个底盘轮对象都有效，可用于 CSV 记录
    uint8_t calib_active;        // 1 表示底盘已经切入电流响应标定开环入口
    uint8_t auto_running;        // 1 表示自动流程正在 baseline/pulse/coast/推进
    uint8_t auto_done;           // 1 表示 80 次自动 trial 已完成
    uint8_t auto_abort;          // 1 表示自动流程因安全条件中止
    uint8_t auto_state;          // ChassisForceCalibAutoState_e
    uint8_t wheel_index;         // 当前 trial 轮序：0~3 LF/RF/RB/LB
    int8_t direction;            // 当前 trial 方向：+1 正向，-1 反向，0 无效
    uint8_t current_level_index; // 当前电流档位序号
    uint8_t repeat_index;        // 当前重复序号
    uint8_t pulse_done;          // 1 表示本次自动脉冲已经结束
    uint8_t hard_timeout;        // 1 表示 1.25 s 硬超时保护曾触发
    uint8_t abort_reason;        // ChassisForceCalibAbortReason_e
    uint8_t reserved[2];
    float sample_time_s;         // 本帧 debug 的 DWT 时间戳，单位 s
    float state_elapsed_s;       // 当前自动状态已持续时间，单位 s
    float trigger_time_s;        // 本次 pulse 开始时间戳，单位 s
    float pulse_elapsed_s;       // 当前脉冲从触发开始经过的时间，单位 s
    float current_cmd_fit;       // 当前 trial 目标 fit 电流，单位 cmd
    float applied_current_cmd_fit; // 本周期实际施加 fit 电流，baseline/coast 为 0
    float current_limit;         // 本次标定实际使用的电流命令限幅
    float current_cmd_motor_lf;  // 实际发给左前 DJI 的原始电流命令
    float current_cmd_motor_rf;  // 实际发给右前 DJI 的原始电流命令
    float current_cmd_motor_rb;  // 实际发给右后 DJI 的原始电流命令
    float current_cmd_motor_lb;  // 实际发给左后 DJI 的原始电流命令
    float real_current_selected; // 当前 trial 目标轮的 fit 反馈电流
    float real_current_fit_lf;   // 乘 wheel_dir 后的左前反馈电流
    float real_current_fit_rf;   // 乘 wheel_dir 后的右前反馈电流
    float real_current_fit_rb;   // 乘 wheel_dir 后的右后反馈电流
    float real_current_fit_lb;   // 乘 wheel_dir 后的左后反馈电流
    float wheel_speed_fit_max_abs; // 四轮方向修正后轮端速度最大绝对值
    float wheel_speed_fit_lf;    // 乘 wheel_dir 后的左前轮端速度
    float wheel_speed_fit_rf;    // 乘 wheel_dir 后的右前轮端速度
    float wheel_speed_fit_rb;    // 乘 wheel_dir 后的右后轮端速度
    float wheel_speed_fit_lb;    // 乘 wheel_dir 后的左后轮端速度
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
    uint8_t hole_pitch_zero_torque;
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
    float vision_pitch;         // USB视觉解析出的pitch绝对目标角，单位rad
    float vision_pitch_vel;     // USB视觉解析出的pitch目标角速度，单位rad/s
    float vision_pitch_acc;     // USB视觉解析出的pitch目标角加速度，单位rad/s^2
    int32_t vision_frame_seq;    // USB视觉有效帧序号，用于下板检测新视觉包
    float vision_period_s;       // USB视觉有效帧平均周期，单位s
    gimbal_mode_e gimbal_mode;
    chassis_mode_e chassis_mode;
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    nuc_mode_e nuc_mode;
    uint8_t vision_tx_mode;
    uint8_t reset_flag;
    uint8_t UI_SendFlag;
    uint8_t superCap_flag;
    uint8_t hole_offset_yaw_takeover;
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
    float chassis_wz;
    float lf_speed;
    float rf_speed;
    float rb_speed;
    float lb_speed;
    float lf_current;
    float rf_current;
    float rb_current;
    float lb_current;
    float chassis_total_power;
    version_color color;
    uint8_t hole_yaw_ready;
    uint8_t hole_offset_yaw_takeover;
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
