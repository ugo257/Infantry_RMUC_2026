// app
#include "math.h"
#include "robot_board.h"
#include "robot_params.h"
#include "robot_types.h"
#include "robot_cmd.h"
#include "omni_UI.h"
// module
#include "QuaternionEKF.h"
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "referee_UI.h"
#include "referee_init.h"
#include "tool.h"
#include "super_cap.h"
#include "rs485_cmd.h"
#include "tick_timer.h"
#include "AHRS_middleware.h"
#include "rm_referee.h"
#include "crc_ref.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "vofa.h"
//看门狗
#include "iwdg.h"
// 图传协议中C档的值为0，因此丢控不能再靠开关字段是否为0判断。
#define RC_LOST (!RemoteControlIsOnline())
#define CHASSIS_DR16_VXY_SCALE 12.0f
#define CHASSIS_DR16_STALE_TICKS 100u
#define CHASSIS_DR16_YAW_STICK_FULL 660.0f
#define CHASSIS_DR16_YAW_ANGLE_MAX_DEFAULT_DEG 35.0f
#define CHASSIS_DR16_YAW_ANGLE_KP_DEFAULT 35.0f
#define CHASSIS_DR16_YAW_WZ_MAX_DEFAULT 1200.0f
#define CHASSIS_DR16_YAW_CMD_TO_RADPS 1.3e-3f
#define CHASSIS_DR16_YAW_SETTLE_DEG 1.0f
#define VISION_RX_PERIOD_MIN_S      0.0001f
#define VISION_RX_PERIOD_MAX_S      0.1000f
#define VISION_RX_PERIOD_AVG_ALPHA  0.20f
#define VISION_YAW_ACC_COMP_ALPHA_DEFAULT 0.1f

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#if PITCH_FEED_TYPE                                                  // Pitch电机反馈数据源为陀螺仪
#define PTICH_HORIZON_ANGLE 0                                        // PITCH水平时电机的角度
#if PITCH_ECD_UP_ADD
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#else
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
#else           // PITCH电机反馈数据源为编码器

#define PTICH_HORIZON_ANGLE    (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // PITCH水平时电机的角度,0-360
#define PITCH_LIMIT_ANGLE_UP   (PITCH_POS_MAX_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (PITCH_POS_MIN_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
//发给邓浩宇的云台数据
static float yaw_reverse,vision_pitch_gyro_rad_s,vision_pitch_rad,vision_yaw_gyro_rad_s;
//接收宇哥的数据
typedef struct {
    uint32_t online_frame_cnt;
    uint32_t len_fail_cnt;
    uint32_t head_fail_cnt;
    uint32_t crc_ok_cnt;
    uint32_t crc_fail_cnt;
    int32_t frame_seq;
    float period_s;
    uint8_t last_mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
} VisionRxDebug_s;

volatile VisionRxDebug_s g_vision_rx_debug;
/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef CHASSIS_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
#endif
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Ctrl_Cmd_s_uart chassis_cmd_send_uart;
static Chassis_Upload_Data_s_uart chassis_fetch_data_uart;
Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;
extern referee_info_t *referee_data_for_ui;
static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
HostInstance *host_instance; // 上位机接口
RS485Instance *rs485_master_instance; // 接口
RS485Instance *rs485_slaver_instance; // RS485从机接口
#ifdef GIMBAL_BOARD
static RobotCMD485TxState_s ctrl_tx_state;
#endif
#ifdef CHASSIS_BOARD
static RobotCMD485TxState_s upload_tx_state;
#endif
extern INS_Instance *INS;
// 这里的四元数以wxyz的顺序
uint8_t vision_send_data[43]; // 给视觉上位机发送的数据
uint8_t fifo_pack[NUC_RECV_SIZE]; // NUC接收缓冲区
static Publisher_t *gimbal_cmd_pub  ;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息
#ifdef CHASSIS_BOARD
static Publisher_t *ui_cmd_pub;        // UI控制消息发布者
static Subscriber_t *ui_feed_sub;      // UI反馈信息订阅者
static UI_Cmd_s ui_cmd_send;           // 传递给UI的控制信息
#endif
static Robot_Status_e robot_state; // 机器人整体工作状态
static referee_info_t *referee_data; // 用于获取裁判系统的数据
uint8_t UI_SendFlag = 1; // UI发送标志位
#define MECANUM_FORCE_UI_FLAG_BIT 0x80u
#define UI_SEND_FLAG_MASK         0x7Fu
#define ROBOT_CMD_SUPERCAP_POWER_BONUS_W 100U
#define ROBOT_CMD_17MM_HEAT_PER_BULLET 10U
#define ROBOT_CMD_17MM_HEAT_RESERVED_BULLETS 5U
#define ROBOT_CMD_17MM_HEAT_STOP_MARGIN \
    (ROBOT_CMD_17MM_HEAT_PER_BULLET * ROBOT_CMD_17MM_HEAT_RESERVED_BULLETS)
uint8_t auto_rune; // 自瞄打符标志位
volatile gimbal_mode_e gimbal_last_mode=GIMBAL_ZERO_FORCE;
float fire_advice;
float vision_yaw_vel = 0; // 视觉提供的yaw速度前馈
// #define Chassis_Ctrl_Cmd_s_uart_size sizeof(Chassis_Ctrl_Cmd_s_uart)
// #define Chassis_Upload_Data_s_uart_size sizeof(Chassis_Upload_Data_s_uart)
uint8_t SuperCap_flag_from_user = 0; // 超电标志位
volatile uint8_t rc_update_flag = 0;//遥控器数据更新标志位（防止同一个周期多次触发）
static uint8_t mecanum_force_ctrl_enable = 0u; // 麦轮力控独立开关（与 chassis_mode 解耦）
// 图传遥控离散按键是瞬时状态，这两个变量保存上升沿触发后的锁存开关状态。
static uint8_t remote_nuc_toggle_enable = 0u;
static uint8_t remote_rotate_enable = 0u;
static uint8_t mouse_key_z_count_last = 0u;

#if defined(CHASSIS_DR16_DIRECT_CONTROL)
volatile float chassis_dr16_yaw_angle_max_deg = CHASSIS_DR16_YAW_ANGLE_MAX_DEFAULT_DEG;
volatile float chassis_dr16_yaw_angle_kp = CHASSIS_DR16_YAW_ANGLE_KP_DEFAULT;
volatile float chassis_dr16_yaw_wz_max = CHASSIS_DR16_YAW_WZ_MAX_DEFAULT;
volatile float chassis_dr16_yaw_angle_ref_watch = 0.0f;
volatile float chassis_dr16_yaw_angle_est_watch = 0.0f;
volatile float chassis_dr16_yaw_angle_error_watch = 0.0f;
volatile float chassis_dr16_yaw_wz_cmd_watch = 0.0f;
volatile uint8_t chassis_dr16_yaw_angle_active_watch = 0U;
#endif

static uint16_t RobotCMDGet17mmHeatStopHeat(uint16_t heat_limit)
{
    return (heat_limit > ROBOT_CMD_17MM_HEAT_STOP_MARGIN) ?
        (uint16_t)(heat_limit - ROBOT_CMD_17MM_HEAT_STOP_MARGIN) :
        0u;
}

static uint8_t RobotCMD17mmHeatReserveReached(uint16_t heat_limit, uint16_t shooter_heat)
{
    // heat_limit == 0 means referee data is not ready; do not block shooting on boot defaults.
    if (heat_limit == 0u) {
        return 0u;
    }

    return (shooter_heat >= RobotCMDGet17mmHeatStopHeat(heat_limit)) ? 1u : 0u;
}

static void RobotCMDStopShootingLoadMode(void)
{
    if (shoot_cmd_send.load_mode == LOAD_1_BULLET ||
        shoot_cmd_send.load_mode == LOAD_BURSTFIRE) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

#ifdef CHASSIS_BOARD
// 1.0完全跟随视觉acc，0.0保持当前滤波输出；可在debug里在线修改。
volatile float g_vision_yaw_acc_comp_alpha = VISION_YAW_ACC_COMP_ALPHA_DEFAULT;
volatile float g_vision_yaw_acc_raw = 0.0f;
volatile float g_vision_yaw_acc_filt = 0.0f;
volatile int32_t g_vision_yaw_acc_filter_frame_seq = 0;
static uint8_t vision_yaw_acc_filter_primed = 0u;
static int32_t vision_yaw_acc_filter_last_seq = 0;

static float RobotCMDClamp01(float value)
{
    if (isfinite(value) == 0) {
        return VISION_YAW_ACC_COMP_ALPHA_DEFAULT;
    }
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

static void RobotCMDFilterChassisVisionYawAcc(void)
{
    float alpha = RobotCMDClamp01(g_vision_yaw_acc_comp_alpha);
    const float recv_acc = chassis_rs485_recv.vision_yaw_acc;

    g_vision_yaw_acc_comp_alpha = alpha;

    if (vision_yaw_acc_filter_primed == 0u ||
        chassis_rs485_recv.vision_frame_seq != vision_yaw_acc_filter_last_seq ||
        fabsf(recv_acc - g_vision_yaw_acc_filt) > 1.0e-6f) {
        g_vision_yaw_acc_raw = recv_acc;
        vision_yaw_acc_filter_last_seq = chassis_rs485_recv.vision_frame_seq;
        g_vision_yaw_acc_filter_frame_seq = vision_yaw_acc_filter_last_seq;

        if (vision_yaw_acc_filter_primed == 0u ||
            isfinite(g_vision_yaw_acc_filt) == 0) {
            g_vision_yaw_acc_filt = g_vision_yaw_acc_raw;
            vision_yaw_acc_filter_primed = 1u;
        }
    }

    if (isfinite(g_vision_yaw_acc_raw) == 0) {
        g_vision_yaw_acc_raw = 0.0f;
    }
    if (isfinite(g_vision_yaw_acc_filt) == 0) {
        g_vision_yaw_acc_filt = g_vision_yaw_acc_raw;
    }

    g_vision_yaw_acc_filt += alpha * (g_vision_yaw_acc_raw - g_vision_yaw_acc_filt);
    chassis_rs485_recv.vision_yaw_acc = g_vision_yaw_acc_filt;
}

static uint16_t RobotCMDGetChassisEffectivePowerLimit(uint16_t referee_limit, uint8_t supercap_flag)
{
    uint32_t effective_limit = referee_limit;

    if (supercap_flag == SUPERCAP_USE) {
        effective_limit += ROBOT_CMD_SUPERCAP_POWER_BONUS_W;
    }
    if (effective_limit > 0xFFFFU) {
        effective_limit = 0xFFFFU;
    }

    return (uint16_t)effective_limit;
}
#endif
#ifdef GIMBAL_BOARD
static uint8_t robot_cmd_hole_restore_active = 0u;
static uint8_t robot_cmd_hole_restore_force_follow = 0u;
static uint8_t robot_cmd_hole_restore_force_key_c_count = 0u;
static uint8_t robot_cmd_hole_restore_force_remote_rotate = 0u;
static uint8_t robot_cmd_hole_yaw_control_unlocked = 0u;
static uint8_t robot_cmd_hole_restore_yaw_aligned = 0u;
static gimbal_mode_e robot_cmd_last_gimbal_mode = GIMBAL_ZERO_FORCE;
static void RobotCMDVisionRxDecode(HostInstance *instance, const uint8_t *buf, uint16_t len);
static uint8_t vision_auto_load_last_mode = 0u;
static uint8_t vision_auto_load_wait_shot = 0u;
static uint8_t vision_mode2_fire_allowed = 0u;
static int32_t vision_auto_load_mode2_shoot_count = 0;
#ifndef ROBOT_CMD_ALLOW_GIMBAL_HOLE_MODE
#define ROBOT_CMD_ALLOW_GIMBAL_HOLE_MODE 1
#endif
#define VISION_AUTO_FIRE_MODE 2u
static uint8_t vision_tx_mode = VISION_TX_MODE_DEFAULT;
static uint8_t vision_tx_key_b_count_last = 0u;
static uint8_t vision_tx_key_f_count_last = 0u;
#endif

void RobotCMDSetMecanumForceCtrl(uint8_t enable)
{
    mecanum_force_ctrl_enable = (enable != 0u) ? 1u : 0u;
}

uint8_t RobotCMDGetMecanumForceCtrl(void)
{
    return mecanum_force_ctrl_enable;
}
// float imu_angle[3];
// float imu_gyro[3];
static PIDInstance YAW_version_PID = {
    .Kp            = 0.0037,//0.02,//0.033,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 0.00055,//0.001,//0.003, // 0.0,  // 0.07,  // 0
    .DeadBand      = 0.75,  // 0.75,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 3000,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement|PID_OutputFilter,
    .MaxOut        = 0.1,//0.05,//0.045,
    .Output_LPF_RC=0.7,
};
//欧拉角到四元数转换
void EularAngleToQuaternion(float Y, float P, float R, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    // Y /= 57.295779513f;
    // P /= 57.295779513f;
    // R /= 57.295779513f;
    cosPitch = arm_cos_f32(P / 2);
    cosYaw   = arm_cos_f32(Y / 2);
    cosRoll  = arm_cos_f32(R / 2);
    sinPitch = arm_sin_f32(P / 2);
    sinYaw   = arm_sin_f32(Y / 2);
    sinRoll  = arm_sin_f32(R / 2);
q[0] = cosYaw * cosPitch * cosRoll + sinYaw * sinPitch * sinRoll;
q[1] = cosYaw * cosPitch * sinRoll - sinYaw * sinPitch * cosRoll;
q[2] = cosYaw * sinPitch * cosRoll + sinYaw * cosPitch * sinRoll;
q[3] = sinYaw * cosPitch * cosRoll - cosYaw * sinPitch * sinRoll;
}

// 将 float 拆分为 4 个 uint8_t 字节（手动处理高低位）
void float_to_uint8_manual(float value, uint8_t *bytes) {
    uint32_t as_int;
    memcpy(&as_int, &value, sizeof(float)); // 将 float 转换为 uint32_t

    *bytes = (uint8_t)(as_int & 0xFF);         // 第 1 个字节（低位）
    *(bytes+1) = (uint8_t)((as_int >> 8) & 0xFF);  // 第 2 个字节
    *(bytes+2) = (uint8_t)((as_int >> 16) & 0xFF); // 第 3 个字节
    *(bytes+3) = (uint8_t)((as_int >> 24) & 0xFF); // 第 4 个字节（高位）
}

// 将 4 个 uint8_t 字节解析回 float（手动处理高低位）
float uint8_to_float_manual(uint8_t *bytes) {
    uint32_t as_int = 0;

    as_int |= ((uint32_t)bytes[0]);
    as_int |= ((uint32_t)bytes[1] << 8);
    as_int |= ((uint32_t)bytes[2] << 16);
    as_int |= ((uint32_t)bytes[3] << 24);

    float value;
    memcpy(&value, &as_int, sizeof(float)); // 将 uint32_t 转换回 float
    return value;
}

// 新增字段只改对应方向表的一行：字段名、第一帧头、周期tick。
// 第二帧头按表中顺序自动从0x01递增。
// period_ticks: 1=1000Hz, 2=500Hz, 10=100Hz, 20=50Hz, 50=20Hz。
static const RobotCMD485Signal_s robot_cmd_ctrl_signals[] = {
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, reset_flag,         0xA1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vx,                 0xA1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vy,                 0xA1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, wz,                 0xA1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, yaw_control,        0xA1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, yaw_gyro,           0xA1u, 1u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, yaw_angle,          0xA1u, 1u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, gimbal_yaw_signal_sound_angle, 0xA1u, 1u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, nuc_yaw,            0xA1u, 0u),
    // ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, shoot_count,        0xA1u, 1u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, gimbal_mode,        0xA1u, 13u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, chassis_mode,       0xA1u, 13u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, shoot_mode,         0xA1u, 13u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, load_mode,          0xA1u, 13u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, friction_mode,      0xA1u, 13u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, nuc_mode,           0xA1u, 13u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vision_tx_mode,     0xA1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, UI_SendFlag,        0xA1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, superCap_flag,      0xA1u, 13u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vision_yaw,         0xA1u, 7u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vision_yaw_vel,     0xA1u, 7u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vision_yaw_acc,     0xA1u, 7u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vision_pitch,       0xA1u, 7u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vision_pitch_vel,   0xA1u, 7u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vision_pitch_acc,   0xA1u, 7u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, hole_offset_yaw_takeover, 0xA1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vision_frame_seq,   0xA1u, 7u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vision_period_s,    0xA1u, 7u),
    ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart,reset_flag  , 0xa1,13u)
};

// 控制分项信号表项数量；每个表项对应一种控制单项帧，帧长由payload类型决定。
#define ROBOT_CMD_CTRL_SIGNAL_COUNT ((uint8_t)(sizeof(robot_cmd_ctrl_signals) / sizeof(robot_cmd_ctrl_signals[0])))

static const RobotCMD485Signal_s robot_cmd_upload_signals[] = {
    ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart, chassis_pitch_angle,          0xB1u, 1u),
    ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart, initial_speed,                0xB1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart, color,                        0xB1u, 50u),
    ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart, hole_yaw_ready,               0xB1u, 10u),
    ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart, chassis_yaw_gyro,             0xB1u, 1u),
    ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart, yaw_motor_vel,                0xB1u, 1u),
    ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart, hole_offset_yaw_takeover,     0xB1u, 10u),
    // ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart , lf_speed ,0xb1u,20u),
    // ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart , rf_speed ,0xb1u,20u),
    // ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart , lb_speed ,0xb1u,20u),
    // ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart , rb_speed ,0xb1u,20u),  
    // ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart,lf_current,0xb1u,20u),
    // ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart,rf_current,0xb1u,20u),
    // ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart,lb_current,0xb1u,20u),
    // ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart,rb_current,0xb1u,20u),
    // ROBOT_CMD_485_SIGNAL(Chassis_Upload_Data_s_uart, chassis_total_power, 0xB1u, 20u),
};

#define ROBOT_CMD_UPLOAD_SIGNAL_COUNT ((uint8_t)(sizeof(robot_cmd_upload_signals) / sizeof(robot_cmd_upload_signals[0])))

#ifdef CHASSIS_BOARD
static void RobotCMD485SlaveRecvCallback(RS485Instance *instance, void *arg)
{
    RobotCMD485SendItems(instance,
                         robot_cmd_upload_signals,
                         ROBOT_CMD_UPLOAD_SIGNAL_COUNT,
                         &chassis_fetch_data_uart,
                         &upload_tx_state);
    (void)RobotCMD485RecvItems(instance,
                               robot_cmd_ctrl_signals,
                               ROBOT_CMD_CTRL_SIGNAL_COUNT,
                               &chassis_rs485_recv);
    (void)arg;
}
#endif

void RobotCMDInit()
{
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
#ifdef CHASSIS_BOARD
    
    ui_cmd_pub  = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    ui_feed_sub = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));
    chassis_cmd_send.power_limit=100;//预设功率,防止没有裁判系统车动不了
    chassis_cmd_send.supercap_power_limit=100;
    referee_data = RefereeHardwareInit(&huart6); // 裁判系统初始化,会同时初始化UI
    //初始化双板通信，底盘板作为从机，云台板作为主机，底盘板接收云台板的控制数据并反馈底盘状态给云台板
    rs485_slaver_instance = RobotCMD485InitSlave(&huart1); // 双板通信串口
    RobotCMD485TxStateInit(&upload_tx_state,
                           robot_cmd_upload_signals,
                           ROBOT_CMD_UPLOAD_SIGNAL_COUNT);
    RS485SetRecvCallback(rs485_slaver_instance, RobotCMD485SlaveRecvCallback, NULL);
#if defined(CHASSIS_DR16_DIRECT_CONTROL)
    rc_data = RemoteControlInit(&huart3);
#endif
    // 注册底盘控制的发布者和订阅者
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; //预设底盘模式为跟随云台
#endif

#ifdef GIMBAL_BOARD
#if defined(RC_DR16)
    // DR16/DBUS uses the board's inverted 100 kbit/s receiver on USART3.
    rc_data = RemoteControlInit(&huart3);
#elif defined(RC_VTM)
    rc_data = RemoteControlInit(&huart6);
#endif
#if defined(VISION_USE_VCP)
    HostInstanceConf host_conf0 = {
        .comm_mode = HOST_VCP,
        .RECV_SIZE = NUC_RECV_SIZE,
        .rx_decode_callback = RobotCMDVisionRxDecode,
    };
    host_instance = HostInit(&host_conf0,1000); // 视觉通信串口
#endif
    //初始化双板通信，底盘板作为从机，云台板作为主机，底盘板接收云台板的控制数据并反馈底盘状态给云台板
    rs485_master_instance = RobotCMD485InitMaster(&huart1);// 双板通信串口
    RobotCMD485TxStateInit(&ctrl_tx_state,
                           robot_cmd_ctrl_signals,
                           ROBOT_CMD_CTRL_SIGNAL_COUNT);

#endif

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

#ifdef CHASSIS_BOARD
/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // 2026协议中红方机器人ID为1~99，蓝方机器人ID从101开始；0按红方默认值处理。
    referee_data->referee_id.Robot_Color       = (referee_data->GameRobotState.robot_id >= RobotID_BHero) ? Robot_Blue : Robot_Red;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->GameRobotState.robot_id; // 计算客户端ID
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;          // 计算机器人ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}
#endif

float yaw_control;   // 遥控器YAW自由度输入值
float pitch_control; // 遥控器PITCH自由度输入值，单位为 pitch-up IMU rad
volatile float pitch_up_cmd_lower_limit_rad = 0.0f;
volatile float pitch_up_cmd_upper_limit_rad = 0.0f;
volatile float pitch_up_motor_imu_slope = 1.0060169f;
volatile uint8_t pitch_up_cmd_limit_active = 0U;
volatile float pitch_coupled_imu_lower_limit_rad = 0.0f;
volatile float pitch_coupled_imu_upper_limit_rad = 0.0f;
volatile float pitch_coupled_imu_estimate_rad = 0.0f;
volatile float pitch_coupled_imu_error_rad = 0.0f;
volatile uint8_t pitch_coupled_cmd_limit_active = 0U;

static float RobotCMDWrap180Deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

#if defined(CHASSIS_DR16_DIRECT_CONTROL)
static float RobotCMDClampFloat(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static float RobotCMDFiniteOr(float value, float fallback)
{
    return (isfinite(value) != 0) ? value : fallback;
}
#endif

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 * @todo  将单圈角度修改为-180~180
 *
 */
static void  CalcOffsetAngle()
{
    const INS_Instance *gimbal_imu = gimbal_fetch_data.gimbal_imu_data;
    const float yaw_current_deg =
        (float)gimbal_fetch_data.yaw_motor_single_round_angle;
    const float yaw_forward_deg = YAW_DM_Forward_ANGLE_POS * RAD_TO_ANGLE;
    static float gimbal_yaw_current_angle; // 云台yaw轴当前角度
    static float gimbal_yaw_set_angle;     // 云台yaw轴目标角度

    chassis_cmd_send.offset_angle =
        -RobotCMDWrap180Deg(yaw_current_deg - yaw_forward_deg);
    gimbal_yaw_current_angle =
        (gimbal_imu != NULL) ? gimbal_imu->output.Yaw_total_angle_deg : 0.0f;
    gimbal_yaw_set_angle = yaw_control;
    chassis_cmd_send.gimbal_error_angle =
        gimbal_yaw_set_angle - gimbal_yaw_current_angle;
}

#ifdef CHASSIS_BOARD
static uint8_t robot_cmd_hole_yaw_ready_latched = 0u;

static uint8_t RobotCMDHoleYawOffsetReady(float offset_deg)
{
    return (fabsf(offset_deg - HOLE_YAW_OFFSET_TARGET_DEG) <= HOLE_READY_THRESHOLD_DEG) ? 1u : 0u;
}

static uint8_t RobotCMDUpdateHoleYawReadyLatch(uint8_t hole_mode_active,
                                               uint8_t hole_takeover,
                                               float offset_deg)
{
    if (hole_mode_active == 0u || hole_takeover != 0u) {
        robot_cmd_hole_yaw_ready_latched = 0u;
        return 0u;
    }

    if (RobotCMDHoleYawOffsetReady(offset_deg) != 0u) {
        robot_cmd_hole_yaw_ready_latched = 1u;
    }

    return robot_cmd_hole_yaw_ready_latched;
}

#endif

#ifdef GIMBAL_BOARD
static uint8_t RobotCMDHoleRestoreReleaseReady(void)
{
    const uint8_t down_ready =
        (fabsf(gimbal_fetch_data.pitch_down_motor_pos_rad -
               HOLE_DOWN_RESTORE_TARGET_RAD) <=
         HOLE_DOWN_RESTORE_RELEASE_THRESHOLD_RAD) ? 1u : 0u;

    return down_ready;
}

static uint8_t RobotCMDHoleDownTakeoverReady(void)
{
    return (gimbal_fetch_data.pitch_down_motor_pos_rad <= HOLE_DOWN_TAKEOVER_READY_RAD) ? 1u : 0u;
}

static float RobotCMDGetCurrentYawAngleDeg(void)
{
    const INS_Instance *gimbal_imu = gimbal_fetch_data.gimbal_imu_data;

    return (gimbal_imu != NULL) ?
           gimbal_imu->output.Yaw_total_angle_deg :
           yaw_control;
}

static void RobotCMDHoleYawControlLock(void)
{
    robot_cmd_hole_yaw_control_unlocked = 0u;
}

static void RobotCMDHoleRestoreAlignYawControlOnce(void)
{
    if (robot_cmd_hole_restore_yaw_aligned != 0u) {
        return;
    }

    yaw_control = RobotCMDGetCurrentYawAngleDeg();
    gimbal_cmd_send.yaw = yaw_control;
    robot_cmd_hole_restore_yaw_aligned = 1u;
}

static void RobotCMDHoleRestoreForceYawControl(void)
{
    gimbal_cmd_send.nuc_mode = none_version_control;
    gimbal_cmd_send.yaw = yaw_control;
}

static void RobotCMDUpdateHoleYawControlUnlock(uint8_t hole_mode_active)
{
    if (hole_mode_active == 0u) {
        RobotCMDHoleYawControlLock();
        return;
    }

    if (gimbal_cmd_send.hole_offset_yaw_takeover != 0u &&
        robot_cmd_hole_yaw_control_unlocked == 0u) {
        yaw_control = RobotCMDGetCurrentYawAngleDeg();
        robot_cmd_hole_yaw_control_unlocked = 1u;
    }

    /*
     * 狗洞模式内 yaw_control 才是底盘 yaw 目标。视觉/自瞄可以继续更新
     * gimbal_cmd_send 的其他字段，但不能把 yaw 目标改成视觉角。
     */
    gimbal_cmd_send.yaw = yaw_control;
}

static void RobotCMDHoleRestoreClearForceFollow(void)
{
    robot_cmd_hole_restore_force_follow = 0u;
}

static void RobotCMDHoleRestoreStartForceFollow(void)
{
    robot_cmd_hole_restore_force_follow = 1u;
    robot_cmd_hole_restore_force_key_c_count =
        rc_data[TEMP].key_count[KEY_PRESS][Key_C];
    robot_cmd_hole_restore_force_remote_rotate = remote_rotate_enable;
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
}

static void RobotCMDHoleRestoreUpdateForceFollow(void)
{
    if (robot_cmd_hole_restore_force_follow == 0u) {
        return;
    }

    if (gimbal_cmd_send.gimbal_mode == GIMBAL_HOLE_MODE ||
        gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE ||
        rc_data[TEMP].key_count[KEY_PRESS][Key_C] !=
            robot_cmd_hole_restore_force_key_c_count ||
        remote_rotate_enable != robot_cmd_hole_restore_force_remote_rotate) {
        RobotCMDHoleRestoreClearForceFollow();
        return;
    }

    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
}

static void RobotCMDHoleRestoreFinish(void)
{
    robot_cmd_hole_restore_active = 0u;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    RobotCMDHoleRestoreStartForceFollow();
}

static void RobotCMDApplyConservativeHoleModeGate(void)
{
#if (ROBOT_CMD_ALLOW_GIMBAL_HOLE_MODE == 0)
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_HOLE_MODE) {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.hole_yaw_ready = 0u;
        gimbal_cmd_send.hole_offset_yaw_takeover = 0u;
        gimbal_cmd_send.hole_restore_active = 0u;
        robot_cmd_hole_restore_active = 0u;
        RobotCMDHoleRestoreClearForceFollow();
        RobotCMDHoleYawControlLock();
        robot_cmd_hole_restore_yaw_aligned = 0u;
        robot_cmd_last_gimbal_mode = GIMBAL_GYRO_MODE;
    }
#endif
}

static uint8_t RobotCMDChassisYawModeNeedsDownLqreso(chassis_mode_e mode)
{
    return (mode == CHASSIS_FOLLOW_GIMBAL_YAW ||
            mode == CHASSIS_ROTATE ||
            mode == CHASSIS_REVERSE_ROTATE) ? 1u : 0u;
}

static uint8_t RobotCMDDownInLqresoRange(void)
{
    return (gimbal_fetch_data.pitch_down_motor_pos_rad >=
            PITCH_DOWN_LQRESO_SWITCH_POS_RAD) ? 1u : 0u;
}

static void RobotCMDApplyChassisYawOutputGate(Chassis_Ctrl_Cmd_s_uart *cmd)
{
    if (cmd == NULL || gimbal_cmd_send.gimbal_mode == GIMBAL_HOLE_MODE) {
        return;
    }

    if (robot_cmd_hole_restore_active != 0u) {
        cmd->chassis_mode = CHASSIS_NO_FOLLOW;
        cmd->wz = 0.0f;
        return;
    }

    if (RobotCMDChassisYawModeNeedsDownLqreso(cmd->chassis_mode) != 0u &&
        RobotCMDDownInLqresoRange() == 0u) {
        cmd->chassis_mode = CHASSIS_NO_FOLLOW;
        cmd->wz = 0.0f;
    }
}

static void RobotCMDUpdateHoleModeState(void)
{
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_HOLE_MODE) {
        robot_cmd_hole_restore_active = 0u;
        RobotCMDHoleRestoreClearForceFollow();
        robot_cmd_hole_restore_yaw_aligned = 0u;
    } else if (robot_cmd_last_gimbal_mode == GIMBAL_HOLE_MODE) {
        /*
         * 狗洞退出时先把 yaw_control 对齐当前 yaw，恢复阶段 yaw 电机只跟
         * yaw_control；down 回到 1.95rad±10deg 后释放底盘 yaw 跟随。
        */
        RobotCMDHoleRestoreAlignYawControlOnce();
        if (RobotCMDHoleRestoreReleaseReady() != 0u) {
            RobotCMDHoleRestoreFinish();
        } else {
            robot_cmd_hole_restore_active = 1u;
            RobotCMDHoleRestoreClearForceFollow();
            pitch_control = 0.0f;
            gimbal_cmd_send.pitch = 0.0f;
        }
    }

    if (robot_cmd_hole_restore_active != 0u &&
        RobotCMDHoleRestoreReleaseReady() != 0u) {
        RobotCMDHoleRestoreFinish();
    }

    if (robot_cmd_hole_restore_active != 0u) {
        RobotCMDHoleRestoreForceYawControl();
    }

    RobotCMDHoleRestoreUpdateForceFollow();

    /*
     * hole_yaw_ready 来自底盘板 offset 闭环结果。pitch down <= 0.1rad 后
     * 复用 hole_offset_yaw_takeover 通知底盘放开 yaw_control 闭环。
     */
    const uint8_t hole_state_enable =
        (gimbal_cmd_send.gimbal_mode == GIMBAL_HOLE_MODE) ? 1u : 0u;
    gimbal_cmd_send.hole_yaw_ready =
        (hole_state_enable != 0u) ? chassis_fetch_data_uart.hole_yaw_ready : 0u;
    gimbal_cmd_send.hole_offset_yaw_takeover =
        (hole_state_enable != 0u && RobotCMDHoleDownTakeoverReady() != 0u) ? 1u : 0u;
    gimbal_cmd_send.hole_restore_active = robot_cmd_hole_restore_active;
    RobotCMDUpdateHoleYawControlUnlock(hole_state_enable);
    robot_cmd_last_gimbal_mode = gimbal_cmd_send.gimbal_mode;
}
#endif
/**
 * @brief 云台Yaw轴反馈值改单圈角度后过圈处理
 *
 */
static void YawControlProcess()
{
    const INS_Instance *gimbal_imu = gimbal_fetch_data.gimbal_imu_data;
    float yaw_measure_deg =
        (gimbal_imu != NULL) ? gimbal_imu->output.Yaw_total_angle_deg : yaw_control;
    if (yaw_control - yaw_measure_deg > 180.0f) { //读取连续展开 yaw 数据进行过圈处理
        yaw_control -= 360.0f;
    } else if (yaw_control - yaw_measure_deg < -180.0f) {
        yaw_control += 360.0f;
    }
}

static uint8_t RobotCMDHoleYawInputBlocked(void)
{
#ifdef GIMBAL_BOARD
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_HOLE_MODE ||
        (rc_data[TEMP].key_count[KEY_PRESS][Key_X] % 2u) != 0u) {
        return (robot_cmd_hole_yaw_control_unlocked == 0u) ? 1u : 0u;
    }
#else
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_HOLE_MODE) {
        return 1u;
    }
#endif

    return 0u;
}

static void HeatControl()
{
    if (shoot_cmd_send.friction_mode == FRICTION_OFF) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }

    if (referee_data == NULL) {
        return;
    }

    if (RobotCMD17mmHeatReserveReached(referee_data->GameRobotState.shooter_heat_limit,
                                       referee_data->PowerHeatData.shooter_17mm_heat) != 0u) {
        RobotCMDStopShootingLoadMode();
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 */
static void EmergencyHandler()
{
    gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.mecanum_force_enable = 0u;
    shoot_cmd_send.friction_mode  = FRICTION_OFF;
    shoot_cmd_send.load_mode      = LOAD_STOP;
    shoot_cmd_send.shoot_mode     = SHOOT_OFF;
    SuperCap_flag_from_user     = SUPERCAP_UNUSE;
    // LOGERROR("[CMD] emergency stop!");
    (void)0;
}

#if defined(CHASSIS_DR16_DIRECT_CONTROL)
static float RobotCMDApplyChassisDR16YawAngleControl(int16_t yaw_stick)
{
    static uint8_t yaw_angle_active = 0u;
    static uint32_t yaw_angle_dwt_cnt = 0u;
    static uint8_t yaw_angle_dt_primed = 0u;
    static float yaw_angle_est_deg = 0.0f;
    static float yaw_last_wz_cmd = 0.0f;
    float dt = DWT_GetDeltaT(&yaw_angle_dwt_cnt);

    if (yaw_angle_dt_primed == 0u || isfinite(dt) == 0 ||
        dt <= 0.0f || dt > 0.02f) {
        dt = 0.001f;
        yaw_angle_dt_primed = 1u;
    }

    yaw_angle_est_deg += yaw_last_wz_cmd *
                         (CHASSIS_DR16_YAW_CMD_TO_RADPS * RAD_TO_ANGLE) *
                         dt;

    if (yaw_stick == 0) {
        yaw_angle_active = 0u;
        yaw_angle_est_deg = 0.0f;
        yaw_last_wz_cmd = 0.0f;
        chassis_dr16_yaw_angle_active_watch = 0U;
        chassis_dr16_yaw_angle_ref_watch = 0.0f;
        chassis_dr16_yaw_angle_est_watch = 0.0f;
        chassis_dr16_yaw_angle_error_watch = 0.0f;
        chassis_dr16_yaw_wz_cmd_watch = 0.0f;
        return 0.0f;
    }

    if (yaw_angle_active == 0u) {
        yaw_angle_est_deg = 0.0f;
        yaw_last_wz_cmd = 0.0f;
        yaw_angle_active = 1u;
    }

    const float stick_norm = RobotCMDClampFloat((float)yaw_stick /
                                                CHASSIS_DR16_YAW_STICK_FULL,
                                                -1.0f,
                                                1.0f);
    const float yaw_angle_max_deg =
        fabsf(RobotCMDFiniteOr((float)chassis_dr16_yaw_angle_max_deg,
                               CHASSIS_DR16_YAW_ANGLE_MAX_DEFAULT_DEG));
    const float yaw_kp =
        fabsf(RobotCMDFiniteOr((float)chassis_dr16_yaw_angle_kp,
                               CHASSIS_DR16_YAW_ANGLE_KP_DEFAULT));
    const float yaw_wz_max =
        fabsf(RobotCMDFiniteOr((float)chassis_dr16_yaw_wz_max,
                               CHASSIS_DR16_YAW_WZ_MAX_DEFAULT));
    const float yaw_ref_deg = stick_norm * yaw_angle_max_deg;
    const float yaw_est_limit = (yaw_angle_max_deg > 1.0f) ?
        (yaw_angle_max_deg * 1.5f) :
        1.5f;
    yaw_angle_est_deg = RobotCMDClampFloat(yaw_angle_est_deg,
                                           -yaw_est_limit,
                                           yaw_est_limit);
    const float yaw_error_deg = yaw_ref_deg - yaw_angle_est_deg;
    float wz_cmd = 0.0f;
    if (fabsf(yaw_error_deg) > CHASSIS_DR16_YAW_SETTLE_DEG) {
        wz_cmd = RobotCMDClampFloat(yaw_error_deg * yaw_kp,
                                    -yaw_wz_max,
                                    yaw_wz_max);
    }

    chassis_dr16_yaw_angle_active_watch = 1U;
    chassis_dr16_yaw_angle_ref_watch = yaw_ref_deg;
    chassis_dr16_yaw_angle_est_watch = yaw_angle_est_deg;
    chassis_dr16_yaw_angle_error_watch = yaw_error_deg;
    chassis_dr16_yaw_wz_cmd_watch = wz_cmd;
    yaw_last_wz_cmd = wz_cmd;
    return wz_cmd;
}

static void RobotCMDApplyChassisDR16DirectControl(void)
{
    static uint32_t last_update_count = 0u;
    static uint16_t stale_ticks = CHASSIS_DR16_STALE_TICKS;
    const uint32_t update_count = RemoteControlGetUpdateCount();

    if (update_count != last_update_count) {
        last_update_count = update_count;
        stale_ticks = 0u;
    } else if (stale_ticks < CHASSIS_DR16_STALE_TICKS) {
        stale_ticks++;
    }

    if (update_count == 0u || stale_ticks >= CHASSIS_DR16_STALE_TICKS ||
        RC_LOST || !switch_is_n(rc_data[TEMP].rc.switch_mode)) {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        chassis_cmd_send.wz = 0.0f;
        (void)RobotCMDApplyChassisDR16YawAngleControl(0);
        EmergencyHandler();
        return;
    }

    // Keep the existing translation scaling; right horizontal stick commands a bounded yaw angle.
    chassis_cmd_send.vx = CHASSIS_DR16_VXY_SCALE * (float)rc_data[TEMP].rc.rocker_l1;
    chassis_cmd_send.vy = CHASSIS_DR16_VXY_SCALE * (float)rc_data[TEMP].rc.rocker_l_;
    chassis_cmd_send.wz =
        RobotCMDApplyChassisDR16YawAngleControl(rc_data[TEMP].rc.rocker_r_);

    chassis_cmd_send.offset_angle = 0.0f;
    chassis_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_DIRECT;
    chassis_cmd_send.mecanum_force_enable = 0u;
    chassis_cmd_send.SuperCap_flag_from_user = SUPERCAP_UNUSE;

    // Allow four-wheel bring-up without referee data, matching RobotCMDInit defaults.
    if (chassis_cmd_send.power_limit == 0u) {
        chassis_cmd_send.power_limit = 100u;
    }
    if (chassis_cmd_send.supercap_power_limit == 0u) {
        chassis_cmd_send.supercap_power_limit = 100u;
    }
}
#endif

static float PitchCmdClampFloat(float value, float lower, float upper)
{
    if (value > upper) {
        return upper;
    }
    if (value < lower) {
        return lower;
    }
    return value;
}

#define PITCH_COUPLED_IMU_UP_COEF          0.9779975f
#define PITCH_COUPLED_IMU_DOWN_COEF        0.9808414f
#define PITCH_COUPLED_IMU_OFFSET_RAD       (-1.6046749f)
#define PITCH_COUPLED_DOWN_VALID_MIN_RAD   1.30f
#define PITCH_COUPLED_DOWN_VALID_MAX_RAD   2.50f
#define PITCH_COUPLED_UPPER_RAW_UP_RAD     PITCH_UP_MOTOR_RAW_LOW_LIMIT_RAD

static uint8_t pitch_coupled_cmd_limit_has_valid = 0U;

static float PitchCoupledUpLimitCoordRad(float raw_pos_rad)
{
    const float half_period = PITCH_UP_MOTOR_WRAP_PERIOD_RAD * 0.5f;

    while ((raw_pos_rad - PITCH_UP_MOTOR_UNWRAP_CENTER_RAD) > half_period) {
        raw_pos_rad -= PITCH_UP_MOTOR_WRAP_PERIOD_RAD;
    }
    while ((raw_pos_rad - PITCH_UP_MOTOR_UNWRAP_CENTER_RAD) < -half_period) {
        raw_pos_rad += PITCH_UP_MOTOR_WRAP_PERIOD_RAD;
    }
    return raw_pos_rad;
}

static float PitchCoupledEstimateImuRad(float up_limit_coord_rad,
                                        float down_pos_rad)
{
    return PITCH_COUPLED_IMU_UP_COEF * up_limit_coord_rad -
           PITCH_COUPLED_IMU_DOWN_COEF * down_pos_rad +
           PITCH_COUPLED_IMU_OFFSET_RAD;
}

static float PitchCoupledDynamicUpperLimitRad(float down_pos_rad)
{
    float upper_limit_rad =
        PitchCoupledEstimateImuRad(PitchCoupledUpLimitCoordRad(PITCH_COUPLED_UPPER_RAW_UP_RAD),
                                   down_pos_rad);

    if (upper_limit_rad < PITCH_IMU_CMD_HARD_LOWER_LIMIT_RAD) {
        upper_limit_rad = PITCH_IMU_CMD_HARD_LOWER_LIMIT_RAD;
    }

    return upper_limit_rad;
}

static uint8_t PitchCoupledFeedbackValid(float down_pos_rad)
{
    if (gimbal_fetch_data.gimbal_imu_data == NULL) {
        return 0U;
    }

    return (down_pos_rad >= PITCH_COUPLED_DOWN_VALID_MIN_RAD) &&
           (down_pos_rad <= PITCH_COUPLED_DOWN_VALID_MAX_RAD);
}

static void PitchCoupledUpdateImuLimits(float down_pos_rad)
{
    const float up_pos_rad =
        PitchCoupledUpLimitCoordRad(gimbal_fetch_data.pitch_up_motor_pos_rad);
    const float imu_pitch_rad = gimbal_fetch_data.pitch_up_imu_rad;

    pitch_coupled_imu_estimate_rad =
        PitchCoupledEstimateImuRad(up_pos_rad, down_pos_rad);
    pitch_coupled_imu_error_rad = imu_pitch_rad - pitch_coupled_imu_estimate_rad;

    pitch_coupled_imu_lower_limit_rad = PITCH_IMU_CMD_HARD_LOWER_LIMIT_RAD;
    pitch_coupled_imu_upper_limit_rad =
        PitchCoupledDynamicUpperLimitRad(down_pos_rad);

    pitch_up_cmd_lower_limit_rad = pitch_coupled_imu_lower_limit_rad;
    pitch_up_cmd_upper_limit_rad = pitch_coupled_imu_upper_limit_rad;
    pitch_up_motor_imu_slope = PITCH_COUPLED_IMU_UP_COEF;
}

static void PitchCoupledLimitPitchControlByImu(void)
{
    const float pitch_before = pitch_control;
    const float down_pos_rad = gimbal_fetch_data.pitch_down_motor_pos_rad;
    const uint8_t feedback_valid = PitchCoupledFeedbackValid(down_pos_rad);

    pitch_up_cmd_limit_active = 0U;
    pitch_coupled_cmd_limit_active = 0U;

    if (feedback_valid != 0U) {
        PitchCoupledUpdateImuLimits(down_pos_rad);
        pitch_coupled_cmd_limit_has_valid = 1U;
    }

    if (pitch_coupled_cmd_limit_has_valid == 0U) {
        return;
    }

    pitch_control = PitchCmdClampFloat(pitch_control,
                                       pitch_coupled_imu_lower_limit_rad,
                                       pitch_coupled_imu_upper_limit_rad);
    if (pitch_control != pitch_before) {
        pitch_coupled_cmd_limit_active = 1U;
    }
    pitch_up_cmd_limit_active = pitch_coupled_cmd_limit_active;
}

void gimbal_limit(void)
{
    PitchCoupledLimitPitchControlByImu();
}

static void RemoteButtonEdgeUpdate(void)
{
    /*
     * 图传遥控的 pause/custom_left/custom_right 是离散按键状态。
     * 这里用 TEMP/LAST 做上升沿检测，把“按下一次”转换成锁存开关，
     * 避免按住按键时在多个控制周期内反复翻转状态。
     */
    if (rc_update_flag != 1u)
        return;

    // pause：切换 NUC 自瞄/普通控制。
    if (rc_data[TEMP].rc.pause && !rc_data[LAST].rc.pause)
        remote_nuc_toggle_enable = remote_nuc_toggle_enable ? 0u : 1u;

    // custom_right：切换摩擦轮开关。
    if (rc_data[TEMP].rc.custom_right && !rc_data[LAST].rc.custom_right)
        shoot_cmd_send.friction_mode = (shoot_cmd_send.friction_mode == FRICTION_ON) ? FRICTION_OFF : FRICTION_ON;

    // custom_left：切换小陀螺/旋转状态。
    if (rc_data[TEMP].rc.custom_left && !rc_data[LAST].rc.custom_left)
        remote_rotate_enable = remote_rotate_enable ? 0u : 1u;

    // 本周期遥控边沿已经消费，等待下一次遥控数据更新后再检测。
    rc_update_flag = 0u;
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 */
static void RemoteControlSet()
{
    const uint8_t yaw_input_blocked = RobotCMDHoleYawInputBlocked();

    RemoteButtonEdgeUpdate();
    gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
    shoot_cmd_send.shoot_rate   = 20; // 射频默认20Hz
    shoot_cmd_send.shoot_mode=SHOOT_ON;
    // N档遥控模式: pause锁存自瞄，自定义左键锁存小陀螺，自定义右键锁存摩擦轮。
    // gimbal_cmd_send.nuc_mode = remote_nuc_toggle_enable ? version_control : none_version_control;
    chassis_cmd_send.chassis_mode = remote_rotate_enable ? CHASSIS_ROTATE : CHASSIS_FOLLOW_GIMBAL_YAW;

    // 扳机为按住发射；自瞄开启时仍需要视觉给出fire_advice才连续发射。

    if (shoot_cmd_send.friction_mode == FRICTION_ON && rc_data[TEMP].rc.trigger)
    {
        if (gimbal_cmd_send.nuc_mode == none_version_control)
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        // else if (fire_advice == 2)
        //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        // else
        //     shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }

    // 云台控制输入来源：
    // none_version_control: 直接使用遥控器摇杆
    // version_control:      使用上位机给定角度
    pitch_control -= PITCH_K * (float)rc_data[TEMP].rc.rocker_r1;
    if (yaw_input_blocked == 0u) {
        yaw_control -= YAW_K * (float)rc_data[TEMP].rc.rocker_r_;
    }
    // 左摇杆映射到底盘平移速度。
    chassis_cmd_send.vx = 70.0f * (float)rc_data[TEMP].rc.rocker_l1;
    chassis_cmd_send.vy = 70.0f * (float)rc_data[TEMP].rc.rocker_l_;
    // 云台软件限位
    gimbal_limit();
    if (yaw_input_blocked == 0u) {
        YawControlProcess();
    }
}

/**
 * @brief 键盘设定速度
 *
 */
static void ChassisSet()
{
    // 键鼠控制时给底盘一个基准模式，后续可被 KeyGetMode 覆盖（例如 C 键切小陀螺）
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    // chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    // chassis_cmd_send.power_limit=100;//临时更改

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    static float filtered_vx = 0.0f;
    static float filtered_vy = 0.0f;

    // KEY_PRESS keeps priority over KEY_PRESS_WITH_CTRL to preserve previous input behavior.
    if (rc_data[TEMP].key[KEY_PRESS].w) {
        target_vx = (float)CHASSIS_SPEED;
    } else if (rc_data[TEMP].key[KEY_PRESS].s) {
        target_vx = -(float)CHASSIS_SPEED;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w) { // 防止逃跑关小陀螺进入慢速移动
        target_vx = 4000.0f;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s) {
        target_vx = -4000.0f;
    }

    if (rc_data[TEMP].key[KEY_PRESS].d) {
        target_vy = (float)CHASSIS_SPEED;
    } else if (rc_data[TEMP].key[KEY_PRESS].a) {
        target_vy = -(float)CHASSIS_SPEED;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d) {
        target_vy = 4000.0f;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a) {
        target_vy = -4000.0f;
    }

    filtered_vx = 0.08f * target_vx + 0.92f * filtered_vx;
    filtered_vy = 0.08f * target_vy + 0.92f * filtered_vy;
    chassis_cmd_send.vx = filtered_vx;
    chassis_cmd_send.vy = filtered_vy;
}
/**
 * @brief 鼠标移动云台
 *
 */
static void GimbalSet()
{
    const uint8_t yaw_input_blocked = RobotCMDHoleYawInputBlocked();

    // 键鼠控制下，云台默认用陀螺仪模式：
    // 鼠标输入提供增量控制；若开启自瞄则直接使用视觉给定值。

    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    if(gimbal_cmd_send.nuc_mode == none_version_control)
    {
        if (yaw_input_blocked == 0u) {
            yaw_control -= rc_data[TEMP].mouse.x / 1000.0f;
        }
        pitch_control -= rc_data[TEMP].mouse.y / 30000.0f;
    }
    // else
    // {
    //     yaw_control = gimbal_cmd_send.yaw_version;
    // }
    // static float yaw_rotato_vision;
    // E 键/右键按住时，根据视觉误差做补偿（PID）
    // if(rc_data[TEMP].key[KEY_PRESS].e){
    //     if(fabs(gimbal_cmd_send.yaw_version)<20){
    //         yaw_rotato_vision=PIDCalculate(&YAW_version_PID,gimbal_cmd_send.yaw_version,0);
    //         // yaw_control-=PIDCalculate(&YAW_version_PID,gimbal_cmd_send.yaw_version,0);
    //         if(fabs(yaw_rotato_vision<0.08))yaw_control-=yaw_rotato_vision;
    //     }
    // }
    // else if (rc_data[TEMP].mouse.press_r){
    //     // pitch-up 手动目标暂不接视觉 pitch 补偿，右键不再改写 pitch_control。
    // }
    
    // if(fabs(gimbal_cmd_send.yaw_version)<20)yaw_control-=PIDCalculate(&YAW_version_PID,gimbal_cmd_send.yaw_version,0);
    if (yaw_input_blocked == 0u) {
        YawControlProcess();
    }
    gimbal_limit();
}

/**
 * @brief 键鼠设定机器人发射模式
 *
 */
static void ShootSet()
{
    // 发射逻辑说明：
    // 1) 仅摩擦轮开启时，左键触发发射
    // 2) 视觉模式下按 fire_advice 决定连发/停发
    // 3) G 键强制反转处理卡弹
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.shoot_rate = 16; // 射频默认16Hz

    // 仅在摩擦轮开启时有效
    if (shoot_cmd_send.friction_mode == FRICTION_ON) {
        // 打弹，单击左键单发，长按连发
        if (rc_data[TEMP].mouse.press_l) 
        {
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            // 打符，单发
            // if (auto_rune == 1) {
            //     shoot_cmd_send.load_mode = LOAD_1_BULLET;
            // } else {
            //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            // }
        } else {
            if (gimbal_cmd_send.nuc_mode == none_version_control)
            {
                shoot_cmd_send.load_mode = LOAD_STOP;
            }
            else
            {
                if (fire_advice == 2)
                {
                    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                }
                else
                {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
            }
            
        }
    } else {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    if (rc_data[TEMP].key[KEY_PRESS].g){
        shoot_cmd_send.load_mode = LOAD_REVERSE;
    }
    if(rc_data[TEMP].mouse.press_r)
    {
        remote_nuc_toggle_enable=1;
        gimbal_cmd_send.nuc_mode = version_control;
    }
    else
    {
        remote_nuc_toggle_enable=0;
        gimbal_cmd_send.nuc_mode = none_version_control;
    }
    HeatControl();
}

/**
 * @brief 键盘处理模式标志位
 *
 */

#ifdef GIMBAL_BOARD
static void MouseKeyApplyHoleModeToggle(void)
{
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_X] % 2u) {
        case 1:
            gimbal_cmd_send.gimbal_mode = GIMBAL_HOLE_MODE;
            break;
        case 0:
            if (gimbal_cmd_send.gimbal_mode == GIMBAL_HOLE_MODE)
                gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            break;
    }
}

static void MouseKeyUpdateVisionTxMode(void)
{
    const uint8_t key_b_count = rc_data[TEMP].key_count[KEY_PRESS][Key_B];
    const uint8_t key_f_count = rc_data[TEMP].key_count[KEY_PRESS][Key_F];

    if (key_b_count != vision_tx_key_b_count_last) {
        vision_tx_mode =
            (vision_tx_mode == VISION_TX_MODE_KEY_B) ?
                VISION_TX_MODE_DEFAULT :
                VISION_TX_MODE_KEY_B;
        vision_tx_key_b_count_last = key_b_count;
    }

    if (key_f_count != vision_tx_key_f_count_last) {
        vision_tx_mode =
            (vision_tx_mode == VISION_TX_MODE_KEY_F) ?
                VISION_TX_MODE_DEFAULT :
                VISION_TX_MODE_KEY_F;
        vision_tx_key_f_count_last = key_f_count;
    }
}
#endif

static void MouseKeyUpdateNucToggle(void)
{
    const uint8_t key_z_count = rc_data[TEMP].key_count[KEY_PRESS][Key_Z];

    if (key_z_count != mouse_key_z_count_last) {
        remote_nuc_toggle_enable = remote_nuc_toggle_enable ? 0u : 1u;
        mouse_key_z_count_last = key_z_count;
    }
}

static void KeyGetMode()
{
    // 键盘模式切换：
    // C: 底盘 跟随<->旋转
    // V: 摩擦轮 开/关
    // Z: 自瞄 开/关
    // X: 狗洞模式 奇数进入/偶数恢复陀螺模式
    // Shift: 超电使能
    // Ctrl: 打符模式标志
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) {
        case 1:
            if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
                chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            break;
        case 0:
            if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE)
                chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 2) {
        case 1:
            if (shoot_cmd_send.friction_mode != FRICTION_ON)
                shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
    }
#ifdef GIMBAL_BOARD
    MouseKeyApplyHoleModeToggle();
    MouseKeyUpdateVisionTxMode();
#endif
    switch (rc_data[TEMP].key[KEY_PRESS].r) {
        case 1:
            if (UI_SendFlag == 1) {
                UI_SendFlag = 0;
            }
            break;
        case 0:
            UI_SendFlag = 1;
            break;
    }
    
    switch (rc_data[TEMP].key[KEY_PRESS].ctrl) {
        case 1:
            auto_rune = 1;
            break;
        case 0:
            auto_rune = 0;
            break;
        default:
            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) {
        case 1:
            SuperCap_flag_from_user = SUPERCAP_USE;
            break;
        case 0:
            SuperCap_flag_from_user = SUPERCAP_UNUSE;
            break;
    }
}

/**
 * @brief 机器人复位函数，按下Ctrl+Shift+r
 *
 *
 */
static void RobotReset()
{
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].r) {
        osDelay(500);
        __set_FAULTMASK(1);
        NVIC_SystemReset(); // 软件复位
    }
    if (rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].r) {
    chassis_cmd_send_uart.reset_flag=1;
    }
    else chassis_cmd_send_uart.reset_flag=0;
}

/**
 * @brief 软件复位
 *
 */
static void MouseKeySet()
{
    // 键鼠控制主流程：
    // 底盘->云台->发射->模式键处理->复位检查
    ChassisSet();
    // MouseKeyUpdateNucToggle();
    // C档键鼠模式下，右键临时自瞄；视觉 last_mode 为 0 时本周期强制回手瞄。

    GimbalSet();
    ShootSet();
    KeyGetMode();
    gimbal_cmd_send.nuc_mode =
        ((remote_nuc_toggle_enable || rc_data[TEMP].mouse.press_r) && g_vision_rx_debug.last_mode != 0u) ?
            version_control :
            none_version_control;
    RobotReset(); // 机器人复位处理
}
extern int32_t shoot_count; 
// extern referee_info_t *referee_data_for_ui;
uint8_t nuc_rx[8];

float fp_pitch,fp_yaw,pitch_vel = 0;
//-----------------------------------------------------------------------------------------NUC版本解码-----------------------------------------------------------------------------------------//
#define NUC_FRAME_HEAD0 'C'
#define NUC_FRAME_HEAD1 'B'

#ifdef GIMBAL_BOARD
typedef struct __attribute__((packed)) {
    uint8_t head[2];
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} VisionToGimbal;


#endif
void USB_Version_devode(){
    float yaw_vel;
    if (fifo_pack[0] != NUC_FRAME_HEAD0 || fifo_pack[1] != NUC_FRAME_HEAD1)
        return;

    fire_advice = fifo_pack[2];
    
    fp_yaw = uint8_to_float_manual(fifo_pack + 3);
    yaw_vel = RAD_2_DEGREE * uint8_to_float_manual(fifo_pack + 7);
    fp_pitch = uint8_to_float_manual(fifo_pack + 15);
    pitch_vel = uint8_to_float_manual(fifo_pack + 19);
    // 将视觉yaw速度作为前馈
    vision_yaw_vel = yaw_vel;
    if (fire_advice == 0)
    {
        gimbal_cmd_send.pitch_version = pitch_control;
        gimbal_cmd_send.yaw_version = yaw_control;
    }
    else
    {
        gimbal_cmd_send.pitch_version = fp_pitch;
        gimbal_cmd_send.yaw_version = RAD_2_DEGREE * fp_yaw;
    }
}

static uint32_t nuc_rx_crc_ok_cnt = 0u;
static uint32_t nuc_rx_crc_fail_cnt = 0u;
#ifdef GIMBAL_BOARD
static uint32_t vision_rx_dwt_cnt = 0u;
static uint8_t vision_rx_period_primed = 0u;
static float vision_rx_period_avg_s = 0.0f;
#endif

static uint8_t NucCRC16Check(const uint8_t *frame, uint16_t len)
{
    if (frame == NULL || len < 3u)
        return 0u;
    return (Verify_CRC16_Check_Sum((uint8_t *)frame, len) == TRUE) ? 1u : 0u;
}

#ifdef GIMBAL_BOARD
static void RobotCMDVisionRxDecode(HostInstance *instance, const uint8_t *buf, uint16_t len)
{
    (void)instance;
    g_vision_rx_debug.online_frame_cnt++;

    if (buf == NULL || len != (uint16_t)sizeof(VisionToGimbal)) {
        g_vision_rx_debug.len_fail_cnt++;
        return;
    }

    if (buf[0] != NUC_FRAME_HEAD0 || buf[1] != NUC_FRAME_HEAD1) {
        g_vision_rx_debug.head_fail_cnt++;
        return;
    }

    if (NucCRC16Check(buf, (uint16_t)sizeof(VisionToGimbal)) == 0u) {
        g_vision_rx_debug.crc_fail_cnt++;
        return;
    }

    VisionToGimbal frame;
    float period_s;
    memcpy(&frame, buf, sizeof(frame));

    period_s = DWT_GetDeltaT(&vision_rx_dwt_cnt);
    if (vision_rx_period_primed == 0u) {
        vision_rx_period_avg_s = 0.0f;
        vision_rx_period_primed = 1u;
    } else if (period_s >= VISION_RX_PERIOD_MIN_S &&
               period_s <= VISION_RX_PERIOD_MAX_S) {
        if (vision_rx_period_avg_s <= 0.0f) {
            vision_rx_period_avg_s = period_s;
        } else {
            vision_rx_period_avg_s +=
                VISION_RX_PERIOD_AVG_ALPHA * (period_s - vision_rx_period_avg_s);
        }
    }
    g_vision_rx_debug.period_s = vision_rx_period_avg_s;
    if (g_vision_rx_debug.frame_seq >= 2147483647) {
        g_vision_rx_debug.frame_seq = 1;
    } else {
        g_vision_rx_debug.frame_seq++;
    }
    g_vision_rx_debug.crc_ok_cnt++;
    g_vision_rx_debug.last_mode = frame.mode;
    g_vision_rx_debug.yaw = frame.yaw;
    g_vision_rx_debug.yaw_vel = frame.yaw_vel;
    g_vision_rx_debug.yaw_acc = frame.yaw_acc;
    g_vision_rx_debug.pitch = frame.pitch;
    g_vision_rx_debug.pitch_vel = frame.pitch_vel;
    g_vision_rx_debug.pitch_acc = frame.pitch_acc;
}
#endif

/* 机器人核心控制任务 */
static void RobotCMDApplyControlInput(void)
{
    const INS_Instance *gimbal_imu;
    float imu_yaw_deg;
    float imu_pitch_rad;

    // 控制源仲裁：
    // C档 -> 键鼠
    // N档 -> 遥控
    // S档或遥控丢失 -> 急停
    if (RC_LOST || switch_is_s(rc_data[TEMP].rc.switch_mode))
        EmergencyHandler();
    else if (switch_is_c(rc_data[TEMP].rc.switch_mode))
        MouseKeySet();
    else if (switch_is_n(rc_data[TEMP].rc.switch_mode))
        RemoteControlSet();
    else
        EmergencyHandler();

    gimbal_imu = gimbal_fetch_data.gimbal_imu_data;
    imu_yaw_deg =
        (gimbal_imu != NULL) ? gimbal_imu->output.Yaw_total_angle_deg : yaw_control;
    imu_pitch_rad =
        (gimbal_imu != NULL) ?
        gimbal_imu->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] :
        pitch_control;

    //如果是刚进入控制，云台角度不变
    if(gimbal_cmd_send.gimbal_mode==GIMBAL_GYRO_MODE||gimbal_cmd_send.gimbal_mode==GIMBAL_MOTOR_MODE)
        if(gimbal_last_mode!=gimbal_cmd_send.gimbal_mode){
            yaw_control=imu_yaw_deg;
            pitch_control=imu_pitch_rad;
        }
    static uint8_t nuc_mode_last=0;
    if(!remote_nuc_toggle_enable || g_vision_rx_debug.last_mode == 0u){
        gimbal_cmd_send.nuc_mode = none_version_control;
        //恢复手瞄的时候不变云台
        if(nuc_mode_last!=gimbal_cmd_send.nuc_mode){
            if (gimbal_cmd_send.gimbal_mode != GIMBAL_HOLE_MODE) {
                yaw_control=imu_yaw_deg;
            }
            pitch_control=imu_pitch_rad;
        }
        gimbal_cmd_send.yaw   = yaw_control;
        gimbal_cmd_send.pitch = pitch_control;
    }
    else {
        gimbal_cmd_send.nuc_mode = version_control;
        gimbal_cmd_send.yaw     =  imu_yaw_deg+(g_vision_rx_debug.yaw - yaw_reverse)*RAD_TO_ANGLE;
        gimbal_cmd_send.pitch = pitch_control;
    }  
    nuc_mode_last=gimbal_cmd_send.nuc_mode;
}

static void RobotCMDUpdateShootReferee(uint16_t cooling_rate, uint16_t referee_heat, uint16_t cooling_limit)
{
    memcpy(&shoot_cmd_send.shooter_heat_cooling_rate, &cooling_rate, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_referee_heat, &referee_heat, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_cooling_limit, &cooling_limit, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.bullet_speed, &referee_data->ShootData.initial_speed_mps, sizeof(float));
}

#ifdef CHASSIS_BOARD
static uint8_t RobotCMDShooter17mmHeatReserveReached(void)
{
    const uint16_t heat_limit = referee_data->GameRobotState.shooter_heat_limit;
    const uint16_t shooter_heat = referee_data->PowerHeatData.shooter_17mm_heat;

    return RobotCMD17mmHeatReserveReached(heat_limit, shooter_heat);
}

static void RobotCMDApply17mmHeatStopGate(void)
{
    if (RobotCMDShooter17mmHeatReserveReached() == 0u) {
        return;
    }

    RobotCMDStopShootingLoadMode();
}
#endif

#ifdef GIMBAL_BOARD
/**
 * @brief 根据 USB 视觉帧的 last_mode 接管自动拨弹命令。
 *
 * last_mode 来自 g_vision_rx_debug.last_mode，也就是 VisionToGimbal.mode；
 * 这里不处理 gimbal_mode，也不负责切换自瞄跟随，只处理发射 load_mode。
 *
 * 状态机含义：
 * - last_mode == 0：视觉无目标/退出跟随，立即停拨弹并清自动发射等待状态。
 * - 自瞄锁存未开或摩擦轮未开：只同步当前视觉 mode 并清等待状态，不强行改 load_mode。
 * - last_mode == 2：先等视觉 yaw/pitch 目标与当前控制基准误差半径 <= 0.035 rad；
 *   达标后本次 mode2 会话锁存允许 LOAD_BURSTFIRE，并记录当前 shoot_count。
 * - 从已允许发射的 last_mode == 2 退出到非 0 mode：保持 LOAD_BURSTFIRE，
 *   直到本地 shoot_count 增加 1 次后再切 LOAD_STOP，避免半途停在弹道中。
 */
static void RobotCMDUpdateVisionAutoLoadMode(void)
{
    const uint8_t vision_mode = g_vision_rx_debug.last_mode;

    // C档左键人工开火优先级最高，不受视觉 last_mode 自动拨弹状态机覆盖。
    if (switch_is_c(rc_data[TEMP].rc.switch_mode) &&
        rc_data[TEMP].mouse.press_l &&
        shoot_cmd_send.friction_mode == FRICTION_ON) {
        vision_auto_load_last_mode = vision_mode;
        vision_auto_load_wait_shot = 0u;
        vision_mode2_fire_allowed = 0u;
        vision_auto_load_mode2_shoot_count = shoot_count;
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        return;
    }

    // 视觉 mode 0 直接退出自瞄发射链路，不等待 shoot_count 收尾。
    if (vision_mode == 0u) {
        vision_auto_load_last_mode = vision_mode;
        vision_auto_load_wait_shot = 0u;
        vision_mode2_fire_allowed = 0u;
        vision_auto_load_mode2_shoot_count = shoot_count;
        // shoot_cmd_send.load_mode = LOAD_STOP;
        return;
    }

    // 自瞄锁存没开或摩擦轮没开时，本函数不接管拨弹，只重置自动发射状态。
    if (remote_nuc_toggle_enable == 0u || shoot_cmd_send.friction_mode != FRICTION_ON) {
        vision_auto_load_last_mode = vision_mode;
        vision_auto_load_wait_shot = 0u;
        vision_mode2_fire_allowed = 0u;
        vision_auto_load_mode2_shoot_count = shoot_count;
        return;
    }

    // 视觉 mode 2 是自动发射请求；先等角误差半径达标，再锁存允许发射。
    if (vision_mode == VISION_AUTO_FIRE_MODE) {
        if (vision_auto_load_last_mode != VISION_AUTO_FIRE_MODE) {
            vision_mode2_fire_allowed = 0u;
        }

        if (vision_mode2_fire_allowed == 0u) {
            static float yaw_error ;
               yaw_error = g_vision_rx_debug.yaw - yaw_reverse;
            static float pitch_error ;
            pitch_error = g_vision_rx_debug.pitch - vision_pitch_rad;
            // static float error_radius ;
            // error_radius = sqrtf(yaw_error * yaw_error + pitch_error * pitch_error);
            // static float yaw_law_error=0.2f ;
            // if(fabs(pitch_error)<0.5/57.3&&fabs(yaw_error)<yaw_law_error/57.3)
            //     vision_mode2_fire_allowed = 1u;
            // if (error_radius <= 0.0035f) {
            //     vision_mode2_fire_allowed = 1u;
            // }
            if(yaw_error<=0.02f&&pitch_error<=0.035f){
                vision_mode2_fire_allowed = 1u;
            }
        }

        vision_auto_load_last_mode = vision_mode;
        vision_auto_load_wait_shot = 0u;
        vision_auto_load_mode2_shoot_count = shoot_count;
        shoot_cmd_send.load_mode = (vision_mode2_fire_allowed != 0u) ? LOAD_BURSTFIRE : LOAD_STOP;
        return;
    }

    // 从已经允许发射的 mode 2 离开后，等待本地 shoot_count 跳变，确认当前拨弹周期完成。
    if (vision_auto_load_last_mode == VISION_AUTO_FIRE_MODE && vision_mode2_fire_allowed != 0u) {
        vision_auto_load_wait_shot = 1u;
    }
    vision_mode2_fire_allowed = 0u;

    // 等待期间继续连发；确认 shoot_count 增加 1 次后再停拨弹。
    if (vision_auto_load_wait_shot != 0u) {
        if ((shoot_count - vision_auto_load_mode2_shoot_count) >= 1) {
            shoot_cmd_send.load_mode = LOAD_STOP;
            vision_auto_load_wait_shot = 0u;
        } else {
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        }
    }
    
    vision_auto_load_last_mode = vision_mode;
}

static void RobotCMDApplyHoleModeShootSafety(void)
{
    if (gimbal_cmd_send.gimbal_mode != GIMBAL_HOLE_MODE) {
        return;
    }

    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    vision_auto_load_last_mode = g_vision_rx_debug.last_mode;
    vision_auto_load_wait_shot = 0u;
    vision_mode2_fire_allowed = 0u;
    vision_auto_load_mode2_shoot_count = shoot_count;
}
#endif

#ifdef CHASSIS_BOARD
static void RobotCMDTaskChassisBoard(void)
{
    //Judge_Uart();//判断串口是否卡死且没有进入错误中断回调重新使能
    DeterminRobotID(); //根据裁判系统数据判断机器人ID
    //获得底盘、云台、发射的控制数据
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    // RS485控制数据在串口接收回调中解算；这里直接使用最近一次解算后的结果。
        
    //判断是否需要重置底盘板
    if (chassis_rs485_recv.reset_flag != 0) {
        // __disable_irq();
        osDelay(500);
        __set_FAULTMASK(1);
        NVIC_SystemReset();
        // __enable_irq();
    }
    RobotCMDFilterChassisVisionYawAcc();
    CalcOffsetAngle();
/*--------------------------底盘反馈数据赋值--------------------------------------------------------------------------------*/
    Chassis_Upload_Data_s_uart upload_snapshot = {0};
    const INS_Instance *gimbal_imu = gimbal_fetch_data.gimbal_imu_data;
    const uint8_t hole_mode_active =
        (chassis_rs485_recv.gimbal_mode == GIMBAL_HOLE_MODE) ? 1u : 0u;
    const uint8_t hole_offset_yaw_takeover =
        (hole_mode_active != 0u &&
         chassis_rs485_recv.hole_offset_yaw_takeover != 0u) ? 1u : 0u;
    const uint8_t hole_yaw_ready =
        RobotCMDUpdateHoleYawReadyLatch(hole_mode_active,
                                        hole_offset_yaw_takeover,
                                        chassis_cmd_send.offset_angle);
    upload_snapshot.yaw_motor_single_round_angle = gimbal_fetch_data.yaw_motor_single_round_angle;
    upload_snapshot.yaw_total_angle = gimbal_fetch_data.yaw_total_angle;
    upload_snapshot.yaw_ecd = gimbal_fetch_data.yaw_ecd;
    upload_snapshot.chassis_yaw_gyro =
        (gimbal_imu != NULL) ? gimbal_imu->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET] : 0.0f;
    upload_snapshot.chassis_pitch_angle =
        (gimbal_imu != NULL) ? gimbal_imu->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] : 0.0f;
    upload_snapshot.initial_speed = referee_data->ShootData.initial_speed_mps;
    upload_snapshot.color = referee_data->referee_id.Robot_Color;
    upload_snapshot.yaw_angle_pidout = gimbal_fetch_data.yaw_angle_pidout;
    upload_snapshot.yaw_motor_vel = gimbal_fetch_data.yaw_motor_vel;
    upload_snapshot.hole_yaw_ready = hole_yaw_ready;
    upload_snapshot.hole_offset_yaw_takeover = hole_offset_yaw_takeover;
    upload_snapshot.chassis_wz=gimbal_fetch_data.gimbal_imu_data->INS_data.INS_gyro[2];
    chassis_fetch_data_uart = upload_snapshot;

/*--------------------------底盘控制数据赋值--------------------------------------------------------------------------------*/
    gimbal_cmd_send.yaw = chassis_rs485_recv.yaw_control;
    gimbal_cmd_send.gimbal_mode = chassis_rs485_recv.gimbal_mode;
    gimbal_cmd_send.nuc_mode = chassis_rs485_recv.nuc_mode;
    gimbal_cmd_send.yaw_version = chassis_rs485_recv.nuc_yaw;
    gimbal_cmd_send.yaw_measure_angle=chassis_rs485_recv.yaw_angle;
    gimbal_cmd_send.yaw_measure_gyro=chassis_rs485_recv.yaw_gyro;
    gimbal_cmd_send.offset_angle = chassis_cmd_send.offset_angle;
    gimbal_cmd_send.hole_yaw_ready = upload_snapshot.hole_yaw_ready;
    gimbal_cmd_send.hole_offset_yaw_takeover = hole_offset_yaw_takeover;
    gimbal_cmd_send.yaw_ref_rad = chassis_rs485_recv.vision_yaw;
    gimbal_cmd_send.yaw_ref_vel_rad_s = chassis_rs485_recv.vision_yaw_vel;
    gimbal_cmd_send.yaw_ref_acc_rad_s2 = chassis_rs485_recv.vision_yaw_acc;
    gimbal_cmd_send.vision_frame_seq = chassis_rs485_recv.vision_frame_seq;
    gimbal_cmd_send.vision_period_s = chassis_rs485_recv.vision_period_s;
    gimbal_cmd_send.pitch_ref_rad = chassis_rs485_recv.vision_pitch;
    gimbal_cmd_send.pitch_ref_vel_rad_s = chassis_rs485_recv.vision_pitch_vel;
    gimbal_cmd_send.pitch_ref_acc_rad_s2 = chassis_rs485_recv.vision_pitch_acc;
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    shoot_cmd_send.load_mode = chassis_rs485_recv.load_mode;
    shoot_cmd_send.shoot_mode = chassis_rs485_recv.shoot_mode;
    shoot_cmd_send.shoot_count = chassis_rs485_recv.shoot_count;
    shoot_cmd_send.friction_mode = chassis_rs485_recv.friction_mode;
#if !defined(CHASSIS_DR16_DIRECT_CONTROL)
    chassis_cmd_send.vx = chassis_rs485_recv.vx;
    chassis_cmd_send.vy = chassis_rs485_recv.vy;
    chassis_cmd_send.wz = chassis_rs485_recv.wz;
#endif
    chassis_cmd_send.yaw_control = chassis_rs485_recv.yaw_control;
    chassis_cmd_send.nuc_enable_flag=remote_nuc_toggle_enable;
#if !defined(CHASSIS_DR16_DIRECT_CONTROL)
    chassis_cmd_send.chassis_mode = chassis_rs485_recv.chassis_mode;
    chassis_cmd_send.gimbal_mode = chassis_rs485_recv.gimbal_mode;
#endif
    chassis_cmd_send.hole_offset_yaw_takeover = hole_offset_yaw_takeover;
    //chassis_cmd_send.offset_angle = chassis_rs485_recv.offset_angle;
    //chassis_cmd_send.gimbal_error_angle = chassis_rs485_recv.gimbal_error_angle;
    chassis_cmd_send.power_limit =
        RobotCMDGetChassisEffectivePowerLimit(referee_data->GameRobotState.chassis_power_limit,
                                              chassis_rs485_recv.superCap_flag);
    chassis_cmd_send.supercap_power_limit = referee_data->GameRobotState.chassis_power_limit;
    chassis_cmd_send.level = referee_data->GameRobotState.robot_level;
    chassis_cmd_send.power_buffer = referee_data->PowerHeatData.buffer_energy_j;
    chassis_cmd_send.SuperCap_flag_from_user = chassis_rs485_recv.superCap_flag;
    chassis_cmd_send.gimbal_yaw_total_angle=chassis_rs485_recv.yaw_angle;
    chassis_cmd_send.gimbal_yaw_signal_sound_angle =
        chassis_rs485_recv.gimbal_yaw_signal_sound_angle;
    chassis_cmd_send.mecanum_force_enable = (chassis_rs485_recv.UI_SendFlag & MECANUM_FORCE_UI_FLAG_BIT) ? 1u : 0u;
#if defined(CHASSIS_DR16_DIRECT_CONTROL)
    RobotCMDApplyChassisDR16DirectControl();
#endif
	/*--------------------------发射数据更新--------------------------------------------------------------------------------*/
    RobotCMDUpdateShootReferee(referee_data->GameRobotState.shooter_heat_cooling_per_s,
                               referee_data->PowerHeatData.shooter_17mm_heat,
                               referee_data->GameRobotState.shooter_heat_limit);
    RobotCMDApply17mmHeatStopGate();
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
/*--------------------------UI数据更新--------------------------------------------------------------------------------*/
    referee_data_for_ui = referee_data;
    {
        uint8_t ui_send_flag_rx = (uint8_t)(chassis_rs485_recv.UI_SendFlag & UI_SEND_FLAG_MASK);
        memcpy(&ui_cmd_send.ui_send_flag, &ui_send_flag_rx, sizeof(uint8_t));
    }
    memcpy(&ui_cmd_send.chassis_mode, &chassis_cmd_send.chassis_mode, sizeof(chassis_mode_e));
    memcpy(&ui_cmd_send.chassis_attitude_angle, &gimbal_fetch_data.yaw_motor_single_round_angle, sizeof(uint16_t));
    {
        float pitch_actual_angle_deg = gimbal_fetch_data.pitch_up_imu_rad * RAD_TO_ANGLE;
        memcpy(&ui_cmd_send.pitch_actual_angle_deg, &pitch_actual_angle_deg, sizeof(float));
    }
    memcpy(&ui_cmd_send.friction_mode, &shoot_cmd_send.friction_mode, sizeof(friction_mode_e));
    memcpy(&ui_cmd_send.rune_mode, &auto_rune, sizeof(uint8_t));
    memcpy(&ui_cmd_send.SuperCap_mode, &chassis_rs485_recv.superCap_flag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.supercap_voltage, &chassis_fetch_data.cap_voltage, sizeof(float));
    memcpy(&ui_cmd_send.Chassis_Ctrl_power, &chassis_fetch_data.chassis_power_output, sizeof(float));
    memcpy(&ui_cmd_send.Chassis_power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    memcpy(&ui_cmd_send.Shooter_heat, &referee_data->PowerHeatData.shooter_17mm_heat, sizeof(uint16_t));
    memcpy(&ui_cmd_send.cap_online_flag, &chassis_fetch_data.cap_online_flag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.cap_ready_flag, &chassis_fetch_data.cap_ready_flag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.cap_active_flag, &chassis_fetch_data.cap_active_flag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.nuc_flag, &chassis_rs485_recv.nuc_mode, sizeof(uint8_t));
    {
        uint8_t hole_mode = (chassis_rs485_recv.gimbal_mode == GIMBAL_HOLE_MODE) ? 1u : 0u;
        memcpy(&ui_cmd_send.hole_mode, &hole_mode, sizeof(uint8_t));
    }
    memcpy(&ui_cmd_send.vision_tx_mode, &chassis_rs485_recv.vision_tx_mode, sizeof(uint8_t));
    memcpy(&ui_cmd_send.robot_level, &referee_data->GameRobotState.robot_level, sizeof(uint8_t));
    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
}
#endif
#ifdef GIMBAL_BOARD
static void RobotCMDTaskGimbalBoard(void)
{
    const INS_Instance *gimbal_imu;

    // NUC数据校验：回调中已memcpy到内部缓冲区，这里直接校验
    // if (HostDataReady(host_instance)) {
    //     memcpy(fifo_pack, HostGetRecvBuf(host_instance), HostGetRecvLen(host_instance));
    //     if (fifo_pack[0] == NUC_FRAME_HEAD0 && fifo_pack[1] == NUC_FRAME_HEAD1) {
    //         if (NucCRC16Check(fifo_pack, NUC_RECV_SIZE)) {
    //             nuc_rx_crc_ok_cnt++;
    //             USB_Version_devode();
    //         } else {
    //             nuc_rx_crc_fail_cnt++;
    //         }
    //     }
    // }
    //IWDG喂狗
    HAL_IWDG_Refresh(&hiwdg);
    // 底盘上传数据校验：回调中已memcpy到内部缓冲区，这里直接校验
    if (RobotCMD485RecvItems(rs485_master_instance,
                             robot_cmd_upload_signals,
                             ROBOT_CMD_UPLOAD_SIGNAL_COUNT,
                             &chassis_fetch_data_uart) > 0u) {
        if (chassis_fetch_data_uart.color)
            vision_send_data[8] = 1;
        else
            vision_send_data[8] = 0;
    }

    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    gimbal_imu = gimbal_fetch_data.gimbal_imu_data;
    
// remote_nuc_toggle_enable=1;
    RobotCMDApplyControlInput();
    RobotCMDUpdateHoleModeState();
    RobotCMDUpdateVisionAutoLoadMode();
    RobotCMDApplyConservativeHoleModeGate();
    RobotCMDApplyHoleModeShootSafety();

    //RobotCMDUpdateShootReferee(referee_data->GameRobotState.shooter_heat_cooling_per_s,
    //                           referee_data->PowerHeatData.shooter_17mm_heat,
    //                           referee_data->GameRobotState.shooter_heat_limit);
    //将底盘控制数据赋值给结构体，并通过RS485发送给底盘板
    // chassis_cmd_send_uart.reset_flag = 0;
    chassis_cmd_send_uart.vx = chassis_cmd_send.vx;
    chassis_cmd_send_uart.vy = chassis_cmd_send.vy;
    chassis_cmd_send_uart.wz = chassis_cmd_send.wz;
    chassis_cmd_send_uart.offset_angle = chassis_cmd_send.offset_angle;
    chassis_cmd_send_uart.gimbal_error_angle = chassis_cmd_send.gimbal_error_angle;
    /*
     * 狗洞退出恢复阶段只暂时拦截底盘 yaw 模式，让 down/up 先回安全位置；
     * vx/vy 不在这里清零，避免额外改变驾驶员给出的平移命令。
     */
	    chassis_cmd_send_uart.chassis_mode =
	        (robot_cmd_hole_restore_active != 0u) ? CHASSIS_NO_FOLLOW : chassis_cmd_send.chassis_mode;
        if (robot_cmd_hole_restore_force_follow != 0u) {
            chassis_cmd_send_uart.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        }
	    chassis_cmd_send_uart.gimbal_mode = gimbal_cmd_send.gimbal_mode;
	    chassis_cmd_send_uart.hole_offset_yaw_takeover = gimbal_cmd_send.hole_offset_yaw_takeover;
	    RobotCMDApplyChassisYawOutputGate(&chassis_cmd_send_uart);
    chassis_cmd_send_uart.yaw_control =
        (gimbal_cmd_send.gimbal_mode == GIMBAL_HOLE_MODE) ?
        yaw_control :
        gimbal_cmd_send.yaw;
    chassis_cmd_send_uart.gimbal_yaw_signal_sound_angle =gimbal_fetch_data.yaw_motor_single_round_angle;//yaw 电机单圈反馈角
    chassis_cmd_send_uart.yaw_angle =
        (gimbal_imu != NULL) ? gimbal_imu->output.Yaw_total_angle_deg : 0.0f;//连续展开yaw角输出
    chassis_cmd_send_uart.yaw_gyro =
        (gimbal_imu != NULL) ? gimbal_imu->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET] : 0.0f;//

    chassis_cmd_send_uart.yaw_vel = vision_yaw_vel;    // 将视觉yaw速度通过RS485发送给下板
    //chassis_cmd_send_uart.yaw_vel = 100;    // 将视觉yaw速度通过RS485发送给下板

    chassis_cmd_send_uart.vision_yaw = g_vision_rx_debug.yaw;
    chassis_cmd_send_uart.vision_yaw_vel = g_vision_rx_debug.yaw_vel;
    chassis_cmd_send_uart.vision_yaw_acc = g_vision_rx_debug.yaw_acc;

    chassis_cmd_send_uart.vision_frame_seq = g_vision_rx_debug.frame_seq;
    chassis_cmd_send_uart.vision_period_s = g_vision_rx_debug.period_s;
    chassis_cmd_send_uart.vision_pitch = g_vision_rx_debug.pitch;
    chassis_cmd_send_uart.vision_pitch_vel = g_vision_rx_debug.pitch_vel;
    chassis_cmd_send_uart.vision_pitch_acc = g_vision_rx_debug.pitch_acc;
    gimbal_cmd_send.yaw_ref_rad = g_vision_rx_debug.yaw;
    gimbal_cmd_send.yaw_ref_vel_rad_s = g_vision_rx_debug.yaw_vel;
    gimbal_cmd_send.yaw_ref_acc_rad_s2 = g_vision_rx_debug.yaw_acc;
    gimbal_cmd_send.vision_frame_seq = g_vision_rx_debug.frame_seq;
    gimbal_cmd_send.vision_period_s = g_vision_rx_debug.period_s;
    gimbal_cmd_send.pitch_ref_rad = g_vision_rx_debug.pitch;
    gimbal_cmd_send.pitch_ref_vel_rad_s = g_vision_rx_debug.pitch_vel;
    gimbal_cmd_send.pitch_ref_acc_rad_s2 = g_vision_rx_debug.pitch_acc;

    chassis_cmd_send_uart.nuc_yaw = gimbal_cmd_send.yaw_version;
    // chassis_cmd_send_uart.nuc_yaw=0;
    chassis_cmd_send_uart.shoot_mode = shoot_cmd_send.shoot_mode;
    chassis_cmd_send_uart.load_mode = shoot_cmd_send.load_mode;
    chassis_cmd_send_uart.shoot_count = shoot_count;
    chassis_cmd_send_uart.friction_mode = shoot_cmd_send.friction_mode;
    chassis_cmd_send_uart.nuc_mode = gimbal_cmd_send.nuc_mode;
    chassis_cmd_send_uart.vision_tx_mode = vision_tx_mode;
    chassis_cmd_send_uart.UI_SendFlag = (uint8_t)(UI_SendFlag & UI_SEND_FLAG_MASK);
    if (chassis_cmd_send.mecanum_force_enable) {
        chassis_cmd_send_uart.UI_SendFlag |= MECANUM_FORCE_UI_FLAG_BIT;
    }
    chassis_cmd_send_uart.superCap_flag = SuperCap_flag_from_user;
    //测使用
    // chassis_cmd_send_uart.chassis_mode=CHASSIS_ZERO_FORCE;
    RobotCMD485SendItems(rs485_master_instance,
                         robot_cmd_ctrl_signals,
                         ROBOT_CMD_CTRL_SIGNAL_COUNT,
                         &chassis_cmd_send_uart,
                         &ctrl_tx_state);//打包发送
    gimbal_last_mode=gimbal_cmd_send.gimbal_mode;
    shoot_cmd_send.bullet_speed = chassis_fetch_data_uart.initial_speed;
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    vision_send_data[0] = NUC_FRAME_HEAD0;
    vision_send_data[1] = NUC_FRAME_HEAD1;
    vision_send_data[2] = vision_tx_mode;

    // EularAngleToQuaternion(gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_YAW_ADDRESS_OFFSET],
    //                        gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],
    //                        gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_ROLL_ADDRESS_OFFSET],
    //                        p);//输出四元数

    //memcpy(vision_send_data + 3, p, 16);
    if (gimbal_imu != NULL) {
        memcpy(vision_send_data + 3, &gimbal_imu->INS_data.INS_quat[0], 4*sizeof(float));
        yaw_reverse = gimbal_imu->output.INS_angle[INS_YAW_ADDRESS_OFFSET];
        vision_pitch_gyro_rad_s =
            gimbal_imu->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
        vision_pitch_rad =
            gimbal_imu->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
        vision_yaw_gyro_rad_s =
            gimbal_imu->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];
    } else {
        const float identity_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        memcpy(vision_send_data + 3, identity_quat, sizeof(identity_quat));
        yaw_reverse = 0.0f;
        vision_pitch_gyro_rad_s = 0.0f;
        vision_pitch_rad = 0.0f;
        vision_yaw_gyro_rad_s = 0.0f;
    }
    float_to_uint8_manual(yaw_reverse, vision_send_data + 19);
    
    memcpy(vision_send_data + 23, &vision_yaw_gyro_rad_s, sizeof(float));
    float_to_uint8_manual(vision_pitch_rad, vision_send_data + 27);
    memcpy(vision_send_data + 31, &vision_pitch_gyro_rad_s, sizeof(float));
    memcpy(&vision_send_data[35], &chassis_fetch_data_uart.initial_speed, 4);
    const uint16_t vision_shoot_count = (uint16_t)shoot_count;
    memcpy(&vision_send_data[39], &vision_shoot_count, sizeof(vision_shoot_count));
    Append_CRC16_Check_Sum(vision_send_data, 43);
    HostSend(host_instance, vision_send_data, 43);
}
#endif

#ifdef CHASSIS_BOARD
static void RobotCMDTaskPostProcess(void)
{
    if (shoot_cmd_send.load_mode == LOAD_STOP) {
        shoot_cmd_send.shoot_aim_angle = shoot_fetch_data.loader_angle - ONE_BULLET_DELTA_ANGLE;
        shoot_cmd_send.Shoot_Once_Flag = 1;
    } else if (shoot_cmd_send.load_mode == LOAD_1_BULLET) {
        if (fabs(shoot_fetch_data.loader_angle - shoot_cmd_send.shoot_aim_angle) < 0.1)
            shoot_cmd_send.Shoot_Once_Flag = 0;
    }

    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
#endif

void RobotCMDTask()
{
    //CMD控制逻辑
    #ifdef CHASSIS_BOARD
    RobotCMDTaskChassisBoard();
    //RobotCMDTaskPostProcess();//拨弹盘在下板
    #elif defined(GIMBAL_BOARD)
    RobotCMDTaskGimbalBoard();
    #endif

}


