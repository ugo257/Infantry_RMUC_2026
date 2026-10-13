/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "omni_velocity_ctrl.h"
#include "robot_board.h"
#include "robot_params.h"
#include "robot_types.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_init.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "arm_math.h"
#include "tool.h"
#include "remote_control.h"
#include "ins_task.h"
#include <math.h>
#include <string.h>
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE  (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)    // 半轮距
#define PERIMETER_WHEEL  (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define LF               0
#define RF               1
#define RB               2
#define LB               3
#define CHASSIS_WHEEL_REDUCTION_RATIO (268.0f / 17.0f)
#define CHASSIS_FORCE_CALIB_DEFAULT_CURRENT_LIMIT 5000.0f
#define CHASSIS_FORCE_CALIB_HARD_CURRENT_LIMIT 6000.0f
#define CHASSIS_FORCE_CALIB_BASELINE_TIME_S 1.000f
#define CHASSIS_FORCE_CALIB_PULSE_TIME_S 0.600f
#define CHASSIS_FORCE_CALIB_COAST_WAIT_TIME_S 2.000f
#define CHASSIS_FORCE_CALIB_HARD_TIMEOUT_S 1.250f
#define CHASSIS_FORCE_CALIB_ALL_WHEELS 0xFFU
#define CHASSIS_FORCE_CALIB_CURRENT_LEVEL_NUM 5U
#define CHASSIS_FORCE_CALIB_REPEAT_NUM 2U
#define CHASSIS_FORCE_CALIB_DIRECTION_NUM 2U
#define CHASSIS_FORCE_CALIB_SINGLE_WHEEL_TRIALS \
    (CHASSIS_FORCE_CALIB_DIRECTION_NUM * \
     CHASSIS_FORCE_CALIB_CURRENT_LEVEL_NUM * CHASSIS_FORCE_CALIB_REPEAT_NUM)
#define CHASSIS_FORCE_CALIB_TOTAL_TRIALS \
    (OMNI_WHEEL_NUM * CHASSIS_FORCE_CALIB_DIRECTION_NUM * \
     CHASSIS_FORCE_CALIB_CURRENT_LEVEL_NUM * CHASSIS_FORCE_CALIB_REPEAT_NUM)
#define CHASSIS_H_CALIB_DEFAULT_AMP_VXY 20000.0f
#define CHASSIS_H_CALIB_DEFAULT_AMP_WZ 4000.0f
#define CHASSIS_H_CALIB_DEFAULT_ZERO_TICKS 500U
#define CHASSIS_H_CALIB_DEFAULT_RAMP_TICKS 500U
#define CHASSIS_H_CALIB_DEFAULT_HOLD_TICKS 1200U
#define CHASSIS_H_CALIB_DEFAULT_REPEAT_COUNT 1U
#define CHASSIS_H_CALIB_DEFAULT_WHEEL_SPEED_LIMIT 2500.0f
#define CHASSIS_H_CALIB_AXIS_COUNT 3U
#define CHASSIS_H_CALIB_SIGN_COUNT 2U
#define CHASSIS_H_CALIB_ROLLING_DIR_Q 0.70710678f
#define CHASSIS_H_CALIB_CMD_TO_MPS 6.8e-5f
#define CHASSIS_H_CALIB_CMD_TO_RADPS 1.3e-3f
#define CHASSIS_GRAVITY_FF_DEADZONE_RAD (5.0f * DEGREE_2_RAD)
#define CHASSIS_RAD_TO_DEG 57.2957795f
#define HOLE_YAW_MOTOR_TARGET_DEG ((HOLE_YAW_MOTOR_TARGET_RAD * CHASSIS_RAD_TO_DEG) + 360.0f)
#define CHASSIS_REACHABLE_FLASH_INVALID_SLOT 0xFFU
#define CHASSIS_REACHABLE_FLASH_INVALID_POWER 0xFFFFU
#define CHASSIS_ROTATE_VXY_ACTIVE_EPS 1.0f
#define CHASSIS_ROTATE_WZ_CMD_TO_RADPS 1.3e-3f
#define CHASSIS_SUPERCAP_POWER_FEEDBACK_OFFSET_W 5.0f

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Subscriber_t *gimbal_feed_sub;               // 用于订阅云台反馈中的底盘 IMU 指针
                                        
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static Gimbal_Upload_Data_s chassis_gimbal_feedback; // 云台反馈缓存，用于读取 gimbal_IMU_data

static SuperCapInstance *supercap;                                        // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
extern referee_info_t *referee_data_for_ui;
uint16_t power_supdata_watch  ;
volatile uint8_t superCap_watchdog;
// 为了方便调试加入的量
static uint8_t center_gimbal_offset_x = CENTER_GIMBAL_OFFSET_X; // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
static uint8_t center_gimbal_offset_y = CENTER_GIMBAL_OFFSET_Y; // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0

static INS_Instance *Chassis_IMU_data; // 底盘IMU数据
// extern INS_Instance *gimbal_IMU_data; // 云台IMU数据（预留）
extern RC_ctrl_t rc_ctrl[2]; 
// 跟随模式底盘的pid
// 目前没有设置单位，有些不规范，之后有需要再改
// 跟随 / 狗洞模式共用同一个 PID 实例:
//   - FOLLOW: setpoint = 0 (offset_angle 闭环到 0,底盘对齐云台)
//   - HOLE  : 先闭 offset_angle 到 -64°；down 到 0rad 后用 yaw_control
//             作为目标、RS485 yaw_angle 作为反馈闭环输出 wz
// Ki=0 + Derivative_On_Measurement 保证 setpoint 跳变时无积分残留 / 无微分踢。
static PIDInstance Chassis_Follow_PID = {
    .Kp            = 150.0f,
    .Ki            = 0,
    .Kd            = 1,
    .DeadBand      = 1.5f,
    .IntegralLimit = 0,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 20000.0f,
};
static uint8_t chassis_hole_yaw_total_stage_last = 0U;

static float ChassisWrap180Deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void ChassisFollowPIDClearRuntime(void)
{
    Chassis_Follow_PID.Measure = 0.0f;
    Chassis_Follow_PID.Last_Measure = 0.0f;
    Chassis_Follow_PID.Err = 0.0f;
    Chassis_Follow_PID.Last_Err = 0.0f;
    Chassis_Follow_PID.Last_ITerm = 0.0f;
    Chassis_Follow_PID.Pout = 0.0f;
    Chassis_Follow_PID.Iout = 0.0f;
    Chassis_Follow_PID.Dout = 0.0f;
    Chassis_Follow_PID.ITerm = 0.0f;
    Chassis_Follow_PID.Output = 0.0f;
    Chassis_Follow_PID.Last_Output = 0.0f;
    Chassis_Follow_PID.Last_Dout = 0.0f;
    Chassis_Follow_PID.Ref = 0.0f;
    Chassis_Follow_PID.dt = 0.0f;
    (void)DWT_GetDeltaT(&Chassis_Follow_PID.DWT_CNT);
}

static uint8_t ChassisFollowPIDResetOnHoleYawTotalStage(uint8_t yaw_total_stage)
{
    yaw_total_stage = (yaw_total_stage != 0U) ? 1U : 0U;
    if (yaw_total_stage != chassis_hole_yaw_total_stage_last) {
        ChassisFollowPIDClearRuntime();
        chassis_hole_yaw_total_stage_last = yaw_total_stage;
        return 1U;
    }

    return 0U;
}
/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy, chassis_vw; // 将云台系的速度投影到底盘
static float chassis_vx_cmd_prev = 0.0f, chassis_vy_cmd_prev = 0.0f, chassis_wz_cmd_prev = 0.0f; // 命令系下 vx/vy/wz 的上一拍限幅输出，给斜率限制留状态
static float chassis_vx_body_prev = 0.0f, chassis_vy_body_prev = 0.0f, chassis_wz_body_prev = 0.0f; // 底盘系二级斜率限制状态
static float vt_lf, vt_rf, vt_lb, vt_rb;         // omni 控制器输出的轮端速度参考，发送给 DJI 前需要乘减速比
static float kl=1;
extern INS_Instance *INS;
static float Power_Output;
static float chassis_speed_current_ff_lf = 0.0f;
static float chassis_speed_current_ff_rf = 0.0f;
static float chassis_speed_current_ff_rb = 0.0f;
static float chassis_speed_current_ff_lb = 0.0f;

static OmniVelocityCtrl_s omni_ctrl;
static OmniVelocityCtrlOutput_s omni_output;
static uint32_t omni_ctrl_dwt_cnt;
static uint8_t chassis_lqi_current_active = 0U;
static uint8_t chassis_rotate_power_priority_active = 0U;
volatile OmniHCalibCmd_s g_omni_h_calib_cmd = {
    .enable = 0U,
    .start = 0U,
    .abort = 0U,
    .run_id = 0U,
    .amp_vx = CHASSIS_H_CALIB_DEFAULT_AMP_VXY,
    .amp_vy_left = CHASSIS_H_CALIB_DEFAULT_AMP_VXY,
    .amp_wz = CHASSIS_H_CALIB_DEFAULT_AMP_WZ,
    .zero_ticks = CHASSIS_H_CALIB_DEFAULT_ZERO_TICKS,
    .ramp_ticks = CHASSIS_H_CALIB_DEFAULT_RAMP_TICKS,
    .hold_ticks = CHASSIS_H_CALIB_DEFAULT_HOLD_TICKS,
    .repeat_count = CHASSIS_H_CALIB_DEFAULT_REPEAT_COUNT,
    .wheel_speed_limit = CHASSIS_H_CALIB_DEFAULT_WHEEL_SPEED_LIMIT,
};
volatile OmniHCalibRecord_s g_omni_h_calib_record;
volatile ChassisLqiCurrentCmd_s g_chassis_lqi_current_cmd = {
    .enable = 0U,
    .current_offset_enable = 1U,
    .eso_compensate_enable = 1U,
    .current_limit = 16384.0f,
    .current_rate_limit = 700000.0f,
    .kp_vx = 8.0f,
    .kp_vy = 8.0f,
    .kp_wz = 6.0f,
    .ki_vx = 0.0f,
    .ki_vy = 0.0f,
    .ki_wz = 0.0f,
    .eso_comp_vx = 0.5f,
    .eso_comp_vy = 0.5f,
    .eso_comp_wz = 0.3f,
};
volatile ChassisCurrentForceCalibCmd_s g_chassis_force_calib_cmd = {
    .enable = 0U,
    .wheel_index = 0xFFU,
    .current_cmd_fit = 0.0f,
    .current_limit = CHASSIS_FORCE_CALIB_DEFAULT_CURRENT_LIMIT,
};
volatile ChassisCurrentForceCalibRecord_s g_chassis_force_calib_record;
volatile ChassisOmniForceDebug_s g_chassis_omni_force_debug;
volatile ChassisReachableFlashDebug_s g_chassis_reachable_flash_debug = {
    .slot_index = CHASSIS_REACHABLE_FLASH_INVALID_SLOT,
};
volatile float chassis_vxy_body_max_accel = 300000.0f;
volatile float chassis_wz_body_max_accel = 1000000.0f;
volatile uint8_t chassis_gravity_ff_enable = 1U;
volatile float chassis_gravity_ff_gain = 1.0f;
volatile float chassis_gravity_ff_pitch_rad = 0.0f;
volatile float chassis_gravity_ff_roll_rad = 0.0f;
volatile float chassis_gravity_ff_force_x_n = 0.0f;
volatile float chassis_gravity_ff_force_y_right_n = 0.0f;
volatile float chassis_rotate_wz_search_cmd = 12000.0f;
volatile uint8_t chassis_rotate_speed_allocation_enable = 0U;
static uint8_t chassis_force_calib_active = 0U;
static ChassisForceCalibAutoState_e chassis_force_calib_auto_state = CHASSIS_FORCE_CALIB_AUTO_IDLE;
static float chassis_force_calib_state_start_time_s = 0.0f;
static float chassis_force_calib_trigger_time_s = 0.0f;
static float chassis_force_calib_pulse_elapsed_s = 0.0f;
static float chassis_force_calib_target_current_cmd_fit = 0.0f;
static float chassis_force_calib_applied_current_cmd_fit = 0.0f;
static uint32_t chassis_force_calib_trial_id = 0U;
static uint8_t chassis_force_calib_run_wheel_select = CHASSIS_FORCE_CALIB_ALL_WHEELS;
static uint8_t chassis_force_calib_wheel_index = 0U;
static uint8_t chassis_force_calib_direction_index = 0U;
static uint8_t chassis_force_calib_current_level_index = 0U;
static uint8_t chassis_force_calib_repeat_index = 0U;
static uint8_t chassis_force_calib_pulse_done = 0U;
static uint8_t chassis_force_calib_hard_timeout = 0U;
static uint8_t chassis_force_calib_auto_done = 0U;
static uint8_t chassis_force_calib_auto_abort = 0U;
static ChassisForceCalibAbortReason_e chassis_force_calib_abort_reason =
    CHASSIS_FORCE_CALIB_ABORT_NONE;
static OmniHCalibState_e chassis_h_calib_state = OMNI_H_CALIB_STATE_IDLE;
static uint32_t chassis_h_calib_state_tick = 0U;
static uint32_t chassis_h_calib_segment_index = 0U;
static uint32_t chassis_h_calib_repeat_index = 0U;
static OmniHCalibAxis_e chassis_h_calib_axis = OMNI_H_CALIB_AXIS_VX;
static int8_t chassis_h_calib_sign = 1;
static uint8_t chassis_h_calib_start_last = 0U;
static float chassis_h_calib_cmd_vx = 0.0f;
static float chassis_h_calib_cmd_vy_left = 0.0f;
static float chassis_h_calib_cmd_wz = 0.0f;
static float chassis_h_calib_record_dt_s = 0.0f;
static uint16_t chassis_reachable_active_power_limit = CHASSIS_REACHABLE_FLASH_INVALID_POWER;
static uint8_t chassis_reachable_pending_save = 0U;
static const float chassis_force_calib_current_levels[CHASSIS_FORCE_CALIB_CURRENT_LEVEL_NUM] = {
    2500.0f,
    3000.0f,
    3500.0f,
    4000.0f,
    4500.0f,
};

static void ChassisReachableInvalidateRuntimeCache(void);
static void ChassisReachableHandlePowerLimitRuntime(uint16_t power_limit);

/*电流-轮力标定数据：
vx：
2000:2.085-3.8
3000:3.36-5.0
4000:3.82-6.3
vy：
2000:1.48-3.38
3000:2.31-5
4000：4.375-5.44
5000:4.6-6.5
*/
/*
 * 四轮共同测得的整车合力标定结果，拉力计两端读数取中值作为轮力本体。
 * vx 中值拟合约为 76.5 cmd/N，这里先写入保守值 80 cmd/N。
 * vy 中值拟合约为 87.1 cmd/N，这里先写入保守值 88 cmd/N。
 * 半差只反映地面静摩擦和滞回，不作为轮力系数使用。
 */
const float CHASSIS_FORCE_CALIB_X_CMD_PER_N = 80.0f;
const float CHASSIS_FORCE_CALIB_Y_CMD_PER_N = 88.0f;
const float CHASSIS_FORCE_CALIB_X_N_PER_CMD = 1.0f / 80.0f;
const float CHASSIS_FORCE_CALIB_Y_N_PER_CMD = 1.0f / 88.0f;

void ChassisInit()
{
//     ;
// #if defined(CHASSIS_BOARD)
//     BMI088_Init_Config_s config = {
//         .acc_int_config  = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_4},
//         .gyro_int_config = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_5},
//         .heat_pid_config = {
//             .Kp            = 0.32f,
//             .Ki            = 0.0004f,
//             .Kd            = 0,
//             .Improve       = PID_IMPROVE_NONE,
//             .IntegralLimit = 0.90f,
//             .MaxOut        = 0.95f,
//         },
//         .heat_pwm_config = {
//             .htim      = &htim10,
//             .channel   = TIM_CHANNEL_1,
//             .dutyratio = 0,
//             .period    = 5000 - 1,
//         },
//         .spi_acc_config = {
//             .GPIOx      = GPIOA,
//             .cs_pin     = GPIO_PIN_4,
//             .spi_handle = &hspi1,
//         },
//         .spi_gyro_config = {
//             .GPIOx      = GPIOB,
//             .cs_pin     = GPIO_PIN_0,
//             .spi_handle = &hspi1,
//         },
//         .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
//         //.cali_mode = BMI088_LOAD_PRE_CALI_MODE,
//         .work_mode = BMI088_BLOCK_PERIODIC_MODE,
// };
//     Chassis_IMU_data = INS_Init(BMI088Register(&config)); // IMU初始化，获取底盘姿态反馈（含yaw角速度）
// #endif
    #if defined(CHASSIS_BOARD)
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config_1 = {
        .can_init_config.can_handle   = &hcan2,
        .controller_param_init_config = {
            .current_feedforward_ptr = &chassis_speed_current_ff_lf,
            .speed_PID = {
                .Kp            = 3 ,// 4.0
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s chassis_motor_config_2 = {
        .can_init_config.can_handle   = &hcan2,
        .controller_param_init_config = {
            .current_feedforward_ptr = &chassis_speed_current_ff_rf,
            .speed_PID = {
                .Kp            = 3 ,// 4.5 , 3.75
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
     Motor_Init_Config_s chassis_motor_config_3 = {
        .can_init_config.can_handle   = &hcan2,
        .controller_param_init_config = {
            .current_feedforward_ptr = &chassis_speed_current_ff_rb,
            .speed_PID = {
                .Kp            = 3 ,// 4.5
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
     Motor_Init_Config_s chassis_motor_config_4 = {
        .can_init_config.can_handle   = &hcan2,
        .controller_param_init_config = {
            .current_feedforward_ptr = &chassis_speed_current_ff_lb,
            .speed_PID = {
                .Kp            = 3 ,// 4.5
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config_1.can_init_config.tx_id                             = 1;
    chassis_motor_config_1.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config_1);
    
    chassis_motor_config_2.can_init_config.tx_id                             = 2;
    chassis_motor_config_2.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config_2);
    
    chassis_motor_config_3.can_init_config.tx_id                             = 3;
    chassis_motor_config_3.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config_3);

    chassis_motor_config_4.can_init_config.tx_id                             = 4;
    chassis_motor_config_4.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config_4);
    if (motor_lf != NULL) {
        motor_lf->motor_controller.current_feedforward_ptr = &chassis_speed_current_ff_lf;
    }
    if (motor_rf != NULL) {
        motor_rf->motor_controller.current_feedforward_ptr = &chassis_speed_current_ff_rf;
    }
    if (motor_rb != NULL) {
        motor_rb->motor_controller.current_feedforward_ptr = &chassis_speed_current_ff_rb;
    }
    if (motor_lb != NULL) {
        motor_lb->motor_controller.current_feedforward_ptr = &chassis_speed_current_ff_lb;
    }
    
    SuperCap_Init_Config_s supercap_config = {
        .can_config = {
            .can_handle = &hcan1,
        },
    };
    supercap = SuperCapInit(&supercap_config);
    // (void)supercap_config;

    #endif


    // motor_lb->motor_controller.speed_PID.Iout=0;motor_rb->motor_controller.speed_PID.Iout=0;
    // motor_lf->motor_controller.speed_PID.Iout=0;motor_rf->motor_controller.speed_PID.Iout=0;

#if defined(CHASSIS_BOARD)
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // CHASSIS_BOARD
    OmniVelocityCtrl_Init(&omni_ctrl, NULL);
    ChassisReachableInvalidateRuntimeCache();
    (void)DWT_GetDeltaT(&omni_ctrl_dwt_cnt);
    ;
}

#define LF_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
float vxy_max_accel = 150000.0f;
float wz_max_accel = 500000.0f; // wz 命令斜率限制，cmd/s；1 cmd ≈ 1.3e-3 rad/s，故 100000 ≈ 130 rad/s²
volatile float chassis_direct_speed_pid_maxout = 6400.0f;
volatile uint8_t chassis_direct_rf_feedback_reverse = 0U;
static inline float SlewRateLimit(float target, float prev, float max_step)
{
    float delta = target - prev;
    if (delta >  max_step) delta =  max_step;
    if (delta < -max_step) delta = -max_step;
    return prev + delta;
}

static inline float ChassisClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint8_t ChassisFloatIsFinite(float value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

static float ChassisAbsFiniteFloat(float value, float fallback)
{
    if (ChassisFloatIsFinite(value) == 0U) {
        return fallback;
    }
    return fabsf(value);
}

static float ChassisInterpolateRotateWzGyro(uint16_t power_limit)
{
    typedef struct {
        float power_w;
        float gyro_rad_s;
    } ChassisRotateWzPoint_s;

    static const ChassisRotateWzPoint_s points[] = {
        {45.0f, 6.0f},
        {60.0f, 7.5f},
        {70.0f, 8.0f},
        {80.0f, 9.5f},
        {100.0f, 10.0f},
        {165.0f, 14.0f},
    };
    const uint8_t point_count = (uint8_t)(sizeof(points) / sizeof(points[0]));
    const float power_w = (float)power_limit;

    if (power_w <= points[0].power_w) {
        return points[0].gyro_rad_s;
    }
    for (uint8_t i = 1U; i < point_count; ++i) {
        if (power_w <= points[i].power_w) {
            const float span_w = points[i].power_w - points[i - 1U].power_w;
            const float ratio = (span_w > 1.0e-6f) ?
                ((power_w - points[i - 1U].power_w) / span_w) : 0.0f;

            return points[i - 1U].gyro_rad_s +
                   ratio * (points[i].gyro_rad_s - points[i - 1U].gyro_rad_s);
        }
    }
    return points[point_count - 1U].gyro_rad_s;
}

static float ChassisRotateGyroToWzCmd(float gyro_rad_s)
{
    if (ChassisFloatIsFinite(gyro_rad_s) == 0U || gyro_rad_s <= 0.0f) {
        return 0.0f;
    }
    return gyro_rad_s / CHASSIS_ROTATE_WZ_CMD_TO_RADPS;
}

static uint8_t ChassisRotateVxyActive(float vx, float vy)
{
    return (fabsf(vx) > CHASSIS_ROTATE_VXY_ACTIVE_EPS ||
            fabsf(vy) > CHASSIS_ROTATE_VXY_ACTIVE_EPS) ? 1U : 0U;
}

static uint8_t ChassisDirectInputIdle(float vx, float vy_left, float wz)
{
    return (fabsf(vx) <= CHASSIS_ROTATE_VXY_ACTIVE_EPS &&
            fabsf(vy_left) <= CHASSIS_ROTATE_VXY_ACTIVE_EPS &&
            fabsf(wz) <= CHASSIS_ROTATE_VXY_ACTIVE_EPS) ? 1U : 0U;
}

static void ChassisReachableFlashSetDebug(uint16_t power_limit,
                                          uint8_t load_valid,
                                          uint8_t slot_index)
{
    g_chassis_reachable_flash_debug.power_limit = power_limit;
    g_chassis_reachable_flash_debug.load_valid = load_valid;
    g_chassis_reachable_flash_debug.pending_save = chassis_reachable_pending_save;
    g_chassis_reachable_flash_debug.slot_index = slot_index;
}

static void ChassisReachableInvalidateRuntimeCache(void)
{
    OmniVelocityCtrl_ClearReachableCache(&omni_ctrl);
    chassis_reachable_active_power_limit = CHASSIS_REACHABLE_FLASH_INVALID_POWER;
    chassis_reachable_pending_save = 0U;
    ChassisReachableFlashSetDebug(CHASSIS_REACHABLE_FLASH_INVALID_POWER,
                                  0U,
                                  CHASSIS_REACHABLE_FLASH_INVALID_SLOT);
}

static void ChassisReachableHandlePowerLimitRuntime(uint16_t power_limit)
{
    if (power_limit == chassis_reachable_active_power_limit) {
        return;
    }
    chassis_reachable_active_power_limit = power_limit;
    chassis_reachable_pending_save = 0U;
    OmniVelocityCtrl_ClearReachableCache(&omni_ctrl);
    ChassisReachableFlashSetDebug(power_limit,
                                  0U,
                                  CHASSIS_REACHABLE_FLASH_INVALID_SLOT);
}

static void ChassisApplyBodyFrameSlewLimit(float dt)
{
    if (ChassisFloatIsFinite(dt) == 0U || dt <= 0.0f) {
        chassis_vx_body_prev = chassis_vx;
        chassis_vy_body_prev = chassis_vy;
        chassis_wz_body_prev = chassis_cmd_recv.wz;
        return;
    }

    const float vxy_body_max_step =
        ChassisAbsFiniteFloat((float)chassis_vxy_body_max_accel, 300000.0f) * dt;
    const float wz_body_max_step =
        ChassisAbsFiniteFloat((float)chassis_wz_body_max_accel, 1000000.0f) * dt;

    chassis_vx = SlewRateLimit(chassis_vx, chassis_vx_body_prev, vxy_body_max_step);
    chassis_vy = SlewRateLimit(chassis_vy, chassis_vy_body_prev, vxy_body_max_step);
    chassis_cmd_recv.wz = SlewRateLimit(chassis_cmd_recv.wz, chassis_wz_body_prev, wz_body_max_step);

    chassis_vx_body_prev = chassis_vx;
    chassis_vy_body_prev = chassis_vy;
    chassis_wz_body_prev = chassis_cmd_recv.wz;
}

static void ChassisClearSpeedCurrentFeedforward(void);

static void ChassisZeroWheelRef(void)
{
    chassis_vx = 0.0f;
    chassis_vy = 0.0f;
    chassis_vw = 0.0f;
    vt_lf = 0.0f;
    vt_rf = 0.0f;
    vt_lb = 0.0f;
    vt_rb = 0.0f;
    chassis_vx_cmd_prev = 0.0f;
    chassis_vy_cmd_prev = 0.0f;
    chassis_wz_cmd_prev = 0.0f;
    chassis_vx_body_prev = 0.0f;
    chassis_vy_body_prev = 0.0f;
    chassis_wz_body_prev = 0.0f;
    ChassisClearSpeedCurrentFeedforward();
}

static void ChassisStopWheelMotors(void)
{
    if (motor_lf != NULL) {
        DJIMotorStop(motor_lf);
        motor_lf->motor_controller.speed_PID.MaxOut = 0.0f;
    }
    if (motor_rf != NULL) {
        DJIMotorStop(motor_rf);
        motor_rf->motor_controller.speed_PID.MaxOut = 0.0f;
    }
    if (motor_lb != NULL) {
        DJIMotorStop(motor_lb);
        motor_lb->motor_controller.speed_PID.MaxOut = 0.0f;
    }
    if (motor_rb != NULL) {
        DJIMotorStop(motor_rb);
        motor_rb->motor_controller.speed_PID.MaxOut = 0.0f;
    }
}

static void ChassisEnableWheelMotors(void)
{
    if (motor_lf != NULL) {
        DJIMotorEnable(motor_lf);
        motor_lf->motor_controller.speed_PID.MaxOut = 16384.0f;
    }
    if (motor_rf != NULL) {
        DJIMotorEnable(motor_rf);
        motor_rf->motor_controller.speed_PID.MaxOut = 16384.0f;
    }
    if (motor_lb != NULL) {
        DJIMotorEnable(motor_lb);
        motor_lb->motor_controller.speed_PID.MaxOut = 16384.0f;
    }
    if (motor_rb != NULL) {
        DJIMotorEnable(motor_rb);
        motor_rb->motor_controller.speed_PID.MaxOut = 16384.0f;
    }
}

static void ChassisSetWheelSpeedPidMaxOut(float max_out)
{
    max_out = ChassisClampFloat(ChassisAbsFiniteFloat(max_out, 3000.0f),
                                0.0f,
                                16384.0f);
    if (motor_lf != NULL) {
        motor_lf->motor_controller.speed_PID.MaxOut = max_out;
    }
    if (motor_rf != NULL) {
        motor_rf->motor_controller.speed_PID.MaxOut = max_out;
    }
    if (motor_lb != NULL) {
        motor_lb->motor_controller.speed_PID.MaxOut = max_out;
    }
    if (motor_rb != NULL) {
        motor_rb->motor_controller.speed_PID.MaxOut = max_out;
    }
}

static float ChassisMotorSpeedToWheelSpeed(float motor_speed);
static float ChassisWheelSpeedToMotorSpeed(float wheel_speed);

/*
 * 底盘输出最后一跳：
 * - LQI 电流接管时，四个 DJI 电机已经切到 OPEN_LOOP，直接写入 raw current cmd。
 * - 回退速度环时，vt_* 是轮端速度参考，发给 DJI 前必须乘 268/17 换回电机轴速度。
 */
static void ChassisApplyWheelOutput(void)
{
    if (chassis_lqi_current_active != 0U) {
        if (motor_lf != NULL) {
            DJIMotorSetRef(motor_lf, omni_output.wheel_current_ref[OMNI_WHEEL_LF]);
        }
        if (motor_rf != NULL) {
            DJIMotorSetRef(motor_rf, omni_output.wheel_current_ref[OMNI_WHEEL_RF]);
        }
        if (motor_lb != NULL) {
            DJIMotorSetRef(motor_lb, omni_output.wheel_current_ref[OMNI_WHEEL_LB]);
        }
        if (motor_rb != NULL) {
            DJIMotorSetRef(motor_rb, omni_output.wheel_current_ref[OMNI_WHEEL_RB]);
        }
        return;
    }

    omni_output.wheel_current_ref[OMNI_WHEEL_LF] = chassis_speed_current_ff_lf;
    omni_output.wheel_current_ref[OMNI_WHEEL_RF] = chassis_speed_current_ff_rf;
    omni_output.wheel_current_ref[OMNI_WHEEL_RB] = chassis_speed_current_ff_rb;
    omni_output.wheel_current_ref[OMNI_WHEEL_LB] = chassis_speed_current_ff_lb;

    if (motor_lf != NULL) {
        DJIMotorSetRef(motor_lf, ChassisWheelSpeedToMotorSpeed(vt_lf));
    }
    if (motor_rf != NULL) {
        DJIMotorSetRef(motor_rf, ChassisWheelSpeedToMotorSpeed(vt_rf));
    }
    if (motor_lb != NULL) {
        DJIMotorSetRef(motor_lb, ChassisWheelSpeedToMotorSpeed(vt_lb));
    }
    if (motor_rb != NULL) {
        DJIMotorSetRef(motor_rb, ChassisWheelSpeedToMotorSpeed(vt_rb));
    }
}

static uint8_t ChassisClampSuperCapPowerLimit(uint16_t power_limit)
{
    return (power_limit > 255U) ? 255U : (uint8_t)power_limit;
}

static uint8_t ChassisSuperCapIsReady(void)
{
    if (supercap == NULL || SuperCapIsOnline(supercap) == 0U) {
        return 0U;
    }

    return (SuperCapGetReadyFlag(supercap) != 0U) ? 1U : 0U;
}

static float ChassisGetSuperCapChassisRealPower(void)
{
    if (supercap == NULL) {
        return 0.0f;
    }

    return SuperCapGetChassisRealPower(supercap) + CHASSIS_SUPERCAP_POWER_FEEDBACK_OFFSET_W;
}

static void ChassisUpdateSuperCapControl(void)
{
    if (supercap == NULL) {
        return;
    }

    SuperCapEnable(supercap);
    SuperCapSetPowerLimit(supercap,
                          ChassisClampSuperCapPowerLimit(chassis_cmd_recv.supercap_power_limit));
    SuperCapTask();
}

static void ChassisPublishFeedback(void)
{
    float cap_voltage = 0.0f;
    uint8_t cap_online = 0U;
    uint8_t cap_ready = 0U;
    uint8_t cap_active = 0U;

    Power_Output = 0.0f;
    if (supercap != NULL) {
        cap_online = SuperCapIsOnline(supercap);
        if (cap_online != 0U) {
            cap_ready = (SuperCapGetReadyFlag(supercap) != 0U) ? 1U : 0U;
            cap_active = (cap_ready != 0U &&
                          chassis_cmd_recv.SuperCap_flag_from_user == SUPERCAP_USE) ? 1U : 0U;
            cap_voltage = SuperCapGetBatteryVoltage(supercap);
            if (cap_ready != 0U) {
                Power_Output = ChassisGetSuperCapChassisRealPower();
            }
        }
    }

    memcpy(&chassis_feedback_data.cap_voltage, &cap_voltage, sizeof(float));
    memcpy(&chassis_feedback_data.cap_online_flag, &cap_online, sizeof(uint8_t));
    memcpy(&chassis_feedback_data.cap_ready_flag, &cap_ready, sizeof(uint8_t));
    memcpy(&chassis_feedback_data.cap_active_flag, &cap_active, sizeof(uint8_t));
    memcpy(&chassis_feedback_data.chassis_power_output, &Power_Output, sizeof(float));
    memcpy(&chassis_feedback_data.speed[0], &motor_lf->measure.speed_aps, sizeof(float));
    memcpy(&chassis_feedback_data.speed[1], &motor_rf->measure.speed_aps, sizeof(float));
    memcpy(&chassis_feedback_data.speed[2], &motor_lb->measure.speed_aps, sizeof(float));
    memcpy(&chassis_feedback_data.speed[3], &motor_rb->measure.speed_aps, sizeof(float));
    memcpy(&chassis_feedback_data.current[0], &motor_lf->measure.real_current, sizeof(float));
    memcpy(&chassis_feedback_data.current[1], &motor_rf->measure.real_current, sizeof(float));
    memcpy(&chassis_feedback_data.current[2], &motor_lb->measure.real_current, sizeof(float));
    memcpy(&chassis_feedback_data.current[3], &motor_rb->measure.real_current, sizeof(float));
    memcpy(&chassis_feedback_data.super_cap_power, &Power_Output, sizeof(float));
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}

static float ChassisMotorSpeedToWheelSpeed(float motor_speed)
{
    // DJI 反馈是电机轴速度，底盘控制器内部统一使用轮端输出轴速度。
    return motor_speed / CHASSIS_WHEEL_REDUCTION_RATIO;
}

static float ChassisWheelSpeedToMotorSpeed(float wheel_speed)
{
    // DJI 速度环参考仍是电机轴速度，发送前按 268/17 减速比换回电机轴单位。
    return wheel_speed * CHASSIS_WHEEL_REDUCTION_RATIO;
}

static float ChassisGetWheelSpeed(const DJIMotorInstance *motor)
{
    return (motor != NULL) ? ChassisMotorSpeedToWheelSpeed(motor->measure.speed_aps) : 0.0f;
}

static DJIMotorInstance *ChassisMotorByOmniWheel(uint8_t wheel)
{
    switch (wheel) {
        case OMNI_WHEEL_LF:
            return motor_lf;
        case OMNI_WHEEL_RF:
            return motor_rf;
        case OMNI_WHEEL_RB:
            return motor_rb;
        case OMNI_WHEEL_LB:
            return motor_lb;
        default:
            return NULL;
    }
}

static void ChassisFillSpeedPowerModelInput(OmniVelocityCtrlInput_s *input)
{
    if (input == NULL) {
        return;
    }

    input->speed_power_model_enable = (chassis_lqi_current_active == 0U) ? 1U : 0U;
    input->wheel_to_motor_speed_ratio = CHASSIS_WHEEL_REDUCTION_RATIO;

    for (uint8_t wheel = 0U; wheel < OMNI_WHEEL_NUM; ++wheel) {
        DJIMotorInstance *motor = ChassisMotorByOmniWheel(wheel);

        if (motor == NULL) {
            input->speed_power_kp[wheel] = 0.0f;
            input->speed_power_iout[wheel] = 0.0f;
            input->speed_power_deadband[wheel] = 0.0f;
            input->speed_power_output_limit[wheel] = 0.0f;
            continue;
        }

        input->speed_power_kp[wheel] = motor->motor_controller.speed_PID.Kp;
        input->speed_power_iout[wheel] = motor->motor_controller.speed_PID.Iout;
        input->speed_power_deadband[wheel] = motor->motor_controller.speed_PID.DeadBand;
        input->speed_power_output_limit[wheel] = motor->motor_controller.speed_PID.MaxOut;
    }
}

static void ChassisInvalidateOmniForceDebug(float dt)
{
    volatile ChassisOmniForceDebug_s *record = &g_chassis_omni_force_debug;

    record->sample_count++;
    record->valid = 0U;
    record->dt_s = dt;
    record->chassis_mode = (uint8_t)chassis_cmd_recv.chassis_mode;
    record->lqi_active = chassis_lqi_current_active;
    record->current_offset_enable =
        (g_chassis_lqi_current_cmd.current_offset_enable != 0U) ? 1U : 0U;
    record->limit_active = 0U;
    record->current_limit_active = 0U;
    record->power_limit_active = 0U;
    record->reachable_vxy_valid = 0U;
    record->reachable_wz_valid = 0U;
    record->rotate_strategy_active = 0U;
    record->reachable_vxy_max = 0.0f;
    record->reachable_wz_max = 0.0f;
    record->reachable_vxy_dir_x = 0.0f;
    record->reachable_vxy_dir_y = 0.0f;
}

static void ChassisUpdateOmniForceDebug(float dt,
                                        float cmd_vx_pre_slew,
                                        float cmd_vy_pre_slew,
                                        float cmd_wz_pre_slew)
{
    volatile ChassisOmniForceDebug_s *record = &g_chassis_omni_force_debug;
    const uint8_t current_offset_enable =
        (g_chassis_lqi_current_cmd.current_offset_enable != 0U) ? 1U : 0U;

    record->sample_count++;
    record->valid = 1U;
    record->dt_s = dt;
    record->chassis_mode = (uint8_t)chassis_cmd_recv.chassis_mode;
    record->lqi_active = chassis_lqi_current_active;
    record->current_offset_enable = current_offset_enable;
    record->limit_active = omni_output.limit_active;
    record->current_limit_active = omni_output.current_limit_active;
    record->power_limit_active = omni_output.power_limit_active;
    record->reachable_vxy_valid = omni_output.reachable_vxy_valid;
    record->reachable_wz_valid = omni_output.reachable_wz_valid;
    record->rotate_strategy_active = omni_output.rotate_strategy_active;
    record->current_limit = g_chassis_lqi_current_cmd.current_limit;
    record->power_vxy_scale = omni_output.power_vxy_scale;
    record->power_wz_scale = omni_output.power_wz_scale;
    record->power_effective_limit_w = omni_output.power_effective_limit_w;
    record->power_real_w = omni_output.power_real_w;
    record->reachable_vxy_max = omni_output.reachable_vxy_max;
    record->reachable_wz_max = omni_output.reachable_wz_max;
    record->reachable_vxy_dir_x = omni_output.reachable_vxy_dir_x;
    record->reachable_vxy_dir_y = omni_output.reachable_vxy_dir_y;
    record->cmd_vx_pre_slew = cmd_vx_pre_slew;
    record->cmd_vy_pre_slew = cmd_vy_pre_slew;
    record->cmd_wz_pre_slew = cmd_wz_pre_slew;
    record->cmd_vx = chassis_cmd_recv.vx;
    record->cmd_vy = chassis_cmd_recv.vy;
    record->cmd_wz = chassis_cmd_recv.wz;
    record->body_vx = chassis_vx;
    record->body_vy_right = chassis_vy;
    record->omni_vy_left = -chassis_vy;

    for (uint8_t axis = 0U; axis < OMNI_AXIS_NUM; ++axis) {
        record->nu_ref[axis] = omni_output.nu_ref[axis];
        record->nu_hat[axis] = omni_output.nu_hat[axis];
        record->lqi_error[axis] = omni_output.lqi_error[axis];
        record->wrench_des[axis] = omni_output.wrench_des[axis];
        record->wrench_act[axis] = omni_output.wrench_act[axis];
        record->wrench_err[axis] =
            omni_output.wrench_des[axis] - omni_output.wrench_act[axis];
    }

    for (uint8_t wheel = 0U; wheel < OMNI_WHEEL_NUM; ++wheel) {
        DJIMotorInstance *motor = ChassisMotorByOmniWheel(wheel);
        float current_limit = g_chassis_lqi_current_cmd.current_limit;
        float pos_k = omni_ctrl.config.force_to_cmd_pos[wheel];
        float neg_k = omni_ctrl.config.force_to_cmd_neg[wheel];
        float usable_current;

        if (current_limit <= 0.0f) {
            current_limit = omni_ctrl.config.wheel_current_limit[wheel];
        }
        if (pos_k <= 1.0e-6f) {
            pos_k = 1.0f;
        }
        if (neg_k <= 1.0e-6f) {
            neg_k = pos_k;
        }
        if (current_limit <= 0.0f) {
            current_limit = 30.0f * pos_k;
        }

        usable_current = fabsf(current_limit);
        if (current_offset_enable != 0U) {
            usable_current -= fabsf(omni_ctrl.config.current_offset[wheel]);
            if (usable_current < 0.0f) {
                usable_current = 0.0f;
            }
        }

        record->force_to_cmd_pos[wheel] = omni_ctrl.config.force_to_cmd_pos[wheel];
        record->force_to_cmd_neg[wheel] = omni_ctrl.config.force_to_cmd_neg[wheel];
        record->force_bound_pos[wheel] = usable_current / pos_k;
        record->force_bound_neg_abs[wheel] = usable_current / neg_k;
        record->force_cmd[wheel] = omni_output.force_cmd[wheel];
        record->wheel_current_ref[wheel] = omni_output.wheel_current_ref[wheel];
        record->real_current_fit[wheel] =
            (motor != NULL) ? omni_ctrl.config.wheel_dir[wheel] * (float)motor->measure.real_current : 0.0f;
        record->wheel_speed_fit[wheel] =
            (motor != NULL) ? omni_ctrl.config.wheel_dir[wheel] * ChassisGetWheelSpeed(motor) : 0.0f;
        record->slip_prob[wheel] = omni_output.slip_prob[wheel];
    }
}

static float ChassisClampAbs(float value, float max_abs)
{
    float limit = max_abs;

    if (limit < 0.0f) {
        limit = -limit;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void ChassisClearSpeedCurrentFeedforward(void)
{
    chassis_speed_current_ff_lf = 0.0f;
    chassis_speed_current_ff_rf = 0.0f;
    chassis_speed_current_ff_rb = 0.0f;
    chassis_speed_current_ff_lb = 0.0f;
}

static void ChassisSetWheelSpeedLoop(DJIMotorInstance *motor)
{
    if (motor == NULL) {
        return;
    }

    // 恢复原速度环路径：后续 ChassisApplyWheelOutput() 使用 wheel_speed_ref -> motor speed ref。
    DJIMotorSetRef(motor, 0.0f);
    motor->motor_controller.speed_PID.Iout = 0.0f;
    motor->motor_controller.speed_PID.ITerm = 0.0f;
    motor->motor_controller.speed_PID.Output = 0.0f;
    motor->motor_controller.speed_PID.Last_Output = 0.0f;
    motor->motor_settings.outer_loop_type = SPEED_LOOP;
    motor->motor_settings.close_loop_type = SPEED_LOOP;
    motor->motor_settings.feedforward_flag = CURRENT_FEEDFORWARD;
}

static void ChassisSetWheelOpenLoop(DJIMotorInstance *motor)
{
    if (motor == NULL) {
        return;
    }

    // LQI 电流模式绕过 DJI 速度 PID，DJIMotorSetRef() 写入的是原始电流命令。
    DJIMotorEnable(motor);
    DJIMotorSetRef(motor, 0.0f);
    motor->motor_controller.speed_PID.Iout = 0.0f;
    motor->motor_controller.speed_PID.ITerm = 0.0f;
    motor->motor_controller.speed_PID.Output = 0.0f;
    motor->motor_controller.speed_PID.Last_Output = 0.0f;
    motor->motor_settings.outer_loop_type = OPEN_LOOP;
    motor->motor_settings.close_loop_type = OPEN_LOOP;
    motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
}

static void ChassisSetAllWheelRefZero(void)
{
    if (motor_lf != NULL) {
        DJIMotorSetRef(motor_lf, 0.0f);
    }
    if (motor_rf != NULL) {
        DJIMotorSetRef(motor_rf, 0.0f);
    }
    if (motor_rb != NULL) {
        DJIMotorSetRef(motor_rb, 0.0f);
    }
    if (motor_lb != NULL) {
        DJIMotorSetRef(motor_lb, 0.0f);
    }
}

static void ChassisEnterLqiCurrentMode(void)
{
    // 进入 LQI 电流接管前清掉旧速度/电流参考，并重置 omni 内部积分、ESO、限流状态。
    ChassisZeroWheelRef();
    ChassisSetAllWheelRefZero();
    ChassisSetWheelOpenLoop(motor_lf);
    ChassisSetWheelOpenLoop(motor_rf);
    ChassisSetWheelOpenLoop(motor_rb);
    ChassisSetWheelOpenLoop(motor_lb);
    OmniVelocityCtrl_Reset(&omni_ctrl);
    ChassisReachableInvalidateRuntimeCache();
    chassis_lqi_current_active = 1U;
}

static void ChassisExitLqiCurrentMode(void)
{
    // 退出电流接管时先清零，再恢复四轮速度环，避免 raw current cmd 残留到速度环。
    ChassisSetAllWheelRefZero();
    ChassisSetWheelSpeedLoop(motor_lf);
    ChassisSetWheelSpeedLoop(motor_rf);
    ChassisSetWheelSpeedLoop(motor_rb);
    ChassisSetWheelSpeedLoop(motor_lb);
    ChassisZeroWheelRef();
    OmniVelocityCtrl_Reset(&omni_ctrl);
    ChassisReachableInvalidateRuntimeCache();
    chassis_lqi_current_active = 0U;
}

static void ChassisUpdateLqiCurrentMode(void)
{
    // 全局入口开关：enable=1 走 omni/LQI 电流链路，enable=0 回退到原速度环输出。
    if (g_chassis_lqi_current_cmd.enable != 0U) {
        if (chassis_lqi_current_active == 0U) {
            ChassisEnterLqiCurrentMode();
        }
    } else if (chassis_lqi_current_active != 0U) {
        ChassisExitLqiCurrentMode();
    }
}

static INS_Instance *ChassisSelectLqiImu(void)
{
    return (chassis_gimbal_feedback.gimbal_imu_data != NULL) ?
        chassis_gimbal_feedback.gimbal_imu_data : INS;
}

static void ChassisApplyLqiRuntimeConfig(void)
{
    omni_ctrl.config.kp[OMNI_AXIS_VX] = g_chassis_lqi_current_cmd.kp_vx;
    omni_ctrl.config.kp[OMNI_AXIS_VY] = g_chassis_lqi_current_cmd.kp_vy;
    omni_ctrl.config.kp[OMNI_AXIS_WZ] = g_chassis_lqi_current_cmd.kp_wz;
    omni_ctrl.config.ki[OMNI_AXIS_VX] = g_chassis_lqi_current_cmd.ki_vx;
    omni_ctrl.config.ki[OMNI_AXIS_VY] = g_chassis_lqi_current_cmd.ki_vy;
    omni_ctrl.config.ki[OMNI_AXIS_WZ] = g_chassis_lqi_current_cmd.ki_wz;
    omni_ctrl.config.eso_enable = 1U;
    omni_ctrl.config.eso_compensate_enable = g_chassis_lqi_current_cmd.eso_compensate_enable;
    omni_ctrl.config.eso_compensate_gain[OMNI_AXIS_VX] = g_chassis_lqi_current_cmd.eso_comp_vx;
    omni_ctrl.config.eso_compensate_gain[OMNI_AXIS_VY] = g_chassis_lqi_current_cmd.eso_comp_vy;
    omni_ctrl.config.eso_compensate_gain[OMNI_AXIS_WZ] = g_chassis_lqi_current_cmd.eso_comp_wz;
}

static void ChassisClearGravityFeedforwardDebug(void)
{
    chassis_gravity_ff_pitch_rad = 0.0f;
    chassis_gravity_ff_roll_rad = 0.0f;
    chassis_gravity_ff_force_x_n = 0.0f;
    chassis_gravity_ff_force_y_right_n = 0.0f;
}

static void ChassisDirectApplyWheelFeedbackDirection(void);

static void ChassisDirectIdleStop(void)
{
    chassis_cmd_recv.vx = 0.0f;
    chassis_cmd_recv.vy = 0.0f;
    chassis_cmd_recv.wz = 0.0f;
    ChassisZeroWheelRef();
    ChassisSetAllWheelRefZero();
    ChassisSetWheelSpeedLoop(motor_lf);
    ChassisSetWheelSpeedLoop(motor_rf);
    ChassisSetWheelSpeedLoop(motor_rb);
    ChassisSetWheelSpeedLoop(motor_lb);
    ChassisDirectApplyWheelFeedbackDirection();
    ChassisSetWheelSpeedPidMaxOut((float)chassis_direct_speed_pid_maxout);
    ChassisClearSpeedCurrentFeedforward();
    ChassisClearGravityFeedforwardDebug();
    OmniVelocityCtrl_Reset(&omni_ctrl);
    memset(&omni_output, 0, sizeof(omni_output));
}

static void ChassisDirectApplyWheelFeedbackDirection(void)
{
    if (motor_lf != NULL) {
        motor_lf->motor_settings.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    }
    if (motor_rf != NULL) {
        motor_rf->motor_settings.feedback_reverse_flag =
            (chassis_direct_rf_feedback_reverse != 0U) ?
            FEEDBACK_DIRECTION_REVERSE :
            FEEDBACK_DIRECTION_NORMAL;
    }
    if (motor_rb != NULL) {
        motor_rb->motor_settings.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    }
    if (motor_lb != NULL) {
        motor_lb->motor_settings.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    }
}

static float ChassisSpeedFeedforwardCurrentLimit(uint8_t wheel)
{
    float current_limit = g_chassis_lqi_current_cmd.current_limit;

    if (current_limit <= 0.0f || ChassisFloatIsFinite(current_limit) == 0U) {
        current_limit = omni_ctrl.config.wheel_current_limit[wheel];
    }
    if (current_limit <= 0.0f || ChassisFloatIsFinite(current_limit) == 0U) {
        current_limit = 3000.0f;
    }
    if (current_limit > 16384.0f) {
        current_limit = 16384.0f;
    }
    return fabsf(current_limit);
}

static float ChassisForceToSpeedFeedforwardCurrent(uint8_t wheel, float force_n)
{
    const float k = (force_n >= 0.0f) ?
        omni_ctrl.config.force_to_cmd_pos[wheel] :
        omni_ctrl.config.force_to_cmd_neg[wheel];
    float current_fit = 0.0f;

    if (ChassisFloatIsFinite(force_n) == 0U ||
        ChassisFloatIsFinite(k) == 0U ||
        k <= 1.0e-6f) {
        return 0.0f;
    }

    current_fit = k * force_n;
    current_fit = ChassisClampAbs(current_fit, ChassisSpeedFeedforwardCurrentLimit(wheel));
    return omni_ctrl.config.wheel_dir[wheel] * current_fit;
}

static void ChassisApplySpeedGravityCurrentFeedforward(float force_x_n,
                                                       float force_y_left_n)
{
    const float q_half = 0.5f * CHASSIS_H_CALIB_ROLLING_DIR_Q;
    const float force_lf = q_half * (force_x_n - force_y_left_n);
    const float force_rf = q_half * (force_x_n + force_y_left_n);
    const float force_rb = q_half * (force_x_n - force_y_left_n);
    const float force_lb = q_half * (force_x_n + force_y_left_n);

    chassis_speed_current_ff_lf =
        ChassisForceToSpeedFeedforwardCurrent(OMNI_WHEEL_LF, force_lf);
    chassis_speed_current_ff_rf =
        ChassisForceToSpeedFeedforwardCurrent(OMNI_WHEEL_RF, force_rf);
    chassis_speed_current_ff_rb =
        ChassisForceToSpeedFeedforwardCurrent(OMNI_WHEEL_RB, force_rb);
    chassis_speed_current_ff_lb =
        ChassisForceToSpeedFeedforwardCurrent(OMNI_WHEEL_LB, force_lb);
}

static uint8_t ChassisHoleGravityFeedforwardReady(void)
{
    if (chassis_cmd_recv.gimbal_mode != GIMBAL_HOLE_MODE) {
        return 0U;
    }

    if (chassis_cmd_recv.hole_offset_yaw_takeover != 0U) {
        const float yaw_motor_error_deg =
            ChassisWrap180Deg(HOLE_YAW_MOTOR_TARGET_DEG -
                              chassis_cmd_recv.gimbal_yaw_signal_sound_angle);

        return (fabsf(yaw_motor_error_deg) <= HOLE_READY_THRESHOLD_DEG) ? 1U : 0U;
    }

    return (fabsf(chassis_cmd_recv.offset_angle - HOLE_YAW_OFFSET_TARGET_DEG) <=
            HOLE_READY_THRESHOLD_DEG) ? 1U : 0U;
}

static void ChassisApplyGravityFeedforward(OmniVelocityCtrlInput_s *input,
                                           const INS_Instance *imu)
{
    float pitch_rad;
    float roll_rad;
    float gain;
    float mass_x;
    float mass_y;
    float force_x_n;
    float force_y_left_n;

    if (input == NULL ||
        imu == NULL ||
        chassis_gravity_ff_enable == 0U ||
        ChassisHoleGravityFeedforwardReady() == 0U) {
        ChassisClearGravityFeedforwardDebug();
        ChassisClearSpeedCurrentFeedforward();
        return;
    }

    pitch_rad = imu->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
    roll_rad = imu->output.INS_angle[INS_ROLL_ADDRESS_OFFSET];
    gain = (float)chassis_gravity_ff_gain;
    mass_x = omni_ctrl.config.mass[OMNI_AXIS_VX];
    mass_y = omni_ctrl.config.mass[OMNI_AXIS_VY];

    if (ChassisFloatIsFinite(pitch_rad) == 0U ||
        ChassisFloatIsFinite(roll_rad) == 0U ||
        ChassisFloatIsFinite(gain) == 0U ||
        ChassisFloatIsFinite(mass_x) == 0U ||
        ChassisFloatIsFinite(mass_y) == 0U ||
        mass_x <= 0.0f ||
        mass_y <= 0.0f) {
        ChassisClearGravityFeedforwardDebug();
        ChassisClearSpeedCurrentFeedforward();
        return;
    }

    force_x_n = (fabsf(pitch_rad) <= CHASSIS_GRAVITY_FF_DEADZONE_RAD) ?
        0.0f :
        -gain * mass_x * BMI088_GRAVITY_MPS2 * arm_sin_f32(pitch_rad);
    force_y_left_n = (fabsf(roll_rad) <= CHASSIS_GRAVITY_FF_DEADZONE_RAD) ?
        0.0f :
        -gain * mass_y * BMI088_GRAVITY_MPS2 * arm_sin_f32(roll_rad);

    input->external_wrench_ff[OMNI_AXIS_VX] += force_x_n;
    input->external_wrench_ff[OMNI_AXIS_VY] += force_y_left_n;
    ChassisApplySpeedGravityCurrentFeedforward(force_x_n, force_y_left_n);

    chassis_gravity_ff_pitch_rad = pitch_rad;
    chassis_gravity_ff_roll_rad = roll_rad;
    chassis_gravity_ff_force_x_n = force_x_n;
    chassis_gravity_ff_force_y_right_n = -force_y_left_n;
}

static uint8_t ChassisCurrentForceCalibMotorsValid(void)
{
    return (uint8_t)(motor_lf != NULL &&
                     motor_rf != NULL &&
                     motor_rb != NULL &&
                     motor_lb != NULL);
}

static uint8_t ChassisCurrentForceCalibWheelSelectValid(uint8_t wheel_select)
{
    return (uint8_t)(wheel_select == CHASSIS_FORCE_CALIB_ALL_WHEELS ||
                     wheel_select < (uint8_t)OMNI_WHEEL_NUM);
}

static uint8_t ChassisCurrentForceCalibSingleWheelMode(void)
{
    return (uint8_t)(chassis_force_calib_run_wheel_select < (uint8_t)OMNI_WHEEL_NUM);
}

static uint32_t ChassisCurrentForceCalibTotalTrials(void)
{
    return (ChassisCurrentForceCalibSingleWheelMode() != 0U) ?
        CHASSIS_FORCE_CALIB_SINGLE_WHEEL_TRIALS :
        CHASSIS_FORCE_CALIB_TOTAL_TRIALS;
}

static float ChassisCurrentForceCalibLimit(void)
{
    float current_limit = g_chassis_force_calib_cmd.current_limit;

    if (current_limit <= 0.0f) {
        current_limit = CHASSIS_FORCE_CALIB_DEFAULT_CURRENT_LIMIT;
    }
    if (current_limit > CHASSIS_FORCE_CALIB_HARD_CURRENT_LIMIT) {
        current_limit = CHASSIS_FORCE_CALIB_HARD_CURRENT_LIMIT;
    }
    return current_limit;
}

static int8_t ChassisCurrentForceCalibDirectionSign(uint8_t direction_index)
{
    return (direction_index == 0U) ? 1 : -1;
}

static uint32_t ChassisCurrentForceCalibTrialId(void)
{
    uint32_t trial_id = 0U;

    if (ChassisCurrentForceCalibSingleWheelMode() == 0U) {
        trial_id = chassis_force_calib_wheel_index;
    }
    trial_id = trial_id * CHASSIS_FORCE_CALIB_DIRECTION_NUM + chassis_force_calib_direction_index;
    trial_id = trial_id * CHASSIS_FORCE_CALIB_CURRENT_LEVEL_NUM + chassis_force_calib_current_level_index;
    trial_id = trial_id * CHASSIS_FORCE_CALIB_REPEAT_NUM + chassis_force_calib_repeat_index;
    return trial_id;
}

static void ChassisDecodeCurrentForceCalibTrial(uint32_t trial_id)
{
    uint32_t rem = trial_id;
    const uint32_t per_wheel_trials = CHASSIS_FORCE_CALIB_SINGLE_WHEEL_TRIALS;

    if (ChassisCurrentForceCalibSingleWheelMode() != 0U) {
        chassis_force_calib_wheel_index = chassis_force_calib_run_wheel_select;
    } else {
        chassis_force_calib_wheel_index = (uint8_t)(rem / per_wheel_trials);
        rem %= per_wheel_trials;
        if (chassis_force_calib_wheel_index >= (uint8_t)OMNI_WHEEL_NUM) {
            chassis_force_calib_wheel_index = (uint8_t)OMNI_WHEEL_LB;
        }
    }

    chassis_force_calib_direction_index =
        (uint8_t)(rem / (CHASSIS_FORCE_CALIB_CURRENT_LEVEL_NUM * CHASSIS_FORCE_CALIB_REPEAT_NUM));
    rem %= (CHASSIS_FORCE_CALIB_CURRENT_LEVEL_NUM * CHASSIS_FORCE_CALIB_REPEAT_NUM);
    chassis_force_calib_current_level_index =
        (uint8_t)(rem / CHASSIS_FORCE_CALIB_REPEAT_NUM);
    chassis_force_calib_repeat_index =
        (uint8_t)(rem % CHASSIS_FORCE_CALIB_REPEAT_NUM);
}

static void ChassisRefreshCurrentForceCalibTarget(float current_limit)
{
    const float current_level =
        chassis_force_calib_current_levels[chassis_force_calib_current_level_index];
    const float signed_current =
        (float)ChassisCurrentForceCalibDirectionSign(chassis_force_calib_direction_index) *
        current_level;

    chassis_force_calib_trial_id = ChassisCurrentForceCalibTrialId();
    chassis_force_calib_target_current_cmd_fit =
        ChassisClampAbs(signed_current, current_limit);
}

static void ChassisSetCurrentForceCalibState(ChassisForceCalibAutoState_e state,
                                             float now_s)
{
    chassis_force_calib_auto_state = state;
    chassis_force_calib_state_start_time_s = now_s;

    if (state == CHASSIS_FORCE_CALIB_AUTO_PULSE_ACTIVE) {
        chassis_force_calib_trigger_time_s = now_s;
        chassis_force_calib_pulse_elapsed_s = 0.0f;
        chassis_force_calib_pulse_done = 0U;
        chassis_force_calib_hard_timeout = 0U;
    }
}

static void ChassisResetCurrentForceCalibRuntime(ChassisForceCalibAutoState_e state)
{
    const float now_s = DWT_GetTimeline_s();

    chassis_force_calib_auto_state = state;
    chassis_force_calib_state_start_time_s = now_s;
    chassis_force_calib_trigger_time_s = 0.0f;
    chassis_force_calib_pulse_elapsed_s = 0.0f;
    chassis_force_calib_target_current_cmd_fit = 0.0f;
    chassis_force_calib_applied_current_cmd_fit = 0.0f;
    chassis_force_calib_trial_id = 0U;
    chassis_force_calib_run_wheel_select = CHASSIS_FORCE_CALIB_ALL_WHEELS;
    chassis_force_calib_wheel_index = 0U;
    chassis_force_calib_direction_index = 0U;
    chassis_force_calib_current_level_index = 0U;
    chassis_force_calib_repeat_index = 0U;
    chassis_force_calib_pulse_done = 0U;
    chassis_force_calib_hard_timeout = 0U;
    chassis_force_calib_auto_done = 0U;
    chassis_force_calib_auto_abort = 0U;
    chassis_force_calib_abort_reason = CHASSIS_FORCE_CALIB_ABORT_NONE;
}

static void ChassisBuildCurrentForceCalibRefs(uint8_t wheel_index,
                                              float current_cmd_fit,
                                              float *current_cmd_lf,
                                              float *current_cmd_rf,
                                              float *current_cmd_rb,
                                              float *current_cmd_lb)
{
    *current_cmd_lf = 0.0f;
    *current_cmd_rf = 0.0f;
    *current_cmd_rb = 0.0f;
    *current_cmd_lb = 0.0f;

    if (wheel_index == 0xFFU) {
        *current_cmd_lf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LF] * current_cmd_fit;
        *current_cmd_rf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RF] * current_cmd_fit;
        *current_cmd_rb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RB] * current_cmd_fit;
        *current_cmd_lb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LB] * current_cmd_fit;
        return;
    }

    switch (wheel_index) {
        case OMNI_WHEEL_LF:
            *current_cmd_lf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LF] * current_cmd_fit;
            break;
        case OMNI_WHEEL_RF:
            *current_cmd_rf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RF] * current_cmd_fit;
            break;
        case OMNI_WHEEL_RB:
            *current_cmd_rb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RB] * current_cmd_fit;
            break;
        case OMNI_WHEEL_LB:
            *current_cmd_lb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LB] * current_cmd_fit;
            break;
        default:
            break;
    }
}

static void ChassisSetCurrentForceCalibRefs(float current_cmd_lf,
                                            float current_cmd_rf,
                                            float current_cmd_rb,
                                            float current_cmd_lb)
{
    if (motor_lf != NULL) {
        DJIMotorSetRef(motor_lf, current_cmd_lf);
    }
    if (motor_rf != NULL) {
        DJIMotorSetRef(motor_rf, current_cmd_rf);
    }
    if (motor_rb != NULL) {
        DJIMotorSetRef(motor_rb, current_cmd_rb);
    }
    if (motor_lb != NULL) {
        DJIMotorSetRef(motor_lb, current_cmd_lb);
    }
}

static void ChassisRestoreCurrentForceCalibOutput(void)
{
    ChassisSetCurrentForceCalibRefs(0.0f, 0.0f, 0.0f, 0.0f);
    ChassisSetWheelSpeedLoop(motor_lf);
    ChassisSetWheelSpeedLoop(motor_rf);
    ChassisSetWheelSpeedLoop(motor_rb);
    ChassisSetWheelSpeedLoop(motor_lb);
    ChassisZeroWheelRef();
    OmniVelocityCtrl_Reset(&omni_ctrl);
    ChassisReachableInvalidateRuntimeCache();
    chassis_force_calib_active = 0U;
}

static void ChassisEnterCurrentForceCalib(void)
{
    // 标定模式直接发送 DJI 原始电流命令，四轮先统一切到开环并清零。
    if (chassis_lqi_current_active != 0U) {
        ChassisExitLqiCurrentMode();
    }
    ChassisZeroWheelRef();
    ChassisSetCurrentForceCalibRefs(0.0f, 0.0f, 0.0f, 0.0f);
    ChassisSetWheelOpenLoop(motor_lf);
    ChassisSetWheelOpenLoop(motor_rf);
    ChassisSetWheelOpenLoop(motor_rb);
    ChassisSetWheelOpenLoop(motor_lb);
    ChassisResetCurrentForceCalibRuntime(CHASSIS_FORCE_CALIB_AUTO_ARM);
    chassis_force_calib_run_wheel_select = g_chassis_force_calib_cmd.wheel_index;
    ChassisDecodeCurrentForceCalibTrial(0U);
    chassis_force_calib_active = 1U;
}

static void ChassisExitCurrentForceCalib(void)
{
    // 退出标定模式时恢复正常速度环，避免开环状态残留到主底盘控制。
    ChassisRestoreCurrentForceCalibOutput();
    ChassisResetCurrentForceCalibRuntime(CHASSIS_FORCE_CALIB_AUTO_IDLE);
}

static void ChassisAbortCurrentForceCalib(ChassisForceCalibAbortReason_e reason,
                                          float now_s)
{
    ChassisRestoreCurrentForceCalibOutput();
    chassis_force_calib_auto_done = 0U;
    chassis_force_calib_auto_abort = 1U;
    chassis_force_calib_abort_reason = reason;
    chassis_force_calib_applied_current_cmd_fit = 0.0f;
    chassis_force_calib_hard_timeout =
        (reason == CHASSIS_FORCE_CALIB_ABORT_HARD_TIMEOUT) ? 1U : chassis_force_calib_hard_timeout;
    ChassisSetCurrentForceCalibState(CHASSIS_FORCE_CALIB_AUTO_ABORT, now_s);
}

static void ChassisFinishCurrentForceCalib(float now_s)
{
    ChassisRestoreCurrentForceCalibOutput();
    chassis_force_calib_auto_done = 1U;
    chassis_force_calib_auto_abort = 0U;
    chassis_force_calib_abort_reason = CHASSIS_FORCE_CALIB_ABORT_NONE;
    chassis_force_calib_applied_current_cmd_fit = 0.0f;
    ChassisSetCurrentForceCalibState(CHASSIS_FORCE_CALIB_AUTO_FINISHED, now_s);
}

static uint8_t ChassisAdvanceCurrentForceCalibTrial(void)
{
    const uint32_t next_trial_id = chassis_force_calib_trial_id + 1U;
    const uint32_t total_trials = ChassisCurrentForceCalibTotalTrials();

    if (next_trial_id < total_trials) {
        ChassisDecodeCurrentForceCalibTrial(next_trial_id);
        return 1U;
    }

    if (total_trials > 0U) {
        ChassisDecodeCurrentForceCalibTrial(total_trials - 1U);
    }
    return 0U;
}

static uint8_t ChassisCurrentForceCalibAutoHolding(void)
{
    return (uint8_t)(chassis_force_calib_auto_state == CHASSIS_FORCE_CALIB_AUTO_FINISHED ||
                     chassis_force_calib_auto_state == CHASSIS_FORCE_CALIB_AUTO_ABORT);
}

static float ChassisCurrentForceCalibSelectedCurrent(float current_fit_lf,
                                                     float current_fit_rf,
                                                     float current_fit_rb,
                                                     float current_fit_lb)
{
    switch (chassis_force_calib_wheel_index) {
        case OMNI_WHEEL_LF:
            return current_fit_lf;
        case OMNI_WHEEL_RF:
            return current_fit_rf;
        case OMNI_WHEEL_RB:
            return current_fit_rb;
        case OMNI_WHEEL_LB:
            return current_fit_lb;
        default:
            return 0.0f;
    }
}

static void ChassisUpdateCurrentForceCalibRecord(uint8_t valid,
                                                 float current_limit,
                                                 float current_cmd_lf,
                                                 float current_cmd_rf,
                                                 float current_cmd_rb,
                                                 float current_cmd_lb)
{
    volatile ChassisCurrentForceCalibRecord_s *record = &g_chassis_force_calib_record;
    const float current_fit_lf = (motor_lf != NULL) ?
        omni_ctrl.config.wheel_dir[OMNI_WHEEL_LF] * (float)motor_lf->measure.real_current : 0.0f;
    const float current_fit_rf = (motor_rf != NULL) ?
        omni_ctrl.config.wheel_dir[OMNI_WHEEL_RF] * (float)motor_rf->measure.real_current : 0.0f;
    const float current_fit_rb = (motor_rb != NULL) ?
        omni_ctrl.config.wheel_dir[OMNI_WHEEL_RB] * (float)motor_rb->measure.real_current : 0.0f;
    const float current_fit_lb = (motor_lb != NULL) ?
        omni_ctrl.config.wheel_dir[OMNI_WHEEL_LB] * (float)motor_lb->measure.real_current : 0.0f;
    const float speed_fit_lf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LF] * ChassisGetWheelSpeed(motor_lf);
    const float speed_fit_rf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RF] * ChassisGetWheelSpeed(motor_rf);
    const float speed_fit_rb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RB] * ChassisGetWheelSpeed(motor_rb);
    const float speed_fit_lb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LB] * ChassisGetWheelSpeed(motor_lb);
    float speed_max_abs = fabsf(speed_fit_lf);

    if (fabsf(speed_fit_rf) > speed_max_abs) {
        speed_max_abs = fabsf(speed_fit_rf);
    }
    if (fabsf(speed_fit_rb) > speed_max_abs) {
        speed_max_abs = fabsf(speed_fit_rb);
    }
    if (fabsf(speed_fit_lb) > speed_max_abs) {
        speed_max_abs = fabsf(speed_fit_lb);
    }

    record->sample_count++;
    record->trial_id = chassis_force_calib_trial_id;
    record->enable = g_chassis_force_calib_cmd.enable;
    record->valid = valid;
    record->calib_active = chassis_force_calib_active;
    record->auto_running =
        (uint8_t)(g_chassis_force_calib_cmd.enable != 0U &&
                  ChassisCurrentForceCalibAutoHolding() == 0U);
    record->auto_done = chassis_force_calib_auto_done;
    record->auto_abort = chassis_force_calib_auto_abort;
    record->auto_state = (uint8_t)chassis_force_calib_auto_state;
    record->wheel_index = chassis_force_calib_wheel_index;
    record->direction = ChassisCurrentForceCalibDirectionSign(chassis_force_calib_direction_index);
    record->current_level_index = chassis_force_calib_current_level_index;
    record->repeat_index = chassis_force_calib_repeat_index;
    record->pulse_done = chassis_force_calib_pulse_done;
    record->hard_timeout = chassis_force_calib_hard_timeout;
    record->abort_reason = (uint8_t)chassis_force_calib_abort_reason;
    record->reserved[0] = 0U;
    record->reserved[1] = 0U;
    record->sample_time_s = DWT_GetTimeline_s();
    record->state_elapsed_s = record->sample_time_s - chassis_force_calib_state_start_time_s;
    if (record->state_elapsed_s < 0.0f) {
        record->state_elapsed_s = 0.0f;
    }
    record->trigger_time_s = chassis_force_calib_trigger_time_s;
    record->pulse_elapsed_s = chassis_force_calib_pulse_elapsed_s;
    record->current_cmd_fit = chassis_force_calib_target_current_cmd_fit;
    record->applied_current_cmd_fit = chassis_force_calib_applied_current_cmd_fit;
    record->current_limit = current_limit;
    record->current_cmd_motor_lf = current_cmd_lf;
    record->current_cmd_motor_rf = current_cmd_rf;
    record->current_cmd_motor_rb = current_cmd_rb;
    record->current_cmd_motor_lb = current_cmd_lb;
    record->real_current_selected =
        ChassisCurrentForceCalibSelectedCurrent(current_fit_lf,
                                                current_fit_rf,
                                                current_fit_rb,
                                                current_fit_lb);
    record->real_current_fit_lf = current_fit_lf;
    record->real_current_fit_rf = current_fit_rf;
    record->real_current_fit_rb = current_fit_rb;
    record->real_current_fit_lb = current_fit_lb;
    record->wheel_speed_fit_max_abs = speed_max_abs;
    record->wheel_speed_fit_lf = speed_fit_lf;
    record->wheel_speed_fit_rf = speed_fit_rf;
    record->wheel_speed_fit_rb = speed_fit_rb;
    record->wheel_speed_fit_lb = speed_fit_lb;
}

static uint8_t ChassisRunCurrentForceCalib(void)
{
    float current_limit;
    float current_cmd_lf;
    float current_cmd_rf;
    float current_cmd_rb;
    float current_cmd_lb;
    uint8_t valid;
    float now_s;
    float state_elapsed_s;

    ChassisClearGravityFeedforwardDebug();
    if (g_chassis_force_calib_cmd.enable == 0U) {
        if (chassis_force_calib_active != 0U) {
            ChassisExitCurrentForceCalib();
        }
        if (chassis_force_calib_auto_state != CHASSIS_FORCE_CALIB_AUTO_IDLE) {
            ChassisResetCurrentForceCalibRuntime(CHASSIS_FORCE_CALIB_AUTO_IDLE);
        }
        ChassisUpdateCurrentForceCalibRecord(0U,
                                            CHASSIS_FORCE_CALIB_DEFAULT_CURRENT_LIMIT,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f);
        return 0U;
    }

    if (chassis_force_calib_active == 0U) {
        if (ChassisCurrentForceCalibAutoHolding() == 0U) {
            if (ChassisCurrentForceCalibWheelSelectValid(g_chassis_force_calib_cmd.wheel_index) == 0U) {
                now_s = DWT_GetTimeline_s();
                ChassisResetCurrentForceCalibRuntime(CHASSIS_FORCE_CALIB_AUTO_ARM);
                ChassisAbortCurrentForceCalib(CHASSIS_FORCE_CALIB_ABORT_INVALID_WHEEL, now_s);
                ChassisUpdateCurrentForceCalibRecord(0U,
                                                    ChassisCurrentForceCalibLimit(),
                                                    0.0f,
                                                    0.0f,
                                                    0.0f,
                                                    0.0f);
                return 1U;
            }
            ChassisEnterCurrentForceCalib();
        } else {
            ChassisSetCurrentForceCalibRefs(0.0f, 0.0f, 0.0f, 0.0f);
            ChassisUpdateCurrentForceCalibRecord((uint8_t)(ChassisCurrentForceCalibMotorsValid() != 0U),
                                                ChassisCurrentForceCalibLimit(),
                                                0.0f,
                                                0.0f,
                                                0.0f,
                                                0.0f);
            return 1U;
        }
    }

    current_limit = ChassisCurrentForceCalibLimit();
    now_s = DWT_GetTimeline_s();
    valid = ChassisCurrentForceCalibMotorsValid();

    if (valid == 0U) {
        ChassisAbortCurrentForceCalib(CHASSIS_FORCE_CALIB_ABORT_MOTOR_INVALID, now_s);
        ChassisUpdateCurrentForceCalibRecord(0U,
                                            current_limit,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f);
        return 1U;
    }

    ChassisRefreshCurrentForceCalibTarget(current_limit);
    if (fabsf(chassis_force_calib_target_current_cmd_fit) > CHASSIS_FORCE_CALIB_HARD_CURRENT_LIMIT) {
        ChassisAbortCurrentForceCalib(CHASSIS_FORCE_CALIB_ABORT_CURRENT_LIMIT, now_s);
        ChassisUpdateCurrentForceCalibRecord(0U,
                                            current_limit,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f);
        return 1U;
    }

    state_elapsed_s = now_s - chassis_force_calib_state_start_time_s;
    if (state_elapsed_s < 0.0f) {
        state_elapsed_s = 0.0f;
    }
    chassis_force_calib_applied_current_cmd_fit = 0.0f;

    switch (chassis_force_calib_auto_state) {
        case CHASSIS_FORCE_CALIB_AUTO_ARM:
            ChassisSetCurrentForceCalibState(CHASSIS_FORCE_CALIB_AUTO_BASELINE, now_s);
            break;

        case CHASSIS_FORCE_CALIB_AUTO_BASELINE:
            if (state_elapsed_s >= CHASSIS_FORCE_CALIB_BASELINE_TIME_S) {
                ChassisSetCurrentForceCalibState(CHASSIS_FORCE_CALIB_AUTO_PULSE_ACTIVE, now_s);
            }
            break;

        case CHASSIS_FORCE_CALIB_AUTO_PULSE_ACTIVE:
            chassis_force_calib_pulse_elapsed_s = now_s - chassis_force_calib_trigger_time_s;
            if (chassis_force_calib_pulse_elapsed_s < 0.0f) {
                chassis_force_calib_pulse_elapsed_s = 0.0f;
            }
            if (chassis_force_calib_pulse_elapsed_s >= CHASSIS_FORCE_CALIB_HARD_TIMEOUT_S) {
                chassis_force_calib_pulse_done = 1U;
                chassis_force_calib_hard_timeout = 1U;
                ChassisAbortCurrentForceCalib(CHASSIS_FORCE_CALIB_ABORT_HARD_TIMEOUT, now_s);
            } else if (chassis_force_calib_pulse_elapsed_s >= CHASSIS_FORCE_CALIB_PULSE_TIME_S) {
                chassis_force_calib_pulse_done = 1U;
                ChassisSetCurrentForceCalibState(CHASSIS_FORCE_CALIB_AUTO_COAST_WAIT, now_s);
            } else {
                chassis_force_calib_applied_current_cmd_fit =
                    chassis_force_calib_target_current_cmd_fit;
            }
            break;

        case CHASSIS_FORCE_CALIB_AUTO_COAST_WAIT:
            if (state_elapsed_s >= CHASSIS_FORCE_CALIB_COAST_WAIT_TIME_S) {
                ChassisSetCurrentForceCalibState(CHASSIS_FORCE_CALIB_AUTO_NEXT_TRIAL, now_s);
            }
            break;

        case CHASSIS_FORCE_CALIB_AUTO_NEXT_TRIAL:
            if (ChassisAdvanceCurrentForceCalibTrial() != 0U) {
                ChassisRefreshCurrentForceCalibTarget(current_limit);
                chassis_force_calib_pulse_done = 0U;
                chassis_force_calib_hard_timeout = 0U;
                chassis_force_calib_trigger_time_s = 0.0f;
                chassis_force_calib_pulse_elapsed_s = 0.0f;
                ChassisSetCurrentForceCalibState(CHASSIS_FORCE_CALIB_AUTO_BASELINE, now_s);
            } else {
                chassis_force_calib_trial_id = ChassisCurrentForceCalibTotalTrials() - 1U;
                ChassisFinishCurrentForceCalib(now_s);
            }
            break;

        case CHASSIS_FORCE_CALIB_AUTO_FINISHED:
        case CHASSIS_FORCE_CALIB_AUTO_ABORT:
            break;

        default:
            ChassisAbortCurrentForceCalib(CHASSIS_FORCE_CALIB_ABORT_ILLEGAL_STATE, now_s);
            break;
    }

    ChassisBuildCurrentForceCalibRefs(chassis_force_calib_wheel_index,
                                      chassis_force_calib_applied_current_cmd_fit,
                                      &current_cmd_lf,
                                      &current_cmd_rf,
                                      &current_cmd_rb,
                                      &current_cmd_lb);
    ChassisSetCurrentForceCalibRefs(current_cmd_lf,
                                    current_cmd_rf,
                                    current_cmd_rb,
                                    current_cmd_lb);

    ChassisZeroWheelRef();
    ChassisUpdateCurrentForceCalibRecord(valid,
                                        current_limit,
                                        current_cmd_lf,
                                        current_cmd_rf,
                                        current_cmd_rb,
                                        current_cmd_lb);
    return 1U;
}

static uint8_t ChassisOmniHCalibWheelValidMask(void)
{
    uint8_t mask = 0U;

    mask |= (motor_lf != NULL) ? (1U << OMNI_WHEEL_LF) : 0U;
    mask |= (motor_rf != NULL) ? (1U << OMNI_WHEEL_RF) : 0U;
    mask |= (motor_rb != NULL) ? (1U << OMNI_WHEEL_RB) : 0U;
    mask |= (motor_lb != NULL) ? (1U << OMNI_WHEEL_LB) : 0U;
    return mask;
}

static uint8_t ChassisOmniHCalibMotorsValid(void)
{
    return (ChassisOmniHCalibWheelValidMask() == 0x0FU) ? 1U : 0U;
}

static uint32_t ChassisOmniHCalibTicks(uint32_t value, uint32_t fallback)
{
    return (value == 0U) ? fallback : value;
}

static float ChassisOmniHCalibAbsOrDefault(float value, float fallback)
{
    if (ChassisFloatIsFinite(value) == 0U || fabsf(value) <= 1.0e-6f) {
        return fallback;
    }
    return fabsf(value);
}

static uint32_t ChassisOmniHCalibRepeatCount(void)
{
    const uint32_t repeat_count = g_omni_h_calib_cmd.repeat_count;

    return (repeat_count == 0U) ? CHASSIS_H_CALIB_DEFAULT_REPEAT_COUNT : repeat_count;
}

static void ChassisOmniHCalibSetState(OmniHCalibState_e state)
{
    chassis_h_calib_state = state;
    chassis_h_calib_state_tick = 0U;
}

static void ChassisOmniHCalibClearCommand(void)
{
    chassis_h_calib_cmd_vx = 0.0f;
    chassis_h_calib_cmd_vy_left = 0.0f;
    chassis_h_calib_cmd_wz = 0.0f;
    chassis_vx = 0.0f;
    chassis_vy = 0.0f;
    chassis_cmd_recv.wz = 0.0f;
    vt_lf = 0.0f;
    vt_rf = 0.0f;
    vt_rb = 0.0f;
    vt_lb = 0.0f;
}

static void ChassisOmniHCalibResetRuntime(OmniHCalibState_e state)
{
    ChassisOmniHCalibClearCommand();
    chassis_h_calib_segment_index = 0U;
    chassis_h_calib_repeat_index = 0U;
    chassis_h_calib_axis = OMNI_H_CALIB_AXIS_VX;
    chassis_h_calib_sign = 1;
    ChassisOmniHCalibSetState(state);
}

static void ChassisOmniHCalibEnterSpeedLoop(void)
{
    if (chassis_lqi_current_active != 0U) {
        ChassisExitLqiCurrentMode();
    } else {
        ChassisSetAllWheelRefZero();
        ChassisSetWheelSpeedLoop(motor_lf);
        ChassisSetWheelSpeedLoop(motor_rf);
        ChassisSetWheelSpeedLoop(motor_rb);
        ChassisSetWheelSpeedLoop(motor_lb);
        ChassisZeroWheelRef();
        OmniVelocityCtrl_Reset(&omni_ctrl);
        ChassisReachableInvalidateRuntimeCache();
    }
}

static void ChassisOmniHCalibStart(void)
{
    ChassisOmniHCalibEnterSpeedLoop();
    ChassisOmniHCalibResetRuntime(OMNI_H_CALIB_STATE_ZERO);
}

static float ChassisOmniHCalibAxisAmplitude(OmniHCalibAxis_e axis)
{
    switch (axis) {
    case OMNI_H_CALIB_AXIS_VX:
        return ChassisOmniHCalibAbsOrDefault(g_omni_h_calib_cmd.amp_vx,
                                             CHASSIS_H_CALIB_DEFAULT_AMP_VXY);
    case OMNI_H_CALIB_AXIS_VY_LEFT:
        return ChassisOmniHCalibAbsOrDefault(g_omni_h_calib_cmd.amp_vy_left,
                                             CHASSIS_H_CALIB_DEFAULT_AMP_VXY);
    case OMNI_H_CALIB_AXIS_WZ:
    default:
        return ChassisOmniHCalibAbsOrDefault(g_omni_h_calib_cmd.amp_wz,
                                             CHASSIS_H_CALIB_DEFAULT_AMP_WZ);
    }
}

static void ChassisOmniHCalibBuildCommand(float ratio,
                                          float *cmd_vx,
                                          float *cmd_vy_left,
                                          float *cmd_wz)
{
    const float amp = ChassisOmniHCalibAxisAmplitude(chassis_h_calib_axis);
    const float signed_value = amp * (float)chassis_h_calib_sign *
                               ChassisClampFloat(ratio, 0.0f, 1.0f);

    *cmd_vx = 0.0f;
    *cmd_vy_left = 0.0f;
    *cmd_wz = 0.0f;
    switch (chassis_h_calib_axis) {
    case OMNI_H_CALIB_AXIS_VX:
        *cmd_vx = signed_value;
        break;
    case OMNI_H_CALIB_AXIS_VY_LEFT:
        *cmd_vy_left = signed_value;
        break;
    case OMNI_H_CALIB_AXIS_WZ:
    default:
        *cmd_wz = signed_value;
        break;
    }
}

static void ChassisOmniHCalibBuildIdealWheelSpeed(float cmd_vx,
                                                  float cmd_vy_left,
                                                  float cmd_wz,
                                                  float wheel_speed_fit[OMNI_WHEEL_NUM])
{
    const float wheel_radius_m = RADIUS_WHEEL * 0.001f;
    const float yaw_lever_m =
        (WHEEL_BASE + TRACK_WIDTH) * 0.0005f * CHASSIS_H_CALIB_ROLLING_DIR_Q;
    const float wheel_radps_to_degps =
        (wheel_radius_m > 1.0e-6f) ? (1.0f / (DEGREE_2_RAD * wheel_radius_m)) : 0.0f;
    const float vx_term =
        CHASSIS_H_CALIB_ROLLING_DIR_Q * cmd_vx * CHASSIS_H_CALIB_CMD_TO_MPS;
    const float vy_term =
        CHASSIS_H_CALIB_ROLLING_DIR_Q * cmd_vy_left * CHASSIS_H_CALIB_CMD_TO_MPS;
    const float wz_term =
        yaw_lever_m * cmd_wz * CHASSIS_H_CALIB_CMD_TO_RADPS;

    /*
     * 理想 X 型 omni 速度分解：
     * vx 前向、vy_left 左向、wz 逆时针为正；输出为 wheel_dir 修正前的轮端 deg/s。
     */
    wheel_speed_fit[OMNI_WHEEL_LF] = (vx_term - vy_term - wz_term) * wheel_radps_to_degps;
    wheel_speed_fit[OMNI_WHEEL_RF] = (vx_term + vy_term + wz_term) * wheel_radps_to_degps;
    wheel_speed_fit[OMNI_WHEEL_RB] = (vx_term - vy_term + wz_term) * wheel_radps_to_degps;
    wheel_speed_fit[OMNI_WHEEL_LB] = (vx_term + vy_term - wz_term) * wheel_radps_to_degps;
}

static uint8_t ChassisOmniHCalibApplyCommand(float cmd_vx,
                                             float cmd_vy_left,
                                             float cmd_wz)
{
    float wheel_speed_fit[OMNI_WHEEL_NUM];
    float max_abs_speed = 0.0f;
    const float wheel_speed_limit =
        ChassisOmniHCalibAbsOrDefault(g_omni_h_calib_cmd.wheel_speed_limit,
                                      CHASSIS_H_CALIB_DEFAULT_WHEEL_SPEED_LIMIT);

    if (omni_ctrl.inited == 0U) {
        OmniVelocityCtrl_Init(&omni_ctrl, NULL);
    }

    ChassisOmniHCalibBuildIdealWheelSpeed(cmd_vx,
                                          cmd_vy_left,
                                          cmd_wz,
                                          wheel_speed_fit);

    for (uint8_t wheel = 0U; wheel < OMNI_WHEEL_NUM; ++wheel) {
        if (fabsf(wheel_speed_fit[wheel]) > max_abs_speed) {
            max_abs_speed = fabsf(wheel_speed_fit[wheel]);
        }
    }

    if (max_abs_speed > wheel_speed_limit) {
        ChassisOmniHCalibResetRuntime(OMNI_H_CALIB_STATE_FAULT);
        return 0U;
    }

    chassis_h_calib_cmd_vx = cmd_vx;
    chassis_h_calib_cmd_vy_left = cmd_vy_left;
    chassis_h_calib_cmd_wz = cmd_wz;
    chassis_vx = cmd_vx;
    chassis_vy = -cmd_vy_left;
    chassis_cmd_recv.wz = cmd_wz;
    vt_lf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LF] * wheel_speed_fit[OMNI_WHEEL_LF];
    vt_rf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RF] * wheel_speed_fit[OMNI_WHEEL_RF];
    vt_rb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RB] * wheel_speed_fit[OMNI_WHEEL_RB];
    vt_lb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LB] * wheel_speed_fit[OMNI_WHEEL_LB];
    return 1U;
}

static void ChassisDirectApplyMecanumSpeed(float vx, float vy_right, float wz)
{
    float wheel_speed_fit[OMNI_WHEEL_NUM];

    ChassisSetWheelSpeedLoop(motor_lf);
    ChassisSetWheelSpeedLoop(motor_rf);
    ChassisSetWheelSpeedLoop(motor_rb);
    ChassisSetWheelSpeedLoop(motor_lb);
    ChassisDirectApplyWheelFeedbackDirection();
    ChassisSetWheelSpeedPidMaxOut((float)chassis_direct_speed_pid_maxout);
    ChassisClearSpeedCurrentFeedforward();
    ChassisClearGravityFeedforwardDebug();
    memset(&omni_output, 0, sizeof(omni_output));

    ChassisOmniHCalibBuildIdealWheelSpeed(vx, -vy_right, wz, wheel_speed_fit);
    vt_lf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LF] * wheel_speed_fit[OMNI_WHEEL_LF];
    vt_rf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RF] * wheel_speed_fit[OMNI_WHEEL_RF];
    vt_rb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RB] * wheel_speed_fit[OMNI_WHEEL_RB];
    vt_lb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LB] * wheel_speed_fit[OMNI_WHEEL_LB];

    omni_output.wheel_speed_ref[OMNI_WHEEL_LF] = vt_lf;
    omni_output.wheel_speed_ref[OMNI_WHEEL_RF] = vt_rf;
    omni_output.wheel_speed_ref[OMNI_WHEEL_RB] = vt_rb;
    omni_output.wheel_speed_ref[OMNI_WHEEL_LB] = vt_lb;

    if (motor_lf != NULL) {
        DJIMotorSetRef(motor_lf, ChassisWheelSpeedToMotorSpeed(vt_lf));
    }
    if (motor_rf != NULL) {
        DJIMotorSetRef(motor_rf, ChassisWheelSpeedToMotorSpeed(vt_rf));
    }
    if (motor_rb != NULL) {
        DJIMotorSetRef(motor_rb, ChassisWheelSpeedToMotorSpeed(vt_rb));
    }
    if (motor_lb != NULL) {
        DJIMotorSetRef(motor_lb, ChassisWheelSpeedToMotorSpeed(vt_lb));
    }
}

static void ChassisOmniHCalibAdvanceSegment(void)
{
    if (chassis_h_calib_sign > 0) {
        chassis_h_calib_sign = -1;
        chassis_h_calib_segment_index++;
        ChassisOmniHCalibSetState(OMNI_H_CALIB_STATE_ZERO);
        return;
    }

    chassis_h_calib_sign = 1;
    chassis_h_calib_axis = (OmniHCalibAxis_e)((uint8_t)chassis_h_calib_axis + 1U);
    if ((uint8_t)chassis_h_calib_axis >= CHASSIS_H_CALIB_AXIS_COUNT) {
        chassis_h_calib_axis = OMNI_H_CALIB_AXIS_VX;
        chassis_h_calib_repeat_index++;
        if (chassis_h_calib_repeat_index >= ChassisOmniHCalibRepeatCount()) {
            ChassisOmniHCalibResetRuntime(OMNI_H_CALIB_STATE_DONE);
            return;
        }
    }

    chassis_h_calib_segment_index++;
    ChassisOmniHCalibSetState(OMNI_H_CALIB_STATE_ZERO);
}

static void ChassisOmniHCalibAdvanceState(void)
{
    uint32_t duration_ticks = 1U;

    switch (chassis_h_calib_state) {
    case OMNI_H_CALIB_STATE_ZERO:
        duration_ticks = ChassisOmniHCalibTicks(g_omni_h_calib_cmd.zero_ticks,
                                                CHASSIS_H_CALIB_DEFAULT_ZERO_TICKS);
        break;
    case OMNI_H_CALIB_STATE_RAMP_UP:
    case OMNI_H_CALIB_STATE_RAMP_DOWN:
        duration_ticks = ChassisOmniHCalibTicks(g_omni_h_calib_cmd.ramp_ticks,
                                                CHASSIS_H_CALIB_DEFAULT_RAMP_TICKS);
        break;
    case OMNI_H_CALIB_STATE_HOLD:
        duration_ticks = ChassisOmniHCalibTicks(g_omni_h_calib_cmd.hold_ticks,
                                                CHASSIS_H_CALIB_DEFAULT_HOLD_TICKS);
        break;
    default:
        return;
    }

    chassis_h_calib_state_tick++;
    if (chassis_h_calib_state_tick < duration_ticks) {
        return;
    }

    switch (chassis_h_calib_state) {
    case OMNI_H_CALIB_STATE_ZERO:
        ChassisOmniHCalibSetState(OMNI_H_CALIB_STATE_RAMP_UP);
        break;
    case OMNI_H_CALIB_STATE_RAMP_UP:
        ChassisOmniHCalibSetState(OMNI_H_CALIB_STATE_HOLD);
        break;
    case OMNI_H_CALIB_STATE_HOLD:
        ChassisOmniHCalibSetState(OMNI_H_CALIB_STATE_RAMP_DOWN);
        break;
    case OMNI_H_CALIB_STATE_RAMP_DOWN:
        ChassisOmniHCalibAdvanceSegment();
        break;
    default:
        break;
    }
}

static float ChassisOmniHCalibStateRatio(void)
{
    const uint32_t ramp_ticks = ChassisOmniHCalibTicks(g_omni_h_calib_cmd.ramp_ticks,
                                                       CHASSIS_H_CALIB_DEFAULT_RAMP_TICKS);

    switch (chassis_h_calib_state) {
    case OMNI_H_CALIB_STATE_RAMP_UP:
        return (float)(chassis_h_calib_state_tick + 1U) / (float)ramp_ticks;
    case OMNI_H_CALIB_STATE_HOLD:
        return 1.0f;
    case OMNI_H_CALIB_STATE_RAMP_DOWN:
        return 1.0f - (float)(chassis_h_calib_state_tick + 1U) / (float)ramp_ticks;
    case OMNI_H_CALIB_STATE_ZERO:
    default:
        return 0.0f;
    }
}

static uint8_t ChassisRunOmniHCalibAuto(float dt)
{
    float cmd_vx;
    float cmd_vy_left;
    float cmd_wz;
    const uint8_t enable = (g_omni_h_calib_cmd.enable != 0U) ? 1U : 0U;
    const uint8_t start = (g_omni_h_calib_cmd.start != 0U) ? 1U : 0U;

    chassis_h_calib_record_dt_s = dt;
    if (enable == 0U) {
        if (chassis_h_calib_state != OMNI_H_CALIB_STATE_IDLE) {
            ChassisOmniHCalibResetRuntime(OMNI_H_CALIB_STATE_IDLE);
        }
        chassis_h_calib_start_last = start;
        return 0U;
    }

    if (g_omni_h_calib_cmd.abort != 0U) {
        ChassisOmniHCalibResetRuntime(OMNI_H_CALIB_STATE_ABORT);
        ChassisOmniHCalibEnterSpeedLoop();
        ChassisApplyWheelOutput();
        chassis_h_calib_start_last = start;
        return 1U;
    }

    if (start != 0U && chassis_h_calib_start_last == 0U &&
        (chassis_h_calib_state == OMNI_H_CALIB_STATE_IDLE ||
         chassis_h_calib_state == OMNI_H_CALIB_STATE_DONE ||
         chassis_h_calib_state == OMNI_H_CALIB_STATE_ABORT ||
         chassis_h_calib_state == OMNI_H_CALIB_STATE_FAULT)) {
        ChassisOmniHCalibStart();
    }
    chassis_h_calib_start_last = start;

    if (chassis_h_calib_state == OMNI_H_CALIB_STATE_IDLE) {
        return 0U;
    }

    if (chassis_h_calib_state == OMNI_H_CALIB_STATE_DONE ||
        chassis_h_calib_state == OMNI_H_CALIB_STATE_ABORT ||
        chassis_h_calib_state == OMNI_H_CALIB_STATE_FAULT) {
        ChassisOmniHCalibClearCommand();
        ChassisOmniHCalibEnterSpeedLoop();
        ChassisApplyWheelOutput();
        return 1U;
    }

    if (ChassisOmniHCalibMotorsValid() == 0U) {
        ChassisOmniHCalibResetRuntime(OMNI_H_CALIB_STATE_FAULT);
        ChassisApplyWheelOutput();
        return 1U;
    }

    ChassisOmniHCalibBuildCommand(ChassisOmniHCalibStateRatio(),
                                  &cmd_vx,
                                  &cmd_vy_left,
                                  &cmd_wz);
    if (ChassisOmniHCalibApplyCommand(cmd_vx, cmd_vy_left, cmd_wz) == 0U) {
        ChassisApplyWheelOutput();
        return 1U;
    }

    ChassisApplyWheelOutput();
    ChassisOmniHCalibAdvanceState();
    return 1U;
}

static void ChassisUpdateOmniHCalibRecord(void)
{
    volatile OmniHCalibRecord_s *record = &g_omni_h_calib_record;
    const uint8_t wheel_valid_mask = ChassisOmniHCalibWheelValidMask();

    record->sample_count++;
    record->run_id = g_omni_h_calib_cmd.run_id;
    record->segment_index = chassis_h_calib_segment_index;
    record->state_tick = chassis_h_calib_state_tick;
    record->enable = (g_omni_h_calib_cmd.enable != 0U &&
                      chassis_h_calib_state != OMNI_H_CALIB_STATE_IDLE) ? 1U : 0U;
    record->state = (uint8_t)chassis_h_calib_state;
    record->axis = (uint8_t)chassis_h_calib_axis;
    record->sign = chassis_h_calib_sign;
    record->valid = (wheel_valid_mask == 0x0FU) ? 1U : 0U;
    record->wheel_valid_mask = wheel_valid_mask;
    record->repeat_index = (uint8_t)chassis_h_calib_repeat_index;
    record->reserved = 0U;
    record->sample_time_s = DWT_GetTimeline_s();
    record->dt_s = chassis_h_calib_record_dt_s;
    record->cmd_vx = chassis_h_calib_cmd_vx;
    record->cmd_vy_left = chassis_h_calib_cmd_vy_left;
    record->cmd_wz = chassis_h_calib_cmd_wz;
    record->wheel_speed_fit_lf =
        (motor_lf != NULL) ? omni_ctrl.config.wheel_dir[OMNI_WHEEL_LF] * ChassisGetWheelSpeed(motor_lf) : 0.0f;
    record->wheel_speed_fit_rf =
        (motor_rf != NULL) ? omni_ctrl.config.wheel_dir[OMNI_WHEEL_RF] * ChassisGetWheelSpeed(motor_rf) : 0.0f;
    record->wheel_speed_fit_rb =
        (motor_rb != NULL) ? omni_ctrl.config.wheel_dir[OMNI_WHEEL_RB] * ChassisGetWheelSpeed(motor_rb) : 0.0f;
    record->wheel_speed_fit_lb =
        (motor_lb != NULL) ? omni_ctrl.config.wheel_dir[OMNI_WHEEL_LB] * ChassisGetWheelSpeed(motor_lb) : 0.0f;
}

static void ChassisFillPowerFeedbackInput(OmniVelocityCtrlInput_s *input)
{
    if (input == NULL) {
        return;
    }

    input->measured_power_w = 0.0f;
    input->measured_power_valid = 0U;
    if (ChassisSuperCapIsReady() != 0U) {
        const float measured_power_w = ChassisGetSuperCapChassisRealPower();

        if (isfinite(measured_power_w) != 0 && measured_power_w > 0.0f) {
            input->measured_power_w = measured_power_w;
            input->measured_power_valid = 1U;
        }
    }
}

static void ChassisRunOmniVelocityControlDirect(float vx_ref,
                                                float vy_left_ref,
                                                float wz_ref,
                                                float dt)
{
    OmniVelocityCtrlInput_s input;
    INS_Instance *lqi_imu = ChassisSelectLqiImu();
    const uint8_t vxy_active =
        (fabsf(vx_ref) > 1.0f || fabsf(vy_left_ref) > 1.0f) ? 1U : 0U;
    const uint8_t search_only =
        (chassis_rotate_power_priority_active == 0U &&
         chassis_lqi_current_active == 0U &&
         vxy_active != 0U &&
         chassis_cmd_recv.power_limit > 0U) ? 1U : 0U;
    const uint8_t rotate_search =
        (chassis_rotate_power_priority_active != 0U &&
         chassis_rotate_speed_allocation_enable != 0U &&
         chassis_lqi_current_active == 0U &&
         chassis_cmd_recv.power_limit > 0U) ? 1U : 0U;

    memset(&input, 0, sizeof(input));
    ChassisApplyLqiRuntimeConfig();
    /*
     * 这里是 chassis.c 到 omni_velocity_ctrl.c 的边界：
     * vx_ref 使用底盘前向；vy_left_ref 使用左向坐标；wz_ref 使用底盘 yaw 指令。
     * 轮速反馈先由电机轴 speed_aps 换算成轮端速度，wz 闭环使用规范化后的 REP-103 IMU yaw gyro。
     */
    input.vx_ref = vx_ref;
    input.vy_ref = vy_left_ref;
    input.wz_ref = wz_ref;
    input.dt = dt;
    input.current_limit = g_chassis_lqi_current_cmd.current_limit;
    input.current_rate_limit = g_chassis_lqi_current_cmd.current_rate_limit;
    /*
     * 功率硬限幅下放到 DJI 输出层的 CurrentOutputCalc()。
     * omni 这里只拿预算搜索最大可达速度；普通模式只缓存，不改写速度参考。
     * rotate 的 vxy/wz 速度分配默认关闭，Ozone 手动置
     * chassis_rotate_speed_allocation_enable=1 后才重新启用。
     */
    input.temp_power_limit_w = (rotate_search != 0U || search_only != 0U) ?
        (float)chassis_cmd_recv.power_limit : 0.0f;
    input.rotate_wz_search_limit = (rotate_search != 0U) ?
        ChassisAbsFiniteFloat((float)chassis_rotate_wz_search_cmd, 12000.0f) : 0.0f;
    input.rotate_power_priority_enable = rotate_search;
    input.power_search_only_enable = search_only;
    ChassisFillSpeedPowerModelInput(&input);
    ChassisFillPowerFeedbackInput(&input);
    input.current_offset_enable = g_chassis_lqi_current_cmd.current_offset_enable;
    ChassisApplyGravityFeedforward(&input, lqi_imu);
    if (lqi_imu != NULL) {
        input.body_omega_z = lqi_imu->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];
        input.body_omega_z_valid = 1U;
    }

    if (motor_lf != NULL) {
        input.wheel_speed[OMNI_WHEEL_LF] = ChassisGetWheelSpeed(motor_lf);
        input.wheel_current[OMNI_WHEEL_LF] = (float)motor_lf->measure.real_current;
    }
    if (motor_rf != NULL) {
        input.wheel_speed[OMNI_WHEEL_RF] = ChassisGetWheelSpeed(motor_rf);
        input.wheel_current[OMNI_WHEEL_RF] = (float)motor_rf->measure.real_current;
    }
    if (motor_rb != NULL) {
        input.wheel_speed[OMNI_WHEEL_RB] = ChassisGetWheelSpeed(motor_rb);
        input.wheel_current[OMNI_WHEEL_RB] = (float)motor_rb->measure.real_current;
    }
    if (motor_lb != NULL) {
        input.wheel_speed[OMNI_WHEEL_LB] = ChassisGetWheelSpeed(motor_lb);
        input.wheel_current[OMNI_WHEEL_LB] = (float)motor_lb->measure.real_current;
    }

    OmniVelocityCtrl_Step(&omni_ctrl, &input, &omni_output);

    vt_lf = omni_output.wheel_speed_ref[OMNI_WHEEL_LF];
    vt_rf = omni_output.wheel_speed_ref[OMNI_WHEEL_RF];
    vt_rb = omni_output.wheel_speed_ref[OMNI_WHEEL_RB];
    vt_lb = omni_output.wheel_speed_ref[OMNI_WHEEL_LB];

    chassis_feedback_data.real_vx = omni_output.nu_hat[OMNI_AXIS_VX];
    chassis_feedback_data.real_vy = -omni_output.nu_hat[OMNI_AXIS_VY];
    chassis_feedback_data.real_wz = omni_output.nu_hat[OMNI_AXIS_WZ];
}

static void ChassisRunOmniVelocityControl(float dt)
{
    // chassis.c 的 y 正向为右，omni_velocity_ctrl 内部 y 正向为左，因此这里对 vy 取反。
    ChassisRunOmniVelocityControlDirect(chassis_vx,
                                        -chassis_vy,
                                        chassis_cmd_recv.wz,
                                        dt);
}

uint64_t can_send_count=0;
float offset_angle_watch;
float text_vx=0;
static float freequence = 0,freequence_last=0; 
extern Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;
 float follow_kp,follow_kd;
float super_speed=10000;
float textkd=6,text_speed=10000;
static float getRotato_K(uint8_t level){
    switch(level){
        case 1:
            return textkd;
            break;
        case 2:
            return 75;
            break;
            case 3:
            return 80;
            break;
            case 4:
            return 85;
            break;
            case 5:
            return 90;
            break;
            case 6:
            return 95;
            break;
            case 7:
            return 100;
            break;
            case 8:
            return 105;
            break;
            case 9:
            return 110;
            break;
            case 10:
            return 120;
            break;
        default:
            return 100;
}
}
extern uint16_t powerLim;

static float getWzspeed(uint16_t powerLimit){
    switch (powerLimit){
        case 70://------------------
            return 3500;
        break;
        case 75:
        return 3750;
        break;
        case 80://------------------
                return 4000;
        break;
        case 85:
        return 4175;
        break;
        case 90:
        return 4350;
        break;
        case 95:
        return 4525;
        break;
        case 100://------------------
        return 4700;
        break;
        case 105:
        return 4900;
        break;
        case 110:
        return 5100;
        break;
        case 120://------------------
        return 5300;
        break;
        default:
        return 3000;
        break;
    }
}

float offset_angle_watch;
/*
 * 机器人底盘控制核心任务，1 ms 调度一次：
 * robot_cmd 发布 chassis_cmd/gimbal_feed -> 本任务处理模式、坐标变换和优先级 ->
 * omni_velocity_ctrl 完成 LQI/ESO/轮力分配/限流 -> ChassisApplyWheelOutput 写 DJI 参考 ->
 * ChassisPublishFeedback 回传速度估计、超电电压和在线状态。
 */
void ChassisTask()
{
    // 控制周期由 DWT 实测，传给 omni 控制器用于积分、ESO 和斜率限制。
    const float ctrl_dt = DWT_GetDeltaT(&omni_ctrl_dwt_cnt);
    chassis_h_calib_record_dt_s = ctrl_dt;

    /*
     * 输入边界：
     * chassis_cmd 来自 robot_cmd，是本周期底盘唯一的命令入口；
     * gimbal_feed 只作为反馈缓存，主要给底盘侧读取云台/IMU相关状态。
     */
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    //调自瞄的时候底盘不动
    // chassis_cmd_recv.chassis_mode=0;
    if (gimbal_feed_sub != NULL) {
        (void)SubGetMessage(gimbal_feed_sub, &chassis_gimbal_feedback);
    }
    ChassisUpdateSuperCapControl();
    PowerControlupdate(chassis_cmd_recv.power_limit, CHASSIS_WHEEL_REDUCTION_RATIO);
    if (ChassisSuperCapIsReady() != 0U) {
        const float measured_power_w = ChassisGetSuperCapChassisRealPower();
        const uint8_t measured_power_valid =
            (isfinite(measured_power_w) != 0 && measured_power_w > 0.0f) ? 1U : 0U;
        PowerControlFeedbackUpdate(measured_power_w, measured_power_valid);
    } else {
        PowerControlFeedbackUpdate(0.0f, 0U);
    }
    ChassisReachableHandlePowerLimitRuntime(chassis_cmd_recv.power_limit);

    // 裁判系统底盘电源位去抖逻辑暂时保留为注释，避免单帧掉电标志导致整车瞬间无力。
    //static uint16_t mains_power_off_cnt = 0;
    // if (referee_data_for_ui != NULL && referee_data_for_ui->GameRobotState.mains_power_chassis_output == 0) {
    //     if (mains_power_off_cnt < 1000)
    //         mains_power_off_cnt++;
    // } else {
    //     mains_power_off_cnt = 0;
    // }
    // if (mains_power_off_cnt > 30)
    //     chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
    /*
     * 电流-轮力标定优先级高于正常速度控制。
     * 若标定函数已经完成本周期 raw current 下发，就只更新调试记录并提前返回。
     */
    if (g_chassis_force_calib_cmd.enable != 0U) {
        ChassisReachableInvalidateRuntimeCache();
        ChassisEnableWheelMotors();
        if (ChassisRunCurrentForceCalib() != 0U) {
            ChassisInvalidateOmniForceDebug(ctrl_dt);
            ChassisUpdateOmniHCalibRecord();
            ChassisPublishFeedback();
            return;
        }
    }

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE){ 
        ChassisFollowPIDClearRuntime();
        /*
         * 零力模式优先级最高：
         * 清空轮速参考、退出可能残留的标定状态、复位 omni 控制器，最后停掉四个底盘电机。
         */
        ChassisZeroWheelRef();
        ChassisOmniHCalibResetRuntime(OMNI_H_CALIB_STATE_IDLE);
        ChassisClearGravityFeedforwardDebug();
        if (chassis_force_calib_active != 0U) {
            ChassisExitCurrentForceCalib();
        }
        OmniVelocityCtrl_Reset(&omni_ctrl);
        ChassisReachableInvalidateRuntimeCache();
        memset(&omni_output, 0, sizeof(omni_output));
        chassis_lqi_current_active = 0U;
        ChassisStopWheelMotors();
        ChassisUpdateCurrentForceCalibRecord(0U,
                                            CHASSIS_FORCE_CALIB_DEFAULT_CURRENT_LIMIT,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f);
        ChassisInvalidateOmniForceDebug(ctrl_dt);
        ChassisUpdateOmniHCalibRecord();
        ChassisPublishFeedback();
        return;
    } else { 
        /*
         * 非零力模式先确保电机使能。
         * LQI 电流接管是否生效由 g_chassis_lqi_current_cmd.enable 决定；
         * 不生效时后面仍会把 omni 输出转换成普通 DJI 速度参考。
        */
        ChassisEnableWheelMotors();
        if (chassis_force_calib_active != 0U && g_chassis_force_calib_cmd.enable == 0U) {
            // 上一拍还处于标定时，这里给标定状态机一次收尾机会。
            (void)ChassisRunCurrentForceCalib();
        }
    }

    if (ChassisRunCurrentForceCalib() != 0U) {
        /*
         * 标定状态机可能在非零力分支后才进入 active。
         * 一旦它接管输出，本周期不再运行 yaw/follow/omni 正常控制，避免两套输出互相覆盖。
         */
        ChassisReachableInvalidateRuntimeCache();
        ChassisInvalidateOmniForceDebug(ctrl_dt);
        ChassisUpdateOmniHCalibRecord();
        ChassisPublishFeedback();
        return;
    }

    if (ChassisRunOmniHCalibAuto(ctrl_dt) != 0U) {
        ChassisReachableInvalidateRuntimeCache();
        ChassisInvalidateOmniForceDebug(ctrl_dt);
        ChassisUpdateOmniHCalibRecord();
        ChassisPublishFeedback();
        return;
    }

    ChassisUpdateLqiCurrentMode();

    static float vw_set;

    // vx/vy 统一按 yaw/云台帧解释；模式段只负责 wz / powerLim。
    // 平移坐标系在底盘解算段统一旋到底盘系。
    chassis_rotate_power_priority_active = 0U;
    if (chassis_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) {
        const uint8_t hole_yaw_total_stage =
            (chassis_cmd_recv.hole_offset_yaw_takeover != 0U) ? 1U : 0U;
        const uint8_t hole_yaw_stage_changed =
            ChassisFollowPIDResetOnHoleYawTotalStage(hole_yaw_total_stage);
        if (hole_yaw_total_stage != 0U) {
            const float yaw_meas_deg = chassis_cmd_recv.gimbal_yaw_total_angle;
            const float yaw_ref_deg = chassis_cmd_recv.yaw_control;
            if (hole_yaw_stage_changed != 0U) {
                chassis_wz_cmd_prev = 0.0f;
                chassis_wz_body_prev = 0.0f;
            }
            chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID,
                                               yaw_meas_deg,
                                               yaw_ref_deg);
        } else {
            chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID,
                                               chassis_cmd_recv.offset_angle,
                                               HOLE_YAW_OFFSET_TARGET_DEG);
        }
        powerLim = 0;
    } else {
        (void)ChassisFollowPIDResetOnHoleYawTotalStage(0U);
        switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW:
            // 不跟随模式只保留平移，底盘 yaw 不主动闭环。
            chassis_cmd_recv.wz = 0.0f;
            powerLim = 0;
            break;

        case CHASSIS_FOLLOW_GIMBAL_YAW:
            // 跟随模式把 offset_angle 闭到 0，使底盘正方向重新对齐云台 yaw。
            powerLim = 0;
            chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, chassis_cmd_recv.offset_angle, 0);
            break;

        case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
            /*
             * 自旋模式下 vx/vy 仍参与全向移动。
             * 无平移时直接给最大自旋命令；有平移时按功率限制插值得到可用 wz，
             * 剩余功率交给 vx/vy 和 DJI 输出层 CurrentOutputCalc()。
             */
            powerLim = chassis_cmd_recv.power_limit;
            chassis_rotate_power_priority_active = 1U;
            vw_set = ChassisAbsFiniteFloat((float)chassis_rotate_wz_search_cmd, 12000.0f);
            chassis_vw = vw_set;
            if (ChassisRotateVxyActive(chassis_cmd_recv.vx, chassis_cmd_recv.vy) != 0U) {
                const float rotate_gyro_rad_s =
                    ChassisInterpolateRotateWzGyro(chassis_cmd_recv.power_limit);

                chassis_vw = ChassisRotateGyroToWzCmd(rotate_gyro_rad_s);
            }
            chassis_cmd_recv.wz = chassis_vw;
            break;

        case CHASSIS_REVERSE_ROTATE:
            // 反向自旋保留为独立模式，直接给负向 wz。
            chassis_cmd_recv.wz = -5000;
            break;

        case CHASSIS_DIRECT:
            // Direct chassis control keeps the upstream body-frame wz command unchanged.
            break;

        default:
            // 未识别模式不改写 wz，沿用 robot_cmd 下发值，便于调试新模式时观察原始命令。
            break;
        }
    }

/*-------------------------底盘解算------------------------------------------------------*/
    /*
     * 加速度限幅：在命令(云台)系对 vx/vy/wz 做斜率限制，避免操作员/视觉给出阶跃量造成的过大加速度。
     * vx/vy 放在旋转到底盘系之前——自旋时 offset_angle 快速变化会让底盘系 vx/vy 正弦摆动，
     * 在那一侧限会"咬住"摆动；在命令系限只约束操作员的指令变化率。
     * wz 覆盖 REVERSE_ROTATE 等无软启动模式产生的阶跃，所有模式分支汇合后统一过这一关，
	 * 比 omni 内部的 wz_acc_limit 更早一层，且暴露为文件内可调旋钮。
	 */
    const float cmd_vx_pre_slew = chassis_cmd_recv.vx;
    const float cmd_vy_pre_slew = chassis_cmd_recv.vy;
    const float cmd_wz_pre_slew = chassis_cmd_recv.wz;
    const float vxy_max_step = vxy_max_accel * ctrl_dt;
    chassis_cmd_recv.vx = SlewRateLimit(chassis_cmd_recv.vx, chassis_vx_cmd_prev, vxy_max_step);
    chassis_cmd_recv.vy = SlewRateLimit(chassis_cmd_recv.vy, chassis_vy_cmd_prev, vxy_max_step);
    chassis_vx_cmd_prev = chassis_cmd_recv.vx;
    chassis_vy_cmd_prev = chassis_cmd_recv.vy;

    const float wz_max_step = wz_max_accel * ctrl_dt;
    chassis_cmd_recv.wz = SlewRateLimit(chassis_cmd_recv.wz, chassis_wz_cmd_prev, wz_max_step);
    chassis_wz_cmd_prev = chassis_cmd_recv.wz;

    /*
     * 普通模式下根据 offset_angle 把遥控/上层速度从云台坐标系旋到底盘坐标系。
     * 狗洞模式下平移命令直接按底盘坐标系解释，并只交换 vx/vy。
     */
    if (chassis_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) {
        chassis_vx = chassis_cmd_recv.vy;
        chassis_vy = -chassis_cmd_recv.vx;
    } else {
        const float theta_rad = chassis_cmd_recv.offset_angle * DEGREE_2_RAD;
        const float cos_theta = arm_cos_f32(theta_rad);
        const float sin_theta = arm_sin_f32(theta_rad);
        chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
        chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    }
    ChassisApplyBodyFrameSlewLimit(ctrl_dt);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_DIRECT) {
        /*
         * DR16 bring-up path: bypass LQI/ESO/omni closed-loop control and
         * write four DJI speed references directly after mecanum solving.
         */
        if (chassis_lqi_current_active != 0U) {
            ChassisExitLqiCurrentMode();
        }
        if (ChassisDirectInputIdle(cmd_vx_pre_slew,
                                   cmd_vy_pre_slew,
                                   cmd_wz_pre_slew) != 0U) {
            ChassisDirectIdleStop();
            ChassisInvalidateOmniForceDebug(ctrl_dt);
            ChassisUpdateOmniHCalibRecord();
            ChassisPublishFeedback();
            return;
        }
        ChassisDirectApplyMecanumSpeed(chassis_vx, chassis_vy, chassis_cmd_recv.wz);
        ChassisInvalidateOmniForceDebug(ctrl_dt);
        ChassisUpdateOmniHCalibRecord();
        ChassisPublishFeedback();
        return;
    }

    /*
     * 进入 omni/LQI/ESO/轮力分配链路。
     * 输出同时包含 vt_* 轮端速度参考和 raw current 参考，最终采用哪套由 chassis_lqi_current_active 决定。
     */
    ChassisRunOmniVelocityControl(ctrl_dt);

    // 按当前输出模式把 raw current 或 speed ref 写入 DJI 电机参考。
    ChassisApplyWheelOutput();

    ChassisUpdateOmniForceDebug(ctrl_dt,
                                cmd_vx_pre_slew,
                                cmd_vy_pre_slew,
                                cmd_wz_pre_slew);

    // Ozone/CSV 调试记录在输出后更新，保证记录的是本周期实际发送前后的控制状态。
    ChassisUpdateOmniHCalibRecord();

    // 发布给 robot_cmd/UI：速度估计、超电电压、超电在线状态和底盘功率输出。
    ChassisPublishFeedback();
}
