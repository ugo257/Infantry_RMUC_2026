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
#include "power_calc.h"
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
#define CHASSIS_FORCE_CALIB_DEFAULT_CURRENT_LIMIT 3000.0f
#define CHASSIS_FORCE_CALIB_HARD_CURRENT_LIMIT 6000.0f
#define CHASSIS_FORCE_CALIB_PULSE_TIME_S 0.600f
#define CHASSIS_FORCE_CALIB_HARD_TIMEOUT_S 1.250f
#define CHASSIS_FORCE_CALIB_TRIGGER_EPS 1.0f
#define CHASSIS_VXY_PRIORITY_SW_SEARCH_STEPS 32U
#define CHASSIS_VXY_PRIORITY_EPS 1.0e-4f
#define CHASSIS_VXY_PRIORITY_AXIS_EPS 1.0e-6f

typedef enum
{
    CHASSIS_FORCE_CALIB_PULSE_IDLE = 0,
    CHASSIS_FORCE_CALIB_PULSE_WAIT_TRIGGER = 1,
    CHASSIS_FORCE_CALIB_PULSE_ACTIVE = 2,
    CHASSIS_FORCE_CALIB_PULSE_DONE = 3,
    CHASSIS_FORCE_CALIB_PULSE_HARD_TIMEOUT = 4,
} ChassisForceCalibPulseState_e;

typedef struct
{
    float sv;
    float sw;
    uint8_t feasible;
} ChassisVxyPriorityScale_s;

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
//   - HOLE  : setpoint = HOLE_YAW_OFFSET_TARGET_DEG (-67°)
// Ki=0 + Derivative_On_Measurement 保证 setpoint 跳变时无积分残留 / 无微分踢。
// DeadBand 2° 比 HOLE_READY_THRESHOLD_DEG(3°) 小,保证狗洞 ready 能触发。
static PIDInstance Chassis_Follow_PID = {
    .Kp            = 60.0f,
    .Ki            = 0,
    .Kd            = 1,
    .DeadBand      = 1.0,
    .IntegralLimit = 0,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 100000,
};

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy, chassis_vw; // 将云台系的速度投影到底盘
static float chassis_vx_cmd_prev = 0.0f, chassis_vy_cmd_prev = 0.0f, chassis_wz_cmd_prev = 0.0f; // 命令系下 vx/vy/wz 的上一拍限幅输出，给斜率限制留状态
static float vt_lf, vt_rf, vt_lb, vt_rb;         // omni 控制器输出的轮端速度参考，发送给 DJI 前需要乘减速比
static float kl=1;
extern INS_Instance *INS;
int8_t sign_lf, sign_rf, sign_lb, sign_rb;

static float friction_lf_state = 0.0f;
static float friction_rf_state = 0.0f;
static float friction_lb_state = 0.0f;
static float friction_rb_state = 0.0f;
static float Power_Output;

static OmniVelocityCtrl_s omni_ctrl;
static OmniVelocityCtrlOutput_s omni_output;
static uint32_t omni_ctrl_dwt_cnt;
static uint8_t chassis_lqi_current_active = 0U;
volatile OmniHCalibCmd_s g_omni_h_calib_cmd = {
    .enable = 0U,
    .run_id = 0U,
};
volatile OmniHCalibRecord_s g_omni_h_calib_record;
volatile ChassisLqiCurrentCmd_s g_chassis_lqi_current_cmd = {
    .enable = 1U,
    .current_offset_enable = 1U,
    .eso_compensate_enable = 1U,
    .current_limit = 16384.0f,
    .current_rate_limit = 2000000.0f,
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
volatile uint8_t chassis_vxy_priority_enable = 1U;
volatile float chassis_vxy_priority_wheel_speed_limit = 3000.0f;
volatile float chassis_vxy_priority_wz_min_scale = 0.7f;
volatile float chassis_vxy_priority_wz_deadband = 1.0f;
static uint8_t chassis_force_calib_active = 0U;
static ChassisForceCalibPulseState_e chassis_force_calib_pulse_state = CHASSIS_FORCE_CALIB_PULSE_IDLE;
static float chassis_force_calib_trigger_time_s = 0.0f;
static float chassis_force_calib_pulse_elapsed_s = 0.0f;
static float chassis_force_calib_latched_current_cmd_fit = 0.0f;
static float chassis_force_calib_last_current_cmd_fit = 0.0f;
static uint8_t chassis_force_calib_latched_wheel_index = 0xFFU;
static uint8_t chassis_force_calib_pulse_done = 0U;
static uint8_t chassis_force_calib_hard_timeout = 0U;

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
            .feedforward_flag      = FEEDFORWARD_NONE,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s chassis_motor_config_2 = {
        .can_init_config.can_handle   = &hcan2,
        .controller_param_init_config = {
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
            .feedforward_flag      = FEEDFORWARD_NONE,
        },
        .motor_type = M3508,
    };
     Motor_Init_Config_s chassis_motor_config_3 = {
        .can_init_config.can_handle   = &hcan2,
        .controller_param_init_config = {
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
            .feedforward_flag      = FEEDFORWARD_NONE,
        },
        .motor_type = M3508,
    };
     Motor_Init_Config_s chassis_motor_config_4 = {
        .can_init_config.can_handle   = &hcan2,
        .controller_param_init_config = {
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
            .feedforward_flag      = FEEDFORWARD_NONE,
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
    
    SuperCap_Init_Config_s supercap_config = {
        .can_config = {
            .can_handle = &hcan1,
        },
    };
    supercap = SuperCapInit(&supercap_config);
    (void)supercap_config;
    // 当前工程默认没有初始化超电对象，只有指针有效时才允许使能。
    if (supercap != NULL) {
        SuperCapEnable(supercap);
    }

    #endif


    // motor_lb->motor_controller.speed_PID.Iout=0;motor_rb->motor_controller.speed_PID.Iout=0;
    // motor_lf->motor_controller.speed_PID.Iout=0;motor_rf->motor_controller.speed_PID.Iout=0;

#if defined(CHASSIS_BOARD)
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
 #endif // CHASSIS_BOARD
    OmniVelocityCtrl_Init(&omni_ctrl, NULL);
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
float friction_compensation_lf = 600.0f;
float friction_compensation_rf = -750.0f;  
float friction_compensation_lb = 110.0f;  
float friction_compensation_rb = -100.0f;  

// float friction_compensation_lf = 0.0f;  
// float friction_compensation_rf = 0.0f;  
// float friction_compensation_lb = 0.0f;  
// float friction_compensation_rb = 0.0f;  

// 死区与滞回参数
#define FRICTION_KICK_THRESHOLD 100.0f    // 低速启动阈值（度/秒）
#define FRICTION_HYSTERESIS 30.0f          // 滞回带宽度（度/秒）
#define FRICTION_SMOOTHING 0.2f            // 补偿平滑系数（0~1）

/**
 * @brief 标准摩擦补偿：定值随指令方向变号，带智能死区
 * @param velocity_cmd 速度指令（度/秒）
 * @param compensation 补偿定值
 * @param last_sign 上次方向状态指针（用于滞回）
 * @return 补偿力（与指令方向相反）
 */
static float calculateFrictionCompensation(float velocity_cmd, float compensation, int8_t *last_sign)
{
    //无指令时：补偿归零，确保绝对静止
    if (fabsf(velocity_cmd) < 1.0f) {
        *last_sign = 0;
        return 0.0f;
    }
    
    int8_t current_sign;
    
    // 滞回判断：防止方向符号抖动
    if (fabsf(velocity_cmd) > FRICTION_KICK_THRESHOLD + FRICTION_HYSTERESIS) {
        current_sign = (velocity_cmd > 0.0f) ? 1 : -1;
        *last_sign = current_sign;
    } else if (fabsf(velocity_cmd) < FRICTION_KICK_THRESHOLD - FRICTION_HYSTERESIS) {
        // 低速启动区：给kick启动力
        if (*last_sign == 0) {
            current_sign = (velocity_cmd > 0.0f) ? 1 : -1;
            *last_sign = current_sign;
        } else {
            current_sign = *last_sign;  // 保持上次方向
        }
    } else {
        // 滞回带内：保持上次状态，防止翻符号
        current_sign = (*last_sign != 0) ? *last_sign : ((velocity_cmd > 0.0f) ? 1 : -1);
        *last_sign = current_sign;
    }
    return -compensation * (float)current_sign;
}

/**
 * @brief 更新四个轮子的摩擦补偿
 * @param velocity_cmd_* 各轮速度指令（度/秒）
 */
static void updateAllWheelsFriction(float velocity_cmd_lf, float velocity_cmd_rf, 
                                     float velocity_cmd_lb, float velocity_cmd_rb)
{
    
    float target_lf = calculateFrictionCompensation(velocity_cmd_lf, friction_compensation_lf, &sign_lf);
    float target_rf = calculateFrictionCompensation(velocity_cmd_rf, friction_compensation_rf, &sign_rf);
    float target_lb = calculateFrictionCompensation(velocity_cmd_lb, friction_compensation_lb, &sign_lb);
    float target_rb = calculateFrictionCompensation(velocity_cmd_rb, friction_compensation_rb, &sign_rb);
    // 平滑过渡到目标值
    friction_lf_state += FRICTION_SMOOTHING * (target_lf - friction_lf_state);
    friction_rf_state += FRICTION_SMOOTHING * (target_rf - friction_rf_state);
    friction_lb_state += FRICTION_SMOOTHING * (target_lb - friction_lb_state);
    friction_rb_state += FRICTION_SMOOTHING * (target_rb - friction_rb_state);
    // 调试输出
    
}
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

static uint8_t ChassisFindMaxVxyScaleForWzScale(const float a[OMNI_WHEEL_NUM],
                                                const float b[OMNI_WHEEL_NUM],
                                                float sw,
                                                float wheel_speed_limit,
                                                float *sv_out)
{
    float sv_min = 0.0f;
    float sv_max = 1.0f;

    for (uint8_t wheel = 0U; wheel < OMNI_WHEEL_NUM; ++wheel) {
        const float aw = a[wheel];
        const float bw = b[wheel] * sw;

        if (fabsf(aw) < CHASSIS_VXY_PRIORITY_AXIS_EPS) {
            if (fabsf(bw) > wheel_speed_limit) {
                return 0U;
            }
            continue;
        }

        float lo = (-wheel_speed_limit - bw) / aw;
        float hi = ( wheel_speed_limit - bw) / aw;
        if (lo > hi) {
            const float tmp = lo;
            lo = hi;
            hi = tmp;
        }

        if (lo > sv_min) {
            sv_min = lo;
        }
        if (hi < sv_max) {
            sv_max = hi;
        }
        if (sv_min > sv_max) {
            return 0U;
        }
    }

    if (sv_max < 0.0f || sv_min > 1.0f) {
        return 0U;
    }

    if (sv_out != NULL) {
        *sv_out = ChassisClampFloat(sv_max, 0.0f, 1.0f);
    }
    return 1U;
}

static uint8_t ChassisPureWzScaleFeasible(const float b[OMNI_WHEEL_NUM],
                                          float sw,
                                          float wheel_speed_limit)
{
    for (uint8_t wheel = 0U; wheel < OMNI_WHEEL_NUM; ++wheel) {
        if (fabsf(b[wheel] * sw) > wheel_speed_limit) {
            return 0U;
        }
    }
    return 1U;
}

static ChassisVxyPriorityScale_s ChassisSearchVxyPriorityScale(const float a[OMNI_WHEEL_NUM],
                                                               const float b[OMNI_WHEEL_NUM],
                                                               float sw_min,
                                                               float sw_max,
                                                               float wheel_speed_limit)
{
    ChassisVxyPriorityScale_s best = {
        .sv = 0.0f,
        .sw = sw_min,
        .feasible = 0U,
    };

    if (sw_max < sw_min) {
        const float tmp = sw_min;
        sw_min = sw_max;
        sw_max = tmp;
    }

    for (uint8_t k = 0U; k <= CHASSIS_VXY_PRIORITY_SW_SEARCH_STEPS; ++k) {
        const float t = (float)k / (float)CHASSIS_VXY_PRIORITY_SW_SEARCH_STEPS;
        const float sw = sw_min + (sw_max - sw_min) * t;
        float sv = 0.0f;

        if (ChassisFindMaxVxyScaleForWzScale(a, b, sw, wheel_speed_limit, &sv) == 0U) {
            continue;
        }

        if (best.feasible == 0U ||
            sv > best.sv + CHASSIS_VXY_PRIORITY_EPS ||
            (fabsf(sv - best.sv) <= CHASSIS_VXY_PRIORITY_EPS && sw > best.sw)) {
            best.sv = sv;
            best.sw = sw;
            best.feasible = 1U;
        }
    }

    return best;
}

/*
 * 速度层资源分配入口，放在 chassis_vx/chassis_vy 旋到底盘系之后、
 * 进入 omni/LQI/ESO/轮力分配之前。这里只改三自由度速度参考，不改
 * omni 控制器内部的速度估计、LQI、ESO、轮力/电流分配逻辑。
 */
static void ChassisApplyVxyPrioritySpeedScale(void)
{
    float a[OMNI_WHEEL_NUM];
    float b[OMNI_WHEEL_NUM];
    ChassisVxyPriorityScale_s scale;
    const float wheel_speed_limit = fabsf((float)chassis_vxy_priority_wheel_speed_limit);
    float sw_min = ChassisClampFloat((float)chassis_vxy_priority_wz_min_scale, 0.0f, 1.0f);
    const float wz_deadband = fabsf((float)chassis_vxy_priority_wz_deadband);
    // chassis.c 的 y 正向为右，omni_ctrl.h 的输入 y 正向为左，预测轮速前先换符号。
    const float vy_left = -chassis_vy;

    // 未使能、omni 矩阵未初始化或轮速上限无效时，直接保持原始参考。
    if (chassis_vxy_priority_enable == 0U ||
        omni_ctrl.inited == 0U ||
        wheel_speed_limit <= CHASSIS_VXY_PRIORITY_AXIS_EPS) {
        return;
    }

    // wz 接近 0 时不需要强制 70% 保底，避免零附近的小量被保底策略放大。
    if (fabsf(chassis_cmd_recv.wz) < wz_deadband) {
        sw_min = 0.0f;
    }

    /*
     * 用当前实车拟合矩阵 h 预测四轮速度占用：
     * a 是平移 vx/vy 对四轮的贡献，b 是当前 wz 对四轮的贡献。
     * 后续搜索实际检查的是 sv * a + sw * b 是否逐轮小于 wheel_speed_limit。
     */
    for (uint8_t wheel = 0U; wheel < OMNI_WHEEL_NUM; ++wheel) {
        a[wheel] = omni_ctrl.h[wheel][OMNI_AXIS_VX] * chassis_vx +
                   omni_ctrl.h[wheel][OMNI_AXIS_VY] * vy_left;
        b[wheel] = omni_ctrl.h[wheel][OMNI_AXIS_WZ] * chassis_cmd_recv.wz;
    }

    /*
     * 正常工作区：sw 在 [wz_min_scale, 1] 内搜索，优先最大化 sv 以保平移。
     * 如果纯 wz_min_scale 都已经让轮速超限，说明 70% 保底本身不安全，
     * 此时进入安全降级区，允许 sw 从 0 搜到 wz_min_scale。
     */
    if (sw_min > 0.0f &&
        ChassisPureWzScaleFeasible(b, sw_min, wheel_speed_limit) == 0U) {
        scale = ChassisSearchVxyPriorityScale(a, b, 0.0f, sw_min, wheel_speed_limit);
    } else {
        scale = ChassisSearchVxyPriorityScale(a, b, sw_min, 1.0f, wheel_speed_limit);
    }

    // 理论上只有轮速上限过小等异常配置才会无解；无解时清零，避免输出不可控参考。
    if (scale.feasible == 0U) {
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        chassis_cmd_recv.wz = 0.0f;
        return;
    }

    // 搜索目标是先保 vx/vy 的比例 sv；在 sv 相同的情况下再尽量保留 wz 的比例 sw。
    chassis_vx *= scale.sv;
    chassis_vy *= scale.sv;
    chassis_cmd_recv.wz *= scale.sw;
}

static void MecanumCalculate()
{
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_vx - chassis_vy + chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = -chassis_vx - chassis_vy + chassis_cmd_recv.wz * RB_CENTER;

    updateAllWheelsFriction(vt_lf, vt_rf, vt_lb, vt_rb);
}

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

static void ChassisPublishFeedback(void)
{
    float cap_voltage = 0.0f;
    uint8_t cap_online = 0U;

    Power_Output = chassis_cmd_recv.power_limit;
    if (supercap != NULL) {
        cap_voltage = SuperCapGetChassisVoltage(supercap);
        cap_online = SuperCapIsOnline(supercap);
        SuperCapSetPowerLimit(supercap, chassis_cmd_recv.power_limit);
    }

    memcpy(&chassis_feedback_data.cap_voltage, &cap_voltage, sizeof(float));
    memcpy(&chassis_feedback_data.cap_online_flag, &cap_online, sizeof(uint8_t));
    memcpy(&chassis_feedback_data.chassis_power_output, &Power_Output, sizeof(float));
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
    motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
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

static float ChassisOmniSiToCmd(uint8_t axis, float value_si)
{
    const float scale = (axis < (uint8_t)OMNI_AXIS_NUM) ? omni_ctrl.cmd_to_si[axis] : 0.0f;

    if (fabsf(scale) <= 1.0e-9f) {
        return 0.0f;
    }

    return value_si / scale;
}

static uint8_t ChassisCurrentForceCalibWheelIndexValid(uint8_t wheel_index)
{
    return (uint8_t)((wheel_index == 0xFFU) || (wheel_index < (uint8_t)OMNI_WHEEL_NUM));
}

static void ChassisResetCurrentForceCalibPulse(ChassisForceCalibPulseState_e state)
{
    chassis_force_calib_pulse_state = state;
    chassis_force_calib_trigger_time_s = 0.0f;
    chassis_force_calib_pulse_elapsed_s = 0.0f;
    chassis_force_calib_latched_current_cmd_fit = 0.0f;
    chassis_force_calib_last_current_cmd_fit = 0.0f;
    chassis_force_calib_latched_wheel_index = 0xFFU;
    chassis_force_calib_pulse_done = 0U;
    chassis_force_calib_hard_timeout = 0U;
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

static void ChassisEnterCurrentForceCalib(void)
{
    // 标定模式直接发送 DJI 原始电流命令，四轮先统一切到开环并清零。
    if (chassis_lqi_current_active != 0U) {
        ChassisExitLqiCurrentMode();
    }
    ChassisZeroWheelRef();
    ChassisSetWheelOpenLoop(motor_lf);
    ChassisSetWheelOpenLoop(motor_rf);
    ChassisSetWheelOpenLoop(motor_rb);
    ChassisSetWheelOpenLoop(motor_lb);
    ChassisResetCurrentForceCalibPulse(CHASSIS_FORCE_CALIB_PULSE_WAIT_TRIGGER);
    chassis_force_calib_active = 1U;
}

static void ChassisExitCurrentForceCalib(void)
{
    // 退出标定模式时恢复正常速度环，避免开环状态残留到主底盘控制。
    ChassisSetAllWheelRefZero();
    ChassisSetWheelSpeedLoop(motor_lf);
    ChassisSetWheelSpeedLoop(motor_rf);
    ChassisSetWheelSpeedLoop(motor_rb);
    ChassisSetWheelSpeedLoop(motor_lb);
    ChassisZeroWheelRef();
    OmniVelocityCtrl_Reset(&omni_ctrl);
    ChassisResetCurrentForceCalibPulse(CHASSIS_FORCE_CALIB_PULSE_IDLE);
    chassis_force_calib_active = 0U;
}

static void ChassisUpdateCurrentForceCalibRecord(uint8_t valid,
                                                 float current_limit,
                                                 float current_cmd_fit,
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
    record->enable = g_chassis_force_calib_cmd.enable;
    record->wheel_index =
        (chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_ACTIVE ||
         chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_DONE ||
         chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_HARD_TIMEOUT) ?
            chassis_force_calib_latched_wheel_index : g_chassis_force_calib_cmd.wheel_index;
    record->valid = valid;
    record->calib_active = chassis_force_calib_active;
    record->pulse_state = (uint8_t)chassis_force_calib_pulse_state;
    record->pulse_done = chassis_force_calib_pulse_done;
    record->hard_timeout = chassis_force_calib_hard_timeout;
    record->reserved = 0U;
    record->current_cmd_fit = current_cmd_fit;
    record->current_limit = current_limit;
    record->current_cmd_motor_lf = current_cmd_lf;
    record->current_cmd_motor_rf = current_cmd_rf;
    record->current_cmd_motor_rb = current_cmd_rb;
    record->current_cmd_motor_lb = current_cmd_lb;
    record->real_current = (current_fit_lf + current_fit_rf + current_fit_rb + current_fit_lb) * 0.25f;
    record->real_current_fit_lf = current_fit_lf;
    record->real_current_fit_rf = current_fit_rf;
    record->real_current_fit_rb = current_fit_rb;
    record->real_current_fit_lb = current_fit_lb;
    record->wheel_speed_fit = speed_max_abs;
    record->wheel_speed_fit_lf = speed_fit_lf;
    record->wheel_speed_fit_rf = speed_fit_rf;
    record->wheel_speed_fit_rb = speed_fit_rb;
    record->wheel_speed_fit_lb = speed_fit_lb;
    // 底盘 IMU debug 记录使用 REP-103 坐标：x forward, y left, z up。
    if (INS != NULL) {
        record->body_acc_x = INS->INS_data.INS_accel[0];
        record->body_acc_y = INS->INS_data.INS_accel[1];
        record->body_acc_z = INS->INS_data.INS_accel[2];
        record->body_omega_z = INS->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];
        record->body_roll = INS->output.INS_angle[INS_ROLL_ADDRESS_OFFSET];
        record->body_pitch = INS->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
        record->body_yaw = INS->output.INS_angle[INS_YAW_ADDRESS_OFFSET];
        record->body_yaw_total = INS->output.Yaw_total_angle;
    } else {
        record->body_acc_x = 0.0f;
        record->body_acc_y = 0.0f;
        record->body_acc_z = 0.0f;
        record->body_omega_z = 0.0f;
        record->body_roll = 0.0f;
        record->body_pitch = 0.0f;
        record->body_yaw = 0.0f;
        record->body_yaw_total = 0.0f;
    }
    record->sample_time_s = DWT_GetTimeline_s();
    record->trigger_time_s = chassis_force_calib_trigger_time_s;
    record->pulse_elapsed_s = chassis_force_calib_pulse_elapsed_s;
    record->force_raw = 0.0f;
    record->force_fit = 0.0f;
}

static uint8_t ChassisRunCurrentForceCalib(void)
{
    float current_limit = g_chassis_force_calib_cmd.current_limit;
    float current_cmd_fit;
    float debug_current_cmd_fit;
    float applied_current_cmd_fit = 0.0f;
    float current_cmd_lf;
    float current_cmd_rf;
    float current_cmd_rb;
    float current_cmd_lb;
    const uint8_t wheel_index = g_chassis_force_calib_cmd.wheel_index;
    uint8_t wheel_index_valid;
    uint8_t cmd_nonzero;
    uint8_t cmd_rising;
    uint8_t valid;
    float now_s;

    if (g_chassis_force_calib_cmd.enable == 0U) {
        if (chassis_force_calib_active != 0U) {
            ChassisExitCurrentForceCalib();
            ChassisUpdateCurrentForceCalibRecord(0U,
                                                CHASSIS_FORCE_CALIB_DEFAULT_CURRENT_LIMIT,
                                                0.0f,
                                                0.0f,
                                                0.0f,
                                                0.0f,
                                                0.0f);
        }
        return 0U;
    }

    if (chassis_force_calib_active == 0U) {
        ChassisEnterCurrentForceCalib();
    }

    if (current_limit <= 0.0f) {
        current_limit = CHASSIS_FORCE_CALIB_DEFAULT_CURRENT_LIMIT;
    }
    if (current_limit > CHASSIS_FORCE_CALIB_HARD_CURRENT_LIMIT) {
        current_limit = CHASSIS_FORCE_CALIB_HARD_CURRENT_LIMIT;
    }

    current_cmd_fit = ChassisClampAbs(g_chassis_force_calib_cmd.current_cmd_fit, current_limit);
    wheel_index_valid = ChassisCurrentForceCalibWheelIndexValid(wheel_index);
    cmd_nonzero = (fabsf(current_cmd_fit) > CHASSIS_FORCE_CALIB_TRIGGER_EPS) ? 1U : 0U;
    cmd_rising = (cmd_nonzero != 0U &&
                  fabsf(chassis_force_calib_last_current_cmd_fit) <= CHASSIS_FORCE_CALIB_TRIGGER_EPS) ? 1U : 0U;
    now_s = DWT_GetTimeline_s();

    if (chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_IDLE) {
        ChassisResetCurrentForceCalibPulse(CHASSIS_FORCE_CALIB_PULSE_WAIT_TRIGGER);
    }

    if (wheel_index_valid == 0U) {
        if (chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_ACTIVE) {
            chassis_force_calib_pulse_state = CHASSIS_FORCE_CALIB_PULSE_DONE;
            chassis_force_calib_pulse_done = 1U;
        }
    } else if (chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_WAIT_TRIGGER) {
        chassis_force_calib_pulse_done = 0U;
        chassis_force_calib_hard_timeout = 0U;
        chassis_force_calib_pulse_elapsed_s = 0.0f;
        if (cmd_rising != 0U) {
            chassis_force_calib_latched_current_cmd_fit = current_cmd_fit;
            chassis_force_calib_latched_wheel_index = wheel_index;
            chassis_force_calib_trigger_time_s = now_s;
            chassis_force_calib_pulse_state = CHASSIS_FORCE_CALIB_PULSE_ACTIVE;
        }
    } else if ((chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_DONE ||
                chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_HARD_TIMEOUT) &&
               cmd_nonzero == 0U) {
        ChassisResetCurrentForceCalibPulse(CHASSIS_FORCE_CALIB_PULSE_WAIT_TRIGGER);
    }

    if (chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_ACTIVE) {
        chassis_force_calib_pulse_elapsed_s = now_s - chassis_force_calib_trigger_time_s;
        if (chassis_force_calib_pulse_elapsed_s < 0.0f) {
            chassis_force_calib_pulse_elapsed_s = 0.0f;
        }

        if (cmd_nonzero == 0U) {
            chassis_force_calib_pulse_state = CHASSIS_FORCE_CALIB_PULSE_DONE;
            chassis_force_calib_pulse_done = 1U;
        } else if (chassis_force_calib_pulse_elapsed_s >= CHASSIS_FORCE_CALIB_HARD_TIMEOUT_S) {
            chassis_force_calib_pulse_state = CHASSIS_FORCE_CALIB_PULSE_HARD_TIMEOUT;
            chassis_force_calib_pulse_done = 1U;
            chassis_force_calib_hard_timeout = 1U;
        } else if (chassis_force_calib_pulse_elapsed_s >= CHASSIS_FORCE_CALIB_PULSE_TIME_S) {
            chassis_force_calib_pulse_state = CHASSIS_FORCE_CALIB_PULSE_DONE;
            chassis_force_calib_pulse_done = 1U;
        } else {
            applied_current_cmd_fit = chassis_force_calib_latched_current_cmd_fit;
        }
    } else if (chassis_force_calib_trigger_time_s > 0.0f) {
        chassis_force_calib_pulse_elapsed_s = now_s - chassis_force_calib_trigger_time_s;
        if (chassis_force_calib_pulse_elapsed_s < 0.0f) {
            chassis_force_calib_pulse_elapsed_s = 0.0f;
        }
    }

    ChassisBuildCurrentForceCalibRefs(chassis_force_calib_latched_wheel_index,
                                      applied_current_cmd_fit,
                                      &current_cmd_lf,
                                      &current_cmd_rf,
                                      &current_cmd_rb,
                                      &current_cmd_lb);
    ChassisSetCurrentForceCalibRefs(current_cmd_lf,
                                    current_cmd_rf,
                                    current_cmd_rb,
                                    current_cmd_lb);

    valid = (motor_lf != NULL &&
             motor_rf != NULL &&
             motor_rb != NULL &&
             motor_lb != NULL &&
             wheel_index_valid != 0U) ? 1U : 0U;
    debug_current_cmd_fit =
        (chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_ACTIVE ||
         chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_DONE ||
         chassis_force_calib_pulse_state == CHASSIS_FORCE_CALIB_PULSE_HARD_TIMEOUT) ?
            chassis_force_calib_latched_current_cmd_fit : current_cmd_fit;
    ChassisZeroWheelRef();
    ChassisUpdateCurrentForceCalibRecord(valid,
                                        current_limit,
                                        debug_current_cmd_fit,
                                        current_cmd_lf,
                                        current_cmd_rf,
                                        current_cmd_rb,
                                        current_cmd_lb);
    chassis_force_calib_last_current_cmd_fit = current_cmd_fit;
    return 1U;
}

static void ChassisUpdateOmniHCalibRecord(void)
{
    volatile OmniHCalibRecord_s *record = &g_omni_h_calib_record;
    const float real_current_lf = (motor_lf != NULL) ? (float)motor_lf->measure.real_current : 0.0f;
    const float real_current_rf = (motor_rf != NULL) ? (float)motor_rf->measure.real_current : 0.0f;
    const float real_current_rb = (motor_rb != NULL) ? (float)motor_rb->measure.real_current : 0.0f;
    const float real_current_lb = (motor_lb != NULL) ? (float)motor_lb->measure.real_current : 0.0f;
    const float cmd_vx = (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE ||
                          g_chassis_force_calib_cmd.enable != 0U) ? 0.0f : chassis_vx;
    const float cmd_vy_left = (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE ||
                               g_chassis_force_calib_cmd.enable != 0U) ? 0.0f : -chassis_vy;
    const float cmd_wz = (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE ||
                          g_chassis_force_calib_cmd.enable != 0U) ? 0.0f : chassis_cmd_recv.wz;

    record->current_limit_active = omni_output.current_limit_active;
    record->current_rate_limit_active = omni_output.current_rate_limit_active;
    record->eso_compensate_enable = g_chassis_lqi_current_cmd.eso_compensate_enable;
    record->reserved = 0U;
    record->cmd_vx = cmd_vx;
    record->cmd_vy_left = cmd_vy_left;
    record->cmd_wz = cmd_wz;
    record->eso_comp_vx = g_chassis_lqi_current_cmd.eso_comp_vx;
    record->eso_comp_vy = g_chassis_lqi_current_cmd.eso_comp_vy;
    record->eso_comp_wz = g_chassis_lqi_current_cmd.eso_comp_wz;
    record->omni_vx_hat = omni_output.nu_hat[OMNI_AXIS_VX];
    record->omni_vy_left_hat = omni_output.nu_hat[OMNI_AXIS_VY];
    record->omni_wz_hat = omni_output.nu_hat[OMNI_AXIS_WZ];
    record->eso_vx_hat = ChassisOmniSiToCmd(OMNI_AXIS_VX, omni_output.nu_eso[OMNI_AXIS_VX]);
    record->eso_vy_left_hat = ChassisOmniSiToCmd(OMNI_AXIS_VY, omni_output.nu_eso[OMNI_AXIS_VY]);
    record->eso_wz_hat = ChassisOmniSiToCmd(OMNI_AXIS_WZ, omni_output.nu_eso[OMNI_AXIS_WZ]);
    record->eso_dist_vx = omni_output.dist_hat[OMNI_AXIS_VX];
    record->eso_dist_vy = omni_output.dist_hat[OMNI_AXIS_VY];
    record->eso_dist_wz = omni_output.dist_hat[OMNI_AXIS_WZ];
    record->wheel_current_ref_fit_lf =
        omni_ctrl.config.wheel_dir[OMNI_WHEEL_LF] * omni_output.wheel_current_ref[OMNI_WHEEL_LF];
    record->wheel_current_ref_fit_rf =
        omni_ctrl.config.wheel_dir[OMNI_WHEEL_RF] * omni_output.wheel_current_ref[OMNI_WHEEL_RF];
    record->wheel_current_ref_fit_rb =
        omni_ctrl.config.wheel_dir[OMNI_WHEEL_RB] * omni_output.wheel_current_ref[OMNI_WHEEL_RB];
    record->wheel_current_ref_fit_lb =
        omni_ctrl.config.wheel_dir[OMNI_WHEEL_LB] * omni_output.wheel_current_ref[OMNI_WHEEL_LB];
    record->real_current_fit_lf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LF] * real_current_lf;
    record->real_current_fit_rf = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RF] * real_current_rf;
    record->real_current_fit_rb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_RB] * real_current_rb;
    record->real_current_fit_lb = omni_ctrl.config.wheel_dir[OMNI_WHEEL_LB] * real_current_lb;
    record->force_cmd_lf = omni_output.force_cmd[OMNI_WHEEL_LF];
    record->force_cmd_rf = omni_output.force_cmd[OMNI_WHEEL_RF];
    record->force_cmd_rb = omni_output.force_cmd[OMNI_WHEEL_RB];
    record->force_cmd_lb = omni_output.force_cmd[OMNI_WHEEL_LB];
    record->slip_prob_lf = omni_output.slip_prob[OMNI_WHEEL_LF];
    record->slip_prob_rf = omni_output.slip_prob[OMNI_WHEEL_RF];
    record->slip_prob_rb = omni_output.slip_prob[OMNI_WHEEL_RB];
    record->slip_prob_lb = omni_output.slip_prob[OMNI_WHEEL_LB];
}

static void ChassisRunOmniVelocityControlDirect(float vx_ref,
                                                float vy_left_ref,
                                                float wz_ref,
                                                float dt)
{
    OmniVelocityCtrlInput_s input;
    INS_Instance *lqi_imu = ChassisSelectLqiImu();

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
    // 临时 60W 功率限制：后续接入真实当前功率数据后删除或替换这条新链路。
    input.temp_power_limit_w = 60.0f;
    input.current_offset_enable = g_chassis_lqi_current_cmd.current_offset_enable;
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

static ramp_t super_ramp;
const float buffer_energy_loop_kp = 0.5f;
const float cap_voltage_loop_kp = 20.0f;
/**
 * @brief 超电控制算法
 *
 *
 */
 void Super_Cap_control()
 {
    float buffer_power_rectification, cap_power_rectification;
    float buffer_energy_target, buffer_energy_actual, cap_voltage_target, cap_voltage_actual;

    Power_Output = chassis_cmd_recv.power_limit;

    //缓冲能量环
    buffer_energy_actual = chassis_cmd_recv.power_buffer;
    buffer_energy_target = 50.0f;
    buffer_power_rectification = (buffer_energy_actual - buffer_energy_target) * buffer_energy_loop_kp;
    Power_Output += buffer_power_rectification;

    //电容能量环
    if (supercap != NULL && SuperCapIsOnline(supercap))
    {
        cap_voltage_actual = SuperCapGetChassisVoltage(supercap);
        if (chassis_cmd_recv.SuperCap_flag_from_user == SUPERCAP_USE)
            cap_voltage_target = 15.0f;
        else cap_voltage_target = 23.0f;
        cap_power_rectification = (cap_voltage_actual - cap_voltage_target) * cap_voltage_loop_kp;
        if (cap_power_rectification > 50.0f) cap_power_rectification = 70.0f;
        Power_Output += cap_power_rectification;
    }
    
    Power_Output -= 10.0f; //静态功耗

    PowerControlupdate(Power_Output, 1.0f / REDUCTION_RATIO_WHEEL);

    if (supercap != NULL) {
        SuperCapSetPowerLimit(supercap, chassis_cmd_recv.power_limit);
    }
    // 设定速度参考值
    ChassisApplyWheelOutput();
 }

// 获取功率裆位
 static void Power_get()
 {
     //cap->cap_msg_g.power_limit = chassis_cmd_recv.power_limit - 30 + 30 * (cap->cap_msg_s.CapVot - 17.0f) / 6.0f;
 }
extern Power_Data_s power_data; // 电机功率数据;
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

    /*
     * 输入边界：
     * chassis_cmd 来自 robot_cmd，是本周期底盘唯一的命令入口；
     * gimbal_feed 只作为反馈缓存，主要给底盘侧读取云台/IMU相关状态。
     */
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    chassis_cmd_recv.chassis_mode=CHASSIS_ZERO_FORCE;
    if (gimbal_feed_sub != NULL) {
        (void)SubGetMessage(gimbal_feed_sub, &chassis_gimbal_feedback);
    }

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
        ChassisEnableWheelMotors();
        if (ChassisRunCurrentForceCalib() != 0U) {
            ChassisUpdateOmniHCalibRecord();
            ChassisPublishFeedback();
            return;
        }
    }

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE){ 
        /*
         * 零力模式优先级最高：
         * 清空轮速参考、退出可能残留的标定状态、复位 omni 控制器，最后停掉四个底盘电机。
         */
        ChassisZeroWheelRef();
        if (chassis_force_calib_active != 0U) {
            ChassisExitCurrentForceCalib();
        }
        OmniVelocityCtrl_Reset(&omni_ctrl);
        memset(&omni_output, 0, sizeof(omni_output));
        chassis_lqi_current_active = 0U;
        ChassisStopWheelMotors();
        ChassisUpdateCurrentForceCalibRecord(0U,
                                            CHASSIS_FORCE_CALIB_DEFAULT_CURRENT_LIMIT,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f);
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
        ChassisUpdateLqiCurrentMode();
    }

    if (ChassisRunCurrentForceCalib() != 0U) {
        /*
         * 标定状态机可能在非零力分支后才进入 active。
         * 一旦它接管输出，本周期不再运行 yaw/follow/omni 正常控制，避免两套输出互相覆盖。
         */
        ChassisUpdateOmniHCalibRecord();
        ChassisPublishFeedback();
        return;
    }

    // current_speed_vw 用于自旋角速度软启动，避免切入 rotate 时 wz 阶跃。
    static float current_speed_vw, vw_set;
    static ramp_t rotate_ramp;

    // 所有可控模式下 vx/vy 都按 yaw 帧解释：推杆"前"= 沿云台指向移动。
    // 旋转矩阵只算一次，模式段只负责 wz / powerLim。
    const float theta_rad = chassis_cmd_recv.offset_angle * DEGREE_2_RAD;
    const float cos_theta = arm_cos_f32(theta_rad);
    const float sin_theta = arm_sin_f32(theta_rad);

    if (chassis_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) {
        /*
         * 狗洞模式 = 跟随模式 + 目标 -67°。共用 Chassis_Follow_PID,
         * setpoint 由 0 切到 HOLE_YAW_OFFSET_TARGET_DEG;避免普通 follow/rotate 抢占 yaw 指令。
         */
        chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID,
                                           chassis_cmd_recv.offset_angle,
                                           HOLE_YAW_OFFSET_TARGET_DEG);
        powerLim = 0;
        // 离开普通自旋时清掉自旋斜坡状态，下次 rotate 从 0 平滑起转。
        current_speed_vw = 0.0f;
    } else switch (chassis_cmd_recv.chassis_mode) {
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
             * 自旋模式下 vx/vy 仍参与全向移动，wz 由固定目标和斜坡生成。
             * supercap 标志原本用于区分目标转速，当前后面强制 vw_set=8000，调参时注意这个覆盖。
             */
            powerLim = chassis_cmd_recv.power_limit;
            if (chassis_cmd_recv.SuperCap_flag_from_user == SUPERCAP_USE) {
                vw_set = 5000;
            } else {
                vw_set = 3000;
            }
            chassis_vw = 8000;
            // chassis_vw       = (current_speed_vw + (vw_set - current_speed_vw) * ramp_calc(&rotate_ramp));
            current_speed_vw = chassis_vw;
            chassis_cmd_recv.wz = chassis_vw;
            break;

        case CHASSIS_REVERSE_ROTATE:
            // 反向自旋保留为独立模式，直接给负向 wz，不走 rotate_ramp。
            chassis_cmd_recv.wz = -5000;
            break;

        default:
            // 未识别模式不改写 wz，沿用 robot_cmd 下发值，便于调试新模式时观察原始命令。
            break;
    }

/*-------------------------底盘解算------------------------------------------------------*/
    /*
     * 加速度限幅：在命令(云台)系对 vx/vy/wz 做斜率限制，避免操作员/视觉给出阶跃量造成的过大加速度。
     * vx/vy 放在旋转到底盘系之前——自旋时 offset_angle 快速变化会让底盘系 vx/vy 正弦摆动，
     * 在那一侧限会"咬住"摆动；在命令系限只约束操作员的指令变化率。
     * wz 覆盖 REVERSE_ROTATE 等无软启动模式产生的阶跃，所有模式分支汇合后统一过这一关，
     * 比 omni 内部的 wz_acc_limit 更早一层，且暴露为文件内可调旋钮。
     */
    const float vxy_max_step = vxy_max_accel * ctrl_dt;
    chassis_cmd_recv.vx = SlewRateLimit(chassis_cmd_recv.vx, chassis_vx_cmd_prev, vxy_max_step);
    chassis_cmd_recv.vy = SlewRateLimit(chassis_cmd_recv.vy, chassis_vy_cmd_prev, vxy_max_step);
    chassis_vx_cmd_prev = chassis_cmd_recv.vx;
    chassis_vy_cmd_prev = chassis_cmd_recv.vy;

    const float wz_max_step = wz_max_accel * ctrl_dt;
    chassis_cmd_recv.wz = SlewRateLimit(chassis_cmd_recv.wz, chassis_wz_cmd_prev, wz_max_step);
    chassis_wz_cmd_prev = chassis_cmd_recv.wz;

    /*
     * 根据云台和底盘的 offset_angle 把遥控/上层速度从云台坐标系旋到底盘坐标系。
     * 底盘逆时针为角度正方向；云台命令的 x 轴沿云台指向，y 轴按右手系定义。
     */
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    ChassisApplyVxyPrioritySpeedScale();

    /*
     * 进入 omni/LQI/ESO/轮力分配链路。
     * 输出同时包含 vt_* 轮端速度参考和 raw current 参考，最终采用哪套由 chassis_lqi_current_active 决定。
     */
    ChassisRunOmniVelocityControl(ctrl_dt);

    // 按当前输出模式把 raw current 或 speed ref 写入 DJI 电机参考。
    ChassisApplyWheelOutput();

    // Ozone/CSV 调试记录在输出后更新，保证记录的是本周期实际发送前后的控制状态。
    ChassisUpdateOmniHCalibRecord();

    // 下面的超电主动控制链路当前未启用；保留调用位置，避免以后恢复时接错到输出之后。
    //Super_Cap_control();
    //SuperCapTask();                                                        
    // 获得给电容传输的电容吸取功率等级
    // Power_get();
    // 给电容传输数据
    // SuperCapSend(cap, (uint8_t *)&cap->cap_msg_g);

    // 发布给 robot_cmd/UI：速度估计、超电电压、超电在线状态和底盘功率输出。
    ChassisPublishFeedback();
}
