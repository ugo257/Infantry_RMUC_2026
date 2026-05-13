#include "shoot.h"
#include "robot_board.h"
#include "robot_params.h"
#include "robot_types.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "tool.h"
#include "referee_UI.h"
#include "led.h"
#include "DMmotor.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r; // 拨盘电机
// float DM_4310_speed_target = 0;                                                                                             // DM电机目标
// int DM_enable_flag         = 0;                                                                                             // DM电机指令,1转0停
static Publisher_t *shoot_pub;
static Subscriber_t *shoot_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;         // 来自cmd的发射控制信息
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static int16_t hit_cnt = 3000;
// static int16_t shoot_delay=3000;
// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
float d_watch;
int32_t shoot_count;                     // 已发弹量
#define DEBOUNCE_DELAY_MS 50             // 消抖延迟时间，单位：毫秒
#define BUTTON_PRESSED    GPIO_PIN_RESET // 按键按下时引脚电平，通常为低电平
#define BUTTON_RELEASED   GPIO_PIN_SET   // 按键释放时引脚电平，通常为高电平
#define LOADER_MD_SPEED_KP_RAW_PER_DEG_S 0.07146f
#define LOADER_MD_SPEED_KI_RAW_PER_DEG_S 0.07760f
#define LOADER_MD_ANGLE_KP               80.0f
#define LOADER_MD_ANGLE_KD               3.0f
#define LOADER_MD_RPM_PER_HZ             270.0f
#define LOADER_MD_DEG_S_PER_RPM          6.0f
#define LOADER_MD_SPEED_LIMIT_RPM        20568.0f
#define LOADER_MD_GEAR_RATIO             36.0f
#define LOADER_REVERSE_OUTPUT_ANGLE_DEG_DEFAULT 5.0f
int load_speed           = 15000;
float shoot_speed_target = 36000, shoot2_speed_target = 36000, limit_speed_target = 400;
// 定义按键状态
typedef enum {
    BUTTON_STATE_IDLE,     // 按键未按下状态
    BUTTON_STATE_DEBOUNCE, // 按键消抖中
    BUTTON_STATE_PRESSED   // 按键已稳定按下
} ButtonState;

// 定义按键状态变量
static ButtonState buttonState   = BUTTON_STATE_IDLE;
static uint32_t lastDebounceTime = 0; // 记录消抖开始时间
void Laser_Init()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // 使能GPIOC时钟
    GPIOC->MODER &= ~(0x3 << (2 * 8));   // 清除PC8的模式位
    GPIOC->MODER |= (0x1 << (2 * 8));    // 配置为输出模式 (01)
    GPIOC->OTYPER &= ~(0x1 << 8);        // 配置为推挽输出（0）
    GPIOC->OSPEEDR |= (0x3 << (2 * 8));  // 设置输出速度为高速
    GPIOC->ODR &= (0x1 << 8);            // 设置PC8为高电平
}
void Laser_on()
{
    GPIOC->ODR |= (0x1 << 8); // 设置PC8为高电平
}
void Laser_off()
{
    GPIOC->ODR &= ~(0x1 << 8); // 设置PC8为高电平
}

// 按键读取函数
bool Button_ReadState(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
}

// 按键消抖处理函数
void Button_Debounce(void)
{
    bool buttonReading   = Button_ReadState();   // 读取当前按键状态
    uint32_t currentTime = DWT_GetTimeline_ms(); // 获取当前时间，单位为毫秒

    switch (buttonState) {
        case BUTTON_STATE_IDLE:
            if (buttonReading == BUTTON_PRESSED) {
                // 按键按下，进入消抖状态
                buttonState      = BUTTON_STATE_DEBOUNCE;
                lastDebounceTime = currentTime; // 记录按键按下的时间
            }
            break;

        case BUTTON_STATE_DEBOUNCE:
            // 如果按键保持按下状态超过消抖延迟时间
            if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
                if (buttonReading == BUTTON_PRESSED) {
                    // 按键稳定按下，进入按下状态
                    buttonState = BUTTON_STATE_PRESSED;
                    shoot_count++;
                    C_board_LEDSet(0xFF0000); // printf("Button Pressed!\n");  // 执行按键按下时的操作
                } else {
                    // 如果按键松开，返回空闲状态
                    buttonState = BUTTON_STATE_IDLE;
                }
            }
            break;

        case BUTTON_STATE_PRESSED:
            // 检测按键是否松开
            if (buttonReading == BUTTON_RELEASED) {
                // 按键释放，回到空闲状态
                buttonState = BUTTON_STATE_IDLE;
                C_board_LEDSet(0x00FF00); // printf("Button Released!\n");  // 执行按键释放时的操作
            }
            break;
    }
}
/* ---------------------- Friction-wheel ESO disturbance compensation ----------------------
 * 2nd-order linear ESO per markdown §3-§4:
 *     e_o = z1 - omega_meas
 *     z1 += dt * (z2 + b0*i_actual - beta1*e_o)
 *     z2 += dt * (-beta2*e_o)
 *     i_eso = -z2 / b0
 * Observer bandwidth tracks the live speed-PI gain: omega_0 = k_obs * speed_PID.Kp,
 * beta1 = 2*omega_0, beta2 = omega_0^2 (§3 bandwidth parameterization).
 * i_actual uses motor->measure.real_current (ESC current sense) per §6.1.
 */
typedef struct {
    /* Tunable parameters (watch / set via debugger) */
    float b0;          /* current -> angular-acceleration gain, raw_current -> deg/s^2. TUNE. */
    float k_obs;       /* omega_0 = k_obs * Kp (markdown §11.2: start low, raise gradually) */
    float i_eso_max;   /* clamp on the comp current; <= 0 disables this secondary clamp */
    float reach_ratio; /* once |speed| >= reach_ratio * |target|, i_eso_out is zeroed
                        * (observer keeps running). <= 0 disables this gate. */
    uint8_t enable;      /* master observer enable: 0 freezes z1/z2 and zeros output */
    uint8_t comp_enable; /* compensation enable: 0 -> observe-only (z1/z2 update,
                          * but i_eso_out is held at 0 -- useful for tuning the
                          * observer without touching the control loop yet) */

    /* Observer states */
    float z1;          /* estimated speed (deg/s); should track speed_aps */
    float z2;          /* estimated lumped disturbance (deg/s^2) */

    /* Cached derived gains (for live-watch sanity) */
    float wo;
    float beta1;
    float beta2;

    /* Last comp output (live-watch / publishing to friction_feedforward_*) */
    float i_eso_out;
} FrictionEso_t;

/* The two floats the DJI driver's current_feedforward_ptr will read each tick. */
float friction_feedforward_l = 0.0f;
float friction_feedforward_r = 0.0f;

static FrictionEso_t friction_l_eso = {
    .b0          = 1.0f,    /* TUNE: see markdown §12.1, estimate with current-step test */
    .k_obs       = 4.0f,    /* TUNE: omega_0 = k_obs * speed_PID.Kp */
    .i_eso_max   = 4000.0f, /* secondary clip; speed PI MaxOut is 16384 */
    .reach_ratio = 0.8f,    /* stop injecting comp once |speed| >= 80% of |target| */
    .enable      = 1,
    .comp_enable = 0,       /* observe-only; flip to 1 in debugger to enable comp */
    .z1 = 0.0f, .z2 = 0.0f,
    .wo = 0.0f, .beta1 = 0.0f, .beta2 = 0.0f,
    .i_eso_out = 0.0f,
};

static FrictionEso_t friction_r_eso = {
    .b0 = 1.0f, .k_obs = 4.0f, .i_eso_max = 4000.0f, .reach_ratio = 0.8f,
    .enable = 1, .comp_enable = 0,
    .z1 = 0.0f, .z2 = 0.0f,
    .wo = 0.0f, .beta1 = 0.0f, .beta2 = 0.0f,
    .i_eso_out = 0.0f,
};

#define SHOOT_TASK_DT_S 0.001f  /* ShootTask runs at 1 kHz via osDelay(1) */

/* User-intended Ki for each friction wheel. Mirrored into speed_PID.Ki every
 * tick while |fric_speed| >= FRIC_REF_EPS; forced to 0 (pure-P) otherwise.
 * Tune these via the debugger -- do NOT write speed_PID.Ki directly, since
 * the stop-gate below will clobber it. */
static float friction_l_speed_ki = 0.0f;
static float friction_r_speed_ki = 0.0f;

/* Magnitude (deg/s) below which the commanded friction-wheel speed is treated
 * as "stopped": ESO is bypassed and the speed loop runs as pure P. */
#define FRIC_REF_EPS 1.0f

/* ---------------------- Loader ESO disturbance compensation ----------------------
 * Same 2nd-order ESO struct, applied to the 拨盘 (loader) speed loop. The loader
 * runs an ANGLE_LOOP->SPEED_LOOP cascade most of the time; the inner speed loop's
 * setpoint is the angle PI output, exposed as loader->motor_controller.speed_PID.Ref
 * after each PIDCalculate (controller.c:178). When that magnitude is below
 * LOADER_REF_EPS the gate forces pure-P and skips ESO comp, matching the request
 * "目标速度=0 时取消补偿并只用纯P". */
float loader_feedforward = 0.0f;

static FrictionEso_t loader_eso = {
    .b0 = 1.0f, .k_obs = 4.0f, .i_eso_max = 4000.0f, .reach_ratio = 0.8f,
    .enable = 1, .comp_enable = 0,
    .z1 = 0.0f, .z2 = 0.0f,
    .wo = 0.0f, .beta1 = 0.0f, .beta2 = 0.0f,
    .i_eso_out = 0.0f,
};

/* User-intended Ki for the loader speed loop. Mirrored into speed_PID.Ki while
 * |loader.speed_PID.Ref| >= LOADER_REF_EPS; forced to 0 (pure-P) otherwise.
 * Seeded from the init-config value so behaviour matches the bare PI until the
 * user retunes via debugger. Tune THIS variable, not speed_PID.Ki directly. */
static float loader_speed_ki = LOADER_MD_SPEED_KI_RAW_PER_DEG_S;

#define LOADER_REF_EPS 1.0f  /* (deg/s, motor side) speed-Ref magnitude treated as "0" */

DJIMotorInstance *loader;

static float loader_offset_angle = 25;
static float current_angle;

void ShootInit()
{
#if defined(GIMBAL_BOARD)
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 0.3,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 15000,
                .MaxOut        = 16384,
            },

        },

        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedforward_flag   = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508};

    friction_config.can_init_config.tx_id                                = 1; // 左摩擦轮
    friction_config.controller_setting_init_config.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL;
    friction_config.controller_param_init_config.current_feedforward_ptr = &friction_feedforward_l;
    friction_l                                                           = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id                                = 2; // 右摩擦轮
    friction_config.controller_setting_init_config.motor_reverse_flag    = MOTOR_DIRECTION_REVERSE;
    friction_config.controller_param_init_config.current_feedforward_ptr = &friction_feedforward_r;
    friction_r                                                           = DJIMotorInit(&friction_config);
    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
    // 限位电机
    // Motor_Init_Config_s limit_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan1,
    //         .tx_id      = 1,
    //     },
    //     .controller_param_init_config = {
    //         .speed_PID = {
    //             .Kp            = 1,  // 10
    //             .Ki            = 10, // 1
    //             .Kd            = 0,
    //             .Improve       = PID_Integral_Limit,
    //             .IntegralLimit = 10000,
    //             .MaxOut        = 10000,
    //         },

    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED, .outer_loop_type = SPEED_LOOP, .close_loop_type = SPEED_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
    //     },
    //     .motor_type = M2006,
    // };
    // limit = DJIMotorInit(&limit_config);
    // DJIMotorStop(limit);
    //Laser_Init();
#endif
#if defined(CHASSIS_BOARD)
    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 4,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = LOADER_MD_ANGLE_KP,
                .Ki            = 0,
                .Kd            = LOADER_MD_ANGLE_KD,
                .Improve       = PID_Derivative_On_Measurement,
                .IntegralLimit = 0,
                .MaxOut        = LOADER_MD_SPEED_LIMIT_RPM * LOADER_MD_DEG_S_PER_RPM,
            },
            .speed_PID = {
                .Kp            = LOADER_MD_SPEED_KP_RAW_PER_DEG_S,
                .Ki            = LOADER_MD_SPEED_KI_RAW_PER_DEG_S,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut        = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type    = ANGLE_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type    = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
            .feedforward_flag   = CURRENT_FEEDFORWARD,
        },
        .motor_type = M2006 // 英雄使用m3508
    };

    loader_config.controller_param_init_config.current_feedforward_ptr = &loader_feedforward;
    loader = DJIMotorInit(&loader_config);

    DJIMotorStop(loader);


#endif
    shoot_cmd_recv.shoot_mode = SHOOT_ON; // 初始化后摩擦轮进入准备模式,也可将右拨杆拨至上一次来手动开启

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

float Block_Time;              // 堵转时间
float Reverse_Time;            // 反转时间
float current_record[6] = {0}; // 第五个为最近的射速 第六个为平均射速
float Block_Status;            // 拨弹盘状态
/**
 * @brief 堵转，弹速检测
 *
 */
// static void Load_Reverse()
// {
//     // 获取拨弹盘转速
//     current_record[0] = current_record[1];
//     current_record[1] = current_record[2];
//     current_record[2] = current_record[3];
//     current_record[3] = current_record[4];
//     // current_record[4] = loader->measure.real_current; // 第五个为最近的拨弹盘電流
//     current_record[5] = (current_record[0] + current_record[1] + current_record[2] + current_record[3] + current_record[4]) / 5.0;

//     if (current_record[5] < -9900) {
//         Block_Time++;
//     }
//     if (Reverse_Time >= 1) {
//         shoot_cmd_recv.load_mode = LOAD_REVERSE;
//         // shoot_cmd_recv.shoot_aim_angle=loader->measure.total_angle+10;
//         Reverse_Time++;

//         // 反转时间
//         if (Reverse_Time >= 200) {
//             Reverse_Time = 0;
//             Block_Time   = 0;
//         }
//     }

//     // 电流较大恢复正转
//     if (loader->measure.real_current > 2000) {
//         Reverse_Time = 0;
//         Block_Time   = 0;
//     }

//     else {
//         // 堵转时间5*发射任务周期（5ms）= 25ms
//         if (Block_Time > 200) {
//             Reverse_Time = 1;
//         }
//     }
// }

int heat_control    = 25; // 热量控制
float local_heat    = 0;  // 本地热量
int One_bullet_heat = 10; // 打一发消耗热量

// 热量控制算法
static void Shoot_Fric_data_process(void)
{
    /*----------------------------------变量常量------------------------------------------*/
    static bool bullet_waiting_confirm = false; // 等待比较器确认

    //获得第一级任一摩擦轮的转速
    float data = friction_l->measure.speed_aps; // 获取摩擦轮转速

    static uint16_t data_histroy[MAX_HISTROY]; // 做循环队列
    static uint8_t head = 0, rear = 0;         // 队列下标
    float moving_average[2];                   // 移动平均滤波
    uint8_t data_num;                          // 循环队列元素个数
    float derivative;                          // 微分
    /*-----------------------------------逻辑控制-----------------------------------------*/
    data = abs(data);
    /*入队*/
    data_histroy[head] = data;
    head++;
    head %= MAX_HISTROY;
    /*判断队列数据量*/
    data_num = (head - rear + MAX_HISTROY) % MAX_HISTROY;
    if (data_num >= Fliter_windowSize + 1) // 队列数据量满足要求
    {
        moving_average[0] = 0;
        moving_average[1] = 0;
        /*同时计算两个滤波*/
        for (uint8_t i = rear, j = rear + 1, index = rear; index < rear + Fliter_windowSize; i++, j++, index++) {
            i %= MAX_HISTROY;
            j %= MAX_HISTROY;
            moving_average[0] += data_histroy[i];
            moving_average[1] += data_histroy[j];
        }
        moving_average[0] /= Fliter_windowSize;
        moving_average[1] /= Fliter_windowSize;
        /*滤波求导*/
        derivative = moving_average[1] - moving_average[0];
        d_watch    = derivative;
        /*导数比较*/
        if (derivative < -300) {
            bullet_waiting_confirm = true;
        } else if (derivative > 30) {
            if (bullet_waiting_confirm == true) {
                local_heat += One_bullet_heat; // 确认打出
                shoot_count++;
                bullet_waiting_confirm = false;
            }
        }
        rear++;
        rear %= MAX_HISTROY;
    }
}

static float CalculateNextAngle(float current_angle)
{
    const float angle_step = PI / 3.0f;
    float target_angle, temp;
    uint16_t number;//与正方向相差几个PI/3
    temp = loader_offset_angle - current_angle + 0.2;
    number = (uint16_t)(temp / angle_step);
    target_angle = loader_offset_angle - ((float)(number + 1) * angle_step);
    return target_angle;
}

static int one_bullet;
static ramp_t fric_on_ramp;
static ramp_t fric_off_ramp;
float fric_speed = 0; // 摩擦轮转速参考值
uint32_t shoot_heat_count[2];
int load_mode_private = 1;
float diff1, diff2;
uint8_t loadmode;
float pid_kp = 1;

loader_mode_e last_load_mode = LOAD_STOP;
loader_state_e loader_state = LOAD_UNINIT;
float loader_initial_offset = 0;
float loader_pitch_offset = 0;
float loader_offset = 0;
int32_t load_count = 0;
float cool_down_time = 0;
float load_time_ms = 0;
volatile float loader_burst_speed_ref_deg_s = 0.0f;
volatile float loader_burst_rate_hz = 0.0f;
volatile uint8_t loader_burst_speed_mode_active = 0U;
volatile float loader_reverse_target_angle_deg = 0.0f;
volatile float loader_reverse_angle_output_deg = LOADER_REVERSE_OUTPUT_ANGLE_DEG_DEFAULT;
volatile float loader_reverse_angle_motor_deg = LOADER_REVERSE_OUTPUT_ANGLE_DEG_DEFAULT * LOADER_MD_GEAR_RATIO;

loader_mode_e last_mode;
// friction_mode_e friction_text_mode = FRICTION_OFF;
// shoot_mode_e shoot_text_mode       = SHOOT_OFF;
// static int cnt                     = 100;



/* 2nd-order linear ESO update per markdown §3-§4. beta1, beta2 are recomputed
 * every tick from the live speed-PI Kp so "beta tracks PI in real time" works
 * automatically as the user retunes Kp (or, later, Ki) at runtime. */
static void FrictionEso_Update(FrictionEso_t *eso, DJIMotorInstance *motor, float dt)
{
    /* Idle: zero comp output and freeze the disturbance state. z1 is held so
     * re-enable doesn't inject a step. */
    if (!eso->enable || motor->stop_flag == MOTOR_STOP) {
        eso->z2        = 0.0f;
        eso->i_eso_out = 0.0f;
        return;
    }

    /* Live beta tracking: omega_0 = k_obs * Kp, beta1 = 2*omega_0, beta2 = omega_0^2. */
    float kp = motor->motor_controller.speed_PID.Kp;
    eso->wo    = eso->k_obs * kp;
    eso->beta1 = 2.0f * eso->wo;
    eso->beta2 = eso->wo * eso->wo;

    /* Inputs: omega from encoder, current from ESC sense (markdown §6.1). */
    float omega_meas = motor->measure.speed_aps;
    float i_actual   = motor->measure.real_current;

    /* 2nd-order linear ESO, Forward-Euler step at dt. */
    float e_o = eso->z1 - omega_meas;
    eso->z1 += dt * (eso->z2 + eso->b0 * i_actual - eso->beta1 * e_o);
    eso->z2 += dt * (-eso->beta2 * e_o);

    /* Compensation current i_ESO = -z2 / b0 (markdown §1). */
    float i_eso = (eso->b0 != 0.0f) ? (-eso->z2 / eso->b0) : 0.0f;

    /* Optional secondary clamp before the driver's +/-16384 saturation. */
    if (eso->i_eso_max > 0.0f) {
        if (i_eso >  eso->i_eso_max) i_eso =  eso->i_eso_max;
        if (i_eso < -eso->i_eso_max) i_eso = -eso->i_eso_max;
    }
    /* comp_enable lets the user observe (z1, z2 keep updating above) without
     * actually injecting any compensation current into the speed loop. */
    eso->i_eso_out = eso->comp_enable ? i_eso : 0.0f;
}

/* Reach-gate: once |speed_meas| has climbed to reach_ratio * |speed_target|, zero
 * the ESO compensation output. The observer keeps running, so if speed dips back
 * below the threshold (e.g. heavy load step) comp resumes on the next tick.
 * reach_ratio <= 0 disables the gate entirely. */
static void FrictionEso_GateByReach(FrictionEso_t *eso, float speed_meas, float speed_target)
{
    if (eso->reach_ratio <= 0.0f) return;
    float target_abs = speed_target >= 0.0f ? speed_target : -speed_target;
    float meas_abs   = speed_meas   >= 0.0f ? speed_meas   : -speed_meas;
    if (meas_abs >= eso->reach_ratio * target_abs) {
        eso->i_eso_out = 0.0f;
    }
}

/* 云台板发射任务：摩擦轮、激光与本地热量反馈 */
void ShootTask_Gimbal(void)
{
    static float shoot_speed = 0;

    // shoot_cmd 由 robot_cmd 统一生成；云台板这里只消费摩擦轮、激光和热量相关字段。
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    /*
     * shoot_mode 是发射总开关。
     * 关闭时直接停两个摩擦轮，开启时只允许速度环工作，真正目标速度还要看 friction_mode。
     */
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
    } else {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
    }

    // switch (shoot_cmd_recv.load_mode) {
    //     case LOAD_STOP:
    //         Laser_off();
    //         shoot_heat_count[0] = shoot_count;
    //         shoot_heat_count[1] = shoot_heat_count[0];
    //         one_bullet          = 0;
    //         last_mode           = LOAD_STOP;
    //         break;

    //     case LOAD_1_BULLET:
    //         hibernate_time = DWT_GetTimeline_ms();
    //         dead_time      = 150;
    //         break;

    //     case LOAD_BURSTFIRE:
    //     case LOAD_REVERSE:
    //         break;

    //     default:
    //         while (1);
    // }

    /*
     * friction_mode 控制摩擦轮启停。
     * 开启/关闭都走 ramp，避免目标速度从 0 到 shoot_speed_target 或反向掉到 0 时产生电流冲击。
     * 激光跟随摩擦轮状态：摩擦轮开则开激光，摩擦轮关则关激光。
     */
    if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        fric_speed  = (shoot_speed + (shoot_speed_target - shoot_speed) * ramp_calc(&fric_on_ramp));
        ramp_init(&fric_off_ramp, 3000);
        Laser_on();
    } else if (shoot_cmd_recv.friction_mode == FRICTION_OFF) {
        fric_speed  = (shoot_speed + (0 - shoot_speed) * ramp_calc(&fric_off_ramp));
        ramp_init(&fric_on_ramp, 8000);
        Laser_off();
    }

    /*
     * 摩擦轮停止门：
     * 目标速度接近 0 时强制纯 P，清积分并跳过 ESO 补偿，防止停转附近积分和扰动估计残留。
     * 目标速度非 0 时再恢复调试变量里的 Ki，并更新 ESO；输出的补偿电流由 DJI 驱动作为电流前馈叠加。
     */
    if (fric_speed > -FRIC_REF_EPS && fric_speed < FRIC_REF_EPS) {
        friction_l->motor_controller.speed_PID.Ki    = 0.0f;
        friction_l->motor_controller.speed_PID.ITerm = 0.0f;
        friction_l->motor_controller.speed_PID.Iout  = 0.0f;
        friction_r->motor_controller.speed_PID.Ki    = 0.0f;
        friction_r->motor_controller.speed_PID.ITerm = 0.0f;
        friction_r->motor_controller.speed_PID.Iout  = 0.0f;
        friction_l_eso.z2        = 0.0f;
        friction_r_eso.z2        = 0.0f;
        friction_l_eso.i_eso_out = 0.0f;
        friction_r_eso.i_eso_out = 0.0f;
    } else {
        friction_l->motor_controller.speed_PID.Ki = friction_l_speed_ki;
        friction_r->motor_controller.speed_PID.Ki = friction_r_speed_ki;
        FrictionEso_Update(&friction_l_eso, friction_l, SHOOT_TASK_DT_S);
        FrictionEso_Update(&friction_r_eso, friction_r, SHOOT_TASK_DT_S);
        /* Reach-gate: once each wheel hits 80% of |fric_speed| stop the comp injection
         * (the observer keeps tracking and resumes if speed drops back). */
        FrictionEso_GateByReach(&friction_l_eso, friction_l->measure.speed_aps, fric_speed);
        FrictionEso_GateByReach(&friction_r_eso, friction_r->measure.speed_aps, fric_speed);
    }

    // 将 ESO 估计出的补偿电流发布到电机 current_feedforward_ptr 指向的变量。
    friction_feedforward_l = friction_l_eso.i_eso_out;
    friction_feedforward_r = friction_r_eso.i_eso_out;

    // 左右摩擦轮安装方向相反，最终目标都取 -fric_speed，由电机 reverse 配置处理物理方向。
    DJIMotorSetRef(friction_l, -fric_speed);
    DJIMotorSetRef(friction_r, -fric_speed);

    // 保存本周期摩擦轮目标，下一周期 ramp 以此为起点继续平滑变化。
    shoot_speed  = fric_speed;
    last_load_mode = shoot_cmd_recv.load_mode;

    // 本地热量和热量保护阈值回传给 robot_cmd/UI，供上层限制拨弹命令。
    memcpy(&shoot_feedback_data.shooter_local_heat, &local_heat, sizeof(float));
    memcpy(&shoot_feedback_data.shooter_heat_control, &heat_control, sizeof(int));
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

/* 底盘板发射任务：拨弹盘初始化、单发、连发与反转 */
void ShootTask_Chassis(void)
{
    // 从 CMD 任务接收云台板转发下来的发射控制指令。
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // shoot_mode 是总开关：关闭时直接停拨弹盘，开启时允许拨弹盘闭环工作。
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        DJIMotorStop(loader);
    } else {
        DJIMotorEnable(loader);
    }

    /*
     * 连发射频由 shoot_rate 给出，单位 Hz。
     * cool_down_time 是角度式连发旧逻辑的节拍间隔；当前速度式连发仍保留它，方便回退调试。
     */
    loader_burst_rate_hz = (shoot_cmd_recv.shoot_rate > 0.0f) ? shoot_cmd_recv.shoot_rate : 20.0f;
    cool_down_time = 1000.0f / loader_burst_rate_hz;

    switch (shoot_cmd_recv.load_mode) {
        case LOAD_STOP:
            /*
             * 停止拨弹：
             * 清连发速度模式和反转目标，切回角度环并把目标锁在当前角度，避免拨盘漂移。
             */
            loader_burst_speed_mode_active = 0U;
            loader_burst_speed_ref_deg_s = 0.0f;
            loader_reverse_target_angle_deg = loader->measure.total_angle;
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, loader->measure.total_angle);
            // 同步上板发来的发弹计数，并清除单发状态，防止再次进入单发时重复加一发。
            shoot_heat_count[0] = shoot_cmd_recv.shoot_count;
            shoot_heat_count[1] = shoot_heat_count[0];
            one_bullet          = 0;
            last_mode           = LOAD_STOP;
            break;

        case LOAD_1_BULLET:
            /*
             * 单发模式使用拨盘角度环。
             * 只有从 LOAD_STOP 刚切入时才推进一次 load_count，长时间保持单发命令不会连续加目标角。
             */
            loader_burst_speed_mode_active = 0U;
            loader_burst_speed_ref_deg_s = 0.0f;
            loader_reverse_target_angle_deg = loader->measure.total_angle;
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time      = 150;
            // 摩擦轮未开启时禁止拨弹，只把拨盘锁在当前位置。
            if (shoot_cmd_recv.friction_mode == FRICTION_OFF) {
                DJIMotorSetRef(loader, loader->measure.total_angle);
                break;
            }
            // 只有从停止进入单发时才尝试推进一发，避免长时间停留在单发模式中反复加计数。
            if (last_load_mode == LOAD_STOP) {
                // 当前拨盘已经越过上一发目标位置后，才把目标切到下一发。
                if (((load_count - 1) * LOADER_ANGLE_PER_BULLET + loader_initial_offset + loader_offset + loader_pitch_offset
                     - loader->measure.total_angle) < 0) {
                    load_count++;
                }
            }
            // 按发弹计数换算拨盘目标角。
            DJIMotorSetRef(loader, load_count * LOADER_ANGLE_PER_BULLET + loader_initial_offset + loader_offset + loader_pitch_offset);
            break;

        case LOAD_BURSTFIRE: {
            float loader_target_rpm;

            /*
             * 连发模式使用拨盘速度环。
             * 摩擦轮未开启时禁止拨弹，并切回角度环锁当前位置，防止拨盘空转推弹。
             */
            if (shoot_cmd_recv.friction_mode == FRICTION_OFF) {
                loader_burst_speed_mode_active = 0U;
                loader_burst_speed_ref_deg_s = 0.0f;
                loader_reverse_target_angle_deg = loader->measure.total_angle;
                DJIMotorOuterLoop(loader, ANGLE_LOOP);
                DJIMotorSetRef(loader, loader->measure.total_angle);
                break;
            }
            // 到达射频节拍后，尝试推进到下一发。
            // if ((DWT_GetTimeline_ms() - load_time_ms) > cool_down_time) {
            //     // 当前拨盘已经越过上一发目标位置后，才允许 load_count 增加。
            //     if (((load_count - 1) * LOADER_ANGLE_PER_BULLET + loader_initial_offset + loader_offset + loader_pitch_offset
            //          - loader->measure.total_angle) < 0) {
            //         load_count++;
            //     }
            //     load_time_ms = DWT_GetTimeline_ms();
            // }
            /*
             * 速度式连发：
             * shoot_rate 先按 MD 拨盘经验系数换算为 rpm，再限幅并换算成电机侧 deg/s，
             * 这样目标单位和 DJI 电机 speed_aps 反馈一致。
             */
            loader_target_rpm = loader_burst_rate_hz * LOADER_MD_RPM_PER_HZ;
            if (loader_target_rpm > LOADER_MD_SPEED_LIMIT_RPM) {
                loader_target_rpm = LOADER_MD_SPEED_LIMIT_RPM;
            } else if (loader_target_rpm < -LOADER_MD_SPEED_LIMIT_RPM) {
                loader_target_rpm = -LOADER_MD_SPEED_LIMIT_RPM;
            }
            loader_burst_speed_ref_deg_s = loader_target_rpm * LOADER_MD_DEG_S_PER_RPM;
            loader_burst_speed_mode_active = 1U;
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, loader_burst_speed_ref_deg_s);
            break;
        }

        case LOAD_REVERSE:
            /*
             * 退弹/防卡反转：
             * 刚进入 LOAD_REVERSE 时锁存一次反转目标，后续保持该目标，避免每拍都继续往后累加。
             */
            loader_burst_speed_mode_active = 0U;
            loader_burst_speed_ref_deg_s = 0.0f;
            loader_reverse_angle_motor_deg = loader_reverse_angle_output_deg * LOADER_MD_GEAR_RATIO;
            if (last_load_mode != LOAD_REVERSE) {
                loader_reverse_target_angle_deg = loader->measure.total_angle - loader_reverse_angle_motor_deg;
            }
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            // 退弹：默认反转 5 deg 输出轴，按 P36 齿比换算为 180 deg 电机侧角度。
            DJIMotorSetRef(loader, loader_reverse_target_angle_deg);
            break;

        default:
            // 未知 load_mode 不改目标；保留断点位置，方便暴露通信或枚举异常。
            //while (1);
    }

    // 记录本周期模式，供下一周期判断是否刚从 LOAD_STOP 进入单发。
    last_load_mode = shoot_cmd_recv.load_mode;

    /*
     * 拨盘停止门和 ESO 更新：
     * speed_PID.Ref 是内速度环参考，角度环模式下来自外角度 PI 输出，速度环模式下就是连发速度目标。
     * 当 |Ref| 很小时强制纯 P、清积分并清 ESO 输出，避免静止锁角时补偿电流顶住拨盘；
     * 有实际速度需求时再恢复 loader_speed_ki 并运行 ESO。
     */
    float loader_speed_ref = loader->motor_controller.speed_PID.Ref;
    if (loader_speed_ref > -LOADER_REF_EPS && loader_speed_ref < LOADER_REF_EPS) {
        loader->motor_controller.speed_PID.Ki    = 0.0f;
        loader->motor_controller.speed_PID.ITerm = 0.0f;
        loader->motor_controller.speed_PID.Iout  = 0.0f;
        loader_eso.z2        = 0.0f;
        loader_eso.i_eso_out = 0.0f;
    } else {
        loader->motor_controller.speed_PID.Ki = loader_speed_ki;
        FrictionEso_Update(&loader_eso, loader, SHOOT_TASK_DT_S);
        /* Reach-gate: once loader has hit 80% of its commanded speed (from angle-cascade
         * or direct burst-fire setpoint) drop the comp; observer keeps running. */
        FrictionEso_GateByReach(&loader_eso, loader->measure.speed_aps, loader_speed_ref);
    }
    loader_feedforward = loader_eso.i_eso_out;

    // 发布发射反馈给 CMD/UI 使用；当前主要包含本地热量和热量保护阈值。
    memcpy(&shoot_feedback_data.shooter_local_heat, &local_heat, sizeof(float));
    memcpy(&shoot_feedback_data.shooter_heat_control, &heat_control, sizeof(int));
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void Shoot_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)//再次确保是tim6
    {
        /*-------------------------------------------热量控制部分---------------------------------------------*/
        local_heat -= (shoot_cmd_recv.shooter_heat_cooling_rate / 1000.0f); // 1000Hz冷却
        if (local_heat < 0) {
            local_heat = 0;
        }
        if (shoot_cmd_recv.shooter_referee_heat - shoot_cmd_recv.shooter_cooling_limit >= 15) // 裁判系统判断已经超了热量
        {
            local_heat = shoot_cmd_recv.shooter_referee_heat;
        }
        Shoot_Fric_data_process();
    }
    else {
        //系统运行出现了某种未知的错误,因为这个回调函数正常情况下只会被tim6触发,其他定时器不应该调用这个函数,如果调用了说明代码中可能存在指针越界或者内存溢出等问题导致htim指向了错误的定时器
    }
}
