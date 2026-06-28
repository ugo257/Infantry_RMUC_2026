#include "DMmotor.h"
#include "stdlib.h"
#include "string.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"

#define DM_DEFAULT_P_MAX 12.5f
#define DM_DEFAULT_V_MAX 30.0f
#define DM_DEFAULT_T_MAX 10.0f

uint8_t idm                                               = 0;
static DMMotorInstance *dmmotor_instance[DM_MOTOR_MX_CNT] = {NULL};
void DMMotorEnableMode(DMMotorInstance *motor);
void DMMotorErrorDetection(DMMotorInstance *motor);
void mit_ctrl(DMMotorInstance *motor);

static float DMMotorAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float DMMotorLimit(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float DMMotorClampRange(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void DMMotorEnsureProtocolRange(DMMotorInstance *motor)
{
    if (motor->motor_settings.control_range.P_max <= 0.0f) {
        motor->motor_settings.control_range.P_max = DM_DEFAULT_P_MAX;
    }
    if (motor->motor_settings.control_range.V_max <= 0.0f) {
        motor->motor_settings.control_range.V_max = DM_DEFAULT_V_MAX;
    }
    if (motor->motor_settings.control_range.T_max <= 0.0f) {
        motor->motor_settings.control_range.T_max = DM_DEFAULT_T_MAX;
    }
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    if (x_float <= x_min) {
        return 0;
    }
    if (x_float >= x_max) {
        return (1 << bits) - 1;
    }
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void DMMotorDecode(CANInstance *_instance)
{
    DMMotorInstance *motor  = (DMMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    motor_fbpara_t *measure = &motor->measure;
    uint8_t *rx_buff        = _instance->rx_buff;

    // 只要能收到反馈就刷新守护；具体是否可控由 state/online 判断。
    if (motor->daemon != NULL) {
        DaemonReload(motor->daemon); // 喂狗
    }
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
    measure->last_pos = measure->pos;

    measure->id    = (rx_buff[0]) & 0x0F;
    measure->state = (rx_buff[0]) >> 4;
    measure->p_int = (rx_buff[1] << 8) | rx_buff[2];
    measure->v_int = (rx_buff[3] << 4) | (rx_buff[4] >> 4);
    measure->t_int = ((rx_buff[4] & 0xF) << 8) | rx_buff[5];
    measure->pos   = uint_to_float(measure->p_int, -motor->motor_settings.control_range.P_max, motor->motor_settings.control_range.P_max, 16); // (-12.5,12.5)
    measure->vel   = uint_to_float(measure->v_int, -motor->motor_settings.control_range.V_max, motor->motor_settings.control_range.V_max, 12); // (-45.0,45.0)
    measure->tor   = uint_to_float(measure->t_int, -motor->motor_settings.control_range.T_max, motor->motor_settings.control_range.T_max, 12); // (-18.0,18.0)
    measure->Tmos  = (float)(rx_buff[6]);
    measure->Tcoil = (float)(rx_buff[7]);
    motor->online  = (measure->state == 1) ? 1 : 0;

    // DM 协议回传的是单圈位置，这里补偿跨越正负边界时的累计圈数。
    if (measure->pos - measure->last_pos > (motor->motor_settings.control_range.P_max / 2)) measure->total_round--;
    if (measure->pos - measure->last_pos < -(motor->motor_settings.control_range.P_max / 2)) measure->total_round++;
    measure->total_pos = measure->pos + measure->total_round * 2 * motor->motor_settings.control_range.P_max;

    DMMotorErrorDetection(motor);
}

void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    motor->online          = 0;
    motor->ctrl.pos_set = 0.0f;
    motor->ctrl.vel_set = 0.0f;
    motor->ctrl.tor_set = 0.0f;
    motor->ctrl.kp_set  = 0.0f;
    motor->ctrl.kd_set  = 0.0f;
    // DMMotorControl(motor); // 立即下发清零控制量，停止电机动作
    // 掉线后持续发送使能帧，等待电机重新进入可控状态。
    DMMotorEnableMode(motor);
    CANTransmit(motor->motor_can_instance, 2);

    //LOGWARNING("[DMMotor] motor lost, id: %d", (motor->motor_can_instance->tx_id & (0x1f<<5)) >> 5);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    // motor = (DRMotorInstance *)malloc(sizeof(DRMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    // motor basic setting 电机基本设置
    motor->ctrl.kp_set    = 0;
    motor->ctrl.kd_set    = 0; // config->controller_param_init_config.speed_PID.Kd;
    motor->motor_settings = config->controller_setting_init_config;
    DMMotorEnsureProtocolRange(motor);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;
    motor->ctrl.mode                                 = config->motor_contro_type; // 控制模式
    motor->motor_type                                = config->motor_type;
    config->can_init_config.id                       = motor;
    config->can_init_config.can_module_callback      = DMMotorDecode;

    motor->motor_can_instance = CANRegister(&config->can_init_config);
    motor->online             = 0;

    DMMotorEnableMode(motor); // 使能电机模式,发送使能指令
    CANTransmit(motor->motor_can_instance, 2);
    motor->ctrl.vel_set = 0;
    motor->ctrl.tor_set = 0; //   默认关闭，避免出问题

    dmmotor_instance[idm++] = motor;
    //软件看门狗
    Daemon_Init_Config_s daemon_config = {
        .callback     = DMMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 4, // 4ms
    };
    motor->daemon = DaemonRegister(&daemon_config);
    motor->stop_flag = MOTOR_STOP;

    return motor;
}

/**
 * @brief 组包 DM 位置-速度模式报文
 *
 * @note 该模式直接发送 float 位置和速度, 与 MIT 压缩报文不同。
 */
void pos_ctrl(DMMotorInstance *motor)
{

    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    pbuf = (uint8_t *)&motor->ctrl.pos_set;
    vbuf = (uint8_t *)&motor->ctrl.vel_set;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    DMMotorSetTorque(motor, ref);
}

/**
 * @brief 发送所有 DM 电机控制报文
 *
 * @details DM 驱动层不再计算 PID。应用层直接写入 ctrl.tor_set 或
 *          ctrl.pos_set/ctrl.vel_set；本函数只处理在线检测、停机清零、
 *          报文组包和 CAN 发送。
 */
uint16_t cnt[2] ;
void DMMotorControl()
{
    DMMotorInstance *motor;
    for (size_t i = 0; i < idm; ++i) {
        motor = dmmotor_instance[i];
        cnt[i]= (cnt[i] + 1) % 500; // 每500次控制周期发送一次失能帧（如果处于失能状态）
        if(cnt[i] == 0){
            DMMotorEnableMode(motor);
            CANTransmit(motor->motor_can_instance, 2);
            continue;
        }
        /*
         * protocol_disable 是真正的 DM 协议失能状态，需要持续发送 FF..FD。
         * 这里放在自动重使能之前，避免失能期间被离线/非 state=1 分支重新拉起。
         */
        // if (motor->protocol_disable != 0u) {
        //     DMMotorDisableMode(motor);
        //     CANTransmit(motor->motor_can_instance, 2);
        //     continue;
        // }

        // 电机未进入正常控制状态前不要下发闭环输出，优先尝试重新使能。
        if (!motor->online || motor->measure.state != 1) {
            DMMotorEnableMode(motor);
            CANTransmit(motor->motor_can_instance, 2);
            continue;
        }

        // 停止时保留通信，但控制量清零，避免重新使能瞬间出现残留输出。
        if (motor->stop_flag == MOTOR_STOP) {
            motor->ctrl.pos_set = 0;
            motor->ctrl.vel_set = 0;
            motor->ctrl.tor_set = 0;
            motor->ctrl.kp_set  = 0;
            motor->ctrl.kd_set  = 0;
        }

        switch (motor->ctrl.mode) {
        case TORQUE_LOOP_CONTRO:
            DMMotorEnsureProtocolRange(motor);
            motor->ctrl.pos_set = DMMotorLimit(motor->ctrl.pos_set, DMMotorAbs(motor->motor_settings.control_range.P_max));
            motor->ctrl.vel_set = DMMotorLimit(motor->ctrl.vel_set, DMMotorAbs(motor->motor_settings.control_range.V_max));
            motor->ctrl.tor_set = DMMotorLimit(motor->ctrl.tor_set, DMMotorAbs(motor->motor_settings.control_range.T_max));
            motor->ctrl.kp_set  = DMMotorClampRange(motor->ctrl.kp_set, KP_MIN, KP_MAX);
            motor->ctrl.kd_set  = DMMotorClampRange(motor->ctrl.kd_set, KD_MIN, KD_MAX);
            mit_ctrl(motor);
            break;
        case ANGLE_LOOP_CONTRO:
            pos_ctrl(motor);
            break;
        default:
            mit_ctrl(motor);
            break;
        }
        CANTransmit(motor->motor_can_instance, 2);
    }
}

/**
 * @brief 组包 DM 电机使能帧
 */
void DMMotorEnableMode(DMMotorInstance *motor) //
{
    uint8_t data[8];

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}

/**
 * @brief 组包 DM 电机协议失能帧
 */
void DMMotorDisableMode(DMMotorInstance *motor)
{
    uint8_t data[8];

    if (motor == NULL) {
        return;
    }

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;
    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}

/**
 * @brief 组包 DM MIT 模式压缩控制报文
 *
 * @note 位置/速度/力矩量程来自 motor_settings.control_range,
 *       Kp/Kd 使用 DM 协议默认范围。
 */
void mit_ctrl(DMMotorInstance *motor) //
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = float_to_uint(motor->ctrl.pos_set, -motor->motor_settings.control_range.P_max, motor->motor_settings.control_range.P_max, 16);
    vel_tmp = float_to_uint(motor->ctrl.vel_set, -motor->motor_settings.control_range.V_max, motor->motor_settings.control_range.V_max, 12);
    kp_tmp  = float_to_uint(motor->ctrl.kp_set, KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(motor->ctrl.kd_set, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(motor->ctrl.tor_set, -motor->motor_settings.control_range.T_max, motor->motor_settings.control_range.T_max, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}

/**
 * @brief 查询 DM 电机是否在线且处于正常控制状态
 */
uint8_t DMMotorIsOnline(DMMotorInstance *motor)
{
    return motor != NULL && motor->online && DaemonIsOnline(motor->daemon);
}

// 异常检测
void DMMotorErrorDetection(DMMotorInstance *motor)
{
}

// DM电机软重启
//  void DMMotorReset(DMMotorInstance* motor){
//     DMMotorEnableMode(motor); // 使能电机模式,发送使能指令
//      CANTransmit_once(motor->motor_can_instance->can_handle,
//                           (motor->motor_can_instance->tx_id) + 0x0000,
//                           motor->motor_can_instance->tx_buff, 2);
//  }

void DMMotorStop(DMMotorInstance *motor)
{
    if (motor == NULL) {
        return;
    }
    motor->ctrl.pos_set = 0.0f;
    motor->ctrl.vel_set = 0.0f;
    motor->ctrl.tor_set = 0.0f;
    motor->ctrl.kp_set  = 0.0f;
    motor->ctrl.kd_set  = 0.0f;
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorEnable1(DMMotorInstance *motor)
{
    if (motor == NULL) {
        return;
    }
    motor->protocol_disable = 0u;
    motor->stop_flag = MOTOR_ENABLED;
}

void DMMotorDisable1(DMMotorInstance *motor)
{
    if (motor == NULL) {
        return;
    }
    motor->ctrl.pos_set = 0.0f;
    motor->ctrl.vel_set = 0.0f;
    motor->ctrl.tor_set = 0.0f;
    motor->ctrl.kp_set  = 0.0f;
    motor->ctrl.kd_set  = 0.0f;
    motor->protocol_disable = 1u;
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorSetTorque(DMMotorInstance *motor, float torque)
{
    if (motor == NULL) {
        return;
    }
    DMMotorEnsureProtocolRange(motor);
    motor->ctrl.pos_set = 0.0f;
    motor->ctrl.vel_set = 0.0f;
    motor->ctrl.kp_set  = 0.0f;
    motor->ctrl.kd_set  = 0.0f;
    motor->ctrl.tor_set = DMMotorLimit(torque, DMMotorAbs(motor->motor_settings.control_range.T_max));
}

void DMMotorSetMITPosition(DMMotorInstance *motor, float pos, float kp, float kd, float torque_ff)
{
    DMMotorSetMITPositionVelocity(motor, pos, 0.0f, kp, kd, torque_ff);
}

void DMMotorSetMITPositionVelocity(DMMotorInstance *motor, float pos, float vel, float kp, float kd, float torque_ff)
{
    if (motor == NULL) {
        return;
    }
    DMMotorEnsureProtocolRange(motor);
    motor->ctrl.pos_set = DMMotorLimit(pos, DMMotorAbs(motor->motor_settings.control_range.P_max));
    motor->ctrl.vel_set = DMMotorLimit(vel, DMMotorAbs(motor->motor_settings.control_range.V_max));
    motor->ctrl.kp_set  = DMMotorClampRange(kp, KP_MIN, KP_MAX);
    motor->ctrl.kd_set  = DMMotorClampRange(kd, KD_MIN, KD_MAX);
    motor->ctrl.tor_set = DMMotorLimit(torque_ff, DMMotorAbs(motor->motor_settings.control_range.T_max));
}

void DMMotorSetPos(DMMotorInstance *motor, float pos, float vel)
{
    if (motor == NULL) {
        return;
    }
    DMMotorEnsureProtocolRange(motor);
    motor->ctrl.pos_set = pos;
    motor->ctrl.vel_set = vel;
    motor->ctrl.tor_set = 0.0f;
    motor->ctrl.kp_set  = 0.0f;
    motor->ctrl.kd_set  = 0.0f;
}
