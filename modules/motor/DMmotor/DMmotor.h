#ifndef DMmotor_H
#define DMmotor_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define MIT_mode 0x000
#define DM_MOTOR_MX_CNT 4 // 最多允许4个DM电机使用多电机指令,挂载在一条总线上

#define Infantry_PMAX 3.141593f
#define Infantry_VMAX 30.0f
#define Infantry_TMAX 7.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

typedef struct
{
	uint8_t mode;    // 控制模式, 见 Motor_Control_Type_e
	float pos_set;  // MIT/位置模式位置给定, 单位随 DM 电机协议量程
	float vel_set;  // MIT/位置模式速度给定
	float tor_set;  // MIT 模式力矩给定
	float kp_set;   // MIT 模式位置刚度
	float kd_set;   // MIT 模式速度阻尼
}motor_ctrl_t;
typedef struct
{
	int id;          // 电机反馈 ID
	int state;       // DM 电机状态, state == 1 视为可正常控制
	int p_int;       // 协议原始位置值
	int v_int;       // 协议原始速度值
	int t_int;       // 协议原始力矩值
	int kp_int;
	int kd_int;
	float pos;       // 解算后单圈位置
	float vel;       // 解算后速度
	float tor;       // 解算后力矩
	float Kp;
	float Kd;
	float Tmos;      // MOS 温度
	float Tcoil;     // 线圈温度
	float last_pos;  // 上一帧单圈位置, 用于跨圈判断
	int total_round; // 累计圈数
	float total_pos; // 累计位置
}motor_fbpara_t;



typedef struct
{
    Motor_Type_e motor_type;        // 电机类型
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    CANInstance *motor_can_instance; // 电机CAN实例
    motor_ctrl_t ctrl;
    motor_ctrl_t cmd;
    motor_fbpara_t measure;            // 电机测量值
    Motor_Working_Type_e stop_flag; // 启停标志
    DaemonInstance *daemon; // 离线检测守护实例
    uint32_t feed_cnt;      // DWT 计时器计数, 用于计算反馈周期
    float dt;               // 两次反馈间隔
    uint8_t online;         // 最近一次反馈是否处于正常控制状态
    uint8_t protocol_disable; // 协议失能锁存: 持续发送 FF..FD, 防止自动重使能
} DMMotorInstance;

// 电机回传信息结构体

/**
 * @brief 初始化DM电机
 *
 * @param config 电机配置
 * @return DMMotorInstance* 返回实例指针
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
void DMMotorControl();
// Deprecated compatibility API: direct torque alias, not a PID reference.
void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorEnableMode(DMMotorInstance *motor);
void DMMotorDisableMode(DMMotorInstance *motor);
void DMMotorSetTorque(DMMotorInstance *motor, float torque); // 设置力矩
void DMMotorSetMITPosition(DMMotorInstance *motor, float pos, float kp, float kd, float torque_ff);
void DMMotorSetMITPositionVelocity(DMMotorInstance *motor, float pos, float vel, float kp, float kd, float torque_ff);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorEnable1(DMMotorInstance *motor);
void DMMotorDisable1(DMMotorInstance *motor);
void DMMotorSetPos(DMMotorInstance *motor, float pos, float vel);
uint8_t DMMotorIsOnline(DMMotorInstance *motor);
#endif // DMmotor_H
