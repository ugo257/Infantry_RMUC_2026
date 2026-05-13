#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include <stdint.h>

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

/**
 * @brief 麦轮力控开关接口（与 chassis_mode 解耦）
 * @param enable 0:关闭  非0:开启
 */
void RobotCMDSetMecanumForceCtrl(uint8_t enable);

/**
 * @brief 获取麦轮力控开关状态
 */
uint8_t RobotCMDGetMecanumForceCtrl(void);
extern  float cym1;

#endif // !ROBOT_CMD_H
