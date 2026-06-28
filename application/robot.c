/**
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-26 21:51:44
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\application\robot.c
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#include "bsp_init.h"
#include "robot.h"
#include "robot_board.h"
#include "robot_task.h"
#include "buzzer.h"
#include "robot_test.h"
#include "vision.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#include "yaw_jbc.h"

void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    buzzer_init();//蜂鸣器初始化
    BSPInit();
    RobotCMDInit();
    YawJbc_LoadFromFlash();
    GimbalInit();
    ShootInit();
#ifdef GIMBAL_BOARD
    HAL_TIM_Base_Start_IT(&htim6);//Tim6定时器，用来定时检测枪口热量
    //buzzer_one_note(Re_freq, 0.1f);
    //buzzer_one_note(Mi_freq, 0.1f);
    C_board_LEDSet(0x00FF00);
    //buzzer_one_note(Fa_freq, 0.1f);
#endif

#ifdef CHASSIS_BOARD
    ChassisInit();
    UI_Init();
    //buzzer_one_note(So_freq, 0.1f);
    C_board_LEDSet(0x0000FF);
#endif
    // // 初始化完成,开启中断
     __enable_irq();
}
