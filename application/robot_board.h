#pragma once
#ifndef ROBOT_BOARD_H
#define ROBOT_BOARD_H

/* Board selection: keep exactly one of the following enabled */
// 双板定义修改
#define CHASSIS_BOARD //ip为192.168.1.12
// #define GIMBAL_BOARD //ip为192.168.1.14

/* Board macro check */
#if defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD)
#error Conflict board definition! You can only define one board type.
#endif

#if !defined(CHASSIS_BOARD) && !defined(GIMBAL_BOARD)
#error Missing board definition! Define CHASSIS_BOARD or GIMBAL_BOARD.
#endif

/* Remote controller selection: keep exactly one of the following enabled */
#define RC_DR16 //旧遥控器
// #define RC_VTM //新图传遥控器

/* Remote controller macro check */
#if defined(RC_DR16) && defined(RC_VTM)
#error Conflict RC definition! You can only define one RC type.
#endif

#if !defined(RC_DR16) && !defined(RC_VTM)
#error Missing RC definition! Define RC_DR16 or RC_VTM.
#endif

/* Feature selection */
#if defined(CHASSIS_BOARD)
#if defined(RC_DR16)
#define CHASSIS_DR16_DIRECT_CONTROL
#endif
#elif defined(GIMBAL_BOARD)
#define VISION_USE_VCP
// #define VISION_USE_UART
#endif

#endif // ROBOT_BOARD_H
