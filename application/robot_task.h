/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "referee_init.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "user_lib.h"
#include "robot_params.h"

#include "bsp_log.h"
#include "led.h"
#include "buzzer.h"
#include "ins_task.h"
#include "referee_UI.h"

#include "robot_cmd.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "motor_task.h"
#include "omni_UI.h"
#include "tick_timer.h"
#include "yaw_jbc.h"
//rtos任务最大延迟时间，超过设定的时间重启
#define RTOS_TASK_CHECK_ENABLE    1
#define RTOS_WDG_TIMEOUT_1MS_TASK_S 0.002f
#define RTOS_WDG_TIMEOUT_UI_TASK_S  0.030f

static inline void RTOSTaskCheckLog(const char *task_name, float dt_ms)
{
#if RTOS_TASK_CHECK_ENABLE
    (void)task_name;
    (void)dt_ms;
    // char dt_str[16];
    // Float2Str(dt_str, dt_ms);
    // LOGERROR("[freeRTOS] %s Task timeout! dt = [%s]ms", task_name, dt_str);
#else
    (void)task_name;
    (void)dt_ms;
#endif
}

//任务对象由 CubeMX 创建; 本文件提供任务函数实现以覆盖 freertos.c 中的默认弱实现
__attribute__((noreturn)) void StartINSTASK(void *argument)
{
    static float ins_dt;
    static DWTWatchdog_t ins_wdg;
    DWTWatchdogInit(&ins_wdg, RTOS_WDG_TIMEOUT_1MS_TASK_S);
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;) {
        INS_EKF();
        ins_dt = 1000.0f * DWTWatchdogGetLastDt(&ins_wdg);
        if (!DWTWatchdogUpdate(&ins_wdg)) {
            
            RTOSTaskCheckLog("INS", ins_dt);
        }
        osDelay(1);
    }
}

__attribute__((noreturn)) void _RobotCMDTask(void *argument)
{
    static float cmd_dt;
    static DWTWatchdog_t cmd_wdg;
    DWTWatchdogInit(&cmd_wdg, RTOS_WDG_TIMEOUT_1MS_TASK_S);
    LOGINFO("[freeRTOS] RobotCMD Task Start");
    for (;;) {
        RobotCMDTask();
        cmd_dt = 1000.0f * DWTWatchdogGetLastDt(&cmd_wdg);
        if (!DWTWatchdogUpdate(&cmd_wdg)) {
            
            RTOSTaskCheckLog("RobotCMD", cmd_dt);
        }
        osDelay(1);
    }
}
__attribute__((noreturn)) void _ChassisTask(void *argument)
{
    static float chassis_dt;
    static DWTWatchdog_t chassis_wdg;
    DWTWatchdogInit(&chassis_wdg, RTOS_WDG_TIMEOUT_1MS_TASK_S);

    for (;;) {
        
#ifdef CHASSIS_BOARD
        ChassisTask();
        
#endif
        chassis_dt = 1000.0f * DWTWatchdogGetLastDt(&chassis_wdg);
        if (!DWTWatchdogUpdate(&chassis_wdg)) {
            
            RTOSTaskCheckLog("Chassis", chassis_dt);
        }
        osDelay(1);
    }
}
float Gimbal_dt, Shoot_dt, motorControl_dt, Daemon_dt,cal_dt;
__attribute__((noreturn)) void _GimbalTask(void *argument)
{
    static DWTWatchdog_t gimbal_wdg;
    DWTWatchdogInit(&gimbal_wdg, RTOS_WDG_TIMEOUT_1MS_TASK_S);

    for (;;) {
#if defined(GIMBAL_BOARD)
        GimbalTask_PITCH();
        // GimbalTask();
#elif defined(CHASSIS_BOARD)
        GimbalTask_YAW();
        
#endif
        Gimbal_dt = 1000.0f * DWTWatchdogGetLastDt(&gimbal_wdg);
        if (!DWTWatchdogUpdate(&gimbal_wdg)) {
            
            RTOSTaskCheckLog("Gimbal", Gimbal_dt);
        }
        osDelay(1);
    }
}
__attribute__((noreturn)) void _ShootTask(void *argument)
{
    static DWTWatchdog_t shoot_wdg;
    DWTWatchdogInit(&shoot_wdg, RTOS_WDG_TIMEOUT_1MS_TASK_S);
    for (;;) {
#if defined(GIMBAL_BOARD)
        ShootTask_Gimbal();
#elif defined(CHASSIS_BOARD)
        ShootTask_Chassis();
#endif
    Shoot_dt = 1000.0f * DWTWatchdogGetLastDt(&shoot_wdg);
        if (!DWTWatchdogUpdate(&shoot_wdg)) {
            
            RTOSTaskCheckLog("Shoot", Shoot_dt);
        }
        osDelay(1);
    }
}
__attribute__((noreturn)) void motorControlTask(void *argument)
{
    static DWTWatchdog_t motor_wdg;
    DWTWatchdogInit(&motor_wdg, RTOS_WDG_TIMEOUT_1MS_TASK_S);

    for (;;) {
        MotorControlTask();
        motorControl_dt = 1000.0f * DWTWatchdogGetLastDt(&motor_wdg);
        if (!DWTWatchdogUpdate(&motor_wdg)) {
            
            RTOSTaskCheckLog("Motor", motorControl_dt);
        }
        osDelay(1);
    }
}
__attribute__((noreturn)) void _UITask(void *argument)
{
    static float ui_dt;
    static DWTWatchdog_t ui_wdg;
    DWTWatchdogInit(&ui_wdg, RTOS_WDG_TIMEOUT_UI_TASK_S);

    for (;;) {
#ifdef CHASSIS_BOARD
        UIDynamicRefresh();
        ui_dt = 1000.0f * DWTWatchdogGetLastDt(&ui_wdg);
#endif
        
        if (!DWTWatchdogUpdate(&ui_wdg)) {
            
            RTOSTaskCheckLog("UI", ui_dt);
        }
        osDelay(10);
    }
}
__attribute__((noreturn)) void _DaemonTask(void *argument)
{
    static DWTWatchdog_t daemon_wdg;
    DWTWatchdogInit(&daemon_wdg, RTOS_WDG_TIMEOUT_1MS_TASK_S);

    for (;;) {
        DaemonTask();
        Daemon_dt = 1000.0f * DWTWatchdogGetLastDt(&daemon_wdg);
        if (!DWTWatchdogUpdate(&daemon_wdg)) {
            
            RTOSTaskCheckLog("Daemon", Daemon_dt);
        }
        osDelay(1);
    }
}
__attribute__((noreturn)) void _CalTask(void *argument)
{
    static DWTWatchdog_t cal_wdg;
    DWTWatchdogInit(&cal_wdg, RTOS_WDG_TIMEOUT_1MS_TASK_S);

    for (;;) {
        Cal_JBCTask();
        cal_dt = 1000.0f * DWTWatchdogGetLastDt(&cal_wdg);
        if (!DWTWatchdogUpdate(&cal_wdg)) {
            RTOSTaskCheckLog("Cal_JBC", cal_dt);
        }
        osDelay(1);
    }
}
