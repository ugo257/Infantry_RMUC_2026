#ifndef TICK_TIMER_H
#define TICK_TIMER_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_dwt.h"

typedef struct {
    uint32_t dwt_cnt;
    float timeout_s;
    float last_dt_s;
    bool started;
} DWTWatchdog_t;

/**
 * @brief 初始化一个DWT软件看门狗实例
 * @param wdg 看门狗实例
 * @param timeout_s 超时时间(秒),当任务周期dt大于该值时判定超时
 */
void DWTWatchdogInit(DWTWatchdog_t *wdg, float timeout_s);

/**
 * @brief 更新(喂狗)并检查任务实时性
 * @param wdg 看门狗实例
 * @return bool true: 周期正常; false: 周期超时
 */
bool DWTWatchdogUpdate(DWTWatchdog_t *wdg);

/**
 * @brief 获取最近一次更新得到的dt(秒)
 * @param wdg 看门狗实例
 * @return float 最近周期dt(秒)
 */
float DWTWatchdogGetLastDt(const DWTWatchdog_t *wdg);

#endif // TICK_TIMER_H
