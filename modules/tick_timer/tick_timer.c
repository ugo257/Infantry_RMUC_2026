#include "tick_timer.h"

void DWTWatchdogInit(DWTWatchdog_t *wdg, float timeout_s) {
    if (wdg == NULL) {
        return;
    }

    wdg->dwt_cnt = 0u;
    wdg->last_dt_s = 0.0f;
    wdg->started = false;
    wdg->timeout_s = (timeout_s > 0.0f) ? timeout_s : 0.001f;
}

bool DWTWatchdogUpdate(DWTWatchdog_t *wdg) {
    if (wdg == NULL) {
        return false;
    }

    if (!wdg->started) {
        /* Prime DWT timestamp and ignore the first dt sample. */
        (void)DWT_GetDeltaT(&wdg->dwt_cnt);
        wdg->last_dt_s = 0.0f;
        wdg->started = true;
        return true;
    }

    wdg->last_dt_s = DWT_GetDeltaT(&wdg->dwt_cnt);
    return (wdg->last_dt_s <= wdg->timeout_s);
}

float DWTWatchdogGetLastDt(const DWTWatchdog_t *wdg) {
    if (wdg == NULL) {
        return 0.0f;
    }
    return wdg->last_dt_s;
}
