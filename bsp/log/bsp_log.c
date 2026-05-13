#include "bsp_log.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#if BSP_LOG_USE_SEGGER
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#endif


void BSPLogInit()
{
#if BSP_LOG_USE_SEGGER
    SEGGER_RTT_Init();
#endif
}

int PrintLog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
#if BSP_LOG_USE_SEGGER
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
#else
    int n = vprintf(fmt, args);
#endif
    va_end(args);
    return n;
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}
