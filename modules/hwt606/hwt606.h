#ifndef __HWT606_H
#define __HWT606_H
#include "robot_types.h"
#include "usart.h"
#include "memory.h"
#define HWT606_recv_size 16u
typedef struct{
    short angle[3];
    short accel[3];
    short gyro[3];
    float vx;
    float vy;
    float wz;
    chassis_mode_e chassis_mode;
    uint32_t DWT_CNT;
    float dt;
    uint8_t init_flag;
}hwt606_info_t;
hwt606_info_t *HWT606_HardwareInit(UART_HandleTypeDef *hwt606_usart_handle);
hwt606_info_t *HWT606_Init(UART_HandleTypeDef *hwt606_usart_handle);
#endif // !__HWT606_H
