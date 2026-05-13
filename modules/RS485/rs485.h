/**
 ******************************************************************************
 * @file    rs485.h
 * @author  Dai Zixuan
 * @version V1.0.0
 * @date    2026/5/2
 * @brief   RS485 instance interface based on USART and daemon service
 ******************************************************************************
 */

#ifndef RS485_H
#define RS485_H

#include "bsp_usart.h"
#include "daemon.h"
#include "tick_timer.h"

/*
 * RS485底层实例说明
 * =================
 *
 * rs485.c/h只负责把一个USART注册成RS485通信实例：
 * - 接收完成后把USART接收缓冲复制到RS485Instance::recv_buf。
 * - 置位data_ready，供旧接口通过RS485DataReady轮询处理。
 * - 重载daemon，避免通信正常时被判离线。
 *
 * 普通业务通常不用直接操作这些底层函数，而是使用rs485_cmd.h里的
 * RobotCMD485InitMaster/Slave、RobotCMD485SendItems、RobotCMD485RecvItems。
 *
 * 直接使用底层接口的最小流程：
 *
 *     RS485_Init_Config_s conf = {
 *         .usart_handle = &huart1,
 *         .recv_size = 64,
 *         .daemon_reload_count = 30,
 *     };
 *     RS485Instance *ins = RS485Init(&conf);
 *
 *     if (RS485DataReady(ins)) {
 *         uint8_t *buf = RS485GetRecvBuf(ins);
 *         uint16_t len = RS485GetRecvLen(ins);
 *         // 在任务上下文中解析buf[0..len)
 *     }
 *
 *     if (RS485IsTxReady(ins)) {
 *         RS485Send(ins, tx_buf, tx_len, USART_TRANSFER_DMA);
 *     }
 */

#define RS485_INSTANCE_MX_CNT 2
#define RS485_RECV_BUF_SIZE    256u
#define RS485_RECV_WDG_TIMEOUT_S 1.0f

typedef struct RS485Instance RS485Instance;
typedef void (*rs485_recv_callback_t)(RS485Instance *instance, void *arg);

struct RS485Instance
{
    void *comm_instance;    // 通信实例，当前为USARTInstance*
    DaemonInstance *daemon; // 守护进程
    uint8_t recv_size;      // 接收缓冲大小
    uint16_t recv_len;      // 最近一次实际接收长度
    uint8_t recv_buf[RS485_RECV_BUF_SIZE];  // 内部接收缓冲区
    volatile uint8_t data_ready; // 数据就绪标志
    DWTWatchdog_t recv_wdg; // 用DWT测量两次RS485接收之间的间隔
    volatile float recv_dt_s;  // 最近一次接收间隔，单位s
    volatile float recv_dt_ms; // 最近一次接收间隔，单位ms
    volatile float recv_freq_hz; // 根据recv_dt_s换算的接收频率
    volatile uint32_t recv_count; // 接收回调触发次数
    rs485_recv_callback_t recv_callback; // 接收完成后在串口回调上下文触发的业务钩子
    void *recv_callback_arg; // 业务钩子参数
};

typedef struct
{
    UART_HandleTypeDef *usart_handle; // 对应串口句柄
    uint8_t recv_size;                // 接收缓冲区大小
    uint16_t daemon_reload_count;     // 守护重载计数
    rs485_recv_callback_t recv_callback; // 可选：接收完成后立即触发
    void *recv_callback_arg;          // 可选：传给recv_callback
} RS485_Init_Config_s;

// 注册一个RS485实例。recv_size不要超过RS485Instance::recv_buf大小。
RS485Instance *RS485Init(RS485_Init_Config_s *config);

// 设置接收完成后的业务钩子；该钩子在USART接收回调上下文中执行。
void RS485SetRecvCallback(RS485Instance *instance, rs485_recv_callback_t callback, void *arg);

// 发送一段原始字节流；mode通常使用USART_TRANSFER_DMA，返回HAL发送状态。
HAL_StatusTypeDef RS485Send(RS485Instance *instance, uint8_t *send_buf, uint16_t tx_len, USART_TRANSFER_MODE mode);

// 查询并清除数据就绪标志。返回1表示可读取RS485GetRecvBuf/GetRecvLen。
uint8_t RS485DataReady(RS485Instance *instance);

// 获取最近一次接收数据的内部缓冲指针；下次接收会覆盖。
uint8_t *RS485GetRecvBuf(RS485Instance *instance);

// 获取最近一次实际接收长度。
uint16_t RS485GetRecvLen(RS485Instance *instance);

// 原子取走最近一次接收数据；无新数据时返回0，返回非0时data_ready已被清除。
uint16_t RS485FetchRecvData(RS485Instance *instance, uint8_t *dst, uint16_t max_len);

// 获取最近两次RS485接收之间的间隔，单位ms；第一次接收返回0。
float RS485GetRecvDtMs(RS485Instance *instance);

// 获取最近两次RS485接收之间的频率，单位Hz；第一次接收返回0。
float RS485GetRecvFreqHz(RS485Instance *instance);

// 获取RS485接收回调累计触发次数。
uint32_t RS485GetRecvCount(RS485Instance *instance);

// 查询USART发送状态，返回1表示可以开始下一次发送。
uint8_t RS485IsTxReady(RS485Instance *instance);

#endif // RS485_H
