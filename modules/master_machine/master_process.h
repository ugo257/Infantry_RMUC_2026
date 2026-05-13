#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "bsp_usb.h"
#include "daemon.h"
#include "vision.h" // 可以在不修改代码的情况下更新model层

#define Host_Instance_MX_CNT   5

typedef enum {
    HOST_USART = 0, // 串口通信
    HOST_VCP,       // 虚拟串口/USB通信
} host_comm_mode;

typedef struct HostInstance HostInstance;
typedef void (*HostRxDecodeCallback)(HostInstance *instance, const uint8_t *buf, uint16_t len);

struct HostInstance {
    void *comm_instance;      // 通信实例，会被转换为串口或USB通信实例
    host_comm_mode comm_mode; // 通信方式
    DaemonInstance *daemon;   // 守护进程
    uint8_t RECV_SIZE;        // 接收缓冲大小
    uint8_t recv_buf[256];    // 内部接收缓冲区
    volatile uint8_t data_ready;  // 数据就绪标志
    uint16_t last_recv_len;   // 最近一次接收长度
    HostRxDecodeCallback rx_decode_callback; // 可选：接收完成后立即解析
};

typedef struct {
    host_comm_mode comm_mode;         // 通信方式
    uint8_t RECV_SIZE;                // 接收缓冲区大小
    UART_HandleTypeDef *usart_handle; // 串口通信对应的handle
    HostRxDecodeCallback rx_decode_callback; // 可选：接收完成后立即解析
} HostInstanceConf;

/**
 * @brief 调用此函数初始化上位机
 *
 */
HostInstance *HostInit(HostInstanceConf *host_conf,uint8_t reload_count);
void HostSend(HostInstance *instance, uint8_t *send_buf, uint16_t tx_len);
uint8_t HostDataReady(HostInstance *instance);
uint8_t *HostGetRecvBuf(HostInstance *instance);
uint16_t HostGetRecvLen(HostInstance *instance);

#endif // !MASTER_PROCESS_H
