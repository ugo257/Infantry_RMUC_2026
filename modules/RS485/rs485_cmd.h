/**
 ******************************************************************************
 * @file    rs485_cmd.h
 * @author  Dai Zixuan
 * @version V1.0.0
 * @date    2026/5/2
 * @brief   Robot CMD RS485 item-frame API and signal table helpers
 ******************************************************************************
 */

#ifndef RS485_CMD_H
#define RS485_CMD_H

#include <stddef.h>
#include <stdint.h>

#include "rs485.h"
#include "unicomm.h"

/*
 * robot_cmd RS485分项帧使用说明
 * ============================
 *
 * 这一层用于CMD双板通信；底层RS485收发实例仍由rs485.c/h管理。
 * CMD业务结构体字段表不放在RS485模块里，应该放在robot_cmd.c等业务文件中：
 *
 *     static const RobotCMD485Signal_s robot_cmd_ctrl_signals[] = {
 *         ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, vx,          0xA1u, 13u),
 *         ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, yaw_angle,   0xA1u, 1u),
 *         ROBOT_CMD_485_SIGNAL(Chassis_Ctrl_Cmd_s_uart, gimbal_mode, 0xA1u, 13u),
 *     };
 *
 *     #define ROBOT_CMD_CTRL_SIGNAL_COUNT \
 *         ((uint8_t)(sizeof(robot_cmd_ctrl_signals) / sizeof(robot_cmd_ctrl_signals[0])))
 *
 * 信号表每一行含义：
 * 1. struct_type：字段所在结构体类型，例如Chassis_Ctrl_Cmd_s_uart。
 * 2. field：要发送/接收的字段名，必须真实存在于struct_type中。
 * 3. head0：单项帧第一帧头，用来区分控制表、上传表等方向；例如控制用0xA1，上传用0xB1。
 * 4. period_ticks：发送周期，单位是调用RobotCMD485SendItems的任务tick。
 *    例如CMD任务1000Hz时：1=1000Hz，2=500Hz，10=100Hz，20=50Hz，50=20Hz。
 *    填0表示这个字段暂不发送；接收端仍可解析表中对应字段。
 *
 * 第二帧头head1不需要手写：内部按表项顺序自动生成，第一项为0x01，第二项为0x02。
 * 因此协议双方的表项顺序必须完全一致；新增字段时优先追加到表尾，避免改变既有head1。
 *
 * 支持的字段类型由ROBOT_CMD_485_PAYLOAD_TYPE自动推导：
 * - float   -> UNICOMM_PAYLOAD_FLOAT，4字节小端
 * - int32_t -> UNICOMM_PAYLOAD_I32，4字节小端
 * - uint8_t / char -> UNICOMM_PAYLOAD_U8，1字节
 * 工程启用短枚举时，模式类enum通常按uint8_t处理。不支持的字段类型会在编译期报错。
 *
 * 典型调用方式：
 * - 云台板初始化主机：
 *     rs485_master_instance = RobotCMD485InitMaster(&huart1);
 * - 底盘板初始化从机：
 *     rs485_slaver_instance = RobotCMD485InitSlave(&huart1);
 * - 云台板周期发送控制项：
 *     RobotCMD485SendItems(rs485_master_instance,
 *                          robot_cmd_ctrl_signals,
 *                          ROBOT_CMD_CTRL_SIGNAL_COUNT,
 *                          &chassis_cmd_send_uart,
 *                          &ctrl_tx_state);
 * - 底盘板周期接收控制项：
 *     RobotCMD485RecvItems(rs485_slaver_instance,
 *                          robot_cmd_ctrl_signals,
 *                          ROBOT_CMD_CTRL_SIGNAL_COUNT,
 *                          &chassis_rs485_recv);
 * - 云台板周期接收上传项：
 *     RobotCMD485RecvItems(rs485_master_instance,
 *                          robot_cmd_upload_signals,
 *                          ROBOT_CMD_UPLOAD_SIGNAL_COUNT,
 *                          &chassis_fetch_data_uart);
 *
 * RobotCMD485TxState_s必须每个发送方向单独一份，不能控制表和上传表共用。
 * state在RobotCMDInit()中通过RobotCMD485TxStateInit()初始化发送节拍；
 * 周期任务里只负责按节拍打包并立即DMA发送。
 */

#define ROBOT_CMD_485_HOST_DAEMON_RELOAD 30u
#define ROBOT_CMD_485_ITEM_TX_BUF_LEN    255u
#define ROBOT_CMD_485_RECV_BUF_LEN       255u
#define ROBOT_CMD_485_MAX_SIGNAL_COUNT   63u

// 单个业务字段与RS485单项帧之间的映射关系。
typedef struct
{
    const char *name;                  // 字段名，仅用于调试/阅读
    uint8_t head0;                     // 第一帧头，业务方向标识
    uint16_t period_ticks;             // 周期tick；0表示调度发送时跳过
    UniCommPayloadType_e payload_type; // 字段载荷类型
    uint16_t field_offset;             // 字段在结构体中的偏移，由宏自动生成
    uint8_t field_size;                // 字段大小，由宏自动生成
} RobotCMD485Signal_s;

// C11 _Generic在编译期根据结构体字段类型选择payload类型；不支持的字段类型会直接编译报错。
// 工程启用短枚举时，模式类enum与uint8_t兼容，会自动按U8发送低8位。
#define ROBOT_CMD_485_PAYLOAD_TYPE(struct_type, field) \
    _Generic(((struct_type *)0)->field,                \
             float: UNICOMM_PAYLOAD_FLOAT,             \
             int32_t: UNICOMM_PAYLOAD_I32,             \
             char: UNICOMM_PAYLOAD_U8,                 \
             uint8_t: UNICOMM_PAYLOAD_U8)

#define ROBOT_CMD_485_SIGNAL(struct_type, field, head0_, period_ticks_)                     \
    {#field,                                                                                \
     (head0_),                                                                              \
     (period_ticks_),                                                                       \
     ROBOT_CMD_485_PAYLOAD_TYPE(struct_type, field),                                        \
     (uint16_t)offsetof(struct_type, field),                                                \
     (uint8_t)sizeof(((struct_type *)0)->field)}

typedef struct
{
    uint32_t tick;                                       // 当前发送调度tick
    uint32_t next_ticks[ROBOT_CMD_485_MAX_SIGNAL_COUNT]; // 每个字段下一次发送tick
    uint32_t tx_success_count;                           // DMA成功启动次数
    uint32_t tx_busy_skip_count;                         // 发送忙导致本tick未推进次数
    uint32_t tx_fail_count;                              // HAL拒绝发送次数
    uint8_t tx_buf[ROBOT_CMD_485_ITEM_TX_BUF_LEN];       // 本次打包缓冲
} RobotCMD485TxState_s;

// 初始化CMD RS485主机。主机一般在云台板上使用，主动周期发送控制项并接收底盘上传项。
RS485Instance *RobotCMD485InitMaster(UART_HandleTypeDef *usart_handle);

// 初始化CMD RS485从机。从机一般在底盘板上使用，接收主机控制项并在任务中解析。
RS485Instance *RobotCMD485InitSlave(UART_HandleTypeDef *usart_handle);

// 初始化发送状态和节拍表；应在RobotCMDInit()里为每个发送方向调用一次。
void RobotCMD485TxStateInit(RobotCMD485TxState_s *state,
                            const RobotCMD485Signal_s *signals,
                            uint8_t signal_count);

// 解析一段连续字节流中的所有已知单项帧，并写入dst_base指向的业务结构体。
// 返回成功应用到结构体字段的帧数量；src中允许夹杂未知头或残帧，解析器会逐字节滑动查找。
uint16_t RobotCMD485ApplyStream(const uint8_t *src,
                                uint16_t src_len,
                                const RobotCMD485Signal_s *signals,
                                uint8_t signal_count,
                                void *dst_base);

// 从RS485实例最近一次DMA接收数据中解析完整单项帧；返回本次更新的字段数。
uint16_t RobotCMD485RecvItems(RS485Instance *instance,
                              const RobotCMD485Signal_s *signals,
                              uint8_t signal_count,
                              void *dst_base);

// 按初始化好的节拍表打包本tick到期单项帧；仅DMA成功启动后推进发送节拍。
uint16_t RobotCMD485SendItems(RS485Instance *instance,
                              const RobotCMD485Signal_s *signals,
                              uint8_t signal_count,
                              const void *src_base,
                              RobotCMD485TxState_s *state);

// 兼容旧整包协议的发送接口；新代码优先使用SendItems/RecvItems分项帧。
uint16_t RobotCMD485SendCtrlFrame(RS485Instance *instance, const Chassis_Ctrl_Cmd_s_uart *data);
uint16_t RobotCMD485SendUploadFrame(RS485Instance *instance, const Chassis_Upload_Data_s_uart *data);

#endif // RS485_CMD_H
