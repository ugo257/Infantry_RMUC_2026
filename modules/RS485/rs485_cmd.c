/**
 ******************************************************************************
 * @file    rs485_cmd.c
 * @author  Dai Zixuan
 * @version V1.0.0
 * @date    2026/5/2
 * @brief   Robot CMD RS485 item-frame packing, parsing, and scheduling
 ******************************************************************************
 */

#include "rs485_cmd.h"

#include <string.h>

static uint8_t robot_cmd_485_recv_buf[ROBOT_CMD_485_RECV_BUF_LEN];

static const void *RobotCMD485FieldPtrConst(const void *base, const RobotCMD485Signal_s *signal)
{
    return (const void *)((const uint8_t *)base + signal->field_offset);
}

static void *RobotCMD485FieldPtr(void *base, const RobotCMD485Signal_s *signal)
{
    return (void *)((uint8_t *)base + signal->field_offset);
}

static int32_t RobotCMD485BytesToI32(const uint8_t *bytes)
{
    uint32_t raw = 0u;
    int32_t value = 0;

    raw |= (uint32_t)bytes[0];
    raw |= (uint32_t)bytes[1] << 8;
    raw |= (uint32_t)bytes[2] << 16;
    raw |= (uint32_t)bytes[3] << 24;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static uint8_t RobotCMD485SignalHead1(uint8_t signal_index)
{
    // head1固定由表项索引生成，因此协议双方必须保持信号表顺序一致。
    return (uint8_t)(signal_index + 1u);
}

static uint16_t RobotCMD485PackSignal(const void *src_base,
                                      const RobotCMD485Signal_s *signal,
                                      uint8_t signal_index,
                                      uint8_t *dst,
                                      uint16_t dst_len)
{
    const void *field = RobotCMD485FieldPtrConst(src_base, signal);
    uint8_t u8_value = 0u;

    // 模式类字段可能是短枚举或char/uint8_t，统一只发送低8位。
    if (signal->payload_type == UNICOMM_PAYLOAD_U8) {
        u8_value = *(const uint8_t *)field;
        field = &u8_value;
    }

    return UniCommPackItemFrame(signal->head0, RobotCMD485SignalHead1(signal_index), field, signal->payload_type, dst, dst_len);
}

static const RobotCMD485Signal_s *RobotCMD485FindSignalByHead(const RobotCMD485Signal_s *signals,
                                                              uint8_t signal_count,
                                                              uint8_t head0,
                                                              uint8_t head1)
{
    for (uint8_t i = 0u; i < signal_count; ++i) {
        if (signals[i].head0 == head0 && RobotCMD485SignalHead1(i) == head1) {
            return &signals[i];
        }
    }

    return NULL;
}

static void RobotCMD485ApplySignal(void *dst_base, const RobotCMD485Signal_s *signal, const UniCommItemFrame_s *item)
{
    uint8_t *field = (uint8_t *)RobotCMD485FieldPtr(dst_base, signal);

    // 接收侧按信号表记录的字段偏移直接写回业务结构体。
    // U8字段写入前先清零整个字段大小，兼容短枚举底层类型大于1字节的情况。
    switch (signal->payload_type) {
        case UNICOMM_PAYLOAD_FLOAT: {
            if (signal->field_size >= (uint8_t)sizeof(float)) {
                float value = UniCommBytesToFloat(item->payload);
                memcpy(field, &value, sizeof(value));
            }
            break;
        }
        case UNICOMM_PAYLOAD_I32: {
            if (signal->field_size >= (uint8_t)sizeof(int32_t)) {
                int32_t value = RobotCMD485BytesToI32(item->payload);
                memcpy(field, &value, sizeof(value));
            }
            break;
        }
        case UNICOMM_PAYLOAD_U8:
            if (signal->field_size > 0u) {
                memset(field, 0, signal->field_size);
                field[0] = item->payload[0];
            }
            break;
        default:
            break;
    }
}

uint16_t RobotCMD485ApplyStream(const uint8_t *src,
                                uint16_t src_len,
                                const RobotCMD485Signal_s *signals,
                                uint8_t signal_count,
                                void *dst_base)
{
    if (src == NULL || signals == NULL || dst_base == NULL || src_len < UNICOMM_ITEM_FRAME_MIN_LEN) {
        return 0u;
    }

    // 接收缓冲可能从任意位置开始，也可能夹杂残帧/噪声。
    // 找到合法(head0, head1)后按该信号的payload长度尝试解包；
    // 解包失败则滑动1字节继续找，避免单个坏帧阻塞后续有效帧。
    uint16_t applied = 0u;
    for (uint16_t i = 0u; (uint16_t)(i + UNICOMM_ITEM_FRAME_MIN_LEN) <= src_len;) {
        const RobotCMD485Signal_s *signal = RobotCMD485FindSignalByHead(signals, signal_count, src[i], src[i + 1u]);
        if (signal != NULL) {
            uint16_t frame_len = UniCommItemFrameLen(signal->payload_type);
            UniCommItemFrame_s item;

            if (frame_len > 0u &&
                (uint16_t)(i + frame_len) <= src_len &&
                UniCommUnpackItemFrame(src + i, (uint16_t)(src_len - i), signal->payload_type, &item)) {
                RobotCMD485ApplySignal(dst_base, signal, &item);
                ++applied;
                i = (uint16_t)(i + frame_len);
                continue;
            }
        }
        ++i;
    }

    return applied;
}

static void RobotCMD485InitNextTicks(uint32_t *next_ticks, const RobotCMD485Signal_s *signals, uint8_t signal_count)
{
    // 初始化每个信号的第一次发送tick。
    // 同周期的多个字段会被均匀摊开，避免某一个周期点同时塞入太多单项帧。
    // 例如period=10且有5个字段，初始tick大致为0、2、4、6、8。
    for (uint8_t i = 0u; i < signal_count; ++i) {
        uint16_t period = signals[i].period_ticks;
        uint8_t same_period_count = 0u;
        uint8_t same_period_index = 0u;

        if (period == 0u) {
            next_ticks[i] = 0u;
            continue;
        }

        for (uint8_t j = 0u; j < signal_count; ++j) {
            if (signals[j].period_ticks == period) {
                if (j < i) {
                    ++same_period_index;
                }
                ++same_period_count;
            }
        }

        if (period == 1u || same_period_count == 1u) {
            next_ticks[i] = 0u;
        } else if (same_period_count <= period) {
            next_ticks[i] = (uint32_t)(((float)same_period_index * (float)period /
                                        (float)same_period_count) + 0.5f);
        } else {
            next_ticks[i] = same_period_index % period;
        }
    }
}

RS485Instance *RobotCMD485InitMaster(UART_HandleTypeDef *usart_handle)
{
    // 主机不需要接收完成后立即回包，因此不设置recv_callback。
    RS485_Init_Config_s rs485_conf = {
        .usart_handle = usart_handle,
        .recv_size = ROBOT_CMD_485_ITEM_TX_BUF_LEN,
        .daemon_reload_count = ROBOT_CMD_485_HOST_DAEMON_RELOAD,
    };

    return RS485Init(&rs485_conf);
}

RS485Instance *RobotCMD485InitSlave(UART_HandleTypeDef *usart_handle)
{
    RS485_Init_Config_s rs485_conf = {
        .usart_handle = usart_handle,
        .recv_size = ROBOT_CMD_485_ITEM_TX_BUF_LEN,
        .daemon_reload_count = ROBOT_CMD_485_HOST_DAEMON_RELOAD,
    };

    return RS485Init(&rs485_conf);
}

void RobotCMD485TxStateInit(RobotCMD485TxState_s *state,
                            const RobotCMD485Signal_s *signals,
                            uint8_t signal_count)
{
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
    if (signals == NULL || signal_count == 0u || signal_count > ROBOT_CMD_485_MAX_SIGNAL_COUNT) {
        return;
    }

    RobotCMD485InitNextTicks(state->next_ticks, signals, signal_count);
}

uint16_t RobotCMD485RecvItems(RS485Instance *instance,
                              const RobotCMD485Signal_s *signals,
                              uint8_t signal_count,
                              void *dst_base)
{
    if (instance == NULL || signals == NULL || dst_base == NULL || signal_count == 0u) {
        return 0u;
    }

    uint16_t recv_len = RS485FetchRecvData(instance, robot_cmd_485_recv_buf, sizeof(robot_cmd_485_recv_buf));
    if (recv_len < UNICOMM_ITEM_FRAME_MIN_LEN) {
        return 0u;
    }

    return RobotCMD485ApplyStream(robot_cmd_485_recv_buf, recv_len, signals, signal_count, dst_base);
}

uint16_t RobotCMD485SendItems(RS485Instance *instance,
                              const RobotCMD485Signal_s *signals,
                              uint8_t signal_count,
                              const void *src_base,
                              RobotCMD485TxState_s *state)
{
    if (instance == NULL || signals == NULL || src_base == NULL || state == NULL || signal_count == 0u) {
        return 0u;
    }

    if (!RS485IsTxReady(instance)) {
        ++state->tx_busy_skip_count;
        return 0u;
    }

    uint16_t tx_len = 0u;
    uint8_t packed_flags[ROBOT_CMD_485_MAX_SIGNAL_COUNT] = {0u};
    for (uint8_t i = 0u; i < signal_count; ++i) {
        if (signals[i].period_ticks == 0u || state->tick < state->next_ticks[i]) {
            continue;
        }

        uint16_t frame_len = UniCommItemFrameLen(signals[i].payload_type);
        if (frame_len == 0u) {
            continue;
        }
        if ((uint16_t)(tx_len + frame_len) > sizeof(state->tx_buf)) {
            break;
        }

        uint16_t packed_len = RobotCMD485PackSignal(src_base,
                                                    &signals[i],
                                                    i,
                                                    state->tx_buf + tx_len,
                                                    (uint16_t)(sizeof(state->tx_buf) - tx_len));
        if (packed_len == frame_len) {
            tx_len = (uint16_t)(tx_len + packed_len);
            packed_flags[i] = 1u;
        }
    }

    if (tx_len > 0u) {
        static uint8_t debug_tx_buf[ROBOT_CMD_485_ITEM_TX_BUF_LEN];
        HAL_StatusTypeDef tx_status = RS485Send(instance, state->tx_buf, tx_len, USART_TRANSFER_DMA);
        memcpy(debug_tx_buf, state->tx_buf, tx_len);
        if (tx_status != HAL_OK) {
            if (tx_status == HAL_BUSY) {
                ++state->tx_busy_skip_count;
            } else {
                ++state->tx_fail_count;
            }
            return 0u;
        }

        ++state->tx_success_count;
        for (uint8_t i = 0u; i < signal_count; ++i) {
            if (packed_flags[i] != 0u) {
                while (state->next_ticks[i] <= state->tick) {
                    state->next_ticks[i] += signals[i].period_ticks;
                }
            }
        }
    }

    ++state->tick;
    return tx_len;
}

uint16_t RobotCMD485SendCtrlFrame(RS485Instance *instance, const Chassis_Ctrl_Cmd_s_uart *data)
{
    static uint8_t master_tx_buf[UNICOMM_CTRL_FRAME_LEN];

    if (instance != NULL && data != NULL &&
        RS485IsTxReady(instance) &&
        UniCommPackChassisCtrl(data, master_tx_buf, sizeof(master_tx_buf)) == UNICOMM_CTRL_FRAME_LEN) {
        RS485Send(instance, master_tx_buf, UNICOMM_CTRL_FRAME_LEN, USART_TRANSFER_DMA);
        return UNICOMM_CTRL_FRAME_LEN;
    }

    return 0u;
}

uint16_t RobotCMD485SendUploadFrame(RS485Instance *instance, const Chassis_Upload_Data_s_uart *data)
{
    static uint8_t slaver_tx_buf[UNICOMM_UPLOAD_FRAME_LEN];

    if (instance != NULL && data != NULL &&
        RS485IsTxReady(instance) &&
        UniCommPackChassisUpload(data, slaver_tx_buf, sizeof(slaver_tx_buf)) == UNICOMM_UPLOAD_FRAME_LEN) {
        RS485Send(instance, slaver_tx_buf, UNICOMM_UPLOAD_FRAME_LEN, USART_TRANSFER_DMA);
        return UNICOMM_UPLOAD_FRAME_LEN;
    }

    return 0u;
}
