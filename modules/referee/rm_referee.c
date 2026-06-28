/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 254u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;        // 裁判系统守护进程
static referee_info_t referee_info;           // 裁判系统数据

static uint16_t RefereeMinU16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

static void RefereeCopyData(void *dst, uint16_t dst_len, const uint8_t *src, uint16_t frame_data_len, uint16_t protocol_len)
{
    uint16_t copy_len;

    if (dst == NULL || src == NULL || dst_len == 0u) {
        return;
    }

    memset(dst, 0, dst_len);
    copy_len = RefereeMinU16(RefereeMinU16(frame_data_len, protocol_len), dst_len);
    if (copy_len > 0u) {
        memcpy(dst, src, copy_len);
    }
}

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff, uint16_t buff_len)
{
    uint16_t judge_length = 0u; // 统计一帧数据长度
    uint16_t data_len;
    uint8_t *data_ptr;
    uint8_t parsed_frame = 0u;

    if (buff == NULL || buff_len < LEN_HEADER)      // 空数据包，则不作任何处理
        return;

    // 判断帧头数据(0)是否为0xA5
    if (buff[SOF] == REFEREE_SOF) {
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE) {
            // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
            memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
            data_len = referee_info.FrameHeader.DataLength;
            // 统计一帧数据长度(byte),用于CR16校验
            judge_length = data_len + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            if (judge_length > buff_len) {
                return;
            }
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE) {
                parsed_frame = 1u;
                // 2个8位拼成16位int
                referee_info.CmdID = ((uint16_t)buff[CMD_ID_Offset + 1u] << 8) | buff[CMD_ID_Offset];
                data_ptr = buff + DATA_Offset;
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (referee_info.CmdID) {
                    case ID_game_state: // 0x0001
                        RefereeCopyData(&referee_info.GameState, sizeof(referee_info.GameState), data_ptr, data_len, LEN_game_state);
                        break;
                    case ID_game_result: // 0x0002
                        RefereeCopyData(&referee_info.GameResult, sizeof(referee_info.GameResult), data_ptr, data_len, LEN_game_result);
                        break;
                    case ID_game_robot_survivors: // 0x0003
                        RefereeCopyData(&referee_info.GameRobotHP, sizeof(referee_info.GameRobotHP), data_ptr, data_len, LEN_game_robot_HP);
                        break;
                    case ID_event_data: // 0x0101
                        RefereeCopyData(&referee_info.EventData, sizeof(referee_info.EventData), data_ptr, data_len, LEN_event_data);
                        break;
                    case ID_referee_warning://0x104
                        RefereeCopyData(&referee_info.referee_warning, sizeof(referee_info.referee_warning), data_ptr, data_len, LEN_REF_warning);
                        break;
                    case ID_dart_info: // 0x0105
                        RefereeCopyData(&referee_info.DartInfo, sizeof(referee_info.DartInfo), data_ptr, data_len, LEN_dart_info);
                        break;

                    // case ID_supply_projectile_action: // 0x0102
                    //     memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
                    //     break;
                    case ID_game_robot_state: // 0x0201
                        RefereeCopyData(&referee_info.GameRobotState, sizeof(referee_info.GameRobotState), data_ptr, data_len, LEN_game_robot_state);
                        break;
                    case ID_power_heat_data: // 0x0202
                        RefereeCopyData(&referee_info.PowerHeatData, sizeof(referee_info.PowerHeatData), data_ptr, data_len, LEN_power_heat_data);
                        break;
                    case ID_game_robot_pos: // 0x0203
                        RefereeCopyData(&referee_info.GameRobotPos, sizeof(referee_info.GameRobotPos), data_ptr, data_len, LEN_game_robot_pos);
                        break;
                    case ID_buff_musk: // 0x0204
                        RefereeCopyData(&referee_info.BuffMusk, sizeof(referee_info.BuffMusk), data_ptr, data_len, LEN_buff_musk);
                        break;
                    case ID_aerial_robot_energy: // 0x0205, 2026常规表已不发送，保留旧协议兼容
                        RefereeCopyData(&referee_info.AerialRobotEnergy, sizeof(referee_info.AerialRobotEnergy), data_ptr, data_len, LEN_aerial_robot_energy);
                        break;

                    case ID_robot_hurt: // 0x0206
                        RefereeCopyData(&referee_info.RobotHurt, sizeof(referee_info.RobotHurt), data_ptr, data_len, LEN_robot_hurt);
                        break;
                    case ID_shoot_data: // 0x0207
                        RefereeCopyData(&referee_info.ShootData, sizeof(referee_info.ShootData), data_ptr, data_len, LEN_shoot_data);
                        break;
                    case ID_projectile_allowance: // 0x0208
                        RefereeCopyData(&referee_info.ProjectileAllowance, sizeof(referee_info.ProjectileAllowance), data_ptr, data_len, LEN_projectile_allowance);
                        break;
                    case ID_RFID_state: // 0x0209
                        RefereeCopyData(&referee_info.RFID_state, sizeof(referee_info.RFID_state), data_ptr, data_len, LEN_RFID_state);
                        break;
                    case ID_dart_client_cmd: // 0x020A
                        RefereeCopyData(&referee_info.DartClientCmd, sizeof(referee_info.DartClientCmd), data_ptr, data_len, LEN_dart_client_cmd);
                        break;
                    case ID_ground_robot_position: // 0x020B
                        RefereeCopyData(&referee_info.GroundRobotPosition, sizeof(referee_info.GroundRobotPosition), data_ptr, data_len, LEN_ground_robot_position);
                        break;
                    case ID_radar_mark_data: // 0x020C
                        RefereeCopyData(&referee_info.RadarMarkData, sizeof(referee_info.RadarMarkData), data_ptr, data_len, LEN_radar_mark_data);
                        break;
                    case ID_sentry_info: // 0x020D
                        RefereeCopyData(&referee_info.SentryInfo, sizeof(referee_info.SentryInfo), data_ptr, data_len, LEN_sentry_info);
                        break;
                    case ID_radar_info: // 0x020E
                        RefereeCopyData(&referee_info.RadarInfo, sizeof(referee_info.RadarInfo), data_ptr, data_len, LEN_radar_info);
                        break;
                    case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
                        RefereeCopyData(&referee_info.ReceiveData, sizeof(referee_info.ReceiveData), data_ptr, data_len, LEN_receive_data);
                        break;
                }
            } else {
                // 测试用断点
            }
        }
        // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
        if (parsed_frame != 0u && judge_length < buff_len && buff[judge_length] == REFEREE_SOF) { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
            JudgeReadData(buff + judge_length, buff_len - judge_length);
        }
    }
}
/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
    DaemonReload(referee_daemon); 
    JudgeReadData(referee_usart_instance->recv_buff, USARTGetRecvLen(referee_usart_instance));
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
    USARTServiceInit(referee_usart_instance);
    LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback   = RefereeRxCallback;
    conf.usart_handle      = referee_usart_handle;
    conf.recv_buff_size    = RE_RX_BUFFER_SIZE; // mx 255(u8)
    referee_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback     = RefereeLostCallback,
        .owner_id     = referee_usart_instance,
        .reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
    };
    referee_daemon = DaemonRegister(&daemon_conf);

    return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
    USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
    osDelay(35);
}
