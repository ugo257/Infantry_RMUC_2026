#include "remote_control.h"
#include "robot_board.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#if defined(RC_VTM)
#include "crc_ref.h"
#endif

#if defined(RC_DR16)
#define REMOTE_CONTROL_FRAME_SIZE 18u // DR16/DBUS遥控器接收的buffer大小
#elif defined(RC_VTM)
#define REMOTE_CONTROL_FRAME_SIZE 21u // VTM遥控器接收的buffer大小
#define REMOTE_CONTROL_VTM_FRAME_HEAD1 0xA9u
#define REMOTE_CONTROL_VTM_FRAME_HEAD2 0x53u
#else
#error Missing RC definition! Define RC_DR16 or RC_VTM.
#endif

#define DR16_SW_UP   1u
#define DR16_SW_DOWN 2u
#define DR16_SW_MID  3u

// 遥控器数据
RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

extern volatile uint8_t rc_update_flag;

#if defined(RC_VTM)
static uint32_t RemoteControlGetBits(const uint8_t *buf, uint16_t bit_offset, uint8_t bit_len)
{
    uint32_t value = 0;

    // 手册中的偏移以bit为单位，且低bit先传；这里统一按小端位序取任意长度字段。
    for (uint8_t i = 0; i < bit_len; i++)
    {
        uint16_t bit = bit_offset + i;
        value |= (uint32_t)((buf[bit >> 3] >> (bit & 0x07u)) & 0x01u) << i;
    }

    return value;
}
#endif

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    int16_t *joysticks[] = {
        &rc_ctrl[TEMP].rc.rocker_l_,
        &rc_ctrl[TEMP].rc.rocker_l1,
        &rc_ctrl[TEMP].rc.rocker_r_,
        &rc_ctrl[TEMP].rc.rocker_r1,
        &rc_ctrl[TEMP].rc.dial,
    };

    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*joysticks[i]) > 660 || abs(*joysticks[i]) < 3)
            *joysticks[i] = 0;
}

static void RemoteControlUpdateKeyState(void)
{
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
}

static void RemoteControlFinishFrame(void)
{
    RemoteControlUpdateKeyState();
    rc_update_flag = 1;
    DaemonReload(rc_daemon_instance);
}

#if defined(RC_DR16)
static uint8_t RemoteControlDR16SwitchIsValid(uint8_t sw)
{
    return (sw == DR16_SW_UP || sw == DR16_SW_MID || sw == DR16_SW_DOWN) ? 1u : 0u;
}

static uint8_t RemoteControlDR16ChannelIsValid(uint16_t channel)
{
    return (channel >= RC_CH_VALUE_MIN && channel <= RC_CH_VALUE_MAX) ? 1u : 0u;
}

static uint8_t RemoteControlDR16SwitchToMode(uint8_t switch_left, uint8_t switch_right)
{
    if (switch_left == DR16_SW_DOWN && switch_right == DR16_SW_DOWN)
        return RC_SW_S;
    if (switch_left == DR16_SW_UP)
        return RC_SW_C;
    return RC_SW_N;
}

static uint8_t RemoteControlParseDR16(const uint8_t *sbus_buf)
{
    uint16_t ch0 = ((uint16_t)sbus_buf[0] | ((uint16_t)sbus_buf[1] << 8)) & 0x07ffu;
    uint16_t ch1 = (((uint16_t)sbus_buf[1] >> 3) | ((uint16_t)sbus_buf[2] << 5)) & 0x07ffu;
    uint16_t ch2 = (((uint16_t)sbus_buf[2] >> 6) | ((uint16_t)sbus_buf[3] << 2) |
                    ((uint16_t)sbus_buf[4] << 10)) & 0x07ffu;
    uint16_t ch3 = (((uint16_t)sbus_buf[4] >> 1) | ((uint16_t)sbus_buf[5] << 7)) & 0x07ffu;
    uint8_t switch_left = (uint8_t)(((uint16_t)sbus_buf[5] >> 4) & 0x0003u);
    uint8_t switch_right = (uint8_t)((((uint16_t)sbus_buf[5] >> 4) & 0x000cu) >> 2);
    uint16_t wheel = ((uint16_t)sbus_buf[16] | ((uint16_t)sbus_buf[17] << 8)) & 0x07ffu;

    if (!RemoteControlDR16ChannelIsValid(ch0) || !RemoteControlDR16ChannelIsValid(ch1) ||
        !RemoteControlDR16ChannelIsValid(ch2) || !RemoteControlDR16ChannelIsValid(ch3) ||
        !RemoteControlDR16ChannelIsValid(wheel) ||
        !RemoteControlDR16SwitchIsValid(switch_left) || !RemoteControlDR16SwitchIsValid(switch_right))
        return 0;

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断

    // DR16/DBUS通道定义: ch0右水平, ch1右竖直, ch2左竖直, ch3左水平。
    rc_ctrl[TEMP].rc.rocker_r_ = (int16_t)ch0 - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.rocker_r1 = (int16_t)ch1 - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.rocker_l1 = (int16_t)ch2 - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.rocker_l_ = (int16_t)ch3 - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.dial = (int16_t)wheel - RC_CH_VALUE_OFFSET;

    RectifyRCjoystick();

    rc_ctrl[TEMP].rc.switch_mode = RemoteControlDR16SwitchToMode(switch_left, switch_right);
    rc_ctrl[TEMP].rc.pause = 0u;
    rc_ctrl[TEMP].rc.custom_left = 0u;
    rc_ctrl[TEMP].rc.custom_right = 0u;
    rc_ctrl[TEMP].rc.trigger = (rc_ctrl[TEMP].rc.dial > 330) ? 1u : 0u;

    rc_ctrl[TEMP].mouse.x = (int16_t)((uint16_t)sbus_buf[6] | ((uint16_t)sbus_buf[7] << 8));
    rc_ctrl[TEMP].mouse.y = (int16_t)((uint16_t)sbus_buf[8] | ((uint16_t)sbus_buf[9] << 8));
    rc_ctrl[TEMP].mouse.z = (int16_t)((uint16_t)sbus_buf[10] | ((uint16_t)sbus_buf[11] << 8));
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];
    rc_ctrl[TEMP].mouse.press_m = 0u;

    rc_ctrl[TEMP].key[KEY_PRESS].keys = (uint16_t)sbus_buf[14] | ((uint16_t)sbus_buf[15] << 8);

    RemoteControlFinishFrame();
    return 1;
}
#endif

#if defined(RC_VTM)
static uint8_t RemoteControlVTMFrameIsValid(const uint8_t *sbus_buf)
{
    // 帧头固定为0xA9 0x53；无效帧不喂狗，交给daemon判定遥控链路异常。
    if (sbus_buf[0] != REMOTE_CONTROL_VTM_FRAME_HEAD1 || sbus_buf[1] != REMOTE_CONTROL_VTM_FRAME_HEAD2)
        return 0;

    return (Verify_CRC16_Check_Sum((uint8_t *)sbus_buf, REMOTE_CONTROL_FRAME_SIZE) == TRUE) ? 1u : 0u;
}

static uint8_t RemoteControlParseVTM(const uint8_t *sbus_buf)
{
    // 只有完整有效帧才更新TEMP/LAST，避免坏帧污染边沿判断和控制状态。
    if (!RemoteControlVTMFrameIsValid(sbus_buf))
        return 0;

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断

    // VTM图传手册通道定义: ch0右水平, ch1右竖直, ch2左竖直, ch3左水平, 拨轮独立在bit65。
    rc_ctrl[TEMP].rc.rocker_r_ = (int16_t)RemoteControlGetBits(sbus_buf, 16, 11) - RC_CH_VALUE_OFFSET; //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (int16_t)RemoteControlGetBits(sbus_buf, 27, 11) - RC_CH_VALUE_OFFSET; //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l1 = (int16_t)RemoteControlGetBits(sbus_buf, 38, 11) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l_ = (int16_t)RemoteControlGetBits(sbus_buf, 49, 11) - RC_CH_VALUE_OFFSET; //!< Channel 3
    rc_ctrl[TEMP].rc.dial = (int16_t)RemoteControlGetBits(sbus_buf, 65, 11) - RC_CH_VALUE_OFFSET;
    
    RectifyRCjoystick();

    // C/N/S档位和遥控器上的离散按键紧跟在摇杆与拨轮字段之间。
    rc_ctrl[TEMP].rc.switch_mode = (uint8_t)RemoteControlGetBits(sbus_buf, 60, 2);
    rc_ctrl[TEMP].rc.pause = (uint8_t)RemoteControlGetBits(sbus_buf, 62, 1);
    rc_ctrl[TEMP].rc.custom_left = (uint8_t)RemoteControlGetBits(sbus_buf, 63, 1);
    rc_ctrl[TEMP].rc.custom_right = (uint8_t)RemoteControlGetBits(sbus_buf, 64, 1);
    rc_ctrl[TEMP].rc.trigger = (uint8_t)RemoteControlGetBits(sbus_buf, 76, 1);

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (int16_t)(uint16_t)RemoteControlGetBits(sbus_buf, 80, 16);
    rc_ctrl[TEMP].mouse.y = (int16_t)(uint16_t)RemoteControlGetBits(sbus_buf, 96, 16);
    rc_ctrl[TEMP].mouse.z = (int16_t)(uint16_t)RemoteControlGetBits(sbus_buf, 112, 16);
    rc_ctrl[TEMP].mouse.press_l = (uint8_t)(RemoteControlGetBits(sbus_buf, 128, 2) != 0u);
    rc_ctrl[TEMP].mouse.press_r = (uint8_t)(RemoteControlGetBits(sbus_buf, 130, 2) != 0u);
    rc_ctrl[TEMP].mouse.press_m = (uint8_t)(RemoteControlGetBits(sbus_buf, 132, 2) != 0u);

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    // 键盘bit顺序按手册: W,S,A,D,Shift,Ctrl,Q,E,R,F,G,Z,X,C,V,B。
    rc_ctrl[TEMP].key[KEY_PRESS].keys = (uint16_t)RemoteControlGetBits(sbus_buf, 136, 16);

    RemoteControlFinishFrame();
    return 1;
}
#endif

#if defined(RC_DR16)
static uint8_t sbus_to_rc(const uint8_t *sbus_buf)
{
    return RemoteControlParseDR16(sbus_buf);
}
#endif

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
#if defined(RC_DR16)
    if (USARTGetRecvLen(rc_usart_instance) != REMOTE_CONTROL_FRAME_SIZE)
        return;

    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
#elif defined(RC_VTM)
    if (USARTGetRecvLen(rc_usart_instance) != REMOTE_CONTROL_FRAME_SIZE)
        return;

    RemoteControlParseVTM(rc_usart_instance->recv_buff);
#endif
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);
    
    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 30, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14ms(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    // rc_daemon_instance->temp_count--;
    // if(rc_daemon_instance->temp_count <= 0){
    //     rc_daemon_instance->temp_count = 0;
    // }
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}
