/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
#include "bsp_log.h"
#include "cmsis_os2.h"
#define USB_REFRESH_RECONNECT_DELAY_MS 500U

static uint8_t reg_flag = 0;       // 注册标识符，保证板子只注册一个USB实例，多了就报错
static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
static volatile uint8_t usb_refresh_active = 0U;
static volatile uint32_t usb_refresh_release_tick = 0U;
// 注意usb单个数据包(Full speed模式下)最大为64byte,超出可能会出现丢包情况

static uint32_t USBRefreshMsToTicks(uint32_t delay_ms)
{
    const uint32_t tick_freq = osKernelGetTickFreq();
    uint32_t delay_ticks;

    if (tick_freq == 0U) {
        return delay_ms;
    }

    delay_ticks = (uint32_t)(((uint64_t)delay_ms * tick_freq + 999ULL) / 1000ULL);
    return (delay_ticks == 0U) ? 1U : delay_ticks;
}

static void USBRefreshDriveDisconnect(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_11, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_11, GPIO_PIN_RESET);
}

static void USBRefreshRestoreConnect(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_11, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    if (reg_flag == 1) {
        while (1)
            // LOGERROR("[bsp_usb]You cant register more instance for usb.");
            (void)0;
    }
    // usb的软件复位(模拟拔插)在usbd_conf.c中的HAL_PCD_MspInit()中
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk); // 获取接收数据指针
    // usb的接收回调函数会在这里被设置,并将数据保存在bsp_usb_rx_buffer中
    LOGINFO("USB init success");
    reg_flag = 1;
    return bsp_usb_rx_buffer;
}

void USBTransmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_FS(buffer, len); // 发送
}

void USBRefresh()
{
    if (usb_refresh_active != 0U) {
        return;
    }

    // 重新枚举usb设备，先模拟断开，恢复动作由USBRefreshTick()非阻塞完成
    USBRefreshDriveDisconnect();
    usb_refresh_release_tick =
        osKernelGetTickCount() + USBRefreshMsToTicks(USB_REFRESH_RECONNECT_DELAY_MS);
    usb_refresh_active = 1U;
}

void USBRefreshTick(void)
{
    if (usb_refresh_active == 0U) {
        return;
    }

    if ((int32_t)(osKernelGetTickCount() - usb_refresh_release_tick) < 0) {
        return;
    }

    USBRefreshRestoreConnect();
    usb_refresh_active = 0U;
}
