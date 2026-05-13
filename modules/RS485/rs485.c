/**
 ******************************************************************************
 * @file    rs485.c
 * @author  Dai Zixuan
 * @version V1.0.0
 * @date    2026/5/2
 * @brief   RS485 instance registration, receive snapshot, and raw transmit
 ******************************************************************************
 */

#include "rs485.h"
#include "bsp_log.h"
#include <stdlib.h>
#include <string.h>
#include "robot_board.h"
#include "iwdg.h"

static uint8_t idx = 0;
static RS485Instance *rs485_instance[RS485_INSTANCE_MX_CNT];

static void RS485UpdateRecvTiming(RS485Instance *instance)
{
	if (instance == NULL) {
		return;
	}

	(void)DWTWatchdogUpdate(&instance->recv_wdg);
	instance->recv_dt_s = DWTWatchdogGetLastDt(&instance->recv_wdg);
	instance->recv_dt_ms = 1000.0f * instance->recv_dt_s;
	instance->recv_freq_hz = (instance->recv_dt_s > 0.0f) ? (1.0f / instance->recv_dt_s) : 0.0f;
	++instance->recv_count;
}

static void RS485OfflineCallback(void *instance)
{
	RS485Instance *_instance = (RS485Instance *)instance;
	USARTServiceInit((USARTInstance *)_instance->comm_instance);
}

static void RS485RecvCallbackByIndex(uint8_t instance_idx)
{
	RS485Instance *_instance = rs485_instance[instance_idx];
	USARTInstance *usart = (USARTInstance *)_instance->comm_instance;
	uint16_t recv_len = USARTGetRecvLen(usart);
	if (recv_len > _instance->recv_size) {
		recv_len = _instance->recv_size;
	}
	if (recv_len > sizeof(_instance->recv_buf)) {
		recv_len = sizeof(_instance->recv_buf);
	}

	DaemonReload(_instance->daemon);
	RS485UpdateRecvTiming(_instance);
#ifdef CHASSIS_BOARD
	HAL_IWDG_Refresh(&hiwdg);
#endif
	if (recv_len > 0u) {
		memcpy(_instance->recv_buf, usart->recv_buff, recv_len);
	}
	_instance->recv_len = recv_len;
	_instance->data_ready = 1;

	rs485_recv_callback_t callback = _instance->recv_callback;
	if (callback != NULL) {
		callback(_instance, _instance->recv_callback_arg);
	}
}

// 内置接收回调: 按实例索引分别处理
static void RS485RecvCallback0(void)
{
	RS485RecvCallbackByIndex(0u);
}

static void RS485RecvCallback1(void)
{
	RS485RecvCallbackByIndex(1u);
}

static usart_module_callback rs485_recv_callbacks[RS485_INSTANCE_MX_CNT] = {
	RS485RecvCallback0,
	RS485RecvCallback1,
};

RS485Instance *RS485Init(RS485_Init_Config_s *config)
{
	if (idx >= RS485_INSTANCE_MX_CNT) {
		while (1)
			// LOGERROR("[rs485] You cant register more RS485 instance.");
			(void)0;
	}
	uint8_t instance_idx = idx;

	RS485Instance *_instance = (RS485Instance *)malloc(sizeof(RS485Instance));
	memset(_instance, 0, sizeof(RS485Instance));
	_instance->recv_size = config->recv_size;
	_instance->recv_callback = config->recv_callback;
	_instance->recv_callback_arg = config->recv_callback_arg;
	DWTWatchdogInit(&_instance->recv_wdg, RS485_RECV_WDG_TIMEOUT_S);
	rs485_instance[instance_idx] = _instance;

	USART_Init_Config_s usart_conf = {
		.recv_buff_size  = config->recv_size,
		.usart_handle    = config->usart_handle,
		.module_callback = rs485_recv_callbacks[instance_idx],
	};
	_instance->comm_instance = USARTRegister(&usart_conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback     = RS485OfflineCallback,
		.owner_id     = _instance,
		.reload_count = config->daemon_reload_count,
	};
	_instance->daemon = DaemonRegister(&daemon_conf);

	++idx;
	return _instance;
}

void RS485SetRecvCallback(RS485Instance *instance, rs485_recv_callback_t callback, void *arg)
{
	if (instance == NULL) {
		return;
	}

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	instance->recv_callback = callback;
	instance->recv_callback_arg = arg;
	if (primask == 0u) {
		__enable_irq();
	}
}

HAL_StatusTypeDef RS485Send(RS485Instance *instance, uint8_t *send_buf, uint16_t tx_len, USART_TRANSFER_MODE mode)
{
	if (instance == NULL || send_buf == NULL || tx_len == 0u)
		return HAL_ERROR;

	return USARTSend((USARTInstance *)instance->comm_instance, send_buf, tx_len, mode);
}

uint8_t RS485DataReady(RS485Instance *instance)
{
	if (instance == NULL)
		return 0u;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (instance->data_ready) {
		instance->data_ready = 0;
		if (primask == 0u) {
			__enable_irq();
		}
		return 1;
	}
	if (primask == 0u) {
		__enable_irq();
	}
	return 0;
}

uint8_t *RS485GetRecvBuf(RS485Instance *instance)
{
	return instance->recv_buf;
}

uint16_t RS485GetRecvLen(RS485Instance *instance)
{
	if (instance == NULL)
		return 0u;

	return instance->recv_len;
}

uint16_t RS485FetchRecvData(RS485Instance *instance, uint8_t *dst, uint16_t max_len)
{
	if (instance == NULL || dst == NULL || max_len == 0u)
		return 0u;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	if (instance->data_ready == 0u) {
		if (primask == 0u) {
			__enable_irq();
		}
		return 0u;
	}

	uint16_t copy_len = instance->recv_len;
	if (copy_len > max_len) {
		copy_len = max_len;
	}
	if (copy_len > 0u) {
		memcpy(dst, instance->recv_buf, copy_len);
	}
	instance->data_ready = 0u;

	if (primask == 0u) {
		__enable_irq();
	}
	return copy_len;
}

float RS485GetRecvDtMs(RS485Instance *instance)
{
	if (instance == NULL)
		return 0.0f;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	float dt_ms = instance->recv_dt_ms;
	if (primask == 0u) {
		__enable_irq();
	}
	return dt_ms;
}

float RS485GetRecvFreqHz(RS485Instance *instance)
{
	if (instance == NULL)
		return 0.0f;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	float freq_hz = instance->recv_freq_hz;
	if (primask == 0u) {
		__enable_irq();
	}
	return freq_hz;
}

uint32_t RS485GetRecvCount(RS485Instance *instance)
{
	if (instance == NULL)
		return 0u;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	uint32_t recv_count = instance->recv_count;
	if (primask == 0u) {
		__enable_irq();
	}
	return recv_count;
}

uint8_t RS485IsTxReady(RS485Instance *instance)
{
	if (instance == NULL)
		return 0u;

	return USARTIsReady((USARTInstance *)instance->comm_instance);
}
