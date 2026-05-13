#include "bsp_spi.h"
#include "memory.h"
#include "stdlib.h"

/* 所有的spi instance保存于此,用于callback时判断中断来源*/
#define SPI_BLOCK_TIMEOUT_MS 1u

static SPIInstance *spi_instance[MX_SPI_BUS_SLAVE_CNT] = {NULL};
static SPIInstance *spi_active_instance[MX_SPI_BUS_SLAVE_CNT] = {NULL};
static uint8_t spi_idx                                      = 0; // 配合中断以及初始化

volatile SPI_DMA_Debug_t g_spi_dma_debug = {0};

static int8_t SPIFindActiveSlotByHandle(SPI_HandleTypeDef *hspi)
{
    for (uint8_t i = 0; i < MX_SPI_BUS_SLAVE_CNT; i++) {
        if (spi_active_instance[i] != NULL && spi_active_instance[i]->spi_handle == hspi)
            return (int8_t)i;
    }

    return -1;
}

static int8_t SPIFindActiveSlotByInstance(SPIInstance *spi_ins)
{
    for (uint8_t i = 0; i < MX_SPI_BUS_SLAVE_CNT; i++) {
        if (spi_active_instance[i] == spi_ins)
            return (int8_t)i;
    }

    return -1;
}

static int8_t SPIFindFreeActiveSlot(void)
{
    for (uint8_t i = 0; i < MX_SPI_BUS_SLAVE_CNT; i++) {
        if (spi_active_instance[i] == NULL)
            return (int8_t)i;
    }

    return -1;
}

static void SPISetActiveInstance(SPIInstance *spi_ins)
{
    if (spi_ins == NULL || spi_ins->spi_handle == NULL)
        return;

    int8_t slot = SPIFindActiveSlotByHandle(spi_ins->spi_handle);
    if (slot < 0)
        slot = SPIFindFreeActiveSlot();

    if (slot >= 0) {
        spi_active_instance[slot]        = spi_ins;
        g_spi_dma_debug.active_instance = spi_ins;
    }
}

static void SPIClearActiveInstance(SPIInstance *spi_ins)
{
    int8_t slot = SPIFindActiveSlotByInstance(spi_ins);
    if (slot >= 0) {
        spi_active_instance[slot] = NULL;
        g_spi_dma_debug.active_clear_count++;
    }

    if (g_spi_dma_debug.active_instance == spi_ins)
        g_spi_dma_debug.active_instance = NULL;
}

static SPIInstance *SPITakeActiveInstance(SPI_HandleTypeDef *hspi)
{
    int8_t slot = SPIFindActiveSlotByHandle(hspi);
    if (slot < 0)
        return NULL;

    SPIInstance *spi_ins       = spi_active_instance[slot];
    spi_active_instance[slot]  = NULL;
    g_spi_dma_debug.active_clear_count++;
    if (g_spi_dma_debug.active_instance == spi_ins)
        g_spi_dma_debug.active_instance = NULL;

    return spi_ins;
}

static void SPIRecordStatus(SPI_HandleTypeDef *hspi, HAL_StatusTypeDef status)
{
    g_spi_dma_debug.last_hal_status = (uint32_t)status;
    if (hspi != NULL)
        g_spi_dma_debug.last_hal_error = HAL_SPI_GetError(hspi);
}

static void SPIRecordStartFail(SPIInstance *spi_ins, HAL_StatusTypeDef status)
{
    g_spi_dma_debug.start_fail_count++;
    if (status == HAL_BUSY)
        g_spi_dma_debug.hal_busy_count++;
    SPIRecordStatus(spi_ins != NULL ? spi_ins->spi_handle : NULL, status);
}

static void SPIReleaseInstance(SPIInstance *spi_ins)
{
    if (spi_ins == NULL)
        return;

    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
    spi_ins->rx_size   = 0;
    spi_ins->rx_buffer = NULL;
}

static void SPIReleaseCS(SPIInstance *spi_ins)
{
    if (spi_ins == NULL)
        return;

    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
}

static void SPIReleaseActiveInstances(SPI_HandleTypeDef *hspi)
{
    SPIInstance *active = SPITakeActiveInstance(hspi);
    if (active != NULL) {
        SPIReleaseInstance(active);
        return;
    }

    for (size_t i = 0; i < spi_idx; i++) {
        if (spi_instance[i] != NULL &&
            spi_instance[i]->spi_handle == hspi &&
            HAL_GPIO_ReadPin(spi_instance[i]->GPIOx, spi_instance[i]->cs_pin) == GPIO_PIN_RESET) {
            SPIReleaseInstance(spi_instance[i]);
        }
    }
}

SPIInstance *SPIRegister(SPI_Init_Config_s *conf)
{
    if (spi_idx >= MX_SPI_BUS_SLAVE_CNT) // 超过最大实例数
        while (1)
            ;
    SPIInstance *instance = (SPIInstance *)malloc(sizeof(SPIInstance));
    memset(instance, 0, sizeof(SPIInstance));

    instance->spi_handle    = conf->spi_handle;
    instance->GPIOx         = conf->GPIOx;
    instance->cs_pin        = conf->cs_pin;
    instance->spi_work_mode = conf->spi_work_mode;
    instance->callback      = conf->callback;
    instance->id            = conf->id;

    spi_instance[spi_idx++] = instance;
    return instance;
}

void SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    // 拉低片选,开始传输(选中从机)
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode) {
        case SPI_DMA_MODE:
            SPISetActiveInstance(spi_ins);
            status = HAL_SPI_Transmit_DMA(spi_ins->spi_handle, ptr_data, len);
            break;
        case SPI_IT_MODE:
            SPISetActiveInstance(spi_ins);
            status = HAL_SPI_Transmit_IT(spi_ins->spi_handle, ptr_data, len);
            break;
        case SPI_BLOCK_MODE:
            status = HAL_SPI_Transmit(spi_ins->spi_handle, ptr_data, len, SPI_BLOCK_TIMEOUT_MS);
            // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
            SPIReleaseInstance(spi_ins);
            break;
        default:
            while (1)
                ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
    }

    if (status != HAL_OK) {
        SPIRecordStartFail(spi_ins, status);
        SPIClearActiveInstance(spi_ins);
        SPIReleaseInstance(spi_ins);
    } else {
        SPIRecordStatus(spi_ins->spi_handle, status);
    }
}

void SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    // 用于稍后回调使用
    spi_ins->rx_size   = len;
    spi_ins->rx_buffer = ptr_data;
    // 拉低片选,开始传输
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode) {
        case SPI_DMA_MODE:
            SPISetActiveInstance(spi_ins);
            status = HAL_SPI_Receive_DMA(spi_ins->spi_handle, ptr_data, len);
            break;
        case SPI_IT_MODE:
            SPISetActiveInstance(spi_ins);
            status = HAL_SPI_Receive_IT(spi_ins->spi_handle, ptr_data, len);
            break;
        case SPI_BLOCK_MODE:
            status = HAL_SPI_Receive(spi_ins->spi_handle, ptr_data, len, SPI_BLOCK_TIMEOUT_MS);
            // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
            SPIReleaseInstance(spi_ins);
            break;
        default:
            while (1)
                ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
    }

    if (status != HAL_OK) {
        SPIRecordStartFail(spi_ins, status);
        SPIClearActiveInstance(spi_ins);
        SPIReleaseInstance(spi_ins);
    } else {
        SPIRecordStatus(spi_ins->spi_handle, status);
    }
}

uint8_t SPITransRecvStart(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (spi_ins == NULL || spi_ins->spi_handle == NULL || ptr_data_rx == NULL || ptr_data_tx == NULL || len == 0)
        return 0;

    // 用于稍后回调使用,请保证ptr_data_rx在回调函数被调用之前仍然在作用域内,否则析构之后的行为是未定义的!!!
    spi_ins->rx_size   = len;
    spi_ins->rx_buffer = ptr_data_rx;
    // 拉低片选,开始传输
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode) {
        case SPI_DMA_MODE:
            SPISetActiveInstance(spi_ins);
            if (spi_ins->spi_handle->hdmarx != NULL && spi_ins->spi_handle->hdmatx != NULL)
                status = HAL_SPI_TransmitReceive_DMA(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len);
            break;
        case SPI_IT_MODE:
            SPISetActiveInstance(spi_ins);
            status = HAL_SPI_TransmitReceive_IT(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len);
            break;
        case SPI_BLOCK_MODE:
            status = HAL_SPI_TransmitReceive(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len, SPI_BLOCK_TIMEOUT_MS);
            // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
            SPIReleaseInstance(spi_ins);
            SPIRecordStatus(spi_ins->spi_handle, status);
            return (status == HAL_OK) ? 1 : 0;
        default:
            while (1)
                ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
    }

    if (status != HAL_OK) {
        SPIRecordStartFail(spi_ins, status);
        SPIClearActiveInstance(spi_ins);
        SPIReleaseInstance(spi_ins);
        return 0;
    }

    SPIRecordStatus(spi_ins->spi_handle, status);
    return 1;
}

void SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len)
{
    (void)SPITransRecvStart(spi_ins, ptr_data_rx, ptr_data_tx, len);
}

void SPIAbort(SPIInstance *spi_ins)
{
    HAL_StatusTypeDef status;

    if (spi_ins == NULL || spi_ins->spi_handle == NULL)
        return;

    g_spi_dma_debug.abort_count++;
    SPIClearActiveInstance(spi_ins);
    status = HAL_SPI_Abort(spi_ins->spi_handle);
    SPIRecordStatus(spi_ins->spi_handle, status);
    SPIReleaseInstance(spi_ins);
}

void SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode)
{
    if (spi_mode != SPI_DMA_MODE && spi_mode != SPI_IT_MODE && spi_mode != SPI_BLOCK_MODE)
        while (1)
            ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况

    if (spi_ins->spi_work_mode != spi_mode) {
        spi_ins->spi_work_mode = spi_mode;
    }
}

/**
 * @brief 当SPI接收完成,将会调用此回调函数,可以进行协议解析或其他必须的数据处理等
 *
 * @param hspi spi handle
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPIInstance *active = SPITakeActiveInstance(hspi);
    if (active == NULL) {
        SPIReleaseActiveInstances(hspi);
        return;
    }

    g_spi_dma_debug.rx_cplt_count++;
    SPIRecordStatus(hspi, HAL_OK);
    SPIReleaseCS(active);
    // @todo 后续添加holdon模式,由用户自行决定何时释放片选,允许进行连续传输
    if (active->callback != NULL) // 回调函数不为空, 则调用回调函数
        active->callback(active);
}

/**
 * @brief 这是对HAL库的__weak函数的重写,全双工传输使用IT或DMA模式,在传输完成时会调用此函数
 * @param hspi spi handle
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPIInstance *active = SPITakeActiveInstance(hspi);
    if (active == NULL) {
        SPIReleaseActiveInstances(hspi);
        return;
    }

    g_spi_dma_debug.txrx_cplt_count++;
    SPIRecordStatus(hspi, HAL_OK);
    SPIReleaseCS(active);
    if (active->callback != NULL)
        active->callback(active);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    g_spi_dma_debug.tx_cplt_count++;
    SPIRecordStatus(hspi, HAL_OK);
    SPIReleaseActiveInstances(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    g_spi_dma_debug.error_count++;
    SPIRecordStatus(hspi, HAL_ERROR);
    SPIReleaseActiveInstances(hspi);
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    g_spi_dma_debug.abort_count++;
    SPIRecordStatus(hspi, HAL_OK);
    SPIReleaseActiveInstances(hspi);
}
