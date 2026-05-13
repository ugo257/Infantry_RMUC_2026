
#include "bmi088_regNdef.h"
#include "bmi088.h"
#include "user_lib.h"
#include "buzzer.h"
#include "robot_params.h"
#include "bsp_dwt.h"
#include <string.h>

#define BMI088_DMA_TEMP_PERIOD 500u
#define BMI088_BLOCK_TEMP_READ_DIVIDER 100u
#define BMI088_DMA_TIMEOUT_US 2000u
#define BMI088_GYRO_ZERO_SPIKE_MIN_RAD_S 0.35f
#define BMI088_GYRO_JUMP_LIMIT_RAD_S 10.0f
#define BMI088_GYRO_ABS_LIMIT_RAD_S 40.0f

volatile BMI088_DMA_Debug_t g_bmi088_dma_debug = {0};
volatile BMI088_Trigger_Debug_t g_bmi088_trigger_debug = {0};
volatile BMI088_Calibration_Debug_t g_bmi088_cali_debug = {0};
volatile BMI088_Calibration_Result_t g_bmi088_cali_result = {0};

static const float k_bmi088_cali_install_matrix[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};

typedef enum
{
    BMI088_DMA_STATE_IDLE = 0,
    BMI088_DMA_STATE_ACC,
    BMI088_DMA_STATE_GYRO,
    BMI088_DMA_STATE_TEMP,
    BMI088_DMA_STATE_SAMPLE_READY,
    BMI088_DMA_STATE_WAIT_GYRO,
    BMI088_DMA_STATE_WAIT_TEMP,
} BMI088_DMA_State_e;

typedef enum
{
    BMI088_TRIGGER_STATE_IDLE = 0,
    BMI088_TRIGGER_STATE_ACC,
    BMI088_TRIGGER_STATE_GYRO,
    BMI088_TRIGGER_STATE_TEMP,
    BMI088_TRIGGER_STATE_SAMPLE_READY,
} BMI088_Trigger_State_e;

static void BMI088DMASyncDebug(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    g_bmi088_dma_debug.acc_complete_count      = bmi088->dma_debug.acc_complete_count;
    g_bmi088_dma_debug.gyro_complete_count     = bmi088->dma_debug.gyro_complete_count;
    g_bmi088_dma_debug.temp_complete_count     = bmi088->dma_debug.temp_complete_count;
    g_bmi088_dma_debug.new_sample_count        = bmi088->dma_debug.new_sample_count;
    g_bmi088_dma_debug.start_fail_count        = bmi088->dma_debug.start_fail_count;
    g_bmi088_dma_debug.timeout_fallback_count  = bmi088->dma_debug.timeout_fallback_count;
    g_bmi088_dma_debug.acc_drdy_count          = bmi088->dma_debug.acc_drdy_count;
    g_bmi088_dma_debug.gyro_drdy_count         = bmi088->dma_debug.gyro_drdy_count;
    g_bmi088_dma_debug.chain_defer_count       = bmi088->dma_debug.chain_defer_count;
    g_bmi088_dma_debug.zero_frame_count        = bmi088->dma_debug.zero_frame_count;
    g_bmi088_dma_debug.invalid_frame_count     = bmi088->dma_debug.invalid_frame_count;
    g_bmi088_dma_debug.gyro_zero_axis_count    = bmi088->dma_debug.gyro_zero_axis_count;
    g_bmi088_dma_debug.gyro_jump_reject_count  = bmi088->dma_debug.gyro_jump_reject_count;
    g_bmi088_dma_debug.current_state           = bmi088->dma_debug.current_state;
}

static void BMI088DMASetState(BMI088Instance *bmi088, BMI088_DMA_State_e state)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_state               = (uint8_t)state;
    bmi088->dma_debug.current_state = (uint8_t)state;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088DMAMarkStartFail(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_debug.start_fail_count++;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088DMAMarkTimeoutFallback(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_debug.timeout_fallback_count++;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088DMAMarkChainDefer(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_debug.chain_defer_count++;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088DMAMarkAccDRDY(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_debug.acc_drdy_count++;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088DMAMarkGyroDRDY(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_debug.gyro_drdy_count++;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088DMAMarkZeroFrame(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_debug.zero_frame_count++;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088DMAMarkInvalidFrame(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_debug.invalid_frame_count++;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088DMAMarkGyroZeroAxis(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_debug.gyro_zero_axis_count++;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088DMAMarkGyroJumpReject(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->dma_debug.gyro_jump_reject_count++;
    BMI088DMASyncDebug(bmi088);
}

static void BMI088TriggerSyncDebug(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    g_bmi088_trigger_debug.acc_drdy_count          = bmi088->trigger_debug.acc_drdy_count;
    g_bmi088_trigger_debug.gyro_drdy_count         = bmi088->trigger_debug.gyro_drdy_count;
    g_bmi088_trigger_debug.acc_complete_count      = bmi088->trigger_debug.acc_complete_count;
    g_bmi088_trigger_debug.gyro_complete_count     = bmi088->trigger_debug.gyro_complete_count;
    g_bmi088_trigger_debug.temp_complete_count     = bmi088->trigger_debug.temp_complete_count;
    g_bmi088_trigger_debug.new_sample_count        = bmi088->trigger_debug.new_sample_count;
    g_bmi088_trigger_debug.start_fail_count        = bmi088->trigger_debug.start_fail_count;
    g_bmi088_trigger_debug.busy_defer_count        = bmi088->trigger_debug.busy_defer_count;
    g_bmi088_trigger_debug.timeout_count           = bmi088->trigger_debug.timeout_count;
    g_bmi088_trigger_debug.zero_frame_count        = bmi088->trigger_debug.zero_frame_count;
    g_bmi088_trigger_debug.invalid_frame_count     = bmi088->trigger_debug.invalid_frame_count;
    g_bmi088_trigger_debug.gyro_zero_axis_count    = bmi088->trigger_debug.gyro_zero_axis_count;
    g_bmi088_trigger_debug.gyro_jump_reject_count  = bmi088->trigger_debug.gyro_jump_reject_count;
    g_bmi088_trigger_debug.current_state           = bmi088->trigger_debug.current_state;
}

static void BMI088TriggerSetState(BMI088Instance *bmi088, BMI088_Trigger_State_e state)
{
    if (bmi088 == NULL)
        return;

    bmi088->trigger_state               = (uint8_t)state;
    bmi088->trigger_debug.current_state = (uint8_t)state;
    BMI088TriggerSyncDebug(bmi088);
}

static void BMI088TriggerMarkStartFail(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->trigger_debug.start_fail_count++;
    BMI088TriggerSyncDebug(bmi088);
}

static void BMI088TriggerMarkBusyDefer(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->trigger_debug.busy_defer_count++;
    BMI088TriggerSyncDebug(bmi088);
}

static void BMI088TriggerMarkTimeout(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->trigger_debug.timeout_count++;
    BMI088TriggerSyncDebug(bmi088);
}

static void BMI088TriggerMarkAccDRDY(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->trigger_debug.acc_drdy_count++;
    BMI088TriggerSyncDebug(bmi088);
}

static void BMI088TriggerMarkGyroDRDY(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->trigger_debug.gyro_drdy_count++;
    BMI088TriggerSyncDebug(bmi088);
}

static void BMI088MarkZeroFrame(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE) {
        bmi088->trigger_debug.zero_frame_count++;
        BMI088TriggerSyncDebug(bmi088);
    } else {
        BMI088DMAMarkZeroFrame(bmi088);
    }
}

static void BMI088MarkInvalidFrame(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE) {
        bmi088->trigger_debug.invalid_frame_count++;
        BMI088TriggerSyncDebug(bmi088);
    } else {
        BMI088DMAMarkInvalidFrame(bmi088);
    }
}

static void BMI088MarkGyroZeroAxis(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE) {
        bmi088->trigger_debug.gyro_zero_axis_count++;
        BMI088TriggerSyncDebug(bmi088);
    } else {
        BMI088DMAMarkGyroZeroAxis(bmi088);
    }
}

static void BMI088MarkGyroJumpReject(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE) {
        bmi088->trigger_debug.gyro_jump_reject_count++;
        BMI088TriggerSyncDebug(bmi088);
    } else {
        BMI088DMAMarkGyroJumpReject(bmi088);
    }
}

static inline float BMI088Absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

// ---------------------------以下私有函数,用于读写BMI088寄存器封装,blocking--------------------------------//
/**
 * @brief 读取BMI088寄存器Accel. BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg 待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
//加速度计读取函数
static void BMI088AccelRead(BMI088Instance *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
    if (len > 6)
        while (1)
            ;
    static uint8_t tx[8];
    static uint8_t rx[8];
    // 一次读取最多6个字节,加上两个dummy data    第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
    tx[0] = 0x80 | reg;
    SPITransRecv(bmi088->spi_acc, rx, tx, len + 2);
    memcpy(dataptr, rx + 2, len); // @todo : memcpy有额外开销,后续可以考虑优化,在SPI中加入接口或模式,使得在一次传输结束后不释放CS,直接接着传输
}

/**
 * @brief 读取BMI088寄存器Gyro, BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg  待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
//陀螺仪读取函数
static void BMI088GyroRead(BMI088Instance *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
    if (len > 6)
        while (1)
            ;
    static uint8_t tx[7] = {0x80};
    static uint8_t rx[7];
    // 一次读取最多6个字节,加上一个dummy data  ,第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
    tx[0] = 0x80 | reg;
    SPITransRecv(bmi088->spi_gyro, rx, tx, len + 1);
    memcpy(dataptr, rx + 1, len); // @todo : memcpy有额外开销,后续可以考虑优化,在SPI中加入接口或模式,使得在一次传输结束后不释放CS,直接接着传输
}

static void BMI088DecodeAccel(BMI088Instance *bmi088, const uint8_t *raw)
{
    for (uint8_t i = 0; i < 3; i++)
        bmi088->acc[i] = bmi088->BMI088_ACCEL_SEN * (float)(int16_t)(((raw[2 * i + 1]) << 8) | raw[2 * i]);
}

static void BMI088DecodeGyro(BMI088Instance *bmi088, const uint8_t *raw)
{
    for (uint8_t i = 0; i < 3; i++)
        bmi088->gyro[i] = bmi088->BMI088_GYRO_SEN * (float)(int16_t)(((raw[2 * i + 1]) << 8) | raw[2 * i]);
}

static void BMI088DecodeGyroTo(BMI088Instance *bmi088, const uint8_t *raw, float gyro[3])
{
    if (bmi088 == NULL || raw == NULL || gyro == NULL)
        return;

    for (uint8_t i = 0; i < 3; i++)
        gyro[i] = bmi088->BMI088_GYRO_SEN * (float)(int16_t)(((raw[2 * i + 1]) << 8) | raw[2 * i]);
}

static void BMI088RememberGyro(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    for (uint8_t i = 0; i < 3; i++)
        bmi088->last_gyro[i] = bmi088->gyro[i];
    bmi088->gyro_guard_valid = 1u;
}

static void BMI088DecodeTemp(BMI088Instance *bmi088, const uint8_t *raw)
{
    int16_t temperate_raw_temp = (int16_t)((raw[0] << 3) | (raw[1] >> 5));

    if (temperate_raw_temp > 1023)
        temperate_raw_temp -= 2048;

    bmi088->temperature = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

static uint8_t BMI088RawAllSame(const uint8_t *raw, uint8_t len, uint8_t value)
{
    if (raw == NULL || len == 0)
        return 1;

    for (uint8_t i = 0; i < len; i++) {
        if (raw[i] != value)
            return 0;
    }

    return 1;
}

static uint8_t BMI088RawFrameValid(BMI088Instance *bmi088, const uint8_t *raw, uint8_t len)
{
    if (BMI088RawAllSame(raw, len, 0x00u) != 0) {
        BMI088MarkZeroFrame(bmi088);
        return 0;
    }
    if (BMI088RawAllSame(raw, len, 0xFFu) != 0) {
        BMI088MarkInvalidFrame(bmi088);
        return 0;
    }

    return 1;
}

static uint8_t BMI088AccelRawFrameValid(BMI088Instance *bmi088, const uint8_t *raw)
{
    float acc[3];
    float acc_power;

    if (BMI088RawFrameValid(bmi088, raw, 6) == 0)
        return 0;

    for (uint8_t i = 0; i < 3; i++)
        acc[i] = bmi088->BMI088_ACCEL_SEN * (float)(int16_t)(((raw[2 * i + 1]) << 8) | raw[2 * i]);

    acc_power = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];
    if (acc_power < 0.25f || acc_power > 2500.0f) {
        BMI088MarkInvalidFrame(bmi088);
        return 0;
    }

    return 1;
}

static uint8_t BMI088GyroRawFrameValid(BMI088Instance *bmi088, const uint8_t *raw)
{
    float gyro[3] = {0.0f, 0.0f, 0.0f};

    if (BMI088RawFrameValid(bmi088, raw, 6) == 0)
        return 0;

    BMI088DecodeGyroTo(bmi088, raw, gyro);
    for (uint8_t i = 0; i < 3; i++) {
        if (BMI088Absf(gyro[i]) > BMI088_GYRO_ABS_LIMIT_RAD_S) {
            BMI088MarkInvalidFrame(bmi088);
            return 0;
        }
    }

    if (bmi088 != NULL && bmi088->gyro_guard_valid != 0u) {
        for (uint8_t i = 0; i < 3; i++) {
            uint8_t axis_zero = (uint8_t)(raw[2 * i] == 0x00u && raw[2 * i + 1] == 0x00u);
            if (axis_zero != 0u &&
                BMI088Absf(bmi088->last_gyro[i]) > BMI088_GYRO_ZERO_SPIKE_MIN_RAD_S) {
                BMI088MarkGyroZeroAxis(bmi088);
                return 0;
            }
            if (BMI088Absf(gyro[i] - bmi088->last_gyro[i]) > BMI088_GYRO_JUMP_LIMIT_RAD_S) {
                BMI088MarkGyroJumpReject(bmi088);
                return 0;
            }
        }
    }

    return 1;
}

static uint8_t BMI088TempRawFrameValid(BMI088Instance *bmi088, const uint8_t *raw)
{
    if (BMI088RawAllSame(raw, 2, 0xFFu) != 0) {
        BMI088MarkInvalidFrame(bmi088);
        return 0;
    }

    return 1;
}

static void BMI088CopyData(BMI088Instance *bmi088, BMI088_Data_t *data_store)
{
    if (data_store == NULL)
        return;

    data_store->acc[0]      = bmi088->acc[0];
    data_store->acc[1]      = bmi088->acc[1];
    data_store->acc[2]      = bmi088->acc[2];
    data_store->gyro[0]     = bmi088->gyro[0];
    data_store->gyro[1]     = bmi088->gyro[1];
    data_store->gyro[2]     = bmi088->gyro[2];
    data_store->temperature = bmi088->temperature;
}

static uint8_t BMI088AcquireBlocking(BMI088Instance *bmi088, BMI088_Data_t *data_store)
{
    uint8_t buf[6] = {0}; // 最多读取6个byte(gyro/acc,temp是2)
    uint8_t acc_raw[6] = {0};
    uint8_t gyro_raw[6] = {0};
    uint8_t temp_raw[2] = {0};
    static uint16_t temp_read_divider = 0;
    SPI_TXRX_MODE_e acc_mode;
    SPI_TXRX_MODE_e gyro_mode;

    if (bmi088 == NULL || bmi088->spi_acc == NULL || bmi088->spi_gyro == NULL)
        return 0;

    acc_mode  = bmi088->spi_acc->spi_work_mode;
    gyro_mode = bmi088->spi_gyro->spi_work_mode;
    SPISetMode(bmi088->spi_acc, SPI_BLOCK_MODE);
    SPISetMode(bmi088->spi_gyro, SPI_BLOCK_MODE);

    BMI088AccelRead(bmi088, BMI088_ACCEL_XOUT_L, buf, 6);
    memcpy(acc_raw, buf, 6);
    if (BMI088AccelRawFrameValid(bmi088, acc_raw) == 0) {
        SPISetMode(bmi088->spi_acc, acc_mode);
        SPISetMode(bmi088->spi_gyro, gyro_mode);
        return 0;
    }

    BMI088GyroRead(bmi088, BMI088_GYRO_X_L, buf, 6);
    memcpy(gyro_raw, buf, 6);
    if (BMI088GyroRawFrameValid(bmi088, gyro_raw) == 0) {
        SPISetMode(bmi088->spi_acc, acc_mode);
        SPISetMode(bmi088->spi_gyro, gyro_mode);
        return 0;
    }

    memcpy(bmi088->acc_raw, acc_raw, 6);
    BMI088DecodeAccel(bmi088, bmi088->acc_raw);

    memcpy(bmi088->gyro_raw, gyro_raw, 6);
    BMI088DecodeGyro(bmi088, bmi088->gyro_raw);
    BMI088RememberGyro(bmi088);

    if (bmi088->update_flag.temp == 0 || temp_read_divider >= (BMI088_BLOCK_TEMP_READ_DIVIDER - 1u)) {
        BMI088AccelRead(bmi088, BMI088_TEMP_M, buf, 2);
        memcpy(temp_raw, buf, 2);
        if (BMI088TempRawFrameValid(bmi088, temp_raw) != 0) {
            memcpy(bmi088->temp_raw, temp_raw, 2);
            BMI088DecodeTemp(bmi088, bmi088->temp_raw);
            bmi088->update_flag.temp = 1;
        }
        temp_read_divider = 0;
    } else {
        temp_read_divider++;
    }

    bmi088->update_flag.acc       = 1;
    bmi088->update_flag.gyro      = 1;
    bmi088->update_flag.imu_ready = 1;
    BMI088CopyData(bmi088, data_store);

    SPISetMode(bmi088->spi_acc, acc_mode);
    SPISetMode(bmi088->spi_gyro, gyro_mode);
    return 1;
}

static uint8_t BMI088DMAAvailable(BMI088Instance *bmi088)
{
    if (bmi088 == NULL || bmi088->spi_acc == NULL || bmi088->spi_gyro == NULL)
        return 0;
    if (bmi088->spi_acc->spi_handle == NULL || bmi088->spi_gyro->spi_handle == NULL)
        return 0;
    if (bmi088->spi_acc->spi_handle->hdmarx == NULL || bmi088->spi_acc->spi_handle->hdmatx == NULL)
        return 0;
    if (bmi088->spi_gyro->spi_handle->hdmarx == NULL || bmi088->spi_gyro->spi_handle->hdmatx == NULL)
        return 0;
    return 1;
}

static uint8_t BMI088SPIReadyForDMA(SPIInstance *spi)
{
    SPI_HandleTypeDef *hspi;

    if (spi == NULL || spi->spi_handle == NULL)
        return 0;

    hspi = spi->spi_handle;
    if (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
        return 0;
    if (hspi->hdmarx != NULL && HAL_DMA_GetState(hspi->hdmarx) != HAL_DMA_STATE_READY)
        return 0;
    if (hspi->hdmatx != NULL && HAL_DMA_GetState(hspi->hdmatx) != HAL_DMA_STATE_READY)
        return 0;

    return 1;
}

static uint8_t BMI088StartDMARead(BMI088Instance *bmi088,
                                  SPIInstance *spi,
                                  uint8_t reg,
                                  uint8_t *tx,
                                  uint8_t *rx,
                                  uint8_t frame_len,
                                  BMI088_DMA_State_e state)
{
    if (BMI088DMAAvailable(bmi088) == 0 || spi == NULL || tx == NULL || rx == NULL) {
        BMI088DMAMarkStartFail(bmi088);
        return 0;
    }
    if (BMI088SPIReadyForDMA(spi) == 0)
        return 0;

    for (uint8_t i = 0; i < frame_len; i++) {
        tx[i] = 0;
        rx[i] = 0;
    }
    tx[0] = 0x80 | reg;

    BMI088DMASetState(bmi088, state);
    bmi088->dma_start_us = DWT_GetTimeline_us();
    if (SPITransRecvStart(spi, rx, tx, frame_len) == 0) {
        BMI088DMAMarkStartFail(bmi088);
        bmi088->dma_start_us = 0;
        BMI088DMASetState(bmi088, BMI088_DMA_STATE_IDLE);
        return 0;
    }
    return 1;
}

static uint8_t BMI088StartDMAAccel(BMI088Instance *bmi088)
{
    return BMI088StartDMARead(bmi088,
                              bmi088->spi_acc,
                              BMI088_ACCEL_XOUT_L,
                              bmi088->acc_dma_tx,
                              bmi088->acc_dma_rx,
                              8,
                              BMI088_DMA_STATE_ACC);
}

static uint8_t BMI088StartDMAGyro(BMI088Instance *bmi088)
{
    return BMI088StartDMARead(bmi088,
                              bmi088->spi_gyro,
                              BMI088_GYRO_X_L,
                              bmi088->gyro_dma_tx,
                              bmi088->gyro_dma_rx,
                              7,
                              BMI088_DMA_STATE_GYRO);
}

static uint8_t BMI088StartDMATemp(BMI088Instance *bmi088)
{
    return BMI088StartDMARead(bmi088,
                              bmi088->spi_acc,
                              BMI088_TEMP_M,
                              bmi088->temp_dma_tx,
                              bmi088->temp_dma_rx,
                              4,
                              BMI088_DMA_STATE_TEMP);
}

static void BMI088DMAFault(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    BMI088DMASetState(bmi088, BMI088_DMA_STATE_IDLE);
    bmi088->dma_start_us      = 0;
    bmi088->dma_read_temp     = 0;
    bmi088->dma_sample_ready  = 0;
}

static void BMI088DMACompleteSample(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    BMI088DMASetState(bmi088, BMI088_DMA_STATE_SAMPLE_READY);
    bmi088->dma_start_us              = 0;
    bmi088->dma_read_temp             = 0;
    bmi088->dma_sample_ready          = 1;
    bmi088->update_flag.imu_ready     = 1;
    bmi088->dma_debug.new_sample_count++;
    BMI088DMASyncDebug(bmi088);
}

static uint8_t BMI088StartDMASample(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return 0;
    if (bmi088->dma_state != BMI088_DMA_STATE_IDLE)
        return 0;
    if (BMI088DMAAvailable(bmi088) == 0)
        return 0;

    bmi088->dma_sample_ready = 0;
    bmi088->dma_temp_prescaler++;
    if (bmi088->dma_temp_prescaler >= BMI088_DMA_TEMP_PERIOD) {
        bmi088->dma_temp_prescaler = 0;
        bmi088->dma_read_temp      = 1;
    } else {
        bmi088->dma_read_temp = 0;
    }

    if (BMI088StartDMAAccel(bmi088) == 0) {
        return 0;
    }
    return 1;
}

static void BMI088DMAPump(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    if (bmi088->dma_state == BMI088_DMA_STATE_WAIT_GYRO) {
        if (BMI088StartDMAGyro(bmi088) == 0)
            BMI088DMAMarkChainDefer(bmi088);
    } else if (bmi088->dma_state == BMI088_DMA_STATE_WAIT_TEMP) {
        if (BMI088StartDMATemp(bmi088) == 0)
            BMI088DMAMarkChainDefer(bmi088);
    }
}

static void BMI088AbortDMATransfer(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    if (bmi088->dma_state == BMI088_DMA_STATE_GYRO) {
        SPIAbort(bmi088->spi_gyro);
    } else if (bmi088->dma_state == BMI088_DMA_STATE_ACC || bmi088->dma_state == BMI088_DMA_STATE_TEMP) {
        SPIAbort(bmi088->spi_acc);
    }
}

static uint8_t BMI088HandleDMATimeout(BMI088Instance *bmi088, BMI088_Data_t *data_store)
{
    uint64_t now_us;
    uint8_t ok;

    if (bmi088 == NULL || bmi088->dma_state == BMI088_DMA_STATE_IDLE || bmi088->dma_start_us == 0)
        return 0;

    now_us = DWT_GetTimeline_us();
    if ((uint64_t)(now_us - bmi088->dma_start_us) <= BMI088_DMA_TIMEOUT_US)
        return 0;

    BMI088DMAMarkTimeoutFallback(bmi088);
    BMI088AbortDMATransfer(bmi088);
    BMI088DMAFault(bmi088);

    ok = BMI088AcquireBlocking(bmi088, data_store);
    bmi088->dma_primed = ok;
    (void)BMI088StartDMASample(bmi088);
    return ok;
}

static uint8_t BMI088SPIReadyForIT(SPIInstance *spi)
{
    if (spi == NULL || spi->spi_handle == NULL)
        return 0;

    return (HAL_SPI_GetState(spi->spi_handle) == HAL_SPI_STATE_READY) ? 1u : 0u;
}

static uint8_t BMI088StartTriggerRead(BMI088Instance *bmi088,
                                      SPIInstance *spi,
                                      uint8_t reg,
                                      uint8_t *tx,
                                      uint8_t *rx,
                                      uint8_t frame_len,
                                      BMI088_Trigger_State_e state)
{
    if (bmi088 == NULL || spi == NULL || tx == NULL || rx == NULL || frame_len == 0u) {
        BMI088TriggerMarkStartFail(bmi088);
        return 0;
    }
    if (BMI088SPIReadyForIT(spi) == 0u) {
        BMI088TriggerMarkBusyDefer(bmi088);
        return 0;
    }

    for (uint8_t i = 0; i < frame_len; i++) {
        tx[i] = 0u;
        rx[i] = 0u;
    }
    tx[0] = 0x80u | reg;

    BMI088TriggerSetState(bmi088, state);
    bmi088->trigger_start_us = DWT_GetTimeline_us();
    if (SPITransRecvStart(spi, rx, tx, frame_len) == 0u) {
        BMI088TriggerMarkStartFail(bmi088);
        bmi088->trigger_start_us = 0u;
        BMI088TriggerSetState(bmi088, BMI088_TRIGGER_STATE_IDLE);
        return 0;
    }

    return 1;
}

static uint8_t BMI088StartTriggerAccel(BMI088Instance *bmi088)
{
    return BMI088StartTriggerRead(bmi088,
                                  bmi088->spi_acc,
                                  BMI088_ACCEL_XOUT_L,
                                  bmi088->acc_trig_tx,
                                  bmi088->acc_trig_rx,
                                  8u,
                                  BMI088_TRIGGER_STATE_ACC);
}

static uint8_t BMI088StartTriggerGyro(BMI088Instance *bmi088)
{
    return BMI088StartTriggerRead(bmi088,
                                  bmi088->spi_gyro,
                                  BMI088_GYRO_X_L,
                                  bmi088->gyro_trig_tx,
                                  bmi088->gyro_trig_rx,
                                  7u,
                                  BMI088_TRIGGER_STATE_GYRO);
}

static uint8_t BMI088StartTriggerTemp(BMI088Instance *bmi088)
{
    return BMI088StartTriggerRead(bmi088,
                                  bmi088->spi_acc,
                                  BMI088_TEMP_M,
                                  bmi088->temp_trig_tx,
                                  bmi088->temp_trig_rx,
                                  4u,
                                  BMI088_TRIGGER_STATE_TEMP);
}

static void BMI088TriggerFault(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    bmi088->trigger_start_us = 0u;
    BMI088TriggerSetState(bmi088, BMI088_TRIGGER_STATE_IDLE);
}

static void BMI088TriggerCompleteSample(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;

    BMI088TriggerSetState(bmi088, BMI088_TRIGGER_STATE_SAMPLE_READY);
    bmi088->trigger_start_us          = 0u;
    bmi088->trigger_read_temp         = 0u;
    bmi088->trigger_sample_ready      = 1u;
    bmi088->trigger_acc_valid         = 0u;
    bmi088->trigger_gyro_valid        = 0u;
    bmi088->update_flag.imu_ready     = 1u;
    bmi088->trigger_debug.new_sample_count++;
    BMI088TriggerSyncDebug(bmi088);
}

static void BMI088TriggerMaybeScheduleTemp(BMI088Instance *bmi088)
{
    if (bmi088 == NULL || bmi088->trigger_acc_valid == 0u || bmi088->trigger_gyro_valid == 0u)
        return;

    bmi088->trigger_temp_prescaler++;
    if (bmi088->trigger_temp_prescaler >= BMI088_DMA_TEMP_PERIOD) {
        bmi088->trigger_temp_prescaler = 0u;
        bmi088->trigger_read_temp = 1u;
    } else {
        bmi088->trigger_read_temp = 0u;
    }
}

static void BMI088TriggerPump(BMI088Instance *bmi088)
{
    if (bmi088 == NULL)
        return;
    if (bmi088->trigger_state != BMI088_TRIGGER_STATE_IDLE)
        return;

    if (bmi088->trigger_acc_valid == 0u) {
        if (bmi088->trigger_acc_pending != 0u || bmi088->trigger_gyro_pending != 0u) {
            bmi088->trigger_acc_pending = 0u;
            (void)BMI088StartTriggerAccel(bmi088);
        }
        return;
    }

    if (bmi088->trigger_gyro_valid == 0u) {
        if (bmi088->trigger_gyro_pending != 0u) {
            bmi088->trigger_gyro_pending = 0u;
            (void)BMI088StartTriggerGyro(bmi088);
        }
        return;
    }

    BMI088TriggerMaybeScheduleTemp(bmi088);
    if (bmi088->trigger_read_temp != 0u) {
        (void)BMI088StartTriggerTemp(bmi088);
    } else {
        BMI088TriggerCompleteSample(bmi088);
    }
}

static void BMI088HandleTriggerTimeout(BMI088Instance *bmi088)
{
    uint64_t now_us;

    if (bmi088 == NULL || bmi088->trigger_start_us == 0u)
        return;
    if (bmi088->trigger_state != BMI088_TRIGGER_STATE_ACC &&
        bmi088->trigger_state != BMI088_TRIGGER_STATE_GYRO &&
        bmi088->trigger_state != BMI088_TRIGGER_STATE_TEMP)
        return;

    now_us = DWT_GetTimeline_us();
    if ((uint64_t)(now_us - bmi088->trigger_start_us) <= BMI088_DMA_TIMEOUT_US)
        return;

    BMI088TriggerMarkTimeout(bmi088);
    if (bmi088->trigger_state == BMI088_TRIGGER_STATE_ACC ||
        bmi088->trigger_state == BMI088_TRIGGER_STATE_TEMP) {
        SPIAbort(bmi088->spi_acc);
        bmi088->trigger_acc_pending = 1u;
    } else if (bmi088->trigger_state == BMI088_TRIGGER_STATE_GYRO) {
        SPIAbort(bmi088->spi_gyro);
        bmi088->trigger_gyro_pending = 1u;
    }
    BMI088TriggerFault(bmi088);
}

/**
 * @brief 写accel寄存器.对spitransmit形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
//寄存器写函数,写入加速度计数据
static void BMI088AccelWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    SPITransmit(bmi088->spi_acc, tx, 2);
}

/**
 * @brief 写gyro寄存器.形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
//寄存器写函数,写入陀螺仪数据
static void BMI088GyroWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    SPITransmit(bmi088->spi_gyro, tx, 2);
}
// -------------------------以上为私有函数,封装了BMI088寄存器读写函数,blocking--------------------------------//

// -------------------------以下为私有函数,用于初始化BMI088acc和gyro的硬件和配置--------------------------------//
#define BMI088REG   0
#define BMI088DATA  1
#define BMI088ERROR 2
// BMI088初始化配置数组for accel,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};
// BMI088初始化配置数组for gyro,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};
// @attention : 以上两个数组配合各自的初始化函数使用. 若要修改请参照BMI088 datasheet

/**
 * @brief 初始化BMI088加速度计,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE if any problems here
 */
static uint8_t BMI088AccelInit(BMI088Instance *bmi088)
{
    uint8_t WhoAmI_check = 0;

    // 加速度计以I2C模式启动,需要一次上升沿来切换到SPI模式,因此进行一次fake write
    BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1);
    DWT_Delay(0.001f);

    BMI088AccelWriteSingleReg(bmi088, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE); // 软复位
    DWT_Delay(BMI088_COM_WAIT_SENSOR_TIME / 1000);

    // 检查ID,如果不是0x1E(bmi088 whoami寄存器值),则返回错误
    BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1);
    if (WhoAmI_check != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001f);
    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Accel_Init_Table) / sizeof(BMI088_Accel_Init_Table[0]); i++) {
        reg  = BMI088_Accel_Init_Table[i][BMI088REG];
        data = BMI088_Accel_Init_Table[i][BMI088DATA];
        BMI088AccelWriteSingleReg(bmi088, reg, data); // 写入寄存器
        DWT_Delay(0.01f);
        BMI088AccelRead(bmi088, reg, &data, 1); // 写完之后立刻读回检查
        DWT_Delay(0.01f);
        if (data != BMI088_Accel_Init_Table[i][BMI088DATA])
            error |= BMI088_Accel_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,如果retry次数用完了,则返回error
    }
    // 设置灵敏度
    switch (BMI088_Accel_Init_Table[3][1]) {
        case BMI088_ACC_RANGE_3G:
            bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
            break;
        case BMI088_ACC_RANGE_6G:
            bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
            break;
        case BMI088_ACC_RANGE_12G:
            bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_12G_SEN;
            break;
        case BMI088_ACC_RANGE_24G:
            bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_24G_SEN;
            break;
        default:
            break;
    }
    return (uint8_t)error;
}

/**
 * @brief 初始化BMI088陀螺仪,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE
 */
static uint8_t BMI088GyroInit(BMI088Instance *bmi088)
{
    // 后续添加reset和通信检查?
    // code to go here ...
    BMI088GyroWriteSingleReg(bmi088, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE); // 软复位
    DWT_Delay(0.08f);

    // 检查ID,如果不是0x0F(bmi088 whoami寄存器值),则返回错误
    uint8_t WhoAmI_check = 0;
    BMI088GyroRead(bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1);
    if (WhoAmI_check != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001f);

    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Gyro_Init_Table) / sizeof(BMI088_Gyro_Init_Table[0]); i++) {
        reg  = BMI088_Gyro_Init_Table[i][BMI088REG];
        data = BMI088_Gyro_Init_Table[i][BMI088DATA];
        BMI088GyroWriteSingleReg(bmi088, reg, data); // 写入寄存器
        DWT_Delay(0.001f);
        BMI088GyroRead(bmi088, reg, &data, 1); // 写完之后立刻读回对应寄存器检查是否写入成功
        DWT_Delay(0.001f);
        if (data != BMI088_Gyro_Init_Table[i][BMI088DATA])
            error |= BMI088_Gyro_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,尝试重新写入.如果retry次数用完了,则返回error
    }
    // 设置灵敏度
    switch (BMI088_Gyro_Init_Table[0][1]) {
        case BMI088_GYRO_2000:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
            break;
        case BMI088_GYRO_1000:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_1000_SEN;
            break;
        case BMI088_GYRO_500:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_500_SEN;
            break;
        case BMI088_GYRO_250:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_250_SEN;
            break;
        case BMI088_GYRO_125:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_125_SEN;
            break;
        default:
            break;
    }
    return (uint8_t)error;
}
// -------------------------以上为私有函数,用于初始化BMI088acc和gyro的硬件和配置--------------------------------//

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
void BMI088_temp_control(BMI088Instance *bmi088)
{
    static uint8_t temp_constant_time = 0;
    static uint8_t first_temperate    = 0; // 第一次达到设定温度
    static float target_temp          = 0;
    target_temp                       = BMI088_TEMP_TARGET_C;
    if (target_temp > 45.0f) target_temp = 45.0f; // 限制在45度以内

    if (first_temperate) {
        PIDCalculate(bmi088->heat_pid, bmi088->temperature, target_temp);
        // 限制在正数范围
        if (bmi088->heat_pid->Output < 0.0f) {
            bmi088->heat_pid->Output = 0.0f;
        }
        if(bmi088->heat_pid->Iout<0){
            bmi088->heat_pid->Iout = 0;
        }
        PWMSetDutyRatio(bmi088->heat_pwm, bmi088->heat_pid->Output);
    } else {
        // 在没有达到设置的温度-4，一直最大功率加热
        PWMSetDutyRatio(bmi088->heat_pwm, 0.95f);
        if (bmi088->temperature > target_temp - 4) {
            temp_constant_time++;
            if (temp_constant_time > 200) {
                // 达到设置温度，设置积分项，加速收敛
                first_temperate        = 1;
                bmi088->heat_pid->Iout = 0.05f;
            }
        }
    }
}

// -------------------------以下为私有函数,private用于IT模式下的中断处理---------------------------------//

static void BMI088AccSPIFinishCallback(SPIInstance *spi)
{
    static BMI088Instance *bmi088;
    static uint16_t callback_time = 0;
    bmi088                        = (BMI088Instance *)(spi->id);
    if (bmi088 == NULL)
        return;

    if (bmi088->work_mode == BMI088_DMA_PERIODIC_MODE) {
        if (bmi088->dma_state == BMI088_DMA_STATE_ACC) {
            if (BMI088AccelRawFrameValid(bmi088, bmi088->acc_dma_rx + 2) == 0) {
                BMI088DMAFault(bmi088);
                bmi088->dma_start_pending = 1;
                return;
            }
            memcpy(bmi088->acc_raw, bmi088->acc_dma_rx + 2, 6);
            BMI088DecodeAccel(bmi088, bmi088->acc_raw);
            bmi088->update_flag.acc = 1;
            bmi088->dma_debug.acc_complete_count++;
            BMI088DMASyncDebug(bmi088);
            BMI088DMASetState(bmi088, BMI088_DMA_STATE_WAIT_GYRO);
            bmi088->dma_start_us = DWT_GetTimeline_us();
            return;
        }

        if (bmi088->dma_state == BMI088_DMA_STATE_TEMP) {
            if (BMI088TempRawFrameValid(bmi088, bmi088->temp_dma_rx + 2) != 0) {
                memcpy(bmi088->temp_raw, bmi088->temp_dma_rx + 2, 2);
                BMI088DecodeTemp(bmi088, bmi088->temp_raw);
                bmi088->update_flag.temp = 1;
            }
            bmi088->dma_debug.temp_complete_count++;
            BMI088DMASyncDebug(bmi088);
            BMI088DMACompleteSample(bmi088);
            return;
        }
    }

    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE) {
        if (bmi088->trigger_state == BMI088_TRIGGER_STATE_ACC) {
            if (BMI088AccelRawFrameValid(bmi088, bmi088->acc_trig_rx + 2) == 0u) {
                bmi088->trigger_acc_valid = 0u;
                bmi088->trigger_acc_pending = 1u;
                BMI088TriggerFault(bmi088);
                return;
            }
            memcpy(bmi088->acc_raw, bmi088->acc_trig_rx + 2, 6);
            BMI088DecodeAccel(bmi088, bmi088->acc_raw);
            bmi088->update_flag.acc = 1u;
            bmi088->trigger_acc_valid = 1u;
            bmi088->trigger_start_us = 0u;
            bmi088->trigger_debug.acc_complete_count++;
            BMI088TriggerSetState(bmi088, BMI088_TRIGGER_STATE_IDLE);
            BMI088TriggerSyncDebug(bmi088);
            return;
        }

        if (bmi088->trigger_state == BMI088_TRIGGER_STATE_TEMP) {
            if (BMI088TempRawFrameValid(bmi088, bmi088->temp_trig_rx + 2) != 0u) {
                memcpy(bmi088->temp_raw, bmi088->temp_trig_rx + 2, 2);
                BMI088DecodeTemp(bmi088, bmi088->temp_raw);
                bmi088->update_flag.temp = 1u;
            }
            bmi088->trigger_debug.temp_complete_count++;
            BMI088TriggerSyncDebug(bmi088);
            BMI088TriggerCompleteSample(bmi088);
            return;
        }

        return;
    }

    // 如果是加速度计的中断,则启动加速度计数据读取,并转换为实际值
    if (bmi088->update_flag.acc == 1) {
        for (uint8_t i = 0; i < 3; i++)
            bmi088->acc[i] = bmi088->BMI088_ACCEL_SEN * (float)(int16_t)(((bmi088->acc_raw[2 * i + 1]) << 8) | bmi088->acc_raw[2 * i]);
        bmi088->update_flag.acc = 0;
    }
    if (callback_time >= 500) {
        BMI088AccelRead(bmi088, BMI088_TEMP_M, bmi088->temp_raw, 2);
        int16_t temperate_raw_temp;
        temperate_raw_temp = (int16_t)((bmi088->temp_raw[0] << 3) | (bmi088->temp_raw[1] >> 5));
        if (temperate_raw_temp > 1023) temperate_raw_temp -= 2048;

        bmi088->temperature = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
        // 设置环境温度，仅在首次读取时赋值，并校验合法性（10~35℃），否则使用默认值25℃
        if (bmi088->ambient_temperature < 0) {
            if (bmi088->temperature >= 10.0f && bmi088->temperature <= 35.0f)
                bmi088->ambient_temperature = bmi088->temperature;
            else
                bmi088->ambient_temperature = BMI088_AMBIENT_TEMPERATURE;
        }
        callback_time = 0;
    }
    callback_time++;
}
extern osThreadId_t instaskHandle; // 本来是全局变量直接extern
static void BMI088GyroSPIFinishCallback(SPIInstance *spi)
{
    static BMI088Instance *bmi088 = NULL;
    bmi088                        = (BMI088Instance *)(spi->id);
    if (bmi088 == NULL)
        return;

    if (bmi088->work_mode == BMI088_DMA_PERIODIC_MODE && bmi088->dma_state == BMI088_DMA_STATE_GYRO) {
        if (BMI088GyroRawFrameValid(bmi088, bmi088->gyro_dma_rx + 1) == 0) {
            BMI088DMAFault(bmi088);
            bmi088->dma_start_pending = 1;
            return;
        }
        memcpy(bmi088->gyro_raw, bmi088->gyro_dma_rx + 1, 6);
        BMI088DecodeGyro(bmi088, bmi088->gyro_raw);
        BMI088RememberGyro(bmi088);
        bmi088->update_flag.gyro = 1;
        bmi088->dma_debug.gyro_complete_count++;
        BMI088DMASyncDebug(bmi088);

        if (bmi088->dma_read_temp != 0) {
            BMI088DMASetState(bmi088, BMI088_DMA_STATE_WAIT_TEMP);
            bmi088->dma_start_us = DWT_GetTimeline_us();
        } else {
            BMI088DMACompleteSample(bmi088);
        }
        return;
    }

    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE) {
        if (bmi088->trigger_state != BMI088_TRIGGER_STATE_GYRO)
            return;

        if (BMI088GyroRawFrameValid(bmi088, bmi088->gyro_trig_rx + 1) == 0u) {
            bmi088->trigger_gyro_valid = 0u;
            bmi088->trigger_gyro_pending = 1u;
            BMI088TriggerFault(bmi088);
            return;
        }
        memcpy(bmi088->gyro_raw, bmi088->gyro_trig_rx + 1, 6);
        BMI088DecodeGyro(bmi088, bmi088->gyro_raw);
        BMI088RememberGyro(bmi088);
        bmi088->update_flag.gyro = 1u;
        bmi088->trigger_gyro_valid = 1u;
        bmi088->trigger_start_us = 0u;
        bmi088->trigger_debug.gyro_complete_count++;
        BMI088TriggerSetState(bmi088, BMI088_TRIGGER_STATE_IDLE);
        BMI088TriggerSyncDebug(bmi088);
        return;
    }

    // 将陀螺仪数据转换为实际值
    for (uint8_t i = 0; i < 3; i++)
        bmi088->gyro[i] = bmi088->BMI088_GYRO_SEN * (float)(int16_t)(((bmi088->gyro_raw[2 * i + 1]) << 8) | bmi088->gyro_raw[2 * i]);
    bmi088->update_flag.gyro = 0;
    // 由于SPI速率为10.5Mbit/s，超出了BMI088的10Mbit/s限制，所以有些陀螺仪数据会出错
    // 有尝试调低速率，但是这样加速度计中断在开机几秒后便无法触发，原因未知
    ///@todo 找到原因降低速率，使数据恢复正常，可能原因是初始化需要较低的速率，之后可以拉满
    // 下面的函数用于过滤错误数据,无法完全过滤
    // begin
    for (uint8_t i = 0; i < 3; i++) {
        if (fabsf(bmi088->last_gyro[i] - bmi088->gyro[i]) > 10.0f)
            return;
    }
    for (uint8_t i = 0; i < 3; i++) {
        bmi088->last_gyro[i] = bmi088->gyro[i];
    }
    bmi088->gyro_guard_valid = 1u;
    // end
    bmi088->update_flag.imu_ready = 1;
    osThreadFlagsSet(instaskHandle, IMU_READY_FLAG); // 通知主线程IMU数据准备完毕（以陀螺仪中断为准 1000Hz）
}

static void BMI088AccINTCallback(GPIOInstance *gpio)
{
    static BMI088Instance *bmi088;
    bmi088 = (BMI088Instance *)(gpio->id);
    if (bmi088 == NULL)
        return;

    if (bmi088->work_mode == BMI088_DMA_PERIODIC_MODE) {
        BMI088DMAMarkAccDRDY(bmi088);
        return;
    }
    if (bmi088->work_mode != BMI088_BLOCK_TRIGGER_MODE)
        return;

    bmi088->trigger_acc_pending = 1u;
    BMI088TriggerMarkAccDRDY(bmi088);
}

static void BMI088GyroINTCallback(GPIOInstance *gpio)
{
    static BMI088Instance *bmi088 = NULL;
    bmi088                        = (BMI088Instance *)(gpio->id);
    if (bmi088 == NULL)
        return;

    if (bmi088->work_mode == BMI088_DMA_PERIODIC_MODE) {
        BMI088DMAMarkGyroDRDY(bmi088);
        bmi088->dma_start_pending = 1;
        return;
    }
    if (bmi088->work_mode != BMI088_BLOCK_TRIGGER_MODE)
        return;

    bmi088->trigger_gyro_pending = 1u;
    BMI088TriggerMarkGyroDRDY(bmi088);
}

// -------------------------以上为私有函数,private用于IT模式下的中断处理---------------------------------//

// -------------------------以下为私有函数,用于改变BMI088的配置--------------------------------//
static void BMI088SetMode(BMI088Instance *bmi088Instance, BMI088_Work_Mode_e mode)
{
    bmi088Instance->work_mode = mode;
    if (mode == BMI088_BLOCK_PERIODIC_MODE) {
        SPISetMode(bmi088Instance->spi_acc, SPI_BLOCK_MODE);
        SPISetMode(bmi088Instance->spi_gyro, SPI_BLOCK_MODE);
    } else if (mode == BMI088_BLOCK_TRIGGER_MODE) {
        SPISetMode(bmi088Instance->spi_acc, SPI_IT_MODE);
        SPISetMode(bmi088Instance->spi_gyro, SPI_IT_MODE);
        bmi088Instance->trigger_primed = 0u;
        BMI088TriggerSetState(bmi088Instance, BMI088_TRIGGER_STATE_IDLE);
    } else if (mode == BMI088_DMA_PERIODIC_MODE) {
        SPISetMode(bmi088Instance->spi_acc, SPI_DMA_MODE);
        SPISetMode(bmi088Instance->spi_gyro, SPI_DMA_MODE);
    }
}
// -------------------------以上为私有函数,用于改变BMI088的配置--------------------------------//

// -------------------------以下为公有函数,用于注册BMI088,标定和数据读取--------------------------------//

/**
 * @brief
 * @param bmi088
 * @return BMI088_Data_t
 */
//----------------------------------------------------------------BMI088数据获取----------------------------------------------------------------------------------//
uint8_t BMI088Acquire(BMI088Instance *bmi088, BMI088_Data_t *data_store)
{
    if (bmi088 == NULL)
        return 0;

    // 如果是blocking模式,则主动触发一次读取并返回数据
    if (bmi088->work_mode == BMI088_BLOCK_PERIODIC_MODE) {
        return BMI088AcquireBlocking(bmi088, data_store);
    }
    // DMA周期模式: 只有DMA链路提交完整acc+gyro样本后才返回1, 未完成时不刷新data_store.
    if (bmi088->work_mode == BMI088_DMA_PERIODIC_MODE) {
        if (BMI088DMAAvailable(bmi088) == 0)
            return BMI088AcquireBlocking(bmi088, data_store);

        if (bmi088->dma_primed == 0) {
            uint8_t ok = BMI088AcquireBlocking(bmi088, data_store);
            bmi088->dma_primed = ok;
            bmi088->dma_start_pending = 1;
            (void)BMI088StartDMASample(bmi088);
            return ok;
        }

        BMI088DMAPump(bmi088);

        if (BMI088HandleDMATimeout(bmi088, data_store) != 0)
            return 1;

        if (bmi088->dma_sample_ready != 0) {
            bmi088->dma_sample_ready = 0;
            BMI088DMASetState(bmi088, BMI088_DMA_STATE_IDLE);
            BMI088CopyData(bmi088, data_store);
            (void)BMI088StartDMASample(bmi088);
            return 1;
        }

        if (bmi088->dma_state == BMI088_DMA_STATE_IDLE) {
            if (bmi088->dma_start_pending != 0)
                bmi088->dma_start_pending = 0;
            (void)BMI088StartDMASample(bmi088);
        }
        return 0;
    }

    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE) {
        if (bmi088->trigger_primed == 0u) {
            uint8_t ok = BMI088AcquireBlocking(bmi088, data_store);
            bmi088->trigger_primed = ok;
            return ok;
        }

        BMI088HandleTriggerTimeout(bmi088);
        BMI088TriggerPump(bmi088);

        if (bmi088->trigger_sample_ready != 0u || bmi088->update_flag.imu_ready == 1u) {
            bmi088->trigger_sample_ready = 0u;
            bmi088->update_flag.imu_ready = 0u;
            BMI088CopyData(bmi088, data_store);
            BMI088TriggerSetState(bmi088, BMI088_TRIGGER_STATE_IDLE);
            BMI088TriggerPump(bmi088);
            return 1;
        }
        return 0;
    }

    // 如果数据还没准备好,则返回空数据
    if (bmi088->update_flag.imu_ready == 0) {
        data_store = NULL;
        return 0;
    }
    return 255;
}

/**
 * @brief :  读取BMI088的IMU更新完成标志位
 * @param *bmi088
 * @return 1 数据准备完毕 0 没有数据
 */
uint8_t BMI088Acquire_IT_Status(BMI088Instance *bmi088)
{
    // 只有中断才能读取标志位
    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE &&
        (bmi088->trigger_sample_ready != 0u || bmi088->update_flag.imu_ready == 1u)) {
        bmi088->trigger_sample_ready = 0u;
        bmi088->update_flag.imu_ready = 0;
        return 1;
    } else
        return 0;
}
#pragma message ("REMEMBER TO CHANGE CALI PARAMETERS IF YOU CHOOSE NOT TO CALIBRATE ONLINE(parameters in robot_def.h)")

static void BMI088ApplyInstallMatrixForCali(const BMI088_Data_t *raw,
                                            float gyro_body[3],
                                            float acc_body[3])
{
    for (uint8_t i = 0; i < 3; i++) {
        gyro_body[i] = raw->gyro[0] * k_bmi088_cali_install_matrix[i][0] +
                       raw->gyro[1] * k_bmi088_cali_install_matrix[i][1] +
                       raw->gyro[2] * k_bmi088_cali_install_matrix[i][2];

        acc_body[i] = raw->acc[0] * k_bmi088_cali_install_matrix[i][0] +
                      raw->acc[1] * k_bmi088_cali_install_matrix[i][1] +
                      raw->acc[2] * k_bmi088_cali_install_matrix[i][2];
    }
}

static void BMI088UpdateCaliDebugOffsets(const BMI088Instance *bmi088)
{
    if (bmi088 == NULL) {
        return;
    }

    g_bmi088_cali_debug.temperature = bmi088->temperature;
    g_bmi088_cali_debug.target_temperature = BMI088_TEMP_TARGET_C;
    g_bmi088_cali_debug.ready_low_temperature = BMI088_TEMP_READY_LOW_C;
    g_bmi088_cali_debug.ready_high_temperature = BMI088_TEMP_READY_HIGH_C;
    for (uint8_t i = 0; i < 3; i++) {
        g_bmi088_cali_debug.gyro_offset[i] = bmi088->gyro_offset[i];
        g_bmi088_cali_debug.acc_offset[i] = bmi088->acc_offset[i];
    }
}

static void BMI088SaveCaliResult(const BMI088Instance *bmi088, uint32_t sample_count, uint8_t valid)
{
    if (bmi088 == NULL) {
        return;
    }

    g_bmi088_cali_result.temperature = bmi088->temperature;
    g_bmi088_cali_result.sample_count = sample_count;
    g_bmi088_cali_result.valid = valid;
    for (uint8_t i = 0; i < 3; i++) {
        g_bmi088_cali_result.gyro_offset[i] = bmi088->gyro_offset[i];
        g_bmi088_cali_result.acc_offset[i] = bmi088->acc_offset[i];
    }
}

static void BMI088WaitTemperatureReadyForCali(BMI088Instance *bmi088)
{
    BMI088_Data_t bmi088_data = {0};

    g_bmi088_cali_debug.temp_ready = 0u;
    g_bmi088_cali_debug.stable_ms = 0u;
    g_bmi088_cali_debug.sample_count = 0u;
    BMI088UpdateCaliDebugOffsets(bmi088);

    while (bmi088->temperature < BMI088_TEMP_READY_LOW_C) {
        if (BMI088Acquire(bmi088, &bmi088_data) != 0u) {
            BMI088_temp_control(bmi088);
            g_bmi088_cali_debug.temperature = bmi088->temperature;
        }
        DWT_Delay(0.001f);
    }

    g_bmi088_cali_debug.temp_ready = 1u;
    g_bmi088_cali_debug.stable_ms = 1u;
}

/**
 * @brief BMI088 六轴零漂标定
 * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
 * @param _bmi088 待标定的BMI088实例
 */
// BMI088CalibrateIMU 根据 _bmi088->cali_mode 选择在线标定或预设数据导入。
void BMI088CalibrateIMU(BMI088Instance *_bmi088)
{
    if (_bmi088 == NULL) {
        return;
    }

    g_bmi088_cali_debug.cali_running = 1u;
    g_bmi088_cali_debug.cali_done = 0u;
    g_bmi088_cali_debug.sample_count = 0u;
    g_bmi088_cali_debug.stable_ms = 0u;
    g_bmi088_cali_debug.temp_ready = 0u;
    //BMI088UpdateCaliDebugOffsets(_bmi088); //debug的时候看bias用的

    if (_bmi088->cali_mode == BMI088_CALIBRATE_ONLINE_MODE) {
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        float acc_sum[3] = {0.0f, 0.0f, 0.0f};
        uint32_t cali_time_count = 0u;

        BMI088WaitTemperatureReadyForCali(_bmi088);

        while (cali_time_count < BMI088_ONLINE_CALI_SAMPLE_CNT) {
            BMI088_Data_t bmi088_data = {0};
            float gyro_body[3] = {0.0f, 0.0f, 0.0f};
            float acc_body[3] = {0.0f, 0.0f, 0.0f};

            if (cali_time_count % 500u == 0u) {
                buzzer_one_note(1047, 0.2f);
            }

            if (BMI088Acquire(_bmi088, &bmi088_data) != 0u) {
                BMI088_temp_control(_bmi088);
                BMI088ApplyInstallMatrixForCali(&bmi088_data, gyro_body, acc_body);
                for (uint8_t i = 0; i < 3; i++) {
                    gyro_sum[i] += gyro_body[i];
                    acc_sum[i] += acc_body[i];
                }
                cali_time_count++;
                g_bmi088_cali_debug.temperature = _bmi088->temperature;
                g_bmi088_cali_debug.sample_count = cali_time_count;
            }

            DWT_Delay(0.001f);
        }

        for (uint8_t i = 0; i < 3; i++) {
            const float gyro_mean = gyro_sum[i] / (float)BMI088_ONLINE_CALI_SAMPLE_CNT;
            const float acc_mean = acc_sum[i] / (float)BMI088_ONLINE_CALI_SAMPLE_CNT;
            const float gravity_axis = (i == 2u) ? BMI088_GRAVITY_MPS2 : 0.0f;

            _bmi088->gyro_offset[i] = gyro_mean;
            _bmi088->acc_offset[i] = acc_mean - gravity_axis;
            g_bmi088_cali_debug.gyro_mean[i] = gyro_mean;
            g_bmi088_cali_debug.acc_mean[i] = acc_mean;
        }
    }
    // 导入数据
    else if (_bmi088->cali_mode == BMI088_LOAD_PRE_CALI_MODE) {
        _bmi088->gyro_offset[0] = BMI088_PRE_CALI_GYRO_X_OFFSET;
        _bmi088->gyro_offset[1] = BMI088_PRE_CALI_GYRO_Y_OFFSET;
        _bmi088->gyro_offset[2] = BMI088_PRE_CALI_GYRO_Z_OFFSET;
        _bmi088->acc_offset[0]  = BMI088_PRE_CALI_ACC_X_OFFSET;
        _bmi088->acc_offset[1]  = BMI088_PRE_CALI_ACC_Y_OFFSET;
        _bmi088->acc_offset[2]  = BMI088_PRE_CALI_ACC_Z_OFFSET;
    }
    //-----------debug用的---------------------------------
    // BMI088UpdateCaliDebugOffsets(_bmi088);
    // BMI088SaveCaliResult(_bmi088, g_bmi088_cali_debug.sample_count, 1u);
    // g_bmi088_cali_debug.cali_running = 0u;
    // g_bmi088_cali_debug.cali_done = 1u;
}

BMI088Instance *BMI088Register(BMI088_Init_Config_s *config)
{
    // 申请内存
    BMI088Instance *bmi088_instance = (BMI088Instance *)malloc(sizeof(BMI088Instance));
    if (bmi088_instance == NULL)
        while (1)
            ;
    memset(bmi088_instance, 0, sizeof(BMI088Instance));
    bmi088_instance->work_mode = config->work_mode;
    // 从右向左赋值,让bsp instance保存指向bmi088_instance的指针(父指针),便于在底层中断中访问bmi088_instance
    config->acc_int_config.id =
        config->gyro_int_config.id =
            config->spi_acc_config.id =
                config->spi_gyro_config.id =
                    config->heat_pwm_config.id = bmi088_instance;

    // 根据参数选择工作模式
    if (config->work_mode == BMI088_BLOCK_PERIODIC_MODE) {
        config->spi_acc_config.spi_work_mode  = SPI_BLOCK_MODE;
        config->spi_gyro_config.spi_work_mode = SPI_BLOCK_MODE;
        // callbacks are all NULL
    } else if (config->work_mode == BMI088_BLOCK_TRIGGER_MODE) {
        config->spi_acc_config.spi_work_mode  = SPI_IT_MODE;
        config->spi_gyro_config.spi_work_mode = SPI_IT_MODE;
        config->spi_acc_config.callback             = BMI088AccSPIFinishCallback;
        config->spi_gyro_config.callback            = BMI088GyroSPIFinishCallback;
        config->acc_int_config.gpio_model_callback  = BMI088AccINTCallback;
        config->gyro_int_config.gpio_model_callback = BMI088GyroINTCallback;
        bmi088_instance->acc_int                    = GPIORegister(&config->acc_int_config);
        bmi088_instance->gyro_int                   = GPIORegister(&config->gyro_int_config);
    } else if (config->work_mode == BMI088_DMA_PERIODIC_MODE) {
        config->spi_acc_config.spi_work_mode  = SPI_DMA_MODE;
        config->spi_gyro_config.spi_work_mode = SPI_DMA_MODE;
        // 设置回调函数
        config->spi_acc_config.callback             = BMI088AccSPIFinishCallback;
        config->spi_gyro_config.callback            = BMI088GyroSPIFinishCallback;
        config->acc_int_config.gpio_model_callback  = BMI088AccINTCallback;
        config->gyro_int_config.gpio_model_callback = BMI088GyroINTCallback;
        bmi088_instance->acc_int                    = GPIORegister(&config->acc_int_config);
        bmi088_instance->gyro_int                   = GPIORegister(&config->gyro_int_config);
    }
    // 注册实例
    bmi088_instance->spi_acc  = SPIRegister(&config->spi_acc_config);
    bmi088_instance->spi_gyro = SPIRegister(&config->spi_gyro_config);
    bmi088_instance->heat_pwm = PWMRegister(&config->heat_pwm_config);

    bmi088_instance->heat_pid = PIDRegister(&config->heat_pid_config);

    bmi088_instance->ambient_temperature = 25; // 环境温度初值
    DWT_GetDeltaT(&bmi088_instance->bias_dwt_cnt);
    // 初始化时使用阻塞模式
    BMI088SetMode(bmi088_instance, BMI088_BLOCK_PERIODIC_MODE);
    // 初始化acc和gyro
    BMI088_ERORR_CODE_e error = BMI088_NO_ERROR;
    do {
        error = BMI088_NO_ERROR;
        error |= BMI088AccelInit(bmi088_instance);
        error |= BMI088GyroInit(bmi088_instance);
        // 可以增加try out times,超出次数则返回错误
    } while (error != 0);
    bmi088_instance->cali_mode = config->cali_mode;
    BMI088CalibrateIMU(bmi088_instance);               // 标定acc和gyro
    BMI088SetMode(bmi088_instance, config->work_mode); // 恢复工作模式
    return bmi088_instance;
}
