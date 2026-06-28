#include <memory.h>
#include <stdlib.h>

#include "super_cap.h"

#define SUPERCAP_SEND_PERIOD_1MS_TICKS 10U

_Static_assert(sizeof(SuperCap_Tx_Data_s) == SUPERCAP_TX_FRAME_LEN,
               "SuperCap tx frame must be 8 bytes");
_Static_assert(sizeof(SuperCap_Rx_Data_s) == SUPERCAP_RX_FRAME_LEN,
               "SuperCap rx frame must be 8 bytes");

static SuperCapInstance *supercap = NULL;

static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    const float span = x_max - x_min;
    const uint32_t max_int = (1UL << bits) - 1UL;

    return ((float)x_int) * span / ((float)max_int) + x_min;
}

static float CapEnergyTransform(uint8_t raw_energy)
{
    float raw_energy_f = (float)raw_energy;
    const float uvp_square = SOFTWARE_UVP_VCAP * SOFTWARE_UVP_VCAP;
    const float ovp_square = SOFTWARE_OVP_VCAP * SOFTWARE_OVP_VCAP;
    float vcap;
    float energy;

    if (raw_energy_f > 100.0f) {
        raw_energy_f = 100.0f;
    }

    vcap = SOFTWARE_UVP_VCAP + raw_energy_f * SUPERCAP_AVAILABLE_VOLTAGE / 100.0f;
    energy = (vcap * vcap - uvp_square) * 100.0f / (ovp_square - uvp_square);

    if (energy < 0.0f) {
        energy = 0.0f;
    } else if (energy > 100.0f) {
        energy = 100.0f;
    }

    return energy;
}

static void SuperCapClearFeedback(SuperCapInstance *instance)
{
    if (instance == NULL) {
        return;
    }

    memset(&instance->rx_data, 0, sizeof(instance->rx_data));
    instance->chassis_real_power = 0.0f;
    instance->real_energy = 0.0f;
    instance->battery_voltage = 0.0f;
}

static void SuperCapRxCallback(CANInstance *_instance)
{
    if (_instance == NULL || _instance->id == NULL) {
        return;
    }

    SuperCapInstance *instance = (SuperCapInstance *)_instance->id;
    if (_instance->rx_len < SUPERCAP_RX_FRAME_LEN) {
        return;
    }

    if (instance->daemon_instance != NULL) {
        DaemonReload(instance->daemon_instance);
    }

    instance->rx_data.ready_flag = _instance->rx_buff[0];
    instance->rx_data.state = _instance->rx_buff[1];
    instance->rx_data.energy = _instance->rx_buff[2];
    instance->rx_data.chassis_real_power =
        (uint16_t)_instance->rx_buff[3] | ((uint16_t)_instance->rx_buff[4] << 8);
    instance->rx_data.bat_voltage = _instance->rx_buff[5];
    instance->rx_data.bat_power = _instance->rx_buff[6];
    instance->rx_data.reserved = _instance->rx_buff[7];

    instance->chassis_real_power = uint_to_float(instance->rx_data.chassis_real_power, 0.0f, 512.0f, 16U);
    instance->real_energy = CapEnergyTransform(instance->rx_data.energy);
    instance->battery_voltage = (float)instance->rx_data.bat_voltage / 10.0f;
}

static void SuperCapLostCallback(void *supercap_ptr)
{
    SuperCapClearFeedback((SuperCapInstance *)supercap_ptr);
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config)
{
    if (config == NULL) {
        return NULL;
    }

    if (supercap == NULL) {
        supercap = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
        if (supercap == NULL) {
            return NULL;
        }
        memset(supercap, 0, sizeof(SuperCapInstance));
        supercap->tx_data.enable_flag = SUPERCAP_ENABLE;
        supercap->tx_data.charge_flag = 1U;

        config->can_config.tx_id = 0x180;
        config->can_config.rx_id = 0x185;
        config->can_config.can_module_callback = SuperCapRxCallback;
        config->can_config.id = (void *)supercap;
        supercap->can_instance = CANRegister(&config->can_config);

        Daemon_Init_Config_s daemon_config = {
            .callback = SuperCapLostCallback,
            .owner_id = (void *)supercap,
            .reload_count = 100,
            .init_count = 200,
        };
        supercap->daemon_instance = DaemonRegister(&daemon_config);
    }

    return supercap;
}

void SuperCapEnable(SuperCapInstance *instance)
{
    if (instance == NULL) {
        return;
    }

    instance->tx_data.enable_flag = SUPERCAP_ENABLE;
}

void SuperCapDisable(SuperCapInstance *instance)
{
    if (instance == NULL) {
        return;
    }

    instance->tx_data.enable_flag = SUPERCAP_DISABLE;
}

void SuperCapSetPowerLimit(SuperCapInstance *instance, uint8_t power_limit)
{
    if (instance == NULL) {
        return;
    }

    instance->tx_data.power_limit = power_limit;
}

void SuperCapTask(void)
{
    static uint8_t counter = 0U;

    if (supercap == NULL || supercap->can_instance == NULL) {
        return;
    }

    if (counter == 0U) {
        supercap->can_instance->tx_buff[0] = supercap->tx_data.enable_flag;
        supercap->can_instance->tx_buff[1] = supercap->tx_data.charge_flag;
        supercap->can_instance->tx_buff[2] = supercap->tx_data.power_limit;
        supercap->can_instance->tx_buff[3] = supercap->tx_data.reserved[0];
        supercap->can_instance->tx_buff[4] = supercap->tx_data.reserved[1];
        supercap->can_instance->tx_buff[5] = supercap->tx_data.reserved[2];
        supercap->can_instance->tx_buff[6] = supercap->tx_data.reserved[3];
        supercap->can_instance->tx_buff[7] = supercap->tx_data.reserved[4];
        (void)CANTransmit(supercap->can_instance, 1.0f);
    }

    counter++;
    if (counter >= SUPERCAP_SEND_PERIOD_1MS_TICKS) {
        counter = 0U;
    }
}

uint8_t SuperCapIsOnline(SuperCapInstance *instance)
{
    if (instance == NULL || instance->daemon_instance == NULL) {
        return 0U;
    }

    return DaemonIsOnline(instance->daemon_instance);
}

float SuperCapGetBatteryVoltage(SuperCapInstance *instance)
{
    if (instance == NULL) {
        return 0.0f;
    }

    return instance->battery_voltage;
}

float SuperCapGetChassisRealPower(SuperCapInstance *instance)
{
    if (instance == NULL) {
        return 0.0f;
    }

    return instance->chassis_real_power;
}

uint8_t SuperCapGetCapEnergy(SuperCapInstance *instance)
{
    if (instance == NULL) {
        return 0U;
    }

    return instance->rx_data.energy;
}

uint8_t SuperCapGetReadyFlag(SuperCapInstance *instance)
{
    if (instance == NULL) {
        return 0U;
    }

    return instance->rx_data.ready_flag;
}
