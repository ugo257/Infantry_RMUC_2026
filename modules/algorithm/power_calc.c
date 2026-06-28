#include "power_calc.h"

#include <math.h>
#include <stddef.h>

#define POWER_CURRENT_LIMIT_CMD 15000.0f

float k1                   = 1.15041374e-07f;
float k2                   = 1.79e-7f;
float constant             = 0.8f;
float torque_coefficient   = 2.94974577124e-06f;
float machine_power        = 0.0f;
float current_power        = 0.0f;
float speed_power          = 0.0f;
float motor_current_output = 0.0f;
float Power_Input          = 0.0f;

float reduction_ratio = 1.0f;
float total_power = 0.0f;
uint16_t max_power = 0U;
float measured_chassis_power_w = 0.0f;
uint8_t measured_chassis_power_valid = 0U;

float text_k1 = 0.0f;
float text_c = 0.0f;
float text_k2 = 0.0f;
float give_power = 0.0f;
float power_scale = 1.0f;
float torque_output = 0.0f;
float temp = 0.0f;

static float PowerControlClampCurrent(float current)
{
    if (isfinite(current) == 0) {
        return 0.0f;
    }
    if (current > POWER_CURRENT_LIMIT_CMD) {
        return POWER_CURRENT_LIMIT_CMD;
    }
    if (current < -POWER_CURRENT_LIMIT_CMD) {
        return -POWER_CURRENT_LIMIT_CMD;
    }
    return current;
}

void PowerControlupdate(uint16_t max_power_init, float reduction_ratio_init)
{
    max_power = max_power_init;
    if (reduction_ratio_init != 0.0f && isfinite(reduction_ratio_init) != 0) {
        reduction_ratio = reduction_ratio_init;
    }
}

void PowerControlFeedbackUpdate(float measured_power_w, uint8_t measured_power_valid)
{
    if (measured_power_valid != 0U &&
        isfinite(measured_power_w) != 0 &&
        measured_power_w > 0.0f) {
        measured_chassis_power_w = measured_power_w;
        measured_chassis_power_valid = 1U;
    } else {
        measured_chassis_power_w = 0.0f;
        measured_chassis_power_valid = 0U;
    }
}

float PowerInputCalc(float motor_speed, float motor_current)
{
    const float current = PowerControlClampCurrent(motor_current);
    const float power_input =
        current * torque_coefficient * motor_speed +
        k1 * motor_speed * motor_speed +
        k2 * current * current +
        constant;

    Power_Input = power_input;
    machine_power = current * torque_coefficient * motor_speed;
    speed_power = k1 * motor_speed * motor_speed;
    current_power = k2 * current * current + constant;
    text_c = machine_power;
    text_k1 = speed_power;
    text_k2 = k2 * current * current;
    return power_input;
}

float TotalPowerCalc(float input_power[])
{
    total_power = 0.0f;
    if (input_power == NULL) {
        return total_power;
    }

    for (uint8_t i = 0U; i < 4U; ++i) {
        if (isfinite(input_power[i]) != 0 && input_power[i] > 0.0f) {
            total_power += input_power[i];
        }
    }
    return total_power;
}

float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current)
{
    float current = PowerControlClampCurrent(motor_current);

    if (max_power == 0U || isfinite(total_power) == 0 || total_power <= 0.0f) {
        power_scale = 1.0f;
        give_power = motor_power;
        motor_current_output = current;
        return current;
    }

    if (total_power > (float)max_power) {
        float discriminant;
        float a;
        float b;
        float c;

        power_scale = (float)max_power / total_power;
        if (power_scale < 0.0f) {
            power_scale = 0.0f;
        } else if (power_scale > 1.0f) {
            power_scale = 1.0f;
        }
        give_power = motor_power * power_scale;

        if (motor_power < 0.0f) {
            motor_current_output = current;
            return motor_current_output;
        }

        a = k2;
        b = motor_speed * torque_coefficient;
        c = k1 * motor_speed * motor_speed - give_power + constant;
        discriminant = b * b - 4.0f * a * c;

        if (a <= 0.0f ||
            isfinite(discriminant) == 0 ||
            discriminant < 0.0f) {
            motor_current_output = PowerControlClampCurrent(current * power_scale);
            return motor_current_output;
        }

        if (current > 0.0f) {
            temp = (-b + sqrtf(discriminant)) / (2.0f * a);
        } else {
            temp = (-b - sqrtf(discriminant)) / (2.0f * a);
        }
        motor_current_output = PowerControlClampCurrent(temp);
        return motor_current_output;
    }

    power_scale = 1.0f;
    give_power = motor_power;
    motor_current_output = current;
    return current;
}
