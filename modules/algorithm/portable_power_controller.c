#include "portable_power_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

const PPC_Config PPC_DEFAULT_CONFIG_6MOTOR = {
    3.74164882e-7f,
    1.54264578e-4f,
    1.65318449e-7f,
    5.89318230f,
    1.0f,
    6u,
};

const PPC_Config PPC_DEFAULT_CONFIG_4MOTOR = {
    2.37354124e-7f,
    8.13966963e-4f,
    9.83084937e-8f,
    11.0303864f,
    1.0f,
    4u,
};

static float ppc_absf(float value)
{
    return value >= 0.0f ? value : -value;
}

static float ppc_clampf(float value, float min_value, float max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

static float ppc_clamp_abs(float value, float limit)
{
    float abs_limit = ppc_absf(limit);
    return ppc_clampf(value, -abs_limit, abs_limit);
}

static float ppc_same_sign_root(float root_a, float root_b, float cmd)
{
    if (cmd >= 0.0f)
        return root_a > root_b ? root_a : root_b;
    return root_a < root_b ? root_a : root_b;
}

static void ppc_rls_fill_identity(float *matrix, float scale)
{
    uint8_t i;

    if (matrix == NULL)
        return;

    for (i = 0u; i < 9u; i++)
        matrix[i] = 0.0f;

    matrix[0] = scale;
    matrix[4] = scale;
    matrix[8] = scale;
}

static float ppc_rls_loss_estimate(const PPC_RLS *rls, const float *phi)
{
    if (rls == NULL || phi == NULL)
        return 0.0f;

    return rls->theta[0] * phi[0] +
           rls->theta[1] * phi[1] +
           rls->theta[2] * phi[2];
}

static float ppc_rls_mech_power_sum(const PPC_Config *cfg,
                                    const PPC_MotorState *motor,
                                    uint8_t motor_count)
{
    const PPC_Config *c = cfg != NULL ? cfg : &PPC_DEFAULT_CONFIG_6MOTOR;
    float mech_power = 0.0f;
    uint8_t i;

    if (motor == NULL)
        return 0.0f;

    for (i = 0u; i < motor_count; i++) {
        mech_power += c->mech_k * motor[i].current * motor[i].speed;
    }

    return mech_power;
}

static void ppc_rls_build_phi(const PPC_MotorState *motor,
                              uint8_t motor_count,
                              uint8_t enable_k3_update,
                              float *phi)
{
    uint8_t i;

    if (motor == NULL || phi == NULL)
        return;

    phi[0] = 0.0f;
    phi[1] = 0.0f;
    phi[2] = enable_k3_update != 0u ? 1.0f : 0.0f;

    for (i = 0u; i < motor_count; i++) {
        phi[0] += ppc_absf(motor[i].speed);
        phi[1] += motor[i].current * motor[i].current;
    }
}

static void ppc_rls_apply_to_config(const PPC_RLS *rls, PPC_Config *cfg)
{
    if (rls == NULL || cfg == NULL)
        return;

    cfg->k1 = rls->theta[0] > 0.0f ? rls->theta[0] : cfg->k1;
    cfg->k2 = rls->theta[1] > 0.0f ? rls->theta[1] : cfg->k2;

    if (rls->enable_k3_update != 0u && rls->theta[2] >= 0.0f)
        cfg->k3 = rls->theta[2];
}

void PPC_Init(PPC_Config *cfg)
{
    if (cfg == NULL)
        return;

    if (cfg->motor_count == 0u || cfg->motor_count > PPC_MAX_MOTOR_COUNT)
        cfg->motor_count = PPC_DEFAULT_CONFIG_6MOTOR.motor_count;
    if (cfg->mech_k <= 0.0f)
        cfg->mech_k = PPC_DEFAULT_CONFIG_6MOTOR.mech_k;
    if (cfg->k1 < 0.0f)
        cfg->k1 = PPC_DEFAULT_CONFIG_6MOTOR.k1;
    if (cfg->k2 <= 0.0f)
        cfg->k2 = PPC_DEFAULT_CONFIG_6MOTOR.k2;
    if (cfg->k3 < 0.0f)
        cfg->k3 = PPC_DEFAULT_CONFIG_6MOTOR.k3;
    if (cfg->power_margin < 0.0f)
        cfg->power_margin = 0.0f;
}

void PPC_RLS_Init(PPC_RLS *rls,
                  const PPC_Config *cfg,
                  float lambda,
                  float delta,
                  uint8_t enable_k3_update)
{
    const PPC_Config *c = cfg != NULL ? cfg : &PPC_DEFAULT_CONFIG_6MOTOR;

    if (rls == NULL)
        return;

    memset(rls, 0, sizeof(*rls));

    rls->lambda = (lambda > 0.0f && lambda <= 1.0f) ? lambda : 0.9999f;
    rls->delta = delta > 0.0f ? delta : 1.0f;
    rls->enable_k3_update = enable_k3_update != 0u ? 1u : 0u;
    rls->theta[0] = c->k1;
    rls->theta[1] = c->k2;
    rls->theta[2] = c->k3;
    ppc_rls_fill_identity(rls->p, rls->delta);
    rls->initialized = 1u;
}

void PPC_RLS_Reset(PPC_RLS *rls)
{
    if (rls == NULL)
        return;

    ppc_rls_fill_identity(rls->p, rls->delta > 0.0f ? rls->delta : 1.0f);
    rls->update_count = 0u;
    rls->last_loss_measured = 0.0f;
    rls->last_loss_estimated = 0.0f;
    rls->last_residual = 0.0f;
    rls->last_phi[0] = 0.0f;
    rls->last_phi[1] = 0.0f;
    rls->last_phi[2] = 0.0f;
}

uint8_t PPC_RLS_Update(const PPC_Config *cfg,
                       const PPC_MotorState *motor,
                       float measured_total_power,
                       PPC_RLS *rls,
                       PPC_Config *out_cfg)
{
    const PPC_Config *c = cfg != NULL ? cfg : &PPC_DEFAULT_CONFIG_6MOTOR;
    uint8_t motor_count;
    float phi[3];
    float p_phi[3];
    float denom;
    float gain[3];
    float predicted;
    float residual;
    float measured_loss;
    float lambda;
    uint8_t i;
    uint8_t j;

    if (motor == NULL || rls == NULL || rls->initialized == 0u)
        return 0u;

    motor_count = c->motor_count;
    if (motor_count == 0u || motor_count > PPC_MAX_MOTOR_COUNT)
        motor_count = PPC_MAX_MOTOR_COUNT;

    lambda = (rls->lambda > 0.0f && rls->lambda <= 1.0f) ? rls->lambda : 0.9999f;

    ppc_rls_build_phi(motor, motor_count, rls->enable_k3_update, phi);
    measured_loss = measured_total_power - ppc_rls_mech_power_sum(c, motor, motor_count);
    predicted = ppc_rls_loss_estimate(rls, phi);

    for (i = 0u; i < 3u; i++) {
        p_phi[i] = 0.0f;
        for (j = 0u; j < 3u; j++) {
            p_phi[i] += rls->p[i * 3u + j] * phi[j];
        }
    }

    denom = lambda;
    for (i = 0u; i < 3u; i++) {
        denom += phi[i] * p_phi[i];
    }

    if (ppc_absf(denom) < 1.0e-12f)
        return 0u;

    for (i = 0u; i < 3u; i++) {
        gain[i] = p_phi[i] / denom;
    }

    residual = measured_loss - predicted;

    for (i = 0u; i < 3u; i++) {
        if (i == 2u && rls->enable_k3_update == 0u)
            continue;
        rls->theta[i] += gain[i] * residual;
    }

    {
        float new_p[9];
        for (i = 0u; i < 3u; i++) {
            for (j = 0u; j < 3u; j++) {
                float value = rls->p[i * 3u + j] - gain[i] * p_phi[j];
                new_p[i * 3u + j] = value / lambda;
            }
        }
        memcpy(rls->p, new_p, sizeof(new_p));
    }

    if (rls->theta[0] < 1.0e-8f)
        rls->theta[0] = 1.0e-8f;
    if (rls->theta[1] < 1.0e-8f)
        rls->theta[1] = 1.0e-8f;
    if (rls->enable_k3_update != 0u && rls->theta[2] < 0.0f)
        rls->theta[2] = 0.0f;

    rls->last_phi[0] = phi[0];
    rls->last_phi[1] = phi[1];
    rls->last_phi[2] = phi[2];
    rls->last_loss_measured = measured_loss;
    rls->last_loss_estimated = ppc_rls_loss_estimate(rls, phi);
    rls->last_residual = residual;
    rls->update_count++;

    if (out_cfg != NULL)
        ppc_rls_apply_to_config(rls, out_cfg);

    return 1u;
}

float PPC_EstimateMotorPower(const PPC_Config *cfg,
                             float speed,
                             float current,
                             uint8_t motor_count)
{
    const PPC_Config *c = cfg != NULL ? cfg : &PPC_DEFAULT_CONFIG_6MOTOR;
    float count = motor_count == 0u ? (float)c->motor_count : (float)motor_count;

    return c->mech_k * current * speed +
           c->k1 * ppc_absf(speed) +
           c->k2 * current * current +
           c->k3 / count;
}

float PPC_EstimateTotalPower(const PPC_Config *cfg,
                             const PPC_MotorState *motor)
{
    const PPC_Config *c = cfg != NULL ? cfg : &PPC_DEFAULT_CONFIG_6MOTOR;
    uint8_t i;
    float power = 0.0f;

    if (motor == NULL)
        return 0.0f;

    for (i = 0u; i < c->motor_count && i < PPC_MAX_MOTOR_COUNT; i++) {
        power += PPC_EstimateMotorPower(c, motor[i].speed, motor[i].current, c->motor_count);
    }

    return power;
}

void PPC_LimitCurrent(const PPC_Config *cfg,
                      const PPC_MotorCmd *cmd,
                      float power_limit,
                      PPC_Result *result)
{
    const PPC_Config *c = cfg != NULL ? cfg : &PPC_DEFAULT_CONFIG_6MOTOR;
    uint8_t motor_count = c->motor_count;
    float cmd_current[PPC_MAX_MOTOR_COUNT];
    float cmd_power[PPC_MAX_MOTOR_COUNT];
    float weight[PPC_MAX_MOTOR_COUNT];
    float alloc_power[PPC_MAX_MOTOR_COUNT];
    float available_power;
    float positive_power_sum = 0.0f;
    float error_sum = 0.0f;
    float total_before = 0.0f;
    float total_after = 0.0f;
    uint8_t i;

    if (cmd == NULL || result == NULL)
        return;

    memset(result, 0, sizeof(*result));

    if (motor_count == 0u || motor_count > PPC_MAX_MOTOR_COUNT)
        motor_count = PPC_MAX_MOTOR_COUNT;

    available_power = power_limit - c->power_margin;
    if (available_power < 0.0f)
        available_power = 0.0f;

    for (i = 0u; i < motor_count; i++) {
        if (cmd[i].online == 0u) {
            cmd_current[i] = 0.0f;
            cmd_power[i] = 0.0f;
            weight[i] = 0.0f;
            continue;
        }

        cmd_current[i] = ppc_clamp_abs(cmd[i].current_cmd, cmd[i].current_limit);
        cmd_power[i] = PPC_EstimateMotorPower(c, cmd[i].speed, cmd_current[i], motor_count);
        total_before += cmd_power[i];

        if (cmd_power[i] > 0.0f) {
            positive_power_sum += cmd_power[i];
            error_sum += ppc_absf(cmd[i].target_speed - cmd[i].speed);
        }
    }

    if (total_before <= available_power) {
        for (i = 0u; i < motor_count; i++) {
            result->current_out[i] = cmd_current[i];
        }
        result->power_before = total_before;
        result->power_after = total_before;
        result->power_limit_used = available_power;
        result->scale = 1.0f;
        result->limited = 0u;
        return;
    }

    for (i = 0u; i < motor_count; i++) {
        if (cmd[i].online == 0u || cmd_power[i] <= 0.0f) {
            result->current_out[i] = cmd_current[i];
            continue;
        }

        if (error_sum > 1.0e-6f) {
            weight[i] = ppc_absf(cmd[i].target_speed - cmd[i].speed) / error_sum;
        } else if (positive_power_sum > 1.0e-6f) {
            weight[i] = cmd_power[i] / positive_power_sum;
        } else {
            weight[i] = 1.0f / (float)motor_count;
        }

        alloc_power[i] = available_power * weight[i];
        {
            float constant_loss = c->k1 * ppc_absf(cmd[i].speed) + c->k3 / (float)motor_count;
            float a = c->k2;
            float b = c->mech_k * cmd[i].speed;
            float d = b * b - 4.0f * a * (constant_loss - alloc_power[i]);

            if (d > 0.0f && a > 1.0e-12f) {
                float sqrt_d = sqrtf(d);
                float root_a = (-b + sqrt_d) / (2.0f * a);
                float root_b = (-b - sqrt_d) / (2.0f * a);
                result->current_out[i] = ppc_same_sign_root(root_a, root_b, cmd_current[i]);
            } else {
                float scale = positive_power_sum > 1.0e-6f ? available_power / positive_power_sum : 0.0f;
                result->current_out[i] = cmd_current[i] * ppc_clampf(scale, 0.0f, 1.0f);
            }

            result->current_out[i] = ppc_clamp_abs(result->current_out[i], ppc_absf(cmd_current[i]));
            result->current_out[i] = ppc_clamp_abs(result->current_out[i], cmd[i].current_limit);
        }
    }

    for (i = 0u; i < motor_count; i++) {
        if (cmd[i].online == 0u)
            result->current_out[i] = 0.0f;
        total_after += PPC_EstimateMotorPower(c, cmd[i].speed, result->current_out[i], motor_count);
    }

    result->power_before = total_before;
    result->power_after = total_after;
    result->power_limit_used = available_power;
    result->scale = total_before > 1.0e-6f ? total_after / total_before : 0.0f;
    result->limited = 1u;
}
