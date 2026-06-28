#ifndef PORTABLE_POWER_CONTROLLER_H
#define PORTABLE_POWER_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PPC_MAX_MOTOR_COUNT
#define PPC_MAX_MOTOR_COUNT 6u
#endif

typedef struct
{
    float mech_k;
    float k1;
    float k2;
    float k3;
    float power_margin;
    uint8_t motor_count;
} PPC_Config;

typedef struct
{
    float lambda;
    float delta;
    uint8_t enable_k3_update;
    uint8_t initialized;
    uint32_t update_count;
    float theta[3];
    float p[9];
    float last_phi[3];
    float last_loss_measured;
    float last_loss_estimated;
    float last_residual;
} PPC_RLS;

typedef struct
{
    float speed;
    float current;
} PPC_MotorState;

typedef struct
{
    float speed;
    float target_speed;
    float current_cmd;
    float current_limit;
    uint8_t online;
} PPC_MotorCmd;

typedef struct
{
    float current_out[PPC_MAX_MOTOR_COUNT];
    float power_before;
    float power_after;
    float power_limit_used;
    float scale;
    uint8_t limited;
} PPC_Result;

extern const PPC_Config PPC_DEFAULT_CONFIG_6MOTOR;
extern const PPC_Config PPC_DEFAULT_CONFIG_4MOTOR;

void PPC_Init(PPC_Config *cfg);

void PPC_RLS_Init(PPC_RLS *rls,
                  const PPC_Config *cfg,
                  float lambda,
                  float delta,
                  uint8_t enable_k3_update);

void PPC_RLS_Reset(PPC_RLS *rls);

uint8_t PPC_RLS_Update(const PPC_Config *cfg,
                       const PPC_MotorState *motor,
                       float measured_total_power,
                       PPC_RLS *rls,
                       PPC_Config *out_cfg);

float PPC_EstimateMotorPower(const PPC_Config *cfg,
                             float speed,
                             float current,
                             uint8_t motor_count);

float PPC_EstimateTotalPower(const PPC_Config *cfg,
                             const PPC_MotorState *motor);

void PPC_LimitCurrent(const PPC_Config *cfg,
                      const PPC_MotorCmd *cmd,
                      float power_limit,
                      PPC_Result *result);

#ifdef __cplusplus
}
#endif

#endif
