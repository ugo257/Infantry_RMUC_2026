#ifndef DOUBLE_PITCH_LQR_H
#define DOUBLE_PITCH_LQR_H

#include "stdint.h"

typedef uint16_t DoublePitchDebugHalf_t; /* IEEE-754 binary16 bits for low-bandwidth debug records */

typedef struct
{
    uint8_t ready;
    uint8_t accel_ready;
    float tau_big_last_nm;
    float tau_small_last_nm;
    float time_s;
    float omega_big_last_rad_s;
    float omega_small_last_rad_s;
} DoublePitchLqr_s;

typedef struct
{
    float theta_rad;
    float phi_rad;
    float psi_rad;
    float omega_big_rad_s;
    float omega_small_rad_s;
    float theta_ref_rad;
    float phi_ref_rad;
    float theta_ref_dot_rad_s;
    float phi_ref_dot_rad_s;
    float theta_ref_ddot_rad_s2;
    float phi_ref_ddot_rad_s2;
    float torque_big_limit_nm;
    float torque_small_limit_nm;
    float torque_big_slew_rate_nm_s;
    float torque_small_slew_rate_nm_s;
    float tau_big_meas_nm;
    float tau_small_meas_nm;
    float motor_big_temp_c;
    float motor_small_temp_c;
    float motor_big_tmos_c;
    float motor_small_tmos_c;
    float bus_voltage_v;
    uint8_t target_id;
    uint8_t control_mode;
    uint8_t bus_voltage_valid;
    uint8_t torque_slew_enable;
    uint8_t feedback_ok;
} DoublePitchLqrInput_s;

typedef struct
{
    float tau_big_cmd_nm;
    float tau_small_cmd_nm;
    float tau_big_pre_limit_nm;
    float tau_small_pre_limit_nm;
    float tau_big_fb_nm;
    float tau_small_fb_nm;
    float tau_big_g_nm;
    float tau_small_g_nm;
    float v_big_cmd_rad_s2;
    float v_small_cmd_rad_s2;
    float e_theta_rad;
    float e_phi_rad;
    float e_omega_big_rad_s;
    float e_omega_small_rad_s;
    uint8_t tau_big_limit_flag;
    uint8_t tau_small_limit_flag;
    uint8_t k_segment;
} DoublePitchLqrOutput_s;

typedef struct
{
    float dt_s;
    uint8_t feedback_ok;
    uint8_t ready;
    uint8_t k_segment;
    uint8_t tau_big_limit_flag;
    uint8_t tau_small_limit_flag;
    DoublePitchDebugHalf_t theta_rad;
    DoublePitchDebugHalf_t phi_rad;
    DoublePitchDebugHalf_t psi_rad;
    DoublePitchDebugHalf_t theta_ref_rad;
    DoublePitchDebugHalf_t phi_ref_rad;
    DoublePitchDebugHalf_t psi_ref_rad;
    DoublePitchDebugHalf_t omega_big_rad_s;
    DoublePitchDebugHalf_t omega_small_rad_s;
    DoublePitchDebugHalf_t e_theta_rad;
    DoublePitchDebugHalf_t e_phi_rad;
    DoublePitchDebugHalf_t e_omega_big_rad_s;
    DoublePitchDebugHalf_t e_omega_small_rad_s;
    DoublePitchDebugHalf_t v_big_cmd_rad_s2;
    DoublePitchDebugHalf_t v_small_cmd_rad_s2;
    DoublePitchDebugHalf_t tau_big_fb_nm;
    DoublePitchDebugHalf_t tau_small_fb_nm;
    DoublePitchDebugHalf_t tau_big_g_nm;
    DoublePitchDebugHalf_t tau_small_g_nm;
    DoublePitchDebugHalf_t tau_big_pre_limit_nm;
    DoublePitchDebugHalf_t tau_small_pre_limit_nm;
    DoublePitchDebugHalf_t tau_big_cmd_nm;
    DoublePitchDebugHalf_t tau_small_cmd_nm;
} DoublePitchLqrDebug_s;

typedef struct
{
    float time_s;
    float sample_time_s;
    uint8_t target_id;
    uint8_t control_mode;
    uint8_t feedback_ok;
    uint8_t ready;
    uint8_t accel_ready;
    uint8_t tau_big_limit_flag;
    uint8_t tau_small_limit_flag;
    uint8_t c_used;
    uint8_t f_used;
    uint8_t bus_voltage_valid;
    uint8_t k_segment;
    DoublePitchDebugHalf_t k_alpha;
    DoublePitchDebugHalf_t theta_big_rad;
    DoublePitchDebugHalf_t phi_rel_rad;
    DoublePitchDebugHalf_t psi_small_world_rad;
    DoublePitchDebugHalf_t theta_big_dot_radps;
    DoublePitchDebugHalf_t phi_rel_dot_radps;
    DoublePitchDebugHalf_t theta_big_ddot_radps2;
    DoublePitchDebugHalf_t phi_rel_ddot_radps2;
    DoublePitchDebugHalf_t theta_big_ref_rad;
    DoublePitchDebugHalf_t phi_rel_ref_rad;
    DoublePitchDebugHalf_t theta_big_ref_dot_radps;
    DoublePitchDebugHalf_t phi_rel_ref_dot_radps;
    DoublePitchDebugHalf_t theta_big_ref_ddot_radps2;
    DoublePitchDebugHalf_t phi_rel_ref_ddot_radps2;
    DoublePitchDebugHalf_t tau_big_cmd_nm;
    DoublePitchDebugHalf_t tau_small_cmd_nm;
    DoublePitchDebugHalf_t tau_big_meas_nm;
    DoublePitchDebugHalf_t tau_small_meas_nm;
    DoublePitchDebugHalf_t v_big_cmd_radps2;
    DoublePitchDebugHalf_t v_small_cmd_radps2;
    DoublePitchDebugHalf_t tau_big_ff_nm;
    DoublePitchDebugHalf_t tau_small_ff_nm;
    DoublePitchDebugHalf_t tau_big_fb_nm;
    DoublePitchDebugHalf_t tau_small_fb_nm;
    DoublePitchDebugHalf_t tau_big_pre_limit_nm;
    DoublePitchDebugHalf_t tau_small_pre_limit_nm;
    DoublePitchDebugHalf_t bus_voltage_v;
    DoublePitchDebugHalf_t motor_big_temp_c;
    DoublePitchDebugHalf_t motor_small_temp_c;
    DoublePitchDebugHalf_t motor_big_tmos_c;
    DoublePitchDebugHalf_t motor_small_tmos_c;
    DoublePitchDebugHalf_t a0;
    DoublePitchDebugHalf_t ac;
    DoublePitchDebugHalf_t as;
    DoublePitchDebugHalf_t b0;
    DoublePitchDebugHalf_t bc;
    DoublePitchDebugHalf_t bs;
    DoublePitchDebugHalf_t c0;
    DoublePitchDebugHalf_t gb1;
    DoublePitchDebugHalf_t gb2;
    DoublePitchDebugHalf_t gp1;
    DoublePitchDebugHalf_t gp2;
    DoublePitchDebugHalf_t fvb;
    DoublePitchDebugHalf_t fcb;
    DoublePitchDebugHalf_t fvs;
    DoublePitchDebugHalf_t fcs;
    DoublePitchDebugHalf_t omega0;
    DoublePitchDebugHalf_t sin_phi;
    DoublePitchDebugHalf_t cos_phi;
    DoublePitchDebugHalf_t sin_theta;
    DoublePitchDebugHalf_t cos_theta;
    DoublePitchDebugHalf_t sin_theta_plus_phi;
    DoublePitchDebugHalf_t cos_theta_plus_phi;
    DoublePitchDebugHalf_t m11;
    DoublePitchDebugHalf_t m12;
    DoublePitchDebugHalf_t m22;
    DoublePitchDebugHalf_t m11_dot_phi;
    DoublePitchDebugHalf_t m12_dot_phi;
    DoublePitchDebugHalf_t c_big;
    DoublePitchDebugHalf_t c_small;
    DoublePitchDebugHalf_t g_big;
    DoublePitchDebugHalf_t g_small;
    DoublePitchDebugHalf_t f_big;
    DoublePitchDebugHalf_t f_small;
    DoublePitchDebugHalf_t k[2][4];
} DoublePitchModelDebug_s;

extern volatile DoublePitchLqrDebug_s g_double_pitch_lqr_debug;
extern volatile DoublePitchModelDebug_s g_double_pitch_model_debug;
extern volatile float g_double_pitch_k_live[2][4];

void DoublePitchLqr_Init(DoublePitchLqr_s *ctrl);
void DoublePitchLqr_Reset(DoublePitchLqr_s *ctrl);
void DoublePitchLqr_Calc(DoublePitchLqr_s *ctrl,
                         const DoublePitchLqrInput_s *input,
                         float dt_s,
                         DoublePitchLqrOutput_s *output);

#endif
