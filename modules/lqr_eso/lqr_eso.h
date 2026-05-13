/**
 ******************************************************************************
 * @file    lqr_eso.h
 * @author  daizixuan
 * @school  njust
 * @team    combat
 * @brief   LQR controller and ESO observer interfaces for yaw, pitch, chassis.
 *
 * The yaw controller follows LQR_ESO_Porting_Guide.md: external yaw references
 * stay in degrees, measured states are radians/rad/s, and output is MIT torque.
 * Pitch uses a lightweight LQR-style torque controller; chassis keeps an
 * independent zero-output interface for later tuning.
 ******************************************************************************
 */

#ifndef LQR_ESO_H
#define LQR_ESO_H

#include "stdint.h"

#define LQR_ESO_MAX_STATE_DIM 8U
#define LQR_ESO_DEG_TO_RAD    0.017453292519943295f
#define LQR_ESO_RAD_TO_DEG    57.29577951308232f

typedef struct
{
    float theta_rad;
    float omega_rad_s;
    float tau_meas_nm;
    float theta_ref_deg;
    float theta_ref_rad;
    float omega_ref_rad_s;
    float alpha_ref_rad_s2;
    float torque_slew_rate_nm_s;
    uint8_t ref_traj_valid;
    uint8_t auto_aim_mode;
    uint8_t torque_slew_enable;
    uint8_t feedback_ok;
} YawControlInput_t;

typedef struct
{
    float tau_cmd_nm;
    float theta_ref_rad;
    float omega_ref_rad_s;
    float alpha_ref_rad_s2;
} YawControlOutput_t;

typedef struct
{
    float z1;
    float z2;
    float z3;
    float b0;
    float w0;
    float beta1;
    float beta2;
    float beta3;
} YawEso_t;

typedef struct
{
    YawEso_t eso;
    float theta_ref_planned_rad;
    float omega_ref_planned;
    float alpha_ref_planned;
    float eso_u_last;
    float torque_cmd_last;
    float tau_bias;
    float tau_meas_filt;
    float lqi_int;
    uint8_t feedback_ready;
    uint8_t limit_active;
} YawLQRESO_s;

typedef struct
{
    uint32_t sample_count;
    float time_s;
    float dt_s;
    uint8_t valid;
    uint8_t feedback_ok;
    uint8_t gimbal_mode;
    uint8_t nuc_mode;
    uint8_t ref_traj_valid;
    float theta_ref_cmd_deg;
    float theta_ref_rad;
    float omega_ref_rad_s;
    float alpha_ref_rad_s2;
    float theta_meas_rad;
    float omega_meas_rad_s;
    float e_theta_rad;
    float e_omega_rad_s;
    float tau_lqr_nm;
    float tau_bias_nm;
    float tau_cmd_nm;
    float tau_meas_nm;
    float tau_meas_filt_nm;
    float tau_track_err_nm;
    float motor_pos_rad;
    float motor_total_pos_rad;
    float motor_vel_rad_s;
    float motor_tor_nm;
    float ctrl_tor_set_nm;
    int motor_p_int;
    int motor_v_int;
    int motor_t_int;
} YawLqrIdentRecord_s;

typedef struct
{
    uint8_t state_dim;
    float k[LQR_ESO_MAX_STATE_DIM];
    float output_limit;
    float output_slew_rate;
    float velocity_limit;
    uint8_t model_enable;
    float plant_j_kg_m2;
    float plant_b_nms_rad;
    float q_theta;
    float q_omega;
    float r_torque;
    float model_a22;
    float model_b2;
    float model_k_theta;
    float model_k_omega;
} LQR_Config_s;

typedef struct
{
    uint8_t enable;
    float alpha;
    float comp_gain;
    float comp_limit;
    float w0_min;
    float w0_max;
    float beta1;
    float beta2;
    float beta3;
    float b0;
} ESO_Config_s;

typedef struct
{
    float z1;
    float z2;
    float z3;
    float b0;
    float w0;
    float beta1;
    float beta2;
    float beta3;
    float u_last;
} SimpleEso_t;

typedef struct
{
    LQR_Config_s lqr_config;
    ESO_Config_s eso_config;
    SimpleEso_t eso;
    float state[LQR_ESO_MAX_STATE_DIM];
    float output;
    float torque;
    float k_theta_used;
    float k_omega_used;
    float tau_lqr;
    float tau_eso_comp;
    float tau_pre_limit;
} LQRESO_SimpleController_s;

typedef LQRESO_SimpleController_s PitchLQRESO_s;
typedef LQRESO_SimpleController_s ChassisLQRESO_s;

extern volatile float g_lqr_k_theta;
extern volatile float g_lqr_k_omega;
extern volatile float g_lqr_auto_k_theta;
extern volatile float g_lqr_auto_k_omega;
extern volatile float g_ctrl_t_limit;
extern volatile float g_ctrl_angle_deadband_rad;
extern volatile float g_ctrl_vel_deadband;
extern volatile float g_ctrl_max_error_rad;
extern volatile float g_lqr_e_theta;
extern volatile float g_lqr_omega_filt;
extern volatile float g_lqr_tau_raw;
extern volatile float g_tau_meas_filt;
extern volatile float g_tau_track_err;
extern volatile float g_tau_meas_lpf_alpha;
extern volatile float g_eso_torque_feedback_blend;
extern volatile float g_tau_bias_ki;
extern volatile float g_tau_bias_limit;
extern volatile float g_lqi_ki;
extern volatile float g_lqi_int;
extern volatile float g_lqi_int_limit;
extern volatile float g_lqi_integral_enable_rad;
extern volatile float g_theta_ref_planned;
extern volatile float g_omega_ref_planned;
extern volatile float g_alpha_ref_planned;
extern volatile float g_ref_max_vel_deg_s;
extern volatile float g_ref_max_accel_deg_s2;
extern volatile float g_ref_omega_limit_rad_s;
extern volatile float g_ctrl_torque_slew_rate;
extern volatile float g_ctrl_auto_torque_slew_rate;
extern volatile float g_ctrl_min_torque_comp;
extern volatile float g_eso_comp_gain;
extern volatile float g_eso_comp_limit;
extern volatile float g_eso_comp_gain_max;
extern volatile float g_eso_comp_limit_max;
extern volatile float g_ctrl_limit_active;
extern volatile float g_yaw_eso_alpha;
extern volatile float g_yaw_eso_w0_set;
extern volatile float g_yaw_eso_w0;
extern volatile float g_yaw_eso_beta1;
extern volatile float g_yaw_eso_beta2;
extern volatile float g_yaw_eso_beta3;
extern volatile float g_yaw_eso_z1;
extern volatile float g_yaw_eso_z2;
extern volatile float g_yaw_eso_z3;
extern volatile float g_yaw_eso_tau_comp;
extern volatile uint8_t g_yaw_auto_eso_comp_enable;
extern volatile float g_yaw_auto_eso_comp_omega_gate_rad_s;
extern volatile float g_yaw_auto_eso_comp_active;
extern volatile float g_torque_cmd;
extern volatile YawLqrIdentRecord_s g_yaw_lqr_ident_record;
extern volatile float g_pitch_eso_alpha;
extern volatile float g_pitch_eso_comp_gain;
extern volatile float g_pitch_eso_comp_limit;
extern volatile float g_pitch_eso_b0;
extern volatile float g_pitch_eso_w0;
extern volatile float g_pitch_eso_beta1;
extern volatile float g_pitch_eso_beta2;
extern volatile float g_pitch_eso_beta3;
extern volatile float g_pitch_eso_z1;
extern volatile float g_pitch_eso_z2;
extern volatile float g_pitch_eso_z3;
extern volatile float g_pitch_eso_tau_comp;

void YawLQRESO_Init(YawLQRESO_s *ctrl);
void YawLQRESO_Reset(YawLQRESO_s *ctrl);
void YawLQRESO_Calc(YawLQRESO_s *ctrl, const YawControlInput_t *input, float dt, YawControlOutput_t *output);

void PitchLQRESO_Init(PitchLQRESO_s *ctrl, const LQR_Config_s *lqr_config, const ESO_Config_s *eso_config);
void PitchLQRESO_Reset(PitchLQRESO_s *ctrl);
float PitchLQRESO_Calc(PitchLQRESO_s *ctrl, float ref, float measure, float speed, float feedforward, float dt);

void ChassisLQRESO_Init(ChassisLQRESO_s *ctrl, const LQR_Config_s *lqr_config, const ESO_Config_s *eso_config);
void ChassisLQRESO_Reset(ChassisLQRESO_s *ctrl);
float ChassisLQRESO_Calc(ChassisLQRESO_s *ctrl, float ref, float measure, float speed, float feedforward, float dt);

#endif
