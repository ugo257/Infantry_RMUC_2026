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

typedef uint16_t LQRESODebugHalf_t; /* IEEE-754 binary16 bits for low-bandwidth debug records */

typedef struct
{
    float theta_rad;
    float omega_rad_s;
    float tau_meas_nm;
    float theta_ref_deg;
    float theta_ref_rad;
    float omega_ref_rad_s;
    float alpha_ref_rad_s2;
    float omega_base_rad_s;
    int32_t vision_frame_seq;
    float vision_period_s;
    float torque_slew_rate_nm_s;
    uint8_t ref_traj_valid;
    uint8_t auto_aim_mode;
    uint8_t hole_offset_mode;
    uint8_t torque_slew_enable;
    uint8_t base_omega_valid;
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
    /* Auto-aim trajectory extrapolation state */
    float traj_latched_theta_rad;
    float traj_latched_omega_rad_s;
    float traj_latched_alpha_rad_s2;
    float traj_alpha_filt_rad_s2;
    float traj_dt_since_latch_s;
    float traj_latched_period_s;
    float traj_last_input_theta_rad;
    float traj_last_input_omega_rad_s;
    float traj_last_input_alpha_rad_s2;
    int32_t traj_last_vision_frame_seq;
    uint8_t traj_initialized;
    /* Yaw base motion compensation state */
    float base_omega_filt_rad_s;
    uint8_t base_omega_filter_ready;
    /* Identification excitation state */
    float ident_excite_phase_rad;
    float ident_excite_cycle_t_s;
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
    LQRESODebugHalf_t theta_ref_cmd_deg;
    LQRESODebugHalf_t theta_ref_rad;
    LQRESODebugHalf_t omega_ref_rad_s;
    LQRESODebugHalf_t alpha_ref_rad_s2;
    LQRESODebugHalf_t theta_meas_rad;
    LQRESODebugHalf_t omega_meas_rad_s;
    LQRESODebugHalf_t e_theta_rad;
    LQRESODebugHalf_t e_omega_rad_s;
    LQRESODebugHalf_t tau_lqr_nm;
    LQRESODebugHalf_t tau_bias_nm;
    LQRESODebugHalf_t tau_cmd_nm;
    LQRESODebugHalf_t tau_meas_nm;
    LQRESODebugHalf_t tau_meas_filt_nm;
    LQRESODebugHalf_t tau_track_err_nm;
    LQRESODebugHalf_t motor_pos_rad;
    LQRESODebugHalf_t motor_total_pos_rad;
    LQRESODebugHalf_t motor_vel_rad_s;
    LQRESODebugHalf_t motor_tor_nm;
    LQRESODebugHalf_t ctrl_tor_set_nm;
    int motor_p_int;
    int motor_v_int;
    int motor_t_int;
} YawLqrIdentRecord_s;

typedef struct
{
    uint32_t sample_count;
    float time_s;
    float dt_s;
    uint8_t valid;
    uint8_t feedback_ok;
    uint8_t gimbal_mode;
    uint8_t nuc_mode;
    uint8_t auto_aim_mode;
    uint8_t limit_active;
    uint8_t soft_limit_active;
    uint8_t dm_limit_active;
    uint8_t slew_limit_active;
    uint8_t ref_omega_limit_active;
    uint8_t ref_alpha_limit_active;
    uint8_t ff_alpha_limit_active;
    uint8_t eso_comp_enable;
    uint8_t eso_comp_active;
    float theta_ref_cmd_deg;
    float input_theta_ref_rad;
    float input_omega_ref_rad_s;
    float input_alpha_ref_rad_s2;
    float yaw_j_used;
    float yaw_b_used;
    float k_theta_used;
    float k_omega_used;
    float theta_ref_planned_rad;
    float omega_ref_planned_rad_s;
    float alpha_ref_planned_rad_s2;
    float theta_meas_rad;
    float omega_meas_rad_s;
    float motor_pos_rad;
    float motor_total_pos_rad;
    float motor_vel_rad_s;
    float motor_tor_nm;
    float ctrl_tor_set_nm;
    float e_theta_raw_rad;
    float e_theta_rad;
    float e_omega_rad_s;
    float tau_ff_alpha_raw_nm;
    float tau_ff_alpha_nm;
    float tau_ff_viscous_nm;
    float tau_ff_coulomb_nm;
    float tau_ff_nm;
    float tau_theta_nm;
    float tau_omega_nm;
    float tau_lqi_nm;
    float tau_lqr_nm;
    float tau_for_eso_nm;
    float tau_eso_raw_nm;
    float tau_eso_limited_nm;
    float tau_eso_comp_nm;
    float tau_bias_nm;
    float tau_pre_limit_nm;
    float tau_cmd_before_slew_nm;
    float tau_cmd_nm;
    float tau_meas_nm;
    float tau_meas_filt_nm;
    float tau_track_err_nm;
    float eso_comp_gain_used;
    float eso_comp_limit_nm;
    float yaw_eso_b0;
    float yaw_eso_w0;
    float yaw_eso_z1;
    float yaw_eso_z2;
    float yaw_eso_z3;
    /* Identification excitation injector (live-watchable). */
    uint8_t ident_excite_active;
    uint8_t ident_excite_type_used;
    float tau_excitation_nm;
    float ident_excite_freq_hz;
    float ident_excite_phase;
} YawAutoAimDebugRecord_s;

#define YAW_MODEL_IDENT_ABORT_NONE          0U
#define YAW_MODEL_IDENT_ABORT_WAIT_READY    1U
#define YAW_MODEL_IDENT_ABORT_FEEDBACK_LOST 2U
#define YAW_MODEL_IDENT_ABORT_EXIT_AUTO     3U
#define YAW_MODEL_IDENT_ABORT_STOP          4U

#define YAW_MODEL_IDENT_STAGE_ALL           0U
#define YAW_MODEL_IDENT_STAGE_LOW_SPEED     1U
#define YAW_MODEL_IDENT_STAGE_DYNAMIC_CHIRP 2U
#define YAW_MODEL_IDENT_STAGE_DEFAULT       YAW_MODEL_IDENT_STAGE_ALL

typedef struct
{
    uint32_t sample_count;
    float time_s;
    float dt_s;
    uint8_t arm;
    uint8_t stop;
    uint8_t running;
    uint8_t done;
    uint8_t valid;
    uint8_t feedback_ok;
    uint8_t abort_reason;
    uint8_t stage_select;
    uint8_t stage_id;
    uint8_t sample_active;
    uint8_t gimbal_mode;
    uint8_t nuc_mode;
    uint8_t auto_aim_mode;
    uint8_t ref_traj_valid;
    uint8_t k_override_enable;
    uint8_t limit_active;
    uint8_t soft_limit_active;
    uint8_t dm_limit_active;
    uint8_t slew_limit_active;
    uint8_t low_speed_flag;
    uint8_t reverse_flag;
    uint8_t torque_saturated_flag;
    float duration_s;
    float amp_nm;
    float amp_limit_nm;
    float f_start_hz;
    float f_end_hz;
    float k_theta_ident;
    float k_omega_ident;
    float ident_freq_hz;
    float ident_phase_rad;
    float theta_ref_cmd_deg;
    float input_theta_ref_rad;
    float input_omega_ref_rad_s;
    float input_alpha_ref_rad_s2;
    float theta_ref_planned_rad;
    float omega_ref_planned_rad_s;
    float alpha_ref_planned_rad_s2;
    float theta_meas_rad;
    float omega_meas_rad_s;
    float omega_abs_rad_s;
    float yaw_j_used;
    float yaw_b_used;
    float yaw_coulomb_nm;
    float yaw_coulomb_smooth_omega;
    float k_theta_used;
    float k_omega_used;
    float e_theta_raw_rad;
    float e_theta_rad;
    float e_omega_rad_s;
    float tau_ff_alpha_raw_nm;
    float tau_ff_alpha_nm;
    float tau_ff_viscous_nm;
    float tau_ff_coulomb_nm;
    float tau_ff_nm;
    float tau_theta_nm;
    float tau_omega_nm;
    float tau_lqi_nm;
    float tau_lqr_nm;
    float tau_bias_nm;
    float tau_pre_limit_nm;
    float tau_excitation_nm;
    float tau_cmd_before_slew_nm;
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
} YawModelIdentRecord_s;

typedef struct
{
    uint8_t state_dim;
    float k[LQR_ESO_MAX_STATE_DIM];
    float output_limit;
    float output_slew_rate;
    float velocity_limit;
    uint8_t model_enable;
    uint8_t disable_model_feedforward;
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
    uint8_t w0_set_enable;
    float w0_set;
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
extern volatile float g_lqr_hole_offset_k_theta;
extern volatile float g_lqr_hole_offset_k_omega;
extern volatile float g_yaw_j;
extern volatile float g_yaw_b;
extern volatile float g_yaw_tau_coulomb_nm;
extern volatile float g_yaw_coulomb_smooth_omega;
extern volatile float g_yaw_auto_j;
extern volatile float g_yaw_auto_b;
extern volatile float g_yaw_auto_tau_coulomb_nm;
extern volatile float g_yaw_auto_coulomb_smooth_omega;
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
extern volatile float g_yaw_auto_eso_w0_set;
extern volatile float g_yaw_eso_w0;
extern volatile float g_yaw_eso_beta1;
extern volatile float g_yaw_eso_beta2;
extern volatile float g_yaw_eso_beta3;
extern volatile float g_yaw_eso_z1;
extern volatile float g_yaw_eso_z2;
extern volatile float g_yaw_eso_z3;
extern volatile float g_yaw_eso_tau_comp;
extern volatile uint8_t g_yaw_base_comp_enable;
extern volatile float g_yaw_base_omega_gain;
extern volatile float g_yaw_base_omega_lpf_tau_s;
extern volatile float g_yaw_base_omega_rad_s;
extern volatile float g_yaw_omega_rel_ref_rad_s;
extern volatile float g_yaw_omega_rel_meas_rad_s;
extern volatile float g_yaw_eso_omega_rel_rad_s;
extern volatile uint8_t g_yaw_auto_eso_comp_enable;
extern volatile float g_yaw_auto_eso_comp_omega_gate_rad_s;
extern volatile float g_yaw_auto_eso_comp_alpha_gate_rad_s2;
extern volatile float g_yaw_auto_eso_comp_active;
extern volatile float g_yaw_auto_ff_alpha_tau_limit_nm;
extern volatile float g_torque_cmd;
extern volatile uint32_t g_yaw_lqreso_control_seq;
extern volatile YawLqrIdentRecord_s g_yaw_lqr_ident_record;
extern volatile YawAutoAimDebugRecord_s g_yaw_auto_aim_debug_record;
extern volatile YawModelIdentRecord_s g_yaw_model_ident;
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
/* Auto-aim trajectory planner tunables (live-watchable) */
extern volatile uint8_t g_traj_planner_enable;
extern volatile float g_traj_alpha_lpf_rc_s;
extern volatile float g_traj_lookahead_s;
extern volatile float g_traj_change_eps_rad;
/* Yaw identification excitation injector tunables (live-watchable). */
extern volatile uint8_t g_yaw_ident_excite_enable;
extern volatile uint8_t g_yaw_ident_excite_type;
extern volatile float g_yaw_ident_excite_amp_nm;
extern volatile float g_yaw_ident_excite_f0_hz;
extern volatile float g_yaw_ident_excite_f1_hz;
extern volatile float g_yaw_ident_excite_duration_s;
extern volatile float g_yaw_ident_excite_amp_limit_nm;

void YawLQRESO_Init(YawLQRESO_s *ctrl);
void YawLQRESO_Reset(YawLQRESO_s *ctrl);
void YawLQRESO_Calc(YawLQRESO_s *ctrl, const YawControlInput_t *input, float dt, YawControlOutput_t *output);
LQRESODebugHalf_t LQRESODebugFloatToHalf(float value);

void PitchLQRESO_Init(PitchLQRESO_s *ctrl, const LQR_Config_s *lqr_config, const ESO_Config_s *eso_config);
void PitchLQRESO_Reset(PitchLQRESO_s *ctrl);
float PitchLQRESO_CalcTrajectory(PitchLQRESO_s *ctrl,
                                  float ref,
                                  float ref_vel,
                                  float ref_acc,
                                  float measure,
                                  float speed,
                                  float feedforward,
                                  uint8_t eso_runtime_enable,
                                  float dt);
float PitchLQRESO_Calc(PitchLQRESO_s *ctrl, float ref, float measure, float speed, float feedforward, float dt);

void ChassisLQRESO_Init(ChassisLQRESO_s *ctrl, const LQR_Config_s *lqr_config, const ESO_Config_s *eso_config);
void ChassisLQRESO_Reset(ChassisLQRESO_s *ctrl);
float ChassisLQRESO_Calc(ChassisLQRESO_s *ctrl, float ref, float measure, float speed, float feedforward, float dt);

#endif
