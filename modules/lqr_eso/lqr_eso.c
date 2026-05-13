/**
 ******************************************************************************
 * @file    lqr_eso.c
 * @author  daizixuan
 * @school  njust
 * @team    combat
 * @brief   LQR controller and ESO observer implementation for yaw, pitch,
 *          and chassis control paths.
 *
 * Yaw follows LQR_ESO_Porting_Guide.md. Pitch keeps a lightweight LQR-style
 * torque controller for the pitch-up DM motor. Chassis remains a zero-output
 * placeholder until its own plant parameters are identified.
 ******************************************************************************
 */

#include "lqr_eso.h"

#include "math.h"
#include "string.h"

#define YAW_J_DEFAULT                       0.0491133f
#define YAW_B_DEFAULT                       0.0176839f
#define YAW_TS_DEFAULT                      0.001f
#define LQR_K_THETA_DEFAULT                 43.7692812f
#define LQR_K_OMEGA_DEFAULT                 2.09040931f
#define LQR_AUTO_K_THETA_DEFAULT            110.0f
#define LQR_AUTO_K_OMEGA_DEFAULT            1.0f
#define CTRL_T_LIMIT_DEFAULT                7.0f
#define CTRL_ANGLE_DEADBAND_RAD_DEFAULT     0.0f
#define CTRL_VEL_DEADBAND_DEFAULT           0.03f
#define CTRL_MAX_ERROR_RAD_DEFAULT          0.0f
#define TAU_MEAS_LPF_ALPHA_DEFAULT          0.10f
#define ESO_TORQUE_BLEND_DEFAULT            0.0f
#define TAU_BIAS_KI_DEFAULT                 0.0f
#define TAU_BIAS_LIMIT_DEFAULT              0.10f
#define LQI_KI_DEFAULT                      0.0f
#define LQI_INT_LIMIT_DEFAULT               0.10f
#define LQI_INT_ENABLE_RAD_DEFAULT          0.0872665f
#define REF_MAX_VEL_DEFAULT                 180.0f
#define REF_MAX_ACCEL_DEFAULT               540.0f
#define REF_OMEGA_LIMIT_DEFAULT             100.0f
#define REF_ALPHA_LIMIT_RAD_S2_DEFAULT      300.0f
#define CTRL_TORQUE_SLEW_RATE_DEFAULT       1000.0f
#define CTRL_AUTO_TORQUE_SLEW_RATE_DEFAULT  7000.0f
#define CTRL_MIN_TORQUE_COMP_DEFAULT        0.40f
#define ESO_COMP_GAIN_DEFAULT               0.18f
#define ESO_COMP_LIMIT_DEFAULT              0.35f
#define YAW_ESO_ALPHA_DEFAULT               1.10f
#define YAW_ESO_W0_SET_DEFAULT              36.0f
#define YAW_AUTO_ESO_COMP_ENABLE_DEFAULT    1U
#define YAW_AUTO_ESO_COMP_OMEGA_GATE_DEFAULT 50.0f
#define PITCH_LQR_K_THETA_DEFAULT           6.0f
#define PITCH_LQR_K_OMEGA_DEFAULT           0.8f
#define PITCH_CTRL_T_LIMIT_DEFAULT          2.0f
#define PITCH_CTRL_TORQUE_SLEW_RATE_DEFAULT 700.0f
#define PITCH_UP_ESO_ALPHA_DEFAULT          0.0f
#define PITCH_UP_ESO_COMP_GAIN_DEFAULT      0.0f
#define PITCH_UP_ESO_COMP_LIMIT_DEFAULT     0.0f
#define ESO_COMP_GAIN_MAX_DEFAULT           2.0f
#define ESO_COMP_LIMIT_MAX_DEFAULT          7.0f
#define DM_T_MIN                            (-7.0f)
#define DM_T_MAX                            (7.0f)
#define LQR_ESO_TWO_PI                      6.283185307179586f
#define LQR_ESO_MIN_DT                      0.0001f
#define LQR_ESO_MAX_DT                      0.02f
#define PITCH_MODEL_MIN_J                   1.0e-6f
#define PITCH_MODEL_MIN_R                   1.0e-6f
#define PITCH_ESO_MIN_B0                    1.0e-6f
#define PITCH_STATE_READY                   0U
#define PITCH_STATE_LAST_TORQUE             1U
#define PITCH_STATE_E_THETA                 2U
#define PITCH_STATE_E_OMEGA                 3U
#define PITCH_STATE_TAU_LQR                 4U
#define PITCH_STATE_LIMIT_ACTIVE            5U
#define PITCH_STATE_REF                     6U
#define PITCH_STATE_MEASURE                 7U
#define TRAJ_PLANNER_ENABLE_DEFAULT         1U
#define TRAJ_ALPHA_LPF_RC_DEFAULT           0.010f
#define TRAJ_STALE_TIMEOUT_DEFAULT          0.050f
#define TRAJ_DECAY_TAU_DEFAULT              0.030f
#define TRAJ_LOOKAHEAD_DEFAULT              0.0f
#define TRAJ_CHANGE_EPS_RAD_DEFAULT         1e-6f

volatile float g_lqr_k_theta                 = LQR_K_THETA_DEFAULT;
volatile float g_lqr_k_omega                 = LQR_K_OMEGA_DEFAULT;
volatile float g_lqr_auto_k_theta            = LQR_AUTO_K_THETA_DEFAULT;
volatile float g_lqr_auto_k_omega            = LQR_AUTO_K_OMEGA_DEFAULT;
volatile float g_ctrl_t_limit                = CTRL_T_LIMIT_DEFAULT;
volatile float g_ctrl_angle_deadband_rad     = CTRL_ANGLE_DEADBAND_RAD_DEFAULT;
volatile float g_ctrl_vel_deadband           = CTRL_VEL_DEADBAND_DEFAULT;
volatile float g_ctrl_max_error_rad          = CTRL_MAX_ERROR_RAD_DEFAULT;
volatile float g_lqr_e_theta                 = 0.0f;
volatile float g_lqr_omega_filt              = 0.0f;
volatile float g_lqr_tau_raw                 = 0.0f;
volatile float g_tau_meas_filt               = 0.0f;
volatile float g_tau_track_err               = 0.0f;
volatile float g_tau_meas_lpf_alpha          = TAU_MEAS_LPF_ALPHA_DEFAULT;
volatile float g_eso_torque_feedback_blend   = ESO_TORQUE_BLEND_DEFAULT;
volatile float g_tau_bias_ki                 = TAU_BIAS_KI_DEFAULT;
volatile float g_tau_bias_limit              = TAU_BIAS_LIMIT_DEFAULT;
volatile float g_lqi_ki                      = LQI_KI_DEFAULT;
volatile float g_lqi_int                     = 0.0f;
volatile float g_lqi_int_limit               = LQI_INT_LIMIT_DEFAULT;
volatile float g_lqi_integral_enable_rad     = LQI_INT_ENABLE_RAD_DEFAULT;
volatile float g_theta_ref_planned           = 0.0f;
volatile float g_omega_ref_planned           = 0.0f;
volatile float g_alpha_ref_planned           = 0.0f;
volatile float g_ref_max_vel_deg_s           = REF_MAX_VEL_DEFAULT;
volatile float g_ref_max_accel_deg_s2        = REF_MAX_ACCEL_DEFAULT;
volatile float g_ref_omega_limit_rad_s       = REF_OMEGA_LIMIT_DEFAULT;
volatile float g_ctrl_torque_slew_rate       = CTRL_TORQUE_SLEW_RATE_DEFAULT;
volatile float g_ctrl_auto_torque_slew_rate  = CTRL_AUTO_TORQUE_SLEW_RATE_DEFAULT;
volatile float g_ctrl_min_torque_comp        = CTRL_MIN_TORQUE_COMP_DEFAULT;
volatile float g_eso_comp_gain               = ESO_COMP_GAIN_DEFAULT;
volatile float g_eso_comp_limit              = ESO_COMP_LIMIT_DEFAULT;
volatile float g_eso_comp_gain_max           = ESO_COMP_GAIN_MAX_DEFAULT;
volatile float g_eso_comp_limit_max          = ESO_COMP_LIMIT_MAX_DEFAULT;
volatile float g_ctrl_limit_active           = 0.0f;
volatile float g_yaw_eso_alpha               = YAW_ESO_ALPHA_DEFAULT;
volatile float g_yaw_eso_w0_set              = YAW_ESO_W0_SET_DEFAULT;
volatile float g_yaw_eso_w0                  = 0.0f;
volatile float g_yaw_eso_beta1               = 0.0f;
volatile float g_yaw_eso_beta2               = 0.0f;
volatile float g_yaw_eso_beta3               = 0.0f;
volatile float g_yaw_eso_z1                  = 0.0f;
volatile float g_yaw_eso_z2                  = 0.0f;
volatile float g_yaw_eso_z3                  = 0.0f;
volatile float g_yaw_eso_tau_comp            = 0.0f;
volatile uint8_t g_yaw_auto_eso_comp_enable  = YAW_AUTO_ESO_COMP_ENABLE_DEFAULT;
volatile float g_yaw_auto_eso_comp_omega_gate_rad_s = YAW_AUTO_ESO_COMP_OMEGA_GATE_DEFAULT;
volatile float g_yaw_auto_eso_comp_active    = 0.0f;
volatile float g_torque_cmd                  = 0.0f;
volatile YawLqrIdentRecord_s g_yaw_lqr_ident_record = {0};
volatile float g_pitch_eso_alpha             = PITCH_UP_ESO_ALPHA_DEFAULT;
volatile float g_pitch_eso_comp_gain         = PITCH_UP_ESO_COMP_GAIN_DEFAULT;
volatile float g_pitch_eso_comp_limit        = PITCH_UP_ESO_COMP_LIMIT_DEFAULT;
volatile float g_pitch_eso_b0                = 0.0f;
volatile float g_pitch_eso_w0                = 0.0f;
volatile float g_pitch_eso_beta1             = 0.0f;
volatile float g_pitch_eso_beta2             = 0.0f;
volatile float g_pitch_eso_beta3             = 0.0f;
volatile float g_pitch_eso_z1                = 0.0f;
volatile float g_pitch_eso_z2                = 0.0f;
volatile float g_pitch_eso_z3                = 0.0f;
volatile float g_pitch_eso_tau_comp          = 0.0f;
volatile uint8_t g_traj_planner_enable      = TRAJ_PLANNER_ENABLE_DEFAULT;
volatile float g_traj_alpha_lpf_rc_s        = TRAJ_ALPHA_LPF_RC_DEFAULT;
volatile float g_traj_stale_timeout_s       = TRAJ_STALE_TIMEOUT_DEFAULT;
volatile float g_traj_decay_tau_s           = TRAJ_DECAY_TAU_DEFAULT;
volatile float g_traj_lookahead_s           = TRAJ_LOOKAHEAD_DEFAULT;
volatile float g_traj_change_eps_rad        = TRAJ_CHANGE_EPS_RAD_DEFAULT;

static float LQRESOAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float LQRESOSign(float value)
{
    if (value > 0.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
}

static float LQRESOClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float LQRESOClampAbs(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    return LQRESOClamp(value, -limit, limit);
}

static float LQRESOApplyDeadband(float value, float deadband)
{
    const float abs_value = LQRESOAbs(value);

    if (deadband <= 0.0f) {
        return value;
    }
    if (abs_value <= deadband) {
        return 0.0f;
    }
    return (value > 0.0f) ? (value - deadband) : (value + deadband);
}

static float LQRESOSanitizeDt(float dt)
{
    if (dt < LQR_ESO_MIN_DT || dt > LQR_ESO_MAX_DT) {
        return YAW_TS_DEFAULT;
    }
    return dt;
}

static void LQRConfigResolvePitchModel(LQR_Config_s *config)
{
    float j;
    float b;
    float q_theta;
    float q_omega;
    float r;
    float sqrt_qr;
    float omega_term;

    if (config == NULL || config->model_enable == 0U) {
        return;
    }

    j = config->plant_j_kg_m2;
    b = config->plant_b_nms_rad;
    q_theta = config->q_theta;
    q_omega = config->q_omega;
    r = config->r_torque;

    if (j <= PITCH_MODEL_MIN_J ||
        r <= PITCH_MODEL_MIN_R ||
        q_theta <= 0.0f ||
        q_omega < 0.0f) {
        config->model_enable = 0U;
        config->model_a22 = 0.0f;
        config->model_b2 = 0.0f;
        config->model_k_theta = 0.0f;
        config->model_k_omega = 0.0f;
        return;
    }

    config->model_a22 = -b / j;
    config->model_b2 = 1.0f / j;

    sqrt_qr = sqrtf(q_theta * r);
    config->model_k_theta = sqrtf(q_theta / r);
    omega_term = b * b + (2.0f * j * sqrt_qr + q_omega) / r;
    config->model_k_omega =
        (omega_term > 0.0f) ? (sqrtf(omega_term) - b) : 0.0f;

    config->state_dim = 2U;
    if (config->k[0] == 0.0f) {
        config->k[0] = config->model_k_theta;
    }
    if (config->k[1] == 0.0f) {
        config->k[1] = config->model_k_omega;
    }
}

static float LQRESOWrapNearestRad(float target, float measure)
{
    float err = target - measure;

    while (err > 3.141592653589793f) {
        target -= LQR_ESO_TWO_PI;
        err -= LQR_ESO_TWO_PI;
    }
    while (err < -3.141592653589793f) {
        target += LQR_ESO_TWO_PI;
        err += LQR_ESO_TWO_PI;
    }
    return target;
}

static void YawPublishObserver(const YawLQRESO_s *ctrl)
{
    if (ctrl == NULL) {
        return;
    }

    g_lqi_int           = ctrl->lqi_int;
    g_tau_meas_filt     = ctrl->tau_meas_filt;
    g_ctrl_limit_active = (float)ctrl->limit_active;
    g_yaw_eso_w0        = ctrl->eso.w0;
    g_yaw_eso_beta1     = ctrl->eso.beta1;
    g_yaw_eso_beta2     = ctrl->eso.beta2;
    g_yaw_eso_beta3     = ctrl->eso.beta3;
    g_yaw_eso_z1        = ctrl->eso.z1;
    g_yaw_eso_z2        = ctrl->eso.z2;
    g_yaw_eso_z3        = ctrl->eso.z3;
    g_torque_cmd        = ctrl->torque_cmd_last;
}

static void YawEso_UpdateGains(YawLQRESO_s *ctrl)
{
    float w0;
    float w0_set;
    float alpha;
    const float b0 = 1.0f / YAW_J_DEFAULT;
    const float k_theta_abs = LQRESOAbs(g_lqr_k_theta);

    if (ctrl == NULL) {
        return;
    }

    alpha = (g_yaw_eso_alpha > 0.0f) ? g_yaw_eso_alpha : 0.0f;
    g_yaw_eso_alpha = alpha;

    w0_set = g_yaw_eso_w0_set;
    w0 = (w0_set > 0.0f) ? w0_set : (alpha * sqrtf(b0 * k_theta_abs));

    ctrl->eso.b0    = b0;
    ctrl->eso.w0    = w0;
    ctrl->eso.beta1 = 3.0f * w0;
    ctrl->eso.beta2 = 3.0f * w0 * w0;
    ctrl->eso.beta3 = w0 * w0 * w0;
}

static void YawEso_InitInternal(YawLQRESO_s *ctrl)
{
    YawEso_UpdateGains(ctrl);
    YawPublishObserver(ctrl);
}

static void YawEso_ResetInternal(YawLQRESO_s *ctrl, float theta, float omega)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->eso.z1                 = theta;
    ctrl->eso.z2                 = omega;
    ctrl->eso.z3                 = 0.0f;
    ctrl->eso_u_last             = 0.0f;
    ctrl->torque_cmd_last        = 0.0f;
    ctrl->tau_bias               = 0.0f;
    ctrl->lqi_int                = 0.0f;
    ctrl->theta_ref_planned_rad  = theta;
    ctrl->omega_ref_planned      = 0.0f;
    ctrl->alpha_ref_planned      = 0.0f;
    ctrl->limit_active           = 0U;

    YawPublishObserver(ctrl);
}

static void YawEso_Update(YawLQRESO_s *ctrl, float theta_meas, float u_last, float dt)
{
    YawEso_t *eso;
    float e;
    float z1_dot;
    float z2_dot;
    float z3_dot;

    if (ctrl == NULL) {
        return;
    }

    eso    = &ctrl->eso;
    e      = theta_meas - eso->z1;
    z1_dot = eso->z2 + eso->beta1 * e;
    z2_dot = -(YAW_B_DEFAULT / YAW_J_DEFAULT) * eso->z2 + eso->b0 * u_last + eso->z3 + eso->beta2 * e;
    z3_dot = eso->beta3 * e;

    eso->z1 += dt * z1_dot;
    eso->z2 += dt * z2_dot;
    eso->z3 += dt * z3_dot;
}

static void YawLqrIdentRecordPublishInput(const YawControlInput_t *input, float dt)
{
    if (input == NULL) {
        return;
    }

    g_yaw_lqr_ident_record.dt_s = dt;
    g_yaw_lqr_ident_record.valid = 0U;
    g_yaw_lqr_ident_record.feedback_ok = input->feedback_ok;
    g_yaw_lqr_ident_record.ref_traj_valid = input->ref_traj_valid;
    g_yaw_lqr_ident_record.theta_ref_cmd_deg = input->theta_ref_deg;
    g_yaw_lqr_ident_record.theta_ref_rad = input->theta_ref_rad;
    g_yaw_lqr_ident_record.omega_ref_rad_s = input->omega_ref_rad_s;
    g_yaw_lqr_ident_record.alpha_ref_rad_s2 = input->alpha_ref_rad_s2;
    g_yaw_lqr_ident_record.theta_meas_rad = input->theta_rad;
    g_yaw_lqr_ident_record.omega_meas_rad_s = input->omega_rad_s;
    g_yaw_lqr_ident_record.e_theta_rad = 0.0f;
    g_yaw_lqr_ident_record.e_omega_rad_s = 0.0f;
    g_yaw_lqr_ident_record.tau_lqr_nm = 0.0f;
    g_yaw_lqr_ident_record.tau_bias_nm = 0.0f;
    g_yaw_lqr_ident_record.tau_cmd_nm = 0.0f;
    g_yaw_lqr_ident_record.tau_meas_nm = input->tau_meas_nm;
    g_yaw_lqr_ident_record.tau_meas_filt_nm = 0.0f;
    g_yaw_lqr_ident_record.tau_track_err_nm = 0.0f;
}

static void RefPlanner_Reset(YawLQRESO_s *ctrl, float theta_ref_rad)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->theta_ref_planned_rad = theta_ref_rad;
    ctrl->omega_ref_planned     = 0.0f;
    ctrl->alpha_ref_planned     = 0.0f;
}

static void RefPlanner_Update(YawLQRESO_s *ctrl, float theta_target_rad, float dt)
{
    float old_theta_ref;
    float omega_limit;
    float err;

    if (ctrl == NULL) {
        return;
    }

    omega_limit = LQRESOAbs(g_ref_omega_limit_rad_s);
    theta_target_rad = LQRESOWrapNearestRad(theta_target_rad, ctrl->theta_ref_planned_rad);
    old_theta_ref = ctrl->theta_ref_planned_rad;
    err           = theta_target_rad - old_theta_ref;

    ctrl->theta_ref_planned_rad = theta_target_rad;
    if (LQRESOAbs(err) < 1e-6f) {
        ctrl->omega_ref_planned     = 0.0f;
        ctrl->alpha_ref_planned     = 0.0f;
    } else {
        ctrl->omega_ref_planned = err / dt;
        if (omega_limit > 0.0f) {
            ctrl->omega_ref_planned = LQRESOClamp(ctrl->omega_ref_planned, -omega_limit, omega_limit);
        }
        ctrl->alpha_ref_planned = 0.0f;
    }
}

static void RefPlanner_UpdateTrajectory(YawLQRESO_s *ctrl,
                                        float theta_ref_rad,
                                        float omega_ref_rad_s,
                                        float alpha_ref_rad_s2)
{
    float omega_limit;

    if (ctrl == NULL) {
        return;
    }

    omega_limit = LQRESOAbs(g_ref_omega_limit_rad_s);
    ctrl->theta_ref_planned_rad = theta_ref_rad;
    ctrl->omega_ref_planned = omega_ref_rad_s;
    if (omega_limit > 0.0f) {
        ctrl->omega_ref_planned =
            LQRESOClamp(ctrl->omega_ref_planned, -omega_limit, omega_limit);
    }
    ctrl->alpha_ref_planned =
        LQRESOClampAbs(alpha_ref_rad_s2, REF_ALPHA_LIMIT_RAD_S2_DEFAULT);
}

static void RefPlanner_UpdateAutoAimTrajectory(YawLQRESO_s *ctrl,
                                               float theta_in_rad,
                                               float omega_in_rad_s,
                                               float alpha_in_rad_s2,
                                               float dt)
{
    float omega_limit;
    float change_eps;
    float rc;
    float stale_timeout;
    float decay_tau;
    float lookahead_s;
    float theta_out;
    float omega_out;
    float alpha_out;
    uint8_t changed;

    if (ctrl == NULL) {
        return;
    }

    change_eps    = LQRESOAbs(g_traj_change_eps_rad);
    rc            = (g_traj_alpha_lpf_rc_s > 0.0f) ? g_traj_alpha_lpf_rc_s : 0.0f;
    stale_timeout = (g_traj_stale_timeout_s > 0.0f) ? g_traj_stale_timeout_s : 0.0f;
    decay_tau     = (g_traj_decay_tau_s > 0.0f) ? g_traj_decay_tau_s : 1e-3f;
    lookahead_s   = (g_traj_lookahead_s > 0.0f) ? g_traj_lookahead_s : 0.0f;
    omega_limit   = LQRESOAbs(g_ref_omega_limit_rad_s);

    /* (1) First entry: latch directly, prime LPF with input alpha. */
    if (ctrl->traj_initialized == 0U) {
        ctrl->traj_latched_theta_rad     = theta_in_rad;
        ctrl->traj_latched_omega_rad_s   = omega_in_rad_s;
        ctrl->traj_latched_alpha_rad_s2  = alpha_in_rad_s2;
        ctrl->traj_alpha_filt_rad_s2     = alpha_in_rad_s2;
        ctrl->traj_dt_since_latch_s      = 0.0f;
        ctrl->traj_last_input_theta_rad  = theta_in_rad;
        ctrl->traj_last_input_omega_rad_s = omega_in_rad_s;
        ctrl->traj_last_input_alpha_rad_s2 = alpha_in_rad_s2;
        ctrl->traj_initialized           = 1U;

        theta_out = theta_in_rad;
        omega_out = omega_in_rad_s;
        alpha_out = alpha_in_rad_s2;
    } else {
        /* (2) Change detection vs last firmware tick. */
        changed = (LQRESOAbs(theta_in_rad - ctrl->traj_last_input_theta_rad) > change_eps) ||
                  (LQRESOAbs(omega_in_rad_s - ctrl->traj_last_input_omega_rad_s) > change_eps) ||
                  (LQRESOAbs(alpha_in_rad_s2 - ctrl->traj_last_input_alpha_rad_s2) > change_eps);
        ctrl->traj_last_input_theta_rad   = theta_in_rad;
        ctrl->traj_last_input_omega_rad_s = omega_in_rad_s;
        ctrl->traj_last_input_alpha_rad_s2 = alpha_in_rad_s2;

        if (changed != 0U) {
            /* (3a) New PC packet: snap (theta, omega), LPF on alpha (over inter-packet dt). */
            const float dt_pkt = ctrl->traj_dt_since_latch_s;
            const float beta_denom = rc + dt_pkt;
            float beta;
            if (beta_denom > 1e-9f) {
                beta = dt_pkt / beta_denom;
                if (beta < 0.0f) {
                    beta = 0.0f;
                }
                if (beta > 1.0f) {
                    beta = 1.0f;
                }
            } else {
                beta = 0.0f;
            }
            ctrl->traj_alpha_filt_rad_s2 +=
                beta * (alpha_in_rad_s2 - ctrl->traj_alpha_filt_rad_s2);
            ctrl->traj_latched_theta_rad    = theta_in_rad;
            ctrl->traj_latched_omega_rad_s  = omega_in_rad_s;
            ctrl->traj_latched_alpha_rad_s2 = alpha_in_rad_s2;
            ctrl->traj_dt_since_latch_s     = 0.0f;
        } else {
            /* (3b) No new packet: accumulate time. */
            ctrl->traj_dt_since_latch_s += dt;
        }

        /* (4) Compute current output. */
        {
            const float tau = ctrl->traj_dt_since_latch_s;
            const float w_l = ctrl->traj_latched_omega_rad_s;
            const float a_f = ctrl->traj_alpha_filt_rad_s2;
            const float th_l = ctrl->traj_latched_theta_rad;

            if (stale_timeout > 0.0f && tau > stale_timeout) {
                /* (4a) Staleness: freeze theta at stale entry; decay omega/alpha. */
                const float d = stale_timeout;
                const float theta_freeze = th_l + w_l * d + 0.5f * a_f * d * d;
                const float omega_at_freeze = w_l + a_f * d;
                const float decay = expf(-(tau - stale_timeout) / decay_tau);
                theta_out = theta_freeze;
                omega_out = omega_at_freeze * decay;
                alpha_out = a_f * decay;
            } else {
                /* (4b) Normal constant-acceleration extrapolation. */
                theta_out = th_l + w_l * tau + 0.5f * a_f * tau * tau;
                omega_out = w_l + a_f * tau;
                alpha_out = a_f;
            }
        }
    }

    /* (5) Optional lookahead. */
    if (lookahead_s > 0.0f) {
        const float L = lookahead_s;
        theta_out += omega_out * L + 0.5f * alpha_out * L * L;
        omega_out += alpha_out * L;
    }

    /* (6) Clamping (matches the legacy RefPlanner_UpdateTrajectory). */
    if (omega_limit > 0.0f) {
        omega_out = LQRESOClamp(omega_out, -omega_limit, omega_limit);
    }
    alpha_out = LQRESOClampAbs(alpha_out, REF_ALPHA_LIMIT_RAD_S2_DEFAULT);

    ctrl->theta_ref_planned_rad = theta_out;
    ctrl->omega_ref_planned     = omega_out;
    ctrl->alpha_ref_planned     = alpha_out;
}

void YawLQRESO_Init(YawLQRESO_s *ctrl)
{
    if (ctrl == NULL) {
        return;
    }

    memset(ctrl, 0, sizeof(*ctrl));
    YawEso_InitInternal(ctrl);
}

void YawLQRESO_Reset(YawLQRESO_s *ctrl)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->feedback_ready         = 0U;
    ctrl->eso.z1                 = 0.0f;
    ctrl->eso.z2                 = 0.0f;
    ctrl->eso.z3                 = 0.0f;
    ctrl->eso_u_last             = 0.0f;
    ctrl->torque_cmd_last        = 0.0f;
    ctrl->tau_bias               = 0.0f;
    ctrl->tau_meas_filt          = 0.0f;
    ctrl->lqi_int                = 0.0f;
    ctrl->theta_ref_planned_rad  = 0.0f;
    ctrl->omega_ref_planned      = 0.0f;
    ctrl->alpha_ref_planned      = 0.0f;
    ctrl->limit_active           = 0U;
    g_lqr_e_theta                = 0.0f;
    g_lqr_omega_filt             = 0.0f;
    g_lqr_tau_raw                = 0.0f;
    g_tau_track_err              = 0.0f;
    g_theta_ref_planned          = 0.0f;
    g_omega_ref_planned          = 0.0f;
    g_alpha_ref_planned          = 0.0f;
    g_yaw_eso_tau_comp           = 0.0f;
    g_yaw_auto_eso_comp_active   = 0.0f;
    YawPublishObserver(ctrl);
}

void YawLQRESO_Calc(YawLQRESO_s *ctrl, const YawControlInput_t *input, float dt, YawControlOutput_t *output)
{
    float theta_target_rad;
    float theta_ref_rad;
    float omega_ref;
    float alpha_ref;
    float tau_for_eso;
    float e_theta_raw;
    float e_theta;
    float e_omega;
    float tau_ff;
    float tau_lqr;
    float tau_comp;
    float eso_comp_gain_max;
    float eso_comp_limit_max;
    float tau_cmd;
    float soft_limit;
    float torque_slew_rate;
    float old_tau_cmd;
    float k_theta;
    float k_omega;
    float alpha;
    float blend;
    uint8_t limited = 0U;
    uint8_t auto_eso_comp_active = 0U;

    if (output != NULL) {
        output->tau_cmd_nm         = 0.0f;
        output->theta_ref_rad      = 0.0f;
        output->omega_ref_rad_s    = 0.0f;
        output->alpha_ref_rad_s2   = 0.0f;
    }
    if (ctrl == NULL || input == NULL) {
        return;
    }

    dt = LQRESOSanitizeDt(dt);
    YawLqrIdentRecordPublishInput(input, dt);

    if (!input->feedback_ok) {
        YawLQRESO_Reset(ctrl);
        return;
    }

    YawEso_UpdateGains(ctrl);

    theta_target_rad = (input->ref_traj_valid != 0U) ?
        input->theta_ref_rad : (input->theta_ref_deg * LQR_ESO_DEG_TO_RAD);
    theta_target_rad = LQRESOWrapNearestRad(theta_target_rad, input->theta_rad);
    if (g_ctrl_max_error_rad > 0.0f) {
        theta_target_rad = LQRESOClamp(theta_target_rad,
                                       input->theta_rad - g_ctrl_max_error_rad,
                                       input->theta_rad + g_ctrl_max_error_rad);
    }

    if (!ctrl->feedback_ready) {
        ctrl->feedback_ready = 1U;
        ctrl->tau_meas_filt  = input->tau_meas_nm;
        RefPlanner_Reset(ctrl, input->theta_rad);
        YawEso_ResetInternal(ctrl, input->theta_rad, input->omega_rad_s);
        g_yaw_lqr_ident_record.feedback_ok = 1U;
        g_yaw_lqr_ident_record.theta_ref_rad = input->theta_rad;
        g_yaw_lqr_ident_record.omega_ref_rad_s = 0.0f;
        g_yaw_lqr_ident_record.alpha_ref_rad_s2 = 0.0f;
        g_yaw_lqr_ident_record.tau_meas_filt_nm = ctrl->tau_meas_filt;
        if (output != NULL) {
            output->tau_cmd_nm       = 0.0f;
            output->theta_ref_rad    = input->theta_rad;
            output->omega_ref_rad_s  = 0.0f;
            output->alpha_ref_rad_s2 = 0.0f;
        }
        return;
    }

    alpha = LQRESOClamp(g_tau_meas_lpf_alpha, 0.0f, 1.0f);
    ctrl->tau_meas_filt += alpha * (input->tau_meas_nm - ctrl->tau_meas_filt);

    if (input->ref_traj_valid != 0U) {
        RefPlanner_UpdateTrajectory(ctrl,
                                    theta_target_rad,
                                    input->omega_ref_rad_s,
                                    input->alpha_ref_rad_s2);
    } else {
        RefPlanner_Update(ctrl, theta_target_rad, dt);
    }
    theta_ref_rad = ctrl->theta_ref_planned_rad;
    omega_ref     = ctrl->omega_ref_planned;
    alpha_ref     = ctrl->alpha_ref_planned;

    blend       = LQRESOClamp(g_eso_torque_feedback_blend, 0.0f, 1.0f);
    tau_for_eso = (1.0f - blend) * ctrl->eso_u_last + blend * ctrl->tau_meas_filt;
    YawEso_Update(ctrl, input->theta_rad, tau_for_eso, dt);

    e_theta_raw = input->theta_rad - theta_ref_rad;
    e_theta     = LQRESOApplyDeadband(e_theta_raw, g_ctrl_angle_deadband_rad);
    e_omega     = input->omega_rad_s - omega_ref;
    k_theta     = (input->auto_aim_mode != 0U) ? g_lqr_auto_k_theta : g_lqr_k_theta;
    k_omega     = (input->auto_aim_mode != 0U) ? g_lqr_auto_k_omega : g_lqr_k_omega;
    tau_ff      = YAW_J_DEFAULT * alpha_ref + YAW_B_DEFAULT * omega_ref;
    tau_lqr     = tau_ff - k_theta * e_theta - k_omega * e_omega - g_lqi_ki * ctrl->lqi_int;

    tau_comp = 0.0f;
    eso_comp_gain_max = LQRESOAbs(g_eso_comp_gain_max);
    eso_comp_limit_max = LQRESOAbs(g_eso_comp_limit_max);
    g_eso_comp_gain_max = eso_comp_gain_max;
    g_eso_comp_limit_max = eso_comp_limit_max;
    if (LQRESOAbs(ctrl->eso.b0) > 1e-6f) {
        const float eso_comp_gain = LQRESOClamp(g_eso_comp_gain, 0.0f, eso_comp_gain_max);
        g_eso_comp_gain = eso_comp_gain;
        tau_comp = -eso_comp_gain * ctrl->eso.z3 / ctrl->eso.b0;
    }
    g_eso_comp_limit = LQRESOClamp(LQRESOAbs(g_eso_comp_limit), 0.0f, eso_comp_limit_max);
    tau_comp = LQRESOClampAbs(tau_comp, g_eso_comp_limit);

    g_yaw_auto_eso_comp_omega_gate_rad_s =
        LQRESOAbs(g_yaw_auto_eso_comp_omega_gate_rad_s);
    if (input->auto_aim_mode != 0U &&
        g_yaw_auto_eso_comp_enable != 0U &&
        LQRESOAbs(input->omega_rad_s) <= g_yaw_auto_eso_comp_omega_gate_rad_s) {
        auto_eso_comp_active = 1U;
    }

    tau_cmd    = tau_lqr + ((auto_eso_comp_active != 0U) ? tau_comp : 0.0f);
    g_yaw_auto_eso_comp_active = (float)auto_eso_comp_active;
    soft_limit = LQRESOAbs(g_ctrl_t_limit);
    if (soft_limit > 0.0f) {
        const float limited_tau = LQRESOClampAbs(tau_cmd, soft_limit);
        limited = (limited_tau != tau_cmd) ? 1U : limited;
        tau_cmd = limited_tau;
    }

    if (!limited) {
        if (LQRESOAbs(e_theta_raw) <= g_lqi_integral_enable_rad) {
            ctrl->lqi_int += e_theta_raw * dt;
            ctrl->lqi_int = LQRESOClampAbs(ctrl->lqi_int, g_lqi_int_limit);
        }

        g_tau_track_err = tau_cmd - ctrl->tau_meas_filt;
        ctrl->tau_bias += g_tau_bias_ki * g_tau_track_err * dt;
        ctrl->tau_bias = LQRESOClampAbs(ctrl->tau_bias, g_tau_bias_limit);
    }

    tau_cmd += ctrl->tau_bias;
    if (soft_limit > 0.0f) {
        const float limited_tau = LQRESOClampAbs(tau_cmd, soft_limit);
        limited = (limited_tau != tau_cmd) ? 1U : limited;
        tau_cmd = limited_tau;
    }

    if (tau_cmd < DM_T_MIN) {
        tau_cmd = DM_T_MIN;
        limited = 1U;
    } else if (tau_cmd > DM_T_MAX) {
        tau_cmd = DM_T_MAX;
        limited = 1U;
    }

    torque_slew_rate = LQRESOAbs(input->torque_slew_rate_nm_s);
    if (input->torque_slew_enable != 0U && torque_slew_rate > 0.0f) {
        const float max_delta = torque_slew_rate * dt;
        old_tau_cmd = tau_cmd;
        tau_cmd = LQRESOClamp(tau_cmd, ctrl->torque_cmd_last - max_delta, ctrl->torque_cmd_last + max_delta);
        limited = (tau_cmd != old_tau_cmd) ? 1U : limited;
    }

    if (input->auto_aim_mode == 0U &&
        g_ctrl_min_torque_comp > 0.0f &&
        LQRESOAbs(input->omega_rad_s) < g_ctrl_vel_deadband &&
        LQRESOAbs(e_theta_raw) > g_ctrl_angle_deadband_rad &&
        LQRESOAbs(tau_cmd) < g_ctrl_min_torque_comp) {
        tau_cmd = -LQRESOSign(e_theta_raw) * g_ctrl_min_torque_comp;
        tau_cmd = LQRESOClamp(tau_cmd, DM_T_MIN, DM_T_MAX);
    }

    ctrl->torque_cmd_last = tau_cmd;
    ctrl->eso_u_last      = tau_cmd;
    ctrl->limit_active    = limited;

    g_lqr_e_theta       = e_theta;
    g_lqr_omega_filt    = input->omega_rad_s;
    g_lqr_tau_raw       = tau_lqr;
    g_theta_ref_planned = theta_ref_rad * LQR_ESO_RAD_TO_DEG;
    g_omega_ref_planned = omega_ref;
    g_alpha_ref_planned = alpha_ref;
    g_yaw_eso_tau_comp  = tau_comp;
    YawPublishObserver(ctrl);

    g_yaw_lqr_ident_record.valid = 1U;
    g_yaw_lqr_ident_record.feedback_ok = 1U;
    g_yaw_lqr_ident_record.theta_ref_rad = theta_ref_rad;
    g_yaw_lqr_ident_record.omega_ref_rad_s = omega_ref;
    g_yaw_lqr_ident_record.alpha_ref_rad_s2 = alpha_ref;
    g_yaw_lqr_ident_record.e_theta_rad = e_theta;
    g_yaw_lqr_ident_record.e_omega_rad_s = e_omega;
    g_yaw_lqr_ident_record.tau_lqr_nm = tau_lqr;
    g_yaw_lqr_ident_record.tau_bias_nm = ctrl->tau_bias;
    g_yaw_lqr_ident_record.tau_cmd_nm = tau_cmd;
    g_yaw_lqr_ident_record.tau_meas_filt_nm = ctrl->tau_meas_filt;
    g_yaw_lqr_ident_record.tau_track_err_nm = tau_cmd - ctrl->tau_meas_filt;

    if (output != NULL) {
        output->tau_cmd_nm       = tau_cmd;
        output->theta_ref_rad    = theta_ref_rad;
        output->omega_ref_rad_s  = omega_ref;
        output->alpha_ref_rad_s2 = alpha_ref;
    }
}

static void SimpleLQRESO_InitInternal(LQRESO_SimpleController_s *ctrl,
                                      const LQR_Config_s *lqr_config,
                                      const ESO_Config_s *eso_config)
{
    if (ctrl == NULL) {
        return;
    }

    memset(ctrl, 0, sizeof(*ctrl));
    if (lqr_config != NULL) {
        ctrl->lqr_config = *lqr_config;
        LQRConfigResolvePitchModel(&ctrl->lqr_config);
    }
    if (eso_config != NULL) {
        ctrl->eso_config = *eso_config;
    }
}

static void SimpleLQRESO_ResetInternal(LQRESO_SimpleController_s *ctrl)
{
    if (ctrl == NULL) {
        return;
    }

    memset(ctrl->state, 0, sizeof(ctrl->state));
    memset(&ctrl->eso, 0, sizeof(ctrl->eso));
    ctrl->output = 0.0f;
    ctrl->torque = 0.0f;
    ctrl->k_theta_used = 0.0f;
    ctrl->k_omega_used = 0.0f;
    ctrl->tau_lqr = 0.0f;
    ctrl->tau_eso_comp = 0.0f;
    ctrl->tau_pre_limit = 0.0f;
}

static void PitchPublishObserver(const PitchLQRESO_s *ctrl, float tau_comp)
{
    if (ctrl == NULL) {
        g_pitch_eso_alpha     = 0.0f;
        g_pitch_eso_comp_gain = 0.0f;
        g_pitch_eso_comp_limit = 0.0f;
        g_pitch_eso_b0        = 0.0f;
        g_pitch_eso_w0        = 0.0f;
        g_pitch_eso_beta1     = 0.0f;
        g_pitch_eso_beta2     = 0.0f;
        g_pitch_eso_beta3     = 0.0f;
        g_pitch_eso_z1        = 0.0f;
        g_pitch_eso_z2        = 0.0f;
        g_pitch_eso_z3        = 0.0f;
        g_pitch_eso_tau_comp  = 0.0f;
        return;
    }

    g_pitch_eso_alpha      = ctrl->eso_config.alpha;
    g_pitch_eso_comp_gain  = ctrl->eso_config.comp_gain;
    g_pitch_eso_comp_limit = ctrl->eso_config.comp_limit;
    g_pitch_eso_b0         = ctrl->eso.b0;
    g_pitch_eso_w0         = ctrl->eso.w0;
    g_pitch_eso_beta1      = ctrl->eso.beta1;
    g_pitch_eso_beta2      = ctrl->eso.beta2;
    g_pitch_eso_beta3      = ctrl->eso.beta3;
    g_pitch_eso_z1         = ctrl->eso.z1;
    g_pitch_eso_z2         = ctrl->eso.z2;
    g_pitch_eso_z3         = ctrl->eso.z3;
    g_pitch_eso_tau_comp   = tau_comp;
}

static void PitchResolveLQRGains(const PitchLQRESO_s *ctrl,
                                 float *k_theta,
                                 float *k_omega)
{
    if (k_theta == NULL || k_omega == NULL) {
        return;
    }

    *k_theta = PITCH_LQR_K_THETA_DEFAULT;
    *k_omega = PITCH_LQR_K_OMEGA_DEFAULT;
    if (ctrl == NULL) {
        return;
    }

    if (ctrl->lqr_config.state_dim >= 1U &&
        ctrl->lqr_config.k[0] != 0.0f) {
        *k_theta = ctrl->lqr_config.k[0];
    } else if (ctrl->lqr_config.model_enable != 0U &&
               ctrl->lqr_config.model_k_theta != 0.0f) {
        *k_theta = ctrl->lqr_config.model_k_theta;
    }

    if (ctrl->lqr_config.state_dim >= 2U &&
        ctrl->lqr_config.k[1] != 0.0f) {
        *k_omega = ctrl->lqr_config.k[1];
    } else if (ctrl->lqr_config.model_enable != 0U &&
               ctrl->lqr_config.model_k_omega != 0.0f) {
        *k_omega = ctrl->lqr_config.model_k_omega;
    }
}

static void PitchEso_UpdateGains(PitchLQRESO_s *ctrl,
                                 float k_theta,
                                 float k_omega)
{
    float j;
    float b;
    float b0;
    float alpha;
    float wn_term;
    float wn;
    float sigma;
    float w_cl;
    float w0;
    float w0_min;
    float w0_max;

    if (ctrl == NULL) {
        return;
    }

    if (ctrl->eso_config.enable == 0U) {
        ctrl->eso.b0 = 0.0f;
        ctrl->eso.w0 = 0.0f;
        ctrl->eso.beta1 = 0.0f;
        ctrl->eso.beta2 = 0.0f;
        ctrl->eso.beta3 = 0.0f;
        return;
    }

    j = ctrl->lqr_config.plant_j_kg_m2;
    b = ctrl->lqr_config.plant_b_nms_rad;
    if (j <= PITCH_MODEL_MIN_J && ctrl->eso_config.b0 > PITCH_ESO_MIN_B0) {
        j = 1.0f / ctrl->eso_config.b0;
    }
    if (j <= PITCH_MODEL_MIN_J) {
        ctrl->eso.b0 = 0.0f;
        ctrl->eso.w0 = 0.0f;
        ctrl->eso.beta1 = 0.0f;
        ctrl->eso.beta2 = 0.0f;
        ctrl->eso.beta3 = 0.0f;
        return;
    }

    b0 = 1.0f / j;
    ctrl->lqr_config.model_a22 = -b / j;
    ctrl->lqr_config.model_b2 = b0;

    alpha = (ctrl->eso_config.alpha > 0.0f) ? ctrl->eso_config.alpha : 0.0f;
    ctrl->eso_config.alpha = alpha;

    wn_term = k_theta / j;
    wn = (wn_term > 0.0f) ? sqrtf(wn_term) : 0.0f;
    sigma = (b + k_omega) / (2.0f * j);
    sigma = (sigma > 0.0f) ? sigma : 0.0f;
    w_cl = (wn > sigma) ? wn : sigma;

    w0_min = (ctrl->eso_config.w0_min > 0.0f) ?
        ctrl->eso_config.w0_min : 0.0f;
    w0_max = (ctrl->eso_config.w0_max > 0.0f) ?
        ctrl->eso_config.w0_max : 0.0f;
    if (w0_max > 0.0f && w0_min > w0_max) {
        w0_min = w0_max;
    }

    w0 = alpha * w_cl;
    if (w0 < w0_min) {
        w0 = w0_min;
    }
    if (w0_max > 0.0f && w0 > w0_max) {
        w0 = w0_max;
    }

    ctrl->eso.b0 = b0;
    ctrl->eso.w0 = w0;
    ctrl->eso.beta1 = 3.0f * w0;
    ctrl->eso.beta2 = 3.0f * w0 * w0;
    ctrl->eso.beta3 = w0 * w0 * w0;
}

static void PitchEso_Update(PitchLQRESO_s *ctrl,
                            float measure,
                            float u_last,
                            float dt)
{
    SimpleEso_t *eso;
    float e;
    float z1_dot;
    float z2_dot;
    float z3_dot;
    float a22;

    if (ctrl == NULL || ctrl->eso_config.enable == 0U ||
        ctrl->eso.b0 <= PITCH_ESO_MIN_B0 ||
        ctrl->eso.w0 <= 0.0f) {
        return;
    }

    eso = &ctrl->eso;
    a22 = ctrl->lqr_config.model_a22;
    e = measure - eso->z1;
    z1_dot = eso->z2 + eso->beta1 * e;
    z2_dot = a22 * eso->z2 + eso->b0 * u_last + eso->z3 +
             eso->beta2 * e;
    z3_dot = eso->beta3 * e;

    eso->z1 += dt * z1_dot;
    eso->z2 += dt * z2_dot;
    eso->z3 += dt * z3_dot;
}

static float PitchEso_CalcComp(PitchLQRESO_s *ctrl)
{
    float comp_gain;
    float comp_limit;
    float tau_comp;

    if (ctrl == NULL || ctrl->eso_config.enable == 0U ||
        ctrl->eso.b0 <= PITCH_ESO_MIN_B0 ||
        ctrl->eso.w0 <= 0.0f) {
        return 0.0f;
    }

    comp_gain = (ctrl->eso_config.comp_gain > 0.0f) ?
        ctrl->eso_config.comp_gain : 0.0f;
    comp_limit = LQRESOAbs(ctrl->eso_config.comp_limit);
    comp_limit = LQRESOClamp(comp_limit, 0.0f, DM_T_MAX);
    ctrl->eso_config.comp_gain = comp_gain;
    ctrl->eso_config.comp_limit = comp_limit;

    if (comp_limit <= 0.0f || comp_gain <= 0.0f) {
        return 0.0f;
    }

    tau_comp = -comp_gain * ctrl->eso.z3 / ctrl->eso.b0;
    return LQRESOClamp(tau_comp, -comp_limit, comp_limit);
}

static float SimpleLQRESO_CalcInternal(LQRESO_SimpleController_s *ctrl,
                                       float ref,
                                       float measure,
                                       float speed,
                                       float feedforward)
{
    (void)ref;
    (void)measure;
    (void)speed;
    (void)feedforward;

    if (ctrl == NULL) {
        return 0.0f;
    }
    ctrl->output = 0.0f;
    ctrl->torque = 0.0f;
    return 0.0f;
}

void PitchLQRESO_Init(PitchLQRESO_s *ctrl, const LQR_Config_s *lqr_config, const ESO_Config_s *eso_config)
{
    SimpleLQRESO_InitInternal(ctrl, lqr_config, eso_config);
}

void PitchLQRESO_Reset(PitchLQRESO_s *ctrl)
{
    SimpleLQRESO_ResetInternal(ctrl);
    PitchPublishObserver(ctrl, 0.0f);
}

float PitchLQRESO_Calc(PitchLQRESO_s *ctrl, float ref, float measure, float speed, float feedforward, float dt)
{
    float k_theta;
    float k_omega;
    float torque_limit;
    float torque_slew_rate;
    float velocity_limit;
    float e_theta;
    float e_omega;
    float tau_lqr;
    float tau_eso_comp;
    float tau_cmd;
    float old_tau_cmd;
    uint8_t limited = 0U;

    if (ctrl == NULL) {
        return 0.0f;
    }

    dt = LQRESOSanitizeDt(dt);
    PitchResolveLQRGains(ctrl, &k_theta, &k_omega);
    ctrl->k_theta_used = k_theta;
    ctrl->k_omega_used = k_omega;
    PitchEso_UpdateGains(ctrl, k_theta, k_omega);

    torque_limit = PITCH_CTRL_T_LIMIT_DEFAULT;
    if (ctrl->lqr_config.output_limit > 0.0f) {
        torque_limit = ctrl->lqr_config.output_limit;
    }
    torque_limit = LQRESOClamp(torque_limit, 0.0f, DM_T_MAX);
    torque_slew_rate = PITCH_CTRL_TORQUE_SLEW_RATE_DEFAULT;
    if (ctrl->lqr_config.output_slew_rate > 0.0f) {
        torque_slew_rate = ctrl->lqr_config.output_slew_rate;
    }
    velocity_limit = LQRESOAbs(ctrl->lqr_config.velocity_limit);

    if (ctrl->state[PITCH_STATE_READY] < 0.5f) {
        ctrl->state[PITCH_STATE_READY] = 1.0f;
        ctrl->state[PITCH_STATE_LAST_TORQUE] = 0.0f;
        ctrl->state[PITCH_STATE_REF] = ref;
        ctrl->state[PITCH_STATE_MEASURE] = measure;
        ctrl->state[PITCH_STATE_LIMIT_ACTIVE] = 0.0f;
        ctrl->output = 0.0f;
        ctrl->torque = 0.0f;
        ctrl->tau_lqr = 0.0f;
        ctrl->tau_eso_comp = 0.0f;
        ctrl->tau_pre_limit = 0.0f;
        ctrl->eso.z1 = measure;
        ctrl->eso.z2 = speed;
        ctrl->eso.z3 = 0.0f;
        ctrl->eso.u_last = 0.0f;
        PitchPublishObserver(ctrl, 0.0f);
        return 0.0f;
    }

    PitchEso_Update(ctrl, measure, ctrl->eso.u_last, dt);

    e_theta = measure - ref;
    e_omega = speed;
    tau_lqr = -k_theta * e_theta - k_omega * e_omega;
    tau_eso_comp = PitchEso_CalcComp(ctrl);
    tau_cmd = tau_lqr + feedforward + tau_eso_comp;
    ctrl->tau_lqr = tau_lqr;
    ctrl->tau_eso_comp = tau_eso_comp;
    ctrl->tau_pre_limit = tau_cmd;

    old_tau_cmd = tau_cmd;
    tau_cmd = LQRESOClampAbs(tau_cmd, torque_limit);
    limited = (tau_cmd != old_tau_cmd) ? 1U : limited;
    old_tau_cmd = tau_cmd;
    tau_cmd = LQRESOClamp(tau_cmd, DM_T_MIN, DM_T_MAX);
    limited = (tau_cmd != old_tau_cmd) ? 1U : limited;

    if (torque_slew_rate > 0.0f) {
        const float max_delta = torque_slew_rate * dt;
        old_tau_cmd = tau_cmd;
        tau_cmd = LQRESOClamp(tau_cmd,
                              ctrl->state[PITCH_STATE_LAST_TORQUE] - max_delta,
                              ctrl->state[PITCH_STATE_LAST_TORQUE] + max_delta);
        limited = (tau_cmd != old_tau_cmd) ? 1U : limited;
    }
    if (velocity_limit > 0.0f) {
        old_tau_cmd = tau_cmd;
        if ((speed >= velocity_limit && tau_cmd > 0.0f) ||
            (speed <= -velocity_limit && tau_cmd < 0.0f)) {
            tau_cmd = 0.0f;
        }
        limited = (tau_cmd != old_tau_cmd) ? 1U : limited;
    }

    ctrl->state[PITCH_STATE_LAST_TORQUE] = tau_cmd;
    ctrl->state[PITCH_STATE_E_THETA] = e_theta;
    ctrl->state[PITCH_STATE_E_OMEGA] = e_omega;
    ctrl->state[PITCH_STATE_TAU_LQR] = tau_lqr;
    ctrl->state[PITCH_STATE_LIMIT_ACTIVE] = (float)limited;
    ctrl->state[PITCH_STATE_REF] = ref;
    ctrl->state[PITCH_STATE_MEASURE] = measure;
    ctrl->eso.u_last = tau_cmd;
    ctrl->output = tau_cmd;
    ctrl->torque = tau_cmd;
    PitchPublishObserver(ctrl, tau_eso_comp);
    return tau_cmd;
}

void ChassisLQRESO_Init(ChassisLQRESO_s *ctrl, const LQR_Config_s *lqr_config, const ESO_Config_s *eso_config)
{
    SimpleLQRESO_InitInternal(ctrl, lqr_config, eso_config);
}

void ChassisLQRESO_Reset(ChassisLQRESO_s *ctrl)
{
    SimpleLQRESO_ResetInternal(ctrl);
}

float ChassisLQRESO_Calc(ChassisLQRESO_s *ctrl, float ref, float measure, float speed, float feedforward, float dt)
{
    (void)dt;
    return SimpleLQRESO_CalcInternal(ctrl, ref, measure, speed, feedforward);
}
