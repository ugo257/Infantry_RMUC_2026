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

/*
 * Non-auto yaw plant defaults are synchronized with the current auto-aim model.
 * Non-auto K defaults use Qtheta=400, Qomega=0.5, R=1 with that plant.
 * Rollback values before sync: J=0.0491133f, B=0.0176839f,
 * Ktheta=43.7692812f, Komega=2.09040931f, Coulomb=0.0f.
 */
// #define YAW_J_DEFAULT                       0.033740f
#define YAW_J_DEFAULT                       YAW_AUTO_J_DEFAULT
// #define YAW_B_DEFAULT                       0.388210f
#define YAW_B_DEFAULT                       YAW_AUTO_B_DEFAULT
#define YAW_TS_DEFAULT                      0.001f
// #define LQR_K_THETA_DEFAULT                 27.7195954f//54.7722558f
#define LQR_K_THETA_DEFAULT                 20.0f
// #define LQR_K_OMEGA_DEFAULT                 1.33991836f//2.13106347f
#define LQR_K_OMEGA_DEFAULT                 1.68401458f
#define LQR_AUTO_K_THETA_DEFAULT            30.0f//67.6101582f
#define LQR_AUTO_K_OMEGA_DEFAULT            1.0f//2.58721444f
#define LQR_HOLE_OFFSET_K_THETA_DEFAULT     27.7195954f//20.0f
#define LQR_HOLE_OFFSET_K_OMEGA_DEFAULT     1.33991836f//1.0f
// #define YAW_TAU_COULOMB_DEFAULT             0.346952f
#define YAW_TAU_COULOMB_DEFAULT             YAW_AUTO_TAU_COULOMB_DEFAULT
// #define YAW_COULOMB_SMOOTH_OMEGA_DEFAULT    0.15f
#define YAW_COULOMB_SMOOTH_OMEGA_DEFAULT    YAW_AUTO_COULOMB_SMOOTH_OMEGA_DEFAULT
#define YAW_AUTO_FF_ALPHA_TAU_LIMIT_DEFAULT 7.0f
/*
 * Yaw auto-aim uses a CSV-identified plant from yawww.csv stage-2
 * motor_tor_nm fitting. Auto-aim volatile values remain available for Ozone
 * runtime tuning.
 */
#define YAW_AUTO_J_DEFAULT                   0.0610521813f//0.0422563489f
#define YAW_AUTO_B_DEFAULT                   0.513734025f//0.286217284f
#define YAW_AUTO_TAU_COULOMB_DEFAULT         0.3f
#define YAW_AUTO_COULOMB_SMOOTH_OMEGA_DEFAULT 0.150000006f
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
#define REF_OMEGA_LIMIT_DEFAULT             8.0f
#define REF_ALPHA_LIMIT_RAD_S2_DEFAULT      300.0f
#define CTRL_TORQUE_SLEW_RATE_DEFAULT       1000.0f
#define CTRL_AUTO_TORQUE_SLEW_RATE_DEFAULT  700.0f
#define CTRL_MIN_TORQUE_COMP_DEFAULT        0.40f
#define ESO_COMP_GAIN_DEFAULT               1.0f
#define ESO_COMP_LIMIT_DEFAULT              7.0f
#define YAW_ESO_ALPHA_DEFAULT               1.10f
#define YAW_ESO_W0_SET_DEFAULT              0.0f   /* 0 = follow non-auto K formula; >0 = override */
#define YAW_AUTO_ESO_W0_SET_DEFAULT         0.0f   /* 0 = follow K formula; >0 = override */
#define YAW_AUTO_ESO_COMP_ENABLE_DEFAULT    1U
#define YAW_AUTO_ESO_COMP_OMEGA_GATE_DEFAULT 50.0f
#define YAW_AUTO_ESO_COMP_ALPHA_GATE_RAD_S2_DEFAULT 150.0f
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
#define TRAJ_ALPHA_LPF_RC_DEFAULT           0.0f
#define TRAJ_LOOKAHEAD_DEFAULT              0.0f
#define TRAJ_CHANGE_EPS_RAD_DEFAULT         1e-6f
#define TRAJ_PACKET_MIN_DT_S                0.0001f
#define TRAJ_PACKET_MAX_DT_S                0.1000f
/* Yaw identification excitation injector defaults. */
#define YAW_IDENT_EXCITE_ENABLE_DEFAULT       0U
#define YAW_IDENT_EXCITE_TYPE_DEFAULT         0U
#define YAW_IDENT_EXCITE_AMP_NM_DEFAULT       0.8f
#define YAW_IDENT_EXCITE_F0_HZ_DEFAULT        0.5f
#define YAW_IDENT_EXCITE_F1_HZ_DEFAULT        15.0f
#define YAW_IDENT_EXCITE_DURATION_S_DEFAULT   8.0f
#define YAW_IDENT_EXCITE_AMP_LIMIT_NM_DEFAULT 2.0f
#define YAW_IDENT_EXCITE_AMP_HARD_LIMIT_NM    2.0f
#define YAW_IDENT_STAGE_LOW_DURATION_S        10.0f
#define YAW_IDENT_STAGE_LOW_AMP_NM            0.45f
#define YAW_IDENT_STAGE_LOW_AMP_LIMIT_NM      1.0f
#define YAW_IDENT_STAGE_LOW_F0_HZ             0.08f
#define YAW_IDENT_STAGE_LOW_F1_HZ             0.80f
#define YAW_IDENT_STAGE_DYNAMIC_DURATION_S    YAW_IDENT_EXCITE_DURATION_S_DEFAULT
#define YAW_IDENT_STAGE_DYNAMIC_AMP_NM        YAW_IDENT_EXCITE_AMP_NM_DEFAULT
#define YAW_IDENT_STAGE_DYNAMIC_AMP_LIMIT_NM  YAW_IDENT_EXCITE_AMP_LIMIT_NM_DEFAULT
#define YAW_IDENT_STAGE_DYNAMIC_F0_HZ         YAW_IDENT_EXCITE_F0_HZ_DEFAULT
#define YAW_IDENT_STAGE_DYNAMIC_F1_HZ         YAW_IDENT_EXCITE_F1_HZ_DEFAULT
#define YAW_MODEL_IDENT_K_THETA_DEFAULT       3.0f
#define YAW_MODEL_IDENT_K_OMEGA_DEFAULT       0.3f
#define YAW_MODEL_IDENT_LOW_SPEED_GATE_RAD_S  0.15f
#define YAW_BASE_OMEGA_LPF_TAU_DEFAULT        0.010f

volatile float g_lqr_k_theta                 = LQR_K_THETA_DEFAULT;
volatile float g_lqr_k_omega                 = LQR_K_OMEGA_DEFAULT;
volatile float g_lqr_auto_k_theta            = LQR_AUTO_K_THETA_DEFAULT;
volatile float g_lqr_auto_k_omega            = LQR_AUTO_K_OMEGA_DEFAULT;
volatile float g_lqr_hole_offset_k_theta     = LQR_HOLE_OFFSET_K_THETA_DEFAULT;
volatile float g_lqr_hole_offset_k_omega     = LQR_HOLE_OFFSET_K_OMEGA_DEFAULT;
volatile float g_yaw_j                       = YAW_J_DEFAULT;
volatile float g_yaw_b                       = YAW_B_DEFAULT;
volatile float g_yaw_tau_coulomb_nm          = YAW_TAU_COULOMB_DEFAULT;
volatile float g_yaw_coulomb_smooth_omega    = YAW_COULOMB_SMOOTH_OMEGA_DEFAULT;
volatile float g_yaw_auto_ff_alpha_tau_limit_nm = YAW_AUTO_FF_ALPHA_TAU_LIMIT_DEFAULT;
volatile float g_yaw_auto_j                  = YAW_AUTO_J_DEFAULT;
volatile float g_yaw_auto_b                  = YAW_AUTO_B_DEFAULT;
volatile float g_yaw_auto_tau_coulomb_nm     = YAW_AUTO_TAU_COULOMB_DEFAULT;
volatile float g_yaw_auto_coulomb_smooth_omega = YAW_AUTO_COULOMB_SMOOTH_OMEGA_DEFAULT;
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
volatile float g_yaw_auto_eso_w0_set         = YAW_AUTO_ESO_W0_SET_DEFAULT;
volatile float g_yaw_eso_w0                  = 0.0f;
volatile float g_yaw_eso_beta1               = 0.0f;
volatile float g_yaw_eso_beta2               = 0.0f;
volatile float g_yaw_eso_beta3               = 0.0f;
volatile float g_yaw_eso_z1                  = 0.0f;
volatile float g_yaw_eso_z2                  = 0.0f;
volatile float g_yaw_eso_z3                  = 0.0f;
volatile float g_yaw_eso_tau_comp            = 0.0f;
volatile uint8_t g_yaw_base_comp_enable      = 1U;
volatile float g_yaw_base_omega_gain         = 1.0f;
volatile float g_yaw_base_omega_lpf_tau_s    = YAW_BASE_OMEGA_LPF_TAU_DEFAULT;
volatile float g_yaw_base_omega_rad_s        = 0.0f;
volatile float g_yaw_omega_rel_ref_rad_s     = 0.0f;
volatile float g_yaw_omega_rel_meas_rad_s    = 0.0f;
volatile float g_yaw_eso_omega_rel_rad_s     = 0.0f;
volatile uint8_t g_yaw_auto_eso_comp_enable  = YAW_AUTO_ESO_COMP_ENABLE_DEFAULT;
volatile float g_yaw_auto_eso_comp_omega_gate_rad_s = YAW_AUTO_ESO_COMP_OMEGA_GATE_DEFAULT;
volatile float g_yaw_auto_eso_comp_alpha_gate_rad_s2 = YAW_AUTO_ESO_COMP_ALPHA_GATE_RAD_S2_DEFAULT;
volatile float g_yaw_auto_eso_comp_active    = 0.0f;
volatile float g_torque_cmd                  = 0.0f;
volatile uint32_t g_yaw_lqreso_control_seq   = 0U;
volatile YawLqrIdentRecord_s g_yaw_lqr_ident_record = {0};
volatile YawAutoAimDebugRecord_s g_yaw_auto_aim_debug_record = {0};
volatile YawModelIdentRecord_s g_yaw_model_ident = {
    .stage_select = YAW_MODEL_IDENT_STAGE_DEFAULT,
    .stage_id = YAW_MODEL_IDENT_STAGE_LOW_SPEED,
    .duration_s = YAW_IDENT_STAGE_LOW_DURATION_S,
    .amp_nm = YAW_IDENT_STAGE_LOW_AMP_NM,
    .amp_limit_nm = YAW_IDENT_STAGE_LOW_AMP_LIMIT_NM,
    .f_start_hz = YAW_IDENT_STAGE_LOW_F0_HZ,
    .f_end_hz = YAW_IDENT_STAGE_LOW_F1_HZ,
    .k_override_enable = 1U,
    .k_theta_ident = YAW_MODEL_IDENT_K_THETA_DEFAULT,
    .k_omega_ident = YAW_MODEL_IDENT_K_OMEGA_DEFAULT,
};
static uint8_t yaw_model_ident_run_latched = YAW_MODEL_IDENT_STAGE_DEFAULT;
static uint8_t yaw_model_ident_stage_latched = YAW_MODEL_IDENT_STAGE_LOW_SPEED;
static uint8_t yaw_model_ident_stage_switch_pending = 0U;
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
volatile float g_traj_lookahead_s           = TRAJ_LOOKAHEAD_DEFAULT;
volatile float g_traj_change_eps_rad        = TRAJ_CHANGE_EPS_RAD_DEFAULT;
volatile uint8_t g_yaw_ident_excite_enable  = YAW_IDENT_EXCITE_ENABLE_DEFAULT;
volatile uint8_t g_yaw_ident_excite_type    = YAW_IDENT_EXCITE_TYPE_DEFAULT;
volatile float g_yaw_ident_excite_amp_nm    = YAW_IDENT_EXCITE_AMP_NM_DEFAULT;
volatile float g_yaw_ident_excite_f0_hz     = YAW_IDENT_EXCITE_F0_HZ_DEFAULT;
volatile float g_yaw_ident_excite_f1_hz     = YAW_IDENT_EXCITE_F1_HZ_DEFAULT;
volatile float g_yaw_ident_excite_duration_s = YAW_IDENT_EXCITE_DURATION_S_DEFAULT;
volatile float g_yaw_ident_excite_amp_limit_nm = YAW_IDENT_EXCITE_AMP_LIMIT_NM_DEFAULT;

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

LQRESODebugHalf_t LQRESODebugFloatToHalf(float value)
{
    uint32_t bits;
    uint32_t sign;
    uint32_t mantissa;
    uint32_t half_mantissa;
    int32_t exponent;

    memcpy(&bits, &value, sizeof(bits));
    sign = (bits >> 16U) & 0x8000U;
    exponent = (int32_t)((bits >> 23U) & 0xffU);
    mantissa = bits & 0x007fffffU;

    if (exponent == 0xff) {
        return (LQRESODebugHalf_t)(sign | ((mantissa != 0U) ? 0x7e00U : 0x7c00U));
    }

    exponent = exponent - 127 + 15;
    if (exponent >= 31) {
        return (LQRESODebugHalf_t)(sign | 0x7c00U);
    }
    if (exponent <= 0) {
        if (exponent < -10) {
            return (LQRESODebugHalf_t)sign;
        }
        mantissa |= 0x00800000U;
        {
            const uint32_t shift = (uint32_t)(14 - exponent);
            half_mantissa = mantissa >> shift;
            if ((mantissa & (1UL << (shift - 1U))) != 0U) {
                half_mantissa++;
            }
        }
        if (half_mantissa >= 0x0400U) {
            return (LQRESODebugHalf_t)(sign | 0x0400U);
        }
        return (LQRESODebugHalf_t)(sign | half_mantissa);
    }

    half_mantissa = mantissa >> 13U;
    if ((mantissa & 0x00001000U) != 0U) {
        half_mantissa++;
        if (half_mantissa >= 0x0400U) {
            half_mantissa = 0U;
            exponent++;
            if (exponent >= 31) {
                return (LQRESODebugHalf_t)(sign | 0x7c00U);
            }
        }
    }
    return (LQRESODebugHalf_t)(sign | ((uint32_t)exponent << 10U) | half_mantissa);
}

static void YawSelectPlant(uint8_t auto_aim_mode, float *yaw_j, float *yaw_b)
{
    float j = (auto_aim_mode != 0U) ? g_yaw_auto_j : g_yaw_j;
    float b = (auto_aim_mode != 0U) ? g_yaw_auto_b : g_yaw_b;

    if (j <= 1.0e-6f) {
        j = YAW_J_DEFAULT;
    }
    if (b < 0.0f) {
        b = YAW_B_DEFAULT;
    }

    if (yaw_j != NULL) {
        *yaw_j = j;
    }
    if (yaw_b != NULL) {
        *yaw_b = b;
    }
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

static float YawLQRESOResolveBaseOmega(YawLQRESO_s *ctrl,
                                        const YawControlInput_t *input,
                                        float dt)
{
    float tau_s = g_yaw_base_omega_lpf_tau_s;
    float gain = g_yaw_base_omega_gain;
    float raw_omega;

    if (isfinite(tau_s) == 0 || tau_s < 0.0f) {
        tau_s = YAW_BASE_OMEGA_LPF_TAU_DEFAULT;
    }
    if (isfinite(gain) == 0) {
        gain = 1.0f;
    }
    g_yaw_base_omega_lpf_tau_s = tau_s;
    g_yaw_base_omega_gain = gain;

    if (ctrl == NULL || input == NULL ||
        g_yaw_base_comp_enable == 0U ||
        input->base_omega_valid == 0U ||
        isfinite(input->omega_base_rad_s) == 0) {
        if (ctrl != NULL) {
            ctrl->base_omega_filt_rad_s = 0.0f;
            ctrl->base_omega_filter_ready = 0U;
        }
        g_yaw_base_omega_rad_s = 0.0f;
        return 0.0f;
    }

    raw_omega = gain * input->omega_base_rad_s;
    if (isfinite(raw_omega) == 0) {
        raw_omega = 0.0f;
    }

    if (ctrl->base_omega_filter_ready == 0U || tau_s <= 0.0f) {
        ctrl->base_omega_filt_rad_s = raw_omega;
        ctrl->base_omega_filter_ready = 1U;
    } else {
        float alpha = dt / (tau_s + dt);
        alpha = LQRESOClamp(alpha, 0.0f, 1.0f);
        ctrl->base_omega_filt_rad_s +=
            alpha * (raw_omega - ctrl->base_omega_filt_rad_s);
    }

    g_yaw_base_omega_rad_s = ctrl->base_omega_filt_rad_s;
    return ctrl->base_omega_filt_rad_s;
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

static void YawEso_UpdateGains(YawLQRESO_s *ctrl,
                               float yaw_j,
                               float yaw_b,
                               float k_theta,
                               float k_omega,
                               uint8_t auto_aim_mode)
{
    float w0;
    float w0_set;
    float alpha;
    float b0;
    float k_theta_abs = LQRESOAbs(k_theta);
    float k_omega_abs = LQRESOAbs(k_omega);
    float w0_from_k;
    float w0_from_damping;

    if (ctrl == NULL) {
        return;
    }

    if (yaw_j <= 1.0e-6f) {
        yaw_j = YAW_J_DEFAULT;
    }
    b0 = 1.0f / yaw_j;
    if (k_theta_abs <= 1.0e-6f) {
        k_theta_abs = (auto_aim_mode != 0U) ?
            LQRESOAbs(LQR_AUTO_K_THETA_DEFAULT) : LQRESOAbs(LQR_K_THETA_DEFAULT);
    }
    if (k_omega_abs <= 1.0e-6f) {
        k_omega_abs = (auto_aim_mode != 0U) ?
            LQRESOAbs(LQR_AUTO_K_OMEGA_DEFAULT) : LQRESOAbs(LQR_K_OMEGA_DEFAULT);
    }

    alpha = (g_yaw_eso_alpha > 0.0f) ? g_yaw_eso_alpha : 0.0f;
    g_yaw_eso_alpha = alpha;

    w0_from_k = sqrtf(b0 * k_theta_abs);
    w0_from_damping = 0.5f * b0 * LQRESOAbs(yaw_b + k_omega_abs);
    if (w0_from_damping > w0_from_k) {
        w0_from_k = w0_from_damping;
    }

    w0_set = g_yaw_eso_w0_set;
    if (auto_aim_mode != 0U) {
        /*
         * 自瞄 ESO 带宽默认跟随当前自瞄 K 矩阵；
         * 若 g_yaw_auto_eso_w0_set > 0，则用作 debug 覆盖值。
         */
        if (g_yaw_auto_eso_w0_set > 0.0f) {
            w0 = g_yaw_auto_eso_w0_set;
        } else {
            w0 = alpha * w0_from_k;
        }
    } else {
        w0 = (w0_set > 0.0f) ? w0_set : (alpha * w0_from_k);
    }

    ctrl->eso.b0    = b0;
    ctrl->eso.w0    = w0;
    ctrl->eso.beta1 = 3.0f * w0;
    ctrl->eso.beta2 = 3.0f * w0 * w0;
    ctrl->eso.beta3 = w0 * w0 * w0;
}

static void YawEso_InitInternal(YawLQRESO_s *ctrl)
{
    YawEso_UpdateGains(ctrl, YAW_J_DEFAULT, YAW_B_DEFAULT, g_lqr_k_theta, g_lqr_k_omega, 0U);
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
    ctrl->base_omega_filt_rad_s  = 0.0f;
    ctrl->base_omega_filter_ready = 0U;

    YawPublishObserver(ctrl);
}

static void YawEso_Update(YawLQRESO_s *ctrl,
                          float theta_meas,
                          float u_last,
                          float dt,
                          float yaw_j,
                          float yaw_b,
                          float omega_base_rad_s)
{
    YawEso_t *eso;
    float e;
    float z1_dot;
    float z2_dot;
    float z3_dot;
    float damping;
    float omega_rel_eso;

    if (ctrl == NULL) {
        return;
    }

    if (yaw_j <= 1.0e-6f) {
        yaw_j = YAW_J_DEFAULT;
    }
    damping = yaw_b / yaw_j;

    eso    = &ctrl->eso;
    e      = theta_meas - eso->z1;
    omega_rel_eso = eso->z2 - omega_base_rad_s;
    z1_dot = eso->z2 + eso->beta1 * e;
    z2_dot = -damping * omega_rel_eso + eso->b0 * u_last + eso->z3 + eso->beta2 * e;
    z3_dot = eso->beta3 * e;
    g_yaw_eso_omega_rel_rad_s = omega_rel_eso;

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
    g_yaw_lqr_ident_record.theta_ref_cmd_deg = LQRESODebugFloatToHalf(input->theta_ref_deg);
    g_yaw_lqr_ident_record.theta_ref_rad = LQRESODebugFloatToHalf(input->theta_ref_rad);
    g_yaw_lqr_ident_record.omega_ref_rad_s = LQRESODebugFloatToHalf(input->omega_ref_rad_s);
    g_yaw_lqr_ident_record.alpha_ref_rad_s2 = LQRESODebugFloatToHalf(input->alpha_ref_rad_s2);
    g_yaw_lqr_ident_record.theta_meas_rad = LQRESODebugFloatToHalf(input->theta_rad);
    g_yaw_lqr_ident_record.omega_meas_rad_s = LQRESODebugFloatToHalf(input->omega_rad_s);
    g_yaw_lqr_ident_record.e_theta_rad = LQRESODebugFloatToHalf(0.0f);
    g_yaw_lqr_ident_record.e_omega_rad_s = LQRESODebugFloatToHalf(0.0f);
    g_yaw_lqr_ident_record.tau_lqr_nm = LQRESODebugFloatToHalf(0.0f);
    g_yaw_lqr_ident_record.tau_bias_nm = LQRESODebugFloatToHalf(0.0f);
    g_yaw_lqr_ident_record.tau_cmd_nm = LQRESODebugFloatToHalf(0.0f);
    g_yaw_lqr_ident_record.tau_meas_nm = LQRESODebugFloatToHalf(input->tau_meas_nm);
    g_yaw_lqr_ident_record.tau_meas_filt_nm = LQRESODebugFloatToHalf(0.0f);
    g_yaw_lqr_ident_record.tau_track_err_nm = LQRESODebugFloatToHalf(0.0f);
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
    float omega_raw;

    if (ctrl == NULL) {
        return;
    }

    g_yaw_auto_aim_debug_record.ref_omega_limit_active = 0U;
    g_yaw_auto_aim_debug_record.ref_alpha_limit_active = 0U;
    omega_limit = LQRESOAbs(g_ref_omega_limit_rad_s);
    theta_target_rad = LQRESOWrapNearestRad(theta_target_rad, ctrl->theta_ref_planned_rad);
    old_theta_ref = ctrl->theta_ref_planned_rad;
    err           = theta_target_rad - old_theta_ref;

    ctrl->theta_ref_planned_rad = theta_target_rad;
    if (LQRESOAbs(err) < 1e-6f) {
        ctrl->omega_ref_planned     = 0.0f;
        ctrl->alpha_ref_planned     = 0.0f;
    } else {
        omega_raw = err / dt;
        ctrl->omega_ref_planned = omega_raw;
        if (omega_limit > 0.0f) {
            ctrl->omega_ref_planned = LQRESOClamp(ctrl->omega_ref_planned, -omega_limit, omega_limit);
            if (ctrl->omega_ref_planned != omega_raw) {
                g_yaw_auto_aim_debug_record.ref_omega_limit_active = 1U;
            }
        }
        ctrl->alpha_ref_planned = 0.0f;
    }

}

static void RefPlanner_UpdateTrajectory(YawLQRESO_s *ctrl,
                                        float theta_ref_rad,
                                        float omega_ref_rad_s,
                                        float alpha_ref_rad_s2)
{
    if (ctrl == NULL) {
        return;
    }

    g_yaw_auto_aim_debug_record.ref_omega_limit_active = 0U;
    g_yaw_auto_aim_debug_record.ref_alpha_limit_active = 0U;
    ctrl->theta_ref_planned_rad = theta_ref_rad;
    ctrl->omega_ref_planned = omega_ref_rad_s;
    ctrl->alpha_ref_planned = alpha_ref_rad_s2;

}

static void RefPlanner_UpdateAutoAimTrajectory(YawLQRESO_s *ctrl,
                                               float theta_in_rad,
                                               float omega_in_rad_s,
                                               float alpha_in_rad_s2,
                                               float dt)
{
    float change_eps;
    float rc;
    float lookahead_s;
    float theta_out;
    float omega_out;
    float alpha_out;
    uint8_t changed = 0U;

    if (ctrl == NULL) {
        return;
    }

    g_yaw_auto_aim_debug_record.ref_omega_limit_active = 0U;
    g_yaw_auto_aim_debug_record.ref_alpha_limit_active = 0U;
    change_eps    = LQRESOAbs(g_traj_change_eps_rad);
    rc            = (g_traj_alpha_lpf_rc_s > 0.0f) ? g_traj_alpha_lpf_rc_s : 0.0f;
    lookahead_s   = (g_traj_lookahead_s > 0.0f) ? g_traj_lookahead_s : 0.0f;

    /*
     * 自瞄轨迹输入来自视觉侧，单位保持为 rad / rad/s / rad/s^2。
     * 首包直接锁存 theta、omega，并用输入 alpha 初始化滤波器，避免首帧被滞后拉歪。
     */
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
        changed                          = 1U;

        theta_out = theta_in_rad;
        omega_out = omega_in_rad_s;
        alpha_out = alpha_in_rad_s2;
    } else {
        /* 只有视觉输入变化时才认为来了新包；否则沿用锁存轨迹继续外推。 */
        changed = (LQRESOAbs(theta_in_rad - ctrl->traj_last_input_theta_rad) > change_eps) ||
                  (LQRESOAbs(omega_in_rad_s - ctrl->traj_last_input_omega_rad_s) > change_eps) ||
                  (LQRESOAbs(alpha_in_rad_s2 - ctrl->traj_last_input_alpha_rad_s2) > change_eps);
        ctrl->traj_last_input_theta_rad   = theta_in_rad;
        ctrl->traj_last_input_omega_rad_s = omega_in_rad_s;
        ctrl->traj_last_input_alpha_rad_s2 = alpha_in_rad_s2;

        if (changed != 0U) {
            /*
             * 新视觉包到达时，theta/omega 直接跟随；alpha 单独做一阶 LPF。
             * RC 由 g_traj_alpha_lpf_rc_s 调，主要用于压低 J*alpha 前馈尖峰。
             */
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
            /* 没有新包时累计锁存时间，后面按恒加速度模型在 STM32 侧外推。 */
            ctrl->traj_dt_since_latch_s += dt;
        }

        /* 根据锁存的 theta/omega 和滤波后的 alpha 生成当前规划输出。 */
        {
            const float tau = ctrl->traj_dt_since_latch_s;
            const float w_l = ctrl->traj_latched_omega_rad_s;
            const float a_f = ctrl->traj_alpha_filt_rad_s2;
            const float th_l = ctrl->traj_latched_theta_rad;

            theta_out = th_l + w_l * tau + 0.5f * a_f * tau * tau;
            omega_out = w_l + a_f * tau;
            alpha_out = a_f;
        }
    }

    /* lookahead 只把参考提前一点，不改变视觉协议里的原始字段含义。 */
    if (lookahead_s > 0.0f) {
        const float L = lookahead_s;
        theta_out += omega_out * L + 0.5f * alpha_out * L * L;
        omega_out += alpha_out * L;
    }

    ctrl->theta_ref_planned_rad = theta_out;
    ctrl->omega_ref_planned     = omega_out;
    ctrl->alpha_ref_planned     = alpha_out;

    g_yaw_auto_aim_debug_record.theta_ref_planned_rad = ctrl->theta_ref_planned_rad;
    g_yaw_auto_aim_debug_record.omega_ref_planned_rad_s = ctrl->omega_ref_planned;
    g_yaw_auto_aim_debug_record.alpha_ref_planned_rad_s2 = ctrl->alpha_ref_planned;
}

static uint8_t YawModelIdentNormalizeSelect(uint8_t stage)
{
    if (stage == YAW_MODEL_IDENT_STAGE_ALL) {
        return YAW_MODEL_IDENT_STAGE_ALL;
    }
    if (stage == YAW_MODEL_IDENT_STAGE_LOW_SPEED ||
        stage == YAW_MODEL_IDENT_STAGE_DYNAMIC_CHIRP) {
        return stage;
    }
    return YAW_MODEL_IDENT_STAGE_DEFAULT;
}

static uint8_t YawModelIdentNormalizeActiveStage(uint8_t stage)
{
    if (stage == YAW_MODEL_IDENT_STAGE_DYNAMIC_CHIRP) {
        return YAW_MODEL_IDENT_STAGE_DYNAMIC_CHIRP;
    }
    return YAW_MODEL_IDENT_STAGE_LOW_SPEED;
}

static uint8_t YawModelIdentFirstStage(uint8_t stage_select)
{
    stage_select = YawModelIdentNormalizeSelect(stage_select);
    if (stage_select == YAW_MODEL_IDENT_STAGE_DYNAMIC_CHIRP) {
        return YAW_MODEL_IDENT_STAGE_DYNAMIC_CHIRP;
    }
    return YAW_MODEL_IDENT_STAGE_LOW_SPEED;
}

static void YawModelIdentGetStageDefaults(uint8_t stage,
                                          float *duration_s,
                                          float *amp_nm,
                                          float *amp_limit_nm,
                                          float *f_start_hz,
                                          float *f_end_hz)
{
    stage = YawModelIdentNormalizeActiveStage(stage);
    if (stage == YAW_MODEL_IDENT_STAGE_DYNAMIC_CHIRP) {
        if (duration_s != NULL) {
            *duration_s = YAW_IDENT_STAGE_DYNAMIC_DURATION_S;
        }
        if (amp_nm != NULL) {
            *amp_nm = YAW_IDENT_STAGE_DYNAMIC_AMP_NM;
        }
        if (amp_limit_nm != NULL) {
            *amp_limit_nm = YAW_IDENT_STAGE_DYNAMIC_AMP_LIMIT_NM;
        }
        if (f_start_hz != NULL) {
            *f_start_hz = YAW_IDENT_STAGE_DYNAMIC_F0_HZ;
        }
        if (f_end_hz != NULL) {
            *f_end_hz = YAW_IDENT_STAGE_DYNAMIC_F1_HZ;
        }
        return;
    }

    if (duration_s != NULL) {
        *duration_s = YAW_IDENT_STAGE_LOW_DURATION_S;
    }
    if (amp_nm != NULL) {
        *amp_nm = YAW_IDENT_STAGE_LOW_AMP_NM;
    }
    if (amp_limit_nm != NULL) {
        *amp_limit_nm = YAW_IDENT_STAGE_LOW_AMP_LIMIT_NM;
    }
    if (f_start_hz != NULL) {
        *f_start_hz = YAW_IDENT_STAGE_LOW_F0_HZ;
    }
    if (f_end_hz != NULL) {
        *f_end_hz = YAW_IDENT_STAGE_LOW_F1_HZ;
    }
}

static void YawModelIdentLoadStageDefaults(uint8_t stage)
{
    float duration;
    float amp;
    float amp_limit;
    float f0;
    float f1;

    stage = YawModelIdentNormalizeActiveStage(stage);
    YawModelIdentGetStageDefaults(stage, &duration, &amp, &amp_limit, &f0, &f1);
    g_yaw_model_ident.stage_id = stage;
    g_yaw_model_ident.duration_s = duration;
    g_yaw_model_ident.amp_nm = amp;
    g_yaw_model_ident.amp_limit_nm = amp_limit;
    g_yaw_model_ident.f_start_hz = f0;
    g_yaw_model_ident.f_end_hz = f1;
}

static void YawModelIdentSanitizeConfig(void)
{
    uint8_t stage_select = YawModelIdentNormalizeSelect(g_yaw_model_ident.stage_select);
    uint8_t active_stage = YawModelIdentNormalizeActiveStage(yaw_model_ident_stage_latched);
    float default_duration;
    float default_amp;
    float default_amp_limit;
    float default_f0;
    float default_f1;
    float duration = g_yaw_model_ident.duration_s;
    float amp_limit = LQRESOAbs(g_yaw_model_ident.amp_limit_nm);
    float amp = LQRESOAbs(g_yaw_model_ident.amp_nm);
    float f0 = g_yaw_model_ident.f_start_hz;
    float f1 = g_yaw_model_ident.f_end_hz;
    float k_theta = g_yaw_model_ident.k_theta_ident;
    float k_omega = g_yaw_model_ident.k_omega_ident;

    if (g_yaw_model_ident.running == 0U) {
        yaw_model_ident_run_latched = stage_select;
        active_stage = YawModelIdentFirstStage(stage_select);
    }
    yaw_model_ident_run_latched =
        YawModelIdentNormalizeSelect(yaw_model_ident_run_latched);
    yaw_model_ident_stage_latched = active_stage;
    YawModelIdentGetStageDefaults(active_stage,
                                  &default_duration,
                                  &default_amp,
                                  &default_amp_limit,
                                  &default_f0,
                                  &default_f1);

    if (duration <= 0.01f) {
        duration = default_duration;
    }
    if (amp_limit <= 0.0f ||
        amp_limit > YAW_IDENT_EXCITE_AMP_HARD_LIMIT_NM) {
        amp_limit = default_amp_limit;
    }
    if (amp <= 0.0f) {
        amp = default_amp;
    }
    if (amp > amp_limit) {
        amp = amp_limit;
    }
    if (f0 <= 0.01f) {
        f0 = default_f0;
    }
    if (f1 < f0) {
        f1 = (default_f1 >= f0) ? default_f1 : f0;
    }
    if (k_theta <= 0.0f) {
        k_theta = YAW_MODEL_IDENT_K_THETA_DEFAULT;
    }
    if (k_omega < 0.0f) {
        k_omega = YAW_MODEL_IDENT_K_OMEGA_DEFAULT;
    }

    g_yaw_model_ident.stage_select = stage_select;
    g_yaw_model_ident.stage_id = active_stage;
    g_yaw_model_ident.duration_s = duration;
    g_yaw_model_ident.amp_limit_nm = amp_limit;
    g_yaw_model_ident.amp_nm = amp;
    g_yaw_model_ident.f_start_hz = f0;
    g_yaw_model_ident.f_end_hz = f1;
    g_yaw_model_ident.k_theta_ident = k_theta;
    g_yaw_model_ident.k_omega_ident = k_omega;

    /* Keep the legacy scalar watches meaningful, but the struct is the owner. */
    g_yaw_ident_excite_enable = g_yaw_model_ident.running;
    g_yaw_ident_excite_type = 0U;
    g_yaw_ident_excite_amp_nm = amp;
    g_yaw_ident_excite_amp_limit_nm = amp_limit;
    g_yaw_ident_excite_f0_hz = f0;
    g_yaw_ident_excite_f1_hz = f1;
    g_yaw_ident_excite_duration_s = duration;
}

static void YawModelIdentResetStageCycle(YawLQRESO_s *ctrl)
{
    g_yaw_model_ident.valid = 0U;
    g_yaw_model_ident.sample_active = 0U;
    g_yaw_model_ident.sample_count = 0U;
    g_yaw_model_ident.time_s = 0.0f;
    g_yaw_model_ident.ident_freq_hz = 0.0f;
    g_yaw_model_ident.ident_phase_rad = 0.0f;
    g_yaw_model_ident.tau_excitation_nm = 0.0f;
    if (ctrl != NULL) {
        ctrl->ident_excite_phase_rad = 0.0f;
        ctrl->ident_excite_cycle_t_s = 0.0f;
    }
}

static uint8_t YawModelIdentCanAdvanceStage(void)
{
    return (yaw_model_ident_run_latched == YAW_MODEL_IDENT_STAGE_ALL &&
            yaw_model_ident_stage_latched == YAW_MODEL_IDENT_STAGE_LOW_SPEED) ? 1U : 0U;
}

static void YawModelIdentAdvanceStage(YawLQRESO_s *ctrl)
{
    yaw_model_ident_stage_latched = YAW_MODEL_IDENT_STAGE_DYNAMIC_CHIRP;
    YawModelIdentLoadStageDefaults(yaw_model_ident_stage_latched);
    YawModelIdentResetStageCycle(ctrl);
    yaw_model_ident_stage_switch_pending = 1U;
    YawModelIdentSanitizeConfig();
}

static void YawModelIdentStop(YawLQRESO_s *ctrl, uint8_t abort_reason, uint8_t done)
{
    g_yaw_model_ident.running = 0U;
    g_yaw_model_ident.valid = 0U;
    g_yaw_model_ident.sample_active = 0U;
    g_yaw_model_ident.done = done;
    g_yaw_model_ident.stop = 0U;
    g_yaw_model_ident.abort_reason = abort_reason;
    g_yaw_model_ident.tau_excitation_nm = 0.0f;
    g_yaw_model_ident.ident_freq_hz = 0.0f;
    g_yaw_model_ident.ident_phase_rad = 0.0f;
    g_yaw_ident_excite_enable = 0U;
    yaw_model_ident_stage_switch_pending = 0U;
    if (ctrl != NULL) {
        ctrl->ident_excite_phase_rad = 0.0f;
        ctrl->ident_excite_cycle_t_s = 0.0f;
    }
}

static uint8_t YawModelIdentReady(const YawControlInput_t *input)
{
    if (input == NULL) {
        return 0U;
    }
    return (input->hole_offset_mode == 0U &&
            input->feedback_ok != 0U) ? 1U : 0U;
}

static uint8_t YawModelIdentPrepare(YawLQRESO_s *ctrl,
                                    const YawControlInput_t *input,
                                    float dt)
{
    const uint8_t ready = YawModelIdentReady(input);

    YawModelIdentSanitizeConfig();
    g_yaw_model_ident.dt_s = dt;
    g_yaw_model_ident.feedback_ok = (input != NULL) ? input->feedback_ok : 0U;
    g_yaw_model_ident.auto_aim_mode = (input != NULL) ? input->auto_aim_mode : 0U;
    g_yaw_model_ident.ref_traj_valid = (input != NULL) ? input->ref_traj_valid : 0U;
    g_yaw_model_ident.valid = 0U;
    g_yaw_model_ident.sample_active = 0U;

    if (g_yaw_model_ident.stop != 0U) {
        g_yaw_model_ident.arm = 0U;
        YawModelIdentStop(ctrl, YAW_MODEL_IDENT_ABORT_STOP, 1U);
        return 0U;
    }

    if (g_yaw_model_ident.running != 0U) {
        if (input == NULL || input->feedback_ok == 0U) {
            YawModelIdentStop(ctrl, YAW_MODEL_IDENT_ABORT_FEEDBACK_LOST, 1U);
            return 0U;
        }
        if (ready == 0U) {
            YawModelIdentStop(ctrl, YAW_MODEL_IDENT_ABORT_EXIT_AUTO, 1U);
            return 0U;
        }
        if (g_yaw_model_ident.time_s >= g_yaw_model_ident.duration_s) {
            if (YawModelIdentCanAdvanceStage() != 0U) {
                YawModelIdentAdvanceStage(ctrl);
                return 1U;
            }
            YawModelIdentStop(ctrl, YAW_MODEL_IDENT_ABORT_NONE, 1U);
            return 0U;
        }
    } else if (g_yaw_model_ident.arm != 0U) {
        if (ready != 0U) {
            const uint8_t run_select =
                YawModelIdentNormalizeSelect(g_yaw_model_ident.stage_select);
            const uint8_t stage = YawModelIdentFirstStage(run_select);

            g_yaw_model_ident.stage_select = run_select;
            yaw_model_ident_run_latched = run_select;
            yaw_model_ident_stage_latched = stage;
            YawModelIdentLoadStageDefaults(stage);
            g_yaw_model_ident.arm = 0U;
            g_yaw_model_ident.stop = 0U;
            g_yaw_model_ident.running = 1U;
            g_yaw_model_ident.done = 0U;
            g_yaw_model_ident.valid = 0U;
            g_yaw_model_ident.sample_active = 0U;
            g_yaw_model_ident.abort_reason = YAW_MODEL_IDENT_ABORT_NONE;
            yaw_model_ident_stage_switch_pending = 0U;
            YawModelIdentResetStageCycle(ctrl);
        } else {
            g_yaw_model_ident.abort_reason =
                ((input != NULL && input->feedback_ok == 0U) ?
                 YAW_MODEL_IDENT_ABORT_FEEDBACK_LOST :
                 YAW_MODEL_IDENT_ABORT_WAIT_READY);
        }
    }

    YawModelIdentSanitizeConfig();
    return g_yaw_model_ident.running;
}

static void YawModelIdentApplyKOverride(float *k_theta, float *k_omega)
{
    if (k_theta == NULL || k_omega == NULL) {
        return;
    }
    if (g_yaw_model_ident.running == 0U ||
        g_yaw_model_ident.k_override_enable == 0U) {
        return;
    }

    *k_theta = g_yaw_model_ident.k_theta_ident;
    *k_omega = g_yaw_model_ident.k_omega_ident;
}

/*
 * Yaw identification excitation injector.
 *
 * Adds a small torque chirp on top of the auto-aim LQR command so that offline
 * J/B/Fc fits get enough persistent excitation. The single Ozone entrypoint is
 * g_yaw_model_ident.arm; the old scalar watches are kept synchronized only for
 * compatibility.
 */
static float YawExcitationCalc(YawLQRESO_s *ctrl,
                               const YawControlInput_t *input,
                               float dt,
                               uint8_t *active_out,
                               uint8_t *type_used_out,
                               float *freq_hz_out,
                               float *phase_out)
{
    float amp;
    float amp_limit;
    float f0;
    float f1;
    float duration;
    float freq_hz;
    float signal;

    if (active_out != NULL)    { *active_out = 0U; }
    if (type_used_out != NULL) { *type_used_out = 0U; }
    if (freq_hz_out != NULL)   { *freq_hz_out = 0.0f; }
    if (phase_out != NULL)     { *phase_out = 0.0f; }

    if (ctrl == NULL || input == NULL) {
        return 0.0f;
    }

    if (g_yaw_model_ident.running == 0U ||
        input->hole_offset_mode != 0U ||
        input->feedback_ok == 0U ||
        ctrl->feedback_ready == 0U) {
        ctrl->ident_excite_phase_rad = 0.0f;
        ctrl->ident_excite_cycle_t_s = 0.0f;
        return 0.0f;
    }

    if (yaw_model_ident_stage_switch_pending != 0U) {
        yaw_model_ident_stage_switch_pending = 0U;
        g_yaw_model_ident.tau_excitation_nm = 0.0f;
        g_yaw_model_ident.ident_freq_hz = 0.0f;
        g_yaw_model_ident.ident_phase_rad = 0.0f;
        return 0.0f;
    }

    YawModelIdentSanitizeConfig();
    amp_limit = LQRESOAbs(g_yaw_model_ident.amp_limit_nm);
    if (amp_limit > YAW_IDENT_EXCITE_AMP_HARD_LIMIT_NM) {
        amp_limit = YAW_IDENT_EXCITE_AMP_HARD_LIMIT_NM;
    }
    g_yaw_model_ident.amp_limit_nm = amp_limit;
    amp = LQRESOAbs(g_yaw_model_ident.amp_nm);
    if (amp > amp_limit) {
        amp = amp_limit;
    }
    if (amp <= 0.0f) {
        ctrl->ident_excite_phase_rad = 0.0f;
        ctrl->ident_excite_cycle_t_s = 0.0f;
        return 0.0f;
    }

    f0 = (g_yaw_model_ident.f_start_hz > 0.01f) ?
        g_yaw_model_ident.f_start_hz : 0.01f;
    f1 = (g_yaw_model_ident.f_end_hz > f0) ?
        g_yaw_model_ident.f_end_hz : f0;
    duration = (g_yaw_model_ident.duration_s > 0.01f) ?
        g_yaw_model_ident.duration_s : 1.0f;

    {
        float t_norm = ctrl->ident_excite_cycle_t_s / duration;
        if (t_norm < 0.0f) {
            t_norm = 0.0f;
        }
        if (t_norm > 1.0f) {
            t_norm = 1.0f;
        }
        freq_hz = f0 * expf(t_norm * logf(f1 / f0));
    }

    ctrl->ident_excite_phase_rad += LQR_ESO_TWO_PI * freq_hz * dt;
    if (ctrl->ident_excite_phase_rad > 1.0e6f ||
        ctrl->ident_excite_phase_rad < -1.0e6f) {
        ctrl->ident_excite_phase_rad =
            fmodf(ctrl->ident_excite_phase_rad, LQR_ESO_TWO_PI);
    }
    ctrl->ident_excite_cycle_t_s += dt;

    signal = amp * sinf(ctrl->ident_excite_phase_rad);
    g_yaw_model_ident.time_s = ctrl->ident_excite_cycle_t_s;
    g_yaw_model_ident.ident_freq_hz = freq_hz;
    g_yaw_model_ident.ident_phase_rad = ctrl->ident_excite_phase_rad;

    if (active_out != NULL)    { *active_out = 1U; }
    if (type_used_out != NULL) { *type_used_out = 0U; }
    if (freq_hz_out != NULL)   { *freq_hz_out = freq_hz; }
    if (phase_out != NULL)     { *phase_out = ctrl->ident_excite_phase_rad; }
    return signal;
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
    ctrl->base_omega_filt_rad_s  = 0.0f;
    ctrl->base_omega_filter_ready = 0U;
    g_lqr_e_theta                = 0.0f;
    g_lqr_omega_filt             = 0.0f;
    g_lqr_tau_raw                = 0.0f;
    g_tau_track_err              = 0.0f;
    g_theta_ref_planned          = 0.0f;
    g_omega_ref_planned          = 0.0f;
    g_alpha_ref_planned          = 0.0f;
    g_yaw_eso_tau_comp           = 0.0f;
    g_yaw_auto_eso_comp_active   = 0.0f;
    g_yaw_base_omega_rad_s       = 0.0f;
    g_yaw_omega_rel_ref_rad_s    = 0.0f;
    g_yaw_omega_rel_meas_rad_s   = 0.0f;
    g_yaw_eso_omega_rel_rad_s    = 0.0f;
    if (g_yaw_model_ident.running != 0U) {
        YawModelIdentStop(ctrl, YAW_MODEL_IDENT_ABORT_EXIT_AUTO, 1U);
    } else {
        g_yaw_model_ident.valid = 0U;
        g_yaw_model_ident.sample_active = 0U;
        g_yaw_model_ident.tau_excitation_nm = 0.0f;
        g_yaw_model_ident.ident_freq_hz = 0.0f;
        g_yaw_model_ident.ident_phase_rad = 0.0f;
    }
    g_yaw_auto_aim_debug_record.valid = 0U;
    g_yaw_auto_aim_debug_record.feedback_ok = 0U;
    g_yaw_auto_aim_debug_record.gimbal_mode = 0U;
    g_yaw_auto_aim_debug_record.nuc_mode = 0U;
    g_yaw_auto_aim_debug_record.auto_aim_mode = 0U;
    g_yaw_auto_aim_debug_record.limit_active = 0U;
    g_yaw_auto_aim_debug_record.soft_limit_active = 0U;
    g_yaw_auto_aim_debug_record.dm_limit_active = 0U;
    g_yaw_auto_aim_debug_record.slew_limit_active = 0U;
    g_yaw_auto_aim_debug_record.ref_omega_limit_active = 0U;
    g_yaw_auto_aim_debug_record.ref_alpha_limit_active = 0U;
    g_yaw_auto_aim_debug_record.ff_alpha_limit_active = 0U;
    g_yaw_auto_aim_debug_record.eso_comp_enable = 0U;
    g_yaw_auto_aim_debug_record.eso_comp_active = 0U;
    g_yaw_auto_aim_debug_record.theta_ref_cmd_deg = 0.0f;
    g_yaw_auto_aim_debug_record.input_theta_ref_rad = 0.0f;
    g_yaw_auto_aim_debug_record.input_omega_ref_rad_s = 0.0f;
    g_yaw_auto_aim_debug_record.input_alpha_ref_rad_s2 = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_j_used = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_b_used = 0.0f;
    g_yaw_auto_aim_debug_record.k_theta_used = 0.0f;
    g_yaw_auto_aim_debug_record.k_omega_used = 0.0f;
    g_yaw_auto_aim_debug_record.theta_ref_planned_rad = 0.0f;
    g_yaw_auto_aim_debug_record.omega_ref_planned_rad_s = 0.0f;
    g_yaw_auto_aim_debug_record.alpha_ref_planned_rad_s2 = 0.0f;
    g_yaw_auto_aim_debug_record.theta_meas_rad = 0.0f;
    g_yaw_auto_aim_debug_record.omega_meas_rad_s = 0.0f;
    g_yaw_auto_aim_debug_record.motor_pos_rad = 0.0f;
    g_yaw_auto_aim_debug_record.motor_total_pos_rad = 0.0f;
    g_yaw_auto_aim_debug_record.motor_vel_rad_s = 0.0f;
    g_yaw_auto_aim_debug_record.motor_tor_nm = 0.0f;
    g_yaw_auto_aim_debug_record.ctrl_tor_set_nm = 0.0f;
    g_yaw_auto_aim_debug_record.e_theta_raw_rad = 0.0f;
    g_yaw_auto_aim_debug_record.e_theta_rad = 0.0f;
    g_yaw_auto_aim_debug_record.e_omega_rad_s = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_alpha_raw_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_alpha_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_viscous_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_coulomb_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_theta_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_omega_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_lqi_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_lqr_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_for_eso_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_eso_raw_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_eso_limited_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_eso_comp_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_bias_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_pre_limit_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_cmd_before_slew_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_cmd_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_meas_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_meas_filt_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_track_err_nm = 0.0f;
    g_yaw_auto_aim_debug_record.eso_comp_gain_used = 0.0f;
    g_yaw_auto_aim_debug_record.eso_comp_limit_nm = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_b0 = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_w0 = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_z1 = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_z2 = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_z3 = 0.0f;
    g_yaw_auto_aim_debug_record.ident_excite_active = 0U;
    g_yaw_auto_aim_debug_record.ident_excite_type_used = 0U;
    g_yaw_auto_aim_debug_record.tau_excitation_nm = 0.0f;
    g_yaw_auto_aim_debug_record.ident_excite_freq_hz = 0.0f;
    g_yaw_auto_aim_debug_record.ident_excite_phase = 0.0f;
    ctrl->traj_latched_theta_rad      = 0.0f;
    ctrl->traj_latched_omega_rad_s    = 0.0f;
    ctrl->traj_latched_alpha_rad_s2   = 0.0f;
    ctrl->traj_alpha_filt_rad_s2      = 0.0f;
    ctrl->traj_dt_since_latch_s       = 0.0f;
    ctrl->traj_last_input_theta_rad   = 0.0f;
    ctrl->traj_last_input_omega_rad_s = 0.0f;
    ctrl->traj_last_input_alpha_rad_s2 = 0.0f;
    ctrl->traj_initialized            = 0U;
    ctrl->ident_excite_phase_rad      = 0.0f;
    ctrl->ident_excite_cycle_t_s      = 0.0f;
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
    float tau_ff_alpha_raw;
    float tau_ff_alpha;
    float tau_ff_viscous;
    float tau_ff_coulomb;
    float tau_theta;
    float tau_omega;
    float tau_lqi;
    float tau_lqr;
    float tau_comp;
    float tau_eso_raw;
    float tau_eso_active;
    float eso_comp_gain_max;
    float eso_comp_limit_max;
    float tau_cmd;
    float tau_pre_limit;
    float tau_cmd_before_slew;
    float soft_limit;
    float torque_slew_rate;
    float old_tau_cmd;
    float k_theta;
    float k_omega;
    float alpha;
    float blend;
    float yaw_j_sel;
    float yaw_b_sel;
    float omega_base;
    float omega_rel_ref;
    float omega_rel_meas;
    uint8_t limited = 0U;
    uint8_t soft_limit_active = 0U;
    uint8_t dm_limit_active = 0U;
    uint8_t slew_limit_active = 0U;
    uint8_t eso_comp_active = 0U;
    uint8_t ff_alpha_limit_active = 0U;
    uint8_t excite_active = 0U;
    uint8_t excite_type_used = 0U;
    uint8_t model_ident_active = 0U;
    float excite_freq_hz = 0.0f;
    float excite_phase = 0.0f;
    float tau_excitation = 0.0f;

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
    omega_base = YawLQRESOResolveBaseOmega(ctrl, input, dt);
    omega_rel_meas = input->omega_rad_s - omega_base;
    g_yaw_omega_rel_meas_rad_s = omega_rel_meas;
    YawSelectPlant(input->auto_aim_mode, &yaw_j_sel, &yaw_b_sel);
    if (input->hole_offset_mode != 0U) {
        k_theta = g_lqr_hole_offset_k_theta;
        k_omega = g_lqr_hole_offset_k_omega;
    } else {
        k_theta = (input->auto_aim_mode != 0U) ? g_lqr_auto_k_theta : g_lqr_k_theta;
        k_omega = (input->auto_aim_mode != 0U) ? g_lqr_auto_k_omega : g_lqr_k_omega;
    }
    model_ident_active = YawModelIdentPrepare(ctrl, input, dt);
    YawModelIdentApplyKOverride(&k_theta, &k_omega);
    YawLqrIdentRecordPublishInput(input, dt);
    g_yaw_auto_aim_debug_record.dt_s = dt;
    g_yaw_auto_aim_debug_record.valid = 0U;
    g_yaw_auto_aim_debug_record.feedback_ok = input->feedback_ok;
    g_yaw_auto_aim_debug_record.auto_aim_mode = input->auto_aim_mode;
    g_yaw_auto_aim_debug_record.theta_ref_cmd_deg = input->theta_ref_deg;
    g_yaw_auto_aim_debug_record.input_theta_ref_rad = input->theta_ref_rad;
    g_yaw_auto_aim_debug_record.input_omega_ref_rad_s = input->omega_ref_rad_s;
    g_yaw_auto_aim_debug_record.input_alpha_ref_rad_s2 = input->alpha_ref_rad_s2;
    g_yaw_auto_aim_debug_record.yaw_j_used = yaw_j_sel;
    g_yaw_auto_aim_debug_record.yaw_b_used = yaw_b_sel;
    g_yaw_auto_aim_debug_record.k_theta_used = k_theta;
    g_yaw_auto_aim_debug_record.k_omega_used = k_omega;
    g_yaw_auto_aim_debug_record.theta_meas_rad = input->theta_rad;
    g_yaw_auto_aim_debug_record.omega_meas_rad_s = input->omega_rad_s;
    g_yaw_auto_aim_debug_record.tau_meas_nm = input->tau_meas_nm;
    g_yaw_auto_aim_debug_record.limit_active = 0U;
    g_yaw_auto_aim_debug_record.soft_limit_active = 0U;
    g_yaw_auto_aim_debug_record.dm_limit_active = 0U;
    g_yaw_auto_aim_debug_record.slew_limit_active = 0U;
    g_yaw_auto_aim_debug_record.ref_omega_limit_active = 0U;
    g_yaw_auto_aim_debug_record.ref_alpha_limit_active = 0U;
    g_yaw_auto_aim_debug_record.ff_alpha_limit_active = 0U;
    g_yaw_auto_aim_debug_record.eso_comp_enable =
        (input->hole_offset_mode != 0U) ? 1U :
        ((input->auto_aim_mode != 0U && g_yaw_auto_eso_comp_enable != 0U) ? 1U : 0U);
    g_yaw_auto_aim_debug_record.eso_comp_active = 0U;
    g_yaw_auto_aim_debug_record.e_theta_raw_rad = 0.0f;
    g_yaw_auto_aim_debug_record.e_theta_rad = 0.0f;
    g_yaw_auto_aim_debug_record.e_omega_rad_s = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_alpha_raw_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_alpha_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_viscous_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_coulomb_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_ff_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_theta_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_omega_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_lqi_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_lqr_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_for_eso_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_eso_raw_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_eso_limited_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_eso_comp_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_bias_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_pre_limit_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_cmd_before_slew_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_cmd_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_meas_filt_nm = 0.0f;
    g_yaw_auto_aim_debug_record.tau_track_err_nm = 0.0f;
    g_yaw_auto_aim_debug_record.eso_comp_gain_used = 0.0f;
    g_yaw_auto_aim_debug_record.eso_comp_limit_nm = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_b0 = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_w0 = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_z1 = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_z2 = 0.0f;
    g_yaw_auto_aim_debug_record.yaw_eso_z3 = 0.0f;
    g_yaw_auto_aim_debug_record.ident_excite_active = 0U;
    g_yaw_auto_aim_debug_record.ident_excite_type_used = 0U;
    g_yaw_auto_aim_debug_record.tau_excitation_nm = 0.0f;
    g_yaw_auto_aim_debug_record.ident_excite_freq_hz = 0.0f;
    g_yaw_auto_aim_debug_record.ident_excite_phase = 0.0f;

    if (!input->feedback_ok) {
        YawLQRESO_Reset(ctrl);
        return;
    }

    YawEso_UpdateGains(ctrl, yaw_j_sel, yaw_b_sel, k_theta, k_omega, input->auto_aim_mode);

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
        g_yaw_lqr_ident_record.theta_ref_rad = LQRESODebugFloatToHalf(input->theta_rad);
        g_yaw_lqr_ident_record.omega_ref_rad_s = LQRESODebugFloatToHalf(0.0f);
        g_yaw_lqr_ident_record.alpha_ref_rad_s2 = LQRESODebugFloatToHalf(0.0f);
        g_yaw_lqr_ident_record.tau_meas_filt_nm = LQRESODebugFloatToHalf(ctrl->tau_meas_filt);
        g_yaw_auto_aim_debug_record.theta_ref_planned_rad = input->theta_rad;
        g_yaw_auto_aim_debug_record.omega_ref_planned_rad_s = 0.0f;
        g_yaw_auto_aim_debug_record.alpha_ref_planned_rad_s2 = 0.0f;
        g_yaw_auto_aim_debug_record.tau_meas_filt_nm = ctrl->tau_meas_filt;
        g_yaw_omega_rel_ref_rad_s = -omega_base;
        g_yaw_eso_omega_rel_rad_s = input->omega_rad_s - omega_base;
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
        if (g_traj_planner_enable != 0U) {
            RefPlanner_UpdateAutoAimTrajectory(ctrl,
                                               theta_target_rad,
                                               input->omega_ref_rad_s,
                                               input->alpha_ref_rad_s2,
                                               dt);
        } else {
            if (ctrl->traj_initialized != 0U) {
                ctrl->traj_initialized = 0U;
            }
            RefPlanner_UpdateTrajectory(ctrl,
                                        theta_target_rad,
                                        input->omega_ref_rad_s,
                                        input->alpha_ref_rad_s2);
        }
    } else {
        if (ctrl->traj_initialized != 0U) {
            ctrl->traj_initialized = 0U;
        }
        RefPlanner_Update(ctrl, theta_target_rad, dt);
    }
    theta_ref_rad = ctrl->theta_ref_planned_rad;
    omega_ref     = ctrl->omega_ref_planned;
    alpha_ref     = ctrl->alpha_ref_planned;
    omega_rel_ref = omega_ref - omega_base;
    g_yaw_omega_rel_ref_rad_s = omega_rel_ref;
    g_yaw_omega_rel_meas_rad_s = omega_rel_meas;
    g_yaw_auto_aim_debug_record.theta_ref_planned_rad = theta_ref_rad;
    g_yaw_auto_aim_debug_record.omega_ref_planned_rad_s = omega_ref;
    g_yaw_auto_aim_debug_record.alpha_ref_planned_rad_s2 = alpha_ref;

    blend       = LQRESOClamp(g_eso_torque_feedback_blend, 0.0f, 1.0f);
    tau_for_eso = (1.0f - blend) * ctrl->eso_u_last + blend * ctrl->tau_meas_filt;
    g_yaw_auto_aim_debug_record.tau_for_eso_nm = tau_for_eso;
    YawEso_Update(ctrl, input->theta_rad, tau_for_eso, dt, yaw_j_sel, yaw_b_sel, omega_base);

    e_theta_raw = input->theta_rad - theta_ref_rad;
    g_yaw_auto_aim_debug_record.e_theta_raw_rad = e_theta_raw;
    e_theta     = LQRESOApplyDeadband(e_theta_raw, g_ctrl_angle_deadband_rad);
    e_omega     = input->omega_rad_s - omega_ref;
    /*
     * 前馈拆成三项：J*alpha 负责目标加速段预补偿，B*omega 负责粘性阻尼，
     * coulomb 项用于静摩擦/库仑摩擦补偿。自瞄路径不再对 J*alpha 单项限幅。
     */
    {
        const uint8_t ff_auto = (input->auto_aim_mode != 0U) ? 1U : 0U;
        const float coulomb_smooth_raw =
            (ff_auto != 0U) ? g_yaw_auto_coulomb_smooth_omega
                            : g_yaw_coulomb_smooth_omega;
        const float coulomb_smooth =
            (coulomb_smooth_raw > 1e-6f) ? coulomb_smooth_raw
                                         : YAW_COULOMB_SMOOTH_OMEGA_DEFAULT;
        const float coulomb_nm =
            (ff_auto != 0U) ? g_yaw_auto_tau_coulomb_nm : g_yaw_tau_coulomb_nm;

        /* yaw_j_sel/yaw_b_sel 在本周期只选择一次，FF 和 ESO 共用同一套 plant。 */
        tau_ff_alpha_raw = yaw_j_sel * alpha_ref;
        tau_ff_alpha = tau_ff_alpha_raw;
        tau_ff_viscous = yaw_b_sel * omega_rel_ref;
        tau_ff_coulomb = coulomb_nm * tanhf(omega_rel_ref / coulomb_smooth);

        tau_ff = tau_ff_alpha + tau_ff_viscous + tau_ff_coulomb;
    }
    tau_theta   = -k_theta * e_theta;
    tau_omega   = -k_omega * e_omega;
    tau_lqi     = -g_lqi_ki * ctrl->lqi_int;
    tau_lqr     = tau_ff + tau_theta + tau_omega + tau_lqi;

    tau_comp = 0.0f;
    tau_eso_raw = 0.0f;
    eso_comp_gain_max = LQRESOAbs(g_eso_comp_gain_max);
    eso_comp_limit_max = LQRESOAbs(g_eso_comp_limit_max);
    g_eso_comp_gain_max = eso_comp_gain_max;
    g_eso_comp_limit_max = eso_comp_limit_max;
    if (LQRESOAbs(ctrl->eso.b0) > 1e-6f) {
        const float eso_comp_gain = LQRESOClamp(g_eso_comp_gain, 0.0f, eso_comp_gain_max);
        g_eso_comp_gain = eso_comp_gain;
        g_yaw_auto_aim_debug_record.eso_comp_gain_used = eso_comp_gain;
        tau_eso_raw = -eso_comp_gain * ctrl->eso.z3 / ctrl->eso.b0;
        tau_comp = tau_eso_raw;
    }
    if (input->auto_aim_mode == 0U) {
        g_eso_comp_limit = LQRESOClamp(LQRESOAbs(g_eso_comp_limit), 0.0f, eso_comp_limit_max);
        tau_comp = LQRESOClampAbs(tau_comp, g_eso_comp_limit);
        g_yaw_auto_aim_debug_record.eso_comp_limit_nm = g_eso_comp_limit;
    } else {
        g_yaw_auto_aim_debug_record.eso_comp_limit_nm = 0.0f;
    }
    g_yaw_auto_aim_debug_record.tau_eso_raw_nm = tau_eso_raw;
    g_yaw_auto_aim_debug_record.tau_eso_limited_nm = tau_comp;

    /*
     * 辨识时只允许 LQR + chirp 力矩进入输出，ESO 继续观测但不补偿。
     * 普通非自瞄仍只保留 ESO 观测与 debug；
     * 狗洞 yaw offset 模式强制接入补偿，自瞄仍由原开关控制。
     */
    if (model_ident_active != 0U) {
        g_yaw_auto_aim_debug_record.eso_comp_enable = 0U;
        eso_comp_active = 0U;
    } else if (input->hole_offset_mode != 0U) {
        eso_comp_active = 1U;
    } else if (input->auto_aim_mode == 0U) {
        eso_comp_active = 0U;
    } else if (g_yaw_auto_eso_comp_enable != 0U) {
        eso_comp_active = 1U;
    }
    tau_eso_active = (eso_comp_active != 0U) ? tau_comp : 0.0f;
    tau_excitation = YawExcitationCalc(ctrl,
                                       input,
                                       dt,
                                       &excite_active,
                                       &excite_type_used,
                                       &excite_freq_hz,
                                       &excite_phase);
    tau_cmd        = tau_lqr + tau_eso_active + tau_excitation;
    g_yaw_auto_eso_comp_active = (float)eso_comp_active;
    g_yaw_auto_aim_debug_record.eso_comp_active = eso_comp_active;
    g_yaw_auto_aim_debug_record.tau_eso_comp_nm = tau_eso_active;
    soft_limit = LQRESOAbs(g_ctrl_t_limit);
    if (model_ident_active == 0U &&
        input->auto_aim_mode == 0U &&
        soft_limit > 0.0f) {
        const float limited_tau = LQRESOClampAbs(tau_cmd, soft_limit);
        if (limited_tau != tau_cmd) {
            soft_limit_active = 1U;
            limited = 1U;
        }
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

    /* bias 加入后、第二次 soft limit 前的中间量；字段名保留兼容 Ozone/CSV。 */
    tau_pre_limit = tau_lqr + tau_eso_active + ctrl->tau_bias;
    tau_cmd += ctrl->tau_bias;
    if (model_ident_active == 0U &&
        input->auto_aim_mode == 0U &&
        soft_limit > 0.0f) {
        const float limited_tau = LQRESOClampAbs(tau_cmd, soft_limit);
        if (limited_tau != tau_cmd) {
            soft_limit_active = 1U;
            limited = 1U;
        }
        tau_cmd = limited_tau;
    }

    if (tau_cmd < DM_T_MIN) {
        tau_cmd = DM_T_MIN;
        dm_limit_active = 1U;
        limited = 1U;
    } else if (tau_cmd > DM_T_MAX) {
        tau_cmd = DM_T_MAX;
        dm_limit_active = 1U;
        limited = 1U;
    }

    tau_cmd_before_slew = tau_cmd;
    torque_slew_rate = LQRESOAbs(input->torque_slew_rate_nm_s);
    if (model_ident_active == 0U &&
        input->auto_aim_mode == 0U &&
        input->torque_slew_enable != 0U &&
        torque_slew_rate > 0.0f) {
        const float max_delta = torque_slew_rate * dt;
        old_tau_cmd = tau_cmd;
        tau_cmd = LQRESOClamp(tau_cmd, ctrl->torque_cmd_last - max_delta, ctrl->torque_cmd_last + max_delta);
        if (tau_cmd != old_tau_cmd) {
            slew_limit_active = 1U;
            limited = 1U;
        }
    }

    if (model_ident_active == 0U &&
        input->auto_aim_mode == 0U &&
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
    g_yaw_eso_tau_comp  = tau_eso_active;
    YawPublishObserver(ctrl);

    g_yaw_lqr_ident_record.valid = 1U;
    g_yaw_lqr_ident_record.feedback_ok = 1U;
    g_yaw_lqr_ident_record.theta_ref_rad = LQRESODebugFloatToHalf(theta_ref_rad);
    g_yaw_lqr_ident_record.omega_ref_rad_s = LQRESODebugFloatToHalf(omega_ref);
    g_yaw_lqr_ident_record.alpha_ref_rad_s2 = LQRESODebugFloatToHalf(alpha_ref);
    g_yaw_lqr_ident_record.e_theta_rad = LQRESODebugFloatToHalf(e_theta);
    g_yaw_lqr_ident_record.e_omega_rad_s = LQRESODebugFloatToHalf(e_omega);
    g_yaw_lqr_ident_record.tau_lqr_nm = LQRESODebugFloatToHalf(tau_lqr);
    g_yaw_lqr_ident_record.tau_bias_nm = LQRESODebugFloatToHalf(ctrl->tau_bias);
    g_yaw_lqr_ident_record.tau_cmd_nm = LQRESODebugFloatToHalf(tau_cmd);
    g_yaw_lqr_ident_record.tau_meas_filt_nm = LQRESODebugFloatToHalf(ctrl->tau_meas_filt);
    g_yaw_lqr_ident_record.tau_track_err_nm = LQRESODebugFloatToHalf(tau_cmd - ctrl->tau_meas_filt);

    g_yaw_auto_aim_debug_record.valid = 1U;
    g_yaw_auto_aim_debug_record.feedback_ok = 1U;
    g_yaw_auto_aim_debug_record.auto_aim_mode = input->auto_aim_mode;
    g_yaw_auto_aim_debug_record.limit_active = limited;
    g_yaw_auto_aim_debug_record.soft_limit_active = soft_limit_active;
    g_yaw_auto_aim_debug_record.dm_limit_active = dm_limit_active;
    g_yaw_auto_aim_debug_record.slew_limit_active = slew_limit_active;
    g_yaw_auto_aim_debug_record.ff_alpha_limit_active = ff_alpha_limit_active;
    g_yaw_auto_aim_debug_record.theta_ref_planned_rad = theta_ref_rad;
    g_yaw_auto_aim_debug_record.omega_ref_planned_rad_s = omega_ref;
    g_yaw_auto_aim_debug_record.alpha_ref_planned_rad_s2 = alpha_ref;
    g_yaw_auto_aim_debug_record.e_theta_raw_rad = e_theta_raw;
    g_yaw_auto_aim_debug_record.e_theta_rad = e_theta;
    g_yaw_auto_aim_debug_record.e_omega_rad_s = e_omega;
    g_yaw_auto_aim_debug_record.tau_ff_alpha_raw_nm = tau_ff_alpha_raw;
    g_yaw_auto_aim_debug_record.tau_ff_alpha_nm = tau_ff_alpha;
    g_yaw_auto_aim_debug_record.tau_ff_viscous_nm = tau_ff_viscous;
    g_yaw_auto_aim_debug_record.tau_ff_coulomb_nm = tau_ff_coulomb;
    g_yaw_auto_aim_debug_record.tau_ff_nm = tau_ff;
    g_yaw_auto_aim_debug_record.tau_theta_nm = tau_theta;
    g_yaw_auto_aim_debug_record.tau_omega_nm = tau_omega;
    g_yaw_auto_aim_debug_record.tau_lqi_nm = tau_lqi;
    g_yaw_auto_aim_debug_record.tau_lqr_nm = tau_lqr;
    g_yaw_auto_aim_debug_record.tau_for_eso_nm = tau_for_eso;
    g_yaw_auto_aim_debug_record.tau_eso_raw_nm = tau_eso_raw;
    g_yaw_auto_aim_debug_record.tau_eso_limited_nm = tau_comp;
    g_yaw_auto_aim_debug_record.tau_eso_comp_nm = tau_eso_active;
    g_yaw_auto_aim_debug_record.tau_bias_nm = ctrl->tau_bias;
    g_yaw_auto_aim_debug_record.tau_pre_limit_nm = tau_pre_limit;
    g_yaw_auto_aim_debug_record.tau_cmd_before_slew_nm =
        tau_cmd_before_slew;
    g_yaw_auto_aim_debug_record.tau_cmd_nm = tau_cmd;
    g_yaw_auto_aim_debug_record.tau_meas_filt_nm = ctrl->tau_meas_filt;
    g_yaw_auto_aim_debug_record.tau_track_err_nm = tau_cmd - ctrl->tau_meas_filt;
    g_yaw_auto_aim_debug_record.eso_comp_gain_used = g_eso_comp_gain;
    g_yaw_auto_aim_debug_record.eso_comp_limit_nm = g_eso_comp_limit;
    g_yaw_auto_aim_debug_record.yaw_eso_b0 = ctrl->eso.b0;
    g_yaw_auto_aim_debug_record.yaw_eso_w0 = ctrl->eso.w0;
    g_yaw_auto_aim_debug_record.yaw_eso_z1 = ctrl->eso.z1;
    g_yaw_auto_aim_debug_record.yaw_eso_z2 = ctrl->eso.z2;
    g_yaw_auto_aim_debug_record.yaw_eso_z3 = ctrl->eso.z3;
    g_yaw_auto_aim_debug_record.ident_excite_active = excite_active;
    g_yaw_auto_aim_debug_record.ident_excite_type_used = excite_type_used;
    g_yaw_auto_aim_debug_record.tau_excitation_nm = tau_excitation;
    g_yaw_auto_aim_debug_record.ident_excite_freq_hz = excite_freq_hz;
    g_yaw_auto_aim_debug_record.ident_excite_phase = excite_phase;
    g_yaw_lqreso_control_seq++;

    g_yaw_model_ident.valid =
        (model_ident_active != 0U && excite_active != 0U) ? 1U : 0U;
    g_yaw_model_ident.sample_active = g_yaw_model_ident.valid;
    if (g_yaw_model_ident.valid != 0U) {
        g_yaw_model_ident.sample_count++;
    }
    g_yaw_model_ident.dt_s = dt;
    g_yaw_model_ident.feedback_ok = input->feedback_ok;
    g_yaw_model_ident.auto_aim_mode = input->auto_aim_mode;
    g_yaw_model_ident.ref_traj_valid = input->ref_traj_valid;
    g_yaw_model_ident.limit_active = limited;
    g_yaw_model_ident.soft_limit_active = soft_limit_active;
    g_yaw_model_ident.dm_limit_active = dm_limit_active;
    g_yaw_model_ident.slew_limit_active = slew_limit_active;
    g_yaw_model_ident.low_speed_flag =
        (LQRESOAbs(input->omega_rad_s) < YAW_MODEL_IDENT_LOW_SPEED_GATE_RAD_S) ? 1U : 0U;
    g_yaw_model_ident.reverse_flag =
        ((input->omega_rad_s * omega_ref) < -1.0e-4f) ? 1U : 0U;
    g_yaw_model_ident.torque_saturated_flag =
        (limited != 0U || soft_limit_active != 0U ||
         dm_limit_active != 0U || slew_limit_active != 0U) ? 1U : 0U;
    g_yaw_model_ident.theta_ref_cmd_deg = input->theta_ref_deg;
    g_yaw_model_ident.input_theta_ref_rad = input->theta_ref_rad;
    g_yaw_model_ident.input_omega_ref_rad_s = input->omega_ref_rad_s;
    g_yaw_model_ident.input_alpha_ref_rad_s2 = input->alpha_ref_rad_s2;
    g_yaw_model_ident.theta_ref_planned_rad = theta_ref_rad;
    g_yaw_model_ident.omega_ref_planned_rad_s = omega_ref;
    g_yaw_model_ident.alpha_ref_planned_rad_s2 = alpha_ref;
    g_yaw_model_ident.theta_meas_rad = input->theta_rad;
    g_yaw_model_ident.omega_meas_rad_s = input->omega_rad_s;
    g_yaw_model_ident.omega_abs_rad_s = LQRESOAbs(input->omega_rad_s);
    g_yaw_model_ident.yaw_j_used = yaw_j_sel;
    g_yaw_model_ident.yaw_b_used = yaw_b_sel;
    g_yaw_model_ident.yaw_coulomb_nm =
        (input->auto_aim_mode != 0U) ?
        g_yaw_auto_tau_coulomb_nm : g_yaw_tau_coulomb_nm;
    g_yaw_model_ident.yaw_coulomb_smooth_omega =
        (input->auto_aim_mode != 0U) ?
        g_yaw_auto_coulomb_smooth_omega : g_yaw_coulomb_smooth_omega;
    g_yaw_model_ident.k_theta_used = k_theta;
    g_yaw_model_ident.k_omega_used = k_omega;
    g_yaw_model_ident.e_theta_raw_rad = e_theta_raw;
    g_yaw_model_ident.e_theta_rad = e_theta;
    g_yaw_model_ident.e_omega_rad_s = e_omega;
    g_yaw_model_ident.tau_ff_alpha_raw_nm = tau_ff_alpha_raw;
    g_yaw_model_ident.tau_ff_alpha_nm = tau_ff_alpha;
    g_yaw_model_ident.tau_ff_viscous_nm = tau_ff_viscous;
    g_yaw_model_ident.tau_ff_coulomb_nm = tau_ff_coulomb;
    g_yaw_model_ident.tau_ff_nm = tau_ff;
    g_yaw_model_ident.tau_theta_nm = tau_theta;
    g_yaw_model_ident.tau_omega_nm = tau_omega;
    g_yaw_model_ident.tau_lqi_nm = tau_lqi;
    g_yaw_model_ident.tau_lqr_nm = tau_lqr;
    g_yaw_model_ident.tau_bias_nm = ctrl->tau_bias;
    g_yaw_model_ident.tau_pre_limit_nm = tau_pre_limit;
    g_yaw_model_ident.tau_excitation_nm = tau_excitation;
    g_yaw_model_ident.tau_cmd_before_slew_nm = tau_cmd_before_slew;
    g_yaw_model_ident.tau_cmd_nm = tau_cmd;
    g_yaw_model_ident.tau_meas_nm = input->tau_meas_nm;
    g_yaw_model_ident.tau_meas_filt_nm = ctrl->tau_meas_filt;
    g_yaw_model_ident.tau_track_err_nm = tau_cmd - ctrl->tau_meas_filt;

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

    if (ctrl->eso_config.w0_set_enable != 0U) {
        w0 = (ctrl->eso_config.w0_set > 0.0f) ?
            ctrl->eso_config.w0_set : 0.0f;
    } else {
        wn_term = k_theta / j;
        wn = (wn_term > 0.0f) ? sqrtf(wn_term) : 0.0f;
        sigma = (b + k_omega) / (2.0f * j);
        sigma = (sigma > 0.0f) ? sigma : 0.0f;
        w_cl = (wn > sigma) ? wn : sigma;
        w0 = alpha * w_cl;
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

float PitchLQRESO_CalcTrajectory(PitchLQRESO_s *ctrl,
                                  float ref,
                                  float ref_vel,
                                  float ref_acc,
                                  float measure,
                                  float speed,
                                  float feedforward,
                                  uint8_t eso_runtime_enable,
                                  float dt)
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
    float model_feedforward;
    uint8_t eso_active;
    uint8_t limited = 0U;

    if (ctrl == NULL) {
        return 0.0f;
    }

    dt = LQRESOSanitizeDt(dt);
    PitchResolveLQRGains(ctrl, &k_theta, &k_omega);
    ctrl->k_theta_used = k_theta;
    ctrl->k_omega_used = k_omega;
    eso_active = (eso_runtime_enable != 0U && ctrl->eso_config.enable != 0U) ? 1U : 0U;
    if (eso_active != 0U) {
        PitchEso_UpdateGains(ctrl, k_theta, k_omega);
    } else {
        ctrl->eso.b0 = 0.0f;
        ctrl->eso.w0 = 0.0f;
        ctrl->eso.beta1 = 0.0f;
        ctrl->eso.beta2 = 0.0f;
        ctrl->eso.beta3 = 0.0f;
    }

    torque_limit = PITCH_CTRL_T_LIMIT_DEFAULT;
    if (ctrl->lqr_config.output_limit > 0.0f) {
        torque_limit = ctrl->lqr_config.output_limit;
    }
    torque_limit = LQRESOClamp(torque_limit, 0.0f, DM_T_MAX);
    torque_slew_rate = PITCH_CTRL_TORQUE_SLEW_RATE_DEFAULT;
    if (ctrl->lqr_config.output_slew_rate < 0.0f) {
        torque_slew_rate = 0.0f;
    } else if (ctrl->lqr_config.output_slew_rate > 0.0f) {
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

    if (eso_active != 0U) {
        PitchEso_Update(ctrl, measure, ctrl->eso.u_last, dt);
    } else {
        ctrl->eso.z1 = measure;
        ctrl->eso.z2 = speed;
        ctrl->eso.z3 = 0.0f;
        ctrl->eso.u_last = 0.0f;
    }

    e_theta = measure - ref;
    e_omega = speed - ref_vel;
    tau_lqr = -k_theta * e_theta - k_omega * e_omega;
    model_feedforward = 0.0f;
    if (ctrl->lqr_config.disable_model_feedforward == 0U &&
        ctrl->lqr_config.plant_j_kg_m2 > PITCH_MODEL_MIN_J) {
        model_feedforward =
            ctrl->lqr_config.plant_j_kg_m2 * ref_acc +
            ctrl->lqr_config.plant_b_nms_rad * ref_vel;
    }
    tau_eso_comp = (eso_active != 0U) ? PitchEso_CalcComp(ctrl) : 0.0f;
    tau_cmd = tau_lqr + feedforward + model_feedforward + tau_eso_comp;
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
    ctrl->eso.u_last = (eso_active != 0U) ? tau_cmd : 0.0f;
    ctrl->output = tau_cmd;
    ctrl->torque = tau_cmd;
    PitchPublishObserver(ctrl, tau_eso_comp);
    return tau_cmd;
}

float PitchLQRESO_Calc(PitchLQRESO_s *ctrl, float ref, float measure, float speed, float feedforward, float dt)
{
    return PitchLQRESO_CalcTrajectory(ctrl,
                                      ref,
                                      0.0f,
                                      0.0f,
                                      measure,
                                      speed,
                                      feedforward,
                                      0U,
                                      dt);
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
