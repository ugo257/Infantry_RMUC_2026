#include "double_pitch_lqr.h"

#include "arm_math.h"
#include "math.h"
#include "string.h"

#define DOUBLE_PITCH_PI            3.14159265358979323846f
#define DOUBLE_PITCH_TWO_PI        6.28318530717958647692f
#define DOUBLE_PITCH_MIN_DT_S      0.0001f
#define DOUBLE_PITCH_MAX_DT_S      0.0200f
#define DOUBLE_PITCH_DEFAULT_DT_S  0.0010f
#define DOUBLE_PITCH_DM_T_LIMIT_NM 7.0f

#define DOUBLE_PITCH_MODEL_A0      0.0314f
#define DOUBLE_PITCH_MODEL_AC      0.0000f
#define DOUBLE_PITCH_MODEL_AS      0.0000f
#define DOUBLE_PITCH_MODEL_B0     -0.0057f
#define DOUBLE_PITCH_MODEL_BC      0.0000f
#define DOUBLE_PITCH_MODEL_BS      0.0000f
#define DOUBLE_PITCH_MODEL_C0      0.011526f
#define DOUBLE_PITCH_MODEL_GB1     1.0666f
#define DOUBLE_PITCH_MODEL_GB2     0.0000f
#define DOUBLE_PITCH_MODEL_GP1     0.373492f
#define DOUBLE_PITCH_MODEL_GP2    -0.929636f
#define DOUBLE_PITCH_MODEL_FVB     0.0000f
#define DOUBLE_PITCH_MODEL_FCB     0.0000f
#define DOUBLE_PITCH_MODEL_FVS     0.063380f
#define DOUBLE_PITCH_MODEL_FCS     0.333765f
#define DOUBLE_PITCH_MODEL_OMEGA0  0.0500f

typedef struct
{
    float k[2][4];
    float k_alpha;
    float theta_big_ddot_radps2;
    float phi_rel_ddot_radps2;
    float sin_phi;
    float cos_phi;
    float sin_theta;
    float cos_theta;
    float sin_theta_plus_phi;
    float cos_theta_plus_phi;
    float m11;
    float m12;
    float m22;
    float m11_dot_phi;
    float m12_dot_phi;
    float c_big;
    float c_small;
    float g_big;
    float g_small;
    float f_big;
    float f_small;
    uint8_t k_segment;
} DoublePitchModelCalc_s;

volatile DoublePitchLqrDebug_s g_double_pitch_lqr_debug = {0};
volatile DoublePitchModelDebug_s g_double_pitch_model_debug = {0};

volatile float g_double_pitch_k_live[2][4] = {
    {343.2f,  -149.0f,  12.0f,  -10.0f},
    {-638.6f, 1777.2f, -22.7f, 103.1f},
};

static float DoublePitchClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float DoublePitchAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float DoublePitchClampAbs(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    return DoublePitchClamp(value, -limit, limit);
}

static DoublePitchDebugHalf_t DoublePitchDebugFloatToHalf(float value)
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
        return (DoublePitchDebugHalf_t)(sign | ((mantissa != 0U) ? 0x7e00U : 0x7c00U));
    }

    exponent = exponent - 127 + 15;
    if (exponent >= 31) {
        return (DoublePitchDebugHalf_t)(sign | 0x7c00U);
    }
    if (exponent <= 0) {
        if (exponent < -10) {
            return (DoublePitchDebugHalf_t)sign;
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
            return (DoublePitchDebugHalf_t)(sign | 0x0400U);
        }
        return (DoublePitchDebugHalf_t)(sign | half_mantissa);
    }

    half_mantissa = mantissa >> 13U;
    if ((mantissa & 0x00001000U) != 0U) {
        half_mantissa++;
        if (half_mantissa >= 0x0400U) {
            half_mantissa = 0U;
            exponent++;
            if (exponent >= 31) {
                return (DoublePitchDebugHalf_t)(sign | 0x7c00U);
            }
        }
    }
    return (DoublePitchDebugHalf_t)(sign | ((uint32_t)exponent << 10U) | half_mantissa);
}

static float DoublePitchSmoothSign(float omega_rad_s)
{
    if (DOUBLE_PITCH_MODEL_OMEGA0 <= 1.0e-6f) {
        return (omega_rad_s > 0.0f) ? 1.0f :
               ((omega_rad_s < 0.0f) ? -1.0f : 0.0f);
    }
    return tanhf(omega_rad_s / DOUBLE_PITCH_MODEL_OMEGA0);
}

static float DoublePitchWrapRad(float angle_rad)
{
    while (angle_rad > DOUBLE_PITCH_PI) {
        angle_rad -= DOUBLE_PITCH_TWO_PI;
    }
    while (angle_rad < -DOUBLE_PITCH_PI) {
        angle_rad += DOUBLE_PITCH_TWO_PI;
    }
    return angle_rad;
}

static float DoublePitchSanitizeDt(float dt_s)
{
    if (dt_s < DOUBLE_PITCH_MIN_DT_S || dt_s > DOUBLE_PITCH_MAX_DT_S) {
        return DOUBLE_PITCH_DEFAULT_DT_S;
    }
    return dt_s;
}

static void DoublePitchClearModelDebug(void)
{
    g_double_pitch_model_debug = (DoublePitchModelDebug_s){0};
}

static void DoublePitchClearOutput(DoublePitchLqrOutput_s *output)
{
    if (output != NULL) {
        memset(output, 0, sizeof(*output));
    }
}

static void DoublePitchClearDebug(void)
{
    g_double_pitch_lqr_debug = (DoublePitchLqrDebug_s){0};
    DoublePitchClearModelDebug();
}

static uint8_t DoublePitchSelectK(float phi_rad, float k_out[2][4], float *alpha_out)
{
    uint8_t row;
    uint8_t col;

    (void)phi_rad;
    if (alpha_out != NULL) {
        *alpha_out = 0.0f;
    }

    for (row = 0U; row < 2U; row++) {
        for (col = 0U; col < 4U; col++) {
            k_out[row][col] = g_double_pitch_k_live[row][col];
        }
    }
    return 0U;
}

static void DoublePitchPublishDebug(const DoublePitchLqr_s *ctrl,
                                    const DoublePitchLqrInput_s *input,
                                    const DoublePitchLqrOutput_s *output,
                                    float dt_s)
{
    if (input == NULL || output == NULL) {
        DoublePitchClearDebug();
        return;
    }

    g_double_pitch_lqr_debug.dt_s = dt_s;
    g_double_pitch_lqr_debug.feedback_ok = input->feedback_ok;
    g_double_pitch_lqr_debug.ready = (ctrl == NULL) ? 0U : ctrl->ready;
    g_double_pitch_lqr_debug.k_segment = output->k_segment;
    g_double_pitch_lqr_debug.tau_big_limit_flag = output->tau_big_limit_flag;
    g_double_pitch_lqr_debug.tau_small_limit_flag = output->tau_small_limit_flag;
    g_double_pitch_lqr_debug.theta_rad = DoublePitchDebugFloatToHalf(input->theta_rad);
    g_double_pitch_lqr_debug.phi_rad = DoublePitchDebugFloatToHalf(input->phi_rad);
    g_double_pitch_lqr_debug.psi_rad = DoublePitchDebugFloatToHalf(input->psi_rad);
    g_double_pitch_lqr_debug.theta_ref_rad = DoublePitchDebugFloatToHalf(input->theta_ref_rad);
    g_double_pitch_lqr_debug.phi_ref_rad = DoublePitchDebugFloatToHalf(input->phi_ref_rad);
    g_double_pitch_lqr_debug.psi_ref_rad =
        DoublePitchDebugFloatToHalf(DoublePitchWrapRad(input->theta_ref_rad + input->phi_ref_rad));
    g_double_pitch_lqr_debug.omega_big_rad_s = DoublePitchDebugFloatToHalf(input->omega_big_rad_s);
    g_double_pitch_lqr_debug.omega_small_rad_s = DoublePitchDebugFloatToHalf(input->omega_small_rad_s);
    g_double_pitch_lqr_debug.e_theta_rad = DoublePitchDebugFloatToHalf(output->e_theta_rad);
    g_double_pitch_lqr_debug.e_phi_rad = DoublePitchDebugFloatToHalf(output->e_phi_rad);
    g_double_pitch_lqr_debug.e_omega_big_rad_s = DoublePitchDebugFloatToHalf(output->e_omega_big_rad_s);
    g_double_pitch_lqr_debug.e_omega_small_rad_s = DoublePitchDebugFloatToHalf(output->e_omega_small_rad_s);
    g_double_pitch_lqr_debug.v_big_cmd_rad_s2 = DoublePitchDebugFloatToHalf(output->v_big_cmd_rad_s2);
    g_double_pitch_lqr_debug.v_small_cmd_rad_s2 = DoublePitchDebugFloatToHalf(output->v_small_cmd_rad_s2);
    g_double_pitch_lqr_debug.tau_big_fb_nm = DoublePitchDebugFloatToHalf(output->tau_big_fb_nm);
    g_double_pitch_lqr_debug.tau_small_fb_nm = DoublePitchDebugFloatToHalf(output->tau_small_fb_nm);
    g_double_pitch_lqr_debug.tau_big_g_nm = DoublePitchDebugFloatToHalf(output->tau_big_g_nm);
    g_double_pitch_lqr_debug.tau_small_g_nm = DoublePitchDebugFloatToHalf(output->tau_small_g_nm);
    g_double_pitch_lqr_debug.tau_big_pre_limit_nm = DoublePitchDebugFloatToHalf(output->tau_big_pre_limit_nm);
    g_double_pitch_lqr_debug.tau_small_pre_limit_nm = DoublePitchDebugFloatToHalf(output->tau_small_pre_limit_nm);
    g_double_pitch_lqr_debug.tau_big_cmd_nm = DoublePitchDebugFloatToHalf(output->tau_big_cmd_nm);
    g_double_pitch_lqr_debug.tau_small_cmd_nm = DoublePitchDebugFloatToHalf(output->tau_small_cmd_nm);
}

static void DoublePitchPublishModelDebug(const DoublePitchLqr_s *ctrl,
                                         const DoublePitchLqrInput_s *input,
                                         const DoublePitchLqrOutput_s *output,
                                         const DoublePitchModelCalc_s *model,
                                         float dt_s)
{
    DoublePitchModelDebug_s debug = {0};
    uint8_t row;
    uint8_t col;

    if (ctrl == NULL || input == NULL || output == NULL) {
        DoublePitchClearModelDebug();
        return;
    }

    debug.time_s = ctrl->time_s;
    debug.sample_time_s = dt_s;
    debug.target_id = input->target_id;
    debug.control_mode = input->control_mode;
    debug.feedback_ok = input->feedback_ok;
    debug.ready = ctrl->ready;
    debug.accel_ready = ctrl->accel_ready;
    debug.tau_big_limit_flag = output->tau_big_limit_flag;
    debug.tau_small_limit_flag = output->tau_small_limit_flag;
    debug.c_used = 0U;
    debug.f_used = 0U;
    debug.bus_voltage_valid = input->bus_voltage_valid;
    debug.k_segment = output->k_segment;
    debug.theta_big_rad = DoublePitchDebugFloatToHalf(input->theta_rad);
    debug.phi_rel_rad = DoublePitchDebugFloatToHalf(input->phi_rad);
    debug.psi_small_world_rad = DoublePitchDebugFloatToHalf(input->psi_rad);
    debug.theta_big_dot_radps = DoublePitchDebugFloatToHalf(input->omega_big_rad_s);
    debug.phi_rel_dot_radps = DoublePitchDebugFloatToHalf(input->omega_small_rad_s);
    debug.theta_big_ref_rad = DoublePitchDebugFloatToHalf(input->theta_ref_rad);
    debug.phi_rel_ref_rad = DoublePitchDebugFloatToHalf(input->phi_ref_rad);
    debug.theta_big_ref_dot_radps = DoublePitchDebugFloatToHalf(input->theta_ref_dot_rad_s);
    debug.phi_rel_ref_dot_radps = DoublePitchDebugFloatToHalf(input->phi_ref_dot_rad_s);
    debug.theta_big_ref_ddot_radps2 = DoublePitchDebugFloatToHalf(input->theta_ref_ddot_rad_s2);
    debug.phi_rel_ref_ddot_radps2 = DoublePitchDebugFloatToHalf(input->phi_ref_ddot_rad_s2);
    debug.tau_big_cmd_nm = DoublePitchDebugFloatToHalf(output->tau_big_cmd_nm);
    debug.tau_small_cmd_nm = DoublePitchDebugFloatToHalf(output->tau_small_cmd_nm);
    debug.tau_big_meas_nm = DoublePitchDebugFloatToHalf(input->tau_big_meas_nm);
    debug.tau_small_meas_nm = DoublePitchDebugFloatToHalf(input->tau_small_meas_nm);
    debug.v_big_cmd_radps2 = DoublePitchDebugFloatToHalf(output->v_big_cmd_rad_s2);
    debug.v_small_cmd_radps2 = DoublePitchDebugFloatToHalf(output->v_small_cmd_rad_s2);
    debug.tau_big_ff_nm = DoublePitchDebugFloatToHalf(output->tau_big_g_nm);
    debug.tau_small_ff_nm = DoublePitchDebugFloatToHalf(output->tau_small_g_nm);
    debug.tau_big_fb_nm = DoublePitchDebugFloatToHalf(output->tau_big_fb_nm);
    debug.tau_small_fb_nm = DoublePitchDebugFloatToHalf(output->tau_small_fb_nm);
    debug.tau_big_pre_limit_nm = DoublePitchDebugFloatToHalf(output->tau_big_pre_limit_nm);
    debug.tau_small_pre_limit_nm = DoublePitchDebugFloatToHalf(output->tau_small_pre_limit_nm);
    debug.bus_voltage_v = DoublePitchDebugFloatToHalf(input->bus_voltage_v);
    debug.motor_big_temp_c = DoublePitchDebugFloatToHalf(input->motor_big_temp_c);
    debug.motor_small_temp_c = DoublePitchDebugFloatToHalf(input->motor_small_temp_c);
    debug.motor_big_tmos_c = DoublePitchDebugFloatToHalf(input->motor_big_tmos_c);
    debug.motor_small_tmos_c = DoublePitchDebugFloatToHalf(input->motor_small_tmos_c);
    debug.a0 = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_A0);
    debug.ac = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_AC);
    debug.as = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_AS);
    debug.b0 = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_B0);
    debug.bc = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_BC);
    debug.bs = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_BS);
    debug.c0 = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_C0);
    debug.gb1 = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_GB1);
    debug.gb2 = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_GB2);
    debug.gp1 = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_GP1);
    debug.gp2 = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_GP2);
    debug.fvb = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_FVB);
    debug.fcb = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_FCB);
    debug.fvs = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_FVS);
    debug.fcs = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_FCS);
    debug.omega0 = DoublePitchDebugFloatToHalf(DOUBLE_PITCH_MODEL_OMEGA0);

    if (model != NULL) {
        debug.k_alpha = DoublePitchDebugFloatToHalf(model->k_alpha);
        debug.k_segment = model->k_segment;
        debug.theta_big_ddot_radps2 = DoublePitchDebugFloatToHalf(model->theta_big_ddot_radps2);
        debug.phi_rel_ddot_radps2 = DoublePitchDebugFloatToHalf(model->phi_rel_ddot_radps2);
        debug.sin_phi = DoublePitchDebugFloatToHalf(model->sin_phi);
        debug.cos_phi = DoublePitchDebugFloatToHalf(model->cos_phi);
        debug.sin_theta = DoublePitchDebugFloatToHalf(model->sin_theta);
        debug.cos_theta = DoublePitchDebugFloatToHalf(model->cos_theta);
        debug.sin_theta_plus_phi = DoublePitchDebugFloatToHalf(model->sin_theta_plus_phi);
        debug.cos_theta_plus_phi = DoublePitchDebugFloatToHalf(model->cos_theta_plus_phi);
        debug.m11 = DoublePitchDebugFloatToHalf(model->m11);
        debug.m12 = DoublePitchDebugFloatToHalf(model->m12);
        debug.m22 = DoublePitchDebugFloatToHalf(model->m22);
        debug.m11_dot_phi = DoublePitchDebugFloatToHalf(model->m11_dot_phi);
        debug.m12_dot_phi = DoublePitchDebugFloatToHalf(model->m12_dot_phi);
        debug.c_big = DoublePitchDebugFloatToHalf(model->c_big);
        debug.c_small = DoublePitchDebugFloatToHalf(model->c_small);
        debug.g_big = DoublePitchDebugFloatToHalf(model->g_big);
        debug.g_small = DoublePitchDebugFloatToHalf(model->g_small);
        debug.f_big = DoublePitchDebugFloatToHalf(model->f_big);
        debug.f_small = DoublePitchDebugFloatToHalf(model->f_small);
        for (row = 0U; row < 2U; row++) {
            for (col = 0U; col < 4U; col++) {
                debug.k[row][col] = DoublePitchDebugFloatToHalf(model->k[row][col]);
            }
        }
    }

    g_double_pitch_model_debug = debug;
}

void DoublePitchLqr_Init(DoublePitchLqr_s *ctrl)
{
    if (ctrl == NULL) {
        return;
    }
    memset(ctrl, 0, sizeof(*ctrl));
    DoublePitchClearDebug();
}

void DoublePitchLqr_Reset(DoublePitchLqr_s *ctrl)
{
    if (ctrl == NULL) {
        return;
    }
    ctrl->ready = 0U;
    ctrl->accel_ready = 0U;
    ctrl->tau_big_last_nm = 0.0f;
    ctrl->tau_small_last_nm = 0.0f;
    ctrl->time_s = 0.0f;
    ctrl->omega_big_last_rad_s = 0.0f;
    ctrl->omega_small_last_rad_s = 0.0f;
    DoublePitchClearDebug();
}

void DoublePitchLqr_Calc(DoublePitchLqr_s *ctrl,
                         const DoublePitchLqrInput_s *input,
                         float dt_s,
                         DoublePitchLqrOutput_s *output)
{
    float k[2][4];
    DoublePitchModelCalc_s model = {0};
    float e_theta;
    float e_phi;
    float e_omega_big;
    float e_omega_small;
    float v_big;
    float v_small;
    float sin_phi;
    float cos_phi;
    float sin_theta;
    float cos_theta;
    float theta_plus_phi;
    float sin_theta_plus_phi;
    float cos_theta_plus_phi;
    float m11;
    float m12;
    float m22;
    float m11_dot_phi;
    float m12_dot_phi;
    float c_big;
    float c_small;
    float f_big;
    float f_small;
    float tau_big_fb;
    float tau_small_fb;
    float tau_big_g;
    float tau_small_g;
    float tau_big;
    float tau_small;
    float old_tau;
    float torque_limit;
    float slew_rate;
    float k_alpha = 0.0f;
    float theta_big_ddot_radps2 = 0.0f;
    float phi_rel_ddot_radps2 = 0.0f;
    uint8_t k_segment;
    uint8_t row;
    uint8_t col;

    DoublePitchClearOutput(output);
    if (ctrl == NULL || input == NULL || output == NULL) {
        DoublePitchClearDebug();
        return;
    }

    dt_s = DoublePitchSanitizeDt(dt_s);
    if (input->feedback_ok == 0U) {
        DoublePitchLqr_Reset(ctrl);
        return;
    }

    ctrl->time_s += dt_s;
    if (ctrl->accel_ready != 0U) {
        theta_big_ddot_radps2 =
            (input->omega_big_rad_s - ctrl->omega_big_last_rad_s) / dt_s;
        phi_rel_ddot_radps2 =
            (input->omega_small_rad_s - ctrl->omega_small_last_rad_s) / dt_s;
    }
    ctrl->omega_big_last_rad_s = input->omega_big_rad_s;
    ctrl->omega_small_last_rad_s = input->omega_small_rad_s;
    ctrl->accel_ready = 1U;

    if (ctrl->ready == 0U) {
        ctrl->ready = 1U;
        ctrl->tau_big_last_nm = 0.0f;
        ctrl->tau_small_last_nm = 0.0f;
        model.theta_big_ddot_radps2 = theta_big_ddot_radps2;
        model.phi_rel_ddot_radps2 = phi_rel_ddot_radps2;
        DoublePitchPublishDebug(ctrl, input, output, dt_s);
        DoublePitchPublishModelDebug(ctrl, input, output, &model, dt_s);
        return;
    }

    e_theta = DoublePitchWrapRad(input->theta_rad - input->theta_ref_rad);
    e_phi = DoublePitchWrapRad(input->phi_rad - input->phi_ref_rad);
    e_omega_big = input->omega_big_rad_s - input->theta_ref_dot_rad_s;
    e_omega_small = input->omega_small_rad_s - input->phi_ref_dot_rad_s;

    k_segment = DoublePitchSelectK(input->phi_rad, k, &k_alpha);
    v_big = input->theta_ref_ddot_rad_s2 -
            (k[0][0] * e_theta + k[0][1] * e_phi +
             k[0][2] * e_omega_big + k[0][3] * e_omega_small);
    v_small = input->phi_ref_ddot_rad_s2 -
              (k[1][0] * e_theta + k[1][1] * e_phi +
               k[1][2] * e_omega_big + k[1][3] * e_omega_small);

    sin_phi = arm_sin_f32(input->phi_rad);
    cos_phi = arm_cos_f32(input->phi_rad);
    sin_theta = arm_sin_f32(input->theta_rad);
    cos_theta = arm_cos_f32(input->theta_rad);
    theta_plus_phi = input->theta_rad + input->phi_rad;
    sin_theta_plus_phi = arm_sin_f32(theta_plus_phi);
    cos_theta_plus_phi = arm_cos_f32(theta_plus_phi);

    m11 = DOUBLE_PITCH_MODEL_A0 +
          DOUBLE_PITCH_MODEL_AC * cos_phi +
          DOUBLE_PITCH_MODEL_AS * sin_phi;
    m12 = DOUBLE_PITCH_MODEL_B0 +
          DOUBLE_PITCH_MODEL_BC * cos_phi +
          DOUBLE_PITCH_MODEL_BS * sin_phi;
    m22 = DOUBLE_PITCH_MODEL_C0;
    m11_dot_phi = -DOUBLE_PITCH_MODEL_AC * sin_phi +
                  DOUBLE_PITCH_MODEL_AS * cos_phi;
    m12_dot_phi = -DOUBLE_PITCH_MODEL_BC * sin_phi +
                  DOUBLE_PITCH_MODEL_BS * cos_phi;
    c_big = m11_dot_phi * input->omega_big_rad_s * input->omega_small_rad_s +
            m12_dot_phi * input->omega_small_rad_s * input->omega_small_rad_s;
    c_small = -0.5f * m11_dot_phi *
              input->omega_big_rad_s * input->omega_big_rad_s;

    tau_big_fb = m11 * v_big + m12 * v_small;
    tau_small_fb = m12 * v_big + m22 * v_small;
    tau_big_g = DOUBLE_PITCH_MODEL_GB1 * sin_theta +
                DOUBLE_PITCH_MODEL_GB2 * cos_theta +
                DOUBLE_PITCH_MODEL_GP1 * sin_theta_plus_phi +
                DOUBLE_PITCH_MODEL_GP2 * cos_theta_plus_phi;
    tau_small_g = DOUBLE_PITCH_MODEL_GP1 * sin_theta_plus_phi +
                  DOUBLE_PITCH_MODEL_GP2 * cos_theta_plus_phi;
    f_big = DOUBLE_PITCH_MODEL_FVB * input->omega_big_rad_s +
            DOUBLE_PITCH_MODEL_FCB *
            DoublePitchSmoothSign(input->omega_big_rad_s);
    f_small = DOUBLE_PITCH_MODEL_FVS * input->omega_small_rad_s +
              DOUBLE_PITCH_MODEL_FCS *
              DoublePitchSmoothSign(input->omega_small_rad_s);

    tau_big = tau_big_fb + tau_big_g + f_big;
    tau_small = tau_small_fb + tau_small_g + f_small;

    output->tau_big_pre_limit_nm = tau_big;
    output->tau_small_pre_limit_nm = tau_small;

    torque_limit = DoublePitchAbs(input->torque_big_limit_nm);
    if (torque_limit <= 0.0f || torque_limit > DOUBLE_PITCH_DM_T_LIMIT_NM) {
        torque_limit = DOUBLE_PITCH_DM_T_LIMIT_NM;
    }
    old_tau = tau_big;
    tau_big = DoublePitchClampAbs(tau_big, torque_limit);
    output->tau_big_limit_flag =
        (tau_big != old_tau) ? 1U : output->tau_big_limit_flag;

    torque_limit = DoublePitchAbs(input->torque_small_limit_nm);
    if (torque_limit <= 0.0f || torque_limit > DOUBLE_PITCH_DM_T_LIMIT_NM) {
        torque_limit = DOUBLE_PITCH_DM_T_LIMIT_NM;
    }
    old_tau = tau_small;
    tau_small = DoublePitchClampAbs(tau_small, torque_limit);
    output->tau_small_limit_flag =
        (tau_small != old_tau) ? 1U : output->tau_small_limit_flag;

    old_tau = tau_big;
    tau_big = DoublePitchClampAbs(tau_big, DOUBLE_PITCH_DM_T_LIMIT_NM);
    output->tau_big_limit_flag =
        (tau_big != old_tau) ? 1U : output->tau_big_limit_flag;
    old_tau = tau_small;
    tau_small = DoublePitchClampAbs(tau_small, DOUBLE_PITCH_DM_T_LIMIT_NM);
    output->tau_small_limit_flag =
        (tau_small != old_tau) ? 1U : output->tau_small_limit_flag;

    if (input->torque_slew_enable != 0U) {
        slew_rate = DoublePitchAbs(input->torque_big_slew_rate_nm_s);
        if (slew_rate > 0.0f) {
            const float max_delta = slew_rate * dt_s;
            old_tau = tau_big;
            tau_big = DoublePitchClamp(tau_big,
                                       ctrl->tau_big_last_nm - max_delta,
                                       ctrl->tau_big_last_nm + max_delta);
            output->tau_big_limit_flag =
                (tau_big != old_tau) ? 1U : output->tau_big_limit_flag;
        }

        slew_rate = DoublePitchAbs(input->torque_small_slew_rate_nm_s);
        if (slew_rate > 0.0f) {
            const float max_delta = slew_rate * dt_s;
            old_tau = tau_small;
            tau_small = DoublePitchClamp(tau_small,
                                         ctrl->tau_small_last_nm - max_delta,
                                         ctrl->tau_small_last_nm + max_delta);
            output->tau_small_limit_flag =
                (tau_small != old_tau) ? 1U : output->tau_small_limit_flag;
        }
    }

    ctrl->tau_big_last_nm = tau_big;
    ctrl->tau_small_last_nm = tau_small;

    output->tau_big_cmd_nm = tau_big;
    output->tau_small_cmd_nm = tau_small;
    output->tau_big_fb_nm = tau_big_fb;
    output->tau_small_fb_nm = tau_small_fb;
    output->tau_big_g_nm = tau_big_g;
    output->tau_small_g_nm = tau_small_g;
    output->v_big_cmd_rad_s2 = v_big;
    output->v_small_cmd_rad_s2 = v_small;
    output->e_theta_rad = e_theta;
    output->e_phi_rad = e_phi;
    output->e_omega_big_rad_s = e_omega_big;
    output->e_omega_small_rad_s = e_omega_small;
    output->k_segment = k_segment;

    model.k_alpha = k_alpha;
    model.theta_big_ddot_radps2 = theta_big_ddot_radps2;
    model.phi_rel_ddot_radps2 = phi_rel_ddot_radps2;
    model.sin_phi = sin_phi;
    model.cos_phi = cos_phi;
    model.sin_theta = sin_theta;
    model.cos_theta = cos_theta;
    model.sin_theta_plus_phi = sin_theta_plus_phi;
    model.cos_theta_plus_phi = cos_theta_plus_phi;
    model.m11 = m11;
    model.m12 = m12;
    model.m22 = m22;
    model.m11_dot_phi = m11_dot_phi;
    model.m12_dot_phi = m12_dot_phi;
    model.c_big = c_big;
    model.c_small = c_small;
    model.g_big = tau_big_g;
    model.g_small = tau_small_g;
    model.f_big = f_big;
    model.f_small = f_small;
    model.k_segment = k_segment;
    for (row = 0U; row < 2U; row++) {
        for (col = 0U; col < 4U; col++) {
            model.k[row][col] = k[row][col];
        }
    }

    DoublePitchPublishDebug(ctrl, input, output, dt_s);
    DoublePitchPublishModelDebug(ctrl, input, output, &model, dt_s);
}
