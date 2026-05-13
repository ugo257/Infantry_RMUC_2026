#include "stdio.h"
#include "math.h"

#include "gimbal.h"
#include "robot_board.h"
#include "robot_params.h"
#include "robot_types.h"
#include "DMmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"

#include "bmi088.h"
#include "referee_UI.h"
#include "controller.h"
#include "bsp_dwt.h"
#include "lqr_eso.h"
#define DZX_YAW_TORQUE_LIMIT   7.0f
#define DZX_YAW_RAD_TO_DEG     57.2957795f
#if defined(GIMBAL_BOARD)
#define PITCH_DOWN_DEFAULT_REF_POS_RAD      1.95f
#define PITCH_DOWN_LQR_K_THETA              31.622777f
#define PITCH_DOWN_LQR_K_OMEGA              1.558379f
#define PITCH_DOWN_LQR_TORQUE_LIMIT_NM      5.00f
#define PITCH_DOWN_TORQUE_SLEW_RATE_NM_S    150.0f
#define PITCH_DOWN_VEL_LIMIT_RAD_S          1.5f
#define PITCH_DOWN_MODEL_J_KG_M2            0.0411f
#define PITCH_DOWN_MODEL_B_NMS_RAD          0.0869f
#define PITCH_DOWN_MODEL_Q_THETA            1800.0f
#define PITCH_DOWN_MODEL_Q_OMEGA            1.0f
#define PITCH_DOWN_MODEL_R_TORQUE           1.0f
#define PITCH_DOWN_DEFAULT_LOWER_LIMIT_RAD  1.30f
#define PITCH_DOWN_DEFAULT_UPPER_LIMIT_RAD  2.50f
#define PITCH_DOWN_PITCH_CMD_TO_POS_GAIN    0.9935044f
#define PITCH_DOWN_FOLLOW_MAX_OFFSET_RAD    0.17453293f
#define PITCH_DOWN_MIT_SWITCH_POS_RAD       (PITCH_DOWN_DEFAULT_REF_POS_RAD - PITCH_DOWN_FOLLOW_MAX_OFFSET_RAD)
#define PITCH_DOWN_VEL_LPF_DEFAULT_RC_S     0.030f
#define PITCH_DOWN_HOLD_I_KI_NM_PER_RAD_S   4.0f
#define PITCH_DOWN_HOLD_I_LIMIT_NM          0.20f
#define PITCH_DOWN_HOLD_I_ERROR_DEADBAND_RAD 0.005f
#define PITCH_DOWN_HOLD_I_ENABLE_ERROR_RAD  0.12f
#define PITCH_DOWN_HOLD_I_ENABLE_SPEED_RAD_S 0.15f
#define PITCH_DOWN_CONTROL_MIN_DT_S         0.0001f
#define PITCH_DOWN_CONTROL_MAX_DT_S         0.02f
#define PITCH_DOWN_GRAVITY_CENTER_RAD       1.65f
#define PITCH_DOWN_GRAVITY_BIAS_NM          0.507f
#define PITCH_DOWN_GRAVITY_LOW_GAIN_NM      1.868f
#define PITCH_DOWN_GRAVITY_HIGH_GAIN_NM     1.901f
#define PITCH_DOWN_GRAVITY_COMP_LIMIT_NM    1.25f
#define PITCH_DOWN_ESO_ENABLE               1U
#define PITCH_DOWN_ESO_ALPHA                1.0f
#define PITCH_DOWN_ESO_COMP_GAIN            0.7f
#define PITCH_DOWN_ESO_COMP_LIMIT_NM        2.0f
#define PITCH_DOWN_ESO_W0_MIN_RAD_S         0.0f
#define PITCH_DOWN_ESO_W0_MAX_RAD_S         80.0f
#define PITCH_UP_TORQUE_LIMIT_NM            7.00f
/* Public IMU pitch is inverted at INS output, so the motor torque sign is inverted with it. */
#define PITCH_UP_CONTROL_TO_MOTOR_TORQUE_SIGN (1.0f)
#define PITCH_UP_LQR_K_THETA                73.029674f
#define PITCH_UP_LQR_K_OMEGA                2.899296f
#define PITCH_UP_GRAVITY_BIAS_NM            (-1.05f)
#define PITCH_UP_GRAVITY_SIN_GAIN_NM        0.25f
#define PITCH_UP_GRAVITY_COMP_LIMIT_NM      1.20f
#define PITCH_UP_ESO_TRIAL_ENABLE           1U
#define PITCH_UP_MODEL_J_KG_M2              0.036f
#define PITCH_UP_MODEL_B_NMS_RAD            0.032f
#define PITCH_UP_ESO_ENABLE                 1U
#define PITCH_UP_ESO_ALPHA                  1.5f
#define PITCH_UP_ESO_COMP_GAIN              0.5f
#define PITCH_UP_ESO_COMP_LIMIT_NM          1.2f
#define PITCH_UP_ESO_W0_MIN_RAD_S           0.0f
#define PITCH_UP_ESO_W0_MAX_RAD_S           80.0f
#define PITCH_UP_IMU_ZERO_REF_RAD           0.0f
#define PITCH_UP_IMU_COUPLE_UP_COEF         0.9938710f
#define PITCH_UP_IMU_COUPLE_DOWN_COEF       1.0048164f
#define PITCH_UP_IMU_COUPLE_OFFSET_RAD      (-1.1482353f)
#endif
#ifndef DZX_YAW_FEEDBACK_SOURCE_IMU_TEST
#define DZX_YAW_FEEDBACK_SOURCE_IMU_TEST 1
#endif
static INS_Instance *gimbal_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor;
DMMotorInstance *pitch_motor_up;
DMMotorInstance *pitch_motor_down;
static YawLQRESO_s dzx_yaw_lqr_eso;
#if defined(GIMBAL_BOARD)
static PitchLQRESO_s dzx_pitch_lqr_eso;
static PitchLQRESO_s dzx_pitch_down_lqr_eso;
#endif
static uint32_t dzx_yaw_lqr_dwt_cnt;
#if defined(GIMBAL_BOARD)
static uint32_t dzx_pitch_lqr_dwt_cnt;
static uint32_t dzx_pitch_down_lqr_dwt_cnt;
#endif
static float dzx_yaw_torque;
#if defined(GIMBAL_BOARD)
static float dzx_pitch_torque;
static float pitch_ref_limited;
#endif
static Publisher_t *gimbal_pub;                   // 云台反馈消息发布者（发布给cmd）
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 发送给cmd的云台反馈数据
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 从cmd接收的云台控制命令
#if defined(GIMBAL_BOARD)
static float pitch_speed_feedforward = 0;
typedef struct
{
    float time_s;
    float debug_dt_s;
    float cmd_pitch_rad;
    float cmd_pitch_version_rad;
    uint8_t gimbal_mode;
    uint8_t nuc_mode;
    float down_ref_pos_rad;
    float down_ref_limited_pos_rad;
    float down_ref_target_limited_pos_rad;
    float down_ref_ramp_rate_rad_s;
    uint8_t down_ref_ramp_active;
    float down_pos_rad;
    float down_pos_error_rad;
    float down_vel_raw_rad_s;
    float down_vel_lpf_rad_s;
    float down_torque_cmd_nm;
    float down_gravity_torque_ff_nm;
    float down_gravity_torque_raw_nm;
    float down_gravity_torque_pre_limit_nm;
    float down_gravity_angle_offset_sin;
    float down_gravity_gain_nm;
    uint8_t down_gravity_high_down_scaled;
    float down_model_j_kg_m2;
    float down_model_b_nms_rad;
    float down_model_a22;
    float down_model_b2;
    float down_lqr_k_theta_model;
    float down_lqr_k_omega_model;
    float down_model_q_theta;
    float down_model_q_omega;
    float down_model_r_torque;
    float down_lqr_k_theta_used;
    float down_lqr_k_omega_used;
    uint8_t down_eso_enable;
    float down_eso_alpha;
    float down_eso_b0;
    float down_eso_w0;
    float down_eso_beta1;
    float down_eso_beta2;
    float down_eso_beta3;
    float down_eso_z1;
    float down_eso_z2;
    float down_eso_z3;
    float down_eso_tau_comp_nm;
    float down_eso_comp_gain;
    float down_eso_comp_limit_nm;
    float down_ident_torque_nm;
    float down_lqr_e_theta_rad;
    float down_lqr_e_omega_rad_s;
    float down_lqr_tau_raw_nm;
    float down_lqr_pre_limit_torque_nm;
    float down_lqr_output_nm;
    float down_hold_i_error_rad;
    float down_hold_i_torque_nm;
    uint8_t down_hold_i_active;
    float down_motor_tor_nm;
    float down_ctrl_tor_set_nm;
    int down_motor_p_int;
    int down_motor_v_int;
    int down_motor_t_int;
    float down_motor_total_pos_rad;
    float down_vel_lpf_alpha;
    float up_imu_lower_limit_rad;
    float up_imu_upper_limit_rad;
    float up_torque_cmd_nm;
    float up_lqr_torque_nm;
    float up_gravity_torque_ff_nm;
    float up_motor_pos_rad;
    float up_motor_vel_rad_s;
    float up_motor_tor_nm;
    float imu_pitch_rad;
    float imu_pitch_gyro_rad_s;
    float coupled_imu_estimate_rad;
    float coupled_imu_error_rad;
} PitchDownDebugCsv_s;

volatile PitchDownDebugCsv_s pitch_down_debug_csv;
volatile float pitch_down_ref_pos_rad          = PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float pitch_down_pos_rad              = 0.0f;
volatile float pitch_down_vel_rad_s            = 0.0f;
volatile float pitch_down_vel_raw_rad_s        = 0.0f;
volatile float pitch_down_pos_error_rad        = 0.0f;
volatile float pitch_down_ref_limited_pos_rad  = PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float pitch_down_ref_target_limited_pos_rad =
    PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile uint8_t pitch_down_ref_ramp_active    = 0U;
volatile float pitch_down_follow_gain          = 0.08f;
volatile float pitch_down_follow_offset_rad    = 0.0f;
volatile float hole_down_enter_pre_mit_kp      = 2.0f;
volatile float hole_down_enter_pre_mit_kd      = 0.001f;
volatile float hole_down_enter_final_mit_kp    = 0.1f;
volatile float hole_down_enter_final_mit_kd    = 0.001f;
volatile float hole_v_des                      =0.01f;
volatile uint8_t hole_down_ready               = 0U;
volatile uint8_t hole_down_pre_ready           = 0U;
volatile uint8_t hole_up_ready                 = 0U;
volatile uint8_t hole_up_torque_zero_latched   = 0U;
volatile float pitch_down_torque_cmd_nm        = 0.0f;
volatile float pitch_down_gravity_torque_ff_nm = 0.0f;
volatile float pitch_down_gravity_torque_raw_nm = 0.0f;
volatile float pitch_down_gravity_torque_pre_limit_nm = 0.0f;
volatile float pitch_down_gravity_angle_offset_sin = 0.0f;
volatile float pitch_down_gravity_gain_nm = 0.0f;
volatile uint8_t pitch_down_gravity_high_down_scaled = 0U;
volatile float pitch_down_lqr_e_theta_rad      = 0.0f;
volatile float pitch_down_lqr_e_omega_rad_s    = 0.0f;
volatile float pitch_down_lqr_tau_raw_nm       = 0.0f;
volatile float pitch_down_lqr_pre_limit_torque_nm = 0.0f;
volatile float pitch_down_lqr_output_nm        = 0.0f;
volatile uint8_t pitch_down_lqr_limit_active   = 0U;
volatile float pitch_down_hold_i_torque_nm     = 0.0f;
volatile float pitch_down_hold_i_error_rad     = 0.0f;
volatile uint8_t pitch_down_hold_i_active      = 0U;
volatile uint8_t pitch_down_soft_limit_active  = 0U;
volatile float pitch_down_vel_lpf_alpha        = 1.0f;
volatile float pitch_up_ref_raw_rad          = PITCH_UP_IMU_ZERO_REF_RAD;
volatile float pitch_up_ref_rad              = 0.0f;
volatile float pitch_up_ref_limited_rad      = PITCH_UP_IMU_ZERO_REF_RAD;
volatile float pitch_up_imu_lower_limit_rad  = 0.0f;
volatile float pitch_up_imu_upper_limit_rad  = 0.0f;
volatile float pitch_up_measure_rad          = 0.0f;
volatile float pitch_up_gyro_rad_s           = 0.0f;
volatile float pitch_up_motor_pos_rad        = 0.0f;
volatile float pitch_up_motor_vel_rad_s      = 0.0f;
volatile float pitch_up_lqr_torque_nm        = 0.0f;
volatile float pitch_up_gravity_torque_ff_nm = 0.0f;
volatile float pitch_up_torque_cmd_nm        = 0.0f;
volatile uint8_t pitch_up_soft_limit_active  = 0U;
volatile uint8_t pitch_up_ref_limit_active   = 0U;
static uint32_t pitch_down_vel_lpf_dwt_cnt;
static uint8_t pitch_down_vel_lpf_primed = 0U;
static float pitch_down_debug_last_time_s = 0.0f;
static uint8_t pitch_down_debug_time_primed = 0U;
static uint8_t pitch_down_ref_override_active = 0U;
static uint8_t pitch_down_ref_limit_bypass = 0U;
static float pitch_down_ref_override_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;
static uint8_t pitch_up_ref_override_active = 0U;
static float pitch_up_ref_override_rad = PITCH_UP_IMU_ZERO_REF_RAD;

static void PitchDownClearLQRDebug(void)
{
    pitch_down_lqr_e_theta_rad = 0.0f;
    pitch_down_lqr_e_omega_rad_s = 0.0f;
    pitch_down_lqr_tau_raw_nm = 0.0f;
    pitch_down_lqr_pre_limit_torque_nm = 0.0f;
    pitch_down_lqr_output_nm = 0.0f;
    pitch_down_lqr_limit_active = 0U;
}

static void PitchDownClearGravityDebug(void)
{
    pitch_down_gravity_torque_raw_nm = 0.0f;
    pitch_down_gravity_torque_pre_limit_nm = 0.0f;
    pitch_down_gravity_angle_offset_sin = 0.0f;
    pitch_down_gravity_gain_nm = 0.0f;
    pitch_down_gravity_high_down_scaled = 0U;
}
#endif
float pitch_current_feedforward,K_pitch_current_feedforward,B_pitch_current_feedforward;
//static float yaw_angle_imu,yaw_gyro_imu;
extern Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;
float yaw_gyro_twoboard=0,yaw_current_feedforward;
static float gimbal_yaw_angle_feedback_rad = 0.0f;
static float gimbal_yaw_speed_feedback_rad_s = 0.0f;
volatile float test_yaw_ref = 0.0f;
volatile float yaw_fit_time_s = 0.0f;
volatile float yaw_angle_rad = 0.0f;
volatile float yaw_gyro_rad_s = 0.0f;
volatile float motor_torque_nm = 0.0f;
volatile float yaw_motor_pos_rad = 0.0f;
volatile float yaw_motor_total_rad = 0.0f;
volatile float angle_error = 0.0f;

const float pitch_offset = 7.7f;
float pitch_tor_feedforward = 0;
float pitch_gyro_measure = 0;
// static PID_Setting_s pitch_settings,yaw_settings;
// extern float imu_angle[3];
// extern float imu_gyro[3];
float currentFromAngle(float x){
    float y=-0.0011*pow(x,5)+0.0623*pow(x,4)-1.2218*pow(x,3)+7.4649*pow(x,2)-42*x+6776;

    return -y;
}

#if defined(GIMBAL_BOARD)
static float PitchDownClampRange(float value, float lower_limit, float upper_limit)
{
    if (lower_limit > upper_limit) {
        const float temp = lower_limit;
        lower_limit = upper_limit;
        upper_limit = temp;
    }

    if (value < lower_limit) {
        return lower_limit;
    }
    if (value > upper_limit) {
        return upper_limit;
    }
    return value;
}

static void PitchDownResetHoldIntegral(void)
{
    pitch_down_hold_i_torque_nm = 0.0f;
    pitch_down_hold_i_error_rad = 0.0f;
    pitch_down_hold_i_active = 0U;
}

static float PitchUpMotorLimitCoordRad(float raw_pos_rad)
{
    const float half_period = PITCH_UP_MOTOR_WRAP_PERIOD_RAD * 0.5f;

    while ((raw_pos_rad - PITCH_UP_MOTOR_UNWRAP_CENTER_RAD) > half_period) {
        raw_pos_rad -= PITCH_UP_MOTOR_WRAP_PERIOD_RAD;
    }
    while ((raw_pos_rad - PITCH_UP_MOTOR_UNWRAP_CENTER_RAD) < -half_period) {
        raw_pos_rad += PITCH_UP_MOTOR_WRAP_PERIOD_RAD;
    }
    return raw_pos_rad;
}

static float PitchUpEstimateImuRad(float up_raw_pos_rad, float down_pos_rad)
{
    return PITCH_UP_IMU_COUPLE_UP_COEF *
               PitchUpMotorLimitCoordRad(up_raw_pos_rad) -
           PITCH_UP_IMU_COUPLE_DOWN_COEF * down_pos_rad +
           PITCH_UP_IMU_COUPLE_OFFSET_RAD;
}

static float PitchDownSelectTargetRefPos(void)
{
    if (pitch_down_ref_override_active != 0U) {
        return pitch_down_ref_override_rad;
    }
    return PITCH_DOWN_DEFAULT_REF_POS_RAD;
}

static void PitchDownSetRefOverride(float ref_rad, uint8_t bypass_default_limit)
{
    pitch_down_ref_override_active = 1U;
    pitch_down_ref_limit_bypass = bypass_default_limit;
    pitch_down_ref_override_rad = ref_rad;
}

static void PitchDownClearRefOverride(void)
{
    pitch_down_ref_override_active = 0U;
    pitch_down_ref_limit_bypass = 0U;
    pitch_down_ref_override_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;
}

static void PitchDownApplyNormalFollowRef(void)
{
    pitch_down_follow_offset_rad = 0.0f;
    PitchDownSetRefOverride(PITCH_DOWN_DEFAULT_REF_POS_RAD, 0U);
}

static void PitchUpSetRefOverride(float ref_rad)
{
    pitch_up_ref_override_active = 1U;
    pitch_up_ref_override_rad = ref_rad;
}

static void PitchUpClearRefOverride(void)
{
    pitch_up_ref_override_active = 0U;
    pitch_up_ref_override_rad = PITCH_UP_IMU_ZERO_REF_RAD;
}

static void PitchDownResetVelocityFilter(float vel_rad_s)
{
    pitch_down_vel_raw_rad_s = vel_rad_s;
    pitch_down_vel_rad_s = vel_rad_s;
    pitch_down_vel_lpf_alpha = 1.0f;
    pitch_down_vel_lpf_primed = 1U;
    DWT_GetDeltaT(&pitch_down_vel_lpf_dwt_cnt);
}

static void PitchDownClearVelocityFilter(void)
{
    pitch_down_vel_raw_rad_s = 0.0f;
    pitch_down_vel_rad_s = 0.0f;
    pitch_down_vel_lpf_alpha = 1.0f;
    pitch_down_vel_lpf_primed = 0U;
    DWT_GetDeltaT(&pitch_down_vel_lpf_dwt_cnt);
}

static float PitchDownUpdateVelocityFilter(float raw_vel_rad_s)
{
    const float rc = PITCH_DOWN_VEL_LPF_DEFAULT_RC_S;
    float dt = DWT_GetDeltaT(&pitch_down_vel_lpf_dwt_cnt);
    float alpha;

    pitch_down_vel_raw_rad_s = raw_vel_rad_s;
    dt = PitchDownClampRange(dt,
                             PITCH_DOWN_CONTROL_MIN_DT_S,
                             PITCH_DOWN_CONTROL_MAX_DT_S);
    alpha = dt / (rc + dt);
    alpha = PitchDownClampRange(alpha, 0.0f, 1.0f);
    pitch_down_vel_lpf_alpha = alpha;

    if (pitch_down_vel_lpf_primed == 0U) {
        pitch_down_vel_rad_s = raw_vel_rad_s;
        pitch_down_vel_lpf_primed = 1U;
        return pitch_down_vel_rad_s;
    }

    pitch_down_vel_rad_s += alpha * (raw_vel_rad_s - pitch_down_vel_rad_s);
    return pitch_down_vel_rad_s;
}

static uint8_t PitchDownRefreshControlState(void)
{
    pitch_down_ref_pos_rad = PitchDownSelectTargetRefPos();
    pitch_down_ref_target_limited_pos_rad =
        (pitch_down_ref_limit_bypass != 0U) ?
        pitch_down_ref_pos_rad :
        PitchDownClampRange(pitch_down_ref_pos_rad,
                            PITCH_DOWN_DEFAULT_LOWER_LIMIT_RAD,
                            PITCH_DOWN_DEFAULT_UPPER_LIMIT_RAD);
    pitch_down_soft_limit_active = 0U;

    if (pitch_motor_down == NULL) {
        pitch_down_pos_rad = 0.0f;
        PitchDownClearVelocityFilter();
        pitch_down_ref_ramp_active = 0U;
        pitch_down_pos_error_rad = 0.0f;
        pitch_down_ref_limited_pos_rad =
            pitch_down_ref_target_limited_pos_rad;
        pitch_down_torque_cmd_nm = 0.0f;
        pitch_down_gravity_torque_ff_nm = 0.0f;
        return 0U;
    }

    pitch_down_pos_rad = pitch_motor_down->measure.pos;
    pitch_down_vel_rad_s =
        PitchDownUpdateVelocityFilter(pitch_motor_down->measure.vel);
    if (!DMMotorIsOnline(pitch_motor_down)) {
        pitch_down_ref_ramp_active = 0U;
        pitch_down_ref_limited_pos_rad =
            pitch_down_ref_target_limited_pos_rad;
        pitch_down_pos_error_rad = 0.0f;
        return 0U;
    }
    pitch_down_ref_limited_pos_rad =
        pitch_down_ref_target_limited_pos_rad;
    pitch_down_ref_ramp_active = 0U;
    pitch_down_pos_error_rad =
        pitch_down_ref_limited_pos_rad - pitch_down_pos_rad;

    return 1U;
}

static uint8_t PitchDownRefreshMITPositionState(void)
{
    pitch_down_ref_pos_rad = PitchDownSelectTargetRefPos();
    pitch_down_ref_target_limited_pos_rad =
        (pitch_down_ref_limit_bypass != 0U) ?
        pitch_down_ref_pos_rad :
        PitchDownClampRange(pitch_down_ref_pos_rad,
                            PITCH_DOWN_DEFAULT_LOWER_LIMIT_RAD,
                            PITCH_DOWN_DEFAULT_UPPER_LIMIT_RAD);
    pitch_down_ref_limited_pos_rad =
        pitch_down_ref_target_limited_pos_rad;
    pitch_down_ref_ramp_active = 0U;
    pitch_down_soft_limit_active = 0U;

    if (pitch_motor_down == NULL) {
        pitch_down_pos_rad = 0.0f;
        PitchDownClearVelocityFilter();
        pitch_down_ref_ramp_active = 0U;
        pitch_down_pos_error_rad = 0.0f;
        pitch_down_torque_cmd_nm = 0.0f;
        pitch_down_gravity_torque_ff_nm = 0.0f;
        return 0U;
    }

    pitch_down_pos_rad = pitch_motor_down->measure.pos;
    pitch_down_vel_rad_s =
        PitchDownUpdateVelocityFilter(pitch_motor_down->measure.vel);
    if (!DMMotorIsOnline(pitch_motor_down)) {
        pitch_down_pos_error_rad = 0.0f;
        return 0U;
    }
    pitch_down_pos_error_rad =
        pitch_down_ref_limited_pos_rad - pitch_down_pos_rad;

    return 1U;
}

static void PitchDownResetUnavailableControl(void)
{
    if (pitch_motor_down != NULL) {
        DMMotorSetTorque(pitch_motor_down, 0.0f);
    }

    pitch_down_torque_cmd_nm = 0.0f;
    pitch_down_gravity_torque_ff_nm = 0.0f;
    pitch_down_soft_limit_active = 0U;

    PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    PitchDownClearLQRDebug();
    PitchDownClearGravityDebug();
    PitchDownClearVelocityFilter();
    PitchDownResetHoldIntegral();
}

static void PitchDownUpdateDebugCsv(void)
{
    const float now_s = DWT_GetTimeline_s();

    pitch_down_debug_csv.time_s = now_s;
    if (pitch_down_debug_time_primed == 0U) {
        pitch_down_debug_csv.debug_dt_s = 0.0f;
        pitch_down_debug_time_primed = 1U;
    } else {
        pitch_down_debug_csv.debug_dt_s = now_s - pitch_down_debug_last_time_s;
    }
    pitch_down_debug_last_time_s = now_s;

    pitch_down_debug_csv.cmd_pitch_rad =
        gimbal_cmd_recv.pitch;
    pitch_down_debug_csv.cmd_pitch_version_rad =
        gimbal_cmd_recv.pitch_version;
    pitch_down_debug_csv.gimbal_mode =
        (uint8_t)gimbal_cmd_recv.gimbal_mode;
    pitch_down_debug_csv.nuc_mode =
        (uint8_t)gimbal_cmd_recv.nuc_mode;
    pitch_down_debug_csv.down_ref_pos_rad =
        pitch_down_ref_pos_rad;
    pitch_down_debug_csv.down_ref_limited_pos_rad =
        pitch_down_ref_limited_pos_rad;
    pitch_down_debug_csv.down_ref_target_limited_pos_rad =
        pitch_down_ref_target_limited_pos_rad;
    pitch_down_debug_csv.down_ref_ramp_rate_rad_s =
        0.0f;
    pitch_down_debug_csv.down_ref_ramp_active =
        pitch_down_ref_ramp_active;
    pitch_down_debug_csv.down_pos_rad =
        pitch_down_pos_rad;
    pitch_down_debug_csv.down_pos_error_rad =
        pitch_down_pos_error_rad;
    pitch_down_debug_csv.down_vel_raw_rad_s =
        pitch_down_vel_raw_rad_s;
    pitch_down_debug_csv.down_vel_lpf_rad_s =
        pitch_down_vel_rad_s;
    pitch_down_debug_csv.down_torque_cmd_nm =
        pitch_down_torque_cmd_nm;
    pitch_down_debug_csv.down_gravity_torque_ff_nm =
        pitch_down_gravity_torque_ff_nm;
    pitch_down_debug_csv.down_gravity_torque_raw_nm =
        pitch_down_gravity_torque_raw_nm;
    pitch_down_debug_csv.down_gravity_torque_pre_limit_nm =
        pitch_down_gravity_torque_pre_limit_nm;
    pitch_down_debug_csv.down_gravity_angle_offset_sin =
        pitch_down_gravity_angle_offset_sin;
    pitch_down_debug_csv.down_gravity_gain_nm =
        pitch_down_gravity_gain_nm;
    pitch_down_debug_csv.down_gravity_high_down_scaled =
        pitch_down_gravity_high_down_scaled;
    pitch_down_debug_csv.down_model_j_kg_m2 =
        dzx_pitch_down_lqr_eso.lqr_config.plant_j_kg_m2;
    pitch_down_debug_csv.down_model_b_nms_rad =
        dzx_pitch_down_lqr_eso.lqr_config.plant_b_nms_rad;
    pitch_down_debug_csv.down_model_a22 =
        dzx_pitch_down_lqr_eso.lqr_config.model_a22;
    pitch_down_debug_csv.down_model_b2 =
        dzx_pitch_down_lqr_eso.lqr_config.model_b2;
    pitch_down_debug_csv.down_lqr_k_theta_model =
        dzx_pitch_down_lqr_eso.lqr_config.model_k_theta;
    pitch_down_debug_csv.down_lqr_k_omega_model =
        dzx_pitch_down_lqr_eso.lqr_config.model_k_omega;
    pitch_down_debug_csv.down_model_q_theta =
        dzx_pitch_down_lqr_eso.lqr_config.q_theta;
    pitch_down_debug_csv.down_model_q_omega =
        dzx_pitch_down_lqr_eso.lqr_config.q_omega;
    pitch_down_debug_csv.down_model_r_torque =
        dzx_pitch_down_lqr_eso.lqr_config.r_torque;
    pitch_down_debug_csv.down_lqr_k_theta_used =
        dzx_pitch_down_lqr_eso.k_theta_used;
    pitch_down_debug_csv.down_lqr_k_omega_used =
        dzx_pitch_down_lqr_eso.k_omega_used;
    pitch_down_debug_csv.down_eso_enable =
        dzx_pitch_down_lqr_eso.eso_config.enable;
    pitch_down_debug_csv.down_eso_alpha =
        dzx_pitch_down_lqr_eso.eso_config.alpha;
    pitch_down_debug_csv.down_eso_b0 =
        dzx_pitch_down_lqr_eso.eso.b0;
    pitch_down_debug_csv.down_eso_w0 =
        dzx_pitch_down_lqr_eso.eso.w0;
    pitch_down_debug_csv.down_eso_beta1 =
        dzx_pitch_down_lqr_eso.eso.beta1;
    pitch_down_debug_csv.down_eso_beta2 =
        dzx_pitch_down_lqr_eso.eso.beta2;
    pitch_down_debug_csv.down_eso_beta3 =
        dzx_pitch_down_lqr_eso.eso.beta3;
    pitch_down_debug_csv.down_eso_z1 =
        dzx_pitch_down_lqr_eso.eso.z1;
    pitch_down_debug_csv.down_eso_z2 =
        dzx_pitch_down_lqr_eso.eso.z2;
    pitch_down_debug_csv.down_eso_z3 =
        dzx_pitch_down_lqr_eso.eso.z3;
    pitch_down_debug_csv.down_eso_tau_comp_nm =
        dzx_pitch_down_lqr_eso.tau_eso_comp;
    pitch_down_debug_csv.down_eso_comp_gain =
        dzx_pitch_down_lqr_eso.eso_config.comp_gain;
    pitch_down_debug_csv.down_eso_comp_limit_nm =
        dzx_pitch_down_lqr_eso.eso_config.comp_limit;
    pitch_down_debug_csv.down_ident_torque_nm =
        pitch_down_torque_cmd_nm -
        pitch_down_gravity_torque_ff_nm -
        pitch_down_hold_i_torque_nm;
    pitch_down_debug_csv.down_lqr_e_theta_rad =
        pitch_down_lqr_e_theta_rad;
    pitch_down_debug_csv.down_lqr_e_omega_rad_s =
        pitch_down_lqr_e_omega_rad_s;
    pitch_down_debug_csv.down_lqr_tau_raw_nm =
        pitch_down_lqr_tau_raw_nm;
    pitch_down_debug_csv.down_lqr_pre_limit_torque_nm =
        pitch_down_lqr_pre_limit_torque_nm;
    pitch_down_debug_csv.down_lqr_output_nm =
        pitch_down_lqr_output_nm;
    pitch_down_debug_csv.down_hold_i_error_rad =
        pitch_down_hold_i_error_rad;
    pitch_down_debug_csv.down_hold_i_torque_nm =
        pitch_down_hold_i_torque_nm;
    pitch_down_debug_csv.down_hold_i_active =
        pitch_down_hold_i_active;

    if (pitch_motor_down != NULL) {
        pitch_down_debug_csv.down_motor_tor_nm =
            pitch_motor_down->measure.tor;
        pitch_down_debug_csv.down_ctrl_tor_set_nm =
            pitch_motor_down->ctrl.tor_set;
        pitch_down_debug_csv.down_motor_p_int =
            pitch_motor_down->measure.p_int;
        pitch_down_debug_csv.down_motor_v_int =
            pitch_motor_down->measure.v_int;
        pitch_down_debug_csv.down_motor_t_int =
            pitch_motor_down->measure.t_int;
        pitch_down_debug_csv.down_motor_total_pos_rad =
            pitch_motor_down->measure.total_pos;
    } else {
        pitch_down_debug_csv.down_motor_tor_nm = 0.0f;
        pitch_down_debug_csv.down_ctrl_tor_set_nm = 0.0f;
        pitch_down_debug_csv.down_motor_p_int = 0;
        pitch_down_debug_csv.down_motor_v_int = 0;
        pitch_down_debug_csv.down_motor_t_int = 0;
        pitch_down_debug_csv.down_motor_total_pos_rad = 0.0f;
    }

    pitch_down_debug_csv.down_vel_lpf_alpha =
        pitch_down_vel_lpf_alpha;

    pitch_down_debug_csv.up_imu_lower_limit_rad =
        pitch_up_imu_lower_limit_rad;
    pitch_down_debug_csv.up_imu_upper_limit_rad =
        pitch_up_imu_upper_limit_rad;
    pitch_down_debug_csv.up_torque_cmd_nm =
        pitch_up_torque_cmd_nm;
    pitch_down_debug_csv.up_lqr_torque_nm =
        pitch_up_lqr_torque_nm;
    pitch_down_debug_csv.up_gravity_torque_ff_nm =
        pitch_up_gravity_torque_ff_nm;
    pitch_down_debug_csv.up_motor_pos_rad =
        pitch_up_motor_pos_rad;
    pitch_down_debug_csv.up_motor_vel_rad_s =
        pitch_up_motor_vel_rad_s;
    if (pitch_motor_up != NULL) {
        pitch_down_debug_csv.up_motor_tor_nm =
            pitch_motor_up->measure.tor;
    } else {
        pitch_down_debug_csv.up_motor_tor_nm = 0.0f;
    }
    if (gimbal_IMU_data != NULL) {
        pitch_down_debug_csv.imu_pitch_rad =
            gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
        pitch_down_debug_csv.imu_pitch_gyro_rad_s =
            gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
    } else {
        pitch_down_debug_csv.imu_pitch_rad = 0.0f;
        pitch_down_debug_csv.imu_pitch_gyro_rad_s = 0.0f;
    }
    if (pitch_motor_up != NULL && pitch_motor_down != NULL) {
        pitch_down_debug_csv.coupled_imu_estimate_rad =
            PitchUpEstimateImuRad(pitch_motor_up->measure.pos,
                                  pitch_motor_down->measure.pos);
        pitch_down_debug_csv.coupled_imu_error_rad =
            pitch_down_debug_csv.imu_pitch_rad -
            pitch_down_debug_csv.coupled_imu_estimate_rad;
    } else {
        pitch_down_debug_csv.coupled_imu_estimate_rad = 0.0f;
        pitch_down_debug_csv.coupled_imu_error_rad = 0.0f;
    }
}

static float PitchDownCalculateLQRTorque(void)
{
    static float ref_pos;
    float lqr_dt;
    float torque_lqreso;
    float torque_cmd;

    if (PitchDownRefreshControlState() == 0U) {
        PitchDownResetUnavailableControl();
        return 0.0f;
    }

    ref_pos = pitch_down_ref_limited_pos_rad;

    lqr_dt = DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    pitch_down_gravity_torque_ff_nm = 0.0f;
    PitchDownClearGravityDebug();
    PitchDownResetHoldIntegral();

    pitch_down_lqr_e_theta_rad = pitch_down_pos_rad - ref_pos;
    pitch_down_lqr_e_omega_rad_s = pitch_down_vel_rad_s;

    torque_lqreso = PitchLQRESO_Calc(&dzx_pitch_down_lqr_eso,
                                     ref_pos,
                                     pitch_down_pos_rad,
                                     pitch_down_vel_rad_s,
                                     0.0f,
                                     lqr_dt);
    pitch_down_lqr_tau_raw_nm = dzx_pitch_down_lqr_eso.tau_lqr;
    pitch_down_lqr_pre_limit_torque_nm =
        dzx_pitch_down_lqr_eso.tau_pre_limit;
    pitch_down_lqr_output_nm = torque_lqreso;
    pitch_down_lqr_limit_active =
        (torque_lqreso != pitch_down_lqr_pre_limit_torque_nm) ? 1U : 0U;
    pitch_down_soft_limit_active = 0U;
    torque_cmd = torque_lqreso;

    pitch_down_torque_cmd_nm = torque_cmd;
    return torque_cmd;
}

static void PitchDownApplyMITPositionControlWithGain(float kp, float kd, float vel_set_rad_s)
{
    if (PitchDownRefreshMITPositionState() == 0U) {
        PitchDownResetUnavailableControl();
        return;
    }

    PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    PitchDownClearLQRDebug();
    PitchDownClearGravityDebug();
    PitchDownResetHoldIntegral();
    pitch_down_torque_cmd_nm = 0.0f;
    pitch_down_gravity_torque_ff_nm = 0.0f;

    DMMotorSetMITPositionVelocity(pitch_motor_down,
                                  pitch_down_ref_limited_pos_rad,
	                                  vel_set_rad_s,
	                                  kp,
	                                  kd,
	                                  0.0f);
}

static void PitchDownApplyHybridControlWithGain(float mit_kp,
                                                float mit_kd,
                                                float mit_vel_set_rad_s)
{
    if (pitch_motor_down == NULL || !DMMotorIsOnline(pitch_motor_down)) {
        (void)PitchDownRefreshControlState();
        PitchDownResetUnavailableControl();
        return;
    }

    if (pitch_motor_down->measure.pos < PITCH_DOWN_MIT_SWITCH_POS_RAD) {
        PitchDownApplyMITPositionControlWithGain(mit_kp, mit_kd, mit_vel_set_rad_s);
        return;
    }

    DMMotorSetTorque(pitch_motor_down, PitchDownCalculateLQRTorque());
}

static void PitchDownStopControl(void)
{
    PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    PitchDownClearLQRDebug();
    pitch_down_torque_cmd_nm       = 0.0f;
    pitch_down_gravity_torque_ff_nm = 0.0f;
    pitch_down_soft_limit_active   = 0U;
    PitchDownClearGravityDebug();
    PitchDownClearVelocityFilter();
    pitch_down_ref_ramp_active = 0U;
    PitchDownResetHoldIntegral();
    if (pitch_motor_down != NULL) {
        pitch_down_pos_rad       = pitch_motor_down->measure.pos;
        PitchDownResetVelocityFilter(pitch_motor_down->measure.vel);
        pitch_down_ref_pos_rad   = PitchDownSelectTargetRefPos();
        pitch_down_ref_target_limited_pos_rad =
            PitchDownClampRange(pitch_down_ref_pos_rad,
                                PITCH_DOWN_DEFAULT_LOWER_LIMIT_RAD,
                                PITCH_DOWN_DEFAULT_UPPER_LIMIT_RAD);
        pitch_down_ref_limited_pos_rad = pitch_down_ref_target_limited_pos_rad;
        pitch_down_pos_error_rad = pitch_down_ref_limited_pos_rad - pitch_down_pos_rad;
    }
}
#endif

static uint8_t GimbalUpdateYawIMUFeedback(void)
{
    if (gimbal_IMU_data == NULL) {
        gimbal_yaw_angle_feedback_rad = 0.0f;
        gimbal_yaw_speed_feedback_rad_s = 0.0f;
        return 0U;
    }

    gimbal_yaw_angle_feedback_rad =
        gimbal_IMU_data->output.INS_angle[INS_YAW_ADDRESS_OFFSET];
    gimbal_yaw_speed_feedback_rad_s =
        gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];
    return 1U;
}

static void GimbalYawUpdateFitTelemetry(void)
{
    const uint8_t yaw_closed_loop =
        (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE ||
         gimbal_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE ||
         gimbal_cmd_recv.gimbal_mode == GIMBAL_MOTOR_MODE) ? 1U : 0U;

    yaw_fit_time_s = DWT_GetTimeline_ms() * 0.001f;
    yaw_angle_rad  = gimbal_cmd_recv.yaw_measure_angle * LQR_ESO_DEG_TO_RAD;
    yaw_gyro_rad_s = gimbal_cmd_recv.yaw_measure_gyro;

    g_yaw_lqr_ident_record.sample_count++;
    g_yaw_lqr_ident_record.time_s = yaw_fit_time_s;
    g_yaw_lqr_ident_record.gimbal_mode =
        (uint8_t)gimbal_cmd_recv.gimbal_mode;
    g_yaw_lqr_ident_record.nuc_mode =
        (uint8_t)gimbal_cmd_recv.nuc_mode;

    if (yaw_motor == NULL) {
        motor_torque_nm = 0.0f;
        yaw_motor_pos_rad = 0.0f;
        yaw_motor_total_rad = 0.0f;
        g_yaw_lqr_ident_record.feedback_ok = 0U;
        g_yaw_lqr_ident_record.valid = 0U;
        g_yaw_lqr_ident_record.motor_pos_rad = 0.0f;
        g_yaw_lqr_ident_record.motor_total_pos_rad = 0.0f;
        g_yaw_lqr_ident_record.motor_vel_rad_s = 0.0f;
        g_yaw_lqr_ident_record.motor_tor_nm = 0.0f;
        g_yaw_lqr_ident_record.ctrl_tor_set_nm = 0.0f;
        g_yaw_lqr_ident_record.motor_p_int = 0;
        g_yaw_lqr_ident_record.motor_v_int = 0;
        g_yaw_lqr_ident_record.motor_t_int = 0;
        return;
    }

    motor_torque_nm = yaw_motor->measure.tor;
    yaw_motor_pos_rad = yaw_motor->measure.pos;
    yaw_motor_total_rad = yaw_motor->measure.total_pos;
    g_yaw_lqr_ident_record.feedback_ok = DMMotorIsOnline(yaw_motor);
    if (yaw_closed_loop == 0U || g_yaw_lqr_ident_record.feedback_ok == 0U) {
        g_yaw_lqr_ident_record.valid = 0U;
    }
    g_yaw_lqr_ident_record.motor_pos_rad = yaw_motor->measure.pos;
    g_yaw_lqr_ident_record.motor_total_pos_rad = yaw_motor->measure.total_pos;
    g_yaw_lqr_ident_record.motor_vel_rad_s = yaw_motor->measure.vel;
    g_yaw_lqr_ident_record.motor_tor_nm = yaw_motor->measure.tor;
    g_yaw_lqr_ident_record.ctrl_tor_set_nm = yaw_motor->ctrl.tor_set;
    g_yaw_lqr_ident_record.motor_p_int = yaw_motor->measure.p_int;
    g_yaw_lqr_ident_record.motor_v_int = yaw_motor->measure.v_int;
    g_yaw_lqr_ident_record.motor_t_int = yaw_motor->measure.t_int;
}

uint16_t powerLim;
void GimbalInit()
{
    //bmi088参数初始化
    BMI088_Init_Config_s config = {
        .acc_int_config  = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_4},
        .gyro_int_config = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_5},
        .heat_pid_config = {
            .Kp            = 0.32f,
            .Ki            = 0.0004f,
            .Kd            = 0,
            .Improve       = PID_IMPROVE_NONE,
            .IntegralLimit = 0.90f,
            .MaxOut        = 0.95f,
        },
        .heat_pwm_config = {
            .htim      = &htim10,
            .channel   = TIM_CHANNEL_1,
            .dutyratio = 0,
            .period    = 5000 - 1,
        },
        .spi_acc_config = {
            .GPIOx      = GPIOA,
            .cs_pin     = GPIO_PIN_4,
            .spi_handle = &hspi1,
        },
        .spi_gyro_config = {
            .GPIOx      = GPIOB,
            .cs_pin     = GPIO_PIN_0,
            .spi_handle = &hspi1,
        },
        .cali_mode = BMI088_DEFAULT_CALI_MODE,
        .work_mode = BMI088_DMA_PERIODIC_MODE, // 回退阻塞改 BMI088_BLOCK_PERIODIC_MODE; 中断触发改 BMI088_BLOCK_TRIGGER_MODE
    };
    gimbal_IMU_data = INS_Init(BMI088Register(&config)); // IMU初始化，得到姿态解算结果（含yaw角速度等）
    GimbalUpdateYawIMUFeedback();

#if defined(CHASSIS_BOARD)
    // DZX yaw DM motor: direct torque command, no PID in the driver layer.
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 0x12u,
            .rx_id      = 0x02u,
        },
        .controller_param_init_config = {
            .other_angle_feedback_ptr = &gimbal_yaw_angle_feedback_rad,
            .other_speed_feedback_ptr = &gimbal_yaw_speed_feedback_rad_s,
        },
        .controller_setting_init_config = {
            .outer_loop_type       = OPEN_LOOP,
            .close_loop_type       = OPEN_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .feedforward_flag      = FEEDFORWARD_NONE,
            .control_range = {
                .P_max = Infantry_PMAX,
                .V_max = Infantry_VMAX,
                .T_max = Infantry_TMAX,
            },
        },
        .motor_type = DM_Motor,
        .motor_contro_type = TORQUE_LOOP_CONTRO,
    };
    yaw_motor = DMMotorInit(&yaw_config);
    DMMotorStop(yaw_motor);

    YawLQRESO_Init(&dzx_yaw_lqr_eso);
    DWT_GetDeltaT(&dzx_yaw_lqr_dwt_cnt);
#endif
#if defined(GIMBAL_BOARD)
    //pitch电机初始化参数 DM4310
     Motor_Init_Config_s pitch_motor_config_up = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x11u,
            .rx_id = 0x01u,
        },
        .motor_type = DM_Motor,
        .controller_param_init_config ={
            .angle_PID = {
                .Kp = 10.5,
                .Ki = 0,
                .Kd = 0.01,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 0.5,
                .MaxOut = 20,
            },
            .speed_PID = {
                .Kp = 1.0,
                .Ki = 9.81,
                .Kd = 0.0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 1.5,
                .MaxOut = 2,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],
            .other_speed_feedback_ptr = &gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET],
            .speed_feedforward_ptr = &pitch_speed_feedforward,
            .current_feedforward_ptr = &pitch_tor_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag      = SPEED_FEEDFORWARD,
            .control_range = {
                .P_max = Infantry_PMAX,
                .V_max = Infantry_VMAX,
                .T_max = Infantry_TMAX,
            },
        },
    };
    pitch_motor_up = DMMotorInit(&pitch_motor_config_up);
    DMMotorStop(pitch_motor_up);
    Motor_Init_Config_s pitch_motor_config_down = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x13u,
            .rx_id = 0x03u,
        },
        .motor_type = DM_Motor,
        .controller_param_init_config ={
            .angle_PID = {
                .Kp = 10.5,
                .Ki = 0,
                .Kd = 0.01,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 0.5,
                .MaxOut = 20,
            },
            .speed_PID = {
                .Kp = 1.0,
                .Ki = 9.81,
                .Kd = 0.0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 1.5,
                .MaxOut = 2,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET], // pitch角度反馈
            .other_speed_feedback_ptr = &gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET],
            .speed_feedforward_ptr = &pitch_speed_feedforward,
            .current_feedforward_ptr = &pitch_tor_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag      = SPEED_FEEDFORWARD,
            .control_range = {
                .P_max = Infantry_PMAX,
                .V_max = Infantry_VMAX,
                .T_max = Infantry_TMAX,
            },
        },
    };
    pitch_motor_down = DMMotorInit(&pitch_motor_config_down);
    DMMotorStop(pitch_motor_down);
    LQR_Config_s pitch_lqr_config = {
        .state_dim    = 2U,
        .k            = {PITCH_UP_LQR_K_THETA, PITCH_UP_LQR_K_OMEGA},
        .output_limit = PITCH_UP_TORQUE_LIMIT_NM,
#if PITCH_UP_ESO_TRIAL_ENABLE
        .plant_j_kg_m2 = PITCH_UP_MODEL_J_KG_M2,
        .plant_b_nms_rad = PITCH_UP_MODEL_B_NMS_RAD,
#endif
    };
    LQR_Config_s pitch_down_lqr_config = {
        .state_dim    = 2U,
        .k            = {PITCH_DOWN_LQR_K_THETA, PITCH_DOWN_LQR_K_OMEGA},
        .output_limit = PITCH_DOWN_LQR_TORQUE_LIMIT_NM,
        .output_slew_rate = PITCH_DOWN_TORQUE_SLEW_RATE_NM_S,
        .velocity_limit = PITCH_DOWN_VEL_LIMIT_RAD_S,
        .model_enable = 1U,
        .plant_j_kg_m2 = PITCH_DOWN_MODEL_J_KG_M2,
        .plant_b_nms_rad = PITCH_DOWN_MODEL_B_NMS_RAD,
        .q_theta = PITCH_DOWN_MODEL_Q_THETA,
        .q_omega = PITCH_DOWN_MODEL_Q_OMEGA,
        .r_torque = PITCH_DOWN_MODEL_R_TORQUE,
    };
#if PITCH_UP_ESO_TRIAL_ENABLE
    ESO_Config_s pitch_up_eso_config = {
        .enable = PITCH_UP_ESO_ENABLE,
        .alpha = PITCH_UP_ESO_ALPHA,
        .comp_gain = PITCH_UP_ESO_COMP_GAIN,
        .comp_limit = PITCH_UP_ESO_COMP_LIMIT_NM,
        .w0_min = PITCH_UP_ESO_W0_MIN_RAD_S,
        .w0_max = PITCH_UP_ESO_W0_MAX_RAD_S,
    };
#endif
    ESO_Config_s pitch_down_eso_config = {
        .enable = PITCH_DOWN_ESO_ENABLE,
        .alpha = PITCH_DOWN_ESO_ALPHA,
        .comp_gain = PITCH_DOWN_ESO_COMP_GAIN,
        .comp_limit = PITCH_DOWN_ESO_COMP_LIMIT_NM,
        .w0_min = PITCH_DOWN_ESO_W0_MIN_RAD_S,
        .w0_max = PITCH_DOWN_ESO_W0_MAX_RAD_S,
    };
#if PITCH_UP_ESO_TRIAL_ENABLE
    PitchLQRESO_Init(&dzx_pitch_lqr_eso, &pitch_lqr_config, &pitch_up_eso_config);
#else
    PitchLQRESO_Init(&dzx_pitch_lqr_eso, &pitch_lqr_config, NULL);
#endif
    PitchLQRESO_Init(&dzx_pitch_down_lqr_eso,
                     &pitch_down_lqr_config,
                     &pitch_down_eso_config);
    DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    #endif
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}
/*
 * A nonlinear function that maps a value
 * from the range [X_MIN, -X_MAX] to the range [Y_MIN, Y_MAX].
 */
double nonlinear(const double x) {
#define X_MIN (-0.5)
#define X_MAX 0.5
#define Y_MIN (-2700)
#define Y_MAX 3000

    const double scaled = (x - X_MIN) / (X_MAX - X_MIN) * 4.0 - 2.0;
    const double out = (tanh(scaled) + 1.0) / 2.0;
    return out * (Y_MAX - Y_MIN) + Y_MIN;
}

float PitchNonlinear(float x) {
    //return (215.65 * x -3050.0);
    return (186.0 * x - 1292.0);
}

// 分段三次样条参数
#define NUM_SEGMENTS 11

const double coeffs[NUM_SEGMENTS][4] = {
    {0.8782, 47.2692, -548.1474, 7500.0000},
    {0.8782, 49.9038, -450.9744, 7000.0000},
    {-2.4321, 53.3287, -316.7722, 6500.0000},
    {-2.3856, 8.8205, 62.3377, 6000.0000},
    {34.5316, -56.3055, -369.7766, 5500.0000},
    {-6.0712, 78.3677, -341.0958, 5000.0000},
    {-0.4334, 1.8704, -4.0958, 4500.0000},
    {1.7548, -13.4722, -140.9974, 4000.0000},
    {-3.9207, 20.5351, -95.3710, 3000.0000},
    {5.0001, -41.0982, -203.1216, 2500.0000},
    {5.0001, -12.5977, -305.1439, 2000.0000}
};

const double x_start[NUM_SEGMENTS] = {
    -20.0000, -19.0000, -17.7000, -11.6000, -2.5000, -1.2000, 3.0000, 14.8000, 21.2600, 26.5000, 28.4000,
};

// 计算给定 x 对应的 y 值
double evaluate_spline(const double x) {
    if (x < x_start[0]) {
        return coeffs[0][3];
    } else if (x > x_start[NUM_SEGMENTS - 1]) {
        return coeffs[NUM_SEGMENTS - 1][3];
    }

    int i;

    // 查找 x 所在区间
    for (i = 0; i < NUM_SEGMENTS; i++) {
        if (x >= x_start[i] && (i == NUM_SEGMENTS - 1 || x < x_start[i + 1])) {
            break;
        }
    }

    // 计算 y 值
    const double dx = x - x_start[i];
    const double y = coeffs[i][0] * dx * dx * dx + coeffs[i][1] * dx * dx + coeffs[i][2] * dx + coeffs[i][3];

    return y;
}

static float GimbalWrap360Deg(float angle)
{
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle;
}

static uint16_t GimbalDMYawSingleRoundDeg(void)
{
    if (yaw_motor == NULL) {
        return 0U;
    }
    return (uint16_t)(GimbalWrap360Deg(yaw_motor->measure.pos * DZX_YAW_RAD_TO_DEG) + 0.5f);
}

static float GimbalDMYawTotalDeg(void)
{
    if (yaw_motor == NULL) {
        return 0.0f;
    }
    return yaw_motor->measure.total_pos * DZX_YAW_RAD_TO_DEG;
}

static uint8_t GimbalYawBuildControlInput(float ref_deg, YawControlInput_t *input)
{
    if (input == NULL) {
        return 0U;
    }

    input->theta_ref_deg = ref_deg;
    input->theta_ref_rad = ref_deg * LQR_ESO_DEG_TO_RAD;
    input->omega_ref_rad_s = 0.0f;
    input->alpha_ref_rad_s2 = 0.0f;
    input->torque_slew_rate_nm_s = g_ctrl_torque_slew_rate;
    input->ref_traj_valid = 0U;
    input->auto_aim_mode = 0U;
    input->torque_slew_enable = 1U;
    input->tau_meas_nm   = (yaw_motor == NULL) ? 0.0f : yaw_motor->measure.tor;
    input->feedback_ok   = DMMotorIsOnline(yaw_motor);

    input->theta_rad   = gimbal_cmd_recv.yaw_measure_angle * LQR_ESO_DEG_TO_RAD;
    input->omega_rad_s = gimbal_cmd_recv.yaw_measure_gyro;

    if (gimbal_cmd_recv.nuc_mode == version_control) {
        input->theta_ref_rad = gimbal_cmd_recv.yaw_ref_rad;
        input->omega_ref_rad_s = gimbal_cmd_recv.yaw_ref_vel_rad_s;
        input->alpha_ref_rad_s2 = gimbal_cmd_recv.yaw_ref_acc_rad_s2;
        input->torque_slew_rate_nm_s = g_ctrl_auto_torque_slew_rate;
        input->ref_traj_valid = 1U;
        input->auto_aim_mode = 1U;
        input->torque_slew_enable = 1U;
    }

    return input->feedback_ok;
}

static float GimbalYawCalculateTorque(float ref_deg)
{
    YawControlInput_t input;
    YawControlOutput_t output = {0};

    (void)GimbalYawBuildControlInput(ref_deg, &input);
    YawLQRESO_Calc(&dzx_yaw_lqr_eso, &input, DWT_GetDeltaT(&dzx_yaw_lqr_dwt_cnt), &output);
    angle_error = g_lqr_e_theta;
    dzx_yaw_torque = output.tau_cmd_nm;
    return dzx_yaw_torque;
}

static void GimbalUpdatePitchFeedbackMessage(void)
{
    if (pitch_motor_up != NULL) {
        gimbal_feedback_data.pitch_ecd = (uint16_t)pitch_motor_up->measure.p_int;
        gimbal_feedback_data.pitch_total_angle = pitch_motor_up->measure.pos;
        gimbal_feedback_data.pitch_up_motor_pos_rad = pitch_motor_up->measure.pos;
        gimbal_feedback_data.pitch_up_motor_vel_rad_s = pitch_motor_up->measure.vel;
    } else {
        gimbal_feedback_data.pitch_ecd = 0U;
        gimbal_feedback_data.pitch_total_angle = 0.0f;
        gimbal_feedback_data.pitch_up_motor_pos_rad = 0.0f;
        gimbal_feedback_data.pitch_up_motor_vel_rad_s = 0.0f;
    }
    if (pitch_motor_down != NULL) {
        gimbal_feedback_data.pitch_down_motor_pos_rad = pitch_motor_down->measure.pos;
        gimbal_feedback_data.pitch_down_motor_vel_rad_s = pitch_motor_down->measure.vel;
    } else {
        gimbal_feedback_data.pitch_down_motor_pos_rad = 0.0f;
        gimbal_feedback_data.pitch_down_motor_vel_rad_s = 0.0f;
    }

    gimbal_feedback_data.pitch_up_imu_rad =
        (gimbal_IMU_data == NULL) ? 0.0f :
        gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
}

static void GimbalYawUpdateFeedback(void)
{
    gimbal_feedback_data.yaw_angle_pidout             = dzx_yaw_torque;
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = GimbalDMYawSingleRoundDeg();
    gimbal_feedback_data.yaw_ecd                      = (yaw_motor == NULL) ? 0U : (uint16_t)yaw_motor->measure.p_int;
    gimbal_feedback_data.yaw_total_angle              = GimbalDMYawTotalDeg();
    GimbalUpdatePitchFeedbackMessage();
}

static void GimbalYawControlStep(void)
{
    float yaw_ref = gimbal_cmd_recv.yaw;
    static volatile uint8_t codex_flag=0;
    static volatile float yaw_ref_codex=0;
    if(codex_flag)yaw_ref=yaw_ref_codex;
    GimbalUpdateYawIMUFeedback();
    switch (gimbal_cmd_recv.gimbal_mode) {
    case GIMBAL_ZERO_FORCE:
        DMMotorSetTorque(yaw_motor, 0.0f);
        DMMotorStop(yaw_motor);
        YawLQRESO_Reset(&dzx_yaw_lqr_eso);
        DWT_GetDeltaT(&dzx_yaw_lqr_dwt_cnt);
        dzx_yaw_torque        = 0.0f;
        angle_error           = 0.0f;
        break;
    case GIMBAL_GYRO_MODE:
        DMMotorEnable1(yaw_motor);
        // static float yaw_torque_test = 0.0f;
        // DMMotorSetTorque(yaw_motor, yaw_torque_test);
        DMMotorSetTorque(yaw_motor, GimbalYawCalculateTorque(yaw_ref));
        break;
    case GIMBAL_HOLE_MODE:
        /*
         * 狗洞模式不锁存 yaw，也不把 yaw 目标改成 -67deg。
         * yaw 电机继续跟随 yaw_control，-67deg 由底盘 offset 闭环完成。
         */
        DMMotorEnable1(yaw_motor);
        DMMotorSetTorque(yaw_motor, GimbalYawCalculateTorque(yaw_ref));
        break;
    case GIMBAL_MOTOR_MODE:
        DMMotorEnable1(yaw_motor);
        DMMotorSetTorque(yaw_motor, GimbalYawCalculateTorque(gimbal_cmd_recv.yaw));
        break;
    default:
        DMMotorSetTorque(yaw_motor, 0.0f);
        dzx_yaw_torque = 0.0f;
        angle_error    = 0.0f;
        break;
    }

    GimbalYawUpdateFeedback();
}

#if defined(GIMBAL_BOARD)
static void PitchUpResolveImuLimits(float down_pos_rad,
                                    volatile float *lower_limit_rad,
                                    volatile float *upper_limit_rad)
{
    float upper_limit = PitchUpEstimateImuRad(PITCH_UP_MOTOR_RAW_LOW_LIMIT_RAD,
                                              down_pos_rad);

    if (upper_limit < PITCH_IMU_CMD_HARD_LOWER_LIMIT_RAD) {
        upper_limit = PITCH_IMU_CMD_HARD_LOWER_LIMIT_RAD;
    }

    *lower_limit_rad = PITCH_IMU_CMD_HARD_LOWER_LIMIT_RAD;
    *upper_limit_rad = upper_limit;
}

static void PitchUpGetImuLimits(volatile float *lower_limit_rad,
                                volatile float *upper_limit_rad)
{
    float down_pos_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;

    if (pitch_motor_down != NULL && DMMotorIsOnline(pitch_motor_down)) {
        down_pos_rad = pitch_motor_down->measure.pos;
    }

    PitchUpResolveImuLimits(down_pos_rad, lower_limit_rad, upper_limit_rad);
}

void pitch_limit(float angle)
{
    float lower_limit_rad;
    float upper_limit_rad;

    PitchUpGetImuLimits(&lower_limit_rad, &upper_limit_rad);
    pitch_ref_limited = PitchDownClampRange(angle,
                                            lower_limit_rad,
                                            upper_limit_rad);
}

static float PitchUpSelectRef(void)
{
    float ref_limited;

    pitch_up_ref_raw_rad =
        (pitch_up_ref_override_active != 0U) ? pitch_up_ref_override_rad : gimbal_cmd_recv.pitch;
    pitch_up_ref_limit_active = 0U;
    PitchUpGetImuLimits(&pitch_up_imu_lower_limit_rad,
                        &pitch_up_imu_upper_limit_rad);

    ref_limited = PitchDownClampRange(pitch_up_ref_raw_rad,
                                      pitch_up_imu_lower_limit_rad,
                                      pitch_up_imu_upper_limit_rad);
    if (ref_limited != pitch_up_ref_raw_rad) {
        pitch_up_ref_limit_active = 1U;
    }

    pitch_up_ref_limited_rad = ref_limited;
    pitch_ref_limited = pitch_up_ref_limited_rad;
    return pitch_up_ref_limited_rad;
}

#if !PITCH_UP_ESO_TRIAL_ENABLE
static float PitchUpCalculateGravityTorque(float pitch_angle_rad)
{
    const float gravity_torque =
        PITCH_UP_GRAVITY_BIAS_NM +
        PITCH_UP_GRAVITY_SIN_GAIN_NM * sinf(pitch_angle_rad);

    return PitchDownClampRange(gravity_torque,
                               -PITCH_UP_GRAVITY_COMP_LIMIT_NM,
                               PITCH_UP_GRAVITY_COMP_LIMIT_NM);
}
#endif

static float PitchUpApplySoftLimit(float imu_pitch_rad, float torque_cmd)
{
    pitch_up_soft_limit_active = 0U;
    if (imu_pitch_rad >= pitch_up_imu_upper_limit_rad &&
        torque_cmd > 0.0f) {
        pitch_up_soft_limit_active = 1U;
        return 0.0f;
    }
    if (imu_pitch_rad <= pitch_up_imu_lower_limit_rad &&
        torque_cmd < 0.0f) {
        pitch_up_soft_limit_active = 1U;
        return 0.0f;
    }
    return torque_cmd;
}

static uint8_t PitchUpTorqueOutputAllowed(void)
{
    if (gimbal_cmd_recv.hole_yaw_ready != 0U) {
        return 1U;
    }

    if (pitch_motor_down == NULL || !DMMotorIsOnline(pitch_motor_down)) {
        return 0U;
    }

    return (fabsf(pitch_motor_down->measure.pos - PITCH_DOWN_DEFAULT_REF_POS_RAD) <=
            PITCH_DOWN_FOLLOW_MAX_OFFSET_RAD) ? 1U : 0U;
}

static void PitchUpResetTelemetry(void)
{
    pitch_up_ref_raw_rad          = PITCH_UP_IMU_ZERO_REF_RAD;
    pitch_up_ref_rad              = 0.0f;
    pitch_up_ref_limited_rad      = PITCH_UP_IMU_ZERO_REF_RAD;
    pitch_up_imu_lower_limit_rad  = 0.0f;
    pitch_up_imu_upper_limit_rad  = 0.0f;
    pitch_up_measure_rad          = 0.0f;
    pitch_up_gyro_rad_s           = 0.0f;
    pitch_up_motor_pos_rad        = 0.0f;
    pitch_up_motor_vel_rad_s      = 0.0f;
    pitch_up_lqr_torque_nm        = 0.0f;
    pitch_up_gravity_torque_ff_nm = 0.0f;
    pitch_up_torque_cmd_nm        = 0.0f;
    pitch_up_soft_limit_active    = 0U;
    pitch_up_ref_limit_active     = 0U;
}

static void PitchUpStopControl(void)
{
    PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
    PitchUpResetTelemetry();
    if (pitch_motor_up != NULL) {
        pitch_up_motor_pos_rad = pitch_motor_up->measure.pos;
        pitch_up_motor_vel_rad_s = pitch_motor_up->measure.vel;
    }
}

static float PitchUpCalculateTorque(uint8_t bypass_soft_limit)
{
    float torque_lqreso;
    float torque_cmd;
    float torque_motor_nm;
    float feedforward_torque_nm;

    if (gimbal_IMU_data == NULL || pitch_motor_up == NULL ||
        !DMMotorIsOnline(pitch_motor_up)) {
        PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
        pitch_up_ref_rad = 0.0f;
        pitch_up_torque_cmd_nm = 0.0f;
        pitch_up_lqr_torque_nm = 0.0f;
        pitch_up_gravity_torque_ff_nm = 0.0f;
        pitch_up_soft_limit_active = 0U;
        pitch_up_ref_raw_rad = PITCH_UP_IMU_ZERO_REF_RAD;
        pitch_up_ref_limited_rad = PITCH_UP_IMU_ZERO_REF_RAD;
        pitch_up_imu_lower_limit_rad = 0.0f;
        pitch_up_imu_upper_limit_rad = 0.0f;
        pitch_up_ref_limit_active = 0U;
        return 0.0f;
    }

    pitch_up_ref_rad = PitchUpSelectRef();
    pitch_up_measure_rad =
        gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
    pitch_up_gyro_rad_s =
        gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
    pitch_up_motor_pos_rad = pitch_motor_up->measure.pos;
    pitch_up_motor_vel_rad_s = pitch_motor_up->measure.vel;
    if (PitchUpTorqueOutputAllowed() == 0U) {
        PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
        DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
        pitch_up_lqr_torque_nm = 0.0f;
        pitch_up_gravity_torque_ff_nm = 0.0f;
        pitch_up_torque_cmd_nm = 0.0f;
        pitch_up_soft_limit_active = 0U;
        return 0.0f;
    }
#if PITCH_UP_ESO_TRIAL_ENABLE
    pitch_up_gravity_torque_ff_nm = 0.0f;
    feedforward_torque_nm = 0.0f;
#else
    pitch_up_gravity_torque_ff_nm =
        PitchUpCalculateGravityTorque(pitch_up_measure_rad);
    feedforward_torque_nm = pitch_up_gravity_torque_ff_nm;
#endif

    torque_lqreso = PitchLQRESO_Calc(&dzx_pitch_lqr_eso,
                                     pitch_up_ref_rad,
                                     pitch_up_measure_rad,
                                     pitch_up_gyro_rad_s,
                                     feedforward_torque_nm,
                                     DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt));
    pitch_up_lqr_torque_nm = torque_lqreso - feedforward_torque_nm;
    if (bypass_soft_limit != 0U) {
        pitch_up_soft_limit_active = 0U;
        torque_cmd = torque_lqreso;
    } else {
        torque_cmd = PitchUpApplySoftLimit(pitch_up_measure_rad, torque_lqreso);
        if (pitch_up_soft_limit_active != 0U) {
            PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
            DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
            pitch_up_lqr_torque_nm = 0.0f;
        }
    }
    /* Soft-limit logic above uses REP-103 pitch torque; only the motor output is sign-converted. */
    torque_motor_nm = PITCH_UP_CONTROL_TO_MOTOR_TORQUE_SIGN * torque_cmd;
    pitch_up_torque_cmd_nm = torque_motor_nm;
    return torque_motor_nm;
}

static uint8_t GimbalHoleAngleReady(float measure_rad, float target_rad)
{
    return (fabsf(measure_rad - target_rad) <= HOLE_READY_THRESHOLD_RAD) ? 1U : 0U;
}

static void GimbalHoleResetState(void)
{
    hole_down_ready = 0U;
    hole_down_pre_ready = 0U;
    hole_up_ready = 0U;
    hole_up_torque_zero_latched = 0U;
    pitch_down_follow_offset_rad = 0.0f;
    PitchDownClearRefOverride();
    PitchUpClearRefOverride();
}

static void GimbalHoleApplyPitchControl(void)
{
    float up_torque;
    float down_target;
    float down_mit_kp;
    float down_mit_kd;
    uint8_t bypass_down_limit;

    DMMotorEnable1(pitch_motor_down);
    static float v_des=0.0f;
    
    /*
     * 狗洞模式的动作顺序由底盘上传的 hole_yaw_ready 决定。
     * yaw 未到 -67deg 前，down 回默认安全位置并重置二段状态；
     * yaw 到位后先到 HOLE_DOWN_PRE_TARGET_RAD，再进入最终 -0.05rad。
     * down 根据当前位置在 LQRESO/MIT 间切换，低位 MIT 仍使用本段可调 PD。
     */
    if (gimbal_cmd_recv.hole_yaw_ready == 0U) {
        hole_down_pre_ready = 0U;
        down_target = HOLE_DOWN_RESTORE_TARGET_RAD;
        down_mit_kp = hole_down_enter_pre_mit_kp;
        down_mit_kd = hole_down_enter_pre_mit_kd;
        v_des=0.0f;
    } else if (hole_down_pre_ready == 0U) {
        down_target = HOLE_DOWN_PRE_TARGET_RAD;
        down_mit_kp = hole_down_enter_pre_mit_kp;
        down_mit_kd = hole_down_enter_pre_mit_kd;
        v_des=0.0f;
    } else {
        down_target = HOLE_DOWN_MIT_TARGET_RAD;
        down_mit_kp = 0;
        down_mit_kd = 1;
        v_des=-0.1f;
    }
    bypass_down_limit = (gimbal_cmd_recv.hole_yaw_ready != 0U) ? 1U : 0U;
    PitchDownSetRefOverride(down_target, bypass_down_limit);
    PitchDownApplyHybridControlWithGain(down_mit_kp, down_mit_kd, v_des);
    if (gimbal_cmd_recv.hole_yaw_ready != 0U &&
        hole_down_pre_ready == 0U &&
        GimbalHoleAngleReady(pitch_down_pos_rad, HOLE_DOWN_PRE_TARGET_RAD) != 0U) {
        hole_down_pre_ready = 1U;
    }

    /*
     * up 轴先正常闭环到 0rad；三轴全部到位后锁存 0 扭矩。
     * 锁存的目的是避免到位后 up 继续顶着目标输出力矩，退出狗洞时再恢复正常闭环。
     */
    PitchUpSetRefOverride(HOLE_UP_IMU_TARGET_RAD);
    up_torque = PitchUpCalculateTorque(1U);

    hole_down_ready = (gimbal_cmd_recv.hole_yaw_ready != 0U &&
                       hole_down_pre_ready != 0U) ?
                      GimbalHoleAngleReady(pitch_down_pos_rad, HOLE_DOWN_MIT_TARGET_RAD) :
                      0U;
    hole_up_ready = GimbalHoleAngleReady(pitch_up_measure_rad, HOLE_UP_IMU_TARGET_RAD);
    if (gimbal_cmd_recv.hole_yaw_ready != 0U &&
        hole_down_ready != 0U &&
        hole_up_ready != 0U) {
        hole_up_torque_zero_latched = 1U;
    }

    if (hole_up_torque_zero_latched != 0U) {
        up_torque = 0.0f;
        pitch_up_torque_cmd_nm = 0.0f;
    }
    DMMotorSetTorque(pitch_motor_up, up_torque);
}

static void GimbalHoleApplyRestoreControl(void)
{
    float down_restore_pos_rad;

    DMMotorEnable1(pitch_motor_down);

    down_restore_pos_rad = (pitch_motor_down == NULL) ?
                           pitch_down_pos_rad :
                           pitch_motor_down->measure.pos;
    hole_down_ready = (pitch_motor_down == NULL) ?
                      0U :
                      GimbalHoleAngleReady(down_restore_pos_rad,
                                           HOLE_DOWN_RESTORE_TARGET_RAD);

    /*
     * 退出狗洞后，down 必须先回到默认 1.95rad。
     * down 低于 1.95-10deg 时先用 MIT 拉回，到阈值后切 LQRESO 收敛；
     * 没进 3deg 误差前 up 保持 0 扭矩，避免姿态未收回时上轴抢控制；
     * 进误差后 down 继续 LQRESO 默认位，up 恢复普通 pitch 闭环。
     */
    hole_up_torque_zero_latched = 0U;
    hole_down_pre_ready = 0U;
    pitch_down_follow_offset_rad = 0.0f;
    PitchDownSetRefOverride(HOLE_DOWN_RESTORE_TARGET_RAD, 0U);
    PitchDownApplyHybridControlWithGain(hole_down_enter_pre_mit_kp,
                                        hole_down_enter_pre_mit_kd,
                                        0.0f);
    if (hole_down_ready != 0U) {
        PitchDownClearRefOverride();
        DMMotorEnable1(pitch_motor_up);
        PitchUpClearRefOverride();
        DMMotorSetTorque(pitch_motor_up, PitchUpCalculateTorque(0U));
    } else {
        pitch_up_torque_cmd_nm = 0.0f;
        DMMotorSetTorque(pitch_motor_up, 0.0f);
    }
}
#endif
// static PIDInstance YAW_version_PID = {
//     .Kp            = 1,   // 25,//25, // 50,//70, // 4.5
//     .Ki            = 0,    // 0
//     .Kd            = 0, // 0.0,  // 0.07,  // 0
//     .DeadBand      = 0,  // 0.75,  // 跟随模式设置死区，防止抖动
//     .IntegralLimit = 3000,
//     // .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//     .MaxOut        = 30,

// };
// static PIDInstance PITCH_version_PID = {
//     .Kp            = 1,   // 25,//25, // 50,//70, // 4.5
//     .Ki            = 0,    // 0
//     .Kd            = 0, // 0.0,  // 0.07,  // 0
//     .DeadBand      = 0,  // 0.75,  // 跟随模式设置死区，防止抖动
//     .IntegralLimit = 30,
//     .Improve       = PID_OutputFilter|PID_Derivative_On_Measurement|PID_Integral_Limit|PID_Trapezoid_Intergral/*PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement*/,
//     .MaxOut        = 30,
//     .Output_LPF_RC=0.7,
// };
float nuc_version_control[2];
// float filiter(float input){
//     static float alpha=1,input_last;
//     float output=alpha*input+(1-alpha)*input_last;
//     input_last=output;
//     return output;
// }
float yaw_feedforward_angleErrmax=10,yaw_feedforward_max=3200; // 根据角度误差限制前馈上限
/* 云台主任务：读取控制输入，结合IMU反馈并输出电机控制 */
// float fpv_pitch_test=0 , telescope_test=0;
uint16_t pitch_test=0;
uint8_t fpv_state=0;//0:rise 1:down 2:ready
uint16_t fpv_count=0;
float fpv_pitch_up=0,fpv_pitch_down=0,telescope_up=0,telescope_down=0;
extern uint8_t telescope_pos ;//0:normal 1:zoom
extern uint8_t fpv_pos ;//0:normal 1:lob
extern float pitch_vel;
void GimbalTask()
{
    // 获取云台控制消息
    // 执行模式切换与前馈控制
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // yaw current feedforward is now produced by the application controller.
    // if(yaw_current_feedforward>yaw_feedforward_max)yaw_current_feedforward=yaw_feedforward_max;
    // else if(yaw_current_feedforward<-yaw_feedforward_max)yaw_current_feedforward=-yaw_feedforward_max;

    //------------------------------------------------------------------------
    // gimbal_cmd_recv.pitch_version=filiter(gimbal_cmd_recv.pitch_version);
    // gimbal_cmd_recv.yaw_version=filiter(gimbal_cmd_recv.yaw_version);
    // if(gimbal_cmd_recv.nuc_mode){
    //     nuc_version_control[0]=PIDCalculate(&PITCH_version_PID,gimbal_cmd_recv.pitch_version,0);
    //     nuc_version_control[1]=PIDCalculate(&YAW_version_PID,gimbal_cmd_recv.yaw_version,0);
    // }
    // else memset(nuc_version_control,0,sizeof(nuc_version_control));
    // @todo: 双板链路下统一角度/角速度符号，进一步校准IMU与yaw偏置关系
    // 按 robot_cmd 下发的数据更新 yaw_ref 与 pitch_ref
    #if defined(CHASSIS_BOARD)
    GimbalYawControlStep();
    #endif
    #if defined(GIMBAL_BOARD)
    #ifndef BIG_HEAD
    // pitch DM output is provided directly by the application controller.
    // pitch_tor_feedforward = 0.42146 * cos(1.43 + gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET]);
    // pitch_tor_feedforward =
    // pitch_current_feedforward = PitchNonlinear(*pitch_motor->motor_controller.other_angle_feedback_ptr);

    pitch_gyro_measure = gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
    switch (gimbal_cmd_recv.gimbal_mode) {
        // 零力模式
        case GIMBAL_ZERO_FORCE:
            //DMMotorSetTorque(pitch_motor, 0.0f);
            //DMMotorStop(pitch_motor);
            PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
            DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
            pitch_speed_feedforward = 0;
            dzx_pitch_torque        = 0.0f;
            break;
        // 陀螺仪模式：角度闭环，使用姿态反馈与偏置补偿
        case GIMBAL_GYRO_MODE: // 姿态控制模式
            //DMMotorEnable1(pitch_motor);

            if (gimbal_cmd_recv.nuc_mode == version_control)
            {
                pitch_limit(gimbal_cmd_recv.pitch_version);
                pitch_speed_feedforward = pitch_vel;
            }
            else
            {
                pitch_limit(gimbal_cmd_recv.pitch);
                pitch_speed_feedforward = 0;
            }
            dzx_pitch_torque = PitchLQRESO_Calc(&dzx_pitch_lqr_eso,
                                                pitch_ref_limited,
                                                gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],
                                                pitch_gyro_measure,
                                                pitch_speed_feedforward,
                                                DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt));
            //DMMotorSetTorque(pitch_motor, dzx_pitch_torque);
            break;
        case GIMBAL_MOTOR_MODE:
            //DMMotorEnable1(pitch_motor);

            pitch_limit(pitch_offset + gimbal_cmd_recv.pitch);
            pitch_speed_feedforward = 0.0f;
            dzx_pitch_torque = PitchLQRESO_Calc(&dzx_pitch_lqr_eso,
                                                pitch_ref_limited,
                                                gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],
                                                pitch_gyro_measure,
                                                pitch_speed_feedforward,
                                                DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt));
                //DMMotorSetTorque(pitch_motor, dzx_pitch_torque);
        break;
        default:
            break;
    }
    // pitch_motor->motor_controller.other_speed_feedback_ptr = &pitch_speed_feedforward;
    #endif // !BIG_HEAD
    #ifdef BIG_HEAD
    pitch_current_feedforward=-exp((352-gimbal_IMU_data->output.INS_angle_deg[INS_PITCH_ADDRESS_OFFSET])/40);
    // pitch current feedforward is kept for the legacy big-head branch.
    if(pitch_current_feedforward>-3500)pitch_current_feedforward=-3500;
    else if(pitch_current_feedforward<-6500)pitch_current_feedforward=-6500;
        switch (gimbal_cmd_recv.gimbal_mode) {

        // 零力模式
        case GIMBAL_ZERO_FORCE:
            DMMotorSetTorque(pitch_motor, 0.0f);
            DMMotorStop(pitch_motor);
            PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
            DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
            dzx_pitch_torque = 0.0f;
            break;
        // 陀螺仪模式：角度闭环，使用姿态反馈与偏置补偿
        case GIMBAL_GYRO_MODE: // 姿态控制模式
            DMMotorEnable1(pitch_motor);
            pitch_limit(gimbal_cmd_recv.pitch);
            dzx_pitch_torque = PitchLQRESO_Calc(&dzx_pitch_lqr_eso,
                                                pitch_ref_limited,
                                                gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],
                                                pitch_gyro_measure,
                                                pitch_current_feedforward,
                                                DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt));
            DMMotorSetTorque(pitch_motor, dzx_pitch_torque);
            break;
        case GIMBAL_MOTOR_MODE:
            DMMotorEnable1(pitch_motor);
            pitch_limit(gimbal_cmd_recv.pitch);
            dzx_pitch_torque = PitchLQRESO_Calc(&dzx_pitch_lqr_eso,
                                                pitch_ref_limited,
                                                gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],
                                                pitch_gyro_measure,
                                                pitch_current_feedforward,
                                                DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt));
            DMMotorSetTorque(pitch_motor, dzx_pitch_torque);
        break;
        default:
            break;
    }
    #endif
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = GimbalDMYawSingleRoundDeg();
    // gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.p_int;
    // gimbal_feedback_data.pitch_ecd                    = pitch_motor->measure.ecd;
    // gimbal_feedback_data.yaw_total_angle              = GimbalDMYawTotalDeg();
    // gimbal_feedback_data.pitch_total_angle            = pitch_motor->measure.total_angle;

    #endif
    // 如需对 pitch 做额外补偿，可在此处统一处理
    // 建议使用 IMU 角度/角速度作为闭环反馈
    // ...

    // 以下为备用反馈打包方案（imu、yaw、ecd）
    // gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = GimbalDMYawSingleRoundDeg();
    // gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.p_int;
    // gimbal_feedback_data.pitch_ecd                    = pitch_motor->measure.ecd;
    // gimbal_feedback_data.yaw_total_angle              = GimbalDMYawTotalDeg();
    // gimbal_feedback_data.pitch_total_angle            = pitch_motor->measure.total_angle;
    // 发布反馈消息
    GimbalUpdatePitchFeedbackMessage();
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
void GimbalTask_YAW(){
    // 获取云台控制消息
    // 执行模式切换与前馈控制
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    GimbalYawControlStep();
    GimbalYawUpdateFitTelemetry();
    
    // 发布反馈消息
    gimbal_feedback_data.yaw_motor_single_round_angle = GimbalDMYawSingleRoundDeg();
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_vel                = (yaw_motor == NULL) ? 0.0f : yaw_motor->measure.vel;
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
#if defined(GIMBAL_BOARD)
void GimbalTask_PITCH()
{
    // 获取云台控制消息
    // 执行模式切换与前馈控制
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    static volatile gimbal_mode_e test_hole;
    if(test_hole==GIMBAL_HOLE_MODE&&gimbal_cmd_recv.gimbal_mode!=GIMBAL_ZERO_FORCE){
        gimbal_cmd_recv.gimbal_mode=test_hole;
        gimbal_cmd_recv.hole_yaw_ready=1;
    }
    
    if (gimbal_cmd_recv.hole_restore_active != 0U &&
        gimbal_cmd_recv.gimbal_mode != GIMBAL_HOLE_MODE) {
        GimbalHoleApplyRestoreControl();
        PitchDownUpdateDebugCsv();
        gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
        GimbalUpdatePitchFeedbackMessage();
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    switch (gimbal_cmd_recv.gimbal_mode) {
    case GIMBAL_ZERO_FORCE:
        DMMotorSetTorque(pitch_motor_up, 0.0f);
        DMMotorStop(pitch_motor_up);
        DMMotorSetTorque(pitch_motor_down, 0.0f);
        DMMotorStop(pitch_motor_down);
        GimbalHoleResetState();
        PitchUpStopControl();
        PitchDownStopControl();
        break;
    case GIMBAL_GYRO_MODE:
    case GIMBAL_MOTOR_MODE:
        GimbalHoleResetState();
        DMMotorEnable1(pitch_motor_down);
        DMMotorEnable1(pitch_motor_up);
        DMMotorSetTorque(pitch_motor_up, PitchUpCalculateTorque(0U));
        PitchDownApplyNormalFollowRef();
        PitchDownApplyHybridControlWithGain(hole_down_enter_pre_mit_kp,
                                            hole_down_enter_pre_mit_kd,
                                            0.0f);
        break;
    case GIMBAL_HOLE_MODE:
        GimbalHoleApplyPitchControl();
        break;
    default:
        DMMotorSetTorque(pitch_motor_up, 0.0f);
        DMMotorStop(pitch_motor_up);
        DMMotorSetTorque(pitch_motor_down, 0.0f);
        DMMotorStop(pitch_motor_down);
        GimbalHoleResetState();
        PitchUpStopControl();
        PitchDownStopControl();
        break;
    }

    PitchDownUpdateDebugCsv();
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    GimbalUpdatePitchFeedbackMessage();
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
#endif
