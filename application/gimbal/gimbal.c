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
#include "double_pitch_lqr.h"
#include "lqr_eso.h"
#define DZX_YAW_TORQUE_LIMIT   7.0f
#define DZX_YAW_RAD_TO_DEG     57.2957795f
#if defined(GIMBAL_BOARD)
#define PITCH_DOWN_DEFAULT_REF_POS_RAD      PITCH_DOWN_CMD_DEFAULT_REF_RAD
#define PITCH_DOWN_LQR_K_THETA              141.421356f
#define PITCH_DOWN_LQR_K_OMEGA              11.198687f
#define PITCH_DOWN_AUTO_REF_CENTER_RAD      1.95f
#define PITCH_DOWN_AUTO_REF_RANGE_RAD       0.5f
#define PITCH_DOWN_AUTO_REF_LPF_RC_S        0.20f
#define PITCH_DOWN_AUTO_REF_VISION_SIGN     (0.0f)
#define PITCH_DOWN_AUTO_LQR_K_THETA         PITCH_DOWN_LQR_K_THETA
#define PITCH_DOWN_AUTO_LQR_K_OMEGA         PITCH_DOWN_LQR_K_OMEGA
#define PITCH_DOWN_LQR_TORQUE_LIMIT_NM      6.00f
#define PITCH_DOWN_TORQUE_SLEW_RATE_NM_S    700.0f
#define PITCH_DOWN_IDENT_LOWER_RAD          1.22f
#define PITCH_DOWN_IDENT_UPPER_RAD          2.20f
#define PITCH_DOWN_IDENT_LQR_UPPER_RAD      2.50f
#define PITCH_DOWN_IDENT_CMD_GAIN           1.15f
#define PITCH_DOWN_IDENT_UP_HOLD_REF_RAD    0.0f
#define PITCH_DOWN_IDENT_UP_HOLD_VALID_RAD  0.034906585f
#define PITCH_DOWN_IDENT_AUTO_G_POINT_COUNT 10U
#define PITCH_DOWN_IDENT_AUTO_FRICTION_SPEED_COUNT 3U
#define PITCH_DOWN_IDENT_AUTO_PROFILE_GRAVITY 1U
#define PITCH_DOWN_IDENT_AUTO_PROFILE_FRICTION 2U
#define PITCH_DOWN_IDENT_AUTO_PROFILE_CHIRP 3U
#define PITCH_DOWN_IDENT_AUTO_PHASE_IDLE    0U
#define PITCH_DOWN_IDENT_AUTO_PHASE_G_SETTLE 1U
#define PITCH_DOWN_IDENT_AUTO_PHASE_G_SAMPLE 2U
#define PITCH_DOWN_IDENT_AUTO_PHASE_F_HOLD  3U
#define PITCH_DOWN_IDENT_AUTO_PHASE_F_RUN   4U
#define PITCH_DOWN_IDENT_AUTO_PHASE_C_SETTLE 5U
#define PITCH_DOWN_IDENT_AUTO_PHASE_C_RUN   6U
#define PITCH_DOWN_IDENT_AUTO_PHASE_DONE    7U
#define PITCH_DOWN_IDENT_AUTO_PHASE_ABORT   8U
#define PITCH_DOWN_IDENT_AUTO_ABORT_NONE    0U
#define PITCH_DOWN_IDENT_AUTO_ABORT_CANCEL  1U
#define PITCH_DOWN_IDENT_AUTO_ABORT_MODE    2U
#define PITCH_DOWN_IDENT_AUTO_ABORT_FEEDBACK 3U
#define PITCH_DOWN_IDENT_STAGE_GRAVITY_UP   1U
#define PITCH_DOWN_IDENT_STAGE_GRAVITY_DOWN 2U
#define PITCH_DOWN_IDENT_STAGE_F_POS_004    3U
#define PITCH_DOWN_IDENT_STAGE_F_NEG_004    4U
#define PITCH_DOWN_IDENT_STAGE_F_POS_006    5U
#define PITCH_DOWN_IDENT_STAGE_F_NEG_006    6U
#define PITCH_DOWN_IDENT_STAGE_F_POS_009    7U
#define PITCH_DOWN_IDENT_STAGE_F_NEG_009    8U
#define PITCH_DOWN_IDENT_STAGE_CHIRP_LOW    9U
#define PITCH_DOWN_IDENT_STAGE_CHIRP_HIGH   10U
#define PITCH_DOWN_IDENT_STAGE_DEFAULT      PITCH_DOWN_IDENT_STAGE_GRAVITY_UP
#define PITCH_DOWN_IDENT_AUTO_G_SETTLE_TIME_S 1.2f
#define PITCH_DOWN_IDENT_AUTO_G_SAMPLE_TIME_S 1.0f
#define PITCH_DOWN_IDENT_AUTO_F_HOLD_TIME_S 0.4f
#define PITCH_DOWN_IDENT_AUTO_F_PREP_MAX_TIME_S 3.0f
#define PITCH_DOWN_IDENT_AUTO_F_LOW_RAD     1.35f
#define PITCH_DOWN_IDENT_AUTO_F_HIGH_RAD    2.10f
#define PITCH_DOWN_IDENT_AUTO_CHIRP_CENTER_RAD 1.70f
#define PITCH_DOWN_IDENT_AUTO_CHIRP_AMP_RAD 0.14f
#define PITCH_DOWN_IDENT_AUTO_CHIRP_F_START_HZ 0.15f
#define PITCH_DOWN_IDENT_AUTO_CHIRP_F_END_HZ 0.70f
#define PITCH_DOWN_IDENT_STAGE_CHIRP_DURATION_S 15.0f
#define PITCH_DOWN_IDENT_AUTO_CHIRP_SETTLE_S 1.0f
#define PITCH_DOWN_HOLE_TORQUE_LIMIT_NM     2.00f
#define PITCH_DOWN_HOLE_TORQUE_SLEW_RATE_NM_S 700.0f
#define HOLE_DOWN_RESTORE_TORQUE_NM         7.0f
#define HOLE_DOWN_RESTORE_LQR_TAKEOVER_RAD  (HOLE_DOWN_RESTORE_TARGET_RAD - 0.34906585f)
#define HOLE_UP_READY_THRESHOLD_RAD         0.025f
#define HOLE_UP_READY_STABLE_TIME_S         0.15f
#define HOLE_DOWN_PRE_RAMP_RATE_RAD_S       24.0f
#define PITCH_DOWN_VEL_LIMIT_RAD_S          1.5f
#define PITCH_DOWN_MODEL_CONTROL_ENABLE     1U
#define PITCH_DOWN_MODEL_G0_NM              1.82780f
#define PITCH_DOWN_MODEL_GS_NM              (-0.58556f)
#define PITCH_DOWN_MODEL_GC_NM              3.66492f
#define PITCH_DOWN_MODEL_J_KG_M2            0.08984f
#define PITCH_DOWN_MODEL_B_NMS_RAD          0.0f
#define PITCH_DOWN_MODEL_TAU_C_NM           1.18f
#define PITCH_DOWN_MODEL_OMEGA_EPS_RAD_S    0.05f
#define PITCH_DOWN_MODEL_REF_VEL_LIMIT_RAD_S 0.8f
#define PITCH_DOWN_MODEL_REF_ACC_LIMIT_RAD_S2 3.0f
#define PITCH_DOWN_MODEL_Q_THETA            20000.0f
#define PITCH_DOWN_MODEL_Q_OMEGA            100.0f
#define PITCH_DOWN_MODEL_R_TORQUE           1.0f
#define PITCH_DOWN_FRICTION_COMP_NM         0.25f
#define PITCH_DOWN_FRICTION_SMOOTH_VEL_RAD_S 0.10f
#define PITCH_DOWN_DEFAULT_LOWER_LIMIT_RAD  PITCH_DOWN_CMD_LOWER_LIMIT_RAD
#define PITCH_DOWN_DEFAULT_UPPER_LIMIT_RAD  PITCH_DOWN_CMD_UPPER_LIMIT_RAD
#define PITCH_DOWN_PITCH_CMD_TO_POS_GAIN    0.9935044f
#define PITCH_DOWN_MIT_SWITCH_POS_RAD       PITCH_DOWN_LQRESO_SWITCH_POS_RAD
#define PITCH_DOWN_DIRECT_RECOVER_SWITCH_POS_RAD 1.15f
#define PITCH_DOWN_DIRECT_RECOVER_TORQUE_NM 7.0f
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
#define PITCH_DOWN_MIT_GRAVITY_FADE_START_RAD 1.70f
#define PITCH_DOWN_MIT_GRAVITY_FADE_END_RAD 1.80f
#define PITCH_DOWN_MIT_GRAVITY_COMP_LIMIT_NM 3.50f
#define PITCH_DOWN_MIT_GRAVITY_SIN_COEFF_NM (-1.6518525f)
#define PITCH_DOWN_MIT_GRAVITY_COS_COEFF_NM 1.3151889f
#define PITCH_DOWN_MIT_GRAVITY_BIAS_NM      2.6112738f
#define PITCH_DOWN_ESO_ENABLE               0U
#define PITCH_DOWN_ESO_ALPHA                1.0f
#define PITCH_DOWN_ESO_COMP_GAIN            1.0f
#define PITCH_DOWN_ESO_COMP_LIMIT_NM        3.0f
#define PITCH_DOWN_ESO_W0_MIN_RAD_S         0.0f
#define PITCH_DOWN_ESO_W0_MAX_RAD_S         80.0f
#define PITCH_UP_NON_HOLE_DOWN_ZERO_TORQUE_RAD 1.70f
#define PITCH_UP_TORQUE_LIMIT_NM            7.00f
#define PITCH_UP_TORQUE_SLEW_RATE_NM_S      100.0f
#define PITCH_UP_AUTO_TORQUE_SLEW_RATE_NM_S 2000.0f
#define PITCH_UP_VISION_ACC_FILTER_ALPHA_DEFAULT 0.26f
#define PITCH_UP_TRAJ_PLANNER_ENABLE_DEFAULT 1U
#define PITCH_UP_TRAJ_CHANGE_EPS_RAD        1.0e-6f
#define PITCH_UP_TRAJ_MIN_DT_S              0.0001f
#define PITCH_UP_TRAJ_MAX_DT_S              0.0200f
#define PITCH_UP_TRAJ_DEFAULT_DT_S          0.0010f
#define PITCH_UP_SOFT_LIMIT_ENABLE          0
#define PITCH_UP_CONTROL_TO_MOTOR_TORQUE_SIGN (1.0f)
#define PITCH_UP_LQR_K_THETA                14.142136f
#define PITCH_UP_LQR_K_OMEGA                1.089886f
#define PITCH_UP_AUTO_LQR_K_THETA           50.0f//44.72136f
#define PITCH_UP_AUTO_LQR_K_OMEGA           2.0f//1.175544f
#define PITCH_UP_AUTO_PID_ENABLE_DEFAULT    1U
#define PITCH_UP_AUTO_PID_ANGLE_KP_DEFAULT  20.0f
#define PITCH_UP_AUTO_PID_ANGLE_KI_DEFAULT  0.0f
#define PITCH_UP_AUTO_PID_ANGLE_KD_DEFAULT  1.0f
#define PITCH_UP_AUTO_PID_SPEED_KP_DEFAULT  1.0f
#define PITCH_UP_AUTO_PID_SPEED_KI_DEFAULT  0.2f
#define PITCH_UP_AUTO_PID_SPEED_KD_DEFAULT  0.0f
#define PITCH_UP_AUTO_PID_ANGLE_OUT_LIMIT_RAD_S (PITCH_UP_TORQUE_LIMIT_NM / PITCH_UP_AUTO_LQR_K_OMEGA)
#define PITCH_UP_AUTO_PID_TORQUE_LIMIT_NM   PITCH_UP_TORQUE_LIMIT_NM
#define PITCH_UP_GRAVITY_BIAS_NM            (-1.05f)
#define PITCH_UP_GRAVITY_SIN_GAIN_NM        0.25f
#define PITCH_UP_GRAVITY_COMP_LIMIT_NM      1.20f
#define PITCH_UP_ESO_TRIAL_ENABLE           1U
#define PITCH_UP_MODEL_LQR_ENABLE           1U
#define PITCH_UP_MODEL_J_KG_M2              0.011526f
#define PITCH_UP_MODEL_B_NMS_RAD            0.063380f
#define PITCH_UP_MODEL_GS_Q_NM              0.373492f
#define PITCH_UP_MODEL_GC_Q_NM              (-0.929636f)
#define PITCH_UP_MODEL_TAU_C_NM             0.333765f
#define PITCH_UP_MODEL_OMEGA_EPS_RAD_S      0.0500f
#define PITCH_UP_ESO_ENABLE                 1U
#define PITCH_UP_ESO_ALPHA                  3.0f
#define PITCH_UP_ESO_COMP_GAIN              1.0f
#define PITCH_UP_ESO_COMP_LIMIT_NM          1.2f
#define PITCH_UP_ESO_W0_SET_RAD_S           0.0f
#define PITCH_UP_IMU_ZERO_REF_RAD           0.0f
#define PITCH_UP_IMU_COUPLE_UP_COEF         0.9779975f
#define PITCH_UP_IMU_COUPLE_DOWN_COEF       0.9808414f
#define PITCH_UP_IMU_COUPLE_OFFSET_RAD      (-1.6046749f)
#define PITCH_UP_REF_SOURCE_NONE            0U
#define PITCH_UP_REF_SOURCE_CMD             1U
#define PITCH_UP_REF_SOURCE_VISION          2U
#define PITCH_UP_REF_SOURCE_OVERRIDE        3U
#define PITCH_UP_G_AUTO_POINT_COUNT         9U
#define PITCH_UP_G_AUTO_DEG_TO_RAD          0.017453292519943295f
#define PITCH_UP_G_AUTO_PHASE_IDLE          0U
#define PITCH_UP_G_AUTO_PHASE_SETTLE        2U
#define PITCH_UP_G_AUTO_PHASE_SAMPLE        3U
#define PITCH_UP_G_AUTO_PHASE_DONE          4U
#define PITCH_UP_G_AUTO_PHASE_ABORT         5U
#define PITCH_UP_G_AUTO_SETTLE_TIME_S       1.0f
#define PITCH_UP_G_AUTO_SAMPLE_TIME_S       1.0f
#define PITCH_UP_DYN_AUTO_PHASE_IDLE        0U
#define PITCH_UP_DYN_AUTO_PHASE_RAMP        1U
#define PITCH_UP_DYN_AUTO_PHASE_SETTLE      2U
#define PITCH_UP_DYN_AUTO_PHASE_RUN         3U
#define PITCH_UP_DYN_AUTO_PHASE_DONE        4U
#define PITCH_UP_DYN_AUTO_PHASE_ABORT       5U
#define PITCH_UP_DYN_PROFILE_LOW_SPEED      1U
#define PITCH_UP_DYN_PROFILE_CHIRP          2U
#define PITCH_UP_DYN_PROFILE_FRICTION       3U
#define PITCH_UP_DYN_CENTER_RAD             (-0.10f)
#define PITCH_UP_DYN_LOW_AMP_RAD            0.12f
#define PITCH_UP_DYN_LOW_FREQ_HZ            0.08f
#define PITCH_UP_DYN_LOW_DURATION_S         40.0f
#define PITCH_UP_DYN_CHIRP_AMP_RAD          0.07f
#define PITCH_UP_DYN_CHIRP_F_START_HZ       0.20f
#define PITCH_UP_DYN_CHIRP_F_END_HZ         0.80f
#define PITCH_UP_DYN_CHIRP_DURATION_S       30.0f
#define PITCH_UP_DYN_FRICTION_SPEED_COUNT   3U
#define PITCH_UP_DYN_FRICTION_CYCLE_COUNT   2U
#define PITCH_UP_DYN_FRICTION_AMP_RAD       0.12f
#define PITCH_UP_DYN_FRICTION_EDGE_SETTLE_S 0.5f
#define PITCH_UP_DYN_RAMP_RATE_RAD_S        0.15f
#define PITCH_UP_DYN_READY_POS_RAD          0.020f
#define PITCH_UP_DYN_READY_SPEED_RAD_S      0.050f
#define PITCH_UP_DYN_SETTLE_TIME_S          1.0f
#define PITCH_UP_DYN_MOVE_TIMEOUT_S         8.0f
#define PITCH_UP_DYN_SETTLE_TIMEOUT_S       4.0f
#define PITCH_UP_DYN_GRAVITY_BIAS_NM        0.0f
#define PITCH_UP_DYN_GRAVITY_SIN_GAIN_NM    (-PITCH_UP_MODEL_GS_Q_NM)
#define PITCH_UP_DYN_TWO_PI                 6.283185307179586f
#define PITCH_UP_IDENT_AUTO_G_POINT_COUNT   10U
#define PITCH_UP_IDENT_AUTO_FRICTION_SPEED_COUNT 2U
#define PITCH_UP_IDENT_AUTO_PROFILE_GRAVITY 1U
#define PITCH_UP_IDENT_AUTO_PROFILE_FRICTION 2U
#define PITCH_UP_IDENT_AUTO_PROFILE_CHIRP   3U
#define PITCH_UP_IDENT_AUTO_PHASE_IDLE      0U
#define PITCH_UP_IDENT_AUTO_PHASE_G_SETTLE  1U
#define PITCH_UP_IDENT_AUTO_PHASE_G_SAMPLE  2U
#define PITCH_UP_IDENT_AUTO_PHASE_F_HOLD    3U
#define PITCH_UP_IDENT_AUTO_PHASE_F_RUN     4U
#define PITCH_UP_IDENT_AUTO_PHASE_C_SETTLE  5U
#define PITCH_UP_IDENT_AUTO_PHASE_C_RUN     6U
#define PITCH_UP_IDENT_AUTO_PHASE_DONE      7U
#define PITCH_UP_IDENT_AUTO_PHASE_ABORT     8U
#define PITCH_UP_IDENT_AUTO_ABORT_NONE      0U
#define PITCH_UP_IDENT_AUTO_ABORT_CANCEL    1U
#define PITCH_UP_IDENT_AUTO_ABORT_MODE      2U
#define PITCH_UP_IDENT_AUTO_ABORT_FEEDBACK  3U
#define PITCH_UP_IDENT_STAGE_GRAVITY_UP     1U
#define PITCH_UP_IDENT_STAGE_GRAVITY_DOWN   2U
#define PITCH_UP_IDENT_STAGE_F_006          3U
#define PITCH_UP_IDENT_STAGE_F_009          4U
#define PITCH_UP_IDENT_STAGE_CHIRP          5U
#define PITCH_UP_IDENT_STAGE_DEFAULT        PITCH_UP_IDENT_STAGE_GRAVITY_UP
#define PITCH_UP_IDENT_AUTO_G_SETTLE_TIME_S 1.2f
#define PITCH_UP_IDENT_AUTO_G_SAMPLE_TIME_S 1.0f
#define PITCH_UP_IDENT_AUTO_F_HOLD_TIME_S   0.4f
#define PITCH_UP_IDENT_AUTO_F_PREP_MAX_TIME_S 3.0f
#define PITCH_UP_IDENT_AUTO_F_LOW_RAD       (-0.45f)
#define PITCH_UP_IDENT_AUTO_F_HIGH_RAD      0.20f
#define PITCH_UP_IDENT_AUTO_CHIRP_CENTER_RAD (-0.10f)
#define PITCH_UP_IDENT_AUTO_CHIRP_AMP_RAD   0.10f
#define PITCH_UP_IDENT_AUTO_CHIRP_START_HZ  0.15f
#define PITCH_UP_IDENT_AUTO_CHIRP_END_HZ    0.70f
#define PITCH_UP_IDENT_STAGE_CHIRP_DURATION_S 22.0f
#define PITCH_UP_IDENT_AUTO_CHIRP_SETTLE_S  1.0f
#define DOUBLE_PITCH_IDENT_AXIS_THETA       1U
#define DOUBLE_PITCH_IDENT_AXIS_PHI         2U
#define DOUBLE_PITCH_IDENT_AMP_LIMIT_RAD    0.080f
#define DOUBLE_PITCH_STATIC_POINT_COUNT     25U
#define DOUBLE_PITCH_STATIC_POINT_DURATION_S 2.0f
#define DOUBLE_PITCH_STATIC_PHASE_IDLE      0U
#define DOUBLE_PITCH_STATIC_PHASE_MOVE      1U
#define DOUBLE_PITCH_STATIC_PHASE_SETTLE    2U
#define DOUBLE_PITCH_STATIC_PHASE_SAMPLE    3U
#define DOUBLE_PITCH_STATIC_PHASE_DONE      4U
#define DOUBLE_PITCH_STATIC_MOVE_RATE_RAD_S 0.06f
#define DOUBLE_PITCH_STATIC_READY_POS_RAD   0.035f
#define DOUBLE_PITCH_STATIC_READY_SPEED_RAD_S 0.08f
#define DOUBLE_PITCH_STATIC_SETTLE_TIME_S   0.8f
#define DOUBLE_PITCH_IDENT_AXIS_GRAVITY_RAMP 3U
#define DOUBLE_PITCH_GRAVITY_RAMP_PHASE_IDLE   0U
#define DOUBLE_PITCH_GRAVITY_RAMP_PHASE_RAMP   1U
#define DOUBLE_PITCH_GRAVITY_RAMP_PHASE_RETURN 2U
#define DOUBLE_PITCH_GRAVITY_RAMP_PHASE_DONE   3U
#define DOUBLE_PITCH_GRAVITY_STEP_PHASE_DOWN_RAMP       DOUBLE_PITCH_GRAVITY_RAMP_PHASE_RAMP
#define DOUBLE_PITCH_GRAVITY_STEP_PHASE_UP_RAMP_AT_HOLD DOUBLE_PITCH_GRAVITY_RAMP_PHASE_RETURN
#define DOUBLE_PITCH_GRAVITY_STEP_PHASE_FAULT           4U
#define DOUBLE_PITCH_GRAVITY_RAMP_DEFAULT_DURATION_S 25.0f
#define DOUBLE_PITCH_GRAVITY_RAMP_DEFAULT_RETURN_S   8.0f
#define DOUBLE_PITCH_GRAVITY_RAMP_DEFAULT_TORQUE_NM  7.0f
#define DOUBLE_PITCH_GRAVITY_RAMP_MAX_TORQUE_NM      7.0f
#define DOUBLE_PITCH_GRAVITY_RAMP_OMEGA_LIMIT_RAD_S  0.8f
#define DOUBLE_PITCH_GRAVITY_RAMP_PSI_MARGIN_RAD     0.05f
#define DOUBLE_PITCH_GRAVITY_STEP_TORQUE_RATE_NM_S   0.5f
#define DOUBLE_PITCH_GRAVITY_STEP_DOWN_START_TORQUE_NM 3.0f
#define DOUBLE_PITCH_GRAVITY_STEP_DEFAULT_THETA_STEP_RAD 0.3f
#define DOUBLE_PITCH_GRAVITY_STEP_DEFAULT_DOWN_STOP_RAD  1.95f
#define DOUBLE_PITCH_GRAVITY_STEP_STOP_EPS_RAD           0.01f
#define DOUBLE_PITCH_GRAVITY_STEP_SIGN_POSITIVE      1U
#define DOUBLE_PITCH_GRAVITY_STEP_SIGN_NEGATIVE      2U
#define DOUBLE_PITCH_GRAVITY_STEP_STOP_NONE          0U
#define DOUBLE_PITCH_GRAVITY_STEP_STOP_DONE          1U
#define DOUBLE_PITCH_GRAVITY_STEP_STOP_CANCELLED     2U
#define DOUBLE_PITCH_GRAVITY_STEP_STOP_MODE          3U
#define DOUBLE_PITCH_GRAVITY_STEP_STOP_BAD_SIGN      4U
#define DOUBLE_PITCH_GRAVITY_STEP_STOP_ENVELOPE      5U
#define DOUBLE_PITCH_GRAVITY_STEP_STOP_OMEGA         6U
#define DOUBLE_PITCH_GRAVITY_STEP_UP_LIMIT_NONE      0U
#define DOUBLE_PITCH_GRAVITY_STEP_UP_LIMIT_LOW       1U
#define DOUBLE_PITCH_GRAVITY_STEP_UP_LIMIT_HIGH      2U
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
static PIDInstance pitch_up_auto_angle_pid;
static PIDInstance pitch_up_auto_speed_pid;
#endif
static uint32_t dzx_yaw_lqr_dwt_cnt;
#if defined(GIMBAL_BOARD)
static uint32_t dzx_pitch_lqr_dwt_cnt;
static uint32_t dzx_pitch_down_lqr_dwt_cnt;
static uint32_t pitch_up_ident_dwt_cnt;
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

typedef struct
{
    uint32_t sample_count;
    float time_s;
    float dt_s;
    uint8_t valid;
    uint8_t gimbal_mode;
    uint8_t nuc_mode;
    uint8_t feedback_ok;
    uint8_t down_ident_mode_active;
    uint8_t auto_running;
    uint8_t auto_done;
    uint8_t auto_abort;
    uint8_t auto_abort_reason;
    uint8_t auto_phase;
    uint8_t auto_profile;
    uint8_t auto_stage;
    uint8_t auto_pass;
    uint8_t auto_point_index;
    uint8_t auto_sample_active;
    float cmd_pitch_rad;
    float down_ref_raw_rad;
    float down_ref_rad;
    float down_ref_vel_rad_s;
    float down_ref_acc_rad_s2;
    uint8_t down_ref_limit_active;
    float down_pos_rad;
    float down_vel_raw_rad_s;
    float down_vel_lpf_rad_s;
    float down_pos_error_rad;
    float down_tau_cmd_nm;
    float down_tau_lqr_nm;
    float down_tau_pre_limit_nm;
    float down_tau_output_nm;
    uint8_t down_tau_limit_active;
    float down_tau_meas_nm;
    float down_ctrl_tor_set_nm;
    uint8_t down_eso_runtime_enable;
    uint8_t down_eso_enable;
    float down_eso_alpha;
    float down_eso_w0_rad_s;
    float down_eso_comp_gain;
    float down_eso_comp_limit_nm;
    float down_tau_eso_comp_nm;
    float down_feedforward_nm;
    uint8_t down_model_enable;
    float down_model_g0_nm;
    float down_model_gs_nm;
    float down_model_gc_nm;
    float down_model_j_kg_m2;
    float down_model_b_nms_rad;
    float down_model_tau_c_param_nm;
    float down_model_omega_eps_rad_s;
    float down_model_tau_g_nm;
    float down_model_tau_j_nm;
    float down_model_tau_b_nm;
    float down_model_tau_c_nm;
    float down_model_tau_ff_nm;
    float down_tau_no_model_nm;
    float down_k_theta_used;
    float down_k_omega_used;
    float up_hold_ref_rad;
    float up_imu_rad;
    float up_gyro_rad_s;
    float up_hold_error_rad;
    float up_tau_cmd_nm;
    float auto_time_s;
    float auto_phase_time_s;
    float auto_target_rad;
    float auto_ref_rad;
    float auto_ref_vel_rad_s;
    float auto_ref_acc_rad_s2;
    float auto_freq_hz;
    float auto_speed_target_rad_s;
} PitchDownIdentDebug_s;

typedef struct
{
    uint32_t sample_count;
    float time_s;
    float dt_s;
    uint8_t valid;
    uint8_t feedback_ok;
    uint8_t torque_output_allowed;
    uint8_t auto_stage;
    uint8_t auto_pass;
    uint8_t auto_point_index;
    uint8_t auto_sample_active;
    uint8_t auto_ref_active;
    float auto_freq_hz;
    float down_pos_rad;
    float down_vel_rad_s;
    float phi_rel_rad;
    float omega_small_rad_s;
    float up_tau_cmd_nm;
    float up_motor_tor_nm;
} PitchUpIdentDebug_s;

typedef struct
{
    uint32_t sample_count;
    float time_s;
    float dt_s;
    uint8_t valid;
    uint8_t feedback_ok;
    uint8_t running;
    uint8_t arm;
    uint8_t axis;
    uint8_t gimbal_mode;
    uint8_t nuc_mode;
    uint8_t target_id;
    uint8_t lqr_ready;
    uint8_t accel_ready;
    uint8_t tau_big_limit_flag;
    uint8_t tau_small_limit_flag;
    uint8_t phase;
    uint8_t sign;
    uint8_t stop_reason;
    uint8_t up_limit_hit;
    uint8_t up_limit_side;
    float excite_time_s;
    float excite_freq_hz;
    float theta_rad;
    float phi_rad;
    float psi_rad;
    float omega_big_rad_s;
    float omega_small_rad_s;
    float theta_ref_rad;
    float phi_ref_rad;
    float psi_ref_rad;
    float theta_ref_dot_rad_s;
    float phi_ref_dot_rad_s;
    float theta_ref_ddot_rad_s2;
    float phi_ref_ddot_rad_s2;
    float theta_next_hold_rad;
    float theta_stop_rad;
    float psi_low_rad;
    float psi_high_rad;
    float tau_limit_nm;
    float down_hold_tau_nm;
    float tau_big_cmd_nm;
    float tau_small_cmd_nm;
    float tau_big_motor_cmd_nm;
    float tau_small_motor_cmd_nm;
    float tau_big_pre_limit_nm;
    float tau_small_pre_limit_nm;
    float tau_big_fb_nm;
    float tau_small_fb_nm;
    float tau_big_ff_nm;
    float tau_small_ff_nm;
    float tau_big_meas_nm;
    float tau_small_meas_nm;
    float tau_big_ctrl_tor_set_nm;
    float tau_small_ctrl_tor_set_nm;
    float motor_big_pos_rad;
    float motor_small_pos_rad;
    float motor_big_vel_rad_s;
    float motor_small_vel_rad_s;
    float motor_big_temp_c;
    float motor_small_temp_c;
    float motor_big_tmos_c;
    float motor_small_tmos_c;
} DoublePitchIdentDebug_s;

typedef struct
{
    float theta_rad;
    float phi_rad;
} DoublePitchStaticPoint_s;

typedef struct
{
    float theta_rad;
    float psi_low_rad;
    float psi_high_rad;
} DoublePitchGravityEnvelopePoint_s;

volatile PitchDownDebugCsv_s pitch_down_debug_csv;
volatile PitchDownIdentDebug_s g_pitch_down_ident_debug;
volatile PitchUpIdentDebug_s g_pitch_up_ident_debug;
volatile DoublePitchIdentDebug_s g_double_pitch_ident_debug;
volatile float pitch_down_ref_pos_rad          = PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float pitch_down_pos_rad              = 0.0f;
volatile float pitch_down_vel_rad_s            = 0.0f;
volatile float pitch_down_vel_raw_rad_s        = 0.0f;
volatile float pitch_down_pos_error_rad        = 0.0f;
volatile float pitch_down_ref_limited_pos_rad  = PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float pitch_down_ref_target_limited_pos_rad =
    PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile uint8_t pitch_down_ref_ramp_active    = 0U;
volatile float pitch_down_ref_traj_rad         = PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float pitch_down_ref_vel_rad_s        = 0.0f;
volatile float pitch_down_ref_acc_rad_s2       = 0.0f;
volatile float g_pitch_down_ident_center_rad   = PITCH_DOWN_CMD_DEFAULT_REF_RAD;
volatile float g_pitch_down_ident_lower_rad    = PITCH_DOWN_IDENT_LOWER_RAD;
volatile float g_pitch_down_ident_upper_rad    = PITCH_DOWN_IDENT_UPPER_RAD;
volatile float g_pitch_down_ident_cmd_gain     = PITCH_DOWN_IDENT_CMD_GAIN;
volatile uint8_t g_pitch_down_ident_enable     = 0U;
volatile uint8_t g_pitch_down_ident_stage_select =
    PITCH_DOWN_IDENT_STAGE_DEFAULT;
volatile uint8_t g_pitch_down_ident_running    = 0U;
volatile uint8_t g_pitch_down_ident_done       = 0U;
volatile uint8_t g_pitch_down_ident_abort      = 0U;
volatile uint8_t g_pitch_down_ident_abort_reason =
    PITCH_DOWN_IDENT_AUTO_ABORT_NONE;
volatile uint8_t g_pitch_down_ident_auto_phase =
    PITCH_DOWN_IDENT_AUTO_PHASE_IDLE;
volatile uint8_t g_pitch_down_ident_auto_profile = 0U;
volatile uint8_t g_pitch_down_ident_auto_stage =
    PITCH_DOWN_IDENT_STAGE_DEFAULT;
volatile uint8_t g_pitch_down_ident_auto_pass = 0U;
volatile uint8_t g_pitch_down_ident_auto_point_index = 0U;
volatile uint8_t g_pitch_down_ident_auto_sample_active = 0U;
volatile float g_pitch_down_ident_auto_time_s = 0.0f;
volatile float g_pitch_down_ident_auto_phase_time_s = 0.0f;
volatile float g_pitch_down_ident_auto_target_rad =
    PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float g_pitch_down_ident_auto_ref_rad =
    PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float g_pitch_down_ident_auto_ref_vel_rad_s = 0.0f;
volatile float g_pitch_down_ident_auto_ref_acc_rad_s2 = 0.0f;
volatile float g_pitch_down_ident_auto_freq_hz = 0.0f;
volatile float g_pitch_down_ident_auto_speed_target_rad_s = 0.0f;
volatile float pitch_down_ident_ref_raw_rad    = PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float pitch_down_ident_ref_rad        = PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile uint8_t pitch_down_ident_ref_limit_active = 0U;
volatile float pitch_down_follow_gain          = 0.08f;
volatile float pitch_down_follow_offset_rad    = 0.0f;
volatile float hole_down_enter_pre_mit_kp      = 1.0f;
volatile float hole_down_enter_pre_mit_kd      = 0.001f;
volatile float hole_down_enter_final_mit_kp    = 0.1f;
volatile float hole_down_enter_final_mit_kd    = 0.001f;
volatile float hole_v_des                      =0.01f;
volatile uint8_t hole_down_ready               = 0U;
volatile uint8_t hole_down_pre_ready           = 0U;
volatile uint8_t hole_up_ready                 = 0U;
volatile uint8_t hole_up_stable_ready          = 0U;
volatile float hole_up_ready_stable_time_s     = 0.0f;
volatile float hole_down_pre_ramp_target_rad   = HOLE_DOWN_RESTORE_TARGET_RAD;
volatile uint8_t hole_up_torque_zero_latched   = 0U;
volatile uint8_t hole_restore_up_enable_latched = 0U;
static uint8_t hole_down_ramp_target_primed    = 0U;
volatile float pitch_down_torque_cmd_nm        = 0.0f;
volatile float pitch_down_output_torque_target_nm = 0.0f;
volatile float pitch_down_output_torque_applied_nm = 0.0f;
volatile uint8_t pitch_down_output_torque_slew_active = 0U;
volatile float g_pitch_down_output_torque_slew_rate_nm_s =
    PITCH_DOWN_TORQUE_SLEW_RATE_NM_S;
volatile float g_pitch_down_hole_output_torque_slew_rate_nm_s =
    PITCH_DOWN_HOLE_TORQUE_SLEW_RATE_NM_S;
volatile float pitch_down_gravity_torque_ff_nm = 0.0f;
volatile float pitch_down_friction_torque_ff_nm = 0.0f;
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
volatile uint8_t pitch_down_eso_runtime_enable = 0U;
volatile float pitch_down_feedforward_nm       = 0.0f;
volatile float g_pitch_down_lqr_k_theta        = PITCH_DOWN_LQR_K_THETA;
volatile float g_pitch_down_lqr_k_omega        = PITCH_DOWN_LQR_K_OMEGA;
volatile uint8_t g_gimbal_auto_aim_active      = 0U;
volatile float g_pitch_down_auto_lqr_k_theta   = PITCH_DOWN_AUTO_LQR_K_THETA;
volatile float g_pitch_down_auto_lqr_k_omega   = PITCH_DOWN_AUTO_LQR_K_OMEGA;
volatile float g_pitch_down_auto_ref_lpf_rc_s  = PITCH_DOWN_AUTO_REF_LPF_RC_S;
volatile float g_pitch_down_auto_ref_raw_rad   = PITCH_DOWN_AUTO_REF_CENTER_RAD;
volatile float g_pitch_down_auto_ref_rad       = PITCH_DOWN_AUTO_REF_CENTER_RAD;
volatile float g_pitch_down_auto_ref_vel_rad_s = 0.0f;
volatile float g_pitch_down_auto_ref_acc_rad_s2 = 0.0f;
volatile uint8_t g_pitch_down_auto_ref_limit_active = 0U;
volatile float g_pitch_down_auto_ref_lpf_alpha = 0.0f;
volatile uint8_t g_pitch_down_eso_enable       = PITCH_DOWN_ESO_ENABLE;
volatile float g_pitch_down_eso_alpha          = PITCH_DOWN_ESO_ALPHA;
volatile float g_pitch_down_eso_comp_gain      = PITCH_DOWN_ESO_COMP_GAIN;
volatile float g_pitch_down_eso_comp_limit_nm  = PITCH_DOWN_ESO_COMP_LIMIT_NM;
volatile uint8_t g_pitch_down_model_enable     = PITCH_DOWN_MODEL_CONTROL_ENABLE;
volatile float g_pitch_down_model_j_kg_m2      = PITCH_DOWN_MODEL_J_KG_M2;
volatile float g_pitch_down_model_b_nms_rad    = PITCH_DOWN_MODEL_B_NMS_RAD;
volatile float g_pitch_down_model_tau_c_nm     = PITCH_DOWN_MODEL_TAU_C_NM;
volatile float g_pitch_down_model_omega_eps_rad_s =
    PITCH_DOWN_MODEL_OMEGA_EPS_RAD_S;
volatile float g_pitch_down_model_ref_vel_limit_rad_s =
    PITCH_DOWN_MODEL_REF_VEL_LIMIT_RAD_S;
volatile float g_pitch_down_model_ref_acc_limit_rad_s2 =
    PITCH_DOWN_MODEL_REF_ACC_LIMIT_RAD_S2;
volatile float pitch_down_model_tau_g_nm       = 0.0f;
volatile float pitch_down_model_tau_j_nm       = 0.0f;
volatile float pitch_down_model_tau_b_nm       = 0.0f;
volatile float pitch_down_model_tau_c_nm       = 0.0f;
volatile float pitch_down_model_tau_ff_nm      = 0.0f;
volatile float pitch_down_hold_i_torque_nm     = 0.0f;
volatile float pitch_down_hold_i_error_rad     = 0.0f;
volatile uint8_t pitch_down_hold_i_active      = 0U;
volatile uint8_t pitch_down_soft_limit_active  = 0U;
volatile float pitch_down_vel_lpf_alpha        = 1.0f;
volatile float pitch_up_ref_raw_rad          = PITCH_UP_IMU_ZERO_REF_RAD;
volatile float pitch_up_ref_rad              = 0.0f;
volatile float pitch_up_ref_vel_rad_s        = 0.0f;
volatile float pitch_up_ref_acc_rad_s2       = 0.0f;
volatile float pitch_up_ref_limited_rad      = PITCH_UP_IMU_ZERO_REF_RAD;
volatile uint8_t pitch_up_ref_source         = PITCH_UP_REF_SOURCE_NONE;
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
volatile uint8_t pitch_up_auto_ref_active    = 0U;
volatile float pitch_up_auto_lqr_k_theta     = PITCH_UP_AUTO_LQR_K_THETA;
volatile float pitch_up_auto_lqr_k_omega     = PITCH_UP_AUTO_LQR_K_OMEGA;
volatile uint8_t g_pitch_up_auto_pid_enable  = PITCH_UP_AUTO_PID_ENABLE_DEFAULT;
volatile float g_pitch_up_auto_pid_angle_kp  = PITCH_UP_AUTO_PID_ANGLE_KP_DEFAULT;
volatile float g_pitch_up_auto_pid_angle_ki  = PITCH_UP_AUTO_PID_ANGLE_KI_DEFAULT;
volatile float g_pitch_up_auto_pid_angle_kd  = PITCH_UP_AUTO_PID_ANGLE_KD_DEFAULT;
volatile float g_pitch_up_auto_pid_speed_kp  = PITCH_UP_AUTO_PID_SPEED_KP_DEFAULT;
volatile float g_pitch_up_auto_pid_speed_ki  = PITCH_UP_AUTO_PID_SPEED_KI_DEFAULT;
volatile float g_pitch_up_auto_pid_speed_kd  = PITCH_UP_AUTO_PID_SPEED_KD_DEFAULT;
volatile float g_pitch_up_auto_pid_angle_out_limit_rad_s =
    PITCH_UP_AUTO_PID_ANGLE_OUT_LIMIT_RAD_S;
volatile float g_pitch_up_auto_pid_torque_limit_nm =
    PITCH_UP_AUTO_PID_TORQUE_LIMIT_NM;
volatile uint8_t pitch_up_auto_pid_active    = 0U;
volatile float pitch_up_auto_pid_angle_out_rad_s = 0.0f;
volatile float pitch_up_auto_pid_speed_ref_rad_s = 0.0f;
volatile float pitch_up_auto_pid_fb_torque_nm = 0.0f;
volatile float pitch_up_auto_pid_acc_ff_torque_nm = 0.0f;
volatile float pitch_up_auto_pid_torque_cmd_nm = 0.0f;
volatile float g_pitch_up_vision_acc_filter_alpha =
    PITCH_UP_VISION_ACC_FILTER_ALPHA_DEFAULT;
volatile float g_pitch_up_vision_acc_raw_rad_s2  = 0.0f;
volatile float g_pitch_up_vision_acc_filt_rad_s2 = 0.0f;
static uint8_t pitch_up_vision_acc_filter_primed = 0U;
volatile uint8_t g_pitch_up_traj_planner_enable =
    PITCH_UP_TRAJ_PLANNER_ENABLE_DEFAULT;
volatile float g_pitch_up_traj_dt_since_latch_s = 0.0f;
volatile float g_pitch_up_traj_theta_ref_planned_rad = PITCH_UP_IMU_ZERO_REF_RAD;
volatile float g_pitch_up_traj_omega_ref_planned_rad_s = 0.0f;
volatile float g_pitch_up_traj_alpha_ref_planned_rad_s2 = 0.0f;
volatile int32_t g_pitch_up_traj_last_vision_frame_seq = 0;
static uint8_t pitch_up_traj_initialized = 0U;
static uint32_t pitch_up_traj_dwt_cnt = 0U;
static float pitch_up_traj_latched_theta_rad = PITCH_UP_IMU_ZERO_REF_RAD;
static float pitch_up_traj_latched_omega_rad_s = 0.0f;
static float pitch_up_traj_latched_alpha_rad_s2 = 0.0f;
static float pitch_up_traj_last_input_theta_rad = PITCH_UP_IMU_ZERO_REF_RAD;
static float pitch_up_traj_last_input_omega_rad_s = 0.0f;
static float pitch_up_traj_last_input_alpha_rad_s2 = 0.0f;
volatile float g_pitch_up_eso_w0_set_rad_s   = PITCH_UP_ESO_W0_SET_RAD_S;
volatile float g_pitch_up_eso_w0_actual_rad_s = 0.0f;
volatile uint8_t g_pitch_up_model_enable     = PITCH_UP_MODEL_LQR_ENABLE;
volatile float g_pitch_up_model_j_kg_m2      = PITCH_UP_MODEL_J_KG_M2;
volatile float g_pitch_up_model_b_nms_rad    = PITCH_UP_MODEL_B_NMS_RAD;
volatile float g_pitch_up_model_tau_c_nm     = PITCH_UP_MODEL_TAU_C_NM;
volatile float g_pitch_up_model_omega_eps_rad_s =
    PITCH_UP_MODEL_OMEGA_EPS_RAD_S;
volatile float pitch_up_model_tau_g_q_nm     = 0.0f;
volatile float pitch_up_model_tau_j_q_nm     = 0.0f;
volatile float pitch_up_model_tau_b_q_nm     = 0.0f;
volatile float pitch_up_model_tau_c_q_nm     = 0.0f;
volatile float pitch_up_model_tau_q_ff_nm    = 0.0f;
volatile float pitch_up_model_tau_motor_ff_nm = 0.0f;
volatile uint8_t g_pitch_up_ident_enable     = 0U;
volatile uint8_t g_pitch_up_ident_stage_select =
    PITCH_UP_IDENT_STAGE_DEFAULT;
volatile uint8_t g_pitch_up_ident_running    = 0U;
volatile uint8_t g_pitch_up_ident_done       = 0U;
volatile uint8_t g_pitch_up_ident_abort      = 0U;
volatile uint8_t g_pitch_up_ident_abort_reason =
    PITCH_UP_IDENT_AUTO_ABORT_NONE;
volatile uint8_t g_pitch_up_ident_auto_phase =
    PITCH_UP_IDENT_AUTO_PHASE_IDLE;
volatile uint8_t g_pitch_up_ident_auto_profile = 0U;
volatile uint8_t g_pitch_up_ident_auto_stage =
    PITCH_UP_IDENT_STAGE_DEFAULT;
volatile uint8_t g_pitch_up_ident_auto_pass = 0U;
volatile uint8_t g_pitch_up_ident_auto_point_index = 0U;
volatile uint8_t g_pitch_up_ident_auto_sample_active = 0U;
volatile float g_pitch_up_ident_auto_time_s = 0.0f;
volatile float g_pitch_up_ident_auto_phase_time_s = 0.0f;
volatile float g_pitch_up_ident_auto_target_rad = PITCH_UP_IMU_ZERO_REF_RAD;
volatile float g_pitch_up_ident_auto_ref_rad = PITCH_UP_IMU_ZERO_REF_RAD;
volatile float g_pitch_up_ident_auto_ref_vel_rad_s = 0.0f;
volatile float g_pitch_up_ident_auto_ref_acc_rad_s2 = 0.0f;
volatile float g_pitch_up_ident_auto_freq_hz = 0.0f;
volatile float g_pitch_up_ident_auto_speed_target_rad_s = 0.0f;
volatile uint8_t g_pitch_up_g_auto_start     = 0U;
volatile uint8_t g_pitch_up_g_auto_cancel    = 0U;
volatile uint8_t g_pitch_up_g_auto_running   = 0U;
volatile uint8_t g_pitch_up_g_auto_done      = 0U;
volatile uint8_t g_pitch_up_g_auto_abort     = 0U;
volatile uint8_t g_pitch_up_g_auto_phase     = PITCH_UP_G_AUTO_PHASE_IDLE;
volatile uint8_t g_pitch_up_g_auto_pass      = 0U;
volatile uint8_t g_pitch_up_g_auto_point_index = 0U;
volatile uint8_t g_pitch_up_g_auto_sample_active = 0U;
volatile float g_pitch_up_g_auto_target_rad  = 0.0f;
volatile uint8_t g_pitch_up_dyn_auto_start   = 0U;
volatile uint8_t g_pitch_up_dyn_auto_cancel  = 0U;
volatile uint8_t g_pitch_up_dyn_auto_running = 0U;
volatile uint8_t g_pitch_up_dyn_auto_done    = 0U;
volatile uint8_t g_pitch_up_dyn_auto_abort   = 0U;
volatile uint8_t g_pitch_up_dyn_auto_profile = PITCH_UP_DYN_PROFILE_LOW_SPEED;
volatile uint8_t g_pitch_up_dyn_auto_phase   = PITCH_UP_DYN_AUTO_PHASE_IDLE;
volatile uint8_t g_pitch_up_dyn_auto_sample_active = 0U;
volatile float g_pitch_up_dyn_auto_time_s    = 0.0f;
volatile float g_pitch_up_dyn_auto_ref_rad   = PITCH_UP_DYN_CENTER_RAD;
volatile float g_pitch_up_dyn_auto_ref_vel_rad_s = 0.0f;
volatile float g_pitch_up_dyn_auto_ref_acc_rad_s2 = 0.0f;
volatile float g_pitch_up_dyn_auto_freq_hz   = 0.0f;
static float pitch_up_g_auto_ref_rad         = PITCH_UP_IMU_ZERO_REF_RAD;
static float pitch_up_g_auto_phase_time_s    = 0.0f;
static float pitch_up_g_auto_stable_time_s   = 0.0f;
static float pitch_up_dyn_auto_phase_time_s  = 0.0f;
static float pitch_up_dyn_auto_stable_time_s = 0.0f;
static float pitch_up_ident_chirp_time_s     = 0.0f;
static float pitch_up_ident_stage_prep_time_s =
    PITCH_UP_IDENT_AUTO_F_HOLD_TIME_S;
static float pitch_down_ident_chirp_time_s   = 0.0f;
static float pitch_down_ident_stage_prep_time_s =
    PITCH_DOWN_IDENT_AUTO_F_HOLD_TIME_S;

static const float pitch_up_g_auto_points_rad[PITCH_UP_G_AUTO_POINT_COUNT] = {
    -30.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -25.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -20.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -15.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -10.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -5.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    0.0f,
    5.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    10.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
};

static const float pitch_up_ident_g_points_rad[PITCH_UP_IDENT_AUTO_G_POINT_COUNT] = {
    -30.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -25.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -20.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -15.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -10.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    -5.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    0.0f,
    5.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    10.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
    15.0f * PITCH_UP_G_AUTO_DEG_TO_RAD,
};

static const float pitch_up_dyn_friction_speed_rad_s[PITCH_UP_DYN_FRICTION_SPEED_COUNT] = {
    0.025f,
    0.040f,
    0.060f,
};

static const float pitch_up_ident_friction_speed_rad_s[PITCH_UP_IDENT_AUTO_FRICTION_SPEED_COUNT] = {
    0.06f,
    0.09f,
};

static const float pitch_down_ident_g_points_rad[PITCH_DOWN_IDENT_AUTO_G_POINT_COUNT] = {
    1.25f, 1.35f, 1.45f, 1.55f, 1.65f,
    1.75f, 1.85f, 1.95f, 2.05f, 2.15f,
};

static const float pitch_down_ident_friction_speed_rad_s[PITCH_DOWN_IDENT_AUTO_FRICTION_SPEED_COUNT] = {
    0.04f,
    0.06f,
    0.09f,
};

/* ---- Pitch-up chirp injection (system identification) ----------- */
/* Set g_pitch_up_chirp_arm = 1 via debugger to start a sweep. The chirp
 * runs from f_start to f_end over duration_s, with amp_rad superposed on
 * center_rad. Override is released automatically on completion. */
volatile uint8_t g_pitch_up_chirp_arm           = 0U;
volatile float   g_pitch_up_chirp_f_start_hz    = 0.5f;
volatile float   g_pitch_up_chirp_f_end_hz      = 8.0f;
volatile float   g_pitch_up_chirp_duration_s    = 20.0f;
volatile float   g_pitch_up_chirp_amp_rad       = 0.0873f;   /* 5 deg */
volatile float   g_pitch_up_chirp_center_rad    = 0.0f;
volatile uint8_t g_pitch_up_chirp_running       = 0U;
volatile float   g_pitch_up_chirp_time_s        = 0.0f;
volatile float   g_pitch_up_chirp_inst_freq_hz  = 0.0f;
volatile float   g_pitch_up_chirp_ref_rad       = 0.0f;
/* ----------------------------------------------------------------- */

/* ---- Double-pitch closed-loop identification sweep ---------------- */
volatile uint8_t g_double_pitch_ident_arm          = 0U;
volatile uint8_t g_double_pitch_ident_running      = 0U;
volatile uint8_t g_double_pitch_ident_axis         = DOUBLE_PITCH_IDENT_AXIS_THETA;
volatile float g_double_pitch_ident_f_start_hz     = 0.3f;
volatile float g_double_pitch_ident_f_end_hz       = 6.0f;
volatile float g_double_pitch_ident_duration_s     = 20.0f;
volatile float g_double_pitch_ident_amp_rad        = 0.04f;
volatile float g_double_pitch_ident_theta_center_rad =
    PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float g_double_pitch_ident_phi_center_rad =
    (PITCH_UP_IMU_ZERO_REF_RAD - PITCH_DOWN_DEFAULT_REF_POS_RAD);
volatile float g_double_pitch_ident_time_s         = 0.0f;
volatile float g_double_pitch_ident_inst_freq_hz   = 0.0f;
volatile float g_double_pitch_ident_theta_ref_rad  = PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float g_double_pitch_ident_phi_ref_rad =
    (PITCH_UP_IMU_ZERO_REF_RAD - PITCH_DOWN_DEFAULT_REF_POS_RAD);
volatile float g_double_pitch_ident_psi_ref_rad    = PITCH_UP_IMU_ZERO_REF_RAD;
volatile float g_double_pitch_ident_theta_ref_dot_rad_s = 0.0f;
volatile float g_double_pitch_ident_phi_ref_dot_rad_s = 0.0f;
volatile float g_double_pitch_ident_psi_ref_dot_rad_s = 0.0f;
volatile float g_double_pitch_ident_theta_ref_ddot_rad_s2 = 0.0f;
volatile float g_double_pitch_ident_phi_ref_ddot_rad_s2 = 0.0f;
volatile float g_double_pitch_ident_psi_ref_ddot_rad_s2 = 0.0f;
volatile uint8_t g_double_pitch_static_point_index = 0U;
volatile uint8_t g_double_pitch_static_phase = DOUBLE_PITCH_STATIC_PHASE_IDLE;
volatile uint8_t g_double_pitch_static_active_index = 0U;
volatile uint8_t g_double_pitch_gravity_ramp_arm = 0U;
volatile uint8_t g_double_pitch_gravity_ramp_auto_start = 0U;
volatile uint8_t g_double_pitch_gravity_ramp_running = 0U;
volatile uint8_t g_double_pitch_gravity_ramp_phase =
    DOUBLE_PITCH_GRAVITY_RAMP_PHASE_IDLE;
volatile float g_double_pitch_gravity_ramp_duration_s =
    DOUBLE_PITCH_GRAVITY_RAMP_DEFAULT_DURATION_S;
volatile float g_double_pitch_gravity_ramp_return_s =
    DOUBLE_PITCH_GRAVITY_RAMP_DEFAULT_RETURN_S;
volatile float g_double_pitch_gravity_ramp_tau_big_end_nm = 1.0f;
volatile float g_double_pitch_gravity_ramp_tau_small_end_nm = 1.0f;
volatile float g_double_pitch_gravity_ramp_tau_limit_nm =
    DOUBLE_PITCH_GRAVITY_RAMP_MAX_TORQUE_NM;
volatile uint8_t g_double_pitch_gravity_step_arm = 0U;
volatile uint8_t g_double_pitch_gravity_step_sign =
    DOUBLE_PITCH_GRAVITY_STEP_SIGN_POSITIVE;
volatile float g_double_pitch_gravity_step_tau_limit_nm =
    DOUBLE_PITCH_GRAVITY_RAMP_MAX_TORQUE_NM;
volatile float g_double_pitch_gravity_step_down_stop_rad =
    DOUBLE_PITCH_GRAVITY_STEP_DEFAULT_DOWN_STOP_RAD;
volatile float g_double_pitch_gravity_step_theta_step_rad =
    DOUBLE_PITCH_GRAVITY_STEP_DEFAULT_THETA_STEP_RAD;
volatile float g_double_pitch_gravity_step_torque_rate_nm_s =
    DOUBLE_PITCH_GRAVITY_STEP_TORQUE_RATE_NM_S;
volatile uint8_t g_double_pitch_gravity_step_stop_reason =
    DOUBLE_PITCH_GRAVITY_STEP_STOP_NONE;
volatile uint8_t g_double_pitch_gravity_step_up_limit_hit = 0U;
volatile uint8_t g_double_pitch_gravity_step_up_limit_side =
    DOUBLE_PITCH_GRAVITY_STEP_UP_LIMIT_NONE;
volatile float g_double_pitch_gravity_step_theta_next_hold_rad =
    PITCH_DOWN_DEFAULT_REF_POS_RAD;
volatile float g_double_pitch_gravity_step_theta_stop_rad =
    DOUBLE_PITCH_GRAVITY_STEP_DEFAULT_DOWN_STOP_RAD;
volatile float g_double_pitch_gravity_step_psi_low_rad = 0.0f;
volatile float g_double_pitch_gravity_step_psi_high_rad = 0.0f;
volatile float g_double_pitch_gravity_step_down_hold_tau_nm = 0.0f;
volatile uint8_t g_double_pitch_normal_controller_enable = 0U;
volatile uint8_t g_double_pitch_direct_down_torque_arm = 0U;
volatile float g_double_pitch_direct_down_torque_nm = 0.0f;

static float double_pitch_gravity_step_down_tau_nm = 0.0f;
static float double_pitch_gravity_step_up_tau_nm = 0.0f;
static float double_pitch_gravity_step_last_hold_theta_rad =
    PITCH_DOWN_DEFAULT_REF_POS_RAD;
static int8_t double_pitch_gravity_step_stop_dir = 1;

static const DoublePitchStaticPoint_s double_pitch_static_points[DOUBLE_PITCH_STATIC_POINT_COUNT] = {
    {1.45f, -0.677130f}, {1.45f, -1.148950f}, {1.45f, -1.620771f}, {1.45f, -2.092592f}, {1.45f, -2.564413f},
    {1.60f, -0.977852f}, {1.60f, -1.449673f}, {1.60f, -1.921494f}, {1.60f, -2.393314f}, {1.60f, -2.865135f},
    {1.75f, -1.278574f}, {1.75f, -1.750395f}, {1.75f, -2.222216f}, {1.75f, -2.694037f}, {1.75f,  3.117328f},
    {1.90f, -1.579297f}, {1.90f, -2.051118f}, {1.90f, -2.522938f}, {1.90f, -2.994759f}, {1.90f,  2.816605f},
    {2.05f, -1.880019f}, {2.05f, -2.351840f}, {2.05f, -2.823661f}, {2.05f,  2.987704f}, {2.05f,  2.515883f},
};

static const DoublePitchGravityEnvelopePoint_s double_pitch_gravity_envelope[] = {
    {-0.08f, -0.0248f, 0.0724f},
    {0.00f, -0.0694f, 0.1118f},
    {0.05f, -0.1168f, 0.1516f},
    {0.15f, -0.2231f, 0.2311f},
    {0.25f, -0.3335f, 0.3111f},
    {0.35f, -0.4489f, 0.3977f},
    {0.45f, -0.5680f, 0.4539f},
    {0.55f, -0.6681f, 0.4842f},
    {0.65f, -0.7294f, 0.5128f},
    {0.75f, -0.7793f, 0.5745f},
    {0.85f, -0.8039f, 0.6307f},
    {0.95f, -0.8036f, 0.6999f},
    {1.05f, -0.7981f, 0.8320f},
    {1.15f, -0.8149f, 0.8720f},
    {1.25f, -0.8344f, 0.9152f},
    {1.35f, -0.8257f, 0.9186f},
    {1.45f, -0.8205f, 0.8231f},
    {1.55f, -0.7993f, 0.7227f},
    {1.65f, -0.7753f, 0.6220f},
    {1.75f, -0.7729f, 0.5215f},
    {1.85f, -0.7549f, 0.4209f},
    {1.95f, -0.7509f, 0.3209f},
    {2.05f, -0.7340f, 0.2205f},
    {2.15f, -0.7771f, 0.1199f},
    {2.17f, -0.7771f, 0.0999f},
};
/* ------------------------------------------------------------------ */

static uint32_t pitch_down_vel_lpf_dwt_cnt;
static uint8_t pitch_down_vel_lpf_primed = 0U;
static uint8_t pitch_down_auto_ref_filter_primed = 0U;
static uint8_t pitch_down_auto_ref_last_active = 0U;
static float pitch_down_debug_last_time_s = 0.0f;
static uint8_t pitch_down_debug_time_primed = 0U;
static uint8_t pitch_down_ref_override_active = 0U;
static uint8_t pitch_down_ref_limit_bypass = 0U;
static float pitch_down_ref_override_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;
static uint8_t pitch_down_ref_traj_primed = 0U;
static uint8_t pitch_down_output_torque_slew_primed = 0U;
static float pitch_down_output_torque_last_nm = 0.0f;
static uint32_t pitch_down_output_torque_slew_dwt_cnt;
static uint8_t pitch_up_ref_override_active = 0U;
static float pitch_up_ref_override_rad = PITCH_UP_IMU_ZERO_REF_RAD;
static uint8_t pitch_up_auto_ref_last_active = 0U;
static uint8_t pitch_up_auto_pid_last_active = 0U;

static void PitchUpAutoPidReset(void);
static void PitchUpAutoPidClearDebug(void);

static void PitchDownClearLQRDebug(void)
{
    pitch_down_lqr_e_theta_rad = 0.0f;
    pitch_down_lqr_e_omega_rad_s = 0.0f;
    pitch_down_lqr_tau_raw_nm = 0.0f;
    pitch_down_lqr_pre_limit_torque_nm = 0.0f;
    pitch_down_lqr_output_nm = 0.0f;
    pitch_down_friction_torque_ff_nm = 0.0f;
    pitch_down_lqr_limit_active = 0U;
    pitch_down_eso_runtime_enable = 0U;
    pitch_down_model_tau_g_nm = 0.0f;
    pitch_down_model_tau_j_nm = 0.0f;
    pitch_down_model_tau_b_nm = 0.0f;
    pitch_down_model_tau_c_nm = 0.0f;
    pitch_down_model_tau_ff_nm = 0.0f;
    pitch_down_feedforward_nm = 0.0f;
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
static float PitchWrapPiRad(float angle_rad)
{
    while (angle_rad > 3.141592653589793f) {
        angle_rad -= 6.283185307179586f;
    }
    while (angle_rad < -3.141592653589793f) {
        angle_rad += 6.283185307179586f;
    }
    return angle_rad;
}

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

static void PitchDownResetAutoVisionRef(void)
{
    pitch_down_auto_ref_filter_primed = 0U;
    g_pitch_down_auto_ref_raw_rad = PITCH_DOWN_AUTO_REF_CENTER_RAD;
    g_pitch_down_auto_ref_rad = PITCH_DOWN_AUTO_REF_CENTER_RAD;
    g_pitch_down_auto_ref_vel_rad_s = 0.0f;
    g_pitch_down_auto_ref_acc_rad_s2 = 0.0f;
    g_pitch_down_auto_ref_limit_active = 0U;
    g_pitch_down_auto_ref_lpf_alpha = 0.0f;
}

static float PitchDownAutoVisionRefLpfAlpha(float dt_s)
{
    float rc_s = g_pitch_down_auto_ref_lpf_rc_s;
    float alpha;

    if (isfinite(rc_s) == 0 || rc_s < 0.0f) {
        rc_s = PITCH_DOWN_AUTO_REF_LPF_RC_S;
    }
    g_pitch_down_auto_ref_lpf_rc_s = rc_s;

    dt_s = PitchDownClampRange(dt_s,
                               PITCH_DOWN_CONTROL_MIN_DT_S,
                               PITCH_DOWN_CONTROL_MAX_DT_S);
    if (rc_s <= 0.0f) {
        alpha = 1.0f;
    } else {
        alpha = dt_s / (rc_s + dt_s);
    }
    alpha = PitchDownClampRange(alpha, 0.0f, 1.0f);
    g_pitch_down_auto_ref_lpf_alpha = alpha;

    return alpha;
}

static void PitchDownUpdateAutoVisionRef(float dt_s)
{
    const float alpha = PitchDownAutoVisionRefLpfAlpha(dt_s);
    const float raw_offset_rad =
        (isfinite(gimbal_cmd_recv.pitch_ref_rad) != 0) ?
        PITCH_DOWN_AUTO_REF_VISION_SIGN * gimbal_cmd_recv.pitch_ref_rad : 0.0f;
    const float raw_vel_rad_s =
        (isfinite(gimbal_cmd_recv.pitch_ref_vel_rad_s) != 0) ?
        PITCH_DOWN_AUTO_REF_VISION_SIGN * gimbal_cmd_recv.pitch_ref_vel_rad_s : 0.0f;
    const float raw_acc_rad_s2 =
        (isfinite(gimbal_cmd_recv.pitch_ref_acc_rad_s2) != 0) ?
        PITCH_DOWN_AUTO_REF_VISION_SIGN * gimbal_cmd_recv.pitch_ref_acc_rad_s2 : 0.0f;
    static float filt_offset_rad = 0.0f;
    static float filt_vel_rad_s = 0.0f;
    static float filt_acc_rad_s2 = 0.0f;
    float raw_ref_rad;
    float limited_ref_rad;

    if (pitch_down_auto_ref_filter_primed == 0U ||
        isfinite(filt_offset_rad) == 0 ||
        isfinite(filt_vel_rad_s) == 0 ||
        isfinite(filt_acc_rad_s2) == 0) {
        filt_offset_rad = raw_offset_rad;
        filt_vel_rad_s = raw_vel_rad_s;
        filt_acc_rad_s2 = raw_acc_rad_s2;
        pitch_down_auto_ref_filter_primed = 1U;
    } else {
        filt_offset_rad += alpha * (raw_offset_rad - filt_offset_rad);
        filt_vel_rad_s += alpha * (raw_vel_rad_s - filt_vel_rad_s);
        filt_acc_rad_s2 += alpha * (raw_acc_rad_s2 - filt_acc_rad_s2);
    }

    raw_ref_rad = PITCH_DOWN_AUTO_REF_CENTER_RAD + filt_offset_rad;
    limited_ref_rad =
        PitchDownClampRange(raw_ref_rad,
                            PITCH_DOWN_AUTO_REF_CENTER_RAD -
                            PITCH_DOWN_AUTO_REF_RANGE_RAD,
                            PITCH_DOWN_AUTO_REF_CENTER_RAD +
                            PITCH_DOWN_AUTO_REF_RANGE_RAD);
    g_pitch_down_auto_ref_limit_active =
        (limited_ref_rad != raw_ref_rad) ? 1U : 0U;
    if (g_pitch_down_auto_ref_limit_active != 0U) {
        filt_vel_rad_s = 0.0f;
        filt_acc_rad_s2 = 0.0f;
    }

    g_pitch_down_auto_ref_raw_rad = raw_ref_rad;
    g_pitch_down_auto_ref_rad = limited_ref_rad;
    g_pitch_down_auto_ref_vel_rad_s = filt_vel_rad_s;
    g_pitch_down_auto_ref_acc_rad_s2 = filt_acc_rad_s2;

    pitch_down_ref_pos_rad = raw_ref_rad;
    pitch_down_ref_target_limited_pos_rad = limited_ref_rad;
    pitch_down_ref_limited_pos_rad = limited_ref_rad;
    pitch_down_ref_traj_rad = limited_ref_rad;
    pitch_down_ref_vel_rad_s = filt_vel_rad_s;
    pitch_down_ref_acc_rad_s2 = filt_acc_rad_s2;
    pitch_down_ref_traj_primed = 1U;
    pitch_down_ref_ramp_active = 0U;
}

static uint8_t GimbalAutoAimCommandActive(void)
{
    if (gimbal_cmd_recv.nuc_mode != version_control) {
        return 0U;
    }
    return (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE ||
            gimbal_cmd_recv.gimbal_mode == GIMBAL_MOTOR_MODE) ? 1U : 0U;
}

static void GimbalSetAutoAimActive(uint8_t active)
{
    g_gimbal_auto_aim_active = (active != 0U) ? 1U : 0U;
    if (g_gimbal_auto_aim_active == 0U) {
        PitchDownResetAutoVisionRef();
    }
}

static float PitchDownResolveOutputTorqueSlewRate(void)
{
    float slew_rate_nm_s =
        (gimbal_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) ?
        g_pitch_down_hole_output_torque_slew_rate_nm_s :
        g_pitch_down_output_torque_slew_rate_nm_s;

    if (slew_rate_nm_s < 0.0f) {
        slew_rate_nm_s = 0.0f;
    }
    return slew_rate_nm_s;
}

static void PitchDownResetOutputTorqueSlew(float torque_nm)
{
    pitch_down_output_torque_slew_primed = 1U;
    pitch_down_output_torque_last_nm = torque_nm;
    pitch_down_output_torque_target_nm = torque_nm;
    pitch_down_output_torque_applied_nm = torque_nm;
    pitch_down_output_torque_slew_active = 0U;
    pitch_down_torque_cmd_nm = torque_nm;
    DWT_GetDeltaT(&pitch_down_output_torque_slew_dwt_cnt);
}

static void PitchDownSetTorqueImmediate(float torque_nm)
{
    PitchDownResetOutputTorqueSlew(torque_nm);
    DMMotorSetTorque(pitch_motor_down, torque_nm);
}

static float PitchDownSetTorqueWithSlew(float target_torque_nm)
{
    float applied_torque_nm = target_torque_nm;
    const float slew_rate_nm_s = PitchDownResolveOutputTorqueSlewRate();
    float dt_s;

    pitch_down_output_torque_target_nm = target_torque_nm;
    if (pitch_down_output_torque_slew_primed == 0U) {
        PitchDownResetOutputTorqueSlew(0.0f);
        dt_s = 0.001f;
    } else {
        dt_s = DWT_GetDeltaT(&pitch_down_output_torque_slew_dwt_cnt);
    }
    dt_s = PitchDownClampRange(dt_s,
                               PITCH_DOWN_CONTROL_MIN_DT_S,
                               PITCH_DOWN_CONTROL_MAX_DT_S);
    pitch_down_output_torque_target_nm = target_torque_nm;

    if (slew_rate_nm_s > 0.0f) {
        const float max_delta_nm = slew_rate_nm_s * dt_s;

        applied_torque_nm =
            PitchDownClampRange(target_torque_nm,
                                pitch_down_output_torque_last_nm - max_delta_nm,
                                pitch_down_output_torque_last_nm + max_delta_nm);
    }

    pitch_down_output_torque_slew_active =
        (applied_torque_nm != target_torque_nm) ? 1U : 0U;
    if (pitch_down_output_torque_slew_active != 0U) {
        pitch_down_lqr_limit_active = 1U;
    }
    pitch_down_output_torque_last_nm = applied_torque_nm;
    pitch_down_output_torque_applied_nm = applied_torque_nm;
    pitch_down_torque_cmd_nm = applied_torque_nm;
    DMMotorSetTorque(pitch_motor_down, applied_torque_nm);

    return applied_torque_nm;
}

static void PitchDownResetRefTrajectory(float ref_rad)
{
    pitch_down_ref_traj_rad = ref_rad;
    pitch_down_ref_vel_rad_s = 0.0f;
    pitch_down_ref_acc_rad_s2 = 0.0f;
    pitch_down_ref_traj_primed = 1U;
}

static void PitchDownUpdateRefTrajectory(float target_rad, float dt_s)
{
    float vel_limit = fabsf(g_pitch_down_model_ref_vel_limit_rad_s);
    float acc_limit = fabsf(g_pitch_down_model_ref_acc_limit_rad_s2);
    float err_rad;
    float vel_des_rad_s;
    float vel_next_rad_s;
    float max_delta_vel_rad_s;
    float step_rad;

    dt_s = PitchDownClampRange(dt_s,
                               PITCH_DOWN_CONTROL_MIN_DT_S,
                               PITCH_DOWN_CONTROL_MAX_DT_S);
    if (vel_limit <= 0.0f) {
        vel_limit = PITCH_DOWN_MODEL_REF_VEL_LIMIT_RAD_S;
    }
    if (acc_limit <= 0.0f) {
        acc_limit = PITCH_DOWN_MODEL_REF_ACC_LIMIT_RAD_S2;
    }

    if (pitch_down_ref_traj_primed == 0U) {
        PitchDownResetRefTrajectory(pitch_down_pos_rad);
    }

    err_rad = target_rad - pitch_down_ref_traj_rad;
    vel_des_rad_s = PitchDownClampRange(err_rad / dt_s,
                                        -vel_limit,
                                        vel_limit);
    max_delta_vel_rad_s = acc_limit * dt_s;
    vel_next_rad_s =
        PitchDownClampRange(vel_des_rad_s,
                            pitch_down_ref_vel_rad_s - max_delta_vel_rad_s,
                            pitch_down_ref_vel_rad_s + max_delta_vel_rad_s);
    step_rad = vel_next_rad_s * dt_s;
    if (fabsf(step_rad) > fabsf(err_rad)) {
        step_rad = err_rad;
        vel_next_rad_s = step_rad / dt_s;
    }

    pitch_down_ref_acc_rad_s2 =
        (vel_next_rad_s - pitch_down_ref_vel_rad_s) / dt_s;
    pitch_down_ref_vel_rad_s = vel_next_rad_s;
    pitch_down_ref_traj_rad += step_rad;

    if (fabsf(target_rad - pitch_down_ref_traj_rad) < 1e-5f &&
        fabsf(pitch_down_ref_vel_rad_s) < 1e-4f) {
        pitch_down_ref_traj_rad = target_rad;
        pitch_down_ref_vel_rad_s = 0.0f;
        pitch_down_ref_acc_rad_s2 = 0.0f;
    }
}

static void PitchDownResetModelFeedforwardDebug(void)
{
    pitch_down_model_tau_g_nm = 0.0f;
    pitch_down_model_tau_j_nm = 0.0f;
    pitch_down_model_tau_b_nm = 0.0f;
    pitch_down_model_tau_c_nm = 0.0f;
    pitch_down_model_tau_ff_nm = 0.0f;
    pitch_down_feedforward_nm = 0.0f;
}

static float PitchDownCalculateModelFeedforward(void)
{
    float omega_eps;

    if (g_pitch_down_model_enable == 0U) {
        PitchDownResetModelFeedforwardDebug();
        return 0.0f;
    }

    omega_eps = g_pitch_down_model_omega_eps_rad_s;
    if (omega_eps <= 0.0f) {
        omega_eps = PITCH_DOWN_MODEL_OMEGA_EPS_RAD_S;
    }

    pitch_down_model_tau_g_nm =
        PITCH_DOWN_MODEL_G0_NM +
        PITCH_DOWN_MODEL_GS_NM * sinf(pitch_down_pos_rad) +
        PITCH_DOWN_MODEL_GC_NM * cosf(pitch_down_pos_rad);
    pitch_down_model_tau_j_nm =
        g_pitch_down_model_j_kg_m2 * pitch_down_ref_acc_rad_s2;
    pitch_down_model_tau_b_nm =
        g_pitch_down_model_b_nms_rad * pitch_down_vel_rad_s;
    pitch_down_model_tau_c_nm =
        g_pitch_down_model_tau_c_nm *
        tanhf(pitch_down_vel_rad_s / omega_eps);
    pitch_down_model_tau_ff_nm =
        pitch_down_model_tau_g_nm +
        pitch_down_model_tau_j_nm +
        pitch_down_model_tau_b_nm +
        pitch_down_model_tau_c_nm;
    pitch_down_feedforward_nm = pitch_down_model_tau_ff_nm;
    return pitch_down_feedforward_nm;
}

static float PitchDownCalcFrictionFeedforward(float velocity_rad_s)
{
    return PITCH_DOWN_FRICTION_COMP_NM *
           tanhf(velocity_rad_s / PITCH_DOWN_FRICTION_SMOOTH_VEL_RAD_S);
}

static float PitchDownCalcMitGravityFeedforward(float theta_rad)
{
    float tau_raw;
    float tau_pre_limit;
    float fade_ratio;
    float fade;

    tau_raw = PITCH_DOWN_MIT_GRAVITY_SIN_COEFF_NM * sinf(theta_rad) +
              PITCH_DOWN_MIT_GRAVITY_COS_COEFF_NM * cosf(theta_rad) +
              PITCH_DOWN_MIT_GRAVITY_BIAS_NM;

    if (theta_rad >= PITCH_DOWN_MIT_GRAVITY_FADE_END_RAD) {
        tau_pre_limit = 0.0f;
    } else if (theta_rad > PITCH_DOWN_MIT_GRAVITY_FADE_START_RAD) {
        fade_ratio = (PITCH_DOWN_MIT_GRAVITY_FADE_END_RAD - theta_rad) /
                     (PITCH_DOWN_MIT_GRAVITY_FADE_END_RAD -
                      PITCH_DOWN_MIT_GRAVITY_FADE_START_RAD);
        fade_ratio = PitchDownClampRange(fade_ratio, 0.0f, 1.0f);
        fade = fade_ratio * fade_ratio * (3.0f - 2.0f * fade_ratio);
        tau_pre_limit = tau_raw * fade;
    } else {
        tau_pre_limit = tau_raw;
    }

    pitch_down_gravity_torque_raw_nm = tau_raw;
    pitch_down_gravity_torque_pre_limit_nm = tau_pre_limit;
    pitch_down_gravity_torque_ff_nm =
        PitchDownClampRange(tau_pre_limit,
                            0.0f,
                            PITCH_DOWN_MIT_GRAVITY_COMP_LIMIT_NM);
    return pitch_down_gravity_torque_ff_nm;
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
    pitch_down_ident_ref_raw_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;
    pitch_down_ident_ref_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;
    pitch_down_ident_ref_limit_active = 0U;
    PitchDownSetRefOverride(PITCH_DOWN_DEFAULT_REF_POS_RAD, 1U);
}

static void __attribute__((unused)) PitchDownApplyIdentPitchControlRef(void)
{
    const float ref_raw =
        g_pitch_down_ident_center_rad +
        g_pitch_down_ident_cmd_gain * gimbal_cmd_recv.pitch;
    const float ref_limited =
        PitchDownClampRange(ref_raw,
                            g_pitch_down_ident_lower_rad,
                            g_pitch_down_ident_upper_rad);

    pitch_down_follow_offset_rad = 0.0f;
    pitch_down_ident_ref_raw_rad = ref_raw;
    pitch_down_ident_ref_rad = ref_limited;
    pitch_down_ident_ref_limit_active =
        (fabsf(ref_limited - ref_raw) > 1e-6f) ? 1U : 0U;
    PitchDownSetRefOverride(ref_limited, 1U);
}

static uint8_t PitchDownIdentAutoActive(void)
{
    return (g_pitch_down_ident_running != 0U ||
            g_pitch_down_ident_enable != 0U) ? 1U : 0U;
}

static uint8_t PitchDownIdentAutoFeedbackOk(void)
{
    if (pitch_motor_down == NULL ||
        pitch_motor_up == NULL ||
        gimbal_IMU_data == NULL) {
        return 0U;
    }
    if (DMMotorIsOnline(pitch_motor_down) == 0U ||
        DMMotorIsOnline(pitch_motor_up) == 0U) {
        return 0U;
    }
    return 1U;
}

static float PitchDownIdentAutoGravityTarget(uint8_t pass,
                                             uint8_t point_index)
{
    uint8_t index = point_index;

    if (index >= PITCH_DOWN_IDENT_AUTO_G_POINT_COUNT) {
        index = PITCH_DOWN_IDENT_AUTO_G_POINT_COUNT - 1U;
    }
    if (pass != 0U) {
        index = (PITCH_DOWN_IDENT_AUTO_G_POINT_COUNT - 1U) - index;
    }
    return pitch_down_ident_g_points_rad[index];
}

static float PitchDownIdentAutoFrictionSpeed(uint8_t index)
{
    if (index >= PITCH_DOWN_IDENT_AUTO_FRICTION_SPEED_COUNT) {
        index = PITCH_DOWN_IDENT_AUTO_FRICTION_SPEED_COUNT - 1U;
    }
    return pitch_down_ident_friction_speed_rad_s[index];
}

static uint8_t PitchDownIdentAutoNormalizeStage(uint8_t stage)
{
    if (stage >= PITCH_DOWN_IDENT_STAGE_GRAVITY_UP &&
        stage <= PITCH_DOWN_IDENT_STAGE_CHIRP_HIGH) {
        return stage;
    }
    return PITCH_DOWN_IDENT_STAGE_DEFAULT;
}

static uint8_t PitchDownIdentAutoStageProfile(uint8_t stage)
{
    if (stage == PITCH_DOWN_IDENT_STAGE_GRAVITY_UP ||
        stage == PITCH_DOWN_IDENT_STAGE_GRAVITY_DOWN) {
        return PITCH_DOWN_IDENT_AUTO_PROFILE_GRAVITY;
    }
    if (stage >= PITCH_DOWN_IDENT_STAGE_F_POS_004 &&
        stage <= PITCH_DOWN_IDENT_STAGE_F_NEG_009) {
        return PITCH_DOWN_IDENT_AUTO_PROFILE_FRICTION;
    }
    return PITCH_DOWN_IDENT_AUTO_PROFILE_CHIRP;
}

static uint8_t PitchDownIdentAutoStageFrictionIndex(uint8_t stage)
{
    if (stage == PITCH_DOWN_IDENT_STAGE_F_POS_006 ||
        stage == PITCH_DOWN_IDENT_STAGE_F_NEG_006) {
        return 1U;
    }
    if (stage == PITCH_DOWN_IDENT_STAGE_F_POS_009 ||
        stage == PITCH_DOWN_IDENT_STAGE_F_NEG_009) {
        return 2U;
    }
    return 0U;
}

static uint8_t PitchDownIdentAutoStageFrictionPass(uint8_t stage)
{
    return (stage == PITCH_DOWN_IDENT_STAGE_F_NEG_004 ||
            stage == PITCH_DOWN_IDENT_STAGE_F_NEG_006 ||
            stage == PITCH_DOWN_IDENT_STAGE_F_NEG_009) ? 1U : 0U;
}

static float PitchDownIdentAutoStageChirpStartHz(uint8_t stage)
{
    return (stage == PITCH_DOWN_IDENT_STAGE_CHIRP_HIGH) ?
           0.40f : PITCH_DOWN_IDENT_AUTO_CHIRP_F_START_HZ;
}

static float PitchDownIdentAutoStageChirpEndHz(uint8_t stage)
{
    return (stage == PITCH_DOWN_IDENT_STAGE_CHIRP_HIGH) ?
           PITCH_DOWN_IDENT_AUTO_CHIRP_F_END_HZ : 0.40f;
}

static void PitchDownIdentAutoSetPhase(uint8_t phase)
{
    g_pitch_down_ident_auto_phase = phase;
    g_pitch_down_ident_auto_phase_time_s = 0.0f;
    g_pitch_down_ident_auto_sample_active = 0U;
}

static void PitchDownIdentAutoSetRef(float ref_rad,
                                     float ref_vel_rad_s,
                                     float ref_acc_rad_s2,
                                     float target_rad,
                                     float freq_hz,
                                     float speed_target_rad_s,
                                     uint8_t sample_active)
{
    const float ref_limited =
        PitchDownClampRange(ref_rad,
                            PITCH_DOWN_IDENT_LOWER_RAD,
                            PITCH_DOWN_IDENT_UPPER_RAD);

    pitch_down_follow_offset_rad = 0.0f;
    pitch_down_ident_ref_raw_rad = ref_rad;
    pitch_down_ident_ref_rad = ref_limited;
    pitch_down_ident_ref_limit_active =
        (fabsf(ref_limited - ref_rad) > 1e-6f) ? 1U : 0U;
    g_pitch_down_ident_auto_target_rad = target_rad;
    g_pitch_down_ident_auto_ref_rad = ref_limited;
    g_pitch_down_ident_auto_ref_vel_rad_s = ref_vel_rad_s;
    g_pitch_down_ident_auto_ref_acc_rad_s2 = ref_acc_rad_s2;
    g_pitch_down_ident_auto_freq_hz = freq_hz;
    g_pitch_down_ident_auto_speed_target_rad_s = speed_target_rad_s;
    g_pitch_down_ident_auto_sample_active =
        (sample_active != 0U &&
         pitch_down_ident_ref_limit_active == 0U) ? 1U : 0U;
    PitchDownSetRefOverride(ref_limited, 1U);
}

static void PitchDownIdentAutoStop(uint8_t phase, uint8_t abort_reason)
{
    g_pitch_down_ident_running = 0U;
    g_pitch_down_ident_enable = 0U;
    g_pitch_down_ident_auto_sample_active = 0U;
    g_pitch_down_ident_auto_phase = phase;
    g_pitch_down_ident_abort_reason = abort_reason;
    g_pitch_down_ident_auto_ref_vel_rad_s = 0.0f;
    g_pitch_down_ident_auto_ref_acc_rad_s2 = 0.0f;
    g_pitch_down_ident_auto_freq_hz = 0.0f;
    g_pitch_down_ident_auto_speed_target_rad_s = 0.0f;
    pitch_down_ident_chirp_time_s = 0.0f;
    pitch_down_ident_ref_raw_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;
    pitch_down_ident_ref_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;
    pitch_down_ident_ref_limit_active = 0U;
    PitchDownSetRefOverride(PITCH_DOWN_DEFAULT_REF_POS_RAD, 1U);

    if (phase == PITCH_DOWN_IDENT_AUTO_PHASE_DONE) {
        g_pitch_down_ident_done = 1U;
        g_pitch_down_ident_abort = 0U;
        g_pitch_down_ident_abort_reason = PITCH_DOWN_IDENT_AUTO_ABORT_NONE;
    } else {
        g_pitch_down_ident_done = 0U;
        g_pitch_down_ident_abort = 1U;
        if (g_pitch_down_ident_abort_reason == PITCH_DOWN_IDENT_AUTO_ABORT_NONE) {
            g_pitch_down_ident_abort_reason =
                PITCH_DOWN_IDENT_AUTO_ABORT_CANCEL;
        }
    }
}

static void PitchDownIdentAutoAbortIfActive(uint8_t abort_reason)
{
    if (PitchDownIdentAutoActive() != 0U) {
        PitchDownIdentAutoStop(PITCH_DOWN_IDENT_AUTO_PHASE_ABORT,
                               abort_reason);
    }
}

static void PitchDownIdentAutoStart(void)
{
    const uint8_t stage =
        PitchDownIdentAutoNormalizeStage(g_pitch_down_ident_stage_select);
    const float down_pos_now =
        (pitch_motor_down != NULL) ?
        pitch_motor_down->measure.pos : pitch_down_pos_rad;
    float start_ref_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;

    g_pitch_down_lqr_k_theta = PITCH_DOWN_LQR_K_THETA;
    g_pitch_down_lqr_k_omega = PITCH_DOWN_LQR_K_OMEGA;
    g_pitch_down_eso_enable = 0U;
    g_pitch_down_model_enable = 1U;

    g_pitch_down_ident_done = 0U;
    g_pitch_down_ident_abort = 0U;
    g_pitch_down_ident_abort_reason = PITCH_DOWN_IDENT_AUTO_ABORT_NONE;
    g_pitch_down_ident_running = 1U;
    g_pitch_down_ident_enable = 1U;
    g_pitch_down_ident_auto_time_s = 0.0f;
    g_pitch_down_ident_auto_stage = stage;
    g_pitch_down_ident_auto_profile =
        PitchDownIdentAutoStageProfile(stage);
    g_pitch_down_ident_auto_pass = 0U;
    g_pitch_down_ident_auto_point_index = 0U;
    pitch_down_ident_chirp_time_s = 0.0f;
    PitchDownResetRefTrajectory(down_pos_now);

    if (g_pitch_down_ident_auto_profile ==
        PITCH_DOWN_IDENT_AUTO_PROFILE_GRAVITY) {
        g_pitch_down_ident_auto_pass =
            (stage == PITCH_DOWN_IDENT_STAGE_GRAVITY_DOWN) ? 1U : 0U;
        g_pitch_down_ident_auto_target_rad =
            PitchDownIdentAutoGravityTarget(g_pitch_down_ident_auto_pass, 0U);
        PitchDownIdentAutoSetPhase(PITCH_DOWN_IDENT_AUTO_PHASE_G_SETTLE);
        PitchDownIdentAutoSetRef(g_pitch_down_ident_auto_target_rad,
                                 0.0f,
                                 0.0f,
                                 g_pitch_down_ident_auto_target_rad,
                                 0.0f,
                                 0.0f,
                                 0U);
        return;
    }

    if (g_pitch_down_ident_auto_profile ==
        PITCH_DOWN_IDENT_AUTO_PROFILE_FRICTION) {
        g_pitch_down_ident_auto_point_index =
            PitchDownIdentAutoStageFrictionIndex(stage);
        g_pitch_down_ident_auto_pass =
            PitchDownIdentAutoStageFrictionPass(stage);
        start_ref_rad = (g_pitch_down_ident_auto_pass == 0U) ?
                        PITCH_DOWN_IDENT_AUTO_F_LOW_RAD :
                        PITCH_DOWN_IDENT_AUTO_F_HIGH_RAD;
        pitch_down_ident_stage_prep_time_s =
            fabsf(start_ref_rad - down_pos_now) /
            PITCH_DOWN_MODEL_REF_VEL_LIMIT_RAD_S + 0.4f;
        pitch_down_ident_stage_prep_time_s =
            PitchDownClampRange(pitch_down_ident_stage_prep_time_s,
                                PITCH_DOWN_IDENT_AUTO_F_HOLD_TIME_S,
                                PITCH_DOWN_IDENT_AUTO_F_PREP_MAX_TIME_S);
        PitchDownIdentAutoSetPhase(PITCH_DOWN_IDENT_AUTO_PHASE_F_HOLD);
        PitchDownIdentAutoSetRef(start_ref_rad,
                                 0.0f,
                                 0.0f,
                                 start_ref_rad,
                                 0.0f,
                                 PitchDownIdentAutoFrictionSpeed(
                                     g_pitch_down_ident_auto_point_index),
                                 0U);
        return;
    }

    g_pitch_down_ident_auto_pass = 0U;
    g_pitch_down_ident_auto_point_index =
        (stage == PITCH_DOWN_IDENT_STAGE_CHIRP_HIGH) ? 1U : 0U;
    PitchDownIdentAutoSetPhase(PITCH_DOWN_IDENT_AUTO_PHASE_C_SETTLE);
    PitchDownIdentAutoSetRef(PITCH_DOWN_IDENT_AUTO_CHIRP_CENTER_RAD,
                             0.0f,
                             0.0f,
                             PITCH_DOWN_IDENT_AUTO_CHIRP_CENTER_RAD,
                             0.0f,
                             0.0f,
                             0U);
}

static void PitchDownIdentAutoAdvanceGravityPoint(void)
{
    g_pitch_down_ident_auto_point_index++;
    if (g_pitch_down_ident_auto_point_index >=
        PITCH_DOWN_IDENT_AUTO_G_POINT_COUNT) {
        PitchDownIdentAutoStop(PITCH_DOWN_IDENT_AUTO_PHASE_DONE,
                               PITCH_DOWN_IDENT_AUTO_ABORT_NONE);
        return;
    }

    g_pitch_down_ident_auto_target_rad =
        PitchDownIdentAutoGravityTarget(g_pitch_down_ident_auto_pass,
                                        g_pitch_down_ident_auto_point_index);
    PitchDownIdentAutoSetPhase(PITCH_DOWN_IDENT_AUTO_PHASE_G_SETTLE);
    PitchDownIdentAutoSetRef(g_pitch_down_ident_auto_target_rad,
                             0.0f,
                             0.0f,
                             g_pitch_down_ident_auto_target_rad,
                             0.0f,
                             0.0f,
                             0U);
}

static void PitchDownIdentAutoUpdate(float dt_s)
{
    float ref_rad;
    float target_rad;
    float speed_rad_s;

    if (dt_s < 0.0001f || dt_s > 0.05f) {
        dt_s = 0.001f;
    }

    if (g_pitch_down_ident_running != 0U &&
        g_pitch_down_ident_enable == 0U) {
        PitchDownIdentAutoStop(PITCH_DOWN_IDENT_AUTO_PHASE_ABORT,
                               PITCH_DOWN_IDENT_AUTO_ABORT_CANCEL);
        return;
    }

    if (g_pitch_down_ident_running == 0U) {
        if (g_pitch_down_ident_enable == 0U) {
            return;
        }
        if (PitchDownIdentAutoFeedbackOk() == 0U) {
            PitchDownIdentAutoStop(PITCH_DOWN_IDENT_AUTO_PHASE_ABORT,
                                   PITCH_DOWN_IDENT_AUTO_ABORT_FEEDBACK);
            return;
        }
        PitchDownIdentAutoStart();
    }

    if (PitchDownIdentAutoFeedbackOk() == 0U) {
        PitchDownIdentAutoStop(PITCH_DOWN_IDENT_AUTO_PHASE_ABORT,
                               PITCH_DOWN_IDENT_AUTO_ABORT_FEEDBACK);
        return;
    }

    g_pitch_down_lqr_k_theta = PITCH_DOWN_LQR_K_THETA;
    g_pitch_down_lqr_k_omega = PITCH_DOWN_LQR_K_OMEGA;
    g_pitch_down_eso_enable = 0U;
    g_pitch_down_model_enable = 1U;

    g_pitch_down_ident_auto_time_s += dt_s;
    g_pitch_down_ident_auto_phase_time_s += dt_s;

    switch (g_pitch_down_ident_auto_phase) {
    case PITCH_DOWN_IDENT_AUTO_PHASE_G_SETTLE:
        target_rad = g_pitch_down_ident_auto_target_rad;
        PitchDownIdentAutoSetRef(target_rad,
                                 0.0f,
                                 0.0f,
                                 target_rad,
                                 0.0f,
                                 0.0f,
                                 0U);
        if (g_pitch_down_ident_auto_phase_time_s >=
            PITCH_DOWN_IDENT_AUTO_G_SETTLE_TIME_S) {
            PitchDownIdentAutoSetPhase(PITCH_DOWN_IDENT_AUTO_PHASE_G_SAMPLE);
        }
        break;

    case PITCH_DOWN_IDENT_AUTO_PHASE_G_SAMPLE:
        target_rad = g_pitch_down_ident_auto_target_rad;
        if (g_pitch_down_ident_auto_phase_time_s >=
            PITCH_DOWN_IDENT_AUTO_G_SAMPLE_TIME_S) {
            PitchDownIdentAutoAdvanceGravityPoint();
        } else {
            PitchDownIdentAutoSetRef(target_rad,
                                     0.0f,
                                     0.0f,
                                     target_rad,
                                     0.0f,
                                     0.0f,
                                     1U);
        }
        break;

    case PITCH_DOWN_IDENT_AUTO_PHASE_F_HOLD:
        target_rad = (g_pitch_down_ident_auto_pass == 0U) ?
                     PITCH_DOWN_IDENT_AUTO_F_LOW_RAD :
                     PITCH_DOWN_IDENT_AUTO_F_HIGH_RAD;
        PitchDownIdentAutoSetRef(target_rad,
                                 0.0f,
                                 0.0f,
                                 target_rad,
                                 0.0f,
                                 PitchDownIdentAutoFrictionSpeed(
                                     g_pitch_down_ident_auto_point_index),
                                 0U);
        if (g_pitch_down_ident_auto_phase_time_s >=
            pitch_down_ident_stage_prep_time_s) {
            PitchDownIdentAutoSetPhase(PITCH_DOWN_IDENT_AUTO_PHASE_F_RUN);
        }
        break;

    case PITCH_DOWN_IDENT_AUTO_PHASE_F_RUN:
        speed_rad_s =
            PitchDownIdentAutoFrictionSpeed(g_pitch_down_ident_auto_point_index);
        if (g_pitch_down_ident_auto_pass == 0U) {
            target_rad = PITCH_DOWN_IDENT_AUTO_F_HIGH_RAD;
            ref_rad = g_pitch_down_ident_auto_ref_rad + speed_rad_s * dt_s;
            if (ref_rad >= target_rad) {
                PitchDownIdentAutoSetRef(target_rad,
                                         0.0f,
                                         0.0f,
                                         target_rad,
                                         0.0f,
                                         speed_rad_s,
                                         0U);
                PitchDownIdentAutoStop(PITCH_DOWN_IDENT_AUTO_PHASE_DONE,
                                       PITCH_DOWN_IDENT_AUTO_ABORT_NONE);
            } else {
                PitchDownIdentAutoSetRef(ref_rad,
                                         speed_rad_s,
                                         0.0f,
                                         target_rad,
                                         0.0f,
                                         speed_rad_s,
                                         1U);
            }
        } else {
            target_rad = PITCH_DOWN_IDENT_AUTO_F_LOW_RAD;
            ref_rad = g_pitch_down_ident_auto_ref_rad - speed_rad_s * dt_s;
            if (ref_rad <= target_rad) {
                PitchDownIdentAutoSetRef(target_rad,
                                         0.0f,
                                         0.0f,
                                         target_rad,
                                         0.0f,
                                         speed_rad_s,
                                         0U);
                PitchDownIdentAutoStop(PITCH_DOWN_IDENT_AUTO_PHASE_DONE,
                                       PITCH_DOWN_IDENT_AUTO_ABORT_NONE);
            } else {
                PitchDownIdentAutoSetRef(ref_rad,
                                         -speed_rad_s,
                                         0.0f,
                                         target_rad,
                                         0.0f,
                                         speed_rad_s,
                                         1U);
            }
        }
        break;

    case PITCH_DOWN_IDENT_AUTO_PHASE_C_SETTLE:
        PitchDownIdentAutoSetRef(PITCH_DOWN_IDENT_AUTO_CHIRP_CENTER_RAD,
                                 0.0f,
                                 0.0f,
                                 PITCH_DOWN_IDENT_AUTO_CHIRP_CENTER_RAD,
                                 0.0f,
                                 0.0f,
                                 0U);
        if (g_pitch_down_ident_auto_phase_time_s >=
            PITCH_DOWN_IDENT_AUTO_CHIRP_SETTLE_S) {
            pitch_down_ident_chirp_time_s = 0.0f;
            PitchDownIdentAutoSetPhase(PITCH_DOWN_IDENT_AUTO_PHASE_C_RUN);
        }
        break;

    case PITCH_DOWN_IDENT_AUTO_PHASE_C_RUN:
    {
        const float duration_s = PITCH_DOWN_IDENT_STAGE_CHIRP_DURATION_S;
        const float f_start_hz =
            PitchDownIdentAutoStageChirpStartHz(
                g_pitch_down_ident_auto_stage);
        const float f_end_hz =
            PitchDownIdentAutoStageChirpEndHz(
                g_pitch_down_ident_auto_stage);
        const float k_hz_s =
            (f_end_hz - f_start_hz) / duration_s;
        const float t_s = pitch_down_ident_chirp_time_s;
        const float freq_hz = f_start_hz + k_hz_s * t_s;
        const float phase =
            PITCH_UP_DYN_TWO_PI *
            (f_start_hz * t_s +
             0.5f * k_hz_s * t_s * t_s);
        const float phase_dot = PITCH_UP_DYN_TWO_PI * freq_hz;
        const float phase_ddot = PITCH_UP_DYN_TWO_PI * k_hz_s;
        const float amp = PITCH_DOWN_IDENT_AUTO_CHIRP_AMP_RAD;
        const float ref_vel = amp * cosf(phase) * phase_dot;
        const float ref_acc =
            -amp * sinf(phase) * phase_dot * phase_dot +
            amp * cosf(phase) * phase_ddot;

        ref_rad = PITCH_DOWN_IDENT_AUTO_CHIRP_CENTER_RAD +
                  amp * sinf(phase);
        PitchDownIdentAutoSetRef(ref_rad,
                                 ref_vel,
                                 ref_acc,
                                 PITCH_DOWN_IDENT_AUTO_CHIRP_CENTER_RAD,
                                 freq_hz,
                                 0.0f,
                                 1U);
        pitch_down_ident_chirp_time_s += dt_s;
        if (pitch_down_ident_chirp_time_s >= duration_s) {
            PitchDownIdentAutoStop(PITCH_DOWN_IDENT_AUTO_PHASE_DONE,
                                   PITCH_DOWN_IDENT_AUTO_ABORT_NONE);
        }
        break;
    }

    default:
        PitchDownIdentAutoStop(PITCH_DOWN_IDENT_AUTO_PHASE_ABORT,
                               PITCH_DOWN_IDENT_AUTO_ABORT_MODE);
        break;
    }
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

static uint8_t PitchUpShouldUseAutoRef(void)
{
    if (pitch_up_ref_override_active != 0U) {
        return 0U;
    }
    return GimbalAutoAimCommandActive();
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
    pitch_down_ref_target_limited_pos_rad = pitch_down_ref_pos_rad;
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
        pitch_down_friction_torque_ff_nm = 0.0f;
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
    pitch_down_ref_target_limited_pos_rad = pitch_down_ref_pos_rad;
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
        pitch_down_friction_torque_ff_nm = 0.0f;
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
    PitchDownSetTorqueImmediate(0.0f);

    pitch_down_torque_cmd_nm = 0.0f;
    pitch_down_gravity_torque_ff_nm = 0.0f;
    pitch_down_friction_torque_ff_nm = 0.0f;
    PitchDownResetModelFeedforwardDebug();
    pitch_down_soft_limit_active = 0U;

    PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    PitchDownClearLQRDebug();
    PitchDownClearGravityDebug();
    PitchDownClearVelocityFilter();
    pitch_down_ref_traj_primed = 0U;
    pitch_down_ref_traj_rad = pitch_down_ref_limited_pos_rad;
    pitch_down_ref_vel_rad_s = 0.0f;
    pitch_down_ref_acc_rad_s2 = 0.0f;
    GimbalSetAutoAimActive(0U);
    pitch_down_auto_ref_last_active = 0U;
    PitchDownResetHoldIntegral();
}

static void PitchDownUpdateIdentDebug(float now_s, float dt_s)
{
    PitchDownIdentDebug_s debug = {0};
    const uint8_t down_ident_mode_active =
        (PitchDownIdentAutoActive() != 0U) ? 1U : 0U;
    const uint8_t feedback_ok =
        (pitch_motor_down != NULL &&
         pitch_motor_up != NULL &&
         gimbal_IMU_data != NULL &&
         DMMotorIsOnline(pitch_motor_down) != 0U &&
         DMMotorIsOnline(pitch_motor_up) != 0U) ? 1U : 0U;
    const float up_hold_error_rad =
        pitch_up_measure_rad - pitch_up_ref_rad;
    const uint8_t up_hold_ok =
        (fabsf(up_hold_error_rad) <= PITCH_DOWN_IDENT_UP_HOLD_VALID_RAD) ?
        1U : 0U;

    debug.sample_count = g_pitch_down_ident_debug.sample_count + 1U;
    debug.time_s = now_s;
    debug.dt_s = dt_s;
    debug.valid = (g_pitch_down_ident_auto_sample_active != 0U &&
                   feedback_ok != 0U &&
                   down_ident_mode_active != 0U &&
                   pitch_down_ident_ref_limit_active == 0U &&
                   pitch_down_lqr_limit_active == 0U &&
                   pitch_down_eso_runtime_enable == 0U &&
                   up_hold_ok != 0U) ? 1U : 0U;
    debug.gimbal_mode = (uint8_t)gimbal_cmd_recv.gimbal_mode;
    debug.nuc_mode = (uint8_t)gimbal_cmd_recv.nuc_mode;
    debug.feedback_ok = feedback_ok;
    debug.down_ident_mode_active = down_ident_mode_active;
    debug.auto_running = g_pitch_down_ident_running;
    debug.auto_done = g_pitch_down_ident_done;
    debug.auto_abort = g_pitch_down_ident_abort;
    debug.auto_abort_reason = g_pitch_down_ident_abort_reason;
    debug.auto_phase = g_pitch_down_ident_auto_phase;
    debug.auto_profile = g_pitch_down_ident_auto_profile;
    debug.auto_stage = g_pitch_down_ident_auto_stage;
    debug.auto_pass = g_pitch_down_ident_auto_pass;
    debug.auto_point_index = g_pitch_down_ident_auto_point_index;
    debug.auto_sample_active = g_pitch_down_ident_auto_sample_active;
    debug.cmd_pitch_rad = gimbal_cmd_recv.pitch;
    debug.down_ref_raw_rad = pitch_down_ident_ref_raw_rad;
    debug.down_ref_rad = pitch_down_ref_traj_rad;
    debug.down_ref_vel_rad_s = pitch_down_ref_vel_rad_s;
    debug.down_ref_acc_rad_s2 = pitch_down_ref_acc_rad_s2;
    debug.down_ref_limit_active = pitch_down_ident_ref_limit_active;
    debug.down_pos_rad = pitch_down_pos_rad;
    debug.down_vel_raw_rad_s = pitch_down_vel_raw_rad_s;
    debug.down_vel_lpf_rad_s = pitch_down_vel_rad_s;
    debug.down_pos_error_rad = pitch_down_pos_error_rad;
    debug.down_tau_cmd_nm = pitch_down_torque_cmd_nm;
    debug.down_tau_lqr_nm = pitch_down_lqr_tau_raw_nm;
    debug.down_tau_pre_limit_nm = pitch_down_lqr_pre_limit_torque_nm;
    debug.down_tau_output_nm = pitch_down_lqr_output_nm;
    debug.down_tau_limit_active = pitch_down_lqr_limit_active;
    debug.down_eso_runtime_enable = pitch_down_eso_runtime_enable;
    debug.down_eso_enable = dzx_pitch_down_lqr_eso.eso_config.enable;
    debug.down_eso_alpha = dzx_pitch_down_lqr_eso.eso_config.alpha;
    debug.down_eso_w0_rad_s = dzx_pitch_down_lqr_eso.eso.w0;
    debug.down_eso_comp_gain = dzx_pitch_down_lqr_eso.eso_config.comp_gain;
    debug.down_eso_comp_limit_nm =
        dzx_pitch_down_lqr_eso.eso_config.comp_limit;
    debug.down_tau_eso_comp_nm = dzx_pitch_down_lqr_eso.tau_eso_comp;
    debug.down_feedforward_nm = pitch_down_feedforward_nm;
    debug.down_model_enable = g_pitch_down_model_enable;
    debug.down_model_g0_nm = PITCH_DOWN_MODEL_G0_NM;
    debug.down_model_gs_nm = PITCH_DOWN_MODEL_GS_NM;
    debug.down_model_gc_nm = PITCH_DOWN_MODEL_GC_NM;
    debug.down_model_j_kg_m2 = g_pitch_down_model_j_kg_m2;
    debug.down_model_b_nms_rad = g_pitch_down_model_b_nms_rad;
    debug.down_model_tau_c_param_nm = g_pitch_down_model_tau_c_nm;
    debug.down_model_omega_eps_rad_s = g_pitch_down_model_omega_eps_rad_s;
    debug.down_model_tau_g_nm = pitch_down_model_tau_g_nm;
    debug.down_model_tau_j_nm = pitch_down_model_tau_j_nm;
    debug.down_model_tau_b_nm = pitch_down_model_tau_b_nm;
    debug.down_model_tau_c_nm = pitch_down_model_tau_c_nm;
    debug.down_model_tau_ff_nm = pitch_down_model_tau_ff_nm;
    debug.down_tau_no_model_nm =
        debug.down_tau_cmd_nm - debug.down_model_tau_ff_nm;
    debug.down_k_theta_used = dzx_pitch_down_lqr_eso.k_theta_used;
    debug.down_k_omega_used = dzx_pitch_down_lqr_eso.k_omega_used;
    debug.up_hold_ref_rad = pitch_up_ref_rad;
    debug.up_imu_rad = pitch_up_measure_rad;
    debug.up_gyro_rad_s = pitch_up_gyro_rad_s;
    debug.up_hold_error_rad = up_hold_error_rad;
    debug.up_tau_cmd_nm = pitch_up_torque_cmd_nm;
    debug.auto_time_s = g_pitch_down_ident_auto_time_s;
    debug.auto_phase_time_s = g_pitch_down_ident_auto_phase_time_s;
    debug.auto_target_rad = g_pitch_down_ident_auto_target_rad;
    debug.auto_ref_rad = g_pitch_down_ident_auto_ref_rad;
    debug.auto_ref_vel_rad_s = g_pitch_down_ident_auto_ref_vel_rad_s;
    debug.auto_ref_acc_rad_s2 = g_pitch_down_ident_auto_ref_acc_rad_s2;
    debug.auto_freq_hz = g_pitch_down_ident_auto_freq_hz;
    debug.auto_speed_target_rad_s =
        g_pitch_down_ident_auto_speed_target_rad_s;

    if (pitch_motor_down != NULL) {
        debug.down_tau_meas_nm = pitch_motor_down->measure.tor;
        debug.down_ctrl_tor_set_nm = pitch_motor_down->ctrl.tor_set;
    }

    g_pitch_down_ident_debug = debug;
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

    PitchDownUpdateIdentDebug(now_s, pitch_down_debug_csv.debug_dt_s);
}

static void PitchUpResetModelFeedforwardDebug(void)
{
    pitch_up_model_tau_g_q_nm = 0.0f;
    pitch_up_model_tau_j_q_nm = 0.0f;
    pitch_up_model_tau_b_q_nm = 0.0f;
    pitch_up_model_tau_c_q_nm = 0.0f;
    pitch_up_model_tau_q_ff_nm = 0.0f;
    pitch_up_model_tau_motor_ff_nm = 0.0f;
}

static uint8_t PitchUpModelControlActive(void)
{
#if PITCH_UP_MODEL_LQR_ENABLE
    if (g_pitch_up_model_enable == 0U) {
        return 0U;
    }
    return (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE ||
            gimbal_cmd_recv.gimbal_mode == GIMBAL_MOTOR_MODE) ? 1U : 0U;
#else
    return 0U;
#endif
}

static float PitchUpCalculateModelFeedforwardMotor(void)
{
    float omega_eps;
    if (PitchUpModelControlActive() == 0U) {
        PitchUpResetModelFeedforwardDebug();
        return 0.0f;
    }

    omega_eps = g_pitch_up_model_omega_eps_rad_s;
    if (omega_eps <= 0.0f) {
        omega_eps = PITCH_UP_MODEL_OMEGA_EPS_RAD_S;
    }

    pitch_up_model_tau_g_q_nm =
        PITCH_UP_MODEL_GS_Q_NM * sinf(pitch_up_measure_rad) +
        PITCH_UP_MODEL_GC_Q_NM * cosf(pitch_up_measure_rad);
    pitch_up_model_tau_j_q_nm =
        g_pitch_up_model_j_kg_m2 * pitch_up_ref_acc_rad_s2;
    pitch_up_model_tau_b_q_nm =
        g_pitch_up_model_b_nms_rad * pitch_up_gyro_rad_s;
    pitch_up_model_tau_c_q_nm =
        g_pitch_up_model_tau_c_nm *
        tanhf(pitch_up_gyro_rad_s / omega_eps);

    pitch_up_model_tau_q_ff_nm =
        pitch_up_model_tau_g_q_nm +
        pitch_up_model_tau_j_q_nm +
        pitch_up_model_tau_b_q_nm +
        pitch_up_model_tau_c_q_nm;
    pitch_up_model_tau_motor_ff_nm = -pitch_up_model_tau_q_ff_nm;
    return pitch_up_model_tau_motor_ff_nm;
}

static void PitchUpUpdateIdentDebug(float dt_s,
                                    uint8_t valid,
                                    uint8_t feedback_ok,
                                    uint8_t torque_output_allowed,
                                    const DoublePitchLqrInput_s *input,
                                    const DoublePitchLqrOutput_s *output,
                                    float up_tau_motor_nm)
{
    PitchUpIdentDebug_s debug = {0};
    const uint8_t lqr_limit_active =
        (output != NULL) ?
        output->tau_small_limit_flag :
        ((fabsf(dzx_pitch_lqr_eso.tau_pre_limit -
                dzx_pitch_lqr_eso.output) > 1e-6f) ? 1U : 0U);
    const float up_tau_cmd_nm =
        (output != NULL) ?
        output->tau_small_cmd_nm :
        (PITCH_UP_CONTROL_TO_MOTOR_TORQUE_SIGN * up_tau_motor_nm);
    const uint8_t ref_source_valid =
        (pitch_up_ref_source == PITCH_UP_REF_SOURCE_CMD ||
         pitch_up_ref_source == PITCH_UP_REF_SOURCE_VISION) ? 1U : 0U;
    const uint8_t ident_auto_active =
        (g_pitch_up_ident_running != 0U) ? 1U : 0U;
    const float debug_down_pos_rad =
        (input != NULL) ?
        input->theta_rad :
        ((pitch_motor_down != NULL) ?
         pitch_motor_down->measure.pos : pitch_down_pos_rad);
    const float debug_down_vel_rad_s =
        (input != NULL) ?
        input->omega_big_rad_s :
        ((pitch_motor_down != NULL) ?
         pitch_motor_down->measure.vel : pitch_down_vel_rad_s);
    const float debug_up_imu_rad =
        (input != NULL) ? input->psi_rad : pitch_up_measure_rad;
    const float debug_phi_rel_rad =
        (input != NULL) ?
        input->phi_rad :
        PitchWrapPiRad(debug_up_imu_rad - debug_down_pos_rad);
    const float debug_omega_small_rad_s =
        (input != NULL) ?
        input->omega_small_rad_s :
        (pitch_up_gyro_rad_s - debug_down_vel_rad_s);
    const float debug_up_motor_tor_nm =
        (pitch_motor_up != NULL) ?
        pitch_motor_up->measure.tor :
        ((input != NULL) ? input->tau_small_meas_nm : 0.0f);
    const uint8_t ident_auto_valid =
        (valid != 0U &&
         ident_auto_active != 0U &&
         g_pitch_up_ident_auto_sample_active != 0U &&
         feedback_ok != 0U &&
         torque_output_allowed != 0U &&
         pitch_up_ref_limit_active == 0U &&
         pitch_up_soft_limit_active == 0U &&
         lqr_limit_active == 0U &&
         PitchUpModelControlActive() != 0U) ? 1U : 0U;
    const uint8_t manual_model_valid =
        (valid != 0U &&
         feedback_ok != 0U &&
         torque_output_allowed != 0U &&
         ref_source_valid != 0U &&
         pitch_up_ref_limit_active == 0U &&
         pitch_up_soft_limit_active == 0U &&
         lqr_limit_active == 0U &&
         pitch_up_ref_override_active == 0U &&
         g_pitch_up_ident_running == 0U &&
         g_pitch_up_ident_enable == 0U &&
         g_pitch_up_g_auto_running == 0U &&
         g_pitch_up_dyn_auto_running == 0U) ? 1U : 0U;

    debug.sample_count = g_pitch_up_ident_debug.sample_count + 1U;
    debug.time_s = DWT_GetTimeline_s();
    debug.dt_s = dt_s;
    debug.valid = (ident_auto_active != 0U) ?
                  ident_auto_valid : manual_model_valid;
    debug.feedback_ok = feedback_ok;
    debug.torque_output_allowed = torque_output_allowed;
    debug.auto_stage = g_pitch_up_ident_auto_stage;
    debug.auto_pass = g_pitch_up_ident_auto_pass;
    debug.auto_point_index = g_pitch_up_ident_auto_point_index;
    debug.auto_sample_active = g_pitch_up_ident_auto_sample_active;
    debug.auto_ref_active =
        (pitch_up_auto_ref_active != 0U ||
         pitch_up_ref_override_active != 0U ||
         ident_auto_active != 0U) ? 1U : 0U;
    debug.auto_freq_hz = g_pitch_up_ident_auto_freq_hz;
    debug.down_pos_rad = debug_down_pos_rad;
    debug.down_vel_rad_s = debug_down_vel_rad_s;
    debug.phi_rel_rad = debug_phi_rel_rad;
    debug.omega_small_rad_s = debug_omega_small_rad_s;
    debug.up_tau_cmd_nm = up_tau_cmd_nm;
    debug.up_motor_tor_nm = debug_up_motor_tor_nm;

    g_pitch_up_ident_debug = debug;
}

static uint8_t DoublePitchIdentNormalModeActive(void)
{
    return (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE ||
            gimbal_cmd_recv.gimbal_mode == GIMBAL_MOTOR_MODE) ? 1U : 0U;
}

static void DoublePitchIdentPublishRefs(float theta_ref_rad,
                                        float phi_ref_rad,
                                        float theta_ref_dot_rad_s,
                                        float phi_ref_dot_rad_s,
                                        float theta_ref_ddot_rad_s2,
                                        float phi_ref_ddot_rad_s2,
                                        float time_s,
                                        float freq_hz)
{
    g_double_pitch_ident_theta_ref_rad = theta_ref_rad;
    g_double_pitch_ident_phi_ref_rad = PitchWrapPiRad(phi_ref_rad);
    g_double_pitch_ident_psi_ref_rad =
        PitchWrapPiRad(theta_ref_rad + g_double_pitch_ident_phi_ref_rad);
    g_double_pitch_ident_theta_ref_dot_rad_s = theta_ref_dot_rad_s;
    g_double_pitch_ident_phi_ref_dot_rad_s = phi_ref_dot_rad_s;
    g_double_pitch_ident_psi_ref_dot_rad_s =
        theta_ref_dot_rad_s + phi_ref_dot_rad_s;
    g_double_pitch_ident_theta_ref_ddot_rad_s2 = theta_ref_ddot_rad_s2;
    g_double_pitch_ident_phi_ref_ddot_rad_s2 = phi_ref_ddot_rad_s2;
    g_double_pitch_ident_psi_ref_ddot_rad_s2 =
        theta_ref_ddot_rad_s2 + phi_ref_ddot_rad_s2;
    g_double_pitch_ident_time_s = time_s;
    g_double_pitch_ident_inst_freq_hz = freq_hz;
}

static void DoublePitchIdentStop(void)
{
    g_double_pitch_ident_running = 0U;
    g_double_pitch_static_phase = DOUBLE_PITCH_STATIC_PHASE_IDLE;
    g_double_pitch_static_active_index = 0U;
    DoublePitchIdentPublishRefs(g_double_pitch_ident_theta_center_rad,
                                g_double_pitch_ident_phi_center_rad,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f);
}

static float DoublePitchIdentStepLinear(float current,
                                        float target,
                                        float max_step)
{
    const float error = target - current;

    if (error > max_step) {
        return current + max_step;
    }
    if (error < -max_step) {
        return current - max_step;
    }
    return target;
}

static float DoublePitchIdentStepWrapped(float current,
                                         float target,
                                         float max_step)
{
    const float error = PitchWrapPiRad(target - current);

    if (error > max_step) {
        return PitchWrapPiRad(current + max_step);
    }
    if (error < -max_step) {
        return PitchWrapPiRad(current - max_step);
    }
    return PitchWrapPiRad(target);
}

static uint8_t DoublePitchIdentStaticSampleActive(void)
{
    return (g_double_pitch_static_point_index != 0U &&
            g_double_pitch_static_phase == DOUBLE_PITCH_STATIC_PHASE_SAMPLE) ?
           1U : 0U;
}

static uint8_t DoublePitchIdentStaticRequestActive(void)
{
    return (g_double_pitch_static_point_index != 0U) ? 1U : 0U;
}

static void DoublePitchIdentApplyStaticPointSelection(float dt_s)
{
    static float ref_theta = PITCH_DOWN_DEFAULT_REF_POS_RAD;
    static float ref_phi =
        (PITCH_UP_IMU_ZERO_REF_RAD - PITCH_DOWN_DEFAULT_REF_POS_RAD);
    static float settle_time_s = 0.0f;
    static float sample_time_s = 0.0f;
    static float saved_theta_center = PITCH_DOWN_DEFAULT_REF_POS_RAD;
    static float saved_phi_center =
        (PITCH_UP_IMU_ZERO_REF_RAD - PITCH_DOWN_DEFAULT_REF_POS_RAD);
    static float saved_duration_s = 20.0f;
    static float saved_amp_rad = 0.04f;
    static uint8_t saved_axis = DOUBLE_PITCH_IDENT_AXIS_THETA;
    static uint8_t saved_valid = 0U;
    const DoublePitchStaticPoint_s *point;
    float dt;
    float max_step;
    float phi_measure;
    float theta_error;
    float phi_error;
    float omega_small;
    uint8_t ready;
    uint8_t index = g_double_pitch_static_point_index;

    if (index == 0U) {
        g_double_pitch_static_phase = DOUBLE_PITCH_STATIC_PHASE_IDLE;
        g_double_pitch_static_active_index = 0U;
        g_double_pitch_ident_arm = 0U;
        g_double_pitch_ident_running = 0U;
        if (saved_valid != 0U) {
            g_double_pitch_ident_theta_center_rad = saved_theta_center;
            g_double_pitch_ident_phi_center_rad = saved_phi_center;
            g_double_pitch_ident_duration_s = saved_duration_s;
            g_double_pitch_ident_amp_rad = saved_amp_rad;
            g_double_pitch_ident_axis = saved_axis;
            saved_valid = 0U;
        }
        settle_time_s = 0.0f;
        sample_time_s = 0.0f;
        return;
    }
    if (index > DOUBLE_PITCH_STATIC_POINT_COUNT ||
        DoublePitchIdentNormalModeActive() == 0U) {
        g_double_pitch_static_phase = DOUBLE_PITCH_STATIC_PHASE_IDLE;
        g_double_pitch_static_active_index = 0U;
        g_double_pitch_ident_arm = 0U;
        g_double_pitch_ident_running = 0U;
        if (saved_valid != 0U) {
            g_double_pitch_ident_theta_center_rad = saved_theta_center;
            g_double_pitch_ident_phi_center_rad = saved_phi_center;
            g_double_pitch_ident_duration_s = saved_duration_s;
            g_double_pitch_ident_amp_rad = saved_amp_rad;
            g_double_pitch_ident_axis = saved_axis;
            saved_valid = 0U;
        }
        settle_time_s = 0.0f;
        sample_time_s = 0.0f;
        return;
    }

    point = &double_pitch_static_points[index - 1U];
    dt = PitchDownClampRange(dt_s,
                             PITCH_DOWN_CONTROL_MIN_DT_S,
                             PITCH_DOWN_CONTROL_MAX_DT_S);
    max_step = DOUBLE_PITCH_STATIC_MOVE_RATE_RAD_S * dt;

    if (g_double_pitch_static_active_index != index ||
        g_double_pitch_static_phase == DOUBLE_PITCH_STATIC_PHASE_IDLE) {
        if (saved_valid == 0U) {
            saved_theta_center = g_double_pitch_ident_theta_center_rad;
            saved_phi_center = g_double_pitch_ident_phi_center_rad;
            saved_duration_s = g_double_pitch_ident_duration_s;
            saved_amp_rad = g_double_pitch_ident_amp_rad;
            saved_axis = g_double_pitch_ident_axis;
            saved_valid = 1U;
        }
        ref_theta = pitch_down_pos_rad;
        ref_phi = PitchWrapPiRad(pitch_up_measure_rad - pitch_down_pos_rad);
        settle_time_s = 0.0f;
        sample_time_s = 0.0f;
        g_double_pitch_static_active_index = index;
        g_double_pitch_static_phase = DOUBLE_PITCH_STATIC_PHASE_MOVE;
    }

    ref_theta = DoublePitchIdentStepLinear(ref_theta,
                                           point->theta_rad,
                                           max_step);
    ref_phi = DoublePitchIdentStepWrapped(ref_phi,
                                          point->phi_rad,
                                          max_step);
    g_double_pitch_ident_theta_center_rad = ref_theta;
    g_double_pitch_ident_phi_center_rad = ref_phi;
    g_double_pitch_ident_amp_rad = 0.0f;
    g_double_pitch_ident_duration_s = DOUBLE_PITCH_STATIC_POINT_DURATION_S;
    g_double_pitch_ident_axis = DOUBLE_PITCH_IDENT_AXIS_THETA;
    g_double_pitch_ident_arm = 1U;
    g_double_pitch_ident_running =
        (g_double_pitch_static_phase == DOUBLE_PITCH_STATIC_PHASE_SAMPLE) ?
        1U : 0U;

    DoublePitchIdentPublishRefs(ref_theta,
                                ref_phi,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                sample_time_s,
                                0.0f);
    PitchDownSetRefOverride(g_double_pitch_ident_theta_ref_rad, 0U);
    PitchUpSetRefOverride(g_double_pitch_ident_psi_ref_rad);

    phi_measure = PitchWrapPiRad(pitch_up_measure_rad - pitch_down_pos_rad);
    theta_error = point->theta_rad - pitch_down_pos_rad;
    phi_error = PitchWrapPiRad(point->phi_rad - phi_measure);
    omega_small = pitch_up_gyro_rad_s - pitch_down_vel_rad_s;
    ready = (fabsf(theta_error) <= DOUBLE_PITCH_STATIC_READY_POS_RAD &&
             fabsf(phi_error) <= DOUBLE_PITCH_STATIC_READY_POS_RAD &&
             fabsf(pitch_down_vel_rad_s) <= DOUBLE_PITCH_STATIC_READY_SPEED_RAD_S &&
             fabsf(omega_small) <= DOUBLE_PITCH_STATIC_READY_SPEED_RAD_S &&
             g_double_pitch_ident_debug.tau_big_limit_flag == 0U &&
             g_double_pitch_ident_debug.tau_small_limit_flag == 0U) ? 1U : 0U;

    if (g_double_pitch_static_phase == DOUBLE_PITCH_STATIC_PHASE_MOVE &&
        ref_theta == point->theta_rad &&
        ref_phi == PitchWrapPiRad(point->phi_rad)) {
        g_double_pitch_static_phase = DOUBLE_PITCH_STATIC_PHASE_SETTLE;
        settle_time_s = 0.0f;
    }

    if (g_double_pitch_static_phase == DOUBLE_PITCH_STATIC_PHASE_SETTLE) {
        if (ready != 0U) {
            settle_time_s += dt;
        } else {
            settle_time_s = 0.0f;
        }
        if (settle_time_s >= DOUBLE_PITCH_STATIC_SETTLE_TIME_S) {
            g_double_pitch_static_phase = DOUBLE_PITCH_STATIC_PHASE_SAMPLE;
            g_double_pitch_ident_running = 1U;
            sample_time_s = 0.0f;
        }
    } else if (g_double_pitch_static_phase == DOUBLE_PITCH_STATIC_PHASE_SAMPLE) {
        sample_time_s += dt;
        if (sample_time_s >= DOUBLE_PITCH_STATIC_POINT_DURATION_S) {
            g_double_pitch_static_phase = DOUBLE_PITCH_STATIC_PHASE_DONE;
            g_double_pitch_ident_running = 0U;
            g_double_pitch_ident_arm = 0U;
        }
    } else if (g_double_pitch_static_phase == DOUBLE_PITCH_STATIC_PHASE_DONE) {
        g_double_pitch_ident_running = 0U;
        g_double_pitch_ident_arm = 0U;
    }
}

static uint8_t DoublePitchIdentApplyNormalRef(float dt_s)
{
    static float sweep_time_s = 0.0f;
    const float duration_s = g_double_pitch_ident_duration_s;
    const float f0 = g_double_pitch_ident_f_start_hz;
    const float f1 = g_double_pitch_ident_f_end_hz;
    const uint8_t axis = g_double_pitch_ident_axis;
    float amp = fabsf(g_double_pitch_ident_amp_rad);
    float dt;
    float k;
    float freq_hz;
    float phase;
    float phase_dot;
    float phase_ddot;
    float ref_delta;
    float ref_delta_dot;
    float ref_delta_ddot;
    float theta_ref;
    float phi_ref;
    float theta_ref_dot = 0.0f;
    float phi_ref_dot = 0.0f;
    float theta_ref_ddot = 0.0f;
    float phi_ref_ddot = 0.0f;

    if (DoublePitchIdentNormalModeActive() == 0U) {
        DoublePitchIdentStop();
        return 0U;
    }

    if (g_double_pitch_ident_arm == 0U ||
        duration_s <= 0.0f ||
        (axis != DOUBLE_PITCH_IDENT_AXIS_THETA &&
         axis != DOUBLE_PITCH_IDENT_AXIS_PHI)) {
        sweep_time_s = 0.0f;
        DoublePitchIdentStop();
        return 0U;
    }

    if (g_double_pitch_ident_running == 0U) {
        sweep_time_s = 0.0f;
        g_double_pitch_ident_running = 1U;
    }

    if (sweep_time_s >= duration_s) {
        g_double_pitch_ident_arm = 0U;
        sweep_time_s = 0.0f;
        DoublePitchIdentStop();
        return 0U;
    }

    if (amp > DOUBLE_PITCH_IDENT_AMP_LIMIT_RAD) {
        amp = DOUBLE_PITCH_IDENT_AMP_LIMIT_RAD;
    }

    k = (f1 - f0) / duration_s;
    freq_hz = f0 + k * sweep_time_s;
    phase = 6.283185307179586f *
            (f0 * sweep_time_s + 0.5f * k * sweep_time_s * sweep_time_s);
    phase_dot = 6.283185307179586f * freq_hz;
    phase_ddot = 6.283185307179586f * k;
    ref_delta = amp * sinf(phase);
    ref_delta_dot = amp * cosf(phase) * phase_dot;
    ref_delta_ddot = amp *
                     (cosf(phase) * phase_ddot -
                      sinf(phase) * phase_dot * phase_dot);

    theta_ref = g_double_pitch_ident_theta_center_rad;
    phi_ref = g_double_pitch_ident_phi_center_rad;
    if (axis == DOUBLE_PITCH_IDENT_AXIS_THETA) {
        theta_ref += ref_delta;
        theta_ref_dot = ref_delta_dot;
        theta_ref_ddot = ref_delta_ddot;
    } else {
        phi_ref += ref_delta;
        phi_ref_dot = ref_delta_dot;
        phi_ref_ddot = ref_delta_ddot;
    }

    DoublePitchIdentPublishRefs(theta_ref,
                                phi_ref,
                                theta_ref_dot,
                                phi_ref_dot,
                                theta_ref_ddot,
                                phi_ref_ddot,
                                sweep_time_s,
                                freq_hz);
    PitchDownSetRefOverride(g_double_pitch_ident_theta_ref_rad, 0U);
    PitchUpSetRefOverride(g_double_pitch_ident_psi_ref_rad);

    dt = PitchDownClampRange(dt_s,
                             PITCH_DOWN_CONTROL_MIN_DT_S,
                             PITCH_DOWN_CONTROL_MAX_DT_S);
    sweep_time_s += dt;
    return 1U;
}

static uint8_t DoublePitchGravityRampRequestActive(void)
{
    return (g_double_pitch_gravity_ramp_auto_start != 0U ||
            g_double_pitch_gravity_step_arm != 0U ||
            g_double_pitch_gravity_ramp_arm != 0U ||
            g_double_pitch_gravity_ramp_running != 0U) ? 1U : 0U;
}

static uint8_t DoublePitchDirectDownTorqueActive(void)
{
    return (g_double_pitch_direct_down_torque_arm != 0U) ? 1U : 0U;
}

static uint8_t DoublePitchGravityRampSampleActive(void)
{
    return (g_double_pitch_gravity_ramp_running != 0U &&
            (g_double_pitch_gravity_ramp_phase ==
                 DOUBLE_PITCH_GRAVITY_STEP_PHASE_DOWN_RAMP ||
             g_double_pitch_gravity_ramp_phase ==
                 DOUBLE_PITCH_GRAVITY_STEP_PHASE_UP_RAMP_AT_HOLD)) ? 1U : 0U;
}

static void DoublePitchGravityStepResetRuntime(void)
{
    double_pitch_gravity_step_down_tau_nm = 0.0f;
    double_pitch_gravity_step_up_tau_nm = 0.0f;
    double_pitch_gravity_step_last_hold_theta_rad = PITCH_DOWN_DEFAULT_REF_POS_RAD;
    double_pitch_gravity_step_stop_dir = 1;
    g_double_pitch_gravity_step_up_limit_hit = 0U;
    g_double_pitch_gravity_step_up_limit_side =
        DOUBLE_PITCH_GRAVITY_STEP_UP_LIMIT_NONE;
    g_double_pitch_gravity_step_down_hold_tau_nm = 0.0f;
}

static void DoublePitchGravityRampStop(uint8_t done, uint8_t stop_reason)
{
    g_double_pitch_gravity_ramp_auto_start = 0U;
    g_double_pitch_gravity_ramp_arm = 0U;
    g_double_pitch_gravity_step_arm = 0U;
    g_double_pitch_gravity_ramp_running = 0U;
    g_double_pitch_gravity_ramp_phase =
        (done != 0U) ? DOUBLE_PITCH_GRAVITY_RAMP_PHASE_DONE :
                       DOUBLE_PITCH_GRAVITY_RAMP_PHASE_IDLE;
    if (stop_reason != DOUBLE_PITCH_GRAVITY_STEP_STOP_NONE) {
        g_double_pitch_gravity_step_stop_reason = stop_reason;
    }
    DoublePitchGravityStepResetRuntime();
    if (g_double_pitch_ident_axis == DOUBLE_PITCH_IDENT_AXIS_GRAVITY_RAMP) {
        g_double_pitch_ident_arm = 0U;
        g_double_pitch_ident_running = 0U;
    }
}

static void DoublePitchGravityRampApplyAutoStart(void)
{
    const uint8_t request = g_double_pitch_gravity_ramp_auto_start;

    if (g_double_pitch_gravity_ramp_running != 0U) {
        return;
    }

    if (request != 0U) {
        g_double_pitch_gravity_step_sign =
            (request >= 4U) ? DOUBLE_PITCH_GRAVITY_STEP_SIGN_NEGATIVE :
                              DOUBLE_PITCH_GRAVITY_STEP_SIGN_POSITIVE;
        g_double_pitch_gravity_step_arm = 1U;
        g_double_pitch_gravity_ramp_auto_start = 0U;
    } else if (g_double_pitch_gravity_ramp_arm != 0U &&
               g_double_pitch_gravity_step_arm == 0U) {
        g_double_pitch_gravity_step_sign =
            (g_double_pitch_gravity_ramp_tau_big_end_nm < 0.0f ||
             g_double_pitch_gravity_ramp_tau_small_end_nm < 0.0f) ?
            DOUBLE_PITCH_GRAVITY_STEP_SIGN_NEGATIVE :
            DOUBLE_PITCH_GRAVITY_STEP_SIGN_POSITIVE;
        g_double_pitch_gravity_step_arm = 1U;
    }

    if (g_double_pitch_gravity_ramp_duration_s <= 0.0f) {
        g_double_pitch_gravity_ramp_duration_s =
            DOUBLE_PITCH_GRAVITY_RAMP_DEFAULT_DURATION_S;
    }
}

static uint8_t DoublePitchGravityRampGetPsiLimits(float theta_rad,
                                                  float *psi_low_rad,
                                                  float *psi_high_rad)
{
    const size_t count = sizeof(double_pitch_gravity_envelope) /
                         sizeof(double_pitch_gravity_envelope[0]);
    size_t i;

    if (count < 2U || psi_low_rad == NULL || psi_high_rad == NULL) {
        return 0U;
    }
    if (theta_rad < double_pitch_gravity_envelope[0].theta_rad) {
        theta_rad = double_pitch_gravity_envelope[0].theta_rad;
    } else if (theta_rad > double_pitch_gravity_envelope[count - 1U].theta_rad) {
        theta_rad = double_pitch_gravity_envelope[count - 1U].theta_rad;
    }

    for (i = 0U; i < count - 1U; i++) {
        const DoublePitchGravityEnvelopePoint_s *a =
            &double_pitch_gravity_envelope[i];
        const DoublePitchGravityEnvelopePoint_s *b =
            &double_pitch_gravity_envelope[i + 1U];

        if (theta_rad <= b->theta_rad) {
            const float span = b->theta_rad - a->theta_rad;
            const float ratio = (span > 1.0e-6f) ?
                                ((theta_rad - a->theta_rad) / span) : 0.0f;
            *psi_low_rad =
                a->psi_low_rad + (b->psi_low_rad - a->psi_low_rad) * ratio;
            *psi_high_rad =
                a->psi_high_rad + (b->psi_high_rad - a->psi_high_rad) * ratio;
            *psi_low_rad += DOUBLE_PITCH_GRAVITY_RAMP_PSI_MARGIN_RAD;
            *psi_high_rad -= DOUBLE_PITCH_GRAVITY_RAMP_PSI_MARGIN_RAD;
            return (*psi_low_rad <= *psi_high_rad) ? 1U : 0U;
        }
    }

    return 0U;
}

static float DoublePitchGravityRampClampTorque(float torque_nm,
                                               float limit_nm,
                                               uint8_t *limit_flag)
{
    const float old_torque = torque_nm;
    const float limit_abs = fabsf(limit_nm);

    torque_nm = PitchDownClampRange(torque_nm, -limit_abs, limit_abs);
    if (limit_flag != NULL && torque_nm != old_torque) {
        *limit_flag = 1U;
    }
    return torque_nm;
}

static float DoublePitchGravityStepAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float DoublePitchGravityStepClampAbs(float value, float limit)
{
    return PitchDownClampRange(value, -limit, limit);
}

static void DoublePitchGravityStepUpdateNextHold(float theta_rad,
                                                 float step_rad,
                                                 float stop_rad)
{
    float next_hold =
        theta_rad + (float)double_pitch_gravity_step_stop_dir * step_rad;

    (void)stop_rad;
    g_double_pitch_gravity_step_theta_next_hold_rad = next_hold;
}

static uint8_t DoublePitchGravityRampApply(float dt_s,
                                           const DoublePitchLqrInput_s *input,
                                           DoublePitchLqrOutput_s *output,
                                           float *tau_big_motor_cmd_nm,
                                           float *tau_small_motor_cmd_nm,
                                           uint8_t *debug_valid)
{
    float dt;
    float limit_nm;
    float step_rad;
    float stop_rad;
    float torque_rate_nm_s;
    float psi_low = 0.0f;
    float psi_high = 0.0f;
    float tau_big = 0.0f;
    float tau_small = 0.0f;
    float tau_big_pre_limit;
    float tau_small_pre_limit;
    uint8_t tau_big_limit_flag = 0U;
    uint8_t tau_small_limit_flag = 0U;
    uint8_t sample_valid = 0U;
    int8_t torque_sign;

    if (debug_valid != NULL) {
        *debug_valid = 0U;
    }
    DoublePitchGravityRampApplyAutoStart();
    if (DoublePitchGravityRampRequestActive() == 0U) {
        return 0U;
    }
    if (input == NULL) {
        return 1U;
    }
    if (g_double_pitch_gravity_ramp_running != 0U &&
        g_double_pitch_gravity_step_arm == 0U) {
        DoublePitchGravityRampStop(0U,
                                   DOUBLE_PITCH_GRAVITY_STEP_STOP_CANCELLED);
        return 1U;
    }

    dt = PitchDownClampRange(dt_s,
                             PITCH_DOWN_CONTROL_MIN_DT_S,
                             PITCH_DOWN_CONTROL_MAX_DT_S);
    limit_nm = PitchDownClampRange(fabsf(g_double_pitch_gravity_step_tau_limit_nm),
                                   0.0f,
                                   DOUBLE_PITCH_GRAVITY_RAMP_MAX_TORQUE_NM);
    step_rad = fabsf(g_double_pitch_gravity_step_theta_step_rad);
    if (step_rad < 1.0e-3f) {
        step_rad = DOUBLE_PITCH_GRAVITY_STEP_DEFAULT_THETA_STEP_RAD;
    }
    stop_rad = g_double_pitch_gravity_step_down_stop_rad;
    g_double_pitch_gravity_step_theta_stop_rad = stop_rad;
    g_double_pitch_gravity_ramp_tau_limit_nm = limit_nm;
    torque_rate_nm_s = fabsf(g_double_pitch_gravity_step_torque_rate_nm_s);
    if (torque_rate_nm_s <= 1.0e-6f) {
        torque_rate_nm_s = DOUBLE_PITCH_GRAVITY_STEP_TORQUE_RATE_NM_S;
    }

    g_double_pitch_gravity_step_sign = DOUBLE_PITCH_GRAVITY_STEP_SIGN_POSITIVE;
    torque_sign = 1;
    if (DoublePitchGravityRampGetPsiLimits(input->theta_rad,
                                           &psi_low,
                                           &psi_high) == 0U) {
        psi_low = -3.141592653589793f;
        psi_high = 3.141592653589793f;
    }
    g_double_pitch_gravity_step_psi_low_rad = psi_low;
    g_double_pitch_gravity_step_psi_high_rad = psi_high;

    if (g_double_pitch_gravity_ramp_running == 0U ||
        g_double_pitch_gravity_ramp_phase ==
            DOUBLE_PITCH_GRAVITY_RAMP_PHASE_IDLE ||
        g_double_pitch_gravity_ramp_phase ==
            DOUBLE_PITCH_GRAVITY_RAMP_PHASE_DONE ||
        g_double_pitch_gravity_ramp_phase ==
            DOUBLE_PITCH_GRAVITY_STEP_PHASE_FAULT) {
        g_double_pitch_gravity_ramp_running = 1U;
        g_double_pitch_gravity_step_arm = 1U;
        g_double_pitch_gravity_ramp_arm = 1U;
        g_double_pitch_gravity_ramp_phase =
            DOUBLE_PITCH_GRAVITY_STEP_PHASE_DOWN_RAMP;
        g_double_pitch_gravity_step_stop_reason =
            DOUBLE_PITCH_GRAVITY_STEP_STOP_NONE;
        double_pitch_gravity_step_down_tau_nm =
            DoublePitchGravityStepClampAbs(
                (float)torque_sign *
                DOUBLE_PITCH_GRAVITY_STEP_DOWN_START_TORQUE_NM,
                limit_nm);
        double_pitch_gravity_step_up_tau_nm = 0.0f;
        double_pitch_gravity_step_last_hold_theta_rad = input->theta_rad;
        double_pitch_gravity_step_stop_dir = (torque_sign >= 0) ? 1 : -1;
        DoublePitchGravityStepUpdateNextHold(input->theta_rad,
                                             step_rad,
                                             stop_rad);
        g_double_pitch_gravity_step_down_hold_tau_nm = 0.0f;
        g_double_pitch_gravity_step_up_limit_hit = 0U;
        g_double_pitch_gravity_step_up_limit_side =
            DOUBLE_PITCH_GRAVITY_STEP_UP_LIMIT_NONE;
    }

    if (g_double_pitch_gravity_ramp_phase ==
        DOUBLE_PITCH_GRAVITY_STEP_PHASE_DOWN_RAMP) {
        double_pitch_gravity_step_up_tau_nm = 0.0f;
        double_pitch_gravity_step_down_tau_nm +=
            (float)torque_sign * torque_rate_nm_s * dt;
        double_pitch_gravity_step_down_tau_nm =
            DoublePitchGravityStepClampAbs(double_pitch_gravity_step_down_tau_nm,
                                           limit_nm);
        if (DoublePitchGravityStepAbs(input->theta_rad -
                double_pitch_gravity_step_last_hold_theta_rad) >=
            step_rad) {
            g_double_pitch_gravity_ramp_phase =
                DOUBLE_PITCH_GRAVITY_STEP_PHASE_UP_RAMP_AT_HOLD;
            g_double_pitch_gravity_step_down_hold_tau_nm =
                double_pitch_gravity_step_down_tau_nm;
            double_pitch_gravity_step_up_tau_nm = 0.0f;
            g_double_pitch_gravity_step_up_limit_hit = 0U;
            g_double_pitch_gravity_step_up_limit_side =
                DOUBLE_PITCH_GRAVITY_STEP_UP_LIMIT_NONE;
        }
    } else if (g_double_pitch_gravity_ramp_phase ==
               DOUBLE_PITCH_GRAVITY_STEP_PHASE_UP_RAMP_AT_HOLD) {
        double_pitch_gravity_step_down_tau_nm =
            g_double_pitch_gravity_step_down_hold_tau_nm;
        if ((torque_sign > 0 && input->psi_rad >= psi_high) ||
            (torque_sign < 0 && input->psi_rad <= psi_low)) {
            g_double_pitch_gravity_step_up_limit_hit = 1U;
            g_double_pitch_gravity_step_up_limit_side =
                (torque_sign > 0) ? DOUBLE_PITCH_GRAVITY_STEP_UP_LIMIT_HIGH :
                                    DOUBLE_PITCH_GRAVITY_STEP_UP_LIMIT_LOW;
            double_pitch_gravity_step_up_tau_nm = 0.0f;
            double_pitch_gravity_step_last_hold_theta_rad = input->theta_rad;
            DoublePitchGravityStepUpdateNextHold(input->theta_rad,
                                                 step_rad,
                                                 stop_rad);
            g_double_pitch_gravity_ramp_phase =
                DOUBLE_PITCH_GRAVITY_STEP_PHASE_DOWN_RAMP;
        } else {
            double_pitch_gravity_step_up_tau_nm +=
                (float)torque_sign * torque_rate_nm_s * dt;
            double_pitch_gravity_step_up_tau_nm =
                DoublePitchGravityStepClampAbs(double_pitch_gravity_step_up_tau_nm,
                                               limit_nm);
        }
    }

    if (g_double_pitch_gravity_ramp_running != 0U) {
        sample_valid = 1U;
        tau_big = double_pitch_gravity_step_down_tau_nm;
        tau_small = double_pitch_gravity_step_up_tau_nm;
    }

    tau_big_pre_limit = tau_big;
    tau_small_pre_limit = tau_small;
    tau_big = DoublePitchGravityRampClampTorque(tau_big,
                                                limit_nm,
                                                &tau_big_limit_flag);
    tau_small = DoublePitchGravityRampClampTorque(tau_small,
                                                  limit_nm,
                                                  &tau_small_limit_flag);
    if (DoublePitchGravityStepAbs(tau_big) >= (limit_nm - 1.0e-6f)) {
        tau_big_limit_flag = 1U;
    }
    if (DoublePitchGravityStepAbs(tau_small) >= (limit_nm - 1.0e-6f)) {
        tau_small_limit_flag = 1U;
    }

    if (tau_big_motor_cmd_nm != NULL) {
        *tau_big_motor_cmd_nm = tau_big;
    }
    if (tau_small_motor_cmd_nm != NULL) {
        *tau_small_motor_cmd_nm = tau_small;
    }
    if (output != NULL) {
        output->tau_big_cmd_nm = tau_big;
        output->tau_small_cmd_nm = tau_small;
        output->tau_big_pre_limit_nm = tau_big_pre_limit;
        output->tau_small_pre_limit_nm = tau_small_pre_limit;
        output->tau_big_fb_nm = 0.0f;
        output->tau_small_fb_nm = 0.0f;
        output->tau_big_g_nm = 0.0f;
        output->tau_small_g_nm = 0.0f;
        output->tau_big_limit_flag = tau_big_limit_flag;
        output->tau_small_limit_flag = tau_small_limit_flag;
    }

    if (input != NULL) {
        DoublePitchIdentPublishRefs(input->theta_rad,
                                    input->phi_rad,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    0.0f);
    }
    g_double_pitch_ident_axis = DOUBLE_PITCH_IDENT_AXIS_GRAVITY_RAMP;
    g_double_pitch_ident_arm = g_double_pitch_gravity_step_arm;
    g_double_pitch_ident_running = g_double_pitch_gravity_ramp_running;
    g_double_pitch_gravity_ramp_arm = g_double_pitch_gravity_step_arm;
    if (debug_valid != NULL) {
        *debug_valid = sample_valid;
    }
    return 1U;
}

static void DoublePitchIdentUpdateDebug(float dt_s,
                                        uint8_t valid,
                                        uint8_t feedback_ok,
                                        uint8_t target_id,
                                        const DoublePitchLqr_s *ctrl,
                                        const DoublePitchLqrInput_s *input,
                                        const DoublePitchLqrOutput_s *output,
                                        float tau_big_motor_cmd_nm,
                                        float tau_small_motor_cmd_nm)
{
    DoublePitchIdentDebug_s debug = {0};

    debug.sample_count = g_double_pitch_ident_debug.sample_count + 1U;
    debug.time_s = DWT_GetTimeline_s();
    debug.dt_s = dt_s;
    debug.feedback_ok = feedback_ok;
    if (DoublePitchGravityRampRequestActive() != 0U) {
        debug.running = g_double_pitch_gravity_ramp_running;
        debug.arm = g_double_pitch_gravity_step_arm;
        debug.axis = DOUBLE_PITCH_IDENT_AXIS_GRAVITY_RAMP;
    } else {
        debug.running = g_double_pitch_ident_running;
        debug.arm = g_double_pitch_ident_arm;
        debug.axis = g_double_pitch_ident_axis;
    }
    debug.gimbal_mode = (uint8_t)gimbal_cmd_recv.gimbal_mode;
    debug.nuc_mode = (uint8_t)gimbal_cmd_recv.nuc_mode;
    debug.target_id = target_id;
    debug.lqr_ready = (ctrl == NULL) ? 0U : ctrl->ready;
    debug.accel_ready = (ctrl == NULL) ? 0U : ctrl->accel_ready;
    debug.phase = g_double_pitch_gravity_ramp_phase;
    debug.sign = g_double_pitch_gravity_step_sign;
    debug.stop_reason = g_double_pitch_gravity_step_stop_reason;
    debug.up_limit_hit = g_double_pitch_gravity_step_up_limit_hit;
    debug.up_limit_side = g_double_pitch_gravity_step_up_limit_side;
    debug.excite_time_s = g_double_pitch_ident_time_s;
    debug.excite_freq_hz = g_double_pitch_ident_inst_freq_hz;
    debug.theta_ref_rad = g_double_pitch_ident_theta_ref_rad;
    debug.phi_ref_rad = g_double_pitch_ident_phi_ref_rad;
    debug.psi_ref_rad = g_double_pitch_ident_psi_ref_rad;
    debug.theta_ref_dot_rad_s = g_double_pitch_ident_theta_ref_dot_rad_s;
    debug.phi_ref_dot_rad_s = g_double_pitch_ident_phi_ref_dot_rad_s;
    debug.theta_ref_ddot_rad_s2 = g_double_pitch_ident_theta_ref_ddot_rad_s2;
    debug.phi_ref_ddot_rad_s2 = g_double_pitch_ident_phi_ref_ddot_rad_s2;
    debug.theta_next_hold_rad = g_double_pitch_gravity_step_theta_next_hold_rad;
    debug.theta_stop_rad = g_double_pitch_gravity_step_theta_stop_rad;
    debug.psi_low_rad = g_double_pitch_gravity_step_psi_low_rad;
    debug.psi_high_rad = g_double_pitch_gravity_step_psi_high_rad;
    debug.tau_limit_nm = g_double_pitch_gravity_ramp_tau_limit_nm;
    debug.down_hold_tau_nm = g_double_pitch_gravity_step_down_hold_tau_nm;
    debug.tau_big_motor_cmd_nm = tau_big_motor_cmd_nm;
    debug.tau_small_motor_cmd_nm = tau_small_motor_cmd_nm;

    if (input != NULL) {
        debug.theta_rad = input->theta_rad;
        debug.phi_rad = input->phi_rad;
        debug.psi_rad = input->psi_rad;
        debug.omega_big_rad_s = input->omega_big_rad_s;
        debug.omega_small_rad_s = input->omega_small_rad_s;
        debug.theta_ref_rad = input->theta_ref_rad;
        debug.phi_ref_rad = input->phi_ref_rad;
        debug.psi_ref_rad = PitchWrapPiRad(input->theta_ref_rad +
                                           input->phi_ref_rad);
        debug.theta_ref_dot_rad_s = input->theta_ref_dot_rad_s;
        debug.phi_ref_dot_rad_s = input->phi_ref_dot_rad_s;
        debug.theta_ref_ddot_rad_s2 = input->theta_ref_ddot_rad_s2;
        debug.phi_ref_ddot_rad_s2 = input->phi_ref_ddot_rad_s2;
        debug.tau_big_meas_nm = input->tau_big_meas_nm;
        debug.tau_small_meas_nm = input->tau_small_meas_nm;
        debug.motor_big_temp_c = input->motor_big_temp_c;
        debug.motor_small_temp_c = input->motor_small_temp_c;
    }

    if (output != NULL) {
        debug.tau_big_limit_flag = output->tau_big_limit_flag;
        debug.tau_small_limit_flag = output->tau_small_limit_flag;
        debug.tau_big_cmd_nm = output->tau_big_cmd_nm;
        debug.tau_small_cmd_nm = output->tau_small_cmd_nm;
        debug.tau_big_pre_limit_nm = output->tau_big_pre_limit_nm;
        debug.tau_small_pre_limit_nm = output->tau_small_pre_limit_nm;
        debug.tau_big_fb_nm = output->tau_big_fb_nm;
        debug.tau_small_fb_nm = output->tau_small_fb_nm;
        debug.tau_big_ff_nm = output->tau_big_g_nm;
        debug.tau_small_ff_nm = output->tau_small_g_nm;
    }

    if (pitch_motor_down != NULL) {
        debug.tau_big_meas_nm = pitch_motor_down->measure.tor;
        debug.tau_big_ctrl_tor_set_nm = pitch_motor_down->ctrl.tor_set;
        debug.motor_big_pos_rad = pitch_motor_down->measure.pos;
        debug.motor_big_vel_rad_s = pitch_motor_down->measure.vel;
        debug.motor_big_temp_c = pitch_motor_down->measure.Tcoil;
        debug.motor_big_tmos_c = pitch_motor_down->measure.Tmos;
    }
    if (pitch_motor_up != NULL) {
        debug.tau_small_meas_nm = pitch_motor_up->measure.tor;
        debug.tau_small_ctrl_tor_set_nm = pitch_motor_up->ctrl.tor_set;
        debug.motor_small_pos_rad = pitch_motor_up->measure.pos;
        debug.motor_small_vel_rad_s = pitch_motor_up->measure.vel;
        debug.motor_small_temp_c = pitch_motor_up->measure.Tcoil;
        debug.motor_small_tmos_c = pitch_motor_up->measure.Tmos;
    }

    if (DoublePitchGravityRampRequestActive() != 0U) {
        debug.valid =
            (valid != 0U && DoublePitchGravityRampSampleActive() != 0U) ?
            1U : 0U;
    } else {
        debug.valid =
            (valid != 0U &&
             feedback_ok != 0U &&
             ((g_double_pitch_static_point_index != 0U) ?
              DoublePitchIdentStaticSampleActive() :
              ((g_double_pitch_ident_running != 0U) ? 1U : 0U)) != 0U &&
             DoublePitchIdentNormalModeActive() != 0U) ? 1U : 0U;
    }

    g_double_pitch_ident_debug = debug;
}

static float PitchDownCalculateLQRTorque(void)
{
    static float ref_pos;
    uint8_t auto_aim_active;
    float lqr_dt;
    float torque_lqreso;
    float torque_cmd;

    if (PitchDownRefreshControlState() == 0U) {
        PitchDownResetUnavailableControl();
        return 0.0f;
    }

    auto_aim_active = (g_gimbal_auto_aim_active != 0U) ? 1U : 0U;
    if (auto_aim_active != pitch_down_auto_ref_last_active) {
        PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
        DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
        pitch_down_auto_ref_last_active = auto_aim_active;
    }

    lqr_dt = DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    if (auto_aim_active != 0U) {
        PitchDownUpdateAutoVisionRef(lqr_dt);
    } else {
        PitchDownResetAutoVisionRef();
        PitchDownUpdateRefTrajectory(pitch_down_ref_limited_pos_rad, lqr_dt);
    }
    ref_pos = pitch_down_ref_traj_rad;
    pitch_down_pos_error_rad = ref_pos - pitch_down_pos_rad;

    pitch_down_gravity_torque_ff_nm = 0.0f;
    PitchDownClearGravityDebug();
    PitchDownResetHoldIntegral();
    pitch_down_friction_torque_ff_nm = 0.0f;
    dzx_pitch_down_lqr_eso.lqr_config.k[0] =
        (auto_aim_active != 0U) ?
        g_pitch_down_auto_lqr_k_theta :
        g_pitch_down_lqr_k_theta;
    dzx_pitch_down_lqr_eso.lqr_config.k[1] =
        (auto_aim_active != 0U) ?
        g_pitch_down_auto_lqr_k_omega :
        g_pitch_down_lqr_k_omega;
    dzx_pitch_down_lqr_eso.lqr_config.plant_j_kg_m2 =
        g_pitch_down_model_j_kg_m2;
    dzx_pitch_down_lqr_eso.lqr_config.plant_b_nms_rad =
        g_pitch_down_model_b_nms_rad;
    dzx_pitch_down_lqr_eso.eso_config.enable = g_pitch_down_eso_enable;
    dzx_pitch_down_lqr_eso.eso_config.alpha = g_pitch_down_eso_alpha;
    dzx_pitch_down_lqr_eso.eso_config.comp_gain =
        g_pitch_down_eso_comp_gain;
    dzx_pitch_down_lqr_eso.eso_config.comp_limit =
        g_pitch_down_eso_comp_limit_nm;
    pitch_down_eso_runtime_enable =
        (g_pitch_down_eso_enable != 0U) ? 1U : 0U;
    pitch_down_feedforward_nm = PitchDownCalculateModelFeedforward();

    pitch_down_lqr_e_theta_rad = pitch_down_pos_rad - ref_pos;
    pitch_down_lqr_e_omega_rad_s =
        pitch_down_vel_rad_s - pitch_down_ref_vel_rad_s;

    torque_lqreso = PitchLQRESO_CalcTrajectory(&dzx_pitch_down_lqr_eso,
                                               ref_pos,
                                               pitch_down_ref_vel_rad_s,
                                               pitch_down_ref_acc_rad_s2,
                                               pitch_down_pos_rad,
                                               pitch_down_vel_rad_s,
                                               pitch_down_feedforward_nm,
                                               pitch_down_eso_runtime_enable,
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
    float gravity_ff_nm;

    if (PitchDownRefreshMITPositionState() == 0U) {
        PitchDownResetUnavailableControl();
        return;
    }

    PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    PitchDownClearLQRDebug();
    PitchDownClearGravityDebug();
    PitchDownResetHoldIntegral();
    pitch_down_friction_torque_ff_nm = 0.0f;
    pitch_down_eso_runtime_enable = 0U;
    gravity_ff_nm = PitchDownCalcMitGravityFeedforward(pitch_down_pos_rad);
    pitch_down_feedforward_nm = gravity_ff_nm;
    pitch_down_torque_cmd_nm = gravity_ff_nm;
    PitchDownResetOutputTorqueSlew(gravity_ff_nm);

    DMMotorSetMITPositionVelocity(pitch_motor_down,
                                  pitch_down_ref_limited_pos_rad,
                                  vel_set_rad_s,
                                  kp,
                                  kd,
                                
                                  gravity_ff_nm);
}

static void PitchDownApplyDirectRecoverTorque(void)
{
    if (PitchDownRefreshControlState() == 0U) {
        PitchDownResetUnavailableControl();
        return;
    }

    PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    PitchDownClearGravityDebug();
    PitchDownResetHoldIntegral();
    PitchDownResetModelFeedforwardDebug();

    pitch_down_lqr_e_theta_rad = pitch_down_pos_rad - pitch_down_ref_limited_pos_rad;
    pitch_down_lqr_e_omega_rad_s = pitch_down_vel_rad_s;
    pitch_down_lqr_tau_raw_nm = 0.0f;
    pitch_down_lqr_pre_limit_torque_nm = PITCH_DOWN_DIRECT_RECOVER_TORQUE_NM;
    pitch_down_lqr_output_nm = PITCH_DOWN_DIRECT_RECOVER_TORQUE_NM;
    pitch_down_lqr_limit_active = 0U;
    pitch_down_gravity_torque_ff_nm = 0.0f;
    pitch_down_friction_torque_ff_nm = 0.0f;
    pitch_down_feedforward_nm = 0.0f;
    pitch_down_eso_runtime_enable = 0U;
    pitch_down_ref_traj_primed = 0U;
    pitch_down_ref_traj_rad = pitch_down_ref_limited_pos_rad;
    pitch_down_ref_vel_rad_s = 0.0f;
    pitch_down_ref_acc_rad_s2 = 0.0f;
    pitch_down_torque_cmd_nm = PITCH_DOWN_DIRECT_RECOVER_TORQUE_NM;
    pitch_down_soft_limit_active = 0U;

    (void)PitchDownSetTorqueWithSlew(PITCH_DOWN_DIRECT_RECOVER_TORQUE_NM);
}

static void PitchDownApplyHoleRestoreTorque(void)
{
    PitchDownSetRefOverride(HOLE_DOWN_RESTORE_TARGET_RAD, 0U);
    if (PitchDownRefreshControlState() == 0U) {
        PitchDownResetUnavailableControl();
        return;
    }

    PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    PitchDownClearGravityDebug();
    PitchDownResetHoldIntegral();
    PitchDownResetModelFeedforwardDebug();

    pitch_down_lqr_e_theta_rad = pitch_down_pos_rad - HOLE_DOWN_RESTORE_TARGET_RAD;
    pitch_down_lqr_e_omega_rad_s = pitch_down_vel_rad_s;
    pitch_down_lqr_tau_raw_nm = 0.0f;
    pitch_down_lqr_pre_limit_torque_nm = HOLE_DOWN_RESTORE_TORQUE_NM;
    pitch_down_lqr_output_nm = HOLE_DOWN_RESTORE_TORQUE_NM;
    pitch_down_lqr_limit_active = 0U;
    pitch_down_gravity_torque_ff_nm = 0.0f;
    pitch_down_friction_torque_ff_nm = 0.0f;
    pitch_down_feedforward_nm = 0.0f;
    pitch_down_eso_runtime_enable = 0U;
    pitch_down_ref_traj_primed = 0U;
    pitch_down_ref_traj_rad = pitch_down_ref_limited_pos_rad;
    pitch_down_ref_vel_rad_s = 0.0f;
    pitch_down_ref_acc_rad_s2 = 0.0f;
    pitch_down_torque_cmd_nm = HOLE_DOWN_RESTORE_TORQUE_NM;
    pitch_down_soft_limit_active = 0U;

    PitchDownSetTorqueImmediate(HOLE_DOWN_RESTORE_TORQUE_NM);
}

static void PitchDownApplyHybridControlWithGain(float mit_kp,
                                                float mit_kd,
                                                float mit_vel_set_rad_s)
{
    const float down_pos_rad =
        (pitch_motor_down == NULL) ? pitch_down_pos_rad : pitch_motor_down->measure.pos;

    if (pitch_motor_down == NULL || !DMMotorIsOnline(pitch_motor_down)) {
        (void)PitchDownRefreshControlState();
        PitchDownResetUnavailableControl();
        return;
    }

    if (down_pos_rad < PITCH_DOWN_DIRECT_RECOVER_SWITCH_POS_RAD) {
        if (gimbal_cmd_recv.gimbal_mode != GIMBAL_HOLE_MODE) {
            PitchDownApplyDirectRecoverTorque();
            return;
        }
        PitchDownApplyMITPositionControlWithGain(mit_kp, mit_kd, mit_vel_set_rad_s);
        return;
    }

    (void)PitchDownSetTorqueWithSlew(PitchDownCalculateLQRTorque());
    // PitchDownSetTorqueImmediate(0.0f);
}

static void PitchDownStopControl(void)
{
    PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    PitchDownClearLQRDebug();
    pitch_down_torque_cmd_nm       = 0.0f;
    pitch_down_gravity_torque_ff_nm = 0.0f;
    pitch_down_friction_torque_ff_nm = 0.0f;
    pitch_down_soft_limit_active   = 0U;
    PitchDownClearGravityDebug();
    PitchDownClearVelocityFilter();
    pitch_down_ref_ramp_active = 0U;
    pitch_down_ref_traj_primed = 0U;
    pitch_down_ref_traj_rad = pitch_down_ref_limited_pos_rad;
    pitch_down_ref_vel_rad_s = 0.0f;
    pitch_down_ref_acc_rad_s2 = 0.0f;
    GimbalSetAutoAimActive(0U);
    pitch_down_auto_ref_last_active = 0U;
    PitchDownResetHoldIntegral();
    if (pitch_motor_down != NULL) {
        pitch_down_pos_rad       = pitch_motor_down->measure.pos;
        PitchDownResetVelocityFilter(pitch_motor_down->measure.vel);
        pitch_down_ref_pos_rad   = PitchDownSelectTargetRefPos();
        pitch_down_ref_target_limited_pos_rad = pitch_down_ref_pos_rad;
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
    const uint8_t normal_yaw_ident_mode =
        (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE &&
         gimbal_cmd_recv.nuc_mode != version_control) ? 1U : 0U;
    const uint8_t auto_yaw_debug_mode =
        (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE &&
         gimbal_cmd_recv.nuc_mode == version_control) ? 1U : 0U;

    yaw_fit_time_s = DWT_GetTimeline_ms() * 0.001f;
    yaw_angle_rad  = gimbal_cmd_recv.yaw_measure_angle * LQR_ESO_DEG_TO_RAD;
    yaw_gyro_rad_s = gimbal_cmd_recv.yaw_measure_gyro;

    g_yaw_lqr_ident_record.sample_count++;
    g_yaw_lqr_ident_record.time_s = yaw_fit_time_s;
    g_yaw_lqr_ident_record.gimbal_mode =
        (uint8_t)gimbal_cmd_recv.gimbal_mode;
    g_yaw_lqr_ident_record.nuc_mode =
        (uint8_t)gimbal_cmd_recv.nuc_mode;
    g_yaw_auto_aim_debug_record.sample_count++;
    g_yaw_auto_aim_debug_record.time_s = yaw_fit_time_s;
    g_yaw_auto_aim_debug_record.gimbal_mode =
        (uint8_t)gimbal_cmd_recv.gimbal_mode;
    g_yaw_auto_aim_debug_record.nuc_mode =
        (uint8_t)gimbal_cmd_recv.nuc_mode;
    g_yaw_model_ident.gimbal_mode =
        (uint8_t)gimbal_cmd_recv.gimbal_mode;
    g_yaw_model_ident.nuc_mode =
        (uint8_t)gimbal_cmd_recv.nuc_mode;

    if (yaw_motor == NULL) {
        motor_torque_nm = 0.0f;
        yaw_motor_pos_rad = 0.0f;
        yaw_motor_total_rad = 0.0f;
        g_yaw_lqr_ident_record.feedback_ok = 0U;
        g_yaw_lqr_ident_record.valid = 0U;
        g_yaw_lqr_ident_record.motor_pos_rad = LQRESODebugFloatToHalf(0.0f);
        g_yaw_lqr_ident_record.motor_total_pos_rad = LQRESODebugFloatToHalf(0.0f);
        g_yaw_lqr_ident_record.motor_vel_rad_s = LQRESODebugFloatToHalf(0.0f);
        g_yaw_lqr_ident_record.motor_tor_nm = LQRESODebugFloatToHalf(0.0f);
        g_yaw_lqr_ident_record.ctrl_tor_set_nm = LQRESODebugFloatToHalf(0.0f);
        g_yaw_lqr_ident_record.motor_p_int = 0;
        g_yaw_lqr_ident_record.motor_v_int = 0;
        g_yaw_lqr_ident_record.motor_t_int = 0;
        g_yaw_auto_aim_debug_record.feedback_ok = 0U;
        g_yaw_auto_aim_debug_record.valid = 0U;
        g_yaw_auto_aim_debug_record.motor_pos_rad = 0.0f;
        g_yaw_auto_aim_debug_record.motor_total_pos_rad = 0.0f;
        g_yaw_auto_aim_debug_record.motor_vel_rad_s = 0.0f;
        g_yaw_auto_aim_debug_record.motor_tor_nm = 0.0f;
        g_yaw_auto_aim_debug_record.ctrl_tor_set_nm = 0.0f;
        g_yaw_model_ident.feedback_ok = 0U;
        g_yaw_model_ident.valid = 0U;
        g_yaw_model_ident.sample_active = 0U;
        g_yaw_model_ident.motor_pos_rad = 0.0f;
        g_yaw_model_ident.motor_total_pos_rad = 0.0f;
        g_yaw_model_ident.motor_vel_rad_s = 0.0f;
        g_yaw_model_ident.motor_tor_nm = 0.0f;
        g_yaw_model_ident.ctrl_tor_set_nm = 0.0f;
        g_yaw_model_ident.motor_p_int = 0;
        g_yaw_model_ident.motor_v_int = 0;
        g_yaw_model_ident.motor_t_int = 0;
        return;
    }

    motor_torque_nm = yaw_motor->measure.tor;
    yaw_motor_pos_rad = yaw_motor->measure.pos;
    yaw_motor_total_rad = yaw_motor->measure.total_pos;
    g_yaw_lqr_ident_record.feedback_ok = DMMotorIsOnline(yaw_motor);
    g_yaw_auto_aim_debug_record.feedback_ok =
        g_yaw_lqr_ident_record.feedback_ok;
    g_yaw_model_ident.feedback_ok = g_yaw_lqr_ident_record.feedback_ok;
    if (yaw_closed_loop == 0U ||
        normal_yaw_ident_mode == 0U ||
        g_yaw_lqr_ident_record.feedback_ok == 0U) {
        g_yaw_lqr_ident_record.valid = 0U;
    }
    if (yaw_closed_loop == 0U ||
        auto_yaw_debug_mode == 0U ||
        g_yaw_lqr_ident_record.feedback_ok == 0U) {
        g_yaw_auto_aim_debug_record.valid = 0U;
    }
    g_yaw_lqr_ident_record.motor_pos_rad = LQRESODebugFloatToHalf(yaw_motor->measure.pos);
    g_yaw_lqr_ident_record.motor_total_pos_rad = LQRESODebugFloatToHalf(yaw_motor->measure.total_pos);
    g_yaw_lqr_ident_record.motor_vel_rad_s = LQRESODebugFloatToHalf(yaw_motor->measure.vel);
    g_yaw_lqr_ident_record.motor_tor_nm = LQRESODebugFloatToHalf(yaw_motor->measure.tor);
    g_yaw_lqr_ident_record.ctrl_tor_set_nm = LQRESODebugFloatToHalf(yaw_motor->ctrl.tor_set);
    g_yaw_lqr_ident_record.motor_p_int = yaw_motor->measure.p_int;
    g_yaw_lqr_ident_record.motor_v_int = yaw_motor->measure.v_int;
    g_yaw_lqr_ident_record.motor_t_int = yaw_motor->measure.t_int;
    g_yaw_auto_aim_debug_record.motor_pos_rad = yaw_motor->measure.pos;
    g_yaw_auto_aim_debug_record.motor_total_pos_rad = yaw_motor->measure.total_pos;
    g_yaw_auto_aim_debug_record.motor_vel_rad_s = yaw_motor->measure.vel;
    g_yaw_auto_aim_debug_record.motor_tor_nm = yaw_motor->measure.tor;
    g_yaw_auto_aim_debug_record.ctrl_tor_set_nm = yaw_motor->ctrl.tor_set;
    g_yaw_model_ident.motor_pos_rad = yaw_motor->measure.pos;
    g_yaw_model_ident.motor_total_pos_rad = yaw_motor->measure.total_pos;
    g_yaw_model_ident.motor_vel_rad_s = yaw_motor->measure.vel;
    g_yaw_model_ident.motor_tor_nm = yaw_motor->measure.tor;
    g_yaw_model_ident.ctrl_tor_set_nm = yaw_motor->ctrl.tor_set;
    g_yaw_model_ident.motor_p_int = yaw_motor->measure.p_int;
    g_yaw_model_ident.motor_v_int = yaw_motor->measure.v_int;
    g_yaw_model_ident.motor_t_int = yaw_motor->measure.t_int;
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
        .output_slew_rate = PITCH_UP_TORQUE_SLEW_RATE_NM_S,
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
        .disable_model_feedforward = 1U,
        .plant_j_kg_m2 = PITCH_DOWN_MODEL_J_KG_M2,
        .plant_b_nms_rad = PITCH_DOWN_MODEL_B_NMS_RAD,
        .q_theta = PITCH_DOWN_MODEL_Q_THETA,
        .q_omega = PITCH_DOWN_MODEL_Q_OMEGA,
        .r_torque = PITCH_DOWN_MODEL_R_TORQUE,
    };
    ESO_Config_s pitch_down_eso_config = {
        .enable = PITCH_DOWN_ESO_ENABLE,
        .alpha = PITCH_DOWN_ESO_ALPHA,
        .comp_gain = PITCH_DOWN_ESO_COMP_GAIN,
        .comp_limit = PITCH_DOWN_ESO_COMP_LIMIT_NM,
        .w0_min = PITCH_DOWN_ESO_W0_MIN_RAD_S,
        .w0_max = PITCH_DOWN_ESO_W0_MAX_RAD_S,
        .w0_set_enable = 0U,
        .w0_set = 0.0f,
    };
#if PITCH_UP_ESO_TRIAL_ENABLE
    ESO_Config_s pitch_up_eso_config = {
        .enable = PITCH_UP_ESO_ENABLE,
        .alpha = PITCH_UP_ESO_ALPHA,
        .comp_gain = PITCH_UP_ESO_COMP_GAIN,
        .comp_limit = PITCH_UP_ESO_COMP_LIMIT_NM,
        .w0_set_enable = 0U,
        .w0_set = 0.0f,
    };
    PitchLQRESO_Init(&dzx_pitch_lqr_eso,
                     &pitch_lqr_config,
                     &pitch_up_eso_config);
#else
    PitchLQRESO_Init(&dzx_pitch_lqr_eso, &pitch_lqr_config, NULL);
#endif
    PitchLQRESO_Init(&dzx_pitch_down_lqr_eso,
                     &pitch_down_lqr_config,
                     &pitch_down_eso_config);
    DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    DWT_GetDeltaT(&pitch_up_ident_dwt_cnt);
    PitchUpAutoPidReset();
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

static float GimbalWrap180Deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float GimbalDMYawSingleRoundDegFloat(void)
{
    if (yaw_motor == NULL) {
        return 0.0f;
    }
    return GimbalWrap360Deg(yaw_motor->measure.pos * DZX_YAW_RAD_TO_DEG);
}

static uint16_t GimbalDMYawSingleRoundDeg(void)
{
    return (uint16_t)(GimbalDMYawSingleRoundDegFloat() + 0.5f);
}

static float GimbalDMYawTotalDeg(void)
{
    if (yaw_motor == NULL) {
        return 0.0f;
    }
    return yaw_motor->measure.total_pos * DZX_YAW_RAD_TO_DEG;
}

static uint8_t GimbalYawHoleEncoderTakeoverActive(void)
{
    return (gimbal_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE &&
            gimbal_cmd_recv.hole_yaw_ready != 0U &&
            gimbal_cmd_recv.hole_offset_yaw_takeover != 0U) ? 1U : 0U;
}

static float GimbalYawWrapNearestMotorTargetRad(float target_rad, float measure_rad)
{
    float err_rad = target_rad - measure_rad;

    while (err_rad > 3.141592653589793f) {
        target_rad -= 6.283185307179586f;
        err_rad -= 6.283185307179586f;
    }
    while (err_rad < -3.141592653589793f) {
        target_rad += 6.283185307179586f;
        err_rad += 6.283185307179586f;
    }
    return target_rad;
}

static uint8_t GimbalYawBuildControlInput(float ref_deg, YawControlInput_t *input)
{
    const uint8_t hole_encoder_takeover =
        GimbalYawHoleEncoderTakeoverActive();

    if (input == NULL) {
        return 0U;
    }

    input->theta_ref_deg = ref_deg;
    input->theta_ref_rad = ref_deg * LQR_ESO_DEG_TO_RAD;
    input->omega_ref_rad_s = 0.0f;
    input->alpha_ref_rad_s2 = 0.0f;
    input->omega_base_rad_s = gimbal_yaw_speed_feedback_rad_s;
    input->vision_frame_seq = 0;
    input->vision_period_s = 0.0f;
    input->torque_slew_rate_nm_s = g_ctrl_torque_slew_rate;
    input->ref_traj_valid = 0U;
    input->auto_aim_mode = 0U;
    input->hole_offset_mode =
        (gimbal_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) ? 1U : 0U;
    input->torque_slew_enable = 1U;
    input->base_omega_valid = (gimbal_IMU_data != NULL) ? 1U : 0U;
    input->tau_meas_nm   = (yaw_motor == NULL) ? 0.0f : yaw_motor->measure.tor;
    input->feedback_ok   = DMMotorIsOnline(yaw_motor);

    input->theta_rad   = gimbal_cmd_recv.yaw_measure_angle * LQR_ESO_DEG_TO_RAD;
    input->omega_rad_s = gimbal_cmd_recv.yaw_measure_gyro;

    if (hole_encoder_takeover != 0U) {
        const float yaw_total_pos_rad =
            (yaw_motor == NULL) ? 0.0f : yaw_motor->measure.total_pos;
        const float yaw_target_rad =
            GimbalYawWrapNearestMotorTargetRad(HOLE_YAW_MOTOR_TARGET_RAD,
                                               yaw_total_pos_rad);

        input->theta_ref_deg = yaw_target_rad * DZX_YAW_RAD_TO_DEG;
        input->theta_ref_rad = yaw_target_rad;
        input->omega_ref_rad_s = 0.0f;
        input->alpha_ref_rad_s2 = 0.0f;
        input->omega_base_rad_s = 0.0f;
        input->base_omega_valid = 0U;
        input->theta_rad = yaw_total_pos_rad;
        input->omega_rad_s = (yaw_motor == NULL) ? 0.0f : yaw_motor->measure.vel;
        input->hole_offset_mode = 1U;
    }

    if (gimbal_cmd_recv.nuc_mode == version_control &&
        gimbal_cmd_recv.hole_restore_active == 0U &&
        gimbal_cmd_recv.gimbal_mode != GIMBAL_HOLE_MODE) {
        input->theta_ref_rad = gimbal_cmd_recv.yaw_ref_rad;
        input->omega_ref_rad_s = gimbal_cmd_recv.yaw_ref_vel_rad_s;
        input->alpha_ref_rad_s2 = gimbal_cmd_recv.yaw_ref_acc_rad_s2;
        input->vision_frame_seq = gimbal_cmd_recv.vision_frame_seq;
        input->vision_period_s = gimbal_cmd_recv.vision_period_s;
        input->torque_slew_rate_nm_s = g_ctrl_auto_torque_slew_rate;
        input->ref_traj_valid = 1U;
        input->auto_aim_mode = 1U;
        input->torque_slew_enable = 1U;
    }

    return input->feedback_ok;
}

static float GimbalYawSelectHoleTakeoverRefDeg(void)
{
    const float yaw_total_pos_rad =
        (yaw_motor == NULL) ? 0.0f : yaw_motor->measure.total_pos;
    const float yaw_target_rad =
        GimbalYawWrapNearestMotorTargetRad(HOLE_YAW_MOTOR_TARGET_RAD,
                                           yaw_total_pos_rad);

    return yaw_target_rad * DZX_YAW_RAD_TO_DEG;
}

static uint8_t gimbal_yaw_hole_hold_active = 0U;
static uint8_t gimbal_yaw_hole_takeover_stage_last = 0U;
static uint8_t gimbal_yaw_hole_zero_torque_latched = 0U;
static float gimbal_yaw_hole_hold_ref_deg = 0.0f;
volatile uint8_t g_hole_yaw_takeover_zero_torque_fallback = 0U;

static void GimbalYawResetControllerRuntime(void)
{
    YawLQRESO_Reset(&dzx_yaw_lqr_eso);
    DWT_GetDeltaT(&dzx_yaw_lqr_dwt_cnt);
}

static void GimbalYawClearHoleHoldState(void)
{
    if (gimbal_yaw_hole_zero_torque_latched != 0U) {
        GimbalYawResetControllerRuntime();
    }
    gimbal_yaw_hole_hold_active = 0U;
    gimbal_yaw_hole_takeover_stage_last = 0U;
    gimbal_yaw_hole_zero_torque_latched = 0U;
    gimbal_yaw_hole_hold_ref_deg = 0.0f;
}

static uint8_t GimbalYawHoleZeroTorqueActive(void)
{
    if (g_hole_yaw_takeover_zero_torque_fallback == 0U) {
        if (gimbal_yaw_hole_zero_torque_latched != 0U) {
            gimbal_yaw_hole_zero_torque_latched = 0U;
            GimbalYawResetControllerRuntime();
        }
        return 0U;
    }

    if (gimbal_yaw_hole_zero_torque_latched == 0U &&
        GimbalYawHoleEncoderTakeoverActive() != 0U) {
        gimbal_yaw_hole_zero_torque_latched = 1U;
        gimbal_yaw_hole_hold_active = 0U;
        gimbal_yaw_hole_takeover_stage_last = 0U;
        gimbal_yaw_hole_hold_ref_deg = 0.0f;
        GimbalYawResetControllerRuntime();
    }

    return gimbal_yaw_hole_zero_torque_latched;
}

static float GimbalYawSelectHoleRefDeg(void)
{
    const uint8_t hole_takeover_stage =
        GimbalYawHoleEncoderTakeoverActive();

    if (gimbal_yaw_hole_hold_active == 0U) {
        gimbal_yaw_hole_hold_ref_deg = gimbal_cmd_recv.yaw_measure_angle;
        gimbal_yaw_hole_hold_active = 1U;
        gimbal_yaw_hole_takeover_stage_last = hole_takeover_stage;
        GimbalYawResetControllerRuntime();
    } else if (hole_takeover_stage != gimbal_yaw_hole_takeover_stage_last) {
        gimbal_yaw_hole_takeover_stage_last = hole_takeover_stage;
        GimbalYawResetControllerRuntime();
    }

    if (hole_takeover_stage != 0U) {
        return GimbalYawSelectHoleTakeoverRefDeg();
    }

    return gimbal_yaw_hole_hold_ref_deg;
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
#if defined(GIMBAL_BOARD)
    gimbal_feedback_data.hole_pitch_zero_torque =
        (hole_up_torque_zero_latched != 0U) ? 1U : 0U;
#else
    gimbal_feedback_data.hole_pitch_zero_torque = 0U;
#endif
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
    float yaw_ref = 0.0f;
    static volatile uint8_t codex_flag=0;
    static volatile float yaw_ref_codex=0;
    GimbalUpdateYawIMUFeedback();
    switch (gimbal_cmd_recv.gimbal_mode) {
    case GIMBAL_ZERO_FORCE:
        DMMotorSetTorque(yaw_motor, 0.0f);
        DMMotorStop(yaw_motor);
        GimbalYawResetControllerRuntime();
        GimbalYawClearHoleHoldState();
        dzx_yaw_torque        = 0.0f;
        angle_error           = 0.0f;
        break;
    case GIMBAL_GYRO_MODE:
        GimbalYawClearHoleHoldState();
        DMMotorEnable1(yaw_motor);
        // static float yaw_torque_test = 0.0f;
        // DMMotorSetTorque(yaw_motor, yaw_torque_test);
        yaw_ref = (codex_flag != 0U) ? yaw_ref_codex : gimbal_cmd_recv.yaw;
        DMMotorSetTorque(yaw_motor, GimbalYawCalculateTorque(yaw_ref));
        break;
    case GIMBAL_HOLE_MODE:
        DMMotorEnable1(yaw_motor);
        if (GimbalYawHoleZeroTorqueActive() != 0U) {
            DMMotorSetTorque(yaw_motor, 0.0f);
            dzx_yaw_torque = 0.0f;
            angle_error = 0.0f;
            break;
        }
        yaw_ref = GimbalYawSelectHoleRefDeg();
        DMMotorSetTorque(yaw_motor, GimbalYawCalculateTorque(yaw_ref));
        break;
    case GIMBAL_MOTOR_MODE:
        GimbalYawClearHoleHoldState();
        DMMotorEnable1(yaw_motor);
        DMMotorSetTorque(yaw_motor, GimbalYawCalculateTorque(gimbal_cmd_recv.yaw));
        break;
    default:
        GimbalYawClearHoleHoldState();
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

static void PitchUpResetVisionAccFilter(void)
{
    pitch_up_vision_acc_filter_primed = 0U;
    g_pitch_up_vision_acc_raw_rad_s2 = 0.0f;
    g_pitch_up_vision_acc_filt_rad_s2 = 0.0f;
}

static float PitchUpFilterVisionAcc(float acc_rad_s2)
{
    float alpha = PitchDownClampRange(g_pitch_up_vision_acc_filter_alpha,
                                      0.0f,
                                      1.0f);

    g_pitch_up_vision_acc_filter_alpha = alpha;
    g_pitch_up_vision_acc_raw_rad_s2 =
        (isfinite(acc_rad_s2) != 0) ? acc_rad_s2 : 0.0f;

    if (pitch_up_vision_acc_filter_primed == 0U ||
        isfinite(g_pitch_up_vision_acc_filt_rad_s2) == 0) {
        g_pitch_up_vision_acc_filt_rad_s2 = g_pitch_up_vision_acc_raw_rad_s2;
        pitch_up_vision_acc_filter_primed = 1U;
    } else {
        g_pitch_up_vision_acc_filt_rad_s2 +=
            alpha * (g_pitch_up_vision_acc_raw_rad_s2 -
                     g_pitch_up_vision_acc_filt_rad_s2);
    }

    return g_pitch_up_vision_acc_filt_rad_s2;
}

static float PitchUpVisionTrajDt(void)
{
    float dt_s = DWT_GetDeltaT(&pitch_up_traj_dwt_cnt);

    if (dt_s < PITCH_UP_TRAJ_MIN_DT_S ||
        dt_s > PITCH_UP_TRAJ_MAX_DT_S ||
        isfinite(dt_s) == 0) {
        dt_s = PITCH_UP_TRAJ_DEFAULT_DT_S;
    }

    return dt_s;
}

static void PitchUpResetVisionTrajectory(void)
{
    pitch_up_traj_initialized = 0U;
    pitch_up_traj_latched_theta_rad = PITCH_UP_IMU_ZERO_REF_RAD;
    pitch_up_traj_latched_omega_rad_s = 0.0f;
    pitch_up_traj_latched_alpha_rad_s2 = 0.0f;
    pitch_up_traj_last_input_theta_rad = PITCH_UP_IMU_ZERO_REF_RAD;
    pitch_up_traj_last_input_omega_rad_s = 0.0f;
    pitch_up_traj_last_input_alpha_rad_s2 = 0.0f;
    g_pitch_up_traj_dt_since_latch_s = 0.0f;
    g_pitch_up_traj_theta_ref_planned_rad = PITCH_UP_IMU_ZERO_REF_RAD;
    g_pitch_up_traj_omega_ref_planned_rad_s = 0.0f;
    g_pitch_up_traj_alpha_ref_planned_rad_s2 = 0.0f;
    g_pitch_up_traj_last_vision_frame_seq = 0;
    PitchUpResetVisionAccFilter();
    (void)DWT_GetDeltaT(&pitch_up_traj_dwt_cnt);
}

static void PitchUpUpdateVisionTrajectory(float theta_in_rad,
                                          float omega_in_rad_s,
                                          float alpha_in_rad_s2,
                                          int32_t vision_frame_seq,
                                          float dt_s)
{
    const float change_eps = PITCH_UP_TRAJ_CHANGE_EPS_RAD;
    uint8_t new_packet = 0U;
    float alpha_use;
    float theta_out;
    float omega_out;
    float alpha_out;

    if (g_pitch_up_traj_planner_enable == 0U) {
        pitch_up_traj_initialized = 0U;
        pitch_up_ref_raw_rad = theta_in_rad;
        pitch_up_ref_vel_rad_s = omega_in_rad_s;
        pitch_up_ref_acc_rad_s2 = PitchUpFilterVisionAcc(alpha_in_rad_s2);
        g_pitch_up_traj_dt_since_latch_s = 0.0f;
        g_pitch_up_traj_theta_ref_planned_rad = pitch_up_ref_raw_rad;
        g_pitch_up_traj_omega_ref_planned_rad_s = pitch_up_ref_vel_rad_s;
        g_pitch_up_traj_alpha_ref_planned_rad_s2 = pitch_up_ref_acc_rad_s2;
        g_pitch_up_traj_last_vision_frame_seq = vision_frame_seq;
        return;
    }

    if (pitch_up_traj_initialized == 0U) {
        alpha_use = PitchUpFilterVisionAcc(alpha_in_rad_s2);
        pitch_up_traj_latched_theta_rad = theta_in_rad;
        pitch_up_traj_latched_omega_rad_s = omega_in_rad_s;
        pitch_up_traj_latched_alpha_rad_s2 = alpha_use;
        pitch_up_traj_last_input_theta_rad = theta_in_rad;
        pitch_up_traj_last_input_omega_rad_s = omega_in_rad_s;
        pitch_up_traj_last_input_alpha_rad_s2 = alpha_in_rad_s2;
        g_pitch_up_traj_dt_since_latch_s = 0.0f;
        g_pitch_up_traj_last_vision_frame_seq = vision_frame_seq;
        pitch_up_traj_initialized = 1U;

        theta_out = theta_in_rad;
        omega_out = omega_in_rad_s;
        alpha_out = alpha_use;
    } else {
        new_packet =
            (vision_frame_seq != g_pitch_up_traj_last_vision_frame_seq ||
             fabsf(theta_in_rad - pitch_up_traj_last_input_theta_rad) > change_eps ||
             fabsf(omega_in_rad_s - pitch_up_traj_last_input_omega_rad_s) > change_eps ||
             fabsf(alpha_in_rad_s2 - pitch_up_traj_last_input_alpha_rad_s2) > change_eps) ?
            1U : 0U;

        pitch_up_traj_last_input_theta_rad = theta_in_rad;
        pitch_up_traj_last_input_omega_rad_s = omega_in_rad_s;
        pitch_up_traj_last_input_alpha_rad_s2 = alpha_in_rad_s2;

        if (new_packet != 0U) {
            alpha_use = PitchUpFilterVisionAcc(alpha_in_rad_s2);
            pitch_up_traj_latched_theta_rad = theta_in_rad;
            pitch_up_traj_latched_omega_rad_s = omega_in_rad_s;
            pitch_up_traj_latched_alpha_rad_s2 = alpha_use;
            g_pitch_up_traj_dt_since_latch_s = 0.0f;
            g_pitch_up_traj_last_vision_frame_seq = vision_frame_seq;
        } else {
            g_pitch_up_traj_dt_since_latch_s += dt_s;
        }

        theta_out = pitch_up_traj_latched_theta_rad +
                    pitch_up_traj_latched_omega_rad_s *
                    g_pitch_up_traj_dt_since_latch_s +
                    0.5f * pitch_up_traj_latched_alpha_rad_s2 *
                    g_pitch_up_traj_dt_since_latch_s *
                    g_pitch_up_traj_dt_since_latch_s;
        omega_out = pitch_up_traj_latched_omega_rad_s +
                    pitch_up_traj_latched_alpha_rad_s2 *
                    g_pitch_up_traj_dt_since_latch_s;
        alpha_out = pitch_up_traj_latched_alpha_rad_s2;
    }

    pitch_up_ref_raw_rad = theta_out;
    pitch_up_ref_vel_rad_s = omega_out;
    pitch_up_ref_acc_rad_s2 = alpha_out;
    g_pitch_up_traj_theta_ref_planned_rad = theta_out;
    g_pitch_up_traj_omega_ref_planned_rad_s = omega_out;
    g_pitch_up_traj_alpha_ref_planned_rad_s2 = alpha_out;
}

static float PitchUpSelectRef(void)
{
    float ref_limited;

    if (pitch_up_ref_override_active != 0U) {
        pitch_up_auto_ref_active = 0U;
        pitch_up_ref_raw_rad = pitch_up_ref_override_rad;
        pitch_up_ref_vel_rad_s = 0.0f;
        pitch_up_ref_acc_rad_s2 = 0.0f;
        PitchUpResetVisionTrajectory();
        pitch_up_ref_source = PITCH_UP_REF_SOURCE_OVERRIDE;
    } else {
        pitch_up_auto_ref_active = PitchUpShouldUseAutoRef();
        if (pitch_up_auto_ref_active != 0U) {
            PitchUpUpdateVisionTrajectory(gimbal_cmd_recv.pitch_ref_rad,
                                          gimbal_cmd_recv.pitch_ref_vel_rad_s,
                                          gimbal_cmd_recv.pitch_ref_acc_rad_s2,
                                          gimbal_cmd_recv.vision_frame_seq,
                                          PitchUpVisionTrajDt());
            pitch_up_ref_source = PITCH_UP_REF_SOURCE_VISION;
        } else {
            pitch_up_ref_raw_rad = gimbal_cmd_recv.pitch;
            pitch_up_ref_vel_rad_s = 0.0f;
            pitch_up_ref_acc_rad_s2 = 0.0f;
            PitchUpResetVisionTrajectory();
            pitch_up_ref_source = PITCH_UP_REF_SOURCE_CMD;
        }
    }
    pitch_up_ref_limit_active = 0U;
    PitchUpGetImuLimits(&pitch_up_imu_lower_limit_rad,
                        &pitch_up_imu_upper_limit_rad);

#if PITCH_UP_SOFT_LIMIT_ENABLE
    ref_limited = PitchDownClampRange(pitch_up_ref_raw_rad,
                                      pitch_up_imu_lower_limit_rad,
                                      pitch_up_imu_upper_limit_rad);
    if (ref_limited != pitch_up_ref_raw_rad) {
        pitch_up_ref_limit_active = 1U;
        pitch_up_ref_vel_rad_s = 0.0f;
        pitch_up_ref_acc_rad_s2 = 0.0f;
        PitchUpResetVisionTrajectory();
    }
#else
    ref_limited = pitch_up_ref_raw_rad;
#endif

    pitch_up_ref_limited_rad = ref_limited;
    pitch_ref_limited = pitch_up_ref_limited_rad;
    return pitch_up_ref_limited_rad;
}

static float PitchUpAutoPidFiniteOrDefault(float value, float fallback)
{
    return (isfinite(value) != 0) ? value : fallback;
}

static float PitchUpAutoPidPositiveLimit(float value, float fallback)
{
    value = PitchUpAutoPidFiniteOrDefault(value, fallback);
    value = fabsf(value);
    fallback = fabsf(fallback);
    return (value > 0.0f) ? value : fallback;
}

static void PitchUpAutoPidClearDebug(void)
{
    pitch_up_auto_pid_active = 0U;
    pitch_up_auto_pid_angle_out_rad_s = 0.0f;
    pitch_up_auto_pid_speed_ref_rad_s = 0.0f;
    pitch_up_auto_pid_fb_torque_nm = 0.0f;
    pitch_up_auto_pid_acc_ff_torque_nm = 0.0f;
    pitch_up_auto_pid_torque_cmd_nm = 0.0f;
}

static void PitchUpAutoPidApplyRuntimeConfig(void)
{
    const float angle_limit =
        PitchUpAutoPidPositiveLimit(g_pitch_up_auto_pid_angle_out_limit_rad_s,
                                    PITCH_UP_AUTO_PID_ANGLE_OUT_LIMIT_RAD_S);
    const float torque_limit =
        PitchUpAutoPidPositiveLimit(g_pitch_up_auto_pid_torque_limit_nm,
                                    PITCH_UP_AUTO_PID_TORQUE_LIMIT_NM);

    pitch_up_auto_angle_pid.Kp =
        PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_angle_kp,
                                      PITCH_UP_AUTO_PID_ANGLE_KP_DEFAULT);
    pitch_up_auto_angle_pid.Ki =
        PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_angle_ki,
                                      PITCH_UP_AUTO_PID_ANGLE_KI_DEFAULT);
    pitch_up_auto_angle_pid.Kd =
        PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_angle_kd,
                                      PITCH_UP_AUTO_PID_ANGLE_KD_DEFAULT);
    pitch_up_auto_angle_pid.MaxOut = angle_limit;
    pitch_up_auto_angle_pid.DeadBand = 0.0f;
    pitch_up_auto_angle_pid.Improve = PID_Integral_Limit;
    pitch_up_auto_angle_pid.IntegralLimit = angle_limit;

    pitch_up_auto_speed_pid.Kp =
        PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_speed_kp,
                                      PITCH_UP_AUTO_PID_SPEED_KP_DEFAULT);
    pitch_up_auto_speed_pid.Ki =
        PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_speed_ki,
                                      PITCH_UP_AUTO_PID_SPEED_KI_DEFAULT);
    pitch_up_auto_speed_pid.Kd =
        PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_speed_kd,
                                      PITCH_UP_AUTO_PID_SPEED_KD_DEFAULT);
    pitch_up_auto_speed_pid.MaxOut = torque_limit;
    pitch_up_auto_speed_pid.DeadBand = 0.0f;
    pitch_up_auto_speed_pid.Improve = PID_Integral_Limit;
    pitch_up_auto_speed_pid.IntegralLimit = torque_limit;
}

static void PitchUpAutoPidReset(void)
{
    PID_Init_Config_s angle_config = {
        .Kp = PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_angle_kp,
                                            PITCH_UP_AUTO_PID_ANGLE_KP_DEFAULT),
        .Ki = PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_angle_ki,
                                            PITCH_UP_AUTO_PID_ANGLE_KI_DEFAULT),
        .Kd = PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_angle_kd,
                                            PITCH_UP_AUTO_PID_ANGLE_KD_DEFAULT),
        .MaxOut = PitchUpAutoPidPositiveLimit(g_pitch_up_auto_pid_angle_out_limit_rad_s,
                                              PITCH_UP_AUTO_PID_ANGLE_OUT_LIMIT_RAD_S),
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = PitchUpAutoPidPositiveLimit(g_pitch_up_auto_pid_angle_out_limit_rad_s,
                                                     PITCH_UP_AUTO_PID_ANGLE_OUT_LIMIT_RAD_S),
    };
    PID_Init_Config_s speed_config = {
        .Kp = PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_speed_kp,
                                            PITCH_UP_AUTO_PID_SPEED_KP_DEFAULT),
        .Ki = PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_speed_ki,
                                            PITCH_UP_AUTO_PID_SPEED_KI_DEFAULT),
        .Kd = PitchUpAutoPidFiniteOrDefault(g_pitch_up_auto_pid_speed_kd,
                                            PITCH_UP_AUTO_PID_SPEED_KD_DEFAULT),
        .MaxOut = PitchUpAutoPidPositiveLimit(g_pitch_up_auto_pid_torque_limit_nm,
                                              PITCH_UP_AUTO_PID_TORQUE_LIMIT_NM),
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = PitchUpAutoPidPositiveLimit(g_pitch_up_auto_pid_torque_limit_nm,
                                                     PITCH_UP_AUTO_PID_TORQUE_LIMIT_NM),
    };

    PIDInit(&pitch_up_auto_angle_pid, &angle_config);
    PIDInit(&pitch_up_auto_speed_pid, &speed_config);
    pitch_up_auto_pid_last_active = 0U;
    PitchUpAutoPidClearDebug();
}

static float PitchUpCalculateAutoPidTorque(void)
{
    const float vision_pitch_ref_rad =
        PitchUpAutoPidFiniteOrDefault(gimbal_cmd_recv.pitch_ref_rad,
                                      pitch_up_measure_rad);
    const float vision_pitch_vel_rad_s =
        PitchUpAutoPidFiniteOrDefault(gimbal_cmd_recv.pitch_ref_vel_rad_s,
                                      0.0f);
    const float vision_pitch_acc_rad_s2 =
        PitchUpAutoPidFiniteOrDefault(gimbal_cmd_recv.pitch_ref_acc_rad_s2,
                                      0.0f);
    float model_j =
        PitchUpAutoPidFiniteOrDefault(g_pitch_up_model_j_kg_m2,
                                      PITCH_UP_MODEL_J_KG_M2);
    float torque_limit;
    float torque_cmd;

    if (model_j <= 0.0f) {
        model_j = PITCH_UP_MODEL_J_KG_M2;
    }

    PitchUpAutoPidApplyRuntimeConfig();
    pitch_up_auto_pid_angle_out_rad_s =
        PIDCalculate(&pitch_up_auto_angle_pid,
                     pitch_up_measure_rad,
                     vision_pitch_ref_rad);
    pitch_up_auto_pid_speed_ref_rad_s =
        pitch_up_auto_pid_angle_out_rad_s + vision_pitch_vel_rad_s;
    pitch_up_auto_pid_fb_torque_nm =
        PIDCalculate(&pitch_up_auto_speed_pid,
                     pitch_up_gyro_rad_s,
                     pitch_up_auto_pid_speed_ref_rad_s);
    pitch_up_auto_pid_acc_ff_torque_nm =
        model_j * vision_pitch_acc_rad_s2;

    torque_limit = PitchUpAutoPidPositiveLimit(g_pitch_up_auto_pid_torque_limit_nm,
                                               PITCH_UP_AUTO_PID_TORQUE_LIMIT_NM);
    torque_cmd =
        pitch_up_auto_pid_fb_torque_nm +
        pitch_up_auto_pid_acc_ff_torque_nm;
    torque_cmd = PitchDownClampRange(torque_cmd,
                                     -torque_limit,
                                     torque_limit);

    pitch_up_auto_pid_active = 1U;
    pitch_up_auto_pid_torque_cmd_nm = torque_cmd;
    pitch_up_lqr_torque_nm = pitch_up_auto_pid_fb_torque_nm;
    pitch_up_gravity_torque_ff_nm = pitch_up_auto_pid_acc_ff_torque_nm;

    return torque_cmd;
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

static uint8_t PitchUpNonHoleDownZeroTorqueActive(void)
{
    float down_pos_rad;

    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) {
        return 0U;
    }
    if (gimbal_cmd_recv.hole_restore_active != 0U ||
        hole_restore_up_enable_latched != 0U) {
        return 0U;
    }

    down_pos_rad =
        (pitch_motor_down == NULL) ? pitch_down_pos_rad : pitch_motor_down->measure.pos;

    return (down_pos_rad < PITCH_UP_NON_HOLE_DOWN_ZERO_TORQUE_RAD) ? 1U : 0U;
}

static void PitchUpApplyZeroTorqueProtection(void)
{
    PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
    pitch_up_lqr_torque_nm = 0.0f;
    pitch_up_gravity_torque_ff_nm = 0.0f;
    pitch_up_torque_cmd_nm = 0.0f;
    dzx_pitch_torque = 0.0f;
    PitchUpResetModelFeedforwardDebug();
    PitchUpAutoPidReset();
    pitch_up_soft_limit_active = 0U;
    pitch_up_auto_ref_last_active = 0U;
}

static void PitchUpClearDoublePitchSmallOutput(DoublePitchLqrOutput_s *output)
{
    if (output == NULL) {
        return;
    }

    output->tau_small_cmd_nm = 0.0f;
    output->tau_small_pre_limit_nm = 0.0f;
    output->tau_small_fb_nm = 0.0f;
    output->tau_small_g_nm = 0.0f;
    output->tau_small_limit_flag = 0U;
}

static uint8_t PitchUpTorqueOutputAllowed(void)
{
    if (PitchDownIdentAutoActive() != 0U) {
        return 1U;
    }

    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) {
        return 1U;
    }
    if (gimbal_cmd_recv.hole_restore_active != 0U ||
        hole_restore_up_enable_latched != 0U) {
        return 1U;
    }

    if (pitch_motor_down == NULL || !DMMotorIsOnline(pitch_motor_down)) {
        return 0U;
    }

    if (pitch_motor_down->measure.pos >= PITCH_DOWN_IDENT_LOWER_RAD &&
        pitch_motor_down->measure.pos <= PITCH_DOWN_IDENT_LQR_UPPER_RAD) {
        return 1U;
    }

    return (pitch_motor_down->measure.pos >= PITCH_DOWN_MIT_SWITCH_POS_RAD) ? 1U : 0U;
}

static uint8_t PitchUpGAutoActive(void)
{
    return (g_pitch_up_g_auto_running != 0U ||
            g_pitch_up_g_auto_start != 0U) ? 1U : 0U;
}

static float PitchUpGAutoTargetRad(uint8_t pass, uint8_t point_index)
{
    uint8_t index = point_index;

    if (index >= PITCH_UP_G_AUTO_POINT_COUNT) {
        index = PITCH_UP_G_AUTO_POINT_COUNT - 1U;
    }
    if (pass == 0U) {
        index = (PITCH_UP_G_AUTO_POINT_COUNT - 1U) - index;
    }

    return pitch_up_g_auto_points_rad[index];
}

static uint8_t PitchUpGAutoModeAllowed(void)
{
    return (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE ||
            gimbal_cmd_recv.gimbal_mode == GIMBAL_MOTOR_MODE) ? 1U : 0U;
}

static void PitchUpGAutoSetPhase(uint8_t phase)
{
    g_pitch_up_g_auto_phase = phase;
    pitch_up_g_auto_phase_time_s = 0.0f;
    pitch_up_g_auto_stable_time_s = 0.0f;
    g_pitch_up_g_auto_sample_active =
        (phase == PITCH_UP_G_AUTO_PHASE_SAMPLE) ? 1U : 0U;
}

static void PitchUpGAutoStop(uint8_t phase)
{
    g_pitch_up_g_auto_running = 0U;
    g_pitch_up_g_auto_sample_active = 0U;
    g_pitch_up_g_auto_start = 0U;
    g_pitch_up_g_auto_cancel = 0U;
    g_pitch_up_g_auto_phase = phase;
    if (phase == PITCH_UP_G_AUTO_PHASE_DONE) {
        g_pitch_up_g_auto_done = 1U;
        g_pitch_up_g_auto_abort = 0U;
    } else {
        g_pitch_up_g_auto_done = 0U;
        g_pitch_up_g_auto_abort = 1U;
    }
    PitchUpClearRefOverride();
}

static void PitchUpGAutoAbortIfActive(void)
{
    if (g_pitch_up_g_auto_running != 0U ||
        g_pitch_up_g_auto_start != 0U) {
        PitchUpGAutoStop(PITCH_UP_G_AUTO_PHASE_ABORT);
    }
}

static void PitchUpGAutoStart(void)
{
    g_pitch_up_dyn_auto_done = 0U;
    g_pitch_up_dyn_auto_abort = 0U;
    if (g_pitch_up_dyn_auto_running == 0U) {
        g_pitch_up_dyn_auto_phase = PITCH_UP_DYN_AUTO_PHASE_IDLE;
        g_pitch_up_dyn_auto_sample_active = 0U;
    }
    g_pitch_up_g_auto_start = 0U;
    g_pitch_up_g_auto_cancel = 0U;
    g_pitch_up_g_auto_done = 0U;
    g_pitch_up_g_auto_abort = 0U;
    g_pitch_up_g_auto_running = 1U;
    g_pitch_up_g_auto_pass = 0U;
    g_pitch_up_g_auto_point_index = 0U;
    g_pitch_up_g_auto_target_rad = PitchUpGAutoTargetRad(0U, 0U);
    pitch_up_g_auto_ref_rad = g_pitch_up_g_auto_target_rad;
    PitchUpGAutoSetPhase(PITCH_UP_G_AUTO_PHASE_SETTLE);
    PitchUpSetRefOverride(pitch_up_g_auto_ref_rad);
}

static void PitchUpGAutoNextPoint(void)
{
    g_pitch_up_g_auto_point_index++;
    if (g_pitch_up_g_auto_point_index >= PITCH_UP_G_AUTO_POINT_COUNT) {
        if (g_pitch_up_g_auto_pass == 0U) {
            g_pitch_up_g_auto_pass = 1U;
            g_pitch_up_g_auto_point_index = 0U;
        } else {
            PitchUpGAutoStop(PITCH_UP_G_AUTO_PHASE_DONE);
            return;
        }
    }

    g_pitch_up_g_auto_target_rad =
        PitchUpGAutoTargetRad(g_pitch_up_g_auto_pass,
                              g_pitch_up_g_auto_point_index);
    pitch_up_g_auto_ref_rad = g_pitch_up_g_auto_target_rad;
    PitchUpGAutoSetPhase(PITCH_UP_G_AUTO_PHASE_SETTLE);
    PitchUpSetRefOverride(pitch_up_g_auto_ref_rad);
}

static void PitchUpGAutoUpdate(float dt_s)
{
    float target_rad;
    const float point_time_s =
        PITCH_UP_G_AUTO_SETTLE_TIME_S + PITCH_UP_G_AUTO_SAMPLE_TIME_S;

    if (dt_s < 0.0001f || dt_s > 0.05f) {
        dt_s = 0.001f;
    }

    if (g_pitch_up_g_auto_cancel != 0U) {
        if (g_pitch_up_g_auto_running != 0U || g_pitch_up_g_auto_start != 0U) {
            PitchUpGAutoStop(PITCH_UP_G_AUTO_PHASE_ABORT);
        } else {
            g_pitch_up_g_auto_cancel = 0U;
            PitchUpClearRefOverride();
        }
        return;
    }

    if (g_pitch_up_g_auto_running == 0U) {
        if (g_pitch_up_g_auto_start == 0U) {
            return;
        }
        PitchUpGAutoStart();
    } else if (g_pitch_up_g_auto_start != 0U) {
        g_pitch_up_g_auto_start = 0U;
    }

    target_rad = g_pitch_up_g_auto_target_rad;
    PitchUpSetRefOverride(target_rad);
    pitch_up_g_auto_phase_time_s += dt_s;
    if (pitch_up_g_auto_phase_time_s >= point_time_s) {
        PitchUpGAutoNextPoint();
    } else if (pitch_up_g_auto_phase_time_s >=
               PITCH_UP_G_AUTO_SETTLE_TIME_S) {
        g_pitch_up_g_auto_phase = PITCH_UP_G_AUTO_PHASE_SAMPLE;
        g_pitch_up_g_auto_sample_active = 1U;
    } else {
        g_pitch_up_g_auto_phase = PITCH_UP_G_AUTO_PHASE_SETTLE;
        g_pitch_up_g_auto_sample_active = 0U;
    }
}

static uint8_t PitchUpDynAutoProfileValid(uint8_t profile)
{
    return (profile == PITCH_UP_DYN_PROFILE_LOW_SPEED ||
            profile == PITCH_UP_DYN_PROFILE_CHIRP ||
            profile == PITCH_UP_DYN_PROFILE_FRICTION) ? 1U : 0U;
}

static float PitchUpDynFrictionSpeed(uint8_t index)
{
    if (index >= PITCH_UP_DYN_FRICTION_SPEED_COUNT) {
        index = PITCH_UP_DYN_FRICTION_SPEED_COUNT - 1U;
    }
    return pitch_up_dyn_friction_speed_rad_s[index];
}

static float PitchUpDynFrictionDurationS(void)
{
    float duration_s =
        PITCH_UP_DYN_FRICTION_AMP_RAD / PitchUpDynFrictionSpeed(0U) +
        PITCH_UP_DYN_FRICTION_EDGE_SETTLE_S;

    for (uint8_t i = 0U; i < PITCH_UP_DYN_FRICTION_SPEED_COUNT; i++) {
        const float leg_time_s =
            (2.0f * PITCH_UP_DYN_FRICTION_AMP_RAD) /
            PitchUpDynFrictionSpeed(i);
        duration_s +=
            (float)PITCH_UP_DYN_FRICTION_CYCLE_COUNT *
            (2.0f * leg_time_s +
             2.0f * PITCH_UP_DYN_FRICTION_EDGE_SETTLE_S);
    }

    return duration_s;
}

static void PitchUpDynAutoEvaluateFriction(float t_s)
{
    const float center = PITCH_UP_DYN_CENTER_RAD;
    const float min_ref = center - PITCH_UP_DYN_FRICTION_AMP_RAD;
    const float max_ref = center + PITCH_UP_DYN_FRICTION_AMP_RAD;
    const float travel_rad = 2.0f * PITCH_UP_DYN_FRICTION_AMP_RAD;
    float remain_s = t_s;
    float speed_rad_s;
    float segment_s;

    g_pitch_up_dyn_auto_sample_active = 0U;
    g_pitch_up_dyn_auto_ref_acc_rad_s2 = 0.0f;
    g_pitch_up_dyn_auto_freq_hz = 0.0f;

    speed_rad_s = PitchUpDynFrictionSpeed(0U);
    segment_s = PITCH_UP_DYN_FRICTION_AMP_RAD / speed_rad_s;
    if (remain_s < segment_s) {
        g_pitch_up_dyn_auto_ref_rad = center - speed_rad_s * remain_s;
        g_pitch_up_dyn_auto_ref_vel_rad_s = -speed_rad_s;
        return;
    }
    remain_s -= segment_s;

    if (remain_s < PITCH_UP_DYN_FRICTION_EDGE_SETTLE_S) {
        g_pitch_up_dyn_auto_ref_rad = min_ref;
        g_pitch_up_dyn_auto_ref_vel_rad_s = 0.0f;
        return;
    }
    remain_s -= PITCH_UP_DYN_FRICTION_EDGE_SETTLE_S;

    for (uint8_t speed_index = 0U;
         speed_index < PITCH_UP_DYN_FRICTION_SPEED_COUNT;
         speed_index++) {
        speed_rad_s = PitchUpDynFrictionSpeed(speed_index);
        segment_s = travel_rad / speed_rad_s;

        for (uint8_t cycle = 0U;
             cycle < PITCH_UP_DYN_FRICTION_CYCLE_COUNT;
             cycle++) {
            if (remain_s < segment_s) {
                g_pitch_up_dyn_auto_ref_rad =
                    min_ref + speed_rad_s * remain_s;
                g_pitch_up_dyn_auto_ref_vel_rad_s = speed_rad_s;
                g_pitch_up_dyn_auto_sample_active = 1U;
                return;
            }
            remain_s -= segment_s;

            if (remain_s < PITCH_UP_DYN_FRICTION_EDGE_SETTLE_S) {
                g_pitch_up_dyn_auto_ref_rad = max_ref;
                g_pitch_up_dyn_auto_ref_vel_rad_s = 0.0f;
                return;
            }
            remain_s -= PITCH_UP_DYN_FRICTION_EDGE_SETTLE_S;

            if (remain_s < segment_s) {
                g_pitch_up_dyn_auto_ref_rad =
                    max_ref - speed_rad_s * remain_s;
                g_pitch_up_dyn_auto_ref_vel_rad_s = -speed_rad_s;
                g_pitch_up_dyn_auto_sample_active = 1U;
                return;
            }
            remain_s -= segment_s;

            if (remain_s < PITCH_UP_DYN_FRICTION_EDGE_SETTLE_S) {
                g_pitch_up_dyn_auto_ref_rad = min_ref;
                g_pitch_up_dyn_auto_ref_vel_rad_s = 0.0f;
                return;
            }
            remain_s -= PITCH_UP_DYN_FRICTION_EDGE_SETTLE_S;
        }
    }

    g_pitch_up_dyn_auto_ref_rad = min_ref;
    g_pitch_up_dyn_auto_ref_vel_rad_s = 0.0f;
}

static void PitchUpDynAutoSetPhase(uint8_t phase)
{
    g_pitch_up_dyn_auto_phase = phase;
    pitch_up_dyn_auto_phase_time_s = 0.0f;
    pitch_up_dyn_auto_stable_time_s = 0.0f;
    g_pitch_up_dyn_auto_sample_active =
        (phase == PITCH_UP_DYN_AUTO_PHASE_RUN &&
         g_pitch_up_dyn_auto_profile != PITCH_UP_DYN_PROFILE_FRICTION) ?
        1U : 0U;
    if (phase == PITCH_UP_DYN_AUTO_PHASE_RUN) {
        g_pitch_up_dyn_auto_time_s = 0.0f;
    }
}

static void PitchUpDynAutoStop(uint8_t phase)
{
    g_pitch_up_dyn_auto_running = 0U;
    g_pitch_up_dyn_auto_sample_active = 0U;
    g_pitch_up_dyn_auto_start = 0U;
    g_pitch_up_dyn_auto_cancel = 0U;
    g_pitch_up_dyn_auto_phase = phase;
    g_pitch_up_dyn_auto_ref_vel_rad_s = 0.0f;
    g_pitch_up_dyn_auto_ref_acc_rad_s2 = 0.0f;
    g_pitch_up_dyn_auto_freq_hz = 0.0f;
    if (phase == PITCH_UP_DYN_AUTO_PHASE_DONE) {
        g_pitch_up_dyn_auto_done = 1U;
        g_pitch_up_dyn_auto_abort = 0U;
    } else {
        g_pitch_up_dyn_auto_done = 0U;
        g_pitch_up_dyn_auto_abort = 1U;
    }
    PitchUpClearRefOverride();
}

static void PitchUpDynAutoAbortIfActive(void)
{
    if (g_pitch_up_dyn_auto_running != 0U ||
        g_pitch_up_dyn_auto_start != 0U) {
        PitchUpDynAutoStop(PITCH_UP_DYN_AUTO_PHASE_ABORT);
    }
}

static void PitchUpDynAutoEvaluate(float t_s)
{
    const float center = PITCH_UP_DYN_CENTER_RAD;
    float phase;
    float phase_dot;

    if (g_pitch_up_dyn_auto_profile == PITCH_UP_DYN_PROFILE_FRICTION) {
        PitchUpDynAutoEvaluateFriction(t_s);
    } else if (g_pitch_up_dyn_auto_profile == PITCH_UP_DYN_PROFILE_CHIRP) {
        const float duration_s = PITCH_UP_DYN_CHIRP_DURATION_S;
        const float k_hz_s =
            (PITCH_UP_DYN_CHIRP_F_END_HZ -
             PITCH_UP_DYN_CHIRP_F_START_HZ) / duration_s;
        const float freq_hz = PITCH_UP_DYN_CHIRP_F_START_HZ + k_hz_s * t_s;
        const float phase_ddot = PITCH_UP_DYN_TWO_PI * k_hz_s;

        g_pitch_up_dyn_auto_sample_active = 1U;
        phase =
            PITCH_UP_DYN_TWO_PI *
            (PITCH_UP_DYN_CHIRP_F_START_HZ * t_s +
             0.5f * k_hz_s * t_s * t_s);
        phase_dot = PITCH_UP_DYN_TWO_PI * freq_hz;
        g_pitch_up_dyn_auto_ref_rad =
            center + PITCH_UP_DYN_CHIRP_AMP_RAD * sinf(phase);
        g_pitch_up_dyn_auto_ref_vel_rad_s =
            PITCH_UP_DYN_CHIRP_AMP_RAD * cosf(phase) * phase_dot;
        g_pitch_up_dyn_auto_ref_acc_rad_s2 =
            PITCH_UP_DYN_CHIRP_AMP_RAD *
            (cosf(phase) * phase_ddot -
             sinf(phase) * phase_dot * phase_dot);
        g_pitch_up_dyn_auto_freq_hz = freq_hz;
    } else {
        g_pitch_up_dyn_auto_sample_active = 1U;
        phase = PITCH_UP_DYN_TWO_PI * PITCH_UP_DYN_LOW_FREQ_HZ * t_s;
        phase_dot = PITCH_UP_DYN_TWO_PI * PITCH_UP_DYN_LOW_FREQ_HZ;
        g_pitch_up_dyn_auto_ref_rad =
            center + PITCH_UP_DYN_LOW_AMP_RAD * sinf(phase);
        g_pitch_up_dyn_auto_ref_vel_rad_s =
            PITCH_UP_DYN_LOW_AMP_RAD * cosf(phase) * phase_dot;
        g_pitch_up_dyn_auto_ref_acc_rad_s2 =
            -PITCH_UP_DYN_LOW_AMP_RAD * sinf(phase) *
            phase_dot * phase_dot;
        g_pitch_up_dyn_auto_freq_hz = PITCH_UP_DYN_LOW_FREQ_HZ;
    }
}

static float PitchUpDynAutoDurationS(void)
{
    if (g_pitch_up_dyn_auto_profile == PITCH_UP_DYN_PROFILE_CHIRP) {
        return PITCH_UP_DYN_CHIRP_DURATION_S;
    }
    if (g_pitch_up_dyn_auto_profile == PITCH_UP_DYN_PROFILE_FRICTION) {
        return PitchUpDynFrictionDurationS();
    }
    return PITCH_UP_DYN_LOW_DURATION_S;
}

static void PitchUpDynAutoStart(float measure_rad)
{
    g_pitch_up_g_auto_done = 0U;
    g_pitch_up_g_auto_abort = 0U;
    if (g_pitch_up_g_auto_running == 0U) {
        g_pitch_up_g_auto_phase = PITCH_UP_G_AUTO_PHASE_IDLE;
        g_pitch_up_g_auto_sample_active = 0U;
    }
    g_pitch_up_dyn_auto_start = 0U;
    g_pitch_up_dyn_auto_cancel = 0U;
    g_pitch_up_dyn_auto_done = 0U;
    g_pitch_up_dyn_auto_abort = 0U;
    g_pitch_up_dyn_auto_running = 1U;
    g_pitch_up_dyn_auto_time_s = 0.0f;
    g_pitch_up_dyn_auto_ref_rad = measure_rad;
    g_pitch_up_dyn_auto_ref_vel_rad_s = 0.0f;
    g_pitch_up_dyn_auto_ref_acc_rad_s2 = 0.0f;
    g_pitch_up_dyn_auto_freq_hz = 0.0f;
    PitchUpDynAutoSetPhase(PITCH_UP_DYN_AUTO_PHASE_RAMP);
    PitchUpSetRefOverride(g_pitch_up_dyn_auto_ref_rad);
}

static void PitchUpDynAutoUpdate(float dt_s)
{
    float measure_rad = pitch_up_measure_rad;
    float gyro_rad_s = pitch_up_gyro_rad_s;
    float error_rad;
    float max_delta_rad;
    uint8_t pos_ready;
    uint8_t speed_ready;

    if (dt_s < 0.0001f || dt_s > 0.05f) {
        dt_s = 0.001f;
    }

    if (gimbal_IMU_data != NULL) {
        measure_rad =
            gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
        gyro_rad_s =
            gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
    }

    if (g_pitch_up_dyn_auto_cancel != 0U) {
        if (g_pitch_up_dyn_auto_running != 0U ||
            g_pitch_up_dyn_auto_start != 0U) {
            PitchUpDynAutoStop(PITCH_UP_DYN_AUTO_PHASE_ABORT);
        } else {
            g_pitch_up_dyn_auto_cancel = 0U;
            g_pitch_up_dyn_auto_done = 0U;
            g_pitch_up_dyn_auto_abort = 0U;
            g_pitch_up_dyn_auto_phase = PITCH_UP_DYN_AUTO_PHASE_IDLE;
            PitchUpClearRefOverride();
        }
        return;
    }

    if (g_pitch_up_dyn_auto_running == 0U) {
        if (g_pitch_up_dyn_auto_start == 0U) {
            return;
        }
        if (PitchUpDynAutoProfileValid(g_pitch_up_dyn_auto_profile) == 0U ||
            PitchUpGAutoModeAllowed() == 0U ||
            gimbal_IMU_data == NULL ||
            pitch_motor_up == NULL ||
            DMMotorIsOnline(pitch_motor_up) == 0U ||
            PitchUpTorqueOutputAllowed() == 0U ||
            g_pitch_up_g_auto_running != 0U ||
            g_pitch_up_g_auto_start != 0U) {
            PitchUpDynAutoStop(PITCH_UP_DYN_AUTO_PHASE_ABORT);
            return;
        }
        PitchUpDynAutoStart(measure_rad);
    } else if (g_pitch_up_dyn_auto_start != 0U) {
        g_pitch_up_dyn_auto_start = 0U;
    }

    if (PitchUpGAutoModeAllowed() == 0U ||
        gimbal_IMU_data == NULL ||
        pitch_motor_up == NULL ||
        DMMotorIsOnline(pitch_motor_up) == 0U ||
        PitchUpTorqueOutputAllowed() == 0U ||
        pitch_up_ref_limit_active != 0U ||
        pitch_up_soft_limit_active != 0U ||
        (g_pitch_up_dyn_auto_phase == PITCH_UP_DYN_AUTO_PHASE_RUN &&
         fabsf(dzx_pitch_lqr_eso.tau_pre_limit -
               dzx_pitch_lqr_eso.output) > 1e-6f)) {
        PitchUpDynAutoStop(PITCH_UP_DYN_AUTO_PHASE_ABORT);
        return;
    }

    switch (g_pitch_up_dyn_auto_phase) {
    case PITCH_UP_DYN_AUTO_PHASE_RAMP:
        pitch_up_dyn_auto_phase_time_s += dt_s;
        error_rad = PITCH_UP_DYN_CENTER_RAD - g_pitch_up_dyn_auto_ref_rad;
        max_delta_rad = PITCH_UP_DYN_RAMP_RATE_RAD_S * dt_s;
        if (fabsf(error_rad) <= max_delta_rad) {
            g_pitch_up_dyn_auto_ref_rad = PITCH_UP_DYN_CENTER_RAD;
        } else if (error_rad > 0.0f) {
            g_pitch_up_dyn_auto_ref_rad += max_delta_rad;
        } else {
            g_pitch_up_dyn_auto_ref_rad -= max_delta_rad;
        }
        g_pitch_up_dyn_auto_ref_vel_rad_s = 0.0f;
        g_pitch_up_dyn_auto_ref_acc_rad_s2 = 0.0f;
        g_pitch_up_dyn_auto_freq_hz = 0.0f;
        PitchUpSetRefOverride(g_pitch_up_dyn_auto_ref_rad);
        if (pitch_up_dyn_auto_phase_time_s > PITCH_UP_DYN_MOVE_TIMEOUT_S) {
            PitchUpDynAutoStop(PITCH_UP_DYN_AUTO_PHASE_ABORT);
            return;
        }
        if (fabsf(measure_rad - PITCH_UP_DYN_CENTER_RAD) <=
            PITCH_UP_DYN_READY_POS_RAD &&
            fabsf(gyro_rad_s) <= PITCH_UP_DYN_READY_SPEED_RAD_S) {
            g_pitch_up_dyn_auto_ref_rad = PITCH_UP_DYN_CENTER_RAD;
            PitchUpSetRefOverride(g_pitch_up_dyn_auto_ref_rad);
            PitchUpDynAutoSetPhase(PITCH_UP_DYN_AUTO_PHASE_SETTLE);
        }
        break;

    case PITCH_UP_DYN_AUTO_PHASE_SETTLE:
        PitchUpSetRefOverride(PITCH_UP_DYN_CENTER_RAD);
        g_pitch_up_dyn_auto_ref_rad = PITCH_UP_DYN_CENTER_RAD;
        g_pitch_up_dyn_auto_ref_vel_rad_s = 0.0f;
        g_pitch_up_dyn_auto_ref_acc_rad_s2 = 0.0f;
        g_pitch_up_dyn_auto_freq_hz = 0.0f;
        pitch_up_dyn_auto_phase_time_s += dt_s;
        pos_ready =
            (fabsf(measure_rad - PITCH_UP_DYN_CENTER_RAD) <=
             PITCH_UP_DYN_READY_POS_RAD) ? 1U : 0U;
        speed_ready =
            (fabsf(gyro_rad_s) <= PITCH_UP_DYN_READY_SPEED_RAD_S) ?
            1U : 0U;
        if (pos_ready != 0U && speed_ready != 0U) {
            pitch_up_dyn_auto_stable_time_s += dt_s;
        } else {
            pitch_up_dyn_auto_stable_time_s = 0.0f;
        }
        if (pitch_up_dyn_auto_stable_time_s >= PITCH_UP_DYN_SETTLE_TIME_S ||
            (pos_ready != 0U &&
             pitch_up_dyn_auto_phase_time_s >= PITCH_UP_DYN_SETTLE_TIMEOUT_S)) {
            PitchUpDynAutoEvaluate(0.0f);
            PitchUpSetRefOverride(g_pitch_up_dyn_auto_ref_rad);
            PitchUpDynAutoSetPhase(PITCH_UP_DYN_AUTO_PHASE_RUN);
        } else if (pos_ready == 0U &&
                   pitch_up_dyn_auto_phase_time_s > PITCH_UP_DYN_MOVE_TIMEOUT_S) {
            PitchUpDynAutoStop(PITCH_UP_DYN_AUTO_PHASE_ABORT);
            return;
        }
        break;

    case PITCH_UP_DYN_AUTO_PHASE_RUN:
        if (g_pitch_up_dyn_auto_time_s >= PitchUpDynAutoDurationS()) {
            PitchUpDynAutoStop(PITCH_UP_DYN_AUTO_PHASE_DONE);
            return;
        }
        PitchUpDynAutoEvaluate(g_pitch_up_dyn_auto_time_s);
        PitchUpSetRefOverride(g_pitch_up_dyn_auto_ref_rad);
        pitch_up_dyn_auto_phase_time_s += dt_s;
        g_pitch_up_dyn_auto_time_s += dt_s;
        break;

    default:
        PitchUpDynAutoStop(PITCH_UP_DYN_AUTO_PHASE_ABORT);
        break;
    }
}

static void PitchUpResetTelemetry(void)
{
    pitch_up_ref_raw_rad          = PITCH_UP_IMU_ZERO_REF_RAD;
    pitch_up_ref_rad              = 0.0f;
    pitch_up_ref_vel_rad_s        = 0.0f;
    pitch_up_ref_acc_rad_s2       = 0.0f;
    pitch_up_ref_limited_rad      = PITCH_UP_IMU_ZERO_REF_RAD;
    pitch_up_ref_source           = PITCH_UP_REF_SOURCE_NONE;
    pitch_up_imu_lower_limit_rad  = 0.0f;
    pitch_up_imu_upper_limit_rad  = 0.0f;
    pitch_up_measure_rad          = 0.0f;
    pitch_up_gyro_rad_s           = 0.0f;
    pitch_up_motor_pos_rad        = 0.0f;
    pitch_up_motor_vel_rad_s      = 0.0f;
    pitch_up_lqr_torque_nm        = 0.0f;
    pitch_up_gravity_torque_ff_nm = 0.0f;
    pitch_up_torque_cmd_nm        = 0.0f;
    PitchUpResetModelFeedforwardDebug();
    PitchUpAutoPidReset();
    pitch_up_soft_limit_active    = 0U;
    pitch_up_ref_limit_active     = 0U;
    pitch_up_auto_ref_active      = 0U;
    pitch_up_auto_ref_last_active = 0U;
    PitchUpResetVisionTrajectory();
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

static uint8_t PitchUpIdentAutoActive(void)
{
    return (g_pitch_up_ident_running != 0U ||
            g_pitch_up_ident_enable != 0U) ? 1U : 0U;
}

static uint8_t PitchUpIdentAutoFeedbackOk(void)
{
    if (pitch_motor_up == NULL ||
        pitch_motor_down == NULL ||
        gimbal_IMU_data == NULL) {
        return 0U;
    }
    if (DMMotorIsOnline(pitch_motor_up) == 0U ||
        DMMotorIsOnline(pitch_motor_down) == 0U) {
        return 0U;
    }
    return 1U;
}

static float PitchUpIdentAutoGravityTarget(uint8_t pass,
                                           uint8_t point_index)
{
    uint8_t index = point_index;

    if (index >= PITCH_UP_IDENT_AUTO_G_POINT_COUNT) {
        index = PITCH_UP_IDENT_AUTO_G_POINT_COUNT - 1U;
    }
    if (pass != 0U) {
        index = (PITCH_UP_IDENT_AUTO_G_POINT_COUNT - 1U) - index;
    }
    return pitch_up_ident_g_points_rad[index];
}

static float PitchUpIdentAutoFrictionSpeed(uint8_t index)
{
    if (index >= PITCH_UP_IDENT_AUTO_FRICTION_SPEED_COUNT) {
        index = PITCH_UP_IDENT_AUTO_FRICTION_SPEED_COUNT - 1U;
    }
    return pitch_up_ident_friction_speed_rad_s[index];
}

static uint8_t PitchUpIdentAutoNormalizeStage(uint8_t stage)
{
    if (stage >= PITCH_UP_IDENT_STAGE_GRAVITY_UP &&
        stage <= PITCH_UP_IDENT_STAGE_CHIRP) {
        return stage;
    }
    return PITCH_UP_IDENT_STAGE_DEFAULT;
}

static uint8_t PitchUpIdentAutoStageProfile(uint8_t stage)
{
    if (stage == PITCH_UP_IDENT_STAGE_GRAVITY_UP ||
        stage == PITCH_UP_IDENT_STAGE_GRAVITY_DOWN) {
        return PITCH_UP_IDENT_AUTO_PROFILE_GRAVITY;
    }
    if (stage == PITCH_UP_IDENT_STAGE_F_006 ||
        stage == PITCH_UP_IDENT_STAGE_F_009) {
        return PITCH_UP_IDENT_AUTO_PROFILE_FRICTION;
    }
    return PITCH_UP_IDENT_AUTO_PROFILE_CHIRP;
}

static uint8_t PitchUpIdentAutoStageFrictionIndex(uint8_t stage)
{
    if (stage == PITCH_UP_IDENT_STAGE_F_009) {
        return 1U;
    }
    return 0U;
}

static void PitchUpIdentAutoSetPhase(uint8_t phase)
{
    g_pitch_up_ident_auto_phase = phase;
    g_pitch_up_ident_auto_phase_time_s = 0.0f;
    g_pitch_up_ident_auto_sample_active = 0U;
}

static void PitchUpIdentAutoSetRef(float ref_rad,
                                   float ref_vel_rad_s,
                                   float ref_acc_rad_s2,
                                   float target_rad,
                                   float freq_hz,
                                   float speed_target_rad_s,
                                   uint8_t sample_active)
{
    g_pitch_up_ident_auto_target_rad = target_rad;
    g_pitch_up_ident_auto_ref_rad = ref_rad;
    g_pitch_up_ident_auto_ref_vel_rad_s = ref_vel_rad_s;
    g_pitch_up_ident_auto_ref_acc_rad_s2 = ref_acc_rad_s2;
    g_pitch_up_ident_auto_freq_hz = freq_hz;
    g_pitch_up_ident_auto_speed_target_rad_s = speed_target_rad_s;
    g_pitch_up_ident_auto_sample_active = (sample_active != 0U) ? 1U : 0U;
    PitchUpSetRefOverride(ref_rad);
}

static void PitchUpIdentAutoStop(uint8_t phase, uint8_t abort_reason)
{
    g_pitch_up_ident_running = 0U;
    g_pitch_up_ident_enable = 0U;
    g_pitch_up_ident_auto_sample_active = 0U;
    g_pitch_up_ident_auto_phase = phase;
    g_pitch_up_ident_abort_reason = abort_reason;
    g_pitch_up_ident_auto_ref_vel_rad_s = 0.0f;
    g_pitch_up_ident_auto_ref_acc_rad_s2 = 0.0f;
    g_pitch_up_ident_auto_freq_hz = 0.0f;
    g_pitch_up_ident_auto_speed_target_rad_s = 0.0f;
    pitch_up_ident_chirp_time_s = 0.0f;
    PitchUpClearRefOverride();

    if (phase == PITCH_UP_IDENT_AUTO_PHASE_DONE) {
        g_pitch_up_ident_done = 1U;
        g_pitch_up_ident_abort = 0U;
        g_pitch_up_ident_abort_reason = PITCH_UP_IDENT_AUTO_ABORT_NONE;
    } else {
        g_pitch_up_ident_done = 0U;
        g_pitch_up_ident_abort = 1U;
        if (g_pitch_up_ident_abort_reason == PITCH_UP_IDENT_AUTO_ABORT_NONE) {
            g_pitch_up_ident_abort_reason =
                PITCH_UP_IDENT_AUTO_ABORT_CANCEL;
        }
    }
}

static void PitchUpIdentAutoAbortIfActive(uint8_t abort_reason)
{
    if (PitchUpIdentAutoActive() != 0U) {
        PitchUpIdentAutoStop(PITCH_UP_IDENT_AUTO_PHASE_ABORT,
                             abort_reason);
    }
}

static void PitchUpIdentAutoStart(void)
{
    const uint8_t stage =
        PitchUpIdentAutoNormalizeStage(g_pitch_up_ident_stage_select);
    const float up_measure_now =
        (gimbal_IMU_data != NULL) ?
        gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] :
        pitch_up_measure_rad;
    float start_ref_rad = PITCH_UP_IDENT_AUTO_CHIRP_CENTER_RAD;

    PitchUpGAutoAbortIfActive();
    PitchUpDynAutoAbortIfActive();
    PitchDownIdentAutoAbortIfActive(PITCH_DOWN_IDENT_AUTO_ABORT_MODE);
    g_pitch_up_model_enable = 1U;

    g_pitch_up_ident_done = 0U;
    g_pitch_up_ident_abort = 0U;
    g_pitch_up_ident_abort_reason = PITCH_UP_IDENT_AUTO_ABORT_NONE;
    g_pitch_up_ident_running = 1U;
    g_pitch_up_ident_enable = 1U;
    g_pitch_up_ident_auto_time_s = 0.0f;
    g_pitch_up_ident_auto_stage = stage;
    g_pitch_up_ident_auto_profile = PitchUpIdentAutoStageProfile(stage);
    g_pitch_up_ident_auto_pass = 0U;
    g_pitch_up_ident_auto_point_index = 0U;
    pitch_up_ident_chirp_time_s = 0.0f;

    if (g_pitch_up_ident_auto_profile ==
        PITCH_UP_IDENT_AUTO_PROFILE_GRAVITY) {
        g_pitch_up_ident_auto_pass =
            (stage == PITCH_UP_IDENT_STAGE_GRAVITY_DOWN) ? 1U : 0U;
        g_pitch_up_ident_auto_target_rad =
            PitchUpIdentAutoGravityTarget(g_pitch_up_ident_auto_pass, 0U);
        PitchUpIdentAutoSetPhase(PITCH_UP_IDENT_AUTO_PHASE_G_SETTLE);
        PitchUpIdentAutoSetRef(g_pitch_up_ident_auto_target_rad,
                               0.0f,
                               0.0f,
                               g_pitch_up_ident_auto_target_rad,
                               0.0f,
                               0.0f,
                               0U);
        return;
    }

    if (g_pitch_up_ident_auto_profile ==
        PITCH_UP_IDENT_AUTO_PROFILE_FRICTION) {
        g_pitch_up_ident_auto_point_index =
            PitchUpIdentAutoStageFrictionIndex(stage);
        g_pitch_up_ident_auto_pass = 0U;
        start_ref_rad = PITCH_UP_IDENT_AUTO_F_LOW_RAD;
        pitch_up_ident_stage_prep_time_s =
            fabsf(start_ref_rad - up_measure_now) /
            PITCH_UP_DYN_RAMP_RATE_RAD_S + PITCH_UP_IDENT_AUTO_F_HOLD_TIME_S;
        pitch_up_ident_stage_prep_time_s =
            PitchDownClampRange(pitch_up_ident_stage_prep_time_s,
                                PITCH_UP_IDENT_AUTO_F_HOLD_TIME_S,
                                PITCH_UP_IDENT_AUTO_F_PREP_MAX_TIME_S);
        PitchUpIdentAutoSetPhase(PITCH_UP_IDENT_AUTO_PHASE_F_HOLD);
        PitchUpIdentAutoSetRef(start_ref_rad,
                               0.0f,
                               0.0f,
                               start_ref_rad,
                               0.0f,
                               PitchUpIdentAutoFrictionSpeed(
                                   g_pitch_up_ident_auto_point_index),
                               0U);
        return;
    }

    g_pitch_up_ident_auto_pass = 0U;
    g_pitch_up_ident_auto_point_index = 0U;
    PitchUpIdentAutoSetPhase(PITCH_UP_IDENT_AUTO_PHASE_C_SETTLE);
    PitchUpIdentAutoSetRef(PITCH_UP_IDENT_AUTO_CHIRP_CENTER_RAD,
                           0.0f,
                           0.0f,
                           PITCH_UP_IDENT_AUTO_CHIRP_CENTER_RAD,
                           0.0f,
                           0.0f,
                           0U);
}

static void PitchUpIdentAutoAdvanceGravityPoint(void)
{
    g_pitch_up_ident_auto_point_index++;
    if (g_pitch_up_ident_auto_point_index >=
        PITCH_UP_IDENT_AUTO_G_POINT_COUNT) {
        PitchUpIdentAutoStop(PITCH_UP_IDENT_AUTO_PHASE_DONE,
                             PITCH_UP_IDENT_AUTO_ABORT_NONE);
        return;
    }

    g_pitch_up_ident_auto_target_rad =
        PitchUpIdentAutoGravityTarget(g_pitch_up_ident_auto_pass,
                                      g_pitch_up_ident_auto_point_index);
    PitchUpIdentAutoSetPhase(PITCH_UP_IDENT_AUTO_PHASE_G_SETTLE);
    PitchUpIdentAutoSetRef(g_pitch_up_ident_auto_target_rad,
                           0.0f,
                           0.0f,
                           g_pitch_up_ident_auto_target_rad,
                           0.0f,
                           0.0f,
                           0U);
}

static void PitchUpIdentAutoUpdate(float dt_s)
{
    float ref_rad;
    float target_rad;
    float speed_rad_s;

    if (dt_s < 0.0001f || dt_s > 0.05f) {
        dt_s = 0.001f;
    }

    if (g_pitch_up_ident_running != 0U &&
        g_pitch_up_ident_enable == 0U) {
        PitchUpIdentAutoStop(PITCH_UP_IDENT_AUTO_PHASE_ABORT,
                             PITCH_UP_IDENT_AUTO_ABORT_CANCEL);
        return;
    }

    if (g_pitch_up_ident_running == 0U) {
        if (g_pitch_up_ident_enable == 0U) {
            return;
        }
        if (PitchUpIdentAutoFeedbackOk() == 0U) {
            PitchUpIdentAutoStop(PITCH_UP_IDENT_AUTO_PHASE_ABORT,
                                 PITCH_UP_IDENT_AUTO_ABORT_FEEDBACK);
            return;
        }
        PitchUpIdentAutoStart();
    }

    if (PitchUpIdentAutoFeedbackOk() == 0U) {
        PitchUpIdentAutoStop(PITCH_UP_IDENT_AUTO_PHASE_ABORT,
                             PITCH_UP_IDENT_AUTO_ABORT_FEEDBACK);
        return;
    }

    g_pitch_up_model_enable = 1U;
    g_pitch_up_ident_auto_time_s += dt_s;
    g_pitch_up_ident_auto_phase_time_s += dt_s;

    switch (g_pitch_up_ident_auto_phase) {
    case PITCH_UP_IDENT_AUTO_PHASE_G_SETTLE:
        target_rad = g_pitch_up_ident_auto_target_rad;
        PitchUpIdentAutoSetRef(target_rad,
                               0.0f,
                               0.0f,
                               target_rad,
                               0.0f,
                               0.0f,
                               0U);
        if (g_pitch_up_ident_auto_phase_time_s >=
            PITCH_UP_IDENT_AUTO_G_SETTLE_TIME_S) {
            PitchUpIdentAutoSetPhase(PITCH_UP_IDENT_AUTO_PHASE_G_SAMPLE);
        }
        break;

    case PITCH_UP_IDENT_AUTO_PHASE_G_SAMPLE:
        target_rad = g_pitch_up_ident_auto_target_rad;
        if (g_pitch_up_ident_auto_phase_time_s >=
            PITCH_UP_IDENT_AUTO_G_SAMPLE_TIME_S) {
            PitchUpIdentAutoAdvanceGravityPoint();
        } else {
            PitchUpIdentAutoSetRef(target_rad,
                                   0.0f,
                                   0.0f,
                                   target_rad,
                                   0.0f,
                                   0.0f,
                                   1U);
        }
        break;

    case PITCH_UP_IDENT_AUTO_PHASE_F_HOLD:
        target_rad = (g_pitch_up_ident_auto_pass == 0U) ?
                     PITCH_UP_IDENT_AUTO_F_LOW_RAD :
                     PITCH_UP_IDENT_AUTO_F_HIGH_RAD;
        PitchUpIdentAutoSetRef(target_rad,
                               0.0f,
                               0.0f,
                               target_rad,
                               0.0f,
                               PitchUpIdentAutoFrictionSpeed(
                                   g_pitch_up_ident_auto_point_index),
                               0U);
        if (g_pitch_up_ident_auto_phase_time_s >=
            pitch_up_ident_stage_prep_time_s) {
            PitchUpIdentAutoSetPhase(PITCH_UP_IDENT_AUTO_PHASE_F_RUN);
        }
        break;

    case PITCH_UP_IDENT_AUTO_PHASE_F_RUN:
        speed_rad_s =
            PitchUpIdentAutoFrictionSpeed(g_pitch_up_ident_auto_point_index);
        if (g_pitch_up_ident_auto_pass == 0U) {
            target_rad = PITCH_UP_IDENT_AUTO_F_HIGH_RAD;
            ref_rad = g_pitch_up_ident_auto_ref_rad + speed_rad_s * dt_s;
            if (ref_rad >= target_rad) {
                PitchUpIdentAutoSetRef(target_rad,
                                       0.0f,
                                       0.0f,
                                       target_rad,
                                       0.0f,
                                       speed_rad_s,
                                       0U);
                g_pitch_up_ident_auto_pass = 1U;
                pitch_up_ident_stage_prep_time_s =
                    PITCH_UP_IDENT_AUTO_F_HOLD_TIME_S;
                PitchUpIdentAutoSetPhase(PITCH_UP_IDENT_AUTO_PHASE_F_HOLD);
            } else {
                PitchUpIdentAutoSetRef(ref_rad,
                                       speed_rad_s,
                                       0.0f,
                                       target_rad,
                                       0.0f,
                                       speed_rad_s,
                                       1U);
            }
        } else {
            target_rad = PITCH_UP_IDENT_AUTO_F_LOW_RAD;
            ref_rad = g_pitch_up_ident_auto_ref_rad - speed_rad_s * dt_s;
            if (ref_rad <= target_rad) {
                PitchUpIdentAutoSetRef(target_rad,
                                       0.0f,
                                       0.0f,
                                       target_rad,
                                       0.0f,
                                       speed_rad_s,
                                       0U);
                PitchUpIdentAutoStop(PITCH_UP_IDENT_AUTO_PHASE_DONE,
                                     PITCH_UP_IDENT_AUTO_ABORT_NONE);
            } else {
                PitchUpIdentAutoSetRef(ref_rad,
                                       -speed_rad_s,
                                       0.0f,
                                       target_rad,
                                       0.0f,
                                       speed_rad_s,
                                       1U);
            }
        }
        break;

    case PITCH_UP_IDENT_AUTO_PHASE_C_SETTLE:
        PitchUpIdentAutoSetRef(PITCH_UP_IDENT_AUTO_CHIRP_CENTER_RAD,
                               0.0f,
                               0.0f,
                               PITCH_UP_IDENT_AUTO_CHIRP_CENTER_RAD,
                               0.0f,
                               0.0f,
                               0U);
        if (g_pitch_up_ident_auto_phase_time_s >=
            PITCH_UP_IDENT_AUTO_CHIRP_SETTLE_S) {
            pitch_up_ident_chirp_time_s = 0.0f;
            PitchUpIdentAutoSetPhase(PITCH_UP_IDENT_AUTO_PHASE_C_RUN);
        }
        break;

    case PITCH_UP_IDENT_AUTO_PHASE_C_RUN:
    {
        const float duration_s = PITCH_UP_IDENT_STAGE_CHIRP_DURATION_S;
        const float f_start_hz = PITCH_UP_IDENT_AUTO_CHIRP_START_HZ;
        const float f_end_hz = PITCH_UP_IDENT_AUTO_CHIRP_END_HZ;
        const float k_hz_s =
            (f_end_hz - f_start_hz) / duration_s;
        const float t_s = pitch_up_ident_chirp_time_s;
        const float freq_hz = f_start_hz + k_hz_s * t_s;
        const float phase =
            PITCH_UP_DYN_TWO_PI *
            (f_start_hz * t_s +
             0.5f * k_hz_s * t_s * t_s);
        const float phase_dot = PITCH_UP_DYN_TWO_PI * freq_hz;
        const float phase_ddot = PITCH_UP_DYN_TWO_PI * k_hz_s;
        const float amp = PITCH_UP_IDENT_AUTO_CHIRP_AMP_RAD;
        const float ref_vel = amp * cosf(phase) * phase_dot;
        const float ref_acc =
            -amp * sinf(phase) * phase_dot * phase_dot +
            amp * cosf(phase) * phase_ddot;

        ref_rad = PITCH_UP_IDENT_AUTO_CHIRP_CENTER_RAD +
                  amp * sinf(phase);
        PitchUpIdentAutoSetRef(ref_rad,
                               ref_vel,
                               ref_acc,
                               PITCH_UP_IDENT_AUTO_CHIRP_CENTER_RAD,
                               freq_hz,
                               0.0f,
                               1U);
        pitch_up_ident_chirp_time_s += dt_s;
        if (pitch_up_ident_chirp_time_s >= duration_s) {
            PitchUpIdentAutoStop(PITCH_UP_IDENT_AUTO_PHASE_DONE,
                                 PITCH_UP_IDENT_AUTO_ABORT_NONE);
        }
        break;
    }

    default:
        PitchUpIdentAutoStop(PITCH_UP_IDENT_AUTO_PHASE_ABORT,
                             PITCH_UP_IDENT_AUTO_ABORT_MODE);
        break;
    }
}

static float PitchUpCalculateTorque(uint8_t bypass_soft_limit)
{
    float torque_lqreso;
    float torque_cmd;
    float torque_motor_nm;
    float feedforward_torque_nm;
    uint8_t model_active;
    uint8_t pitch_up_eso_runtime_enable;
    uint8_t auto_pid_should_active;

    if (gimbal_IMU_data == NULL || pitch_motor_up == NULL ||
        !DMMotorIsOnline(pitch_motor_up)) {
        PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
        pitch_up_ref_rad = 0.0f;
        pitch_up_ref_vel_rad_s = 0.0f;
        pitch_up_ref_acc_rad_s2 = 0.0f;
        pitch_up_torque_cmd_nm = 0.0f;
        pitch_up_lqr_torque_nm = 0.0f;
        pitch_up_gravity_torque_ff_nm = 0.0f;
        PitchUpResetModelFeedforwardDebug();
        pitch_up_soft_limit_active = 0U;
        pitch_up_ref_raw_rad = PITCH_UP_IMU_ZERO_REF_RAD;
        pitch_up_ref_limited_rad = PITCH_UP_IMU_ZERO_REF_RAD;
        pitch_up_ref_source = PITCH_UP_REF_SOURCE_NONE;
        pitch_up_imu_lower_limit_rad = 0.0f;
        pitch_up_imu_upper_limit_rad = 0.0f;
        pitch_up_ref_limit_active = 0U;
        pitch_up_auto_ref_active = 0U;
        pitch_up_auto_ref_last_active = 0U;
        PitchUpResetVisionTrajectory();
        PitchUpAutoPidReset();
        return 0.0f;
    }

    pitch_up_ref_rad = PitchUpSelectRef();
    pitch_up_measure_rad =
        gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
    pitch_up_gyro_rad_s =
        gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
    pitch_up_motor_pos_rad = pitch_motor_up->measure.pos;
    pitch_up_motor_vel_rad_s = pitch_motor_up->measure.vel;
    if (PitchUpNonHoleDownZeroTorqueActive() != 0U) {
        PitchUpApplyZeroTorqueProtection();
        return 0.0f;
    }
    if (PitchUpTorqueOutputAllowed() == 0U &&
        PitchUpIdentAutoActive() == 0U &&
        PitchUpGAutoActive() == 0U) {
        PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
        DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
        pitch_up_lqr_torque_nm = 0.0f;
        pitch_up_gravity_torque_ff_nm = 0.0f;
        pitch_up_torque_cmd_nm = 0.0f;
        PitchUpResetModelFeedforwardDebug();
        pitch_up_soft_limit_active = 0U;
        pitch_up_auto_ref_last_active = 0U;
        PitchUpAutoPidReset();
        return 0.0f;
    }

    if (pitch_up_auto_ref_active != pitch_up_auto_ref_last_active) {
        PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
        DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
        PitchUpAutoPidReset();
    }
    pitch_up_auto_ref_last_active = pitch_up_auto_ref_active;

    auto_pid_should_active =
        (pitch_up_auto_ref_active != 0U &&
         g_pitch_up_auto_pid_enable != 0U) ? 1U : 0U;
    if (auto_pid_should_active != 0U) {
        if (pitch_up_auto_pid_last_active == 0U) {
            PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
            DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
            PitchUpAutoPidReset();
        }
        pitch_up_auto_pid_last_active = 1U;
        PitchUpResetModelFeedforwardDebug();
        torque_cmd = PitchUpCalculateAutoPidTorque();
        if (bypass_soft_limit != 0U) {
            pitch_up_soft_limit_active = 0U;
        } else {
#if PITCH_UP_SOFT_LIMIT_ENABLE
            torque_cmd = PitchUpApplySoftLimit(pitch_up_measure_rad, torque_cmd);
            if (pitch_up_soft_limit_active != 0U) {
                PitchUpAutoPidReset();
            }
#else
            pitch_up_soft_limit_active = 0U;
#endif
        }
        pitch_up_auto_pid_torque_cmd_nm = torque_cmd;
        torque_motor_nm = PITCH_UP_CONTROL_TO_MOTOR_TORQUE_SIGN * torque_cmd;
        pitch_up_torque_cmd_nm = torque_motor_nm;
        return torque_motor_nm;
    }
    if (pitch_up_auto_pid_last_active != 0U) {
        PitchUpAutoPidReset();
        PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
        DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
    } else {
        PitchUpAutoPidClearDebug();
    }

    model_active = PitchUpModelControlActive();
#if PITCH_UP_ESO_TRIAL_ENABLE
    if (model_active != 0U) {
        feedforward_torque_nm = PitchUpCalculateModelFeedforwardMotor();
        pitch_up_gravity_torque_ff_nm = -pitch_up_model_tau_g_q_nm;
    } else {
        PitchUpResetModelFeedforwardDebug();
        pitch_up_gravity_torque_ff_nm = 0.0f;
        feedforward_torque_nm = 0.0f;
    }
#else
    if (model_active != 0U) {
        feedforward_torque_nm = PitchUpCalculateModelFeedforwardMotor();
        pitch_up_gravity_torque_ff_nm = -pitch_up_model_tau_g_q_nm;
    } else {
        PitchUpResetModelFeedforwardDebug();
        pitch_up_gravity_torque_ff_nm =
            PitchUpCalculateGravityTorque(pitch_up_measure_rad);
        feedforward_torque_nm = pitch_up_gravity_torque_ff_nm;
    }
#endif

#if PITCH_UP_ESO_TRIAL_ENABLE
    pitch_up_eso_runtime_enable =
        (pitch_up_auto_ref_active != 0U) ? 1U : 0U;
#else
    pitch_up_eso_runtime_enable = 0U;
#endif
    dzx_pitch_lqr_eso.lqr_config.output_slew_rate =
        (pitch_up_auto_ref_active != 0U) ?
        PITCH_UP_AUTO_TORQUE_SLEW_RATE_NM_S :
        PITCH_UP_TORQUE_SLEW_RATE_NM_S;
    dzx_pitch_lqr_eso.lqr_config.k[0] =
        (pitch_up_auto_ref_active != 0U) ?
        pitch_up_auto_lqr_k_theta :
        PITCH_UP_LQR_K_THETA;
    dzx_pitch_lqr_eso.lqr_config.k[1] =
        (pitch_up_auto_ref_active != 0U) ?
        pitch_up_auto_lqr_k_omega :
        PITCH_UP_LQR_K_OMEGA;
    dzx_pitch_lqr_eso.eso_config.w0_set_enable =
        (g_pitch_up_eso_w0_set_rad_s > 0.0f) ? 1U : 0U;
    dzx_pitch_lqr_eso.eso_config.w0_set =
        (g_pitch_up_eso_w0_set_rad_s > 0.0f) ?
        g_pitch_up_eso_w0_set_rad_s : 0.0f;
#if PITCH_UP_ESO_TRIAL_ENABLE
    dzx_pitch_lqr_eso.lqr_config.plant_j_kg_m2 =
        (g_pitch_up_model_j_kg_m2 > 0.0f) ?
        g_pitch_up_model_j_kg_m2 :
        PITCH_UP_MODEL_J_KG_M2;
    dzx_pitch_lqr_eso.lqr_config.plant_b_nms_rad =
        g_pitch_up_model_b_nms_rad;
    dzx_pitch_lqr_eso.lqr_config.disable_model_feedforward =
        (model_active != 0U) ? 1U : 0U;
#endif
    torque_lqreso = PitchLQRESO_CalcTrajectory(&dzx_pitch_lqr_eso,
                                               pitch_up_ref_rad,
                                               pitch_up_ref_vel_rad_s,
                                               pitch_up_ref_acc_rad_s2,
                                               pitch_up_measure_rad,
                                               pitch_up_gyro_rad_s,
                                               feedforward_torque_nm,
                                               pitch_up_eso_runtime_enable,
                                               DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt));
    g_pitch_up_eso_w0_actual_rad_s = dzx_pitch_lqr_eso.eso.w0;
    pitch_up_lqr_torque_nm = torque_lqreso - feedforward_torque_nm;
    if (bypass_soft_limit != 0U) {
        pitch_up_soft_limit_active = 0U;
        torque_cmd = torque_lqreso;
    } else {
#if PITCH_UP_SOFT_LIMIT_ENABLE
        torque_cmd = PitchUpApplySoftLimit(pitch_up_measure_rad, torque_lqreso);
        if (pitch_up_soft_limit_active != 0U) {
            PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
            DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);
            pitch_up_lqr_torque_nm = 0.0f;
        }
#else
        pitch_up_soft_limit_active = 0U;
        torque_cmd = torque_lqreso;
#endif
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

static uint8_t GimbalHoleDownRestoreReached(float down_pos_rad)
{
    return (down_pos_rad >= HOLE_DOWN_RESTORE_TARGET_RAD) ? 1U : 0U;
}

static uint8_t GimbalHoleDownRestoreLqrTakeoverReady(float down_pos_rad)
{
    return (down_pos_rad >= HOLE_DOWN_RESTORE_LQR_TAKEOVER_RAD) ? 1U : 0U;
}

static float GimbalHoleSanitizeDt(float dt_s)
{
    if (dt_s < PITCH_DOWN_CONTROL_MIN_DT_S || dt_s > PITCH_DOWN_CONTROL_MAX_DT_S) {
        return 0.001f;
    }
    return dt_s;
}

static float GimbalHoleGetDownMeasureRad(void)
{
    return (pitch_motor_down == NULL) ? pitch_down_pos_rad : pitch_motor_down->measure.pos;
}

static void GimbalHolePrimeDownRampTarget(void)
{
    if (hole_down_ramp_target_primed == 0U) {
        hole_down_pre_ramp_target_rad = GimbalHoleGetDownMeasureRad();
        hole_down_ramp_target_primed = 1U;
    }
}

static float GimbalHoleUpdateDownPreRampTarget(float dt_s)
{
    const float target_rad = HOLE_DOWN_MIT_TARGET_RAD;
    const float max_delta = HOLE_DOWN_PRE_RAMP_RATE_RAD_S * GimbalHoleSanitizeDt(dt_s);

    GimbalHolePrimeDownRampTarget();
    if (hole_down_pre_ramp_target_rad > target_rad) {
        hole_down_pre_ramp_target_rad -= max_delta;
        if (hole_down_pre_ramp_target_rad < target_rad) {
            hole_down_pre_ramp_target_rad = target_rad;
        }
    } else if (hole_down_pre_ramp_target_rad < target_rad) {
        hole_down_pre_ramp_target_rad += max_delta;
        if (hole_down_pre_ramp_target_rad > target_rad) {
            hole_down_pre_ramp_target_rad = target_rad;
        }
    } else {
        hole_down_pre_ramp_target_rad = target_rad;
    }

    return hole_down_pre_ramp_target_rad;
}

static void GimbalHoleResetState(void)
{
    hole_down_ready = 0U;
    hole_down_pre_ready = 0U;
    hole_up_ready = 0U;
    hole_up_stable_ready = 0U;
    hole_up_ready_stable_time_s = 0.0f;
    hole_down_pre_ramp_target_rad = HOLE_DOWN_RESTORE_TARGET_RAD;
    hole_up_torque_zero_latched = 0U;
    hole_down_ramp_target_primed = 0U;
    pitch_down_follow_offset_rad = 0.0f;
    PitchDownClearRefOverride();
    PitchUpClearRefOverride();
}

static float GimbalHoleSelectDownTarget(uint8_t *target_id, float dt_s)
{
    if (gimbal_cmd_recv.hole_yaw_ready == 0U) {
        hole_down_pre_ready = 0U;
        hole_up_stable_ready = 0U;
        hole_up_ready_stable_time_s = 0.0f;
        GimbalHolePrimeDownRampTarget();
        if (target_id != NULL) {
            *target_id = 2U;
        }
        return hole_down_pre_ramp_target_rad;
    }

    if (target_id != NULL) {
        *target_id = 4U;
    }
    return GimbalHoleUpdateDownPreRampTarget(dt_s);
}

static void GimbalHoleApplyZeroTorqueLatch(void)
{
    hole_up_torque_zero_latched = 1U;
    PitchLQRESO_Reset(&dzx_pitch_down_lqr_eso);
    PitchLQRESO_Reset(&dzx_pitch_lqr_eso);
    DWT_GetDeltaT(&dzx_pitch_down_lqr_dwt_cnt);
    DWT_GetDeltaT(&dzx_pitch_lqr_dwt_cnt);

    pitch_down_lqr_e_theta_rad = 0.0f;
    pitch_down_lqr_e_omega_rad_s = 0.0f;
    pitch_down_lqr_tau_raw_nm = 0.0f;
    pitch_down_lqr_pre_limit_torque_nm = 0.0f;
    pitch_down_lqr_output_nm = 0.0f;
    pitch_down_lqr_limit_active = 0U;
    pitch_down_torque_cmd_nm = 0.0f;
    pitch_down_gravity_torque_ff_nm = 0.0f;
    pitch_down_friction_torque_ff_nm = 0.0f;
    pitch_down_soft_limit_active = 0U;
    pitch_up_lqr_torque_nm = 0.0f;
    pitch_up_gravity_torque_ff_nm = 0.0f;
    pitch_up_torque_cmd_nm = 0.0f;
    pitch_up_soft_limit_active = 0U;

    PitchDownSetTorqueImmediate(0.0f);
    DMMotorSetTorque(pitch_motor_up, 0.0f);
}

static uint8_t GimbalHoleDownZeroTorqueReady(float down_pos_rad)
{
    return (down_pos_rad <= HOLE_DOWN_TAKEOVER_READY_RAD) ? 1U : 0U;
}

static uint8_t GimbalHoleRestoreUpReady(void)
{
    return (fabsf(pitch_up_measure_rad - HOLE_UP_IMU_TARGET_RAD) <=
            HOLE_RESTORE_READY_THRESHOLD_RAD) ? 1U : 0U;
}

static void GimbalHoleApplyPitchControl(void)
{
    float up_torque;
    float down_target;
    float down_mit_kp;
    float down_mit_kd;

    DMMotorEnable1(pitch_motor_down);
    DMMotorEnable1(pitch_motor_up);
    static float v_des=0.0f;
    
    if (hole_up_torque_zero_latched != 0U) {
        GimbalHoleApplyZeroTorqueLatch();
        return;
    }

    /*
     * yaw 未到 -64deg 前 down 用 MIT 保持当前锁存位置；
     * yaw ready 后 down 直接使用 MIT 位置控制到狗洞目标角。
     */
    if (gimbal_cmd_recv.hole_yaw_ready == 0U) {
        GimbalHolePrimeDownRampTarget();
        down_target = hole_down_pre_ramp_target_rad;
    } else {
        down_target = HOLE_DOWN_MIT_TARGET_RAD;
    }
    down_mit_kp = hole_down_enter_pre_mit_kp;
    down_mit_kd = hole_down_enter_pre_mit_kd;
    v_des = 0.0f;
    PitchDownSetRefOverride(down_target, 1U);
    PitchDownApplyMITPositionControlWithGain(down_mit_kp, down_mit_kd, v_des);
    hole_down_pre_ready = (gimbal_cmd_recv.hole_yaw_ready != 0U) ? 1U : 0U;

    hole_down_ready = GimbalHoleDownZeroTorqueReady(pitch_down_pos_rad);
    if (hole_down_ready != 0U) {
        GimbalHoleApplyZeroTorqueLatch();
        return;
    }

    /*
     * up 轴在狗洞模式内持续闭环到狗洞 IMU 目标角。
     * down 到 0.1rad 以下后由上面的锁存分支让 up/down 同时 0Nm。
     */
    PitchUpSetRefOverride(HOLE_UP_IMU_TARGET_RAD);
    up_torque = PitchUpCalculateTorque(1U);

    hole_up_ready =
        (fabsf(pitch_up_measure_rad - HOLE_UP_IMU_TARGET_RAD) <=
         HOLE_UP_READY_THRESHOLD_RAD) ? 1U : 0U;
    hole_up_torque_zero_latched = 0U;
    DMMotorSetTorque(pitch_motor_up, up_torque);
}

static void GimbalHoleApplyRestoreControl(void)
{
    float down_restore_pos_rad;
    uint8_t down_lqr_takeover_ready;
    float up_torque = 0.0f;

    DMMotorEnable1(pitch_motor_down);
    DMMotorEnable1(pitch_motor_up);

    down_restore_pos_rad = (pitch_motor_down == NULL) ?
                           pitch_down_pos_rad :
                           pitch_motor_down->measure.pos;
    hole_down_ready = (pitch_motor_down == NULL) ?
                      0U :
                      GimbalHoleDownRestoreReached(down_restore_pos_rad);
    down_lqr_takeover_ready = (pitch_motor_down == NULL) ?
                              0U :
                              GimbalHoleDownRestoreLqrTakeoverReady(down_restore_pos_rad);

    /*
     * 退出狗洞后 down 用 7Nm 直推到 1.95rad - 20deg，后段由 LQR 接管到 1.95rad；
     * up 立即闭环到狗洞 IMU 目标角。
     * 底盘侧仍等 down/up 都 ready 后才释放跟随。
     */
    hole_restore_up_enable_latched = 1U;
    hole_up_torque_zero_latched = 0U;
    hole_down_pre_ready = 0U;
    hole_up_stable_ready = 0U;
    hole_up_ready_stable_time_s = 0.0f;
    hole_down_pre_ramp_target_rad = HOLE_DOWN_RESTORE_TARGET_RAD;
    hole_down_ramp_target_primed = 0U;
    pitch_down_follow_offset_rad = 0.0f;
    if (down_lqr_takeover_ready == 0U) {
        PitchDownApplyHoleRestoreTorque();
    } else {
        PitchDownSetRefOverride(HOLE_DOWN_RESTORE_TARGET_RAD, 0U);
        PitchDownApplyHybridControlWithGain(hole_down_enter_pre_mit_kp,
                                            hole_down_enter_pre_mit_kd,
                                            0.0f);
    }
    PitchUpSetRefOverride(HOLE_UP_IMU_TARGET_RAD);
    up_torque = PitchUpCalculateTorque(1U);
    hole_up_ready = GimbalHoleRestoreUpReady();
    DMMotorSetTorque(pitch_motor_up, up_torque);
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
static volatile gimbal_mode_e test_hole;
#ifdef GIMBAL_BOARD
void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* ---- Pitch-up chirp injection (system identification) -------- */
    {
        static uint8_t  s_chirp_armed_prev = 0U;
        static uint8_t  s_chirp_running    = 0U;
        static float    s_chirp_t_s        = 0.0f;
        static uint32_t s_chirp_dwt        = 0U;

        /* Rising edge of g_pitch_up_chirp_arm: start a new sweep */
        if (g_pitch_up_chirp_arm != 0U && s_chirp_armed_prev == 0U) {
            s_chirp_running = 1U;
            s_chirp_t_s     = 0.0f;
            DWT_GetDeltaT(&s_chirp_dwt);
            pitch_up_ref_override_active = 1U;
            pitch_up_ref_override_rad    = g_pitch_up_chirp_center_rad;
        }
        s_chirp_armed_prev = g_pitch_up_chirp_arm;

        if (s_chirp_running != 0U) {
            const float dt     = DWT_GetDeltaT(&s_chirp_dwt);
            const float t      = s_chirp_t_s;
            const float T      = g_pitch_up_chirp_duration_s;
            const float f0     = g_pitch_up_chirp_f_start_hz;
            const float f1     = g_pitch_up_chirp_f_end_hz;
            const float amp    = g_pitch_up_chirp_amp_rad;
            const float center = g_pitch_up_chirp_center_rad;

            /* Disarm flag set externally OR duration elapsed: stop and release */
            if (g_pitch_up_chirp_arm == 0U || T <= 0.0f || t >= T) {
                s_chirp_running                 = 0U;
                pitch_up_ref_override_active    = 0U;
                pitch_up_ref_override_rad       = PITCH_UP_IMU_ZERO_REF_RAD;
                g_pitch_up_chirp_running        = 0U;
                g_pitch_up_chirp_arm            = 0U; /* auto-disarm */
                g_pitch_up_chirp_inst_freq_hz   = 0.0f;
                g_pitch_up_chirp_ref_rad        = center;
            } else {
                /* Linear chirp: f(t)=f0+k*t,  phase=2π·(f0·t+0.5·k·t²) */
                const float k     = (f1 - f0) / T;
                const float phase = 6.28318530717959f *
                                    (f0 * t + 0.5f * k * t * t);
                const float ref   = center + amp * sinf(phase);

                pitch_up_ref_override_active    = 1U;
                pitch_up_ref_override_rad       = ref;
                g_pitch_up_chirp_running        = 1U;
                g_pitch_up_chirp_time_s         = t;
                g_pitch_up_chirp_inst_freq_hz   = f0 + k * t;
                g_pitch_up_chirp_ref_rad        = ref;
                s_chirp_t_s                    += dt;
            }
        }
    }
    /* -------------------------------------------------------------- */

#if defined(CHASSIS_BOARD)
    GimbalYawControlStep();
#elif defined(GIMBAL_BOARD)
    if(test_hole==GIMBAL_HOLE_MODE&&gimbal_cmd_recv.gimbal_mode!=GIMBAL_ZERO_FORCE){
        gimbal_cmd_recv.gimbal_mode=test_hole;
        gimbal_cmd_recv.hole_yaw_ready=1;
    }
    static DoublePitchLqr_s double_pitch_lqr;
    static uint8_t double_pitch_lqr_inited = 0U;
    static uint32_t double_pitch_lqr_dwt_cnt;
    static gimbal_mode_e double_pitch_last_mode = GIMBAL_ZERO_FORCE;
    DoublePitchLqrInput_s double_pitch_input = {0};
    DoublePitchLqrOutput_s double_pitch_output = {0};
    uint8_t run_double_pitch = 0U;
    uint8_t hold_up_current_ref = 0U;
    uint8_t force_down_zero_torque = 0U;
    uint8_t force_up_zero_torque = 0U;
    uint8_t bypass_up_soft_limit = 0U;
    uint8_t feedback_ok = 0U;
    uint8_t double_pitch_target_id = 0U;
    uint8_t gravity_ramp_valid = 0U;
    uint8_t gravity_step_requested = 0U;
    uint8_t direct_down_torque_requested = 0U;
    uint8_t pitch_down_state_ok = 0U;
    float dt_s;
    float down_target = PITCH_DOWN_DEFAULT_REF_POS_RAD;
    float down_restore_pos_rad = pitch_down_pos_rad;
    float up_torque_cmd_nm = 0.0f;
    float down_torque_cmd_nm = 0.0f;

    if (double_pitch_lqr_inited == 0U) {
        DoublePitchLqr_Init(&double_pitch_lqr);
        DWT_GetDeltaT(&double_pitch_lqr_dwt_cnt);
        double_pitch_last_mode = gimbal_cmd_recv.gimbal_mode;
        double_pitch_lqr_inited = 1U;
    }

    dt_s = DWT_GetDeltaT(&double_pitch_lqr_dwt_cnt);
    direct_down_torque_requested =
        (gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE &&
         DoublePitchDirectDownTorqueActive() != 0U) ? 1U : 0U;
    gravity_step_requested =
        (direct_down_torque_requested == 0U &&
         gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE &&
         DoublePitchGravityRampRequestActive() != 0U) ? 1U : 0U;
    if (double_pitch_last_mode != gimbal_cmd_recv.gimbal_mode) {
        DoublePitchLqr_Reset(&double_pitch_lqr);
        double_pitch_last_mode = gimbal_cmd_recv.gimbal_mode;
    }

    if (direct_down_torque_requested != 0U) {
        DoublePitchIdentStop();
        DoublePitchLqr_Reset(&double_pitch_lqr);
        GimbalHoleResetState();
        PitchUpClearRefOverride();
        DMMotorEnable1(pitch_motor_down);
        DMMotorEnable1(pitch_motor_up);
        run_double_pitch = 1U;
    } else if (gravity_step_requested != 0U) {
        DoublePitchIdentStop();
        GimbalHoleResetState();
        PitchUpClearRefOverride();
        DMMotorEnable1(pitch_motor_down);
        DMMotorEnable1(pitch_motor_up);
        run_double_pitch = 1U;
    } else if (gimbal_cmd_recv.hole_restore_active != 0U &&
        gimbal_cmd_recv.gimbal_mode != GIMBAL_HOLE_MODE) {
        DMMotorEnable1(pitch_motor_down);
        DMMotorEnable1(pitch_motor_up);
        double_pitch_target_id = 1U;
        DoublePitchIdentStop();

        down_restore_pos_rad = (pitch_motor_down == NULL) ?
                               pitch_down_pos_rad :
                               pitch_motor_down->measure.pos;
        hole_down_ready = (pitch_motor_down == NULL) ?
                          0U :
                          GimbalHoleAngleReady(down_restore_pos_rad,
                                               HOLE_DOWN_RESTORE_TARGET_RAD);
        hole_restore_up_enable_latched = 1U;
        hole_up_torque_zero_latched = 0U;
        hole_down_pre_ready = 0U;
        hole_up_ready = 0U;
        hole_up_stable_ready = 0U;
        hole_up_ready_stable_time_s = 0.0f;
        hole_down_pre_ramp_target_rad = HOLE_DOWN_RESTORE_TARGET_RAD;
        hole_down_ramp_target_primed = 0U;
        pitch_down_follow_offset_rad = 0.0f;

        if (hole_down_ready != 0U) {
            PitchDownClearRefOverride();
        } else {
            PitchDownSetRefOverride(HOLE_DOWN_RESTORE_TARGET_RAD, 0U);
        }

        PitchUpSetRefOverride(HOLE_UP_IMU_TARGET_RAD);
        bypass_up_soft_limit = 1U;
        run_double_pitch = 1U;
    } else {
        switch (gimbal_cmd_recv.gimbal_mode) {
        case GIMBAL_ZERO_FORCE:
            DMMotorSetTorque(pitch_motor_up, 0.0f);
            DMMotorStop(pitch_motor_up);
            PitchDownSetTorqueImmediate(0.0f);
            DMMotorStop(pitch_motor_down);
            pitch_up_torque_cmd_nm = 0.0f;
            pitch_down_torque_cmd_nm = 0.0f;
            dzx_pitch_torque = 0.0f;
            GimbalHoleResetState();
            PitchUpStopControl();
            PitchDownStopControl();
            DoublePitchLqr_Reset(&double_pitch_lqr);
            PitchUpUpdateIdentDebug(dt_s, 0U, 0U, 0U, NULL, NULL, 0.0f);
            DoublePitchIdentStop();
            DoublePitchIdentUpdateDebug(dt_s,
                                        0U,
                                        0U,
                                        double_pitch_target_id,
                                        &double_pitch_lqr,
                                        NULL,
                                        NULL,
                                        0.0f,
                                        0.0f);
            break;
        case GIMBAL_GYRO_MODE:
        case GIMBAL_MOTOR_MODE:
            GimbalHoleResetState();
            PitchDownApplyNormalFollowRef();
            PitchUpClearRefOverride();
            if (DoublePitchGravityRampRequestActive() != 0U) {
                DoublePitchIdentStop();
            } else {
                DoublePitchIdentApplyStaticPointSelection(dt_s);
                if (DoublePitchIdentStaticRequestActive() == 0U) {
                    (void)DoublePitchIdentApplyNormalRef(dt_s);
                }
            }
            DMMotorEnable1(pitch_motor_down);
            DMMotorEnable1(pitch_motor_up);
            run_double_pitch = 1U;
            break;
        case GIMBAL_HOLE_MODE:
            DoublePitchIdentStop();
            DMMotorEnable1(pitch_motor_down);
            DMMotorEnable1(pitch_motor_up);
            down_target = GimbalHoleSelectDownTarget(&double_pitch_target_id, dt_s);
            PitchDownSetRefOverride(down_target,
                                    (gimbal_cmd_recv.hole_yaw_ready != 0U) ? 1U : 0U);
            PitchUpSetRefOverride(HOLE_UP_IMU_TARGET_RAD);
            bypass_up_soft_limit = 1U;
            run_double_pitch = 1U;
            break;
        default:
            DMMotorSetTorque(pitch_motor_up, 0.0f);
            DMMotorStop(pitch_motor_up);
            PitchDownSetTorqueImmediate(0.0f);
            DMMotorStop(pitch_motor_down);
            pitch_up_torque_cmd_nm = 0.0f;
            pitch_down_torque_cmd_nm = 0.0f;
            dzx_pitch_torque = 0.0f;
            GimbalHoleResetState();
            PitchUpStopControl();
            PitchDownStopControl();
            DoublePitchLqr_Reset(&double_pitch_lqr);
            PitchUpUpdateIdentDebug(dt_s, 0U, 0U, 0U, NULL, NULL, 0.0f);
            DoublePitchIdentStop();
            DoublePitchIdentUpdateDebug(dt_s,
                                        0U,
                                        0U,
                                        double_pitch_target_id,
                                        &double_pitch_lqr,
                                        NULL,
                                        NULL,
                                        0.0f,
                                        0.0f);
            break;
        }
    }

    if (run_double_pitch != 0U) {
        feedback_ok = (pitch_motor_up != NULL &&
                       pitch_motor_down != NULL &&
                       DMMotorIsOnline(pitch_motor_up) != 0U &&
                       DMMotorIsOnline(pitch_motor_down) != 0U) ? 1U : 0U;
        pitch_down_state_ok = PitchDownRefreshControlState();

        if (direct_down_torque_requested == 0U &&
            gravity_step_requested == 0U &&
            (feedback_ok == 0U || pitch_down_state_ok == 0U)) {
            DMMotorSetTorque(pitch_motor_up, 0.0f);
            PitchDownSetTorqueImmediate(0.0f);
            pitch_up_torque_cmd_nm = 0.0f;
            pitch_down_torque_cmd_nm = 0.0f;
            dzx_pitch_torque = 0.0f;
            PitchUpStopControl();
            PitchDownResetUnavailableControl();
            DoublePitchLqr_Reset(&double_pitch_lqr);
            PitchUpUpdateIdentDebug(dt_s, 0U, feedback_ok, 0U, NULL, NULL, 0.0f);
            DoublePitchIdentUpdateDebug(dt_s,
                                        0U,
                                        feedback_ok,
                                        double_pitch_target_id,
                                        &double_pitch_lqr,
                                        NULL,
                                        NULL,
                                        0.0f,
                                        0.0f);
        } else {
            pitch_up_motor_pos_rad =
                (pitch_motor_up != NULL) ? pitch_motor_up->measure.pos :
                                           pitch_up_motor_pos_rad;
            pitch_up_motor_vel_rad_s =
                (pitch_motor_up != NULL) ? pitch_motor_up->measure.vel :
                                           pitch_up_motor_vel_rad_s;
            if (gimbal_IMU_data != NULL) {
                pitch_up_measure_rad =
                    gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
                pitch_up_gyro_rad_s =
                    gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
            } else {
                pitch_up_measure_rad =
                    PitchUpEstimateImuRad(pitch_up_motor_pos_rad,
                                          pitch_down_pos_rad);
                pitch_up_gyro_rad_s =
                    PITCH_UP_IMU_COUPLE_UP_COEF * pitch_up_motor_vel_rad_s -
                    PITCH_UP_IMU_COUPLE_DOWN_COEF * pitch_down_vel_rad_s;
            }

            if (hold_up_current_ref != 0U) {
                pitch_up_ref_raw_rad = pitch_up_measure_rad;
                PitchUpGetImuLimits(&pitch_up_imu_lower_limit_rad,
                                    &pitch_up_imu_upper_limit_rad);
                pitch_up_ref_rad =
                    PitchDownClampRange(pitch_up_measure_rad,
                                        pitch_up_imu_lower_limit_rad,
                                        pitch_up_imu_upper_limit_rad);
                pitch_up_ref_vel_rad_s = 0.0f;
                pitch_up_ref_acc_rad_s2 = 0.0f;
                pitch_up_ref_limited_rad = pitch_up_ref_rad;
                pitch_ref_limited = pitch_up_ref_limited_rad;
                pitch_up_ref_limit_active = 0U;
                pitch_up_auto_ref_active = 0U;
                PitchUpResetVisionTrajectory();
            } else {
                pitch_up_ref_rad = PitchUpSelectRef();
            }
            if (g_double_pitch_ident_running != 0U &&
                DoublePitchIdentNormalModeActive() != 0U) {
                pitch_up_ref_vel_rad_s = g_double_pitch_ident_psi_ref_dot_rad_s;
                pitch_up_ref_acc_rad_s2 = g_double_pitch_ident_psi_ref_ddot_rad_s2;
            }

            double_pitch_input.theta_rad = pitch_down_pos_rad;
            double_pitch_input.psi_rad = pitch_up_measure_rad;
            double_pitch_input.phi_rad =
                pitch_up_measure_rad - pitch_down_pos_rad;
            while (double_pitch_input.phi_rad > 3.141592653589793f) {
                double_pitch_input.phi_rad -= 6.283185307179586f;
            }
            while (double_pitch_input.phi_rad < -3.141592653589793f) {
                double_pitch_input.phi_rad += 6.283185307179586f;
            }
            double_pitch_input.omega_big_rad_s = pitch_down_vel_rad_s;
            double_pitch_input.omega_small_rad_s =
                pitch_up_gyro_rad_s - pitch_down_vel_rad_s;
            double_pitch_input.theta_ref_rad = pitch_down_ref_limited_pos_rad;
            double_pitch_input.phi_ref_rad =
                pitch_up_ref_rad - pitch_down_ref_limited_pos_rad;
            while (double_pitch_input.phi_ref_rad > 3.141592653589793f) {
                double_pitch_input.phi_ref_rad -= 6.283185307179586f;
            }
            while (double_pitch_input.phi_ref_rad < -3.141592653589793f) {
                double_pitch_input.phi_ref_rad += 6.283185307179586f;
            }
            if (g_double_pitch_ident_running != 0U &&
                DoublePitchIdentNormalModeActive() != 0U) {
                double_pitch_input.theta_ref_dot_rad_s =
                    g_double_pitch_ident_theta_ref_dot_rad_s;
                double_pitch_input.phi_ref_dot_rad_s =
                    g_double_pitch_ident_phi_ref_dot_rad_s;
                double_pitch_input.theta_ref_ddot_rad_s2 =
                    g_double_pitch_ident_theta_ref_ddot_rad_s2;
                double_pitch_input.phi_ref_ddot_rad_s2 =
                    g_double_pitch_ident_phi_ref_ddot_rad_s2;
            } else {
                double_pitch_input.theta_ref_dot_rad_s = 0.0f;
                double_pitch_input.phi_ref_dot_rad_s = pitch_up_ref_vel_rad_s;
                double_pitch_input.theta_ref_ddot_rad_s2 = 0.0f;
                double_pitch_input.phi_ref_ddot_rad_s2 = pitch_up_ref_acc_rad_s2;
            }
            double_pitch_input.torque_big_limit_nm =
                (gimbal_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) ?
                PITCH_DOWN_HOLE_TORQUE_LIMIT_NM :
                PITCH_DOWN_LQR_TORQUE_LIMIT_NM;
            double_pitch_input.torque_small_limit_nm =
                PITCH_UP_TORQUE_LIMIT_NM;
            double_pitch_input.torque_big_slew_rate_nm_s =
                (gimbal_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) ?
                PITCH_DOWN_HOLE_TORQUE_SLEW_RATE_NM_S :
                PITCH_DOWN_TORQUE_SLEW_RATE_NM_S;
            double_pitch_input.torque_small_slew_rate_nm_s =
                PITCH_UP_TORQUE_SLEW_RATE_NM_S;
            double_pitch_input.tau_big_meas_nm =
                (pitch_motor_down != NULL) ? pitch_motor_down->measure.tor : 0.0f;
            double_pitch_input.tau_small_meas_nm =
                (pitch_motor_up != NULL) ? pitch_motor_up->measure.tor : 0.0f;
            double_pitch_input.motor_big_temp_c =
                (pitch_motor_down != NULL) ? pitch_motor_down->measure.Tcoil : 0.0f;
            double_pitch_input.motor_small_temp_c =
                (pitch_motor_up != NULL) ? pitch_motor_up->measure.Tcoil : 0.0f;
            double_pitch_input.motor_big_tmos_c =
                (pitch_motor_down != NULL) ? pitch_motor_down->measure.Tmos : 0.0f;
            double_pitch_input.motor_small_tmos_c =
                (pitch_motor_up != NULL) ? pitch_motor_up->measure.Tmos : 0.0f;
            double_pitch_input.bus_voltage_v = 0.0f;
            double_pitch_input.target_id = double_pitch_target_id;
            double_pitch_input.control_mode = (uint8_t)gimbal_cmd_recv.gimbal_mode;
            double_pitch_input.bus_voltage_valid = 0U;
            double_pitch_input.torque_slew_enable = 1U;
            double_pitch_input.feedback_ok = 1U;

            if (direct_down_torque_requested != 0U) {
                DoublePitchLqr_Reset(&double_pitch_lqr);
                down_torque_cmd_nm = g_double_pitch_direct_down_torque_nm;
                up_torque_cmd_nm = 0.0f;
                pitch_down_lqr_e_theta_rad = 0.0f;
                pitch_down_lqr_e_omega_rad_s = 0.0f;
                pitch_down_lqr_tau_raw_nm = 0.0f;
                pitch_down_lqr_pre_limit_torque_nm = down_torque_cmd_nm;
                pitch_down_lqr_output_nm = down_torque_cmd_nm;
                pitch_down_lqr_limit_active = 0U;
                pitch_down_gravity_torque_ff_nm = 0.0f;
                pitch_down_friction_torque_ff_nm = 0.0f;
                pitch_down_gravity_torque_raw_nm = 0.0f;
                pitch_down_gravity_torque_pre_limit_nm = 0.0f;
                pitch_down_torque_cmd_nm = down_torque_cmd_nm;
                pitch_down_soft_limit_active = 0U;
                pitch_up_lqr_torque_nm = 0.0f;
                pitch_up_gravity_torque_ff_nm = 0.0f;
                dzx_pitch_torque = 0.0f;
                pitch_up_torque_cmd_nm = 0.0f;
                pitch_up_soft_limit_active = 0U;
                double_pitch_output.tau_big_cmd_nm = down_torque_cmd_nm;
                double_pitch_output.tau_small_cmd_nm = 0.0f;
                double_pitch_output.tau_big_pre_limit_nm = down_torque_cmd_nm;
                double_pitch_output.tau_small_pre_limit_nm = 0.0f;
                down_torque_cmd_nm = PitchDownSetTorqueWithSlew(down_torque_cmd_nm);
                if (pitch_down_output_torque_slew_active != 0U) {
                    double_pitch_output.tau_big_limit_flag = 1U;
                }
                double_pitch_output.tau_big_cmd_nm = down_torque_cmd_nm;
                DMMotorSetTorque(pitch_motor_up, 0.0f);
                PitchUpUpdateIdentDebug(dt_s,
                                        1U,
                                        feedback_ok,
                                        0U,
                                        &double_pitch_input,
                                        &double_pitch_output,
                                        0.0f);
                DoublePitchIdentUpdateDebug(dt_s,
                                            1U,
                                            feedback_ok,
                                            double_pitch_target_id,
                                            NULL,
                                            &double_pitch_input,
                                            &double_pitch_output,
                                            down_torque_cmd_nm,
                                            0.0f);
            } else if (DoublePitchGravityRampApply(dt_s,
                                            &double_pitch_input,
                                            &double_pitch_output,
                                            &down_torque_cmd_nm,
                                            &up_torque_cmd_nm,
                                            &gravity_ramp_valid) != 0U) {
                DoublePitchLqr_Reset(&double_pitch_lqr);
                pitch_down_lqr_e_theta_rad = 0.0f;
                pitch_down_lqr_e_omega_rad_s = 0.0f;
                pitch_down_lqr_tau_raw_nm = 0.0f;
                pitch_down_lqr_pre_limit_torque_nm =
                    double_pitch_output.tau_big_pre_limit_nm;
                pitch_down_lqr_output_nm = down_torque_cmd_nm;
                pitch_down_lqr_limit_active =
                    double_pitch_output.tau_big_limit_flag;
                pitch_down_gravity_torque_ff_nm = 0.0f;
                pitch_down_friction_torque_ff_nm = 0.0f;
                pitch_down_gravity_torque_raw_nm = 0.0f;
                pitch_down_gravity_torque_pre_limit_nm = 0.0f;
                pitch_down_torque_cmd_nm = down_torque_cmd_nm;
                pitch_down_soft_limit_active = 0U;
                pitch_up_lqr_torque_nm = 0.0f;
                pitch_up_gravity_torque_ff_nm = 0.0f;
                dzx_pitch_torque = up_torque_cmd_nm;
                pitch_up_torque_cmd_nm = dzx_pitch_torque;
                pitch_up_soft_limit_active = 0U;
                if (PitchUpNonHoleDownZeroTorqueActive() != 0U) {
                    up_torque_cmd_nm = 0.0f;
                    PitchUpApplyZeroTorqueProtection();
                    PitchUpClearDoublePitchSmallOutput(&double_pitch_output);
                }
                down_torque_cmd_nm = PitchDownSetTorqueWithSlew(down_torque_cmd_nm);
                if (pitch_down_output_torque_slew_active != 0U) {
                    double_pitch_output.tau_big_limit_flag = 1U;
                }
                double_pitch_output.tau_big_cmd_nm = down_torque_cmd_nm;
                DMMotorSetTorque(pitch_motor_up, up_torque_cmd_nm);
                PitchUpUpdateIdentDebug(dt_s,
                                        gravity_ramp_valid,
                                        feedback_ok,
                                        gravity_ramp_valid,
                                        &double_pitch_input,
                                        &double_pitch_output,
                                        up_torque_cmd_nm);
                DoublePitchIdentUpdateDebug(dt_s,
                                            gravity_ramp_valid,
                                            feedback_ok,
                                            double_pitch_target_id,
                                            NULL,
                                            &double_pitch_input,
                                            &double_pitch_output,
                                            down_torque_cmd_nm,
                                            up_torque_cmd_nm);
            } else if (DoublePitchIdentNormalModeActive() != 0U &&
                       g_double_pitch_normal_controller_enable == 0U) {
                DoublePitchLqr_Reset(&double_pitch_lqr);
                down_torque_cmd_nm = 0.0f;
                up_torque_cmd_nm = 0.0f;
                pitch_down_lqr_e_theta_rad = 0.0f;
                pitch_down_lqr_e_omega_rad_s = 0.0f;
                pitch_down_lqr_tau_raw_nm = 0.0f;
                pitch_down_lqr_pre_limit_torque_nm = 0.0f;
                pitch_down_lqr_output_nm = 0.0f;
                pitch_down_lqr_limit_active = 0U;
                pitch_down_gravity_torque_ff_nm = 0.0f;
                pitch_down_friction_torque_ff_nm = 0.0f;
                pitch_down_gravity_torque_raw_nm = 0.0f;
                pitch_down_gravity_torque_pre_limit_nm = 0.0f;
                pitch_down_torque_cmd_nm = 0.0f;
                pitch_down_soft_limit_active = 0U;
                pitch_up_lqr_torque_nm = 0.0f;
                pitch_up_gravity_torque_ff_nm = 0.0f;
                dzx_pitch_torque = 0.0f;
                pitch_up_torque_cmd_nm = 0.0f;
                pitch_up_soft_limit_active = 0U;
                PitchDownSetTorqueImmediate(0.0f);
                DMMotorSetTorque(pitch_motor_up, 0.0f);
                PitchUpUpdateIdentDebug(dt_s,
                                        0U,
                                        feedback_ok,
                                        0U,
                                        &double_pitch_input,
                                        &double_pitch_output,
                                        0.0f);
                DoublePitchIdentUpdateDebug(dt_s,
                                            0U,
                                            feedback_ok,
                                            double_pitch_target_id,
                                            &double_pitch_lqr,
                                            &double_pitch_input,
                                            &double_pitch_output,
                                            0.0f,
                                            0.0f);
            } else {
                DoublePitchLqr_Calc(&double_pitch_lqr,
                                    &double_pitch_input,
                                    dt_s,
                                    &double_pitch_output);

                down_torque_cmd_nm = double_pitch_output.tau_big_cmd_nm;
                up_torque_cmd_nm =
                    PITCH_UP_CONTROL_TO_MOTOR_TORQUE_SIGN *
                    double_pitch_output.tau_small_cmd_nm;

                pitch_down_lqr_e_theta_rad = double_pitch_output.e_theta_rad;
                pitch_down_lqr_e_omega_rad_s =
                    double_pitch_output.e_omega_big_rad_s;
                pitch_down_lqr_tau_raw_nm = double_pitch_output.tau_big_fb_nm;
                pitch_down_lqr_pre_limit_torque_nm =
                    double_pitch_output.tau_big_pre_limit_nm;
                pitch_down_lqr_output_nm = down_torque_cmd_nm;
                pitch_down_lqr_limit_active =
                    double_pitch_output.tau_big_limit_flag;
                pitch_down_gravity_torque_ff_nm = double_pitch_output.tau_big_g_nm;
                pitch_down_friction_torque_ff_nm = 0.0f;
                pitch_down_gravity_torque_raw_nm = double_pitch_output.tau_big_g_nm;
                pitch_down_gravity_torque_pre_limit_nm =
                    double_pitch_output.tau_big_g_nm;
                pitch_down_torque_cmd_nm = down_torque_cmd_nm;
                pitch_down_soft_limit_active = 0U;
                pitch_up_lqr_torque_nm = double_pitch_output.tau_small_fb_nm;
                pitch_up_gravity_torque_ff_nm = double_pitch_output.tau_small_g_nm;

                if (gimbal_cmd_recv.gimbal_mode == GIMBAL_HOLE_MODE) {
                    hole_down_pre_ready =
                        (gimbal_cmd_recv.hole_yaw_ready != 0U) ? 1U : 0U;
                    hole_down_ready =
                        GimbalHoleDownZeroTorqueReady(pitch_down_pos_rad);
                    hole_up_ready =
                        (fabsf(pitch_up_measure_rad - HOLE_UP_IMU_TARGET_RAD) <=
                         HOLE_UP_READY_THRESHOLD_RAD) ? 1U : 0U;
                    if (hole_down_ready != 0U) {
                        hole_up_torque_zero_latched = 1U;
                        up_torque_cmd_nm = 0.0f;
                        down_torque_cmd_nm = 0.0f;
                        pitch_up_torque_cmd_nm = 0.0f;
                        pitch_down_torque_cmd_nm = 0.0f;
                        force_down_zero_torque = 1U;
                    } else {
                        hole_up_torque_zero_latched = 0U;
                        force_down_zero_torque = 0U;
                    }
                }
                if (PitchUpNonHoleDownZeroTorqueActive() != 0U) {
                    force_up_zero_torque = 1U;
                    PitchUpApplyZeroTorqueProtection();
                    PitchUpClearDoublePitchSmallOutput(&double_pitch_output);
                }

                if (force_up_zero_torque != 0U) {
                    up_torque_cmd_nm = 0.0f;
                    pitch_up_soft_limit_active = 0U;
                } else if (bypass_up_soft_limit == 0U) {
#if PITCH_UP_SOFT_LIMIT_ENABLE
                    up_torque_cmd_nm =
                        PitchUpApplySoftLimit(pitch_up_measure_rad,
                                              up_torque_cmd_nm);
#else
                    pitch_up_soft_limit_active = 0U;
#endif
                } else {
                    pitch_up_soft_limit_active = 0U;
                }

                dzx_pitch_torque = up_torque_cmd_nm;
                pitch_up_torque_cmd_nm = dzx_pitch_torque;
                if (force_down_zero_torque != 0U) {
                    down_torque_cmd_nm = 0.0f;
                    PitchDownSetTorqueImmediate(0.0f);
                } else {
                    down_torque_cmd_nm = PitchDownSetTorqueWithSlew(down_torque_cmd_nm);
                }
                if (pitch_down_output_torque_slew_active != 0U) {
                    double_pitch_output.tau_big_limit_flag = 1U;
                }
                double_pitch_output.tau_big_cmd_nm = down_torque_cmd_nm;
                DMMotorSetTorque(pitch_motor_up, up_torque_cmd_nm);
                PitchUpUpdateIdentDebug(dt_s,
                                        1U,
                                        feedback_ok,
                                        (force_up_zero_torque == 0U) ? 1U : 0U,
                                        &double_pitch_input,
                                        &double_pitch_output,
                                        up_torque_cmd_nm);
                DoublePitchIdentUpdateDebug(dt_s,
                                            1U,
                                            feedback_ok,
                                            double_pitch_target_id,
                                            &double_pitch_lqr,
                                            &double_pitch_input,
                                            &double_pitch_output,
                                            down_torque_cmd_nm,
                                            up_torque_cmd_nm);
            }
        }
    }

    PitchDownUpdateDebugCsv();
    gimbal_feedback_data.gimbal_imu_data = gimbal_IMU_data;
#endif

    GimbalUpdatePitchFeedbackMessage();
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
#endif
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
    float pitch_ident_dt_s;
    float pitch_up_torque_motor_nm;
    uint8_t pitch_feedback_ok;
    uint8_t pitch_torque_output_allowed;

    // 获取云台控制消息
    // 执行模式切换与前馈控制
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    pitch_ident_dt_s = DWT_GetDeltaT(&pitch_up_ident_dwt_cnt);
    if(test_hole==GIMBAL_HOLE_MODE&&gimbal_cmd_recv.gimbal_mode!=GIMBAL_ZERO_FORCE){
        gimbal_cmd_recv.gimbal_mode=test_hole;
        gimbal_cmd_recv.hole_yaw_ready=1;
    }
    
    if (gimbal_cmd_recv.hole_restore_active != 0U &&
        gimbal_cmd_recv.gimbal_mode != GIMBAL_HOLE_MODE) {
        GimbalSetAutoAimActive(0U);
        PitchUpIdentAutoAbortIfActive(PITCH_UP_IDENT_AUTO_ABORT_MODE);
        PitchUpGAutoAbortIfActive();
        PitchUpDynAutoAbortIfActive();
        PitchDownIdentAutoAbortIfActive(PITCH_DOWN_IDENT_AUTO_ABORT_MODE);
        GimbalHoleApplyRestoreControl();
        PitchDownUpdateDebugCsv();
        gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
        GimbalUpdatePitchFeedbackMessage();
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    switch (gimbal_cmd_recv.gimbal_mode) {
    case GIMBAL_ZERO_FORCE:
        GimbalSetAutoAimActive(0U);
        DMMotorSetTorque(pitch_motor_up, 0.0f);
        DMMotorStop(pitch_motor_up);
        PitchDownSetTorqueImmediate(0.0f);
        DMMotorStop(pitch_motor_down);
        GimbalHoleResetState();
        PitchUpIdentAutoAbortIfActive(PITCH_UP_IDENT_AUTO_ABORT_MODE);
        PitchUpGAutoAbortIfActive();
        PitchUpDynAutoAbortIfActive();
        PitchDownIdentAutoAbortIfActive(PITCH_DOWN_IDENT_AUTO_ABORT_MODE);
        PitchUpStopControl();
        PitchDownStopControl();
        PitchUpUpdateIdentDebug(pitch_ident_dt_s, 0U, 0U, 0U, NULL, NULL, 0.0f);
        break;
    case GIMBAL_GYRO_MODE:
    case GIMBAL_MOTOR_MODE:
        GimbalHoleResetState();
        DMMotorEnable1(pitch_motor_down);

        DMMotorEnable1(pitch_motor_up);
        if (PitchUpIdentAutoActive() != 0U) {
            GimbalSetAutoAimActive(0U);
            PitchUpGAutoAbortIfActive();
            PitchUpDynAutoAbortIfActive();
            PitchDownIdentAutoAbortIfActive(PITCH_DOWN_IDENT_AUTO_ABORT_MODE);
            PitchDownApplyNormalFollowRef();
            PitchUpIdentAutoUpdate(pitch_ident_dt_s);
        } else if (PitchDownIdentAutoActive() != 0U) {
            GimbalSetAutoAimActive(0U);
            PitchUpGAutoAbortIfActive();
            PitchUpDynAutoAbortIfActive();
            PitchUpSetRefOverride(PITCH_DOWN_IDENT_UP_HOLD_REF_RAD);
            PitchDownIdentAutoUpdate(pitch_ident_dt_s);
        } else {
            PitchUpGAutoAbortIfActive();
            PitchUpDynAutoAbortIfActive();
            PitchDownIdentAutoAbortIfActive(PITCH_DOWN_IDENT_AUTO_ABORT_MODE);
            PitchUpClearRefOverride();
            PitchDownApplyNormalFollowRef();
            GimbalSetAutoAimActive(GimbalAutoAimCommandActive());
        }
        pitch_up_torque_motor_nm = PitchUpCalculateTorque(0U);
        DMMotorSetTorque(pitch_motor_up, pitch_up_torque_motor_nm);
        PitchDownApplyHybridControlWithGain(hole_down_enter_pre_mit_kp,
                                            hole_down_enter_pre_mit_kd,
                                            0.0f);
        pitch_feedback_ok = (gimbal_IMU_data != NULL &&
                             pitch_motor_up != NULL &&
                             DMMotorIsOnline(pitch_motor_up) != 0U) ? 1U : 0U;
        pitch_torque_output_allowed =
            (PitchUpIdentAutoActive() != 0U ||
             PitchDownIdentAutoActive() != 0U) ?
            1U :
            ((PitchUpGAutoActive() != 0U) ?
             1U : PitchUpTorqueOutputAllowed());
        if (PitchUpNonHoleDownZeroTorqueActive() != 0U) {
            pitch_torque_output_allowed = 0U;
        }
        PitchUpUpdateIdentDebug(pitch_ident_dt_s,
                                (pitch_feedback_ok != 0U &&
                                 pitch_torque_output_allowed != 0U) ? 1U : 0U,
                                pitch_feedback_ok,
                                pitch_torque_output_allowed,
                                NULL,
                                NULL,
                                pitch_up_torque_motor_nm);
        break;
    case GIMBAL_HOLE_MODE:
        GimbalSetAutoAimActive(0U);
        PitchUpIdentAutoAbortIfActive(PITCH_UP_IDENT_AUTO_ABORT_MODE);
        PitchUpGAutoAbortIfActive();
        PitchUpDynAutoAbortIfActive();
        PitchDownIdentAutoAbortIfActive(PITCH_DOWN_IDENT_AUTO_ABORT_MODE);
        GimbalHoleApplyPitchControl();
        break;
    default:
        GimbalSetAutoAimActive(0U);
        DMMotorSetTorque(pitch_motor_up, 0.0f);
        DMMotorStop(pitch_motor_up);
        PitchDownSetTorqueImmediate(0.0f);
        DMMotorStop(pitch_motor_down);
        GimbalHoleResetState();
        PitchUpIdentAutoAbortIfActive(PITCH_UP_IDENT_AUTO_ABORT_MODE);
        PitchUpGAutoAbortIfActive();
        PitchUpDynAutoAbortIfActive();
        PitchDownIdentAutoAbortIfActive(PITCH_DOWN_IDENT_AUTO_ABORT_MODE);
        PitchUpStopControl();
        PitchDownStopControl();
        PitchUpUpdateIdentDebug(pitch_ident_dt_s, 0U, 0U, 0U, NULL, NULL, 0.0f);
        break;
    }

    PitchDownUpdateDebugCsv();
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    GimbalUpdatePitchFeedbackMessage();
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
#endif
