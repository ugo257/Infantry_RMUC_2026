/**
 * @file ins_task.c
 * @brief BMI088 惯导采集和 QuaternionEKF 姿态解算。
 */

#include "ins_task.h"
#include "AHRS_middleware.h"
#include "QuaternionEKF.h"
#include "robot_params.h"
#include "arm_math.h"
#include <stddef.h>

/* ----------------------------- 参数与状态 ----------------------------- */

/**
 * @brief EKF 和运行时零偏检测参数。
 *
 * 这些参数只在本文件内部维护。PC 端当前只允许在线修改 q1/q2/r/lambda，
 * 其余参数作为固件默认策略保留。
 */
typedef struct
{
    float q1_base;                 // EKF 姿态状态过程噪声 Q1
    float q2_base;                 // EKF 零偏状态过程噪声 Q2
    float r;                       // EKF 加速度观测噪声 R
    float lambda;                  // EKF 零偏协方差遗忘因子
    float chi_square_threshold;    // 加速度方向残差平方门限
    float static_gyro_norm_dps;    // 静止判定：三轴角速度模长上限，单位 deg/s
    float static_accel_norm_tol;   // 静止判定：加速度模长相对 9.81 的容许误差
    float static_accel_delta_tol;  // 静止判定：原始加速度相对低通加速度的抖动上限
    float static_lpf_hz;           // 静止判定指标低通频率，单位 Hz
    float static_hold_time_s;      // 满足候选条件后进入静止的保持时间
    float static_release_time_s;   // 不满足候选条件后退出静止的保持时间
    float gyro_z_bias_lpf_hz;      // z 轴零偏估计低通频率，单位 Hz
    float gyro_z_bias_limit_dps;   // z 轴零偏估计限幅，单位 deg/s
    uint8_t chi_square_test_enable; // 是否启用 EKF 内部卡方检测
} INS_EKF_Param_t;

static const INS_EKF_Param_t k_ins_ekf_default_param = {
    .q1_base = 4.5f,
    .q2_base = 0.00035f,
    .r = 2500000.0f,
    .lambda = 0.9999f,
    .chi_square_threshold = 0.0300f,
    .static_gyro_norm_dps = 1.60f,
    .static_accel_norm_tol = 0.30f,
    .static_accel_delta_tol = 0.45f,
    .static_lpf_hz = 8.0f,
    .static_hold_time_s = 0.20f,
    .static_release_time_s = 0.0f,
    .gyro_z_bias_lpf_hz = 0.80f,
    .gyro_z_bias_limit_dps = 1.20f,
    .chi_square_test_enable = 1u,
};

typedef struct
{
    float gyro_norm_dps_lpf;
    float accel_norm_lpf;
    float accel_err_lpf;
    float accel_delta_lpf;
    float hold_time;
    float release_time;
    uint8_t candidate;
    uint8_t is_static;
} INS_StaticDetector_t;

static INS_EKF_Param_t g_ins_ekf_param = {0};

/* INS 保持为全局符号，便于 SWD/GDB 工具直接观察 INS->output。 */
INS_Instance *INS = NULL;

static const float k_gyro_install_matrix[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
static const float k_accel_install_matrix[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
static const float k_accel_lpf_coef[3] = {
    1.929454039488895f,
    -0.93178349823448126f,
    0.002329458745586203f,
};

static float g_gyro_offset[3] = {0.0f, 0.0f, 0.0f};
static float g_accel_offset[3] = {0.0f, 0.0f, 0.0f};
static float g_accel_lpf_1[3] = {0.0f, 0.0f, 0.0f};
static float g_accel_lpf_2[3] = {0.0f, 0.0f, 0.0f};
static float g_accel_lpf_3[3] = {0.0f, 0.0f, 0.0f};
static float g_gyro_z_bias_est_rad = 0.0f;
static INS_StaticDetector_t g_static_detector = {0};
static uint8_t g_ekf_initialized = 0u;

#define IMU_BIAS_CALIB_DEFAULT_SAMPLE_TARGET 20000U
#define IMU_BIAS_CALIB_DEFAULT_WARMUP_TICKS  1000U
#define IMU_BIAS_CALIB_SAMPLE_TARGET_MAX     600000U
#define IMU_BIAS_CALIB_WARMUP_TICKS_MAX      60000U

volatile ImuBiasCalibCmd_s g_imu_bias_calib_cmd = {
    .enable = 0u,
    .start = 0u,
    .abort = 0u,
    .reserved = 0u,
    .run_id = 0u,
    .sample_target = IMU_BIAS_CALIB_DEFAULT_SAMPLE_TARGET,
    .warmup_ticks = IMU_BIAS_CALIB_DEFAULT_WARMUP_TICKS,
};

volatile ImuBiasCalibRecord_s g_imu_bias_calib_record = {0};

static ImuBiasCalibState_e g_imu_bias_calib_state = IMU_BIAS_CALIB_STATE_IDLE;
static uint8_t g_imu_bias_calib_prev_start = 0u;
static float g_imu_bias_calib_gyro_m2[3] = {0.0f, 0.0f, 0.0f};
static float g_imu_bias_calib_acc_m2[3] = {0.0f, 0.0f, 0.0f};

/* ----------------------------- 内部工具函数 ----------------------------- */

static inline float INS_Clampf(float x, float min, float max)
{
    if (x < min)
    {
        return min;
    }
    if (x > max)
    {
        return max;
    }
    return x;
}

static inline float INS_Absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static inline uint8_t INS_IsFinitef(float x)
{
    return (uint8_t)((x == x) && (x <= 3.402823466e38f) && (x >= -3.402823466e38f));
}

static inline float INS_Sqrtf(float x)
{
    float result = 0.0f;

    if (INS_IsFinitef(x) && (x > 0.0f) && (arm_sqrt_f32(x, &result) == ARM_MATH_SUCCESS))
    {
        return result;
    }

    return 0.0f;
}

static uint8_t INS_QuatValid(const float q[4])
{
    float norm_sq;

    if (q == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (!INS_IsFinitef(q[i]))
        {
            return 0u;
        }
    }

    norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    return (uint8_t)(INS_IsFinitef(norm_sq) && norm_sq > 0.25f && norm_sq < 2.25f);
}

static uint8_t INS_QEKFOutputValid(void)
{
    if (INS_QuatValid(QEKF_INS.q) == 0u)
    {
        return 0u;
    }

    return (uint8_t)(INS_IsFinitef(QEKF_INS.Roll) &&
                     INS_IsFinitef(QEKF_INS.Pitch) &&
                     INS_IsFinitef(QEKF_INS.Yaw) &&
                     INS_IsFinitef(QEKF_INS.YawTotalAngle));
}

static uint8_t INS_QEKFZeroOutputGlitch(const float last_angle_deg[3],
                                        const float candidate_angle_deg[3],
                                        float candidate_yaw_total_deg)
{
    float last_norm;
    float candidate_norm;

    if (last_angle_deg == NULL || candidate_angle_deg == NULL)
    {
        return 0u;
    }

    last_norm = INS_Sqrtf(last_angle_deg[0] * last_angle_deg[0] +
                          last_angle_deg[1] * last_angle_deg[1] +
                          last_angle_deg[2] * last_angle_deg[2]);
    candidate_norm = INS_Sqrtf(candidate_angle_deg[0] * candidate_angle_deg[0] +
                               candidate_angle_deg[1] * candidate_angle_deg[1] +
                               candidate_angle_deg[2] * candidate_angle_deg[2]);

    return (uint8_t)(last_norm > 0.50f &&
                     candidate_norm < 0.02f &&
                     INS_Absf(candidate_yaw_total_deg) < 0.02f);
}

static void INS_RestoreQEKFOutput(const float quat[4],
                                  const float angle_deg[3],
                                  float yaw_total_deg)
{
    if (quat == NULL || angle_deg == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < 4u; i++)
    {
        QEKF_INS.q[i] = quat[i];
        QEKF_INS.IMU_QuaternionEKF.FilteredValue[i] = quat[i];
        QEKF_INS.IMU_QuaternionEKF.xhat_data[i] = quat[i];
        QEKF_INS.IMU_QuaternionEKF.xhatminus_data[i] = quat[i];
    }

    QEKF_INS.Roll = angle_deg[INS_ROLL_ADDRESS_OFFSET];
    QEKF_INS.Pitch = angle_deg[INS_PITCH_ADDRESS_OFFSET];
    QEKF_INS.Yaw = angle_deg[INS_YAW_ADDRESS_OFFSET];
    QEKF_INS.YawTotalAngle = yaw_total_deg;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
    QEKF_INS.YawRoundCount = (int16_t)((QEKF_INS.YawTotalAngle - QEKF_INS.Yaw) / 360.0f);
}

static inline float INS_Vec3Norm(const float v[3])
{
    return INS_Sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static uint8_t INS_BMI088DataValid(const BMI088_Data_t *raw)
{
    float acc_power;

    if (raw == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < 3u; i++)
    {
        if (!INS_IsFinitef(raw->gyro[i]) || !INS_IsFinitef(raw->acc[i]))
        {
            return 0u;
        }
    }
    if (!INS_IsFinitef(raw->temperature))
    {
        return 0u;
    }

    acc_power = raw->acc[0] * raw->acc[0] +
                raw->acc[1] * raw->acc[1] +
                raw->acc[2] * raw->acc[2];

    return (uint8_t)(INS_IsFinitef(acc_power) && acc_power > 0.25f && acc_power < 2500.0f);
}

static inline float INS_FirstOrderLpfAlpha(float hz, float dt)
{
    float alpha;

    if (dt <= 1e-6f || hz <= 0.0f)
    {
        return 0.0f;
    }

    alpha = 2.0f * PI * hz * dt;
    alpha = alpha / (1.0f + alpha);
    return INS_Clampf(alpha, 0.0f, 1.0f);
}

static void INS_RpyRadToQuatRep103(float roll_rad, float pitch_rad, float yaw_rad, float quat[4])
{
    const float half_roll = roll_rad * 0.5f;
    const float half_pitch = pitch_rad * 0.5f;
    const float half_yaw = yaw_rad * 0.5f;
    const float cr = arm_cos_f32(half_roll);
    const float sr = arm_sin_f32(half_roll);
    const float cp = arm_cos_f32(half_pitch);
    const float sp = arm_sin_f32(half_pitch);
    const float cy = arm_cos_f32(half_yaw);
    const float sy = arm_sin_f32(half_yaw);

    if (quat == NULL)
    {
        return;
    }

    quat[0] = cy * cp * cr + sy * sp * sr;
    quat[1] = cy * cp * sr - sy * sp * cr;
    quat[2] = cy * sp * cr + sy * cp * sr;
    quat[3] = sy * cp * cr - cy * sp * sr;
}

static void INS_EKF_ParamSanitize(INS_EKF_Param_t *param)
{
    if (param == NULL)
    {
        return;
    }

    param->q1_base = INS_Clampf(param->q1_base, 1e-6f, 1e6f);
    param->q2_base = INS_Clampf(param->q2_base, 1e-8f, 1e3f);
    param->r = INS_Clampf(param->r, 1.0f, 1e9f);
    param->lambda = INS_Clampf(param->lambda, 1e-4f, 1.0f);
    param->chi_square_threshold = INS_Clampf(param->chi_square_threshold, 1e-6f, 1e3f);
    param->static_gyro_norm_dps = INS_Clampf(param->static_gyro_norm_dps, 0.0f, 100.0f);
    param->static_accel_norm_tol = INS_Clampf(param->static_accel_norm_tol, 0.001f, 5.0f);
    param->static_accel_delta_tol = INS_Clampf(param->static_accel_delta_tol, 0.001f, 10.0f);
    param->static_lpf_hz = INS_Clampf(param->static_lpf_hz, 0.1f, 100.0f);
    param->static_hold_time_s = INS_Clampf(param->static_hold_time_s, 0.0f, 5.0f);
    param->static_release_time_s = INS_Clampf(param->static_release_time_s, 0.0f, 1.0f);
    param->gyro_z_bias_lpf_hz = INS_Clampf(param->gyro_z_bias_lpf_hz, 0.001f, 10.0f);
    param->gyro_z_bias_limit_dps = INS_Clampf(param->gyro_z_bias_limit_dps, 0.0f, 10.0f);
    param->chi_square_test_enable = (param->chi_square_test_enable != 0u) ? 1u : 0u;
}

static void INS_EKF_ParamResetToDefault(void)
{
    g_ins_ekf_param = k_ins_ekf_default_param;
    INS_EKF_ParamSanitize(&g_ins_ekf_param);
}

void INS_EKF_SetCoreParams(float q1_pc, float q2_pc, float r_pc, float lambda_pc)
{
    g_ins_ekf_param.q1_base = q1_pc;
    g_ins_ekf_param.q2_base = q2_pc;
    g_ins_ekf_param.r = r_pc;
    g_ins_ekf_param.lambda = lambda_pc;
    INS_EKF_ParamSanitize(&g_ins_ekf_param);
}

static void INS_ImuBiasCalibCopyOldOffsets(void)
{
    for (uint8_t i = 0u; i < 3u; i++)
    {
        g_imu_bias_calib_record.old_gyro_offset[i] = g_gyro_offset[i];
        g_imu_bias_calib_record.old_acc_offset[i] = g_accel_offset[i];
    }
}

static void INS_ImuBiasCalibClearStats(void)
{
    g_imu_bias_calib_record.sample_count = 0u;
    g_imu_bias_calib_record.invalid_count = 0u;
    g_imu_bias_calib_record.warmup_count = 0u;
    g_imu_bias_calib_record.valid = 0u;
    g_imu_bias_calib_record.done = 0u;
    g_imu_bias_calib_record.abort_reason = IMU_BIAS_CALIB_ABORT_NONE;

    for (uint8_t i = 0u; i < 3u; i++)
    {
        g_imu_bias_calib_record.gyro_mean_body_raw[i] = 0.0f;
        g_imu_bias_calib_record.acc_mean_body_raw[i] = 0.0f;
        g_imu_bias_calib_record.gyro_std_body_raw[i] = 0.0f;
        g_imu_bias_calib_record.acc_std_body_raw[i] = 0.0f;
        g_imu_bias_calib_gyro_m2[i] = 0.0f;
        g_imu_bias_calib_acc_m2[i] = 0.0f;
    }
}

static uint8_t INS_ImuBiasCalibTempReady(float temperature_c)
{
    return (uint8_t)(INS_IsFinitef(temperature_c) &&
                     temperature_c >= BMI088_TEMP_READY_LOW_C &&
                     temperature_c <= BMI088_TEMP_READY_HIGH_C);
}

static uint8_t INS_ImuBiasCalibSampleReady(void)
{
    return (uint8_t)(g_static_detector.candidate != 0u &&
                     g_static_detector.is_static != 0u);
}

static uint8_t INS_ImuBiasCalibValuesValid(const float gyro_body_raw[3],
                                           const float acc_body_raw[3],
                                           float temperature_c)
{
    if (gyro_body_raw == NULL || acc_body_raw == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < 3u; i++)
    {
        if (!INS_IsFinitef(gyro_body_raw[i]) || !INS_IsFinitef(acc_body_raw[i]))
        {
            return 0u;
        }
    }

    return INS_IsFinitef(temperature_c);
}

static void INS_ImuBiasCalibAbort(ImuBiasCalibAbortReason_e reason)
{
    g_imu_bias_calib_state = IMU_BIAS_CALIB_STATE_ABORT;
    g_imu_bias_calib_record.state = IMU_BIAS_CALIB_STATE_ABORT;
    g_imu_bias_calib_record.valid = 0u;
    g_imu_bias_calib_record.done = 0u;
    g_imu_bias_calib_record.abort_reason = (uint8_t)reason;
}

static void INS_ImuBiasCalibEnterIdle(ImuBiasCalibAbortReason_e reason)
{
    g_imu_bias_calib_state = IMU_BIAS_CALIB_STATE_IDLE;
    g_imu_bias_calib_record.state = IMU_BIAS_CALIB_STATE_IDLE;
    g_imu_bias_calib_record.valid = 0u;
    g_imu_bias_calib_record.done = 0u;
    g_imu_bias_calib_record.abort_reason = (uint8_t)reason;
}

static void INS_ImuBiasCalibStart(void)
{
    uint32_t sample_target = g_imu_bias_calib_cmd.sample_target;
    uint32_t warmup_ticks = g_imu_bias_calib_cmd.warmup_ticks;

    if (sample_target == 0u)
    {
        sample_target = IMU_BIAS_CALIB_DEFAULT_SAMPLE_TARGET;
    }
    if (warmup_ticks == 0u)
    {
        warmup_ticks = IMU_BIAS_CALIB_DEFAULT_WARMUP_TICKS;
    }
    if (sample_target > IMU_BIAS_CALIB_SAMPLE_TARGET_MAX ||
        warmup_ticks > IMU_BIAS_CALIB_WARMUP_TICKS_MAX)
    {
        INS_ImuBiasCalibAbort(IMU_BIAS_CALIB_ABORT_BAD_PARAM);
        return;
    }

    INS_ImuBiasCalibClearStats();
    INS_ImuBiasCalibCopyOldOffsets();
    g_imu_bias_calib_record.run_id = g_imu_bias_calib_cmd.run_id;
    g_imu_bias_calib_record.sample_target = sample_target;
    g_imu_bias_calib_record.warmup_ticks = warmup_ticks;
    g_imu_bias_calib_state = IMU_BIAS_CALIB_STATE_WAIT_TEMP;
    g_imu_bias_calib_record.state = IMU_BIAS_CALIB_STATE_WAIT_TEMP;
}

static void INS_ImuBiasCalibAddSample(const float gyro_body_raw[3],
                                      const float acc_body_raw[3])
{
    uint32_t n;

    g_imu_bias_calib_record.sample_count++;
    n = g_imu_bias_calib_record.sample_count;

    for (uint8_t i = 0u; i < 3u; i++)
    {
        float gyro_delta;
        float gyro_delta2;
        float acc_delta;
        float acc_delta2;

        gyro_delta = gyro_body_raw[i] - g_imu_bias_calib_record.gyro_mean_body_raw[i];
        g_imu_bias_calib_record.gyro_mean_body_raw[i] += gyro_delta / (float)n;
        gyro_delta2 = gyro_body_raw[i] - g_imu_bias_calib_record.gyro_mean_body_raw[i];
        g_imu_bias_calib_gyro_m2[i] += gyro_delta * gyro_delta2;

        acc_delta = acc_body_raw[i] - g_imu_bias_calib_record.acc_mean_body_raw[i];
        g_imu_bias_calib_record.acc_mean_body_raw[i] += acc_delta / (float)n;
        acc_delta2 = acc_body_raw[i] - g_imu_bias_calib_record.acc_mean_body_raw[i];
        g_imu_bias_calib_acc_m2[i] += acc_delta * acc_delta2;

        if (n > 1u)
        {
            g_imu_bias_calib_record.gyro_std_body_raw[i] =
                INS_Sqrtf(g_imu_bias_calib_gyro_m2[i] / (float)(n - 1u));
            g_imu_bias_calib_record.acc_std_body_raw[i] =
                INS_Sqrtf(g_imu_bias_calib_acc_m2[i] / (float)(n - 1u));
        }
    }
}

static void INS_ImuBiasCalibUpdate(const float gyro_body_raw[3],
                                   const float acc_body_raw[3],
                                   float temperature_c)
{
    const uint8_t enable = g_imu_bias_calib_cmd.enable;
    const uint8_t start = g_imu_bias_calib_cmd.start;
    const uint8_t start_edge = (uint8_t)(enable != 0u &&
                                         start != 0u &&
                                         g_imu_bias_calib_prev_start == 0u);
    const uint8_t temp_ready = INS_ImuBiasCalibTempReady(temperature_c);
    const uint8_t static_ready = INS_ImuBiasCalibSampleReady();
    const uint8_t values_valid =
        INS_ImuBiasCalibValuesValid(gyro_body_raw, acc_body_raw, temperature_c);

    g_imu_bias_calib_prev_start = (enable != 0u) ? start : 0u;
    g_imu_bias_calib_record.sample_time_s = DWT_GetTimeline_s();
    g_imu_bias_calib_record.temperature_c = temperature_c;
    g_imu_bias_calib_record.gyro_norm_dps = INS_Vec3Norm(gyro_body_raw) * RAD_TO_ANGLE;
    g_imu_bias_calib_record.acc_norm_mps2 = INS_Vec3Norm(acc_body_raw);
    INS_ImuBiasCalibCopyOldOffsets();

    if (enable == 0u)
    {
        INS_ImuBiasCalibEnterIdle((g_imu_bias_calib_state == IMU_BIAS_CALIB_STATE_IDLE) ?
                                      IMU_BIAS_CALIB_ABORT_NONE :
                                      IMU_BIAS_CALIB_ABORT_DISABLED);
        return;
    }

    if (g_imu_bias_calib_cmd.abort != 0u)
    {
        INS_ImuBiasCalibAbort(IMU_BIAS_CALIB_ABORT_USER);
        return;
    }

    if (start_edge != 0u)
    {
        INS_ImuBiasCalibStart();
    }

    switch (g_imu_bias_calib_state)
    {
    case IMU_BIAS_CALIB_STATE_WAIT_TEMP:
        if (temp_ready != 0u)
        {
            g_imu_bias_calib_state = IMU_BIAS_CALIB_STATE_WARMUP;
            g_imu_bias_calib_record.state = IMU_BIAS_CALIB_STATE_WARMUP;
        }
        break;

    case IMU_BIAS_CALIB_STATE_WARMUP:
        if (temp_ready != 0u && static_ready != 0u && values_valid != 0u)
        {
            if (g_imu_bias_calib_record.warmup_count < g_imu_bias_calib_record.warmup_ticks)
            {
                g_imu_bias_calib_record.warmup_count++;
            }
            if (g_imu_bias_calib_record.warmup_count >= g_imu_bias_calib_record.warmup_ticks)
            {
                g_imu_bias_calib_state = IMU_BIAS_CALIB_STATE_SAMPLE;
                g_imu_bias_calib_record.state = IMU_BIAS_CALIB_STATE_SAMPLE;
            }
        }
        else
        {
            g_imu_bias_calib_record.invalid_count++;
        }
        break;

    case IMU_BIAS_CALIB_STATE_SAMPLE:
        if (temp_ready != 0u && static_ready != 0u && values_valid != 0u)
        {
            INS_ImuBiasCalibAddSample(gyro_body_raw, acc_body_raw);
            if (g_imu_bias_calib_record.sample_count >= g_imu_bias_calib_record.sample_target)
            {
                g_imu_bias_calib_state = IMU_BIAS_CALIB_STATE_DONE;
                g_imu_bias_calib_record.state = IMU_BIAS_CALIB_STATE_DONE;
                g_imu_bias_calib_record.valid = 1u;
                g_imu_bias_calib_record.done = 1u;
                g_imu_bias_calib_record.abort_reason = IMU_BIAS_CALIB_ABORT_NONE;
            }
        }
        else
        {
            g_imu_bias_calib_record.invalid_count++;
        }
        break;

    case IMU_BIAS_CALIB_STATE_DONE:
        g_imu_bias_calib_record.valid = 1u;
        g_imu_bias_calib_record.done = 1u;
        break;

    case IMU_BIAS_CALIB_STATE_ABORT:
    case IMU_BIAS_CALIB_STATE_IDLE:
    default:
        g_imu_bias_calib_record.state = (uint8_t)g_imu_bias_calib_state;
        break;
    }
}

/**
 * @brief 将 BMI088 数据旋转到机体系，并扣除静态零偏。
 */
static void INS_ApplyInstallMatrix(float gyro[3], float accel[3], float mag[3], const BMI088_Data_t *raw)
{
    for (uint8_t i = 0u; i < 3u; i++)
    {
        gyro[i] = raw->gyro[0] * k_gyro_install_matrix[i][0] +
                  raw->gyro[1] * k_gyro_install_matrix[i][1] +
                  raw->gyro[2] * k_gyro_install_matrix[i][2] -
                  g_gyro_offset[i];

        accel[i] = raw->acc[0] * k_accel_install_matrix[i][0] +
                   raw->acc[1] * k_accel_install_matrix[i][1] +
                   raw->acc[2] * k_accel_install_matrix[i][2] -
                   g_accel_offset[i];

        mag[i] = 0.0f; // 当前 EKF 不使用磁力计，保持字段存在以兼容 INS_Instance。
    }
}

/**
 * @brief 二阶 IIR 低通滤波加速度。
 *
 * 低通后的加速度用于 EKF 观测更新，可降低机械振动对姿态估计的影响。
 */
static void INS_UpdateAccelLpf(const float accel[3])
{
    for (uint8_t i = 0u; i < 3u; i++)
    {
        g_accel_lpf_1[i] = g_accel_lpf_2[i];
        g_accel_lpf_2[i] = g_accel_lpf_3[i];
        g_accel_lpf_3[i] = g_accel_lpf_2[i] * k_accel_lpf_coef[0] +
                            g_accel_lpf_1[i] * k_accel_lpf_coef[1] +
                            accel[i] * k_accel_lpf_coef[2];
    }
}

static void INS_EKF_UpdateStaticDetector(float dt)
{
    float gyro_motion[3];
    float accel_delta[3];
    float gyro_norm_dps;
    float accel_norm;
    float accel_err;
    float accel_delta_norm;
    float alpha;
    float gyro_limit;
    float accel_limit;
    float accel_delta_limit;

    if (dt <= 1e-6f)
    {
        return;
    }

    gyro_motion[INS_ROLL_ADDRESS_OFFSET] =
        (INS->INS_data.INS_gyro[INS_ROLL_ADDRESS_OFFSET] - QEKF_INS.GyroBias[INS_ROLL_ADDRESS_OFFSET]) *
        RAD_TO_ANGLE;
    gyro_motion[INS_PITCH_ADDRESS_OFFSET] =
        (INS->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET] - QEKF_INS.GyroBias[INS_PITCH_ADDRESS_OFFSET]) *
        RAD_TO_ANGLE;
    gyro_motion[INS_YAW_ADDRESS_OFFSET] =
        (INS->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET] - g_gyro_z_bias_est_rad) * RAD_TO_ANGLE;

    accel_delta[0] = INS->INS_data.INS_accel[0] - g_accel_lpf_3[0];
    accel_delta[1] = INS->INS_data.INS_accel[1] - g_accel_lpf_3[1];
    accel_delta[2] = INS->INS_data.INS_accel[2] - g_accel_lpf_3[2];

    gyro_norm_dps = INS_Vec3Norm(gyro_motion);
    accel_norm = INS_Vec3Norm(g_accel_lpf_3);
    accel_err = INS_Absf(accel_norm - 9.81f);
    accel_delta_norm = INS_Vec3Norm(accel_delta);

    alpha = INS_FirstOrderLpfAlpha(g_ins_ekf_param.static_lpf_hz, dt);
    g_static_detector.gyro_norm_dps_lpf += (gyro_norm_dps - g_static_detector.gyro_norm_dps_lpf) * alpha;
    g_static_detector.accel_norm_lpf += (accel_norm - g_static_detector.accel_norm_lpf) * alpha;
    g_static_detector.accel_err_lpf += (accel_err - g_static_detector.accel_err_lpf) * alpha;
    g_static_detector.accel_delta_lpf += (accel_delta_norm - g_static_detector.accel_delta_lpf) * alpha;

    gyro_limit = g_ins_ekf_param.static_gyro_norm_dps;
    accel_limit = g_ins_ekf_param.static_accel_norm_tol;
    accel_delta_limit = g_ins_ekf_param.static_accel_delta_tol;

    if (g_static_detector.is_static != 0u)
    {
        gyro_limit = gyro_limit * 1.8f + 0.15f;
        accel_limit *= 1.7f;
        accel_delta_limit *= 1.7f;
    }

    g_static_detector.candidate =
        (g_static_detector.gyro_norm_dps_lpf < gyro_limit) &&
        (g_static_detector.accel_err_lpf < accel_limit) &&
        (g_static_detector.accel_delta_lpf < accel_delta_limit);

    if (g_static_detector.candidate != 0u)
    {
        g_static_detector.hold_time = INS_Clampf(g_static_detector.hold_time + dt,
                                                  0.0f,
                                                  g_ins_ekf_param.static_hold_time_s);
        g_static_detector.release_time = 0.0f;
        if (g_static_detector.hold_time >= g_ins_ekf_param.static_hold_time_s)
        {
            g_static_detector.is_static = 1u;
        }
    }
    else
    {
        g_static_detector.release_time = INS_Clampf(g_static_detector.release_time + dt,
                                                     0.0f,
                                                     g_ins_ekf_param.static_release_time_s);
        if (g_static_detector.release_time >= g_ins_ekf_param.static_release_time_s)
        {
            g_static_detector.is_static = 0u;
            g_static_detector.hold_time = 0.0f;
        }
    }
}

/**
 * @brief 在稳定静止条件下估计 z 轴陀螺运行时零偏。
 *
 * EKF 状态中只估计 x/y 轴陀螺零偏，z 轴零偏会直接造成 yaw 漂移。
 * 静止状态由独立检测器给出，避免单点阈值把慢速转动误学成零偏。
 */
static void INS_EKF_UpdateGyroZBias(float dt)
{
    if (dt <= 1e-6f || g_static_detector.is_static == 0u || g_static_detector.candidate == 0u)
    {
        return;
    }

    if (g_static_detector.gyro_norm_dps_lpf >= g_ins_ekf_param.static_gyro_norm_dps)
    {
        return;
    }

    if (INS_Absf((INS->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET] - g_gyro_z_bias_est_rad) * RAD_TO_ANGLE) >=
        g_ins_ekf_param.static_gyro_norm_dps)
    {
        return;
    }

    const float bias_limit_rad = g_ins_ekf_param.gyro_z_bias_limit_dps * ANGLE_TO_RAD;
    const float target_bias_rad = INS_Clampf(INS->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],
                                             -bias_limit_rad,
                                             bias_limit_rad);
    const float alpha = INS_FirstOrderLpfAlpha(g_ins_ekf_param.gyro_z_bias_lpf_hz, dt);
    g_gyro_z_bias_est_rad += (target_bias_rad - g_gyro_z_bias_est_rad) * alpha;
}

static void INS_EKF_InitOnce(void)
{
    if (g_ekf_initialized != 0u)
    {
        return;
    }

    INS_EKF_ParamResetToDefault();
    IMU_QuaternionEKF_Init(g_ins_ekf_param.q1_base,
                           g_ins_ekf_param.q2_base,
                           g_ins_ekf_param.r,
                           g_ins_ekf_param.lambda);
    QEKF_INS.ChiSquareTestThreshold = g_ins_ekf_param.chi_square_threshold;
    enChiSquareTest = g_ins_ekf_param.chi_square_test_enable;
    g_ekf_initialized = 1u;
}

/* ----------------------------- 对外接口 ----------------------------- */

INS_Instance *INS_Init(BMI088Instance *bmi088)
{
    BMI088_Data_t raw_data = {0};
    INS_Instance *ins = NULL;

    if (bmi088 == NULL)
    {
        return NULL;
    }

    ins = (INS_Instance *)zmalloc(sizeof(INS_Instance));
    if (ins == NULL)
    {
        return NULL;
    }

    ins->BMI088 = bmi088;
    for (uint8_t i = 0u; i < 3u; i++)
    {
        g_gyro_offset[i] = bmi088->gyro_offset[i];
        g_accel_offset[i] = bmi088->acc_offset[i];
    }

    if (BMI088Acquire(bmi088, &raw_data) != 0u)
    {
        INS_ApplyInstallMatrix(ins->INS_data.INS_gyro,
                               ins->INS_data.INS_accel,
                               ins->INS_data.INS_mag,
                               &raw_data);
    }

    ins->INS_data.INS_quat[0] = 1.0f;
    ins->INS_data.INS_quat[1] = 0.0f;
    ins->INS_data.INS_quat[2] = 0.0f;
    ins->INS_data.INS_quat[3] = 0.0f;

    for (uint8_t i = 0u; i < 3u; i++)
    {
        g_accel_lpf_1[i] = ins->INS_data.INS_accel[i];
        g_accel_lpf_2[i] = ins->INS_data.INS_accel[i];
        g_accel_lpf_3[i] = ins->INS_data.INS_accel[i];
    }

    g_static_detector.gyro_norm_dps_lpf = 100.0f;
    g_static_detector.accel_norm_lpf = INS_Vec3Norm(g_accel_lpf_3);
    g_static_detector.accel_err_lpf = INS_Absf(g_static_detector.accel_norm_lpf - 9.81f);
    g_static_detector.accel_delta_lpf = 0.0f;
    g_static_detector.hold_time = 0.0f;
    g_static_detector.release_time = 0.0f;
    g_static_detector.candidate = 0u;
    g_static_detector.is_static = 0u;
    g_gyro_z_bias_est_rad = 0.0f;

    INS = ins;
    INS_EKF_InitOnce();
    return ins;
}

//ins任务
void INS_EKF(void)
{
    static BMI088_Data_t raw_data;
    static uint8_t output_valid = 0u;
    static float last_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    static float last_angle_deg[3] = {0.0f, 0.0f, 0.0f};
    static float last_yaw_total_deg = 0.0f;
    float candidate_angle_deg[3];
    float public_angle_deg[3];
    float gyro_body_raw[3];
    float acc_body_raw[3];
    float dt;

    if (INS == NULL || INS->BMI088 == NULL)
    {
        return;
    }

    if (BMI088Acquire(INS->BMI088, &raw_data) == 0u)
    {
        return;
    }

    if (INS_BMI088DataValid(&raw_data) == 0u)
    {
        return;
    }

    //INS->BMI088->ambient_temperature = BMI088_AMBIENT_TEMPERATURE; //直接给目标温度35度
    BMI088_temp_control(INS->BMI088);

    INS_ApplyInstallMatrix(INS->INS_data.INS_gyro,
                           INS->INS_data.INS_accel,
                           INS->INS_data.INS_mag,
                           &raw_data);
    for (uint8_t i = 0u; i < 3u; i++)
    {
        gyro_body_raw[i] = INS->INS_data.INS_gyro[i] + g_gyro_offset[i];
        acc_body_raw[i] = INS->INS_data.INS_accel[i] + g_accel_offset[i];
    }
    INS_UpdateAccelLpf(INS->INS_data.INS_accel);
    dt = DWT_GetDeltaT(&INS->BMI088->bias_dwt_cnt);
    if (!INS_IsFinitef(dt) || dt <= 0.0f)
    {
        return;
    }
    dt = INS_Clampf(dt, 0.0001f, 0.005f);
    INS->timing_time = dt;

    INS_EKF_UpdateStaticDetector(dt);
    INS_ImuBiasCalibUpdate(gyro_body_raw, acc_body_raw, raw_data.temperature);
    INS_EKF_UpdateGyroZBias(dt);
    IMU_QuaternionEKF_Update(INS->INS_data.INS_gyro[INS_ROLL_ADDRESS_OFFSET],
                             INS->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET],
                             INS->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET] - g_gyro_z_bias_est_rad,
                             g_accel_lpf_3[0],
                             g_accel_lpf_3[1],
                             g_accel_lpf_3[2],
                             dt);
    INS->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET] =
        -INS->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];

    candidate_angle_deg[INS_YAW_ADDRESS_OFFSET] = QEKF_INS.Yaw;
    candidate_angle_deg[INS_PITCH_ADDRESS_OFFSET] = QEKF_INS.Pitch;
    candidate_angle_deg[INS_ROLL_ADDRESS_OFFSET] = QEKF_INS.Roll;

    if (INS_QEKFOutputValid() == 0u)
    {
        if (output_valid != 0u)
        {
            INS_RestoreQEKFOutput(last_quat, last_angle_deg, last_yaw_total_deg);
        }
        return;
    }

    if (output_valid != 0u &&
        INS_QEKFZeroOutputGlitch(last_angle_deg,
                                 candidate_angle_deg,
                                 QEKF_INS.YawTotalAngle) != 0u)
    {
        INS_RestoreQEKFOutput(last_quat, last_angle_deg, last_yaw_total_deg);
        return;
    }

    public_angle_deg[INS_YAW_ADDRESS_OFFSET] = candidate_angle_deg[INS_YAW_ADDRESS_OFFSET];
    public_angle_deg[INS_PITCH_ADDRESS_OFFSET] = -candidate_angle_deg[INS_PITCH_ADDRESS_OFFSET];
    public_angle_deg[INS_ROLL_ADDRESS_OFFSET] = candidate_angle_deg[INS_ROLL_ADDRESS_OFFSET];

    INS->output.INS_angle[INS_YAW_ADDRESS_OFFSET] = public_angle_deg[INS_YAW_ADDRESS_OFFSET] * ANGLE_TO_RAD;
    INS->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] = public_angle_deg[INS_PITCH_ADDRESS_OFFSET] * ANGLE_TO_RAD;
    INS->output.INS_angle[INS_ROLL_ADDRESS_OFFSET] = public_angle_deg[INS_ROLL_ADDRESS_OFFSET] * ANGLE_TO_RAD;
    INS->output.Yaw_total_angle = QEKF_INS.YawTotalAngle * ANGLE_TO_RAD;

    INS->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] = public_angle_deg[INS_YAW_ADDRESS_OFFSET];
    INS->output.INS_angle_deg[INS_PITCH_ADDRESS_OFFSET] = public_angle_deg[INS_PITCH_ADDRESS_OFFSET];
    INS->output.INS_angle_deg[INS_ROLL_ADDRESS_OFFSET] = public_angle_deg[INS_ROLL_ADDRESS_OFFSET];
    INS->output.Yaw_total_angle_deg = QEKF_INS.YawTotalAngle;
    INS_RpyRadToQuatRep103(INS->output.INS_angle[INS_ROLL_ADDRESS_OFFSET],
                           INS->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],
                           INS->output.INS_angle[INS_YAW_ADDRESS_OFFSET],
                           INS->INS_data.INS_quat);

    for (uint8_t i = 0u; i < 4u; i++)
    {
        last_quat[i] = QEKF_INS.q[i];
    }
    for (uint8_t i = 0u; i < 3u; i++)
    {
        last_angle_deg[i] = candidate_angle_deg[i];
    }
    last_yaw_total_deg = QEKF_INS.YawTotalAngle;
    output_valid = 1u;
}
