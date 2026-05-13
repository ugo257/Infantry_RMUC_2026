/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 */

#include "QuaternionEKF.h"

#ifndef QEKF_FAST_EULER_TRIG_ENABLE
#define QEKF_FAST_EULER_TRIG_ENABLE 0
#endif

#ifndef QEKF_OBSERVE_MATRIX_ENABLE
#define QEKF_OBSERVE_MATRIX_ENABLE 0
#endif

#if !QEKF_FAST_EULER_TRIG_ENABLE
#include <math.h>
#endif

QEKF_INS_t QEKF_INS;
uint8_t enChiSquareTest = TRUE;

static const float k_qekf_chi_converge_ratio = 0.45f;
static const float k_qekf_chi_converge_time_s = 0.20f;
static const float k_qekf_rad_to_deg = 57.29577951308232f;
#if QEKF_FAST_EULER_TRIG_ENABLE
static const float k_qekf_pi = 3.14159265358979f;
static const float k_qekf_half_pi = 1.57079632679490f;
#endif

// -------------------------------------------------------------------------
// 本文件对应的标准 EKF 状态空间模型可写为：
//
// 状态向量：
//   x_k = [w, x, y, z, bx, by]^T
//   其中 q=[w,x,y,z]^T 为姿态四元数，b=[bx,by]^T 为陀螺零偏估计。
//   四元数对应的机体系语义遵循 REP-103: x forward, y left, z up。
//   当前实现里 z 轴零偏通常由上层额外处理，因此未并入状态。
//
// 输入向量：
//   u_k = [gx, gy, gz]^T
//   即陀螺仪三轴角速度。
//
// 观测向量：
//   z_k = a_k / ||a_k||
//   即单位化后的加速度方向；在静止或低动态下可视为“机体系下的重力方向观测”。
//
// 状态方程：
//   x_(k+1|k) = f(x_k, u_k) + w_k
//
// 其中姿态部分满足四元数运动学：
//   q_dot = 1/2 * Omega(ω-b) * q
//
// 离散一阶近似为：
//   q_(k+1|k) ≈ (I + 1/2 * Omega(ω-b) * dt) q_(k|k)
//
// 观测方程：
//   z_k = h(x_k) + v_k
//
// 其中 h(q) 为由姿态四元数预测出的机体系重力方向：
//   h(q) = [
//      2(q1q3 - q0q2),
//      2(q0q1 + q2q3),
//      q0^2 - q1^2 - q2^2 + q3^2
//   ]^T
//
// 在 EKF 中实际使用的是上述两个非线性模型在当前工作点的 Jacobian：
//   F_k = ∂f/∂x
//   H_k = ∂h/∂x
//
// 本文件里的主要函数对应关系：
//   IMU_QuaternionEKF_SetStateTransitionJacobian : 构造 F_k
//   IMU_QuaternionEKF_SetMeasurementVector       : 构造 z_k
//   IMU_QuaternionEKF_SetMeasurementPrediction   : 计算 h(x)
//   IMU_QuaternionEKF_SetMeasurementJacobian     : 构造 H_k
//
// 与底层 Kalman_Filter_t 变量的对应关系：
//   xhat         <-> 后验状态估计 x(k|k)
//   xhatminus    <-> 先验状态估计 x(k|k-1)
//   P            <-> 后验协方差 P(k|k)
//   Pminus       <-> 先验协方差 P(k|k-1)
//   F            <-> 状态转移 Jacobian F_k = ∂f/∂x
//   H            <-> 观测 Jacobian H_k = ∂h/∂x
//   Q            <-> 过程噪声协方差
//   R            <-> 量测噪声协方差
//   K            <-> 卡尔曼增益 K_k
//   MeasuredVector <-> 原始观测缓存，当前文件中用于存放 z_k
//   FilteredValue  <-> 最终滤波输出，对应本轮更新后的状态估计
//
// 在当前姿态 EKF 中，这些量的具体含义为：
//   xhat_data[0..3]      = 四元数后验估计 [w, x, y, z]
//   xhat_data[4]         = x 轴陀螺零偏 bx
//   xhat_data[5]         = y 轴陀螺零偏 by
//   xhatminus_data[0..3] = 由状态方程预测得到的四元数先验估计
//   MeasuredVector[0..2] = 单位化加速度，即机体系下的重力方向观测 z_k
//   FilteredValue[0..5]  = 本轮滤波后的状态结果，会同步到 QEKF_INS.q / GyroBias
// -------------------------------------------------------------------------

// EKF 的状态维数为 6：
// x = [w, x, y, z, bx, by]^T
// 其中 q 为姿态四元数，bx/by 为陀螺 x/y 轴零偏。
//
// 对应的状态转移矩阵 F 先以单位阵初始化，
// 后续每个采样周期再根据当前角速度和姿态工作点写入线性化项。
static const float IMU_QuaternionEKF_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

// 状态协方差 P。
// 对角线越大，表示滤波器一开始越“不确定”当前状态。
// 这里四元数和零偏都给了较大的初值，表示允许滤波器在初期快速收敛到合理状态。
static float IMU_QuaternionEKF_P[36] = {
    100000, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 100000, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 100000, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 100000, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 10000, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 10000
};

// 过程噪声 Q。
// 它描述“状态模型本身有多不准”：
// 1. 四元数对应的姿态状态会随着角速度快速变化，因此 Q1 通常比零偏状态大。
// 2. 零偏通常是慢变量，因此 Q2 取更小，表示 bias 在相邻采样间不应剧烈跳变。
static float IMU_QuaternionEKF_Q[36] = {
    0.01f, 0, 0, 0, 0, 0,
    0, 0.01f, 0, 0, 0, 0,
    0, 0, 0.01f, 0, 0, 0,
    0, 0, 0, 0.01f, 0, 0,
    0, 0, 0, 0, 0.000001f, 0,
    0, 0, 0, 0, 0, 0.000001f
};

// 观测噪声 R。
// 本 EKF 只使用“加速度单位向量”作为观测，因此观测维数为 3。
// 从数学上看，z 不是原始加速度值，而是单位化后的“重力方向观测”。
static float IMU_QuaternionEKF_R[9] = {
    1000000, 0, 0,
    0, 1000000, 0,
    0, 0, 1000000
};

#if QEKF_OBSERVE_MATRIX_ENABLE
static float IMU_QuaternionEKF_K[18];
static float IMU_QuaternionEKF_H[18];
#endif

static inline float QEKF_Absf(float x);
static inline float QEKF_Clampf(float x, float min, float max);
static inline float QEKF_Sqrtf(float x);
static inline uint8_t QEKF_IsFinitef(float x);
#if QEKF_FAST_EULER_TRIG_ENABLE
static inline float QEKF_FastAtanUnit(float x);
static inline float QEKF_FastAtan2(float y, float x);
static inline float QEKF_FastAsin(float x);
#endif
static float IMU_QuaternionEKF_VectorInvNorm(const float *vec, uint32_t len);
static uint8_t IMU_QuaternionEKF_NormalizeQuat(float *quat, const float *fallback);
static void IMU_QuaternionEKF_SetMeasurementVector(KalmanFilter_t *kf, float ax, float ay, float az);
static void IMU_QuaternionEKF_SetProcessNoise(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetStateTransitionJacobian(KalmanFilter_t *kf, float gx, float gy, float gz, float dt);
static void IMU_QuaternionEKF_SetMeasurementPrediction(const float *state, float *gravity_body);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_AfterPredict(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetMeasurementJacobian(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_CustomMeasurementUpdate(KalmanFilter_t *kf);

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda)
{
    // Q1/Q2/R/lambda 均开放给上层调参：
    // Q1 控制姿态状态响应速度，Q2 控制零偏收敛速度，R 控制对加速度观测的信任程度。
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 0.01f;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ChiSquareRejectFlag = 0;
    QEKF_INS.AccelValidFlag = 0;
    QEKF_INS.ChiSquare = 0.0f;
    QEKF_INS.ChiSquareGoodTime = 0.0f;
    QEKF_INS.ChiSquareBadTime = 0.0f;
    QEKF_INS.AccelResidualNorm = 0.0f;

    QEKF_INS.YawTotalAngle = 0.0f;
    QEKF_INS.YawRoundCount = 0;
    QEKF_INS.YawAngleLast = 0.0f;
    QEKF_INS.Roll = 0.0f;
    QEKF_INS.Pitch = 0.0f;
    QEKF_INS.Yaw = 0.0f;
    QEKF_INS.q[0] = 1.0f;
    QEKF_INS.q[1] = 0.0f;
    QEKF_INS.q[2] = 0.0f;
    QEKF_INS.q[3] = 0.0f;
    QEKF_INS.GyroBias[0] = 0.0f;
    QEKF_INS.GyroBias[1] = 0.0f;
    QEKF_INS.GyroBias[2] = 0.0f;

    if (lambda > 1.0f)
    {
        lambda = 1.0f;
    }
    QEKF_INS.lambda = lambda;

    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);

    // 初始姿态设为单位四元数：
    // q = [1, 0, 0, 0]
    // 这表示初始时默认机体系和参考系重合。
    // 如果上电姿态与参考系差别不大，这个初值通常可以较快收敛。
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1.0f;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0.0f;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0.0f;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0.0f;

    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_AfterPredict;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetMeasurementJacobian;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_CustomMeasurementUpdate;

    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    arm_copy_f32(IMU_QuaternionEKF_F, QEKF_INS.IMU_QuaternionEKF.F_data, 36);
    arm_copy_f32(IMU_QuaternionEKF_P, QEKF_INS.IMU_QuaternionEKF.P_data, 36);
    arm_copy_f32(IMU_QuaternionEKF_Q, QEKF_INS.IMU_QuaternionEKF.Q_data, 36);
    arm_copy_f32(IMU_QuaternionEKF_R, QEKF_INS.IMU_QuaternionEKF.R_data, 9);
}

void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float q0;
    float q1;
    float q2;
    float q3;
    float yaw_y;
    float yaw_x;
    float roll_y;
    float roll_x;
    float pitch_sin;
    float q_fallback[4];

    QEKF_INS.dt = dt;
    // 标准状态空间写法：
    // x_k = [w, x, y, z, bx, by]^T
    // u_k = [gx, gy, gz]^T
    // z_k = a_k / ||a_k||
    // x_(k+1|k) = f(x_k, u_k) + w_k
    // z_k = h(x_k) + v_k
    IMU_QuaternionEKF_SetStateTransitionJacobian(&QEKF_INS.IMU_QuaternionEKF, gx, gy, gz, dt);
    IMU_QuaternionEKF_SetMeasurementVector(&QEKF_INS.IMU_QuaternionEKF, ax, ay, az);
    IMU_QuaternionEKF_SetProcessNoise(&QEKF_INS.IMU_QuaternionEKF);

    q_fallback[0] = QEKF_INS.q[0];
    q_fallback[1] = QEKF_INS.q[1];
    q_fallback[2] = QEKF_INS.q[2];
    q_fallback[3] = QEKF_INS.q[3];
    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];
    if (IMU_QuaternionEKF_NormalizeQuat(QEKF_INS.q, q_fallback) == 0u)
    {
        QEKF_INS.q[0] = 1.0f;
        QEKF_INS.q[1] = 0.0f;
        QEKF_INS.q[2] = 0.0f;
        QEKF_INS.q[3] = 0.0f;
    }
    QEKF_INS.IMU_QuaternionEKF.FilteredValue[0] = QEKF_INS.q[0];
    QEKF_INS.IMU_QuaternionEKF.FilteredValue[1] = QEKF_INS.q[1];
    QEKF_INS.IMU_QuaternionEKF.FilteredValue[2] = QEKF_INS.q[2];
    QEKF_INS.IMU_QuaternionEKF.FilteredValue[3] = QEKF_INS.q[3];
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = QEKF_INS.q[0];
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = QEKF_INS.q[1];
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = QEKF_INS.q[2];
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = QEKF_INS.q[3];

    if (QEKF_IsFinitef(QEKF_INS.IMU_QuaternionEKF.FilteredValue[4]))
    {
        QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
        QEKF_INS.IMU_QuaternionEKF.xhat_data[4] = QEKF_INS.GyroBias[0];
    }
    else
    {
        QEKF_INS.IMU_QuaternionEKF.FilteredValue[4] = QEKF_INS.GyroBias[0];
        QEKF_INS.IMU_QuaternionEKF.xhat_data[4] = QEKF_INS.GyroBias[0];
    }
    if (QEKF_IsFinitef(QEKF_INS.IMU_QuaternionEKF.FilteredValue[5]))
    {
        QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
        QEKF_INS.IMU_QuaternionEKF.xhat_data[5] = QEKF_INS.GyroBias[1];
    }
    else
    {
        QEKF_INS.IMU_QuaternionEKF.FilteredValue[5] = QEKF_INS.GyroBias[1];
        QEKF_INS.IMU_QuaternionEKF.xhat_data[5] = QEKF_INS.GyroBias[1];
    }
    QEKF_INS.GyroBias[2] = 0.0f;

    q0 = QEKF_INS.q[0];
    q1 = QEKF_INS.q[1];
    q2 = QEKF_INS.q[2];
    q3 = QEKF_INS.q[3];

    yaw_y = 2.0f * (q0 * q3 + q1 * q2);
    yaw_x = 2.0f * (q0 * q0 + q1 * q1) - 1.0f;
    roll_y = 2.0f * (q0 * q1 + q2 * q3);
    roll_x = 2.0f * (q0 * q0 + q3 * q3) - 1.0f;
    pitch_sin = -2.0f * (q1 * q3 - q0 * q2);

    // ------------------------------
    // 4. 由四元数反解欧拉角
    // ------------------------------
    // EKF 真正估计的是四元数 q，而不是欧拉角。
    // 这里将滤波后的 q 转成 REP-103 roll/pitch/yaw，
    // 只是为了方便上层控制逻辑和调试输出直接使用。
#if QEKF_FAST_EULER_TRIG_ENABLE
    QEKF_INS.Yaw = QEKF_FastAtan2(yaw_y, yaw_x) * k_qekf_rad_to_deg;
    QEKF_INS.Roll = QEKF_FastAtan2(roll_y, roll_x) * k_qekf_rad_to_deg;
    QEKF_INS.Pitch = QEKF_FastAsin(pitch_sin) * k_qekf_rad_to_deg;
#else
    QEKF_INS.Yaw = atan2f(yaw_y, yaw_x) * k_qekf_rad_to_deg;
    QEKF_INS.Roll = atan2f(roll_y, roll_x) * k_qekf_rad_to_deg;
    QEKF_INS.Pitch = asinf(QEKF_Clampf(pitch_sin, -1.0f, 1.0f)) * k_qekf_rad_to_deg;
#endif

    // 将 [-180, 180] 的 yaw 展开成连续角，避免过零点时跳变。
    // 例如从 179 跳到 -179，本质上是继续正向旋转 2 度，而不是突然反向 358 度。
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f)
    {
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f)
    {
        QEKF_INS.YawRoundCount++;
    }

    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
}

static void IMU_QuaternionEKF_AfterPredict(KalmanFilter_t *kf)
{
    float q0;
    float q1;
    float q2;
    float q3;

    // ------------------------------
    // 5. 对预测后的四元数做归一化
    // ------------------------------
    // 理论上四元数应满足：
    // q0^2 + q1^2 + q2^2 + q3^2 = 1
    // 但数值积分和浮点误差会让模长逐渐偏离 1，
    // 因此每轮预测后都要重新单位化。
    (void)IMU_QuaternionEKF_NormalizeQuat(kf->xhatminus_data, QEKF_INS.q);

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // 这里补的是 F 的右上角 4x2 分块，
    // 也就是四元数状态对 bias 状态的偏导项：
    // ∂q(k+1)/∂b
    //
    // 它的意义是：
    // 如果零偏估计改变一点，会对下一拍姿态预测造成多大影响。
    // 这部分 Jacobian 对零偏能否被正确估计非常关键。
    kf->F_data[4] = q1 * QEKF_INS.dt * 0.5f;
    kf->F_data[5] = q2 * QEKF_INS.dt * 0.5f;

    kf->F_data[10] = -q0 * QEKF_INS.dt * 0.5f;
    kf->F_data[11] = q3 * QEKF_INS.dt * 0.5f;

    kf->F_data[16] = -q3 * QEKF_INS.dt * 0.5f;
    kf->F_data[17] = -q0 * QEKF_INS.dt * 0.5f;

    kf->F_data[22] = q2 * QEKF_INS.dt * 0.5f;
    kf->F_data[23] = -q1 * QEKF_INS.dt * 0.5f;
}

static void IMU_QuaternionEKF_SetMeasurementJacobian(KalmanFilter_t *kf)
{
    const float doubleq0 = 2.0f * kf->xhatminus_data[0];
    const float doubleq1 = 2.0f * kf->xhatminus_data[1];
    const float doubleq2 = 2.0f * kf->xhatminus_data[2];
    const float doubleq3 = 2.0f * kf->xhatminus_data[3];
    float *H = kf->H_data;

    // ------------------------------
    // 6. 构造观测 Jacobian：H = ∂h/∂x
    // ------------------------------
    // 当前观测模型不是线性的，而是：
    //
    // h(q) = [
    //   2(q1q3 - q0q2),
    //   2(q0q1 + q2q3),
    //   q0^2 - q1^2 - q2^2 + q3^2
    // ]^T
    //
    // 这个 h(q) 表示“由当前姿态预测出来的重力方向”。
    // EKF 需要在当前工作点附近对它线性化，因此这里填写 Jacobian H。
    //
    // 注意观测只与四元数有关，不直接与 bias 有关，
    // 所以 H 的后两列全为 0。
    H[0] = -doubleq2;
    H[1] = doubleq3;
    H[2] = -doubleq0;
    H[3] = doubleq1;
    H[4] = 0.0f;
    H[5] = 0.0f;

    H[6] = doubleq1;
    H[7] = doubleq0;
    H[8] = doubleq3;
    H[9] = doubleq2;
    H[10] = 0.0f;
    H[11] = 0.0f;

    H[12] = doubleq0;
    H[13] = -doubleq1;
    H[14] = -doubleq2;
    H[15] = doubleq3;
    H[16] = 0.0f;
    H[17] = 0.0f;
}

static void IMU_QuaternionEKF_CustomMeasurementUpdate(KalmanFilter_t *kf)
{
    float gravity_body[3];
    float residual_sq;

    QEKF_INS.ChiSquareRejectFlag = 0u;

    if (QEKF_INS.AccelValidFlag == 0u)
    {
        QEKF_INS.ChiSquare = QEKF_INS.ChiSquareTestThreshold;
        QEKF_INS.AccelResidualNorm = 0.0f;
        QEKF_INS.ChiSquareGoodTime = 0.0f;
        QEKF_INS.ChiSquareBadTime += QEKF_INS.dt;
        QEKF_INS.ChiSquareRejectFlag = 1u;
        arm_copy_f32(kf->xhatminus_data, kf->xhat_data, kf->xhatSize);
        arm_copy_f32(kf->Pminus_data, kf->P_data, kf->xhatSize * kf->xhatSize);
        kf->SkipEq5 = TRUE;
        return;
    }

    // ------------------------------
    // 8. 计算观测残差 r = z - h(x^-)
    // ------------------------------
    // 这里先用预测四元数 q^- 计算理论重力方向 h(q^-)，
    // 再与单位化后的加速度方向 z 做差。
    //
    // 如果姿态估计准确、且线加速度较小，
    // 那么实测加速度方向应与预测重力方向接近。
    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    IMU_QuaternionEKF_SetMeasurementPrediction(kf->xhatminus_data, gravity_body);
    arm_copy_f32(gravity_body, kf->temp_vector_data, 3);

    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

    residual_sq = kf->temp_vector_data1[0] * kf->temp_vector_data1[0] +
                  kf->temp_vector_data1[1] * kf->temp_vector_data1[1] +
                  kf->temp_vector_data1[2] * kf->temp_vector_data1[2];
    // ------------------------------
    // 9. 加速度方向残差检验
    // ------------------------------
    // 这里使用单位重力方向残差平方 ||z - h(q^-)||^2 作为门控量。
    // 它不随 R 的调参而失效：R 仍然决定融合力度，残差门限只负责判断
    // 当前加速度方向是否可信。
    //
    // 若残差过大，常见原因包括：
    // 1. 机器人正在线性加速，加速度计测到的不再只是重力
    // 2. 碰撞、振动、冲击造成瞬时异常
    // 3. 传感器噪声或采样异常
    //
    // 对单位向量而言，残差平方约等于 4*sin(theta/2)^2，
    // 阈值 0.03 大约对应 10 度左右的方向不一致。
    QEKF_INS.ChiSquare = residual_sq;
    QEKF_INS.AccelResidualNorm = QEKF_Sqrtf(residual_sq);

    if (QEKF_INS.ChiSquare < k_qekf_chi_converge_ratio * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ChiSquareGoodTime += QEKF_INS.dt;
        QEKF_INS.ChiSquareBadTime = 0.0f;
        if (QEKF_INS.ChiSquareGoodTime >= k_qekf_chi_converge_time_s)
        {
            // 只有当残差连续足够小，才认为滤波器进入“已收敛”状态。
            // 在未收敛前，不宜太早激活严格拒绝逻辑，否则初期可能难以收敛。
            QEKF_INS.ConvergeFlag = 1;
        }
    }
    else
    {
        QEKF_INS.ChiSquareGoodTime = 0.0f;
        if (QEKF_INS.ChiSquare > QEKF_INS.ChiSquareTestThreshold)
        {
            QEKF_INS.ChiSquareBadTime += QEKF_INS.dt;
        }
        else
        {
            QEKF_INS.ChiSquareBadTime = 0.0f;
        }
    }

    if (QEKF_INS.ChiSquare > QEKF_INS.ChiSquareTestThreshold && enChiSquareTest && QEKF_INS.ConvergeFlag)
    {
        // 若已收敛但当前观测残差过大，则直接拒绝这次量测更新：
        //
        // x(k) = x^-(k)
        // P(k) = P^-(k)
        //
        // 这样可避免“非重力加速度”把 pitch/roll 瞬间拉歪。
        arm_copy_f32(kf->xhatminus_data, kf->xhat_data, kf->xhatSize);
        arm_copy_f32(kf->Pminus_data, kf->P_data, kf->xhatSize * kf->xhatSize);
        kf->SkipEq5 = TRUE;
        QEKF_INS.ChiSquareRejectFlag = 1u;
        return;
    }
    else
    {
        // 若本次观测可接受，则对 bias 的方差做遗忘因子放大：
        //
        // P_bias <- P_bias / lambda
        //
        // 因为 lambda < 1 时，除以 lambda 会让协方差略微增大，
        // 相当于告诉滤波器：
        // “零偏虽然已经估过了，但仍可能缓慢漂移，不要把它看得过于确定。”
        kf->P_data[28] /= QEKF_INS.lambda;
        kf->P_data[35] /= QEKF_INS.lambda;
        kf->SkipEq5 = FALSE;
    }

    // ------------------------------
    // 7. 计算创新协方差 S
    // ------------------------------
    // 残差门控先于 S^-1 计算，若本轮量测被拒绝，可省掉一次 3x3 矩阵求逆。
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);
    if (kf->MatStatus != ARM_MATH_SUCCESS)
    {
        arm_copy_f32(kf->xhatminus_data, kf->xhat_data, kf->xhatSize);
        arm_copy_f32(kf->Pminus_data, kf->P_data, kf->xhatSize * kf->xhatSize);
        kf->SkipEq5 = TRUE;
        QEKF_INS.ChiSquareRejectFlag = 1u;
        return;
    }

    // ------------------------------
    // 10. 计算卡尔曼增益 K
    // ------------------------------
    // 标准公式：
    // K = Pminus * H^T * S^-1
    //
    // K 的物理意义：
    // 当前该信多少观测，信多少预测。
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);
    // ------------------------------
    // 11. 更新状态
    // ------------------------------
    // 标准公式：
    // x = x^- + K * r
    //
    // 这里显式禁止量测更新直接改动 q3，
    // 这是该工程里的一个实现选择，保留原有行为一致性。
    kf->temp_vector.pData[3] = 0.0f;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
#if QEKF_OBSERVE_MATRIX_ENABLE
    arm_copy_f32(kf->P_data, IMU_QuaternionEKF_P, 36);
    arm_copy_f32(kf->K_data, IMU_QuaternionEKF_K, 18);
    arm_copy_f32(kf->H_data, IMU_QuaternionEKF_H, 18);
#else
    (void)kf;
#endif
}

static inline float QEKF_Absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static inline float QEKF_Clampf(float x, float min, float max)
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

static inline float QEKF_Sqrtf(float x)
{
    float result = 0.0f;

    if (QEKF_IsFinitef(x) && (x > 0.0f) && (arm_sqrt_f32(x, &result) == ARM_MATH_SUCCESS))
    {
        return result;
    }

    return 0.0f;
}

static inline uint8_t QEKF_IsFinitef(float x)
{
    return (uint8_t)((x == x) && (x <= 3.402823466e38f) && (x >= -3.402823466e38f));
}

#if QEKF_FAST_EULER_TRIG_ENABLE
static inline float QEKF_FastAtanUnit(float x)
{
    const float abs_x = QEKF_Absf(x);
    return (0.78539816339745f * x) -
           (x * (abs_x - 1.0f) * (0.2447f + 0.0663f * abs_x));
}

static inline float QEKF_FastAtan2(float y, float x)
{
    const float abs_y = QEKF_Absf(y);
    const float abs_x = QEKF_Absf(x);
    float angle;

    if ((abs_x <= 1.0e-12f) && (abs_y <= 1.0e-12f))
    {
        return 0.0f;
    }

    if (abs_x > abs_y)
    {
        angle = QEKF_FastAtanUnit(y / x);
        if (x < 0.0f)
        {
            angle += (y >= 0.0f) ? k_qekf_pi : -k_qekf_pi;
        }
    }
    else
    {
        angle = k_qekf_half_pi - QEKF_FastAtanUnit(x / y);
        if (y < 0.0f)
        {
            angle -= k_qekf_pi;
        }
    }

    return angle;
}

static inline float QEKF_FastAsin(float x)
{
    float cos_sq;

    x = QEKF_Clampf(x, -1.0f, 1.0f);
    cos_sq = 1.0f - x * x;
    return QEKF_FastAtan2(x, QEKF_Sqrtf(cos_sq));
}
#endif

static float IMU_QuaternionEKF_VectorInvNorm(const float *vec, uint32_t len)
{
    float power = 0.0f;
    float norm = 0.0f;

    if (len == 3u)
    {
        power = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
    }
    else if (len == 4u)
    {
        power = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2] + vec[3] * vec[3];
    }
    else
    {
        for (uint32_t i = 0u; i < len; i++)
        {
            power += vec[i] * vec[i];
        }
    }

    if (!QEKF_IsFinitef(power) || power <= 0.0f)
    {
        return 0.0f;
    }

    norm = QEKF_Sqrtf(power);
    if (!QEKF_IsFinitef(norm) || norm <= 0.0f)
    {
        return 0.0f;
    }

    return 1.0f / norm;
}

static uint8_t IMU_QuaternionEKF_NormalizeQuat(float *quat, const float *fallback)
{
    float inv_norm = IMU_QuaternionEKF_VectorInvNorm(quat, 4);

    if (QEKF_IsFinitef(inv_norm) && inv_norm > 0.0f)
    {
        quat[0] *= inv_norm;
        quat[1] *= inv_norm;
        quat[2] *= inv_norm;
        quat[3] *= inv_norm;
        return 1u;
    }

    if (fallback != NULL)
    {
        quat[0] = fallback[0];
        quat[1] = fallback[1];
        quat[2] = fallback[2];
        quat[3] = fallback[3];
        inv_norm = IMU_QuaternionEKF_VectorInvNorm(quat, 4);
        if (QEKF_IsFinitef(inv_norm) && inv_norm > 0.0f)
        {
            quat[0] *= inv_norm;
            quat[1] *= inv_norm;
            quat[2] *= inv_norm;
            quat[3] *= inv_norm;
            return 1u;
        }
    }

    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
    return 0u;
}

static void IMU_QuaternionEKF_SetMeasurementVector(KalmanFilter_t *kf, float ax, float ay, float az)
{
    float accel_vec[3] = {ax, ay, az};
    float accel_inv_norm = IMU_QuaternionEKF_VectorInvNorm(accel_vec, 3);

    if (accel_inv_norm <= 0.0f)
    {
        kf->MeasuredVector[0] = 0.0f;
        kf->MeasuredVector[1] = 0.0f;
        kf->MeasuredVector[2] = 0.0f;
        QEKF_INS.AccelValidFlag = 0u;
        return;
    }

    // z_k = a_k / ||a_k||
    kf->MeasuredVector[0] = ax * accel_inv_norm;
    kf->MeasuredVector[1] = ay * accel_inv_norm;
    kf->MeasuredVector[2] = az * accel_inv_norm;
    QEKF_INS.AccelValidFlag = 1u;
}

static void IMU_QuaternionEKF_SetProcessNoise(KalmanFilter_t *kf)
{
    // Q_k: 状态过程噪声，R_k: 量测噪声
    kf->Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    kf->Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    kf->Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    kf->Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    kf->Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    kf->Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    kf->R_data[0] = QEKF_INS.R;
    kf->R_data[4] = QEKF_INS.R;
    kf->R_data[8] = QEKF_INS.R;
}

static void IMU_QuaternionEKF_SetStateTransitionJacobian(KalmanFilter_t *kf, float gx, float gy, float gz, float dt)
{
    float halfgxdt = 0.5f * (gx - QEKF_INS.GyroBias[0]) * dt;
    float halfgydt = 0.5f * (gy - QEKF_INS.GyroBias[1]) * dt;
    float halfgzdt = 0.5f * (gz - QEKF_INS.GyroBias[2]) * dt;
    float *F = kf->F_data;

    // 预测模型：
    // q_(k+1|k) ≈ (I + 1/2 * Omega(ω-b) * dt) q_(k|k)
    // 这里写入的是 F_k = ∂f/∂x 在当前工作点的线性化结果。
    F[1] = -halfgxdt;
    F[2] = -halfgydt;
    F[3] = -halfgzdt;
    F[4] = 0.0f;
    F[5] = 0.0f;

    F[6] = halfgxdt;
    F[8] = halfgzdt;
    F[9] = -halfgydt;
    F[10] = 0.0f;
    F[11] = 0.0f;

    F[12] = halfgydt;
    F[13] = -halfgzdt;
    F[15] = halfgxdt;
    F[16] = 0.0f;
    F[17] = 0.0f;

    F[18] = halfgzdt;
    F[19] = halfgydt;
    F[20] = -halfgxdt;
    F[22] = 0.0f;
    F[23] = 0.0f;
}

static void IMU_QuaternionEKF_SetMeasurementPrediction(const float *state, float *gravity_body)
{
    const float q0 = state[0];
    const float q1 = state[1];
    const float q2 = state[2];
    const float q3 = state[3];

    // 观测模型：
    // z_k = h(x_k) + v_k
    // h(q) 表示由姿态四元数预测出的“机体系重力方向”。
    gravity_body[0] = 2.0f * (q1 * q3 - q0 * q2);
    gravity_body[1] = 2.0f * (q0 * q1 + q2 * q3);
    gravity_body[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}
