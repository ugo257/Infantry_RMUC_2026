/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H

#include "kalman_filter.h"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef struct
{
    // 通用卡尔曼滤波器实体，内部保存 EKF 的状态、协方差和临时矩阵。
    KalmanFilter_t IMU_QuaternionEKF;
    // 收敛标志。进入稳定状态后，卡方检验才会开始严格拒绝异常观测。
    uint8_t ConvergeFlag;
    // 本周期是否因加速度方向残差过大而拒绝量测更新。
    uint8_t ChiSquareRejectFlag;
    // 单位化加速度是否有效，防止异常零向量参与量测更新。
    uint8_t AccelValidFlag;
    uint8_t Reserved;

    // 当前姿态四元数估计结果 q = [w, x, y, z]，坐标语义遵循 REP-103。
    float q[4];
    // 当前估计的陀螺零偏。当前实现主要在线估计 x/y，z 轴通常由上层额外处理。
    float GyroBias[3];

    // 由四元数反解得到的欧拉角，单位为度。
    float Roll;
    float Pitch;
    float Yaw;

    // 将 yaw 展开成连续角后的结果，避免跨越 +/-180 度时出现跳变。
    float YawTotalAngle;
    int16_t YawRoundCount;
    float YawAngleLast;

    // EKF 调参量：
    // Q1 控制姿态状态过程噪声，Q2 控制零偏状态过程噪声，R 控制加速度观测噪声。
    float Q1;
    float Q2;
    float R;

    // 当前解算周期、卡方统计量、卡方阈值以及零偏协方差遗忘因子。
    float dt;
    float ChiSquare;
    float ChiSquareTestThreshold;
    float ChiSquareGoodTime;
    float ChiSquareBadTime;
    float AccelResidualNorm;
    float lambda;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern uint8_t enChiSquareTest;

// 初始化四元数 EKF。
// process_noise1: 姿态状态过程噪声
// process_noise2: 零偏状态过程噪声
// measure_noise : 加速度观测噪声
// lambda        : 零偏协方差遗忘因子，越接近 1 越保守
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda);

// 使用陀螺 + 加速度更新 EKF。
// gx/gy/gz 单位为 rad/s，ax/ay/az 单位通常为 m/s^2，dt 单位为 s。
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
