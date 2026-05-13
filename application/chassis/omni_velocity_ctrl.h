#ifndef OMNI_VELOCITY_CTRL_H
#define OMNI_VELOCITY_CTRL_H

#include <stdint.h>

typedef enum
{
    OMNI_WHEEL_LF = 0,
    OMNI_WHEEL_RF = 1,
    OMNI_WHEEL_RB = 2,
    OMNI_WHEEL_LB = 3,
    OMNI_WHEEL_NUM = 4,
} OmniWheelIndex_e;

typedef enum
{
    OMNI_AXIS_VX = 0,
    OMNI_AXIS_VY = 1,
    OMNI_AXIS_WZ = 2,
    OMNI_AXIS_NUM = 3,
} OmniAxisIndex_e;

typedef struct
{
    float wheel_base_mm;                    /* 前后轮中心距，单位 mm */
    float track_width_mm;                   /* 左右轮中心距，单位 mm */
    float control_dt_s;                     /* 底盘控制周期，单位 s */
    float wheel_dir[OMNI_WHEEL_NUM];        /* 四轮电机方向修正，标定时只改这里的正负号 */
    float speed_acc_limit;                  /* vx/vy 参考加速度限幅，沿用当前工程速度单位 */
    float wz_acc_limit;                     /* wz 参考加速度限幅，沿用当前工程角速度单位 */
    float kp[OMNI_AXIS_NUM];                /* 速度误差比例增益 */
    float ki[OMNI_AXIS_NUM];                /* 速度误差积分增益 */
    float integral_limit[OMNI_AXIS_NUM];    /* 三自由度积分限幅 */
    float mass[OMNI_AXIS_NUM];              /* 三自由度名义惯量，用于把目标加速度换成期望合力/力矩 */
    float damping[OMNI_AXIS_NUM];           /* 三自由度名义阻尼补偿 */
    float force_to_cmd_pos[OMNI_WHEEL_NUM]; /* 单轮正向力到 DJI 电流命令，单位 cmd/N */
    float force_to_cmd_neg[OMNI_WHEEL_NUM]; /* 单轮负向力到 DJI 电流命令，单位 cmd/N */
    float current_offset[OMNI_WHEEL_NUM];   /* 单轮静摩擦补偿，单位 cmd */
    float wheel_current_limit[OMNI_WHEEL_NUM]; /* 单轮电流硬限幅，单位 cmd */
    uint8_t eso_enable;                     /* ESO 使能标志，第一版默认关闭，标定稳定后再打开 */
    uint8_t eso_compensate_enable;          /* 1 表示 ESO 扰动估计参与控制补偿；0 仅观测 */
    float eso_compensate_gain[OMNI_AXIS_NUM]; /* ESO 扰动补偿比例，1.0 表示全量补偿 */
    float eso_bandwidth[OMNI_AXIS_NUM];     /* ESO 带宽，单位 rad/s */
    float eso_comp_static_deadband_v;       /* 静止时关闭 vx/vy ESO 补偿的速度死区，单位 m/s */
    float eso_comp_static_deadband_wz;      /* 静止时关闭 wz ESO 补偿的角速度死区，单位 rad/s */
    float allocator_regularization;         /* 轮力分配正则项，避免小矩阵病态 */
    float slip_residual_threshold;          /* 滑移残差归一化阈值 */
    float slip_lpf_alpha;                   /* 滑移概率低通系数 */
    uint8_t slip_force_enable;              /* 1 表示滑移概率参与轮力分配降权 */
    float slip_force_gain;                  /* 滑移轮力降权增益 */
    float slip_force_min_weight;            /* 滑移轮最低轮力分配权重 */
} OmniVelocityCtrlConfig_s;

typedef struct
{
    float vx_ref;                           /* 车体系前向速度指令 */
    float vy_ref;                           /* 车体系左向速度指令 */
    float wz_ref;                           /* 车体系逆时针角速度指令 */
    float wheel_speed[OMNI_WHEEL_NUM];      /* 四轮编码器速度反馈，轮序为 LF/RF/RB/LB */
    float wheel_current[OMNI_WHEEL_NUM];    /* 四轮电流反馈，当前第一版只用于后续扩展 */
    float body_omega_z;                     /* 底盘 IMU yaw gyro，单位 rad/s */
    uint8_t body_omega_z_valid;             /* 1 表示 body_omega_z 可用于 wz 闭环 */
    uint8_t current_offset_enable;          /* 1 表示启用静摩擦电流补偿 */
    uint8_t reserved[2];
    float current_limit;                    /* 本周期统一电流限幅，单位 cmd；<=0 时用 config 默认 */
    float current_rate_limit;               /* 本周期统一电流变化率限幅，单位 cmd/s；<=0 时不限制 */
    float temp_power_limit_w;               /* 临时底盘功率限幅，单位 W；<=0 时关闭 */
    float dt;                               /* 本周期实际时间；小于等于 0 时使用配置默认周期 */
} OmniVelocityCtrlInput_s;

typedef enum
{
    OMNI_LQI_WZ_FEEDBACK_ENCODER = 0,
    OMNI_LQI_WZ_FEEDBACK_IMU = 1,
} OmniLqiWzFeedbackSource_e;

typedef struct
{
    float wheel_speed_ref[OMNI_WHEEL_NUM];  /* 四轮速度环参考值 */
    float wheel_current_ref[OMNI_WHEEL_NUM];/* 四轮 DJI 电流命令，已乘 wheel_dir */
    float force_cmd[OMNI_WHEEL_NUM];        /* 四轮实际分配轮力，方向为 fit 坐标，单位 N */
    float nu_ref[OMNI_AXIS_NUM];            /* 限加速度后的三自由度参考速度 */
    float nu_hat[OMNI_AXIS_NUM];            /* 编码器反解得到的三自由度估计速度 */
    float lqi_ref[OMNI_AXIS_NUM];           /* LQI 使用的 SI 参考，[m/s, m/s, rad/s] */
    float lqi_feedback[OMNI_AXIS_NUM];      /* LQI 使用的 SI 反馈，[m/s, m/s, rad/s] */
    float lqi_error[OMNI_AXIS_NUM];         /* LQI SI 误差 */
    float lqi_integral[OMNI_AXIS_NUM];      /* LQI SI 积分状态 */
    float nu_eso[OMNI_AXIS_NUM];            /* ESO 速度估计，[m/s, m/s, rad/s] */
    float dist_hat[OMNI_AXIS_NUM];          /* ESO 扰动估计，[m/s^2, m/s^2, rad/s^2] */
    float wrench_des[OMNI_AXIS_NUM];        /* LQI/ESO 输出的期望底盘合力/力矩 */
    float wrench_act[OMNI_AXIS_NUM];        /* 限幅后按实际轮力反推的可达合力/力矩 */
    float slip_prob[OMNI_WHEEL_NUM];        /* 每个轮子的滑移概率记录 */
    uint8_t limit_active;                   /* 本周期是否触发轮力或电流限幅 */
    uint8_t current_limit_active;           /* 本周期是否触发电流幅值限幅 */
    uint8_t current_rate_limit_active;      /* 本周期是否触发电流变化率限幅 */
    uint8_t lqi_wz_feedback_source;         /* OmniLqiWzFeedbackSource_e */
} OmniVelocityCtrlOutput_s;

typedef struct
{
    OmniVelocityCtrlConfig_s config;
    float h[OMNI_WHEEL_NUM][OMNI_AXIS_NUM];
    float force_h[OMNI_WHEEL_NUM][OMNI_AXIS_NUM];
    float cmd_to_si[OMNI_AXIS_NUM];
    float nu_ref[OMNI_AXIS_NUM];
    float nu_ref_last[OMNI_AXIS_NUM];
    float nu_ref_dot[OMNI_AXIS_NUM];
    float nu_hat[OMNI_AXIS_NUM];
    float lqi_ref[OMNI_AXIS_NUM];
    float lqi_ref_dot[OMNI_AXIS_NUM];
    float lqi_feedback[OMNI_AXIS_NUM];
    float lqi_error[OMNI_AXIS_NUM];
    float nu_eso[OMNI_AXIS_NUM];
    float dist_hat[OMNI_AXIS_NUM];
    float integral[OMNI_AXIS_NUM];
    float force_prev[OMNI_WHEEL_NUM];
    float current_ref_fit_prev[OMNI_WHEEL_NUM];
    float wrench_des_prev[OMNI_AXIS_NUM];
    float wrench_act_prev[OMNI_AXIS_NUM];
    float slip_prob[OMNI_WHEEL_NUM];
    uint8_t lqi_wz_feedback_source;
    uint8_t inited;
} OmniVelocityCtrl_s;

void OmniVelocityCtrl_GetDefaultConfig(OmniVelocityCtrlConfig_s *config);
void OmniVelocityCtrl_Init(OmniVelocityCtrl_s *ctrl, const OmniVelocityCtrlConfig_s *config);
void OmniVelocityCtrl_Reset(OmniVelocityCtrl_s *ctrl);
void OmniVelocityCtrl_Step(OmniVelocityCtrl_s *ctrl,
                           const OmniVelocityCtrlInput_s *input,
                           OmniVelocityCtrlOutput_s *output);

#endif
