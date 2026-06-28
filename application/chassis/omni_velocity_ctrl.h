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
    float power_vxy_floor_ratio;            /* 功率不足时纯 vxy 最快可达速度的最低保留比例 */
    float power_budget_margin;              /* 前置功率分配使用的预算裕度，避免后置硬限幅频繁介入 */
    float power_wheel_speed_limit;           /* rotate 搜索用轮端速度参考上限，单位 deg/s */
    float power_feedback_deadband_w;        /* 真实功率反馈死区，单位 W */
    float power_feedback_scale_min;         /* 真实功率超限时允许压低到的最小预算比例 */
    float power_feedback_down_alpha;        /* 真实功率超限时预算比例下降滤波系数 */
    float power_feedback_up_alpha;          /* 真实功率未超限或无效时预算比例恢复滤波系数 */
    float power_feedback_lpf_tau_s;         /* supercap 真实功率低通时间常数，单位 s */
    float power_feedback_local_min_ratio;   /* 本地估算功率超过该比例的功率限制时才允许真实功率下压预算 */
} OmniVelocityCtrlConfig_s;

typedef struct
{
    float vx_ref;                           /* 车体系前向速度指令 */
    float vy_ref;                           /* 车体系左向速度指令 */
    float wz_ref;                           /* 车体系逆时针角速度指令 */
    float wheel_speed[OMNI_WHEEL_NUM];      /* 四轮编码器速度反馈，轮序为 LF/RF/RB/LB */
    float wheel_current[OMNI_WHEEL_NUM];    /* 四轮电流反馈，当前第一版只用于后续扩展 */
    float external_wrench_ff[OMNI_AXIS_NUM]; /* 外部合力/力矩前馈，[Fx, Fy_left, Tz]，单位 N/N/N*m */
    float body_omega_z;                     /* 底盘 IMU yaw gyro，单位 rad/s */
    uint8_t body_omega_z_valid;             /* 1 表示 body_omega_z 可用于 wz 闭环 */
    uint8_t current_offset_enable;          /* 1 表示启用静摩擦电流补偿 */
    uint8_t measured_power_valid;           /* 1 表示 measured_power_w 为 supercap 真实功率 */
    uint8_t speed_power_model_enable;       /* 1 表示用速度环预测电流和轮速参考估算功率 */
    uint8_t rotate_power_priority_enable;   /* 1 表示 rotate 下按 vxy 保留比例优先把剩余预算给 wz */
    uint8_t power_search_only_enable;       /* 1 表示只更新最大可达缓存，不改写 nu_ref */
    float current_limit;                    /* 本周期统一电流限幅，单位 cmd；<=0 时用 config 默认 */
    float current_rate_limit;               /* 本周期统一电流变化率限幅，单位 cmd/s；<=0 时不限制 */
    float temp_power_limit_w;               /* 最大可达速度搜索预算，单位 W；<=0 时关闭 */
    float rotate_wz_search_limit;           /* rotate 模式搜索 wz_max 的绝对上界，单位 cmd；<=0 时退回 wz_ref */
    float measured_power_w;                 /* supercap 反馈的真实底盘功率，单位 W */
    float wheel_to_motor_speed_ratio;        /* 轮端速度到电机轴速度的换算系数，用于速度环功率预测 */
    float speed_power_kp[OMNI_WHEEL_NUM];    /* 四轮速度环 Kp，用于预测本周期速度 PID 电流 */
    float speed_power_iout[OMNI_WHEEL_NUM];  /* 四轮速度环当前 Iout，用于预测本周期速度 PID 电流 */
    float speed_power_deadband[OMNI_WHEEL_NUM]; /* 四轮速度环死区，单位为电机轴速度 */
    float speed_power_output_limit[OMNI_WHEEL_NUM]; /* 四轮速度环输出限幅，单位 cmd */
    float dt;                               /* 本周期实际时间；小于等于 0 时使用配置默认周期 */
} OmniVelocityCtrlInput_s;

typedef enum
{
    OMNI_LQI_WZ_FEEDBACK_ENCODER = 0,
    OMNI_LQI_WZ_FEEDBACK_IMU = 1,
} OmniLqiWzFeedbackSource_e;

typedef struct
{
    uint8_t reachable_vxy_valid;            /* 1 表示 vxy 最大可达缓存有效 */
    uint8_t reachable_wz_valid;             /* 1 表示 wz 最大可达缓存有效 */
    uint8_t reserved[2];
    float reachable_vxy_max;                /* 当前方向纯平移最大可达速度幅值 */
    float reachable_wz_max;                 /* 纯自旋最大可达 wz 幅值 */
    float reachable_vxy_dir_x;              /* vxy 缓存方向，vx 单位方向 */
    float reachable_vxy_dir_y;              /* vxy 缓存方向，vy_left 单位方向 */
    float reachable_vxy_budget_w;           /* vxy 缓存对应的搜索预算 */
    float reachable_wz_budget_w;            /* wz 缓存对应的搜索预算 */
    float reachable_vxy_wheel_speed_limit;  /* vxy 缓存对应的轮速约束 */
    float reachable_wz_wheel_speed_limit;   /* wz 缓存对应的轮速约束 */
} OmniReachableCache_s;

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
    uint8_t power_limit_active;             /* 1 表示本周期触发 rotate 参考分配；最终硬限幅在 DJI 输出层 */
    uint8_t power_real_valid;               /* 1 表示本周期接入 supercap 真实功率反馈 */
    uint8_t reachable_vxy_valid;            /* 1 表示 reachable_vxy_max 可用于 rotate 分配 */
    uint8_t reachable_wz_valid;             /* 1 表示 reachable_wz_max 可用于 rotate 分配 */
    uint8_t rotate_strategy_active;         /* 1 表示本周期实际使用缓存最大可达速度做 rotate 分配 */
    uint8_t reachable_vxy_search_success;   /* 1 表示本周期实时搜索更新了 vxy_max */
    uint8_t reachable_wz_search_success;    /* 1 表示本周期实时搜索更新了 wz_max */
    float power_vxy_scale;                  /* rotate 搜索后的 vx/vy 参考缩放 */
    float power_wz_scale;                   /* rotate 搜索后的 wz 参考缩放 */
    float power_est_full_w;                 /* 未做功率缩放时的估算正机械功率，单位 W */
    float power_est_limited_w;              /* 功率缩放/硬限幅后的估算正机械功率，单位 W */
    float power_real_w;                     /* supercap 真实底盘功率，单位 W */
    float power_feedback_scale;             /* 真实功率反馈修正后的预算比例 */
    float power_effective_limit_w;          /* 实际用于本周期功率分配/限幅的功率预算，单位 W */
    float reachable_vxy_max;                /* 当前缓存方向的纯平移最大可达速度幅值 */
    float reachable_wz_max;                 /* 当前 rotate 纯自旋最大可达 wz 幅值 */
    float reachable_vxy_dir_x;              /* reachable_vxy_max 对应的 vx 单位方向 */
    float reachable_vxy_dir_y;              /* reachable_vxy_max 对应的 vy_left 单位方向 */
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
    float power_vxy_scale;
    float power_wz_scale;
    float power_est_full_w;
    float power_est_limited_w;
    float power_real_w;
    float power_real_lpf_w;
    float power_feedback_scale;
    float power_effective_limit_w;
    float reachable_vxy_max;
    float reachable_wz_max;
    float reachable_vxy_dir_x;
    float reachable_vxy_dir_y;
    float reachable_vxy_budget_w;
    float reachable_wz_budget_w;
    float reachable_vxy_wheel_speed_limit;
    float reachable_wz_wheel_speed_limit;
    uint8_t power_limit_active;
    uint8_t power_real_valid;
    uint8_t power_real_lpf_valid;
    uint8_t reachable_vxy_valid;
    uint8_t reachable_wz_valid;
    uint8_t rotate_strategy_active;
    uint8_t reachable_vxy_search_success;
    uint8_t reachable_wz_search_success;
    uint8_t lqi_wz_feedback_source;
    uint8_t inited;
} OmniVelocityCtrl_s;

void OmniVelocityCtrl_GetDefaultConfig(OmniVelocityCtrlConfig_s *config);
void OmniVelocityCtrl_Init(OmniVelocityCtrl_s *ctrl, const OmniVelocityCtrlConfig_s *config);
void OmniVelocityCtrl_Reset(OmniVelocityCtrl_s *ctrl);
void OmniVelocityCtrl_ClearReachableCache(OmniVelocityCtrl_s *ctrl);
void OmniVelocityCtrl_ExportReachableCache(const OmniVelocityCtrl_s *ctrl,
                                           OmniReachableCache_s *cache);
void OmniVelocityCtrl_ImportReachableCache(OmniVelocityCtrl_s *ctrl,
                                           const OmniReachableCache_s *cache);
void OmniVelocityCtrl_Step(OmniVelocityCtrl_s *ctrl,
                           const OmniVelocityCtrlInput_s *input,
                           OmniVelocityCtrlOutput_s *output);

#endif
