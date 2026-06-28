# Yaw 自瞄 LQR + ESO 模型与控制方案

本文按当前 checkout 的源码整理 yaw 自瞄控制器。核心实现位于：

- `application/cmd/robot_cmd.c`：视觉数据解包、双板 RS485 转发、`nuc_mode` 仲裁。
- `application/gimbal/gimbal.c`：构造 `YawControlInput_t`，调用控制器，输出 DM yaw 电机力矩。
- `modules/lqr_eso/lqr_eso.c` / `modules/lqr_eso/lqr_eso.h`：yaw LQR、ESO、参考轨迹和调试记录。
- `modules/motor/DMmotor/DMmotor.c`：DM MIT 扭矩帧打包与发送。

当前 `application/robot_board.h` 选择 `CHASSIS_BOARD`，因此 yaw 电机控制在底盘板编译运行；云台板侧把视觉和 IMU yaw 数据通过 RS485 发到底盘板。

## 1. 一句话结论

yaw 自瞄不是驱动层 PID，也不是 DM 内部位置环。它是 firmware 侧 1 kHz 左右运行的二阶 yaw 模型 LQR + 前馈 + ESO 扰动补偿，输出单位为 `N*m` 的 MIT 扭矩：

```text
视觉 PC: yaw_ref, yaw_ref_vel, yaw_ref_acc
    -> 云台板 RobotCMDVisionRxDecode()
    -> RS485: vision_yaw / vision_yaw_vel / vision_yaw_acc
    -> 底盘板 gimbal_cmd_send.yaw_ref_*
    -> GimbalYawBuildControlInput()
    -> YawLQRESO_Calc()
    -> DMMotorSetTorque(yaw_motor, tau_cmd_nm)
    -> DM MIT frame: pos=0, vel=0, kp=0, kd=0, tor=tau_cmd
```

自瞄生效条件：

- `gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE`
- `gimbal_cmd_recv.nuc_mode == version_control`
- 不是 `GIMBAL_HOLE_MODE`
- `DMMotorIsOnline(yaw_motor) != 0`

进入自瞄后，`gimbal_cmd_recv.yaw` 仍会作为函数入参传入，但实际 LQRESO 目标会被 `yaw_ref_rad / yaw_ref_vel_rad_s / yaw_ref_acc_rad_s2` 覆盖。

## 2. 数据链路

### 2.1 视觉包

云台板 `RobotCMDVisionRxDecode()` 接收 `VisionToGimbal`：

```c
typedef struct __attribute__((packed)) {
    uint8_t head[2];   // 'C', 'B'
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} VisionToGimbal;
```

CRC 通过后写入：

| 变量 | 单位 | 含义 |
|---|---:|---|
| `g_vision_rx_debug.last_mode` | - | 视觉模式/有效标志，0 时退出自瞄 |
| `g_vision_rx_debug.yaw` | rad | yaw 参考角 |
| `g_vision_rx_debug.yaw_vel` | rad/s | yaw 参考角速度 |
| `g_vision_rx_debug.yaw_acc` | rad/s^2 | yaw 参考角加速度 |

### 2.2 自瞄模式仲裁

`RobotCMDApplyControlInput()` 中：

```c
if (!remote_nuc_toggle_enable || g_vision_rx_debug.last_mode == 0u) {
    gimbal_cmd_send.nuc_mode = none_version_control;
} else {
    gimbal_cmd_send.nuc_mode = version_control;
}
```

云台板随后通过 RS485 把视觉三元组发到底盘板：

| RS485 字段 | 周期 tick | 目标频率约 | 单位 |
|---|---:|---:|---|
| `vision_yaw` | 7 | 约 143 Hz | rad |
| `vision_yaw_vel` | 7 | 约 143 Hz | rad/s |
| `vision_yaw_acc` | 7 | 约 143 Hz | rad/s^2 |
| `yaw_angle` | 1 | 约 1 kHz | deg，连续展开 yaw |
| `yaw_gyro` | 1 | 约 1 kHz | rad/s |
| `nuc_mode` | 13 | 约 77 Hz | enum |

底盘板 `RobotCMDTaskChassisBoard()` 再发布给 yaw 控制：

```c
gimbal_cmd_send.yaw_ref_rad = chassis_rs485_recv.vision_yaw;
gimbal_cmd_send.yaw_ref_vel_rad_s = chassis_rs485_recv.vision_yaw_vel;
gimbal_cmd_send.yaw_ref_acc_rad_s2 = chassis_rs485_recv.vision_yaw_acc;
gimbal_cmd_send.yaw_measure_angle = chassis_rs485_recv.yaw_angle;
gimbal_cmd_send.yaw_measure_gyro = chassis_rs485_recv.yaw_gyro;
```

### 2.3 控制器输入

`GimbalYawBuildControlInput()` 在自瞄下填充：

```c
input.theta_rad = gimbal_cmd_recv.yaw_measure_angle * DEG_TO_RAD;
input.omega_rad_s = gimbal_cmd_recv.yaw_measure_gyro;
input.tau_meas_nm = yaw_motor->measure.tor;
input.theta_ref_rad = gimbal_cmd_recv.yaw_ref_rad;
input.omega_ref_rad_s = gimbal_cmd_recv.yaw_ref_vel_rad_s;
input.alpha_ref_rad_s2 = gimbal_cmd_recv.yaw_ref_acc_rad_s2;
input.ref_traj_valid = 1U;
input.auto_aim_mode = 1U;
```

注意：当前 yaw 自瞄主反馈角速度是 RS485 传来的 IMU yaw gyro，即 `gimbal_cmd_recv.yaw_measure_gyro`，不是 `yaw_motor->measure.vel`。电机速度仅进入调试记录。

## 3. 被控对象模型

控制器使用二阶刚体加粘性阻尼模型：

```math
J \ddot{\theta} + B \dot{\theta} = \tau + d
```

状态空间形式：

```math
x = \begin{bmatrix}\theta \\ \omega\end{bmatrix}, \quad
\dot{x} =
\begin{bmatrix}
0 & 1 \\
0 & -B/J
\end{bmatrix}x
+
\begin{bmatrix}
0 \\
1/J
\end{bmatrix}\tau
+
\begin{bmatrix}
0 \\
d_a
\end{bmatrix}
```

其中：

- `theta`：连续 yaw 角，rad。
- `omega`：yaw 角速度，rad/s。
- `tau`：DM yaw 电机输出侧等效力矩，N*m。
- `d_a`：用角加速度维度表示的总扰动，rad/s^2。
- `J`、`B`：CSV 辨识得到的自瞄模型参数。

当前自瞄默认模型参数：

| 参数 | 当前默认值 | 单位 | 源码变量 |
|---|---:|---|---|
| `J` | `0.0638054955` | kg*m^2 | `g_yaw_auto_j` |
| `B` | `0.69288821` | N*m*s/rad | `g_yaw_auto_b` |
| Coulomb | `-0.00366569931` | N*m | `g_yaw_auto_tau_coulomb_nm` |
| Coulomb 平滑速度 | `0.150000006` | rad/s | `g_yaw_auto_coulomb_smooth_omega` |

非自瞄路径当前也默认复用 `YAW_AUTO_J_DEFAULT` 和 `YAW_AUTO_B_DEFAULT`，但增益和补偿开关仍按模式区分。

## 4. 参考轨迹

### 4.1 目标角包角

进入 `YawLQRESO_Calc()` 后：

```c
theta_target_rad = input->theta_ref_rad;
theta_target_rad = LQRESOWrapNearestRad(theta_target_rad, input->theta_rad);
```

也就是把视觉目标角调整到离当前实测角最近的一圈，避免 `+pi/-pi` 边界引入大阶跃。

如果 `g_ctrl_max_error_rad > 0`，还会把目标夹在当前角附近：

```text
theta_target_rad in [theta_meas - g_ctrl_max_error_rad,
                     theta_meas + g_ctrl_max_error_rad]
```

当前默认 `g_ctrl_max_error_rad = 0`，该保护关闭。

### 4.2 当前默认路径：直接使用视觉三元组

当前源码默认：

```c
#define TRAJ_PLANNER_ENABLE_DEFAULT 0U
```

因此自瞄 `ref_traj_valid != 0` 时默认走 `RefPlanner_UpdateTrajectory()`：

```c
theta_ref_planned_rad = theta_target_rad;
omega_ref_planned = input->omega_ref_rad_s;
alpha_ref_planned = input->alpha_ref_rad_s2;
```

这条路径不对 `omega_ref` 和 `alpha_ref` 做额外限幅；限幅主要发生在最终 DM 力矩 `[-7, 7] N*m`。

### 4.3 可选路径：STM32 侧外推参考轨迹

将 `g_traj_planner_enable` 置 1 后，会走 `RefPlanner_UpdateAutoAimTrajectory()`：

```text
新视觉包到达:
    latch theta, omega
    alpha 进入一阶 LPF
两包之间:
    theta = theta_latch + omega_latch * t + 0.5 * alpha_filt * t^2
    omega = omega_latch + alpha_filt * t
可选 lookahead:
    theta += omega * L + 0.5 * alpha * L^2
    omega += alpha * L
```

相关调参量：

| 变量 | 默认 | 说明 |
|---|---:|---|
| `g_traj_planner_enable` | `0U` | 0=直接覆盖，1=启用外推 |
| `g_traj_alpha_lpf_rc_s` | `0.0f` | alpha 一阶 LPF 时间常数 |
| `g_traj_lookahead_s` | `0.0f` | 输出端提前量 |
| `g_traj_change_eps_rad` | `1e-6f` | 新包变化判定阈值 |

该外推器用于把 100 到 200 Hz 级别的视觉包补成 1 kHz 连续参考，主要目标是减小自瞄曲线的阶梯化和相位滞后。

## 5. LQR 控制律

误差定义采用“测量减参考”：

```math
e_\theta = \theta_{meas} - \theta_{ref}
```

```math
e_\omega = \omega_{meas} - \omega_{ref}
```

角度误差先经过死区：

```c
e_theta = LQRESOApplyDeadband(e_theta_raw, g_ctrl_angle_deadband_rad);
```

当前默认 `g_ctrl_angle_deadband_rad = 0`。

### 5.1 前馈

自瞄前馈分三项：

```math
\tau_{ff} =
J \alpha_{ref}
+ B \omega_{ref}
+ \tau_c \tanh(\omega_{ref}/\omega_s)
```

对应源码变量：

| 项 | 源码变量 | 说明 |
|---|---|---|
| `J * alpha_ref` | `tau_ff_alpha` | 目标加速度前馈 |
| `B * omega_ref` | `tau_ff_viscous` | 粘性阻尼前馈 |
| `tau_c * tanh(...)` | `tau_ff_coulomb` | 库仑摩擦前馈 |

当前代码保留了 `g_yaw_auto_ff_alpha_tau_limit_nm` 和 `ff_alpha_limit_active` 字段，但自瞄路径没有对 `J * alpha_ref` 单项限幅。

### 5.2 反馈

反馈项：

```math
\tau_\theta = -K_\theta e_\theta
```

```math
\tau_\omega = -K_\omega e_\omega
```

```math
\tau_{lqi} = -K_i \int e_\theta dt
```

当前自瞄默认增益：

| 参数 | 默认值 | 源码变量 |
|---|---:|---|
| `K_theta` | `50.0` | `g_lqr_auto_k_theta` |
| `K_omega` | `2.11079573` | `g_lqr_auto_k_omega` |
| `K_i` | `0.0` | `g_lqi_ki` |
| 积分限幅 | `0.10` | `g_lqi_int_limit` |
| 积分开启角误差 | `0.0872665 rad` | `g_lqi_integral_enable_rad` |

当前默认 `g_lqi_ki = 0`，所以 LQI 积分项默认关闭。

### 5.3 LQR 合成

```math
\tau_{lqr} =
\tau_{ff}
+ \tau_\theta
+ \tau_\omega
+ \tau_{lqi}
```

源码中对应：

```c
tau_lqr = tau_ff + tau_theta + tau_omega + tau_lqi;
```

## 6. ESO 扩张状态观测器

ESO 使用三阶扩张状态：

| 状态 | 单位 | 含义 |
|---|---|---|
| `z1` | rad | yaw 角估计 |
| `z2` | rad/s | yaw 角速度估计 |
| `z3` | rad/s^2 | 总扰动的角加速度等效量 |

离散欧拉更新：

```math
e = \theta_{meas} - z_1
```

```math
\dot{z}_1 = z_2 + \beta_1 e
```

```math
\dot{z}_2 = -\frac{B}{J}z_2 + \frac{1}{J}u_{last} + z_3 + \beta_2 e
```

```math
\dot{z}_3 = \beta_3 e
```

其中：

```math
b_0 = 1/J
```

ESO 带宽由当前模型和反馈增益实时计算：

```math
\omega_{0,k} = \sqrt{b_0 |K_\theta|}
```

```math
\omega_{0,d} = 0.5 b_0 |B + K_\omega|
```

```math
\omega_0 = \alpha \max(\omega_{0,k}, \omega_{0,d})
```

```math
\beta_1 = 3\omega_0,\quad
\beta_2 = 3\omega_0^2,\quad
\beta_3 = \omega_0^3
```

自瞄可用 `g_yaw_auto_eso_w0_set > 0` 强制覆盖带宽。否则默认跟随上述公式。

相关默认值：

| 变量 | 默认 | 说明 |
|---|---:|---|
| `g_yaw_eso_alpha` | `1.10` | ESO 带宽缩放 |
| `g_yaw_auto_eso_w0_set` | `0.0` | 自瞄 ESO 带宽覆盖，0=自动计算 |
| `g_eso_torque_feedback_blend` | `0.0` | ESO 输入在上一拍命令和实测力矩之间混合 |

ESO 输入力矩：

```math
u_{eso} = (1 - blend)u_{last} + blend \tau_{meas,filt}
```

当前默认 `blend = 0`，ESO 用上一拍控制命令作为输入。

### 6.1 ESO 扰动补偿

ESO 补偿把 `z3` 从角加速度量纲转成力矩：

```math
\tau_{eso,raw} = -g_{eso}\frac{z_3}{b_0} = -g_{eso}Jz_3
```

自瞄默认：

| 变量 | 默认 | 说明 |
|---|---:|---|
| `g_yaw_auto_eso_comp_enable` | `1U` | 自瞄 ESO 补偿开关 |
| `g_eso_comp_gain` | `1.0` | ESO 补偿增益 |
| `g_eso_comp_gain_max` | `2.0` | 补偿增益上限 |
| `g_eso_comp_limit` | `7.0 N*m` | 当前只在非自瞄路径对 `tau_comp` 限幅 |

当前代码中 `g_yaw_auto_eso_comp_omega_gate_rad_s` 和 `g_yaw_auto_eso_comp_alpha_gate_rad_s2` 已定义为可观测变量，但没有参与自瞄 ESO 补偿门控逻辑。

## 7. 力矩输出与保护

自瞄主路径合成：

```math
\tau_{cmd} =
\tau_{lqr}
+ \tau_{eso,active}
+ \tau_{excitation}
+ \tau_{bias}
```

其中：

- `tau_excitation` 只在模型辨识运行时注入。
- `tau_bias` 由力矩跟踪误差积分得到，当前 `g_tau_bias_ki = 0`，默认关闭。
- 非自瞄才使用 `g_ctrl_t_limit` 软限幅和 torque slew。
- 自瞄最终仍受 DM 电机协议力矩范围限制。

DM yaw 电机输出限制：

```text
DM_T_MIN = -7.0 N*m
DM_T_MAX =  7.0 N*m
```

`DMMotorSetTorque()` 会发送 MIT 扭矩帧：

```c
pos_set = 0;
vel_set = 0;
kp_set  = 0;
kd_set  = 0;
tor_set = clamp(tau_cmd, +/- T_max);
```

当前 `g_ctrl_auto_torque_slew_rate = 7000.0` 会被填入 `YawControlInput_t`，但 `YawLQRESO_Calc()` 中 torque slew 的执行条件包含 `input->auto_aim_mode == 0U`，所以自瞄路径实际不启用 slew limiter。

## 8. 初始化与复位

`YawLQRESO_Init()`：

- `memset(ctrl, 0, sizeof(*ctrl))`
- 初始化 ESO 默认增益
- 发布 `g_yaw_eso_*` 观测变量

首次收到有效 yaw 电机反馈时：

- `feedback_ready = 1`
- `tau_meas_filt = tau_meas_nm`
- 参考角初始化为当前 yaw 实测角
- ESO `z1 = theta_meas`，`z2 = omega_meas`，`z3 = 0`
- 本周期输出 `0 N*m`

失去反馈或进入 `GIMBAL_ZERO_FORCE` 时：

- `YawLQRESO_Reset()`
- 清 `feedback_ready`
- 清 ESO、轨迹、积分、上一拍扭矩、调试有效标志
- 若模型辨识正在运行，则中止辨识

## 9. 模型辨识注入

当前代码提供 yaw 模型辨识用的 chirp 激励，入口是 `g_yaw_model_ident.arm`。

辨识运行时：

- `YawModelIdentApplyKOverride()` 可把 `K_theta/K_omega` 切到辨识用小增益。
- `YawExcitationCalc()` 在 LQR 输出上叠加小幅扫频力矩。
- `model_ident_active != 0` 时 ESO 继续观测，但不把 ESO 补偿注入输出。
- 调试记录写入 `g_yaw_model_ident`。

默认有两个辨识阶段：

| 阶段 | 默认时长 | 幅值 | 频率范围 | 说明 |
|---|---:|---:|---:|---|
| low speed | `10 s` | `0.45 N*m` | `0.08 -> 0.80 Hz` | 低速摩擦/阻尼观察 |
| dynamic chirp | `8 s` | `0.8 N*m` | `0.5 -> 15 Hz` | 动态模型辨识 |

辨识数据可配合 `tools/yaw_model_ident_fit.py`、`tools/yaw_lqreso_fit.py` 等离线脚本更新 `J/B/Coulomb/K`。

## 10. 主要 Live Watch 变量

### 10.1 输入与参考

| 变量 | 说明 |
|---|---|
| `g_yaw_auto_aim_debug_record.input_theta_ref_rad` | 视觉输入 yaw 参考角 |
| `g_yaw_auto_aim_debug_record.input_omega_ref_rad_s` | 视觉输入 yaw 参考速度 |
| `g_yaw_auto_aim_debug_record.input_alpha_ref_rad_s2` | 视觉输入 yaw 参考加速度 |
| `g_yaw_auto_aim_debug_record.theta_ref_planned_rad` | 控制器实际使用的参考角 |
| `g_yaw_auto_aim_debug_record.omega_ref_planned_rad_s` | 控制器实际使用的参考速度 |
| `g_yaw_auto_aim_debug_record.alpha_ref_planned_rad_s2` | 控制器实际使用的参考加速度 |
| `g_theta_ref_planned` | planned yaw，单位 deg |

### 10.2 误差与力矩分解

| 变量 | 说明 |
|---|---|
| `g_yaw_auto_aim_debug_record.e_theta_rad` | yaw 角误差 |
| `g_yaw_auto_aim_debug_record.e_omega_rad_s` | yaw 速度误差 |
| `g_yaw_auto_aim_debug_record.tau_ff_alpha_nm` | 加速度前馈 |
| `g_yaw_auto_aim_debug_record.tau_ff_viscous_nm` | 粘性前馈 |
| `g_yaw_auto_aim_debug_record.tau_ff_coulomb_nm` | Coulomb 前馈 |
| `g_yaw_auto_aim_debug_record.tau_theta_nm` | 角度反馈力矩 |
| `g_yaw_auto_aim_debug_record.tau_omega_nm` | 速度反馈力矩 |
| `g_yaw_auto_aim_debug_record.tau_lqr_nm` | LQR 合成力矩 |
| `g_yaw_auto_aim_debug_record.tau_eso_comp_nm` | ESO 实际注入力矩 |
| `g_yaw_auto_aim_debug_record.tau_cmd_nm` | 最终命令力矩 |

### 10.3 ESO 与限幅

| 变量 | 说明 |
|---|---|
| `g_yaw_eso_w0` | 当前 ESO 带宽 |
| `g_yaw_eso_z1` | ESO 角估计 |
| `g_yaw_eso_z2` | ESO 速度估计 |
| `g_yaw_eso_z3` | ESO 总扰动估计 |
| `g_yaw_auto_aim_debug_record.dm_limit_active` | DM `+-7 N*m` 限幅触发 |
| `g_yaw_auto_aim_debug_record.eso_comp_active` | ESO 补偿是否注入 |
| `g_yaw_auto_aim_debug_record.valid` | 当前自瞄调试记录是否有效 |
| `g_yaw_auto_aim_debug_record.feedback_ok` | yaw 电机反馈是否在线 |

## 11. 调参顺序建议

1. 先确认数据链路：`valid=1`、`feedback_ok=1`、`nuc_mode=version_control`，并确认 `theta_ref_planned_rad` 与视觉目标一致。
2. 检查符号：目标角增大时，若 `theta_meas < theta_ref`，则 `e_theta < 0`，`tau_theta` 应为正。
3. 先只看 LQR：必要时临时关 `g_yaw_auto_eso_comp_enable`，确认 `tau_ff/tau_theta/tau_omega` 分解合理。
4. 若相位滞后明显，优先尝试 `g_traj_planner_enable=1`，再小步调整 `g_traj_lookahead_s`，例如 `0.010 -> 0.015 -> 0.020`。
5. 若高频抖动明显，降低 `g_traj_lookahead_s`，或给 `g_traj_alpha_lpf_rc_s` 一个小时间常数。
6. 若经常 `dm_limit_active=1`，不要继续加 `K_theta` 或 `alpha` 前馈，先看 `tau_ff_alpha/tau_theta/tau_omega` 哪一项吃掉力矩余量。
7. 若开启 ESO 后低频稳态改善但抖动变大，降低 `g_eso_comp_gain` 或 `g_yaw_eso_alpha`。
8. Coulomb 前馈当前默认接近 0；历史 CSV 分析显示较大的 Coulomb 常值补偿会增加力矩 RMS 和饱和比例，建议只小范围试 `0.03`、`0.05 N*m`。

## 12. 当前实现中容易误解的点

- `gimbal_cmd_recv.yaw` 在自瞄模式下不是 LQRESO 的主目标，主目标是 `yaw_ref_rad` 三元组。
- `yaw_motor->measure.vel` 当前不参与自瞄主反馈，主速度反馈是 IMU gyro。
- `g_ctrl_auto_torque_slew_rate` 已传入 input，但自瞄路径没有执行 slew limiter。
- `g_yaw_auto_ff_alpha_tau_limit_nm`、`ff_alpha_limit_active` 当前没有实际限制 `J * alpha_ref`。
- `g_yaw_auto_eso_comp_omega_gate_rad_s`、`g_yaw_auto_eso_comp_alpha_gate_rad_s2` 当前没有参与 ESO 补偿门控。
- 自瞄 `RefPlanner_UpdateTrajectory()` 默认直接覆盖三元组，不对视觉给定 `omega/alpha` 做额外限幅。
