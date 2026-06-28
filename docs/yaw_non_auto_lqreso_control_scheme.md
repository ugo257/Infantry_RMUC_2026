# Yaw 非自瞄 LQR 控制方案

本文按当前 checkout 的源码整理 yaw 非自瞄控制器，规格尽量对齐 `docs/yaw_auto_aim_lqreso_control_scheme.md`。核心实现位于：

- `application/cmd/robot_cmd.c`：遥控器/键鼠目标生成、`nuc_mode` 仲裁、双板 RS485 转发。
- `application/gimbal/gimbal.c`：构造 `YawControlInput_t`，调用 yaw 控制器，输出 DM yaw 电机扭矩。
- `modules/lqr_eso/lqr_eso.c` / `modules/lqr_eso/lqr_eso.h`：yaw LQR、参考规划、ESO 观测和调试记录。
- `modules/motor/DMmotor/DMmotor.c`：DM MIT 扭矩帧打包与发送。

当前 `application/robot_board.h` 选择 `CHASSIS_BOARD`，因此 yaw 电机控制在底盘板运行；云台板侧把 `yaw_control`、IMU yaw 角和 yaw gyro 通过 RS485 发到底盘板。

## 1. 一句话结论

yaw 非自瞄普通手瞄模式不是驱动层 PID，也不是 DM 内部位置环。它仍然使用 firmware 侧 `YawLQRESO_Calc()` 计算 MIT 扭矩，但普通非自瞄路径只把 LQR/前馈输出接到电机，ESO 只作为观测器和调试量运行，不参与普通手瞄力矩输出：

```text
遥控器 / 鼠标
    -> yaw_control 连续目标角，deg
    -> 云台板 RS485: yaw_control + yaw_angle + yaw_gyro
    -> 底盘板 gimbal_cmd_send.yaw / yaw_measure_*
    -> GimbalYawBuildControlInput(ref_deg)
    -> YawLQRESO_Calc(auto_aim_mode=0, ref_traj_valid=0)
    -> DMMotorSetTorque(yaw_motor, tau_cmd_nm)
    -> DM MIT frame: pos=0, vel=0, kp=0, kd=0, tor=tau_cmd
```

本文默认“普通非自瞄”指：

- `gimbal_mode == GIMBAL_GYRO_MODE` 或 `GIMBAL_MOTOR_MODE`
- `nuc_mode != version_control`
- `hole_offset_mode == 0`

`GIMBAL_HOLE_MODE` 也是 `auto_aim_mode=0`，但它使用狗洞专用增益，并且强制接入 ESO 补偿；本文把它作为例外单独标出。

## 2. 数据链路

### 2.1 目标角来源

遥控器 N 档普通控制在 `RemoteControlSet()` 内生成 yaw 目标：

```c
yaw_control -= YAW_K * (float)rc_data[TEMP].rc.rocker_r_;
```

键鼠 C 档普通控制在 `GimbalSet()` 内生成 yaw 目标：

```c
if (gimbal_cmd_send.nuc_mode == none_version_control) {
    yaw_control -= rc_data[TEMP].mouse.x / 1000.0f;
}
```

进入普通非自瞄时，`RobotCMDApplyControlInput()` 会把目标锁到当前 IMU yaw，避免切换瞬间跳变：

```c
if (nuc_mode_last != gimbal_cmd_send.nuc_mode) {
    yaw_control = imu_yaw_deg;
}
gimbal_cmd_send.yaw = yaw_control;
```

目标角单位是 `deg`，并通过 `YawControlProcess()` 做过圈处理：

```c
if (yaw_control - yaw_measure_deg > 180.0f) {
    yaw_control -= 360.0f;
} else if (yaw_control - yaw_measure_deg < -180.0f) {
    yaw_control += 360.0f;
}
```

因此 `yaw_control` 会保持在离当前连续 yaw 最近的一圈。

### 2.2 双板转发

云台板通过 RS485 发到底盘板的关键字段：

| RS485 字段 | 周期 tick | 目标频率约 | 单位 | 说明 |
|---|---:|---:|---|---|
| `yaw_control` | 10 | 约 100 Hz | deg | 非自瞄目标角 |
| `yaw_angle` | 1 | 约 1 kHz | deg | 连续展开 yaw 实测角 |
| `yaw_gyro` | 1 | 约 1 kHz | rad/s | IMU yaw 角速度 |
| `gimbal_mode` | 13 | 约 77 Hz | enum | yaw 模式 |
| `nuc_mode` | 13 | 约 77 Hz | enum | 自瞄/非自瞄 |

底盘板 `RobotCMDTaskChassisBoard()` 再发布给 gimbal 应用：

```c
gimbal_cmd_send.yaw = chassis_rs485_recv.yaw_control;
gimbal_cmd_send.yaw_measure_angle = chassis_rs485_recv.yaw_angle;
gimbal_cmd_send.yaw_measure_gyro = chassis_rs485_recv.yaw_gyro;
gimbal_cmd_send.nuc_mode = chassis_rs485_recv.nuc_mode;
```

### 2.3 控制器输入

普通非自瞄时，`GimbalYawBuildControlInput()` 填充：

```c
input.theta_ref_deg = ref_deg;
input.theta_ref_rad = ref_deg * DEG_TO_RAD;
input.omega_ref_rad_s = 0.0f;
input.alpha_ref_rad_s2 = 0.0f;
input.ref_traj_valid = 0U;
input.auto_aim_mode = 0U;
input.hole_offset_mode = 0U;
input.torque_slew_enable = 1U;
input.torque_slew_rate_nm_s = g_ctrl_torque_slew_rate;

input.theta_rad = gimbal_cmd_recv.yaw_measure_angle * DEG_TO_RAD;
input.omega_rad_s = gimbal_cmd_recv.yaw_measure_gyro;
input.tau_meas_nm = yaw_motor->measure.tor;
input.feedback_ok = DMMotorIsOnline(yaw_motor);
```

注意：普通非自瞄的 yaw 角速度反馈同样来自 IMU yaw gyro，即 `gimbal_cmd_recv.yaw_measure_gyro`，不是 `yaw_motor->measure.vel`。电机速度主要进入调试记录。

## 3. 被控对象模型

普通非自瞄和自瞄共用二阶 yaw 模型：

```math
J \ddot{\theta} + B \dot{\theta} = \tau + d
```

状态形式：

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

当前非自瞄默认模型宏被同步到自瞄模型：

```c
#define YAW_J_DEFAULT YAW_AUTO_J_DEFAULT
#define YAW_B_DEFAULT YAW_AUTO_B_DEFAULT
#define YAW_TAU_COULOMB_DEFAULT YAW_AUTO_TAU_COULOMB_DEFAULT
#define YAW_COULOMB_SMOOTH_OMEGA_DEFAULT YAW_AUTO_COULOMB_SMOOTH_OMEGA_DEFAULT
```

当前默认参数：

| 参数 | 当前默认值 | 单位 | 源码变量 |
|---|---:|---|---|
| `J` | `0.0638054955` | kg*m^2 | `YAW_J_DEFAULT` / `g_yaw_auto_j` |
| `B` | `0.69288821` | N*m*s/rad | `YAW_B_DEFAULT` / `g_yaw_auto_b` |
| Coulomb | `-0.00366569931` | N*m | `g_yaw_tau_coulomb_nm` |
| Coulomb 平滑速度 | `0.150000006` | rad/s | `g_yaw_coulomb_smooth_omega` |

虽然默认值同步，自瞄和非自瞄仍使用不同入口变量：普通非自瞄通过 `g_lqr_k_theta/g_lqr_k_omega`，自瞄通过 `g_lqr_auto_k_theta/g_lqr_auto_k_omega`。

## 4. 参考轨迹

### 4.1 目标角包角

非自瞄 `ref_traj_valid=0`，所以 `YawLQRESO_Calc()` 使用角度命令：

```c
theta_target_rad = input->theta_ref_deg * DEG_TO_RAD;
theta_target_rad = LQRESOWrapNearestRad(theta_target_rad, input->theta_rad);
```

这一步把目标角调整到离当前实测角最近的一圈，和云台板侧 `YawControlProcess()` 形成双保险。

如果 `g_ctrl_max_error_rad > 0`，目标角还会被夹在当前角附近；当前默认 `0.0f`，该限制关闭。

### 4.2 非自瞄参考规划器

普通非自瞄走 `RefPlanner_Update()`：

```c
err = theta_target_rad - old_theta_ref;
theta_ref_planned_rad = theta_target_rad;

if (abs(err) < 1e-6f) {
    omega_ref_planned = 0.0f;
    alpha_ref_planned = 0.0f;
} else {
    omega_ref_planned = clamp(err / dt, +/- g_ref_omega_limit_rad_s);
    alpha_ref_planned = 0.0f;
}
```

也就是说：

- 参考角 `theta_ref_planned_rad` 直接更新到最新目标。
- 参考速度 `omega_ref_planned` 由目标角变化量除以本周期 `dt` 得到，并受 `g_ref_omega_limit_rad_s` 限制。
- 参考加速度 `alpha_ref_planned` 固定为 0。
- 当前没有对非自瞄目标角做斜坡积分式平滑；`g_ref_omega_limit_rad_s` 只限制用于前馈和速度误差的参考速度。

相关默认值：

| 变量 | 默认 | 说明 |
|---|---:|---|
| `g_ref_omega_limit_rad_s` | `8.0` | 非自瞄参考速度估计限幅 |
| `g_ctrl_max_error_rad` | `0.0` | 目标角最大误差限幅，0=关闭 |
| `g_ctrl_angle_deadband_rad` | `0.0` | 角误差死区 |

## 5. LQR 控制律

误差定义同自瞄，采用“测量减参考”：

```math
e_\theta = \theta_{meas} - \theta_{ref}
```

```math
e_\omega = \omega_{meas} - \omega_{ref}
```

角度误差经过死区：

```c
e_theta = LQRESOApplyDeadband(e_theta_raw, g_ctrl_angle_deadband_rad);
```

### 5.1 前馈

非自瞄仍使用同一套前馈公式：

```math
\tau_{ff} =
J \alpha_{ref}
+ B \omega_{ref}
+ \tau_c \tanh(\omega_{ref}/\omega_s)
```

但由于普通非自瞄 `alpha_ref = 0`，实际主要是：

```math
\tau_{ff} =
B \omega_{ref}
+ \tau_c \tanh(\omega_{ref}/\omega_s)
```

对应变量：

| 项 | 非自瞄含义 |
|---|---|
| `tau_ff_alpha` | 通常为 0，因为 `alpha_ref_planned=0` |
| `tau_ff_viscous` | 由目标角变化估算出的 `omega_ref` 产生 |
| `tau_ff_coulomb` | 按 `omega_ref` 符号和平滑速度产生 |

### 5.2 反馈

普通非自瞄反馈：

```math
\tau_\theta = -K_\theta e_\theta
```

```math
\tau_\omega = -K_\omega e_\omega
```

```math
\tau_{lqi} = -K_i \int e_\theta dt
```

当前普通非自瞄默认增益跟随自瞄默认值：

| 参数 | 默认值 | 源码变量 |
|---|---:|---|
| `K_theta` | `50.0` | `g_lqr_k_theta` |
| `K_omega` | `2.11079573` | `g_lqr_k_omega` |
| `K_i` | `0.0` | `g_lqi_ki` |
| 积分限幅 | `0.10` | `g_lqi_int_limit` |
| 积分开启角误差 | `0.0872665 rad` | `g_lqi_integral_enable_rad` |

当前默认 `g_lqi_ki = 0`，所以积分项默认关闭。

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

## 6. ESO 观测器与非自瞄补偿状态

非自瞄每周期仍会更新 ESO：

```c
YawEso_Update(ctrl, input->theta_rad, tau_for_eso, dt, yaw_j_sel, yaw_b_sel);
```

ESO 状态：

| 状态 | 单位 | 含义 |
|---|---|---|
| `z1` | rad | yaw 角估计 |
| `z2` | rad/s | yaw 速度估计 |
| `z3` | rad/s^2 | 总扰动估计 |

更新模型：

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

带宽计算：

```math
b_0 = 1/J
```

```math
\omega_0 =
\begin{cases}
g\_yaw\_eso\_w0\_set, & g\_yaw\_eso\_w0\_set > 0 \\
g\_yaw\_eso\_alpha \max(\sqrt{b_0 |K_\theta|}, 0.5b_0|B+K_\omega|), & otherwise
\end{cases}
```

```math
\beta_1 = 3\omega_0,\quad
\beta_2 = 3\omega_0^2,\quad
\beta_3 = \omega_0^3
```

当前关键默认值：

| 变量 | 默认 | 说明 |
|---|---:|---|
| `g_yaw_eso_alpha` | `1.10` | ESO 带宽缩放 |
| `g_yaw_eso_w0_set` | `0.0` | 非自瞄 ESO 带宽覆盖，0=自动计算 |
| `g_eso_torque_feedback_blend` | `0.0` | ESO 输入在上一拍命令和实测力矩之间混合 |

### 6.1 普通非自瞄：ESO 只观测，不补偿

虽然会计算：

```math
\tau_{eso,raw} = -g_{eso} \frac{z_3}{b_0}
```

并对非自瞄路径做 `g_eso_comp_limit` 限幅，但普通非自瞄随后会明确关掉补偿注入：

```c
} else if (input->auto_aim_mode == 0U) {
    eso_comp_active = 0U;
}
tau_eso_active = (eso_comp_active != 0U) ? tau_comp : 0.0f;
```

所以普通手瞄/电机 yaw 的实际输出中：

```math
\tau_{eso,active} = 0
```

`g_yaw_eso_z1/z2/z3`、`g_yaw_eso_tau_comp` 等仍可用于观察，但 `g_yaw_eso_tau_comp` 在普通非自瞄下应为 0。

### 6.2 狗洞模式例外

`GIMBAL_HOLE_MODE` 会令：

```c
input->hole_offset_mode = 1U;
```

此时：

- 增益切换为 `g_lqr_hole_offset_k_theta / g_lqr_hole_offset_k_omega`。
- `eso_comp_active = 1U`，强制把 ESO 补偿注入输出。
- 如果 `hole_offset_yaw_takeover != 0`，目标角由 `GimbalYawSelectHoleTakeoverRefDeg()` 按 yaw 电机单圈目标重新生成。

狗洞模式虽然不是自瞄，但控制行为不等同于普通手瞄。

## 7. 力矩输出与保护

普通非自瞄主路径：

```math
\tau_{cmd} =
\tau_{lqr}
+ \tau_{bias}
```

普通非自瞄默认不含 `tau_eso_active`。如果模型辨识运行，还会叠加 `tau_excitation`。

### 7.1 软限幅

普通非自瞄会使用 `g_ctrl_t_limit` 做 soft limit：

```c
if (input->auto_aim_mode == 0U && soft_limit > 0.0f) {
    tau_cmd = clamp(tau_cmd, +/- g_ctrl_t_limit);
}
```

当前默认：

| 变量 | 默认 | 说明 |
|---|---:|---|
| `g_ctrl_t_limit` | `7.0 N*m` | firmware 侧软限幅 |
| `DM_T_MIN/MAX` | `+-7.0 N*m` | DM 协议最终限幅 |

### 7.2 bias 慢补偿

如果没有限幅，控制器会计算：

```math
\tau_{track\_err} = \tau_{cmd} - \tau_{meas,filt}
```

```math
\tau_{bias} += g_{\tau bias,ki} \tau_{track\_err} dt
```

当前默认：

| 变量 | 默认 |
|---|---:|
| `g_tau_bias_ki` | `0.0` |
| `g_tau_bias_limit` | `0.10 N*m` |
| `g_tau_meas_lpf_alpha` | `0.10` |

因此 bias 慢补偿默认关闭。

### 7.3 torque slew

普通非自瞄会启用 torque slew：

```c
max_delta = g_ctrl_torque_slew_rate * dt;
tau_cmd = clamp(tau_cmd,
                torque_cmd_last - max_delta,
                torque_cmd_last + max_delta);
```

当前默认：

| 变量 | 默认 | 说明 |
|---|---:|---|
| `g_ctrl_torque_slew_rate` | `1000.0 N*m/s` | 普通非自瞄力矩斜率限制 |

在 1 ms 周期下，默认每周期最大变化约 `1 N*m`。

### 7.4 最小力矩补偿

普通非自瞄低速且有角误差时，如果命令力矩太小，会补到最小值：

```c
if (abs(omega_meas) < g_ctrl_vel_deadband &&
    abs(e_theta_raw) > g_ctrl_angle_deadband_rad &&
    abs(tau_cmd) < g_ctrl_min_torque_comp) {
    tau_cmd = -sign(e_theta_raw) * g_ctrl_min_torque_comp;
}
```

当前默认：

| 变量 | 默认 |
|---|---:|
| `g_ctrl_vel_deadband` | `0.03 rad/s` |
| `g_ctrl_min_torque_comp` | `0.40 N*m` |

### 7.5 DM 输出

最终 `DMMotorSetTorque()` 写入 MIT 扭矩帧：

```c
pos_set = 0;
vel_set = 0;
kp_set  = 0;
kd_set  = 0;
tor_set = clamp(tau_cmd, +/- T_max);
```

## 8. 初始化与复位

`YawLQRESO_Init()`：

- `memset(ctrl, 0, sizeof(*ctrl))`
- 初始化 ESO 默认增益
- 发布 `g_yaw_eso_*` 观测变量

首次收到 yaw 电机有效反馈时：

- `feedback_ready = 1`
- `tau_meas_filt = tau_meas_nm`
- 参考角初始化为当前实测角
- ESO `z1 = theta_meas`，`z2 = omega_meas`，`z3 = 0`
- 本周期输出 `0 N*m`

进入 `GIMBAL_ZERO_FORCE` 或反馈丢失时：

- `DMMotorSetTorque(yaw_motor, 0.0f)`
- `DMMotorStop(yaw_motor)`
- `YawLQRESO_Reset(&dzx_yaw_lqr_eso)`
- 清 `feedback_ready`、ESO、轨迹、积分、上一拍扭矩和调试有效标志

## 9. 模型辨识注入

模型辨识入口同自瞄文档，为 `g_yaw_model_ident.arm`。当前 `YawModelIdentReady()` 要求：

```c
input->hole_offset_mode == 0U && input->feedback_ok != 0U
```

也就是普通非自瞄和自瞄都可以进入辨识，只要不是狗洞模式且 yaw 电机反馈在线。

辨识运行时：

- 可用 `YawModelIdentApplyKOverride()` 切到辨识用小增益。
- `YawExcitationCalc()` 在 LQR 输出上叠加 chirp 力矩。
- `model_ident_active != 0` 时 ESO 继续观测，但不补偿。
- 普通非自瞄 soft limit / slew / 最小力矩补偿会被 `model_ident_active == 0U` 条件旁路，避免污染辨识激励。

默认两个辨识阶段：

| 阶段 | 默认时长 | 幅值 | 频率范围 |
|---|---:|---:|---:|
| low speed | `10 s` | `0.45 N*m` | `0.08 -> 0.80 Hz` |
| dynamic chirp | `8 s` | `0.8 N*m` | `0.5 -> 15 Hz` |

## 10. 主要 Live Watch 变量

### 10.1 输入与参考

| 变量 | 说明 |
|---|---|
| `g_yaw_lqr_ident_record.theta_ref_cmd_deg` | 非自瞄目标角，deg |
| `g_yaw_lqr_ident_record.theta_ref_rad` | 控制器 planned 参考角 |
| `g_yaw_lqr_ident_record.omega_ref_rad_s` | 由目标角变化估算出的参考速度 |
| `g_theta_ref_planned` | planned yaw，deg |
| `g_omega_ref_planned` | planned yaw 速度，rad/s |
| `g_alpha_ref_planned` | planned yaw 加速度，普通非自瞄通常为 0 |

### 10.2 误差与力矩

| 变量 | 说明 |
|---|---|
| `g_lqr_e_theta` | yaw 角误差，rad |
| `g_lqr_omega_filt` | 当前 yaw 速度反馈，rad/s |
| `g_lqr_tau_raw` | LQR 合成力矩 |
| `g_tau_meas_filt` | DM 反馈力矩低通 |
| `g_tau_track_err` | 命令与反馈力矩差 |
| `g_torque_cmd` | 最终命令力矩 |
| `g_ctrl_limit_active` | 当前控制器是否限幅 |

### 10.3 ESO 与调试记录

| 变量 | 说明 |
|---|---|
| `g_yaw_eso_w0` | 当前 ESO 带宽 |
| `g_yaw_eso_z1` | ESO 角估计 |
| `g_yaw_eso_z2` | ESO 速度估计 |
| `g_yaw_eso_z3` | ESO 总扰动估计 |
| `g_yaw_eso_tau_comp` | 实际注入的 ESO 补偿；普通非自瞄应为 0 |
| `g_yaw_lqr_ident_record.valid` | 普通非自瞄辨识/调试记录是否有效 |
| `g_yaw_lqr_ident_record.feedback_ok` | yaw 电机反馈是否在线 |

也可以看 `g_yaw_auto_aim_debug_record`。虽然名字带 `auto_aim`，它在 `YawLQRESO_Calc()` 内也会被填充非自瞄字段，适合看 `tau_ff_* / tau_theta / tau_omega / soft_limit_active / slew_limit_active / dm_limit_active` 等完整分解。

## 11. 调参顺序建议

1. 先确认模式：`nuc_mode=none_version_control`、`auto_aim_mode=0`、`feedback_ok=1`，且不是 `GIMBAL_HOLE_MODE`。
2. 检查目标与反馈：`theta_ref_cmd_deg` 应跟随遥控器/鼠标；`theta_meas_rad` 应随车体 yaw 连续变化。
3. 检查符号：若目标在当前角正方向，通常 `e_theta = theta_meas - theta_ref < 0`，`tau_theta` 应为正。
4. 先调 `g_lqr_k_theta/g_lqr_k_omega`，观察响应速度、超调和 `dm_limit_active`。
5. 如果起步克服静摩擦差，优先小步调 `g_ctrl_min_torque_comp`，不要直接大幅增加 `K_theta`。
6. 如果手感有力矩突变，调低 `g_ctrl_torque_slew_rate`；如果响应被明显拖慢，再适当调高。
7. 如果目标频繁跳变导致 `omega_ref_planned` 很大，检查 `g_ref_omega_limit_rad_s` 和遥控/鼠标增量是否合适。
8. 普通非自瞄默认不接 ESO 补偿；若要试验 ESO 补偿，建议单独加开关，不要直接复用狗洞强制补偿逻辑。

## 12. 当前实现中容易误解的点

- 普通非自瞄没有使用视觉三元组，`ref_traj_valid=0`。
- 普通非自瞄速度反馈来自 IMU gyro，不是 yaw 电机反馈速度。
- `RefPlanner_Update()` 会直接更新参考角，`g_ref_omega_limit_rad_s` 限的是参考速度估计，不是目标角本身的斜坡速度。
- ESO 在普通非自瞄下仍然更新 `z1/z2/z3`，但 `eso_comp_active=0`，不参与输出。
- `GIMBAL_HOLE_MODE` 虽然 `auto_aim_mode=0`，但不是普通手瞄：它使用狗洞增益并强制注入 ESO 补偿。
- 自瞄和普通非自瞄当前默认 `J/B/Coulomb/K` 已同步，但变量入口仍不同，后续在线调参时要确认改的是 `g_lqr_k_*` 还是 `g_lqr_auto_k_*`。
