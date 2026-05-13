# Yaw 电机控制方案整理

本文按当前 checkout 的实际代码链路整理 yaw 电机控制方案。当前 `application/robot_board.h` 选择的是 `CHASSIS_BOARD`，因此 yaw 电机控制运行在底盘板；云台板负责产生 yaw 目标角和 IMU yaw 测量值，并通过 RS485 下发到底盘板。

## 1. 总体结论

当前 yaw 电机不是驱动层 PID 位置环，也不是 DM 内部 MIT 位置保持。它的实际方案是：

```text
云台板遥控/鼠标/视觉输入
    -> yaw_control 连续角度目标, 单位 deg
    -> RS485 下发 yaw_control + yaw_angle
底盘板 robot_cmd
    -> 发布 gimbal_cmd
底盘板 GimbalTask_YAW
    -> YawLQRESO_Calc 计算 MIT torque, 单位 N*m
    -> DMMotorSetTorque 写入 DM MIT 帧: pos=0, vel=0, kp=0, kd=0, torque=tau
MotorControlTask
    -> DMMotorControl 组包并通过 CAN1 发给 yaw DM 电机
```

关键点：

- yaw 目标角是 `gimbal_cmd_recv.yaw`，单位是连续展开角度 `deg`。
- yaw 反馈角用于 LQRESO 的是 `gimbal_cmd_recv.yaw_measure_angle`，单位是连续展开角度 `deg`，进入控制器前转成 `rad`。
- yaw 速度反馈用于 LQRESO 的是 `yaw_motor->measure.vel`，单位 `rad/s`，不是 `gimbal_cmd_recv.yaw_measure_gyro`。
- LQRESO 输出是扭矩 `tau_cmd_nm`，最终写入 DM MIT 压缩帧的 `tor_set`。
- 狗洞模式下 yaw 电机仍跟随普通 `yaw_control`，`-67 deg` 是底盘 `offset_angle` 闭环目标，不是 yaw 电机目标。

## 2. 当前编译和任务归属

### 2.1 板级宏

`application/robot_board.h` 当前状态：

```c
#define CHASSIS_BOARD
// #define GIMBAL_BOARD
#define RC_VTM
```

影响：

- `GimbalInit()` 中 yaw DM 电机初始化只在 `#if defined(CHASSIS_BOARD)` 内编译。
- `robot_task.h` 中 `_GimbalTask()` 在 `CHASSIS_BOARD` 下调用 `GimbalTask_YAW()`。
- `GIMBAL_BOARD` 下 `_GimbalTask()` 调用 `GimbalTask_PITCH()`，不直接控制 yaw 电机。

### 2.2 周期任务

相关任务都使用 `osDelay(1)`，目标周期约 1 ms：

| 任务 | 入口 | 当前作用 |
|---|---|---|
| INS | `INS_EKF()` | 刷新 BMI088/EKF 姿态，提供 yaw/pitch/gyro |
| RobotCMD | `RobotCMDTask()` | 产生/转发 yaw 目标和反馈，处理双板通信 |
| Gimbal | `GimbalTask_YAW()` | 在底盘板计算 yaw 电机扭矩 |
| Motor | `MotorControlTask()` | 调用 `DMMotorControl()` 统一发送 DM 控制帧 |
| Chassis | `ChassisTask()` | 使用 yaw 电机单圈角计算底盘 follow/狗洞 offset |

实际相位由 FreeRTOS 调度决定，不能假设这些任务严格顺序执行，只能认为它们都在 1 kHz 附近刷新各自缓存。

## 3. 硬件和驱动配置

### 3.1 yaw DM 电机初始化

位置：`application/gimbal/gimbal.c::GimbalInit()`

当前 yaw 电机配置：

| 项目 | 当前值 | 说明 |
|---|---:|---|
| CAN | `hcan1` | 底盘板 CAN1 |
| tx_id | `0x12` | 发给 yaw 电机的控制 ID |
| rx_id | `0x02` | yaw 电机反馈 ID |
| motor_type | `DM_Motor` | 达妙电机 |
| control type | `TORQUE_LOOP_CONTRO` | 驱动层按 MIT 扭矩帧发送 |
| P range | `Infantry_PMAX = pi` | 位置协议量程 |
| V range | `Infantry_VMAX = 30.0f` | 速度协议量程 |
| T range | `Infantry_TMAX = 7.0f` | 力矩协议量程 |
| feedback source | `OTHER_FEED` | 配了外部角度/速度反馈指针，但当前 yaw 应用层控制不靠驱动 PID |
| driver PID | 无 | `outer_loop_type = OPEN_LOOP`, `close_loop_type = OPEN_LOOP` |

初始化后：

1. `yaw_motor = DMMotorInit(&yaw_config)` 注册 CAN 回调和 daemon。
2. `DMMotorStop(yaw_motor)` 把 `stop_flag` 置为 `MOTOR_STOP`，控制量清零。
3. `YawLQRESO_Init(&dzx_yaw_lqr_eso)` 初始化 yaw 控制器。
4. `DWT_GetDeltaT(&dzx_yaw_lqr_dwt_cnt)` 初始化控制周期计时。

### 3.2 DM 驱动层行为

位置：`modules/motor/DMmotor/DMmotor.c`

驱动层现在只负责：

- 解码反馈。
- 判断在线和 daemon 状态。
- 根据 `ctrl` 字段组包。
- 控制 `enable/disable/stop` 状态。
- 发送 CAN 帧。

驱动层不再替 yaw 算 PID。

#### 反馈解码

`DMMotorDecode()` 解码 DM 反馈：

| 字段 | 单位/意义 |
|---|---|
| `measure.pos` | 单圈位置，按 `[-P_max, P_max]` 解算，当前 yaw 是 `rad` |
| `measure.vel` | 速度，按 `[-V_max, V_max]` 解算，当前 yaw 是 `rad/s` |
| `measure.tor` | 电机反馈力矩，按 `[-T_max, T_max]` 解算，单位近似 `N*m` |
| `measure.total_round` | 根据单圈跨界累计圈数 |
| `measure.total_pos` | 展开后的累计位置，单位 `rad` |
| `online` | `measure.state == 1` 时为 1 |

展开逻辑：

```c
if (pos - last_pos > P_max / 2)  total_round--;
if (pos - last_pos < -P_max / 2) total_round++;
total_pos = pos + total_round * 2 * P_max;
```

当前 yaw 的 `P_max = pi`，所以 `2 * P_max` 正好是 `2*pi`。

#### 扭矩写入

`DMMotorSetTorque(motor, torque)` 会做：

```c
ctrl.pos_set = 0;
ctrl.vel_set = 0;
ctrl.kp_set  = 0;
ctrl.kd_set  = 0;
ctrl.tor_set = limit(torque, T_max);
```

所以 yaw 电机实际发送的是“MIT 扭矩帧”，不是“MIT 位置帧”：

```text
pos = 0
vel = 0
kp  = 0
kd  = 0
tor = LQRESO 输出
```

`DMMotorControl()` 每个 MotorControlTask 周期检查：

1. 如果 `protocol_disable != 0`，持续发送 `FF FF FF FF FF FF FF FD`。
2. 如果电机未在线或 `state != 1`，发送使能帧 `FF FF FF FF FF FF FF FC`。
3. 如果 `stop_flag == MOTOR_STOP`，控制量清零。
4. `TORQUE_LOOP_CONTRO` 下调用 `mit_ctrl()` 组 MIT 压缩帧。
5. `CANTransmit()` 发送。

## 4. 输入来源和双板通信链路

### 4.1 云台板产生 yaw 目标

位置：`application/cmd/robot_cmd.c`

云台板在 `RobotCMDTaskGimbalBoard()` 中读取遥控/鼠标/视觉并更新 `yaw_control`。

#### 遥控器模式

`RemoteControlSet()` 中：

```c
yaw_control -= YAW_K * rc_data[TEMP].rc.rocker_r_;
```

`YAW_K` 当前是：

```c
#define YAW_K 0.00035f
```

如果 `nuc_mode == version_control`，则：

```c
yaw_control = gimbal_cmd_send.yaw_version;
```

#### 键鼠模式

`GimbalSet()` 中：

```c
yaw_control -= rc_data[TEMP].mouse.x / 500.0f;
```

如果进入视觉模式：

```c
yaw_control = gimbal_cmd_send.yaw_version;
```

按住 `E` 时还会对视觉 yaw 偏差做一个 PID 修正：

```c
yaw_rotato_vision = PIDCalculate(&YAW_version_PID, gimbal_cmd_send.yaw_version, 0);
yaw_control -= yaw_rotato_vision;
```

这里的 `YAW_version_PID` 是目标生成侧的小修正，不是 yaw 电机主闭环。主闭环仍在底盘板 `YawLQRESO_Calc()`。

#### 视觉输入

`USB_Version_devode()` 中：

```c
fp_yaw = uint8_to_float_manual(fifo_pack + 3);
gimbal_cmd_send.yaw_version = RAD_2_DEGREE * fp_yaw;
```

因此视觉 yaw 误差输入是弧度进来，转成 deg 后进入 `yaw_version`。

当前代码还解析了视觉 yaw 速度：

```c
yaw_vel = RAD_2_DEGREE * uint8_to_float_manual(fifo_pack + 7);
vision_yaw_vel = yaw_vel;
```

但是 `yaw_vel` 没有列入 `robot_cmd_ctrl_signals[]` 的 RS485 发送表，当前 yaw LQRESO 也没有使用它。

### 4.2 yaw 目标连续化

`YawControlProcess()` 使用连续展开 yaw 角做过圈处理：

```c
yaw_measure_deg = gimbal_fetch_data.gimbal_imu_data->output.Yaw_total_angle_deg;
if (yaw_control - yaw_measure_deg > 180) {
    yaw_control -= 360;
} else if (yaw_control - yaw_measure_deg < -180) {
    yaw_control += 360;
}
```

目的：

- `yaw_control` 保持在离当前连续 yaw 最近的那一圈。
- 避免目标角在 `+180/-180` 附近跳变。
- 后级 LQRESO 还会再做一次最近角 wrap，形成双保险。

注意：

- 这里使用的是 `Yaw_total_angle_deg`，不是 `INS_angle_deg[2]`。
- `INS_angle_deg[2]` 是包在 `[-180, 180]` 附近的普通欧拉角，不适合作 yaw 目标连续化的唯一依据。

### 4.3 云台板通过 RS485 下发到底盘板

`robot_cmd_ctrl_signals[]` 定义了双板控制项发送周期：

| 字段 | 周期 tick | 目标频率约 | 当前意义 |
|---|---:|---:|---|
| `yaw_control` | 10 | 100 Hz | yaw 目标角，deg |
| `yaw_angle` | 1 | 1000 Hz | yaw 测量角，deg，连续展开 |
| `nuc_yaw` | 10 | 100 Hz | 视觉 yaw 目标/误差，deg |
| `gimbal_mode` | 13 | 约 77 Hz | 云台模式 |
| `chassis_mode` | 13 | 约 77 Hz | 底盘模式 |
| `nuc_mode` | 13 | 约 77 Hz | 是否视觉控制 |

云台板发送：

```c
chassis_cmd_send_uart.yaw_control = gimbal_cmd_send.yaw;
chassis_cmd_send_uart.yaw_angle =
    gimbal_fetch_data.gimbal_imu_data->output.Yaw_total_angle_deg;
chassis_cmd_send_uart.nuc_yaw = gimbal_cmd_send.yaw_version;
```

`yaw_gyro` 当前发送语句被注释：

```c
// chassis_cmd_send_uart.yaw_gyro = ...
```

这意味着底盘板收到的 `yaw_measure_gyro` 当前不应作为有效 yaw 速度反馈来依赖。好在当前 yaw LQRESO 实际使用的是 `yaw_motor->measure.vel`。

### 4.4 底盘板接收并发布 gimbal_cmd

底盘板 RS485 从机回调先刷新 `chassis_rs485_recv`。随后 `RobotCMDTaskChassisBoard()` 将它映射到本地 `gimbal_cmd_send`：

```c
gimbal_cmd_send.yaw               = chassis_rs485_recv.yaw_control;
gimbal_cmd_send.gimbal_mode       = chassis_rs485_recv.gimbal_mode;
gimbal_cmd_send.nuc_mode          = chassis_rs485_recv.nuc_mode;
gimbal_cmd_send.yaw_version       = chassis_rs485_recv.nuc_yaw;
gimbal_cmd_send.yaw_measure_angle = chassis_rs485_recv.yaw_angle;
gimbal_cmd_send.yaw_measure_gyro  = chassis_rs485_recv.yaw_gyro;
PubPushMessage(gimbal_cmd_pub, &gimbal_cmd_send);
```

`GimbalTask_YAW()` 再通过 `SubGetMessage(gimbal_sub, &gimbal_cmd_recv)` 获取这些值。

## 5. yaw 控制器输入输出契约

位置：

- `application/gimbal/gimbal.c::GimbalYawBuildControlInput()`
- `modules/lqr_eso/lqr_eso.h::YawControlInput_t`

当前输入打包：

```c
input->theta_ref_deg = ref_deg;
input->tau_meas_nm   = yaw_motor ? yaw_motor->measure.tor : 0;
input->feedback_ok   = DMMotorIsOnline(yaw_motor);
input->theta_rad     = gimbal_cmd_recv.yaw_measure_angle * LQR_ESO_DEG_TO_RAD;
input->omega_rad_s   = yaw_motor ? yaw_motor->measure.vel : 0;
```

数据契约表：

| 信号 | 生产者 | 消费者 | 单位 | 说明 |
|---|---|---|---|---|
| `gimbal_cmd_recv.yaw` | 云台板 `yaw_control` | yaw LQRESO | deg | 目标角，连续展开 |
| `gimbal_cmd_recv.yaw_measure_angle` | 云台板 IMU `Yaw_total_angle_deg` | yaw LQRESO | deg | 测量角，连续展开 |
| `yaw_motor->measure.vel` | DM yaw 反馈 | yaw LQRESO | rad/s | 速度反馈 |
| `yaw_motor->measure.tor` | DM yaw 反馈 | yaw ESO/调试 | N*m | 可选力矩反馈 |
| `YawLQRESO_Calc().tau_cmd_nm` | yaw LQRESO | `DMMotorSetTorque()` | N*m | 最终扭矩命令 |
| `yaw_motor->measure.pos` | DM yaw 反馈 | offset/上传 | rad | 单圈角 |
| `yaw_motor->measure.total_pos` | DM yaw 反馈 | 上传/调试 | rad | 展开角 |

## 6. yaw 模式状态机

位置：`application/gimbal/gimbal.c::GimbalYawControlStep()`

### 6.1 GIMBAL_ZERO_FORCE

```c
DMMotorSetTorque(yaw_motor, 0.0f);
DMMotorStop(yaw_motor);
YawLQRESO_Reset(&dzx_yaw_lqr_eso);
```

行为：

- 清零 yaw 扭矩。
- `stop_flag = MOTOR_STOP`。
- LQRESO 状态和 DWT dt 重新同步。
- `dzx_yaw_torque = 0`，`angle_error = 0`。

### 6.2 GIMBAL_GYRO_MODE

```c
DMMotorEnable1(yaw_motor);
DMMotorSetTorque(yaw_motor, GimbalYawCalculateTorque(yaw_ref));
```

行为：

- 清除 `protocol_disable`，使能正常控制。
- 目标 `yaw_ref = gimbal_cmd_recv.yaw`。
- 由 LQRESO 输出扭矩。

### 6.3 GIMBAL_HOLE_MODE

```c
DMMotorEnable1(yaw_motor);
DMMotorSetTorque(yaw_motor, GimbalYawCalculateTorque(yaw_ref));
```

行为与 `GIMBAL_GYRO_MODE` 对 yaw 电机相同。

关键区别在底盘侧：

- `GIMBAL_HOLE_MODE` 不把 yaw 电机目标改成 `-67 deg`。
- `-67 deg` 由 `ChassisTask()` 里的 `offset_angle -> HOLE_YAW_OFFSET_TARGET_DEG` 完成。
- yaw 电机继续稳定在操作者/视觉给出的 `yaw_control`。

### 6.4 GIMBAL_MOTOR_MODE

当前实现同样走 LQRESO 扭矩闭环：

```c
DMMotorEnable1(yaw_motor);
DMMotorSetTorque(yaw_motor, GimbalYawCalculateTorque(gimbal_cmd_recv.yaw));
```

也就是说当前 `GIMBAL_MOTOR_MODE` 并不是 DM 电机位置模式，只是模式名保留了旧语义。

### 6.5 default

未知模式下：

```c
DMMotorSetTorque(yaw_motor, 0.0f);
dzx_yaw_torque = 0;
angle_error = 0;
```

这里没有调用 `DMMotorStop()`，所以 `stop_flag` 状态取决于之前模式；但本周期扭矩会被写为 0。

## 7. LQRESO 控制律

位置：`modules/lqr_eso/lqr_eso.c`

### 7.1 当前默认参数

当前固件默认值：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `YAW_J_DEFAULT` | `0.0491133f` | yaw 转动惯量模型 |
| `YAW_B_DEFAULT` | `0.0176839f` | yaw 粘性阻尼模型 |
| `YAW_TS_DEFAULT` | `0.001f` | 默认控制周期 |
| `g_lqr_k_theta` | `43.7692812f` | 非自瞄角度误差 LQR 增益 |
| `g_lqr_k_omega` | `2.09040931f` | 非自瞄角速度误差 LQR 增益 |
| `g_lqr_auto_k_theta` | `55.0f` | 自瞄角度误差 LQR 增益 |
| `g_lqr_auto_k_omega` | `0.5f` | 自瞄角速度误差 LQR 增益 |
| `g_ctrl_t_limit` | `7.0f` | 软力矩限幅 |
| `g_ctrl_angle_deadband_rad` | `0.0f` | 角度死区 |
| `g_ctrl_vel_deadband` | `0.03f` | 低速判定阈值 |
| `g_ref_omega_limit_rad_s` | `100.0f` | 目标速度限幅 |
| `g_ctrl_torque_slew_rate` | `1000.0f` | 普通 yaw 扭矩斜率限幅，N*m/s |
| `g_ctrl_auto_torque_slew_rate` | `2000.0f` | 自瞄 yaw 扭矩斜率限幅，2 N*m/ms |
| `g_ctrl_min_torque_comp` | `0.40f` | 非自瞄小误差低速静摩擦补偿 |
| `g_eso_comp_gain` | `0.18f` | ESO 扰动补偿增益 |
| `g_eso_comp_limit` | `0.35f` | ESO 补偿限幅 |
| `g_eso_comp_gain_max` | `2.0f` | Ozone 可调的 ESO 补偿增益上限 |
| `g_eso_comp_limit_max` | `7.0f` | Ozone 可调的 ESO 补偿限幅上限 |
| `g_yaw_eso_alpha` | `1.10f` | `g_yaw_eso_w0_set <= 0` 时使用的 ESO 带宽系数 |
| `g_yaw_eso_w0_set` | `36.0f` | yaw ESO 直接带宽入口，`>0` 时作为 `w0` |
| `g_yaw_auto_eso_comp_enable` | `1U` | 自瞄低速 ESO 扭矩补偿使能 |
| `g_yaw_auto_eso_comp_omega_gate_rad_s` | `2.5f` | 自瞄 ESO 补偿低速门限 |

注意：`tools/dm_4310_lqr_yaw.m` 里写的 `J=0.0360`、`Bv=0.0320` 与当前 `lqr_eso.c` 不一致。调参时应先同步脚本和固件，固件当前值才是运行时事实。

### 7.2 dt 处理

`YawLQRESO_Calc()` 的 dt 来自：

```c
DWT_GetDeltaT(&dzx_yaw_lqr_dwt_cnt)
```

进入控制器后会被保护：

```c
if (dt < 0.0001f || dt > 0.02f) {
    dt = 0.001f;
}
```

即任务抖动过大时使用 1 ms 默认值。

### 7.3 目标角 wrap

目标先从 deg 转 rad：

```c
theta_target_rad = theta_ref_deg * LQR_ESO_DEG_TO_RAD;
```

然后用 `LQRESOWrapNearestRad(target, measure)` 把目标折到离当前测量最近的等效角：

```text
while target - measure >  pi: target -= 2*pi
while target - measure < -pi: target += 2*pi
```

如果 `g_ctrl_max_error_rad > 0`，还会把目标限在当前测量附近：

```text
theta_target_rad in [theta_meas - max_error, theta_meas + max_error]
```

当前默认 `g_ctrl_max_error_rad = 0`，等于不启用这个误差限幅。

### 7.4 首次反馈初始化

如果 `feedback_ok == 0`：

- 直接 `YawLQRESO_Reset()`。
- 输出 0。

如果刚从未 ready 进入 ready：

```c
feedback_ready = 1;
tau_meas_filt = tau_meas_nm;
RefPlanner_Reset(ctrl, input->theta_rad);
YawEso_ResetInternal(ctrl, input->theta_rad, input->omega_rad_s);
output = 0;
return;
```

所以电机刚在线的第一拍不会突然输出扭矩。

### 7.5 参考规划器

当前 `RefPlanner_Update()` 不是完整角度斜坡规划，而是：

```c
theta_ref_planned_rad = theta_target_rad;
omega_ref_planned = (theta_target_rad - old_theta_ref) / dt;
omega_ref_planned = clamp(omega_ref_planned, -g_ref_omega_limit_rad_s, +g_ref_omega_limit_rad_s);
alpha_ref_planned = 0;
```

结论：

- 角度参考本身直接等于目标角。
- 只有 `omega_ref_planned` 被 `g_ref_omega_limit_rad_s` 限幅。
- `g_ref_max_vel_deg_s` 和 `g_ref_max_accel_deg_s2` 当前在 yaw `RefPlanner_Update()` 中没有实际参与控制。

### 7.6 ESO

ESO 增益每次计算前更新：

```c
b0 = 1 / YAW_J_DEFAULT;
w0 = g_yaw_eso_alpha * sqrt(b0 * abs(g_lqr_k_theta));
beta1 = 3 * w0;
beta2 = 3 * w0 * w0;
beta3 = w0 * w0 * w0;
```

当前源码没有对 `w0` 做 min/max clamp。

状态更新：

```text
e      = theta_meas - z1
z1_dot = z2 + beta1 * e
z2_dot = -(B/J) * z2 + b0 * u_last + z3 + beta2 * e
z3_dot = beta3 * e
```

含义：

- `z1` 是角度估计。
- `z2` 是角速度估计。
- `z3` 是总扰动估计。
- `u_last` 默认来自上一次控制输出，也可以用 `g_eso_torque_feedback_blend` 混入力矩反馈。

力矩反馈混合：

```c
tau_for_eso =
    (1 - blend) * ctrl->eso_u_last +
    blend * ctrl->tau_meas_filt;
```

默认 `g_eso_torque_feedback_blend = 0`，即 ESO 使用上一次命令力矩，不使用反馈力矩。

### 7.7 LQR 主控制律

误差：

```text
e_theta_raw = input->theta_rad - theta_ref_planned
e_theta     = deadband(e_theta_raw, g_ctrl_angle_deadband_rad)
e_omega     = input->omega_rad_s - omega_ref_planned
```

`z1/z2` 仍由 ESO 更新和发布，但不参与 yaw 主反馈。

前馈：

```text
tau_ff = J * alpha_ref + B * omega_ref
```

当前 `alpha_ref = 0`，所以实际主要是：

```text
tau_ff = B * omega_ref
```

LQR：

```text
tau_lqr = tau_ff
          - k_theta * e_theta
          - k_omega * e_omega
          - g_lqi_ki * lqi_int
```

非自瞄时 `k_theta/k_omega` 来自 `g_lqr_k_theta/g_lqr_k_omega`；自瞄时来自 `g_lqr_auto_k_theta/g_lqr_auto_k_omega`。

默认 `g_lqi_ki = 0`，积分项默认不生效。

ESO 观测：

```text
tau_comp = -g_eso_comp_gain * z3 / b0
tau_comp = clamp_abs(tau_comp, g_eso_comp_limit)
```

ESO 不参与角度/角速度反馈。当前只在自瞄 `auto_aim_mode != 0` 且角速度门控满足时，把 `tau_comp` 注入最终扭矩：

```text
abs(input->omega_rad_s) <= g_yaw_auto_eso_comp_omega_gate_rad_s
g_yaw_auto_eso_comp_enable != 0
```

最终基础命令：

```text
非自瞄: tau_cmd = tau_lqr
自瞄门控未满足: tau_cmd = tau_lqr
自瞄门控满足: tau_cmd = tau_lqr + tau_comp
```

### 7.8 限幅和保护

控制器内部按这个顺序保护：

1. 自瞄低速区可把 `tau_comp` 加到 `tau_lqr`。
2. `g_ctrl_t_limit` 软限幅，默认 `7 N*m`。
3. 未限幅时更新 LQI 积分和力矩 bias。
4. 加上 `tau_bias` 后再次软限幅。
5. 硬限幅到 `DM_T_MIN = -7`、`DM_T_MAX = 7`。
6. 普通模式按 `g_ctrl_torque_slew_rate * dt` 做扭矩斜率限制；自瞄 `nuc_mode == version_control` 时按 `g_ctrl_auto_torque_slew_rate * dt` 做扭矩斜率限制，默认 `2000 N*m/s = 2 N*m/ms`。
7. 非自瞄低速小扭矩时做固定静摩擦补偿；自瞄跳过该固定补偿。

静摩擦补偿条件：

```text
auto_aim_mode == 0
g_ctrl_min_torque_comp > 0
abs(input->omega_rad_s) < g_ctrl_vel_deadband
abs(e_theta_raw) > g_ctrl_angle_deadband_rad
abs(tau_cmd) < g_ctrl_min_torque_comp
```

满足时：

```text
tau_cmd = -sign(e_theta_raw) * g_ctrl_min_torque_comp
```

当前默认最小补偿是 `0.40 N*m`。自瞄模式用 ESO 低速小误差补偿替代它，避免目标附近出现固定 0.40 N*m 阶跃。

## 8. 反馈上传和底盘 offset 链路

### 8.1 yaw 反馈打包

位置：`application/gimbal/gimbal.c::GimbalYawUpdateFeedback()`

每次 yaw 控制后更新：

```c
gimbal_feedback_data.yaw_angle_pidout = dzx_yaw_torque;
gimbal_feedback_data.gimbal_imu_data = gimbal_IMU_data;
gimbal_feedback_data.yaw_motor_single_round_angle = GimbalDMYawSingleRoundDeg();
gimbal_feedback_data.yaw_ecd = yaw_motor->measure.p_int;
gimbal_feedback_data.yaw_total_angle = GimbalDMYawTotalDeg();
```

其中：

```c
GimbalDMYawSingleRoundDeg()
    = wrap360(yaw_motor->measure.pos * 57.2957795f)

GimbalDMYawTotalDeg()
    = yaw_motor->measure.total_pos * 57.2957795f
```

`GimbalTask_YAW()` 末尾还会再次保证：

```c
gimbal_feedback_data.yaw_motor_single_round_angle = GimbalDMYawSingleRoundDeg();
gimbal_feedback_data.gimbal_imu_data = gimbal_IMU_data;
PubPushMessage(gimbal_pub, &gimbal_feedback_data);
```

### 8.2 offset_angle 计算

位置：`application/cmd/robot_cmd.c::CalcOffsetAngle()`

当前计算：

```c
yaw_current_deg = gimbal_fetch_data.yaw_motor_single_round_angle;
yaw_forward_deg = YAW_DM_Forward_ANGLE_POS * RAD_TO_ANGLE;
chassis_cmd_send.offset_angle =
    -RobotCMDWrap180Deg(yaw_current_deg - yaw_forward_deg);
```

参数：

```c
#define YAW_DM_Forward_ANGLE_POS  -0.533785582f
```

含义：

- `YAW_DM_Forward_ANGLE_POS` 是 yaw 电机和底盘正方向对齐时的 DM 位置，单位 `rad`。
- 先转成 deg，再与当前 DM 单圈角做差。
- `RobotCMDWrap180Deg()` 把结果限到 `[-180, 180]`。
- 前面再取负号，形成底盘 follow 使用的 `offset_angle`。

这个 `offset_angle` 是底盘 yaw 跟随控制的反馈量，不是 yaw 电机 LQRESO 的角度反馈量。

### 8.3 底盘 follow 和狗洞 yaw

位置：`application/chassis/chassis.c`

底盘侧 `Chassis_Follow_PID` 共用于：

- 普通 follow：`offset_angle -> 0 deg`
- 狗洞模式：`offset_angle -> -67 deg`

普通 follow：

```c
chassis_cmd_recv.wz =
    PIDCalculate(&Chassis_Follow_PID, chassis_cmd_recv.offset_angle, 0);
```

狗洞模式：

```c
chassis_cmd_recv.wz =
    PIDCalculate(&Chassis_Follow_PID,
                 chassis_cmd_recv.offset_angle,
                 HOLE_YAW_OFFSET_TARGET_DEG);
```

参数：

```c
#define HOLE_YAW_OFFSET_TARGET_DEG (-67.0f)
#define HOLE_READY_THRESHOLD_DEG   3.0f
```

狗洞 ready 由底盘板 `RobotCMDHoleYawOffsetReady()` 判断：

```c
abs(offset_angle - HOLE_YAW_OFFSET_TARGET_DEG) <= 3 deg
```

然后通过 RS485 upload 的 `hole_yaw_ready` 传回云台板，云台板 pitch 任务据此进入后续 down/up 动作。

重要边界：

- yaw 电机控制目标保持 `yaw_control`。
- 底盘 yaw 姿态通过 `offset_angle` 和四轮底盘闭环完成。
- 不能把 `HOLE_YAW_OFFSET_TARGET_DEG` 当成 yaw 电机目标角。

## 9. 当前调试观测量

### 9.1 yaw 控制器观测量

`modules/lqr_eso/lqr_eso.h` 中对外暴露的 yaw watch 变量：

| 变量 | 意义 |
|---|---|
| `g_lqr_k_theta` | 在线调非自瞄 LQR 角度增益 |
| `g_lqr_k_omega` | 在线调非自瞄 LQR 速度增益 |
| `g_lqr_auto_k_theta` | 在线调自瞄 LQR 角度增益 |
| `g_lqr_auto_k_omega` | 在线调自瞄 LQR 速度增益 |
| `g_yaw_eso_w0_set` | yaw ESO 直接带宽输入，`>0` 时生效 |
| `g_yaw_eso_w0` | yaw ESO 实际生效带宽 |
| `g_yaw_eso_z3` | yaw ESO 扰动估计 |
| `g_yaw_eso_tau_comp` | yaw ESO 扭矩补偿计算值 |
| `g_yaw_auto_eso_comp_active` | 当前周期是否向自瞄扭矩注入 ESO 补偿 |
| `g_lqr_e_theta` | 当前控制角度误差，rad |
| `g_lqr_omega_filt` | 当前控制使用的原始 yaw 角速度 |
| `g_lqr_tau_raw` | LQR 扭矩，未注入 ESO 补偿 |
| `g_torque_cmd` | 最终保存的扭矩命令 |
| `g_theta_ref_planned` | 规划后目标角，deg |
| `g_omega_ref_planned` | 规划后目标角速度，rad/s |
| `g_yaw_eso_z1` | ESO 角度估计 |
| `g_yaw_eso_z2` | ESO 角速度估计 |
| `g_yaw_eso_z3` | ESO 扰动估计 |
| `g_yaw_eso_tau_comp` | ESO 扰动补偿观察量，不注入最终扭矩 |
| `g_ctrl_limit_active` | 本周期是否触发限幅 |

`application/gimbal/gimbal.c` 中额外 watch：

| 变量 | 意义 |
|---|---|
| `dzx_yaw_torque` | 应用层 yaw 扭矩输出 |
| `angle_error` | 当前复制的 `g_lqr_e_theta` |
| `yaw_fit_time_s` | 采样时间 |
| `yaw_angle_rad` | 拟合用 yaw 角 |
| `yaw_gyro_rad_s` | 拟合用 yaw 速度，但当前来源不可靠，见下文 |
| `motor_torque_nm` | DM 反馈力矩 |
| `yaw_motor_pos_rad` | DM 单圈位置 |
| `yaw_motor_total_rad` | DM 展开位置 |

### 9.2 DM 电机状态观测量

建议 Ozone 同时看：

```text
yaw_motor->online
yaw_motor->stop_flag
yaw_motor->protocol_disable
yaw_motor->measure.state
yaw_motor->measure.pos
yaw_motor->measure.total_pos
yaw_motor->measure.vel
yaw_motor->measure.tor
yaw_motor->ctrl.tor_set
```

判断规则：

- `online=1` 且 `measure.state=1` 才会真正发送正常控制帧。
- `protocol_disable=1` 时会持续发送 DM 失能帧，不会自动恢复扭矩控制。
- `stop_flag=MOTOR_STOP` 时 `DMMotorControl()` 会把控制量清零。

### 9.3 输入链路观测量

建议同时看：

```text
gimbal_cmd_recv.gimbal_mode
gimbal_cmd_recv.yaw
gimbal_cmd_recv.yaw_measure_angle
gimbal_cmd_recv.yaw_measure_gyro
chassis_rs485_recv.yaw_control
chassis_rs485_recv.yaw_angle
```

对照关系：

- `gimbal_cmd_recv.yaw` 应跟随 `chassis_rs485_recv.yaw_control`。
- `gimbal_cmd_recv.yaw_measure_angle` 应跟随 `chassis_rs485_recv.yaw_angle`。
- `yaw_measure_gyro` 当前可能无效，因为云台板发送端对应赋值被注释。

## 10. 调参与拟合入口

### 10.1 在线调参首选变量

调响应速度：

- 非自瞄调 `g_lqr_k_theta/g_lqr_k_omega`。
- 自瞄调 `g_lqr_auto_k_theta/g_lqr_auto_k_omega`，默认 `55.0f/0.5f`。
- 增大 `g_ctrl_t_limit`：允许更大扭矩，但当前 DM 硬限制仍是 `±7 N*m`。
- 减小 `g_ctrl_torque_slew_rate`：普通模式扭矩更平滑，但响应更慢。
- 减小 `g_ctrl_auto_torque_slew_rate`：自瞄扭矩更平滑，但响应更慢，默认 `2 N*m/ms`。
- 调 `g_ref_omega_limit_rad_s`：限制目标变化造成的参考速度前馈。

调小误差卡滞：

- `g_ctrl_min_torque_comp` 是静摩擦补偿，不是模型 `J/B`。
- 非自瞄低速、小扭矩、仍有角误差时才触发。
- 自瞄低速区由 ESO `tau_comp` 替代固定最小扭矩补偿。
- 如果目标附近来回抖，先确认这个补偿没有过大。

调 ESO：

- `g_yaw_eso_w0_set > 0` 时直接决定 yaw ESO 带宽，默认 `36.0f`。
- `g_yaw_eso_w0_set <= 0` 时回退到 `g_yaw_eso_alpha * sqrt(b0 * abs(g_lqr_k_theta))`。
- `g_eso_comp_gain` 和 `g_eso_comp_limit` 影响 `g_yaw_eso_tau_comp`，自瞄门控满足时会注入扭矩。
- `g_eso_comp_gain_max` 和 `g_eso_comp_limit_max` 是 Ozone 可调上限；如果增益/限幅被写回，先检查这两个上限。
- `g_yaw_auto_eso_comp_omega_gate_rad_s` 控制自瞄补偿工作区，当前不限制角度误差。
- yaw ESO 仍不接入角度/角速度反馈。

### 10.2 MATLAB 脚本

`tools/dm_4310_lqr_yaw.m` 用于离线算 `g_lqr_k_theta` 和 `g_lqr_k_omega`。

当前使用前必须注意：

- 脚本顶部写的固件同步参数与 `lqr_eso.c` 当前默认值不一致。
- `lqr_eso.c` 当前是 `J=0.0280`、`B=0.0140`。
- 脚本当前是 `J=0.0360`、`Bv=0.0320`。
- 若要保持 MATLAB 和固件一致，先同步这两个参数，再运行脚本。

### 10.3 CSV 拟合脚本

`tools/yaw_lqreso_fit.py` 期望列：

```text
Time, test_yaw_ref, yaw_angle_rad, yaw_gyro_rad_s, motor_torque_nm
```

当前源码里需要特别注意：

- `test_yaw_ref` 只是全局变量，当前 `GimbalYawControlStep()` 不读取它作为扭矩开环源。
- `yaw_gyro_rad_s` 当前来自 `gimbal_cmd_recv.yaw_measure_gyro`，而云台板 RS485 发送端对 `yaw_gyro` 的赋值被注释。
- 如果按当前代码直接采样，`yaw_gyro_rad_s` 可能不是有效 yaw 速度。
- 真正用于控制的速度是 `yaw_motor->measure.vel`。

因此如果要做 yaw 植物辨识，建议采样：

```text
yaw_fit_time_s
g_torque_cmd 或 yaw_motor->ctrl.tor_set
yaw_motor->measure.pos 或 yaw_angle_rad
yaw_motor->measure.vel
motor_torque_nm
```

或者先恢复/改正 `yaw_gyro_rad_s` 的来源，再使用拟合脚本。

## 11. 易错点清单

1. `GIMBAL_HOLE_MODE` 的 `-67 deg` 不是 yaw 电机目标角。
   它是底盘 follow PID 的 `offset_angle` 目标。

2. `yaw_motor->measure.pos` 和 `yaw_motor->measure.total_pos` 是 `rad`。
   `yaw_motor_single_round_angle` 是把 `measure.pos` 转成 `0..360 deg` 后上传。

3. `gimbal_cmd_recv.yaw` 是 `deg`。
   `YawLQRESO_Calc()` 内部会转成 `rad`。

4. `gimbal_cmd_recv.yaw_measure_angle` 是 `deg`。
   它应该来自连续展开 yaw，即 `Yaw_total_angle_deg`。

5. 当前 yaw 速度反馈不是 IMU gyro。
   控制输入里实际使用 `yaw_motor->measure.vel`。

6. `g_ref_max_vel_deg_s` 和 `g_ref_max_accel_deg_s2` 当前不控制 yaw 角度斜坡。
   当前参考规划器只直接赋角度目标，并限 `omega_ref`。

7. `DZX_YAW_TORQUE_LIMIT` 当前未参与 yaw 控制限幅。
   实际限幅看 `g_ctrl_t_limit`、`DM_T_MIN/DM_T_MAX` 和 DM 电机 `Infantry_TMAX`。

8. `yaw_speedFeed`、`yaw_kp`、`yaw_kd`、`ifSpeedFeed` 当前不在 yaw 主控制链路里使用。

9. `test_yaw_ref` 当前不是开环扭矩入口。
   如果需要开环扫频/阶跃，需要确认代码中实际把它接入了 `DMMotorSetTorque()` 或 LQRESO 输入。

10. Ozone 判断 yaw 是否“没使能”时，要区分：
    `stop_flag`、`online/state`、`protocol_disable`、`ctrl.tor_set` 四件事。

## 12. 推荐验证顺序

### 12.1 静态通信验证

先不转电机，确认数据链：

```text
云台板:
  yaw_control
  gimbal_cmd_send.yaw
  chassis_cmd_send_uart.yaw_control
  chassis_cmd_send_uart.yaw_angle

底盘板:
  chassis_rs485_recv.yaw_control
  chassis_rs485_recv.yaw_angle
  gimbal_cmd_recv.yaw
  gimbal_cmd_recv.yaw_measure_angle
```

目标：底盘板收到的目标和测量与云台板发送一致，且单位仍是 deg。

### 12.2 电机在线验证

看：

```text
yaw_motor->online
yaw_motor->measure.state
yaw_motor->protocol_disable
yaw_motor->stop_flag
```

目标：

- 非零力模式下 `stop_flag` 应为 enabled。
- `protocol_disable` 应为 0。
- `online` 和 `measure.state` 应允许正常控制。

### 12.3 闭环低增益验证

先减小 ESO 或关闭补偿：

```text
g_eso_comp_gain = 0
```

观察：

```text
g_lqr_e_theta
g_lqr_omega_filt
g_lqr_tau_raw
g_torque_cmd
yaw_motor->ctrl.tor_set
yaw_motor->measure.vel
```

目标：

- 目标变化时扭矩方向正确。
- `g_lqr_e_theta` 收敛到 0 附近。
- `ctrl.tor_set` 没有长期打满。

### 12.4 wrap 验证

在 `+180/-180` 附近转动，重点看：

```text
gimbal_cmd_recv.yaw
gimbal_cmd_recv.yaw_measure_angle
g_theta_ref_planned
g_lqr_e_theta
```

目标：

- `yaw_measure_angle` 使用连续角，不跳 `360 deg`。
- `g_lqr_e_theta` 不出现接近 `2*pi` 的大跳变。

### 12.5 狗洞模式验证

看底盘侧：

```text
chassis_cmd_recv.gimbal_mode
chassis_cmd_recv.offset_angle
chassis_cmd_recv.wz
chassis_fetch_data_uart.hole_yaw_ready
```

看 yaw 侧：

```text
gimbal_cmd_recv.gimbal_mode
gimbal_cmd_recv.yaw
g_theta_ref_planned
```

目标：

- `GIMBAL_HOLE_MODE` 时 `offset_angle` 向 `-67 deg` 收敛。
- yaw 电机目标不被改成 `-67 deg`。
- `hole_yaw_ready` 只在 offset 进入 `-67 ± 3 deg` 后置位。

## 13. 一句话版本

当前 yaw 电机方案是双板架构下的“云台板给连续 yaw 目标和 IMU 连续 yaw 测量，底盘板用 yaw DM 电机速度反馈和 LQRESO 算扭矩，DM 驱动层只发送 MIT 扭矩帧；底盘 follow/狗洞 yaw 通过 yaw 电机单圈角计算 `offset_angle` 后由底盘四轮闭环完成”。
