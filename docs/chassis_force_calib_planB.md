---
title: "底盘四轮力-电流系数标定手册（方案 B：滚动加速度反推）"
author: "戴子轩"
date: "2026-05-08"
scope: "Infantry_RMUC_2026 / X 型普通 omni 底盘"
---

# 0. 概述

本文给出在**不顶起底盘、不使用拉力计**的条件下，标定四个 omni 轮各自正负方向「电流命令 → 等效推力」系数 \(K_i^{\pm}\) 的完整流程。

标定目的：替换工程中现有的「整车 80 / 88 cmd/N」近似（[chassis.c:127-130](../application/chassis/chassis.c#L127-L130)），为后续把 `OmniVelocityCtrl` 的 `force_cmd[4]` 通过 \(I_{cmd,i}=K_i\cdot f_{cmd,i}+I_{0,i}\) 真正接入电机电流通道做准备。

核心原理：单轮单方向发电流，**其他三轮命令 0、自然接地**，用 IMU 测车体加速度，再通过差分电流消静摩擦，反推该轮的等效力系数。

预计工作量：场地 5×5 m 平整水泥地 / 室内 + IMU 标定良好的整机 → **30~60 分钟数据采集 + 30 分钟离线拟合**。

---

# 1. 标定原理

## 1.1 单轮单方向的物理关系

在水平地面上，只有第 \(i\) 个轮子被驱动、其他三轮电流命令为 0 时，车体动力学方程为：

\[
m\ddot{p}_b
=
H^T
\begin{bmatrix}0\\\vdots\\F_i\\\vdots\\0\end{bmatrix}
+ d_{passive}
\]

其中：

- \(\ddot{p}_b\) 是车体系下的加速度向量 \([\dot{v}_x,\dot{v}_y,\dot{\omega}_z]^T\)。
- \(F_i\) 是第 \(i\) 轮在自身主滚动方向上的实际推力（N）。
- \(d_{passive}\) 是其他三轮接地带来的被动反作用：电磁阻尼 + 齿轮箱摩擦 + 自由滚柱滚动摩擦。

对**短时启动瞬态**而言：

- 电磁阻尼 \(\propto\) 车速，启动初瞬车速 ≈ 0，可忽略。
- 齿轮箱静摩擦是常值阈值，下面用差分电流法消掉。
- 自由滚柱滚动摩擦量级小（约 1~3 N/轮），且对所有测试同样存在，最终都被吸收进等效系数 \(K_i^{eff}\)。

## 1.2 等效力系数 \(K_i^{eff}\) 的定义

定义被测系数为：

\[
K_i^{eff}
\;\triangleq\;
\frac{\partial F_i^{\text{施加}}}{\partial I_i^{\text{命令}}}
\quad\text{（单位：N/cmd）}
\]

它已经把以下因素**合并进一个等效数**：

- 真实电机扭矩常数 \(K_t\)、减速比 \(n\)、传动效率 \(\eta\)
- 轮半径 \(r\)
- C620 ESC 的 cmd→电流增益误差
- 其他三轮接地带来的被动负载分摊

**这正是控制器需要的系数**——它直接描述「我发多少 cmd、车体能拿到多少 N 力」，不需要再拆分。用它去做后续力分配 \(I_{cmd,i}=K_i^{-1}\cdot f_{cmd,i}\)，分配器和真实物体用同一组系数，**不会引入零空间内力**。

## 1.3 为什么不用腾空

如果用方案 A 单轮单激励 + 拉力计：被动三轮电流 0 但接地，反作用力 5~15 N/轮 × 3 = 15~45 N，会被「吃掉」拉力计读数 → 测出来的是 \(F_i^{真实} - F_{passive}^{三轮}\)，不是真 \(F_i\)。

方案 B 测的是「该电流命令下整车能获得的加速度」，被动负载本来就该计入控制传递关系。**装车跑控制时的工况就是四轮接地**，所以方案 B 给出的 \(K_i^{eff}\) 在控制意义下比方案 A 的"理想 K"更对口。

## 1.4 主滚动方向投影

理想 X 型 omni 的主滚动方向单位向量（车体系）：

\[
t_{LF}=\frac{1}{\sqrt{2}}\!
\begin{bmatrix}1\\-1\end{bmatrix},\quad
t_{RF}=\frac{1}{\sqrt{2}}\!
\begin{bmatrix}1\\1\end{bmatrix},\quad
t_{RB}=\frac{1}{\sqrt{2}}\!
\begin{bmatrix}1\\-1\end{bmatrix},\quad
t_{LB}=\frac{1}{\sqrt{2}}\!
\begin{bmatrix}1\\1\end{bmatrix}
\]

把车体系实测加速度 \(\mathbf{a}_b=[a_x,a_y]^T\) 投影到 \(t_i\)：

\[
a_{\parallel,i} = t_i^T \mathbf{a}_b
\]

则有

\[
F_i^{\parallel} = m\cdot a_{\parallel,i}
\quad\Rightarrow\quad
K_i^{eff} = \frac{m\cdot a_{\parallel,i}}{I_i}
\]

> 注意：此处用**理想几何 \(t_i\)**做投影，不要用工程里 [omni_velocity_ctrl.c:51-65](../application/chassis/omni_velocity_ctrl.c#L51-L65) 标定出来的 H —— 那个 H 是把命令单位（cmd）下的车体速度映射到电机端 deg/s 速度的**运动学**矩阵，含命令单位标度，不是力学方向矩阵。

---

# 2. 标定产物

最终输出 8 个数（每轮正负方向各一个），写入 `OmniVelocityCtrlConfig_s`（建议新增字段）：

| 字段 | 含义 | 单位 |
|---|---|---|
| `force_to_current_pos[4]` | \(F_i\ge 0\) 时的 \(1/K_i^+\) | cmd/N |
| `force_to_current_neg[4]` | \(F_i< 0\) 时的 \(1/K_i^-\) | cmd/N |
| `current_offset[4]` | 静摩擦补偿（DJI 命令零偏） | cmd |

如果实测正负方向偏差 \(< 5\%\)，可以合并成单方向系数 `force_to_current[4]`。

辅助产物：

- 每个轮子的死区 / 静摩擦阈值 \(I_{0,i}\)（cmd），对应「车体加速度刚抬起来」那个电流点。
- 有效电流量程 \(I_{lim,i}\)（cmd），对应「再加电流加速度不再线性增长」的饱和点。

---

# 3. 前置准备

## 3.1 场地与机械

- 平整水泥 / 环氧地坪，**5 m × 5 m 净空**。粗糙地面、裂缝、灰尘会增大不一致性。
- 车上拆掉 / 关掉所有非底盘的子系统（拨弹、摩擦轮空载、上位机不发指令）。
- 电池电压 ≥ 23 V，整机重量记录到 0.5 kg 精度（**\(m\) 是后续公式核心常数**）。
- 使用**底盘固定 IMU**采集加速度；该 IMU 与四个轮子保持相对静止，坐标系只需按安装矩阵固定到底盘系。云台 IMU 不参与本实验。
- 记录底盘 IMU 相对底盘几何中心 / 质心的偏移 \(r_{imu}=[x,y]\)。若偏心小于 1~2 cm 可先忽略；否则按 §6.5 做离线修正。

## 3.2 IMU 准备

工程当前通过 `extern INS *INS` 读取 BMI088；标定前必须确认该指针绑定的是底盘固定 IMU，而不是云台姿态 IMU。建议：

1. **六轴零漂标定**：车体静止，连续采集 20000 组 gyro/acc 数据（约 20 s），得到 chassis/gimbal 分开的六轴零偏。
2. **INS 输出确认**：当前固件已删除 raw-only 开关，INS 任务固定运行 `INS_EKF()`；PlanB 采集仍使用 `INS->INS_data.INS_gyro[] / INS_accel[]` 中已完成安装矩阵旋转和零偏补偿的六轴数据，不使用 EKF 姿态输出参与拟合。
3. **标定结果读取**：SWD 观察 `g_bmi088_cali_debug`，记录 `gyro_offset[3] / acc_offset[3]`、`sample_count`。
4. **静止输出验证**：标定后静止观察 `INS->INS_data.INS_gyro[]` 三轴均值接近 0，`INS->INS_data.INS_accel[]` 接近 \([0,0,9.81]\) m/s²。若偏差明显，先回填 [robot_params.h](../application/robot_params.h) 中对应 `BMI088_CHASSIS_PRE_CALI_*` 或 `BMI088_GIMBAL_PRE_CALI_*`。
5. **采样率确认**：INS 任务运行频率应为 1 kHz（在 `modules/imu/ins_task.c` / `application/robot_task.h` 中确认）。

## 3.3 软件改动

`ChassisRunCurrentForceCalib` 支持四轮联动和单轮脉冲两种标定入口：

1. 让 `g_chassis_force_calib_cmd.wheel_index` 字段从「保留 0xFF」变为有效：
   - `0xFF` → 现有四轮联动行为（保留向下兼容）
   - `0~3` → 仅给指定轮发 `current_cmd_fit`（含 `wheel_dir` 修正），其他三轮 `DJIMotorSetRef(0)`

2. 上位机调试结构体记录 `body_acc_x/y/z`、`body_omega_z`、EKF `body_roll/pitch/yaw/yaw_total`、`sample_time_s`、`trigger_time_s`、`pulse_elapsed_s` 和 `pulse_state`。

3. `current_cmd_fit` 从 0 跳变到非零时，STM32 端锁存 `wheel_index/current_cmd_fit`，输出 1 s 后自动四轮清零；1.25 s 是硬超时保护。

4. `g_chassis_force_calib_cmd.enable = 1` 时，标定路径优先于普通 `chassis_mode`，因此可以在底盘零力控制命令下完成开环脉冲标定。

5. 关闭 / 旁路非标定相关补偿：
   - `friction_compensation_*`（[chassis.c:308-311](../application/chassis/chassis.c#L308-L311)）必须为 0。
   - `Chassis_Follow_PID` 不参与（已通过 `chassis_mode == ZERO_FORCE` 自动旁路）。
   - 标定模式入口（`ChassisEnterCurrentForceCalib`）已切到 OPEN_LOOP，速度 PID 不工作。

> 上述改动属于**标定专用**，不影响正常控制路径。

## 3.4 安全护栏

- 单次电流命令由 STM32 自动输出 1 s 后清零，时长**严格 ≤ 1.25 s**。
- 启动前**车头朝场地中央**（不要朝墙、朝人），后退方向同样要有 ≥ 2 m 净空。
- 电池电流上限 100 A 不要打到，单次 cmd 不超过 6000（≈ 满载 1/3）。

---

# 4. 实验流程

## 4.1 单次试验步骤

| 步骤 | 动作 | 时长 |
|---|---|---|
| 1 | `g_chassis_force_calib_cmd.wheel_index = i`，`current_cmd_fit = 0`，`enable = 1`，车体静止采基线 | 1.0 s |
| 2 | 把 `current_cmd_fit` 跳变到目标值 \(I_{test}\)，STM32 自动输出 1 s 后清零 | 1.0 s |
| 3 | 观察到 `pulse_done = 1` 后，把 `current_cmd_fit = 0`，车体自由滑行至停；若结束本次记录再 `enable = 0` | ~2 s |
| 4 | 数据采集器持续记录 0~3.5 s 全段 | 总 3.5 s |

## 4.2 测试矩阵

\(4\) 轮 × \(2\) 方向 × \(3\) 电流点 = **24 次试验**：

| 轮 | 方向 | \(I_{test}\)（cmd） |
|---|---|---|
| LF (0) | + | 1500 / 2500 / 3500 |
| LF (0) | − | −1500 / −2500 / −3500 |
| RF (1) | + | 1500 / 2500 / 3500 |
| RF (1) | − | −1500 / −2500 / −3500 |
| RB (2) | + | 1500 / 2500 / 3500 |
| RB (2) | − | −1500 / −2500 / −3500 |
| LB (3) | + | 1500 / 2500 / 3500 |
| LB (3) | − | −1500 / −2500 / −3500 |

> 电流点选择依据：
> - 1500 cmd 接近静摩擦阈值（参考 [chassis.c:111-119](../application/chassis/chassis.c#L111-L119) 的 vx 2000 cmd 数据）。
> - 3500 cmd 远离 ESC 限流；1 s 脉冲仍会产生明显位移和 yaw，第一次上车建议先用 1500 cmd 确认方向和场地余量。
> - 三点取直线斜率，差分掉静摩擦零偏。

每个组合**重复 2 次**取均值（共 48 次试验），用于评估重复性 \(\sigma_K / \bar{K}\)，目标 < 8%。

## 4.3 试验间车体复位

由于单轮发力会让车产生 \(\omega_z\)（X-omni 单轮主滚动方向不过几何中心），1 s 测试结束时 \(\omega_z\) 可能明显增大。试验间需要：

1. 等待车自由滑停（≈ 2 s）。
2. 人工把车推回起始姿态（车头朝同一参考方向）。
3. 静止 1 s 让 IMU 滤波稳定后再开下一次。

如果使用云台跟随陀螺方向调试模式，**记得关闭**——会污染数据。

---

# 5. 数据采集

## 5.1 必须记录的信号（1 kHz）

STM32 端已经把必须导出的量收进同一个结构体：`g_chassis_force_calib_record`。
采 CSV 时只需要导出这个结构体的字段，不需要再单独抓 `motor_*`、`INS` 或内部 flag。

| 信号 | 来源 | 单位 | 用途 |
|---|---|---|---|
| `sample_time_s` | `g_chassis_force_calib_record` | s | 当前采样时间轴 |
| `trigger_time_s` | `g_chassis_force_calib_record` | s | 脉冲触发时间轴 |
| `calib_active` | `g_chassis_force_calib_record` | — | 标记标定开环入口是否激活 |
| `pulse_state / pulse_done / hard_timeout` | `g_chassis_force_calib_record` | — | 激活段、安全清零状态 |
| `wheel_index` | `g_chassis_force_calib_record` | — | 区分试验 |
| `current_cmd_fit` | `g_chassis_force_calib_record` | cmd | 锁存输入 |
| `real_current_fit_lf/rf/rb/lb` | `g_chassis_force_calib_record` | cmd | 验证电机响应 |
| `body_acc_x/y/z` | `g_chassis_force_calib_record` | m/s² | **核心被测量**，raw-only 模式下经安装矩阵和零偏扣除 |
| `body_omega_z` | `g_chassis_force_calib_record` | rad/s | 离心力修正、yaw 旋转 |
| `body_roll / body_pitch / body_yaw_total` | `g_chassis_force_calib_record` | rad | yaw 一致性和姿态变化检查 |
| `wheel_speed_fit_lf/rf/rb/lb` | `g_chassis_force_calib_record` | deg/s | 验证轮端实际转速 |

工具链建议复用现有 SWD 抓取脚本：
- 模板：[tools/swd_livewatch_print.py](../tools/swd_livewatch_print.py) / [tools/swd_livewatch_plot.py](../tools/swd_livewatch_plot.py)
- 把上述变量加入 `Cortex Live Watch` 的导出列表，落盘 CSV。

## 5.2 采样率与时长

- 采样率 ≥ 200 Hz（建议 1 kHz，与控制周期一致）。
- 每次试验录 3.5 s，共 48 × 3.5 = 168 s 净录制时长。
- CSV 文件命名：`calib_planB_<日期>_<wheel>_<sign>_<I>cmd_run<n>.csv`。

---

# 6. 数据拟合

## 6.1 时窗选取

对单次试验 \([0, 3.5\text{ s}]\)：

- **基线段** \([0, 1.0]\) s：`current_cmd_fit = 0`，记底盘固定 IMU raw-only 加速度均值 \(\bar{\mathbf{a}}_{base}\)。
- **激活段** \([1.0, 1.2]\) s：`current_cmd_fit = I_{test}`。
- **拟合窗** \([1.05, 1.18]\) s（激活段中段，避开起始/收尾的电流上升下降）：取底盘固定 IMU raw-only 加速度均值 \(\bar{\mathbf{a}}_{act}\)。

净加速度：

\[
\Delta \mathbf{a} = \bar{\mathbf{a}}_{act} - \bar{\mathbf{a}}_{base}
\]

这一步同时去掉剩余 IMU 零偏、地面坡度引入的重力分量，以及静止安装误差带来的常值项。

## 6.2 单次试验 K 估计

把 \(\Delta\mathbf{a}\) 投影到该轮主滚动方向：

\[
\Delta a_{\parallel} = t_i^T \cdot \Delta \mathbf{a}
\]

单次估计：

\[
\hat{K}_i^{(test)}
=
\frac{m \cdot \Delta a_{\parallel}}{I_{test}}
\]

注意：**此估计含静摩擦**，不能直接用作最终 \(K_i\)，要用 §6.3 的差分法。

## 6.3 三电流点直线拟合（消静摩擦）

对同一轮同一方向的 3 个电流点 \(\{I_1, I_2, I_3\} = \{1500, 2500, 3500\}\)（或负）：

\[
m\cdot \Delta a_{\parallel}^{(j)}
=
K_i\cdot I_j - F_{static,i}\cdot \mathrm{sgn}(I_j),\quad j=1,2,3
\]

最小二乘拟合 \(K_i\) 和 \(F_{static,i}\)：

\[
\begin{bmatrix}\hat{K}_i\\ \hat{F}_{static,i}\end{bmatrix}
=
\left(\Phi^T \Phi\right)^{-1}\Phi^T y
\]

其中

\[
\Phi=
\begin{bmatrix}
I_1 & -\mathrm{sgn}(I_1)\\
I_2 & -\mathrm{sgn}(I_2)\\
I_3 & -\mathrm{sgn}(I_3)
\end{bmatrix},
\quad
y=m\cdot
\begin{bmatrix}\Delta a_{\parallel}^{(1)}\\ \Delta a_{\parallel}^{(2)}\\ \Delta a_{\parallel}^{(3)}\end{bmatrix}
\]

死区电流（\(F_i = 0\) 对应的 \(I_i\)）：

\[
I_{0,i}=\frac{F_{static,i}\cdot\mathrm{sgn}(I_{test})}{K_i}
\]

## 6.4 正负方向合并

得到 \(K_i^+, K_i^-, I_{0,i}^+, I_{0,i}^-\) 后判断：

- 若 \(\left|K_i^+ - K_i^-\right| / \max(K_i^+, K_i^-) < 5\%\)：合并为 \(K_i = (K_i^+ + K_i^-)/2\)，工程实现简化。
- 否则保留两份系数，控制器侧按 `f_cmd[i] >= 0 ? K_i^+ : K_i^-` 切换。

死区也是同理处理。

## 6.5 底盘 IMU 偏心修正（仅当偏心不可忽略）

底盘 IMU 与四轮保持相对静止时，不再需要云台 yaw 角度修正；但如果 IMU 不在底盘质心 / 几何中心附近，单轮激励产生的角加速度会污染 IMU 线加速度。记录 \(r_{imu}=[r_x,r_y]\)（IMU 相对质心，车体系）后，可用：

\[
a_{com,x} \approx a_{imu,x} + \alpha_z r_y,\quad
a_{com,y} \approx a_{imu,y} - \alpha_z r_x
\]

其中 \(\alpha_z=\dot{\omega}_z\)，由 `omega_z` 低通后差分得到。1 s 脉冲下 \(\omega_z\) 会积累，IMU 偏心修正必须保留；如果只想看早期近似线性段，可以在脚本里用 `--active START END` 缩短拟合窗口。

## 6.6 拟合脚本

已新增 `tools/calib_single_wheel_response_fit.py`。脚本读取 `g_chassis_force_calib_record` 导出的 CSV，用 `sample_time_s - trigger_time_s` 切窗，输出：

- 每次试验的 `delta_a_com_x / delta_a_com_y / alpha_z`
- 每次试验的 `a_x_per_cmd / a_y_per_cmd / alpha_z_per_cmd`
- 每次试验的 `k_parallel_n_per_cmd`
- 每轮每方向的线性拟合斜率

示例：

```bash
python3 tools/calib_single_wheel_response_fit.py data/calib/*.csv \
  --mass-kg 18.5 \
  --imu-rx-m 0.10 \
  --imu-ry-m -0.03 \
  --output calib_single_wheel_response_results.json
```

输出文件 `calib_single_wheel_response_results.json`，结构示例：

```json
{
  "mass_kg": 18.5,
  "imu_offset_m": {"rx": 0.10, "ry": -0.03},
  "trials": [
    {
      "wheel": "LF",
      "current_cmd": 1500.0,
      "delta_a_com_x": 0.42,
      "delta_a_com_y": -0.39,
      "alpha_z": 3.1,
      "alpha_z_per_cmd": 0.00207,
      "k_parallel_n_per_cmd": 0.0071
    }
  ],
  "groups": {
    "LF_pos": {
      "delta_a_com_x_per_cmd_fit": 0.00028,
      "delta_a_com_y_per_cmd_fit": -0.00026,
      "alpha_z_per_cmd_fit": 0.00205,
      "k_parallel_abs_n_per_cmd_mean": 0.0070
    }
  },
  "errors": {}
}
```

---

# 7. 结果回填到工程

## 7.1 数据结构扩展

在 [omni_velocity_ctrl.h](../application/chassis/omni_velocity_ctrl.h) 的 `OmniVelocityCtrlConfig_s` 添加：

```c
float force_to_cmd_pos[OMNI_WHEEL_NUM];   /* 单位 cmd/N，对应 F>=0 */
float force_to_cmd_neg[OMNI_WHEEL_NUM];   /* 单位 cmd/N，对应 F<0  */
float current_offset[OMNI_WHEEL_NUM];     /* 单位 cmd，静摩擦补偿 */
float wheel_current_limit[OMNI_WHEEL_NUM];/* 单位 cmd，单轮电流硬限幅 */
```

在 `OmniVelocityCtrlOutput_s` 添加：

```c
float wheel_current_ref[OMNI_WHEEL_NUM];  /* 单位 cmd，给 DJI 的电流命令 */
```

## 7.2 默认值

`OmniVelocityCtrl_GetDefaultConfig` 中按 `calib_planB_results.json` 实测值写：

```c
config->force_to_cmd_pos[OMNI_WHEEL_LF] = 1.0f / 0.0124f;  /* ≈ 80.6 cmd/N */
/* ... 其他 7 个同理 */
config->current_offset[OMNI_WHEEL_LF]   = 280.0f;
config->wheel_current_limit[OMNI_WHEEL_LF] = 12000.0f;
```

> 注意 `1/K = cmd/N`，与现有 `CHASSIS_FORCE_CALIB_X_CMD_PER_N = 80` 同量纲，方便横向对照。

## 7.3 输出阶段改造

在 `OmniFillOutput`（[omni_velocity_ctrl.c:339](../application/chassis/omni_velocity_ctrl.c#L339)）追加：

```c
for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
    const float f = force_cmd[wheel];
    const float k = (f >= 0.0f) ?
        ctrl->config.force_to_cmd_pos[wheel] :
        ctrl->config.force_to_cmd_neg[wheel];
    float i_cmd = k * f + ctrl->config.current_offset[wheel] * ((f >= 0.0f) ? 1.0f : -1.0f);
    i_cmd = OmniClampAbs(i_cmd, ctrl->config.wheel_current_limit[wheel]);
    output->wheel_current_ref[wheel] = ctrl->config.wheel_dir[wheel] * i_cmd;
}
```

`wheel_speed_ref` 字段保留（兼容速度环 fallback），但主控制路径切到 `wheel_current_ref`。

## 7.4 chassis.c 切换开关

新增 `g_chassis_current_mode_enable`（默认 0）：

- 0：维持现有速度环行为，发 `wheel_speed_ref` 给 `DJIMotorSetRef`。
- 1：把电机切到 `OPEN_LOOP`，发 `wheel_current_ref` 给 `DJIMotorSetRef`。

切换瞬间必须把电机参考清零、速度 PID 积分清零，避免输出突变。

---

# 8. 标定验证

## 8.1 拟合质量检查（离线，立即可做）

| 指标 | 目标 | 不达标的处理 |
|---|---|---|
| 三点直线拟合残差 RMS | < 1 N | 复测该轮该方向 |
| 同一组合两次重复性 \(\sigma/\bar{K}\) | < 8% | 检查地面、IMU 零偏、电池电压 |
| LF/RF/RB/LB 间 \(K\) 离散系数 | < 25% | 正常（差异主要来自轮胎装配 + 摩擦补偿） |
| \(I_0\)（静摩擦 cmd） | 100~500 cmd | > 800 说明轮端机械卡 |

## 8.2 装机回归（在线，标定数据回填后）

新增标定模式 `chassis_mode = CHASSIS_VERIFY_CALIB`（不在主分支，临时分支调试）：

1. **静推力一致性**：拉力计绑车，让控制器在 `chassis_mode = ZERO_FORCE` 下手动设 `force_cmd = [F_x, 0, 0] N` 直发，看拉力计读到的 \(F_x\) 与设定值偏差是否 < 10%。
2. **闭环加速度一致性**：手动给 \(\nu_{ref} = [0.5, 0, 0]\) m/s 阶跃，看 IMU \(a_x\) 上升段斜率是否符合 LQI Kp × 0.5 的预期。
3. **零空间内力检查**：对比「速度环模式」和「电流模式」下，同样跟 \(\nu_{ref}=[1.0, 0, 0]\) m/s 时的 4 轮电流总和。后者应不大于前者，否则说明 \(K_i\) 错配仍显著。

## 8.3 长跑回归

电流模式上线后，跑 10 分钟标准遥控操作：

- 检查电机温度：四轮温升差异 < 5 °C（错配会让某轮持续高电流 → 高温）。
- 检查 ESO 估出来的 \(\hat{d}\)：稳态下 \(\|\hat{d}\|\) 应小于 50% 的 \(\|W_d\|\)。如果 ESO 一直在补一个大常数扰动，说明 \(K_i\) 仍有系统偏差。

---

# 9. 风险与故障排查

## 9.1 数据无效的判据（拟合前先过一遍）

| 现象 | 原因 | 处理 |
|---|---|---|
| 基线段 \(\|\bar{\mathbf{a}}_{base}\| > 0.3\) m/s² | 地面不平 / IMU 没标好 | 换场地 / 重做 IMU 零偏 |
| 激活段 1 s 内 IMU 数据有跳变 | INS 任务被打断 | 检查 freertos 是否有同优先级任务死循环 |
| `I_meas[i]` 与 `current_cmd_fit` 偏差 > 15% | C620 限流触发 / 电池电压太低 | 减小 \(I_{test}\)，电池充满 |
| 同一组合两次试验 \(\Delta a_\parallel\) 差 > 30% | 轮胎温度变化 / 路面湿气 | 回到 §3.1 检查场地 |
| \(K_i\) 拟合出来为负 | 接线错 / `wheel_dir` 符号错 | 先用 §3 单轮缓推法核对 H 与电机方向 |

## 9.2 常见失败模式

- **车体没动**：\(I_{test}\) 低于静摩擦阈值。提高至 2500 / 3000 / 4000 重测。
- **车体加速度方向不对**：电机方向反 / wheel_dir 符号反 / 力学 \(t_i\) 投影方向错。先关掉控制器，手动单轮发 +500 cmd 看车走向哪边，对照 §1.4 的 \(t_i\) 表确认。
- **三个电流点拟合不在一条直线上**：在某点附近 ESC 进入限流区。看 `I_meas` 是否被钳住。
- **正负方向 \(K\) 差 > 20%**：很可能是 omni 自由滚柱在某一旋转方向上有黏滑，**这是真实的不对称**，要保留 \(K^+\) 和 \(K^-\) 分别建模，不能合并。
- **车体在加速过程中 \(\omega_z\) 飞走**：单轮发力本来就有 M_z，这是预期的；但若 \(\omega_z > 2\) rad/s 很快出现，说明 \(I_{test}\) 选大了（或场地附着低），降低电流重测。

## 9.3 整机质量 \(m\) 估计错的代价

\(K_i\) 与 \(m\) 严格成正比。如果整机质量误差 10%，则全部 \(K_i\) 同比例偏，但**所有轮等比偏不影响零空间内力**——只影响整体加速度增益。回填后会被 LQI Kp 吸收。所以 \(m\) 测到 ±0.5 kg 即可。

## 9.4 误用云台 IMU 的影响

本方案默认使用底盘固定 IMU。如果误用云台 IMU，云台 yaw/pitch 的相对运动会把加速度方向和角加速度项一起带入数据，单轮 \(K_i\) 会系统偏。采集前必须用 SWD 确认 `INS` 指向底盘固定 BMI088，并确认 `INS->INS_data.INS_accel[]` 随底盘静止/运动而不是随云台相对运动变化。

---

# 10. 数据表模板

试验前打印此表，每次试验填写：

```
日期：______ 操作者：______ 整机质量 m = ______ kg
电池开始电压：______ V    场地：______
IMU 零漂采样时长：20 s；零漂记录文件 / 截图：______
IMU 静止输出（10 s 均值）：gyro = [___, ___, ___] rad/s；acc = [___, ___, ___] m/s²

试验编号 | wheel | 方向 | I_test (cmd) | 时间戳 | CSV 文件名 | 备注
---------+-------+------+--------------+--------+-----------+------
   1     |  LF   |  +   |    1500      |  ...   |  ...      |
   2     |  LF   |  +   |    2500      |  ...   |  ...      |
   3     |  LF   |  +   |    3500      |  ...   |  ...      |
   4     |  LF   |  −   |   −1500      |  ...   |  ...      |
  ...    |  ...  | ...  |    ...       |  ...   |  ...      |
   24    |  LB   |  −   |   −3500      |  ...   |  ...      |
重复     |  LF   |  +   |    1500      |  ...   |  ...      | (二次)
  ...

电池结束电压：______ V
异常事件记录：______________________
```

---

# 11. 完成判据

下列**全部成立**才算标定通过，可进入 §7 回填工程：

- [ ] 24 + 24 = 48 次试验数据全部采到、CSV 落盘。
- [ ] 每个 `(轮, 方向)` 组合的三点直线拟合残差 RMS < 1 N。
- [ ] 每个 `(轮, 方向)` 组合两次重复性 \(\sigma/\bar{K}\) < 8%。
- [ ] 4 个轮的 \(K_i\) 数据落在合理量级（参考整车 80 cmd/N → 单轮 \(K\) 大约 0.012 N/cmd 上下）。
- [ ] 静摩擦 \(I_0\) 落在 100~500 cmd 之间。
- [ ] §8.2 的拉力计静态校核（如可做）误差 < 10%。

---

# 12. 后续工作

本手册产出的 \(K_i^{\pm}\) 是后续阶段 1（电流通道接通）→ 阶段 2（LQI anti-windup）→ 阶段 3（约束分配三合一）的**唯一依赖参数**。标定通过后，按部署路线图（见上层规划文档）进入阶段 1 实施。

如果发现标定无法收敛（重复性永远不达标、地面太烂等），退回方案 A 改进版（多轮联动 + 拉力计），不要硬着头皮上。
