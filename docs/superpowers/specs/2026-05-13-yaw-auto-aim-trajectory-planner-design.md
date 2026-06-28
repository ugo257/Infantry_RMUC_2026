# Yaw 自瞄轨迹规划增强 设计文档

- 日期：2026-05-13
- 范围：`modules/lqr_eso/`（yaw 自瞄模式）
- 不在范围：手动模式 ref 规划、pitch 轨迹规划、PC 协议字段、LQR/ESO 增益再调

## 1. 背景

PC 自瞄当前下发 `(yaw_ref_rad, yaw_ref_vel_rad_s, yaw_ref_acc_rad_s2)` 三元组到 firmware。当 `nuc_mode == version_control` 时，`gimbal.c` 把它们填进 `YawControlInput_t`，`ref_traj_valid=1`、`auto_aim_mode=1`，传给 `YawLQRESO_Calc`。

[`lqr_eso.c:561-568`](../../../modules/lqr_eso/lqr_eso.c) 调用 `RefPlanner_UpdateTrajectory`，该函数当前实现是**直接覆盖** `theta_ref_planned_rad / omega_ref_planned / alpha_ref_planned`，仅做 ω/α 限幅。

## 2. 问题现象

实测 `plan_yaw`（PC 下发）和 `gimbal_yaw`（实测 yaw）曲线，3Hz 左右周期目标振荡场景下：

- 实测相对下发有 30–50ms 相位滞后
- 下发曲线本身呈"阶梯化"（PC 200–500Hz，firmware 1kHz，两包之间 ref 不变）
- 振幅基本跟得上 → 反馈环工作正常，问题集中在**滞后**

## 3. 根因

1. **Ref 阶梯化** 让 `tau_ff = J·α + B·ω` 在两包之间的平段瞬时偏小（α 在平段=0），错过了"平滑前馈"的机会。LQR 反馈环要单独把这部分补回来，固有引入相位滞后。
2. **α 未滤波**，所以现有调参选择压低对 α 前馈的依赖（间接拉高 K_auto_theta=110 弥补），跟踪带宽受限。

## 4. 设计目标

| 目标 | 衡量 |
|---|---|
| 消除 ref 阶梯，输出连续 (θ, ω, α) | 看 `g_theta_ref_planned` 曲线，无 staircase |
| 显著减少 3Hz 场景下的相位滞后 | 实测 `gimbal_yaw` vs `plan_yaw` 相位差 |
| 不引入 α 噪声放大力矩抖动 | 看 `yaw_motor.tor` 高频频谱，对比改前 |
| 不动手动模式行为 | 手动模式回归测试 |

## 5. 方案

### 5.1 整体思路

把"PC 帧 → 直接覆盖"替换为：

```
PC(θ, ω, α) → 检测是否新包
  ├─ 有变化：latch (θ, ω, α)，重置时间计数器，α 进单极 LPF
  └─ 无变化：按 dt 用匀加速运动学外推 (θ, ω)
→ 经可选 lookahead 后写入 theta/omega/alpha_planned
→ 下游 LQR/ESO 不变
```

### 5.2 `YawLQRESO_s` 新增字段

```c
/* 自瞄轨迹外推状态 */
float traj_latched_theta_rad;       /* 上次新包到达时 θ */
float traj_latched_omega_rad_s;     /* 上次新包到达时 ω */
float traj_latched_alpha_rad_s2;    /* 上次新包到达时 α（未滤波） */
float traj_alpha_filt_rad_s2;       /* α 滤波后值，用于输出和外推 */
float traj_dt_since_latch_s;        /* 自上次新包累计时间 */
float traj_last_input_theta_rad;    /* 上一 tick 输入，用于变化检测 */
float traj_last_input_omega_rad_s;
float traj_last_input_alpha_rad_s2;
uint8_t traj_initialized;           /* 是否已完成初次锁存 */
```

### 5.3 新增 volatile 可调全局参数

| 名 | 默认 | 说明 |
|---|---|---|
| `g_traj_planner_enable`  | 1U     | **回退开关**：1=新外推器（默认）；0=回退到 legacy 直接覆盖 |
| `g_traj_alpha_lpf_rc_s`  | 0.0f | α 一阶 LPF 时间常数（按包间隔做，默认不额外滤波） |
| `g_traj_lookahead_s`     | 0.0f   | 输出端 lookahead（默认关，供后续按曲线调） |
| `g_traj_change_eps_rad`  | 1e-6f  | (θ/ω/α) 变化判定阈值 |

全部通过 `extern volatile` 暴露给 SWD Live Watch / 调参。**`g_traj_planner_enable` 可在运行时通过调试器切换**，不需要重新烧写。

### 5.4 新增函数

```c
static void RefPlanner_UpdateAutoAimTrajectory(YawLQRESO_s *ctrl,
                                               float theta_in_rad,
                                               float omega_in_rad_s,
                                               float alpha_in_rad_s2,
                                               float dt);
```

调用点：`YawLQRESO_Calc` 内，原本调用 `RefPlanner_UpdateTrajectory` 的位置改为调用新函数。原 `RefPlanner_UpdateTrajectory` 删除（无其他调用方）。

### 5.5 算法（伪代码）

```c
RefPlanner_UpdateAutoAimTrajectory(ctrl, θ_in, ω_in, α_in, dt):

  if (!ctrl->traj_initialized):
      // 第一次进自瞄：直接锁存
      ctrl->traj_latched_θ = θ_in
      ctrl->traj_latched_ω = ω_in
      ctrl->traj_latched_α = α_in
      ctrl->traj_α_filt    = α_in
      ctrl->traj_dt_since_latch = 0
      ctrl->traj_initialized = 1
      记录 traj_last_input
      goto OUTPUT

  // 变化检测
  changed = (|θ_in - last_θ| > eps) ||
            (|ω_in - last_ω| > eps) ||
            (|α_in - last_α| > eps)
  写回 traj_last_input

  if (changed):
      // 新包到达：snap (θ, ω)；α 进 LPF
      Δt_pkt = ctrl->traj_dt_since_latch         // 两包间隔
      β = clamp(Δt_pkt / (RC + Δt_pkt), 0, 1)
      ctrl->traj_α_filt += β * (α_in - ctrl->traj_α_filt)
      ctrl->traj_latched_θ = θ_in
      ctrl->traj_latched_ω = ω_in
      ctrl->traj_latched_α = α_in
      ctrl->traj_dt_since_latch = 0
  else:
      ctrl->traj_dt_since_latch += dt

  OUTPUT:
  τ = ctrl->traj_dt_since_latch

  θ_out = ctrl->traj_latched_θ
          + ctrl->traj_latched_ω * τ
          + 0.5 * ctrl->traj_α_filt * τ²
  ω_out = ctrl->traj_latched_ω + ctrl->traj_α_filt * τ
  α_out = ctrl->traj_α_filt

  // 可选 lookahead（默认 L=0）
  L = g_traj_lookahead_s
  if (L > 0):
      θ_out += ω_out * L + 0.5 * α_out * L²
      ω_out += α_out * L

  // 限幅，与原 RefPlanner_UpdateTrajectory 一致
  ω_out = clamp(ω_out, ±g_ref_omega_limit_rad_s)
  α_out = clampAbs(α_out, REF_ALPHA_LIMIT_RAD_S2_DEFAULT)

  ctrl->theta_ref_planned_rad = θ_out
  ctrl->omega_ref_planned     = ω_out
  ctrl->alpha_ref_planned     = α_out
```

### 5.6 调用点修改

`YawLQRESO_Calc` 内：

```c
if (input->ref_traj_valid != 0U) {
    if (g_traj_planner_enable != 0U) {
        RefPlanner_UpdateAutoAimTrajectory(ctrl,
                                           theta_target_rad,
                                           input->omega_ref_rad_s,
                                           input->alpha_ref_rad_s2,
                                           dt);
    } else {
        /* 回退路径：保留原直接覆盖行为 */
        if (ctrl->traj_initialized != 0U) {
            ctrl->traj_initialized = 0U;  /* 下次再切回新逻辑时重新初始化 */
        }
        RefPlanner_UpdateTrajectory(ctrl,
                                    theta_target_rad,
                                    input->omega_ref_rad_s,
                                    input->alpha_ref_rad_s2);
    }
} else {
    if (ctrl->traj_initialized != 0U) {
        ctrl->traj_initialized = 0U;  /* 自瞄→手动也清状态 */
    }
    RefPlanner_Update(ctrl, theta_target_rad, dt);
}
```

**保留** `RefPlanner_UpdateTrajectory` 不删除，作为回退路径调用对象。

### 5.7 复位规则

| 触发 | 行为 |
|---|---|
| `YawLQRESO_Init` | memset 已覆盖，`traj_initialized=0` |
| `YawLQRESO_Reset` | 清所有 traj_* 字段，`traj_initialized=0` |
| `feedback_ok==0` 走的 reset 路径 | 同上 |
| `ref_traj_valid==0`（自瞄→手动） | 清 `traj_initialized=0`，再进自瞄重新初始化 |
| 首次自瞄进入 | 直接锁存 PC 当前 (θ, ω, α)，`traj_α_filt = α_in`（不滤波） |

### 5.8 Wrap-around 处理

调用方 `YawLQRESO_Calc` 已对 `theta_target_rad` 做 `LQRESOWrapNearestRad(., input->theta_rad)`，规划器内部不再处理 wrap。新包到达时 `latched_θ` 接受 wrap 后值；两包之间外推 θ 在小范围内连续推进，不会触发 wrap 边界。

### 5.9 数据流不变项

- `g_theta_ref_planned / g_omega_ref_planned / g_alpha_ref_planned` 监视量含义不变
- `YawControlInput_t / YawControlOutput_t` 字段不变
- 调用方 `gimbal.c` 不动
- 下游 LQR、ESO、tau_ff、力矩 slew/bias、ident record 全部不动

## 6. 单元逻辑边界

- `traj_dt_since_latch` 最大单调累加；不做翻转保护（float 在分钟级仍精确）
- `β` 因子在 `Δt_pkt=0`（同一 tick 多次调用）时 β=0，等于不更新 α_filt；该情形物理上不会出现
- 限幅与原 `RefPlanner_UpdateTrajectory` 完全一致（`g_ref_omega_limit_rad_s` 和 `REF_ALPHA_LIMIT_RAD_S2_DEFAULT=300 rad/s²`）

## 7. 测试 / 验证计划

实机调试通过 SWD Live Watch 看以下变量：

1. **静态目标**：自瞄锁定不动目标，看 `g_theta_ref_planned` 是否光滑（无台阶）
2. **3Hz 目标振荡**：原曲线场景复测，比较 plan_yaw vs gimbal_yaw 相位差
3. **无新包窗口**：保持输入不变，看规划输出是否继续按最后锁存轨迹外推
4. **手动↔自瞄切换**：来回切，看是否平滑无跳变
5. **力矩噪声**：录一段 `yaw_motor.tor`，FFT 对比改前是否高频抖动有改善

## 8. 风险

- 若调大 α LPF RC，会引入小相位滞后；默认 `0.0f` 不额外滤波
- 若 PC 实际带宽远低于 200Hz（如 50Hz），外推 20ms 后误差累积可能可见，需要从 PC 侧更新率或 lookahead 上处理
- `g_traj_change_eps_rad=1e-6` 偏紧，可能把浮点噪声当新包；但因为 latch 仅替换数据、不重启滤波，几乎无副作用

## 9. 实施清单

1. `lqr_eso.h`：新增 `YawLQRESO_s` 字段、4 个 `extern volatile` 全局参数（含 `g_traj_planner_enable`）
2. `lqr_eso.c`：
   - 4 个全局参数定义 + 默认值宏
   - `RefPlanner_UpdateAutoAimTrajectory` 实现
   - `YawLQRESO_Calc` 调用点改为按 `g_traj_planner_enable` 分支
   - `YawLQRESO_Reset` 清新字段
   - **保留** `RefPlanner_UpdateTrajectory`（作为回退路径调用对象）
3. 构建 + Live Watch 验证

## 10. 回退操作

任何阶段如果发现新外推器有问题，**调试器把 `g_traj_planner_enable` 改为 0** 即可立即恢复原行为，无需重新烧写或重启。若验证一段时间后确认新逻辑稳定可靠，再考虑后续是否清理 legacy 路径。
