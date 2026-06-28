# Pitch Up / Down 重力前馈位置门控阶梯辨识

**日期**：2026-05-19
**主题**：替换 `DoublePitchGravityRampApply` 系列代码，改为"位置门控阶梯保持 + 上电机 phi 越界锁 0"的双电机重力前馈辨识流程；统一一个 debug 结构体供 Live Watch / CSV 导出。

---

## 1. 背景

`application/gimbal/gimbal.c` 当前的 `DoublePitchGravityRamp*` 是时间线性斜坡：两路电机扭矩在 `DEFAULT_DURATION_S = 25 s` 内同步从 0 拉到指定终值，再线性回 0；安全带由 `double_pitch_gravity_envelope[]` 给出 `(θ → ψ_low, ψ_high)` 包络。

这种实现对**稳态**重力前馈拟合不友好——扭矩、角度、速度同时变化，每个采样点都是动态过渡过程，离线拟合 `τ_g(θ, φ)` 时必须依赖动力学模型剥离惯性、Coriolis、摩擦项，误差大。

新流程目标：

- 每个 (θ, φ, τ) 数据点都是**速度近零、扭矩保持**的准静态点，离线直接做最小二乘拟合 `τ_g`。
- 上电机 phi 走到机械（或编码器）限位时只把上电机扭矩切到 0，让下电机继续推进，最大化数据采集范围。
- 一次测试覆盖整个 θ 行程到 `|τ_big| ≥ 1.95 Nm`。
- 符号位一键切换正反方向。
- 所有 debug 数据集中到一个 struct，方便 SWO/Live Watch CSV 导出。

---

## 2. 总体结构

**改动范围**：`application/gimbal/gimbal.c` 内：

- **删除**：
  - 函数 `DoublePitchGravityRampApply` / `DoublePitchGravityRampSafetyOk` / `DoublePitchGravityRampApplyAutoStart` / `DoublePitchGravityRampStop` / `DoublePitchGravityRampRequestActive` / `DoublePitchGravityRampSampleActive` / `DoublePitchGravityRampClampTorque` / `DoublePitchGravityRampGetPsiLimits`
  - 全局变量 `g_double_pitch_gravity_ramp_*`（arm / auto_start / running / phase / duration_s / return_s / tau_big_end_nm / tau_small_end_nm / tau_limit_nm）
  - 常量 `DOUBLE_PITCH_GRAVITY_RAMP_*`（phase 枚举、默认 duration / return / torque / max_torque / omega_limit / psi_margin）
  - 类型 `DoublePitchGravityEnvelopePoint_s` 及表 `double_pitch_gravity_envelope[]`
- **新增**：
  - 函数 `DoublePitchGravityStepApply` / `DoublePitchGravityStepSafetyOk` / `DoublePitchGravityStepStop` / `DoublePitchGravityStepRequestActive` / `DoublePitchGravityStepSampleActive` / `DoublePitchGravityStepAdvance`
  - 全局状态、配置参数、符号位、debug struct（详见 §3 §4 §5）
- **改名**：`DOUBLE_PITCH_IDENT_AXIS_GRAVITY_RAMP` → `DOUBLE_PITCH_IDENT_AXIS_GRAVITY_STEP`，同步更新 `DoublePitchIdentUpdateDebug` 内引用。

不改的：现有 `DoublePitchIdentDebug_s`、`DoublePitchIdentNormalModeActive`、`DoublePitchIdentPublishRefs` 等共用工具继续沿用，只是 `DoublePitchGravityStepApply` 替换原 `DoublePitchGravityRampApply` 的调用点。

---

## 3. 控制流程

### 状态机

```text
IDLE ─arm=1─▶ STEPPING ──|Δθ|≥step_size──▶ SETTLING ─速度稳定且
                                            settle_timer≥settle_time─▶ STEPPING(下一步)
                                                                       │
                                                                       ▼
                                            |τ_big_cmd|≥1.95 / 安全失败 ──▶ DONE
```

`phase ∈ {0=IDLE, 1=STEPPING, 2=SETTLING, 3=DONE}`。`safety_ok=0` 任何时候触发 → `DONE` + 输出 0。

### 单次 tick 算法

```text
on tick (dt_s, input):
  if !arm && !running: return  // 未启动
  if !feedback_ok:    Stop(DONE); return
  if !SafetyOk:       Stop(DONE); return  // ω/θ/phi 硬安全

  // 启动一次性初始化
  if running==0:
    step_index = 0
    tau_big_cmd  = sign * dtau_big          // 第 1 步直接到 Δτ，非 0
    tau_small_cmd = sign * dtau_small
    theta_step_start = θ_now
    phase = STEPPING; running = 1; settle_timer = 0; phi_locked_this_step = 0

  switch phase:
    STEPPING:
      if |θ - theta_step_start| >= step_size_rad:
        phase = SETTLING; settle_timer = 0

    SETTLING:
      if |ω_big|<ω_settle && |ω_small|<ω_settle:
        settle_timer += dt
        if settle_timer >= settle_time_s:
          snapshot_valid_this_tick = 1     // 本 tick 的 debug 行记录的是刚刚保持完的这一步
          // ↑ 关键：debug 结构体在 tick 末尾整体填入，必须在 step_index++ /
          //   tau_big_cmd += / theta_step_start = 之前 latch 出 snapshot 行的字段值。
          //   实现上：先把当前 (step_index, tau_big_cmd, tau_small_cmd, theta_step_start)
          //   写到 debug 局部副本，再做下面的递增。
          step_index++
          tau_big_cmd  += sign * dtau_big
          tau_small_cmd += sign * dtau_small
          theta_step_start = θ_now
          phi_locked_this_step = 0          // 新一步重新解锁 up
          phase = STEPPING
          if |tau_big_cmd| >= tau_big_target_nm:
            tau_big_target_reached = 1
            Stop(DONE); tau_big_motor=0; tau_small_motor=0
            return
      else:
        settle_timer = 0  // 速度抖动重新计时

  // phi 越界处理（每 tick 独立判定，对当前 step 内 sticky）
  phi_in_bounds = (phi_min ≤ phi ≤ phi_max)
  if !phi_in_bounds: phi_locked_this_step = 1
  tau_small_motor = phi_locked_this_step ? 0 : tau_small_cmd
  tau_big_motor   = tau_big_cmd

  // 硬限幅（防呆）
  clamp(tau_big_motor,  ±DOUBLE_PITCH_DM_T_LIMIT_NM)
  clamp(tau_small_motor, ±DOUBLE_PITCH_DM_T_LIMIT_NM)
```

**phi 锁 0 语义**（用户原话："在刚刚的 down 扭矩下不给扭矩，等 down 继续加扭矩直到角度稳定后，up 电机继续给扭矩"）：

- `phi_locked_this_step` 一旦在 STEPPING/SETTLING 期间被置 1，整个本步 up 电机输出恒 0。
- 切换到下一步（snapshot 完成）时 `phi_locked_this_step = 0` 复位，下一步 up 又以新的 `tau_small_cmd` 起跑。
- 这样如果 down 推进让 phi 又回到带内，up 在新步里会重新尝试给扭矩；若仍越界则同样锁本步。

### 安全检查 `SafetyOk`

任一条件不满足返回 0：

- `|ω_big| ≤ ω_safety_limit_rad_s`
- `|ω_small| ≤ ω_safety_limit_rad_s`
- `theta_safe_low_rad ≤ θ ≤ theta_safe_high_rad`
- `phi_safe_low_rad ≤ phi ≤ phi_safe_high_rad`（比 `phi_min/phi_max` 更宽，作为硬保护）

---

## 4. 配置与运行控制

全部为 `volatile`，方便 Live Watch 改值。默认值如下，单位 `Nm`/`rad`/`rad·s⁻¹`/`s`：

```c
/* 流程参数 */
volatile float    g_grav_step_size_rad             = 0.30f;   /* 0.2-0.5 */
volatile float    g_grav_step_dtau_big_nm          = 0.20f;
volatile float    g_grav_step_dtau_small_nm        = 0.10f;
volatile float    g_grav_step_tau_big_target_nm    = 1.95f;
volatile float    g_grav_step_phi_min_rad          = -3.00f;  /* CSV 推算，初值，再调 */
volatile float    g_grav_step_phi_max_rad          =  2.60f;
volatile float    g_grav_step_theta_safe_low_rad   = -0.10f;
volatile float    g_grav_step_theta_safe_high_rad  =  2.20f;
volatile float    g_grav_step_phi_safe_low_rad     = -3.14f;
volatile float    g_grav_step_phi_safe_high_rad    =  3.14f;
volatile float    g_grav_step_omega_settle_rad_s   = 0.05f;
volatile float    g_grav_step_settle_time_s        = 0.50f;
volatile float    g_grav_step_omega_safety_limit_rad_s = 0.80f;

/* 启停 */
volatile int8_t   g_grav_step_sign                 = +1;      /* +1 / -1，user-set */
volatile uint8_t  g_grav_step_arm                  = 0U;      /* user-set 1 → 启动 */
volatile uint8_t  g_grav_step_running              = 0U;      /* 内部 */
volatile uint8_t  g_grav_step_phase                = 0U;      /* 内部 */
```

**启动**：用户写 `g_grav_step_sign = ±1`，再写 `g_grav_step_arm = 1`，下一个 tick `DoublePitchGravityStepApply` 看到 `arm=1 && running=0` 开始流程并把 `running=1, arm` 保留 1 直到结束。

**停止**（完成 / 安全 / 退出 normal mode）：`Stop()` 把 `arm=0, running=0, phase=DONE`，输出 0。

---

## 5. Debug 结构体

```c
typedef struct
{
    /* --- 帧头 --- */
    uint32_t sample_count;
    float    time_s;
    float    dt_s;

    /* --- 状态 / flag --- */
    uint8_t  feedback_ok;
    uint8_t  arm;
    uint8_t  running;
    uint8_t  phase;                   /* 0=IDLE 1=STEPPING 2=SETTLING 3=DONE */
    int8_t   sign;
    uint16_t step_index;
    uint8_t  safety_ok;
    uint8_t  snapshot_valid;          /* 本 tick 跨步完成（步索引++） */
    uint8_t  phi_in_bounds;
    uint8_t  phi_locked_this_step;
    uint8_t  tau_big_target_reached;
    uint8_t  tau_big_limit_flag;
    uint8_t  tau_small_limit_flag;

    /* --- 步状态簿记 --- */
    float    theta_step_start_rad;
    float    theta_step_delta_rad;
    float    settle_timer_s;

    /* --- 电机反馈 --- */
    float    theta_rad;
    float    phi_rad;
    float    psi_rad;
    float    omega_big_rad_s;
    float    omega_small_rad_s;
    float    tau_big_meas_nm;
    float    tau_small_meas_nm;
    float    motor_big_temp_c;
    float    motor_small_temp_c;
    float    motor_big_tmos_c;
    float    motor_small_tmos_c;

    /* --- 指令扭矩 --- */
    float    tau_big_cmd_nm;
    float    tau_small_cmd_nm;
    float    tau_big_motor_cmd_nm;
    float    tau_small_motor_cmd_nm;

    /* --- 配置快照 --- */
    float    cfg_step_size_rad;
    float    cfg_dtau_big_nm;
    float    cfg_dtau_small_nm;
    float    cfg_tau_big_target_nm;
    float    cfg_phi_min_rad;
    float    cfg_phi_max_rad;
    float    cfg_theta_safe_low_rad;
    float    cfg_theta_safe_high_rad;
    float    cfg_omega_settle_rad_s;
    float    cfg_settle_time_s;
    float    cfg_omega_safety_limit_rad_s;
} DoublePitchGravityStepDebug_s;

extern volatile DoublePitchGravityStepDebug_s g_double_pitch_gravity_step_debug;
```

**填充时机**：`DoublePitchGravityStepApply` 末尾一次性整体写入 `g_double_pitch_gravity_step_debug = local_debug`，保证 Live Watch 抓取的瞬间值是一致快照。

**CSV 列**：列名直接用字段名；离线 `phase` 数字 0..3 自行映射；`snapshot_valid=1` 的行即每步快照点，做拟合时 grep 这些行即可。

---

## 6. 与现有调用点的衔接

旧 `DoublePitchGravityRampApply` 在 `DoublePitchControlTask` 内的调用点（搜 `gimbal.c` 内调用）替换为 `DoublePitchGravityStepApply`，签名保持一致：

```c
uint8_t DoublePitchGravityStepApply(float dt_s,
                                    const DoublePitchLqrInput_s *input,
                                    DoublePitchLqrOutput_s *output,
                                    float *tau_big_motor_cmd_nm,
                                    float *tau_small_motor_cmd_nm,
                                    uint8_t *debug_valid);
```

返回值 1 = 本 tick 由 step 流程接管输出；0 = 未激活，让 LQR 接管。`output` 内对应字段（`tau_big_cmd_nm` / `tau_small_cmd_nm` / `tau_big_pre_limit_nm` / `tau_small_pre_limit_nm`）按旧实现习惯填好。`DoublePitchIdentUpdateDebug` 内 `axis = DOUBLE_PITCH_IDENT_AXIS_GRAVITY_STEP`。

---

## 7. 验证计划

1. **编译**：从 `Infantry_RMUC_2026/` 运行 `cmake --build --preset Debug -j`，确保无新增 warning。
2. **静态自检**：`sign=+1` + `arm=1` 时，启动前先把 down 电机停在 θ≈0 安全位置；记录第一帧 `tau_big_cmd ≈ +0.20`、`phase=1`、`step_index=0`。
3. **越限检查**：手动把 phi 拨到边界外，确认 `phi_locked_this_step` 立即 = 1，`tau_small_motor_cmd_nm = 0`，但 down 继续；下一步开始时 `phi_locked_this_step = 0` 复位。
4. **完整跑通**：跑到 `tau_big_target_reached = 1`，phase 进 DONE，两路扭矩为 0。
5. **反方向**：`sign = -1` 重复一次，确认台阶值符号取反、安全检查依旧有效。
6. **数据导出**：Live Watch 抓 `g_double_pitch_gravity_step_debug`，记录到 CSV；按 `snapshot_valid=1` 过滤行做 `τ_g` 拟合。

---

## 8. 范围外（暂不做）

- 不在本次自动用 CSV `max.csv` / `min.csv` 生成 `phi_min/phi_max` 表 —— 用户已选 "phi-only fixed bounds"，`phi_min/phi_max` 直接给运行时可调常量，CSV 仅作为初值参考。
- 不做 θ 包络回归 / 离线拟合脚本 —— 留给后续 `tools/` 下的 Python 工具，本设计范围只到固件采集端。
- 不动 LQR 控制器 `DoublePitchLqr_Calc` / 模型常量 `DOUBLE_PITCH_MODEL_*`。
