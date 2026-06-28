# Yaw 自瞄轨迹规划增强 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 yaw 自瞄模式下，把 PC 下发的离散 (θ, ω, α) 升级为 1kHz 连续匀加速外推 + α 一阶 LPF，缓解 3Hz 跟踪场景下的相位滞后；保留 `g_traj_planner_enable` 运行时回退开关。

**Architecture:** 在 `modules/lqr_eso/` 内部修改：扩展 `YawLQRESO_s` 状态、新增可调全局参数、新增 `RefPlanner_UpdateAutoAimTrajectory` 函数、在 `YawLQRESO_Calc` 内按 `g_traj_planner_enable` 分支调用新/旧规划器。`gimbal.c`、`robot_cmd.c`、PC 协议、下游 LQR/ESO 全部不动。

**Tech Stack:** STM32F407 + FreeRTOS + HAL + CMake + STM32CubeCLT 交叉工具链。无主机单元测试框架，验证靠 **编译通过 + SWD Live Watch**。

**设计文档参考：** [docs/superpowers/specs/2026-05-13-yaw-auto-aim-trajectory-planner-design.md](../specs/2026-05-13-yaw-auto-aim-trajectory-planner-design.md)

---

## 准备工作

仓库当前 **无任何 commit**，但有 ~3200 个已 staged 文件作为 baseline。在开始之前：

```sh
cd /Users/lamda/Embedded_Software/Njust_DZX/Infantry_RMUC_2026
git status | head -3
# 应显示 "On branch main / No commits yet"
git commit -m "Initial baseline commit"
# 把已 staged 的 3218 个 baseline 文件锁成 root commit
```

之后每个 task 都用 `git add <specific files>` 只提交本任务改动。

**Build verify 命令**（每个 task 最后都会跑）：

```sh
cd /Users/lamda/Embedded_Software/Njust_DZX/Infantry_RMUC_2026
cmake --build --preset Debug -j
```

预期：无报错、无警告新增、`build/Debug/Infantry_RMUC_2026.elf` 时间戳更新。

---

## Task 1: 扩展 YawLQRESO_s 结构体字段

**Files:**
- Modify: `modules/lqr_eso/lqr_eso.h:61-74`

**说明：** 在 `YawLQRESO_s` 结构体内添加 9 个新字段。`YawLQRESO_Init` 已经通过 `memset(ctrl, 0, sizeof(*ctrl))` 把整个结构体清零，因此新字段自动初始化为 0，符合"首次进入时未初始化"语义。

- [ ] **Step 1: 打开 `lqr_eso.h`，定位到 `YawLQRESO_s` 结构体**

Read `modules/lqr_eso/lqr_eso.h:61-74`，应看到：

```c
typedef struct
{
    YawEso_t eso;
    float theta_ref_planned_rad;
    float omega_ref_planned;
    float alpha_ref_planned;
    float eso_u_last;
    float torque_cmd_last;
    float tau_bias;
    float tau_meas_filt;
    float lqi_int;
    uint8_t feedback_ready;
    uint8_t limit_active;
} YawLQRESO_s;
```

- [ ] **Step 2: 在 `limit_active` 之后、`} YawLQRESO_s;` 之前插入 9 个新字段**

把整个结构体替换为：

```c
typedef struct
{
    YawEso_t eso;
    float theta_ref_planned_rad;
    float omega_ref_planned;
    float alpha_ref_planned;
    float eso_u_last;
    float torque_cmd_last;
    float tau_bias;
    float tau_meas_filt;
    float lqi_int;
    uint8_t feedback_ready;
    uint8_t limit_active;
    /* Auto-aim trajectory extrapolation state */
    float traj_latched_theta_rad;
    float traj_latched_omega_rad_s;
    float traj_latched_alpha_rad_s2;
    float traj_alpha_filt_rad_s2;
    float traj_dt_since_latch_s;
    float traj_last_input_theta_rad;
    float traj_last_input_omega_rad_s;
    float traj_last_input_alpha_rad_s2;
    uint8_t traj_initialized;
} YawLQRESO_s;
```

- [ ] **Step 3: 构建验证**

```sh
cd /Users/lamda/Embedded_Software/Njust_DZX/Infantry_RMUC_2026
cmake --build --preset Debug -j
```

Expected: 编译成功，无新警告。结构体变大但无引用变化。

- [ ] **Step 4: Commit**

```sh
git add modules/lqr_eso/lqr_eso.h
git commit -m "lqr_eso: add yaw auto-aim trajectory extrapolation state fields"
```

---

## Task 2: 添加 4 个可调全局参数（extern + definition + 默认宏）

**Files:**
- Modify: `modules/lqr_eso/lqr_eso.h:174-235`（extern declarations 区）
- Modify: `modules/lqr_eso/lqr_eso.c:21-77`（默认值宏区）
- Modify: `modules/lqr_eso/lqr_eso.c:78-139`（global definitions 区）

**说明：** 4 个参数都用 `volatile` 暴露给 SWD Live Watch，运行时可调。`g_traj_planner_enable` 是回退总开关。

- [ ] **Step 1: 在 `lqr_eso.h` 的 extern 块尾部插入 4 个 extern 声明**

定位 `lqr_eso.h:235`（`extern volatile float g_pitch_eso_tau_comp;` 这一行），在它之后插入：

```c
/* Auto-aim trajectory planner tunables (live-watchable) */
extern volatile uint8_t g_traj_planner_enable;
extern volatile float g_traj_alpha_lpf_rc_s;
extern volatile float g_traj_lookahead_s;
extern volatile float g_traj_change_eps_rad;
```

- [ ] **Step 2: 在 `lqr_eso.c` 默认值宏区尾部插入 4 个默认值宏**

定位 `lqr_eso.c:77`（`#define PITCH_STATE_MEASURE 7U` 这一行），在它之后插入：

```c
#define TRAJ_PLANNER_ENABLE_DEFAULT         1U
#define TRAJ_ALPHA_LPF_RC_DEFAULT           0.0f
#define TRAJ_LOOKAHEAD_DEFAULT              0.0f
#define TRAJ_CHANGE_EPS_RAD_DEFAULT         1e-6f
```

- [ ] **Step 3: 在 `lqr_eso.c` 全局变量定义区尾部插入 4 个变量定义**

定位 `lqr_eso.c:139`（`volatile float g_pitch_eso_tau_comp = 0.0f;` 这一行），在它之后插入：

```c
volatile uint8_t g_traj_planner_enable      = TRAJ_PLANNER_ENABLE_DEFAULT;
volatile float g_traj_alpha_lpf_rc_s        = TRAJ_ALPHA_LPF_RC_DEFAULT;
volatile float g_traj_lookahead_s           = TRAJ_LOOKAHEAD_DEFAULT;
volatile float g_traj_change_eps_rad        = TRAJ_CHANGE_EPS_RAD_DEFAULT;
```

- [ ] **Step 4: 构建验证**

```sh
cmake --build --preset Debug -j
```

Expected: 编译成功，无未定义引用（因为 4 个变量都同时声明+定义了）。

- [ ] **Step 5: Commit**

```sh
git add modules/lqr_eso/lqr_eso.h modules/lqr_eso/lqr_eso.c
git commit -m "lqr_eso: add auto-aim trajectory planner tunables (enable + LPF)"
```

---

## Task 3: 实现 `RefPlanner_UpdateAutoAimTrajectory`

**Files:**
- Modify: `modules/lqr_eso/lqr_eso.c`（在 `RefPlanner_UpdateTrajectory` 之后、`YawLQRESO_Init` 之前插入新函数）

**说明：** 新函数包含运动学外推、变化检测、α LPF、可选 lookahead、限幅 5 个段落，全部按设计文档 §5.5 实现。**保留** 原有 `RefPlanner_UpdateTrajectory` 不删（回退路径会调用它）。

- [ ] **Step 1: 定位插入点**

Read `modules/lqr_eso/lqr_eso.c:421-441` 验证 `RefPlanner_UpdateTrajectory` 函数体结尾（最后一个 `}` 在 line 441），下一行是空行，再下一行是 `void YawLQRESO_Init(YawLQRESO_s *ctrl)`。新函数插在这两者之间。

- [ ] **Step 2: 在 `RefPlanner_UpdateTrajectory` 结束 `}` 之后、`void YawLQRESO_Init(` 之前插入新函数**

```c
static void RefPlanner_UpdateAutoAimTrajectory(YawLQRESO_s *ctrl,
                                               float theta_in_rad,
                                               float omega_in_rad_s,
                                               float alpha_in_rad_s2,
                                               float dt)
{
    float omega_limit;
    float change_eps;
    float rc;
    float lookahead_s;
    float theta_out;
    float omega_out;
    float alpha_out;
    uint8_t changed;

    if (ctrl == NULL) {
        return;
    }

    change_eps    = LQRESOAbs(g_traj_change_eps_rad);
    rc            = (g_traj_alpha_lpf_rc_s > 0.0f) ? g_traj_alpha_lpf_rc_s : 0.0f;
    lookahead_s   = (g_traj_lookahead_s > 0.0f) ? g_traj_lookahead_s : 0.0f;
    omega_limit   = LQRESOAbs(g_ref_omega_limit_rad_s);

    /* (1) First entry: latch directly, prime LPF with input alpha. */
    if (ctrl->traj_initialized == 0U) {
        ctrl->traj_latched_theta_rad     = theta_in_rad;
        ctrl->traj_latched_omega_rad_s   = omega_in_rad_s;
        ctrl->traj_latched_alpha_rad_s2  = alpha_in_rad_s2;
        ctrl->traj_alpha_filt_rad_s2     = alpha_in_rad_s2;
        ctrl->traj_dt_since_latch_s      = 0.0f;
        ctrl->traj_last_input_theta_rad  = theta_in_rad;
        ctrl->traj_last_input_omega_rad_s = omega_in_rad_s;
        ctrl->traj_last_input_alpha_rad_s2 = alpha_in_rad_s2;
        ctrl->traj_initialized           = 1U;

        theta_out = theta_in_rad;
        omega_out = omega_in_rad_s;
        alpha_out = alpha_in_rad_s2;
    } else {
        /* (2) Change detection vs last firmware tick. */
        changed = (LQRESOAbs(theta_in_rad - ctrl->traj_last_input_theta_rad) > change_eps) ||
                  (LQRESOAbs(omega_in_rad_s - ctrl->traj_last_input_omega_rad_s) > change_eps) ||
                  (LQRESOAbs(alpha_in_rad_s2 - ctrl->traj_last_input_alpha_rad_s2) > change_eps);
        ctrl->traj_last_input_theta_rad   = theta_in_rad;
        ctrl->traj_last_input_omega_rad_s = omega_in_rad_s;
        ctrl->traj_last_input_alpha_rad_s2 = alpha_in_rad_s2;

        if (changed != 0U) {
            /* (3a) New PC packet: snap (theta, omega), LPF on alpha (over inter-packet dt). */
            const float dt_pkt = ctrl->traj_dt_since_latch_s;
            const float beta_denom = rc + dt_pkt;
            float beta;
            if (beta_denom > 1e-9f) {
                beta = dt_pkt / beta_denom;
                if (beta < 0.0f) {
                    beta = 0.0f;
                }
                if (beta > 1.0f) {
                    beta = 1.0f;
                }
            } else {
                beta = 0.0f;
            }
            ctrl->traj_alpha_filt_rad_s2 +=
                beta * (alpha_in_rad_s2 - ctrl->traj_alpha_filt_rad_s2);
            ctrl->traj_latched_theta_rad    = theta_in_rad;
            ctrl->traj_latched_omega_rad_s  = omega_in_rad_s;
            ctrl->traj_latched_alpha_rad_s2 = alpha_in_rad_s2;
            ctrl->traj_dt_since_latch_s     = 0.0f;
        } else {
            /* (3b) No new packet: accumulate time. */
            ctrl->traj_dt_since_latch_s += dt;
        }

        /* (4) Compute current output. */
        {
            const float tau = ctrl->traj_dt_since_latch_s;
            const float w_l = ctrl->traj_latched_omega_rad_s;
            const float a_f = ctrl->traj_alpha_filt_rad_s2;
            const float th_l = ctrl->traj_latched_theta_rad;

            theta_out = th_l + w_l * tau + 0.5f * a_f * tau * tau;
            omega_out = w_l + a_f * tau;
            alpha_out = a_f;
        }
    }

    /* (5) Optional lookahead. */
    if (lookahead_s > 0.0f) {
        const float L = lookahead_s;
        theta_out += omega_out * L + 0.5f * alpha_out * L * L;
        omega_out += alpha_out * L;
    }

    /* (6) Clamping (matches the legacy RefPlanner_UpdateTrajectory). */
    if (omega_limit > 0.0f) {
        omega_out = LQRESOClamp(omega_out, -omega_limit, omega_limit);
    }
    alpha_out = LQRESOClampAbs(alpha_out, REF_ALPHA_LIMIT_RAD_S2_DEFAULT);

    ctrl->theta_ref_planned_rad = theta_out;
    ctrl->omega_ref_planned     = omega_out;
    ctrl->alpha_ref_planned     = alpha_out;
}
```

- [ ] **Step 3: 构建验证**

```sh
cmake --build --preset Debug -j
```

Expected: 编译成功。新函数被 `static` 修饰且暂无调用方，编译器可能给 "defined but not used" 警告 —— **预期警告**，下一个 task 会消除。如果是 `-Werror` 失败，临时把 `static` 去掉即可。

- [ ] **Step 4: 检查是否有"未使用"警告**

```sh
cmake --build --preset Debug -j 2>&1 | grep -i "RefPlanner_UpdateAutoAimTrajectory"
```

Expected: 可能出现 "defined but not used"。若工具链严格则需要先临时去掉 `static` 让它链接出去，但目前的 CMakePresets 通常不开 `-Werror`，照理只是 warning，可以接受。

- [ ] **Step 5: Commit**

```sh
git add modules/lqr_eso/lqr_eso.c
git commit -m "lqr_eso: implement RefPlanner_UpdateAutoAimTrajectory (extrapolation + alpha LPF)"
```

---

## Task 4: 在 `YawLQRESO_Reset` 中清新增字段

**Files:**
- Modify: `modules/lqr_eso/lqr_eso.c:453-482`

**说明：** `YawLQRESO_Reset` 在多处被调用（首次进入、feedback 丢失、模式切换）。新增 9 个字段全部清零，确保 `traj_initialized=0` 让下次自瞄重新走"首次锁存"分支。

- [ ] **Step 1: 定位 `YawLQRESO_Reset` 函数尾部**

Read `modules/lqr_eso/lqr_eso.c:453-482`，函数体最后几行是：

```c
    g_yaw_eso_tau_comp           = 0.0f;
    g_yaw_auto_eso_comp_active   = 0.0f;
    YawPublishObserver(ctrl);
}
```

在 `g_yaw_auto_eso_comp_active = 0.0f;` 之后、`YawPublishObserver(ctrl);` 之前插入清零逻辑。

- [ ] **Step 2: 应用 Edit**

把以下两行：

```c
    g_yaw_auto_eso_comp_active   = 0.0f;
    YawPublishObserver(ctrl);
```

替换为：

```c
    g_yaw_auto_eso_comp_active   = 0.0f;
    ctrl->traj_latched_theta_rad      = 0.0f;
    ctrl->traj_latched_omega_rad_s    = 0.0f;
    ctrl->traj_latched_alpha_rad_s2   = 0.0f;
    ctrl->traj_alpha_filt_rad_s2      = 0.0f;
    ctrl->traj_dt_since_latch_s       = 0.0f;
    ctrl->traj_last_input_theta_rad   = 0.0f;
    ctrl->traj_last_input_omega_rad_s = 0.0f;
    ctrl->traj_last_input_alpha_rad_s2 = 0.0f;
    ctrl->traj_initialized            = 0U;
    YawPublishObserver(ctrl);
```

- [ ] **Step 3: 构建验证**

```sh
cmake --build --preset Debug -j
```

Expected: 编译成功。

- [ ] **Step 4: Commit**

```sh
git add modules/lqr_eso/lqr_eso.c
git commit -m "lqr_eso: clear auto-aim trajectory state in YawLQRESO_Reset"
```

---

## Task 5: 在 `YawLQRESO_Calc` 内按 `g_traj_planner_enable` 分支调用

**Files:**
- Modify: `modules/lqr_eso/lqr_eso.c:561-568`

**说明：** 这是真正切换行为的地方。`g_traj_planner_enable=1`（默认）走新外推器；=0 走 legacy 直接覆盖；`ref_traj_valid=0`（手动模式）保持原行为。注意进手动时清 `traj_initialized` 让下次回自瞄重新初始化。

- [ ] **Step 1: 定位调用点**

Read `modules/lqr_eso/lqr_eso.c:560-570` 应看到：

```c
    if (input->ref_traj_valid != 0U) {
        RefPlanner_UpdateTrajectory(ctrl,
                                    theta_target_rad,
                                    input->omega_ref_rad_s,
                                    input->alpha_ref_rad_s2);
    } else {
        RefPlanner_Update(ctrl, theta_target_rad, dt);
    }
```

- [ ] **Step 2: 替换为分支版本**

把上面 9 行替换为：

```c
    if (input->ref_traj_valid != 0U) {
        if (g_traj_planner_enable != 0U) {
            RefPlanner_UpdateAutoAimTrajectory(ctrl,
                                               theta_target_rad,
                                               input->omega_ref_rad_s,
                                               input->alpha_ref_rad_s2,
                                               dt);
        } else {
            if (ctrl->traj_initialized != 0U) {
                ctrl->traj_initialized = 0U;
            }
            RefPlanner_UpdateTrajectory(ctrl,
                                        theta_target_rad,
                                        input->omega_ref_rad_s,
                                        input->alpha_ref_rad_s2);
        }
    } else {
        if (ctrl->traj_initialized != 0U) {
            ctrl->traj_initialized = 0U;
        }
        RefPlanner_Update(ctrl, theta_target_rad, dt);
    }
```

- [ ] **Step 3: 构建验证**

```sh
cmake --build --preset Debug -j
```

Expected: 编译成功。Task 3 的 "defined but not used" 警告应消失（新函数现在被调用了）。

- [ ] **Step 4: 确认产物**

```sh
ls -la build/Debug/Infantry_RMUC_2026.elf build/Debug/Infantry_RMUC_2026.hex build/Debug/Infantry_RMUC_2026.bin
```

Expected: 三个文件存在，时间戳是本次构建。

- [ ] **Step 5: Commit**

```sh
git add modules/lqr_eso/lqr_eso.c
git commit -m "lqr_eso: route auto-aim ref through new extrapolator (with g_traj_planner_enable rollback)"
```

---

## Task 6: 设备端验证（manual checklist）

**说明：** 无主机单元测试框架，这是上车验证步骤。每项打勾后填写实测。Live Watch 配置见 [docs/debugging.md](../../debugging.md)。

**Live Watch 监视项列表：**

```
g_traj_planner_enable
g_theta_ref_planned   // 度
g_omega_ref_planned   // rad/s
g_alpha_ref_planned   // rad/s²
g_lqr_e_theta
g_torque_cmd
yaw_motor_pos_rad
yaw_gyro_rad_s
```

- [ ] **Step 1: 烧录验证**

VS Code → Run and Debug → "Cortex Debug (OpenOCD, CMSIS-DAP, STM32F407)"。
启动后开 Live Watch，加入上面 8 个变量。

- [ ] **Step 2: 验证回退开关默认状态**

启动后 `g_traj_planner_enable` 应为 1，无任何报错。

- [ ] **Step 3: 静态目标测试 — 看 ref 平滑**

让自瞄锁定一个不动的目标。观察 `g_theta_ref_planned`：

- 改前预期：阶梯化（细小台阶）
- 改后预期：连续光滑

Pass criteria: `g_theta_ref_planned` 在静止目标下变化幅度更小且无可见 staircase。

- [ ] **Step 4: 3Hz 振荡目标 — 看相位差**

复测原"自瞄跟踪振荡 3Hz 目标"场景（蓝/绿曲线那张图的工况）。

录一段 `g_theta_ref_planned`（蓝）和 `yaw_motor_pos_rad`（绿）。

Pass criteria: 改前 30-50ms 相位滞后；改后应显著缩小（目标 <20ms）。如未达预期，下一步用 lookahead 微调。

- [ ] **Step 5: 无新包窗口行为确认**

自瞄锁定中保持输入不变，观察规划器不进入额外保护分支。

Pass criteria:
- `g_theta_ref_planned / g_omega_ref_planned / g_alpha_ref_planned` 继续按最后锁存轨迹外推
- 不出现额外归零行为

- [ ] **Step 6: 回退开关验证**

跑步骤 4 的振荡跟踪时，在 Live Watch 把 `g_traj_planner_enable` 改为 0。

Pass criteria: 立刻回退到原阶梯行为，无重启、无 fault。再改回 1 应立即恢复新行为，且无大幅 yaw 跳动（首次锁存自动重新初始化）。

- [ ] **Step 7: 手动↔自瞄切换**

来回切 `nuc_mode`（按 R 鼠标键或遥控开关）。

Pass criteria: 切换瞬间 yaw 无跳变、无 fault；每次回到自瞄都重新平滑跟踪。

- [ ] **Step 8: 力矩噪声对比（可选）**

录一段 `yaw_motor->measure.tor` 或 `g_torque_cmd` 在 3Hz 跟踪场景的数据，FFT 看高频段。改前 vs 改后对比，应无明显恶化。

- [ ] **Step 9: 全部通过后归档测试报告**

新建 `docs/yaw_traj_planner_test_2026-05-13.md` 记录每个 step 实测数据（相位差、外推连续性、有无回退异常）。

```sh
git add docs/yaw_traj_planner_test_2026-05-13.md
git commit -m "docs: yaw auto-aim trajectory planner on-device test report"
```

---

## 验收标准（汇总）

1. 编译无新增警告/错误
2. `g_traj_planner_enable=1` 时跟踪 3Hz 目标的相位滞后较改前显著减小
3. `g_traj_planner_enable=0` 时行为与改前完全一致（回归保险）
4. 无新包窗口不触发额外保护，仍按锁存轨迹外推
5. 手动↔自瞄切换无跳变
6. 设备端测试报告完整

## 回退手段

任何阶段如发现新外推器异常：
- **运行时**：调试器把 `g_traj_planner_enable` 改为 0
- **永久**：`git revert` Task 5 的 commit（保留 Task 1-4 的基础设施，最差也只是 unused）

## 与设计文档的对应

| 设计文档章节 | 实现 Task |
|---|---|
| §5.2 YawLQRESO_s 新增字段 | Task 1 |
| §5.3 新增全局参数 | Task 2 |
| §5.4 新增函数签名 | Task 3 |
| §5.5 算法（伪代码） | Task 3 |
| §5.6 调用点修改 | Task 5 |
| §5.7 复位规则 | Task 4 + Task 5（手动分支） |
| §5.8 Wrap-around | 无需新代码（由 `YawLQRESO_Calc` 在调用前处理） |
| §5.9 数据流不变项 | 由设计约束保证，无 task |
| §7 测试 / 验证计划 | Task 6 |
| §10 回退操作 | Task 5 + 验收标准 |
