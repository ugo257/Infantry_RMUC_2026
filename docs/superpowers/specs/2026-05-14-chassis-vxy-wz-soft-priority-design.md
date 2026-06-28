# 底盘 vxy/wz 速度分配丝滑化 设计文档

- 日期：2026-05-14
- 范围：`application/chassis/chassis.c` 中的 `ChassisApplyVxyPrioritySpeedScale()` 及其相关辅助函数、调参旋钮、状态变量
- 不在范围：`omni_velocity_ctrl.c` 内部的 LQI / ESO / 轮力分配 / 限流逻辑；命令系斜率限制（`vxy_max_accel`、`wz_max_accel`）；模式机（`CHASSIS_ROTATE` / `FOLLOW` / `HOLE` 等）逻辑

## 1. 背景

当前 [`chassis.c:563-621`](../../../application/chassis/chassis.c) 的 `ChassisApplyVxyPrioritySpeedScale()` 在底盘系 `(chassis_vx, chassis_vy, chassis_cmd_recv.wz)` 进入 omni 控制器前对它们做硬优先级缩放：

- 用 `omni_ctrl.h[w][axis]` 矩阵预测四轮速度
- 求 `(sv, sw)` 使 `sv·a[w] + sw·b[w]` 不超 `chassis_vxy_priority_wheel_speed_limit`，其中：
  - `a[w] = h[w][VX]·vx + h[w][VY]·vy_left`（平移对第 w 轮贡献）
  - `b[w] = h[w][WZ]·wz`（自旋对第 w 轮贡献）
- 求解策略：32 步线性扫描 `sw ∈ [wz_min_scale, 1]`，每个 `sw` 下解析求最大 `sv`；以"先 vxy 后 wz"的字典序作目标函数
- `sw_min_scale = 0.7` 给 wz 留 70% 保底；`wz_deadband = 1.0` 控制保底是否生效

## 2. 问题现象

操作员在 `CHASSIS_ROTATE` 模式（`chassis_vw = 8000`）下推杆平移时，体感**顿挫**：

- 推杆前段（vxy 命令小）：底盘平移响应正常
- 推杆中段（vxy 命令到达饱和阈值）：底盘平移**几乎不再增长**，但 wz 仍维持 70%–100%
- 推杆全段：底盘平移有一个"推过头就推不动"的平台段

操作员对推杆量与平移量的关系预期是单调连续的，当前实现违反这一预期。

## 3. 根因

1. **"先 vxy 后 wz"的字典序目标函数本质上非平滑**。在 `(vx_cmd, wz_cmd)` 平面上，"先把 sv 推到 1，再考虑 sw"会在饱和边界处产生一条不可微的折线 —— 越过这条折线，sv 从 1 跳到 1-ε，sw 从 1 跳到 1-δ，跃迁不连续。
2. **`sw_min_scale = 0.7` 硬底**额外引入一道阶梯：wz 缩放被强制 ≥ 0.7，越过这道下限时（进入安全降级区）发生第二次跳变。
3. **`wz_deadband` 阈值**让 sw_min 的取值在 wz 命令小幅穿越 deadband 时呈 0 ↔ 0.7 的二值切换。
4. **32 步线性扫描的离散化误差 ≈ 3%**（每步 `sw` 间隔约 0.022），在临界饱和点附近 scale 值有 1 kHz 级别的小幅抖动。
5. **scale 是纯前馈缩放**，没有任何滤波；当 scale 从 1.0 跳到 0.5 时，omni 控制器内部的 ESO / 积分器需要应对一个 50% 的输入阶跃。

## 4. 设计目标

| 目标 | 衡量 |
|---|---|
| 消除"推到一半推不动"的平台段 | 推杆量 → 底盘平移量保持单调连续 |
| 自旋 + 平移场景下 vxy 与 wz 衰减幅度相近 | 操作员体感"同步软化"，不偏废 |
| 保留轻度 vxy 偏好（默认行为不破坏战术习惯） | α=0.3 时 vxy 比 wz 多保 15%–25% |
| 消除临界点处 scale 抖动 | `sv`、`sw` 在稳态下高频分量 < 0.5% |
| 保护 omni 内环免受 scale 跃迁冲击 | LQI/ESO 在 scale 突变后 50 ms 内回稳 |
| 不破坏现有边界场景 | ZERO_FORCE / 标定 / LQI 模式行为零回归 |

## 5. 方案概览

三个组件叠加：

1. **加权二次型软优先优化器**（替代 32 步搜索）
2. **scale 一阶低通滤波**（消除抖动 + 平滑跃迁）
3. **底盘系二级斜率限制**（防御性，保护 omni 内环）

数据流：

```
chassis_cmd_recv.vx/vy/wz (命令系)
   │
   ├─ 命令系斜率限制 (vxy_max_accel, wz_max_accel)  ← 保留不动
   │
   ├─ 旋转矩阵 (offset_angle)                      ← 保留不动
   │
   ▼
chassis_vx, chassis_vy (底盘系) + chassis_cmd_recv.wz
   │
   ├─ 【新】加权二次型软优先优化器 → (sv_raw, sw_raw)
   │
   ├─ 【新】scale 一阶低通 → (sv_applied, sw_applied)
   │
   ├─ 应用 scale: vx *= sv, vy *= sv, wz *= sw
   │
   ├─ 【新】底盘系二级斜率限制
   │
   ▼
ChassisRunOmniVelocityControl(...)
```

## 6. 组件一：加权二次型软优先优化器

### 6.1 数学模型

```
minimize   J(sv, sw) = (1 - sv)² + α · (1 - sw)²
subject to |sv·a[w] + sw·b[w]| ≤ V_max,  w = 0..3
           0 ≤ sv, sw ≤ 1
```

其中：
- `a[w] = h[w][VX]·chassis_vx + h[w][VY]·(-chassis_vy)`
- `b[w] = h[w][WZ]·chassis_cmd_recv.wz`
- `V_max = chassis_vxy_priority_wheel_speed_limit`
- `α = chassis_vxy_priority_alpha`（默认 0.3）

α 的语义：
- α → 0：极强 vxy 偏好（接近现状）
- α = 0.3（默认）：偏 vxy 但不出现平台段
- α = 1：完全均匀缩放
- α > 1：偏 wz

### 6.2 几何性质

可行域是 (sv, sw) 平面上的凸多边形，由 ≤ 8 条轮速约束 + 4 条 box 约束围成。目标函数 J 是以 (1, 1) 为中心的椭圆等值线（短轴在 sv 方向，长轴在 sw 方向，α 控制椭圆形状）。

最优解一定在以下三类点中之一：
- **内点**：(1, 1) 本身（仅当全部约束都松弛时）
- **边上点**：某条约束 = 等号成立，从 (1, 1) 沿 α-加权梯度方向投影到该边
- **顶点**：两条约束同时等号成立的交点

### 6.3 算法（候选枚举法）

```
Step 0  快速返回
        计算 max_pred = max(|a[w] + b[w]|) over w
        if max_pred ≤ V_max: return (sv=1, sw=1)

Step 1  枚举边投影候选（8 条轮约束 × 各 1 个投影点）
        for w in 0..3:
            for σ in {+1, -1}:
                c = σ · V_max
                if |a[w]| > AXIS_EPS and |b[w]| > AXIS_EPS:
                    sw_star = (α·a[w]² + b[w]·(c - a[w])) / (b[w]² + α·a[w]²)
                    sv_star = (c - sw_star·b[w]) / a[w]
                elif |a[w]| > AXIS_EPS:        # b[w] ≈ 0
                    sv_star = c / a[w]; sw_star = 1
                elif |b[w]| > AXIS_EPS:        # a[w] ≈ 0
                    sw_star = c / b[w]; sv_star = 1
                else:
                    continue                  # 退化，跳过
                clip (sv_star, sw_star) to [0, 1]²
                if feasible(sv_star, sw_star):
                    push candidate with J value

Step 2  枚举轮-轮顶点候选（C(8,2) - 4 = 24 对）
        # 同一个轮的两条边方向向量相同（仅偏移不同 / 完全重合），永远不会有有限交点，
        # 因此跳过所有 w1 == w2 的对，包括同号（重合）和异号（平行）两种情况。
        for w1 in 0..3:
            for w2 in (w1+1)..3:                  # 只取 w1 < w2，避免对称重复
                for σ1 in {+1, -1}:
                    for σ2 in {+1, -1}:
                        solve 2×2 linear system:
                            sv·a[w1] + sw·b[w1] = σ1·V_max
                            sv·a[w2] + sw·b[w2] = σ2·V_max
                        if |det| < AXIS_EPS: skip (近平行，例如对角轮)
                        clip (sv, sw) to [0, 1]²
                        if feasible: push candidate

Step 3  枚举轮-box 顶点（8 × 4 = 32 对）
        for each wheel edge and each of {sv=0, sv=1, sw=0, sw=1}:
            substitute, solve 1D
            clip to [0, 1]²
            if feasible: push candidate

Step 4  枚举 box 角点
        for (sv, sw) in {(0,0), (0,1), (1,0)}:    # (1,1) 已在 Step 0 处理
            if feasible: push candidate

Step 5  返回 J 最小的可行候选
        if candidates empty: return (0, 0)        # 不可行，安全降级
```

`feasible(sv, sw)` 定义：

```
0 ≤ sv ≤ 1
0 ≤ sw ≤ 1
|sv·a[w] + sw·b[w]| ≤ V_max  for all w in 0..3
```

候选数量上界：8 + 24 + 32 + 3 = 67。在 STM32F407 @ 168 MHz 上估计 < 50 μs。Step 0 命中时（绝大多数稳态情况）只需 4 次乘加 + 4 次比较，< 1 μs。

### 6.4 数学推导附录（拉格朗日法解 sw*）

在边 `sv·a + sw·b = c` 上，目标函数 `J(sv, sw) = (1-sv)² + α(1-sw)²` 取极小。

由约束消元 `sv = (c - sw·b) / a`，代入 J 并对 sw 求导置零：

```
dJ/dsw = 2(1 - sv)·(b/a) - 2α(1 - sw) = 0
       ⇒ (1 - sv)·b = α·a·(1 - sw)
       ⇒ b - b·(c - sw·b)/a = α·a - α·a·sw
       ⇒ a·b - b·c + sw·b² = α·a² - α·a²·sw
       ⇒ sw·(b² + α·a²) = α·a² + b·(c - a)

⇒ sw* = (α·a² + b·(c - a)) / (b² + α·a²)
   sv* = (c - sw*·b) / a
```

分母 `b² + α·a²` 在 a、b 不同时为 0 且 α > 0 时严格正，无除零风险。

## 7. 组件二：scale 一阶低通

### 7.1 状态变量

```c
static float chassis_sv_lpf_state = 1.0f;
static float chassis_sw_lpf_state = 1.0f;
```

### 7.2 更新

优化器算出 `(sv_raw, sw_raw)` 后：

```c
chassis_sv_lpf_state += chassis_scale_lpf_alpha * (sv_raw - chassis_sv_lpf_state);
chassis_sw_lpf_state += chassis_scale_lpf_alpha * (sw_raw - chassis_sw_lpf_state);
```

应用到底盘系速度（**用 LPF 输出，不用 raw**）：

```c
chassis_vx          *= chassis_sv_lpf_state;
chassis_vy          *= chassis_sv_lpf_state;
chassis_cmd_recv.wz *= chassis_sw_lpf_state;
```

### 7.3 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `chassis_scale_lpf_alpha` | 0.2 | 1 kHz 下时间常数 ≈ 4 ms |

LPF 系数选取理由：
- 4 ms 时间常数：比操作员人手 reaction（≥ 100 ms）短 25 倍，不会产生体感延迟
- 比 omni 控制器内部 ESO 带宽（≥ 50 Hz，时间常数 ≤ 3 ms）略大，输入端波动不会激发 ESO 高频响应

### 7.4 不可行情况下的处理

优化器返回 (0, 0) 时（理论上仅当 V_max 配置过小才出现），仍然走 LPF：

```c
// 即便 raw 跳到 0, LPF 也会把"骤降"平滑到 4ms 时间常数内
chassis_sv_lpf_state += chassis_scale_lpf_alpha * (0.0f - chassis_sv_lpf_state);
chassis_sw_lpf_state += chassis_scale_lpf_alpha * (0.0f - chassis_sw_lpf_state);
```

这样即便发生异常配置，也不会有"瞬间清零"的冲击。

### 7.5 复位

在以下时机把两个 state 强制回 1.0（"无饱和"状态）：

- 进入 `ChassisZeroWheelRef()`（已统一管理所有底盘速度状态）

由于 `ChassisZeroWheelRef()` 已经被以下路径调用，无需额外插桩：
- `CHASSIS_ZERO_FORCE` 模式入口
- `ChassisEnterLqiCurrentMode` / `ChassisExitLqiCurrentMode`
- `ChassisEnterCurrentForceCalib` / `ChassisExitCurrentForceCalib`

## 8. 组件三：底盘系二级斜率限制

### 8.1 位置

紧跟在 `ChassisApplyVxyPrioritySpeedScale()` **之后**、`ChassisRunOmniVelocityControl()` **之前**。

### 8.2 状态变量

```c
static float chassis_vx_body_prev = 0.0f;
static float chassis_vy_body_prev = 0.0f;
static float chassis_wz_body_prev = 0.0f;
```

### 8.3 更新

```c
const float vxy_body_max_step = chassis_vxy_body_max_accel * ctrl_dt;
const float wz_body_max_step  = chassis_wz_body_max_accel  * ctrl_dt;

chassis_vx          = SlewRateLimit(chassis_vx,          chassis_vx_body_prev, vxy_body_max_step);
chassis_vy          = SlewRateLimit(chassis_vy,          chassis_vy_body_prev, vxy_body_max_step);
chassis_cmd_recv.wz = SlewRateLimit(chassis_cmd_recv.wz, chassis_wz_body_prev, wz_body_max_step);

chassis_vx_body_prev = chassis_vx;
chassis_vy_body_prev = chassis_vy;
chassis_wz_body_prev = chassis_cmd_recv.wz;
```

### 8.4 参数（设计原则：比命令系斜率更宽松，只兜底，不限性能）

| 参数 | 默认值 | 命令系对应值 | 倍率 |
|---|---|---|---|
| `chassis_vxy_body_max_accel` | 300000.0 | `vxy_max_accel = 150000.0` | 2× |
| `chassis_wz_body_max_accel` | 1000000.0 | `wz_max_accel = 500000.0` | 2× |

2× 倍率的理由：
- 命令系斜率限制操作员能感知的速度变化率
- 底盘系斜率限制旋转 + scale 突变带来的"间接"变化率
- 在自旋时旋转矩阵让命令系恒定输入变成底盘系正弦摆动，2× 倍率保证不被这一项夹死
- 同时仍能保护 scale 阶跃（典型 50% → 100% 或反之）的过冲

### 8.5 复位

在 `ChassisZeroWheelRef()` 中把三个 `*_body_prev` 都清零。

## 9. 代码改动定位

### 9.1 删除

| 位置 | 内容 |
|---|---|
| `chassis.c:50` | `#define CHASSIS_VXY_PRIORITY_SW_SEARCH_STEPS 32U` |
| `chassis.c:51` | `#define CHASSIS_VXY_PRIORITY_EPS 1.0e-4f` |
| `chassis.c:159` | `volatile float chassis_vxy_priority_wz_min_scale = 0.7f;` |
| `chassis.c:160` | `volatile float chassis_vxy_priority_wz_deadband = 1.0f;` |
| `chassis.c:458-505` | `static uint8_t ChassisFindMaxVxyScaleForWzScale(...)` |
| `chassis.c:507-517` | `static uint8_t ChassisPureWzScaleFeasible(...)` |
| `chassis.c:519-556` | `static ChassisVxyPriorityScale_s ChassisSearchVxyPriorityScale(...)` |

### 9.2 保留

| 位置 | 内容 |
|---|---|
| `chassis.c:52` | `#define CHASSIS_VXY_PRIORITY_AXIS_EPS 1.0e-6f`（新优化器仍用） |
| `chassis.c:63-68` | `ChassisVxyPriorityScale_s` 结构体，重命名为 `ChassisVxyScaleResult_s`，字段含义不变（sv、sw、feasible） |
| `chassis.c:157` | `volatile uint8_t chassis_vxy_priority_enable = 1U;`（总开关） |
| `chassis.c:158` | `volatile float chassis_vxy_priority_wheel_speed_limit = 3000.0f;`（V_max） |
| `chassis.c:439-456` | `SlewRateLimit`、`ChassisClampFloat`（工具函数） |

### 9.3 新增

**调参旋钮（volatile，便于 SWD live watch）**

```c
volatile float chassis_vxy_priority_alpha       = 0.3f;       // 软优先权重
volatile float chassis_scale_lpf_alpha          = 0.2f;       // scale LPF α
volatile float chassis_vxy_body_max_accel       = 300000.0f;  // 底盘系 vxy 二级斜率
volatile float chassis_wz_body_max_accel        = 1000000.0f; // 底盘系 wz 二级斜率
```

**静态状态变量**

```c
static float chassis_sv_lpf_state = 1.0f;
static float chassis_sw_lpf_state = 1.0f;
static float chassis_vx_body_prev = 0.0f;
static float chassis_vy_body_prev = 0.0f;
static float chassis_wz_body_prev = 0.0f;
```

**新优化器函数**

```c
static uint8_t ChassisCheckAllWheelConstraints(
    const float a[OMNI_WHEEL_NUM],
    const float b[OMNI_WHEEL_NUM],
    float sv, float sw, float Vmax);

static float ChassisEvalSoftPriorityCost(float sv, float sw, float alpha);

static ChassisVxyScaleResult_s ChassisSolveSoftPriorityScale(
    const float a[OMNI_WHEEL_NUM],
    const float b[OMNI_WHEEL_NUM],
    float Vmax,
    float alpha);
```

### 9.4 重写

| 位置 | 内容 |
|---|---|
| `chassis.c:563-621` | `ChassisApplyVxyPrioritySpeedScale()` 改写为：调用新优化器 → 更新 LPF state → 用 LPF state 缩放 vx/vy/wz |
| `chassis.c:633-645` | `ChassisZeroWheelRef()` 补：复位 sv/sw LPF state 至 1.0、复位三个 `*_body_prev` 至 0.0 |

### 9.5 插入

| 位置 | 内容 |
|---|---|
| `chassis.c:1606`（`ChassisApplyVxyPrioritySpeedScale()` 调用之后） | 底盘系二级斜率限制段（参见 §8.3） |

## 10. 调参旋钮汇总

### 10.1 新增

| 变量 | 默认 | 单位 | 说明 |
|---|---|---|---|
| `chassis_vxy_priority_alpha` | 0.3 | 无 | 软优先权重；越小越偏 vxy，1.0 = 均匀 |
| `chassis_scale_lpf_alpha` | 0.2 | 无 | scale LPF α（1 kHz 下 ≈ 4 ms 时间常数） |
| `chassis_vxy_body_max_accel` | 300000 | cmd/s | 底盘系 vxy 二级斜率 |
| `chassis_wz_body_max_accel` | 1000000 | cmd/s | 底盘系 wz 二级斜率 |

### 10.2 保留

| 变量 | 默认 | 用途 |
|---|---|---|
| `chassis_vxy_priority_enable` | 1 | 总开关，0 = 完全跳过本段 |
| `chassis_vxy_priority_wheel_speed_limit` | 3000.0 | V_max |
| `vxy_max_accel` | 150000 | 命令系 vxy 斜率（外层斜率） |
| `wz_max_accel` | 500000 | 命令系 wz 斜率（外层斜率） |

### 10.3 删除

| 变量 | 替代 |
|---|---|
| `chassis_vxy_priority_wz_min_scale` | 由 α 隐式取代 |
| `chassis_vxy_priority_wz_deadband` | 不再需要（α 给出连续过渡） |

## 11. 边界条件与异常处理

| 场景 | 处理 |
|---|---|
| `chassis_vxy_priority_enable == 0` | 直接 return，不缩放、不过 LPF、不复位（与现状一致） |
| `omni_ctrl.inited == 0` | 同上 |
| `V_max ≤ AXIS_EPS` | 同上 |
| `α ≤ 0` 或非有限 | 在优化器入口内部 clamp 到 0.01 |
| 优化器无可行解 | 返回 (0, 0)，LPF 仍正常更新（让骤降也被平滑） |
| `(1, 1)` 已可行 | Step 0 早返回，跳过候选枚举 |
| 候选 `sv*` 或 `sw*` 计算为 NaN/Inf | `feasible()` 内 `isfinite` 检查，跳过该候选 |
| `CHASSIS_ZERO_FORCE` 模式 | 走 `ChassisZeroWheelRef()`，复位所有 LPF state + body_prev |
| 标定模式切入/切出 | 已通过 `ChassisZeroWheelRef()` 统一复位 |
| LQI 模式切换 | 同上 |
| 优化器执行时间过长 | 现状预算 < 50 μs，远低于 1 ms 周期；若实测超时再考虑 active-set 优化 |

## 12. 测试与验证

### 12.1 离线单元测试（强烈推荐）

新建 `tools/vxy_priority_optimizer_test.py`：

- 在 Python 中重新实现优化器（同样的算法）
- 对若干 (a, b, V_max, α) 组合用 `scipy.optimize.minimize` 求数值解作 ground truth
- 验证 C 版本（重新实现于 Python，作为参考）与 ground truth 的最大偏差 < 1e-4
- 用例覆盖：
  - α = 0.01（极强 vxy 偏好）
  - α = 0.3（默认）
  - α = 1.0（完全均匀）
  - α = 10.0（极强 wz 偏好）
  - V_max = 0（退化）
  - a[w] 全零（无平移）
  - b[w] 全零（无旋转）

### 12.2 板上分阶段调试

**Stage 1：仅替换优化器，关闭 LPF + 二级斜率**

- 临时把 `chassis_scale_lpf_alpha` 设为 1.0（LPF 立即穿透）
- 临时把 `chassis_vxy_body_max_accel`、`chassis_wz_body_max_accel` 设为 1e9（斜率限制实际失效）
- 自旋 + 推杆，确认平台段消失
- Ozone live watch 看 `sv_raw`、`sw_raw` 是否平滑连续

**Stage 2：打开 LPF**

- 恢复 `chassis_scale_lpf_alpha = 0.2`
- 看 `sv_applied`、`sw_applied` 是否进一步平滑
- 自旋开关瞬间过渡是否柔和

**Stage 3：打开二级斜率限制**

- 恢复 `chassis_vxy_body_max_accel = 300000`、`chassis_wz_body_max_accel = 1000000`
- 操作员快速反向推杆，观察 omni 的 `nu_eso`、`dist_hat` 是否平稳
- 不应出现明显推杆延迟感（若有，分别加大两个斜率值看延迟是否消失）

**Stage 4：α 扫描**

- α ∈ {0.1, 0.3, 1.0, 3.0}
- 每档录 5 秒 CSV：自旋 + 全推杆平移
- 对比四组的轮速分布、车身轨迹、操作员体感

### 12.3 回归测试（必须）

确认下列模式行为不变：

- ✅ `CHASSIS_ZERO_FORCE`：四电机停转
- ✅ 电流-轮力标定模式（`g_chassis_force_calib_cmd.enable == 1`）：脉冲标定流程
- ✅ LQI 电流接管模式：进入/退出无电流尖峰
- ✅ `CHASSIS_FOLLOW_GIMBAL_YAW`：底盘对齐云台 yaw
- ✅ 狗洞模式（`GIMBAL_HOLE_MODE`）：底盘对齐 -67°
- ✅ 自旋模式（`CHASSIS_ROTATE`）：底盘转速接近 8000
- ✅ 反向自旋模式（`CHASSIS_REVERSE_ROTATE`）：底盘转速接近 -5000

### 12.4 调试记录扩展（可选）

在 `OmniHCalibRecord_s` 或新独立结构 `ChassisVxyPriorityRecord_s` 中新增字段：

```c
float vxy_priority_sv_raw;       // 优化器原始 sv
float vxy_priority_sw_raw;       // 优化器原始 sw
float vxy_priority_sv_applied;   // LPF 后 sv
float vxy_priority_sw_applied;   // LPF 后 sw
uint8_t vxy_priority_feasible;   // 本拍是否可行
uint8_t vxy_priority_step0_hit;  // 是否在快速路径返回
```

在 `ChassisUpdateOmniHCalibRecord()` 或类似函数中填入，方便 SWD live watch / CSV 工具直接读取。

## 13. 实施顺序（codex 应按此顺序）

每完成一步建议编译过一次，避免错误堆积难以定位。

**Step 1：参数与状态变量改造**
- 删除 §9.1 列出的常量、变量、函数声明
- 重命名 `ChassisVxyPriorityScale_s` → `ChassisVxyScaleResult_s`
- 新增 §9.3 中的 4 个调参旋钮 + 5 个静态状态变量

**Step 2：实现新优化器**
- 添加 `ChassisCheckAllWheelConstraints`、`ChassisEvalSoftPriorityCost`、`ChassisSolveSoftPriorityScale` 三个函数
- 完整实现 §6.3 的 5 步算法

**Step 3：重写 `ChassisApplyVxyPrioritySpeedScale()`**
- 调用新优化器
- 应用 LPF（§7）
- 用 LPF 输出缩放 vx/vy/wz
- 不可行时返回 (0, 0) 走 LPF 平滑

**Step 4：补 `ChassisZeroWheelRef()`**
- 复位 sv/sw LPF state 至 1.0
- 复位 `*_body_prev` 至 0.0

**Step 5：插入底盘系二级斜率限制**
- 在 `ChassisTask()` 中 `ChassisApplyVxyPrioritySpeedScale()` 调用之后插入 §8.3 的段

**Step 6（可选）：调试记录扩展**
- 按 §12.4 在记录结构中新增字段

**Step 7：板上联调**
- 按 §12.2 四阶段流程
- 完整跑 §12.3 回归测试清单

## 14. 验收标准

1. ✅ `CHASSIS_ROTATE` 模式下推杆平移，不出现"推到一半推不动"的平台段
2. ✅ 推杆量与底盘平移量保持单调正相关
3. ✅ 自旋角速度随推杆缓慢、连续下降（不出现台阶）
4. ✅ 在临界饱和状态下，`sv`、`sw` 稳态高频分量 < 0.5%
5. ✅ 退出自旋/平移瞬间，omni 内部 ESO/LQI 状态 50 ms 内回稳
6. ✅ §12.3 全部 7 项回归测试通过
7. ✅ 优化器单拍执行时间 < 50 μs（DWT 测量）
