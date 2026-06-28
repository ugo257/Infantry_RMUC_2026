# 云台自瞄跟随优化:Yaw 增量前馈解耦详细设计

> 状态:设计/待评审 · 作者:协作整理 · 适用:`Infantry_RMUC_2026` 步兵云台(STM32F407 + FreeRTOS)
> 范围:仅在现有 yaw LQR+ESO 控制器上**增量**加入前馈/解耦项,默认全部关闭、可在线调参、零增益即逐位回退。**不重写控制器、不更换闭环 IMU。**

---

## 0. 目录

1. 背景与问题定义
2. 系统现状(已确认的拓扑与控制结构)
3. 物理建模与三个补偿模块的推导
4. 详细实现(数据结构、注入点、跨板传输、代码骨架)
5. 参数辨识流程(给初值 + 在线微调)
6. 分步实施与验证
7. 风险、回退与边界

---

## 1. 背景与问题定义

打自瞄时观察到两类问题:

- **(a) pitch 快速运动时 yaw 抖动/偏移** —— 俯仰加速度越大越明显。
- **(b) 指向误差随 pitch / yaw 角度增大而系统性变大** —— 小角度准,大角度偏。

机械上已知三点:① yaw 电机旋转平面可能不水平(yaw 轴相对重力倾斜);② pitch 组件在 roll 轴上约有 **3°** 安装偏移;③ pitch 加速度不为零时会对 yaw 电机产生影响。

**目标**:让 yaw 更好地跟随自瞄轨迹。约束:**增量前馈解耦**(用户选定),低风险、可在线调参、可逐项开关回退;参数用"辨识给初值 + Ozone 在线微调"获得。

---

## 2. 系统现状(已确认)

### 2.1 双板 / 双 IMU 拓扑(实现前必读 —— 与"yaw 反馈来自底座 IMU"的直觉不同)

| 项 | 事实 | 依据 |
|---|---|---|
| yaw 控制器运行板 | **底座(CHASSIS)板**,`GimbalTask_YAW() → GimbalYawControlStep()` 驱动 yaw 电机 | `application/gimbal/gimbal.c:6979`、`:4354` |
| pitch 控制器运行板 | **枪管(GIMBAL)板**,`GimbalTask_PITCH()`,持有枪管 IMU + 视觉(VCP) | `gimbal.c:6993`、`application/robot_board.h:35` |
| 两颗 IMU | 枪管板一颗(枪管 IMU)、底座板一颗(底座 IMU);各板本地 `gimbal_IMU_data` 指向各自 BMI088 | `gimbal.c:3832` |
| **yaw 主反馈** | = **枪管 IMU** 的 `Yaw_total_angle_deg`,经 RS485 主(枪管)→从(底座)回传 | 发 `robot_cmd.c:1806-1809`;收 `robot_cmd.c:1659-1660` → `gimbal.c:4184-4185` |
| 底座 IMU 用途 | **仅用于 base-omega(底盘旋转)解耦** `input->omega_base_rad_s` | `gimbal.c:4171`、`modules/lqr_eso/lqr_eso.c:375-421` |
| 视觉来源 | 相机在枪管板,NUC 经 USB-VCP 进入,`vision_yaw/pitch(+vel/acc)` | `robot_cmd.c:1353-1406`、`robot_types.h:509-516` |

**两个关键后果:**

1. **yaw 闭环其实闭在枪管 IMU 上**,而视觉 `vision_yaw` 也由枪管 IMU 定义。若 NUC 输出"枪管应有的 Euler-yaw",则 3° roll 的 `r·pitch` 偏移在**反馈侧与参考侧自动抵消**(`r` 拟合≈0,Module C 无需开);若 NUC 输出"真实目标世界方位",则残留 `≈ r·pitch` 指向误差。→ **Module C 是否启用必须先辨识**(§5 步骤 0)。
2. yaw 控制器在底座板、pitch 信息在枪管板 → **Module A / C 需把枪管 pitch 角/率/角加速度经 RS485 跨板传到底座板**(走与 `vision_pitch*` 相同的"控制方向"通道)。Module B 只用 yaw 角(底座板已有),无需跨板。

> RS485 信号表按表项顺序自动生成第二帧头(`modules/RS485/rs485_cmd.h:44-45`),**新字段必须追加到表尾,两块板必须用完全一致的表、同时重新编译并烧录**,否则协议错位。当前 `robot_board.h` 启用 `CHASSIS_BOARD`,需分别构建/烧录两个二进制。

### 2.2 yaw 控制器结构(`modules/lqr_eso/`,1 kHz,DM4310 力矩控制)

控制律(`lqr_eso.c:1611-1681`):

```
ω_rel_ref = ω_ref − ω_base                       // base-omega(底盘)解耦,唯一现存的跨轴补偿
τ_ff      = J·α_ref + B·ω_rel_ref + coulomb·tanh(ω_rel_ref/smooth)
τ_lqr     = τ_ff − Kθ·eθ − Kω·eω − ki·∫eθ
τ_eso     = −gain·z3/b0                           // 3 阶 ESO 估计集总扰动 z3(加速度量纲)
τ_cmd     = τ_lqr + τ_eso_active + τ_excite + τ_bias   // 经 DM_T_MIN/MAX 钳位、(非自瞄)软限/限斜率
```

- 3 阶 ESO:`z1,z2,z3`,`β1=3w0, β2=3w0², β3=w0³`,扰动以加速度量纲估计后除 `b0=1/J` 转回力矩补偿(`lqr_eso.c:600-636, 1641-1647`)。
- 视觉轨迹:首包锁存 + **恒加速度外推 + lookahead 前瞻** + α 一阶 LPF(`RefPlanner_UpdateAutoAimTrajectory`,`lqr_eso.c:728-837`)。
- 现存跨轴补偿**只有底盘 base-omega**;**无** pitch→yaw、**无** yaw 重力倾斜、**无**非正交运动学修正。

### 2.3 现有可复用基础(避免重复造轮子)

- `volatile float g_*` 全局量 + Ozone live-watch 调参范式(`lqr_eso.h:370-467`)。
- **pitch 扫频激励器已存在**(`g_pitch_up_chirp_*`,`gimbal.c:6348+`),Module A 辨识直接复用。
- yaw 辨识/调试记录结构 `g_yaw_auto_aim_debug_record`、`g_yaw_model_ident`(`lqr_eso.h:135-301`)。
- 离线拟合脚本范式 `tools/yaw_model_ident_fit.py`、`tools/yaw_lqreso_fit.py`;CSV 列名为带括号符号路径;venv 在 `Infantry_RMUC_2026/.venv`。
- pitch 侧重力前馈范式(`PITCH_UP_..._GRAVITY_SIN/COS/BIAS`,`robot_params.h`)—— Module B 直接对标。

---

## 3. 物理建模与三模块推导

闭环都闭在 IMU 世界系姿态上,所以**稳态指向**基本不受机械误差影响(前提:IMU 与枪管刚性固连且标定准),机械误差主要劣化**动态跟随**与少量随角度增大的指向偏差。

### 3.1 Module A — pitch 角加速度 → yaw 反作用力矩

pitch 组件装在 yaw 平台上;其角加速度 `α_pitch` 经惯性积/轴非正交在 yaw 轴产生反作用力矩。yaw 动力学加一项:

```
J_yaw·α_yaw = τ_motor − τ_load − I_couple·α_pitch
```

`I_couple`(惯性积/不对中)正是因 3° roll + 轴倾斜而非零。ω-乘积(陀螺)项在云台转速下为二阶小量,忽略。补偿:

```
τ_couple_ff = g_yaw_pitch_couple_k · α_pitch        // 与 τ_ff = J·α_ref 同一力矩空间
```

- 量纲:`[N·m·s²]·[rad/s²]=[N·m]` ✓。
- **符号无法先验确定**(取决于 IMU pitch 率符号、DM 力矩正向、`I_couple` 符号),`g_yaw_pitch_couple_k` 取有符号、默认 0,由辨识定符号。
- `α_pitch` 取**枪管 IMU 实际 pitch 率的滤波微分**(响应真实运动),而非 pitch 参考的 `α`。

### 3.2 Module B — yaw 轴不水平的重力前馈

yaw 轴相对重力倾斜 ε(方位 φ)时,重力在 yaw 轴投影出随 yaw 角变化的力矩 `τ_g(θ)=A·sin(θ−φ)`,目前被 ESO/`tau_bias` 缓慢吸收。改为前馈,可 offload ESO、减小快速 slew 的滞后:

```
τ_grav_ff = C1·sin(θ_yaw) + C2·cos(θ_yaw)          // = A·sin(θ−φ),线性两参数
```

- **用线性 sin/cos 形式**:避免 MCU 上 `atan2` 与相位绕回,OLS 即可拟合 `C1,C2`,符号自然给出。
- `θ_yaw` 用**反馈角** `input->theta_rad`(底座板已有),无需跨板。
- 这是**力矩扰动补偿**,**不**负责静态指向(那是 Module C),两者独立。

### 3.3 Module C — pitch 3° roll 非正交的运动学指向修正(条件启用)

pitch 轴理想为 body-y(水平、垂直 boresight)。带 roll 偏移 r 后轴≈`(0,1,r)`,绕它转 pitch 角 p 时,boresight 方位获得 `≈ r·p` 的分量(小角近似 `tan r·tan p ≈ r·p`)。

- **关键**:由于 yaw 已闭在枪管 IMU、视觉也用枪管 IMU,该偏移**很可能在反馈与参考侧抵消**(`r` 拟合≈0)。仅当辨识(§5 步骤 0)确认存在随 pitch 增大的残留方位误差时才启用:

```
θ_ref_eff = θ_ref_planned − clamp(g_yaw_pitch_roll_offset_r · pitch_angle, ±g_ctrl_max_error_rad)
ω_ref_eff = ω_ref_planned − g_yaw_pitch_roll_offset_r · pitch_rate
```

- 在 **RefPlanner 之后、误差计算之前**注入(`lqr_eso.c:1588-1606`)。**必须对 `corr` 单独 clamp** —— 上游 `lqr_eso.c:1533` 的限幅在 planner 之前,保护不到 planner 之后的修正。
- 需要枪管 pitch 角(同 Module A 的跨板传输)。

> 已校验:**枪管 IMU 的 pitch 闭环会自动消除 yaw 轴倾斜带来的"俯仰耦合"**(pitch 闭在重力参考的 Euler pitch 上),所以 yaw 不水平只需 Module B(力矩扰动)+ 可能的 Module C(方位),**无需**额外的运动学俯仰修正。

---

## 4. 详细实现

### 4.1 新增可调全局量(`lqr_eso.h` extern / `lqr_eso.c` 定义,默认全 0/OFF)

| 模块 | 全局量 | 类型/默认 | 含义 |
|---|---|---|---|
| A | `g_yaw_pitch_couple_enable` | u8 / 0 | 总开关 |
| A | `g_yaw_pitch_couple_k` | f / 0 | pitch 角加速度→yaw 力矩增益(有符号) |
| A | `g_yaw_pitch_couple_tau_limit_nm` | f / 0(0=不限) | 该前馈项限幅 |
| A | `g_yaw_pitch_couple_use_motor` | u8 / 0 | α 源:0=枪管 IMU 率微分,1=pitch 电机速度微分 |
| A | `g_yaw_pitch_alpha_rad_s2` | f(watch) | 当前 α 值 |
| A | `g_yaw_pitch_couple_tau_nm` | f(watch) | 当前 A 前馈力矩 |
| B | `g_yaw_grav_enable` | u8 / 0 | 总开关 |
| B | `g_yaw_grav_sin_nm` | f / 0 | C1 |
| B | `g_yaw_grav_cos_nm` | f / 0 | C2 |
| B | `g_yaw_grav_tau_nm` | f(watch) | 当前 B 前馈力矩 |
| C | `g_yaw_pitch_kinematic_enable` | u8 / 0 | 总开关 |
| C | `g_yaw_pitch_roll_offset_r` | f / 0 | roll 偏移系数 r(有符号) |
| C | `g_yaw_pitch_kinematic_corr_rad` | f(watch) | 当前参考修正量 |

`robot_params.h` 增对应 `*_DEFAULT` 宏(初值 0,辨识后填),避免源码魔数(项目约定)。

### 4.2 数据结构

`YawControlInput_t`(`lqr_eso.h:27-46`)新增:

```c
float   pitch_alpha_rad_s2;   /* 枪管 pitch 角加速度(预滤波) */
float   pitch_angle_rad;      /* 枪管 pitch 角 */
float   pitch_rate_rad_s;     /* 枪管 pitch 角速度 */
uint8_t pitch_aux_valid;      /* 上述三项是否新鲜可用 */
```

保持 `YawLQRESO_Calc` 不直接访问 IMU(维持依赖边界与可测性)。`YawAutoAimDebugRecord_s` 增 `tau_pitch_couple_nm`、`tau_grav_nm` 两个 debug 字段(置于 `tau_ff_nm` 旁)。

`Gimbal_Ctrl_Cmd_s`(`robot_types.h`)增 `pitch_meas_alpha_rad_s2 / pitch_meas_angle_rad / pitch_meas_rate_rad_s`,作为消息中心 → gimbal 任务的载体。

### 4.3 `YawLQRESO_Calc` 注入(`lqr_eso.c`)

A/B —— `τ_ff` 形成后(`:1628`):

```c
/* Module A: pitch 加速度 → yaw 反作用力矩前馈 */
float tau_pitch_couple = 0.0f;
if (g_yaw_pitch_couple_enable && input->pitch_aux_valid) {
    tau_pitch_couple = g_yaw_pitch_couple_k * input->pitch_alpha_rad_s2;
    if (g_yaw_pitch_couple_tau_limit_nm > 0.0f)
        tau_pitch_couple = LQRESOClampAbs(tau_pitch_couple, g_yaw_pitch_couple_tau_limit_nm);
}
g_yaw_pitch_couple_tau_nm = tau_pitch_couple;

/* Module B: yaw 轴不水平 重力前馈 */
float tau_grav = 0.0f;
if (g_yaw_grav_enable) {
    tau_grav = g_yaw_grav_sin_nm * sinf(input->theta_rad)
             + g_yaw_grav_cos_nm * cosf(input->theta_rad);
}
g_yaw_grav_tau_nm = tau_grav;

/* 折入 tau_lqr(放 ESO 补偿之前) */
tau_lqr = tau_ff + tau_pitch_couple + tau_grav + tau_theta + tau_omega + tau_lqi;   /* 改 :1633 */
```

> **为何放 `tau_lqr` 内**:ESO 经 `tau_for_eso`(`:1599`,由 `eso_u_last`/`tau_meas_filt` 构造)观测实际下发力矩,故前馈被正确计入,残差仍由 ESO 兜底(优雅降级)。**不要**加到 `tau_eso_active` 上。

C —— planner 输出后(`:1588-1590`)、误差计算前(`:1603`):

```c
if (g_yaw_pitch_kinematic_enable && input->pitch_aux_valid) {
    float corr = g_yaw_pitch_roll_offset_r * input->pitch_angle_rad;     /* 符号待辨识 */
    corr = LQRESOClampAbs(corr, g_ctrl_max_error_rad);                   /* 必须 clamp */
    theta_ref_rad -= corr;
    omega_ref     -= g_yaw_pitch_roll_offset_r * input->pitch_rate_rad_s;
    g_yaw_pitch_kinematic_corr_rad = corr;
} else {
    g_yaw_pitch_kinematic_corr_rad = 0.0f;
}
```

### 4.4 输入字段填充(`gimbal.c` `GimbalYawBuildControlInput`,`:4185` 后)

```c
input->pitch_aux_valid    = gimbal_cmd_recv.pitch_meas_valid;   /* 见 4.5 跨板新鲜度 */
input->pitch_alpha_rad_s2 = gimbal_cmd_recv.pitch_meas_alpha_rad_s2;
input->pitch_angle_rad    = gimbal_cmd_recv.pitch_meas_angle_rad;
input->pitch_rate_rad_s   = gimbal_cmd_recv.pitch_meas_rate_rad_s;
```

### 4.5 跨板传输(Module A / C 必需)

方向:枪管板(主)→ 底座板(从),即"控制方向"(head0=0xA1),与 `vision_pitch*` 同通道。

1. `robot_types.h` `Chassis_Ctrl_Cmd_s_uart` **表尾追加**:`float barrel_pitch_alpha; float barrel_pitch_angle; float barrel_pitch_rate;`
2. `robot_cmd.c` `robot_cmd_ctrl_signals[]` **表尾追加**对应 `ROBOT_CMD_485_SIGNAL` 行:
   - `barrel_pitch_alpha` period_ticks=1~2(A 对延迟敏感)
   - `barrel_pitch_angle / barrel_pitch_rate` period_ticks=2~5(B/C 慢、容忍延迟)
3. **枪管板** `RobotCMDTaskGimbalBoard()`(`:1739`):用本地枪管 IMU pitch 角/率 + **本地 DWT 差分 + 一阶 LPF** 得**预滤波 α**(传滤波后值,避免跨板抖动放大微分),填 `chassis_cmd_send_uart.barrel_pitch_*`。α 计算骨架:

```c
rate = gimbal_imu->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
dt   = LQRESOSanitizeDt(DWT_GetDeltaT(&s_pitch_alpha_dwt));   /* 独立计数器 */
raw  = (rate - s_pitch_rate_last) / dt;  s_pitch_rate_last = rate;
a    = dt / (g_pitch_alpha_lpf_tau_s + dt);
s_pitch_alpha_filt += a * (raw - s_pitch_alpha_filt);
```

4. **底座板** `RobotCMDTaskChassisBoard()`(`:1606`):把 `chassis_rs485_recv.barrel_pitch_*` 拷进 `gimbal_cmd_send.pitch_meas_*`,并按帧序号/超时置 `pitch_meas_valid`。

### 4.6 最小改动文件集

`lqr_eso.h`、`lqr_eso.c`、`gimbal.c`、`robot_types.h`、`robot_cmd.c`、`robot_params.h`。Module B 路径不碰 RS485。

---

## 5. 参数辨识流程(辨识给初值 + 在线微调)

复用 Ozone live-watch → CSV 导出 → Python 拟合(`.venv`)。新 `g_*` 作为独立列,先加入 watch 集。CSV 列名形如 `(gimbal_cmd_recv).yaw_measure_angle`。

### 步骤 0 —— 前置诊断(定 Module C 是否需要)
自瞄锁定一个固定目标,pitch 全行程**缓慢**扫,记录:残留方位误差(`g_lqr_e_theta` / 实际命中偏差)、`pitch_angle`、枪管 IMU yaw、底座 IMU yaw。
- 误差随 pitch **线性增长** → 启用 Module C,斜率即 `r` 初值;
- 误差≈0 → Module C 保持关闭(yaw 与视觉同帧抵消的情形)。

### 辨识 A —— `g_yaw_pitch_couple_k` 及符号
Module A 关。枪管板用现有 `g_pitch_up_chirp_*` 扫频激励 pitch。Ozone 记录:`barrel_pitch_alpha`(传过来的 α)、yaw 扰动代理(`g_yaw_eso_z3/g_yaw_eso_b0` 或 `g_yaw_auto_aim_debug_record.tau_eso_raw_nm`、`tau_bias_nm`)、`yaw_measure_gyro`、时间。
- 最小二乘 `k = <α, d> / <α, α>`(新建 `tools/yaw_pitch_couple_fit.py`,套用现有 CSV loader)。
- 设 `g_yaw_pitch_couple_k`,硬件上开启后确认 `z3/b0` 与 α 的相关性下降(增大则翻符号)。

### 辨识 B —— `C1 / C2`
Module B 关。yaw 全行程**慢**扫(低速,避开动态项)。记录 `yaw_measure_angle` 与扰动代理。
- OLS:`d ≈ C1·sin θ + C2·cos θ + C0`(C0 留给 bias 路径)。设 `g_yaw_grav_sin_nm/cos_nm`。
- 开启后确认随 yaw 角的基线扰动变平、`tau_bias` 回落≈0。

### 辨识 C —— `r`
由步骤 0 斜率给初值。开启后确认指向误差-pitch 曲线变平(变差则翻符号)。`r` 取小。

---

## 6. 分步实施与验证

每步:从 `Infantry_RMUC_2026/` 执行
```
cmake --preset Debug
cmake --build --preset Debug -j
```
对 `GIMBAL_BOARD` 与 `CHASSIS_BOARD` **各构建一份**(改 `robot_board.h` 宏),**两板同步烧录**(RS485 表必须一致)。Ozone live-watch 相应 `g_*` 与 `g_yaw_auto_aim_debug_record`。

| 步 | 内容 | 验收 | 回退 |
|---|---|---|---|
| 1 | 纯管道:加 4 个 input 字段 + 跨板 pitch 字段 + 枪管板 α 滤波器 + 两板填充 | `barrel_pitch_alpha` 静止≈0、扫频跟随、无 NaN;yaw 行为不变 | 删字段 |
| 2 | 辨识步骤 0 + 辨识 A 采集(不改代码) | 决定 C 是否需要、得 `k` 初值 | — |
| 3 | Module A 接线(默认 OFF) | k=0 无变化;设 k/enable 后 yaw 抖动下降、z3-α 相关下降 | `enable=0` |
| 4 | Module B 接线(默认 OFF) | 辨识 B、设 C1/C2、启用后角度相关扰动变平 | `enable=0` |
| 5 | Module C 接线(默认 OFF,仅当步骤 0 确认) | 设 r、启用后指向-pitch 变平 | `enable=0` |
| 6 | 联调 + 回归 | 全开微调,跑代表性交战,盯限幅标志 | 三个 enable=0 |

**量化验证**:pitch 扫频时对比开/关 A 的 yaw `e_theta` 峰值与 z3-α 相关性;yaw 慢扫对比开/关 B 的角度相关扰动;自瞄固定目标对比开/关 C 的指向误差-pitch 斜率。
**完全回退(无需重编)**:`g_yaw_pitch_couple_enable=0`、`g_yaw_grav_enable=0`、`g_yaw_pitch_kinematic_enable=0` 即逐位回到当前行为。

---

## 7. 风险、回退与边界

- **饱和 / 与 ESO·tau_bias 互搏**:A/B 共用 `DM_T_MIN/MAX` 钳位(`lqr_eso.c:1722-1730`)与(非自瞄)软限/限斜率;ESO 经 `tau_for_eso` 计入前馈。给 A 限幅 `g_yaw_pitch_couple_tau_limit_nm`,盯 `dm_limit_active`/`limit_active`。若 A 在剧烈俯仰时顶到钳位,抵消会退化 → 减小 k 或加大 α 滤波。
- **tau_bias 重复计**(B):B 生效后 `tau_bias`(`:1704`)应回落≈0;辨识 B 时避免使用 bias 已吸收的日志。
- **微分噪声**(A):枪管板预滤波(~10 ms)+ 传滤波后的 α;`dt` 须 sanitize;盯 `g_yaw_pitch_couple_tau_nm` 尖峰;必要时用 `g_yaw_pitch_couple_use_motor` 切到电机速度微分交叉验证。
- **Module C 参考跳变**:`r` 错或 pitch 噪声会直接扰动参考 → `corr` 必须 clamp 到 `g_ctrl_max_error_rad`;`r` 取小。
- **RS485 延迟与协议一致性**:`barrel_pitch_alpha` 传预滤波值;**两板必须同表、同时烧录**,新字段只追加表尾;否则协议错位(部署期最大风险)。
- **模式门控**:A/B 是物理补偿,GYRO/自瞄模式均可生效;若只想自瞄启用,按 `input->auto_aim_mode` 门控。C 在 `r=0` 时本就是 no-op。
- **NUC 侧边界**:若步骤 0 显示指向误差主要来自相机-IMU 外参或 NUC 的 yaw 约定,则属视觉端,固件 Module C 只能部分补偿,需联动 NUC 标定(本计划范围外)。
- **RS485 在 yaw 反馈环路中的延迟**:yaw 主反馈跨板回传本身带相位滞后(架构现状),本计划不改动它;若后续仍有跟随滞后,可单列评估"yaw 闭环改用本地信号融合"的更大改动。

---

*本文件为设计/评审稿。实现前请先完成 §5 步骤 0 的诊断,据此决定 Module C 的取舍与各模块符号。*
