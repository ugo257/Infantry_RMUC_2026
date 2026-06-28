# Yaw 自瞄 Coulomb 前馈 CSV 分析

日期：2026-05-14

## 数据来源

| 文件 | `g_yaw_tau_coulomb_nm` | 行数 | 有效行 | 运动段行 | 导出时长 | Ozone 采样步进 |
|---|---:|---:|---:|---:|---:|---:|
| `/Users/lamda/ozone_csv/0nm.csv` | 0.0 Nm | 2419 | 2398 | 2354 | 12.09 s | 5 ms |
| `/Users/lamda/ozone_csv/0.1nm.csv` | 0.1 Nm | 3397 | 3362 | 3305 | 16.98 s | 5 ms |
| `/Users/lamda/ozone_csv/0.2nm.csv` | 0.2 Nm | 1237 | 1224 | 1199 | 12.36 s | 10 ms |

统计口径：

- 有效行：`valid == 1 && feedback_ok == 1 && nuc_mode == version_control`
- 运动段：`|omega_ref_planned_rad_s| > 0.2` 或 `|omega_meas_rad_s| > 0.2`
- 角度误差：`e_theta_rad = theta_meas_rad - theta_ref_planned_rad`
- `0.2nm.csv` 的采样步进和导出时长与前两组不同，因此只作为趋势对比，不作为严格同窗口 A/B。

## 核心对比

| `g_yaw_tau_coulomb_nm` | 角度误差 RMS | 角度误差 MAE | 角度误差 P95 | 速度误差 RMS | `tau_cmd` RMS | `tau_cmd` P95 | `limit_active` 比例 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.0 Nm | 1.417 deg | 1.159 deg | 2.691 deg | 0.839 rad/s | 2.953 Nm | 5.925 Nm | 0.88% |
| 0.1 Nm | 1.507 deg | 1.232 deg | 2.870 deg | 0.893 rad/s | 3.057 Nm | 6.247 Nm | 1.78% |
| 0.2 Nm | 1.614 deg | 1.358 deg | 2.976 deg | 0.920 rad/s | 3.187 Nm | 6.423 Nm | 2.29% |

结论：

- `g_yaw_tau_coulomb_nm = 0.0f` 当前最好。
- 0.1 Nm 和 0.2 Nm 没有降低角度/速度误差，反而增加力矩 RMS 和接近饱和比例。
- 粗略互相关估计三组 `theta_ref_planned -> theta_meas` 都仍有约 10 ms 滞后，Coulomb 前馈没有解决主要相位问题。

## 关键观察

1. 三组 `traj_stale_active` 都为 0，说明这几段数据里通信/包过期不是误差来源。
2. 三组 `tau_eso_comp_nm` 全部为 0，即使 `auto_eso_comp_active` 接近 99%，当前 ESO 补偿没有实际注入力矩。
3. `g_yaw_tau_coulomb_nm = 0.2f` 在换向/低速附近更差，低速换向段 MAE 明显增大。
4. 当前 `g_yaw_coulomb_smooth_omega = 0.05f` 很小，运动段大部分时间 `tanh(omega_ref / smooth)` 已接近正负 1，Coulomb 前馈基本变成按 `omega_ref` 符号切换的常值力矩。
5. `tau_cmd` 的 P95 在 0 Nm 时已经到 5.925 Nm，距离 7 Nm 上限不远；继续加常值前馈会优先吃掉力矩余量。

## 建议

1. 暂时保持 `g_yaw_tau_coulomb_nm = 0.0f`，不要继续向 0.1/0.2 Nm 增大。
2. 如果仍要验证摩擦补偿，只测试小值：`0.03f`、`0.05f`。
3. 若开启 Coulomb 前馈，同时把 `g_yaw_coulomb_smooth_omega` 放大到 `0.2f ~ 0.4f`，避免换向时变成硬切力矩。
4. 下一轮优先固定 `g_yaw_tau_coulomb_nm = 0.0f`，扫 `g_traj_lookahead_s = 0.012f / 0.015f / 0.018f`，观察相位滞后和力矩 P95。
5. 暂时不要先加 `g_lqr_auto_k_theta` 或 `g_lqr_auto_k_omega`。当前力矩余量有限，直接加增益很可能先增加限幅比例，而不是稳定减少滞后。

推荐下一组实验参数：

```c
g_yaw_tau_coulomb_nm = 0.0f;
g_traj_lookahead_s = 0.015f; // 若无明显超调，再试 0.018f
```

