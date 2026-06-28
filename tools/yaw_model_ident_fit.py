#!/usr/bin/env python3
"""Fit the auto-aim yaw plant from g_yaw_model_ident Ozone CSV exports."""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path


PREFIXES = (
    "g_yaw_model_ident",
    "g_yaw_auto_aim_debug_record",
)

ALIASES = {
    "time_s": ("time_s", "time", "t"),
    "valid": ("valid",),
    "running": ("running",),
    "feedback_ok": ("feedback_ok",),
    "stage_id": ("stage_id", "stage", "ident_stage"),
    "auto_aim_mode": ("auto_aim_mode",),
    "dm_limit_active": ("dm_limit_active",),
    "torque_saturated_flag": ("torque_saturated_flag", "limit_active"),
    "omega": ("omega_meas_rad_s", "motor_vel_rad_s", "yaw_gyro_rad_s"),
    "tau_ctrl": ("ctrl_tor_set_nm", "tau_cmd_nm", "tau_cmd_before_slew_nm"),
    "tau_cmd_nm": ("tau_cmd_nm",),
    "ctrl_tor_set_nm": ("ctrl_tor_set_nm",),
    "motor_tor_nm": ("motor_tor_nm",),
    "tau_excitation_nm": ("tau_excitation_nm",),
    "yaw_coulomb_smooth_omega": ("yaw_coulomb_smooth_omega",),
}

STAGE_LOW_SPEED = 1
STAGE_DYNAMIC_CHIRP = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit J/B/Coulomb friction from g_yaw_model_ident CSV data"
    )
    parser.add_argument("csv", nargs="*", type=Path, help="Ozone CSV export(s)")
    parser.add_argument(
        "--torque-source",
        choices=("auto", "ctrl_tor_set_nm", "tau_cmd_nm", "motor_tor_nm"),
        default="auto",
        help="Torque column used as model input",
    )
    parser.add_argument("--wn", type=float, default=40.0, help="Target natural frequency, rad/s")
    parser.add_argument("--zeta", type=float, default=0.85, help="Target damping ratio")
    parser.add_argument(
        "--aim-mode",
        choices=("any", "auto", "normal"),
        default="any",
        help="Filter auto_aim_mode; default accepts normal yaw and self-aim captures",
    )
    parser.add_argument(
        "--stage",
        choices=("all", "1", "2"),
        default="all",
        help="Filter g_yaw_model_ident.stage_id; default auto-fits stage 2 when stages 1+2 are present",
    )
    parser.add_argument("--smooth-omega", type=float, default=0.0, help="Coulomb tanh smoothing speed")
    parser.add_argument("--min-dt", type=float, default=0.0005)
    parser.add_argument("--max-dt", type=float, default=0.030)
    parser.add_argument("--min-abs-torque", type=float, default=0.02)
    parser.add_argument("--static-omega-gate", type=float, default=0.15)
    parser.add_argument("--keep-saturated", action="store_true")
    parser.add_argument("--write-source", type=Path, default=None)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def normalize_name(name: str) -> str:
    text = name.strip()
    for prefix_name in PREFIXES:
        for prefix in (f"({prefix_name}).", f"{prefix_name}."):
            if text.startswith(prefix):
                text = text[len(prefix):]
    text = text.replace("->", "_").replace(".", "_")
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")


def normalize_columns(fieldnames: list[str]) -> dict[str, str]:
    return {normalize_name(name): name for name in fieldnames}


def pick_column(columns: dict[str, str], key: str, required: bool = True) -> str | None:
    for alias in ALIASES[key]:
        found = columns.get(normalize_name(alias))
        if found is not None:
            return found
    if required:
        raise KeyError(key)
    return None


def parse_float(value: str | None, default: float = math.nan) -> float:
    if value is None:
        return default
    text = value.strip()
    if not text:
        return default
    try:
        return float(text)
    except ValueError:
        return default


def truth(value: float) -> bool:
    return math.isfinite(value) and abs(value) > 0.5


def stage_key(value: float) -> int:
    if not math.isfinite(value):
        return 0
    return int(round(value))


def stage_counts(rows: list[dict[str, float]]) -> dict[int, int]:
    counts: dict[int, int] = {}
    for row in rows:
        stage = stage_key(row.get("stage", 0.0))
        counts[stage] = counts.get(stage, 0) + 1
    return counts


def filter_stage(rows: list[dict[str, float]], args: argparse.Namespace) -> list[dict[str, float]]:
    if args.stage == "all":
        return rows
    wanted = int(args.stage)
    return [row for row in rows if stage_key(row.get("stage", 0.0)) == wanted]


def filter_stage_id(rows: list[dict[str, float]], stage: int) -> list[dict[str, float]]:
    return [row for row in rows if stage_key(row.get("stage", 0.0)) == stage]


def pick_torque_column(columns: dict[str, str],
                       args: argparse.Namespace,
                       stage_col: str | None) -> tuple[str, str]:
    if args.torque_source != "auto":
        return pick_column(columns, args.torque_source), args.torque_source

    if stage_col is not None:
        for key in ("motor_tor_nm", "ctrl_tor_set_nm", "tau_cmd_nm"):
            torque_col = pick_column(columns, key, required=False)
            if torque_col is not None:
                return torque_col, normalize_name(torque_col)

    torque_col = pick_column(columns, "tau_ctrl")
    return torque_col, normalize_name(torque_col or "")


def load_rows(path: Path, args: argparse.Namespace, segment: int = 0) -> tuple[list[dict[str, float]], str]:
    with path.open("r", encoding="utf-8-sig", newline="") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header")
        columns = normalize_columns(reader.fieldnames)

        time_col = pick_column(columns, "time_s")
        valid_col = pick_column(columns, "valid")
        running_col = pick_column(columns, "running")
        feedback_col = pick_column(columns, "feedback_ok")
        stage_col = pick_column(columns, "stage_id", required=False)
        torque_col, torque_name = pick_torque_column(columns, args, stage_col)
        auto_col = pick_column(columns, "auto_aim_mode", required=False)
        dm_limit_col = pick_column(columns, "dm_limit_active", required=False)
        sat_col = pick_column(columns, "torque_saturated_flag", required=False)
        omega_col = pick_column(columns, "omega")
        excite_col = pick_column(columns, "tau_excitation_nm", required=False)
        smooth_col = pick_column(columns, "yaw_coulomb_smooth_omega", required=False)

        rows: list[dict[str, float]] = []
        for row in reader:
            rows.append({
                "t": parse_float(row.get(time_col)),
                "valid": parse_float(row.get(valid_col), 0.0),
                "running": parse_float(row.get(running_col), 0.0),
                "ok": parse_float(row.get(feedback_col), 0.0),
                "stage": parse_float(row.get(stage_col), 0.0) if stage_col else 0.0,
                "segment": float(segment),
                "auto": parse_float(row.get(auto_col), 1.0) if auto_col else 1.0,
                "dm_limit": parse_float(row.get(dm_limit_col), 0.0) if dm_limit_col else 0.0,
                "sat": parse_float(row.get(sat_col), 0.0) if sat_col else 0.0,
                "omega": parse_float(row.get(omega_col)),
                "tau": parse_float(row.get(torque_col)),
                "excite": parse_float(row.get(excite_col), 0.0) if excite_col else 0.0,
                "smooth": parse_float(row.get(smooth_col), 0.0) if smooth_col else 0.0,
            })
    return rows, torque_name


def median(values: list[float]) -> float:
    if not values:
        return math.nan
    data = sorted(values)
    mid = len(data) // 2
    if len(data) % 2:
        return data[mid]
    return 0.5 * (data[mid - 1] + data[mid])


def lstsq(x_rows: list[list[float]], y_values: list[float]) -> list[float]:
    n = len(x_rows[0])
    xtx = [[0.0] * n for _ in range(n)]
    xty = [0.0] * n
    for x_row, y in zip(x_rows, y_values):
        for r in range(n):
            xty[r] += x_row[r] * y
            for c in range(n):
                xtx[r][c] += x_row[r] * x_row[c]

    a = [row[:] + [b] for row, b in zip(xtx, xty)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-15:
            raise ValueError("singular regression matrix")
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
        scale = a[col][col]
        for j in range(col, n + 1):
            a[col][j] /= scale
        for r in range(n):
            if r == col:
                continue
            factor = a[r][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                a[r][j] -= factor * a[col][j]
    return [a[i][n] for i in range(n)]


def choose_smooth(rows: list[dict[str, float]], args: argparse.Namespace) -> float:
    if args.smooth_omega > 0.0:
        return args.smooth_omega
    values = [r["smooth"] for r in rows if r.get("smooth", 0.0) > 1.0e-6]
    value = median(values)
    return value if math.isfinite(value) and value > 1.0e-6 else 0.15


def usable_rows(rows: list[dict[str, float]], args: argparse.Namespace) -> list[dict[str, float]]:
    usable = []
    for row in rows:
        if not (math.isfinite(row["t"]) and math.isfinite(row["omega"]) and math.isfinite(row["tau"])):
            continue
        if not (truth(row["valid"]) and truth(row["running"]) and truth(row["ok"])):
            continue
        if args.aim_mode == "auto" and row["auto"] != 1.0:
            continue
        if args.aim_mode == "normal" and row["auto"] != 0.0:
            continue
        if not args.keep_saturated and (truth(row["dm_limit"]) or truth(row["sat"])):
            continue
        if abs(row["tau"]) < args.min_abs_torque:
            continue
        usable.append(row)
    usable.sort(key=lambda item: (item.get("segment", 0.0), item.get("stage", 0.0), item["t"]))
    return usable


def low_speed_stats(rows: list[dict[str, float]], args: argparse.Namespace) -> dict[str, float]:
    usable = usable_rows(rows, args)
    low_speed = [
        row for row in usable
        if abs(row["omega"]) <= args.static_omega_gate and abs(row["excite"]) > 0.01
    ]
    return {
        "raw_rows": float(len(rows)),
        "usable_rows": float(len(usable)),
        "low_speed_rows": float(len(low_speed)),
        "static_pos": median([row["tau"] for row in low_speed if row["tau"] > 0.0]),
        "static_neg": median([-row["tau"] for row in low_speed if row["tau"] < 0.0]),
        "static_abs": median([abs(row["tau"]) for row in low_speed]),
    }


def fit_rows(rows: list[dict[str, float]], args: argparse.Namespace) -> dict[str, float]:
    smooth = choose_smooth(rows, args)
    usable = usable_rows(rows, args)
    x_rows: list[list[float]] = []
    y_values: list[float] = []
    dts: list[float] = []

    for a, b in zip(usable, usable[1:]):
        if stage_key(a.get("segment", 0.0)) != stage_key(b.get("segment", 0.0)):
            continue
        if stage_key(a.get("stage", 0.0)) != stage_key(b.get("stage", 0.0)):
            continue
        dt = b["t"] - a["t"]
        if dt < args.min_dt or dt > args.max_dt:
            continue
        basis = math.tanh(a["omega"] / smooth)
        x_rows.append([a["omega"], a["tau"], basis, 1.0])
        y_values.append(b["omega"])
        dts.append(dt)

    if len(y_values) < 80:
        raise ValueError(f"insufficient regression rows: {len(y_values)}")

    c0, c1, c2, c3 = lstsq(x_rows, y_values)
    dt_mean = sum(dts) / len(dts)
    if not (0.0 < c0 < 1.0):
        raise ValueError(f"invalid c0={c0:.6g}; expected 0<c0<1")
    if c1 <= 0.0:
        raise ValueError(f"invalid c1={c1:.6g}; expected positive torque gain")

    bj = -math.log(c0) / dt_mean
    yaw_b = (1.0 - c0) / c1
    yaw_j = yaw_b / bj
    coulomb = -c2 / c1
    disturbance = c3 / c1

    residuals = []
    y_mean = sum(y_values) / len(y_values)
    for x_row, y in zip(x_rows, y_values):
        residuals.append(y - sum(ci * xi for ci, xi in zip((c0, c1, c2, c3), x_row)))
    rmse = math.sqrt(sum(r * r for r in residuals) / len(residuals))
    y_var = sum((y - y_mean) ** 2 for y in y_values) / len(y_values)
    r2 = 1.0 - (rmse * rmse) / y_var if y_var > 0.0 else math.nan

    static = low_speed_stats(rows, args)

    if yaw_j <= 0.0 or yaw_b < 0.0:
        raise ValueError(f"non-physical fit: J={yaw_j:.6g}, B={yaw_b:.6g}")

    k_theta = yaw_j * args.wn * args.wn
    k_omega = max(0.0, 2.0 * args.zeta * args.wn * yaw_j - yaw_b)

    return {
        "raw_rows": float(len(rows)),
        "usable_rows": float(len(usable)),
        "fit_rows": float(len(y_values)),
        "dt_mean": dt_mean,
        "smooth": smooth,
        "c0": c0,
        "c1": c1,
        "yaw_j": yaw_j,
        "yaw_b": yaw_b,
        "coulomb": coulomb,
        "disturbance": disturbance,
        "static_pos": static["static_pos"],
        "static_neg": static["static_neg"],
        "static_abs": static["static_abs"],
        "rmse": rmse,
        "r2": r2,
        "k_theta": k_theta,
        "k_omega": k_omega,
        "wn": args.wn,
        "zeta": args.zeta,
    }


def fmt_f(value: float) -> str:
    text = f"{value:.9g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return f"{text}f"


def update_source(path: Path, values: dict[str, float]) -> None:
    replacements = {
        "YAW_AUTO_J_DEFAULT": values["yaw_j"],
        "YAW_AUTO_B_DEFAULT": values["yaw_b"],
        "YAW_AUTO_TAU_COULOMB_DEFAULT": values["coulomb"],
        "YAW_AUTO_COULOMB_SMOOTH_OMEGA_DEFAULT": values["smooth"],
        "LQR_AUTO_K_THETA_DEFAULT": values["k_theta"],
        "LQR_AUTO_K_OMEGA_DEFAULT": values["k_omega"],
    }
    text = path.read_text(encoding="utf-8")
    for name, value in replacements.items():
        pattern = rf"(^#define\s+{name}\s+)([-+0-9.eEfF]+)"
        text, count = re.subn(pattern, rf"\g<1>{fmt_f(value)}", text, count=1, flags=re.MULTILINE)
        if count != 1:
            raise ValueError(f"did not find {name} in {path}")
    path.write_text(text, encoding="utf-8")


def print_result(values: dict[str, float],
                 torque_name: str,
                 counts: dict[int, int],
                 stage_filter: str,
                 fit_stage: str,
                 stage1_stats: dict[str, float] | None = None,
                 two_stage_auto: bool = False) -> None:
    print("Yaw auto-aim model identification")
    print(f"  torque source:       {torque_name}")
    print(f"  stage filter:        {stage_filter}")
    if two_stage_auto:
        print("  fit strategy:        two-stage auto; dynamic fit uses stage 2")
    print(f"  dynamic fit stage:   {fit_stage}")
    if counts:
        parts = []
        for stage in sorted(counts):
            name = "legacy" if stage == 0 else str(stage)
            parts.append(f"{name}={counts[stage]}")
        print(f"  rows by stage:       {', '.join(parts)}")
    print(f"  dyn raw/usable/fit:  {int(values['raw_rows'])}/{int(values['usable_rows'])}/{int(values['fit_rows'])}")
    print(f"  mean dt:             {values['dt_mean'] * 1000.0:.3f} ms")
    print(f"  R^2 / RMSE omega:    {values['r2']:.6f} / {values['rmse']:.6g} rad/s")
    if stage1_stats is not None:
        print()
        print("Stage 1 low-speed check")
        print(f"  rows raw/usable/low: {int(stage1_stats['raw_rows'])}/{int(stage1_stats['usable_rows'])}/{int(stage1_stats['low_speed_rows'])}")
        print(f"  static + / - / abs:  {stage1_stats['static_pos']:.9g} / {stage1_stats['static_neg']:.9g} / {stage1_stats['static_abs']:.9g} N*m")
        if math.isfinite(stage1_stats["static_abs"]):
            delta = stage1_stats["static_abs"] - abs(values["coulomb"])
            print(f"  Fc consistency:      dynamic={abs(values['coulomb']):.9g}, delta={delta:.9g} N*m")
    print()
    print("Selected plant")
    print(f"  J:                   {values['yaw_j']:.9g} kg*m^2")
    print(f"  B:                   {values['yaw_b']:.9g} N*m*s/rad")
    print(f"  Coulomb Fc:          {values['coulomb']:.9g} N*m")
    print(f"  tanh smooth omega:   {values['smooth']:.9g} rad/s")
    print(f"  bias/disturbance:    {values['disturbance']:.9g} N*m")
    print(f"  static + / - / abs:  {values['static_pos']:.9g} / {values['static_neg']:.9g} / {values['static_abs']:.9g} N*m")
    print(f"  target wn/zeta:      {values['wn']:.9g} / {values['zeta']:.9g}")
    print()
    print("Firmware copy block")
    print(f"#define YAW_AUTO_J_DEFAULT                   {fmt_f(values['yaw_j'])}")
    print(f"#define YAW_AUTO_B_DEFAULT                   {fmt_f(values['yaw_b'])}")
    print(f"#define YAW_AUTO_TAU_COULOMB_DEFAULT         {fmt_f(values['coulomb'])}")
    print(f"#define YAW_AUTO_COULOMB_SMOOTH_OMEGA_DEFAULT {fmt_f(values['smooth'])}")
    print(f"#define LQR_AUTO_K_THETA_DEFAULT            {fmt_f(values['k_theta'])}")
    print(f"#define LQR_AUTO_K_OMEGA_DEFAULT            {fmt_f(values['k_omega'])}")
    print()
    print("Ozone assignment block")
    print(f"g_yaw_auto_j = {fmt_f(values['yaw_j'])};")
    print(f"g_yaw_auto_b = {fmt_f(values['yaw_b'])};")
    print(f"g_yaw_auto_tau_coulomb_nm = {fmt_f(values['coulomb'])};")
    print(f"g_yaw_auto_coulomb_smooth_omega = {fmt_f(values['smooth'])};")
    print(f"g_lqr_auto_k_theta = {fmt_f(values['k_theta'])};")
    print(f"g_lqr_auto_k_omega = {fmt_f(values['k_omega'])};")


def synthetic_rows() -> list[dict[str, float]]:
    yaw_j = 0.03374
    yaw_b = 0.38821
    coulomb = 0.346952
    smooth = 0.15
    bias = 0.025
    dt = 0.005
    rows: list[dict[str, float]] = []
    profiles = (
        (1, 10.0, 0.45, 0.08, 0.80),
        (2, 8.0, 0.80, 0.50, 15.0),
    )

    for segment, duration, amp, f0, f1 in profiles:
        omega = 0.0
        phase = 0.0
        sample_count = int(duration / dt)
        for i in range(sample_count):
            t = i * dt
            t_norm = min(max(t / duration, 0.0), 1.0)
            freq = f0 * math.exp(t_norm * math.log(f1 / f0))
            phase += 2.0 * math.pi * freq * dt
            tau = amp * math.sin(phase)
            a = math.exp(-(yaw_b / yaw_j) * dt)
            b = (1.0 - a) / yaw_b
            rows.append({
                "t": t,
                "valid": 1.0,
                "running": 1.0,
                "ok": 1.0,
                "stage": float(segment),
                "segment": 1.0,
                "auto": 1.0,
                "dm_limit": 0.0,
                "sat": 0.0,
                "omega": omega,
                "tau": tau,
                "excite": tau,
                "smooth": smooth,
            })
            omega = a * omega + b * (tau + bias - coulomb * math.tanh(omega / smooth))
    return rows


def main() -> int:
    args = parse_args()
    if args.self_test:
        rows = synthetic_rows()
        torque_name = "synthetic_tau"
    else:
        if not args.csv:
            raise SystemExit("CSV path required unless --self-test is used")
        rows = []
        torque_names = []
        for segment, path in enumerate(args.csv, start=1):
            loaded, torque_name = load_rows(path, args, segment=segment)
            rows.extend(loaded)
            torque_names.append(torque_name)
        torque_name = ",".join(sorted(set(torque_names)))

    counts = stage_counts(rows)
    two_stage_auto = (
        args.stage == "all" and
        counts.get(STAGE_LOW_SPEED, 0) > 0 and
        counts.get(STAGE_DYNAMIC_CHIRP, 0) > 0
    )
    stage1_stats = None
    if two_stage_auto:
        fit_input = filter_stage_id(rows, STAGE_DYNAMIC_CHIRP)
        fit_stage = "2 (auto from two-stage capture)"
        stage1_stats = low_speed_stats(filter_stage_id(rows, STAGE_LOW_SPEED), args)
    else:
        fit_input = filter_stage(rows, args)
        fit_stage = args.stage

    if not fit_input:
        raise SystemExit(f"no rows left after --stage {args.stage} filter")

    values = fit_rows(fit_input, args)
    print_result(values,
                 torque_name,
                 counts,
                 args.stage,
                 fit_stage,
                 stage1_stats=stage1_stats,
                 two_stage_auto=two_stage_auto)

    if args.write_source is not None:
        update_source(args.write_source, values)
        print()
        print(f"updated {args.write_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
