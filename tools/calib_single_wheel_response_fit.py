#!/usr/bin/env python3
"""Fit single-wheel current to chassis acceleration/yaw response.

The input CSVs are exported from g_chassis_force_calib_record. The script also
accepts Ozone Live Watch CSV headers such as
``(g_chassis_force_calib_record).body_acc_x`` and automatically strips that
prefix.

One CSV may contain multiple current pulses. Pulses are split by consecutive
rows whose pulse_state is 2 (active).
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean


T_WHEEL = {
    0: (1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0)),  # LF
    1: (1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0)),   # RF
    2: (1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0)),  # RB
    3: (1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0)),   # LB
}

WHEEL_NAME = {0: "LF", 1: "RF", 2: "RB", 3: "LB", 255: "ALL"}
WHEEL_CURRENT_FIELD = {
    0: "real_current_fit_lf",
    1: "real_current_fit_rf",
    2: "real_current_fit_rb",
    3: "real_current_fit_lb",
}
WHEEL_SPEED_FIELD = {
    0: "wheel_speed_fit_lf",
    1: "wheel_speed_fit_rf",
    2: "wheel_speed_fit_rb",
    3: "wheel_speed_fit_lb",
}


@dataclass
class PulseSegment:
    start: int
    end: int


def normalize_header(name: str) -> str:
    name = name.strip()
    if name.startswith("(") and ")." in name:
        return name.split(").", 1)[1]
    return name


def parse_float(row: dict[str, str], *names: str, default: float = 0.0) -> float:
    for name in names:
        if name in row and row[name] != "":
            return float(row[name])
    return default


def parse_int(row: dict[str, str], *names: str, default: int = 0) -> int:
    for name in names:
        if name in row and row[name] != "":
            return int(float(row[name]))
    return default


def mean_field(rows: list[dict[str, str]], *names: str) -> float:
    vals = [parse_float(row, *names) for row in rows]
    return mean(vals) if vals else 0.0


def slope_xy(xs: list[float], ys: list[float]) -> tuple[float, float]:
    if len(xs) < 2:
        return 0.0, ys[0] if ys else 0.0
    x_bar = mean(xs)
    y_bar = mean(ys)
    den = sum((x - x_bar) * (x - x_bar) for x in xs)
    if abs(den) < 1.0e-12:
        return 0.0, y_bar
    slope = sum((x - x_bar) * (y - y_bar) for x, y in zip(xs, ys)) / den
    return slope, y_bar - slope * x_bar


def rms(values: list[float]) -> float:
    return math.sqrt(mean([value * value for value in values])) if values else 0.0


def discover_files(paths: list[str]) -> list[Path]:
    files: list[Path] = []
    for item in paths:
        matches = glob.glob(item)
        candidates = matches if matches else [item]
        for candidate in candidates:
            path = Path(candidate)
            if path.is_dir():
                files.extend(sorted(path.glob("*.csv")))
            else:
                files.append(path)
    return sorted(dict.fromkeys(files))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="") as f:
        reader = csv.reader(f)
        try:
            header = [normalize_header(name) for name in next(reader)]
        except StopIteration:
            return []
        return [dict(zip(header, row)) for row in reader]


def row_time(row: dict[str, str], fallback_index: int) -> float:
    for name in ("sample_time_s", "time_s", "Time", "t", "timestamp_s"):
        if name in row and row[name] != "":
            return float(row[name])
    return fallback_index * 0.001


def find_active_segments(rows: list[dict[str, str]]) -> list[PulseSegment]:
    segments: list[PulseSegment] = []
    start: int | None = None
    for index, row in enumerate(rows):
        active = parse_int(row, "pulse_state", default=0) == 2
        if active and start is None:
            start = index
        if (not active or index == len(rows) - 1) and start is not None:
            end = index if active else index - 1
            segments.append(PulseSegment(start, end))
            start = None
    return segments


def relative_to_trigger(row: dict[str, str], trigger_time_s: float, fallback_index: int) -> float:
    return row_time(row, fallback_index) - trigger_time_s


def fit_segment(
    path: Path,
    rows: list[dict[str, str]],
    segment: PulseSegment,
    pulse_index: int,
    mass_kg: float,
    imu_rx_m: float,
    imu_ry_m: float,
    baseline_window: tuple[float, float],
    active_window: tuple[float, float],
) -> dict[str, float | int | str | bool]:
    segment_rows = rows[segment.start:segment.end + 1]
    trigger_time_s = parse_float(segment_rows[0], "trigger_time_s")
    if trigger_time_s <= 0.0:
        trigger_time_s = row_time(segment_rows[0], segment.start)

    baseline = [
        row for index, row in enumerate(rows)
        if baseline_window[0] <= relative_to_trigger(row, trigger_time_s, index) <= baseline_window[1]
    ]
    active = [
        (row, relative_to_trigger(row, trigger_time_s, segment.start + offset))
        for offset, row in enumerate(segment_rows)
        if active_window[0] <= relative_to_trigger(row, trigger_time_s, segment.start + offset) <= active_window[1]
    ]

    if len(baseline) < 10 or len(active) < 10:
        raise ValueError("not enough samples in baseline or active window")

    base_ax = mean_field(baseline, "body_acc_x", "ax")
    base_ay = mean_field(baseline, "body_acc_y", "ay")
    base_az = mean_field(baseline, "body_acc_z", "az")
    base_wz = mean_field(baseline, "body_omega_z", "omega_z", "wz")

    active_t = [t for _, t in active]
    active_wz = [parse_float(row, "body_omega_z", "omega_z", "wz") - base_wz for row, _ in active]
    alpha_z, _ = slope_xy(active_t, active_wz)

    active_rows = [row for row, _ in active]
    active_cmds = [parse_float(row, "current_cmd_fit", "I") for row in active_rows]
    active_wheels = [parse_int(row, "wheel_index", default=255) for row in active_rows]
    current_cmd = mean(active_cmds)
    wheel_index = round(mean(active_wheels))
    tx, ty = T_WHEEL.get(wheel_index, (0.0, 0.0))

    corr_ax: list[float] = []
    corr_ay: list[float] = []
    for row, wz in zip(active_rows, active_wz):
        ax = parse_float(row, "body_acc_x", "ax") - base_ax
        ay = parse_float(row, "body_acc_y", "ay") - base_ay
        corr_ax.append(ax + alpha_z * imu_ry_m + wz * wz * imu_rx_m)
        corr_ay.append(ay - alpha_z * imu_rx_m + wz * wz * imu_ry_m)

    delta_a_com_x = mean(corr_ax)
    delta_a_com_y = mean(corr_ay)
    a_parallel = tx * delta_a_com_x + ty * delta_a_com_y
    force_parallel_n = mass_kg * a_parallel
    cmd_abs = current_cmd if abs(current_cmd) > 1.0e-6 else 1.0

    current_field = WHEEL_CURRENT_FIELD.get(wheel_index, "")
    speed_field = WHEEL_SPEED_FIELD.get(wheel_index, "")
    real_current = mean_field(active_rows, current_field) if current_field else 0.0
    wheel_speed = mean_field(active_rows, speed_field) if speed_field else 0.0
    pulse_elapsed = [parse_float(row, "pulse_elapsed_s") for row in segment_rows]
    pulse_duration_s = max(pulse_elapsed) - min(pulse_elapsed) if pulse_elapsed else 0.0

    return {
        "file": str(path),
        "pulse_index": pulse_index,
        "wheel_index": wheel_index,
        "wheel": WHEEL_NAME.get(wheel_index, str(wheel_index)),
        "sign": "pos" if current_cmd >= 0.0 else "neg",
        "current_cmd": current_cmd,
        "real_current": real_current,
        "real_current_per_cmd": real_current / cmd_abs,
        "wheel_speed_fit": wheel_speed,
        "trigger_time_s": trigger_time_s,
        "pulse_duration_s": pulse_duration_s,
        "hard_timeout": any(parse_int(row, "hard_timeout", default=0) != 0 for row in segment_rows),
        "valid": all(parse_int(row, "valid", default=0) != 0 for row in segment_rows),
        "baseline_samples": len(baseline),
        "active_samples": len(active),
        "baseline_acc_z": base_az,
        "base_omega_z": base_wz,
        "a_com_x": delta_a_com_x,
        "a_com_y": delta_a_com_y,
        "a_parallel": a_parallel,
        "force_parallel_n": force_parallel_n,
        "alpha_z": alpha_z,
        "a_x_per_cmd": delta_a_com_x / cmd_abs,
        "a_y_per_cmd": delta_a_com_y / cmd_abs,
        "a_parallel_per_cmd": a_parallel / cmd_abs,
        "alpha_z_per_cmd": alpha_z / cmd_abs,
        "k_parallel_n_per_cmd": force_parallel_n / cmd_abs,
        "k_parallel_abs_n_per_cmd": abs(force_parallel_n / cmd_abs),
    }


def fit_file(
    path: Path,
    mass_kg: float,
    imu_rx_m: float,
    imu_ry_m: float,
    baseline_window: tuple[float, float],
    active_window: tuple[float, float],
) -> tuple[list[dict[str, float | int | str | bool]], dict[str, float | int | str]]:
    rows = read_csv(path)
    if not rows:
        raise ValueError("empty CSV")

    segments = find_active_segments(rows)
    if not segments:
        raise ValueError("no active pulse_state==2 segments")

    trials: list[dict[str, float | int | str | bool]] = []
    errors = 0
    for pulse_index, segment in enumerate(segments):
        try:
            trials.append(
                fit_segment(
                    path,
                    rows,
                    segment,
                    pulse_index,
                    mass_kg,
                    imu_rx_m,
                    imu_ry_m,
                    baseline_window,
                    active_window,
                )
            )
        except Exception:
            errors += 1

    wheel_indices = [parse_int(row, "wheel_index", default=255) for row in rows]
    valid_count = sum(1 for row in rows if parse_int(row, "valid", default=0) != 0)
    hard_timeout_count = sum(1 for row in rows if parse_int(row, "hard_timeout", default=0) != 0)
    summary = {
        "file": str(path),
        "rows": len(rows),
        "wheel_index": round(mean(wheel_indices)) if wheel_indices else 255,
        "wheel": WHEEL_NAME.get(round(mean(wheel_indices)) if wheel_indices else 255, "UNKNOWN"),
        "pulse_segments": len(segments),
        "fitted_pulses": len(trials),
        "skipped_pulses": errors,
        "valid_ratio": valid_count / len(rows),
        "hard_timeout_count": hard_timeout_count,
    }
    return trials, summary


def fit_groups(
    trials: list[dict[str, float | int | str | bool]],
    fit_min_abs_cmd: float,
) -> dict[str, dict[str, float | int | list[int] | str | bool]]:
    groups: dict[str, list[dict[str, float | int | str | bool]]] = {}
    for trial in trials:
        wheel = str(trial["wheel"])
        sign = str(trial["sign"])
        groups.setdefault(f"{wheel}_{sign}", []).append(trial)

    out: dict[str, dict[str, float | int | list[int] | str | bool]] = {}
    for key, items_all in groups.items():
        items = [item for item in items_all if abs(float(item["current_cmd"])) >= fit_min_abs_cmd]
        if len(items) < 2:
            continue

        currents = [float(item["current_cmd"]) for item in items]
        a_parallel = [float(item["a_parallel"]) for item in items]
        force = [float(item["force_parallel_n"]) for item in items]
        alpha = [float(item["alpha_z"]) for item in items]
        a_slope, a_intercept = slope_xy(currents, a_parallel)
        force_slope, force_intercept = slope_xy(currents, force)
        alpha_slope, alpha_intercept = slope_xy(currents, alpha)
        a_pred = [a_slope * current + a_intercept for current in currents]
        force_pred = [force_slope * current + force_intercept for current in currents]
        deadzone_cmd = -a_intercept / a_slope if abs(a_slope) > 1.0e-12 else 0.0

        out[key] = {
            "wheel": str(items[0]["wheel"]),
            "wheel_index": int(items[0]["wheel_index"]),
            "sign": str(items[0]["sign"]),
            "n_all": len(items_all),
            "n_fit": len(items),
            "fit_min_abs_cmd": fit_min_abs_cmd,
            "fit_cmds": [round(current) for current in currents],
            "a_parallel_per_cmd": a_slope,
            "a_parallel_intercept": a_intercept,
            "alpha_z_per_cmd": alpha_slope,
            "alpha_z_intercept": alpha_intercept,
            "k_n_per_cmd": force_slope,
            "k_abs_n_per_cmd": abs(force_slope),
            "force_to_current_cmd_per_n": 1.0 / force_slope if abs(force_slope) > 1.0e-12 else 0.0,
            "deadzone_cmd_est": deadzone_cmd,
            "deadzone_abs_cmd_est": abs(deadzone_cmd),
            "rmse_a_parallel_mps2": rms([pred - actual for pred, actual in zip(a_pred, a_parallel)]),
            "rmse_force_n": rms([pred - actual for pred, actual in zip(force_pred, force)]),
        }
    return out


def build_recommended_table(groups: dict[str, dict[str, float | int | list[int] | str | bool]]) -> list[dict[str, float | int | str]]:
    rows: list[dict[str, float | int | str]] = []
    for wheel_index in range(4):
        wheel = WHEEL_NAME[wheel_index]
        for sign in ("pos", "neg"):
            group = groups.get(f"{wheel}_{sign}")
            if group is None:
                continue
            rows.append(
                {
                    "wheel_index": wheel_index,
                    "wheel": wheel,
                    "direction": "+" if sign == "pos" else "-",
                    "k_n_per_cmd": float(group["k_n_per_cmd"]),
                    "cmd_per_n": float(group["force_to_current_cmd_per_n"]),
                    "deadzone_abs_cmd_est": float(group["deadzone_abs_cmd_est"]),
                    "alpha_z_per_cmd": float(group["alpha_z_per_cmd"]),
                    "a_parallel_per_cmd": float(group["a_parallel_per_cmd"]),
                }
            )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", help="CSV files, globs, or directories")
    parser.add_argument("--mass-kg", type=float, required=True, help="Measured whole-robot mass")
    parser.add_argument("--imu-rx-m", type=float, default=0.0, help="IMU x offset from COM, forward positive, meters")
    parser.add_argument("--imu-ry-m", type=float, default=0.0, help="IMU y offset from COM, left positive, meters")
    parser.add_argument("--baseline", nargs=2, type=float, default=(-0.8, -0.05), metavar=("START", "END"))
    parser.add_argument("--active", nargs=2, type=float, default=(0.10, 0.90), metavar=("START", "END"))
    parser.add_argument("--fit-min-abs-cmd", type=float, default=3000.0, help="Only use abs(current_cmd)>=this for linear K fit")
    parser.add_argument("--output", type=Path, default=Path("calib_single_wheel_response_results.json"))
    args = parser.parse_args()

    files = discover_files(args.csv)
    if not files:
        raise SystemExit("no CSV files found")

    trials: list[dict[str, float | int | str | bool]] = []
    file_summaries = []
    errors = {}
    for path in files:
        try:
            file_trials, summary = fit_file(
                path,
                args.mass_kg,
                args.imu_rx_m,
                args.imu_ry_m,
                tuple(args.baseline),
                tuple(args.active),
            )
            trials.extend(file_trials)
            file_summaries.append(summary)
        except Exception as exc:
            errors[str(path)] = str(exc)

    groups = fit_groups(trials, args.fit_min_abs_cmd)
    result = {
        "mass_kg": args.mass_kg,
        "imu_offset_m": {"rx_forward": args.imu_rx_m, "ry_left": args.imu_ry_m},
        "coordinate_convention": {
            "x": "forward positive",
            "y": "left positive",
            "yaw": "counter-clockwise positive",
        },
        "baseline_window_s": list(args.baseline),
        "active_window_s": list(args.active),
        "fit_min_abs_cmd": args.fit_min_abs_cmd,
        "low_current_note": "Pulses below fit_min_abs_cmd are kept in trials for deadzone/threshold inspection, not used for linear K.",
        "file_summaries": file_summaries,
        "trials": trials,
        "groups": groups,
        "recommended_table": build_recommended_table(groups),
        "errors": errors,
    }

    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(f"wrote {args.output} with {len(trials)} fitted pulses ({len(errors)} file errors)")
    for row in result["recommended_table"]:
        print(
            f"{row['wheel']} {row['direction']}: "
            f"K={row['k_n_per_cmd']:.8f} N/cmd, "
            f"cmd/N={row['cmd_per_n']:.1f}, "
            f"deadzone={row['deadzone_abs_cmd_est']:.0f}"
        )
    return 0 if trials else 1


if __name__ == "__main__":
    raise SystemExit(main())
