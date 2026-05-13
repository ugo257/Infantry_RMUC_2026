#!/usr/bin/env python3
"""Fit the omni H matrix from remote-control chassis logs.

Input CSVs should be exported from g_omni_h_calib_record. The script uses
run_id as the segment key, finds static windows inside each segment, removes
IMU bias, integrates short body-frame acceleration, and fits:

    wheel_speed_fit = H_si * [vx_mps, vy_left_mps, wz_radps]
    [vx_mps, vy_left_mps, wz_radps] = cmd_to_si * [cmd_vx, cmd_vy_left, cmd_wz]

The pasted controller matrix is H_cmd = H_si * cmd_to_si.
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


WHEEL_NAMES = ("LF", "RF", "RB", "LB")
AXIS_NAMES = ("vx", "vy_left", "wz")

OLD_H_CMD = (
    (0.039037455, -0.039722910, -0.286529291),
    (0.039566929, 0.040352354, 0.287104627),
    (0.039479646, -0.039443869, 0.282737276),
    (0.039885362, 0.039679205, -0.285686269),
)


@dataclass
class Row:
    source: str
    index: int
    t: float
    dt: float
    raw: dict[str, str]


@dataclass
class StaticWindow:
    start: int
    end: int
    start_t: float
    end_t: float

    @property
    def duration(self) -> float:
        return max(0.0, self.end_t - self.start_t)


@dataclass
class Sample:
    run_key: str
    t: float
    cmd: tuple[float, float, float]
    raw_cmd: tuple[float, float, float]
    nu_si: tuple[float, float, float]
    wheel: tuple[float, float, float, float]
    steady_for_cmd_scale: bool


def parse_float(row: dict[str, str], *names: str, default: float = 0.0) -> float:
    for name in names:
        value = row.get(name, "")
        if value != "":
            return float(value)
    return default


def parse_int(row: dict[str, str], *names: str, default: int = 0) -> int:
    for name in names:
        value = row.get(name, "")
        if value != "":
            return int(float(value))
    return default


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
        return list(csv.DictReader(f))


def row_time(row: dict[str, str], fallback_index: int) -> float:
    for name in ("sample_time_s", "time_s", "t"):
        value = row.get(name, "")
        if value != "":
            return float(value)
    return fallback_index * 0.001


def row_dt(row: dict[str, str], t: float, last_t: float | None, max_dt: float) -> float:
    dt = parse_float(row, "dt_s", "dt", default=-1.0)
    if dt <= 0.0 and last_t is not None:
        dt = t - last_t
    if dt < 0.0:
        dt = 0.0
    if dt > max_dt:
        dt = max_dt
    return dt


def group_runs(path: Path, rows: list[dict[str, str]], include_disabled: bool, max_dt: float) -> dict[str, list[Row]]:
    groups: dict[str, list[Row]] = {}
    sorted_rows = sorted(enumerate(rows), key=lambda item: row_time(item[1], item[0]))
    last_t_by_run: dict[int, float] = {}
    for fallback_index, row in sorted_rows:
        if parse_int(row, "valid", default=1) == 0:
            continue
        if not include_disabled and parse_int(row, "enable", default=0) == 0:
            continue
        run_id = parse_int(row, "run_id", default=0)
        t = row_time(row, fallback_index)
        last_t = last_t_by_run.get(run_id)
        dt = row_dt(row, t, last_t, max_dt)
        last_t_by_run[run_id] = t
        key = f"{path}:{run_id}"
        groups.setdefault(key, []).append(Row(str(path), fallback_index, t, dt, row))
    return groups


def cmd_tuple(row: dict[str, str]) -> tuple[float, float, float]:
    return (
        parse_float(row, "cmd_vx"),
        parse_float(row, "cmd_vy_left", "cmd_vy"),
        parse_float(row, "cmd_wz"),
    )


def raw_cmd_tuple(row: dict[str, str]) -> tuple[float, float, float]:
    return (
        parse_float(row, "raw_cmd_vx"),
        parse_float(row, "raw_cmd_vy"),
        parse_float(row, "raw_cmd_wz"),
    )


def wheel_tuple(row: dict[str, str]) -> tuple[float, float, float, float]:
    return (
        parse_float(row, "wheel_speed_fit_lf"),
        parse_float(row, "wheel_speed_fit_rf"),
        parse_float(row, "wheel_speed_fit_rb"),
        parse_float(row, "wheel_speed_fit_lb"),
    )


def is_static(row: dict[str, str], static_cmd: float, static_wheel: float, static_gyro: float) -> bool:
    cmd_max = max(abs(value) for value in cmd_tuple(row))
    wheel_max = max(abs(value) for value in wheel_tuple(row))
    gyro = abs(parse_float(row, "body_omega_z", "omega_z"))
    return cmd_max <= static_cmd and wheel_max <= static_wheel and gyro <= static_gyro


def find_static_windows(
    rows: list[Row],
    static_cmd: float,
    static_wheel: float,
    static_gyro: float,
    min_static_s: float,
    max_gap_s: float,
) -> tuple[list[StaticWindow], list[bool]]:
    flags = [is_static(item.raw, static_cmd, static_wheel, static_gyro) for item in rows]
    windows: list[StaticWindow] = []
    start: int | None = None

    for i, flag in enumerate(flags):
        gap_break = i > 0 and rows[i].t - rows[i - 1].t > max_gap_s
        if not flag or gap_break:
            if start is not None:
                end = i - 1
                window = StaticWindow(start, end, rows[start].t, rows[end].t)
                if window.duration >= min_static_s:
                    windows.append(window)
                start = None
        if flag and start is None:
            start = i

    if start is not None:
        end = len(flags) - 1
        window = StaticWindow(start, end, rows[start].t, rows[end].t)
        if window.duration >= min_static_s:
            windows.append(window)

    return windows, flags


def mean_indices(rows: list[Row], indices: list[int], field: str, *aliases: str) -> float:
    values = [parse_float(rows[i].raw, field, *aliases) for i in indices]
    return mean(values) if values else 0.0


def static_indices(windows: list[StaticWindow]) -> list[int]:
    indices: list[int] = []
    for window in windows:
        indices.extend(range(window.start, window.end + 1))
    return indices


def solve3(a: list[list[float]], b: list[float]) -> list[float]:
    m = [[a[r][c] for c in range(3)] + [b[r]] for r in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1.0e-12:
            return [0.0, 0.0, 0.0]
        if pivot != col:
            m[col], m[pivot] = m[pivot], m[col]
        inv = 1.0 / m[col][col]
        for c in range(col, 4):
            m[col][c] *= inv
        for r in range(3):
            if r == col:
                continue
            factor = m[r][col]
            for c in range(col, 4):
                m[r][c] -= factor * m[col][c]
    return [m[r][3] for r in range(3)]


def least_squares_3(xs: list[tuple[float, float, float]], ys: list[float], ridge: float = 1.0e-9) -> list[float]:
    normal = [[0.0 for _ in range(3)] for _ in range(3)]
    rhs = [0.0, 0.0, 0.0]
    for x, y in zip(xs, ys):
        for r in range(3):
            rhs[r] += x[r] * y
            for c in range(3):
                normal[r][c] += x[r] * x[c]
    for i in range(3):
        normal[i][i] += ridge
    return solve3(normal, rhs)


def mat_vec_4x3(h: list[list[float]] | tuple[tuple[float, float, float], ...],
                x: tuple[float, float, float]) -> list[float]:
    return [row[0] * x[0] + row[1] * x[1] + row[2] * x[2] for row in h]


def mat_mul_4x3_3x3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)]
        for r in range(4)
    ]


def rms(values: list[float]) -> float:
    return math.sqrt(mean([v * v for v in values])) if values else 0.0


def interpolate_anchor(
    anchors: list[tuple[float, float, float]],
    t: float,
) -> tuple[float, float] | None:
    if len(anchors) < 2 or t < anchors[0][0] or t > anchors[-1][0]:
        return None
    for i in range(len(anchors) - 1):
        t0, vx0, vy0 = anchors[i]
        t1, vx1, vy1 = anchors[i + 1]
        if t0 <= t <= t1:
            ratio = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return (vx0 + ratio * (vx1 - vx0), vy0 + ratio * (vy1 - vy0))
    return None


def fit_run(
    run_key: str,
    rows: list[Row],
    args: argparse.Namespace,
) -> tuple[list[Sample], dict[str, object]]:
    rows = sorted(rows, key=lambda item: item.t)
    if len(rows) < 20:
        raise ValueError("not enough valid rows")

    windows, static_flags = find_static_windows(
        rows,
        args.static_cmd,
        args.static_wheel_deg_s,
        args.static_gyro_rad_s,
        args.min_static_s,
        args.max_static_gap_s,
    )
    if len(windows) < 2:
        raise ValueError(f"need at least two static windows, found {len(windows)}")

    sidx = static_indices(windows)
    base_ax = mean_indices(rows, sidx, "body_acc_x", "acc_x")
    base_ay = mean_indices(rows, sidx, "body_acc_y", "acc_y")
    base_az = mean_indices(rows, sidx, "body_acc_z", "acc_z")
    base_wz = mean_indices(rows, sidx, "body_omega_z", "omega_z")

    raw_vel: list[tuple[float, float]] = []
    gyro_unbiased: list[float] = []
    vx = 0.0
    vy = 0.0
    for item in rows:
        ax = parse_float(item.raw, "body_acc_x", "acc_x") - base_ax
        ay = parse_float(item.raw, "body_acc_y", "acc_y") - base_ay
        wz = parse_float(item.raw, "body_omega_z", "omega_z") - base_wz
        gyro_unbiased.append(wz)
        vx_old = vx
        vx += item.dt * (ax + wz * vy)
        vy += item.dt * (ay - wz * vx_old)
        raw_vel.append((vx, vy))

    anchors: list[tuple[float, float, float]] = []
    for window in windows:
        indices = list(range(window.start, window.end + 1))
        center_t = mean([rows[i].t for i in indices])
        center_vx = mean([raw_vel[i][0] for i in indices])
        center_vy = mean([raw_vel[i][1] for i in indices])
        anchors.append((center_t, center_vx, center_vy))

    samples: list[Sample] = []
    for i, item in enumerate(rows):
        if static_flags[i]:
            continue
        drift = interpolate_anchor(anchors, item.t)
        if drift is None:
            continue
        cmd = cmd_tuple(item.raw)
        if max(abs(value) for value in cmd) < args.min_fit_cmd:
            continue
        raw_cmd = raw_cmd_tuple(item.raw)
        wz_true = parse_float(item.raw, "body_omega_z", "omega_z") - base_wz
        acc_x = parse_float(item.raw, "body_acc_x", "acc_x") - base_ax
        acc_y = parse_float(item.raw, "body_acc_y", "acc_y") - base_ay
        if i > 0 and item.dt > 1.0e-6:
            gyro_acc = (gyro_unbiased[i] - gyro_unbiased[i - 1]) / item.dt
        else:
            gyro_acc = 0.0
        steady_for_cmd_scale = (
            math.hypot(acc_x, acc_y) <= args.cmd_scale_max_acc_mps2 and
            abs(gyro_acc) <= args.cmd_scale_max_gyro_acc_rad_s2
        )
        vx_true = raw_vel[i][0] - drift[0]
        vy_true = raw_vel[i][1] - drift[1]
        samples.append(
            Sample(
                run_key,
                item.t,
                cmd,
                raw_cmd,
                (vx_true, vy_true, wz_true),
                wheel_tuple(item.raw),
                steady_for_cmd_scale,
            )
        )

    if len(samples) < args.min_run_samples:
        raise ValueError(f"not enough dynamic samples after static-window correction: {len(samples)}")

    first = rows[0].raw
    summary: dict[str, object] = {
        "run_key": run_key,
        "run_id": parse_int(first, "run_id", default=0),
        "rows_valid": len(rows),
        "samples_fit": len(samples),
        "static_windows": [
            {
                "start_s": window.start_t,
                "end_s": window.end_t,
                "duration_s": window.duration,
            }
            for window in windows
        ],
        "gyro_bias_rad_s": base_wz,
        "acc_bias_x_mps2": base_ax,
        "acc_bias_y_mps2": base_ay,
        "acc_z_static_mean_mps2": base_az,
        "cmd_max": max(max(abs(v) for v in cmd_tuple(item.raw)) for item in rows),
        "wheel_fit_max_deg_s": max(max(abs(v) for v in wheel_tuple(item.raw)) for item in rows),
    }
    return samples, summary


def fit_h_si(samples: list[Sample]) -> list[list[float]]:
    xs = [sample.nu_si for sample in samples]
    return [
        least_squares_3(xs, [sample.wheel[wheel] for sample in samples])
        for wheel in range(4)
    ]


def fit_cmd_to_si(samples: list[Sample], min_samples: int) -> tuple[list[list[float]], int, bool]:
    scale_samples = [sample for sample in samples if sample.steady_for_cmd_scale]
    used_fallback = False
    if len(scale_samples) < min_samples:
        scale_samples = samples
        used_fallback = True
    xs = [sample.cmd for sample in scale_samples]
    return [
        least_squares_3(xs, [sample.nu_si[axis] for sample in scale_samples])
        for axis in range(3)
    ], len(scale_samples), used_fallback


def fit_h_cmd_direct(samples: list[Sample]) -> list[list[float]]:
    xs = [sample.cmd for sample in samples]
    return [
        least_squares_3(xs, [sample.wheel[wheel] for sample in samples])
        for wheel in range(4)
    ]


def residual_report(
    samples: list[Sample],
    h_si: list[list[float]],
    h_cmd: list[list[float]],
    h_cmd_direct: list[list[float]],
) -> dict[str, float]:
    si_res: list[float] = []
    cmd_res: list[float] = []
    direct_res: list[float] = []
    old_res: list[float] = []

    for sample in samples:
        pred_si = mat_vec_4x3(h_si, sample.nu_si)
        pred_cmd = mat_vec_4x3(h_cmd, sample.cmd)
        pred_direct = mat_vec_4x3(h_cmd_direct, sample.cmd)
        pred_old = mat_vec_4x3(OLD_H_CMD, sample.cmd)
        for wheel in range(4):
            si_res.append(pred_si[wheel] - sample.wheel[wheel])
            cmd_res.append(pred_cmd[wheel] - sample.wheel[wheel])
            direct_res.append(pred_direct[wheel] - sample.wheel[wheel])
            old_res.append(pred_old[wheel] - sample.wheel[wheel])

    return {
        "rmse_h_si_wheel_deg_s": rms(si_res),
        "rmse_h_cmd_wheel_deg_s": rms(cmd_res),
        "rmse_h_cmd_direct_wheel_deg_s": rms(direct_res),
        "rmse_old_h_cmd_wheel_deg_s": rms(old_res),
    }


def c_matrix_snippet(h_cmd: list[list[float]]) -> str:
    lines = []
    for wheel, name in enumerate(WHEEL_NAMES):
        for axis, axis_name in enumerate(("VX", "VY", "WZ")):
            lines.append(
                f"ctrl->h[OMNI_WHEEL_{name}][OMNI_AXIS_{axis_name}] = {h_cmd[wheel][axis]:.9g}f;"
            )
        if wheel != len(WHEEL_NAMES) - 1:
            lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", help="CSV files, globs, or directories")
    parser.add_argument("--output", type=Path, default=Path("calib_omni_h_results.json"))
    parser.add_argument("--include-disabled", action="store_true", help="Also fit rows whose enable field is 0")
    parser.add_argument("--static-cmd", type=float, default=80.0, help="Max abs command considered static")
    parser.add_argument("--static-wheel-deg-s", type=float, default=120.0, help="Max abs wheel speed considered static")
    parser.add_argument("--static-gyro-rad-s", type=float, default=0.08, help="Max abs gyro z considered static")
    parser.add_argument("--min-static-s", type=float, default=0.25, help="Minimum static window duration")
    parser.add_argument("--max-static-gap-s", type=float, default=0.08, help="Split static windows across larger time gaps")
    parser.add_argument("--max-integrate-dt-s", type=float, default=0.02, help="Clamp integration dt to this value")
    parser.add_argument("--min-fit-cmd", type=float, default=100.0, help="Minimum abs command for dynamic fit samples")
    parser.add_argument("--min-run-samples", type=int, default=20, help="Minimum usable dynamic samples per run")
    parser.add_argument("--min-total-samples", type=int, default=80, help="Minimum usable dynamic samples overall")
    parser.add_argument("--min-cmd-scale-samples", type=int, default=60, help="Minimum steady samples for cmd_to_si fit")
    parser.add_argument("--cmd-scale-max-acc-mps2", type=float, default=0.6, help="Max translational accel for steady cmd_to_si samples")
    parser.add_argument("--cmd-scale-max-gyro-acc-rad-s2", type=float, default=1.0, help="Max yaw acceleration for steady cmd_to_si samples")
    args = parser.parse_args()

    files = discover_files(args.csv)
    if not files:
        raise SystemExit("no CSV files found")

    all_samples: list[Sample] = []
    runs: list[dict[str, object]] = []
    errors: dict[str, str] = {}

    for path in files:
        try:
            rows = read_csv(path)
            groups = group_runs(path, rows, args.include_disabled, args.max_integrate_dt_s)
            if not groups:
                errors[str(path)] = "no rows left after valid/enable filtering"
                continue
            for run_key, run_rows in groups.items():
                try:
                    samples, summary = fit_run(run_key, run_rows, args)
                    all_samples.extend(samples)
                    runs.append(summary)
                except Exception as exc:
                    errors[run_key] = str(exc)
        except Exception as exc:
            errors[str(path)] = str(exc)

    if len(all_samples) < args.min_total_samples:
        detail = "\n".join(f"- {key}: {value}" for key, value in errors.items())
        raise SystemExit(
            f"not enough usable samples: {len(all_samples)} < {args.min_total_samples}\n{detail}"
        )

    h_si = fit_h_si(all_samples)
    warnings: list[str] = []
    cmd_to_si, cmd_scale_sample_count, cmd_scale_fallback = fit_cmd_to_si(all_samples, args.min_cmd_scale_samples)
    if cmd_scale_fallback:
        warnings.append(
            "not enough steady samples for cmd_to_si; used all dynamic samples, so command scale may be transient-biased"
        )
    h_cmd = mat_mul_4x3_3x3(h_si, cmd_to_si)
    h_cmd_direct = fit_h_cmd_direct(all_samples)
    residuals = residual_report(all_samples, h_si, h_cmd, h_cmd_direct)
    snippet = c_matrix_snippet(h_cmd)

    result = {
        "sample_count": len(all_samples),
        "runs": runs,
        "axis_names": AXIS_NAMES,
        "wheel_names": WHEEL_NAMES,
        "h_si_wheel_deg_s_per_si": h_si,
        "cmd_to_si_matrix": cmd_to_si,
        "cmd_to_si_sample_count": cmd_scale_sample_count,
        "cmd_to_si_used_all_dynamic_samples": cmd_scale_fallback,
        "h_cmd_wheel_deg_s_per_cmd": h_cmd,
        "h_cmd_direct_wheel_deg_s_per_cmd": h_cmd_direct,
        "old_h_cmd_wheel_deg_s_per_cmd": OLD_H_CMD,
        "residuals": residuals,
        "c_matrix_snippet": snippet,
        "warnings": warnings,
        "errors": errors,
    }
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(f"wrote {args.output} with {len(runs)} runs and {len(all_samples)} fit samples")
    if errors:
        print(f"skipped {len(errors)} run/file groups; see JSON errors for details")
    for warning in warnings:
        print(f"warning: {warning}")
    print("\nC constants for OmniBuildMatrix():\n")
    print(snippet)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
