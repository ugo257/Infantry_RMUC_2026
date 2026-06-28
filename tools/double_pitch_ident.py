#!/usr/bin/env python3
"""Identify the coupled double-pitch model from g_double_pitch_ident_debug CSVs.

Target firmware model:

  tau = M(phi) * [theta_ddot; phi_ddot] + C(q, dq) + G(q) + F(dq)

The CSV should be exported from Ozone with g_double_pitch_ident_debug fields.
Rows are used only when valid, feedback_ok and running are true. Limit-hit rows
are rejected by default.

For static gravity feedforward calibration, use:

  python3 tools/double_pitch_ident.py --mode gravity-static static_points.csv

Static mode fits only GB1/GB2/GP1/GP2 from near-zero-speed hold points. It
groups continuous valid still samples into one point before fitting so each
hold angle has similar weight.

For open-loop torque-ramp gravity calibration, use:

  python3 tools/double_pitch_ident.py --mode gravity-ramp ramp.csv

Ramp mode fits the same gravity terms from low-speed rows, grouped by
theta/psi angle bins so dense portions of one sweep do not dominate the fit.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


PREFIX = "(g_double_pitch_ident_debug)."
OMEGA0 = 0.05

PARAMS = [
    "A0",
    "AC",
    "AS",
    "B0",
    "BC",
    "BS",
    "C0",
    "GB1",
    "GB2",
    "GP1",
    "GP2",
    "FVB",
    "FCB",
    "FVS",
    "FCS",
]

GRAVITY_PARAMS = [
    "GB1",
    "GB2",
    "GP1",
    "GP2",
]

LOCAL_CTRL_SET_MASS = {
    "A0": 0.02904504,
    "AC": 0.0,
    "AS": 0.0,
    "B0": -0.006220628,
    "BC": 0.0,
    "BS": 0.0,
    "C0": 0.0149174,
}

FW = {
    "A0": 0.0314,
    "AC": 0.0,
    "AS": 0.0,
    "B0": -0.0057,
    "BC": 0.0,
    "BS": 0.0,
    "C0": 0.02849,
    "GB1": 1.0666,
    "GB2": 0.0,
    "GP1": 0.0,
    "GP2": -0.98923,
    "FVB": 0.0,
    "FCB": 0.0,
    "FVS": 0.18155,
    "FCS": 0.1347,
}

MACROS = {
    "A0": "DOUBLE_PITCH_MODEL_A0",
    "AC": "DOUBLE_PITCH_MODEL_AC",
    "AS": "DOUBLE_PITCH_MODEL_AS",
    "B0": "DOUBLE_PITCH_MODEL_B0",
    "BC": "DOUBLE_PITCH_MODEL_BC",
    "BS": "DOUBLE_PITCH_MODEL_BS",
    "C0": "DOUBLE_PITCH_MODEL_C0",
    "GB1": "DOUBLE_PITCH_MODEL_GB1",
    "GB2": "DOUBLE_PITCH_MODEL_GB2",
    "GP1": "DOUBLE_PITCH_MODEL_GP1",
    "GP2": "DOUBLE_PITCH_MODEL_GP2",
    "FVB": "DOUBLE_PITCH_MODEL_FVB",
    "FCB": "DOUBLE_PITCH_MODEL_FCB",
    "FVS": "DOUBLE_PITCH_MODEL_FVS",
    "FCS": "DOUBLE_PITCH_MODEL_FCS",
}

REQUIRED_COMMON = [
    "time_s",
    "valid",
    "feedback_ok",
    "theta_rad",
    "phi_rad",
    "omega_big_rad_s",
    "omega_small_rad_s",
    "tau_big_limit_flag",
    "tau_small_limit_flag",
    "tau_big_cmd_nm",
    "tau_small_cmd_nm",
    "tau_big_meas_nm",
    "tau_small_meas_nm",
    "tau_big_ctrl_tor_set_nm",
    "tau_small_ctrl_tor_set_nm",
]

REQUIRED_DYNAMIC = [
    "running",
    "axis",
]

REQUIRED_RUNNING = [
    "running",
]


@dataclass
class StaticPoint:
    source: str
    point: int
    rows: int
    duration_s: float
    theta_rad: float
    phi_rad: float
    omega_big_abs_max: float
    omega_small_abs_max: float
    tau_big_nm: float
    tau_small_nm: float
    tau_big_std_nm: float
    tau_small_std_nm: float


def smooth_sign(x: np.ndarray) -> np.ndarray:
    return np.tanh(x / OMEGA0)


def load_csv(path: Path, mode: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df.columns = [c.replace(PREFIX, "").strip() for c in df.columns]
    required = list(REQUIRED_COMMON)
    if mode == "dynamic":
        required.extend(REQUIRED_DYNAMIC)
    elif mode == "gravity-ramp":
        required.extend(REQUIRED_RUNNING)
    missing = [name for name in required if name not in df.columns]
    if missing:
        raise ValueError(f"{path}: missing columns: {', '.join(missing)}")
    return df


def odd_window(requested: int, n: int) -> int:
    win = max(5, requested)
    if win % 2 == 0:
        win += 1
    if win >= n:
        win = n - 1 if (n - 1) % 2 == 1 else n - 2
    if win < 5:
        raise ValueError(f"trace too short for Savitzky-Golay window: n={n}")
    return win


def target_columns(target: str) -> tuple[str, str]:
    if target == "measured":
        return "tau_big_meas_nm", "tau_small_meas_nm"
    if target == "ctrl-set":
        return "tau_big_ctrl_tor_set_nm", "tau_small_ctrl_tor_set_nm"
    if target == "cmd":
        return "tau_big_cmd_nm", "tau_small_cmd_nm"
    raise ValueError(f"unknown target: {target}")


def build_regression(df: pd.DataFrame, args: argparse.Namespace):
    if args.axis != "all":
        axis_id = 1 if args.axis == "theta" else 2
        df = df[df["axis"].to_numpy(dtype=float) == axis_id]
        if df.empty:
            raise ValueError(f"no rows for axis={args.axis}")

    t = df["time_s"].to_numpy(dtype=float)
    finite_time = np.isfinite(t)
    df = df[finite_time]
    t = t[finite_time]
    if len(t) < args.window + 2:
        raise ValueError(f"not enough rows after filtering: {len(t)}")

    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("invalid or non-monotonic time_s")

    win = odd_window(args.window, len(t))
    theta = df["theta_rad"].to_numpy(dtype=float)
    phi = df["phi_rad"].to_numpy(dtype=float)
    omega_big = df["omega_big_rad_s"].to_numpy(dtype=float)
    omega_small = df["omega_small_rad_s"].to_numpy(dtype=float)

    theta_ddot = savgol_filter(omega_big,
                               win,
                               args.polyorder,
                               deriv=1,
                               delta=dt,
                               mode="interp")
    phi_ddot = savgol_filter(omega_small,
                             win,
                             args.polyorder,
                             deriv=1,
                             delta=dt,
                             mode="interp")

    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    sin_sum = np.sin(theta + phi)
    cos_sum = np.cos(theta + phi)
    ss_big = smooth_sign(omega_big)
    ss_small = smooth_sign(omega_small)

    zeros = np.zeros_like(theta)
    x_big = np.column_stack([
        theta_ddot,
        cos_phi * theta_ddot - sin_phi * omega_big * omega_small,
        sin_phi * theta_ddot + cos_phi * omega_big * omega_small,
        phi_ddot,
        cos_phi * phi_ddot - sin_phi * omega_small * omega_small,
        sin_phi * phi_ddot + cos_phi * omega_small * omega_small,
        zeros,
        sin_theta,
        cos_theta,
        sin_sum,
        cos_sum,
        omega_big,
        ss_big,
        zeros,
        zeros,
    ])
    x_small = np.column_stack([
        zeros,
        0.5 * sin_phi * omega_big * omega_big,
        -0.5 * cos_phi * omega_big * omega_big,
        theta_ddot,
        cos_phi * theta_ddot,
        sin_phi * theta_ddot,
        phi_ddot,
        zeros,
        zeros,
        sin_sum,
        cos_sum,
        zeros,
        zeros,
        omega_small,
        ss_small,
    ])

    big_col, small_col = target_columns(args.target)
    y_big = df[big_col].to_numpy(dtype=float)
    y_small = df[small_col].to_numpy(dtype=float)

    valid = (
        (df["valid"].to_numpy(dtype=float) > 0.5)
        & (df["feedback_ok"].to_numpy(dtype=float) > 0.5)
        & (df["running"].to_numpy(dtype=float) > 0.5)
    )
    if not args.keep_limited:
        valid &= (
            (df["tau_big_limit_flag"].to_numpy(dtype=float) < 0.5)
            & (df["tau_small_limit_flag"].to_numpy(dtype=float) < 0.5)
        )

    edge = win
    valid[:edge] = False
    valid[-edge:] = False

    finite = (
        np.all(np.isfinite(x_big), axis=1)
        & np.all(np.isfinite(x_small), axis=1)
        & np.isfinite(y_big)
        & np.isfinite(y_small)
    )
    mask = valid & finite
    if int(np.sum(mask)) < args.min_samples:
        raise ValueError(
            f"only {int(np.sum(mask))} valid samples, need {args.min_samples}"
        )

    x = np.vstack([x_big[mask], x_small[mask]])
    y = np.concatenate([y_big[mask], y_small[mask]])
    extras = {
        "dt": dt,
        "window": win,
        "samples": int(np.sum(mask)),
        "limit_ratio": 1.0 - float(np.mean(valid | ~finite)),
    }
    return x, y, extras


def fit_model(x: np.ndarray, y: np.ndarray, ridge: float):
    if ridge > 0.0:
        lhs = x.T @ x + ridge * np.eye(x.shape[1])
        theta = np.linalg.solve(lhs, x.T @ y)
    else:
        theta, *_ = np.linalg.lstsq(x, y, rcond=None)
    residual = y - x @ theta
    sv = np.linalg.svd(x, compute_uv=False)
    cond = float(sv[0] / sv[-1]) if sv[-1] > 0.0 else float("inf")
    return theta, residual, cond


def build_gravity_rows(theta: np.ndarray, phi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    theta_plus_phi = theta + phi
    sin_sum = np.sin(theta_plus_phi)
    cos_sum = np.cos(theta_plus_phi)
    zeros = np.zeros_like(theta)

    x_big = np.column_stack([
        sin_theta,
        cos_theta,
        sin_sum,
        cos_sum,
    ])
    x_small = np.column_stack([
        zeros,
        zeros,
        sin_sum,
        cos_sum,
    ])
    return x_big, x_small


def split_true_segments(mask: np.ndarray, t: np.ndarray, gap_s: float) -> list[tuple[int, int]]:
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []

    segments: list[tuple[int, int]] = []
    start = idx[0]
    prev = idx[0]
    for current in idx[1:]:
        if current != prev + 1 or float(t[current] - t[prev]) > gap_s:
            segments.append((start, prev + 1))
            start = current
        prev = current
    segments.append((start, prev + 1))
    return segments


def build_static_gravity_regression(
    df: pd.DataFrame,
    args: argparse.Namespace,
    source: str,
) -> tuple[np.ndarray, np.ndarray, dict[str, object]]:
    t = df["time_s"].to_numpy(dtype=float)
    theta = df["theta_rad"].to_numpy(dtype=float)
    phi = df["phi_rad"].to_numpy(dtype=float)
    omega_big = df["omega_big_rad_s"].to_numpy(dtype=float)
    omega_small = df["omega_small_rad_s"].to_numpy(dtype=float)
    big_col, small_col = target_columns(args.target)
    y_big = df[big_col].to_numpy(dtype=float)
    y_small = df[small_col].to_numpy(dtype=float)

    finite = (
        np.isfinite(t)
        & np.isfinite(theta)
        & np.isfinite(phi)
        & np.isfinite(omega_big)
        & np.isfinite(omega_small)
        & np.isfinite(y_big)
        & np.isfinite(y_small)
    )
    valid = (
        finite
        & (df["valid"].to_numpy(dtype=float) == 1.0)
        & (df["feedback_ok"].to_numpy(dtype=float) == 1.0)
        & (np.abs(omega_big) <= args.static_omega_threshold)
        & (np.abs(omega_small) <= args.static_omega_threshold)
    )
    if args.require_running and "running" in df.columns:
        valid &= df["running"].to_numpy(dtype=float) == 1.0
    if not args.keep_limited:
        valid &= (
            (df["tau_big_limit_flag"].to_numpy(dtype=float) == 0.0)
            & (df["tau_small_limit_flag"].to_numpy(dtype=float) == 0.0)
        )

    raw_valid_rows = int(np.sum(valid))
    points: list[StaticPoint] = []
    point_theta: list[float] = []
    point_phi: list[float] = []
    point_y_big: list[float] = []
    point_y_small: list[float] = []

    for start, end in split_true_segments(valid, t, args.static_gap_s):
        rows = end - start
        duration_s = float(t[end - 1] - t[start]) if rows > 1 else 0.0
        if rows < args.static_min_segment_samples:
            continue
        if duration_s < args.static_min_segment_s:
            continue

        sl = slice(start, end)
        point_id = len(points) + 1
        theta_mean = float(np.mean(theta[sl]))
        phi_mean = float(np.mean(phi[sl]))
        y_big_mean = float(np.mean(y_big[sl]))
        y_small_mean = float(np.mean(y_small[sl]))
        point_theta.append(theta_mean)
        point_phi.append(phi_mean)
        point_y_big.append(y_big_mean)
        point_y_small.append(y_small_mean)
        points.append(
            StaticPoint(
                source=source,
                point=point_id,
                rows=rows,
                duration_s=duration_s,
                theta_rad=theta_mean,
                phi_rad=phi_mean,
                omega_big_abs_max=float(np.max(np.abs(omega_big[sl]))),
                omega_small_abs_max=float(np.max(np.abs(omega_small[sl]))),
                tau_big_nm=y_big_mean,
                tau_small_nm=y_small_mean,
                tau_big_std_nm=float(np.std(y_big[sl])),
                tau_small_std_nm=float(np.std(y_small[sl])),
            )
        )

    if len(points) < args.min_static_points:
        raise ValueError(
            f"{source}: only {len(points)} static points, "
            f"need {args.min_static_points}; raw still rows={raw_valid_rows}"
        )

    theta_arr = np.asarray(point_theta)
    phi_arr = np.asarray(point_phi)
    y_big_arr = np.asarray(point_y_big)
    y_small_arr = np.asarray(point_y_small)
    x_big, x_small = build_gravity_rows(theta_arr, phi_arr)
    x = np.vstack([x_big, x_small])
    y = np.concatenate([y_big_arr, y_small_arr])
    extras = {
        "points": points,
        "raw_valid_rows": raw_valid_rows,
        "theta_pp": float(np.max(theta_arr) - np.min(theta_arr)),
        "phi_pp": float(np.max(phi_arr) - np.min(phi_arr)),
    }
    return x, y, extras


def build_ramp_gravity_regression(
    df: pd.DataFrame,
    args: argparse.Namespace,
    source: str,
) -> tuple[np.ndarray, np.ndarray, dict[str, object]]:
    t = df["time_s"].to_numpy(dtype=float)
    theta = df["theta_rad"].to_numpy(dtype=float)
    phi = df["phi_rad"].to_numpy(dtype=float)
    if "psi_rad" in df.columns:
        psi = df["psi_rad"].to_numpy(dtype=float)
    else:
        psi = theta + phi
    omega_big = df["omega_big_rad_s"].to_numpy(dtype=float)
    omega_small = df["omega_small_rad_s"].to_numpy(dtype=float)
    big_col, small_col = target_columns(args.target)
    y_big = df[big_col].to_numpy(dtype=float)
    y_small = df[small_col].to_numpy(dtype=float)

    finite = (
        np.isfinite(t)
        & np.isfinite(theta)
        & np.isfinite(phi)
        & np.isfinite(psi)
        & np.isfinite(omega_big)
        & np.isfinite(omega_small)
        & np.isfinite(y_big)
        & np.isfinite(y_small)
    )
    valid = (
        finite
        & (df["valid"].to_numpy(dtype=float) == 1.0)
        & (df["feedback_ok"].to_numpy(dtype=float) == 1.0)
        & (df["running"].to_numpy(dtype=float) == 1.0)
        & (np.abs(omega_big) <= args.ramp_omega_threshold)
        & (np.abs(omega_small) <= args.ramp_omega_threshold)
    )
    if not args.keep_limited:
        valid &= (
            (df["tau_big_limit_flag"].to_numpy(dtype=float) == 0.0)
            & (df["tau_small_limit_flag"].to_numpy(dtype=float) == 0.0)
        )

    raw_valid_rows = int(np.sum(valid))
    if raw_valid_rows == 0:
        raise ValueError(f"{source}: no valid gravity-ramp rows")

    bin_rad = args.ramp_bin_rad
    if bin_rad <= 0.0:
        raise ValueError("--ramp-bin-rad must be positive")

    valid_idx = np.flatnonzero(valid)
    theta_bin = np.floor(theta[valid_idx] / bin_rad).astype(np.int64)
    psi_bin = np.floor(psi[valid_idx] / bin_rad).astype(np.int64)
    groups: dict[tuple[int, int], list[int]] = {}
    for row_idx, key in zip(valid_idx, zip(theta_bin, psi_bin)):
        groups.setdefault(key, []).append(int(row_idx))

    points: list[StaticPoint] = []
    point_theta: list[float] = []
    point_phi: list[float] = []
    point_y_big: list[float] = []
    point_y_small: list[float] = []
    for key in sorted(groups):
        rows_idx = np.asarray(groups[key], dtype=int)
        if len(rows_idx) < args.ramp_min_bin_samples:
            continue

        point_id = len(points) + 1
        theta_mean = float(np.mean(theta[rows_idx]))
        phi_mean = float(np.mean(phi[rows_idx]))
        y_big_mean = float(np.mean(y_big[rows_idx]))
        y_small_mean = float(np.mean(y_small[rows_idx]))
        duration_s = float(np.max(t[rows_idx]) - np.min(t[rows_idx])) \
            if len(rows_idx) > 1 else 0.0
        point_theta.append(theta_mean)
        point_phi.append(phi_mean)
        point_y_big.append(y_big_mean)
        point_y_small.append(y_small_mean)
        points.append(
            StaticPoint(
                source=source,
                point=point_id,
                rows=int(len(rows_idx)),
                duration_s=duration_s,
                theta_rad=theta_mean,
                phi_rad=phi_mean,
                omega_big_abs_max=float(np.max(np.abs(omega_big[rows_idx]))),
                omega_small_abs_max=float(np.max(np.abs(omega_small[rows_idx]))),
                tau_big_nm=y_big_mean,
                tau_small_nm=y_small_mean,
                tau_big_std_nm=float(np.std(y_big[rows_idx])),
                tau_small_std_nm=float(np.std(y_small[rows_idx])),
            )
        )

    if len(points) < args.min_ramp_bins:
        raise ValueError(
            f"{source}: only {len(points)} angle bins, "
            f"need {args.min_ramp_bins}; raw valid rows={raw_valid_rows}"
        )

    theta_arr = np.asarray(point_theta)
    phi_arr = np.asarray(point_phi)
    y_big_arr = np.asarray(point_y_big)
    y_small_arr = np.asarray(point_y_small)
    x_big, x_small = build_gravity_rows(theta_arr, phi_arr)
    x = np.vstack([x_big, x_small])
    y = np.concatenate([y_big_arr, y_small_arr])
    extras = {
        "points": points,
        "raw_valid_rows": raw_valid_rows,
        "theta_pp": float(np.max(theta_arr) - np.min(theta_arr)),
        "phi_pp": float(np.max(phi_arr) - np.min(phi_arr)),
        "psi_pp": float(np.max(theta_arr + phi_arr) -
                        np.min(theta_arr + phi_arr)),
    }
    return x, y, extras


def print_report(label: str, x: np.ndarray, y: np.ndarray, args: argparse.Namespace):
    theta, residual, cond = fit_model(x, y, args.ridge)
    rms = float(np.sqrt(np.mean(residual * residual)))
    y_rms = float(np.sqrt(np.mean(y * y)))
    print(f"\n=== {label} ===")
    print(f"rows={x.shape[0]}  physical_samples={x.shape[0] // 2}  "
          f"cond={cond:.3e}  residual_rms={rms:.5f} Nm  target_rms={y_rms:.5f} Nm")
    print(f"{'param':<4} {'estimate':>13} {'firmware':>13} {'delta':>13}")
    for name, value in zip(PARAMS, theta):
        fw = FW[name]
        print(f"{name:<4} {value:>13.6g} {fw:>13.6g} {value - fw:>13.6g}")
    return theta


def print_static_gravity_report(
    label: str,
    x: np.ndarray,
    y: np.ndarray,
    points: list[StaticPoint],
    args: argparse.Namespace,
    mode_label: str = "gravity-static",
    point_label: str = "static_points",
) -> np.ndarray:
    theta, residual, cond = fit_model(x, y, args.ridge)
    rms = float(np.sqrt(np.mean(residual * residual)))
    y_rms = float(np.sqrt(np.mean(y * y)))
    print(f"\n=== {label} {mode_label} ===")
    print(f"rows={x.shape[0]}  {point_label}={len(points)}  "
          f"cond={cond:.3e}  residual_rms={rms:.5f} Nm  target_rms={y_rms:.5f} Nm")
    print(f"{'param':<4} {'estimate':>13} {'firmware':>13} {'delta':>13}")
    for name, value in zip(GRAVITY_PARAMS, theta):
        fw = FW[name]
        print(f"{name:<4} {value:>13.6g} {fw:>13.6g} {value - fw:>13.6g}")

    residual_big = residual[:len(points)]
    residual_small = residual[len(points):]
    point_abs = np.maximum(np.abs(residual_big), np.abs(residual_small))
    order = np.argsort(point_abs)[::-1]
    print("residual_summary: "
          f"big_rms={np.sqrt(np.mean(residual_big * residual_big)):.5f} Nm  "
          f"small_rms={np.sqrt(np.mean(residual_small * residual_small)):.5f} Nm  "
          f"max_abs={point_abs[order[0]]:.5f} Nm")
    print("worst_points:")
    print("  source point rows duration theta phi tau_big tau_small "
          "res_big res_small")
    for idx in order[:min(8, len(order))]:
        point = points[int(idx)]
        print(f"  {point.source} {point.point:>3d} {point.rows:>5d} "
              f"{point.duration_s:>7.3f} {point.theta_rad:>8.4f} "
              f"{point.phi_rad:>8.4f} {point.tau_big_nm:>8.4f} "
              f"{point.tau_small_nm:>9.4f} {residual_big[idx]:>8.4f} "
              f"{residual_small[idx]:>9.4f}")
    return theta


def print_gravity_firmware_blocks(theta: np.ndarray) -> None:
    values = dict(zip(GRAVITY_PARAMS, theta))
    print("\n--- gravity constants only ---")
    for name in GRAVITY_PARAMS:
        print(f"#define {MACROS[name]:<29} {values[name]: .7g}f")

    print("\n--- control-oriented constants with local ctrl-set mass ---")
    for name in ["A0", "AC", "AS", "B0", "BC", "BS", "C0"]:
        print(f"#define {MACROS[name]:<29} {LOCAL_CTRL_SET_MASS[name]: .7g}f")
    for name in GRAVITY_PARAMS:
        print(f"#define {MACROS[name]:<29} {values[name]: .7g}f")
    for name in ["FVB", "FCB", "FVS", "FCS"]:
        print(f"#define {MACROS[name]:<29} {FW[name]: .7g}f")
    print(f"#define {'DOUBLE_PITCH_MODEL_OMEGA0':<29} {OMEGA0: .7g}f")

    print("\n--- double_pitch_lqr_tune.m gravity block ---")
    for name in ["A0", "AC", "AS", "B0", "BC", "BS", "C0"]:
        print(f"{name} = {LOCAL_CTRL_SET_MASS[name]: .7g};")
    for name in GRAVITY_PARAMS:
        print(f"{name} = {values[name]: .7g};")


def print_firmware_blocks(theta: np.ndarray) -> None:
    values = dict(zip(PARAMS, theta))
    print("\n--- double_pitch_lqr.c constants ---")
    for name in PARAMS:
        print(f"#define {MACROS[name]:<29} {values[name]: .7g}f")
    print(f"#define {'DOUBLE_PITCH_MODEL_OMEGA0':<29} {OMEGA0: .7g}f")

    print("\n--- double_pitch_lqr_tune.m constants ---")
    for name in PARAMS:
        print(f"{name} = {values[name]: .7g};")
    print(f"OMEGA0 = {OMEGA0: .7g};")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csvs", nargs="+", type=Path)
    parser.add_argument("--mode",
                        choices=["dynamic", "gravity-static", "gravity-ramp"],
                        default="dynamic",
                        help="fit full dynamic model or static gravity feedforward")
    parser.add_argument("--axis",
                        choices=["all", "theta", "phi"],
                        default="all",
                        help="use all rows or only one excitation axis")
    parser.add_argument("--target",
                        choices=["measured", "ctrl-set", "cmd"],
                        default="measured",
                        help="torque target used for the fit")
    parser.add_argument("--window", type=int, default=31)
    parser.add_argument("--polyorder", type=int, default=3)
    parser.add_argument("--min-samples", type=int, default=300)
    parser.add_argument("--ridge", type=float, default=0.0)
    parser.add_argument("--keep-limited",
                        action="store_true",
                        help="include rows where torque limit flags are set")
    parser.add_argument("--static-omega-threshold",
                        type=float,
                        default=0.02,
                        help="max absolute speed for a static gravity row, rad/s")
    parser.add_argument("--static-min-segment-s",
                        type=float,
                        default=0.2,
                        help="min duration for one static hold segment")
    parser.add_argument("--static-min-segment-samples",
                        type=int,
                        default=50,
                        help="min rows for one static hold segment")
    parser.add_argument("--static-gap-s",
                        type=float,
                        default=0.05,
                        help="time gap that splits static hold segments")
    parser.add_argument("--min-static-points",
                        type=int,
                        default=8,
                        help="min static hold points required for gravity fit")
    parser.add_argument("--require-running",
                        action="store_true",
                        help="in gravity-static mode, also require running > 0.5 if present")
    parser.add_argument("--ramp-omega-threshold",
                        type=float,
                        default=0.08,
                        help="max absolute speed for a gravity-ramp row, rad/s")
    parser.add_argument("--ramp-bin-rad",
                        type=float,
                        default=0.04,
                        help="theta/psi bin width for gravity-ramp mode, rad")
    parser.add_argument("--ramp-min-bin-samples",
                        type=int,
                        default=20,
                        help="min rows required for one gravity-ramp angle bin")
    parser.add_argument("--min-ramp-bins",
                        type=int,
                        default=20,
                        help="min gravity-ramp angle bins required for fit")
    args = parser.parse_args()

    if args.mode in ("gravity-static", "gravity-ramp") and args.axis != "all":
        print(f"warning: --axis is ignored in --mode {args.mode}",
              file=sys.stderr)

    x_all = []
    y_all = []
    static_points_all: list[StaticPoint] = []
    for csv_path in args.csvs:
        df = load_csv(csv_path, args.mode)
        if args.mode == "gravity-static":
            x, y, extras = build_static_gravity_regression(df, args, csv_path.name)
            points = extras["points"]
            print(f"{csv_path.name}: static_points={len(points)}  "
                  f"raw_still_rows={extras['raw_valid_rows']}  "
                  f"theta_pp={extras['theta_pp']:.6f}rad  "
                  f"phi_pp={extras['phi_pp']:.6f}rad")
            print_static_gravity_report(csv_path.name, x, y, points, args)
            static_points_all.extend(points)
        elif args.mode == "gravity-ramp":
            x, y, extras = build_ramp_gravity_regression(df, args, csv_path.name)
            points = extras["points"]
            print(f"{csv_path.name}: angle_bins={len(points)}  "
                  f"raw_valid_rows={extras['raw_valid_rows']}  "
                  f"theta_pp={extras['theta_pp']:.6f}rad  "
                  f"phi_pp={extras['phi_pp']:.6f}rad  "
                  f"psi_pp={extras['psi_pp']:.6f}rad")
            print_static_gravity_report(csv_path.name,
                                        x,
                                        y,
                                        points,
                                        args,
                                        mode_label="gravity-ramp",
                                        point_label="angle_bins")
            static_points_all.extend(points)
        else:
            x, y, extras = build_regression(df, args)
            print(f"{csv_path.name}: samples={extras['samples']}  "
                  f"dt={extras['dt']:.6f}s  sg_window={extras['window']}")
            print_report(csv_path.name, x, y, args)
        x_all.append(x)
        y_all.append(y)

    x_stack = np.vstack(x_all)
    y_stack = np.concatenate(y_all)
    if args.mode == "gravity-static":
        theta = print_static_gravity_report("ALL",
                                            x_stack,
                                            y_stack,
                                            static_points_all,
                                            args)
        print_gravity_firmware_blocks(theta)
    elif args.mode == "gravity-ramp":
        theta = print_static_gravity_report("ALL",
                                            x_stack,
                                            y_stack,
                                            static_points_all,
                                            args,
                                            mode_label="gravity-ramp",
                                            point_label="angle_bins")
        print_gravity_firmware_blocks(theta)
    else:
        theta = print_report("ALL", x_stack, y_stack, args)
        print_firmware_blocks(theta)
    return 0


if __name__ == "__main__":
    sys.exit(main())
