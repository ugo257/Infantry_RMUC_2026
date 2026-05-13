#!/usr/bin/env python3
"""Fit yaw plant constants from Ozone CSV and print LQRESO defaults.

The default fast profile is tuned for the current yaw LQRESO path:
  J * theta_ddot + B * theta_dot + Fc * sign(theta_dot) = tau

It supports both older simple CSVs and current Ozone exports of
g_yaw_lqr_ident_record, then compares multiple torque sources before choosing
the median high-speed fit.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from pathlib import Path
from typing import Sequence


PROFILES = {
    "fast": {
        "wn": 40.0,
        "damping": 0.85,
        "eso_alpha": 1.10,
        "min_abs_torque": 0.45,
        "min_abs_omega": 1.0,
        "smooth_windows": (11, 21, 41),
    },
    "balanced": {
        "wn": 32.0,
        "damping": 0.95,
        "eso_alpha": 1.10,
        "min_abs_torque": 0.35,
        "min_abs_omega": 0.6,
        "smooth_windows": (11, 21, 41),
    },
    "conservative": {
        "wn": 25.0,
        "damping": 1.05,
        "eso_alpha": 1.00,
        "min_abs_torque": 0.25,
        "min_abs_omega": 0.4,
        "smooth_windows": (11, 21, 41),
    },
}

ALIASES = {
    "time": (
        "time_s",
        "yaw_fit_time_s",
        "time",
        "t",
    ),
    "valid": ("valid",),
    "feedback_ok": ("feedback_ok",),
    "theta": (
        "theta_meas_rad",
        "yaw_angle_rad",
        "theta_rad",
        "yaw_rad",
        "motor_total_pos_rad",
        "motor_pos_rad",
        "yaw_motor_measure_total_pos",
        "yaw_motor_measure_pos",
    ),
    "omega": (
        "omega_meas_rad_s",
        "yaw_gyro_rad_s",
        "omega_rad_s",
        "yaw_gyro_rad",
        "motor_vel_rad_s",
        "yaw_motor_measure_vel",
    ),
    "tau_cmd_nm": (
        "tau_cmd_nm",
        "test_yaw_ref",
        "yaw_torque_cmd_nm",
        "torque_cmd_nm",
    ),
    "ctrl_tor_set_nm": (
        "ctrl_tor_set_nm",
        "yaw_motor_ctrl_tor_set",
    ),
    "tau_meas_nm": (
        "tau_meas_nm",
        "motor_torque_nm",
        "yaw_motor_torque_nm",
        "yaw_motor_measure_tor",
    ),
    "motor_tor_nm": (
        "motor_tor_nm",
        "yaw_motor_measure_tor",
    ),
}

TORQUE_SOURCE_ORDER = (
    "tau_cmd_nm",
    "ctrl_tor_set_nm",
    "tau_meas_nm",
    "motor_tor_nm",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit yaw J/B/Fc and print firmware/Ozone LQRESO parameters."
    )
    parser.add_argument("csv", type=Path, help="Yaw telemetry CSV exported by Ozone")
    parser.add_argument(
        "--profile",
        choices=tuple(PROFILES),
        default="fast",
        help="Tuning target used to convert fitted J/B into LQR gains",
    )
    parser.add_argument(
        "--torque",
        choices=("all", "command", "setpoint", "measured", "motor"),
        default="all",
        help="Torque source selection. fast profile should usually use all.",
    )
    parser.add_argument("--smooth", type=int, default=None, help="Compatibility alias for one smoothing window")
    parser.add_argument(
        "--smooth-windows",
        default=None,
        help="Comma-separated moving-average windows before differentiating omega",
    )
    parser.add_argument("--angle-unit", choices=("rad", "deg"), default="rad")
    parser.add_argument("--gyro-unit", choices=("rad", "deg"), default="rad")
    parser.add_argument("--min-abs-torque", type=float, default=None)
    parser.add_argument("--min-abs-omega", type=float, default=None)
    parser.add_argument("--natural-freq", type=float, default=None, help="Closed-loop wn override, rad/s")
    parser.add_argument("--damping", type=float, default=None, help="Closed-loop damping override")
    parser.add_argument("--eso-alpha", type=float, default=None)
    parser.add_argument("--min-torque-comp", type=float, default=0.40)
    parser.add_argument("--min-torque-comp-band", type=float, default=0.03)
    parser.add_argument(
        "--keep-mincomp",
        action="store_true",
        help="Do not drop rows where tau_cmd sits on the minimum torque compensation",
    )
    parser.add_argument(
        "--write-source",
        type=Path,
        default=None,
        help="Optional lqr_eso.c path to update with the selected defaults",
    )
    return parser.parse_args()


def normalize_name(name: str) -> str:
    text = name.strip()
    prefix = "(g_yaw_lqr_ident_record)."
    if text.startswith(prefix):
        text = text[len(prefix):]
    if text.startswith("g_yaw_lqr_ident_record."):
        text = text.split(".", 1)[1]
    text = text.replace("->", "_").replace(".", "_")
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")


def normalize_columns(fieldnames: Sequence[str]) -> dict[str, str]:
    return {normalize_name(name): name for name in fieldnames}


def pick_column(columns: dict[str, str], key: str, required: bool = True) -> str | None:
    for alias in ALIASES[key]:
        found = columns.get(normalize_name(alias))
        if found is not None:
            return found
    if required:
        raise KeyError(key)
    return None


def parse_float(value: str | None) -> float:
    if value is None:
        return math.nan
    text = value.strip()
    if not text:
        return math.nan
    return float(text)


def moving_average(values: Sequence[float], window: int) -> list[float]:
    if window < 3:
        return list(values)
    if window % 2 == 0:
        window += 1
    half = window // 2
    out: list[float] = []
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)
    for i in range(len(values)):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        out.append((prefix[hi] - prefix[lo]) / (hi - lo))
    return out


def unwrap_rad(values: Sequence[float]) -> list[float]:
    out: list[float] = []
    offset = 0.0
    last = None
    for value in values:
        if last is not None:
            delta = value + offset - last
            if delta > math.pi:
                offset -= 2.0 * math.pi
            elif delta < -math.pi:
                offset += 2.0 * math.pi
        current = value + offset
        out.append(current)
        last = current
    return out


def median(values: Sequence[float]) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def solve_linear(a: list[list[float]], b: list[float]) -> list[float]:
    n = len(b)
    mat = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(mat[r][col]))
        if abs(mat[pivot][col]) < 1e-12:
            raise ValueError("least-squares matrix is singular")
        if pivot != col:
            mat[col], mat[pivot] = mat[pivot], mat[col]
        scale = mat[col][col]
        for j in range(col, n + 1):
            mat[col][j] /= scale
        for r in range(n):
            if r == col:
                continue
            factor = mat[r][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                mat[r][j] -= factor * mat[col][j]
    return [mat[i][n] for i in range(n)]


def least_squares(x_rows: Sequence[Sequence[float]], y_values: Sequence[float]) -> list[float]:
    if not x_rows:
        raise ValueError("no rows for least squares")
    width = len(x_rows[0])
    xtx = [[0.0 for _ in range(width)] for _ in range(width)]
    xty = [0.0 for _ in range(width)]
    for x, y in zip(x_rows, y_values):
        for r in range(width):
            xty[r] += x[r] * y
            for c in range(width):
                xtx[r][c] += x[r] * x[c]
    return solve_linear(xtx, xty)


def source_keys(args: argparse.Namespace, available: set[str]) -> list[str]:
    if args.torque == "all":
        keys = list(TORQUE_SOURCE_ORDER)
    elif args.torque == "command":
        keys = ["tau_cmd_nm"]
    elif args.torque == "setpoint":
        keys = ["ctrl_tor_set_nm"]
    elif args.torque == "measured":
        keys = ["tau_meas_nm"]
    else:
        keys = ["motor_tor_nm"]
    return [key for key in keys if key in available]


def smooth_windows(args: argparse.Namespace) -> list[int]:
    profile = PROFILES[args.profile]
    if args.smooth is not None:
        return [args.smooth]
    if args.smooth_windows is not None:
        return [int(item) for item in args.smooth_windows.split(",") if item.strip()]
    return list(profile["smooth_windows"])


def load_samples(csv_path: Path, args: argparse.Namespace) -> tuple[list[dict[str, float]], set[str]]:
    with csv_path.open("r", encoding="utf-8-sig", newline="") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header row")
        columns = normalize_columns(reader.fieldnames)
        time_col = pick_column(columns, "time")
        theta_col = pick_column(columns, "theta")
        omega_col = pick_column(columns, "omega")
        valid_col = pick_column(columns, "valid", required=False)
        feedback_col = pick_column(columns, "feedback_ok", required=False)
        torque_cols = {
            key: column
            for key in TORQUE_SOURCE_ORDER
            if (column := pick_column(columns, key, required=False)) is not None
        }

        samples: list[dict[str, float]] = []
        for row in reader:
            valid = parse_float(row.get(valid_col)) if valid_col is not None else 1.0
            feedback_ok = parse_float(row.get(feedback_col)) if feedback_col is not None else 1.0
            if valid_col is not None and valid != 1.0:
                continue
            if feedback_col is not None and feedback_ok != 1.0:
                continue

            sample = {
                "time": parse_float(row.get(time_col)),
                "theta": parse_float(row.get(theta_col)),
                "omega": parse_float(row.get(omega_col)),
            }
            for key, column in torque_cols.items():
                sample[key] = parse_float(row.get(column))
            if all(math.isfinite(sample[key]) for key in ("time", "theta", "omega")):
                samples.append(sample)

    if len(samples) < 20:
        raise ValueError(f"not enough valid samples: {len(samples)}")

    samples.sort(key=lambda item: item["time"])
    deduped: list[dict[str, float]] = []
    last_t = None
    for sample in samples:
        if last_t is None or sample["time"] > last_t:
            deduped.append(sample)
            last_t = sample["time"]
    samples = deduped

    if args.angle_unit == "deg":
        for sample in samples:
            sample["theta"] *= math.pi / 180.0
    if args.gyro_unit == "deg":
        for sample in samples:
            sample["omega"] *= math.pi / 180.0

    unwrapped_theta = unwrap_rad([sample["theta"] for sample in samples])
    for sample, theta in zip(samples, unwrapped_theta):
        sample["theta"] = theta

    return samples, set(torque_cols)


def fit_source(
    samples: Sequence[dict[str, float]],
    source: str,
    window: int,
    args: argparse.Namespace,
) -> dict[str, float] | None:
    profile = PROFILES[args.profile]
    min_abs_torque = args.min_abs_torque if args.min_abs_torque is not None else profile["min_abs_torque"]
    min_abs_omega = args.min_abs_omega if args.min_abs_omega is not None else profile["min_abs_omega"]

    time_s = [sample["time"] for sample in samples]
    omega = moving_average([sample["omega"] for sample in samples], window)
    half_span = max(1, window // 4)
    x_rows: list[list[float]] = []
    y_values: list[float] = []

    for i in range(half_span, len(samples) - half_span):
        denom = time_s[i + half_span] - time_s[i - half_span]
        if denom <= 0.0:
            continue
        tau = samples[i].get(source, math.nan)
        alpha = (omega[i + half_span] - omega[i - half_span]) / denom
        if not all(math.isfinite(v) for v in (tau, omega[i], alpha)):
            continue
        if abs(tau) < min_abs_torque or abs(omega[i]) < min_abs_omega:
            continue
        if not args.keep_mincomp:
            tau_cmd = samples[i].get("tau_cmd_nm", math.nan)
            if math.isfinite(tau_cmd) and abs(abs(tau_cmd) - args.min_torque_comp) <= args.min_torque_comp_band:
                continue
        sign_omega = 1.0 if omega[i] > 0.05 else (-1.0 if omega[i] < -0.05 else 0.0)
        x_rows.append([tau, omega[i], sign_omega, 1.0])
        y_values.append(alpha)

    if len(y_values) < 100:
        return None

    coeff = least_squares(x_rows, y_values)
    inv_j, omega_coeff, coulomb_coeff, bias_coeff = coeff
    if inv_j <= 0.0:
        return None

    residuals = [
        y - sum(c * x for c, x in zip(coeff, x_row))
        for x_row, y in zip(x_rows, y_values)
    ]
    yaw_j = 1.0 / inv_j
    yaw_b = -omega_coeff / inv_j
    friction = -coulomb_coeff / inv_j
    disturbance = bias_coeff / inv_j
    return {
        "source": source,
        "window": float(window),
        "sample_count": float(len(y_values)),
        "yaw_j": yaw_j,
        "yaw_b": yaw_b,
        "friction": friction,
        "disturbance": disturbance,
        "rmse_alpha": math.sqrt(sum(v * v for v in residuals) / len(residuals)),
        "mae_alpha": sum(abs(v) for v in residuals) / len(residuals),
    }


def simulate_step(values: dict[str, float], step_deg: float, sim_time_s: float = 1.2) -> dict[str, float]:
    dt = 0.001
    theta = 0.0
    omega = 0.0
    tau_last = 0.0
    target = step_deg * math.pi / 180.0
    target_sign = 1.0 if step_deg >= 0.0 else -1.0
    torque_limit = 7.0
    slew_rate = 1000.0
    min_torque_comp = 0.40
    vel_deadband = 0.03
    band = abs(target) * 0.02
    history: list[tuple[float, float, float, float]] = []
    saturation_count = 0

    for k in range(int(sim_time_s / dt)):
        t = k * dt
        omega_ref = 0.0
        if k == 0:
            omega_ref = 30.0 * target_sign
        e_theta = theta - target
        e_omega = omega - omega_ref
        tau_ff = values["yaw_b"] * omega_ref
        tau = tau_ff - values["k_theta"] * e_theta - values["k_omega"] * e_omega
        tau = max(-torque_limit, min(torque_limit, tau))
        max_delta = slew_rate * dt
        tau = max(tau_last - max_delta, min(tau_last + max_delta, tau))
        if abs(omega) < vel_deadband and abs(e_theta) > 0.0 and abs(tau) < min_torque_comp:
            tau = -math.copysign(min_torque_comp, e_theta)
        if abs(tau) >= torque_limit - 1.0e-9:
            saturation_count += 1

        alpha = (tau - values["yaw_b"] * omega) / values["yaw_j"]
        omega += alpha * dt
        theta += omega * dt
        tau_last = tau
        history.append((t, theta, omega, tau))

    settle_time = math.nan
    for index, item in enumerate(history):
        if all(abs(row[1] - target) <= band and abs(row[2]) < 0.05 for row in history[index:]):
            settle_time = item[0]
            break

    peak_deg = max(target_sign * row[1] for row in history) * 180.0 / math.pi
    return {
        "step_deg": step_deg,
        "settle_time": settle_time,
        "overshoot_deg": max(0.0, peak_deg - abs(step_deg)),
        "final_error_deg": (history[-1][1] - target) * 180.0 / math.pi,
        "max_torque": max(abs(row[3]) for row in history),
        "max_omega": max(abs(row[2]) for row in history),
        "saturation_count": float(saturation_count),
    }


def fit(csv_path: Path, args: argparse.Namespace) -> dict[str, float | list[dict[str, float]]]:
    samples, available_sources = load_samples(csv_path, args)
    sources = source_keys(args, available_sources)
    if not sources:
        raise ValueError(f"no requested torque sources found; available={sorted(available_sources)}")

    fits: list[dict[str, float]] = []
    for source in sources:
        for window in smooth_windows(args):
            result = fit_source(samples, source, window, args)
            if result is not None:
                fits.append(result)

    if not fits:
        raise ValueError("no usable fit variants after filtering")

    profile = PROFILES[args.profile]
    yaw_j = median([item["yaw_j"] for item in fits])
    yaw_b = median([item["yaw_b"] for item in fits])
    friction = median([item["friction"] for item in fits])
    disturbance = median([item["disturbance"] for item in fits])
    natural_freq = args.natural_freq if args.natural_freq is not None else profile["wn"]
    damping = args.damping if args.damping is not None else profile["damping"]
    eso_alpha = args.eso_alpha if args.eso_alpha is not None else profile["eso_alpha"]
    if yaw_j <= 0.0 or natural_freq <= 0.0 or damping <= 0.0:
        raise ValueError("invalid selected J/wn/damping")

    k_theta = yaw_j * natural_freq * natural_freq
    k_omega = max(0.0, 2.0 * damping * natural_freq * yaw_j - yaw_b)
    w0 = eso_alpha * math.sqrt(max(k_theta / yaw_j, 0.0))

    values: dict[str, float | list[dict[str, float]]] = {
        "profile": args.profile,
        "sample_count": float(len(samples)),
        "yaw_j": yaw_j,
        "yaw_b": yaw_b,
        "friction": friction,
        "disturbance": disturbance,
        "k_theta": k_theta,
        "k_omega": k_omega,
        "natural_freq": natural_freq,
        "damping": damping,
        "eso_alpha": eso_alpha,
        "eso_w0": w0,
        "eso_beta1": 3.0 * w0,
        "eso_beta2": 3.0 * w0 * w0,
        "eso_beta3": w0 * w0 * w0,
        "fit_variants": fits,
    }
    return values


def fmt_define(value: float) -> str:
    text = f"{value:.9g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return f"{text}f"


def update_source(path: Path, values: dict[str, float | list[dict[str, float]]]) -> None:
    replacements = {
        "YAW_J_DEFAULT": float(values["yaw_j"]),
        "YAW_B_DEFAULT": float(values["yaw_b"]),
        "LQR_K_THETA_DEFAULT": float(values["k_theta"]),
        "LQR_K_OMEGA_DEFAULT": float(values["k_omega"]),
        "YAW_ESO_ALPHA_DEFAULT": float(values["eso_alpha"]),
    }
    text = path.read_text(encoding="utf-8")
    for name, value in replacements.items():
        pattern = rf"(^#define\s+{name}\s+)([-+0-9.eEfF]+)"
        text, count = re.subn(pattern, rf"\g<1>{fmt_define(value)}", text, count=1, flags=re.MULTILINE)
        if count != 1:
            raise ValueError(f"did not find define {name} in {path}")
    path.write_text(text, encoding="utf-8")


def print_result(values: dict[str, float | list[dict[str, float]]]) -> None:
    fits = values["fit_variants"]
    assert isinstance(fits, list)

    print("Yaw fit result")
    print(f"  profile:            {values['profile']}")
    print(f"  valid samples:      {int(float(values['sample_count']))}")
    print(f"  fit variants:       {len(fits)}")
    print()
    print("High-speed Coulomb-separated fits")
    for item in fits:
        print(
            "  {source:14s} w={window:2.0f} n={sample_count:4.0f} "
            "J={yaw_j:.6g} B={yaw_b:.6g} Fc={friction:.6g} "
            "RMSE/MAE={rmse_alpha:.4g}/{mae_alpha:.4g}".format(**item)
        )

    print()
    print("Selected median model")
    print(f"  J:                  {float(values['yaw_j']):.9g} kg*m^2")
    print(f"  B:                  {float(values['yaw_b']):.9g} N*m*s/rad")
    print(f"  Coulomb friction:   {float(values['friction']):.9g} N*m")
    print(f"  disturbance torque: {float(values['disturbance']):.9g} N*m")
    print(f"  target wn/zeta:     {float(values['natural_freq']):.9g} / {float(values['damping']):.9g}")

    print()
    print("Recommended lqr_eso.c defaults")
    print(f"#define YAW_J_DEFAULT                       {fmt_define(float(values['yaw_j']))}")
    print(f"#define YAW_B_DEFAULT                       {fmt_define(float(values['yaw_b']))}")
    print(f"#define LQR_K_THETA_DEFAULT                 {fmt_define(float(values['k_theta']))}")
    print(f"#define LQR_K_OMEGA_DEFAULT                 {fmt_define(float(values['k_omega']))}")
    print(f"#define YAW_ESO_ALPHA_DEFAULT               {fmt_define(float(values['eso_alpha']))}")

    print()
    print("Ozone watch values")
    print(f"g_lqr_k_theta = {fmt_define(float(values['k_theta']))};")
    print(f"g_lqr_k_omega = {fmt_define(float(values['k_omega']))};")
    print(f"g_yaw_eso_alpha = {fmt_define(float(values['eso_alpha']))};")

    print()
    print("Derived ESO runtime values")
    print(f"  w0:    {float(values['eso_w0']):.9g} rad/s")
    print(f"  beta1: {float(values['eso_beta1']):.9g}")
    print(f"  beta2: {float(values['eso_beta2']):.9g}")
    print(f"  beta3: {float(values['eso_beta3']):.9g}")

    print()
    print("Firmware-style step sanity check")
    for step in (10.0, 30.0):
        result = simulate_step({key: float(values[key]) for key in ("yaw_j", "yaw_b", "k_theta", "k_omega")}, step)
        settle = result["settle_time"]
        settle_text = "nan" if math.isnan(settle) else f"{settle:.3f}s"
        print(
            f"  {step:4.0f} deg: settle={settle_text}, overshoot={result['overshoot_deg']:.4g} deg, "
            f"max_tau={result['max_torque']:.4g} Nm, max_omega={result['max_omega']:.4g} rad/s, "
            f"sat_samples={int(result['saturation_count'])}"
        )

    fallback_j = float(values["yaw_j"])
    fallback_b = float(values["yaw_b"])
    fallback_kt = fallback_j * 38.0 * 38.0
    fallback_kw = max(0.0, 2.0 * 0.85 * 38.0 * fallback_j - fallback_b)
    print()
    print("First rollback watch values if high-frequency oscillation appears")
    print(f"g_lqr_k_theta = {fmt_define(fallback_kt)};")
    print(f"g_lqr_k_omega = {fmt_define(fallback_kw)};")


def main() -> int:
    args = parse_args()
    try:
        values = fit(args.csv, args)
        print_result(values)
        if args.write_source is not None:
            update_source(args.write_source, values)
            print(f"\nUpdated: {args.write_source}")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
