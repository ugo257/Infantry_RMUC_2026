#!/usr/bin/env python3
"""Identify pitch UP plant parameters from g_pitch_up_ident_debug CSVs.

Model (small / pitch-up axis of the coupled DoublePitchLqr):

  tau_small = B0 * thetaB_ddot + C0 * phi_ddot
            + GP1 * sin(thetaB + phi) + GP2 * cos(thetaB + phi)
            + FVS * omega_small + FCS * smooth_sign(omega_small)

(See modules/double_pitch_lqr/double_pitch_lqr.c, with AC=AS=BC=BS=0 so
m22 = C0 and m12 = B0 are constants; Coriolis c_small = 0.)

The measured torque is up_motor_tor_nm (DM motor feedback), already in the
controller frame because PITCH_UP_CONTROL_TO_MOTOR_TORQUE_SIGN = +1.

Usage:
  pitch_up_double_ident.py CSV [CSV ...]   # default: 4 pitch_auto_*.csv
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


# Firmware constants (modules/double_pitch_lqr/double_pitch_lqr.c)
FW = {
    "B0": -0.0057,
    "C0": 0.0188,
    "GP1": 0.0,
    "GP2": -0.9729,
    "FVS": 0.0613,
    "FCS": 0.1347,
}

PARAM_NAMES = ["B0", "C0", "GP1", "GP2", "FVS", "FCS"]
UNITS = ["kg·m²", "kg·m²", "Nm", "Nm", "Nm·s/rad", "Nm"]
PREFIX = "(g_pitch_up_ident_debug)."

# Filtering windows for derivative smoothing (Savitzky-Golay, odd lengths).
SAVGOL_WINDOW = 21
SAVGOL_ORDER = 3
# Coulomb friction smoothing band (rad/s) - same form as DoublePitchSmoothSign.
SMOOTH_SIGN_BAND = 0.05


def smooth_sign(x: np.ndarray, band: float = SMOOTH_SIGN_BAND) -> np.ndarray:
    return np.tanh(x / band)


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df.columns = [c.replace(PREFIX, "") for c in df.columns]
    return df


def build_regression(df: pd.DataFrame):
    """Return (X, y, mask_used, t, extras) from one CSV."""
    t = df["time_s"].to_numpy(dtype=float)
    # Sample period for derivatives
    dt = np.median(np.diff(t))

    theta_b = df["down_pos_rad"].to_numpy(dtype=float)
    omega_b = df["down_vel_rad_s"].to_numpy(dtype=float)
    phi = df["phi_rel_rad"].to_numpy(dtype=float)
    omega_s = df["omega_small_rad_s"].to_numpy(dtype=float)
    # Measured up-axis torque
    tau_meas = df["up_motor_tor_nm"].to_numpy(dtype=float)
    tau_cmd = df["up_tau_cmd_nm"].to_numpy(dtype=float)

    # Smooth-then-differentiate to get accelerations.
    # Savgol over omega gives a much cleaner derivative than raw diff.
    n_min = SAVGOL_WINDOW
    if len(omega_b) < n_min or len(omega_s) < n_min:
        raise ValueError(f"trace too short for SG window {SAVGOL_WINDOW}")
    theta_b_ddot = savgol_filter(omega_b, SAVGOL_WINDOW, SAVGOL_ORDER,
                                 deriv=1, delta=dt, mode="interp")
    phi_ddot = savgol_filter(omega_s, SAVGOL_WINDOW, SAVGOL_ORDER,
                             deriv=1, delta=dt, mode="interp")

    sin_tp = np.sin(theta_b + phi)
    cos_tp = np.cos(theta_b + phi)
    ssgn = smooth_sign(omega_s)

    X = np.column_stack([theta_b_ddot, phi_ddot, sin_tp, cos_tp, omega_s, ssgn])
    y = tau_meas

    # Mask: only steady-controller operation, drop the SG edge artifacts.
    valid = (
        (df["valid"].to_numpy() > 0.5)
        & (df["torque_output_allowed"].to_numpy() > 0.5)
        & (df["auto_ref_active"].to_numpy() > 0.5)
        & (df["feedback_ok"].to_numpy() > 0.5)
    )
    valid[: SAVGOL_WINDOW] = False
    valid[-SAVGOL_WINDOW:] = False
    # Drop rows where any regressor or target is non-finite (rare, but the
    # raw Ozone export sometimes has stray inf/NaN at frame boundaries).
    finite = np.all(np.isfinite(X), axis=1) & np.isfinite(y)
    mask = valid & finite

    return X, y, mask, t, dict(dt=dt, tau_cmd=tau_cmd, tau_meas=tau_meas,
                                omega_s=omega_s)


def fit_ols(X: np.ndarray, y: np.ndarray):
    """Plain least squares. Return (theta_hat, std_err, residuals, cond)."""
    theta_hat, *_ = np.linalg.lstsq(X, y, rcond=None)
    residuals = y - X @ theta_hat
    n, p = X.shape
    dof = max(n - p, 1)
    sigma2 = float(np.dot(residuals, residuals) / dof)
    try:
        cov = sigma2 * np.linalg.inv(X.T @ X)
        std_err = np.sqrt(np.maximum(np.diag(cov), 0.0))
    except np.linalg.LinAlgError:
        std_err = np.full(p, np.nan)
    # Condition number of the normal-equations matrix.
    sv = np.linalg.svd(X, compute_uv=False)
    cond = float(sv[0] / sv[-1]) if sv[-1] > 0 else float("inf")
    return theta_hat, std_err, residuals, cond


def fit_ridge(X: np.ndarray, y: np.ndarray, lam: float = 1e-4):
    """Tikhonov-regularised LS, mostly to confirm OLS isn't blowing up."""
    p = X.shape[1]
    theta_hat = np.linalg.solve(X.T @ X + lam * np.eye(p), X.T @ y)
    residuals = y - X @ theta_hat
    return theta_hat, residuals


def report_run(label: str, X: np.ndarray, y: np.ndarray):
    theta, std, resid, cond = fit_ols(X, y)
    print(f"\n=== {label}  (n={X.shape[0]} samples) ===")
    print(f"  cond(X) = {cond:.3e}    "
          f"residual RMS = {np.sqrt(np.mean(resid**2)):.4f} Nm    "
          f"|tau| RMS = {np.sqrt(np.mean(y**2)):.4f} Nm")
    print(f"  {'param':<5} {'est':>11} {'±1σ':>11}  {'fw':>11}  {'Δ/fw%':>8}  unit")
    for name, est, se, unit in zip(PARAM_NAMES, theta, std, UNITS):
        fw = FW[name]
        pct = (est - fw) / fw * 100.0 if fw != 0 else float("nan")
        pct_s = f"{pct:+7.1f}" if np.isfinite(pct) else "    n/a"
        print(f"  {name:<5} {est:>11.5f} {se:>11.5f}  {fw:>11.5f}  "
              f"{pct_s}  {unit}")
    return theta, resid


def predict(X: np.ndarray, theta: np.ndarray) -> np.ndarray:
    return X @ theta


def plot_overview(per_file, theta_global, out_path: Path):
    fig, axes = plt.subplots(len(per_file), 2, figsize=(13, 2.6 * len(per_file)),
                             squeeze=False, sharex=False)
    for i, item in enumerate(per_file):
        name = item["name"]
        t = item["t_used"]
        y = item["y_used"]
        pred_local = item["pred_local"]
        pred_global = item["pred_global"]
        ax = axes[i, 0]
        ax.plot(t, y, color="#222", lw=0.6, label="τ_motor_meas")
        ax.plot(t, pred_local, color="C0", lw=0.8, alpha=0.85, label="fit (per-file θ)")
        ax.plot(t, pred_global, color="C3", lw=0.8, alpha=0.85, label="fit (global θ)")
        ax.set_title(f"{name}: τ vs prediction", fontsize=10)
        ax.set_ylabel("Nm")
        ax.grid(alpha=0.3)
        if i == 0:
            ax.legend(loc="upper right", fontsize=7, frameon=False)

        ax2 = axes[i, 1]
        ax2.plot(t, y - pred_global, color="C3", lw=0.5, label="residual (global)")
        ax2.plot(t, y - pred_local, color="C0", lw=0.5, alpha=0.6, label="residual (local)")
        ax2.set_title(f"{name}: residual", fontsize=10)
        ax2.set_ylabel("Nm")
        ax2.grid(alpha=0.3)
        if i == 0:
            ax2.legend(loc="upper right", fontsize=7, frameon=False)
    axes[-1, 0].set_xlabel("time (s)")
    axes[-1, 1].set_xlabel("time (s)")
    fig.suptitle(f"PitchUp double-LQR plant identification\n"
                 f"global θ = {dict(zip(PARAM_NAMES, np.round(theta_global, 4)))}",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=140)
    print(f"\nfigure → {out_path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    default_csvs = sorted(Path("/Users/lamda/ozone_csv").glob("pitch_auto_*.csv"))
    ap.add_argument("csvs", nargs="*", type=Path, default=default_csvs,
                    help="CSV files (default: pitch_auto_*.csv)")
    ap.add_argument("--out", type=Path,
                    default=Path("/Users/lamda/ozone_csv/pitch_up_ident_report.png"))
    args = ap.parse_args()

    if not args.csvs:
        ap.error("no CSVs found")

    per_file = []
    X_stack = []
    y_stack = []
    for csv_path in args.csvs:
        df = load_csv(csv_path)
        X, y, mask, t, extras = build_regression(df)
        Xm = X[mask]
        ym = y[mask]
        tm = t[mask] - t[mask][0]
        theta_local, _ = report_run(csv_path.name, Xm, ym)
        per_file.append(dict(
            name=csv_path.name, X_used=Xm, y_used=ym, t_used=tm,
            theta_local=theta_local,
            pred_local=predict(Xm, theta_local),
            extras=extras,
        ))
        X_stack.append(Xm)
        y_stack.append(ym)

    X_all = np.vstack(X_stack)
    y_all = np.concatenate(y_stack)
    theta_global, _ = report_run("ALL 4 traces (stacked)", X_all, y_all)

    for item in per_file:
        item["pred_global"] = predict(item["X_used"], theta_global)

    plot_overview(per_file, theta_global, args.out)

    return 0


if __name__ == "__main__":
    sys.exit(main())
