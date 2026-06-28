#!/usr/bin/env python3
"""One-shot ARX identification for the yaw auto-aim plant.

Model (continuous):
    J * d_omega/dt + B * omega + Fc * sign(omega) = tau + d

ZOH-discretized over a (variable) dt sample (first-order LTI):
    omega[k+1] = a*omega[k] + b*( tau[k] - Fc*sign(omega[k]) + d )
        with a = exp(-B/J * dt),  b = (1 - a) / B

For mixed dt we still linearize per-step. We regress the unknowns:
    omega[k+1] = c0*omega[k] + c1*tau[k] + c2*sign(omega[k]) + c3

and then recover (J, B, Fc, d) using the *average* dt of the surviving rows.
This avoids numerically differentiating omega and is robust to low PE data.

Inputs are read straight from the Ozone CSV.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

PFX = "(g_yaw_auto_aim_debug_record)."


def load(csv_path: Path, torque_col: str):
    with csv_path.open("r", encoding="utf-8-sig") as fp:
        rdr = csv.DictReader(fp)
        rows = []
        for row in rdr:
            try:
                rows.append({
                    "t":    float(row[PFX + "time_s"]),
                    "dt":   float(row[PFX + "dt_s"]),
                    "auto": float(row[PFX + "auto_aim_mode"]),
                    "ok":   float(row[PFX + "feedback_ok"]),
                    "val":  float(row[PFX + "valid"]),
                    "th":   float(row[PFX + "theta_meas_rad"]),
                    "om":   float(row[PFX + "omega_meas_rad_s"]),
                    "tau":  float(row[PFX + torque_col]),
                    "sat":  float(row[PFX + "limit_active"]),
                })
            except (KeyError, ValueError):
                continue
    return rows


def lstsq(X, y):
    n = len(X[0])
    XtX = [[0.0] * n for _ in range(n)]
    Xty = [0.0] * n
    for xi, yi in zip(X, y):
        for r in range(n):
            Xty[r] += xi[r] * yi
            for c in range(n):
                XtX[r][c] += xi[r] * xi[c]
    A = [row[:] + [b] for row, b in zip(XtX, Xty)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(A[r][col]))
        if abs(A[piv][col]) < 1e-15:
            raise ValueError("singular")
        if piv != col:
            A[col], A[piv] = A[piv], A[col]
        s = A[col][col]
        for j in range(col, n + 1):
            A[col][j] /= s
        for r in range(n):
            if r == col:
                continue
            f = A[r][col]
            if f == 0.0:
                continue
            for j in range(col, n + 1):
                A[r][j] -= f * A[col][j]
    return [A[i][n] for i in range(n)]


def run(rows, *, omega_gate=0.0, drop_sat=True, label=""):
    samples = [r for r in rows
               if r["auto"] == 1.0 and r["ok"] == 1.0 and r["val"] == 1.0
               and (not drop_sat or r["sat"] == 0.0)]
    if len(samples) < 200:
        print(f"[{label}] insufficient rows after filter: {len(samples)}")
        return None
    samples.sort(key=lambda r: r["t"])

    X, y, dts = [], [], []
    for k in range(len(samples) - 1):
        a, b = samples[k], samples[k + 1]
        dt = b["t"] - a["t"]
        if dt <= 0.0005 or dt > 0.005:
            continue
        if abs(a["om"]) < omega_gate:
            continue
        sgn = 1.0 if a["om"] > 0.05 else (-1.0 if a["om"] < -0.05 else 0.0)
        X.append([a["om"], a["tau"], sgn, 1.0])
        y.append(b["om"])
        dts.append(dt)
    if len(y) < 200:
        print(f"[{label}] insufficient regression rows: {len(y)}")
        return None

    c = lstsq(X, y)
    c0, c1, c2, c3 = c

    dt_mean = sum(dts) / len(dts)
    if not (0.0 < c0 < 1.0):
        print(f"[{label}] c0={c0:.5f} out of (0,1) — model invalid")
        return None
    if c1 <= 0.0:
        print(f"[{label}] c1={c1:.5f} non-positive — model invalid")
        return None

    # a = c0  =>  B/J = -ln(c0) / dt_mean
    # b = c1 = (1-a)/B  =>  B = (1-a)/c1  =>  J = B / (B/J)
    a = c0
    BJ = -math.log(a) / dt_mean
    B = (1.0 - a) / c1
    J = B / BJ
    Fc = -c2 / c1
    d = c3 / c1

    res = [yi - sum(ci * xi for ci, xi in zip(c, xi_row)) for xi_row, yi in zip(X, y)]
    rmse = math.sqrt(sum(r * r for r in res) / len(res))
    yvar = sum((yi - sum(y)/len(y))**2 for yi in y) / len(y)
    r2 = 1.0 - (rmse * rmse) / yvar if yvar > 0 else float("nan")

    print(f"[{label}] n={len(y):5d}  dt={dt_mean*1000:.3f}ms  "
          f"J={J:.5f}  B={B:.5f}  Fc={Fc:+.4f}  d={d:+.4f}  "
          f"RMSE(om)={rmse*1000:.2f}mrad/s  R^2={r2:.6f}")
    return {"J": J, "B": B, "Fc": Fc, "d": d, "n": len(y), "rmse": rmse, "r2": r2}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    args = ap.parse_args()
    print(f"Source: {args.csv}")
    print(f"Model: omega[k+1] = c0*omega[k] + c1*tau[k] + c2*sgn(omega[k]) + c3")
    print(f"Recovers: J,B,Fc,d via continuous-time mapping at mean dt\n")

    for tcol, label in [
        ("tau_cmd_nm",       "tau_cmd"),
        ("ctrl_tor_set_nm",  "ctrl_set"),
        ("tau_meas_nm",      "tau_meas"),
        ("tau_meas_filt_nm", "tau_filt"),
        ("motor_tor_nm",     "motor_tor"),
    ]:
        rows = load(args.csv, tcol)
        for gate in (0.0, 0.1, 0.3):
            run(rows, omega_gate=gate, label=f"{label:9s} gate={gate}")
        print()


if __name__ == "__main__":
    main()
