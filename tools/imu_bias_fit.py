#!/usr/bin/env python3
"""Fit CHASSIS_BOARD BMI088 static offsets from Ozone CSV export."""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from pathlib import Path


DEFAULT_SAMPLE_TARGET = 20000
DEFAULT_GRAVITY = 9.81


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read g_imu_bias_calib_record CSV and print BMI088_CHASSIS_PRE_CALI_* macros."
    )
    parser.add_argument("csv", type=Path, help="Ozone CSV exported from g_imu_bias_calib_record")
    parser.add_argument("--gravity", type=float, default=DEFAULT_GRAVITY, help="Gravity in m/s^2")
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="Use the last row even when no done=1 row reaches sample_target",
    )
    return parser.parse_args()


def canon(name: str) -> str:
    text = name.strip().lower()
    text = text.replace("[", "_").replace("]", "")
    text = re.sub(r"[^a-z0-9_]+", "_", text)
    text = re.sub(r"_+", "_", text).strip("_")
    return text


def field_tail(name: str) -> str:
    parts = name.strip().split(".")
    if len(parts) >= 2 and parts[-1].isdigit():
        return f"{parts[-2]}_{parts[-1]}"
    return parts[-1]


def row_lookup(row: dict[str, str]) -> dict[str, str]:
    lookup: dict[str, str] = {}
    for key, value in row.items():
        if key is None:
            continue
        clean_value = value.strip() if isinstance(value, str) else value
        lookup[canon(key)] = clean_value
        lookup[canon(field_tail(key))] = clean_value
    return lookup


def get_text(lookup: dict[str, str], name: str, index: int | None = None) -> str | None:
    names = []
    if index is None:
        names.append(name)
    else:
        names.extend(
            (
                f"{name}_{index}",
                f"{name}{index}",
                f"{name}[{index}]",
                f"{name}.{index}",
            )
        )
    for candidate in names:
        value = lookup.get(canon(candidate))
        if value not in (None, ""):
            return value
    return None


def get_float(lookup: dict[str, str], name: str, index: int | None = None, default: float = math.nan) -> float:
    text = get_text(lookup, name, index)
    if text is None:
        return default
    try:
        return float(text)
    except ValueError:
        return default


def get_int(lookup: dict[str, str], name: str, default: int = 0) -> int:
    value = get_float(lookup, name, default=math.nan)
    if not math.isfinite(value):
        return default
    return int(value)


def get_vec3(lookup: dict[str, str], name: str) -> list[float]:
    values = [get_float(lookup, name, i) for i in range(3)]
    missing = [i for i, value in enumerate(values) if not math.isfinite(value)]
    if missing:
        raise ValueError(f"missing {name}[{', '.join(str(i) for i in missing)}] columns")
    return values


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as fp:
        rows = list(csv.DictReader(fp))
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    return rows


def select_row(rows: list[dict[str, str]], allow_incomplete: bool) -> tuple[dict[str, str], bool]:
    complete: list[dict[str, str]] = []
    done_rows: list[dict[str, str]] = []

    for row in rows:
        lookup = row_lookup(row)
        sample_count = get_int(lookup, "sample_count")
        sample_target = get_int(lookup, "sample_target", DEFAULT_SAMPLE_TARGET)
        done = get_int(lookup, "done")
        valid = get_int(lookup, "valid")
        if done != 0:
            done_rows.append(row)
        if done != 0 and valid != 0 and sample_count >= sample_target:
            complete.append(row)

    if complete:
        return complete[-1], True
    if allow_incomplete:
        if done_rows:
            return done_rows[-1], False
        return rows[-1], False
    raise ValueError("no complete done=1 valid=1 row with sample_count >= sample_target")


def fmt_macro(name: str, value: float) -> str:
    return f"#define {name:<39} {value:.9g}f"


def print_vec(label: str, values: list[float], unit: str) -> None:
    print(f"{label}: [{values[0]:.9g}, {values[1]:.9g}, {values[2]:.9g}] {unit}")


def main() -> int:
    args = parse_args()
    try:
        rows = load_rows(args.csv)
        row, complete = select_row(rows, args.allow_incomplete)
        lookup = row_lookup(row)

        gyro_mean = get_vec3(lookup, "gyro_mean_body_raw")
        acc_mean = get_vec3(lookup, "acc_mean_body_raw")
        gyro_std = get_vec3(lookup, "gyro_std_body_raw")
        acc_std = get_vec3(lookup, "acc_std_body_raw")
        old_gyro = [get_float(lookup, "old_gyro_offset", i) for i in range(3)]
        old_acc = [get_float(lookup, "old_acc_offset", i) for i in range(3)]

        new_gyro = gyro_mean
        new_acc = [acc_mean[0], acc_mean[1], acc_mean[2] - args.gravity]

        sample_count = get_int(lookup, "sample_count")
        sample_target = get_int(lookup, "sample_target", DEFAULT_SAMPLE_TARGET)
        invalid_count = get_int(lookup, "invalid_count")
        run_id = get_int(lookup, "run_id")
        temperature = get_float(lookup, "temperature_c")
        acc_norm = math.sqrt(sum(value * value for value in acc_mean))

    except Exception as exc:
        print(f"[imu_bias_fit] error: {exc}", file=sys.stderr)
        return 2

    print(f"source: {args.csv}")
    print(f"run_id={run_id} complete={int(complete)} samples={sample_count}/{sample_target} invalid={invalid_count}")
    print(f"temperature_c={temperature:.3f} acc_mean_norm={acc_norm:.6f} m/s^2")
    print_vec("gyro_mean_body_raw", gyro_mean, "rad/s")
    print_vec("gyro_std_body_raw", gyro_std, "rad/s")
    print_vec("acc_mean_body_raw", acc_mean, "m/s^2")
    print_vec("acc_std_body_raw", acc_std, "m/s^2")

    if all(math.isfinite(value) for value in old_gyro + old_acc):
        print_vec("old_gyro_offset", old_gyro, "rad/s")
        print_vec("old_acc_offset", old_acc, "m/s^2")

    print()
    print("Paste into application/robot_params.h:")
    print(fmt_macro("BMI088_CHASSIS_PRE_CALI_GYRO_X_OFFSET", new_gyro[0]))
    print(fmt_macro("BMI088_CHASSIS_PRE_CALI_GYRO_Y_OFFSET", new_gyro[1]))
    print(fmt_macro("BMI088_CHASSIS_PRE_CALI_GYRO_Z_OFFSET", new_gyro[2]))
    print(fmt_macro("BMI088_CHASSIS_PRE_CALI_ACC_X_OFFSET", new_acc[0]))
    print(fmt_macro("BMI088_CHASSIS_PRE_CALI_ACC_Y_OFFSET", new_acc[1]))
    print(fmt_macro("BMI088_CHASSIS_PRE_CALI_ACC_Z_OFFSET", new_acc[2]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
