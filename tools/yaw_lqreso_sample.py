#!/usr/bin/env python3
"""Sample yaw LQRESO identification variables via OpenOCD telnet.

This sampler reads global float symbols while the target is running. It can
optionally write an open-loop test_yaw_ref step profile for plant fitting.
"""

from __future__ import annotations

import argparse
import csv
import re
import socket
import struct
import subprocess
import time
from pathlib import Path


WATCH_SYMBOLS = (
    "yaw_fit_time_s",
    "test_yaw_ref",
    "yaw_angle_rad",
    "yaw_gyro_rad_s",
    "motor_torque_nm",
    "yaw_motor_pos_rad",
    "yaw_motor_total_rad",
    "g_torque_cmd",
    "g_lqr_e_theta",
    "g_lqr_omega_filt",
    "g_lqr_tau_raw",
    "g_yaw_eso_z1",
    "g_yaw_eso_z2",
    "g_yaw_eso_z3",
    "g_yaw_eso_tau_comp",
)

DEFAULT_PROFILE = "0:0,5:0.4,7.7:0.5,10.5:0.6,14.5:0.7,18.9:0.8,22.5:0.9,25.3:0"


def parse_args() -> argparse.Namespace:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Sample yaw fit telemetry through OpenOCD telnet")
    parser.add_argument("--elf", type=Path, default=repo / "build/Infantry_RMUC_2026.elf")
    parser.add_argument("--out", type=Path, default=repo / "build/yaw_lqreso_samples.csv")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0, help="OpenOCD telnet port; auto when 0")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=28.0)
    parser.add_argument(
        "--profile",
        default="",
        help=f"Optional test_yaw_ref schedule like '0:0,5:0.4'. Use 'default' for {DEFAULT_PROFILE}",
    )
    return parser.parse_args()


def run_cmd(args: list[str]) -> str:
    data = subprocess.check_output(args, stderr=subprocess.DEVNULL)
    return data.decode("utf-8", errors="ignore")


def resolve_nm() -> str:
    for path in (
        "arm-none-eabi-nm",
        "/opt/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-nm",
    ):
        try:
            subprocess.check_output([path, "--version"], stderr=subprocess.DEVNULL)
            return path
        except Exception:
            pass
    raise RuntimeError("arm-none-eabi-nm not found")


def find_symbol_addrs(elf: Path, names: tuple[str, ...]) -> dict[str, int]:
    nm = resolve_nm()
    out = run_cmd([nm, "-n", str(elf)])
    addrs: dict[str, int] = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in names:
            addrs[parts[2]] = int(parts[0], 16)
    missing = [name for name in names if name not in addrs]
    if missing:
        raise RuntimeError(f"symbols not found in ELF: {', '.join(missing)}")
    return addrs


def find_openocd_telnet_port() -> int:
    out = run_cmd(["lsof", "-nP", "-iTCP", "-sTCP:LISTEN"])
    ports: list[int] = []
    for line in out.splitlines():
        if "openocd" not in line:
            continue
        match = re.search(r":(\d+)\s+\(LISTEN\)", line)
        if match:
            ports.append(int(match.group(1)))
    if not ports:
        raise RuntimeError("no openocd telnet port found")
    for port in sorted(set(ports), reverse=True):
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=0.6)
            sock.settimeout(0.6)
            banner = sock.recv(1024).decode(errors="ignore")
            sock.close()
            if "Open On-Chip Debugger" in banner:
                return port
        except Exception:
            continue
    return max(ports)


class OpenOcdTelnet:
    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=2.0)
        self.sock.settimeout(0.25)
        self._read_until_prompt(1.0)

    def _read_until_prompt(self, timeout_s: float) -> str:
        end = time.monotonic() + timeout_s
        buf = ""
        while time.monotonic() < end:
            try:
                chunk = self.sock.recv(4096).decode(errors="ignore")
            except Exception:
                chunk = ""
            if chunk:
                buf += chunk
                if "> " in buf:
                    break
            else:
                time.sleep(0.005)
        return buf

    def cmd(self, text: str, timeout_s: float = 0.25) -> str:
        self.sock.sendall((text + "\r\n").encode())
        return self._read_until_prompt(timeout_s).replace("\x00", " ")

    def close(self) -> None:
        try:
            self.cmd("exit", 0.1)
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass


def parse_mdw_word(out: str) -> int:
    match = re.search(r"0x[0-9a-fA-F]+:\s*([0-9a-fA-F]{8})", out)
    if not match:
        raise RuntimeError(f"bad mdw response: {out.strip()}")
    return int(match.group(1), 16)


def u32_to_f32(word: int) -> float:
    return struct.unpack("<f", struct.pack("<I", word))[0]


def f32_to_u32(value: float) -> int:
    return struct.unpack("<I", struct.pack("<f", value))[0]


def parse_profile(text: str) -> list[tuple[float, float]]:
    if not text:
        return []
    if text == "default":
        text = DEFAULT_PROFILE
    steps: list[tuple[float, float]] = []
    for item in text.split(","):
        t_text, value_text = item.split(":", 1)
        steps.append((float(t_text), float(value_text)))
    steps.sort(key=lambda item: item[0])
    return steps


def main() -> int:
    args = parse_args()
    if not args.elf.exists():
        print(f"ELF not found: {args.elf}")
        return 2

    try:
        addrs = find_symbol_addrs(args.elf, WATCH_SYMBOLS)
        port = args.port or find_openocd_telnet_port()
        profile = parse_profile(args.profile)
    except Exception as exc:
        print(f"Setup failed: {exc}")
        return 2

    args.out.parent.mkdir(parents=True, exist_ok=True)
    period = 1.0 / max(1.0, args.hz)
    t0 = time.monotonic()
    next_profile_idx = 0

    tel = OpenOcdTelnet(args.host, port)
    try:
        with args.out.open("w", newline="", encoding="utf-8") as fp:
            writer = csv.writer(fp)
            writer.writerow(("Time",) + WATCH_SYMBOLS)
            while True:
                now = time.monotonic()
                elapsed = now - t0
                if elapsed >= args.duration:
                    break

                while next_profile_idx < len(profile) and elapsed >= profile[next_profile_idx][0]:
                    _, value = profile[next_profile_idx]
                    tel.cmd(f"mww 0x{addrs['test_yaw_ref']:08x} 0x{f32_to_u32(value):08x}")
                    next_profile_idx += 1

                values = []
                for symbol in WATCH_SYMBOLS:
                    word = parse_mdw_word(tel.cmd(f"mdw 0x{addrs[symbol]:08x} 1"))
                    values.append(u32_to_f32(word))
                writer.writerow([f"{elapsed:.6f}", *[f"{value:.9g}" for value in values]])
                time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        if profile:
            try:
                tel.cmd(f"mww 0x{addrs['test_yaw_ref']:08x} 0x{f32_to_u32(0.0):08x}")
            except Exception:
                pass
        tel.close()

    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
