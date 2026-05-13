#!/usr/bin/env python3
"""
Plot Cortex Live Watch variables as curves (time on x-axis) via SWD/OpenOCD.

Current variables:
  INS->output.INS_angle_deg[0]
  INS->output.INS_angle_deg[1]
  INS->output.INS_angle_deg[2]
"""

from __future__ import annotations

import argparse
import collections
import re
import socket
import struct
import subprocess
import time
from pathlib import Path
from typing import Optional, Tuple

import matplotlib.pyplot as plt


def run_text(cmd: list[str]) -> str:
    data = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
    return data.decode("utf-8", errors="ignore")


def find_ins_symbol_addr(elf: Path) -> int:
    nm_candidates = [
        "arm-none-eabi-nm",
        "/opt/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-nm",
    ]
    nm = None
    for candidate in nm_candidates:
        try:
            subprocess.check_output([candidate, "--version"], stderr=subprocess.DEVNULL)
            nm = candidate
            break
        except Exception:
            continue
    if nm is None:
        raise RuntimeError("arm-none-eabi-nm not found")

    out = run_text([nm, "-n", str(elf)])
    m = re.search(r"^([0-9a-fA-F]+)\s+[BD]\s+INS$", out, flags=re.MULTILINE)
    if not m:
        raise RuntimeError("INS symbol not found in ELF")
    return int(m.group(1), 16)


def find_angle_offset(elf: Path) -> int:
    gdb_candidates = [
        "arm-none-eabi-gdb",
        "/opt/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gdb",
    ]
    gdb = None
    for candidate in gdb_candidates:
        try:
            subprocess.check_output([candidate, "--version"], stderr=subprocess.DEVNULL)
            gdb = candidate
            break
        except Exception:
            continue
    if gdb is None:
        raise RuntimeError("arm-none-eabi-gdb not found")

    out = run_text(
        [
            gdb,
            "-q",
            str(elf),
            "-batch",
            "-ex",
            "set language c",
            "-ex",
            "p/x (unsigned int)&((INS_Instance*)0)->output.INS_angle_deg[0]",
        ]
    )
    m = re.search(r"=\s*0x([0-9a-fA-F]+)", out)
    if not m:
        raise RuntimeError("Failed to compute INS_angle_deg offset")
    return int(m.group(1), 16)


def find_openocd_telnet_port() -> int:
    out = run_text(["lsof", "-nP", "-iTCP", "-sTCP:LISTEN"])
    ports = []
    for line in out.splitlines():
        if "openocd" not in line:
            continue
        m = re.search(r":(\d+)\s+\(LISTEN\)", line)
        if m:
            ports.append(int(m.group(1)))
    if not ports:
        raise RuntimeError("No OpenOCD listening ports found")

    for p in sorted(set(ports), reverse=True):
        try:
            s = socket.create_connection(("127.0.0.1", p), timeout=0.6)
            s.settimeout(0.6)
            banner = s.recv(1024).decode(errors="ignore")
            s.close()
            if "Open On-Chip Debugger" in banner:
                return p
        except Exception:
            continue

    # Fallback if banner probe fails.
    return max(ports)


class OpenOcdTelnet:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.connect()

    def connect(self) -> None:
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=2.0)
        self.sock.settimeout(0.25)
        self._read_until_prompt(1.0)

    def _read_until_prompt(self, timeout_s: float) -> str:
        assert self.sock is not None
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
                time.sleep(0.01)
        return buf.replace("\x00", " ")

    def cmd(self, text: str, timeout_s: float = 0.5) -> str:
        assert self.sock is not None
        self.sock.sendall((text + "\r\n").encode())
        return self._read_until_prompt(timeout_s)

    def read_word(self, addr: int) -> int:
        out = self.cmd(f"mdw 0x{addr:08x} 1")
        m = re.search(r"0x[0-9a-fA-F]+:\s*([0-9a-fA-F]{8})", out)
        if not m:
            raise RuntimeError(f"mdw parse failed: {out.strip()}")
        return int(m.group(1), 16)

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


def u32_to_f32(u: int) -> float:
    return struct.unpack("<f", struct.pack("<I", u))[0]


def read_angles_deg(link: OpenOcdTelnet, ins_sym_addr: int, angle_off: int) -> Tuple[float, float, float]:
    ins_ptr = link.read_word(ins_sym_addr)
    if ins_ptr == 0:
        raise RuntimeError("INS pointer is NULL")

    base = ins_ptr + angle_off
    roll = u32_to_f32(link.read_word(base + 0))
    pitch = u32_to_f32(link.read_word(base + 4))
    yaw = u32_to_f32(link.read_word(base + 8))
    return roll, pitch, yaw


def run_live_plot(args: argparse.Namespace, ins_sym_addr: int, angle_off: int, port: int) -> int:
    link = OpenOcdTelnet(args.host, port)
    period = 1.0 / max(0.5, args.hz)

    maxlen = max(200, int(args.window * args.hz * 1.5))
    ts = collections.deque(maxlen=maxlen)
    rolls = collections.deque(maxlen=maxlen)
    pitchs = collections.deque(maxlen=maxlen)
    yaws = collections.deque(maxlen=maxlen)

    fig, ax = plt.subplots(figsize=(11, 6))
    line_roll, = ax.plot([], [], label="INS_angle_deg[0] (roll)", linewidth=1.5)
    line_pitch, = ax.plot([], [], label="INS_angle_deg[1] (pitch)", linewidth=1.5)
    line_yaw, = ax.plot([], [], label="INS_angle_deg[2] (yaw)", linewidth=1.5)
    value_text = ax.text(
        0.01,
        0.99,
        "ROLL=0.0  PITCH=0.0  YAW=0.0  Hz=0.0",
        transform=ax.transAxes,
        ha="left",
        va="top",
        bbox={"facecolor": "white", "alpha": 0.65, "edgecolor": "none"},
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Cortex Live Watch Variables via SWD (Live)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    start = time.monotonic()
    next_tick = start

    try:
        while plt.fignum_exists(fig.number):
            now = time.monotonic()
            if now < next_tick:
                plt.pause(0.001)
                continue
            next_tick += period

            try:
                r, p, y = read_angles_deg(link, ins_sym_addr, angle_off)
            except Exception:
                # Recover by reconnecting when telnet stream gets desynced.
                try:
                    link.connect()
                    r, p, y = read_angles_deg(link, ins_sym_addr, angle_off)
                except Exception:
                    plt.pause(0.001)
                    continue

            t = now - start
            ts.append(t)
            rolls.append(r)
            pitchs.append(p)
            yaws.append(y)

            x = list(ts)
            y0 = list(rolls)
            y1 = list(pitchs)
            y2 = list(yaws)
            if not x:
                plt.pause(0.001)
                continue

            line_roll.set_data(x, y0)
            line_pitch.set_data(x, y1)
            line_yaw.set_data(x, y2)

            xmin = max(0.0, x[-1] - args.window)
            xmax = max(args.window, x[-1])
            ax.set_xlim(xmin, xmax)

            y_all = y0 + y1 + y2
            ymin = min(y_all)
            ymax = max(y_all)
            margin = max(0.5, (ymax - ymin) * 0.15)
            ax.set_ylim(ymin - margin, ymax + margin)

            effective_hz = 0.0
            if len(x) >= 2 and (x[-1] - x[0]) > 1e-6:
                effective_hz = (len(x) - 1) / (x[-1] - x[0])
            value_text.set_text(
                f"ROLL={y0[-1]:.1f}  PITCH={y1[-1]:.1f}  YAW={y2[-1]:.1f}  Hz={effective_hz:.1f}"
            )

            fig.canvas.draw_idle()
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()

    # Save the last frame so the live result is persisted.
    if len(ts) > 0:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.out, dpi=150)
        print(
            f"Saved {args.out} | samples={len(ts)} | "
            f"latest roll={rolls[-1]:.3f}, pitch={pitchs[-1]:.3f}, yaw={yaws[-1]:.3f}"
        )
    else:
        print("No samples captured in live mode.")

    return 0


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Plot INS livewatch variables (time axis)")
    parser.add_argument("--elf", type=Path, default=repo_root / "build/Debug/Infantry_RMUC_2026.elf")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0, help="OpenOCD telnet port; auto when 0")
    parser.add_argument("--hz", type=float, default=1000.0)
    parser.add_argument("--live", action="store_true", help="Realtime refresh plot window")
    parser.add_argument("--window", type=float, default=20.0, help="Live mode time window in seconds")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--out", type=Path, default=repo_root / "tools/ins_livewatch_plot.png")
    parser.add_argument("--show", action="store_true", help="Show interactive plot window")
    args = parser.parse_args()

    if not args.elf.exists():
        print(f"ELF not found: {args.elf}")
        return 2

    ins_sym_addr = find_ins_symbol_addr(args.elf)
    angle_off = find_angle_offset(args.elf)
    port = args.port if args.port != 0 else find_openocd_telnet_port()

    if args.live:
        return run_live_plot(args, ins_sym_addr, angle_off, port)

    link = OpenOcdTelnet(args.host, port)
    period = 1.0 / max(0.5, args.hz)

    ts = []
    rolls = []
    pitchs = []
    yaws = []

    start = time.monotonic()
    next_tick = start

    try:
        while True:
            now = time.monotonic()
            if now - start >= args.duration:
                break

            if now < next_tick:
                time.sleep(min(0.002, next_tick - now))
                continue
            next_tick += period

            try:
                r, p, y = read_angles_deg(link, ins_sym_addr, angle_off)
            except Exception:
                # Recover by reconnecting when telnet stream gets desynced.
                try:
                    link.connect()
                    r, p, y = read_angles_deg(link, ins_sym_addr, angle_off)
                except Exception:
                    continue

            ts.append(now - start)
            rolls.append(r)
            pitchs.append(p)
            yaws.append(y)
    finally:
        link.close()

    if not ts:
        print("No samples captured. Keep target running and retry.")
        return 1

    plt.figure(figsize=(11, 6))
    plt.plot(ts, rolls, label="INS_angle_deg[0] (roll)", linewidth=1.5)
    plt.plot(ts, pitchs, label="INS_angle_deg[1] (pitch)", linewidth=1.5)
    plt.plot(ts, yaws, label="INS_angle_deg[2] (yaw)", linewidth=1.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Cortex Live Watch Variables via SWD")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(args.out, dpi=150)
    print(
        f"Saved {args.out} | samples={len(ts)} | "
        f"latest roll={rolls[-1]:.3f}, pitch={pitchs[-1]:.3f}, yaw={yaws[-1]:.3f}"
    )

    if args.show:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
