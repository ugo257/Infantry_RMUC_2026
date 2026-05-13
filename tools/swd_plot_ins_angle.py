#!/usr/bin/env python3
"""
Plot INS->output.INS_angle_deg[0..2] in real time via SWD.

This script connects to an existing GDB server (OpenOCD/J-Link) on TCP,
uses arm-none-eabi-gdb MI mode to read target variables, and draws curves
with matplotlib.

Default setup in this repository:
  - GDB server: OpenOCD on localhost:3333
  - ELF: build/Debug/Infantry_RMUC_2026.elf
  - GDB: tools/debug/arm-gdb-wrapper.sh
"""

from __future__ import annotations

import argparse
import queue
import re
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class GdbMiError(RuntimeError):
    """Raised when a GDB MI command fails."""


def _mi_escape(expr: str) -> str:
    return expr.replace("\\", "\\\\").replace('"', '\\"')


def _mi_unescape(value: str) -> str:
    # GDB MI string fields use C-style escapes.
    return bytes(value, "utf-8").decode("unicode_escape")


def _extract_mi_value(result_line: str) -> str:
    match = re.search(r'value="(.*)"$', result_line)
    if not match:
        raise GdbMiError(f"Missing value field in MI response: {result_line}")
    return _mi_unescape(match.group(1))


def _parse_number(text: str) -> float:
    s = text.strip()
    if s.lower().startswith("0x"):
        return float(int(s, 16))
    return float(s)


class GdbMiSession:
    def __init__(self, gdb_path: Path, elf_path: Path):
        self._token = 1
        self._lines: queue.Queue[str] = queue.Queue()
        self._proc = subprocess.Popen(
            [str(gdb_path), "--interpreter=mi2", "--nx", "--quiet", str(elf_path)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        if self._proc.stdin is None or self._proc.stdout is None:
            raise GdbMiError("Failed to open GDB stdin/stdout pipes")

        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()
        self._wait_for_prompt(timeout=3.0)

    def _reader(self) -> None:
        assert self._proc.stdout is not None
        for line in self._proc.stdout:
            self._lines.put(line.rstrip("\n"))

    def _wait_for_prompt(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                line = self._lines.get(timeout=max(0.01, remaining))
            except queue.Empty:
                continue
            if line.strip() == "(gdb)":
                return
        raise GdbMiError("Timed out waiting for GDB prompt")

    def cmd(self, mi_command: str, timeout: float = 2.0, allow_error: bool = False) -> str:
        token = self._token
        self._token += 1
        line_prefix = f"{token}^"

        assert self._proc.stdin is not None
        self._proc.stdin.write(f"{token}{mi_command}\n")
        self._proc.stdin.flush()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                line = self._lines.get(timeout=max(0.01, remaining))
            except queue.Empty:
                continue

            if line.startswith(line_prefix):
                if ("^error" in line) and (not allow_error):
                    raise GdbMiError(f"MI command failed: {mi_command} -> {line}")
                return line

        raise GdbMiError(f"Timed out waiting MI result: {mi_command}")

    def connect(self, remote: str) -> None:
        self.cmd("-gdb-set pagination off")
        self.cmd("-gdb-set confirm off")
        self.cmd("-gdb-set target-async on")
        self.cmd(f"-target-select extended-remote {remote}", timeout=5.0)

    def evaluate(self, expr: str, timeout: float = 2.0) -> str:
        result = self.cmd(f'-data-evaluate-expression "{_mi_escape(expr)}"', timeout=timeout)
        return _extract_mi_value(result)

    def interrupt(self) -> None:
        self.cmd("-exec-interrupt", timeout=3.0, allow_error=True)

    def resume(self) -> None:
        self.cmd("-exec-continue", timeout=2.0, allow_error=True)

    def close(self) -> None:
        if self._proc.poll() is None:
            try:
                self.interrupt()
            except Exception:
                pass
            try:
                self.cmd("-target-detach", allow_error=True)
            except Exception:
                pass
            try:
                self.cmd("-gdb-exit", allow_error=True)
            except Exception:
                pass
            try:
                self._proc.terminate()
            except Exception:
                pass


class AngleSampler:
    def __init__(self, session: GdbMiSession, hz: float, window_s: float):
        self.session = session
        self.period = 1.0 / hz
        self.window_s = window_s
        self.times: deque[float] = deque()
        self.roll: deque[float] = deque()
        self.pitch: deque[float] = deque()
        self.yaw: deque[float] = deque()
        self.t0 = time.monotonic()
        self.last_sample_t = 0.0
        self.ins_ready = False

    def _append(self, now_rel: float, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        self.times.append(now_rel)
        self.roll.append(roll_deg)
        self.pitch.append(pitch_deg)
        self.yaw.append(yaw_deg)

        while self.times and (now_rel - self.times[0] > self.window_s):
            self.times.popleft()
            self.roll.popleft()
            self.pitch.popleft()
            self.yaw.popleft()

    def sample_once(self) -> bool:
        now = time.monotonic()
        if now - self.last_sample_t < self.period:
            return False
        self.last_sample_t = now

        self.session.interrupt()

        try:
            ins_ptr = int(_parse_number(self.session.evaluate("(unsigned int)INS")))
        except Exception:
            ins_ptr = 0

        if ins_ptr != 0:
            try:
                roll_deg = _parse_number(self.session.evaluate("INS->output.INS_angle_deg[0]"))
                pitch_deg = _parse_number(self.session.evaluate("INS->output.INS_angle_deg[1]"))
                yaw_deg = _parse_number(self.session.evaluate("INS->output.INS_angle_deg[2]"))
                self._append(now - self.t0, roll_deg, pitch_deg, yaw_deg)
                self.ins_ready = True
            except Exception:
                pass

        self.session.resume()
        return True


def build_arg_parser(repo_root: Path) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot INS->output.INS_angle_deg[0..2] via SWD (GDB remote)."
    )
    parser.add_argument(
        "--elf",
        type=Path,
        default=repo_root / "build/Debug/Infantry_RMUC_2026.elf",
        help="Path to ELF with symbols.",
    )
    parser.add_argument(
        "--gdb",
        type=Path,
        default=repo_root / "tools/debug/arm-gdb-wrapper.sh",
        help="Path to arm-none-eabi-gdb executable or wrapper.",
    )
    parser.add_argument(
        "--remote",
        default="localhost:3333",
        help="GDB server address, e.g. localhost:3333.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=20.0,
        help="Sampling rate in Hz.",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=15.0,
        help="Visible time window in seconds.",
    )
    return parser


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = build_arg_parser(repo_root)
    args = parser.parse_args()

    if not args.elf.exists():
        print(f"ELF not found: {args.elf}")
        print("Build the firmware first or pass --elf <path>.")
        return 2
    if not args.gdb.exists():
        print(f"GDB wrapper not found: {args.gdb}")
        print("Pass --gdb with arm-none-eabi-gdb path.")
        return 2

    session = GdbMiSession(args.gdb, args.elf)
    try:
        session.connect(args.remote)
        session.resume()

        sampler = AngleSampler(session, hz=args.hz, window_s=args.window)

        fig, ax = plt.subplots(figsize=(11, 6))
        line_roll, = ax.plot([], [], label="INS_angle_deg[0] (roll)")
        line_pitch, = ax.plot([], [], label="INS_angle_deg[1] (pitch)")
        line_yaw, = ax.plot([], [], label="INS_angle_deg[2] (yaw)")

        ax.set_title("INS angles via SWD/GDB")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (deg)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

        status_text = ax.text(
            0.01,
            0.99,
            "waiting INS init...",
            transform=ax.transAxes,
            ha="left",
            va="top",
        )

        def on_frame(_frame_idx: int):
            sampler.sample_once()

            if sampler.times:
                x = list(sampler.times)
                y_roll = list(sampler.roll)
                y_pitch = list(sampler.pitch)
                y_yaw = list(sampler.yaw)

                line_roll.set_data(x, y_roll)
                line_pitch.set_data(x, y_pitch)
                line_yaw.set_data(x, y_yaw)

                x0 = max(0.0, x[-1] - sampler.window_s)
                x1 = max(sampler.window_s, x[-1])
                ax.set_xlim(x0, x1)

                y_all = y_roll + y_pitch + y_yaw
                ymin = min(y_all)
                ymax = max(y_all)
                margin = max(1.0, (ymax - ymin) * 0.15)
                ax.set_ylim(ymin - margin, ymax + margin)

                status_text.set_text(
                    f"latest: roll={y_roll[-1]:.2f}, pitch={y_pitch[-1]:.2f}, yaw={y_yaw[-1]:.2f} deg"
                )
            else:
                status_text.set_text("waiting INS init...")

            return line_roll, line_pitch, line_yaw, status_text

        FuncAnimation(
            fig,
            on_frame,
            interval=max(20, int(1000.0 / args.hz)),
            blit=False,
            cache_frame_data=False,
        )

        plt.tight_layout()
        plt.show()
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"Error: {exc}")
        return 1
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
