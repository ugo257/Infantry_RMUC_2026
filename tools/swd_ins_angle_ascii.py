#!/usr/bin/env python3
"""
Realtime terminal ASCII curves for INS->output.INS_angle_deg[0..2] via SWD.

This script reads memory through an existing OpenOCD telnet endpoint.
It is designed for cases where you are already in a debug session.
"""

from __future__ import annotations

import argparse
import re
import socket
import struct
import subprocess
import sys
import time
from collections import deque
from pathlib import Path


CHARS = " .:-=+*#%@"


def run_cmd(cmd: list[str]) -> str:
    out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
    return out.decode("utf-8", errors="ignore")


def resolve_nm() -> str:
    candidates = [
        "arm-none-eabi-nm",
        "/opt/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-nm",
    ]
    for c in candidates:
        try:
            subprocess.check_output([c, "--version"], text=True, stderr=subprocess.DEVNULL)
            return c
        except Exception:
            continue
    raise RuntimeError("arm-none-eabi-nm not found")


def resolve_ins_symbol_addr(elf: Path) -> int:
    nm = resolve_nm()
    txt = run_cmd([nm, "-n", str(elf)])
    m = re.search(r"^([0-9a-fA-F]+)\s+[BD]\s+INS$", txt, flags=re.MULTILINE)
    if not m:
        raise RuntimeError("Cannot find INS symbol in ELF")
    return int(m.group(1), 16)


def discover_openocd_telnet_port() -> int:
    txt = run_cmd(["lsof", "-nP", "-iTCP", "-sTCP:LISTEN"])
    ports = []
    for line in txt.splitlines():
        if "openocd" not in line:
            continue
        m = re.search(r":(\d+)\s+\(LISTEN\)", line)
        if m:
            ports.append(int(m.group(1)))
    if not ports:
        raise RuntimeError("No openocd listening port found")

    # Probe each candidate; telnet endpoint prints a text banner + prompt.
    for p in sorted(set(ports), reverse=True):
        try:
            s = socket.create_connection(("127.0.0.1", p), timeout=0.6)
            s.settimeout(0.6)
            data = s.recv(1024).decode(errors="ignore")
            s.close()
            if "Open On-Chip Debugger" in data:
                return p
        except Exception:
            continue

    # Fallback to highest port (typical: gdb/tcl/telnet are contiguous).
    return max(ports)


class OpenOcdTelnet:
    def __init__(self, host: str, port: int):
        self.s = socket.create_connection((host, port), timeout=2.0)
        self.s.settimeout(0.25)
        self._read_until_prompt(1.0)

    def _read_until_prompt(self, timeout: float) -> str:
        end = time.monotonic() + timeout
        buf = ""
        while time.monotonic() < end:
            try:
                part = self.s.recv(4096).decode(errors="ignore")
            except Exception:
                part = ""
            if part:
                buf += part
                if "> " in buf:
                    break
            else:
                time.sleep(0.01)
        return buf

    def cmd(self, line: str, timeout: float = 0.4) -> str:
        self.s.sendall((line + "\r\n").encode())
        return self._read_until_prompt(timeout)

    def close(self) -> None:
        try:
            self.cmd("exit", 0.2)
        except Exception:
            pass
        try:
            self.s.close()
        except Exception:
            pass


def parse_mdw_words(out: str) -> list[int]:
    pairs = re.findall(r"0x[0-9a-fA-F]+:\s+([0-9a-fA-F]{8})", out)
    return [int(x, 16) for x in pairs]


def u32_to_f32(word: int) -> float:
    return struct.unpack("<f", struct.pack("<I", word))[0]


def spark(values: deque[float], width: int) -> str:
    if not values:
        return " " * width
    vals = list(values)
    if len(vals) > width:
        vals = vals[-width:]
    lo = min(vals)
    hi = max(vals)
    if abs(hi - lo) < 1e-9:
        body = "-" * len(vals)
    else:
        body_chars = []
        n = len(CHARS) - 1
        for v in vals:
            t = (v - lo) / (hi - lo)
            idx = int(t * n)
            if idx < 0:
                idx = 0
            if idx > n:
                idx = n
            body_chars.append(CHARS[idx])
        body = "".join(body_chars)
    if len(body) < width:
        body = (" " * (width - len(body))) + body
    return body


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="ASCII realtime INS angle curve via SWD")
    parser.add_argument("--elf", type=Path, default=repo_root / "build/Debug/Infantry_RMUC_2026.elf")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0, help="OpenOCD telnet port; auto-detect when 0")
    parser.add_argument("--hz", type=float, default=8.0)
    parser.add_argument("--width", type=int, default=100)
    args = parser.parse_args()

    if not args.elf.exists():
        print(f"ELF not found: {args.elf}")
        return 2

    try:
        ins_symbol_addr = resolve_ins_symbol_addr(args.elf)
    except Exception as exc:
        print(f"Resolve symbol failed: {exc}")
        return 2

    port = args.port
    if port == 0:
        try:
            port = discover_openocd_telnet_port()
        except Exception as exc:
            print(f"Auto-detect OpenOCD port failed: {exc}")
            return 2

    tel = OpenOcdTelnet(args.host, port)
    hist_len = max(30, args.width)
    roll_hist: deque[float] = deque(maxlen=hist_len)
    pitch_hist: deque[float] = deque(maxlen=hist_len)
    yaw_hist: deque[float] = deque(maxlen=hist_len)
    period = 1.0 / max(0.5, args.hz)

    try:
        print(f"Connected OpenOCD telnet: {args.host}:{port}")
        print(f"INS symbol addr: 0x{ins_symbol_addr:08X}")
        print("Press Ctrl+C to stop.")

        while True:
            out_ptr = tel.cmd(f"mdw 0x{ins_symbol_addr:08x} 1")
            ptr_words = parse_mdw_words(out_ptr)
            if not ptr_words or ptr_words[0] == 0:
                print("INS pointer is null or unreadable, waiting...")
                time.sleep(0.3)
                continue

            ins_ptr = ptr_words[0]
            base = ins_ptr + 0x44  # output.INS_angle_deg[0]

            out_angles = tel.cmd(f"mdw 0x{base:08x} 3")
            words = parse_mdw_words(out_angles)
            if len(words) < 3:
                print("Angle read failed, retrying...")
                time.sleep(0.2)
                continue

            roll = u32_to_f32(words[0])
            pitch = u32_to_f32(words[1])
            yaw = u32_to_f32(words[2])

            roll_hist.append(roll)
            pitch_hist.append(pitch)
            yaw_hist.append(yaw)

            sys.stdout.write("\x1b[2J\x1b[H")
            sys.stdout.write(f"OpenOCD {args.host}:{port} | INS@0x{ins_ptr:08X} | {args.hz:.1f} Hz\n")
            sys.stdout.write(f"ROLL  {roll:9.3f} |{spark(roll_hist, args.width)}|\n")
            sys.stdout.write(f"PITCH {pitch:9.3f} |{spark(pitch_hist, args.width)}|\n")
            sys.stdout.write(f"YAW   {yaw:9.3f} |{spark(yaw_hist, args.width)}|\n")
            sys.stdout.flush()

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        tel.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
