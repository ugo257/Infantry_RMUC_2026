#!/usr/bin/env python3
"""
Print INS live-watch variables in terminal via SWD (OpenOCD telnet).

Current variables printed:
  - INS->output.INS_angle_deg[0]
  - INS->output.INS_angle_deg[1]
  - INS->output.INS_angle_deg[2]
"""

from __future__ import annotations

import argparse
import re
import socket
import struct
import subprocess
import time
from pathlib import Path


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


def find_ins_symbol_addr(elf: Path) -> int:
    nm = resolve_nm()
    out = run_cmd([nm, "-n", str(elf)])
    match = re.search(r"^([0-9a-fA-F]+)\s+[BD]\s+INS$", out, flags=re.MULTILINE)
    if not match:
        raise RuntimeError("INS symbol not found in ELF")
    return int(match.group(1), 16)


def find_openocd_telnet_port() -> int:
    out = run_cmd(["lsof", "-nP", "-iTCP", "-sTCP:LISTEN"])
    ports: list[int] = []
    for line in out.splitlines():
        if "openocd" not in line:
            continue
        m = re.search(r":(\d+)\s+\(LISTEN\)", line)
        if m:
            ports.append(int(m.group(1)))

    if not ports:
        raise RuntimeError("No openocd port found")

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
                time.sleep(0.01)
        return buf

    def cmd(self, text: str, timeout_s: float = 0.4) -> str:
        self.sock.sendall((text + "\r\n").encode())
        out = self._read_until_prompt(timeout_s)
        return out.replace("\x00", " ")

    def close(self) -> None:
        try:
            self.cmd("exit", 0.2)
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass


def parse_mdw_word(out: str) -> int:
    # Typical response: "0x20008620: 20009e08"
    m = re.search(r"0x[0-9a-fA-F]+:\s*([0-9a-fA-F]{8})", out)
    if not m:
        raise RuntimeError(f"Bad mdw response: {out.strip()}")
    return int(m.group(1), 16)


def u32_to_f32(word: int) -> float:
    return struct.unpack("<f", struct.pack("<I", word))[0]


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser(description="Print INS live-watch variables via SWD")
    parser.add_argument("--elf", type=Path, default=repo_root / "build/Debug/Infantry_RMUC_2026.elf")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0, help="OpenOCD telnet port; auto when 0")
    parser.add_argument("--hz", type=float, default=5.0, help="Print rate")
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run forever")
    args = parser.parse_args()

    if not args.elf.exists():
        print(f"ELF not found: {args.elf}")
        return 2

    try:
        ins_sym_addr = find_ins_symbol_addr(args.elf)
    except Exception as exc:
        print(f"Resolve symbol failed: {exc}")
        return 2

    port = args.port
    if port == 0:
        try:
            port = find_openocd_telnet_port()
        except Exception as exc:
            print(f"Find openocd port failed: {exc}")
            return 2

    tel = OpenOcdTelnet(args.host, port)
    period = 1.0 / max(0.5, args.hz)
    t0 = time.monotonic()

    print(f"OpenOCD: {args.host}:{port}")
    print(f"INS symbol @ 0x{ins_sym_addr:08X}")
    print("time_s, roll_deg, pitch_deg, yaw_deg")

    try:
        while True:
            now = time.monotonic()
            if args.duration > 0.0 and (now - t0) >= args.duration:
                break

            try:
                ins_ptr = parse_mdw_word(tel.cmd(f"mdw 0x{ins_sym_addr:08x} 1"))
                if ins_ptr == 0:
                    print(f"{now - t0:8.3f}, INS=NULL")
                    time.sleep(period)
                    continue

                # INS_Instance layout: output.INS_angle_deg offset = 0x44
                base = ins_ptr + 0x44
                roll = u32_to_f32(parse_mdw_word(tel.cmd(f"mdw 0x{base + 0:08x} 1")))
                pitch = u32_to_f32(parse_mdw_word(tel.cmd(f"mdw 0x{base + 4:08x} 1")))
                yaw = u32_to_f32(parse_mdw_word(tel.cmd(f"mdw 0x{base + 8:08x} 1")))

                print(f"{now - t0:8.3f}, {roll:8.3f}, {pitch:8.3f}, {yaw:8.3f}")
            except Exception as exc:
                print(f"{now - t0:8.3f}, read_error: {exc}")

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        tel.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
