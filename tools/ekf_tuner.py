#!/usr/bin/env python3
"""
EKF Tuner - 通过固定帧协议与 STM32 通信

MCU -> PC 上传帧:
  C结构体 INS_EKF_Telemetry_t，长度60字节，帧头 [0xAA][0x01]

PC -> MCU 核心参数帧:
  [0xAA][0x02][q1 float][q2 float][r float][lambda float][sum_chk]

PC -> MCU 漂移参数帧:
  [0xAA][0x03][9个float][chi_enable uint8][sum_chk]

sum_chk = 从字节2到校验前最后1字节的逐字节累加低8位。
"""

from __future__ import annotations

import argparse
import select
import struct
import sys
import threading
import time
from typing import Dict, Optional

import serial
import serial.tools.list_ports

FRAME_HEAD = 0xAA
FRAME_TYPE_UPLOAD = 0x01
FRAME_TYPE_MODIFY = 0x02
FRAME_TYPE_MODIFY_DRIFT = 0x03

UPLOAD_STRUCT = struct.Struct("<BBxx13fI")
UPLOAD_LEN = UPLOAD_STRUCT.size

CORE_PARAM_NAMES = ("q1", "q2", "r", "lambda")
CORE_DEFAULTS = {
    "q1": 4.5,
    "q2": 0.00035,
    "r": 2500000.0,
    "lambda": 0.9999,
}

DRIFT_PARAM_NAMES = (
    "chi",
    "static_gyro",
    "static_accel",
    "static_delta",
    "static_lpf",
    "static_hold",
    "static_release",
    "bias_lpf",
    "bias_limit",
    "chi_enable",
)
DRIFT_DEFAULTS = {
    "chi": 0.0300,
    "static_gyro": 1.60,
    "static_accel": 0.30,
    "static_delta": 0.45,
    "static_lpf": 8.0,
    "static_hold": 0.35,
    "static_release": 0.0,
    "bias_lpf": 0.15,
    "bias_limit": 1.20,
    "chi_enable": 1.0,
}

ALIASES = {
    "chi_threshold": "chi",
    "chi_thre": "chi",
    "static_accel_tol": "static_accel",
    "static_accel_delta": "static_delta",
    "static_delta_tol": "static_delta",
    "gyro_z_bias_lpf": "bias_lpf",
    "gyro_z_bias_limit": "bias_limit",
}


def frame_checksum(frame_without_checksum: bytes) -> int:
    return sum(frame_without_checksum[2:]) & 0xFF


def build_core_frame(params: Dict[str, float]) -> bytes:
    frame = bytearray([FRAME_HEAD, FRAME_TYPE_MODIFY])
    for name in CORE_PARAM_NAMES:
        frame.extend(struct.pack("<f", float(params[name])))
    frame.append(frame_checksum(frame))
    return bytes(frame)


def build_drift_frame(params: Dict[str, float]) -> bytes:
    frame = bytearray([FRAME_HEAD, FRAME_TYPE_MODIFY_DRIFT])
    for name in DRIFT_PARAM_NAMES[:-1]:
        frame.extend(struct.pack("<f", float(params[name])))
    frame.append(1 if float(params["chi_enable"]) != 0.0 else 0)
    frame.append(frame_checksum(frame))
    return bytes(frame)


class FrameParser:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.latest: Dict[str, float] = {}
        self.frame_count = 0
        self.lock = threading.Lock()

    def feed(self, data: bytes) -> None:
        self.buf.extend(data)
        while self._try_parse():
            pass

    def _try_parse(self) -> bool:
        head_idx = self.buf.find(bytes([FRAME_HEAD]))
        if head_idx < 0:
            self.buf.clear()
            return False
        if head_idx > 0:
            del self.buf[:head_idx]

        if len(self.buf) < 2:
            return False
        if self.buf[1] != FRAME_TYPE_UPLOAD:
            del self.buf[:1]
            return True
        if len(self.buf) < UPLOAD_LEN:
            return False

        frame = bytes(self.buf[:UPLOAD_LEN])
        try:
            unpacked = UPLOAD_STRUCT.unpack(frame)
        except struct.error:
            del self.buf[:1]
            return True

        if unpacked[0] != FRAME_HEAD or unpacked[1] != FRAME_TYPE_UPLOAD:
            del self.buf[:1]
            return True

        floats = unpacked[2:15]
        sample_count = unpacked[15]
        keys = (
            "gyro_x",
            "gyro_y",
            "gyro_z",
            "yaw_deg",
            "yaw_total_deg",
            "gyro_z_bias_dps",
            "q1",
            "q2",
            "r",
            "lambda",
            "chi_square",
            "chi_threshold",
            "dt",
        )

        with self.lock:
            for key, value in zip(keys, floats):
                self.latest[key] = float(value)
            self.latest["sample_count"] = float(sample_count)
            self.frame_count += 1

        del self.buf[:UPLOAD_LEN]
        return True

    def snapshot(self) -> Dict[str, float]:
        with self.lock:
            return dict(self.latest)


class Receiver:
    def __init__(self, ser: serial.Serial, parser: FrameParser) -> None:
        self.ser = ser
        self.parser = parser
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.ser.read(512)
                if data:
                    self.parser.feed(data)
            except Exception:
                break


def find_uart_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("ERROR: 未找到串口", file=sys.stderr)
        sys.exit(1)

    for p in ports:
        if "usbmodem" in p.device.lower():
            return p.device

    preferred = ("usbmodem", "stm", "usb serial", "cp210", "ch340", "ftdi", "stlink")
    matched = []
    for p in ports:
        desc = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
        if any(key in desc for key in preferred):
            matched.append(p)
    if len(matched) == 1:
        return matched[0].device
    if len(ports) == 1:
        return ports[0].device

    print("可用串口:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} ({p.description})")
    return ports[int(input("选择序号: ").strip())].device


def display(data: Dict[str, float], core: Dict[str, float], drift: Dict[str, float], frame_count: int) -> None:
    print("\033[2J\033[H", end="")
    print("=" * 64)
    print("  EKF Tuner - fixed frame / USB CDC")
    print("=" * 64)

    print("\n--- IMU / Yaw ---")
    print(
        "  Gyro(rad/s): "
        f"[{data.get('gyro_x', 0):+9.4f}, {data.get('gyro_y', 0):+9.4f}, {data.get('gyro_z', 0):+9.4f}]"
    )
    print(
        f"  Yaw={data.get('yaw_deg', 0):+9.3f} deg  "
        f"YawTotal={data.get('yaw_total_deg', 0):+9.3f} deg  "
        f"ZBias={data.get('gyro_z_bias_dps', 0):+8.4f} dps"
    )

    print("\n--- EKF Core Params ---")
    print(f"  q1={core['q1']:.6g}  q2={core['q2']:.6g}  r={core['r']:.6g}  lambda={core['lambda']:.6g}")
    print(f"  chi_square={data.get('chi_square', 0):.6g}  chi_threshold={drift['chi']:.6g}")

    print("\n--- Drift Params (local / last sent) ---")
    print(
        f"  static_gyro={drift['static_gyro']:.4g} dps  "
        f"static_accel={drift['static_accel']:.4g}  "
        f"static_delta={drift['static_delta']:.4g}"
    )
    print(
        f"  static_lpf={drift['static_lpf']:.4g} Hz  "
        f"hold={drift['static_hold']:.4g}s  "
        f"release={drift['static_release']:.4g}s"
    )
    print(
        f"  bias_lpf={drift['bias_lpf']:.4g} Hz  "
        f"bias_limit={drift['bias_limit']:.4g} dps  "
        f"chi_enable={int(drift['chi_enable'] != 0)}"
    )

    print(f"\nframes={frame_count}  sample={int(data.get('sample_count', 0))}  dt={data.get('dt', 0):.6g}s")
    print("\n命令:")
    print("  set q1|q2|r|lambda <value>")
    print("  set chi|static_gyro|static_accel|static_delta|static_lpf <value>")
    print("  set static_hold|static_release|bias_lpf|bias_limit|chi_enable <value>")
    print("  send drift     重新发送当前漂移参数组")
    print("  q              退出")


def update_core_from_telemetry(core: Dict[str, float], data: Dict[str, float]) -> None:
    for name in CORE_PARAM_NAMES:
        if name in data:
            core[name] = data[name]


def update_drift_from_telemetry(drift: Dict[str, float], data: Dict[str, float]) -> None:
    if "chi_threshold" in data:
        drift["chi"] = data["chi_threshold"]


def main() -> int:
    ap = argparse.ArgumentParser(description="EKF Tuner fixed-frame USB/UART client")
    ap.add_argument("--port", default=None, help="串口设备，默认自动检测")
    ap.add_argument("--baud", type=int, default=115200, help="波特率；USB CDC下通常被忽略")
    ap.add_argument("--refresh", type=float, default=0.2, help="刷新间隔，单位秒")
    ap.add_argument("--mode", default="manual", help=argparse.SUPPRESS)
    args = ap.parse_args()

    port = args.port or find_uart_port()
    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except Exception as exc:
        print(f"无法打开串口 {port}: {exc}", file=sys.stderr)
        return 1

    print(f"已连接 {port} @ {args.baud}")

    parser = FrameParser()
    rx = Receiver(ser, parser)
    core = dict(CORE_DEFAULTS)
    drift = dict(DRIFT_DEFAULTS)
    rx.start()

    try:
        while True:
            data = parser.snapshot()
            update_core_from_telemetry(core, data)
            update_drift_from_telemetry(drift, data)
            display(data, core, drift, parser.frame_count)
            print("> ", end="", flush=True)

            ready, _, _ = select.select([sys.stdin], [], [], args.refresh)
            if not ready:
                continue

            line = sys.stdin.readline().strip()
            if not line:
                continue
            if line.lower() == "q":
                break
            if line.lower() == "send drift":
                ser.write(build_drift_frame(drift))
                time.sleep(0.2)
                continue

            parts = line.split()
            if len(parts) != 3 or parts[0].lower() != "set":
                print("\n格式: set <param> <value>")
                time.sleep(0.5)
                continue

            name = ALIASES.get(parts[1].lower(), parts[1].lower())
            try:
                value = float(parts[2])
            except ValueError:
                print(f"\n无效数值: {parts[2]}")
                time.sleep(0.5)
                continue

            if name in CORE_PARAM_NAMES:
                core[name] = value
                ser.write(build_core_frame(core))
                time.sleep(0.2)
            elif name in DRIFT_PARAM_NAMES:
                drift[name] = value
                ser.write(build_drift_frame(drift))
                time.sleep(0.2)
            else:
                print("\n未知参数")
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop()
        ser.close()
        print("\n已断开")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
