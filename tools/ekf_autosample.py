#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import tempfile
import textwrap
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser(description="Auto-sample EKF telemetry over J-Link GDB")
    p.add_argument("--elf", default="build/Infantry_RMUC_2026.elf", help="ELF path")
    p.add_argument("--out", default="build/ekf_samples.csv", help="CSV output path")
    p.add_argument("--duration", type=float, default=12.0, help="Sampling duration in seconds")
    p.add_argument("--interval", type=float, default=0.02, help="Sampling interval in seconds")
    p.add_argument("--device", default="STM32F407IG", help="Target device name")
    p.add_argument("--speed", default="4000", help="SWD speed in kHz")
    p.add_argument("--port", default="2331", help="GDB server port")
    p.add_argument("--ip", default=None, help="J-Link IP address, e.g. 192.168.31.50")
    p.add_argument(
        "--gdb",
        default="/opt/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gdb",
        help="arm-none-eabi-gdb path",
    )
    p.add_argument("--jlink", default="JLinkGDBServerCL", help="JLinkGDBServerCL path")
    return p.parse_args()


def main():
    args = parse_args()
    repo = Path(__file__).resolve().parents[1]
    elf = (repo / args.elf).resolve()
    out = (repo / args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    sample_n = max(1, int(args.duration / args.interval))
    sleep_str = f"{args.interval:.6f}"
    gdb_script = textwrap.dedent(
        f"""
        set pagination off
        set confirm off
        target remote :{args.port}
        set logging file {out}
        set logging overwrite on
        set logging redirect on
        set logging on
        printf "t,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,yaw_deg,yaw_total_deg,roll_deg,pitch_deg,gyro_bias_x,gyro_bias_y,gyro_bias_z,chi_square,chi_thre,chi_rejected,dt\\n"
        set $i = 0
        while $i < {sample_n}
          monitor halt
          printf "%.6f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%.9g\\n", $i*{args.interval:.9f}, INS->INS_data.INS_gyro[0], INS->INS_data.INS_gyro[1], INS->INS_data.INS_gyro[2], INS->INS_data.INS_accel[0], INS->INS_data.INS_accel[1], INS->INS_data.INS_accel[2], INS->output.INS_angle_deg[2], INS->output.Yaw_total_angle_deg, INS->output.INS_angle_deg[0], INS->output.INS_angle_deg[1], QEKF_INS.GyroBias[0], QEKF_INS.GyroBias[1], QEKF_INS.GyroBias[2], QEKF_INS.ChiSquare, QEKF_INS.ChiSquareTestThreshold, QEKF_INS.ChiSquareRejectFlag, INS->timing_time
          monitor go
          shell sleep {sleep_str}
          set $i = $i + 1
        end
        set logging off
        quit
        """
    ).strip() + "\n"

    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".gdb") as fp:
        fp.write(gdb_script)
        gdb_script_path = fp.name

    jlink_log = out.with_suffix(".jlink.log")
    gdb_log = out.with_suffix(".gdb.log")

    server_cmd = [
        args.jlink,
        "-device",
        args.device,
        "-if",
        "SWD",
        "-speed",
        str(args.speed),
    ]
    if args.ip:
        server_cmd.extend(["-select", f"IP={args.ip}"])
    server_cmd.extend([
        "-port",
        str(args.port),
        "-swoport",
        "2332",
        "-telnetport",
        "2333",
        "-singlerun",
        "-nogui",
    ])

    server = None
    try:
        with open(jlink_log, "w") as jlog:
            server = subprocess.Popen(server_cmd, stdout=jlog, stderr=subprocess.STDOUT, cwd=repo)
        # Give server a moment to start
        subprocess.run(["sleep", "1"], check=True, cwd=repo)

        with open(gdb_log, "w") as glog:
            gdb_cmd = [args.gdb, "-q", str(elf), "-x", gdb_script_path]
            result = subprocess.run(gdb_cmd, stdout=glog, stderr=subprocess.STDOUT, cwd=repo)
            if result.returncode != 0:
                raise RuntimeError(
                    f"GDB sampling failed (code={result.returncode}). Check logs: {gdb_log}, {jlink_log}"
                )
    finally:
        if server and server.poll() is None:
            server.terminate()
            try:
                server.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.kill(server.pid, signal.SIGKILL)
        try:
            os.remove(gdb_script_path)
        except OSError:
            pass

    print(f"[ekf_autosample] done: {out} ({sample_n} samples target)")
    print(f"[ekf_autosample] logs: {gdb_log}, {jlink_log}")


if __name__ == "__main__":
    main()
