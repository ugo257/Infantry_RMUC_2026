#!/usr/bin/env python3
"""
Njust_DZX - Real-time RTT Data Plotter for STM32 via OpenOCD

STM32 sends a fixed-size packet of N×4 bytes via SEGGER RTT every cycle.
Each 4-byte slot is either a float or int32 — the type is selectable
per-channel *live* from the GUI.  The number of channels is auto-detected
from the first packet length, or set via --num-channels.

Variable names and data types can all be changed at runtime in the GUI.

=== STM32 side (C) ===
  #include "SEGGER_RTT.h"

  // Example: send 3 variables per cycle (12 bytes total)
  #pragma pack(push, 1)
  typedef struct {
      float   yaw;
      float   pitch;
      int32_t motor_rpm;
  } rtt_packet_t;
  #pragma pack(pop)

  void rtt_send(float yaw, float pitch, int32_t rpm) {
      rtt_packet_t pkt = { yaw, pitch, rpm };
      SEGGER_RTT_Write(0, &pkt, sizeof(pkt));
  }

=== OpenOCD RTT setup ===
  rtt setup 0x20000000 0x10000 "SEGGER RTT"
  rtt start
  rtt server start 9090 0

=== Usage ===
  # Simulate 3 channels:
  python Njust_DZX.py --simulate -n 3

  # Auto-detect from OpenOCD:
  python Njust_DZX.py --port 9090

  # Pre-set 2 channels with names:
  python Njust_DZX.py --simulate -n 2 --names "yaw,pitch" --dtypes "float,int32"

Author : Njust_DZX
License: MIT
"""

from __future__ import annotations

import argparse
import bisect
import collections
import math
import random
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider, TextBox

# ──────────────────────────────────────────────────────────────────
# Supported 4-byte interpretations (each variable is always 4 bytes)
# ──────────────────────────────────────────────────────────────────

DTYPE_LABELS = ["float", "int32"]

def interpret_4bytes(raw: bytes, dtype: str) -> float:
    if dtype == "int32":
        return float(struct.unpack("<i", raw)[0])
    else:  # float
        return float(struct.unpack("<f", raw)[0])

def format_value(val: float, dtype: str) -> str:
    if dtype == "int32":
        return f"{val:.0f}"
    else:
        return f"{val:.4f}"


# ──────────────────────────────────────────────────────────────────
# Thread-safe multi-channel ring buffer
# ──────────────────────────────────────────────────────────────────

class DataRing:
    def __init__(self, n_channels: int, maxlen: int = 20000):
        self.n_channels = n_channels
        self.lock = threading.Lock()
        self.timestamps: Deque[float] = collections.deque(maxlen=maxlen)
        self.channels: List[Deque[float]] = [
            collections.deque(maxlen=maxlen) for _ in range(n_channels)
        ]
        self._t0: Optional[float] = None
        self._count = 0

    def push(self, values: List[float]) -> None:
        now = time.monotonic()
        with self.lock:
            if self._t0 is None:
                self._t0 = now
            self.timestamps.append(now - self._t0)
            for i, v in enumerate(values):
                self.channels[i].append(v)
            self._count += 1

    def snapshot(self) -> Tuple[List[float], List[List[float]], int]:
        with self.lock:
            ts = list(self.timestamps)
            chs = [list(ch) for ch in self.channels]
            cnt = self._count
        return ts, chs, cnt


# ──────────────────────────────────────────────────────────────────
# Per-channel live configuration (name + dtype), GUI-editable
# ──────────────────────────────────────────────────────────────────

class ChannelConfig:
    def __init__(self, index: int, name: str = "", dtype: str = "float"):
        self.index = index
        self.lock = threading.Lock()
        self._name = name or f"ch{index}"
        self._dtype = dtype
        self._visible = True

    @property
    def name(self) -> str:
        with self.lock:
            return self._name

    @name.setter
    def name(self, v: str) -> None:
        with self.lock:
            self._name = v

    @property
    def dtype(self) -> str:
        with self.lock:
            return self._dtype

    @dtype.setter
    def dtype(self, v: str) -> None:
        with self.lock:
            self._dtype = v

    @property
    def visible(self) -> bool:
        with self.lock:
            return self._visible

    @visible.setter
    def visible(self, v: bool) -> None:
        with self.lock:
            self._visible = v


# ──────────────────────────────────────────────────────────────────
# RTT TCP receiver thread
# ──────────────────────────────────────────────────────────────────

class RTTReceiver:
    def __init__(
        self,
        host: str,
        port: int,
        ring: DataRing,
        configs: List[ChannelConfig],
        *,
        reconnect_interval: float = 2.0,
    ):
        self.host = host
        self.port = port
        self.ring = ring
        self.configs = configs
        self.n = len(configs)
        self.packet_size = self.n * 4
        self.reconnect_interval = reconnect_interval
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True, name="rtt-rx")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
                sock.settimeout(0.05)
                print(f"[Njust_DZX] Connected to RTT server {self.host}:{self.port}")
            except OSError as e:
                print(f"[Njust_DZX] Connect failed ({e}), retry in {self.reconnect_interval}s ...")
                self._stop.wait(self.reconnect_interval)
                continue

            buf = bytearray()
            try:
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(4096)
                        if not chunk:
                            break
                        buf.extend(chunk)
                    except socket.timeout:
                        pass

                    while len(buf) >= self.packet_size:
                        values: List[float] = []
                        for i in range(self.n):
                            raw = bytes(buf[i * 4 : i * 4 + 4])
                            values.append(interpret_4bytes(raw, self.configs[i].dtype))
                        del buf[:self.packet_size]
                        self.ring.push(values)
            except OSError:
                pass
            finally:
                sock.close()
                print("[Njust_DZX] Disconnected, reconnecting ...")
                self._stop.wait(self.reconnect_interval)


# ──────────────────────────────────────────────────────────────────
# Simulator (N channels of synthetic data)
# ──────────────────────────────────────────────────────────────────

class Simulator:
    def __init__(self, ring: DataRing, n_channels: int, hz: float):
        self.ring = ring
        self.n = n_channels
        self.hz = hz
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True, name="sim")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def _run(self) -> None:
        period = 1.0 / self.hz
        t0 = time.monotonic()
        tick = 0
        while not self._stop.is_set():
            t = tick * period
            values: List[float] = []
            for i in range(self.n):
                freq = 0.3 + i * 0.25
                amp = 10.0 + i * 8.0
                noise = random.gauss(0, 0.3)
                values.append(amp * math.sin(2 * math.pi * freq * t + i * 1.2) + noise)
            self.ring.push(values)
            tick += 1
            target = t0 + tick * period
            dt = target - time.monotonic()
            if dt > 0:
                time.sleep(dt)


# ──────────────────────────────────────────────────────────────────
# Real-time plotter with per-channel interactive controls
# ──────────────────────────────────────────────────────────────────

COLORS = [
    "#EE7733", "#0077BB", "#33BBEE", "#EE3377",
    "#009988", "#CC3311", "#332288", "#44AA99",
    "#882255", "#DDCC77", "#117733", "#BBBBBB",
]


def run_plot(
    ring: DataRing,
    configs: List[ChannelConfig],
    window_s: float,
    save_path: Optional[Path],
) -> None:
    n = len(configs)

    # ── Dynamic layout based on channel count ──
    if n <= 2:
        ctrl_width = 0.24
        fig_h = 6.5
        font_hdr = 10
        font_val = 10
        font_ctrl = 9
    elif n <= 4:
        ctrl_width = 0.28
        fig_h = max(7.0, 2.0 + n * 1.3)
        font_hdr = 9
        font_val = 9
        font_ctrl = 8
    elif n <= 6:
        ctrl_width = 0.30
        fig_h = max(8.0, 2.0 + n * 1.2)
        font_hdr = 8
        font_val = 8
        font_ctrl = 7
    else:
        ctrl_width = 0.32
        fig_h = max(9.0, 2.0 + n * 1.0)
        font_hdr = 7
        font_val = 7
        font_ctrl = 7

    plot_right = 1.0 - ctrl_width - 0.02

    fig = plt.figure(figsize=(16, fig_h))
    fig.canvas.manager.set_window_title("RTT Plotter")  # type: ignore[union-attr]

    # Leave room at the bottom for sliders (3 rows).
    ax = fig.add_axes([0.06, 0.22, plot_right - 0.06, 0.70])

    # ── X Window slider (min 10 ms, step 10 ms) ──
    ax_xslider = fig.add_axes([0.06, 0.12, plot_right - 0.06, 0.022])
    x_slider = Slider(ax_xslider, "X Window (s)", 0.01, 120, valinit=window_s, valstep=0.01)
    _x_window = [window_s]

    def _on_xslider(val: float) -> None:
        _x_window[0] = val

    x_slider.on_changed(_on_xslider)

    # ── Y Scale slider ──
    ax_yslider = fig.add_axes([0.06, 0.07, plot_right - 0.06, 0.022])
    y_slider = Slider(ax_yslider, "Y Scale", 0, 500, valinit=0, valstep=1)
    y_slider.valtext.set_text("Auto")
    _y_scale = [0.0]

    def _on_yslider(val: float) -> None:
        _y_scale[0] = val
        if val == 0:
            y_slider.valtext.set_text("Auto")
        else:
            y_slider.valtext.set_text(f"±{val:.0f}")

    y_slider.on_changed(_on_yslider)

    # ── History scroll slider (0 = live, drag left to scroll back) ──
    ax_hslider = fig.add_axes([0.06, 0.02, plot_right - 0.06, 0.022])
    h_slider = Slider(ax_hslider, "History (s)", 0, 300, valinit=0, valstep=0.5)
    h_slider.valtext.set_text("LIVE")
    _history_offset = [0.0]  # 0 = live (show latest)

    def _on_hslider(val: float) -> None:
        _history_offset[0] = val
        if val == 0:
            h_slider.valtext.set_text("LIVE")
        else:
            h_slider.valtext.set_text(f"-{val:.1f}s")

    h_slider.on_changed(_on_hslider)

    lines = []
    for i in range(n):
        color = COLORS[i % len(COLORS)]
        ln, = ax.plot([], [], label=configs[i].name, color=color, linewidth=1.3)
        lines.append(ln)

    legend = ax.legend(loc="upper right", fontsize=font_ctrl, ncol=min(n, 4))

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.set_title("RTT Plotter")
    ax.grid(True, alpha=0.3)

    # ── Mouse hover crosshair + value annotation ──
    _vline = ax.axvline(x=0, color="gray", linewidth=0.8, linestyle="--", visible=False)
    _hover_dots = [ax.plot([], [], "o", color=COLORS[i % len(COLORS)], markersize=5, visible=False)[0]
                   for i in range(n)]
    _hover_text = ax.text(0, 0, "", fontsize=8, fontfamily="monospace",
                          bbox={"facecolor": "white", "alpha": 0.90, "edgecolor": "gray",
                                "boxstyle": "round,pad=0.3"},
                          visible=False, zorder=10)
    # Shared snapshot reference for hover callback (updated each animation frame).
    _snap: Dict[str, object] = {"ts": [], "chs": []}
    # Last known mouse position in **display (pixel)** coordinates (None = outside plot).
    _mouse_px: List[Optional[float]] = [None, None]  # [x_pixel, y_pixel]

    def _update_hover() -> None:
        """Refresh crosshair & annotation using stored mouse pos + latest snapshot."""
        px, py = _mouse_px
        if px is not None:
            try:
                data_pt = ax.transData.inverted().transform((px, py))
                mx, my = float(data_pt[0]), float(data_pt[1])
            except Exception:
                mx, my = None, None
        else:
            mx, my = None, None
        ts_list = _snap["ts"]
        chs_list = _snap["chs"]
        if mx is None or not ts_list:
            _vline.set_visible(False)
            _hover_text.set_visible(False)
            for d in _hover_dots:
                d.set_visible(False)
            return

        # Binary search for nearest sample.
        idx = bisect.bisect_left(ts_list, mx)
        if idx >= len(ts_list):
            idx = len(ts_list) - 1
        elif idx > 0:
            if abs(ts_list[idx - 1] - mx) < abs(ts_list[idx] - mx):
                idx = idx - 1

        t_val = ts_list[idx]
        _vline.set_xdata([t_val])
        _vline.set_visible(True)

        parts = []
        for i in range(n):
            if not configs[i].visible:
                _hover_dots[i].set_visible(False)
                continue
            ch = chs_list[i]
            if idx < len(ch):
                v = ch[idx]
                _hover_dots[i].set_data([t_val], [v])
                _hover_dots[i].set_visible(True)
                parts.append(f"{configs[i].name} = {format_value(v, configs[i].dtype)}")
            else:
                _hover_dots[i].set_visible(False)

        _hover_text.set_text("\n".join(parts))
        _hover_text.set_position((t_val, my if my is not None else 0))
        _hover_text.set_visible(True)

    def _on_mouse_move(event) -> None:
        if event.inaxes != ax:
            _mouse_px[0] = None
            _mouse_px[1] = None
        else:
            _mouse_px[0] = event.x
            _mouse_px[1] = event.y

    fig.canvas.mpl_connect("motion_notify_event", _on_mouse_move)
    fig.canvas.mpl_connect("axes_leave_event", lambda e: (_mouse_px.__setitem__(0, None),
                                                           _mouse_px.__setitem__(1, None)))

    # ── Per-channel controls on the right (dynamic sizing) ──
    ctrl_left = plot_right + 0.02
    ctrl_w = ctrl_width - 0.02
    panel_top = 0.92
    panel_bot = 0.22
    slot_height = (panel_top - panel_bot) / max(n, 1)

    _widgets: list = []
    _value_texts: list = []

    for i in range(n):
        base_y = panel_bot + (n - 1 - i) * slot_height
        color = COLORS[i % len(COLORS)]

        # ── Row 1 (top): Header + live value ──
        ax_hdr = fig.add_axes([ctrl_left, base_y + slot_height * 0.82, ctrl_w, slot_height * 0.16])
        ax_hdr.axis("off")
        ax_hdr.text(0.0, 0.5, f"── CH{i} ──", fontsize=font_hdr, fontweight="bold",
                     color=color, va="center")
        val_txt = ax_hdr.text(1.0, 0.5, "= ?", fontsize=font_val, fontfamily="monospace",
                              color=color, va="center", ha="right")
        _value_texts.append(val_txt)

        # ── Row 2: Name TextBox ──
        ax_name = fig.add_axes([ctrl_left, base_y + slot_height * 0.62, ctrl_w, slot_height * 0.17])
        tb = TextBox(ax_name, "", initial=configs[i].name)

        def _make_name_cb(idx: int):
            def _cb(text: str):
                text = text.strip()
                if text:
                    configs[idx].name = text
            return _cb
        tb.on_submit(_make_name_cb(i))
        _widgets.append(tb)

        # ── Row 3: Type toggle button + Show toggle button ──
        btn_w = ctrl_w * 0.48
        gap = ctrl_w * 0.04
        btn_h = slot_height * 0.30
        btn_y = base_y + slot_height * 0.12

        # Dtype toggle button (click to switch between float / int32)
        ax_dtype = fig.add_axes([ctrl_left, btn_y, btn_w, btn_h])
        dtype_btn = Button(ax_dtype, configs[i].dtype, color="#e0e0e0", hovercolor="#c0c0c0")
        dtype_btn.label.set_fontsize(font_ctrl)
        dtype_btn.label.set_fontfamily("monospace")

        def _make_dtype_toggle(idx: int, btn_ref):
            def _cb(_event):
                cur = configs[idx].dtype
                nxt = DTYPE_LABELS[(DTYPE_LABELS.index(cur) + 1) % len(DTYPE_LABELS)]
                configs[idx].dtype = nxt
                btn_ref.label.set_text(nxt)
                fig.canvas.draw_idle()
            return _cb
        dtype_btn.on_clicked(_make_dtype_toggle(i, dtype_btn))
        _widgets.append(dtype_btn)

        # Show toggle button (green = visible, red = hidden)
        _show_color = "#4CAF50" if configs[i].visible else "#F44336"
        _show_label = "ON" if configs[i].visible else "OFF"
        ax_show = fig.add_axes([ctrl_left + btn_w + gap, btn_y, btn_w, btn_h])
        show_btn = Button(ax_show, _show_label, color=_show_color, hovercolor="#888888")
        show_btn.label.set_fontsize(font_ctrl)
        show_btn.label.set_fontweight("bold")
        show_btn.label.set_color("white")

        def _make_vis_toggle(idx: int, btn_ref, ax_ref):
            def _cb(_event):
                configs[idx].visible = not configs[idx].visible
                vis = configs[idx].visible
                c = "#4CAF50" if vis else "#F44336"
                btn_ref.label.set_text("ON" if vis else "OFF")
                ax_ref.set_facecolor(c)
                btn_ref.color = c
                fig.canvas.draw_idle()
            return _cb
        show_btn.on_clicked(_make_vis_toggle(i, show_btn, ax_show))
        _widgets.append(show_btn)

    # ── Animation ──

    def _update(_frame: int) -> list:
        ts, chs, cnt = ring.snapshot()
        if not ts:
            return lines + _value_texts

        # Store snapshot for hover callback.
        _snap["ts"] = ts
        _snap["chs"] = chs

        # Update lines visibility and data.
        for i, ln in enumerate(lines):
            vis = configs[i].visible
            ln.set_visible(vis)
            if vis:
                ln.set_data(ts, chs[i])
            else:
                ln.set_data([], [])

            name = configs[i].name
            ln.set_label(name)
            legend_texts = legend.get_texts()
            if i < len(legend_texts):
                legend_texts[i].set_text(name)

        # X-axis: live or scrolled back.
        xwin = _x_window[0]
        offset = _history_offset[0]
        if offset == 0:
            # Live mode: show the latest data.
            xmax = max(xwin, ts[-1])
            xmin = max(0.0, xmax - xwin)
        else:
            # History mode: scroll back by offset seconds from latest.
            xmax = max(0.0, ts[-1] - offset)
            xmin = max(0.0, xmax - xwin)
        ax.set_xlim(xmin, xmax)

        # y-limits from visible channels in the current x range.
        vis_start = 0
        vis_end = len(ts)
        for j, t in enumerate(ts):
            if t >= xmin:
                vis_start = j
                break
        for j in range(len(ts) - 1, -1, -1):
            if ts[j] <= xmax:
                vis_end = j + 1
                break

        y_all: List[float] = []
        for i, ch in enumerate(chs):
            if configs[i].visible:
                y_all.extend(ch[vis_start:vis_end])
        if y_all:
            scale = _y_scale[0]
            if scale == 0:
                ymin_v, ymax_v = min(y_all), max(y_all)
                margin = max(0.5, (ymax_v - ymin_v) * 0.12)
                ax.set_ylim(ymin_v - margin, ymax_v + margin)
            else:
                mid = (min(y_all) + max(y_all)) / 2.0
                ax.set_ylim(mid - scale, mid + scale)

        # Update per-channel value text on the right panel.
        # Show value at the right edge of the visible window.
        for i in range(n):
            name = configs[i].name
            dtype = configs[i].dtype
            # Find the value closest to xmax.
            if chs[i] and vis_end > 0:
                idx = min(vis_end - 1, len(chs[i]) - 1)
                val_str = format_value(chs[i][idx], dtype)
            else:
                val_str = "?"
            vis_tag = "" if configs[i].visible else " [hidden]"
            _value_texts[i].set_text(f"{name} = {val_str}{vis_tag}")

        # Refresh hover crosshair every frame (even if mouse didn't move).
        _update_hover()

        return lines + _value_texts

    interval_ms = max(1, int(1000 / 1000))  # 1 kHz refresh
    _anim = FuncAnimation(fig, _update, interval=interval_ms, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass

    if save_path:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(str(save_path), dpi=150, bbox_inches="tight")
        print(f"[Njust_DZX] Plot saved to {save_path}")


# ──────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Njust_DZX — Real-time RTT data plotter for STM32 via OpenOCD",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
STM32 sends N×4 bytes per packet.  Each 4-byte slot is float or int32.
Number of channels auto-detected from packet size, or set with -n.
Names and types are editable live in the GUI.

Examples:
  python Njust_DZX.py --simulate -n 3
  python Njust_DZX.py --simulate -n 2 --names "yaw,pitch" --dtypes "float,int32"
  python Njust_DZX.py --port 9090 -n 4
""",
    )
    parser.add_argument("--host", default="127.0.0.1", help="OpenOCD RTT TCP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="OpenOCD RTT TCP port (default: 9090)")
    parser.add_argument("-n", "--num-channels", type=int, default=3, help="Number of 4-byte variables per packet (default: 3)")
    parser.add_argument("--names", default="", help='Comma-separated initial names, e.g. "yaw,pitch,rpm"')
    parser.add_argument("--dtypes", default="", help='Comma-separated initial types (float/int32), e.g. "float,float,int32"')
    parser.add_argument("--hz", type=float, default=1000.0, help="Expected STM32 send rate in Hz (default: 1000)")
    parser.add_argument("--window", type=float, default=10.0, help="Visible time window in seconds (default: 10)")
    parser.add_argument("--maxlen", type=int, default=0, help="Ring buffer max samples (default: auto)")
    parser.add_argument("--save", type=Path, default=None, help="Save final plot to this path on exit")
    parser.add_argument("--simulate", action="store_true", help="Generate fake data (no OpenOCD needed)")
    args = parser.parse_args()

    n = args.num_channels
    names = [s.strip() for s in args.names.split(",") if s.strip()] if args.names else []
    dtypes = [s.strip() for s in args.dtypes.split(",") if s.strip()] if args.dtypes else []

    configs: List[ChannelConfig] = []
    for i in range(n):
        name = names[i] if i < len(names) else f"ch{i}"
        dtype = dtypes[i] if i < len(dtypes) else "float"
        if dtype not in DTYPE_LABELS:
            print(f"Warning: unknown dtype '{dtype}' for ch{i}, defaulting to float.", file=sys.stderr)
            dtype = "float"
        configs.append(ChannelConfig(i, name, dtype))

    maxlen = args.maxlen if args.maxlen > 0 else max(5000, int(args.hz * args.window * 2))
    ring = DataRing(n_channels=n, maxlen=maxlen)

    print(f"[Njust_DZX] {n} channels, packet = {n * 4} bytes, {args.hz:.0f} Hz")
    for c in configs:
        print(f"  ch{c.index}: name={c.name}  dtype={c.dtype}")

    if args.simulate:
        print(f"[Njust_DZX] Simulation mode")
        source: RTTReceiver | Simulator = Simulator(ring, n, args.hz)
    else:
        print(f"[Njust_DZX] Connecting to {args.host}:{args.port} ...")
        source = RTTReceiver(args.host, args.port, ring, configs)

    source.start()

    try:
        run_plot(ring, configs, args.window, args.save)
    finally:
        source.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
