#!/usr/bin/env python3
"""
ardupilot_osd.py
────────────────
Reads an ArduPilot .bin dataflash log and renders a transparent
ProRes 4444 (.mov) OSD overlay video for compositing in DaVinci Resolve,
Final Cut Pro, Premiere, etc.

Usage:
    python3 ardupilot_osd.py <logfile.bin> [options]

Options:
    --config   Path to config file   (default: osd_config.py in same dir)
    --out      Override output path  (default: from config)
    --offset   Time offset seconds   (default: from config)
    --fps      Override output FPS   (default: from config)
    --width    Override width px     (default: from config)
    --height   Override height px    (default: from config)
    --preview  Render first 10s only (for quick checks)
    --dump     Print all message types found in the log and exit
"""

import sys
import os
import argparse
import math
import struct
import subprocess
import tempfile
import shutil
from pathlib import Path

# ── Dependency check ─────────────────────────────────────────────────────────
def _check_deps():
    missing = []
    try:
        import matplotlib
    except ImportError:
        missing.append("matplotlib")
    try:
        from pymavlink import mavutil
    except ImportError:
        missing.append("pymavlink")
    try:
        import numpy
    except ImportError:
        missing.append("numpy")
    if missing:
        print(f"[ERROR] Missing packages: {', '.join(missing)}")
        print(f"        Run: pip3 install {' '.join(missing)} --break-system-packages")
        sys.exit(1)

_check_deps()

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.patches import FancyBboxPatch
from pymavlink import mavutil

# ── Config loader ─────────────────────────────────────────────────────────────
def load_config(config_path: str):
    import importlib.util
    spec = importlib.util.spec_from_file_location("osd_config", config_path)
    cfg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cfg)
    return cfg

# ── ArduPilot mode maps ───────────────────────────────────────────────────────
COPTER_MODES = {
    0:"Stabilize",1:"Acro",2:"AltHold",3:"Auto",4:"Guided",5:"Loiter",
    6:"RTL",7:"Circle",9:"Land",11:"Drift",13:"Sport",14:"Flip",
    15:"AutoTune",16:"PosHold",17:"Brake",18:"Throw",19:"AVOID_ADSB",
    20:"Guided_NoGPS",21:"SmartRTL",22:"FlowHold",23:"Follow",24:"ZigZag",
    25:"SystemID",26:"Heli_Autorotate",27:"AutoRTL",
}
PLANE_MODES = {
    0:"Manual",1:"Circle",2:"Stabilize",3:"Training",4:"ACRO",5:"FBW_A",
    6:"FBW_B",7:"Cruise",8:"AUTOTUNE",10:"Auto",11:"RTL",12:"Loiter",
    13:"TAKEOFF",14:"AVOID_ADSB",15:"Guided",17:"QSTABILIZE",18:"QHOVER",
    19:"QLOITER",20:"QLAND",21:"QRTL",22:"QAUTOTUNE",23:"QACRO",24:"THERMAL",
    25:"LoiterAltQLand",
}
ROVER_MODES = {
    0:"Manual",1:"Acro",3:"Steering",4:"Hold",5:"Loiter",6:"Follow",
    7:"Simple",10:"Auto",11:"RTL",12:"SmartRTL",15:"Guided",16:"Initializing",
}

# ── Log reader ────────────────────────────────────────────────────────────────
class LogData:
    """Parses .bin and exposes time-indexed telemetry."""

    def __init__(self, bin_path: str, verbose=True):
        self.path = bin_path
        self.verbose = verbose
        self.vehicle_type = "copter"

        # time-series arrays (seconds, value)
        self.gps_speed   = []   # m/s
        self.altitude    = []   # m relative
        self.altitude_abs= []   # m AMSL
        self.pitch       = []   # deg
        self.roll        = []   # deg
        self.yaw         = []   # deg
        self.flight_mode = []   # (t, mode_str)
        self.messages    = []   # (t, text, severity)

        self._parse()

    def _parse(self):
        if self.verbose:
            print(f"[log] Opening {self.path}")
        mlog = mavutil.mavlink_connection(self.path, robust_parsing=True)
        t0 = None

        while True:
            msg = mlog.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "BAD_DATA":
                continue

            # Determine base timestamp
            ts = getattr(msg, "TimeUS", None)
            if ts is not None:
                t_sec = ts / 1e6
            else:
                t_sec = None

            if mtype == "GPS":
                if t_sec is None:
                    continue
                if t0 is None:
                    t0 = t_sec
                t = t_sec - t0
                spd = getattr(msg, "Spd", None)
                alt = getattr(msg, "Alt", None)  # AMSL
                if spd is not None:
                    self.gps_speed.append((t, float(spd)))
                if alt is not None:
                    self.altitude_abs.append((t, float(alt)))

            elif mtype == "BARO":
                if t_sec is None:
                    continue
                if t0 is None:
                    t0 = t_sec
                t = t_sec - t0
                alt = getattr(msg, "Alt", None)
                if alt is not None:
                    self.altitude.append((t, float(alt)))

            elif mtype == "ATT":
                if t_sec is None:
                    continue
                if t0 is None:
                    t0 = t_sec
                t = t_sec - t0
                p = getattr(msg, "Pitch", None)
                r = getattr(msg, "Roll", None)
                y = getattr(msg, "Yaw", None)
                if p is not None:
                    self.pitch.append((t, float(p)))
                if r is not None:
                    self.roll.append((t, float(r)))
                if y is not None:
                    self.yaw.append((t, float(y)))

            elif mtype == "MODE":
                if t_sec is None:
                    continue
                if t0 is None:
                    t0 = t_sec
                t = t_sec - t0
                mode_num = getattr(msg, "Mode", 0)
                reason  = getattr(msg, "Rsn", None)
                # Try to detect vehicle type from mode numbers
                if mode_num in PLANE_MODES and mode_num not in COPTER_MODES:
                    self.vehicle_type = "plane"
                name = self._mode_name(mode_num)
                self.flight_mode.append((t, name))

            elif mtype == "MSG":
                if t_sec is None:
                    continue
                if t0 is None:
                    t0 = t_sec
                t = t_sec - t0
                text = getattr(msg, "Message", "")
                sev  = getattr(msg, "Severity", 6)
                self.messages.append((t, str(text), int(sev)))

        if t0 is None:
            print("[warn] No timestamped messages found — log may be empty or corrupt.")
            return

        # Compute relative altitude from BARO Alt by zeroing at start
        if self.altitude:
            alt0 = self.altitude[0][1]
            self.altitude = [(t, a - alt0) for t, a in self.altitude]

        duration = max(
            self.altitude[-1][0] if self.altitude else 0,
            self.gps_speed[-1][0] if self.gps_speed else 0,
            self.pitch[-1][0] if self.pitch else 0,
        )
        if self.verbose:
            print(f"[log] Vehicle type: {self.vehicle_type}")
            print(f"[log] Duration: {duration:.1f}s")
            print(f"[log] GPS speed samples: {len(self.gps_speed)}")
            print(f"[log] Altitude samples:  {len(self.altitude)}")
            print(f"[log] Attitude samples:  {len(self.pitch)}")
            print(f"[log] Mode changes:      {len(self.flight_mode)}")
            print(f"[log] Status messages:   {len(self.messages)}")

    def _mode_name(self, num):
        if self.vehicle_type == "plane":
            return PLANE_MODES.get(num, f"Mode {num}")
        elif self.vehicle_type == "rover":
            return ROVER_MODES.get(num, f"Mode {num}")
        return COPTER_MODES.get(num, f"Mode {num}")

    def duration(self) -> float:
        candidates = [
            self.altitude[-1][0] if self.altitude else 0,
            self.gps_speed[-1][0] if self.gps_speed else 0,
            self.pitch[-1][0] if self.pitch else 0,
            self.flight_mode[-1][0] if self.flight_mode else 0,
        ]
        return max(candidates)

    def sample(self, series: list, t: float, default=0.0):
        """Linear interpolation at time t."""
        if not series:
            return default
        if t <= series[0][0]:
            return series[0][1]
        if t >= series[-1][0]:
            return series[-1][1]
        # Binary search
        lo, hi = 0, len(series) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if series[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, v0 = series[lo]
        t1, v1 = series[hi]
        if t1 == t0:
            return v0
        frac = (t - t0) / (t1 - t0)
        return v0 + frac * (v1 - v0)

    def mode_at(self, t: float) -> str:
        if not self.flight_mode:
            return "Unknown"
        mode = self.flight_mode[0][1]
        for mt, m in self.flight_mode:
            if mt <= t:
                mode = m
            else:
                break
        return mode

    def messages_at(self, t: float, window: float) -> list:
        """Messages visible at time t (within window seconds before t)."""
        return [(mt, txt, sev) for mt, txt, sev in self.messages
                if t - window <= mt <= t]


# ── Frame renderer ────────────────────────────────────────────────────────────
class OSDRenderer:
    def __init__(self, cfg, log: LogData):
        self.cfg = cfg
        self.log = log
        self.dpi = 100
        self.w = cfg.OUTPUT_WIDTH
        self.h = cfg.OUTPUT_HEIGHT
        self.fig_w = self.w / self.dpi
        self.fig_h = self.h / self.dpi

    def _speed_str(self, ms: float):
        u = self.cfg.SPEED_UNITS
        if u == "kmh":
            return f"{ms * 3.6:.0f}", "km/h"
        elif u == "mph":
            return f"{ms * 2.237:.0f}", "mph"
        return f"{ms:.1f}", "m/s"

    def render_frame(self, t: float) -> np.ndarray:
        cfg  = self.cfg
        log  = self.log
        fig  = plt.figure(figsize=(self.fig_w, self.fig_h), dpi=self.dpi)
        ax   = fig.add_axes([0, 0, 1, 1])
        ax.set_xlim(0, self.w)
        ax.set_ylim(0, self.h)
        ax.axis("off")
        fig.patch.set_alpha(0)
        ax.set_facecolor((0, 0, 0, 0))

        m   = cfg.LOWER_THIRD_MARGIN_PX
        ph  = self.h * cfg.LOWER_THIRD_HEIGHT_FRACTION
        pw  = self.w - 2 * m
        px  = m
        py  = m

        # ── Panel background ──────────────────────────────────────────────
        panel = FancyBboxPatch(
            (px, py), pw, ph,
            boxstyle="round,pad=4",
            facecolor=cfg.COLOR_BG,
            edgecolor=(1, 1, 1, 0.06),
            linewidth=0.5,
            zorder=1,
        )
        ax.add_patch(panel)

        # Accent line along top of panel
        ax.plot(
            [px + 8, px + pw - 8],
            [py + ph, py + ph],
            color=cfg.COLOR_ACCENT, linewidth=cfg.ATTITUDE_BAR_THICKNESS,
            solid_capstyle="round", zorder=2,
        )

        # ── Layout columns ───────────────────────────────────────────────
        # We'll place fields left-to-right with flexible widths
        fields  = cfg.ENABLED_FIELDS
        col_pad = 32
        x_cursor = px + col_pad
        cell_h   = ph
        base_y   = py  # bottom of panel

        def label(ax, x, y, txt):
            ax.text(x, y + cell_h * 0.72, txt.upper(),
                    color=cfg.COLOR_LABEL,
                    fontsize=cfg.FONT_LABEL_SIZE,
                    fontweight="normal",
                    fontfamily="monospace",
                    va="bottom", ha="left", zorder=3)

        def value(ax, x, y, txt, unit="", small=False):
            vs = cfg.FONT_VALUE_SIZE if not small else cfg.FONT_VALUE_SIZE * 0.75
            ax.text(x, y + cell_h * 0.38, txt,
                    color=cfg.COLOR_VALUE,
                    fontsize=vs,
                    fontweight="bold",
                    va="center", ha="left", zorder=3)
            if unit:
                ax.text(x + len(txt) * vs * 0.62 + 2, y + cell_h * 0.38, unit,
                        color=cfg.COLOR_LABEL,
                        fontsize=cfg.FONT_LABEL_SIZE,
                        va="center", ha="left", zorder=3)

        def divider(ax, x):
            ax.plot([x, x], [base_y + 10, base_y + cell_h - 10],
                    color=(1, 1, 1, 0.10), linewidth=0.5, zorder=2)

        fixed_w = sum(
            130 if f in ("speed","altitude") else
            240 if f == "attitude" else
            150 if f == "flight_mode" else 0
            for f in fields if f != "messages"
        ) + col_pad * max(len(fields)-1, 0)

        COL_W = {
            "speed":       130,
            "altitude":    130,
            "attitude":    240,
            "flight_mode": 150,
            "messages":    max(100, pw - fixed_w - col_pad * 2),
        }

        for i, field in enumerate(fields):
            cw = COL_W.get(field, 120)

            if field == "speed":
                spd_ms = log.sample(log.gps_speed, t)
                v_str, u_str = self._speed_str(spd_ms)
                label(ax, x_cursor, base_y, "Speed")
                value(ax, x_cursor, base_y, v_str, u_str)

            elif field == "altitude":
                if cfg.ALTITUDE_DATUM == "absolute":
                    alt = log.sample(log.altitude_abs, t)
                else:
                    alt = log.sample(log.altitude, t)
                label(ax, x_cursor, base_y, "Altitude")
                value(ax, x_cursor, base_y, f"{alt:.0f}", "m")

            elif field == "attitude":
                pitch_deg = log.sample(log.pitch, t)
                roll_deg  = log.sample(log.roll, t)
                yaw_deg   = log.sample(log.yaw, t)
                label(ax, x_cursor, base_y, "Attitude  Pitch / Roll / Yaw")

                bar_y   = base_y + cell_h * 0.42
                bar_x0  = x_cursor
                bar_x1  = x_cursor + cw - col_pad
                bar_len = bar_x1 - bar_x0

                def att_bar(ax, bx0, blen, by, val, rng, color, label_str):
                    # Track
                    ax.plot([bx0, bx0 + blen], [by, by],
                            color=(1,1,1,0.12), linewidth=3,
                            solid_capstyle="round", zorder=3)
                    # Indicator
                    frac  = max(-1, min(1, val / rng))
                    mid   = bx0 + blen / 2
                    ind_x = mid + frac * (blen / 2)
                    ax.plot([ind_x], [by], marker="o", color=color,
                            markersize=7, zorder=4)
                    # Centre tick
                    ax.plot([mid, mid], [by - 4, by + 4],
                            color=(1,1,1,0.25), linewidth=0.8, zorder=3)
                    # Value label
                    ax.text(bx0 + blen + 4, by, f"{val:+.0f}°",
                            color=cfg.COLOR_LABEL,
                            fontsize=9, va="center", ha="left", zorder=4)
                    ax.text(bx0 - 4, by, label_str,
                            color=cfg.COLOR_LABEL,
                            fontsize=9, va="center", ha="right", zorder=4)

                blen    = (cw - col_pad) * 0.7
                bx0     = x_cursor + (cw - col_pad - blen) / 2
                spacing = cell_h * 0.2

                att_bar(ax, bx0, blen, bar_y + spacing,
                        pitch_deg, cfg.PITCH_RANGE_DEG, cfg.COLOR_ACCENT, "P")
                att_bar(ax, bx0, blen, bar_y,
                        roll_deg, cfg.ROLL_RANGE_DEG, cfg.COLOR_ACCENT, "R")
                # Yaw as compass text
                ax.text(bx0 + blen / 2, bar_y - spacing,
                        f"↑ {yaw_deg:.0f}°",
                        color=cfg.COLOR_LABEL,
                        fontsize=9, va="center", ha="center", zorder=4)

            elif field == "flight_mode":
                mode = log.mode_at(t)
                label(ax, x_cursor, base_y, "Mode")
                # Pill badge
                pill_w  = min(cw - 16, len(mode) * cfg.FONT_MODE_SIZE * 0.68 + 24)
                pill_h  = cell_h * 0.32
                pill_y  = base_y + cell_h * 0.22
                pill = FancyBboxPatch(
                    (x_cursor, pill_y), pill_w, pill_h,
                    boxstyle="round,pad=3",
                    facecolor=cfg.COLOR_MODE_BG,
                    edgecolor=(0,0,0,0),
                    zorder=3,
                )
                ax.add_patch(pill)
                ax.text(x_cursor + pill_w / 2, pill_y + pill_h / 2,
                        mode,
                        color=cfg.COLOR_MODE_TEXT,
                        fontsize=cfg.FONT_MODE_SIZE,
                        fontweight="bold",
                        va="center", ha="center", zorder=4)

            elif field == "messages":
                msgs = log.messages_at(t, cfg.MESSAGE_DISPLAY_SECONDS)
                label(ax, x_cursor, base_y, "Messages")
                msg_y = base_y + cell_h * 0.55
                max_show = 2
                visible = msgs[-max_show:] if len(msgs) > max_show else msgs
                for mi, (mt, txt, sev) in enumerate(reversed(visible)):
                    age   = t - mt
                    alpha = max(0.2, 1.0 - age / cfg.MESSAGE_DISPLAY_SECONDS)
                    col   = cfg.COLOR_WARN if sev <= 3 else cfg.COLOR_MESSAGE
                    col   = col[:3] + (alpha,)
                    disp  = txt[:cfg.MESSAGE_MAX_CHARS]
                    ax.text(x_cursor, msg_y - mi * cell_h * 0.25,
                            disp,
                            color=col,
                            fontsize=cfg.FONT_MSG_SIZE,
                            va="center", ha="left",
                            clip_on=True,
                            zorder=4)

            # Divider before next field
            if i < len(fields) - 1:
                divider(ax, x_cursor + cw)

            x_cursor += cw

        # ── Convert to RGBA numpy array ───────────────────────────────────
        fig.canvas.draw()
        buf = fig.canvas.buffer_rgba()
        img = np.frombuffer(buf, dtype=np.uint8).reshape(self.h, self.w, 4).copy()
        plt.close(fig)
        return img


# ── FFmpeg pipe writer ────────────────────────────────────────────────────────
class VideoWriter:
    """Pipes RGBA frames to ffmpeg → ProRes 4444 (transparent) .mov"""

    def __init__(self, out_path: str, width: int, height: int, fps: float):
        self.out_path = out_path
        self.fps = fps
        # ProRes 4444 supports alpha channel — perfect for compositing
        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-pix_fmt", "rgba",
            "-s", f"{width}x{height}",
            "-r", str(fps),
            "-i", "-",
            "-vcodec", "prores_ks",
            "-profile:v", "4444",
            "-pix_fmt", "yuva444p10le",
            "-vendor", "apl0",
            "-bits_per_mb", "8000",
            out_path,
        ]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL)

    def write(self, rgba: np.ndarray):
        self.proc.stdin.write(rgba.tobytes())

    def close(self):
        self.proc.stdin.close()
        self.proc.wait()


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="ArduPilot OSD overlay generator")
    p.add_argument("bin",        help="Path to ArduPilot .bin dataflash log")
    p.add_argument("--config",   default=None, help="Path to osd_config.py")
    p.add_argument("--out",      default=None, help="Output .mov path")
    p.add_argument("--offset",   type=float, default=None)
    p.add_argument("--fps",      type=float, default=None)
    p.add_argument("--width",    type=int,   default=None)
    p.add_argument("--height",   type=int,   default=None)
    p.add_argument("--preview",  action="store_true",
                   help="Render first 10 seconds only")
    p.add_argument("--dump",     action="store_true",
                   help="Print message types in log and exit")
    return p.parse_args()


def main():
    args = parse_args()

    # Load config
    script_dir  = Path(__file__).parent
    config_path = args.config or str(script_dir / "osd_config.py")
    if not os.path.exists(config_path):
        print(f"[ERROR] Config not found: {config_path}")
        sys.exit(1)
    cfg = load_config(config_path)

    # Apply CLI overrides
    if args.out:     cfg.OUTPUT_FILE = args.out
    if args.offset is not None: cfg.TIME_OFFSET_SECONDS = args.offset
    if args.fps:     cfg.OUTPUT_FPS  = args.fps
    if args.width:   cfg.OUTPUT_WIDTH  = args.width
    if args.height:  cfg.OUTPUT_HEIGHT = args.height

    # Dump mode
    if args.dump:
        print("Scanning log message types...")
        mlog = mavutil.mavlink_connection(args.bin, robust_parsing=True)
        types = set()
        while True:
            msg = mlog.recv_match(blocking=False)
            if msg is None:
                break
            types.add(msg.get_type())
        for t in sorted(types):
            print(f"  {t}")
        return

    # Load log
    log = LogData(args.bin)
    duration = log.duration()
    if duration == 0:
        print("[ERROR] No telemetry data found in log.")
        sys.exit(1)

    if args.preview:
        duration = min(duration, 10.0)
        print(f"[preview] Rendering first {duration:.1f}s")

    fps      = cfg.OUTPUT_FPS
    offset   = cfg.TIME_OFFSET_SECONDS
    n_frames = int(duration * fps)
    out_path = cfg.OUTPUT_FILE

    print(f"[render] {n_frames} frames @ {fps}fps → {out_path}")
    print(f"[render] Resolution: {cfg.OUTPUT_WIDTH}x{cfg.OUTPUT_HEIGHT}")

    renderer = OSDRenderer(cfg, log)
    writer   = VideoWriter(out_path, cfg.OUTPUT_WIDTH, cfg.OUTPUT_HEIGHT, fps)

    try:
        for i in range(n_frames):
            t = i / fps + offset
            if i % fps == 0:
                pct = i / n_frames * 100
                print(f"\r[render] {pct:5.1f}%  {i}/{n_frames}", end="", flush=True)
            frame = renderer.render_frame(t)
            writer.write(frame)
    except KeyboardInterrupt:
        print("\n[interrupted]")
    finally:
        writer.close()

    print(f"\n[done] Saved: {out_path}")
    print()
    print("── Compositing tips ────────────────────────────────────────────────")
    print("  DaVinci Resolve : Place overlay on track above footage.")
    print("                    Set composite mode to Normal — alpha is baked in.")
    print("  Final Cut Pro   : Drop onto storyline above clip, set blend to Normal.")
    print("  Premiere Pro    : Place on V2, no keying needed (ProRes 4444 alpha).")
    print("────────────────────────────────────────────────────────────────────")


if __name__ == "__main__":
    main()
