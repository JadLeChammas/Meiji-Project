#!/usr/bin/env python3
r"""
live_monitor.py — record and decode a live session of the tennis Dt board (main_live_raw.c).

One session gives you BOTH datasets on the same MCU millisecond clock:
  - what the MCU computed  (SHOT lines)          -> *_shots.csv
  - the raw samples it used (R lines, 400 Hz)    -> *_raw.csv  and  *_takeNN_raw.bin

USAGE
  pip install pyserial
  python live_monitor.py                           # = record on DEFAULT_PORT (COM5), e.g. Run button
  python live_monitor.py record --port COM5        # Windows  (Bluetooth "outgoing" COM port)
      Ctrl+C to stop. Everything goes in a folder NEXT TO THIS SCRIPT, one per day:
          results_YYYY-MM-DD/session_HHMMSS.log         (every line, with PC time)
      and the log is decoded automatically when you stop (see FILES below).
  A live graph (needs matplotlib) shows dtTfC, dtCW and dtTfW of each shot as it arrives,
  like the old live_monitor.py; each shot is also printed in the terminal. --no-plot to hide it.
      C:\Python314\python.exe -m pip install pyserial matplotlib
  python live_monitor.py decode results_YYYY-MM-DD/session_HHMMSS.log
      decodes (again) an existing log; the files are written next to the log.

A "take" = one STREAM_START ... STREAM_STOP (one short press to start, one to stop).

FILES
  *_raw.csv    take, t_ms, seq, x, y, z, ax, ay, az, a_norm, a_norm_mcu
               t_ms   : MCU clock of each sample (first sample of a block + 2.5 ms steps)
               x,y,z  : 16-bit LIS3DH counts, identical to the old flash dump
               a_norm : sqrt(x²+y²+z²) * 0.0073575  (what accel_receiver.py computes)
               a_norm_mcu : same with the MCU's integer sqrt (exact input of its filters)
  *_takeNN_raw.bin  int16 little-endian x,y,z,x,y,z... — same format as the old flash dump,
               so the existing Python pipeline can read it unchanged. Only valid as a
               continuous 400 Hz signal if the report shows 0 missing blocks for that take.
  *_takeNN_shimmer.csv  same raw data in the Shimmer CSV layout (3 header lines,
               time in ms in column A, Accel_X/Y/Z_CAL in m/s²), so the Python reference
               5bands_dt_shimmer_V2806_0.py opens it directly, like a Shimmer file.
               Give it K = the number of MCU shots of that take (see the report).
  *_shots/takeNN_shotMMM.csv   ONE FILE PER SHOT: the raw data from --pre s before to
               --post s after that shot's MCU impact (default 4.0 s / 1.5 s), Shimmer layout
               like above, plus a last column t_rel_ms (time from the MCU impact, ms).
               The MCU's own results for that shot are the matching row of *_shots.csv
               (column "file"). The window starts 4 s before impact so the 1-3 Hz Trunk
               filter of the Python reference has room to settle before its -1.2 s window.
NUMBERING  Takes are numbered through the whole day: the first take of the day is take 1,
  the next one (same or later session) take 2, etc. Shots restart at 1 in every take.
  board_shot_no is the board's own counter (it only resets at power-on / long press).
  *_shots.csv  take, shot, board_shot_no, file, I_ms, I_from_take_start_s, I_sample, Tf, C, W, dtTfC, dtCW, dtTfW,
               dtWI, dtCI, dtTfI, Ipk, Tpk, Cpk, Wpk   (offsets in ms relative to Impact;
               peaks in m/s²; I_from_take_start_s is on the same time base as the Python
               reference's impact_time_from_start_s; I_sample = index in the take's raw data)
"""
import argparse
import bisect
import csv
import math
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

# Port used when the script is started with no arguments (e.g. the VS Code Run button).
DEFAULT_PORT = "COM5"      # Bluetooth outgoing port of the RN4678 (Windows: More Bluetooth settings > COM Ports)

ACC_CONV = 0.0073575          # raw count -> m/s^2 (16G), same as the firmware
FS = 400.0
DT_MS = 1000.0 / FS
RAW_BLOCK = 8
R_LEN = 1 + 8 + 2 + RAW_BLOCK * 9      # 83 characters without CR LF
BLOCK_MS = RAW_BLOCK * DT_MS           # 20 ms
SHOT_FIELDS = ["Tf", "C", "W", "dtTfC", "dtCW", "dtTfW", "dtWI", "dtCI", "dtTfI"]
PEAK_FIELDS = ["Ipk", "Tpk", "Cpk", "Wpk"]


# --------------------------------------------------------------------------- per-shot figure
# Delay of each MCU band (same values as DELAY_xxx_MS in main_live_raw.c): the WAVE curves
# are shifted back by these, like the event times, so curves and events line up.
DELAY_MS = {"T": 570, "C": 500, "W": 305, "I": 55}      # firmware 2026-11-10: FIR Hilbert on Chest/Wrist, smoothing on Impact
FIG_WIN = (-1.5, 0.5)            # s around impact, as in the old live_monitor figure
WAVE_COLS = ("t", "R", "T", "C", "W", "I")


def _parse_wave(s, last_r_ms):
    """'w' line -> (full ms time, R, Trunk, Chest env, Wrist env, Impact env) in m/s^2, or None.
    The line only carries the low 16 bits of the ms clock: the full time is taken from the
    previous R line (sent every 20 ms), so it can't be off by a wrap."""
    if len(s) != 25 or last_r_ms is None:
        return None
    try:
        v = [int(s[1 + 4 * k:5 + 4 * k], 16) for k in range(6)]
    except ValueError:
        return None
    t16 = v[0]
    d = ((t16 - (last_r_ms & 0xFFFF) + 0x8000) & 0xFFFF) - 0x8000
    vals = [(x - 0x10000 if x & 0x8000 else x) / 10.0 for x in v[1:]]
    return (last_r_ms + d, *vals)


# ---- band curves rebuilt from the raw samples, with the same chain as the firmware
# (3.5 g cap on the slow bands, 16 Hz anti-alias + 1 sample in 8, Butterworth bands, FIR Hilbert
#  envelope on Chest / Wrist, |y| + smoothing on Impact). Since 2026-11-10 the board no longer sends
#  WAVE lines (to save Bluetooth bandwidth), so the figures are drawn from these.
SLOW_CAP = 3.5 * 9.80665
FIR_G = {27: [-0.6321743831, -0.1991476702, -0.1064511592, -0.0635005451,
              -0.0382663986, -0.0220953596, -0.0149924415],
         15: [-0.6101728004, -0.1434097380, -0.0401191574, -0.0070748699]}
ALPHA_IMPACT = 0.8546359991532334


def _fir_env(y, n):
    import numpy as np
    g = FIR_G[n]; c = (n - 1) // 2; h = np.zeros(n)
    for i, gk in enumerate(g):
        k = 2 * i + 1; h[c + k] = gk; h[c - k] = -gk
    H = np.convolve(y, h)[:len(y)]
    yd = np.concatenate([np.zeros(c), y[:len(y) - c]])
    return np.sqrt(yd ** 2 + H ** 2)


def wave_from_raw(t_ms, R):
    """(t_ms, |a| in m/s^2) at 400 Hz -> list of (t, Rmax, Trunk, Chest env, Wrist env, Impact env)
    at 50 Hz, like the old WAVE lines. Returns None if numpy/scipy are missing or too little data."""
    try:
        import numpy as np
        from scipy.signal import butter, sosfilt, lfilter
    except ImportError:
        return None
    t = np.asarray(t_ms, float); R = np.asarray(R, float)
    if len(R) < 400:
        return None
    lp = sosfilt(butter(4, 16, fs=400, output='sos'), np.minimum(R, SLOW_CAP))
    idx = np.arange(0, len(R) - 7, 8)
    band = lambda lo, hi: sosfilt(butter(4, [lo, hi], 'band', fs=50, output='sos'), lp[idx])
    tr = band(1, 3); ce = _fir_env(band(2.5, 7), 27); we = _fir_env(band(8, 15), 15)
    yi = sosfilt(butter(4, [20, 40], 'band', fs=400, output='sos'), R)
    ei = lfilter([1 - ALPHA_IMPACT], [1, -ALPHA_IMPACT], np.abs(yi))
    rmax = R[:len(idx) * 8].reshape(-1, 8).max(axis=1)
    imax = ei[:len(idx) * 8].reshape(-1, 8).max(axis=1)
    return list(zip(t[idx], rmax, tr, ce, we, imax))


def impact_track(t_ms, R):
    """(t_ms, |a|) at 400 Hz -> (t, |a|, Impact envelope) at 400 Hz, like the Python analysis:
    20-40 Hz Butterworth (order 4, zero phase) + Hilbert envelope. Used to put 0 s exactly on
    the Impact peak in the shot figures. None if numpy/scipy are missing or too little data."""
    try:
        import numpy as np
        from scipy.signal import butter, sosfiltfilt, hilbert
    except ImportError:
        return None
    t = np.asarray(t_ms, float); R = np.asarray(R, float)
    if len(R) < 400:
        return None
    env = np.abs(hilbert(sosfiltfilt(butter(4, [20, 40], 'band', fs=400, output='sos'), R)))
    return t, R, env


def draw_shot(fig, wave, take, shot, fields, mcu_no, imp=None):
    """The old live_monitor figure: R on the left axis, the 4 MCU band signals on the right,
    Impact at 0 s, the MCU event times as dashed lines. wave = list of WAVE tuples."""
    fig.clear()
    ax1 = fig.add_subplot(111)
    ax2 = ax1.twinx()
    i_board = int(fields["I"])
    i_ms = i_board
    if imp is not None:                     # 0 s = peak of the Impact envelope (as in the Python analysis)
        ti, Ri, ei = imp
        sel = [k for k in range(len(ti)) if abs(ti[k] - i_board) <= 60]
        if sel:
            i_ms = float(ti[max(sel, key=lambda k: ei[k])])
    shift = i_ms - i_board                  # ms between the board's impact time and the Impact peak
    lo, hi = FIG_WIN
    if wave:
        w = list(zip(*wave))
        t = [x for x in w[0]]
        def rel(key):
            d = DELAY_MS.get(key, 0)
            return [(x - d - i_ms) / 1000 for x in t]
        if imp is not None:                 # composite acceleration and Impact at full rate (400 Hz)
            k0 = [k for k in range(len(ti)) if lo - 0.1 <= (ti[k] - i_ms) / 1000 <= hi + 0.1]
            tr = [(ti[k] - i_ms) / 1000 for k in k0]
            rv = [Ri[k] for k in k0]
        else:
            tr = rel("R")
            rv = list(w[1])
        ax1.plot(tr, rv, color="#4a5cd6", lw=1.0, alpha=0.8, label="Composite acceleration")
        specs = [("T", 2, "#7b2fbe", "Trunk"), ("C", 3, "#f0a020", "Chest"),
                 ("W", 4, "#2e9e3e", "Wrist"), ("I", 5, "#e0302a", "Impact")]
        curves = {}
        for key, col, color, label in specs:
            if key == "I" and imp is not None:
                x = tr
                y = [ei[k] for k in k0]
            else:
                x = rel(key)
                y = list(w[col])
            ax2.plot(x, y, color=color, lw=1.6, label=label)
            curves[key] = (x, y)
        rmax = max([v for x, v in zip(tr, rv) if lo <= x <= hi] or [35])
        ax1.set_ylim(0, max(35, rmax * 1.05))
        inwin = [abs(v) for key, (x, y) in curves.items() for xx, v in zip(x, y) if lo <= xx <= hi]
        m = max(inwin or [1]) * 1.15
        ax2.set_ylim(-m, m)
    else:
        curves = {}
    # events: vertical dashed line + dot on the curve + label at the top
    missing = []
    events = [("Tf", "T", "#7b2fbe", 0.97), ("C", "C", "#f0a020", 0.89), ("W", "W", "#2e9e3e", 0.81)]
    ytop = ax2.get_ylim()[1]
    for name, key, color, row in events:
        if fields.get(name) is None:
            missing.append(name)
            continue
        te = (int(fields[name]) - shift) / 1000
        ax2.axvline(te, color=color, ls="--", lw=1.2)
        if key in curves:
            x, y = curves[key]
            j = min(range(len(x)), key=lambda k: abs(x[k] - te))
            ax2.plot([te], [y[j]], "o", color=color, ms=6, mec="white", mew=0.8, zorder=5)
        ax2.text(te, ytop * row, f"{name} {te:+.3f} s ", color=color, fontsize=8,
                 ha="right", va="top",
                 bbox=dict(facecolor="white", edgecolor="none", alpha=0.8, pad=1))
    ax2.axvline(0, color="#e0302a", ls="--", lw=1.2)
    ax2.text(0, ytop * 0.97, " I 0 s", color="#e0302a", fontsize=8, ha="left", va="top",
             bbox=dict(facecolor="white", edgecolor="none", alpha=0.8, pad=1))
    if "I" in curves and len(curves["I"][0]):          # dot on the Impact peak (if it is in the data)
        x, y = curves["I"]
        j = min(range(len(x)), key=lambda k: abs(x[k]))
        if abs(x[j]) < 0.02:
            ax2.plot([0], [y[j]], "o", color="#e0302a", ms=6, mec="white", mew=0.8, zorder=5)
    ax1.set_xlim(lo, hi)
    ax1.set_xlabel("Time relative to the Impact peak (s)  (negative = before impact)"
                   + (f"   [board impact time {-shift:+.0f} ms]" if imp is not None else ""))
    ax1.set_ylabel("Composite acceleration (m/s²)", color="#4a5cd6")
    ax2.set_ylabel("Band signal (m/s²)")
    ax1.grid(True, alpha=0.3)
    nd = f"  (not detected: {', '.join(missing)})" if missing else ""
    dts = "   ".join(f"{k}={fields[k]} ms" for k in ("dtTfC", "dtCW", "dtTfW") if fields.get(k) is not None)
    ax1.set_title(f"Take {take} \u00b7 Shot {shot} \u00b7 Impact at 0 s{nd}   (board #{mcu_no})\n{dts}",
                  fontsize=11)
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8)
    fig.tight_layout()


class ShotWindow:
    """Live window showing the latest shot; every shot is also saved as a PNG."""
    def __init__(self, fig_dir, show=True):
        self.fig_dir = Path(fig_dir)
        self.open = False
        self.last_pause = 0.0
        self.plt = None
        try:
            import matplotlib
            if show:
                import matplotlib.pyplot as plt
                self.plt = plt
                plt.ion()
                self.fig = plt.figure(figsize=(10, 5.5))
                self.fig.canvas.mpl_connect("close_event", self._closed)
                self.fig.text(0.5, 0.5, "Waiting for the first shot...\nClose this window to stop recording.",
                              ha="center", va="center", fontsize=13)
                self.open = True
                plt.pause(0.001)
            else:
                from matplotlib.figure import Figure
                self.fig = Figure(figsize=(10, 5.5))
            self.ok = True
        except ImportError:
            print("[ INFO ] matplotlib not installed - no shot figures. Install it with:\n"
                  "         C:\\Python314\\python.exe -m pip install matplotlib")
            self.ok = False
        self.closed_by_user = False

    def _closed(self, _evt):
        self.closed_by_user = True
        self.open = False

    def add(self, wave, take, shot, fields, mcu_no, imp=None):
        if not self.ok:
            return
        draw_shot(self.fig, wave, take, shot, fields, mcu_no, imp)
        self.fig_dir.mkdir(exist_ok=True)
        self.fig.savefig(self.fig_dir / f"take{take:02d}_shot{shot:03d}.png", dpi=150)
        if self.open:
            self.fig.canvas.draw_idle()
            self.plt.pause(0.001)

    def keep_alive(self):
        if self.open and time.time() - self.last_pause > 0.2:
            self.last_pause = time.time()
            self.plt.pause(0.001)

    def close(self):
        if self.plt is not None:
            self.plt.ioff()
            self.plt.close("all")


def _print_line(s, label=None):
    """Readable terminal output for the board's status lines (R/w/ENV data stay in the log).
    label(board_no) -> "Take 3 - Shot 1" is called for SHOT lines."""
    now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    if s.startswith("SHOT;"):
        sh = _parse_shot(s)
        if sh is None:
            print(f"[{now}] [ WARN ] malformed SHOT line: {s}")
            return None
        n, v = sh
        na = lambda k: "NA" if v.get(k) is None else v[k]
        name = label(n) if label else f"SHOT #{n}"
        print(f"[{now}] {name}  Tf={na('Tf')}  C={na('C')}  W={na('W')}  "
              f"| dtTfC={na('dtTfC')}  dtCW={na('dtCW')}  dtTfW={na('dtTfW')}  "
              f"| peaks x10: I={na('Ipk')} T={na('Tpk')} C={na('Cpk')} W={na('Wpk')}")
        return sh
    msgs = {"BOOT_READY": "Board is up (always-on detection running).",
            "STREAM_START": "Streaming ON - swing when you want.",
            "STREAM_STOP": "Streaming OFF.",
            "WARMUP_DONE": "Warm-up finished - detection active.",
            "RESET": "Detectors reset (long press): blind for 6 s."}
    if s in msgs:
        print(f"[{now}] [ MCU ] {msgs[s]}")
    elif s.startswith("PERF;"):
        print(f"[{now}] [ PERF ] {s}\n           rate_hz ~400; tx_drops = raw data lost (link too slow);"
              f" tx_key_drops must be 0")
    elif s.startswith("ENV;"):
        print(f"[{now}] [ ENV ] {s[4:]}   (x10 m/s^2; thresholds x10: I=55 C=0 W=0 T=30)")
    return None


# --------------------------------------------------------------------------- record
def record(port, baud, out, show_plot=True):
    import serial  # pyserial

    if out:
        out = Path(out)
    else:
        day_dir = Path(__file__).resolve().parent / f"results_{datetime.now():%Y-%m-%d}"
        day_dir.mkdir(exist_ok=True)
        out = day_dir / f"session_{datetime.now():%H%M%S}.log"
    take = _takes_before(out)          # takes already recorded today: this session continues
    print(f"Recording {port} -> {out}   (Ctrl+C to stop)")
    if take:
        print(f"{take} take(s) already recorded today: the next one is take {take + 1}.")
    n_r = n_lines = 0
    t_status = time.time()
    pending = b""
    figs = ShotWindow(out.with_name(out.stem + "_shot_figs"), show_plot)
    wave = []                    # recent WAVE points (last ~6 s), only if the board still sends them
    raw_t, raw_r = [], []        # last ~8 s of raw |a| (m/s^2), for the shot figures
    last_r_ms = None
    n_shots = 0                  # shots in the current take
    in_take = False
    with serial.Serial(port, baud, timeout=0.2) as ser, \
         open(out, "w", encoding="ascii", errors="replace", newline="\n") as f:
        ser.reset_input_buffer()   # drop old data Windows kept from before (stale lines)
        if figs.open:
            print("Close the figure window (or Ctrl+C here) to stop recording.")
        try:
            while not figs.closed_by_user:
                chunk = ser.readline()
                figs.keep_alive()
                if not chunk:
                    continue
                pending += chunk
                if not pending.endswith(b"\n"):      # readline timed out mid-line
                    continue
                t_pc = time.time()
                s = pending.decode("ascii", errors="replace").rstrip("\r\n")
                pending = b""
                f.write(f"{t_pc:.4f}\t{s}\n")
                n_lines += 1
                if s.startswith("R"):
                    n_r += 1
                    r = _parse_r(s)
                    if r is not None:
                        last_r_ms = r[0]
                        for k, (x, y, z) in enumerate(r[2]):
                            raw_t.append(r[0] + k * DT_MS)
                            raw_r.append(math.isqrt(x * x + y * y + z * z) * ACC_CONV)
                        if len(raw_r) > 3600:
                            del raw_t[:400]; del raw_r[:400]
                elif s.startswith("w"):
                    wv = _parse_wave(s, last_r_ms)
                    if wv:
                        wave.append(wv)
                        if len(wave) > 400:
                            del wave[:100]
                else:
                    if s.startswith("STREAM_START") or (s.startswith("SHOT") and not in_take):
                        take += 1
                        n_shots = 0
                        in_take = True
                        print(f"--- Take {take} ---")
                    elif s.startswith("STREAM_STOP"):
                        in_take = False
                    sh = _print_line(s, lambda n: f"Take {take} \u00b7 Shot {n_shots + 1} (board #{n})")
                    if sh is not None:
                        n_shots += 1
                        figs.add(wave_from_raw(raw_t, raw_r) or wave, take, n_shots, sh[1], sh[0],
                                 impact_track(raw_t, raw_r))
                if t_pc - t_status >= 5.0:
                    rate = n_r * RAW_BLOCK / (t_pc - t_status)
                    if in_take and rate < 300:
                        print(f"  !!! raw {rate:5.0f} samples/s: Bluetooth link too weak, raw data is being"
                              f" lost - bring the laptop closer (shot results still get through)")
                    else:
                        print(f"  ... raw {rate:5.0f} samples/s (expect ~400 while streaming)")
                    n_r = 0
                    t_status = t_pc
                    f.flush()
        except KeyboardInterrupt:
            pass
    print(f"Stopped. {n_lines} lines saved in {out}")
    if n_shots and figs.ok:
        print(f"Shot figures saved in {figs.fig_dir.name}/")
    figs.close()
    if n_lines:
        print("Decoding...")
        decode(out)


# --------------------------------------------------------------------------- decode
def _hex12(s):
    v = int(s, 16)
    return v - 0x1000 if v & 0x800 else v


def _parse_r(s):
    """Returns (t0_ms, seq, [(x, y, z) * 8]) or None if the line is damaged."""
    if len(s) != R_LEN:
        return None
    try:
        t0 = int(s[1:9], 16)
        seq = int(s[9:11], 16)
        samples = []
        for k in range(RAW_BLOCK):
            o = 11 + 9 * k
            samples.append((_hex12(s[o:o + 3]) << 4,
                            _hex12(s[o + 3:o + 6]) << 4,
                            _hex12(s[o + 6:o + 9]) << 4))
    except ValueError:
        return None
    return t0, seq, samples


SHOT_KEYS = ["I"] + SHOT_FIELDS + PEAK_FIELDS


def _parse_shot(s):
    """'SHOT;n;I=..;Tf=..;...' -> (n, {field: "int as text" or None}), or None if unusable.
    On a weak Bluetooth link a SHOT line can arrive cut or glued to another line
    (e.g. 'Wpk=96w6C57...'): a value that is not a clean integer is treated as missing
    (None) instead of crashing, and the shot is kept only if its Impact time is intact."""
    parts = s.split(";")
    try:
        shot = int(parts[1])
    except (IndexError, ValueError):
        return None
    fields = {k: None for k in SHOT_KEYS}
    damaged = False
    # the board always sends the fields in this order: stop at the first token that is not
    # the next expected field (that is where another line got glued in)
    for k, p in zip(SHOT_KEYS, parts[2:]):
        if not p.startswith(k + "="):
            damaged = True
            break
        v = p[len(k) + 1:]
        if v == "NA":
            continue
        try:
            fields[k] = str(int(v))
        except ValueError:
            damaged = True
            break
    else:
        damaged = damaged or len(parts[2:]) < len(SHOT_KEYS)
    if fields["I"] is None:
        return None
    if damaged:
        fields["damaged"] = True       # incomplete line: missing values are NOT real NA
    return shot, fields


def _takes_before(log_path):
    """Number of takes recorded earlier the same day (sessions before this one in its folder),
    so take numbers continue through the day: session 1 -> take 1, next session -> take 2...
    Read from the earlier reports ("Take N:"); an earlier log not decoded yet is decoded first."""
    log_path = Path(log_path)
    last = 0
    for prev in sorted(log_path.parent.glob("session_*.log")):
        if prev.stem >= log_path.stem:
            continue
        rep = prev.with_name(prev.stem + "_report.txt")
        if not rep.exists() and prev.stat().st_size > 1000:
            print(f"(decoding earlier session {prev.name} first, to number the takes)")
            decode(prev)
        if rep.exists():
            for line in rep.read_text().splitlines():
                if line.startswith("Take ") and line.rstrip().endswith(":"):
                    try:
                        last = max(last, int(line[5:].rstrip(":")))
                    except ValueError:
                        pass
    return last


def decode(log_path, pre_s=4.0, post_s=1.5):
    log_path = Path(log_path)
    stem = log_path.with_suffix("")
    base = _takes_before(log_path)      # take numbers continue through the day

    takes = []          # each: dict(raw=[...], shots=[...], perf=[], bad=0, missing=0, seq_jumps=0)
    cur = None
    bad_other = 0

    def new_take():
        t = dict(raw=[], shots=[], perf=[], bad=0, missing=0, seq_jumps=0,
                 last_t0=None, last_seq=None, wave=[])
        takes.append(t)
        return t

    log_pc0 = None
    with open(log_path, encoding="ascii", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            if "\t" not in line:
                continue
            pc_s, s = line.split("\t", 1)
            try:
                pc = float(pc_s)
            except ValueError:
                pc = None
            if log_pc0 is None:
                log_pc0 = pc
            if s.startswith("STREAM_START"):
                cur = new_take()
                continue
            if s.startswith("STREAM_STOP"):
                continue                       # PERF follows; it belongs to this take
            if s.startswith("PERF"):
                (cur or new_take())["perf"].append(s)
                continue
            if cur is None:
                if s.startswith(("R", "SHOT")):  # logger started mid-stream
                    cur = new_take()
                else:
                    continue
            if s.startswith("w"):
                wv = _parse_wave(s, cur["last_t0"])
                if wv:
                    cur["wave"].append(wv)
                continue
            if s.startswith("R"):
                r = _parse_r(s)
                if r is None:
                    cur["bad"] += 1
                    continue
                t0, seq, samples = r
                if cur["last_t0"] is not None and abs(t0 - cur["last_t0"]) > 2000:
                    # MCU clock jump > 2 s. In the first seconds after the logger connects it is
                    # old data Windows had buffered -> a separate take (dropped later if tiny).
                    # Later on it is a weak link that lost lines -> same take, samples lost.
                    if t0 < cur["last_t0"] or (pc and log_pc0 and pc - log_pc0 < 3.0):
                        cur = new_take()
                if cur["last_t0"] is not None:
                    gap = t0 - cur["last_t0"]
                    lost = round(gap / BLOCK_MS) - 1
                    if lost > 0:
                        cur["missing"] += lost
                    if seq != (cur["last_seq"] + 1) & 0xFF:
                        cur["seq_jumps"] += 1
                cur["last_t0"], cur["last_seq"], cur["last_pc"] = t0, seq, pc
                for k, (x, y, z) in enumerate(samples):
                    cur["raw"].append((t0 + k * DT_MS, seq, x, y, z))
            elif s.startswith("SHOT"):
                sh = _parse_shot(s)
                if sh is None:
                    cur["bad"] += 1
                else:
                    cur["shots"].append(sh)

    # drop empty takes and the < 2 s fragments of old buffered data seen at connection
    takes = [t for t in takes if t["shots"] or len(t["raw"]) >= 2 * FS]
    if not takes:
        print("No R or SHOT lines found. Was streaming switched on (short press)?")
        return
    for k, t in enumerate(takes, base + 1):
        t["no"] = k

    # ---- raw CSV + one .bin per take
    raw_csv = Path(f"{stem}_raw.csv")
    with open(raw_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["take", "t_ms", "seq", "x", "y", "z", "ax", "ay", "az", "a_norm", "a_norm_mcu"])
        for t in takes:
            ti = t["no"]
            for (tm, seq, x, y, z) in t["raw"]:
                ss = x * x + y * y + z * z
                w.writerow([ti, f"{tm:.1f}", seq, x, y, z,
                            f"{x * ACC_CONV:.4f}", f"{y * ACC_CONV:.4f}", f"{z * ACC_CONV:.4f}",
                            f"{math.sqrt(ss) * ACC_CONV:.4f}", f"{math.isqrt(ss) * ACC_CONV:.4f}"])
    bins = []
    for t in takes:
        ti = t["no"]
        if not t["raw"]:
            continue
        p = Path(f"{stem}_take{ti:02d}_raw.bin")
        with open(p, "wb") as f:
            for (_, _, x, y, z) in t["raw"]:
                f.write(struct.pack("<hhh", x, y, z))
        bins.append(p)
        # Shimmer-style CSV for the Python reference (read_v70: header rows 1-2 = names/units,
        # first column = time; the reference converts ms to s and starts it at 0).
        p = Path(f"{stem}_take{ti:02d}_shimmer.csv")
        with open(p, "w", newline="") as f:
            f.write("sep=,\n")
            f.write("Timestamp_MCU,Accel_X_CAL,Accel_Y_CAL,Accel_Z_CAL\n")
            f.write("ms,m/(s^2),m/(s^2),m/(s^2)\n")
            for (tm, _, x, y, z) in t["raw"]:
                f.write(f"{tm:.1f},{x * ACC_CONV:.5f},{y * ACC_CONV:.5f},{z * ACC_CONV:.5f}\n")
        bins.append(p)

    # ---- shots CSV
    shots_csv = Path(f"{stem}_shots.csv")
    shot_dir = Path(f"{stem}_shots")
    shot_dir.mkdir(exist_ok=True)
    for old in shot_dir.glob("take*_shot*.csv"):      # files of an earlier decode of this log
        old.unlink()
    n_shot_files = 0
    with open(shots_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["take", "shot", "board_shot_no", "file", "I_ms", "I_from_take_start_s",
                    "I_sample"] + SHOT_FIELDS + PEAK_FIELDS + ["window_note"])
        for t in takes:
            ti = t["no"]
            times = [r[0] for r in t["raw"]]
            for shot, (mcu_no, fl) in enumerate(t["shots"], 1):
                i_ms = int(fl["I"])
                idx = ""
                if times and times[0] <= i_ms <= times[-1]:
                    j = bisect.bisect_left(times, i_ms)
                    if j > 0 and (j == len(times) or i_ms - times[j - 1] < times[j] - i_ms):
                        j -= 1
                    idx = j
                t_from_start = f"{(i_ms - times[0]) / 1000:.4f}" if times else ""
                fname, note = _write_shot_window(shot_dir, ti, shot, t["raw"], times, i_ms,
                                                 pre_s, post_s)
                n_shot_files += bool(fname)
                row = [ti, shot, mcu_no, fname, i_ms, t_from_start, idx]
                row += ["" if fl.get(k) is None else fl[k] for k in SHOT_FIELDS]
                row += ["" if fl.get(k) is None else f"{int(fl[k]) / 10:.1f}" for k in PEAK_FIELDS]
                if fl.get("damaged"):
                    note = ("SHOT line damaged in transmission (some values lost); " + note).rstrip("; ")
                row.append(note)
                w.writerow(row)

    # ---- per-shot figures (same as the live window), from the raw data (or the old WAVE lines)
    n_figs = 0
    try:
        from matplotlib.figure import Figure
        fig = Figure(figsize=(10, 5.5))
        fig_dir = Path(f"{stem}_shot_figs")
        if fig_dir.exists():                         # replace the live / earlier figures
            for old in fig_dir.glob("*.png"):
                old.unlink()
        for t in takes:
            raw_tt = [r[0] for r in t["raw"]]
            raw_rr = [math.isqrt(r[2] ** 2 + r[3] ** 2 + r[4] ** 2) * ACC_CONV for r in t["raw"]]
            rw = wave_from_raw(raw_tt, raw_rr)
            imp = impact_track(raw_tt, raw_rr)
            if rw:
                t["wave"] = rw
            wt = [w[0] for w in t["wave"]]
            for k, (mcu_no, fl) in enumerate(t["shots"], 1):
                i_ms = int(fl["I"])
                a = bisect.bisect_left(wt, i_ms - 2500)
                b = bisect.bisect_right(wt, i_ms + 1500)
                if b - a < 5:
                    continue
                fl_int = {kk: (None if v is None else v) for kk, v in fl.items()}
                draw_shot(fig, t["wave"][a:b], t["no"], k, fl_int, mcu_no, imp)
                fig_dir.mkdir(exist_ok=True)
                fig.savefig(fig_dir / f"take{t['no']:02d}_shot{k:03d}.png", dpi=150)
                n_figs += 1
    except ImportError:
        pass

    # ---- report
    lines = [f"Decoded {log_path.name}", ""]
    for t in takes:
        ti = t["no"]
        n = len(t["raw"])
        dur = (t["raw"][-1][0] - t["raw"][0][0]) / 1000 if n > 1 else 0
        rate = (n - 1) / dur if dur > 0 else 0
        exp_blocks = n // RAW_BLOCK + t["missing"]
        lines += [f"Take {ti}:",
                  f"  raw samples   : {n}  over {dur:.1f} s  -> {rate:.1f} Hz (expect 400)",
                  f"  missing blocks: {t['missing']} of {exp_blocks}"
                  f"  ({t['missing'] * RAW_BLOCK} samples lost)",
                  f"  damaged lines : {t['bad']}   seq jumps: {t['seq_jumps']}",
                  f"  shots         : {len(t['shots'])}   (use K = {len(t['shots'])} in the Python reference)"]
        for p in t["perf"]:
            lines.append(f"  {p}")
        if t["missing"] or t["bad"]:
            lines.append("  WARNING: gaps in this take. The .bin is not a continuous 400 Hz signal;"
                         " use t_ms from the CSV (or skip shots near a gap).")
        lines.append("")
    lines += [f"Wrote {raw_csv.name}, {shots_csv.name}"] + [f"Wrote {b.name}" for b in bins]
    if n_figs:
        lines.append(f"Wrote {n_figs} shot figures in {stem.name}_shot_figs/")
    lines.append(f"Wrote {n_shot_files} per-shot files in {shot_dir.name}/"
                 f"  (window -{pre_s:g} s to +{post_s:g} s around each MCU impact)")
    report = "\n".join(lines)
    Path(f"{stem}_report.txt").write_text(report + "\n")
    print(report)


def _write_shot_window(shot_dir, take, shot, raw, times, i_ms, pre_s, post_s):
    """Writes the raw samples around one MCU impact. Returns (file name, note)."""
    if not times:
        return "", "no raw data in this take"
    t0, t1 = i_ms - pre_s * 1000, i_ms + post_s * 1000
    a = bisect.bisect_left(times, t0)
    b = bisect.bisect_right(times, t1)
    if b - a < 10:
        return "", "impact outside the raw data"
    notes = []
    if times[a] > t0 + DT_MS:
        notes.append(f"starts only {(i_ms - times[a]) / 1000:.2f} s before impact")
    if times[b - 1] < t1 - DT_MS:
        notes.append(f"ends {(times[b - 1] - i_ms) / 1000:.2f} s after impact")
    gaps = sum(1 for k in range(a + 1, b) if times[k] - times[k - 1] > DT_MS + BLOCK_MS / 2)  # a whole block missing, not ms jitter
    if gaps:
        notes.append(f"{gaps} gap(s) in the window")
    name = f"take{take:02d}_shot{shot:03d}.csv"
    with open(shot_dir / name, "w", newline="") as f:
        f.write("sep=,\n")
        f.write("Timestamp_MCU,Accel_X_CAL,Accel_Y_CAL,Accel_Z_CAL,t_rel_ms\n")
        f.write("ms,m/(s^2),m/(s^2),m/(s^2),ms\n")
        for (tm, _, x, y, z) in raw[a:b]:
            f.write(f"{tm:.1f},{x * ACC_CONV:.5f},{y * ACC_CONV:.5f},{z * ACC_CONV:.5f},"
                    f"{tm - i_ms:.1f}\n")
    return f"{shot_dir.name}/{name}", "; ".join(notes)


# --------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    r = sub.add_parser("record", help="save everything the board sends")
    r.add_argument("--port", default=DEFAULT_PORT)
    r.add_argument("--baud", type=int, default=115200)
    r.add_argument("--out")
    r.add_argument("--no-plot", action="store_true", help="no live graph window")
    d = sub.add_parser("decode", help="split a .log into raw / shots files")
    d.add_argument("log")
    d.add_argument("--pre", type=float, default=4.0, help="seconds kept before each impact")
    d.add_argument("--post", type=float, default=1.5, help="seconds kept after each impact")
    a = ap.parse_args()
    if a.cmd is None:                       # no arguments (Run button): record on DEFAULT_PORT
        a = ap.parse_args(["record"])
    if a.cmd == "record":
        record(a.port, a.baud, a.out, not a.no_plot)
    else:
        decode(a.log, a.pre, a.post)


if __name__ == "__main__":
    sys.exit(main())