#!/usr/bin/env python3
"""
pcb_analysis.py - run the Python Δt analysis (5bands_dt V2806) on the PCB raw data Excel
made by export_raw.py ("PCB Raw Data <date>.xlsx"), and draw for every shot the
"Waveform wide" graph: impact at 0 s, from -2.0 to +0.5 s.

Run it (Run button in VS Code, or  C:\\Python314\\python.exe pcb_analysis.py ).
It asks which Excel file, which takes (sheets), and how to find the impacts:
  1 = like your Python (5bands_dt V2806): the K largest peaks of the Impact envelope
  2 = like the board (MCU): the board's Impact detector run on the raw data
  3 = both (two sets of results, to compare)

Calculation, same as 5bands_dt_shimmer_V2806 with the PCB data:
  - R = sqrt(X² + Y² + Z²)  (m/s²), resampled on an even 400 Hz grid (gaps filled linearly)
  - 3.5 g limit on R for Trunk / Chest / Wrist only, like the board (MCU); Impact uses the full R
  - 4th-order Butterworth band-pass, zero phase (filtfilt):
        Trunk 1-3 Hz, Chest 2.5-7 Hz, Wrist 8-15 Hz, Impact 20-40 Hz
  - Chest, Wrist, Impact: Hilbert envelope; Trunk: the filtered wave itself
  - Impact: see the 3 choices above; the test shots at the start of the take
    (number in the Info sheet) are left out
  - Tf = last negative valley of Trunk in -1.2..-0.2 s
    C  = largest Chest envelope peak in -0.4..-0.05 s
    W  = last Wrist envelope peak in -0.2..0 s
Output, next to the Excel, in "Calculated Data":
  - Graph data <date>\\<take>\\R_waveform_wide_NN.png   (one per real shot)
  - delta t Table\\dt <take> <date>.xlsx                 (event times and the six Δt, in s)
  With choice 3 the folders and files end with "(Python)" / "(MCU)".
Needs: numpy, scipy, matplotlib, openpyxl, pandas
"""
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, hilbert, find_peaks, sosfilt

G = 9.80665
CAP_G = 3.5                       # same limit as the board (SLOW_CAP)
FS = 400.0
GRID = np.linspace(-3.0, 0.4, 2381)
PLOT_START_S = -2.0              # graphs start at -2 s (the calculation still uses the -3..+0.4 s window)
HERE = Path(__file__).resolve().parent
if not any(HERE.glob("results_*")) and any(HERE.parent.glob("results_*")):
    HERE = HERE.parent

TITLE_FONTSIZE = 15
LABEL_FONTSIZE = 12
TICK_FONTSIZE = 12
LEGEND_FONTSIZE = 10


# ---------------------------------------------------------------- functions from V2806 (unchanged)
def apply_axis_style(ax, title_size=TITLE_FONTSIZE, label_size=LABEL_FONTSIZE, tick_size=TICK_FONTSIZE):
    """軸目盛り数字・軸ラベル・タイトルを少し大きめにそろえる。"""
    try:
        ax.tick_params(axis="both", labelsize=tick_size)
    except Exception:
        pass
    try:
        ax.xaxis.label.set_size(label_size)
        ax.yaxis.label.set_size(label_size)
    except Exception:
        pass
    try:
        ax.title.set_fontsize(title_size)
    except Exception:
        pass


def apply_legend_style(legend, font_size=LEGEND_FONTSIZE):
    if legend is None:
        return
    try:
        for txt in legend.get_texts():
            txt.set_fontsize(font_size)
    except Exception:
        pass


def bandpass_butter(data, fs, f_low, f_high, order=4):
    data = np.asarray(data, dtype=float)
    nyq = 0.5 * fs
    f_low_clipped = max(0.001, min(f_low, nyq * 0.99))
    f_high_clipped = max(f_low_clipped * 1.01, min(f_high, nyq * 0.99))
    if len(data) < max(30, 3 * (2 * order + 1)):
        raise SystemExit(f"データ長が短すぎてフィルタできません。len={len(data)}")
    b, a = butter(order, [f_low_clipped / nyq, f_high_clipped / nyq], btype="band")
    try:
        return filtfilt(b, a, data)
    except Exception as e:
        raise SystemExit(f"フィルタ処理で停止しました: {e}")


def compute_envelope(grid, y_band):
    y = np.asarray(y_band, dtype=float)
    if np.all(~np.isfinite(y)):
        return np.full_like(y, np.nan)
    idx_valid = np.where(np.isfinite(y))[0]
    if len(idx_valid) < 3:
        return np.full_like(y, np.nan)
    y_filled = y.copy()
    y_filled[:idx_valid[0]] = y[idx_valid[0]]
    y_filled[idx_valid[-1] + 1:] = y[idx_valid[-1]]
    analytic = hilbert(y_filled)
    env = np.abs(analytic)
    return env


def compute_last_negative_valley_time(grid, y, t_min, t_max, prom_ratio=0.10):
    """Filtered波形の負側の谷を検出する。

    V2804:
      - 対象は Trunk filtered 1.0–3.0 Hz の波形そのもの
      - 探索範囲は -1.2〜-0.2 s
      - find_peaks(-y) により局所最小を検出
      - そのうち y<0 の負側の谷だけを候補とする
      - 候補が複数ある場合は、インパクトに最も近い最後の谷を Tf とする
    """
    t = np.asarray(grid, dtype=float)
    yy = np.asarray(y, dtype=float)
    m = (t >= float(t_min)) & (t <= float(t_max)) & np.isfinite(yy)
    tt = t[m]
    vv = yy[m]
    if len(tt) < 8:
        return float("nan"), False

    inv = -vv
    pr = float(prom_ratio) * (float(np.nanmax(inv)) - float(np.nanmin(inv)) + 1e-12)
    valleys, _ = find_peaks(inv, prominence=pr)
    if len(valleys) == 0:
        return float("nan"), False

    # 負側の谷だけを Tf 候補にする。
    neg_valleys = [int(i) for i in valleys if np.isfinite(vv[int(i)]) and vv[int(i)] < 0.0]
    if len(neg_valleys) == 0:
        return float("nan"), False

    iv = int(neg_valleys[-1])
    return float(tt[iv]), True


def compute_peak_time(grid, env, t_min, t_max, mode="max"):
    t = np.asarray(grid, dtype=float)
    e = np.asarray(env, dtype=float)
    m = (t >= float(t_min)) & (t <= float(t_max)) & np.isfinite(e)
    if not np.any(m):
        return float("nan"), False
    tt = t[m]
    ee = e[m]

    loc = []
    for j in range(1, len(ee) - 1):
        if (
            np.isfinite(ee[j - 1])
            and np.isfinite(ee[j])
            and np.isfinite(ee[j + 1])
            and (ee[j - 1] < ee[j])
            and (ee[j] >= ee[j + 1])
        ):
            loc.append(j)

    if len(loc) == 0:
        return float("nan"), False

    loc_arr = np.array(loc, dtype=int)
    if mode == "last":
        ip = int(loc_arr[-1])
    else:
        ip = int(loc_arr[np.argmax(ee[loc_arr])])
    return float(tt[ip]), True


def _plot_feature_marker(ax, grid, y_arr, tpk, color, key, kind, text_y, marker=None, linestyle=None):
    """整列時間グラフ上に検出時刻の縦線・マーカーを描く。"""
    if not np.isfinite(tpk):
        return
    y_arr = np.asarray(y_arr, dtype=float)
    if linestyle is None:
        vls = ":" if key == "TrunkFiltered" else "--"
    else:
        vls = linestyle
    ax.axvline(tpk, color=color, ls=vls, lw=2.0, alpha=0.95, zorder=6)
    idx = int(np.nanargmin(np.abs(np.asarray(grid, dtype=float) - float(tpk))))
    y_marker = y_arr[idx] if 0 <= idx < len(y_arr) else np.nan
    if np.isfinite(y_marker):
        if marker is None:
            marker = "x" if key == "TrunkFiltered" else "o"
        if marker == "o":
            ax.plot([tpk], [y_marker], marker=marker, ms=7, color=color,
                    mec="white", mew=1.0, zorder=8, label="_nolegend_")
        else:
            ax.plot([tpk], [y_marker], marker=marker, ms=8, color=color,
                    mew=2.0, zorder=8, label="_nolegend_")
    # V2804_9/V2806_0: 縦線に重ねる注釈文字は表示しない。


def plot_waveform_wide(grid, raw_R, impact_env, trunk_filtered, chest_env, wrist_env, t_feature,
                       impact_idx, impact_time_abs_s, out_png):
    """最終版のwide図を作成する。

    V2806_0:
      - 横軸は Impact基準 -3.0〜+0.5 s
      - 生加速度R（青）を左軸に表示する
      - Trunk filtered（紫, Hilbertなし）, Chest（オレンジ）, Wrist（緑）, Impact（赤）を右軸に表示する
      - Trunkイベントは t_trunk_filtered_valley_rel_s（Tf）を用いる
    """
    fig, ax1 = plt.subplots(figsize=(10, 5.5))
    raw_R = np.asarray(raw_R, dtype=float)
    ax1.plot(grid, raw_R, color="blue", lw=1.0, alpha=0.85, label="Composite acceleration")
    ax1.set_xlim(PLOT_START_S, 0.5)                  # display only: the events are still searched as before
    ax1.set_ylim(0.0, 35.0)
    ax1.set_xlabel("Time aligned to Impact peak (s)")
    ax1.set_ylabel("Composite acceleration (g)", color="blue")
    ax1.tick_params(axis='y', labelcolor="blue")
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    plot_specs = [
        ("Impact", "Impact", np.asarray(impact_env, dtype=float), "red", "peak", 9.2),
        ("TrunkFiltered", "Trunk", np.asarray(trunk_filtered, dtype=float), "purple", "valley", 7.8),
        ("Chest", "Chest", np.asarray(chest_env, dtype=float), "orange", "peak", 6.4),
        ("Wrist", "Wrist", np.asarray(wrist_env, dtype=float), "green", "peak", 5.0),
    ]
    for key, label, y_arr, color, kind, text_y in plot_specs:
        ax2.plot(grid, y_arr, color=color, lw=1.8 if key != "TrunkFiltered" else 1.6,
                 ls="-", alpha=0.95, label=label)
        marker_use = "o"
        linestyle_use = "--"
        _plot_feature_marker(
            ax2, grid, y_arr, float(t_feature.get(key, float("nan"))),
            color, key, kind, text_y, marker=marker_use, linestyle=linestyle_use
        )

    ax1.axvline(0.0, color="k", ls="--", lw=1.2, zorder=5)
    # show the whole Impact peak (it can go above 10 on the PCB, which has no saturation)
    top = max(10.0, 1.1 * float(np.nanmax(impact_env)))
    ax2.set_ylim(-top, top)
    ax2.set_ylabel("Amplitude(g)")
    title_time_abs = f"absolute impact time = {impact_time_abs_s:.3f} s" if np.isfinite(impact_time_abs_s) else "absolute impact time = N/A"
    ax1.set_title(f"Waveform wide (impact {impact_idx:02d}, {title_time_abs})")
    apply_axis_style(ax1)
    apply_axis_style(ax2)
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    lg = ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left")
    apply_legend_style(lg)
    fig.subplots_adjust(left=0.10, right=0.90, bottom=0.14, top=0.90)
    plt.savefig(out_png, dpi=200)
    plt.close(fig)


# ---------------------------------------------------------------- reading the Excel
def read_take(xlsx, sheet):
    from openpyxl import load_workbook
    wb = load_workbook(xlsx, read_only=True, data_only=True)
    ws = wb[sheet]
    t, acc, shots = [], [], []
    for i, row in enumerate(ws.iter_rows(values_only=True)):
        if i < 2:
            continue                                   # header + units
        if row[0] is not None:
            t.append(float(row[0]) / 1000.0)
            acc.append((float(row[1]), float(row[2]), float(row[3])))
        if len(row) > 7 and row[5] is not None and row[7] is not None:
            shots.append((row[5], float(row[7]) / 1000.0))   # (Shot label, impact Unix s)
    wb.close()
    return np.array(t), np.array(acc), shots


def ask(prompt, default=""):
    a = input(prompt).strip()
    return a if a else default


def choose_excel():
    if len(sys.argv) > 1:
        return Path(sys.argv[1])
    files = sorted(HERE.glob("results_*/PCB Raw Data *.xlsx"))
    if not files:
        sys.exit("No 'PCB Raw Data <date>.xlsx' found. Make it first with export_raw.py.")
    print("PCB raw data files:")
    for i, f in enumerate(files, 1):
        print(f"  {i}. {f.name}")
    a = ask(f"Which file? (number, Enter = {files[-1].name}): ")
    return files[int(a) - 1] if a else files[-1]


# ---------------------------------------------------------------- finding the impacts
def impacts_python(imp_env, K):
    """5bands_dt V2806: peaks of the Impact envelope at least 0.30 s apart, the K largest,
    then put back in time order."""
    pk, _ = find_peaks(imp_env, distance=max(1, int(0.30 * FS)))
    if pk.size == 0:
        return np.array([], int)
    top = pk[np.argsort(imp_env[pk])[::-1][:K]]
    return np.sort(top)


def impacts_mcu(R):
    """The board's Impact detector (main_live_raw.c), sample by sample:
    20-40 Hz Butterworth (order 4, causal), |y| smoothed (alpha 0.8546 = 10 Hz),
    pulse above 3.5 m/s², peak confirmed when it falls below 70 %, 300 ms refractory
    (a bigger pulse may start earlier), several pulses within 1 s = one shot (largest kept),
    impact time = peak time - 55 ms (filter delay)."""
    sos = butter(4, [20.0, 40.0], btype="band", fs=FS, output="sos")
    y = sosfilt(sos, R)
    alpha, thr, fall, refr, hold, delay = 0.8546359991532334, 3.5, 0.70, 0.300, 1.0, 0.055
    env = 0.0
    in_pulse = False
    peak_v = peak_i = 0
    last_ev = -1e9
    last_val = 0.0
    shots = []                     # (index, peak value)
    pend = None                    # [index, value, deadline]
    for i, v in enumerate(np.abs(y)):
        t = i / FS
        env = alpha * env + (1 - alpha) * v
        if not in_pulse:
            if env > thr and (t - last_ev > refr or env > last_val):
                in_pulse, peak_v, peak_i = True, env, i
        elif env > peak_v:
            peak_v, peak_i = env, i
        elif env < peak_v * fall:
            in_pulse = False
            last_ev, last_val = peak_i / FS, peak_v
            if pend is None or peak_v > pend[1]:
                pend = [peak_i, peak_v, t + hold]
        if pend is not None and t >= pend[2]:
            shots.append(pend[0])
            pend = None
    if pend is not None:
        shots.append(pend[0])
    return np.array([max(0, k - int(round(delay * FS))) for k in shots], int)


# ---------------------------------------------------------------- analysis of one take
def analyse(xlsx, sheet, n_shots, n_test, method, graph_dir, table_file):
    t, acc, board_shots = read_take(xlsx, sheet)
    t0 = t[0]
    tu = np.arange(t[0], t[-1], 1.0 / FS)              # even 400 Hz grid (Bluetooth gaps filled)
    R = np.interp(tu, t, np.linalg.norm(acc, axis=1))
    Rs = np.minimum(R, CAP_G * G)                     # 3.5 g limit, slow bands only (like the MCU)
    trunk = bandpass_butter(Rs, FS, 1.0, 3.0, 4)
    chest = bandpass_butter(Rs, FS, 2.5, 7.0, 4)
    wrist = bandpass_butter(Rs, FS, 8.0, 15.0, 4)
    impact = bandpass_butter(R, FS, 20.0, 40.0, 4)
    imp_env_full = np.abs(hilbert(impact))

    # shots table of the sheet (export_raw.py): only the shots inside the recorded data count
    board = [(lab, iu) for lab, iu in board_shots if tu[0] <= iu <= tu[-1]]
    if board:
        n_shots = len(board)
        n_test = sum(1 for lab, _ in board if str(lab).strip().lower() == "test")
    if method == "Python":
        ks = impacts_python(imp_env_full, n_shots)
    else:
        ks = impacts_mcu(R)
    found = len(ks)
    # leave out the test shots: impacts within 0.3 s of a shot marked "Test" in the sheet
    # (if there is no shots table, the first n_test impacts)
    tests = [iu for lab, iu in board if str(lab).strip().lower() == "test"]
    if board:
        ks = np.array([k for k in ks if all(abs(tu[k] - iu) > 0.3 for iu in tests)], int)
    else:
        ks = ks[n_test:]
    n_expected = n_shots - n_test

    def window(k, arr):
        s = max(0, int(k - 3.0 * FS))
        e = min(len(tu), int(k + 0.4 * FS))
        return np.interp(GRID, tu[s:e] - tu[k], arr[s:e], left=np.nan, right=np.nan)

    graph_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for shot_no, k in enumerate(ks, 1):
        tr = window(k, trunk)
        ch = compute_envelope(GRID, window(k, chest))
        wr = compute_envelope(GRID, window(k, wrist))
        im = compute_envelope(GRID, window(k, impact))
        tf, ok_tf = compute_last_negative_valley_time(GRID, tr, -1.2, -0.2, 0.10)
        c, ok_c = compute_peak_time(GRID, ch, -0.4, -0.05, "max")
        w, ok_w = compute_peak_time(GRID, wr, -0.2, 0.0, "last")
        tf = tf if ok_tf else np.nan
        c = c if ok_c else np.nan
        w = w if ok_w else np.nan
        t_abs = tu[k] - t0
        plot_waveform_wide(GRID, window(k, R), im, tr, ch, wr,
                           {"Impact": 0.0, "TrunkFiltered": tf, "Chest": c, "Wrist": w},
                           shot_no, t_abs, str(graph_dir / f"R_waveform_wide_{shot_no:02d}.png"))
        rows.append({"shot_no": shot_no, "impact_time_from_start_s": t_abs,
                     "t_trunk_filtered_valley_rel_s": tf, "t_chest_peak_rel_s": c, "t_wrist_peak_rel_s": w,
                     "dt_TfC_s": c - tf, "dt_CW_s": w - c, "dt_WI_s": 0.0 - w,
                     "dt_TfW_s": w - tf, "dt_CI_s": 0.0 - c, "dt_TfI_s": 0.0 - tf})
    df = pd.DataFrame(rows)
    if len(df):
        num = df.drop(columns=["shot_no"])
        df = pd.concat([df, pd.DataFrame([dict(shot_no="mean", **num.mean()),
                                          dict(shot_no="SD", **num.std())])], ignore_index=True)
    table_file.parent.mkdir(parents=True, exist_ok=True)
    try:
        df.round(4).to_excel(table_file, index=False)
    except PermissionError:
        print(f"\n  ! {table_file.name} is open in Excel: close it and run again")
    return found, len(rows), n_expected, [tu[k] for k in ks]


def read_info(xlsx):
    """{sheet: (shots detected, test shots)} from the Info sheet written by export_raw.py."""
    from openpyxl import load_workbook
    wb = load_workbook(xlsx, read_only=True)
    info = {}
    if "Info" in wb.sheetnames:
        for row in wb["Info"].iter_rows(values_only=True):
            if row and row[0] in wb.sheetnames and row[0] != "Info":
                try:
                    info[row[0]] = (int(row[2]), int(row[3]))
                except (TypeError, ValueError, IndexError):
                    pass
    sheets = [s for s in wb.sheetnames if s != "Info"]
    wb.close()
    return sheets, info


def main():
    try:
        import openpyxl  # noqa: F401
    except ImportError:
        sys.exit("openpyxl is missing:  C:\\Python314\\python.exe -m pip install openpyxl")
    matplotlib.use("Agg")
    xlsx = choose_excel()
    m = re.search(r"(\d{4}-\d{2}-\d{2})", xlsx.name)
    date = m.group(1) if m else ""
    sheets, info = read_info(xlsx)
    print(f"\nTakes in {xlsx.name}:")
    for i, s in enumerate(sheets, 1):
        n, nt = info.get(s, ("?", "?"))
        print(f"  {i}. {s}   ({n} shots, {nt} test)")
    a = ask("Which takes? (numbers separated by spaces, Enter = all): ")
    chosen = [sheets[int(x) - 1] for x in re.split(r"[ ,;]+", a) if x] if a else sheets

    print("\nHow to find the impacts?")
    print("  1 = like your Python (K largest peaks of the Impact envelope)")
    print("  2 = like the board (MCU Impact detector)")
    print("  3 = both")
    c = ask("Choice (Enter = 1): ", "1")
    methods = {"1": ["Python"], "2": ["MCU"], "3": ["Python", "MCU"]}.get(c, ["Python"])

    base = xlsx.parent / "Calculated Data"
    for method in methods:
        suffix = f" ({method})" if len(methods) > 1 else ""
        gdir = base / f"Graph data {date}{suffix}".strip()
        tdir = base / f"delta t Table{suffix}"
        print(f"\nImpacts found like {'your Python' if method == 'Python' else 'the board'}:")
        for s in chosen:
            n_shots, n_test = info.get(s, (None, 0))
            if n_shots is None:
                n_shots = int(ask(f"  How many shots in '{s}' (tests included)? "))
            safe = re.sub(r'[<>:"/\\|?*]', "-", s)
            print(f"  {s} ...", end="", flush=True)
            found, done, n_shots, _ = analyse(xlsx, s, n_shots, n_test, method, gdir / safe,
                                  tdir / f"dt {safe} {date}{suffix}.xlsx".replace("  ", " "))
            note = "" if done == n_shots else f"   (! {n_shots} real shots expected: check the graphs)"
            print(f" {done} shots{note}")
    print(f"\nDone: {base}")


if __name__ == "__main__":
    main()
