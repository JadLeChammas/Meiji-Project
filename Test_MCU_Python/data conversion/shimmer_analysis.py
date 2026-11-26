#!/usr/bin/env python3
"""
shimmer_analysis.py - the same graphs and Δt tables as pcb_analysis.py, for the Shimmer.

Run it (Run button in VS Code, or  C:\\Python314\\python.exe shimmer_analysis.py ).
It asks:
  1. the folder with the Shimmer CSV files (Consensys export, "..._Calibrated_PC.csv");
     you can drag the folder into the terminal,
  2. which sessions to analyse,
  3. for each session: a name, and the PCB take recorded at the same time (a sheet of
     "PCB Raw Data <date>.xlsx"). The impacts are then taken from the PCB shots (test shots
     left out), lined up on the Shimmer by matching the two signals, and put on the Shimmer's
     own Impact peak. Without a PCB take, you give the number of shots and test shots and
     the program takes the K largest Impact peaks (less reliable: the Shimmer saturates).

Calculation, same as pcb_analysis.py / 5bands_dt V2806:
  - R = sqrt(X² + Y² + Z²) from the Wide Range accelerometer (m/s²), resampled to 400 Hz
  - no 3.5 g limit: the Shimmer is already saturated at about 3.5 g
  - Trunk 1-3 Hz, Chest 2.5-7 Hz, Wrist 8-15 Hz, Impact 20-40 Hz (Butterworth order 4, zero phase)
  - impacts: from the matching PCB take (recommended), or the K largest Impact peaks
  - Tf = last Trunk valley in -1.2..-0.2 s, C = largest Chest peak in -0.4..-0.05 s,
    W = last Wrist peak in -0.2..0 s
Output, in "Calculated Data" (in results_<date> next to live_monitor.py if that folder exists,
otherwise next to the Shimmer files):
  - Graph data Shimmer <date>\\<session name>\\R_waveform_wide_NN.png
  - delta t Table Shimmer\\dt <session name> <date>.xlsx
Needs: numpy, scipy, matplotlib, pandas, openpyxl, and pcb_analysis.py in the same folder.
"""
import re
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pcb_analysis as P          # same filters, event rules and graph as for the PCB  # noqa: E402

FS = 400.0
G = 9.80665


def read_shimmer(csv_path):
    """(t in s, R in m/s²) from a Consensys CSV: line 1 'sep=', line 2 names, line 3 units."""
    with open(csv_path, encoding="utf-8", errors="replace") as f:
        first = f.readline()
        names = f.readline().rstrip("\n").split("\t")
    skip = 3 if first.lower().startswith(('"sep=', "sep=")) else 2
    if skip == 2:                                     # no "sep=" line: the names were line 1
        names = first.rstrip("\n").split("\t")

    def col(*keys):
        for i, n in enumerate(names):
            if all(k.lower() in n.lower() for k in keys):
                return i
        return None
    it = col("Timestamp", "Unix") if col("Timestamp", "Unix") is not None else col("Timestamp")
    axes = [col("Accel_WR_" + a, "CAL") for a in "XYZ"]
    if None in axes:                                  # no Wide Range accelerometer: Low Noise
        axes = [col("Accel_LN_" + a, "CAL") for a in "XYZ"]
    if it is None or None in axes:
        raise ValueError("timestamp or accelerometer columns not found")
    d = pd.read_csv(csv_path, sep="\t", skiprows=skip, header=None,
                    usecols=[it] + axes, engine="c").dropna()
    d = d[[it] + axes].to_numpy(float)
    t = d[:, 0] / 1000.0
    R = np.linalg.norm(d[:, 1:4], axis=1)
    return t, R


def impacts_from_pcb(tu, R, imp_env, xlsx, sheet):
    """Impacts of the Shimmer session from the PCB take recorded at the same time.
    The PCB timestamps (PC clock) and the Shimmer timestamps differ by about 1 s and drift
    (~0.4 %), so each PCB shot is lined up by matching |a| (both limited to 3.4 g) around it,
    then put on the Shimmer's Impact envelope peak (within 0.1 s)."""
    tp, acc, shots = P.read_take(xlsx, sheet)
    Rp = np.linalg.norm(acc, axis=1)
    cap = 3.4 * G
    real = [(int(lab), iu) for lab, iu in shots if str(lab).strip().lower() != "test"]
    # coarse offset (shimmer time = pcb time + d) on 20 Hz envelopes, +-20 s
    g = np.arange(max(tp[0], tu[0] - 20), min(tp[-1], tu[-1] + 20), 0.05)
    a = np.minimum(np.interp(g, tp, Rp), cap)
    best = (-2, 0.0)
    for d in np.arange(-20, 20, 0.05):
        b = np.interp(g + d, tu, np.minimum(R, cap), left=np.nan, right=np.nan)
        m = ~np.isnan(b)
        if m.sum() > 400:
            c = np.corrcoef(a[m], b[m])[0, 1]
            if c > best[0]:
                best = (c, d)
    d0 = best[1]
    out = []
    for no, iu in real:
        w = np.arange(iu - 2.5, iu + 0.5, 1.0 / FS)
        pw = np.minimum(np.interp(w, tp, Rp), cap)
        bl = None
        for d in np.arange(d0 - 0.6, d0 + 0.6, 0.0025):
            sw = np.interp(w + d, tu, np.minimum(R, cap), left=np.nan, right=np.nan)
            if np.isnan(sw).any():
                continue
            c = np.corrcoef(pw, sw)[0, 1]
            if bl is None or c > bl[0]:
                bl = (c, d)
        if bl is None or bl[0] < 0.5:                 # shot not recorded by the Shimmer
            continue
        d0 = bl[1]                                    # follow the drift
        ts = iu + d0
        m = np.abs(tu - ts) <= 0.1
        out.append((int(np.flatnonzero(m)[np.argmax(imp_env[m])]), no))
    return out, len(real)


def analyse(csv_path, name, n_shots, n_test, graph_dir, table_file, pcb=None):
    t, Rraw = read_shimmer(csv_path)
    tu = np.arange(t[0], t[-1], 1.0 / FS)            # resampled to 400 Hz, like the PCB
    R = np.interp(tu, t, Rraw)
    trunk = P.bandpass_butter(R, FS, 1.0, 3.0, 4)
    chest = P.bandpass_butter(R, FS, 2.5, 7.0, 4)
    wrist = P.bandpass_butter(R, FS, 8.0, 15.0, 4)
    impact = P.bandpass_butter(R, FS, 20.0, 40.0, 4)
    imp_env = np.abs(P.hilbert(impact))
    if pcb is not None:                               # numbered like the PCB shots
        pairs, n_shots = impacts_from_pcb(tu, R, imp_env, *pcb)
        found = len(pairs)
    else:
        ks = P.impacts_python(imp_env, n_shots)
        found = len(ks)
        pairs = [(k, i) for i, k in enumerate(ks[n_test:], 1)]   # the first shots were tests

    def window(k, arr):
        s = max(0, int(k - 3.0 * FS))
        e = min(len(tu), int(k + 0.4 * FS))
        return np.interp(P.GRID, tu[s:e] - tu[k], arr[s:e], left=np.nan, right=np.nan)

    graph_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for k, shot_no in pairs:
        tr = window(k, trunk)
        ch = P.compute_envelope(P.GRID, window(k, chest))
        wr = P.compute_envelope(P.GRID, window(k, wrist))
        im = P.compute_envelope(P.GRID, window(k, impact))
        tf, ok_tf = P.compute_last_negative_valley_time(P.GRID, tr, -1.2, -0.2, 0.10)
        c, ok_c = P.compute_peak_time(P.GRID, ch, -0.4, -0.05, "max")
        w, ok_w = P.compute_peak_time(P.GRID, wr, -0.2, 0.0, "last")
        tf = tf if ok_tf else np.nan
        c = c if ok_c else np.nan
        w = w if ok_w else np.nan
        t_abs = tu[k] - tu[0]
        P.plot_waveform_wide(P.GRID, window(k, R), im, tr, ch, wr,
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
    return found, len(rows), n_shots


def main():
    try:
        import openpyxl  # noqa: F401
    except ImportError:
        sys.exit("openpyxl is missing:  C:\\Python314\\python.exe -m pip install openpyxl")
    matplotlib.use("Agg")

    if len(sys.argv) > 1:
        folder = Path(sys.argv[1])
    else:
        folder = Path(P.ask("Folder with the Shimmer CSV files (you can drag it here): ").strip('"& \''))
    files = sorted(folder.rglob("*.csv"), key=lambda p: [int(x) if x.isdigit() else x
                                                          for x in re.split(r"(\d+)", p.name)])
    files = [f for f in files if "shimmer" in f.name.lower() or "session" in f.name.lower()]
    if not files:
        sys.exit(f"No Shimmer CSV file in {folder}")

    info = []
    print("\nShimmer files:")
    for i, f in enumerate(files, 1):
        try:
            t, _ = read_shimmer(f)
            day = datetime.fromtimestamp(t[0]).strftime("%Y-%m-%d")
            start = datetime.fromtimestamp(t[0]).strftime("%H:%M:%S")
            info.append((f, day))
            print(f"  {i}. {f.name}   start {start}, {t[-1] - t[0]:.0f} s")
        except Exception as e:                         # noqa: BLE001
            info.append((f, None))
            print(f"  {i}. {f.name}   (cannot read: {e})")
    a = P.ask("Which sessions? (numbers separated by spaces, Enter = all): ")
    chosen = [int(x) for x in re.split(r"[ ,;]+", a) if x] if a else list(range(1, len(files) + 1))

    plan = []
    print("\nFor each session: its name, its number of shots (tests included) and its test shots.")
    for i in chosen:
        f, day = info[i - 1]
        if day is None:
            continue
        m = re.search(r"Session(\d+)", f.name)
        default = f"Shimmer Session {m.group(1)}" if m else f.stem
        name = P.ask(f"  Name for {f.name} (Enter = {default}): ", default)
        pcb = None
        xl = [p for p in (HERE.parent / f"results_{day}", HERE / f"results_{day}")
              if (p / f"PCB Raw Data {day}.xlsx").exists()]
        if xl:
            from openpyxl import load_workbook
            xlsx = xl[0] / f"PCB Raw Data {day}.xlsx"
            wb = load_workbook(xlsx, read_only=True)
            sheets = [sh for sh in wb.sheetnames if sh != "Info"]
            wb.close()
            print(f"    PCB takes in {xlsx.name}: " + "   ".join(f"{k}. {sh}" for k, sh in enumerate(sheets, 1)))
            c = P.ask(f"    PCB take recorded at the same time as '{name}' (number, Enter = none): ")
            if c:
                pcb = (xlsx, sheets[int(c) - 1])
        if pcb is None:
            n = int(P.ask(f"  Number of shots in '{name}' (tests included): "))
            nt = int(P.ask(f"  Test shots at the start of '{name}' (Enter = {max(0, n - 20)}): ",
                           str(max(0, n - 20))))
        else:
            n, nt = 0, 0
        plan.append((f, day, name, n, nt, pcb))

    for f, day, name, n, nt, pcb in plan:
        res_dir = HERE.parent / f"results_{day}" if (HERE.parent / f"results_{day}").is_dir() else \
            (HERE / f"results_{day}" if (HERE / f"results_{day}").is_dir() else folder)
        base = res_dir / "Calculated Data"
        safe = re.sub(r'[<>:"/\\|?*]', "-", name)
        print(f"  {name} ...", end="", flush=True)
        found, done, expected = analyse(f, name, n, nt, base / f"Graph data Shimmer {day}" / safe,
                                        base / "delta t Table Shimmer" / f"dt {safe} {day}.xlsx", pcb)
        if pcb is not None:
            note = "" if done == expected else \
                f"   ({expected - done} PCB shot(s) not recorded by the Shimmer)"
        else:
            note = "" if found == n else f"   (! only {found} impacts found)"
        print(f" {done} shots{note}")
        last = base
    if plan:
        print(f"\nDone: graphs and Δt tables in {last}")


if __name__ == "__main__":
    main()
