#!/usr/bin/env python3
"""
export_raw.py - make the "PCB Raw Data <date>.xlsx" file from a day of live_monitor.py sessions.

Run it after the day's sessions (Run button in VS Code, or:
    C:\\Python314\\python.exe export_raw.py
It asks:
  1. which day (one of the results_YYYY-MM-DD folders in Test_MCU_Python),
  2. which takes to put in the file,
  3. for each chosen take: its name (= sheet name) and how many test shots it starts with.

Output, in that day's folder:  PCB Raw Data YYYY-MM-DD.xlsx
  - Info sheet: one line per take (sheet, source session, shots, test shots, samples, duration)
  - one sheet per take, same columns as the Shimmer CSV:
        PCB_TimestampSync_Unix_CAL (ms) | PCB_Accel_X_CAL | PCB_Accel_Y_CAL | PCB_Accel_Z_CAL (m/s^2)
    and next to it (columns F-J) the shots: Shot (Test / 1..n), board no., impact time,
    row in the sheet, peak |a| (g).

It only reads files written by live_monitor.py (session_*_raw.csv, *_shots.csv, *.log);
nothing is changed. If a session was never decoded, run first:
    C:\\Python314\\python.exe live_monitor.py decode results_YYYY-MM-DD\\session_HHMMSS.log
Needs openpyxl:  C:\\Python314\\python.exe -m pip install openpyxl
"""
import csv
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
# the results_YYYY-MM-DD folders are made by live_monitor.py in Test_MCU_Python;
# this script lives in Test_MCU_Python/data conversion, so look one folder up as well
if not any(HERE.glob("results_*")) and any(HERE.parent.glob("results_*")):
    HERE = HERE.parent
G = 9.80665
FS = 400.0


# --------------------------------------------------------------------------- reading
def read_takes(day_dir):
    """All takes of a day: {take_no: dict(session, t_ms, acc, shots)}."""
    takes = {}
    for raw_csv in sorted(day_dir.glob("session_*_raw.csv")):
        sess = raw_csv.name[:-len("_raw.csv")]
        data = {}
        with open(raw_csv, newline="") as f:
            for r in csv.DictReader(f):
                d = data.setdefault(int(r["take"]), ([], []))
                d[0].append(float(r["t_ms"]))
                d[1].append((float(r["ax"]), float(r["ay"]), float(r["az"])))
        shots = {}
        shots_csv = day_dir / f"{sess}_shots.csv"
        if shots_csv.exists():
            with open(shots_csv, newline="") as f:
                for r in csv.DictReader(f):
                    shots.setdefault(int(r["take"]), []).append(
                        (r["shot"], int(r["I_ms"])))
        for tk, (t, acc) in data.items():
            t = np.array(t)
            acc = np.array(acc)
            # a few samples left over from the previous take can come first, then a long
            # pause: keep only what follows the last jump of more than 5 s at the start
            jumps = np.flatnonzero(np.diff(t) > 5000)
            for j in jumps:
                if t[j] - t[0] < 2000:
                    t, acc = t[j + 1:], acc[j + 1:]
            takes[tk] = dict(session=sess, t_ms=t, acc=acc, shots=shots.get(tk, []))
    return takes


def pc_clock(log_path, t_lo, t_hi):
    """Linear map MCU ms -> Unix s, from the PC time at which each data line arrived.
    Uses the fastest arrivals (lowest delay) of every 5 s, so Bluetooth delays do not shift it,
    and corrects the board clock drift (about 0.4 %)."""
    pc, t0 = [], []
    with open(log_path, encoding="ascii", errors="replace") as f:
        for line in f:
            if "\t" not in line:
                continue
            p, s = line.rstrip("\n").split("\t", 1)
            if s.startswith("R") and len(s) == 83:
                try:
                    tm = int(s[1:9], 16)
                    if t_lo - 1 <= tm <= t_hi + 1:
                        pc.append(float(p))
                        t0.append(tm / 1000.0)
                except ValueError:
                    pass
    pc, t0 = np.array(pc), np.array(t0)
    d = pc - t0
    b = ((t0 - t0[0]) // 5).astype(int)
    xs, ys = [], []
    for k in np.unique(b):
        m = b == k
        if m.sum() >= 20:
            xs.append(np.median(t0[m]))
            ys.append(np.percentile(d[m], 10))
    if len(xs) < 3:                                   # very short take: offset only
        return lambda tms: tms / 1000.0 + np.percentile(d, 2)
    xs, ys = np.array(xs), np.array(ys)
    p = np.polyfit(xs, ys, 1)
    for _ in range(3):                                # ignore the bins hit by delays
        r = ys - np.polyval(p, xs)
        keep = r < np.percentile(r, 80) + 0.01
        p = np.polyfit(xs[keep], ys[keep], 1)
    return lambda tms: tms / 1000.0 + np.polyval(p, tms / 1000.0)


# --------------------------------------------------------------------------- asking
def ask(prompt, default=""):
    a = input(prompt).strip()
    return a if a else default


def choose_day():
    days = sorted(p for p in HERE.glob("results_*") if p.is_dir())
    if not days:
        sys.exit(f"No results_YYYY-MM-DD folder next to {Path(__file__).name}.")
    if len(sys.argv) > 1:
        d = Path(sys.argv[1])
        return d if d.is_absolute() else HERE / d
    print("Days recorded:")
    for i, d in enumerate(days, 1):
        print(f"  {i}. {d.name[len('results_'):]}")
    a = ask(f"Which day? (number, Enter = {days[-1].name[len('results_'):]}): ")
    return days[int(a) - 1] if a else days[-1]


def sheet_name(name, used):
    n = re.sub(r"[\[\]:*?/\\]", "-", name).strip()[:31].strip() or "Take"
    base, k = n, 2
    while n.lower() in used:
        n = f"{base[:28]} ({k})"
        k += 1
    used.add(n.lower())
    return n


# --------------------------------------------------------------------------- main
def main():
    try:
        from openpyxl import Workbook
        from openpyxl.cell import WriteOnlyCell
        from openpyxl.styles import Font
    except ImportError:
        sys.exit("openpyxl is missing. Install it with:\n"
                 "    C:\\Python314\\python.exe -m pip install openpyxl")

    day_dir = choose_day()
    date = day_dir.name[len("results_"):]
    takes = read_takes(day_dir)
    if not takes:
        sys.exit(f"No decoded session in {day_dir.name} (no session_*_raw.csv).")

    print(f"\nTakes on {date}:")
    print("  take  session          shots  duration")
    for tk in sorted(takes):
        t = takes[tk]
        dur = (t["t_ms"][-1] - t["t_ms"][0]) / 1000 if len(t["t_ms"]) > 1 else 0
        print(f"  {tk:4d}  {t['session']:15s}  {len(t['shots']):5d}  {dur:6.0f} s")
    big = [tk for tk in sorted(takes) if len(takes[tk]["shots"]) > 20]
    a = ask(f"\nTakes to put in the file (numbers separated by spaces, "
            f"Enter = {' '.join(map(str, big)) or 'all'}): ")
    chosen = [int(x) for x in re.split(r"[ ,;]+", a) if x] if a else (big or sorted(takes))
    chosen = [tk for tk in chosen if tk in takes]
    if not chosen:
        sys.exit("No valid take chosen.")

    used, plan = set(), []
    print("\nFor each take: its name (sheet name, max 31 characters) and its test shots.")
    for tk in chosen:
        n = len(takes[tk]["shots"])
        name = ask(f"  Name for take {tk} (Enter = Take {tk:02d}): ", f"Take {tk:02d}")
        d_test = max(0, n - 20)
        tests = ask(f"  Test shots at the start of take {tk} (Enter = {d_test}): ", str(d_test))
        plan.append((tk, sheet_name(name, used), min(n, max(0, int(tests)))))

    out = day_dir / f"PCB Raw Data {date}.xlsx"
    print(f"\nWriting {out.name} ...")
    F = Font(name="Arial", size=10)
    FB = Font(name="Arial", size=10, bold=True)
    wb = Workbook(write_only=True)

    def cell(ws, v, bold=False, fmt=None):
        c = WriteOnlyCell(ws, value=v)
        c.font = FB if bold else F
        if fmt:
            c.number_format = fmt
        return c

    info = wb.create_sheet("Info")
    info_rows = []
    for tk, name, n_test in plan:
        t = takes[tk]
        tms, acc = t["t_ms"], t["acc"]
        clk = pc_clock(day_dir / f"{t['session']}.log", tms[0], tms[-1])
        unix_s = clk(tms)
        mag = np.linalg.norm(acc, axis=1)
        shot_rows, real = [], 0
        for k, (board_no, i_ms) in enumerate(t["shots"]):
            test = k < n_test
            if not test:
                real += 1
            inside = tms[0] <= i_ms <= tms[-1]
            j = int(np.argmin(np.abs(tms - i_ms)))
            w = np.abs(tms - i_ms) <= 50
            iu = float(clk(np.array([float(i_ms)]))[0])
            shot_rows.append(["Test" if test else real, int(board_no), round(iu * 1000, 1),
                              (j + 3) if inside else "before data",
                              round(float(mag[w].max() / G), 1) if w.any() else None])

        ws = wb.create_sheet(name)
        ws.column_dimensions["A"].width = 30
        for col in "BCD":
            ws.column_dimensions[col].width = 20
        ws.column_dimensions["E"].width = 4
        for col, wd in zip("FGHIJ", [10, 10, 28, 14, 14]):
            ws.column_dimensions[col].width = wd
        head = [["PCB_TimestampSync_Unix_CAL", "PCB_Accel_X_CAL", "PCB_Accel_Y_CAL", "PCB_Accel_Z_CAL"],
                ["ms", "m/(s^2)", "m/(s^2)", "m/(s^2)"]]
        shead = [["Shot", "Board no.", "Impact time (Unix ms)", "Row in sheet", "Peak |a| (g)"],
                 ["", "", "ms", "", "g"]]
        for i in range(len(tms) + 2):
            if i < 2:
                left = [cell(ws, v, True) for v in head[i]]
                right = [None] + [cell(ws, v, True) for v in shead[i]]
            else:
                ax, ay, az = acc[i - 2]
                left = [cell(ws, round(float(unix_s[i - 2] * 1000), 1), fmt="0.0"),
                        cell(ws, round(float(ax), 4), fmt="0.0000"),
                        cell(ws, round(float(ay), 4), fmt="0.0000"),
                        cell(ws, round(float(az), 4), fmt="0.0000")]
                right = []
                if i - 2 < len(shot_rows):
                    right = [None] + [cell(ws, v, fmt=("0.0" if q in (2, 4) else None))
                                      for q, v in enumerate(shot_rows[i - 2])]
            ws.append(left + right)
        info_rows.append([name, t["session"], len(t["shots"]), n_test, len(tms),
                          round((tms[-1] - tms[0]) / 1000, 1)])
        print(f"  {name}: {len(tms)} samples, {len(t['shots'])} shots ({n_test} test)")

    info.column_dimensions["A"].width = 32
    info.column_dimensions["B"].width = 18
    for col in "CDEF":
        info.column_dimensions[col].width = 18
    info.append([cell(info, f"PCB raw data - {date} (LIS3DH, ±16 g, 400 Hz)", True)])
    info.append([])
    info.append([cell(info, h, True) for h in
                 ["Sheet", "Source log", "Shots detected", "Test shots (at start)", "Samples", "Duration (s)"]])
    for r in info_rows:
        info.append([cell(info, v) for v in r])

    try:
        wb.save(out)
    except PermissionError:
        sys.exit(f"Cannot write {out.name}: close it in Excel and run again.")
    print(f"Done: {out}")


if __name__ == "__main__":
    main()
