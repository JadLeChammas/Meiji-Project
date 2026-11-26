# ============================================================
# 5bands_dt_V2806_1_cap35.py  (copy of V2806_0 with ONE change, 2026-10-27)
#   CHANGE: before the Trunk / Chest / Wrist band-pass filters, the composite acceleration R is
#   capped at 3.5 g (SLOW_CAP_G). The Impact band still uses the real R.
#   Why: on the Paul board (LIS3DH, +-16 g) the ball impact (up to 16 g) leaks into the slow
#   bands and creates a fake trunk valley ~0.27 s before impact (see trunk_fix_check.py).
#   The Shimmer3 saturates at 2 g per axis (~3.5 g composite), so it never had this leak:
#   capping at 3.5 g makes the Paul data behave like the Shimmer. On the 101 Paul swings
#   (Sessions 34-38, 41) vs Shimmer: dt_TfI corr 0.32 -> 0.63, dt_TfC median error 214 -> 40 ms.
#   On Shimmer files the cap changes almost nothing (they are already saturated near 3.5 g).
#   Same cap as SLOW_CAP in the MCU firmware main_live_raw.c.
# ------------------------------------------------------------
# (original header of V2806_0 below)
# 5bands_dt_shimmer_V2806_0.py 「Shimmer3解析用 最終確定版 2026-08-12」
# ------------------------------------------------------------
# 概要:
#   Shimmerの3軸加速度CSVから合成加速度Rを作成し、
#   テニスインパクトを0 sとして、Trunk / Chest / Wrist / Impact の
#   時間関係を抽出・可視化する最終整理版です。
#
# V2806_0 最終整理版の方針:
#   1) 使用する信号
#      - Composite acceleration R: 生加速度合成値、青
#      - Trunk: 1.0-3.0 Hz filtered波形そのもの、紫
#        ※ TrunkだけはHilbert envelopeを使わない
#      - Chest: 2.5-7.0 Hz Hilbert envelope、オレンジ
#      - Wrist: 8.0-15.0 Hz Hilbert envelope、緑
#      - Impact: 20.0-40.0 Hz Hilbert envelope、赤
#
#   2) イベント定義
#      - Tf: Trunk 1.0-3.0 Hz filtered波形の負側谷
#      - C : Chest envelopeのピーク
#      - W : Wrist envelopeのピーク
#      - I : Impact envelopeのピーク（相対時間では0 s）
#
#   3) Excel出力
#      - 【追加】all_time_waveformsを追加した。
#        全計測時間の5波形（Composite R / Trunk / Chest / Wrist / Impact）を、
#        結果Excel内の all_time_waveforms シートと、独立した
#        all_time_waveforms_*.xlsx ファイルの両方へ保存する。
#      - 独立した all_time_waveforms_*.xlsx は出力フォルダ内へ保存し、
#        ZIPが「全部入り」「PPTとExcelのみ」のどちらでも必ず含める。
#      - 各イベントについて、測定開始からの絶対時間と、
#        Impactを0 sとした相対時間の両方を保存する。
#      - Δtは以下6本を保存する。
#          dt_TfC_s = C - Tf
#          dt_CW_s  = W - C
#          dt_WI_s  = I - W
#          dt_TfW_s = W - Tf
#          dt_CI_s  = I - C
#          dt_TfI_s = I - Tf
#      - Impact飽和の可能性があるため、現段階の主指標は
#          dt_TfC_s, dt_CW_s, dt_TfW_s
#        とし、Impactを含む dt_WI_s, dt_CI_s, dt_TfI_s は参考指標として保存する。
#
#   4) PNG出力
#      - R_raw_impact_check_xxx.png : 絶対時間・生加速度R + Impact確認
#      - R_raw_all_check_xxx.png    : 絶対時間・生加速度R + 全帯域確認
#      - R_waveform_wide_xx.png     : Impact基準 -3.0〜+0.5 s、生データあり
#      - R_waveform_zoom_xx.png     : Impact基準 -0.8〜+0.2 s、生データなし
#      - R_dt_*.png                 : Δt散布図
#      - overlay_mean_*.png         : 選択ショットの重ね描き
#
#   5) 旧図 R_4bands_* および detail図は、最終版では作成しない。
#
# 注意:
#   - R_raw_impact_check と R_raw_all_check には、抽出されたImpact候補の
#     振幅順位と時間順No.をx軸下に表示する。解析で使う番号は時間順No.です。
#   - Impactが飽和している場合、Impactを含むΔtは参考値として扱う。
# ============================================================

import os
import shutil
import datetime
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks, butter, filtfilt, hilbert


TITLE_FONTSIZE = 15
LABEL_FONTSIZE = 12
TICK_FONTSIZE = 12
LEGEND_FONTSIZE = 10


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


def in_colab():
    try:
        import google.colab  # type: ignore
        return True
    except Exception:
        return False


def ask_1_0(prompt, default=1):
    """1/0形式の質問を行う。Enterのみの場合は default を採用する。

    V2806_0修正:
      - 「はい/いいえ」ではなく、1=作成する、0=作成しない、で選択する。
    """
    default = 1 if int(default) != 0 else 0
    while True:
        ans = input(f"{prompt}（1=作成する / 0=作成しない、空欄={default}）: ").strip()
        if ans == "":
            return bool(default)
        if ans == "1":
            return True
        if ans == "0":
            return False
        print("[WARN] 1 または 0 で入力してください。", flush=True)


def ask_zip_content_mode(default=1):
    """ZIPに入れる内容を選択する。

    V2806_0:
      - 1 = すべての出力ファイルをZIPに入れる（従来通り）
      - 0 = PPTXとExcelのみをZIPに入れる
    """
    default = 1 if int(default) != 0 else 0
    while True:
        ans = input(
            f"ZIPに入れる内容を選択してください（1=全部 / 0=PPTとExcelのみ、空欄={default}）: "
        ).strip()
        if ans == "":
            return bool(default)
        if ans == "1":
            return True
        if ans == "0":
            return False
        print("[WARN] 1 または 0 で入力してください。", flush=True)


def pick_csv_by_upload(prompt="Shimmer 生CSV をアップロードしてください（V2806_0）："):
    """Colab ではアップロード画面を出し、通常PythonではCSVパスを手入力する。"""
    if in_colab():
        from google.colab import files  # type: ignore
        print(prompt, flush=True)
        uploaded = files.upload()
        if not uploaded:
            raise SystemExit("アップロードがキャンセルされました。")
        name = next(iter(uploaded.keys()))
        with open(name, "wb") as f:
            f.write(uploaded[name])
        print(f"[INFO] 読み込んだファイル: {name}", flush=True)
        return os.path.abspath(name)

    print("[INFO] Colabではない環境として実行されています。", flush=True)
    path = input("解析するCSVファイルのパスを入力してください: ").strip().strip('"')
    if not path:
        raise SystemExit("CSVファイルのパスが入力されていません。")
    if not os.path.exists(path):
        raise SystemExit(f"CSVファイルが見つかりません: {path}")
    print(f"[INFO] 読み込んだファイル: {path}", flush=True)
    return os.path.abspath(path)


def extract_session_id(path: str) -> str:
    """Extract session number from input filename, e.g. Session2 / session_2."""
    name = os.path.basename(path)
    m = re.search(r'(?i)session[_\- ]*(\d+)', name)
    if m:
        return m.group(1)
    return 'NA'


def normalize_time_to_seconds(t_raw):
    t_raw = np.asarray(t_raw, dtype=float)
    dt = float(np.nanmedian(np.diff(t_raw)))
    if dt > 1e6:
        scale = 1e9
    elif dt > 1e3:
        scale = 1e6
    elif dt > 1:
        scale = 1e3
    else:
        scale = 1.0
    t0 = t_raw[0]
    return (t_raw - t0) / scale


def read_v70(path: str) -> pd.DataFrame:
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        head = [f.readline() for _ in range(3)]
    sep = "\t" if "\t" in "".join(head) else ","
    try:
        df = pd.read_csv(path, sep=sep, header=[1, 2], engine="python")
        df.columns = [
            f"{a.strip()} [{str(b).strip()}]" if str(b) != "nan" else a.strip()
            for a, b in df.columns
        ]
    except Exception:
        df = pd.read_csv(path, sep=sep, engine="python")
        df.columns = [c.strip() for c in df.columns]

    def first_match(keys):
        for k in keys:
            for c in df.columns:
                if k.lower() in c.lower():
                    return c
        return None

    # V2804: Shimmer生データのA列を時刻列として優先的に使用する。
    # これにより、検出インパクトの実時間差を確認できるようにする。
    c_time = df.columns[0] if len(df.columns) > 0 else first_match(["TimestampSync_Unix_CAL", "TimeStamp", "Time"])
    c_ax = first_match(["Accel_WR_X_CAL", "Accel_LN_X_CAL", "Accel_X_CAL", "Accel X"])
    c_ay = first_match(["Accel_WR_Y_CAL", "Accel_LN_Y_CAL", "Accel_Y_CAL", "Accel Y"])
    c_az = first_match(["Accel_WR_Z_CAL", "Accel_LN_Z_CAL", "Accel_Z_CAL", "Accel Z"])
    if not (c_time and c_ax and c_ay and c_az):
        print("[DEBUG] columns:", df.columns.tolist())
        raise SystemExit("必要な列（Time/Accel_X/Y/Z）が見つかりません。")

    df2 = df[[c_time, c_ax, c_ay, c_az]].copy()
    df2.columns = ["time_raw", "ax", "ay", "az"]
    for c in ["time_raw", "ax", "ay", "az"]:
        df2[c] = pd.to_numeric(df2[c], errors="coerce")
    df2 = df2.dropna().reset_index(drop=True)
    df2["time"] = normalize_time_to_seconds(df2["time_raw"].to_numpy())
    return df2[["time", "ax", "ay", "az"]]


def estimate_Fs(t):
    return 1.0 / np.median(np.diff(t))


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


def compute_last_valley_time(grid, env, t_min, t_max, prom_ratio=0.10):
    t = np.asarray(grid, dtype=float)
    e = np.asarray(env, dtype=float)
    m = (t >= float(t_min)) & (t <= float(t_max)) & np.isfinite(e)
    tt = t[m]
    ee = e[m]
    if len(tt) < 8:
        return float("nan"), False
    inv = -ee
    pr = float(prom_ratio) * (float(np.nanmax(inv)) - float(np.nanmin(inv)) + 1e-12)
    valleys, _ = find_peaks(inv, prominence=pr)
    if len(valleys) == 0:
        return float("nan"), False
    iv = int(valleys[-1])
    return float(tt[iv]), True


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


def plot_envelopes(grid, env_list, band_info_list, tpeak_list, dt_list, impact_idx, out_png, foot_line_time=None):
    fig, ax = plt.subplots(figsize=(10, 5.5))
    y_text_base = 9.4
    y_text_step = 1.0
    text_count = 0

    for env, (label, fL, fH, color), tpk, dt_ms in zip(env_list, band_info_list, tpeak_list, dt_list):
        ax.plot(grid, env, label=f"{label} {fL}-{fH} Hz", color=color, lw=1.8)
        if np.isfinite(tpk):
            lw = 1.8
            zo = 5
            ax.axvline(tpk, color=color, ls="--", lw=lw, alpha=0.95, zorder=zo)
            env_arr = np.asarray(env, dtype=float)
            idx = int(np.nanargmin(np.abs(np.asarray(grid, dtype=float) - float(tpk))))
            y_marker = env_arr[idx] if 0 <= idx < len(env_arr) else np.nan
            if np.isfinite(y_marker):
                ax.plot([tpk], [y_marker], marker="o", ms=5.5, color=color, mec="white", mew=0.8, zorder=zo + 1)
            # V2806_0: 縦線に重ねる注釈文字は表示しない。
            text_count += 1

    if np.isfinite(foot_line_time):
        # Foot も他色と同じ太さで、最後に明示的に再描画する
        ax.axvline(foot_line_time, color="C4", ls="--", lw=1.8, alpha=0.95, zorder=20)
        idxf = int(np.nanargmin(np.abs(np.asarray(grid, dtype=float) - float(foot_line_time))))
        if 0 <= idxf < len(env_list[0]):
            y_f = np.asarray(env_list[0], dtype=float)[idxf]
            if np.isfinite(y_f):
                ax.plot([foot_line_time], [y_f], marker='o', ms=5.5, color='C4', mec='white', mew=0.8, zorder=21)
        # V2806_0: Foot縦線の注釈文字は表示しない。

    ax.axvline(0.0, color="k", ls="--", lw=1.2, zorder=4)
    ax.set_xlim(-3.0, 0.5)
    ax.set_ylim(0.0, 10.0)
    ax.grid(True, alpha=0.3)
    ax.set_title(f"5-band Hilbert envelopes (impact {impact_idx:02d})")
    ax.set_xlabel("Time aligned to Racket-band peak (s)")
    ax.set_ylabel("Amplitude(g)")
    ax.legend(loc="upper right")
    fig.subplots_adjust(left=0.08, right=0.98, bottom=0.14, top=0.90)
    plt.savefig(out_png, dpi=200)
    plt.close(fig)



def plot_raw_impact_check_segments(t, R, impact_env, top_idx, impact_rank_by_idx, outdir, segment_sec=5.0):
    """通信エラー・インパクト検出確認用の時系列グラフを5秒ごとに作成する。

    V2804:
      - フィルタ前の合成加速度 R を青線で表示する
      - インパクト検出用の Impact envelope を赤線で表示する
      - 左軸（合成加速度R）は 0〜35、右軸（Impact envelope）は 0〜6 に固定する
      - 抽出したK個のインパクト候補の測定開始からの時刻に細い点線を描く
      - x軸の下に、上段へ「大きさ順番号」、下段へ時間順番号「No.xx」を表示する
      - 1枚あたり5秒の時間窓で、計測開始から計測終了まで全時間をPNG保存する
    """
    t = np.asarray(t, dtype=float)
    R = np.asarray(R, dtype=float)
    impact_env = np.asarray(impact_env, dtype=float)
    top_idx = np.asarray(top_idx, dtype=int)

    finite_t = t[np.isfinite(t)]
    if len(finite_t) == 0:
        return []

    t_start = float(np.nanmin(finite_t))
    t_end = float(np.nanmax(finite_t))
    if not np.isfinite(t_end) or not np.isfinite(t_start) or t_end <= t_start:
        return []

    # 通常は normalize_time_to_seconds により t_start は 0 になるが、
    # 念のため計測開始からの全時間を5秒窓でカバーする。
    seg0 = int(np.floor(t_start / float(segment_sec)))
    seg_last = int(np.ceil(t_end / float(segment_sec)))
    out_paths = []
    impact_times = {int(k): float(t[int(k)]) for k in top_idx if 0 <= int(k) < len(t)}

    for seg in range(seg0, seg_last):
        t0 = float(seg) * float(segment_sec)
        t1 = t0 + float(segment_sec)

        # 最終セグメントでは t_end のサンプルも含める。
        if seg == seg_last - 1:
            m = (t >= t0) & (t <= t1) & np.isfinite(t) & np.isfinite(R)
            m2 = (t >= t0) & (t <= t1) & np.isfinite(t) & np.isfinite(impact_env)
        else:
            m = (t >= t0) & (t < t1) & np.isfinite(t) & np.isfinite(R)
            m2 = (t >= t0) & (t < t1) & np.isfinite(t) & np.isfinite(impact_env)

        # 波形がない完全な空窓は作らない。ただし計測データがある範囲はすべて出力する。
        if not np.any(m) and not np.any(m2):
            continue

        fig, ax1 = plt.subplots(figsize=(11, 5.3))
        if np.any(m):
            ax1.plot(t[m], R[m], color="blue", lw=1.0, alpha=0.9, label="Composite acceleration")
        ax1.set_xlabel("Time from measurement start (s)")
        ax1.set_ylabel("Composite acceleration R (g)", color="blue")
        ax1.tick_params(axis='y', labelcolor="blue")
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(t0, t1)
        # V2804: 左軸は全セグメントで 0〜35 に固定する。
        ax1.set_ylim(0.0, 35.0)

        ax2 = ax1.twinx()
        if np.any(m2):
            ax2.plot(t[m2], impact_env[m2], color="red", lw=1.2, alpha=0.9, label="Impact")
        ax2.set_ylabel("Impact envelope (g)", color="red")
        ax2.tick_params(axis='y', labelcolor="red")
        # V2804: 右軸は全セグメントで 0〜6 に固定する。
        ax2.set_ylim(0.0, 6.0)

        for time_order_no, (k, tk) in enumerate(sorted(impact_times.items(), key=lambda kv: kv[1]), start=1):
            if t0 <= tk < t1 or (seg == seg_last - 1 and np.isclose(tk, t1)):
                rank = impact_rank_by_idx.get(int(k), None)
                ax1.axvline(tk, color="gray", ls=":", lw=0.8, alpha=0.85)
                if rank is not None:
                    # V2804: x軸の下へ、上段に大きさ順番号、下段に時間順番号 No.xx を表示する。
                    ax1.text(
                        tk, -0.10, str(rank),
                        transform=ax1.get_xaxis_transform(),
                        rotation=0, ha="center", va="top",
                        fontsize=8, color="black", clip_on=False
                    )
                    ax1.text(
                        tk, -0.18, f"No.{time_order_no:02d}",
                        transform=ax1.get_xaxis_transform(),
                        rotation=0, ha="center", va="top",
                        fontsize=7.5, color="black", clip_on=False
                    )

        ax1.set_title(f"Raw acceleration and Impact envelope check: {t0:.1f}-{t1:.1f} s")
        apply_axis_style(ax1)
        apply_axis_style(ax2)
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        # V2806_0: タイトルと重ならないよう、凡例は図の下側余白へ配置する。
        # X軸キャプションや下部の番号表示と重ならないよう、figure legend を使う。
        lg = fig.legend(
            lines1 + lines2, labels1 + labels2,
            loc="lower center", bbox_to_anchor=(0.5, 0.02), ncol=2, frameon=True
        )
        apply_legend_style(lg)
        # 下側には x軸ラベル・番号・凡例を置くため、十分な余白を確保する。
        fig.subplots_adjust(left=0.08, right=0.92, top=0.88, bottom=0.34)
        out_png = os.path.join(outdir, f"R_raw_impact_check_{seg:03d}.png")
        plt.savefig(out_png, dpi=200)
        plt.close(fig)
        out_paths.append(out_png)

    return out_paths



def plot_raw_all_check_segments(t, R, impact_env, trunk_filtered, chest_env, wrist_env,
                                top_idx, impact_rank_by_idx, outdir, segment_sec=5.0):
    """加速度生データと主要帯域波形を5秒ごとに重ねて表示する。

    V2804:
      - PNG名は R_raw_all_check_xxx.png
      - 左軸: 加速度生データR（青）0〜35 g
      - 右軸: Impact（赤）、TrunkFiltered（紫）、Chest（オレンジ）、Wrist（緑） -10〜+10 g
      - R_raw_impact_check と同様に、抽出したK個のインパクト候補の位置と番号を表示する
    """
    t = np.asarray(t, dtype=float)
    R = np.asarray(R, dtype=float)
    impact_env = np.asarray(impact_env, dtype=float)
    trunk_filtered = np.asarray(trunk_filtered, dtype=float)
    chest_env = np.asarray(chest_env, dtype=float)
    wrist_env = np.asarray(wrist_env, dtype=float)
    top_idx = np.asarray(top_idx, dtype=int)

    finite_t = t[np.isfinite(t)]
    if len(finite_t) == 0:
        return []
    t_start = float(np.nanmin(finite_t))
    t_end = float(np.nanmax(finite_t))
    if not np.isfinite(t_end) or not np.isfinite(t_start) or t_end <= t_start:
        return []

    seg0 = int(np.floor(t_start / float(segment_sec)))
    seg_last = int(np.ceil(t_end / float(segment_sec)))
    out_paths = []
    impact_times = {int(k): float(t[int(k)]) for k in top_idx if 0 <= int(k) < len(t)}

    for seg in range(seg0, seg_last):
        t0 = float(seg) * float(segment_sec)
        t1 = t0 + float(segment_sec)
        if seg == seg_last - 1:
            m = (t >= t0) & (t <= t1) & np.isfinite(t)
        else:
            m = (t >= t0) & (t < t1) & np.isfinite(t)
        if not np.any(m):
            continue

        fig, ax1 = plt.subplots(figsize=(11, 4.8))
        if np.any(m & np.isfinite(R)):
            ax1.plot(t[m], R[m], color="blue", lw=1.0, alpha=0.9, label="Composite acceleration")
        ax1.set_xlabel("Time from measurement start (s)")
        ax1.set_ylabel("Composite acceleration R (g)", color="blue")
        ax1.tick_params(axis='y', labelcolor="blue")
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(t0, t1)
        ax1.set_ylim(0.0, 35.0)

        ax2 = ax1.twinx()
        if np.any(m & np.isfinite(impact_env)):
            ax2.plot(t[m], impact_env[m], color="red", lw=1.2, alpha=0.9, label="Impact")
        if np.any(m & np.isfinite(trunk_filtered)):
            ax2.plot(t[m], trunk_filtered[m], color="purple", lw=1.2, ls="-", alpha=0.9, label="Trunk")
        if np.any(m & np.isfinite(chest_env)):
            ax2.plot(t[m], chest_env[m], color="orange", lw=1.1, alpha=0.9, label="Chest")
        if np.any(m & np.isfinite(wrist_env)):
            ax2.plot(t[m], wrist_env[m], color="green", lw=1.1, alpha=0.9, label="Wrist")
        ax2.set_ylabel("Amplitude(g)")
        ax2.set_ylim(-10.0, 10.0)

        for time_order_no, (k, tk) in enumerate(sorted(impact_times.items(), key=lambda kv: kv[1]), start=1):
            if t0 <= tk < t1 or (seg == seg_last - 1 and np.isclose(tk, t1)):
                rank = impact_rank_by_idx.get(int(k), None)
                ax1.axvline(tk, color="gray", ls=":", lw=0.8, alpha=0.85)
                if rank is not None:
                    ax1.text(tk, -0.10, str(rank), transform=ax1.get_xaxis_transform(),
                             ha="center", va="top", fontsize=8, color="black", clip_on=False)
                    ax1.text(tk, -0.18, f"No.{time_order_no:02d}", transform=ax1.get_xaxis_transform(),
                             ha="center", va="top", fontsize=7.5, color="black", clip_on=False)

        ax1.set_title(f"Raw acceleration and all band check: {t0:.1f}-{t1:.1f} s")
        apply_axis_style(ax1)
        apply_axis_style(ax2)
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        # V2806_0: タイトルと重ならないよう、凡例は図の下側余白へ配置する。
        lg = fig.legend(
            lines1 + lines2, labels1 + labels2,
            loc="lower center", bbox_to_anchor=(0.5, 0.02), ncol=3, frameon=True
        )
        apply_legend_style(lg)
        fig.subplots_adjust(left=0.08, right=0.92, top=0.88, bottom=0.34)
        out_png = os.path.join(outdir, f"R_raw_all_check_{seg:03d}.png")
        plt.savefig(out_png, dpi=200)
        plt.close(fig)
        out_paths.append(out_png)
    return out_paths



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
    ax1.set_xlim(-3.0, 0.5)
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
    ax2.set_ylim(-10.0, 10.0)
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


def plot_waveform_zoom(grid, impact_env, trunk_filtered, chest_env, wrist_env, t_feature,
                       impact_idx, impact_time_abs_s, out_png):
    """最終版のzoom図を作成する。

    V2806_0:
      - 横軸は Impact基準 -0.8〜+0.2 s
      - 生加速度Rは入れず、イベント順序とΔtを見やすくする
      - Trunk filtered（紫, Hilbertなし）, Chest（オレンジ）, Wrist（緑）, Impact（赤）を表示する
    """
    fig, ax = plt.subplots(figsize=(10, 5.5))
    plot_specs = [
        ("Impact", "Impact", np.asarray(impact_env, dtype=float), "red", "peak", 9.2),
        ("TrunkFiltered", "Trunk", np.asarray(trunk_filtered, dtype=float), "purple", "valley", 7.8),
        ("Chest", "Chest", np.asarray(chest_env, dtype=float), "orange", "peak", 6.4),
        ("Wrist", "Wrist", np.asarray(wrist_env, dtype=float), "green", "peak", 5.0),
    ]
    for key, label, y_arr, color, kind, text_y in plot_specs:
        ax.plot(grid, y_arr, color=color, lw=1.8 if key != "TrunkFiltered" else 1.6,
                ls="-", alpha=0.95, label=label)
        _plot_feature_marker(
            ax, grid, y_arr, float(t_feature.get(key, float("nan"))),
            color, key, kind, text_y, marker="o", linestyle="--"
        )

    ax.axvline(0.0, color="k", ls="--", lw=1.2, zorder=5)
    ax.set_xlim(-0.8, 0.2)
    ax.set_ylim(-10.0, 10.0)
    ax.grid(True, alpha=0.3)
    title_time_abs = f"absolute impact time = {impact_time_abs_s:.3f} s" if np.isfinite(impact_time_abs_s) else "absolute impact time = N/A"
    ax.set_title(f"Waveform zoom (impact {impact_idx:02d}, {title_time_abs})")
    ax.set_xlabel("Time aligned to Impact peak (s)")
    ax.set_ylabel("Amplitude(g)")
    apply_axis_style(ax)
    lg = ax.legend(loc="upper left")
    apply_legend_style(lg)
    fig.subplots_adjust(left=0.12, right=0.98, bottom=0.14, top=0.90)
    plt.savefig(out_png, dpi=200)
    plt.close(fig)

def plot_overlaid_shots(grid, envs_by_band, tpeaks_by_band, bands, out_png_prefix, label_numbers=None):
    MIN_DX_S = 0.015
    STACK_DY_FRAC = 0.055
    if label_numbers is None:
        label_numbers = []
    for (label, fL, fH, _color) in bands:
        env_list = envs_by_band.get(label, [])
        if len(env_list) == 0:
            continue
        if len(label_numbers) == len(env_list):
            band_label_numbers = list(label_numbers)
        else:
            band_label_numbers = list(range(1, len(env_list) + 1))
        tpk_list = list(tpeaks_by_band.get(label, []))
        if len(tpk_list) < len(env_list):
            tpk_list += [float("nan")] * (len(env_list) - len(tpk_list))
        tpk_list = tpk_list[:len(env_list)]
        env_stack = np.vstack([np.asarray(e, dtype=float) for e in env_list])
        env_mean = np.nanmean(env_stack, axis=0)

        fig, ax = plt.subplots(figsize=(9, 4.8))
        for env in env_list:
            ax.plot(grid, env, alpha=0.25, lw=1.2)
        ax.plot(grid, env_mean, ls="--", lw=2.2, label=f"Mean (n={len(env_list)})")
        ax.axvline(0.0, color="k", ls="--", lw=1.0)

        if label == "Trunk":
            ax.set_xlim(-3.0, 0.5)
            ax.set_ylim(-10.0, 10.0)
        else:
            ax.set_xlim(-3.0, 0.5)
            ax.set_ylim(0.0, 10.0)

        ax.set_title(f"Overlay waveforms: {label} {fL}-{fH} Hz (shots={len(env_list)})")
        ax.set_xlabel("Time aligned to Racket-band peak (s)")
        ax.set_ylabel("Amplitude(g)")
        apply_axis_style(ax)
        lg = ax.legend(loc="upper left")
        apply_legend_style(lg)

        y0, y1 = ax.get_ylim()
        yr = max(1e-12, (y1 - y0))
        y_top = y1 - 0.03 * yr
        dy = STACK_DY_FRAC * yr

        entries = [(float(tpk), int(band_label_numbers[i])) for i, tpk in enumerate(tpk_list) if np.isfinite(tpk)]
        entries_sorted = sorted(entries, key=lambda x: x[0])

        levels = {}
        prev_t = None
        cur_level = 0
        for tpk, n in entries_sorted:
            if prev_t is None:
                cur_level = 0
            elif abs(tpk - prev_t) <= MIN_DX_S:
                cur_level += 1
            else:
                cur_level = 0
            levels[n] = cur_level
            prev_t = tpk

        for i, tpk in enumerate(tpk_list):
            if not np.isfinite(tpk):
                continue
            n = int(band_label_numbers[i])
            ax.axvline(tpk, color="k", alpha=0.18, lw=1.0)
            lvl = int(levels.get(n, 0))
            y_txt = max(y0 + 0.70 * yr, y_top - lvl * dy)
            x_txt = float(tpk) + 0.003
            ax.text(
                x_txt, y_txt, f"{n}", color="k", alpha=0.35, fontsize=9,
                rotation=90, ha="left", va="top"
            )

        plt.tight_layout()
        plt.savefig(f"{out_png_prefix}_{label}.png", dpi=200)
        plt.close(fig)


def _ensure_pptx_available():
    """python-pptx を読み込む。Colabで未導入の場合は自動インストールを試みる。"""
    try:
        from pptx import Presentation  # noqa: F401
        return
    except ModuleNotFoundError:
        pass

    if in_colab():
        print("[INFO] python-pptx が見つからないため、Colab にインストールします...", flush=True)
        import sys
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "python-pptx", "-q"])
        from pptx import Presentation  # noqa: F401
        return

    raise SystemExit(
        "python-pptx が見つかりません。PowerPoint作成には `pip install python-pptx` が必要です。"
    )


def _add_small_zip_label(slide, zip_label):
    from pptx.util import Inches, Pt
    box = slide.shapes.add_textbox(Inches(0.12), Inches(0.06), Inches(4.8), Inches(0.22))
    tf = box.text_frame
    tf.clear()
    p = tf.paragraphs[0]
    p.text = zip_label
    p.font.size = Pt(7)
    p.font.bold = False


def _add_image_fit(slide, image_path, left, top, width, height):
    """指定枠内に画像を縦横比を保って配置する。"""
    try:
        from PIL import Image
        with Image.open(image_path) as im:
            w_px, h_px = im.size
        if w_px > 0 and h_px > 0:
            img_ratio = w_px / h_px
            box_ratio = width / height
            if img_ratio >= box_ratio:
                pic_w = width
                pic_h = width / img_ratio
            else:
                pic_h = height
                pic_w = height * img_ratio
            pic_left = left + (width - pic_w) / 2
            pic_top = top + (height - pic_h) / 2
            slide.shapes.add_picture(image_path, pic_left, pic_top, width=pic_w, height=pic_h)
            return
    except Exception:
        pass

    slide.shapes.add_picture(image_path, left, top, width=width, height=height)


def _collect_numbered_pngs(outdir, prefix):
    """R_5bands_envelope_01.png などを番号順に集める。"""
    files = []
    pat = re.compile(rf"^{re.escape(prefix)}_(\d+)\.png$", re.IGNORECASE)
    for name in os.listdir(outdir):
        m = pat.match(name)
        if m:
            files.append((int(m.group(1)), os.path.join(outdir, name)))
    return [path for _num, path in sorted(files, key=lambda x: x[0])]


def _add_four_pngs_per_slide(prs, image_paths, zip_label, title):
    from pptx.util import Inches, Pt
    blank = prs.slide_layouts[6]

    # V2806_0: 1スライド4枚の図面を少し大きく見せるため、余白を減らした4分割配置にする。
    positions = [
        (Inches(0.20), Inches(0.48), Inches(6.35), Inches(3.18)),
        (Inches(6.78), Inches(0.48), Inches(6.35), Inches(3.18)),
        (Inches(0.20), Inches(3.72), Inches(6.35), Inches(3.18)),
        (Inches(6.78), Inches(3.72), Inches(6.35), Inches(3.18)),
    ]

    for start in range(0, len(image_paths), 4):
        slide = prs.slides.add_slide(blank)
        _add_small_zip_label(slide, zip_label)
        title_box = slide.shapes.add_textbox(Inches(0.25), Inches(0.20), Inches(12.6), Inches(0.24))
        tf = title_box.text_frame
        tf.clear()
        p = tf.paragraphs[0]
        p.text = f"{title}  ({start + 1}-{min(start + 4, len(image_paths))} / {len(image_paths)})"
        p.font.size = Pt(9)
        p.font.bold = True

        for img_path, (left, top, width, height) in zip(image_paths[start:start + 4], positions):
            _add_image_fit(slide, img_path, left, top, width, height)


def _summary_mean_sd_text(df_summary, mean_colname, sd_colname):
    """Summaryシート相当の Mean と SD をPPT表示用文字列にする。"""
    def fmt_value(colname, label):
        if colname not in df_summary.columns or df_summary.empty:
            return f"{label} = N/A"
        val = pd.to_numeric(df_summary[colname], errors="coerce").iloc[0]
        if not np.isfinite(val):
            return f"{label} = N/A"
        return f"{label} = {val:.4f} s  ({val * 1000.0:.1f} ms)"

    return fmt_value(mean_colname, "Mean") + "\n" + fmt_value(sd_colname, "SD")


def _add_overlay_slide(prs, image_path, zip_label, title, mean_sd_text):
    from pptx.util import Inches, Pt
    blank = prs.slide_layouts[6]
    slide = prs.slides.add_slide(blank)
    _add_small_zip_label(slide, zip_label)

    title_box = slide.shapes.add_textbox(Inches(0.45), Inches(0.23), Inches(8.5), Inches(0.30))
    tf = title_box.text_frame
    tf.clear()
    p = tf.paragraphs[0]
    p.text = title
    p.font.size = Pt(11)
    p.font.bold = True

    _add_image_fit(slide, image_path, Inches(0.55), Inches(0.68), Inches(12.25), Inches(6.55))

    # グラフと重ならないよう、右上の画像領域より上側に Mean / SD を配置する。
    # V2801S_PPT3: 以前より少し上へ移動し、フォントも大きくした。
    stat_box = slide.shapes.add_textbox(Inches(9.05), Inches(0.20), Inches(4.05), Inches(0.48))
    tf = stat_box.text_frame
    tf.clear()
    tf.margin_left = 0
    tf.margin_right = 0
    tf.margin_top = 0
    tf.margin_bottom = 0
    p = tf.paragraphs[0]
    p.text = mean_sd_text
    p.font.size = Pt(13)
    p.font.bold = True



def _summary_numeric_value(df_summary, colname):
    """Summaryシート相当の1行データから数値を取り出す。"""
    if colname not in df_summary.columns or df_summary.empty:
        return float("nan")
    val = pd.to_numeric(df_summary[colname], errors="coerce").iloc[0]
    return float(val) if np.isfinite(val) else float("nan")


def _fmt_seconds_for_table(val):
    """秒の値を s と ms の両方で表示する。"""
    if not np.isfinite(val):
        return "N/A"
    return f"{val:.4f} s  ({val * 1000.0:.1f} ms)"


def _add_overview_slide(prs, zip_label, df_summary):
    """PPT先頭に、ZIP名・N・Mean/SDの概要スライドを追加する。"""
    from pptx.util import Inches, Pt
    from pptx.enum.text import PP_ALIGN
    from pptx.dml.color import RGBColor

    blank = prs.slide_layouts[6]
    slide = prs.slides.add_slide(blank)
    _add_small_zip_label(slide, zip_label)

    title_box = slide.shapes.add_textbox(Inches(0.55), Inches(0.45), Inches(12.2), Inches(0.45))
    tf = title_box.text_frame
    tf.clear()
    p = tf.paragraphs[0]
    p.text = "Summary of extracted shots"
    p.font.size = Pt(20)
    p.font.bold = True

    n_total = _summary_numeric_value(df_summary, "n_total")
    info_box = slide.shapes.add_textbox(Inches(0.65), Inches(1.05), Inches(12.0), Inches(0.55))
    tf = info_box.text_frame
    tf.clear()
    p = tf.paragraphs[0]
    p.text = f"ZIP file: {zip_label}    N = {int(n_total) if np.isfinite(n_total) else 'N/A'}"
    p.font.size = Pt(15)
    p.font.bold = True

    rows = [
        ("Trunk filtered Tf", "mean_t_trunk_filtered_valley_rel_s", "sd_t_trunk_filtered_valley_rel_s"),
        ("Chest", "mean_t_chest_peak_rel_s", "sd_t_chest_peak_rel_s"),
        ("Wrist", "mean_t_wrist_peak_rel_s", "sd_t_wrist_peak_rel_s"),
        ("Δt_TfC", "mean_dt_TfC_s", "sd_dt_TfC_s"),
        ("Δt_CW", "mean_dt_CW_s", "sd_dt_CW_s"),
        ("Δt_TfW", "mean_dt_TfW_s", "sd_dt_TfW_s"),
        ("Δt_WI", "mean_dt_WI_s", "sd_dt_WI_s"),
        ("Δt_CI", "mean_dt_CI_s", "sd_dt_CI_s"),
        ("Δt_TfI", "mean_dt_TfI_s", "sd_dt_TfI_s"),
    ]

    table_shape = slide.shapes.add_table(
        rows=len(rows) + 1,
        cols=3,
        left=Inches(0.9),
        top=Inches(1.85),
        width=Inches(11.55),
        height=Inches(3.2),
    )
    table = table_shape.table
    table.columns[0].width = Inches(2.0)
    table.columns[1].width = Inches(4.75)
    table.columns[2].width = Inches(4.75)

    headers = ["Item", "Mean", "SD"]
    for j, text in enumerate(headers):
        cell = table.cell(0, j)
        cell.text = text
        for para in cell.text_frame.paragraphs:
            para.alignment = PP_ALIGN.CENTER
            for run in para.runs:
                run.font.size = Pt(14)
                run.font.bold = True

    for i, (label, mean_col, sd_col) in enumerate(rows, start=1):
        vals = [label, _fmt_seconds_for_table(_summary_numeric_value(df_summary, mean_col)), _fmt_seconds_for_table(_summary_numeric_value(df_summary, sd_col))]
        for j, text in enumerate(vals):
            cell = table.cell(i, j)
            cell.text = text
            for para in cell.text_frame.paragraphs:
                para.alignment = PP_ALIGN.CENTER if j == 0 else PP_ALIGN.LEFT
                for run in para.runs:
                    run.font.size = Pt(13)
                    run.font.bold = (j == 0)

    note_box = slide.shapes.add_textbox(Inches(0.9), Inches(5.35), Inches(11.6), Inches(0.85))
    tf = note_box.text_frame
    tf.clear()
    p = tf.paragraphs[0]
    p.text = "Values are taken from the Summary sheet generated by this analysis."
    p.font.size = Pt(11)
    p.font.italic = True
    p.font.color.rgb = RGBColor(80, 80, 80)


def plot_dt_scatter_by_shot(df_per, out_png, total_shots, dt_col, title, ylabel, y_min, y_max):
    """横軸ショット番号、縦軸Δtの散布図を作成する。

    V2804:
      - 横軸は全ショット数Kに合わせて 1〜K で固定する
      - 選択したショット番号の位置だけに点を表示する
      - 点は線で結ばない
      - Δtが負の場合は、グラフ範囲外なので x軸上（y=0）に×マークを表示する
      - 平均値を実線、平均±SDを破線で表示する
    """
    shot = pd.to_numeric(df_per.get("shot_no"), errors="coerce")
    dt_s = pd.to_numeric(df_per.get(dt_col), errors="coerce")
    m = np.isfinite(shot) & np.isfinite(dt_s)

    shot_vals = shot[m].to_numpy(dtype=float)
    dt_ms = dt_s[m].to_numpy(dtype=float) * 1000.0

    valid_stats = dt_ms[np.isfinite(dt_ms)]
    if len(valid_stats) > 0:
        mean_ms = float(np.nanmean(valid_stats))
        sd_ms = float(np.nanstd(valid_stats, ddof=0)) if len(valid_stats) >= 2 else 0.0
    else:
        mean_ms = float("nan")
        sd_ms = float("nan")

    fig, ax = plt.subplots(figsize=(9.2, 5.2))

    if len(shot_vals) > 0:
        in_range = (dt_ms >= y_min) & (dt_ms <= y_max)
        neg_range = dt_ms < y_min
        high_range = dt_ms > y_max

        if np.any(in_range):
            ax.scatter(shot_vals[in_range], dt_ms[in_range], s=52, marker="o", label="Δt")
        if np.any(neg_range):
            ax.scatter(shot_vals[neg_range], np.full(np.sum(neg_range), y_min),
                       s=72, marker="x", linewidths=2.0, label="Δt < 0")
        if np.any(high_range):
            ax.scatter(shot_vals[high_range], np.full(np.sum(high_range), y_max),
                       s=72, marker="^", label=f"Δt > {y_max:.0f} ms")

    if np.isfinite(mean_ms):
        ax.axhline(mean_ms, lw=2.0, ls="-", color="k", label=f"Mean = {mean_ms:.1f} ms")
    if np.isfinite(mean_ms) and np.isfinite(sd_ms):
        ax.axhline(mean_ms + sd_ms, lw=1.6, ls="--", color="k", label=f"Mean + SD = {mean_ms + sd_ms:.1f} ms")
        ax.axhline(mean_ms - sd_ms, lw=1.6, ls="--", color="k", label=f"Mean - SD = {mean_ms - sd_ms:.1f} ms")

    ax.set_title(title)
    ax.set_xlabel("Shot number")
    ax.set_ylabel(ylabel)
    apply_axis_style(ax)

    # V2804: 横軸は解析対象だけでなく、抽出した全ショット数Kに合わせる。
    if total_shots and total_shots >= 1:
        ax.set_xlim(0.5, float(total_shots) + 0.5)
        if total_shots <= 40:
            ax.set_xticks(np.arange(1, int(total_shots) + 1))
        else:
            ax.set_xticks(np.linspace(1, int(total_shots), 10, dtype=int))
    elif len(shot_vals) > 0:
        ax.set_xlim(max(0.5, float(np.nanmin(shot_vals)) - 0.5), float(np.nanmax(shot_vals)) + 0.5)
        ax.set_xticks(shot_vals.astype(int))

    ax.set_ylim(float(y_min), float(y_max))
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(out_png, dpi=200)
    plt.close(fig)



def plot_dt_tfc_scatter(df_per, out_png, total_shots):
    plot_dt_scatter_by_shot(df_per, out_png, total_shots, "dt_TfC_s", "Δt_TfC by shot number", "Δt_TfC (ms)", 0.0, 1000.0)

def plot_dt_cw_scatter(df_per, out_png, total_shots):
    plot_dt_scatter_by_shot(df_per, out_png, total_shots, "dt_CW_s", "Δt_CW by shot number", "Δt_CW (ms)", 0.0, 500.0)

def plot_dt_wi_scatter(df_per, out_png, total_shots):
    plot_dt_scatter_by_shot(df_per, out_png, total_shots, "dt_WI_s", "Δt_WI by shot number", "Δt_WI (ms)", -300.0, 500.0)

def plot_dt_tfw_scatter(df_per, out_png, total_shots):
    plot_dt_scatter_by_shot(df_per, out_png, total_shots, "dt_TfW_s", "Δt_TfW by shot number", "Δt_TfW (ms)", 0.0, 1000.0)

def plot_dt_ci_scatter(df_per, out_png, total_shots):
    plot_dt_scatter_by_shot(df_per, out_png, total_shots, "dt_CI_s", "Δt_CI by shot number", "Δt_CI (ms)", 0.0, 1000.0)

def plot_dt_tfi_scatter(df_per, out_png, total_shots):
    plot_dt_scatter_by_shot(df_per, out_png, total_shots, "dt_TfI_s", "Δt_TfI by shot number", "Δt_TfI (ms)", 0.0, 1500.0)

def _add_single_image_slide(prs, image_path, zip_label, title):
    """1枚の画像を1スライドに大きく配置する。"""
    from pptx.util import Inches, Pt
    blank = prs.slide_layouts[6]
    slide = prs.slides.add_slide(blank)
    _add_small_zip_label(slide, zip_label)

    title_box = slide.shapes.add_textbox(Inches(0.45), Inches(0.23), Inches(12.0), Inches(0.32))
    tf = title_box.text_frame
    tf.clear()
    p = tf.paragraphs[0]
    p.text = title
    p.font.size = Pt(12)
    p.font.bold = True

    _add_image_fit(slide, image_path, Inches(0.75), Inches(0.75), Inches(11.85), Inches(6.35))

def create_summary_ppt(outdir, session_id, df_summary):
    """出力済みPNGをPowerPointにまとめ、概要スライド付きPPTXをoutdir内に保存する。"""
    _ensure_pptx_available()
    from pptx import Presentation
    from pptx.util import Inches

    prs = Presentation()
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)

    zip_label = f"{os.path.basename(outdir)}.zip"

    # V2806_0: 先頭に概要スライドを追加する。
    _add_overview_slide(prs, zip_label, df_summary)

    # V2806_0: 6本のΔt散布図をPPTへ追加する。
    scatter_specs = [
        ("R_dt_TfC_scatter.png", "Δt_TfC scatter by shot number"),
        ("R_dt_CW_scatter.png", "Δt_CW scatter by shot number"),
        ("R_dt_WI_scatter.png", "Δt_WI scatter by shot number"),
        ("R_dt_TfW_scatter.png", "Δt_TfW scatter by shot number"),
        ("R_dt_CI_scatter.png", "Δt_CI scatter by shot number"),
        ("R_dt_TfI_scatter.png", "Δt_TfI scatter by shot number"),
    ]
    for scatter_filename, scatter_title in scatter_specs:
        scatter_path = os.path.join(outdir, scatter_filename)
        if os.path.exists(scatter_path):
            _add_single_image_slide(prs, scatter_path, zip_label, scatter_title)
        else:
            print(f"[WARN] PPTに追加する散布図が見つかりません: {scatter_filename}", flush=True)

    # V2806_0: 絶対時間の確認図と、wide/zoom図をPPTに追加する。
    raw_check_pngs = _collect_numbered_pngs(outdir, "R_raw_impact_check")
    raw_all_pngs = _collect_numbered_pngs(outdir, "R_raw_all_check")
    waveform_wide_pngs = _collect_numbered_pngs(outdir, "R_waveform_wide")
    waveform_zoom_pngs = _collect_numbered_pngs(outdir, "R_waveform_zoom")

    _add_four_pngs_per_slide(prs, raw_check_pngs, zip_label, "R_raw_impact_check")
    _add_four_pngs_per_slide(prs, raw_all_pngs, zip_label, "R_raw_all_check")
    _add_four_pngs_per_slide(prs, waveform_wide_pngs, zip_label, "R_waveform_wide")
    _add_four_pngs_per_slide(prs, waveform_zoom_pngs, zip_label, "R_waveform_zoom")

    overlay_specs = [
        ("Trunk", "overlay_mean_Trunk.png", "mean_t_trunk_filtered_valley_rel_s", "sd_t_trunk_filtered_valley_rel_s"),
        ("Chest", "overlay_mean_Chest.png", "mean_t_chest_peak_rel_s", "sd_t_chest_peak_rel_s"),
        ("Wrist", "overlay_mean_Wrist.png", "mean_t_wrist_peak_rel_s", "sd_t_wrist_peak_rel_s"),
        ("Impact", "overlay_mean_Impact.png", "mean_t_impact_rel_s", "sd_t_impact_rel_s"),
    ]
    for label, filename, mean_col, sd_col in overlay_specs:
        img_path = os.path.join(outdir, filename)
        if os.path.exists(img_path):
            mean_sd_text = _summary_mean_sd_text(df_summary, mean_col, sd_col)
            _add_overlay_slide(prs, img_path, zip_label, f"overlay_mean_{label}", mean_sd_text)
        else:
            print(f"[WARN] PPTに追加する画像が見つかりません: {filename}", flush=True)

    pptx_path = os.path.join(outdir, f"R_V2806_0_{session_id}.pptx")
    prs.save(pptx_path)
    print(f"[DONE] PPT作成: {pptx_path}", flush=True)
    return pptx_path


def mean_var_sd_cv(series):
    s = pd.to_numeric(series, errors="coerce")
    s = s[np.isfinite(s)]
    if len(s) == 0:
        return float("nan"), float("nan"), float("nan"), float("nan")
    mean = float(s.mean())
    var = float(s.var(ddof=0)) if len(s) >= 2 else 0.0
    sd = float(np.sqrt(var))
    cv = float(sd / abs(mean)) if np.isfinite(mean) and mean != 0 else float("nan")
    return mean, var, sd, cv




def value_at_time(grid, y, t0):
    grid = np.asarray(grid, dtype=float)
    y = np.asarray(y, dtype=float)
    if (not np.isfinite(t0)) or len(grid) == 0 or len(y) == 0:
        return float("nan")
    idx = int(np.nanargmin(np.abs(grid - float(t0))))
    if 0 <= idx < len(y) and np.isfinite(y[idx]):
        return float(y[idx])
    return float("nan")


def main():
    csv_path = pick_csv_by_upload()
    session_id = extract_session_id(csv_path)

    df = read_v70(csv_path)
    t = df["time"].to_numpy()
    ax = df["ax"].to_numpy()
    ay = df["ay"].to_numpy()
    az = df["az"].to_numpy()

    Fs = estimate_Fs(t)
    print(f"[INFO] Fs ≈ {Fs:.2f} Hz")

    R = np.sqrt(ax**2 + ay**2 + az**2)

    # V2806_1: R capped at 3.5 g for the slow bands only (Trunk / Chest / Wrist).
    SLOW_CAP_G = 3.5
    g_unit = 9.80665 if np.nanmedian(R) > 3.0 else 1.0     # data in m/s^2 (rest ~9.8) or in g (rest ~1)
    R_slow = np.minimum(R, SLOW_CAP_G * g_unit)
    print(f"[INFO] V2806_1: R capped at {SLOW_CAP_G} g for Trunk/Chest/Wrist "
          f"({int(np.sum(R > SLOW_CAP_G * g_unit))} samples above the cap)")

    while True:
        sK = input("抽出したいインパクト数 K を入力してください: ").strip()
        if sK.isdigit() and int(sK) >= 1:
            K = int(sK)
            break

    bands = [
        ("Trunk",  1.0,  3.0,  "C0"),
        ("Chest",  2.5,  7.0,  "C1"),
        ("Wrist",  8.0, 15.0,  "C2"),
        ("Impact", 20.0, 40.0, "red"),
    ]
    print("[INFO] 各バンドのフィルタを実行中...")
    R_bands = []
    for lab, fL, fH, _ in bands:
        ord_use = 4
        print(f"  - {lab}: {fL}-{fH} Hz (order={ord_use})")
        R_in = R if lab == "Impact" else R_slow          # V2806_1: cap only for the slow bands
        R_bands.append(bandpass_butter(R_in, Fs, fL, fH, order=ord_use))

    # V2804: 全時間比較図用に、主要帯域の全時間波形も保持する。
    trunk_filtered_full = R_bands[0]
    chest_env_full = np.abs(hilbert(R_bands[1]))
    wrist_env_full = np.abs(hilbert(R_bands[2]))
    env_racket_full = np.abs(hilbert(R_bands[-1]))
    min_dist = int(0.30 * Fs)
    pk_env, _ = find_peaks(env_racket_full, distance=max(1, min_dist))

    if pk_env.size == 0:
        pk_raw, _ = find_peaks(R, distance=max(1, min_dist))
        if pk_raw.size == 0:
            raise SystemExit("ピークが見つからず終了。")
        candidate_idx = pk_raw
        candidate_score = R[pk_raw]
    else:
        candidate_idx = pk_env
        candidate_score = env_racket_full[pk_env]

    # V2804:
    # まずImpact envelope振幅の大きい順に上位K個を選び、
    # その後、解析用ショット番号は時間順に1〜Kへ並べ直す。
    # raw impact check図では、点線に振幅順位（1=最大）を表示する。
    order_desc = np.argsort(candidate_score)[::-1]
    n_pick = min(K, candidate_idx.size)
    top_idx_by_amp = np.asarray(candidate_idx[order_desc[:n_pick]], dtype=int)
    top_score_by_amp = np.asarray(candidate_score[order_desc[:n_pick]], dtype=float)
    impact_rank_by_idx = {int(idx): int(rank + 1) for rank, idx in enumerate(top_idx_by_amp)}
    impact_score_by_idx = {int(idx): float(score) for idx, score in zip(top_idx_by_amp, top_score_by_amp)}

    # 上位 K 個のインパクト候補を、時間順に並べ直す。
    # 以後のショット番号 1〜K は、この時間順番号として固定する。
    top_idx = np.array(sorted(top_idx_by_amp))
    K = len(top_idx)

    # V2804: Shimmer生データA列由来の時刻 t から、各インパクトの絶対時刻を取得する。
    # 時間情報は「測定開始からの絶対時間」のみに統一し、No.1からの相対時間は作成しない。
    if K >= 1:
        impact_time_abs_s_all = t[top_idx]
    else:
        impact_time_abs_s_all = np.array([], dtype=float)

    print(f"[INFO] 検出された {K} 個のインパクトを時間順に 1〜{K} と番号付けしました。")
    print("[INFO] 解析したい番号を 1,2,4 のように入力してください。空欄なら全部解析します。")

    while True:
        s_sel = input("解析したいショット番号（例 1,2,4／空=全部）: ").strip()
        if s_sel == "":
            sel_list = list(range(1, K + 1))
            break

        parts = []
        for chunk in s_sel.replace("、", ",").replace(";", ",").split(","):
            parts.extend(chunk.split())

        invalid = []
        sel_tmp = []
        for p in parts:
            if p.isdigit() and 1 <= int(p) <= K:
                sel_tmp.append(int(p))
            else:
                invalid.append(p)

        if invalid:
            print(f"[WARN] 無効な番号があります: {invalid}  有効範囲は 1〜{K} です。")
            continue
        if len(sel_tmp) == 0:
            print("[WARN] 解析対象が空です。例: 1,2,4")
            continue

        # 元の時間順番号を保持しつつ、解析順も時間順にする。
        sel_list = sorted(set(sel_tmp))
        break

    print(f"[INFO] 時間順 1〜{K} のうち、{sel_list} を解析します。")
    print("[INFO] shot_no_B、画像ファイル名、overlay中の番号は、元の時間順番号を保持します。")

    outdir = f"out_R5bands_V2806_1_cap35_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}_{session_id}"
    os.makedirs(outdir, exist_ok=True)

    # V2806_0修正:
    # 出力フォルダ名に HHMMSS まで入れているため、同じ日に同じCSVを複数回解析しても
    # 前回のPNG/PPTが残って混ざることを防ぐ。
    # 通信エラー確認用の全時間波形（R_raw_impact_check）は、1/0で作成有無を選択する。
    # 1 の場合は従来通り、測定開始から終了まで5秒ごとの全時間波形を作成する。
    # 0 の場合は作成をスキップし、PPTにも追加されない。
    make_raw_impact_check = ask_1_0(
        "全時間波形（R_raw_impact_check と R_raw_all_check）を作成しますか",
        default=1,
    )
    if make_raw_impact_check:
        plot_raw_impact_check_segments(
            t=t,
            R=R,
            impact_env=env_racket_full,
            top_idx=top_idx,
            impact_rank_by_idx=impact_rank_by_idx,
            outdir=outdir,
            segment_sec=5.0,
        )
        plot_raw_all_check_segments(
            t=t,
            R=R,
            impact_env=env_racket_full,
            trunk_filtered=trunk_filtered_full,
            chest_env=chest_env_full,
            wrist_env=wrist_env_full,
            top_idx=top_idx,
            impact_rank_by_idx=impact_rank_by_idx,
            outdir=outdir,
            segment_sec=5.0,
        )
    else:
        print("[INFO] R_raw_impact_check / R_raw_all_check の作成をスキップしました。", flush=True)

    grid = np.linspace(-3.0, 0.4, 2381)

    def window_interp(k_index, arr):
        left, right = 3.0, 0.4
        s = max(0, int(k_index - left * Fs))
        e = min(len(t), int(k_index + right * Fs))
        tt = t[s:e] - t[k_index]
        if len(tt) < 3:
            return np.full_like(grid, np.nan)
        return np.interp(grid, tt, arr[s:e], left=np.nan, right=np.nan)

    # V2806_0: overlay対象は最終イベント定義に合わせる。
    # TrunkはHilbert envelopeではなく、filtered波形そのものを保存する。
    envs_by_band = {"Trunk": [], "Chest": [], "Wrist": [], "Impact": []}
    tpeaks_by_band = {"Trunk": [], "Chest": [], "Wrist": [], "Impact": []}
    per_rows = []

    print(f"[INFO] 解析ショット数: {len(sel_list)}")
    for shot_no in sel_list:
        print(f"[INFO] shot {shot_no} を解析中...", flush=True)
        k = top_idx[shot_no - 1]

        env_list = []
        env_by_label = {}
        filt_by_label = {}

        for (lab, fL, fH, _col), R_band in zip(bands, R_bands):
            y_win = window_interp(k, R_band)
            filt_by_label[lab] = y_win
            env = compute_envelope(grid, y_win)
            env_by_label[lab] = env
            env_list.append(env)

        t_feature = {"Impact": 0.0}

        # V2806_0: Trunk envelope谷は最終版では使わない。
        # Trunkイベントは、下の Trunk filtered 1.0-3.0 Hz の負側谷 Tf として検出する。

        # V2804:
        # Trunk filtered 1.0–3.0 Hz の負側の谷を Tf として検出する。
        # 探索範囲は envelope 版 Trunk と同じ -1.2〜-0.2 s とする。
        t_trunk_filt_valley, trunk_filt_detected = compute_last_negative_valley_time(
            grid, filt_by_label.get("Trunk", np.full_like(grid, np.nan)),
            t_min=-1.2, t_max=-0.2, prom_ratio=0.10
        )
        t_feature["TrunkFiltered"] = float(t_trunk_filt_valley) if trunk_filt_detected else float("nan")

        t_chest_peak, chest_detected = compute_peak_time(
            grid, env_by_label.get("Chest", np.full_like(grid, np.nan)),
            t_min=-0.4, t_max=-0.05, mode="max"
        )
        t_feature["Chest"] = float(t_chest_peak) if chest_detected else float("nan")

        t_wrist_peak, wrist_detected = compute_peak_time(
            grid, env_by_label.get("Wrist", np.full_like(grid, np.nan)),
            t_min=-0.2, t_max=0.0, mode="last"
        )
        t_feature["Wrist"] = float(t_wrist_peak) if wrist_detected else float("nan")

        # V2806_0: overlay用には最終イベント定義の4波形だけ保存する。
        # TrunkはHilbert envelopeではなく、filtered波形そのものを使う。
        envs_by_band["Trunk"].append(filt_by_label.get("Trunk", np.full_like(grid, np.nan)))
        tpeaks_by_band["Trunk"].append(float(t_feature.get("TrunkFiltered", float("nan"))))
        envs_by_band["Chest"].append(env_by_label.get("Chest", np.full_like(grid, np.nan)))
        tpeaks_by_band["Chest"].append(float(t_feature.get("Chest", float("nan"))))
        envs_by_band["Wrist"].append(env_by_label.get("Wrist", np.full_like(grid, np.nan)))
        tpeaks_by_band["Wrist"].append(float(t_feature.get("Wrist", float("nan"))))
        envs_by_band["Impact"].append(env_by_label.get("Impact", np.full_like(grid, np.nan)))
        tpeaks_by_band["Impact"].append(float(t_feature.get("Impact", float("nan"))))

        impact_time_from_start_s = float(impact_time_abs_s_all[shot_no - 1]) if 0 <= shot_no - 1 < len(impact_time_abs_s_all) else float("nan")
        impact_amp_rank = impact_rank_by_idx.get(int(k), np.nan)
        impact_pick_score = impact_score_by_idx.get(int(k), float("nan"))
        print(f"[DEBUG] shot {shot_no}: absolute impact time from start = {impact_time_from_start_s:.4f} s")
        raw_R_win = window_interp(k, R)
        plot_waveform_wide(
            grid,
            raw_R_win,
            env_by_label.get("Impact", np.full_like(grid, np.nan)),
            filt_by_label.get("Trunk", np.full_like(grid, np.nan)),
            env_by_label.get("Chest", np.full_like(grid, np.nan)),
            env_by_label.get("Wrist", np.full_like(grid, np.nan)),
            t_feature,
            shot_no, impact_time_from_start_s,
            os.path.join(outdir, f"R_waveform_wide_{shot_no:02d}.png")
        )
        plot_waveform_zoom(
            grid,
            env_by_label.get("Impact", np.full_like(grid, np.nan)),
            filt_by_label.get("Trunk", np.full_like(grid, np.nan)),
            env_by_label.get("Chest", np.full_like(grid, np.nan)),
            env_by_label.get("Wrist", np.full_like(grid, np.nan)),
            t_feature,
            shot_no, impact_time_from_start_s,
            os.path.join(outdir, f"R_waveform_zoom_{shot_no:02d}.png")
        )

        # V2806_0:
        # 正式なTrunkイベントは、Hilbert envelopeではなく filtered波形の負側谷 Tf とする。
        t_tf = t_feature.get("TrunkFiltered", float("nan"))
        t_chest = t_feature["Chest"]
        t_wrist = t_feature["Wrist"]
        t_impact = 0.0

        h_tf = value_at_time(grid, filt_by_label.get("Trunk", np.full_like(grid, np.nan)), t_tf)
        h_chest = value_at_time(grid, env_by_label.get("Chest", np.full_like(grid, np.nan)), t_chest)
        h_wrist = value_at_time(grid, env_by_label.get("Wrist", np.full_like(grid, np.nan)), t_wrist)
        h_impact = value_at_time(grid, env_by_label.get("Impact", np.full_like(grid, np.nan)), t_impact)

        # 測定開始からの絶対時間。相対時間はImpactを0 sとしているため、絶対時間はImpact絶対時刻を足す。
        trunk_filtered_valley_time_from_start_s = impact_time_from_start_s + t_tf if np.isfinite(t_tf) and np.isfinite(impact_time_from_start_s) else float("nan")
        chest_peak_time_from_start_s = impact_time_from_start_s + t_chest if np.isfinite(t_chest) and np.isfinite(impact_time_from_start_s) else float("nan")
        wrist_peak_time_from_start_s = impact_time_from_start_s + t_wrist if np.isfinite(t_wrist) and np.isfinite(impact_time_from_start_s) else float("nan")

        dt_TfC = t_chest - t_tf if (np.isfinite(t_tf) and np.isfinite(t_chest)) else float("nan")
        dt_CW = t_wrist - t_chest if (np.isfinite(t_chest) and np.isfinite(t_wrist)) else float("nan")
        dt_WI = t_impact - t_wrist if np.isfinite(t_wrist) else float("nan")
        dt_TfW = t_wrist - t_tf if (np.isfinite(t_tf) and np.isfinite(t_wrist)) else float("nan")
        dt_CI = t_impact - t_chest if np.isfinite(t_chest) else float("nan")
        dt_TfI = t_impact - t_tf if np.isfinite(t_tf) else float("nan")

        per_rows.append({
            "impact_time_from_start_s": impact_time_from_start_s,
            "trunk_filtered_valley_time_from_start_s": trunk_filtered_valley_time_from_start_s,
            "chest_peak_time_from_start_s": chest_peak_time_from_start_s,
            "wrist_peak_time_from_start_s": wrist_peak_time_from_start_s,
            "impact_amp_rank": impact_amp_rank,
            "impact_pick_score": impact_pick_score,
            "session_id": session_id,
            "shot_no": int(shot_no),
            "t_impact_rel_s": t_impact,
            "t_trunk_filtered_valley_rel_s": t_tf,
            "t_chest_peak_rel_s": t_chest,
            "t_wrist_peak_rel_s": t_wrist,
            "trunk_filtered_valley_value": h_tf,
            "chest_peak_value": h_chest,
            "wrist_peak_value": h_wrist,
            "impact_peak_value": h_impact,
            "dt_TfC_s": dt_TfC,
            "dt_CW_s": dt_CW,
            "dt_WI_s": dt_WI,
            "dt_TfW_s": dt_TfW,
            "dt_CI_s": dt_CI,
            "dt_TfI_s": dt_TfI,
        })

    # V2806_0: 最終イベント定義に合わせ、Trunk(filtered), Chest, Wrist, Impact をoverlayする。
    overlay_bands = [
        ("Trunk", 1.0, 3.0, "purple"),
        ("Chest", 2.5, 7.0, "orange"),
        ("Wrist", 8.0, 15.0, "green"),
        ("Impact", 20.0, 40.0, "red"),
    ]
    plot_overlaid_shots(grid, envs_by_band, tpeaks_by_band, overlay_bands, os.path.join(outdir, "overlay_mean"), label_numbers=sel_list)

    df_per = pd.DataFrame(per_rows)
    n_total = int(len(df_per))

    trunk_filtered_detect_rate = float(pd.to_numeric(df_per["t_trunk_filtered_valley_rel_s"], errors="coerce").notna().mean()) if n_total > 0 else float("nan")
    chest_detect_rate = float(pd.to_numeric(df_per["t_chest_peak_rel_s"], errors="coerce").notna().mean()) if n_total > 0 else float("nan")
    wrist_detect_rate = float(pd.to_numeric(df_per["t_wrist_peak_rel_s"], errors="coerce").notna().mean()) if n_total > 0 else float("nan")

    mtf, vtf, sdtf, cvtf = mean_var_sd_cv(df_per["t_trunk_filtered_valley_rel_s"])
    mc, vc, sdc, cvc = mean_var_sd_cv(df_per["t_chest_peak_rel_s"])
    mw, vw, sdw, cvw = mean_var_sd_cv(df_per["t_wrist_peak_rel_s"])
    mi, vi, sdi, cvi = mean_var_sd_cv(df_per["t_impact_rel_s"])

    mhTf, _vhTf, sdhTf, _cvhTf = mean_var_sd_cv(df_per["trunk_filtered_valley_value"])
    mhC, _vhC, sdhC, _cvhC = mean_var_sd_cv(df_per["chest_peak_value"])
    mhW, _vhW, sdhW, _cvhW = mean_var_sd_cv(df_per["wrist_peak_value"])
    mhI, _vhI, sdhI, _cvhI = mean_var_sd_cv(df_per["impact_peak_value"])

    mTfC, vTfC, sdTfC, cvTfC = mean_var_sd_cv(df_per["dt_TfC_s"])
    mCW, vCW, sdCW, cvCW = mean_var_sd_cv(df_per["dt_CW_s"])
    mWI, vWI, sdWI, cvWI = mean_var_sd_cv(df_per["dt_WI_s"])
    mTfW, vTfW, sdTfW, cvTfW = mean_var_sd_cv(df_per["dt_TfW_s"])
    mCI, vCI, sdCI, cvCI = mean_var_sd_cv(df_per["dt_CI_s"])
    mTfI, vTfI, sdTfI, cvTfI = mean_var_sd_cv(df_per["dt_TfI_s"])

    df_summary = pd.DataFrame([{
        "session_id": session_id,
        "n_total": n_total,

        "trunk_filtered_valley_detect_rate": trunk_filtered_detect_rate,
        "mean_t_trunk_filtered_valley_rel_s": mtf,
        "var_t_trunk_filtered_valley_rel_s": vtf,
        "sd_t_trunk_filtered_valley_rel_s": sdtf,
        "cv_t_trunk_filtered_valley_rel_s": cvtf,
        "mean_trunk_filtered_valley_value": mhTf,
        "sd_trunk_filtered_valley_value": sdhTf,

        "chest_detect_rate": chest_detect_rate,
        "mean_t_chest_peak_rel_s": mc,
        "var_t_chest_peak_rel_s": vc,
        "sd_t_chest_peak_rel_s": sdc,
        "cv_t_chest_peak_rel_s": cvc,
        "mean_chest_peak_value": mhC,
        "sd_chest_peak_value": sdhC,

        "wrist_detect_rate": wrist_detect_rate,
        "mean_t_wrist_peak_rel_s": mw,
        "var_t_wrist_peak_rel_s": vw,
        "sd_t_wrist_peak_rel_s": sdw,
        "cv_t_wrist_peak_rel_s": cvw,
        "mean_wrist_peak_value": mhW,
        "sd_wrist_peak_value": sdhW,

        "mean_t_impact_rel_s": mi,
        "var_t_impact_rel_s": vi,
        "sd_t_impact_rel_s": sdi,
        "cv_t_impact_rel_s": cvi,
        "mean_impact_peak_value": mhI,
        "sd_impact_peak_value": sdhI,

        "mean_dt_TfC_s": mTfC,
        "var_dt_TfC_s": vTfC,
        "sd_dt_TfC_s": sdTfC,
        "cv_dt_TfC_s": cvTfC,

        "mean_dt_CW_s": mCW,
        "var_dt_CW_s": vCW,
        "sd_dt_CW_s": sdCW,
        "cv_dt_CW_s": cvCW,

        "mean_dt_WI_s": mWI,
        "var_dt_WI_s": vWI,
        "sd_dt_WI_s": sdWI,
        "cv_dt_WI_s": cvWI,

        "mean_dt_TfW_s": mTfW,
        "var_dt_TfW_s": vTfW,
        "sd_dt_TfW_s": sdTfW,
        "cv_dt_TfW_s": cvTfW,

        "mean_dt_CI_s": mCI,
        "var_dt_CI_s": vCI,
        "sd_dt_CI_s": sdCI,
        "cv_dt_CI_s": cvCI,

        "mean_dt_TfI_s": mTfI,
        "var_dt_TfI_s": vTfI,
        "sd_dt_TfI_s": sdTfI,
        "cv_dt_TfI_s": cvTfI,
    }])

    # V2806_0: 4イベント Tf, C, W, I から作る6本のΔt散布図を作成する。
    # 主指標は dt_TfC_s, dt_CW_s, dt_TfW_s。
    # Impact飽和の影響を受ける dt_WI_s, dt_CI_s, dt_TfI_s も参考指標として保存する。
    plot_dt_tfc_scatter(df_per, os.path.join(outdir, "R_dt_TfC_scatter.png"), total_shots=K)
    plot_dt_cw_scatter(df_per, os.path.join(outdir, "R_dt_CW_scatter.png"), total_shots=K)
    plot_dt_wi_scatter(df_per, os.path.join(outdir, "R_dt_WI_scatter.png"), total_shots=K)
    plot_dt_tfw_scatter(df_per, os.path.join(outdir, "R_dt_TfW_scatter.png"), total_shots=K)
    plot_dt_ci_scatter(df_per, os.path.join(outdir, "R_dt_CI_scatter.png"), total_shots=K)
    plot_dt_tfi_scatter(df_per, os.path.join(outdir, "R_dt_TfI_scatter.png"), total_shots=K)

    # V2806_0: per_shotには、絶対時間・相対時間・Δt指標を整理して保存する。
    df_per_excel = df_per

    # 全計測時間について、R_raw_all_check に表示している5波形をExcelへ保存する。
    # 横軸は測定開始からの時間[s]で、各行は同一時刻の波形値に対応する。
    #   Composite_R : フィルタ前の合成加速度R
    #   Trunk       : 1.0-3.0 Hz filtered波形（Hilbert包絡なし）
    #   Chest       : 2.5-7.0 Hz Hilbert包絡
    #   Wrist       : 8.0-15.0 Hz Hilbert包絡
    #   Impact      : 20.0-40.0 Hz Hilbert包絡
    df_all_time_waveforms = pd.DataFrame({
        "time_from_measurement_start_s": t,
        "Composite_R_g": R,
        "Trunk_filtered_1_3Hz_g": trunk_filtered_full,
        "Chest_envelope_2p5_7Hz_g": chest_env_full,
        "Wrist_envelope_8_15Hz_g": wrist_env_full,
        "Impact_envelope_20_40Hz_g": env_racket_full,
    })

    # 【追加】all_time_waveformsを、結果Excel内のシートだけでなく、
    # 独立したExcelファイルとしても保存する。
    # 独立ファイルは出力フォルダ内に置くため、「全部入りZIP」に自動で含まれる。
    excel_path = os.path.join(outdir, f"R_V2806_0_results_{session_id}.xlsx")
    all_time_waveforms_path = os.path.join(
        outdir, f"all_time_waveforms_{session_id}.xlsx"
    )

    with pd.ExcelWriter(excel_path, engine="openpyxl") as writer:
        df_per_excel.to_excel(writer, index=False, sheet_name="per_shot")
        df_summary.to_excel(writer, index=False, sheet_name="summary")
        df_all_time_waveforms.to_excel(
            writer, index=False, sheet_name="all_time_waveforms"
        )

    # 【追加】ZIP内で見つけやすいように、all_time_waveformsを独立ファイルでも出力する。
    with pd.ExcelWriter(all_time_waveforms_path, engine="openpyxl") as writer:
        df_all_time_waveforms.to_excel(
            writer, index=False, sheet_name="all_time_waveforms"
        )
    print(f"[DONE] 全時間5波形Excel作成: {all_time_waveforms_path}", flush=True)

    # V2801S_PPT3: PNGをPowerPointにまとめ、PPTXをoutdir内に保存してからZIP化する。
    pptx_path = create_summary_ppt(outdir, session_id, df_summary)

    # V2806_0: ZIPに入れる内容を選択する。
    # 1=従来通りすべての出力ファイルをZIP化、0=PPTXとExcelのみをZIP化。
    include_all_files_in_zip = ask_zip_content_mode(default=1)

    zip_path = f"{outdir}.zip"
    if include_all_files_in_zip:
        shutil.make_archive(outdir, "zip", outdir)
        print(f"[DONE] ZIP作成（全部入り）: {zip_path}")
    else:
        zip_base = outdir + "_PPT_Excel_only"
        zip_path = f"{zip_base}.zip"
        zip_tmp_dir = zip_base
        if os.path.exists(zip_tmp_dir):
            shutil.rmtree(zip_tmp_dir)
        os.makedirs(zip_tmp_dir, exist_ok=True)
        # 【追加】PPTとExcelのみを選んだ場合にも、独立した
        # all_time_waveforms_*.xlsx を必ずZIPへ含める。
        for src_path in [pptx_path, excel_path, all_time_waveforms_path]:
            if src_path and os.path.exists(src_path):
                shutil.copy2(src_path, os.path.join(zip_tmp_dir, os.path.basename(src_path)))
            else:
                print(f"[WARN] ZIPに入れるファイルが見つかりません: {src_path}", flush=True)
        shutil.make_archive(zip_base, "zip", zip_tmp_dir)
        shutil.rmtree(zip_tmp_dir)
        print(f"[DONE] ZIP作成（PPTとExcelのみ）: {zip_path}")

    if in_colab():
        from google.colab import files  # type: ignore
        files.download(zip_path)


if __name__ == "__main__":
    main()
