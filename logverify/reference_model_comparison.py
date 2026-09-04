"""12.18節: 「多数のログを比較する」のではなく「1本のログを、あらかじめ
用意した基準（安全な振る舞い）と比較して抽象化する」という、ユーザー提案の
2つの方法のプロトタイプ実装。

## 経緯

12.16/12.17節の方法は、多数のログ（衝突／非衝突）を集め、その分布から
Z3でしきい値を統計的に合成するというものだった。12.17節で分かったように、
この方法は分離できるかどうかが手元のログの量と代表性に強く依存する。

これに対しユーザーから、根本的に異なる2つの方針が提案された:

1. **C&Cドライバモデルによる基準ログとの比較。** 多数のログを比較するの
   ではなく、1本のログを分析するために抽象化してはどうか。比較対象と
   なる「安全な振る舞いをするログ」を、ドライバモデル（例:
   Intelligent Driver Model等の追従モデル）であらかじめ生成しておき、
   それとの違いがわかる抽象化をすれば、自動抽象化ができるのではないか。

2. **TTC等のcriticality metricsによる抽象化。** TTC(Time To Collision)
   等の既存のcriticality metricsを、あらかじめ一定間隔で抽象化しておき、
   それに基づいてログを抽象化する。

本節はこの2つを、既知の衝突ログ0067を題材にプロトタイプとして実装し、
12.14〜12.17節の方法との違いを検証する。

## 方法1: TTCによる抽象化

TTC = 縦方向の残り距離 / 縦方向の接近速度（接近速度が正のときのみ定義。
接近していない、またはすでに接触範囲にある場合は特別扱い）を、フレーム
ごとに計算し、業界で広く使われる目安（TTC<1.5秒: 危険、1.5〜3秒: 注意、
3秒以上: 安全）で抽象化する。この抽象値は、他のログとの比較を一切必要と
せず、この1本のログの時間発展だけから直接求まる。

## 方法2: ドライバモデル（IDM）による基準との比較

Intelligent Driver Model (IDM, Treiber et al. 2000)を「安全な基準
ドライバ」のモデルとして採用する。IDMは、追従車両の速度v・先行車両との
車間距離s・相対速度(接近速度)dvから、その状況で「標準的な追従ドライバ」
が出す加速度a_IDMを計算する、確立された交通流モデルである
（本プロジェクトの`required_deceleration_magnitude`(v^2/(2d)の物理限界)
よりも、実際のドライバの快適な追従挙動に近い基準を与える）。

    s*(v, dv) = s0 + max(0, v*T + v*dv / (2*sqrt(a_max*b)))
    a_IDM = a_max * (1 - (v/v0)^delta - (s*/s)^2)

各フレームについて、実際にEgoが達成した加速度と、IDMが「安全な基準
ドライバならこうする」と計算した加速度a_IDMの差（actual - a_IDM）を
求める。この差が大きく負であるほど「基準よりも弱い」ということになり、
これが12.15節の`classify_deceleration_adequacy`に代わる、比較対象の
ログを必要としない抽象化になる。

**12.16/12.17節との関係。** どちらも「複数ログの統計的分布」ではなく
「1本のログ＋あらかじめ用意した基準（物理法則・標準ドライバモデル）」
から抽象化を導く点で共通しており、12.17節で明らかになった
「分離可能性がデータ量に依存する」という問題を原理的に回避できる。
一方で、基準そのもの（driver modelのパラメータ、TTCの閾値）が
「恣意的に選んだものではないか」という、12.9節以来プロジェクトが
警戒してきた問題を、driver modelやTTCという既存の確立された基準に
委ねることで軽減している（ただしゼロにはならない——IDMのパラメータ
a_max, b, T, s0, delta自体は依然として選択の余地がある）。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.reference_model_comparison
"""

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from logverify.synth_thresholds_multilog import (
    _load, vehicle_sizes, relative_xy, closest_approach_frame, cutin_onset_frame,
)

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_TTC = "out_gif/reference_ttc_abstraction.png"
OUT_IDM = "out_gif/reference_idm_comparison.png"

# TTC zone thresholds (widely used rule-of-thumb values, e.g. forward
# collision warning literature): TTC<1.5s = danger, 1.5-3s = caution,
# >=3s = safe.
TTC_DANGER = 1.5
TTC_CAUTION = 3.0

# IDM parameters (typical comfortable-driving values from Treiber et al.)
IDM_A_MAX = 1.0    # m/s^2, max acceleration
IDM_B = 1.8         # m/s^2, comfortable braking deceleration
IDM_T = 1.5         # s, desired time headway
IDM_S0 = 2.0        # m, minimum gap
IDM_DELTA = 4.0


def ego_speed_series(gk):
    return [math.hypot(rec["groundtruth_ego"]["twist"]["linear"]["x"],
                        rec["groundtruth_ego"]["twist"]["linear"]["y"]) for rec in gk]


def actual_accel_series(gk):
    return [rec["groundtruth_ego"]["acceleration"]["linear"]["x"] for rec in gk]


def compute_ttc(rxs, timestamps, eh_l, nh_l, near_rx=40.0):
    """フレームごとのTTC（正: 接近中、None: 定義不能/接近していない）。"""
    n = len(rxs)
    ttcs = [None] * n
    for i in range(n - 1):
        rx = rxs[i]
        if rx is None or abs(rx) > near_rx:
            continue
        dt = timestamps[i + 1] - timestamps[i]
        if dt <= 0 or rxs[i + 1] is None:
            continue
        closing = (rxs[i] - rxs[i + 1]) / dt
        dist = rx - (eh_l + nh_l)
        if closing > 0.01 and dist > 0:
            ttcs[i] = dist / closing
    return ttcs


def ttc_zone(ttc):
    if ttc is None:
        return "safe"
    if ttc < TTC_DANGER:
        return "danger"
    if ttc < TTC_CAUTION:
        return "caution"
    return "safe"


def plot_ttc_abstraction(gk, ttcs, onset_frame, closest_frame, output_path=OUT_TTC):
    ts = [rec["timestamp"] - gk[0]["timestamp"] for rec in gk]
    t0 = ts[onset_frame]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True,
                                    gridspec_kw={"height_ratios": [3, 1]})

    plot_ts = [t - t0 for t in ts]
    finite = [(t, v) for t, v in zip(plot_ts, ttcs) if v is not None and v < 30]
    if finite:
        xs, ys = zip(*finite)
        ax1.plot(xs, ys, color="#1565c0", linewidth=1.4, zorder=3)
    ax1.axhspan(0, TTC_DANGER, color="#e53935", alpha=0.15, zorder=1)
    ax1.axhspan(TTC_DANGER, TTC_CAUTION, color="#fb8c00", alpha=0.15, zorder=1)
    ax1.axhspan(TTC_CAUTION, 30, color="#43a047", alpha=0.08, zorder=1)
    ax1.axhline(TTC_DANGER, color="#e53935", linestyle="--", linewidth=0.8)
    ax1.axhline(TTC_CAUTION, color="#fb8c00", linestyle="--", linewidth=0.8)
    ax1.text(plot_ts[0], TTC_DANGER, " danger < 1.5s", color="#c62828", fontsize=8, va="bottom")
    ax1.text(plot_ts[0], TTC_CAUTION, " caution 1.5-3.0s", color="#e65100", fontsize=8, va="bottom")
    ax1.axvline(plot_ts[closest_frame], color="black", linestyle=":", linewidth=1.2)
    ax1.text(plot_ts[closest_frame], 28, "最近接/衝突", fontsize=8, ha="right")
    ax1.set_ylim(0, 30)
    ax1.set_ylabel("TTC (s)")
    ax1.set_title("方法1: TTCによる1ログ単独の抽象化（ログ0067、他ログとの比較不要）", fontsize=11)

    zones = [ttc_zone(v) for v in ttcs]
    zone_color = {"safe": "#43a047", "caution": "#fb8c00", "danger": "#e53935"}
    for i in range(len(plot_ts) - 1):
        c = zone_color[zones[i]]
        ax2.axvspan(plot_ts[i], plot_ts[i + 1], color=c, alpha=0.85, linewidth=0)
    ax2.set_yticks([])
    ax2.set_xlabel("カットイン開始からの経過時間 (s)")
    ax2.set_ylabel("抽象値")

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


# A car's own physical braking limit (roughly -1g), used only to clip
# IDM's (s*/s)^2 term, which -- like this project's v^2/(2d) formula in
# Sections 12.12/12.13 -- diverges as the gap s approaches 0. Rather than
# hide this, Section 12.18 reports it as further confirmation that any
# single-frame, distance-based instantaneous formula is unstable near
# contact, independent of whether it comes from basic kinematics or from
# a car-following driver model.
PHYSICAL_BRAKING_LIMIT = -9.0


def idm_accel(v, s, dv, v0, clip=True):
    s_gap = max(s, 0.1)
    s_star = IDM_S0 + max(0.0, v * IDM_T + (v * dv) / (2.0 * math.sqrt(IDM_A_MAX * IDM_B)))
    raw = IDM_A_MAX * (1.0 - (v / max(v0, 0.1)) ** IDM_DELTA - (s_star / s_gap) ** 2)
    if clip:
        return max(raw, PHYSICAL_BRAKING_LIMIT)
    return raw


def plot_idm_comparison(gk, rxs, ego_speed, actual_accel, eh_l, nh_l, onset_frame,
                         closest_frame, output_path=OUT_IDM):
    ts = [rec["timestamp"] - gk[0]["timestamp"] for rec in gk]
    dt_arr = np.diff(ts)
    n = len(gk)

    # v0 (desired/free-flow speed) = ego's own cruising speed just before
    # the NPC's cut-in began.
    v0 = float(np.mean(ego_speed[max(0, onset_frame - 50):onset_frame + 1]))

    a_idm = [None] * n
    for i in range(n - 1):
        rx = rxs[i]
        if rx is None:
            continue
        dt = ts[i + 1] - ts[i]
        if dt <= 0 or rxs[i + 1] is None:
            continue
        dv = (rxs[i] - rxs[i + 1]) / dt  # positive when closing
        s = rx - (eh_l + nh_l)
        if s <= 0:
            continue
        a_idm[i] = idm_accel(ego_speed[i], s, dv, v0)

    t0 = ts[onset_frame]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True,
                                    gridspec_kw={"height_ratios": [2, 1]})

    plot_ts = [t - t0 for t in ts]
    win = slice(max(0, onset_frame - 20), min(n, closest_frame + 20))
    ax1.plot(plot_ts[win], actual_accel[win], color="#e53935", linewidth=1.6,
              label="実際のEgo加速度 (achieved)")
    idm_xy = [(plot_ts[i], a_idm[i]) for i in range(win.start, win.stop) if a_idm[i] is not None]
    if idm_xy:
        xs, ys = zip(*idm_xy)
        ax1.plot(xs, ys, color="#1565c0", linewidth=1.6, linestyle="--",
                  label="IDM基準ドライバの加速度 (reference)")
    ax1.axhline(0, color="#9e9e9e", linewidth=0.6)
    ax1.axvline(plot_ts[closest_frame], color="black", linestyle=":", linewidth=1.2)
    ax1.set_ylabel("縦方向加速度 (m/s^2)")
    ax1.legend(loc="lower left", fontsize=9)
    ax1.set_title("方法2: IDM基準ドライバとの比較による1ログ単独の抽象化（ログ0067）", fontsize=11)

    # Express the comparison as achieved/required MAGNITUDE ratio, in the
    # same convention as classify_deceleration_adequacy (Section 12.15) --
    # not a raw difference, whose sign is easy to misread once a_idm is
    # itself negative (a braking recommendation).
    ratio = [None] * n
    for i in range(win.start, win.stop):
        if a_idm[i] is not None and a_idm[i] < 0:
            required_mag = abs(a_idm[i])
            achieved_mag = max(0.0, -actual_accel[i])
            ratio[i] = achieved_mag / required_mag if required_mag > 0 else None
    ratio_xy = [(plot_ts[i], ratio[i]) for i in range(win.start, win.stop) if ratio[i] is not None]
    if ratio_xy:
        xs, ys = zip(*ratio_xy)
        colors = ["#e53935" if y < 0.5 else ("#fb8c00" if y < 1.0 else "#43a047") for y in ys]
        ax2.bar(xs, ys, width=0.08, color=colors)
    ax2.axhline(1.0, color="black", linewidth=0.8, linestyle=":")
    ax2.set_ylabel("achieved/required\n(IDM基準比、<1=弱い)")
    ax2.set_xlabel("カットイン開始からの経過時間 (s)")

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def run():
    data = _load(LOG_PATH)
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)
    timestamps = [rec["timestamp"] for rec in gk]

    closest_frame, risk = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    onset_frame = cutin_onset_frame(rys, closest_frame)
    print(f"onset_frame={onset_frame}, closest_frame={closest_frame}, risk={risk}")

    print("=== 方法1: TTCによる抽象化 ===")
    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)
    zones = [ttc_zone(v) for v in ttcs[onset_frame:closest_frame + 1]]
    from collections import Counter
    print("カットイン区間中のTTCゾーン分布:", Counter(zones))
    path1 = plot_ttc_abstraction(gk, ttcs, onset_frame, closest_frame)
    print(f"図を書き出しました: {path1}")
    print()

    print("=== 方法2: IDM基準ドライバとの比較 ===")
    ego_speed = ego_speed_series(gk)
    actual_accel = actual_accel_series(gk)

    v0 = float(np.mean(ego_speed[max(0, onset_frame - 50):onset_frame + 1]))
    raw_min = 0.0
    for i in range(onset_frame, closest_frame):
        rx = rxs[i]
        if rx is None or rxs[i + 1] is None:
            continue
        dt = timestamps[i + 1] - timestamps[i]
        if dt <= 0:
            continue
        dv = (rxs[i] - rxs[i + 1]) / dt
        s = rx - (eh_l + nh_l)
        if s <= 0:
            continue
        raw = idm_accel(ego_speed[i], s, dv, v0, clip=False)
        raw_min = min(raw_min, raw)
    print(f"クリップ前のIDM加速度の最小値（衝突直前）: {raw_min:.1f} m/s^2 "
          f"(物理的なブレーキ限界 {PHYSICAL_BRAKING_LIMIT} m/s^2 でクリップして描画)")

    path2 = plot_idm_comparison(gk, rxs, ego_speed, actual_accel, eh_l, nh_l,
                                 onset_frame, closest_frame)
    print(f"図を書き出しました: {path2}")


if __name__ == "__main__":
    run()
