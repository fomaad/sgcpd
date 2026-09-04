"""12.19節: JAMA「自動運転の安全性評価フレームワーク」が定義する
「Competent and Careful（有能で慎重な）人間ドライバモデル」（略称C&Cモデル）
を、12.18節のIDMの代わりに採用したプロトタイプ実装。

## 経緯

12.18節では、ユーザー提案の「ドライバモデルによる安全な基準ログとの比較」
を、一般的な交通流モデルであるIDMで具体化した。これに対しユーザーから
「C&Cドライバモデルは JAMA のものをつかってもらえますか？」との指定が
あった。調査の結果、「C&C」は日本自動車工業会(JAMA)が「自動運転の
安全性評価フレームワーク」(Automated Driving Safety Evaluation
Framework)で定義する **Competent and Careful Human Driver Model**
（有能で慎重な人間ドライバモデル）の略称であることが分かった
(JAMA, Automated Driving Safety Evaluation Framework Ver.2.0, Section
2.3.3.1)。これはIDMのような汎用の交通流モデルとは異なり、**自動運転
システムが安全とみなされるために超えるべき最低限の基準そのもの**として
JAMA自身が策定した、公式に確立された基準である——ユーザーが12.18節で
「安全な振る舞いをするログをあらかじめ作成しておいて、それと比較する」
と提案した際に念頭にあったのは、まさにこのモデルだったと考えられる。

## JAMA C&Cモデルの定義（フレームワークVer.2.0, Section 2.3.3.1）

- **知覚反応時間 (perception response time): 0.75秒。** 有能で慎重な
  ドライバがリスクを知覚してから、制動力が立ち上がるまでの遅れ。
- **最大減速度到達時間: 0.6秒。** 制動力立ち上がりから最大減速度に
  達するまでの時間。
- **最大減速度: 0.774G**（日本の教習データおよびNHTSA統計に基づく）。
- **カットインシナリオでの横方向のリスク知覚境界: 1.8 (m/s、NPCの
  最大横速度) × 0.4秒 (リスク知覚時間) = 0.72m。** すなわち、NPCが
  接触境界まで残り0.72mに達した時点で、有能で慎重なドライバはリスクを
  知覚する、と定義される。
- **縦方向のリスク知覚境界: TTC = 2.0秒**（UN規則のガイドラインに基づく）。

本プロジェクトでは、この2つの基準（横方向0.72m、縦方向TTC2.0秒）の
**いずれか早い方**でリスクが知覚されると解釈した（フレームワーク文書
自体は両基準の組み合わせ方を明示していないが、「同じ境界条件を異なる
次元で表現したもの」という記述と整合する自然な解釈である）。

減速度の時間波形（立ち上がり0.6秒間の形状）はフレームワーク文書でも
明示されていないため、本プロトタイプでは最も単純な線形立ち上がりを
仮定した（0.75秒の知覚反応時間の後、0.6秒かけて0から0.774Gへ線形に
増加し、以降0.774Gを維持）。

## 「予防可能性(preventable)」の判定 —— JAMAフレームワーク自身の方法

フレームワークは「このモデルをシミュレーションプログラムに実装し、
有能で慎重な人間ドライバにとって実際に回避可能な範囲を導出することで、
安全基準を定義できる」(Section 2.3.3.1)と述べている。本節はこれを
そのまま実行した: リスク知覚フレームでの実ログのEgo速度・位置を初期値
として、C&Cモデルの減速度プロファイルに従うEgoの反実仮想（カウンター
ファクチュアル）軌道をシミュレートし、NPCの実際の軌道（Egoの挙動には
依存しないと仮定）との間で接触が起きるかどうかを判定する。接触しなければ
「このシナリオは有能で慎重な人間ドライバなら回避できた（preventable）」
——すなわち、実際のAutowareの挙動がこの基準を下回っていた、ということが
直接分かる。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.jama_cc_model
"""

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from logverify.synth_thresholds_multilog import (
    _load, vehicle_sizes, relative_xy, closest_approach_frame,
)
from logverify.reference_model_comparison import compute_ttc, ego_speed_series

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_PATH = "out_gif/jama_cc_model_comparison.png"

# JAMA Automated Driving Safety Evaluation Framework Ver.2.0, Section 2.3.3.1
CC_PERCEPTION_RESPONSE_TIME = 0.75    # s, risk perceived -> brake force onset
CC_TIME_TO_MAX_DECEL = 0.6            # s, brake onset -> max deceleration
CC_MAX_DECEL_G = 0.774                # G
CC_MAX_DECEL = CC_MAX_DECEL_G * 9.81  # m/s^2
CC_LATERAL_RISK_BOUNDARY = 1.8 * 0.4  # m = 0.72m (max lateral speed x risk perception time)
CC_TTC_RISK_BOUNDARY = 2.0            # s (UNR guideline)


def cc_deceleration_at(t, t_risk):
    """C&Cモデルによる、時刻tでの減速度の大きさ（m/s^2、非負）。"""
    t_brake_onset = t_risk + CC_PERCEPTION_RESPONSE_TIME
    if t < t_brake_onset:
        return 0.0
    ramp_t = t - t_brake_onset
    if ramp_t >= CC_TIME_TO_MAX_DECEL:
        return CC_MAX_DECEL
    return CC_MAX_DECEL * (ramp_t / CC_TIME_TO_MAX_DECEL)


def _first_persistent_trigger(n, predicate, persist_frames):
    """`predicate(i)`が`persist_frames`フレーム連続で真になった、その
    連続区間の最初のフレームを返す（見つからなければNone）。

    12.24節: AJISAI cut-in全94本のうち5本を抜き出して分析したところ、
    ログ0002で単発フレーム(471フレーム目)だけTTCが瞬間的に0.32秒まで
    落ち込む（前後のフレームは7.6秒・5.0秒）ノイズが見つかり、これが
    risk知覚フレームとして採用されてしまい、実際のカットイン（1063
    フレーム目）より592フレームも早い、無関係な地点から表示ウィンドウが
    始まってしまう不具合があった。相対速度の有限差分によるTTC計算は、
    単一フレームのノイズに対して脆弱である——この関数は、12.10節の
    ヒステリシス（格子状態の圧縮でノイズによる見せかけの分岐を吸収する
    考え方）と同じ発想を、risk知覚判定に適用したもの。

    ---
    English:
    Returns the first frame of the run in which `predicate(i)` holds for
    `persist_frames` consecutive frames (None if no such run exists).

    Section 12.24: analyzing 5 of the 94 AJISAI cut-in logs surfaced a
    case (log 0002) where TTC dipped to 0.32s for a single frame (471)
    -- pure noise, since the neighboring frames read 7.6s and 5.0s --
    and that single frame was accepted as the risk-perceived frame,
    starting the display window 592 frames before the actual cut-in
    (frame 1063) at an unrelated point in the log. TTC, computed from a
    finite difference of relative velocity, is fragile to single-frame
    noise. This function applies the same idea as Section 12.10's
    hysteresis (absorbing apparent branch points caused by noise when
    compressing grid states) to the risk-perception decision.
    """
    run = 0
    for i in range(n):
        if predicate(i):
            run += 1
            if run >= persist_frames:
                return i - persist_frames + 1
        else:
            run = 0
    return None


def find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w, near_rx=40.0, persist_frames=3):
    """横方向0.72m境界・縦方向TTC=2.0秒境界のいずれか早い方でリスクが
    知覚されるフレームを求める。

    横方向の境界だけを見ると、NPCが縦方向にはるか遠方（無関係な位置）
    にいる場合でも、たまたま横位置が近い偶然の一致でヒットしてしまう
    ことがあるため、`compute_ttc`と同じ`near_rx`（縦方向近接判定の
    範囲）でも絞り込む。

    persist_frames: 境界条件を満たすフレームが単発（測定ノイズ）ではなく
    実際に持続していることを要求する（12.24節、`_first_persistent_trigger`
    参照）。デフォルト3フレームで、実際のリスク知覚イベント（数十
    フレーム以上持続する）を遅らせる影響はほぼ無視できる一方、単一
    フレームのノイズによる誤トリガーを排除できる。

    ---
    English:
    persist_frames: requires the boundary condition to actually persist,
    rather than firing on a single noisy frame (Section 12.24, see
    `_first_persistent_trigger`). The default of 3 frames has negligible
    effect on genuine risk-perception events (which persist for tens of
    frames or more), while filtering out single-frame noise triggers.
    """
    contact_half_w = eh_w + nh_w

    def lateral_ok(i):
        ry = rys[i]
        if ry is None:
            return False
        rx = rxs[i]
        if rx is None or abs(rx) > near_rx:
            return False
        return abs(ry) <= contact_half_w + CC_LATERAL_RISK_BOUNDARY

    def ttc_ok(i):
        ttc = ttcs[i]
        return ttc is not None and ttc <= CC_TTC_RISK_BOUNDARY

    lateral_frame = _first_persistent_trigger(len(rys), lateral_ok, persist_frames)
    ttc_frame = _first_persistent_trigger(len(ttcs), ttc_ok, persist_frames)
    candidates = [f for f in (lateral_frame, ttc_frame) if f is not None]
    if not candidates:
        return None, lateral_frame, ttc_frame
    return min(candidates), lateral_frame, ttc_frame


def simulate_cc_reference(gk, rxs, ego_speed, risk_frame):
    """リスク知覚フレームからの、C&Cモデルに従うEgoの反実仮想軌道を
    シミュレートする。

    Returns:
        rx_ref: 各フレームでのNPCとの相対縦距離（反実仮想）。
                risk_frame以前はrxsと同一（両者ともまだ反応していない
                という前提）。
    """
    ts = [rec["timestamp"] for rec in gk]
    n = len(gk)
    rx_ref = list(rxs)  # copy; frames before risk_frame stay as-is

    v_ref = ego_speed[risk_frame]
    traveled_actual = 0.0
    traveled_ref = 0.0
    for i in range(risk_frame, n - 1):
        dt = ts[i + 1] - ts[i]
        if dt <= 0:
            continue
        a_ref = cc_deceleration_at(ts[i] - ts[risk_frame], 0.0)
        v_ref = max(0.0, v_ref - a_ref * dt)
        traveled_ref += v_ref * dt
        traveled_actual += ego_speed[i] * dt
        if rxs[i + 1] is not None:
            rx_ref[i + 1] = rxs[i + 1] + (traveled_actual - traveled_ref)
    return rx_ref


def run():
    data = _load(LOG_PATH)
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)
    timestamps = [rec["timestamp"] for rec in gk]
    ego_speed = ego_speed_series(gk)

    closest_frame, risk = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)

    risk_frame, lateral_frame, ttc_frame = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    print(f"横方向0.72m境界でのリスク知覚フレーム: {lateral_frame}")
    print(f"縦方向TTC=2.0秒境界でのリスク知覚フレーム: {ttc_frame}")
    print(f"採用するリスク知覚フレーム(早い方): {risk_frame} "
          f"(t={timestamps[risk_frame] - timestamps[0]:.3f}s, "
          f"最近接フレーム{closest_frame}の{timestamps[closest_frame] - timestamps[risk_frame]:.3f}秒前)")

    rx_ref = simulate_cc_reference(gk, rxs, ego_speed, risk_frame)

    # Preventability verdict: does the counterfactual 2D risk ever drop
    # below 1.0 (contact) during/after the risk-perceived window?
    min_risk_ref, min_risk_frame = None, None
    for i in range(risk_frame, min(len(rxs), closest_frame + 30)):
        if rx_ref[i] is None or rys[i] is None:
            continue
        r = max(abs(rx_ref[i]) / (eh_l + nh_l), abs(rys[i]) / (eh_w + nh_w))
        if min_risk_ref is None or r < min_risk_ref:
            min_risk_ref, min_risk_frame = r, i
    preventable = min_risk_ref is not None and min_risk_ref >= 1.0
    print()
    print(f"JAMA C&Cモデルによる反実仮想シミュレーション結果:")
    print(f"  最小2Dリスク値(反実仮想) = {min_risk_ref:.4f} (frame {min_risk_frame})"
          if min_risk_ref is not None else "  最小2Dリスク値: 計算不能")
    print(f"  実際の最小2Dリスク値 = {risk:.4f} (衝突: risk<1)")
    print(f"  判定: {'予防可能 (preventable) -- 有能で慎重な人間ドライバなら回避できた' if preventable else '予防不可能 (unpreventable) -- 有能で慎重な人間ドライバでも回避できなかった'}")

    path = plot_comparison(gk, rxs, rx_ref, rys, ego_speed, eh_l, nh_l, eh_w, nh_w,
                            risk_frame, closest_frame, preventable, min_risk_ref)
    print(f"図を書き出しました: {path}")


def plot_comparison(gk, rxs, rx_ref, rys, ego_speed, eh_l, nh_l, eh_w, nh_w,
                     risk_frame, closest_frame, preventable, min_risk_ref, output_path=OUT_PATH):
    ts = [rec["timestamp"] - gk[0]["timestamp"] for rec in gk]
    t0 = ts[risk_frame]
    win = slice(max(0, risk_frame - 30), min(len(gk), closest_frame + 60))
    plot_ts = [t - t0 for t in ts]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True,
                                    gridspec_kw={"height_ratios": [2, 1]})

    actual_xy = [(plot_ts[i], rxs[i]) for i in range(win.start, win.stop) if rxs[i] is not None]
    ref_xy = [(plot_ts[i], rx_ref[i]) for i in range(win.start, win.stop) if rx_ref[i] is not None]
    xs, ys = zip(*actual_xy)
    ax1.plot(xs, ys, color="#e53935", linewidth=1.8, label="実際のログ (Autoware)")
    xs, ys = zip(*ref_xy)
    ax1.plot(xs, ys, color="#1565c0", linewidth=1.8, linestyle="--",
              label="JAMA C&Cモデルの反実仮想")
    ax1.axhline(eh_l + nh_l, color="black", linestyle=":", linewidth=1.0)
    ax1.text(plot_ts[win.start], eh_l + nh_l, " 接触境界(縦方向)", fontsize=8, va="bottom")
    ax1.axvline(0, color="#757575", linestyle="-", linewidth=0.8)
    ax1.text(0, ax1.get_ylim()[1] if ax1.get_ylim()[1] > 0 else 5, " リスク知覚",
              fontsize=8, ha="left", va="top", color="#757575")
    ax1.axvline(plot_ts[closest_frame] - t0, color="black", linestyle=":", linewidth=1.0)
    ax1.set_ylabel("Egoから見たNPCの縦方向距離 (m)")
    verdict = "予防可能 (preventable)" if preventable else "予防不可能 (unpreventable)"
    ax1.set_title(f"JAMA C&Cモデルとの反実仮想比較（ログ0067）: {verdict}"
                   f"  min risk={min_risk_ref:.3f}" if min_risk_ref is not None else "", fontsize=10.5)
    ax1.legend(loc="upper right", fontsize=9)

    speed_xy_actual = [(plot_ts[i], ego_speed[i]) for i in range(win.start, win.stop)]
    xs, ys = zip(*speed_xy_actual)
    ax2.plot(xs, ys, color="#e53935", linewidth=1.4, label="実際のEgo速度")
    ax2.axvline(0, color="#757575", linewidth=0.8)
    ax2.set_ylabel("Ego速度 (m/s)")
    ax2.set_xlabel("リスク知覚からの経過時間 (s)")
    ax2.legend(loc="upper right", fontsize=9)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


if __name__ == "__main__":
    run()
