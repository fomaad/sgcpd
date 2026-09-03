"""複数ログ（衝突ログ・非衝突ログ）を教師データとして、抽象解釈演算子の
しきい値をZ3で自動合成する。

## 経緯

12.15節では、しきい値合成の「教師信号」として、ドメイン分析ですでに
確立した「この区間の原因はこれだった」という知識を人手で与えていた
（そのため1本のログでは境界を実質的に拘束できない、という限界があった）。

これに対しユーザーから: 「要因（例えば速度）を与えると，衝突を起こして
いるログと，起こしていないログが与えられれば，それを区別するような
抽象化は自動的に見つかるのではないですか？」との指摘があった。これは
まさにその通りで、**衝突の有無というログ単位のラベルは、ログを集める
だけで自動的に手に入る（人手のドメイン分析が不要）**という点で、
12.15節の方法より優れている。

セッションの環境には、`TD-NI-AR-SD-N04-CI-*.json`という同一シナリオ
ファミリー（NPCのカットイン）の6本のログがすでに存在しており
（0030, 0032, 0035, 0047, 0067, 0076）、確認したところ0067だけが実際に
衝突しており、残り5本は衝突していない。これは「同じシナリオの変奏で、
1つだけ衝突に至った」という、しきい値合成にとって理想的な教師データ
である。

## 方法

各ログについて、EGO・NPCの2D的な最近接（`risk = max(|rx|/(eh_l+nh_l),
|ry|/(eh_w+nh_w))`が最小になる瞬間、risk<1が衝突を意味する）を求め、
そのカットイン開始からその瞬間までの区間を「箱」とみなして
（12.15節の`box_aggregated_deceleration_ratio`と同じ考え方）、
achieved/requiredの減速比を1つのログにつき1つの特徴量として計算する。

その上で、Z3に「衝突したログの比は、非衝突ログの比よりも常に小さい
（＝弱かった）」という分離制約を与え、分離可能なしきい値
（`weak_ratio`または`adequate_ratio`の位置）を求める。分離できない
場合は、この特徴量（減速の十分性比）だけでは衝突/非衝突を判別できない
ということが分かる——これも重要な結果であり、隠さず報告する。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.synth_thresholds_multilog
"""

import glob
import json
import math

import matplotlib
import z3

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from logverify.abstract_cause import required_deceleration_magnitude

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

LOG_GLOB = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-*.json"
OUT_PATH = "out_gif/synth_thresholds_multilog.png"


def _load(json_path):
    with open(json_path) as f:
        return json.load(f)


def vehicle_sizes(data):
    sizes = {v["name"]: v["size"] for v in data["groundtruth_size"]["vehicle_sizes"]}
    ego = sizes["ego"]
    npc = sizes.get("npc1", list(v for k, v in sizes.items() if k != "ego")[0])
    return (ego["x"] / 2, ego["y"] / 2), (npc["x"] / 2, npc["y"] / 2)


def relative_xy(data):
    gk = data["groundtruth_kinematic"]
    rxs, rys = [], []
    for rec in gk:
        ego = rec["groundtruth_ego"]
        ex, ey = ego["pose"]["position"]["x"], ego["pose"]["position"]["y"]
        yaw = math.radians(ego["pose"]["rotation"]["z"])
        fwd = (math.cos(yaw), math.sin(yaw))
        left = (-math.sin(yaw), math.cos(yaw))
        npc = next((v for v in rec.get("groundtruth_vehicles", []) if v["name"] == "npc1"), None)
        if npc is None:
            rxs.append(None)
            rys.append(None)
            continue
        dx = npc["pose"]["position"]["x"] - ex
        dy = npc["pose"]["position"]["y"] - ey
        rxs.append(dx * fwd[0] + dy * fwd[1])
        rys.append(dx * left[0] + dy * left[1])
    return rxs, rys


def closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w, near_rx=20.0):
    """EGO・NPCの2D的な最近接のフレームを求める。`risk < 1`はその瞬間に
    衝突していることを意味する。

    衝突ログでは、単純にrisk最小のフレーム（＝衝突がもっとも深く重なった
    瞬間）を使うと、そこでは既にdistance_to_contactが負になっており
    `required_deceleration_magnitude`が発散してratioが定義できない
    （12.15節で見た問題と同じ）。そこで、risk<1に**最初に**達したフレーム
    が存在する場合は、その1つ手前（まだ接触していない最後の瞬間）を返す。
    非衝突ログ（risk<1に一度も達しない）では、単純にrisk最小のフレームを
    返す。

    ---
    English:
    Finds the frame of closest 2D approach between Ego and the NPC;
    `risk < 1` means the two are colliding at that instant.

    For a collision log, simply taking the frame of minimum risk (the
    instant of deepest overlap) means distance_to_contact is already
    negative there, so `required_deceleration_magnitude` diverges and the
    ratio is undefined (the same issue seen in Section 12.15). So, if a
    frame first reaches risk<1, the frame just before it (the last
    instant not yet in contact) is returned instead. For a non-collision
    log (risk never drops below 1), the frame of minimum risk is returned
    directly.
    """
    best_i, best_risk = None, None
    first_contact_i = None
    for i, (rx, ry) in enumerate(zip(rxs, rys)):
        if rx is None or abs(rx) > near_rx:
            continue
        risk = max(abs(rx) / (eh_l + nh_l), abs(ry) / (eh_w + nh_w))
        if risk < 1.0 and first_contact_i is None:
            first_contact_i = i
        if best_risk is None or risk < best_risk:
            best_risk, best_i = risk, i
    if first_contact_i is not None:
        return max(0, first_contact_i - 1), best_risk
    return best_i, best_risk


def cutin_onset_frame(rys, closest_frame, lookback=400):
    """`demo_scenario_snapshot.py`等と同じ簡易ロジックで、NPCの車線変更
    開始フレームを検出する。

    ---
    English:
    Detects the NPC's lane-change onset frame, using the same simple
    logic as `demo_scenario_snapshot.py` and others.
    """
    start = max(0, closest_frame - lookback)
    baseline = rys[start]
    for i in range(start, closest_frame):
        if abs(rys[i] - baseline) > 0.3 and all(
            abs(rys[j] - rys[j - 1]) < 0.05 or (rys[j] - rys[j - 1]) * (rys[i] - baseline) > 0
            for j in range(max(0, i - 5), i + 1)
        ):
            return i
    return max(0, closest_frame - 100)


def log_level_deceleration_ratio(json_path: str):
    """1本のログについて、カットイン開始から最近接瞬間までの区間全体を
    「箱」とみなし、achieved/requiredの減速比を1つ計算する
    （12.15節の`box_aggregated_deceleration_ratio`と同じ考え方の、
    ログ単位版）。

    Returns:
        (ratio, is_collision, closest_frame, onset_frame) のタプル。
        ratioがNoneの場合は、すでに接触範囲にあるなどでratioが定義できない。

    ---
    English:
    For one log, treats the whole interval from cut-in onset to the
    closest-approach instant as a single "box" (the log-level analogue
    of Section 12.15's `box_aggregated_deceleration_ratio`), and computes
    one achieved/required deceleration ratio for that log.

    Returns:
        (ratio, is_collision, closest_frame, onset_frame). ratio is None
        when it cannot be defined (e.g. already within contact range).
    """
    data = _load(json_path)
    gk = data["groundtruth_kinematic"]
    cc = data["control_cmds"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)

    closest_frame, risk = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    is_collision = risk is not None and risk < 1.0
    onset_frame = cutin_onset_frame(rys, closest_frame)

    # `required_deceleration_magnitude` only looks at the longitudinal
    # distance rx - (eh_l+nh_l); during a cut-in this can go negative well
    # before the actual (2D) collision instant, since the NPC can already
    # be longitudinally close while still laterally clear. So evaluate at
    # the last frame at or before closest_frame where the longitudinal
    # distance is still positive, not at closest_frame itself.
    eval_frame = closest_frame
    while eval_frame > onset_frame and rxs[eval_frame] - (eh_l + nh_l) <= 0:
        eval_frame -= 1

    t0, t1 = gk[onset_frame]["timestamp"], gk[eval_frame]["timestamp"]
    dt = t1 - t0
    if dt <= 0:
        return None, is_collision, closest_frame, onset_frame
    closing = (rxs[onset_frame] - rxs[eval_frame]) / dt
    dist = rxs[eval_frame] - (eh_l + nh_l)
    required = required_deceleration_magnitude(closing, dist)
    if not math.isfinite(required) or required <= 0:
        return None, is_collision, closest_frame, onset_frame
    accs = [abs(e["longitudinal"]["acceleration"]) for e in cc if t0 <= e["timestamp"] <= t1]
    if not accs:
        return None, is_collision, closest_frame, onset_frame
    achieved = sum(accs) / len(accs)
    return achieved / required, is_collision, closest_frame, onset_frame


def synthesize_separating_threshold(collision_ratios, safe_ratios, default=1.0):
    """衝突ログの比(`collision_ratios`)が非衝突ログの比(`safe_ratios`)より
    常に小さくなるように分離できるしきい値を、Z3で求める（分離できる場合は
    マージン最大化、`default`（現在の`adequate_ratio`）から最小変化のもの
    ではなく、2群のちょうど中間になるものを返す）。

    ---
    English:
    Uses Z3 to find a threshold that separates the collision logs' ratios
    (always classified below it) from the safe logs' ratios (always at or
    above it), if such a separation exists -- maximizing the margin rather
    than minimizing distance from `default`.
    """
    if not collision_ratios or not safe_ratios:
        return None, "insufficient data (need at least 1 collision and 1 non-collision log)"

    threshold = z3.Real("threshold")
    margin = z3.Real("margin")
    opt = z3.Optimize()
    opt.add(margin > 0)
    for r in collision_ratios:
        opt.add(z3.RealVal(r) + margin <= threshold)
    for r in safe_ratios:
        opt.add(z3.RealVal(r) - margin >= threshold)
    opt.maximize(margin)

    result = opt.check()
    if result != z3.sat:
        return None, f"not separable by a single threshold on this feature ({result})"

    m = opt.model()
    t = float(m.eval(threshold, model_completion=True).as_fraction())
    mg = float(m.eval(margin, model_completion=True).as_fraction())
    return dict(threshold=t, margin=mg), None


def plot_separation(rows, synth_threshold=None, output_path=OUT_PATH) -> str:
    """各ログの減速比を、衝突/非衝突で色分けした1次元の散布図として描き、
    Z3が合成したしきい値と、12.12節の既定のしきい値を縦線として重ねる。

    ---
    English:
    Plots each log's deceleration ratio as a 1D scatter, colored by
    collision/non-collision, with the Z3-synthesized threshold and
    Section 12.12's default thresholds overlaid as vertical lines.
    """
    fig, ax = plt.subplots(figsize=(9, 3.2))
    for name, ratio, is_collision, _, _ in rows:
        if ratio is None:
            continue
        color = "#e53935" if is_collision else "#43a047"
        marker = "X" if is_collision else "o"
        ax.scatter([ratio], [0], s=140, color=color, marker=marker, zorder=3, edgecolor="black", linewidth=0.6)
        ax.annotate(name.replace("TD-NI-AR-SD-N04-CI-", "#").replace(".json", ""),
                    (ratio, 0), xytext=(0, 12 if is_collision else -18), textcoords="offset points",
                    ha="center", fontsize=8, color=color)

    if synth_threshold is not None:
        ax.axvline(synth_threshold, color="#1565c0", linestyle="-", linewidth=1.4, zorder=2)
        ax.text(synth_threshold, 0.55, f"Z3合成: {synth_threshold:.3f}", color="#1565c0", fontsize=8,
                ha="center", rotation=90, va="bottom")
    ax.axvline(0.5, color="#9e9e9e", linestyle="--", linewidth=1.0, zorder=1)
    ax.text(0.5, -0.55, "既定 weak_ratio=0.5", color="#757575", fontsize=8, ha="center", rotation=90, va="top")
    ax.axvline(1.0, color="#9e9e9e", linestyle=":", linewidth=1.0, zorder=1)
    ax.text(1.0, -0.55, "既定 adequate_ratio=1.0", color="#757575", fontsize=8, ha="center", rotation=90, va="top")

    ax.set_xscale("log")
    ax.set_xlabel("減速の十分性比 achieved/required (対数軸)")
    ax.set_yticks([])
    ax.set_ylim(-1.0, 1.0)
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)
    ax.set_title("6本のログ（同一シナリオファミリー、衝突=X/赤・非衝突=○/緑）における減速比の分離", fontsize=11)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def run() -> None:
    paths = sorted(glob.glob(LOG_GLOB))
    print(f"対象ログ: {len(paths)}本")
    collision_ratios, safe_ratios = [], []
    rows = []
    for p in paths:
        name = p.split("/")[-1]
        ratio, is_collision, closest_frame, onset_frame = log_level_deceleration_ratio(p)
        rows.append((name, ratio, is_collision, closest_frame, onset_frame))
        if ratio is None:
            print(f"  {name}: ratio=N/A (定義できず) collision={is_collision}")
            continue
        print(f"  {name}: ratio={ratio:.4f} collision={is_collision} "
              f"(onset={onset_frame}, closest={closest_frame})")
        if is_collision:
            collision_ratios.append(ratio)
        else:
            safe_ratios.append(ratio)
    print()

    print(f"衝突ログの比: {[f'{r:.3f}' for r in collision_ratios]}")
    print(f"非衝突ログの比: {[f'{r:.3f}' for r in safe_ratios]}")
    print()

    print("=== Z3で、衝突ログと非衝突ログを分離するしきい値を合成 ===")
    result, error = synthesize_separating_threshold(collision_ratios, safe_ratios)
    if result is None:
        print(f"分離失敗: {error}")
        print("-> 「減速の十分性比」という特徴量だけでは、このログ集合の衝突/非衝突を"
              "分離できなかった。他の要因（横方向の余裕、予測信頼性等）と組み合わせる、"
              "あるいは特徴量自体を見直す必要がある。")
        return
    print(f"合成されたしきい値: {result['threshold']:.4f} (マージン={result['margin']:.4f})")
    print(f"(参考: 12.12節の既定のadequate_ratio=1.0, weak_ratio=0.5)")
    print(f"-> 減速の十分性比がこのしきい値未満のログは全て衝突しており、以上のログは全て衝突していない。"
          f"6本中1本の衝突という限られたデータではあるが、'減速が要求値の約{result['threshold']:.2f}倍未満'"
          f"という条件が、このシナリオファミリーにおける衝突/非衝突をちょうど分離することが確認できた。")

    path = plot_separation(rows, synth_threshold=result["threshold"])
    print(f"図を書き出しました: {path}")


if __name__ == "__main__":
    run()
