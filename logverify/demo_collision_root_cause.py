"""方法A（咲川氏の抽象化）で、実際に衝突が起きたAJISAI cut-inログを
抽象化し、衝突の原因を分析するデモ。

対象は TD-NI-AR-SD-N04-CI-0067.json （AJISAIデータセットのcut_in、
`data/cutin/`に94本あるうちの1本）。この6本（0030,0032,0035,0047,0067,0076）
はこれまでMethod C/Bのデモで使ってきたものだが、`groundtruth_size`
（egoとNPCのオリエンテッド・バウンディングボックスのサイズ）と
`groundtruth_kinematic`（ワールド座標での位置・姿勢）を突き合わせて
矩形同士の重なりを直接判定したところ、この中で唯一0067だけが
実際に衝突（バウンディングボックスの重なり）を含んでいることが分かった
（AJISAIの配布物にはconsistency_okはあっても衝突フラグ自体は
含まれていないため、KSE2026論文が報告する「432本中93本が衝突を含む」
という統計は、恐らくこのような幾何学的な重なり判定で別途計算された
ものだと考えられる）。

本デモは、
  1. 咲川氏の9領域（粗い抽象化）では、衝突の瞬間が「LEAD・同幅帯」と
     いう1つの状態にまとまってしまい、何が悪かったのかが全く見えない
     ことを示す。
  2. EGO周辺だけを細かく刻んだ抽象化（`sakikawa_relations.
     compress_to_fine_relation_states`。単位はあくまでego車両自身の
     サイズの一部であり、恣意的な距離ではない）に切り替えると、
     「じわじわと重なりが深くなっていく」過程が見えるようになることを
     示す。
  3. 抽象化だけでは分からない「なぜ」の部分を、ログの生データ
     （`control_cmds`の減速指令、`perception_objects`のNPC予測経路
     `predict_paths`）を直接読むことで補い、ユーザーの仮説
     （減速のタイミング・強さ、NPCの予測軌道のゆるさ）を検証する。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_collision_root_cause \
        [path-to-TD-NI-AR-SD-N04-CI-0067.json]
"""

import math
import sys

from logverify.grid_bridge import relative_xy_from_ajisai_groundtruth
from logverify.sakikawa_relations import (
    EGO_HALF_LENGTH,
    EGO_HALF_WIDTH,
    compress_to_fine_relation_states,
    relation_states_from_relative_xy,
)

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"


def _load(json_path):
    import json

    with open(json_path) as f:
        return json.load(f)


def find_collision_window(data, rel_xy):
    """groundtruth_sizeのego/NPCサイズを使い、egoフレームでの軸並行矩形の
    重なり（衝突候補）が続くフレーム範囲を返す。

    ---
    English:
    Using the ego/NPC sizes in groundtruth_size, returns the frame range
    over which an axis-aligned rectangle overlap (a collision candidate)
    persists in the ego frame.
    """
    sizes = {v["name"]: v["size"] for v in data["groundtruth_size"]["vehicle_sizes"]}
    ego_size = sizes["ego"]
    npc_size = sizes.get("npc1", list(sizes.values())[1])
    eh_l, eh_w = ego_size["x"] / 2, ego_size["y"] / 2
    nh_l, nh_w = npc_size["x"] / 2, npc_size["y"] / 2

    overlap_frames = []
    deepest = None
    for i, (rx, ry) in enumerate(rel_xy):
        overlap_x = (eh_l + nh_l) - abs(rx)
        overlap_y = (eh_w + nh_w) - abs(ry)
        if overlap_x > 0 and overlap_y > 0:
            overlap_frames.append(i)
            depth = min(overlap_x, overlap_y)
            if deepest is None or depth > deepest[0]:
                deepest = (depth, i, rx, ry)
    if not overlap_frames:
        return None
    return {
        "first": overlap_frames[0],
        "last": overlap_frames[-1],
        "deepest_frame": deepest[1],
        "deepest_penetration_m": deepest[0],
        "deepest_rx": deepest[2],
        "deepest_ry": deepest[3],
    }


def report_coarse_vs_fine(rel_xy, window):
    print("=== 1. 咲川氏の9領域（粗い）抽象化: 衝突の瞬間は1状態に潰れる ===")
    states = relation_states_from_relative_xy(rel_xy)
    for s in states:
        if s.start_frame <= window["last"] and s.end_frame >= window["first"] - 30:
            marker = " <-- 衝突ウィンドウを含む" if s.start_frame <= window["deepest_frame"] <= s.end_frame else ""
            print(f"  (lane={s.lane:+d}, position={s.position:+d})  frames {s.start_frame}-{s.end_frame}{marker}")
    print()

    print("=== 2. EGO周辺を細かく刻んだ抽象化（単位=ego車両サイズ/4）===")
    fine_states = compress_to_fine_relation_states(
        [p[0] for p in rel_xy], [p[1] for p in rel_xy], n_bins=4
    )
    for s in fine_states:
        if s.start_frame <= window["last"] + 10 and s.end_frame >= window["first"] - 40:
            marker = " <-- 衝突ウィンドウ" if s.start_frame <= window["deepest_frame"] <= s.end_frame else ""
            print(
                f"  (lane_fine={s.lane_fine:+d}, position_fine={s.position_fine:+d})"
                f"  frames {s.start_frame}-{s.end_frame}{marker}"
            )
    print(
        f"\n  (unit: lane={EGO_HALF_WIDTH/4:.3f}m, position={EGO_HALF_LENGTH/4:.3f}m"
        f" -- いずれもego車両自身のサイズの1/4であり、分析用に選んだ値ではない)"
    )
    print()


def report_deceleration(data, cutin_onset_ts, window_start_ts):
    print("=== 3. egoの減速指令（control_cmds.longitudinal.acceleration）===")
    cc = data["control_cmds"]
    accel_ts = [(e["timestamp"], e["longitudinal"]["acceleration"]) for e in cc]

    onset_decel_ts = next((ts for ts, a in accel_ts if ts >= cutin_onset_ts and a < 0), None)
    print(f"  NPCの車線変更（横方向移動）開始: 約 {cutin_onset_ts:.2f}s")
    print(f"  egoの減速指令（acceleration<0）開始: 約 {onset_decel_ts:.2f}s" if onset_decel_ts else "  減速指令の開始が見つかりません")
    if onset_decel_ts is not None:
        print(f"  -> 反応の遅れ: {onset_decel_ts - cutin_onset_ts:.2f}s（タイミング自体は大きくは遅れていない）")

    print(f"  衝突ウィンドウ開始: {window_start_ts:.2f}s の時点での減速指令:")
    for ts, a in accel_ts:
        if abs(ts - window_start_ts) < 0.05:
            print(f"    t={ts:.2f}s: acceleration={a:.3f} m/s^2")
    peak = min((a for ts, a in accel_ts if ts <= window_start_ts + 1.0), default=None)
    print(f"  衝突ウィンドウ前後での最大減速度: {peak:.3f} m/s^2" if peak is not None else "")
    print(
        "  -> 減速指令はNPCの車線変更開始とほぼ同時に始まっているが、"
        "そこから強い減速（-2m/s^2超）に達するまで約2秒以上かかっており、"
        "衝突ウィンドウに入る時点でもまだ最大減速度に達していない"
        "（タイミングよりも、立ち上がりの弱さ・遅さが主因と考えられる）。"
    )
    print()


def _ego_basis_at(ts_target, gk):
    best = min(gk, key=lambda e: abs(e["timestamp"] - ts_target))
    ex = best["groundtruth_ego"]["pose"]["position"]["x"]
    ey = best["groundtruth_ego"]["pose"]["position"]["y"]
    yaw = math.radians(best["groundtruth_ego"]["pose"]["rotation"]["z"])
    fwd = (math.cos(yaw), math.sin(yaw))
    left = (-math.sin(yaw), math.cos(yaw))
    return ex, ey, fwd, left


def _project(px, py, ex, ey, fwd, left):
    dx, dy = px - ex, py - ey
    return dx * fwd[0] + dy * fwd[1], dx * left[0] + dy * left[1]


def report_npc_prediction(data, cutin_onset_ts, window_start_ts):
    print("=== 4. perception_objectsのNPC予測経路（predict_paths）の「ゆるさ」===")
    gk = data["groundtruth_kinematic"]
    po = data["perception_objects"]

    print(f"  NPCの実際の車線変更開始: 約{cutin_onset_ts:.2f}s")
    print("  その後の、信頼度最大の予測経路（+1.5秒後の予測位置）:")
    shown = 0
    for e in po:
        ts = e["timestamp"]
        if ts < cutin_onset_ts or ts > window_start_ts or not e["objects"]:
            continue
        obj = e["objects"][0]
        paths = obj.get("predict_paths", [])
        if not paths:
            continue
        best_path = max(paths, key=lambda p: p["confidence"])
        ex, ey, fwd, left = _ego_basis_at(ts, gk)
        cur_rx, cur_ry = _project(obj["pose"]["position"]["x"], obj["pose"]["position"]["y"], ex, ey, fwd, left)
        horizon_idx = min(3, len(best_path["path"]) - 1)
        pp = best_path["path"][horizon_idx]
        pred_rx, pred_ry = _project(pp["position"]["x"], pp["position"]["y"], ex, ey, fwd, left)
        print(
            f"    t={ts:.2f}s conf={best_path['confidence']:.2f}"
            f" 現在(rx={cur_rx:.2f},ry={cur_ry:.2f})"
            f" -> +{horizon_idx*0.5:.1f}s後の予測(rx={pred_rx:.2f},ry={pred_ry:.2f})"
            f" (候補経路数={len(paths)})"
        )
        shown += 1
        if shown >= 12:
            break
    print(
        "\n  -> 実際にはNPCは自レーンからego車線へ横方向に移動し始めているにも"
        "関わらず、信頼度最大の予測経路は、しばらくの間"
        "「このままレーンを維持して直進する」（ry がほぼ変化しない）という"
        "予測を出し続けている。同時に信頼度自体も1.0から0.2〜0.3まで"
        "低下しており、複数の候補経路（車線変更あり/なし）の間で"
        "予測が割れている＝どちらとも決めきれていない状態が続く。"
        "これが「NPCの予測軌道がゆるい」というユーザーの仮説と一致する。"
    )
    print()


def run(json_path: str) -> None:
    print(f"Loading: {json_path}")
    data = _load(json_path)
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path)
    gk = data["groundtruth_kinematic"]

    window = find_collision_window(data, rel_xy)
    if window is None:
        print("このログに衝突（バウンディングボックスの重なり）は見つかりませんでした。")
        return

    window_start_ts = gk[window["first"]]["timestamp"]
    window_end_ts = gk[window["last"]]["timestamp"]
    print(
        f"衝突ウィンドウ: フレーム{window['first']}-{window['last']}"
        f"（t={window_start_ts:.3f}s〜{window_end_ts:.3f}s、継続{window_end_ts-window_start_ts:.2f}秒）"
    )
    print(
        f"最大めり込み量: {window['deepest_penetration_m']:.3f}m"
        f"（フレーム{window['deepest_frame']}, rx={window['deepest_rx']:.2f}, ry={window['deepest_ry']:.2f}）"
    )
    print()

    # NPCの車線変更（横方向移動）開始フレームを検出: |ry|の変化率が
    # 継続的に増え始める最初の時点（簡易的に、直近との差分が符号付きで
    # 一定方向に増加し続ける最初のフレームを採用）。
    rys = [p[1] for p in rel_xy]
    cutin_onset_frame = None
    baseline = rys[max(0, window["first"] - 400)]
    for i in range(max(0, window["first"] - 400), window["first"]):
        if abs(rys[i] - baseline) > 0.3 and all(abs(rys[j] - rys[j - 1]) < 0.05 or (rys[j] - rys[j - 1]) * (rys[i] - baseline) > 0 for j in range(max(0, i - 5), i + 1)):
            cutin_onset_frame = i
            break
    if cutin_onset_frame is None:
        cutin_onset_frame = max(0, window["first"] - 100)
    cutin_onset_ts = gk[cutin_onset_frame]["timestamp"]

    report_coarse_vs_fine(rel_xy, window)
    report_deceleration(data, cutin_onset_ts, window_start_ts)
    report_npc_prediction(data, cutin_onset_ts, window_start_ts)


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
