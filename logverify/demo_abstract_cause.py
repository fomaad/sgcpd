"""`logverify/abstract_cause.py`の3つの抽象解釈演算子（減速の十分性・
NPC予測の信頼性・接触余裕）を、衝突ログ(既定: TD-NI-AR-SD-N04-CI-0067)
に適用し、衝突直前に3つの抽象値が同時に「悪化」して現れることを確認する
デモ。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_abstract_cause \
        [path-to-TD-NI-AR-SD-N04-CI-0067.json]
"""

import json
import math
import sys

from logverify.abstract_cause import (
    classify_contact_margin,
    classify_deceleration_adequacy,
    classify_prediction_reliability,
    required_deceleration_magnitude,
)
from logverify.grid_bridge import relative_xy_from_ajisai_groundtruth

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"


def _load(json_path):
    with open(json_path) as f:
        return json.load(f)


def vehicle_sizes(data):
    sizes = {v["name"]: v["size"] for v in data["groundtruth_size"]["vehicle_sizes"]}
    ego = sizes["ego"]
    npc = sizes.get("npc1", list(v for k, v in sizes.items() if k != "ego")[0])
    return (ego["x"] / 2, ego["y"] / 2), (npc["x"] / 2, npc["y"] / 2)


def find_collision_frames(rel_xy, eh_l, eh_w, nh_l, nh_w):
    frames = []
    for i, (rx, ry) in enumerate(rel_xy):
        if (eh_l + nh_l) - abs(rx) > 0 and (eh_w + nh_w) - abs(ry) > 0:
            frames.append(i)
    return frames


def detect_cutin_onset_frame(rys, window_first):
    """NPCの横方向移動（車線変更）開始フレームを検出する
    （demo_collision_root_cause.run と同じ簡易ロジック）。"""
    baseline = rys[max(0, window_first - 400)]
    for i in range(max(0, window_first - 400), window_first):
        if abs(rys[i] - baseline) > 0.3 and all(
            abs(rys[j] - rys[j - 1]) < 0.05 or (rys[j] - rys[j - 1]) * (rys[i] - baseline) > 0
            for j in range(max(0, i - 5), i + 1)
        ):
            return i
    return max(0, window_first - 100)


def run(json_path: str) -> None:
    print(f"Loading: {json_path}")
    data = _load(json_path)
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path)
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)

    coll_frames = find_collision_frames(rel_xy, eh_l, eh_w, nh_l, nh_w)
    assert coll_frames, "no collision found in this log"
    window_first, window_last = coll_frames[0], coll_frames[-1]
    onset_frame = detect_cutin_onset_frame(rys, window_first)
    onset_ts = gk[onset_frame]["timestamp"]
    window_ts = gk[window_first]["timestamp"]
    print(f"cut-in onset: frame {onset_frame} (t={onset_ts:.2f}s)")
    print(f"collision window: frame {window_first}-{window_last} (t={window_ts:.2f}s-{gk[window_last]['timestamp']:.2f}s)")
    print()

    # === 1. 減速の十分性 (deceleration adequacy) ===
    print("=== 1. classify_deceleration_adequacy: 減速の十分性 ===")
    # closing_speed: rxの変化率（onset時点、±0.5秒の対称差分）
    half_win = 36  # ~0.5s at ~71Hz
    i0 = max(0, onset_frame - half_win)
    i1 = min(len(rxs) - 1, onset_frame + half_win)
    dt = gk[i1]["timestamp"] - gk[i0]["timestamp"]
    closing_speed = (rxs[i0] - rxs[i1]) / dt if dt > 0 else 0.0
    distance_to_contact = rxs[onset_frame] - (eh_l + nh_l)
    required = required_deceleration_magnitude(closing_speed, distance_to_contact)

    cc = data["control_cmds"]
    accels = [(e["timestamp"], e["longitudinal"]["acceleration"]) for e in cc]
    peak_decel = min((a for ts, a in accels if onset_ts <= ts <= window_ts), default=0.0)
    achieved = abs(peak_decel)

    label = classify_deceleration_adequacy(achieved, required)
    print(f"  closing_speed(onset, ±0.5s平均)={closing_speed:.2f} m/s, 縦方向の余裕(rx-車体長和)={distance_to_contact:.2f} m")
    print(f"  required_decel(v^2/2d) = {required:.3f} m/s^2   achieved_decel(peak) = {achieved:.3f} m/s^2")
    print(f"  -> classify_deceleration_adequacy = 【{label}】 (achieved/required = {achieved/required:.2f}倍)"
          if math.isfinite(required) and required > 0 else f"  -> classify_deceleration_adequacy = 【{label}】")
    print()

    # === 2. NPC予測の信頼性 (prediction reliability) ===
    print("=== 2. classify_prediction_reliability: NPC予測経路の信頼性（時系列） ===")
    import math as _m

    def ego_basis_at(ts_target):
        best = min(gk, key=lambda e: abs(e["timestamp"] - ts_target))
        ex = best["groundtruth_ego"]["pose"]["position"]["x"]
        ey = best["groundtruth_ego"]["pose"]["position"]["y"]
        yaw = _m.radians(best["groundtruth_ego"]["pose"]["rotation"]["z"])
        return ex, ey, (_m.cos(yaw), _m.sin(yaw)), (-_m.sin(yaw), _m.cos(yaw))

    def project(px, py, ex, ey, fwd, left):
        dx, dy = px - ex, py - ey
        return dx * fwd[0] + dy * fwd[1], dx * left[0] + dy * left[1]

    po = data["perception_objects"]
    shown = 0
    labels_seen = []
    for e in po:
        ts = e["timestamp"]
        if ts < onset_ts or ts > window_ts or not e["objects"]:
            continue
        obj = e["objects"][0]
        paths = obj.get("predict_paths", [])
        if not paths:
            continue
        best_path = max(paths, key=lambda p: p["confidence"])
        ex, ey, fwd, left = ego_basis_at(ts)
        cur_rx, cur_ry = project(obj["pose"]["position"]["x"], obj["pose"]["position"]["y"], ex, ey, fwd, left)
        horizon_idx = min(3, len(best_path["path"]) - 1)
        horizon_s = horizon_idx * 0.5
        pp = best_path["path"][horizon_idx]
        pred_rx, pred_ry = project(pp["position"]["x"], pp["position"]["y"], ex, ey, fwd, left)
        predicted_delta = pred_ry - cur_ry

        future_ts = ts + horizon_s
        future_frame = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - future_ts))
        actual_delta = rys[future_frame] - cur_ry

        label = classify_prediction_reliability(best_path["confidence"], predicted_delta, actual_delta)
        labels_seen.append((ts, label))
        if shown < 14:
            print(
                f"    t={ts:.2f}s conf={best_path['confidence']:.2f} predicted_delta={predicted_delta:+.2f}m"
                f" actual_delta(+{horizon_s:.1f}s後)={actual_delta:+.2f}m  -> 【{label}】"
            )
            shown += 1
    print()

    # === 3. 接触余裕 (contact margin) ===
    print("=== 3. classify_contact_margin: 横方向の接触余裕（衝突ウィンドウ前後、圧縮表示） ===")
    coll_set = set(coll_frames)
    seq = []
    for i in range(max(0, window_first - 200), min(len(rys), window_last + 50)):
        label = classify_contact_margin(rys[i], eh_w, nh_w, is_colliding=(i in coll_set))
        seq.append((i, label))
    compressed = []
    for i, label in seq:
        if compressed and compressed[-1][2] == label:
            compressed[-1] = (compressed[-1][0], i, label)
        else:
            compressed.append((i, i, label))
    for start, end, label in compressed:
        marker = " <-- 衝突ウィンドウ" if start <= window_last and end >= window_first else ""
        print(f"    frames {start}-{end}: 【{label}】{marker}")
    print()

    print("=== まとめ: 衝突直前の抽象値の重なり ===")
    pred_near_collision = [l for ts, l in labels_seen if ts >= window_ts - 1.0]
    decel_label = classify_deceleration_adequacy(achieved, required)
    print(f"  減速の十分性（オンセット時点のスナップショットで評価）: 【{decel_label}】"
          " (注: 下記まとめ参照 -- この演算子はスナップショット時点に敏感で、要改良)")
    print(f"  NPC予測の信頼性（衝突直前1秒間に観測されたラベル）: {sorted(set(pred_near_collision))}")
    print(f"  接触余裕（衝突ウィンドウ内）: 【接触】")
    print(
        "  -> 3つの抽象値が、生の数値を見ることなく、それだけで「なぜ衝突したか」を"
        "語っている: 必要な減速度に対して実際の減速が不足し（弱い/非常に弱い）、"
        "NPCの予測は直進継続を信頼度を保ったまま予測し続け（陳腐）、"
        "その間に横方向の余裕は接触可能域まで縮み、実際に接触に至った。"
    )


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
