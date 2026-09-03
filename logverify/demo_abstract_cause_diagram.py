"""12.12節の3つの抽象解釈演算子の分類結果を、時間軸に沿った
スイムレーンとして可視化するデモ（`abstract_cause_diagram.py`を使う）。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_abstract_cause_diagram \
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
from logverify.abstract_cause_diagram import TimeSegment, _compress_segments, plot_abstract_cause_timeline
from logverify.grid_bridge import relative_xy_from_ajisai_groundtruth
from logverify.sakikawa_relations import compress_to_fine_relation_states

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_PATH = "out_gif/collision_0067_abstract_cause_timeline.png"


def _load(json_path):
    with open(json_path) as f:
        return json.load(f)


def vehicle_sizes(data):
    sizes = {v["name"]: v["size"] for v in data["groundtruth_size"]["vehicle_sizes"]}
    ego = sizes["ego"]
    npc = sizes.get("npc1", list(v for k, v in sizes.items() if k != "ego")[0])
    return (ego["x"] / 2, ego["y"] / 2), (npc["x"] / 2, npc["y"] / 2)


def find_collision_frames(rel_xy, eh_l, eh_w, nh_l, nh_w):
    return [
        i for i, (rx, ry) in enumerate(rel_xy)
        if (eh_l + nh_l) - abs(rx) > 0 and (eh_w + nh_w) - abs(ry) > 0
    ]


def detect_cutin_onset_frame(rys, window_first):
    baseline = rys[max(0, window_first - 400)]
    for i in range(max(0, window_first - 400), window_first):
        if abs(rys[i] - baseline) > 0.3 and all(
            abs(rys[j] - rys[j - 1]) < 0.05 or (rys[j] - rys[j - 1]) * (rys[i] - baseline) > 0
            for j in range(max(0, i - 5), i + 1)
        ):
            return i
    return max(0, window_first - 100)


def ego_basis_at(gk, ts_target):
    best = min(gk, key=lambda e: abs(e["timestamp"] - ts_target))
    ex = best["groundtruth_ego"]["pose"]["position"]["x"]
    ey = best["groundtruth_ego"]["pose"]["position"]["y"]
    yaw = math.radians(best["groundtruth_ego"]["pose"]["rotation"]["z"])
    return ex, ey, (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))


def project(px, py, ex, ey, fwd, left):
    dx, dy = px - ex, py - ey
    return dx * fwd[0] + dy * fwd[1], dx * left[0] + dy * left[1]


def run(json_path: str) -> None:
    print(f"Loading: {json_path}")
    data = _load(json_path)
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path)
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    gk = data["groundtruth_kinematic"]
    cc = data["control_cmds"]
    po = data["perception_objects"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)

    coll_frames = find_collision_frames(rel_xy, eh_l, eh_w, nh_l, nh_w)
    window_first, window_last = coll_frames[0], coll_frames[-1]
    onset_frame = detect_cutin_onset_frame(rys, window_first)
    onset_ts = gk[onset_frame]["timestamp"]
    window_ts0, window_ts1 = gk[window_first]["timestamp"], gk[window_last]["timestamp"]

    # 表示する時間範囲: カットイン検出の少し前から、衝突ウィンドウの少し後まで
    t_lo = onset_ts - 1.0
    t_hi = window_ts1 + 1.0
    frame_lo = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - t_lo))
    frame_hi = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - t_hi))
    print(f"表示範囲: t={t_lo:.2f}s〜{t_hi:.2f}s (frame {frame_lo}-{frame_hi})")

    # --- 1. NPC位置（EGO近傍を細かく刻んだ抽象化、12.4節） ---
    fine_states = compress_to_fine_relation_states(rxs, rys, n_bins=4, max_range=6)
    npc_samples = []
    for s in fine_states:
        if s.end_frame < frame_lo or s.start_frame > frame_hi:
            continue
        t = gk[s.start_frame]["timestamp"]
        npc_samples.append((t, f"lane={s.lane_fine:+d},pos={s.position_fine:+d}"))
    npc_segments = _compress_segments(npc_samples, t_end_last=gk[min(frame_hi, len(gk) - 1)]["timestamp"])

    # --- 2. 減速の十分性（±0.5秒の平滑化windowで、control_cmdsのサンプルごとに評価） ---
    half_win = 36  # ~0.5s at ~71Hz (groundtruth_kinematic基準)
    decel_samples = []
    for e in cc:
        t = e["timestamp"]
        if t < t_lo or t > t_hi:
            continue
        i = min(range(len(gk)), key=lambda k: abs(gk[k]["timestamp"] - t))
        i0, i1 = max(0, i - half_win), min(len(rxs) - 1, i + half_win)
        dt = gk[i1]["timestamp"] - gk[i0]["timestamp"]
        closing = (rxs[i0] - rxs[i1]) / dt if dt > 0 else 0.0
        dist = rxs[i] - (eh_l + nh_l)
        required = required_deceleration_magnitude(closing, dist)
        achieved = abs(e["longitudinal"]["acceleration"])
        label = classify_deceleration_adequacy(achieved, required)
        decel_samples.append((t, label))
    decel_segments = _compress_segments(decel_samples, t_end_last=t_hi)

    # --- 3. NPC予測経路の信頼性（predict_pathsのサンプル間隔=約0.093秒ごと） ---
    pred_samples = []
    for e in po:
        t = e["timestamp"]
        if t < t_lo or t > t_hi or not e["objects"]:
            continue
        obj = e["objects"][0]
        paths = obj.get("predict_paths", [])
        if not paths:
            continue
        best_path = max(paths, key=lambda p: p["confidence"])
        ex, ey, fwd, left = ego_basis_at(gk, t)
        cur_rx, cur_ry = project(obj["pose"]["position"]["x"], obj["pose"]["position"]["y"], ex, ey, fwd, left)
        horizon_idx = min(3, len(best_path["path"]) - 1)
        horizon_s = horizon_idx * 0.5
        pp = best_path["path"][horizon_idx]
        pred_rx, pred_ry = project(pp["position"]["x"], pp["position"]["y"], ex, ey, fwd, left)
        predicted_delta = pred_ry - cur_ry
        future_ts = t + horizon_s
        future_frame = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - future_ts))
        actual_delta = rys[future_frame] - cur_ry
        label = classify_prediction_reliability(best_path["confidence"], predicted_delta, actual_delta)
        pred_samples.append((t, label))
    pred_segments = _compress_segments(pred_samples, t_end_last=t_hi)

    # --- 4. 接触余裕（フレームごと） ---
    coll_set = set(coll_frames)
    contact_samples = []
    for i in range(frame_lo, frame_hi + 1):
        label = classify_contact_margin(rys[i], eh_w, nh_w, is_colliding=(i in coll_set))
        contact_samples.append((gk[i]["timestamp"], label))
    contact_segments = _compress_segments(contact_samples, t_end_last=t_hi)

    print(f"NPC位置: {len(npc_segments)}区間, 減速の十分性: {len(decel_segments)}区間, "
          f"予測信頼性: {len(pred_segments)}区間, 接触余裕: {len(contact_segments)}区間")

    path = plot_abstract_cause_timeline(
        OUT_PATH,
        npc_box_segments=npc_segments,
        decel_segments=decel_segments,
        pred_segments=pred_segments,
        contact_segments=contact_segments,
        onset_ts=onset_ts,
        collision_window=(window_ts0, window_ts1),
        title="TD-NI-AR-SD-N04-CI-0067: 抽象解釈演算子による原因の可視化（時間軸）",
    )
    print(f"図を書き出しました: {path}")


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
