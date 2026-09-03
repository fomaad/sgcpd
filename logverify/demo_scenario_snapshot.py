"""`scenario_snapshot_diagram.py`のデモ。

12.9〜12.11節と同じ、ヒステリシス付きnear/far格子で圧縮したCPDの箱列
（`grid_bridge.compress_to_grid_states_variable_hysteresis`、
`auto_grid`で自動導出したパラメータを使用）を、衝突ウィンドウ前後に
絞り込み、各箱について
  - EGO/NPCの位置関係（Ve0・Vo0・Vy・dx0・dy0）
  - 12.12節の3つの抽象解釈演算子（減速の十分性・NPC予測の信頼性・
    接触余裕）の分類結果
を計算し、1箱=1パネルのスナップショット列として可視化する。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_scenario_snapshot \
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
from logverify.auto_grid import auto_grid_params_from_ajisai
from logverify.grid_bridge import (
    compress_to_grid_states_variable_hysteresis,
    relative_xy_from_ajisai_groundtruth,
)
from logverify.scenario_snapshot_diagram import ScenarioSnapshot, plot_scenario_snapshot_sequence

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_PATH = "out_gif/collision_0067_scenario_snapshots.png"


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


def speed_at(gk_entry, key):
    v = gk_entry[key]["twist"]["linear"]
    return math.hypot(v["x"], v["y"])


def npc_speed_at(gk_entry):
    vs = gk_entry.get("groundtruth_vehicles", [])
    if not vs:
        return 0.0
    v = vs[0]["twist"]["linear"]
    return math.hypot(v["x"], v["y"])


def lateral_speed_at(rys, gk, frame, half_win=5):
    i0, i1 = max(0, frame - half_win), min(len(rys) - 1, frame + half_win)
    dt = gk[i1]["timestamp"] - gk[i0]["timestamp"]
    if dt <= 0:
        return 0.0
    return (rys[i1] - rys[i0]) / dt


def decel_label_at(rxs, gk, cc, frame, eh_l, nh_l, half_win=36):
    i0, i1 = max(0, frame - half_win), min(len(rxs) - 1, frame + half_win)
    dt = gk[i1]["timestamp"] - gk[i0]["timestamp"]
    closing = (rxs[i0] - rxs[i1]) / dt if dt > 0 else 0.0
    dist = rxs[frame] - (eh_l + nh_l)
    required = required_deceleration_magnitude(closing, dist)
    ts = gk[frame]["timestamp"]
    nearest = min(cc, key=lambda e: abs(e["timestamp"] - ts))
    achieved = abs(nearest["longitudinal"]["acceleration"])
    return classify_deceleration_adequacy(achieved, required)


def pred_label_at(po, gk, rys, ts, ego_speed_key="groundtruth_ego"):
    candidates = [e for e in po if e["objects"]]
    if not candidates:
        return None
    nearest = min(candidates, key=lambda e: abs(e["timestamp"] - ts))
    if abs(nearest["timestamp"] - ts) > 0.3:
        return None
    obj = nearest["objects"][0]
    paths = obj.get("predict_paths", [])
    if not paths:
        return None
    best_path = max(paths, key=lambda p: p["confidence"])
    ex, ey, fwd, left = ego_basis_at(gk, nearest["timestamp"])
    cur_rx, cur_ry = project(obj["pose"]["position"]["x"], obj["pose"]["position"]["y"], ex, ey, fwd, left)
    horizon_idx = min(3, len(best_path["path"]) - 1)
    horizon_s = horizon_idx * 0.5
    pp = best_path["path"][horizon_idx]
    pred_rx, pred_ry = project(pp["position"]["x"], pp["position"]["y"], ex, ey, fwd, left)
    predicted_delta = pred_ry - cur_ry
    future_ts = nearest["timestamp"] + horizon_s
    future_frame = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - future_ts))
    actual_delta = rys[future_frame] - cur_ry
    return classify_prediction_reliability(best_path["confidence"], predicted_delta, actual_delta)


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
    coll_set = set(coll_frames)
    window_first, window_last = coll_frames[0], coll_frames[-1]
    onset_frame = detect_cutin_onset_frame(rys, window_first)
    onset_ts = gk[onset_frame]["timestamp"]
    window_ts0, window_ts1 = gk[window_first]["timestamp"], gk[window_last]["timestamp"]
    t_lo, t_hi = onset_ts - 1.0, window_ts1 + 1.0
    frame_lo = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - t_lo))
    frame_hi = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - t_hi))
    print(f"表示範囲: t={t_lo:.2f}s〜{t_hi:.2f}s (frame {frame_lo}-{frame_hi})")

    print("=== 車両サイズから格子パラメータを自動導出（12.11節） ===")
    auto = auto_grid_params_from_ajisai(json_path)
    print(f"gy={auto.gy:.3f}, rx_near_cell={auto.rx_near_cell:.3f}, "
          f"rx_near_range={auto.rx_near_range:.3f}, rx_far_cell={auto.rx_far_cell:.3f}")

    states = compress_to_grid_states_variable_hysteresis(
        rxs, rys, auto.rx_near_cell, auto.rx_far_cell, auto.rx_near_range, auto.gy, margin_ratio=0.3,
    )
    sub_states = [s for s in states if s.end_frame >= frame_lo and s.start_frame <= frame_hi]
    print(f"CPDの箱数（全体）: {len(states)}, 表示範囲内の箱数: {len(sub_states)}")

    snapshots = []
    for s in sub_states:
        frame = (s.start_frame + s.end_frame) // 2
        ts = gk[frame]["timestamp"]
        rx, ry = rel_xy[frame]
        ego_speed = speed_at(gk[frame], "groundtruth_ego")
        npc_speed = npc_speed_at(gk[frame])
        vy = lateral_speed_at(rys, gk, frame)
        decel_label = decel_label_at(rxs, gk, cc, frame, eh_l, nh_l)
        pred_label = pred_label_at(po, gk, rys, ts)
        contact_label = classify_contact_margin(ry, eh_w, nh_w, is_colliding=(frame in coll_set))
        snapshots.append(ScenarioSnapshot(
            box_index=s.index, t=ts, rx=rx, ry=ry,
            ego_speed=ego_speed, npc_speed=npc_speed, npc_lateral_speed=vy,
            decel_label=decel_label, pred_label=pred_label, contact_label=contact_label,
            lane_k=s.k, pos_i=s.i,
        ))
        print(f"  box#{s.index} (k={s.k},i={s.i}) frame={frame} t={ts - onset_ts:+.2f}s "
              f"rx={rx:.2f} ry={ry:+.2f} Ve0={ego_speed:.2f} Vo0={npc_speed:.2f} Vy={vy:+.2f} "
              f"-> 減速:{decel_label} 予測:{pred_label} 余裕:{contact_label}")

    path = plot_scenario_snapshot_sequence(
        snapshots, OUT_PATH,
        ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
        title="TD-NI-AR-SD-N04-CI-0067: シナリオ・スナップショット列（= CPDの箱列）",
        t_ref=onset_ts,
    )
    print(f"図を書き出しました: {path}")


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
