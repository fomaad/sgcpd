"""12.25節の最終まとめ: (1)〜(5)の5つの抽象化それぞれについて、
ログ0067での箱列（EGO/NPCの相対位置つき）を可視化する。

ユーザーとの議論を経て確定した5つの比較対象（案A）:
  (1) 車両物理サイズ基準（一様格子の特殊系。cell=0.9526m、near/farの
      区別なし——「車両サイズを基準にするならnear/farを分ける理由が
      ない」という指摘を反映）
  (2) 一様格子ベースライン（cell=2.0m。車両サイズにも安全性モデルにも
      由来しない恣意的な値）
  (3) JAMA C&C述語抽象化（`safety_predicate_abstraction.py`。C&C自身の
      状態変数で近傍を区切り、遠方は単一のFAR箱に潰す。13箱）
  (4) RSS述語抽象化（同上のRSS版。13箱）
  (5) 参考: 訂正前のC&C基準near/far計量格子（near_rangeだけをC&Cの
      onsetに連動させ、near_cell/far_cellは車両サイズ由来のまま。91箱。
      「述語抽象化に直してどれだけ改善したか」を示す比較対象として残す）

(1)(2)(5)は箱数が多く(91〜320)紙面に収まらないため、最接近フレーム
付近の直近25箱に絞って表示する（12.24節`MAX_BOXES`と同じ方針）。
(3)(4)は13箱(16run)しかないため全run(16枚)をそのまま表示する。

各箱(run)の代表位置(rx, ry)は、その箱に属するフレームの実測値の平均
（近似逆写像ではなく、実際のログの値そのもの）を使う。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.visualize_five_abstractions
"""

import statistics
from dataclasses import dataclass
from typing import Hashable, List, Optional

from logverify.synth_thresholds_multilog import _load, vehicle_sizes, relative_xy, closest_approach_frame
from logverify.reference_model_comparison import compute_ttc, ego_speed_series
from logverify.jama_cc_model import find_risk_perceived_frame
from logverify.rss_model import npc_speed_series, find_rss_risk_frame
from logverify.auto_grid import auto_grid_params_from_ajisai, auto_grid_params_naive_uniform, AutoGridParams
from logverify.grid_bridge import compress_to_grid_states_variable_hysteresis
from logverify.safety_predicate_abstraction import (
    compress_by_label, cc_predicate_label_fn, rss_predicate_label_fn,
)
from logverify.scenario_snapshot_diagram import ScenarioSnapshot, plot_scenario_snapshot_sequence

LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_DIR = "out_gif/five_abstractions"
MAX_BOXES = 25


@dataclass
class Run:
    box_id: Hashable
    start_frame: int
    end_frame: int


def _mean_or(values, default=0.0):
    values = [v for v in values if v is not None]
    return statistics.mean(values) if values else default


def make_snapshots(runs: List[Run], rxs, rys, timestamps, ego_speed, npc_speed, eh_l, eh_w, nh_l, nh_w) -> List[ScenarioSnapshot]:
    snapshots = []
    for r in runs:
        frames = range(r.start_frame, r.end_frame + 1)
        rx = _mean_or([rxs[f] for f in frames])
        ry = _mean_or([rys[f] for f in frames])
        t = _mean_or([timestamps[f] for f in frames])
        ev = _mean_or([ego_speed[f] for f in frames])
        nv = _mean_or([npc_speed[f] for f in frames]) if npc_speed else 0.0
        risk2d = max(abs(rx) / (eh_l + nh_l), abs(ry) / (eh_w + nh_w))
        contact_label = "接触" if risk2d < 1.0 else None
        snapshots.append(ScenarioSnapshot(
            box_index=_format_label(r.box_id), t=t, rx=rx, ry=ry,
            ego_speed=ev, npc_speed=nv, npc_lateral_speed=0.0,
            contact_label=contact_label,
        ))
    return snapshots


def runs_from_grid_states(states) -> List[Run]:
    return [Run(box_id=s.index, start_frame=s.start_frame, end_frame=s.end_frame) for s in states]


def _format_label(label) -> str:
    """('RISK', -3) -> 'RISK k=-3'、('FAR',) -> 'FAR'、('CONTACT',) -> 'CONTACT'。"""
    if isinstance(label, tuple):
        if len(label) == 1:
            return str(label[0])
        return f"{label[0]} k={label[1]}"
    return str(label)


def truncate_near_closest(runs: List[Run], closest_frame: int, max_boxes: int) -> List[Run]:
    """最接近フレームまでの直近max_boxes個の箱に絞る(12.24節と同じ方針)。"""
    before = [r for r in runs if r.start_frame <= closest_frame]
    after = [r for r in runs if r.start_frame > closest_frame]
    kept_before = before[-max_boxes:]
    return kept_before + after[: max(0, 3)]  # 最接近後も少しだけ見せる


def run():
    import os
    os.makedirs(OUT_DIR, exist_ok=True)

    data = _load(LOG_PATH)
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)
    timestamps = [rec["timestamp"] for rec in gk]
    ego_speed = ego_speed_series(gk)
    npc_speed = npc_speed_series(data)
    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)

    cc_risk_frame, _, _ = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    rss_risk_frame, _ = find_rss_risk_frame(rxs, ego_speed, npc_speed)
    closest_frame, _ = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)

    geometry_grid = auto_grid_params_from_ajisai(LOG_PATH)
    rx_extent = max(abs(v) for v in rxs if v is not None)
    gy = geometry_grid.gy
    n = len(rxs)
    valid = [rxs[i] is not None and rys[i] is not None for i in range(n)]

    variants = []

    # (1) 車両物理サイズ基準 -- 一様格子の特殊系
    veh_uniform = auto_grid_params_naive_uniform(rx_extent, cell_width=geometry_grid.rx_near_cell, gy=gy)
    states1 = compress_to_grid_states_variable_hysteresis(
        rxs, rys, veh_uniform.rx_near_cell, veh_uniform.rx_far_cell, veh_uniform.rx_near_range, gy, margin_ratio=0.3)
    runs1 = truncate_near_closest(runs_from_grid_states(states1), closest_frame, MAX_BOXES)
    variants.append(("1_vehicle_size_uniform", "(1) 車両物理サイズ基準（一様格子, cell=0.95m）", runs1, len(states1)))

    # (2) 一様格子ベースライン
    baseline = auto_grid_params_naive_uniform(rx_extent, cell_width=2.0, gy=gy)
    states2 = compress_to_grid_states_variable_hysteresis(
        rxs, rys, baseline.rx_near_cell, baseline.rx_far_cell, baseline.rx_near_range, gy, margin_ratio=0.3)
    runs2 = truncate_near_closest(runs_from_grid_states(states2), closest_frame, MAX_BOXES)
    variants.append(("2_baseline_uniform", "(2) 一様格子ベースライン（cell=2.0m）", runs2, len(states2)))

    # (3) JAMA C&C述語抽象化
    cc_label_fn = cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx=40.0, gy=gy)
    cc_runs_raw, cc_boxes = compress_by_label(n, valid, cc_label_fn)
    runs3 = [Run(box_id=r.label, start_frame=r.start_frame, end_frame=r.end_frame) for r in cc_runs_raw]
    variants.append(("3_jama_cc_predicate", "(3) JAMA C&C述語抽象化", runs3, len(cc_boxes)))

    # (4) RSS述語抽象化
    rss_label_fn = rss_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, rss_risk_frame, near_rx=40.0, gy=gy)
    rss_runs_raw, rss_boxes = compress_by_label(n, valid, rss_label_fn)
    runs4 = [Run(box_id=r.label, start_frame=r.start_frame, end_frame=r.end_frame) for r in rss_runs_raw]
    variants.append(("4_rss_predicate", "(4) RSS述語抽象化", runs4, len(rss_boxes)))

    # (5) 参考: 訂正前のC&C基準near/far計量格子
    cc_metric_grid = AutoGridParams(
        gy=gy, rx_near_cell=geometry_grid.rx_near_cell,
        rx_near_range=1.2 * abs(rxs[cc_risk_frame]), rx_far_cell=geometry_grid.rx_far_cell,
    )
    states5 = compress_to_grid_states_variable_hysteresis(
        rxs, rys, cc_metric_grid.rx_near_cell, cc_metric_grid.rx_far_cell, cc_metric_grid.rx_near_range, gy, margin_ratio=0.3)
    runs5 = truncate_near_closest(runs_from_grid_states(states5), closest_frame, MAX_BOXES)
    variants.append(("5_ref_cc_metric_grid", "(5) 参考: 訂正前のC&C基準near/far計量格子", runs5, len(states5)))

    paths = []
    for key, label, runs, total_boxes in variants:
        snapshots = make_snapshots(runs, rxs, rys, timestamps, ego_speed, npc_speed, eh_l, eh_w, nh_l, nh_w)
        out_path = f"{OUT_DIR}/{key}.png"
        n_shown = len(snapshots)
        title = f"{label}　全箱数={total_boxes}　表示={n_shown}箱" + ("（最接近付近に絞って表示）" if n_shown < total_boxes else "")
        plot_scenario_snapshot_sequence(
            snapshots, out_path,
            ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
            title=title, show_time=False, transition_arrow_style="boxes",
            panel_w_in=1.6, panel_h_in=2.1,
        )
        print(f"{label}: 全箱数={total_boxes} 表示箱数={n_shown} -> {out_path}")
        paths.append(out_path)

    return paths


if __name__ == "__main__":
    run()
