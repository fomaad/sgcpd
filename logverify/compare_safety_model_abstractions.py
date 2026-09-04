"""12.25節: safety-model-guided abstraction and refinement の比較実験。

1本のログ(0067)に対して、格子(near/far grid)の粒度を決める4通りの方法
（(1)車両物理サイズ基準=これまでの方法, (2)JAMA C&Cモデル基準,
(3)RSS基準, (4)何も特徴を考えない一様格子=ベースライン）を適用し、

  - 抽象化の度合い: 箱(スナップショット)の数
  - 抽象化で大事なところをつぶしていないか: 各safety modelの
    risk-perceived/violationフレーム(onset)が、格子上でも1つの箱の
    境界としてちょうど分離されているか(pure)、それとも1つの箱の中に
    埋もれてしまっているか(impure -- 情報損失)
  - 構築コスト: 格子への圧縮 + gcpd.Model構築の所要時間

を比較する。Z3のmembership check (`verify_logs_included`)は、11.6/11.7・
12.24節で確認済みの通り箱数に応じて非常に重くなりうる(12.24節では
300箱弱のログで75秒以上)ため、ここでは実行しない(箱数そのものが
その後段のZ3コストの支配的な入力サイズであることは既に分かっている
ので、箱数の比較が実質的なスケーラビリティの proxy になる)。

---
English:
Section 12.25: comparison experiment for safety-model-guided abstraction
and refinement.

Applies 4 different ways of choosing the near/far grid's granularity to a
single log (0067): (1) vehicle physical size (the method used so far),
(2) the JAMA C&C model's own risk boundary, (3) RSS's own violation
boundary, (4) a naive uniform grid that ignores all features (baseline).
Compares:

  - degree of abstraction: number of boxes (snapshots)
  - whether anything important was collapsed: whether each safety model's
    own risk-onset frame lands exactly on a box boundary in the resulting
    grid (pure) or gets buried inside a single box (impure -- information
    loss)
  - construction cost: time to compress to the grid + build the
    gcpd.Model

The Z3 membership check (`verify_logs_included`) is skipped here, since
Sections 11.6/11.7 and 12.24 already established that it can become very
expensive as box count grows (75+ seconds for a log with under 300 boxes
in Section 12.24) -- box count is itself the dominant input size for that
downstream Z3 cost, so comparing box counts already serves as a practical
scalability proxy.

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.compare_safety_model_abstractions
"""

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from logverify.synth_thresholds_multilog import _load, vehicle_sizes, relative_xy
from logverify.reference_model_comparison import compute_ttc, ego_speed_series
from logverify.jama_cc_model import find_risk_perceived_frame
from logverify.rss_model import npc_speed_series, find_rss_risk_frame
from logverify.auto_grid import (
    auto_grid_params_from_ajisai, auto_grid_params, auto_near_range_from_risk_frame,
    auto_grid_params_naive_uniform, AutoGridParams,
)
from logverify.grid_bridge import compress_to_grid_states_variable_hysteresis
from logverify.multi_log_model import build_single_log_model_hysteresis

LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"


@dataclass
class VariantResult:
    name: str
    label_ja: str
    grid: AutoGridParams
    n_boxes: int
    build_time_s: float
    purity: Dict[str, dict] = field(default_factory=dict)  # model_name -> {pure, box_i_of_onset, box_rx_span_m, n_frames_in_box}


def _box_span_containing_frame(rxs, states, target_frame: Optional[int]):
    """target_frameを含む箱(GridState)のrx方向の実座標での広がり(min,max)
    と、その箱がまたぐフレーム数を返す。target_frameがNone、または
    どの箱にも属さない(欠測)場合はNoneを返す。"""
    if target_frame is None:
        return None
    for s in states:
        if s.start_frame <= target_frame <= s.end_frame:
            span = [rxs[f] for f in range(s.start_frame, s.end_frame + 1) if rxs[f] is not None]
            if not span:
                return None
            return {
                "box_index": s.index,
                "rx_min": min(span),
                "rx_max": max(span),
                "n_frames": s.end_frame - s.start_frame + 1,
                "start_frame": s.start_frame,
                "end_frame": s.end_frame,
            }
    return None


def _purity_for_onset(rxs, states, onset_frame: Optional[int]) -> dict:
    """onset_frame(あるsafety modelのrisk/violation開始フレーム)が、
    格子上で1つの箱の境界としてちょうど分離されているか(pure)を判定する。

    「pure」の定義: onset_frameを含む箱が、onset_frameより前のフレーム
    (pre-onset)を1つも含んでいない、かつ、その箱の直前の箱(あれば)が
    onset_frame以降のフレームを含んでいない -- すなわち、onset_frameが
    ちょうど箱の開始フレームと一致している状態。そうでなければimpure
    (pre-onsetとpost-onsetが同じ箱に混在しており、格子だけを見ても
    「もうrisk/violationが始まっているかどうか」を区別できない)。
    """
    if onset_frame is None:
        return {"applicable": False}
    box = _box_span_containing_frame(rxs, states, onset_frame)
    if box is None:
        return {"applicable": False}
    pure = box["start_frame"] == onset_frame
    return {
        "applicable": True,
        "pure": pure,
        "onset_frame": onset_frame,
        "box_index": box["box_index"],
        "box_start_frame": box["start_frame"],
        "box_end_frame": box["end_frame"],
        "box_rx_span_m": round(box["rx_max"] - box["rx_min"], 3),
        "n_frames_smeared_before_onset": onset_frame - box["start_frame"],
    }


def run():
    data = _load(LOG_PATH)
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)
    timestamps = [rec["timestamp"] for rec in gk]
    ego_speed = ego_speed_series(gk)
    npc_speed = npc_speed_series(data)
    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)

    # --- 各safety modelの onset frame を求める (格子とは独立) ---
    cc_risk_frame, _, _ = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    rss_risk_frame, _ = find_rss_risk_frame(rxs, ego_speed, npc_speed)
    print(f"JAMA C&C risk-perceived frame: {cc_risk_frame}")
    print(f"RSS violation-onset frame:     {rss_risk_frame}")
    print()

    rx_extent = max(abs(v) for v in rxs if v is not None)

    # --- 4通りのgrid ---
    geometry_grid = auto_grid_params_from_ajisai(LOG_PATH)
    cc_grid = AutoGridParams(
        gy=geometry_grid.gy,
        rx_near_cell=geometry_grid.rx_near_cell,
        rx_near_range=auto_near_range_from_risk_frame(rxs, cc_risk_frame),
        rx_far_cell=geometry_grid.rx_far_cell,
    )
    rss_grid = AutoGridParams(
        gy=geometry_grid.gy,
        rx_near_cell=geometry_grid.rx_near_cell,
        rx_near_range=auto_near_range_from_risk_frame(rxs, rss_risk_frame),
        rx_far_cell=geometry_grid.rx_far_cell,
    )
    baseline_grid = auto_grid_params_naive_uniform(rx_extent, cell_width=2.0, gy=geometry_grid.gy)

    variants = [
        ("geometry", "(1) 車両物理サイズ基準（従来）", geometry_grid),
        ("jama_cc", "(2) JAMA C&Cモデル基準", cc_grid),
        ("rss", "(3) RSSモデル基準", rss_grid),
        ("baseline_uniform", "(4) 一様格子（ベースライン）", baseline_grid),
    ]

    results: List[VariantResult] = []
    for name, label_ja, grid in variants:
        t0 = time.perf_counter()
        states = compress_to_grid_states_variable_hysteresis(
            rxs, rys, grid.rx_near_cell, grid.rx_far_cell, grid.rx_near_range, grid.gy, margin_ratio=0.3,
        )
        # also build the gcpd.Model itself (pure Python, cheap) to report
        # a construction time that includes what verify_logs_included
        # would need as input, without running the expensive Z3 check.
        rel_xy = list(zip(rxs, rys))
        build_single_log_model_hysteresis(
            rel_xy, grid.rx_near_cell, grid.rx_far_cell, grid.rx_near_range, grid.gy, margin_ratio=0.3,
        )
        build_time = time.perf_counter() - t0

        purity = {
            "jama_cc": _purity_for_onset(rxs, states, cc_risk_frame),
            "rss": _purity_for_onset(rxs, states, rss_risk_frame),
        }
        results.append(VariantResult(
            name=name, label_ja=label_ja, grid=grid, n_boxes=len(states),
            build_time_s=build_time, purity=purity,
        ))

    # --- 結果表示 ---
    print(f"{'variant':32s} {'gy':>6s} {'near_cell':>10s} {'near_range':>11s} {'far_cell':>9s} "
          f"{'n_boxes':>8s} {'build_ms':>9s}")
    for r in results:
        g = r.grid
        print(f"{r.label_ja:32s} {g.gy:6.3f} {g.rx_near_cell:10.3f} {g.rx_near_range:11.2f} "
              f"{g.rx_far_cell:9.2f} {r.n_boxes:8d} {r.build_time_s*1000:9.2f}")

    print()
    print("purity（安全性モデルのonsetフレームが箱の境界とちょうど一致しているか）:")
    for r in results:
        for model_name in ("jama_cc", "rss"):
            p = r.purity[model_name]
            if not p.get("applicable"):
                print(f"  {r.label_ja:32s} [{model_name:8s}] onsetフレームなし(該当なし)")
                continue
            status = "PURE " if p["pure"] else "IMPURE"
            extra = "" if p["pure"] else (
                f" -- 箱#{p['box_index']}が{p['n_frames_smeared_before_onset']}フレーム分"
                f"(rx方向{p['box_rx_span_m']}m)pre-onsetとpost-onsetを混在させている"
            )
            print(f"  {r.label_ja:32s} [{model_name:8s}] {status}{extra}")

    return results


if __name__ == "__main__":
    run()
