"""12.25節の最終まとめ: (1)〜(5)の5つの抽象化について、真の箱数
(distinct box数)とpurityをグラフにする。`visualize_five_abstractions.py`
が作る箱列可視化（シナリオ図）と対になる、数値サマリのグラフ。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.plot_five_abstractions_summary
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from logverify.synth_thresholds_multilog import _load, vehicle_sizes, relative_xy
from logverify.reference_model_comparison import compute_ttc, ego_speed_series
from logverify.jama_cc_model import find_risk_perceived_frame
from logverify.rss_model import npc_speed_series, find_rss_risk_frame
from logverify.auto_grid import auto_grid_params_from_ajisai, auto_grid_params_naive_uniform, AutoGridParams
from logverify.grid_bridge import compress_to_grid_states_variable_hysteresis
from logverify.safety_predicate_abstraction import compress_by_label, cc_predicate_label_fn, rss_predicate_label_fn
from logverify.compare_safety_model_abstractions import _purity_for_onset

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_PATH = "out_gif/five_abstractions_summary.png"


def _smear(purity: dict) -> float:
    if not purity.get("applicable"):
        return 0.0
    return 0.0 if purity.get("pure") else purity.get("box_rx_span_m", 0.0)


def collect():
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

    geometry_grid = auto_grid_params_from_ajisai(LOG_PATH)
    rx_extent = max(abs(v) for v in rxs if v is not None)
    gy = geometry_grid.gy
    n = len(rxs)
    valid = [rxs[i] is not None and rys[i] is not None for i in range(n)]

    results = []

    # (1) vehicle-size uniform
    g1 = auto_grid_params_naive_uniform(rx_extent, cell_width=geometry_grid.rx_near_cell, gy=gy)
    s1 = compress_to_grid_states_variable_hysteresis(rxs, rys, g1.rx_near_cell, g1.rx_far_cell, g1.rx_near_range, gy, margin_ratio=0.3)
    results.append(("(1) 車両物理サイズ基準\n(一様格子, 0.95m)", len(s1), _smear(_purity_for_onset(rxs, s1, cc_risk_frame)), _smear(_purity_for_onset(rxs, s1, rss_risk_frame))))

    # (2) baseline uniform
    g2 = auto_grid_params_naive_uniform(rx_extent, cell_width=2.0, gy=gy)
    s2 = compress_to_grid_states_variable_hysteresis(rxs, rys, g2.rx_near_cell, g2.rx_far_cell, g2.rx_near_range, gy, margin_ratio=0.3)
    results.append(("(2) 一様格子ベースライン\n(cell=2.0m)", len(s2), _smear(_purity_for_onset(rxs, s2, cc_risk_frame)), _smear(_purity_for_onset(rxs, s2, rss_risk_frame))))

    # (3) C&C predicate
    cc_fn = cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx=40.0, gy=gy)
    _, cc_boxes = compress_by_label(n, valid, cc_fn)
    results.append(("(3) JAMA C&C\n述語抽象化", len(cc_boxes), 0.0, None))  # own-onset pure by construction; RSS onset not evaluated (different model's own abstraction)

    # (4) RSS predicate
    rss_fn = rss_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, rss_risk_frame, near_rx=40.0, gy=gy)
    _, rss_boxes = compress_by_label(n, valid, rss_fn)
    results.append(("(4) RSS\n述語抽象化", len(rss_boxes), None, 0.0))

    # (5) reference: pre-correction C&C metric grid
    g5 = AutoGridParams(gy=gy, rx_near_cell=geometry_grid.rx_near_cell,
                         rx_near_range=1.2 * abs(rxs[cc_risk_frame]), rx_far_cell=geometry_grid.rx_far_cell)
    s5 = compress_to_grid_states_variable_hysteresis(rxs, rys, g5.rx_near_cell, g5.rx_far_cell, g5.rx_near_range, gy, margin_ratio=0.3)
    results.append(("(5) 参考: 訂正前の\nC&C基準near/far格子", len(s5), _smear(_purity_for_onset(rxs, s5, cc_risk_frame)), _smear(_purity_for_onset(rxs, s5, rss_risk_frame))))

    return results


def plot(results, output_path=OUT_PATH):
    labels = [r[0] for r in results]
    n_boxes = [r[1] for r in results]
    colors = ["#78909c", "#757575", "#1565c0", "#ef6c00", "#9e9e9e"]

    fig, ax1 = plt.subplots(figsize=(9, 5.5))
    bars = ax1.bar(labels, n_boxes, color=colors)
    ax1.set_yscale("log")
    for i, v in enumerate(n_boxes):
        ax1.text(i, v * 1.15, str(v), ha="center", fontsize=10, fontweight="bold")
    ax1.set_ylabel("真の箱数 (distinct boxes, 対数軸)")
    ax1.set_title("(1)〜(5) 真の箱数の比較（ログ0067、1本での予備実験）", fontsize=12)
    ax1.tick_params(axis="x", labelsize=8.5)
    ax1.set_ylim(1, max(n_boxes) * 3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output_path


if __name__ == "__main__":
    results = collect()
    for r in results:
        print(r[0].replace("\n", " "), "boxes=", r[1])
    path = plot(results)
    print(f"図を書き出しました: {path}")
