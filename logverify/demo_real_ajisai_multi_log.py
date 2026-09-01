"""Method C exercised against SEVERAL real AJISAI logs at once (not just one).

demo_real_ajisai_log.py used a single real AJISAI log
(TD-NI-AR-SD-N04-CI-0035.json). This demo extends that to a small set of
real cut-in logs pulled from the same series in the AJISAI dataset
(J-Storage/Box, folder AJISAI/data/cutin/), all sharing the same scenario
tags (road=non_intersection, position=ahead_right, direction=same_direction,
behavior=cut_in) per jama_summary.csv:

    TD-NI-AR-SD-N04-CI-0030.json
    TD-NI-AR-SD-N04-CI-0032.json
    TD-NI-AR-SD-N04-CI-0035.json
    TD-NI-AR-SD-N04-CI-0047.json
    TD-NI-AR-SD-N04-CI-0067.json
    TD-NI-AR-SD-N04-CI-0076.json

This is a first step towards the "real-data validation with ~10 logs" item
in docs/log_to_cpd_verification_design.md, section 11.2, item 1. With
multiple real logs, the near/far grid's auto_grid=True maximizing search
(find_distinguishing_near_far_grid) is meaningful again -- unlike the
single-log case in demo_real_ajisai_log.py, where it degenerates because
there is no second log to stay distinguishable from.

本デモは、demo_real_ajisai_log.py で使った1本の実AJISAIログ
（TD-NI-AR-SD-N04-CI-0035.json）を、同じシリーズの実cut-inログ数本に
拡張したものである。AJISAIデータセット（J-Storage/Box、
AJISAI/data/cutin/フォルダ）から、jama_summary.csv上で同じシナリオタグ
（road=non_intersection, position=ahead_right, direction=same_direction,
behavior=cut_in）を持つログを選んで使う（上記6ファイル）。

これは、設計ドキュメント（docs/log_to_cpd_verification_design.md）
11.2節・課題1「実データ10本程度での検証」に向けた第一歩である。複数の
実ログを使うことで、非一様格子のauto_grid=True（最大化探索、
find_distinguishing_near_far_grid）が再び意味を持つようになる
（demo_real_ajisai_log.py の1本だけのケースでは、区別すべき相手が
いないため探索が破綻していた点との対比）。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_real_ajisai_multi_log \
        <log1.json> <log2.json> ...
    (with no arguments, defaults to the 6 files listed above, looked up
    under /mnt/user-data/uploads/Downloads/)
"""

import os
import sys
import time

from logverify.grid_bridge import relative_xy_from_ajisai_groundtruth
from logverify.multi_log_model import (
    build_union_model,
    build_union_model_near_far_grid,
    find_distinguishing_near_far_grid,
    verify_logs_included,
    count_scenarios,
)
from logverify.model_diagram import plot_model_with_ego_paper_style
from logverify.world_frame_gif import render_world_frame_gif

OUT_DIR = "out_gif"
GY = 3.5

DEFAULT_LOG_DIR = "/mnt/user-data/uploads/Downloads"
DEFAULT_LOG_NAMES = [
    "TD-NI-AR-SD-N04-CI-0030.json",
    "TD-NI-AR-SD-N04-CI-0032.json",
    "TD-NI-AR-SD-N04-CI-0035.json",
    "TD-NI-AR-SD-N04-CI-0047.json",
    "TD-NI-AR-SD-N04-CI-0067.json",
    "TD-NI-AR-SD-N04-CI-0076.json",
]

# ログ記録がシナリオの本題（cut-in）が終わった後も続いていることがある
# （実際、この6本中 TD-NI-AR-SD-N04-CI-0047.json だけは、フレーム2546/3166
# 以降、NPCがego進行方向に対しほぼ真横に90m以上も離れていく「その後の
# 走行」を記録していた -- おそらく交差点を折れて別経路へ向かった等）。
# これをそのままMethod Cの入力にすると、そのログ1本のためだけに格子・
# モデルが不必要に大きく広がってしまう。ここでは「|ry|（ego進行方向に
# 直交する相対距離）が RY_TRIM_BOUND を超えたまま戻らなくなった時点」を
# シナリオ本題の終わりとみなし、そこで打ち切る。6本中5本はこの打ち切りの
# 影響を受けない（末尾までずっと|ry|<=RY_TRIM_BOUND のため）。
#
# (English) A log's recording sometimes continues after the cut-in
# maneuver itself is over. In fact, of these 6 logs, only
# TD-NI-AR-SD-N04-CI-0047.json has this issue: from frame 2546/3166 onward,
# the NPC drifts more than 90m sideways relative to ego's heading (likely
# turning off onto a different route after the interaction). Feeding that
# tail into Method C as-is would blow up the grid/model size for the sake
# of one log's post-scenario driving. Here we treat "the point after which
# |ry| (the lateral distance relative to ego's heading) exceeds
# RY_TRIM_BOUND and never comes back" as the end of the scenario's relevant
# portion, and trim there. 5 of the 6 logs are unaffected by this (their
# |ry| stays within RY_TRIM_BOUND all the way to the end).
RY_TRIM_BOUND = 15.0


def _trim_trailing_runaway(rel_xy, ry_bound=RY_TRIM_BOUND):
    """|ry|<=ry_bound を最後に満たすフレームの直後で打ち切る。

    ---
    English:
    Truncates right after the last frame satisfying |ry| <= ry_bound.
    """
    last_ok = max(
        (i for i, (_, ry) in enumerate(rel_xy) if abs(ry) <= ry_bound),
        default=len(rel_xy) - 1,
    )
    return rel_xy[: last_ok + 1]


def run(json_paths) -> None:
    print(f"Number of input real logs: {len(json_paths)}")
    trajectories = []
    for p in json_paths:
        rel_xy_full = relative_xy_from_ajisai_groundtruth(p)
        rel_xy = _trim_trailing_runaway(rel_xy_full)
        trimmed_note = ""
        if len(rel_xy) < len(rel_xy_full):
            trimmed_note = f"  (trimmed from {len(rel_xy_full)}: trailing |ry|>{RY_TRIM_BOUND}m run)"
        rxs = [pt[0] for pt in rel_xy]
        rys = [pt[1] for pt in rel_xy]
        print(
            f"  {os.path.basename(p):32s}: {len(rel_xy):5d} frames, "
            f"rx {min(rxs):7.2f}..{max(rxs):7.2f} m, ry {min(rys):6.2f}..{max(rys):6.2f} m{trimmed_note}"
        )
        trajectories.append(rel_xy)
    print()

    # --- Method C: union model on a uniform grid, built from all input logs at once.
    # We only report its size here (no membership check / enumeration): with 6 real
    # logs the uniform grid already reaches max_step ~70, and the same combinatorial
    # cost that made the Ego-synced animation (Section 11.7) impractical at that size
    # also makes plain SAT solving over the model slow -- a membership/enumeration
    # check that took a fraction of a second at max_step~30 (single-log case) did not
    # finish within several minutes here. This is itself useful evidence for why the
    # near/far grid below is necessary once several real logs are combined, not just
    # for animation. ---
    # (English 用のコメントを上に統合。日本語:)
    # 一様格子での統合モデルはサイズのみ報告する（membership check・列挙は行わない）。
    # 実ログ6本では一様格子だけでmax_step~70に達し、Ego同期アニメーションを
    # 実用外にした（11.7節）のと同じ組合せ的コストが、素のSATソルブにも
    # のしかかる -- 単一ログ（max_step~30）では一瞬で終わったmembership
    # check/列挙が、ここでは数分経っても終わらなかった。これは、アニメーション
    # だけでなく、実ログを複数本組み合わせた時点で非一様格子（近遠格子）が
    # 必要になることを裏付ける材料でもある。
    print("=== Method C: union model built from all input logs (uniform grid; size only) ===")
    t0 = time.time()
    mlm = build_union_model(trajectories)
    t1 = time.time()
    print(f"Auto-selected grid: gx={mlm.gx}, gy={mlm.gy}  ({t1 - t0:.2f}s)")
    print(f"Number of boxes: {len(mlm.model.boxes)} (including the dummy start box), max_step: {mlm.model.max_step}")
    print(
        "(Membership check / scenario enumeration skipped on this uniform-grid model -- "
        "did not complete within several minutes; see the near/far grid below instead.)"
    )
    print()

    # --- Near/far grid, now meaningful with multiple logs: auto_grid=True
    # maximizes how coarse the far cell can be while keeping all 6 logs
    # distinguishable from each other. ---
    print("=== Method C with a near/far grid (auto-maximized far cell; meaningful with >=2 logs) ===")
    t6 = time.time()
    far_cell_used, _ = find_distinguishing_near_far_grid(
        trajectories, rx_near_cell=5.0, rx_near_range=20.0, gy=GY
    )
    mlm_nf = build_union_model_near_far_grid(
        trajectories,
        rx_near_cell=5.0,
        rx_far_cell=far_cell_used,
        rx_near_range=20.0,
        gy=GY,
        auto_grid=False,  # already found a distinguishing far cell above
    )
    t7 = time.time()
    print(f"Auto-selected far cell size: {far_cell_used}m  (near cell: 5.0m, boundary: 20.0m)  ({t7 - t6:.2f}s)")
    print(f"Number of boxes: {len(mlm_nf.model.boxes)}  (was {len(mlm.model.boxes)} on the uniform grid)")
    print(f"max_step: {mlm_nf.model.max_step}  (was {mlm.model.max_step} on the uniform grid)")

    t8 = time.time()
    results_nf = verify_logs_included(mlm_nf)
    t9 = time.time()
    n_ok_nf = sum(r.is_member for r in results_nf)
    print(f"Membership check: {n_ok_nf}/{len(json_paths)} logs included (SAT) ({t9 - t8:.2f}s)")

    t10 = time.time()
    n_scenarios_nf = count_scenarios(mlm_nf)
    t11 = time.time()
    print(f"Number of enumerated scenarios: {n_scenarios_nf} (input logs: {len(json_paths)}, {t11 - t10:.2f}s)")
    if n_scenarios_nf > len(json_paths):
        print(
            f"-> {n_scenarios_nf - len(json_paths)} additional scenarios beyond the input logs: some "
            "combination of real trajectory noise (see demo_real_ajisai_log.py, Section 11.8) and "
            "genuine cross-log path merging at this grid resolution."
        )
    print()

    # --- Visualization: Ego-synchronized world-frame animation, enumerating
    # only a handful of scenarios (num_model=5), plus the model structure
    # diagram (pure drawing, no solving). ---
    print("=== Visualization ===")
    os.makedirs(OUT_DIR, exist_ok=True)
    t12 = time.time()
    paths = render_world_frame_gif(
        mlm_nf.model,
        os.path.join(OUT_DIR, "real_ajisai_multilog_with_ego"),
        combined=True,
        ego_speed=1.0,
        num_model=5,
    )
    t13 = time.time()
    print(f"Ego-synchronized animation (num_model=5): {t13 - t12:.2f}s -> {paths}")

    diagram_path = plot_model_with_ego_paper_style(
        mlm_nf.model,
        mlm_nf.box_id_of,
        os.path.join(OUT_DIR, "model_real_ajisai_multilog_with_ego.png"),
        car="NPC",
        ego_lane=0,
        ego_max_step=mlm_nf.model.max_step,
        title=f"Real AJISAI cut-in logs ({len(json_paths)}) - Method C union model + Ego",
    )
    print(f"Model structure diagram: {diagram_path}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        paths = sys.argv[1:]
    else:
        paths = [os.path.join(DEFAULT_LOG_DIR, name) for name in DEFAULT_LOG_NAMES]
    run(paths)
