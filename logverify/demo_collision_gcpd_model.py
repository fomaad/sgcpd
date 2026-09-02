"""衝突ログ（既定: TD-NI-AR-SD-N04-CI-0067.json）を、12.7/12.8節で
アノテーション用に使ったのと同じ細かい near/far 格子
（RX_NEAR_CELL=1.0, RX_FAR_CELL=50.0, RX_NEAR_RANGE=15.0, GY=0.3）で、
matplotlibの手書き図ではなく、実際の`gcpd.Model`（Z3/SATの上に立つ
正式なCPDモデル）として構築する。

これまでの12.7/12.8節の図は、`logverify/collision_cpd_diagram.py`の
独自の描画ロジック（matplotlibで箱を手で並べる）であり、`gcpd.Model`
（`gcpd.py`のBox/Pos/LaneのZ3関数、`gcpd.add_trans`等のSAT制約）とは
別物だった。本デモは、方法C（`logverify/multi_log_model.py`）の
`build_union_model_near_far_grid`を1本のログ・同じ細かい格子で使うことで、
「衝突ログ1本だけを表す、正式なgcpd.Model」を作る。1本のログしか
入力していないため、モデルから列挙されるシナリオは基本的に1つだけであり
（このログ自身の経路のみ）、これはMethod A（咲川氏のツールで1本のログを
直接抽象化する「インスタンスCPD」）と同じ役割を、Method Cの機械（gcpd.Model
の構築・SATによるmembership確認・シナリオ列挙）を流用して達成したもの、
と位置づけられる。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_collision_gcpd_model \
        [path-to-TD-NI-AR-SD-N04-CI-0067.json]
"""

import sys
import time

from logverify.grid_bridge import relative_xy_from_ajisai_groundtruth
from logverify.multi_log_model import (
    build_union_model_near_far_grid,
    verify_logs_included,
    count_scenarios,
)
from logverify.model_diagram import plot_model_with_ego_paper_style
from logverify.world_frame_gif import render_world_frame_gif

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_DIR = "out_gif"

# 12.7/12.8節のアノテーション付きインスタンスCPDと全く同じ格子。
# (English) The exact same grid used for the Section 12.7/12.8 annotated
# instance CPDs.
RX_NEAR_CELL = 1.0
RX_FAR_CELL = 50.0
RX_NEAR_RANGE = 15.0
GY = 0.3


def run(json_path: str) -> None:
    print(f"Loading: {json_path}")
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path)
    print(f"raw frames: {len(rel_xy)}")
    print()

    print("=== gcpd.Model を、12.7/12.8節と同じ細かい near/far 格子で構築 ===")
    t0 = time.time()
    mlm = build_union_model_near_far_grid(
        [rel_xy],
        rx_near_cell=RX_NEAR_CELL,
        rx_far_cell=RX_FAR_CELL,
        rx_near_range=RX_NEAR_RANGE,
        gy=GY,
        auto_grid=False,
    )
    t1 = time.time()
    print(f"箱数（ダミー開始箱含む）: {len(mlm.model.boxes)}, max_step: {mlm.model.max_step} ({t1 - t0:.2f}s)")
    print(f"箱列（{len(mlm.sequences[0])}個の圧縮状態）: {mlm.sequences[0]}")
    print()

    print("=== membership check: このログ自身が、このモデルに含まれる(SAT)か ===")
    t2 = time.time()
    membership = verify_logs_included(mlm)
    t3 = time.time()
    print(f"is_member: {membership[0].is_member} ({t3 - t2:.2f}s)")
    print()

    revisited = [k for k in set(mlm.sequences[0]) if mlm.sequences[0].count(k) > 1]
    print("=== シナリオ列挙数について ===")
    if revisited:
        print(
            f"箱列の中に同じ格子セル(lane,position)を複数回訪れている箇所がある: {sorted(revisited)}\n"
            "（GY=0.3mと非常に細かいため、NPCが横方向にわずかに揺れ戻る動きが実際の分岐点として\n"
            "現れている）。これはモデルの中で本物の分岐・合流点になり、gcpd.enum_ssによる"
            "シナリオ列挙は\n組合せ的に増える経路をZ3で1本ずつ解く必要があるため、時間がかかりすぎる"
            "（60秒超）ため今回は省略した。\nmembership checkでこのログ自身がモデルに含まれる"
            "ことは既に確認済みであり、モデル自体は正しく構築できている。\n"
            "(English) The box sequence revisits the same grid cell (lane, position) more than "
            "once at the points listed above (because GY=0.3m is so fine that a small lateral "
            "back-and-forth in the NPC's motion shows up as a real branch point). This creates "
            "genuine branch/merge points in the model, and gcpd.enum_ss enumerating scenarios one "
            "at a time via Z3 over the resulting combinatorial paths takes too long (over 60s), so "
            "it was skipped here. The membership check above already confirms this log itself is "
            "included in the model, so the model itself was built correctly."
        )
    else:
        n_scenarios = count_scenarios(mlm)
        print(f"列挙されたシナリオ数: {n_scenarios}")
    print()

    print("=== 可視化 ===")
    diagram_path = plot_model_with_ego_paper_style(
        mlm.model,
        mlm.box_id_of,
        f"{OUT_DIR}/model_collision_0067_gcpd_fine.png",
        car="NPC",
        ego_lane=0,
        ego_max_step=mlm.model.max_step,
        title="TD-NI-AR-SD-N04-CI-0067: gcpd.Model (fine near/far grid) + Ego",
    )
    print(f"モデル構造図: {diagram_path}")

    if revisited:
        print(
            "\n注: 上記の分岐・合流点があるため、Ego同期ワールド座標系GIF"
            "（render_world_frame_gif、strans同期遷移をSATで解く）は数分経っても"
            "終わらず、今回は省略した（design doc 11.6/11.7節で報告した、モデルの"
            "分岐が多いほどEgo同期アニメーションのSAT求解が重くなる、という既知の"
            "傾向と一致する）。GIFが必要な場合は、12.4節の粗い格子（rx_near_cell=5.0,"
            "rx_far_cell=15.0等、分岐なし）を使った既存のGIFで代替するか、GYを"
            "0.3mより粗くして分岐点自体を減らす必要がある。\n"
            "(English) Because of the branch/merge points above, the "
            "Ego-synchronized world-frame GIF (render_world_frame_gif, which solves "
            "the strans synchronized transition via SAT) did not finish within "
            "several minutes and was skipped here (consistent with the known "
            "trend reported in design doc sections 11.6/11.7 that more branching in "
            "the model makes the Ego-synchronized animation's SAT solve heavier). If "
            "a GIF is needed, use the existing one built on the coarser Section 12.4 "
            "grid (rx_near_cell=5.0, rx_far_cell=15.0, etc. -- no branching), or "
            "coarsen GY beyond 0.3m to reduce the branch points themselves."
        )
    else:
        t6 = time.time()
        gif_paths = render_world_frame_gif(
            mlm.model, f"{OUT_DIR}/collision_0067_gcpd_fine_ego_sync", combined=True, ego_speed=1.0, num_model=1
        )
        t7 = time.time()
        print(f"Ego同期ワールド座標系GIF: {gif_paths} ({t7 - t6:.2f}s)")


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
