"""
Egoを実際にCPDの car として組み込んだモデル構造図（11.5節のwith_ego_track）を
描画するデモ。

実行方法:
    cd sgcpd && python3 -m logverify.demo_model_diagram_with_ego

---
English:
Demo that draws a model structure diagram with Ego actually incorporated as a
CPD car (with_ego_track from Section 11.5).

How to run:
    cd sgcpd && python3 -m logverify.demo_model_diagram_with_ego
"""

import os

from logverify.reference_models import build_cutin_reference
from logverify.multi_log_model import build_union_model
from logverify.demo_multi_log_model import (
    log_cutin_right_near,
    log_cutin_left_medium,
    log_stays_in_lane,
    log_cutin_right_far,
)
from logverify.model_diagram import plot_model_with_ego_paper_style

OUT_DIR = "out_gif"


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)

    # 方法B: cut-in参照CPD + Ego
    # (English) Method B: cut-in reference CPD + Ego
    model_b, box_id_of_b = build_cutin_reference(i_range=(-1, 0, 1))
    path_b = plot_model_with_ego_paper_style(
        model_b, box_id_of_b, os.path.join(OUT_DIR, "model_cutin_with_ego.png"),
        car="NPC", ego_lane=0, ego_max_step=4,
        title="Cut-in reference CPD + Ego (synchronized) - Method B",
    )
    print("Method B:", path_b)

    # 方法C: 4ログ統合モデル + Ego
    # (English) Method C: unified model from 4 logs + Ego
    logs = {
        "cutin_right_near": log_cutin_right_near(),
        "cutin_left_medium": log_cutin_left_medium(),
        "stays_in_lane": log_stays_in_lane(),
        "cutin_right_far": log_cutin_right_far(),
    }
    mlm = build_union_model(list(logs.values()))
    path_c = plot_model_with_ego_paper_style(
        mlm.model, mlm.box_id_of, os.path.join(OUT_DIR, "model_multilog_with_ego.png"),
        car="NPC", ego_lane=0, ego_max_step=mlm.model.max_step,
        title="Unified CPD from 4 logs + Ego (synchronized) - Method C",
    )
    print("Method C:", path_c)
