"""
Egoが前進するワールド座標系でのGIFアニメーションのデモ（方法B・方法Cの両方）。

実行方法:
    cd sgcpd && python3 -m logverify.demo_world_frame_gif
"""

import os

from logverify.world_frame_gif import render_world_frame_gif
from logverify.reference_models import build_cutin_reference
from logverify.multi_log_model import build_union_model
from logverify.demo_multi_log_model import (
    log_cutin_right_near,
    log_cutin_left_medium,
    log_stays_in_lane,
    log_cutin_right_far,
)

OUT_DIR = "out_gif"


if __name__ == "__main__":
    # 方法B: cut-in参照CPD（近距離のみに絞って見やすくする）
    model_b, _box_id_of_b = build_cutin_reference(i_range=(-1, 0, 1))
    paths_b = render_world_frame_gif(
        model_b, os.path.join(OUT_DIR, "world_cutin"), combined=True, ego_speed=1.0,
        max_step=4, num_model=6,
    )
    print("方法B（ワールド座標系）:", paths_b)

    # 方法C: 4ログ統合モデル
    logs = {
        "cutin_right_near": log_cutin_right_near(),
        "cutin_left_medium": log_cutin_left_medium(),
        "stays_in_lane": log_stays_in_lane(),
        "cutin_right_far": log_cutin_right_far(),
    }
    mlm = build_union_model(list(logs.values()))
    paths_c = render_world_frame_gif(
        mlm.model, os.path.join(OUT_DIR, "world_multilog"), combined=False, ego_speed=1.0
    )
    print("方法C（ワールド座標系、シナリオごと）:", paths_c)

    paths_c_all = render_world_frame_gif(
        mlm.model, os.path.join(OUT_DIR, "world_multilog_all"), combined=True, ego_speed=1.0
    )
    print("方法C（ワールド座標系、まとめ）:", paths_c_all)
