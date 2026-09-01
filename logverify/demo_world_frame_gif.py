"""
Egoが前進するワールド座標系でのGIFアニメーションのデモ（方法B・方法Cの両方）。

実行方法:
    cd sgcpd && python3 -m logverify.demo_world_frame_gif

---
English:
Demo of GIF animation in the world coordinate frame where Ego moves forward
(both Method B and Method C).

How to run:
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
    # max_position_jump=1: 距離帯を1段ずつしか移動できないようにする。
    # これにより、車線変更と距離帯の大ジャンプが同じ1遷移で同時に起きる
    # ことがなくなり、アニメーションの直線補間がEgoを突っ切って見える
    # アーティファクトを抑えられる（詳細はdocs参照）。
    # (English) Method B: cut-in reference CPD (restricted to short range only,
    # for readability).
    # max_position_jump=1: restrict movement between distance zones to one
    # step at a time. This prevents a lane change and a large distance-zone
    # jump from happening together in a single transition, which avoids the
    # animation's linear interpolation artifact of appearing to cut through
    # Ego (see docs for details).
    model_b, _box_id_of_b = build_cutin_reference(i_range=(-1, 0, 1), max_position_jump=1)
    paths_b = render_world_frame_gif(
        model_b, os.path.join(OUT_DIR, "world_cutin"), combined=True, ego_speed=1.0,
        max_step=4, num_model=6, zone_ahead_offset=1,
    )
    print("Method B (world coordinate frame):", paths_b)

    # 方法C: 4ログ統合モデル
    # (English) Method C: unified model from 4 logs
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
    print("Method C (world coordinate frame, per scenario):", paths_c)

    paths_c_all = render_world_frame_gif(
        mlm.model, os.path.join(OUT_DIR, "world_multilog_all"), combined=True, ego_speed=1.0
    )
    print("Method C (world coordinate frame, combined):", paths_c_all)
