"""
logverify のCPDモデル（方法B・方法C）を gcpd_gif.py でGIFアニメーション化するデモ。

実行方法:
    cd sgcpd && python3 -m logverify.demo_gif
"""

import os

from logverify.gif_viz import render_scenarios_gif
from logverify.multi_log_model import build_union_model
from logverify.demo_multi_log_model import (
    log_cutin_right_near,
    log_cutin_left_medium,
    log_stays_in_lane,
    log_cutin_right_far,
)

OUT_DIR = "out_gif"


if __name__ == "__main__":
    logs = {
        "cutin_right_near": log_cutin_right_near(),
        "cutin_left_medium": log_cutin_left_medium(),
        "stays_in_lane": log_stays_in_lane(),
        "cutin_right_far": log_cutin_right_far(),
    }
    mlm = build_union_model(list(logs.values()))
    print(f"格子: gx={mlm.gx}, gy={mlm.gy}, 箱数={len(mlm.model.boxes)}")

    # シナリオごとに別々のGIFを作る（どのログがどう動くか個別に確認しやすい）
    paths = render_scenarios_gif(
        mlm.model, os.path.join(OUT_DIR, "multi_log"), combined=False
    )
    print("生成されたGIF（シナリオごと）:")
    for p in paths:
        print(" ", p, os.path.getsize(p), "bytes")

    # 全シナリオをまとめた1本のGIFも作る
    paths_all = render_scenarios_gif(
        mlm.model, os.path.join(OUT_DIR, "multi_log_all"), combined=True
    )
    print("生成されたGIF（全シナリオまとめ）:")
    for p in paths_all:
        print(" ", p, os.path.getsize(p), "bytes")
