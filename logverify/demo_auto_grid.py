"""12.7〜12.9節で手作業で選んだ近傍/遠方格子のパラメータ
（RX_NEAR_CELL=1.0, RX_FAR_CELL=50.0, RX_NEAR_RANGE=15.0, GY=0.3）を、
`logverify.auto_grid`で車両サイズから自動的に導出した値と比較し、
その自動導出パラメータ＋ヒステリシス（ノイズ除去、12.10節）を使って
log 0067のgcpd.Modelを構築・検証するデモ。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_auto_grid \
        [path-to-TD-NI-AR-SD-N04-CI-0067.json]
"""

import sys
import time

from logverify.auto_grid import auto_grid_params_from_ajisai
from logverify.grid_bridge import (
    relative_xy_from_ajisai_groundtruth,
    compress_to_grid_states_variable,
    compress_to_grid_states_variable_hysteresis,
)
from logverify.multi_log_model import build_single_log_model_hysteresis, verify_logs_included

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"

# 12.7〜12.9節で手で選んだ値（比較対象）
MANUAL = dict(rx_near_cell=1.0, rx_far_cell=50.0, rx_near_range=15.0, gy=0.3)


def run(json_path: str) -> None:
    print(f"Loading: {json_path}")
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path)
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]

    print("=== 車両サイズから格子パラメータを自動導出 ===")
    auto = auto_grid_params_from_ajisai(json_path)
    print(f"自動導出: gy={auto.gy:.3f}, rx_near_cell={auto.rx_near_cell:.3f}, "
          f"rx_near_range={auto.rx_near_range:.3f}, rx_far_cell={auto.rx_far_cell:.3f}")
    print(f"12.7〜12.9節で手で選んだ値: {MANUAL}")
    print(
        "(手で選んだ値と自動導出値が近い範囲に収まっていることは、車両サイズという"
        "物理的な基準からの自動導出が、その場しのぎではなく合理的であることの傍証)"
    )
    print()

    def revisits(seq):
        seen = set()
        dup = set()
        for x in seq:
            if x in seen:
                dup.add(x)
            seen.add(x)
        return dup

    print("=== 格子ごとの箱数・重複（分岐点）の比較 ===")
    for label, params, hysteresis in [
        ("手動格子・ヒステリシスなし", MANUAL, False),
        ("手動格子・ヒステリシスあり", MANUAL, True),
        ("自動格子・ヒステリシスなし", vars(auto), False),
        ("自動格子・ヒステリシスあり", vars(auto), True),
    ]:
        if hysteresis:
            states = compress_to_grid_states_variable_hysteresis(
                rxs, rys, params["rx_near_cell"], params["rx_far_cell"], params["rx_near_range"], params["gy"],
                margin_ratio=0.3,
            )
        else:
            states = compress_to_grid_states_variable(
                rxs, rys, params["rx_near_cell"], params["rx_far_cell"], params["rx_near_range"], params["gy"]
            )
        seq = [(s.k, s.i) for s in states]
        dup = revisits(seq)
        print(f"{label}: 箱数={len(seq)}, 分岐点(重複)={sorted(dup) if dup else 'なし'}")
    print()

    print("=== 自動格子＋ヒステリシスで、log 0067専用のgcpd.Modelを構築 ===")
    t0 = time.time()
    mlm = build_single_log_model_hysteresis(
        rel_xy,
        rx_near_cell=auto.rx_near_cell,
        rx_far_cell=auto.rx_far_cell,
        rx_near_range=auto.rx_near_range,
        gy=auto.gy,
        margin_ratio=0.3,
    )
    t1 = time.time()
    print(f"箱数（ダミー開始箱含む）: {len(mlm.model.boxes)}, max_step: {mlm.model.max_step} ({t1 - t0:.2f}s)")

    t2 = time.time()
    membership = verify_logs_included(mlm)
    t3 = time.time()
    print(f"membership check (is_member): {membership[0].is_member} ({t3 - t2:.2f}s)")


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
