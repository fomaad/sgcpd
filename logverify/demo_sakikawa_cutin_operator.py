"""咲川氏のcut-in演算子を「カットイン判定そのもの」として直接使い、
それ以外の挙動（前後、カットイン後のさらなる車線変更を含む）は
完全に自由にする、という設計を6本の実AJISAIログで確認するデモ。

reference_models.build_cutin_reference_9area は、もはやカットイン判定を
行わない（ほぼ完全に自由な、9領域という語彙だけを持つ構造モデルに
変更した）。カットインが起きたかどうかは、
logverify.sakikawa_relations.detect_cutin （観測された圧縮列に対する
直接の局所パターン照合、SATを介さない）で判定する。これは
vendor/trajectory_abstraction/src/abstraction_9area.py 自身の
abstract_cutin_detected と同じ問いに答えるものなので、6本のログ全てで
両者の判定が一致することを確認する。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_sakikawa_cutin_operator \
        <log1.json> <log2.json> ...
    (引数なしの場合は demo_real_ajisai_multi_log.py と同じ6本を使う)

---
English:
A demo confirming, on 6 real AJISAI logs, the design where Mr. Sakikawa's
cut-in operator is used directly as the cut-in verdict itself, while
everything else (before/after, including further lane changes after the
cut-in) is left completely free.

reference_models.build_cutin_reference_9area no longer performs cut-in
detection at all (it has been changed into an almost fully free structural
model that carries only the 9-area vocabulary). Whether a cut-in occurred
is instead determined by logverify.sakikawa_relations.detect_cutin (a
direct local pattern check against the observed compressed sequence,
without going through SAT). This answers the same question as
vendor/trajectory_abstraction/src/abstraction_9area.py's own
abstract_cutin_detected, so this demo confirms the two verdicts agree on
all 6 logs.
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "vendor", "trajectory_abstraction", "src"))

from logverify.grid_bridge import relative_xy_from_ajisai_groundtruth
from logverify.reference_models import build_cutin_reference_9area_scenario

DEFAULT_LOG_DIR = "/mnt/user-data/uploads/Downloads"
DEFAULT_LOG_NAMES = [
    "TD-NI-AR-SD-N04-CI-0030.json",
    "TD-NI-AR-SD-N04-CI-0032.json",
    "TD-NI-AR-SD-N04-CI-0035.json",
    "TD-NI-AR-SD-N04-CI-0047.json",
    "TD-NI-AR-SD-N04-CI-0067.json",
    "TD-NI-AR-SD-N04-CI-0076.json",
]


def _vendor_abstract_cutin(json_path: str) -> bool:
    """vendor/trajectory_abstraction/src/abstraction_9area.py 自身の判定を
    そのまま呼び出す（別実装との突き合わせのため）。

    ---
    English:
    Calls vendor/trajectory_abstraction/src/abstraction_9area.py's own
    verdict directly (to cross-check against an independent
    implementation).
    """
    import abstraction_9area as vendor

    data = vendor.load_json_data(Path(json_path))
    ego_coords, npc_coords, ego_vel, npc_vel, timestamps, npc_ids = vendor.extract_coordinates_from_json(data)
    ego_n, npc_n, rot = vendor.normalize_coordinates(ego_coords, npc_coords)
    ego_vel_n, npc_vel_n = vendor.normalize_velocities(ego_vel, npc_vel, rot)
    _, detection_results = vendor.generate_output_text(ego_n, npc_n, ego_vel_n, npc_vel_n, timestamps, npc_ids)
    return bool(detection_results["abstract_cutin"])


def run(json_paths) -> None:
    scenario = build_cutin_reference_9area_scenario()
    print(f"Number of input real logs: {len(json_paths)}")
    print()

    n_agree = 0
    for p in json_paths:
        name = os.path.basename(p)
        rel_xy = relative_xy_from_ajisai_groundtruth(p)

        # 1. 構造モデル（ほぼ自由）に対するmembership: 語彙の範囲内かどうかの
        #    弱いチェックのみ。カットインの有無は保証しない。
        # (English) 1. membership against the (almost free) structural
        #    model: only a weak check of staying within the vocabulary. It
        #    guarantees nothing about whether a cut-in occurred.
        states, mem_result = scenario.check(rel_xy)

        # 2. 咲川氏のcut-in演算子を直接適用: これがカットイン判定そのもの。
        # (English) 2. Mr. Sakikawa's cut-in operator applied directly:
        #    this IS the cut-in verdict.
        _, events = scenario.detect_cutin(rel_xy)
        our_cutin = len(events) > 0

        vendor_cutin = _vendor_abstract_cutin(p)
        agree = our_cutin == vendor_cutin
        n_agree += int(agree)

        print(f"--- {name} ---")
        print(f"  raw frames: {len(rel_xy)}, compressed states (9-area): {len(states)}")
        print(f"  structural membership (9-area vocabulary, almost-free model): {mem_result}")
        print(f"  our detect_cutin: {our_cutin} ({len(events)} occurrence(s))")
        for ev in events:
            print(
                f"    (lane={ev.from_state.lane:+d}, pos={ev.from_state.position:+d}) "
                f"@frame{ev.from_state.end_frame} -> "
                f"(lane={ev.to_state.lane:+d}, pos={ev.to_state.position:+d}) "
                f"@frame{ev.to_state.start_frame}"
            )
        print(f"  vendor abstraction_9area.py's abstract_cutin: {vendor_cutin}")
        print(f"  agreement: {'OK' if agree else 'MISMATCH'}")
        print()

    print(f"=== Agreement with vendor tool: {n_agree}/{len(json_paths)} logs ===")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        paths = sys.argv[1:]
    else:
        paths = [os.path.join(DEFAULT_LOG_DIR, name) for name in DEFAULT_LOG_NAMES]
    run(paths)
