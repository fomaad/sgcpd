"""1本のログを、参照モデル自身の抽象度に自動的に合わせて抽象化するデモ。

これまで、参照CPD側の語彙（reference_models.build_cutin_referenceの
i_range=BEHIND/NEAR/MEDIUM/FAR、side_lanes、ego_lane）と、生ログを
その語彙に丸めるしきい値（logverify.zones.ZoneThresholdsのnear_max/
medium_max、横方向の格子幅gy）は、別々の場所で人間が値を合わせて
指定していた（demo_real_ajisai_log.py参照）。本デモは、この2つを
reference_models.CutinReferenceScenario という1つの単位にまとめ、
「生ログを渡すだけで、参照モデルの抽象度に自動的に合わせて抽象化し、
そのままmembership checkまで行う」ことを確認する。

ログは1本（デフォルトはTD-NI-AR-SD-N04-CI-0035.json）だけを扱う。
複数ログの統合（方法C）は本デモの対象外。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_reference_scenario_abstraction \
        [path-to-ajisai-log.json]

---
English:
A demo that automatically abstracts a single log to match a reference
model's own level of abstraction.

Previously, the reference CPD's vocabulary
(reference_models.build_cutin_reference's i_range=BEHIND/NEAR/MEDIUM/FAR,
side_lanes, ego_lane) and the thresholds used to round a raw log into that
vocabulary (logverify.zones.ZoneThresholds's near_max/medium_max, and the
lateral grid width gy) were specified separately, with a human keeping the
two in sync by hand (see demo_real_ajisai_log.py). This demo bundles the
two into a single unit, reference_models.CutinReferenceScenario, and
confirms that "just pass in a raw log" is enough to automatically abstract
it to the reference model's own level of abstraction and run a membership
check against it.

Only a single log is handled here (default:
TD-NI-AR-SD-N04-CI-0035.json). Combining multiple logs (Method C) is out
of scope for this demo.
"""

import sys

from logverify.grid_bridge import relative_xy_from_ajisai_groundtruth
from logverify.reference_models import build_cutin_reference_scenario
from logverify.report import summarize_trace, describe_scenario
from logverify.zones import ZONE_LABELS

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0035.json"


def run(json_path: str) -> None:
    print(f"Loading raw log: {json_path}")
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path)
    print(f"Raw continuous trajectory: {len(rel_xy)} frames (not yet abstracted)")
    print()

    # 参照モデルと、その抽象化ルール（近距離5.0m/中距離20.0m境界、
    # 横方向格子幅3.5m）を1つの単位として構築する。この数値は
    # ここ以外のどこにも重複して書かれていない。
    # (English) Build the reference model together with its abstraction
    # rule (near/medium boundary at 5.0m/20.0m, lateral grid width 3.5m)
    # as a single unit. These numbers are not duplicated anywhere else.
    ref = build_cutin_reference_scenario(near_max=5.0, medium_max=20.0, gy=3.5)
    print(
        f"Reference model's own vocabulary: zones={sorted(ZONE_LABELS)} "
        f"(={[ZONE_LABELS[z] for z in sorted(ZONE_LABELS)]}), "
        f"side_lanes={ref.side_lanes}, ego_lane={ref.ego_lane}"
    )
    print()

    print("=== Automatically abstracting the raw log to the reference model's vocabulary ===")
    states, result = ref.check(rel_xy)
    for s in states:
        n_frames = s.end_frame - s.start_frame + 1
        print(
            f"  (lane={s.lane:+d}, zone={s.zone}={ZONE_LABELS[s.zone]})"
            f"  frames {s.start_frame}-{s.end_frame}  ({n_frames} frames)"
        )
    print(f"-> compressed from {len(rel_xy)} raw frames to {len(states)} states, all within the reference model's own vocabulary")
    print()

    print("=== Membership check against the reference model (using the same abstraction) ===")
    print(result)
    if result.is_member:
        observed = [(s.lane, s.zone) for s in states]
        durations = [s.end_frame - s.start_frame + 1 for s in states]
        steps = summarize_trace(observed, durations=durations, parallel_min_frames=6)
        print(describe_scenario(steps))


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
