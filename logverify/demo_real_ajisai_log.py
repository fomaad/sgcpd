"""Method B and Method C exercised against a real AJISAI log (not synthetic).

Every other demo in this package (demo_multi_log_model.py,
demo_multi_log_model_large.py, demo_cutin_membership.py, ...) uses
hand-crafted synthetic trajectories, because a real AJISAI log was not
available in this environment (see docs/log_to_cpd_verification_design.md
section 11.2, item 1, "real-data validation with ~10 logs"). This demo is
the first to use an actual AJISAI log file: a single-NPC cut-in scenario
recorded from an Autoware simulation run.

方法BとCを、（合成ではなく）実際のAJISAIログに対して試すデモ。

本パッケージの他のデモは全て、この環境に実際のAJISAIログがなかったため
（docs/log_to_cpd_verification_design.md 11.2節・課題1「実データ10本での
検証」参照）、手作りした合成トラジェクトリを使っている。本デモは、実際に
1本のAJISAIログファイル（Autowareシミュレーションで記録された、
NPC1台によるcut-inシナリオ）を使う、最初のデモである。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_real_ajisai_log <path-to-ajisai-log.json>
"""

import sys
import time

from logverify.grid_bridge import (
    relative_xy_from_ajisai_groundtruth,
    grid_states_from_relative_xy,
)
from logverify.zones import zone_states_from_relative_xy, ZoneThresholds
from logverify.reference_models import build_cutin_reference
from logverify.membership import check_membership_cutin
from logverify.report import summarize_trace, describe_scenario
from logverify.multi_log_model import build_union_model, verify_logs_included, count_scenarios

GY = 3.5
THRESHOLDS = ZoneThresholds(near_max=5.0, medium_max=20.0)


def run(json_path: str, npc_name=None) -> None:
    print(f"Loading real AJISAI log: {json_path}")
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path, npc_name=npc_name)
    print(f"Number of raw (rx, ry) frames extracted: {len(rel_xy)}")
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    print(f"rx range: {min(rxs):.2f} .. {max(rxs):.2f} m   ry range: {min(rys):.2f} .. {max(rys):.2f} m")
    print()

    # --- Method B: membership check against the cut-in reference CPD ---
    print("=== Method B: membership check against the cut-in reference CPD ===")
    states = zone_states_from_relative_xy(rel_xy, GY, THRESHOLDS)
    observed = [(s.lane, s.zone) for s in states]
    durations = [s.end_frame - s.start_frame + 1 for s in states]
    print(f"Compressed state sequence (lane, position=zone), with frame durations: {list(zip(observed, durations))}")

    model, box_id_of = build_cutin_reference(i_range=(-1, 0, 1, 2), side_lanes=(-1, 1), ego_lane=0)
    result = check_membership_cutin(model, observed)
    print(result)
    if result.is_member:
        steps = summarize_trace(observed, durations=durations, parallel_min_frames=6)
        print(describe_scenario(steps))
    print()

    # --- Method C: union model from this one real log ---
    print("=== Method C: union model built from this one real log ===")
    mlm = build_union_model([rel_xy])
    print(f"Auto-selected grid: gx={mlm.gx}, gy={mlm.gy}")
    print(f"Number of boxes: {len(mlm.model.boxes)} (including the dummy start box), max_step: {mlm.model.max_step}")
    box_seq = [(s.k, s.i) for s in grid_states_from_relative_xy(rel_xy, mlm.gx, mlm.gy)]
    print(f"Box sequence ({len(box_seq)} compressed states): {box_seq}")

    t0 = time.time()
    membership = verify_logs_included(mlm)
    t1 = time.time()
    print(f"Membership check (log included in its own union model): {membership[0].is_member} ({t1 - t0:.2f}s)")

    t2 = time.time()
    n_scenarios = count_scenarios(mlm)
    t3 = time.time()
    print(f"Number of scenarios enumerated from the model: {n_scenarios} (input logs: 1, {t3 - t2:.2f}s)")
    if n_scenarios > 1:
        print(
            "More than 1 scenario was enumerated from a model built from a SINGLE real log. "
            "This happens because the raw (rx, ry) trajectory is not perfectly monotonic (small "
            "back-and-forth oscillation in rx even while merging), so the same coarse grid cell "
            "gets revisited at different points in time -- creating a box with more than one "
            "outgoing transition, which the model then treats as a genuine branch point. This is "
            "an example of the 'unintended generalization from a grid that is too coarse relative "
            "to the data's noise' issue discussed in design doc section 11.2, item 2 -- and it "
            "shows up even with a single log, once real (non-synthetic) data is used."
        )
    print()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 -m logverify.demo_real_ajisai_log <path-to-ajisai-log.json> [npc_name]")
        sys.exit(1)
    json_path = sys.argv[1]
    npc_name = sys.argv[2] if len(sys.argv) > 2 else None
    run(json_path, npc_name=npc_name)
