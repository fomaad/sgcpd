"""
参照CPD（cut-in）に対して SAT と判定されたログが、そのモデルの
「どの挙動」に対応しているかを人間にわかる形で要約するユーティリティ。

reference_models.build_cutin_reference で作った参照CPDでは、
(lane, position) と箱が1対1に対応しているため（box_id_of が全単射）、
観測列 [(lane_0, position_0), ...] を与えた時点で、SATならばそれは
そのまま「参照モデル中でどの箱を辿ったか」の witness になっている。
つまり "この観測列がモデルの言語に含まれるか" (SAT/UNSAT) と
"含まれるとしてどの経路を通ったか" は、このモデルに限っては
同じ情報から得られる。

position はすでに logverify.zones によって
BEHIND(-1) / NEAR(0) / MEDIUM(1) / FAR(2) に丸められた順序値なので、
ここでは単にそのラベルを引いて、
  - 出発した距離帯
  - 並走とみなせる区間の有無（同じレーン・同じ距離帯に複数状態留まる）
  - 合流が起きたタイミング・距離帯
を日本語のサマリ文字列にする。

---
English:
A utility that summarizes, in human-readable form, which "behavior" of a
reference CPD (cut-in) an observed log corresponds to, once the log has been
judged SAT against that model.

In a reference CPD built with reference_models.build_cutin_reference,
(lane, position) corresponds one-to-one with a box (box_id_of is a bijection),
so once an observation sequence [(lane_0, position_0), ...] is given, a SAT
verdict is itself a witness of "which boxes were traversed in the reference
model". In other words, for this model specifically, "whether this
observation sequence is in the model's language" (SAT/UNSAT) and "which path
it took, given that it is" come from the same information.

Since position has already been rounded by logverify.zones into the ordinal
values BEHIND(-1) / NEAR(0) / MEDIUM(1) / FAR(2), here we simply look up that
label and turn the following into a Japanese summary string:
  - the distance band it started from
  - whether there was a stretch that can be regarded as driving in parallel
    (staying in multiple states in the same lane and same distance band)
  - the timing and distance band at which merging occurred
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from logverify.zones import ZONE_LABELS


def zone_label(position: int) -> str:
    return ZONE_LABELS.get(position, f"position={position}")


@dataclass
class ScenarioStep:
    step: int
    lane: int
    position: int
    zone: str
    is_side_lane: bool
    event: Optional[str] = None  # "merge" | "parallel" | None


def summarize_trace(
    observed_lane_position: Sequence[Tuple[int, int]],
    ego_lane: int = 0,
    durations: Optional[Sequence[int]] = None,
    parallel_min_frames: int = 3,
) -> List[ScenarioStep]:
    """観測列 [(lane, position), ...] を ScenarioStep の列に変換する。

    - is_side_lane: lane != ego_lane かどうか
    - event="merge": この状態で初めて ego_lane に入った（合流の瞬間）
    - event="parallel": 隣接レーンに留まっている状態の継続時間（フレーム数）が
      parallel_min_frames 以上ある場合に付与する（＝速度を合わせて並走した
      とみられる区間）。

    position は logverify.zones で近距離/中距離/遠距離に丸められているため、
    「並走」は状態が何個続いたかではなく、1状態がどれだけ長く継続したか
    （durations、logverify.zones.ZoneState の end_frame - start_frame + 1）
    で判定する。durations を渡さない場合はこの判定をスキップする。

    ---
    English:
    Converts an observation sequence [(lane, position), ...] into a list of
    ScenarioStep.

    - is_side_lane: whether lane != ego_lane.
    - event="merge": the state where ego_lane was entered for the first time
      (the moment of merging).
    - event="parallel": assigned when the duration (in frames) of staying in
      an adjacent lane is at least parallel_min_frames (i.e. a stretch that
      appears to be driving in parallel at matched speed).

    Since position is rounded by logverify.zones into near/medium/far, a
    "parallel" stretch is judged not by how many states in a row occurred,
    but by how long a single state persisted (durations, i.e.
    logverify.zones.ZoneState's end_frame - start_frame + 1). If durations is
    not passed, this judgment is skipped.
    """
    steps: List[ScenarioStep] = []
    merged = False
    for idx, (lane_val, position_val) in enumerate(observed_lane_position):
        is_side = lane_val != ego_lane
        event = None
        if not is_side and not merged:
            event = "merge"
            merged = True
        elif is_side and durations is not None and idx < len(durations) and durations[idx] >= parallel_min_frames:
            event = "parallel"
        steps.append(
            ScenarioStep(
                step=idx,
                lane=lane_val,
                position=position_val,
                zone=zone_label(position_val),
                is_side_lane=is_side,
                event=event,
            )
        )

    return steps


def describe_scenario(steps: Sequence[ScenarioStep]) -> str:
    if not steps:
        return "(観測列が空です)"

    lines = []
    side = "左" if steps[0].lane > 0 else "右"
    lines.append(f"出発: {side}隣接レーン、{steps[0].zone}（position={steps[0].position}）")

    merge_step = next((s for s in steps if s.event == "merge"), None)
    parallel_steps = [s for s in steps if s.event == "parallel"]
    if parallel_steps:
        zones = sorted({s.zone for s in parallel_steps})
        lines.append(f"並走: {'/'.join(zones)}で速度を合わせて並走したとみられる区間あり")
    if merge_step is not None:
        lines.append(f"合流: ステップ{merge_step.step}、{merge_step.zone}（position={merge_step.position}）でego車線へ合流")
    else:
        lines.append("合流: 観測列の中では合流していない（隣接レーンに留まったまま）")

    lines.append(f"終了: {steps[-1].zone}（position={steps[-1].position}）、lane={steps[-1].lane}")
    return "\n".join(lines)
