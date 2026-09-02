"""咲川氏の9領域モデル（vendor/trajectory_abstraction/src/abstraction_9area.py）の
領域境界を、そのままCPDの (lane, position) として使うための橋渡し。

logverify.zones の BEHIND/NEAR/MEDIUM/FAR は、near_max/medium_maxという
「この分析のために都合よく選んだ距離」を境界にしていた。これに対し
咲川氏の9領域モデルは、lane（同一車線か否か）の境界をego車両自身の
車幅の半分、position（前方/重なり/後方）の境界をego車両自身の車長の
半分としており、距離の大きさをどう決めるかという恣意性が入らない
（境界がego車両という、外部から与えられる物理的な実体で決まる）。

さらに重要なのは、咲川氏の「抽象空間でのカットイン判定」ルール
（lead-right/lead-left/right/left のいずれか → lead0、という遷移だけを
カットインとみなす）が、健全（sound）だという点である。これらの4領域は
定義上すべて「other-lane」であり、lead0は定義上「same-lane かつ
前方」なので、この遷移が起きれば、具体的にも「隣接レーンから
ego車線へ移り、かつego車両より前に出た」ことが論理的に保証される
（逆に、後方のまま合流したケースは、この4領域からの遷移としては
表現されないため、SATにならない）。

このモジュールは、この9領域を lane×position の3×3グリッドとして
そのままCPDの箱に対応づける:
    lane:      -1 (右、ego右半分の外) / 0 (同幅帯) / +1 (左、ego左半分の外)
    position:  -1 (follow、ego前半分より後方) / 0 (overlap、egoと縦方向に重なる)
               / +1 (lead、ego前半分より前方)

---
English:
Bridges Mr. Sakikawa's 9-area model's (vendor/trajectory_abstraction/src/
abstraction_9area.py) region boundaries directly into a CPD's (lane,
position).

logverify.zones's BEHIND/NEAR/MEDIUM/FAR used near_max/medium_max --
distances chosen for the convenience of this particular analysis -- as
boundaries. Mr. Sakikawa's 9-area model, in contrast, sets the lane
(same-lane or not) boundary at half the ego vehicle's own width, and the
position (ahead/overlap/behind) boundary at half the ego vehicle's own
length, so no arbitrary choice of "how large a distance" is involved (the
boundary is set by the ego vehicle itself, an externally given physical
entity).

More importantly, Mr. Sakikawa's "abstract-space cut-in" rule (only a
transition from lead-right/lead-left/right/left into lead0 counts as
cut-in) is sound: these four regions are, by definition, always
"other-lane", and lead0 is, by definition, always "same-lane and ahead",
so if this transition occurs, it is logically guaranteed that the vehicle
concretely moved from the adjacent lane into the ego lane AND ended up
ahead of ego (conversely, a case that merges while remaining behind is
not expressible as a transition from these four regions, so it does not
become SAT).

This module maps that 9-area partition directly onto a CPD's boxes as a
3x3 (lane, position) grid:
    lane:      -1 (right, outside ego's right half-width) / 0 (same width
               band as ego) / +1 (left, outside ego's left half-width)
    position:  -1 (follow, behind ego's front half-length) / 0 (overlap,
               longitudinally overlapping ego) / +1 (lead, ahead of ego's
               front half-length)
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# ego車両自身の物理サイズ。abstraction_9area.py と同じ値
# (vendor/trajectory_abstraction/src/abstraction_9area.py の VEHICLE_WIDTH/
# VEHICLE_LENGTH)。scenario分析のための距離しきい値ではなく、車両サイズという
# 物理的な定数であることに注意。
# (English) The ego vehicle's own physical size, matching
# abstraction_9area.py's VEHICLE_WIDTH/VEHICLE_LENGTH. Note this is a
# physical constant (vehicle size), not a scenario-analysis distance
# threshold.
EGO_HALF_WIDTH = 1.9 / 2.0
EGO_HALF_LENGTH = 5.3 / 2.0

FOLLOW = -1
OVERLAP = 0
LEAD = 1

POSITION_LABELS = {FOLLOW: "後方(follow)", OVERLAP: "重なり(overlap)", LEAD: "前方(lead)"}
LANE_LABELS = {-1: "右(right)", 0: "同幅帯(same width band)", 1: "左(left)"}


def classify_lane(ry: float, ego_half_width: float = EGO_HALF_WIDTH) -> int:
    """ry（左方向が正）を、ego車両の車幅だけで前後左右関係に丸める。

    ---
    English:
    Rounds ry (positive is left) into a lane relation, using only the ego
    vehicle's own width.
    """
    if ry >= ego_half_width:
        return 1
    if ry <= -ego_half_width:
        return -1
    return 0


def classify_position(rx: float, ego_half_length: float = EGO_HALF_LENGTH) -> int:
    """rx（前方が正）を、ego車両の車長だけで前後関係に丸める。

    ---
    English:
    Rounds rx (positive is ahead) into a longitudinal relation, using only
    the ego vehicle's own length.
    """
    if rx >= ego_half_length:
        return LEAD
    if rx <= -ego_half_length:
        return FOLLOW
    return OVERLAP


@dataclass
class RelationState:
    index: int
    lane: int       # -1/0/+1 (right/same width band/left)
    position: int   # -1/0/+1 (follow/overlap/lead)
    start_frame: int
    end_frame: int


def compress_to_relation_states(
    rxs: Sequence[Optional[float]],
    rys: Sequence[Optional[float]],
    ego_half_width: float = EGO_HALF_WIDTH,
    ego_half_length: float = EGO_HALF_LENGTH,
) -> List[RelationState]:
    """(rx, ry) の時系列を、ego車両自身のサイズだけで (lane, position) に
    丸め、イベント駆動で圧縮する。near_max/medium_max/gyのような、分析の
    ために選ぶ距離しきい値は一切使わない。

    ---
    English:
    Rounds a time series of (rx, ry) into (lane, position) using only the
    ego vehicle's own size, compressing it in an event-driven manner. No
    analysis-chosen distance threshold (like near_max/medium_max/gy) is
    used anywhere.
    """
    states: List[RelationState] = []
    prev_key: Optional[Tuple[int, int]] = None

    for frame, (rx, ry) in enumerate(zip(rxs, rys)):
        if rx is None or ry is None:
            continue
        lane_val = classify_lane(ry, ego_half_width)
        position_val = classify_position(rx, ego_half_length)
        key = (lane_val, position_val)
        if prev_key is not None and key == prev_key:
            states[-1].end_frame = frame
            continue
        states.append(
            RelationState(index=len(states), lane=lane_val, position=position_val, start_frame=frame, end_frame=frame)
        )
        prev_key = key

    return states


def relation_states_from_relative_xy(
    rel_xy: Sequence[Tuple[float, float]],
    ego_half_width: float = EGO_HALF_WIDTH,
    ego_half_length: float = EGO_HALF_LENGTH,
) -> List[RelationState]:
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    return compress_to_relation_states(rxs, rys, ego_half_width, ego_half_length)


@dataclass
class CutinEvent:
    """detect_cutin が見つけた、カットインとみなされる隣接状態遷移1件。

    ---
    English:
    One adjacent-state transition found by detect_cutin that counts as a
    cut-in.
    """

    from_state: RelationState
    to_state: RelationState


def detect_cutin(
    states: Sequence[RelationState],
    side_lanes: Sequence[int] = (-1, 1),
    ego_lane: int = 0,
) -> List[CutinEvent]:
    """咲川氏の抽象空間カットイン判定ルールを、観測された圧縮状態列に対して
    直接（SATを介さずに）照合する。

    これは vendor/trajectory_abstraction/src/abstraction_9area.py の
    abstract_cutin_detected と同じ問い、すなわち「隣接レーンにいて
    かつ後方ではない(position != FOLLOW)状態から、egoレーンの
    前方(LEAD)状態へ、隣接するステップ間で直接遷移した箇所が
    列のどこかに存在するか」を、局所的・存在的（その前後に何が
    起ころうと関係ない）に判定する。

    これは reference_models.build_cutin_reference_9area によるSAT/UNSAT
    判定とは別物である。SAT/UNSAT はモデル全体の構造（箱と遷移の語彙）に
    対する大域的な適合性を判定するのに対し、こちらはその語彙で圧縮した
    列の中に、特定の隣接ペアパターンが1箇所でも現れるかどうかだけを見る。
    「カットインの前後の挙動は自由にしたい」（合流後にさらに車線変更する
    等）という要求は、この2つを分離することで初めて両立できる:
    参照モデル側はほぼ自由な遷移グラフ（一般的な語彙）にしてよく、
    「カットインが起きたか」はこの関数で直接判定する。

    Returns: 列の中で見つかった全てのカットイン遷移（複数回のカットインも
    そのまま複数件返る）。1件もなければ空リスト。

    ---
    English:
    Checks Mr. Sakikawa's abstract-space cut-in rule directly (without
    going through SAT) against an observed, compressed state sequence.

    This asks the same question as
    vendor/trajectory_abstraction/src/abstraction_9area.py's
    abstract_cutin_detected: does there exist, anywhere in the sequence, an
    adjacent-step transition directly from a side-lane, not-behind
    (position != FOLLOW) state into an ego-lane, LEAD state? This is a
    local, existential check -- it does not care what happens before or
    after that transition.

    This is a different thing from the SAT/UNSAT verdict produced against
    reference_models.build_cutin_reference_9area. SAT/UNSAT judges global
    conformance of the *entire* sequence to the model's structure (its
    vocabulary of boxes and transitions), whereas this function only asks
    whether one specific adjacent-pair pattern occurs anywhere in the
    sequence compressed into that vocabulary. Separating the two is what
    makes it possible to satisfy "the behavior before and after the cut-in
    should be free" (e.g. a further lane change after merging): the
    reference model itself can be an almost fully free transition graph
    (a generic vocabulary), while "did a cut-in occur" is answered
    directly by this function.

    Returns: every cut-in transition found in the sequence (if a cut-in
    pattern occurs more than once, all occurrences are returned). Empty
    list if none is found.
    """
    events: List[CutinEvent] = []
    for prev, cur in zip(states, states[1:]):
        if (
            prev.lane in side_lanes
            and prev.position != FOLLOW
            and cur.lane == ego_lane
            and cur.position == LEAD
        ):
            events.append(CutinEvent(from_state=prev, to_state=cur))
    return events


def classify_fine(value: float, unit: float, max_bins: int = 12) -> int:
    """value（rxまたはry）を、ego車両自身のサイズの一部（unit = 半分の
    さらに1/n_bins）を単位とした整数の目盛りに丸める。

    値の大きさを決める「単位」がego車両自身の物理サイズの一部
    （例えばEGO_HALF_LENGTH/4）であることに変わりはない。すなわち、
    9領域（n_bins=1相当）のように粗い場合はEGO周辺で何が起きているか
    分からない、という問題に対し、この関数は「同じ物理的な単位を、
    より細かく刻む」ことで対応する。分析用に選んだ恣意的なメートル値
    （例えば0.5m単位）を使うわけではない。

    |value| が unit*max_multiple を超えた場合は、それ以上細分せず
    ±(max_multiple+1) に飽和させる（遠方は従来通り粗いまま）。

    ---
    English:
    Rounds value (rx or ry) onto an integer scale whose unit is a fraction
    of the ego vehicle's own size (unit = EGO_HALF_LENGTH/n_bins or
    EGO_HALF_WIDTH/n_bins, say).

    The unit that determines the granularity is still a fraction of the
    ego vehicle's own physical size -- unlike the 9-area partition
    (equivalent to n_bins=1), which is too coarse to see what is
    happening right around ego, this function addresses that by cutting
    the same physical unit more finely rather than introducing an
    arbitrary analysis-chosen metric threshold (such as "0.5m bins").

    If |value| exceeds unit*max_bins, it saturates at ±(max_bins+1)
    instead of subdividing further (far away stays coarse, as before).
    max_bins should be given in units of the *fine* bin (i.e. already
    multiplied by n_bins if you want the saturation range expressed as a
    multiple of the full ego_half_length/ego_half_width) --
    compress_to_fine_relation_states below does this multiplication for
    you.
    """
    if unit <= 0:
        raise ValueError("unit must be positive")
    n = value / unit
    if n >= 0:
        idx = int(n) if n < max_bins else max_bins + 1
    else:
        idx = -int(-n) if -n < max_bins else -(max_bins + 1)
    return idx


@dataclass
class FineRelationState:
    index: int
    lane_fine: int      # ego_half_widthのn_bins分の1を単位とした目盛り
    position_fine: int  # ego_half_lengthのn_bins分の1を単位とした目盛り
    start_frame: int
    end_frame: int


def compress_to_fine_relation_states(
    rxs: Sequence[Optional[float]],
    rys: Sequence[Optional[float]],
    ego_half_width: float = EGO_HALF_WIDTH,
    ego_half_length: float = EGO_HALF_LENGTH,
    n_bins: int = 4,
    max_range: int = 3,
) -> List[FineRelationState]:
    """9領域（FOLLOW/OVERLAP/LEAD、右/同幅帯/左）ではEGOのすぐ近くで
    何が起きているか区別がつかない場合に使う、より細かい版。
    単位はego車両自身のサイズの1/n_binsのまま（恣意的な距離を導入しない）。

    max_range: 分割せず飽和させる範囲を、ego_half_width/ego_half_length
    「何個分」で指定する（9領域のFOLLOW/OVERLAP/LEADや右/同幅帯/左の
    範囲がego_half_width/ego_half_length「1個分」に相当するのに対応する）。

    ---
    English:
    A finer-grained version to use when the 9-area partition (FOLLOW/
    OVERLAP/LEAD, right/same-width-band/left) cannot distinguish what is
    happening right around ego. The unit stays a fraction (1/n_bins) of
    the ego vehicle's own size (no arbitrary distance is introduced).

    max_range: how far out (in units of ego_half_width/ego_half_length)
    to keep subdividing before saturating (matching how the 9-area
    partition's FOLLOW/OVERLAP/LEAD and right/same-width-band/left each
    span exactly "1 unit" of ego_half_length/ego_half_width).
    """
    lane_unit = ego_half_width / n_bins
    pos_unit = ego_half_length / n_bins
    max_bins = n_bins * max_range

    states: List[FineRelationState] = []
    prev_key: Optional[Tuple[int, int]] = None

    for frame, (rx, ry) in enumerate(zip(rxs, rys)):
        if rx is None or ry is None:
            continue
        lane_val = classify_fine(ry, lane_unit, max_bins)
        position_val = classify_fine(rx, pos_unit, max_bins)
        key = (lane_val, position_val)
        if prev_key is not None and key == prev_key:
            states[-1].end_frame = frame
            continue
        states.append(
            FineRelationState(index=len(states), lane_fine=lane_val, position_fine=position_val, start_frame=frame, end_frame=frame)
        )
        prev_key = key

    return states


def fine_relation_states_from_relative_xy(
    rel_xy: Sequence[Tuple[float, float]],
    ego_half_width: float = EGO_HALF_WIDTH,
    ego_half_length: float = EGO_HALF_LENGTH,
    n_bins: int = 4,
    max_range: int = 3,
) -> List[FineRelationState]:
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    return compress_to_fine_relation_states(rxs, rys, ego_half_width, ego_half_length, n_bins, max_range)


def detect_cutin_from_relative_xy(
    rel_xy: Sequence[Tuple[float, float]],
    side_lanes: Sequence[int] = (-1, 1),
    ego_lane: int = 0,
    ego_half_width: float = EGO_HALF_WIDTH,
    ego_half_length: float = EGO_HALF_LENGTH,
) -> List[CutinEvent]:
    """relation_states_from_relative_xy + detect_cutin をまとめたショートカット。

    ---
    English:
    A shortcut combining relation_states_from_relative_xy and detect_cutin.
    """
    states = relation_states_from_relative_xy(rel_xy, ego_half_width, ego_half_length)
    return detect_cutin(states, side_lanes=side_lanes, ego_lane=ego_lane)
