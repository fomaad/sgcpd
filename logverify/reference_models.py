"""
グリッドベースの「参照CPDモデル」定義。

咲川氏の抽象化ツールの出力から1本のログだけを表す「インスタンスCPD」を作る
cpd_bridge.py の方式（既存, vendor/trajectory_abstraction/src/cpd_bridge.py）とは異なり、
ここでは特定のログに依存しない「cut-inというシナリオ集合そのもの」を表す
CPDモデルを、格子(grid)の (lane, position) を直接使って書き下す。

設計方針（cut-in の一般形。v0.4で近距離/中距離/遠距離・並走・加速を1モデルに統合）:
  1. NPCは最初、ego と異なる「隣接レーン」("side lane", lane != ego_lane) の
     どこか (position は i_range の範囲内で任意 = 近距離でも中距離でも遠距離でもよい)
     から出発する。
  2. 隣接レーンに留まっている間は、position が自由に変化しうる
     （車線はまたがず、縦方向にだけ動く = 接近・離反・並走・加速）。
     `max_position_jump` を指定しない限り、1ステップでの position の変化量に
     上限を設けない。これにより「一定の距離を保って並走してから、
     加速して一気に前方へ抜けて合流する」のように position が大きく飛ぶ
     挙動も、同じモデルの中の別の経路として表現できる
     （並走そのものは、複数の圧縮済み状態が同じ/近い position に留まる
     経路として自然に表れる）。
  3. あるステップで「合流（マージ）」が起こる: 隣接レーンから ego レーン
     (lane == ego_lane) へ移る。position も同様に自由に変化しうる
     （合流の瞬間に速度差があれば position は大きく変わりうる）。
  4. 一度 ego レーンに入ったら、以後は二度と隣接レーンへは戻らない
     （lane はもう変化しない。position だけがまだ変化しうる）。

  この4番目の制約が「カットイン」と「蛇行(swerve)」を区別する核であり、
  一度合流したあとにまた元のレーンへ戻る、あるいは往復するような
  トラジェクトリは、この参照モデルでは絶対に受理されない
  （= 対応する遷移が存在しないため、その時点でUNSATになる）。

  位置（position）の変化量を自由にした一方で、レーンの遷移構造
  （隣接→ego の一方向のみ）は変えていない。CPDの強みは、こうして
  「本質的な制約（レーン遷移の一方向性）」と「自由度（位置の変化）」を
  同じモデルの中で書き分けられることにある。 = 1つの参照モデルが
  近距離cut-in・中距離cut-in・遠距離cut-in・並走からのcut-inを
  すべて別々の充足解（SATの異なるwitness）としてカバーする。

  side_lanes に複数の値（例: 左右両方）を許すことで、左からのカットイン・
  右からのカットインの両方を1つの参照モデルでカバーする
  （実際にどちらが起きたかは、SATソルバがどちらのinitを選ぶかで決まる）。

---
English:
Definition of a grid-based "reference CPD model".

Unlike the existing cpd_bridge.py approach (vendor/trajectory_abstraction/src/cpd_bridge.py),
which builds an "instance CPD" representing a single log from the output of
Mr. Sakikawa's abstraction tool, here we write down a CPD model that
represents "the set of cut-in scenarios itself", independent of any
particular log, directly using the grid's (lane, position).

Design policy (the general form of cut-in; v0.4 unifies near/medium/far
distance, side-by-side driving, and acceleration into a single model):
  1. The NPC starts somewhere in a "side lane" ("side lane", lane !=
     ego_lane) different from ego's (position may be anywhere within the
     range of i_range — near, medium, or far distance are all fine)
     as its initial position.
  2. While remaining in the side lane, position may change freely
     (only longitudinal movement, no lane change — this covers
     approaching, receding, driving side-by-side, and accelerating).
     Unless `max_position_jump` is specified, there is no upper bound on
     how much position may change in a single step. This lets behavior
     such as "drive side-by-side at a constant distance, then accelerate
     and jump far ahead to merge in" be expressed as just another path
     within the same model, with a large position jump
     (driving side-by-side itself naturally appears as a path where
     several compressed states stay at the same or a nearby position).
  3. At some step a "merge" happens: moving from the side lane to the
     ego lane (lane == ego_lane). Position may again change freely
     (if there is a speed difference at the moment of merging, position
     can change a lot).
  4. Once the car has entered the ego lane, it never returns to the side
     lane again (lane no longer changes; only position can still change).

  This fourth constraint is the core of what distinguishes "cut-in" from
  "swerving": a trajectory that merges and then returns to (or oscillates
  back to) the original lane is never accepted by this reference model
  (i.e. no corresponding transition exists, so the model becomes UNSAT at
  that point).

  While we made the change in position free, we did not change the lane
  transition structure (side lane -> ego lane, one direction only). The
  strength of CPD is that it lets us separate "essential constraints"
  (the one-directional lane transition) from "degrees of freedom"
  (position change) within the same model — so a single reference model
  covers near-distance cut-in, medium-distance cut-in, far-distance
  cut-in, and cut-in from driving side-by-side, all as separate
  satisfying solutions (different SAT witnesses).

  Allowing side_lanes to hold multiple values (e.g. both left and right)
  lets a single reference model cover cut-in from the left and cut-in
  from the right at once (which one actually occurred is determined by
  which init the SAT solver picks).
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from gcpd import Model


BoxKey = Tuple[int, int]  # (lane, position)


START_BOX = -1  # 「まだどこにも出現していない」ことを表すダミーの初期箱の番号
# (English) dummy initial box number representing "not yet appeared anywhere"


def build_cutin_reference(
    i_range: Sequence[int] = (-1, 0, 1, 2),  # logverify.zones の BEHIND/NEAR/MEDIUM/FAR に対応
    # (English) corresponds to BEHIND/NEAR/MEDIUM/FAR in logverify.zones
    side_lanes: Sequence[int] = (-1, 1),
    ego_lane: int = 0,
    car: str = "NPC",
    max_position_jump: Optional[int] = None,
) -> Tuple[Model, Dict[BoxKey, int]]:
    """cut-in の参照CPDモデルを構築する。

    「隣接レーンのどこから出発してもよい」という非決定性を表すため、
    実座標を持たないダミーの開始箱 START_BOX を1つ用意し、
    そこから全ての隣接レーン初期候補箱への遷移を張る。
    gcpd.py の Box(car,box,step) はモデル内で常に「その車について
    ちょうど1つの箱がアクティブ」という不変条件を、
    「唯一の初期箱から辿れる遷移だけを許す」ことで保っているため、
    もし複数の箱を直接 inits に入れてしまうと、
    それらが全ステップにわたって同時にアクティブのままになりうる
    （＝観測列との突き合わせが無意味になる）。
    そのため、必ずこのダミー箱1つだけを inits にする。

    membership.check_membership に渡す観測列は、この分だけ
    ステップが1つずれる（モデルの step 0 はダミー箱の期間に対応し、
    実際の観測はモデルの step 1 以降に対応する）。
    そのため check_membership を呼ぶ際は start_offset=1 を指定すること
    （logverify.membership.check_membership_cutin を使えば自動で処理される）。

    Args:
        max_position_jump: 1回の遷移で position が変化してよい最大量。
            None（デフォルト）なら無制限（近距離/中距離/遠距離・並走・
            加速など、position の変化量が様々なケースを同じモデルで
            まとめて扱いたい場合はこちら）。
            整数を指定すると、圧縮済み状態1つあたりの物理的な移動量に
            上限を課したい場合（データの粒度が細かく、
            大きなジャンプを異常値として弾きたい場合など）に使える。

    Returns:
        (model, box_id_of): box_id_of は (lane, position) -> box番号 の対応表
        （デバッグ・可視化用。START_BOX はキー (None, None) で登録される）。

    ---
    English:
    Build the reference CPD model for cut-in.

    To express the non-determinism of "it's fine to start anywhere in the
    side lane", we set up a single dummy start box START_BOX that has no
    real coordinates, and add transitions from it to every candidate
    initial box in the side lanes.
    gcpd.py's Box(car,box,step) maintains the model invariant that
    "exactly one box is active for a given car at any time" by allowing
    only transitions reachable from a single, unique initial box; if
    multiple boxes were put directly into inits, they could all remain
    active simultaneously across every step (making matching against the
    observation sequence meaningless).
    For that reason, inits must always be exactly this one dummy box.

    Because of this, the observation sequence passed to
    membership.check_membership is shifted by one step relative to the
    model (the model's step 0 corresponds to the dummy box's period, and
    the actual observations correspond to the model's step 1 onward).
    So when calling check_membership you must pass start_offset=1
    (using logverify.membership.check_membership_cutin handles this
    automatically).

    Args:
        max_position_jump: the maximum amount position may change in a
            single transition.
            None (the default) means unlimited (use this when you want a
            single model to handle cases with varying amounts of position
            change together, such as near/medium/far distance, driving
            side-by-side, and acceleration).
            Passing an integer imposes an upper bound on the physical
            distance moved per compressed state — useful when the data is
            fine-grained and you want to reject large jumps as outliers.

    Returns:
        (model, box_id_of): box_id_of is a mapping from
        (lane, position) -> box number (for debugging/visualization;
        START_BOX is registered under the key (None, None)).
    """
    i_values = list(i_range)
    lanes = list(side_lanes) + [ego_lane]

    def jump_ok(i1: int, i2: int) -> bool:
        return max_position_jump is None or abs(i2 - i1) <= max_position_jump

    m = Model()
    m.set_car([car])

    boxes: List[Tuple[str, int]] = [(car, START_BOX)]
    position: List[Tuple[str, int, int]] = []
    lane: List[Tuple[str, int, int]] = []
    box_id_of: Dict[BoxKey, int] = {(None, None): START_BOX}

    bid = 0
    for lane_val in lanes:
        for i in i_values:
            box_id_of[(lane_val, i)] = bid
            boxes.append((car, bid))
            position.append((car, bid, i))
            lane.append((car, bid, lane_val))
            bid += 1

    m.set_box(boxes)
    m.set_position(position)
    m.set_lane(lane)

    ntrans: List[Tuple[str, int, str, int]] = []

    def add_edge(from_key: BoxKey, to_key: BoxKey) -> None:
        if from_key in box_id_of and to_key in box_id_of and from_key != to_key:
            ntrans.append((car, box_id_of[from_key], car, box_id_of[to_key]))

    # 0. ダミー開始箱 -> 隣接レーンの候補となる全ての箱（非決定的な出発点選択:
    #    近距離/中距離/遠距離、どこから始まってもよい）
    # (English) 0. dummy start box -> every candidate box in the side lanes
    #    (non-deterministic choice of starting point: near/medium/far
    #    distance, any of them is fine).
    for lane_val in side_lanes:
        for i in i_values:
            add_edge((None, None), (lane_val, i))

    # 1. 隣接レーン内での縦方向移動（車線はまたがない。position の変化量は自由
    #    = 一定距離を保つ「並走」も、大きく詰める「加速」もこの中の経路として表現される）
    # (English) 1. longitudinal movement within the side lane (no lane
    #    change; position may change by any amount = both "driving
    #    side-by-side" at a constant distance and "accelerating" to close
    #    the gap sharply are expressed as paths within this).
    for lane_val in side_lanes:
        for i1 in i_values:
            for i2 in i_values:
                if i1 != i2 and jump_ok(i1, i2):
                    add_edge((lane_val, i1), (lane_val, i2))

    # 2. 合流（隣接レーン -> egoレーン）。position の変化量は自由。
    # (English) 2. merge (side lane -> ego lane). Position may change by
    #    any amount.
    for lane_val in side_lanes:
        for i1 in i_values:
            for i2 in i_values:
                if jump_ok(i1, i2):
                    add_edge((lane_val, i1), (ego_lane, i2))

    # 3. 合流後は ego レーン内でのみ縦方向に移動できる（隣接レーンへは戻れない）
    # (English) 3. after merging, longitudinal movement is only possible
    #    within the ego lane (cannot return to the side lane).
    for i1 in i_values:
        for i2 in i_values:
            if i1 != i2 and jump_ok(i1, i2):
                add_edge((ego_lane, i1), (ego_lane, i2))

    m.set_ntrans(ntrans)
    m.set_init([(car, START_BOX)])

    m.max_step = 0  # membership.check_membership が観測系列の長さに合わせて設定する
    # (English) set by membership.check_membership to match the length of
    # the observation sequence
    return m, box_id_of


def describe(model: Model, box_id_of: Dict[BoxKey, int]) -> str:
    lines = [f"cars={model.cars}", f"#boxes={len(model.boxes)}", f"#ntrans={len(model.ntrans)}"]
    id_of_box = {v: k for k, v in box_id_of.items()}
    lines.append(f"inits={[id_of_box[n] for (_, n) in model.inits]}")
    return "\n".join(lines)


@dataclass
class CutinReferenceScenario:
    """参照CPDモデルと、それに対応するログの抽象化ルールを1つにまとめたもの。

    これまで、参照CPD側の語彙（`i_range=(-1,0,1,2)` = BEHIND/NEAR/MEDIUM/FAR、
    `side_lanes`, `ego_lane`）と、生ログをその語彙に丸めるための実際の
    しきい値（`logverify.zones.ZoneThresholds` の near_max/medium_max、
    および横方向の格子幅 gy）は、別々の場所（`build_cutin_reference` の
    呼び出しと `ZoneThresholds(...)` の呼び出し）で、デモスクリプトを書く
    人間が「値を合わせておく」ことに頼って別々に指定されていた
    （例: demo_real_ajisai_log.py 参照）。値を変え忘れる、あるいは
    i_range を BEHIND..FAR 以外に変えたのに thresholds は据え置く、
    といった不整合が起きても、実行時までそれと気づけない。

    このクラスは、「参照モデル」を、その箱構造（`model`/`box_id_of`）
    と、ログをその箱構造の語彙に自動的に落とし込むための抽象化ルール
    （`thresholds`/`gy`/`ego_lane`/`side_lanes`）を持つ1つの単位として
    まとめ、`abstract()` / `check()` の2つの操作をその単位に対して
    行えるようにする。これにより、「ログを、参照モデルの抽象度に
    合わせて自動的に抽象化する」ことが、2つの独立した設定を人間が
    手で同期させる作業ではなく、1つのオブジェクトに対する1回の
    呼び出しになる。

    ---
    English:
    Bundles a reference CPD model together with the abstraction rule used
    to round a raw log into that model's own vocabulary.

    Previously, the reference CPD's vocabulary (`i_range=(-1,0,1,2)` =
    BEHIND/NEAR/MEDIUM/FAR, `side_lanes`, `ego_lane`) and the actual
    thresholds used to round a raw log into that vocabulary
    (`logverify.zones.ZoneThresholds`'s near_max/medium_max, and the
    lateral grid width gy) were specified separately (one call to
    `build_cutin_reference`, one call to `ZoneThresholds(...)`), relying on
    whoever wrote the demo script to keep the two in sync by hand (see
    demo_real_ajisai_log.py). If they drift apart -- e.g. i_range is
    changed to something other than BEHIND..FAR but thresholds is left
    as-is -- nothing catches it until runtime.

    This class packages "a reference model" as a single unit holding both
    its box structure (`model`/`box_id_of`) and the abstraction rule that
    automatically maps a raw log into that box structure's vocabulary
    (`thresholds`/`gy`/`ego_lane`/`side_lanes`), and exposes that unit's
    two operations, `abstract()` and `check()`. This turns "automatically
    abstract a log to match the reference model's level of abstraction"
    from a manual synchronization task between two independent settings
    into a single call against a single object.
    """

    model: Model
    box_id_of: Dict[BoxKey, int]
    thresholds: "ZoneThresholds"
    gy: float
    ego_lane: int
    side_lanes: Tuple[int, ...]
    car: str = "NPC"

    def abstract(self, rel_xy: Sequence[Tuple[float, float]]) -> List["ZoneState"]:
        """生の (rx, ry) 系列を、この参照モデル自身の語彙（BEHIND/NEAR/MEDIUM/FAR
        × side_lanes/ego_lane）に自動的に丸め、イベント駆動で圧縮する。

        ---
        English:
        Automatically rounds a raw (rx, ry) sequence into this reference
        model's own vocabulary (BEHIND/NEAR/MEDIUM/FAR x
        side_lanes/ego_lane), compressing it in an event-driven manner.
        """
        from logverify.zones import zone_states_from_relative_xy

        return zone_states_from_relative_xy(rel_xy, self.gy, self.thresholds)

    def check(self, rel_xy: Sequence[Tuple[float, float]]):
        """生ログを自動的に抽象化し、この参照モデルに対するmembership checkまで行う。

        抽象化された (lane, zone) の各値が、この参照モデルの語彙
        （box_id_of のキー）の外に出ていた場合は、`check_membership` が
        `unmatched_step` 付きの UNSAT として検出する（＝「参照モデルの
        抽象度に合わせて自動的に抽象化できたか」を、SAT/UNSATとは別の
        観点から検証したことになる）。

        Returns:
            (states, result): states は abstract() の戻り値、result は
            logverify.membership.MembershipResult。

        ---
        English:
        Automatically abstracts a raw log and runs a membership check
        against this reference model.

        If any abstracted (lane, zone) value falls outside this reference
        model's own vocabulary (the keys of box_id_of), `check_membership`
        detects it as an UNSAT with `unmatched_step` set (this amounts to
        a check, orthogonal to SAT/UNSAT itself, of whether the log could
        actually be abstracted to match the reference model's level of
        abstraction).

        Returns:
            (states, result): states is abstract()'s return value, result
            is a logverify.membership.MembershipResult.
        """
        from logverify.membership import check_membership_cutin

        states = self.abstract(rel_xy)
        observed = [(s.lane, s.zone) for s in states]
        result = check_membership_cutin(self.model, observed, car=self.car)
        return states, result


def build_cutin_reference_scenario(
    near_max: float = 5.0,
    medium_max: float = 20.0,
    gy: float = 3.5,
    side_lanes: Sequence[int] = (-1, 1),
    ego_lane: int = 0,
    car: str = "NPC",
    max_position_jump: Optional[int] = None,
) -> CutinReferenceScenario:
    """cut-in の参照モデルを、その抽象化ルール（distance帯のしきい値・
    横方向の格子幅）ごと1つにまとめて構築する。

    `build_cutin_reference` の `i_range` は、常に
    `logverify.zones` の BEHIND(-1)/NEAR(0)/MEDIUM(1)/FAR(2) に固定する
    （この2つが食い違うことを、そもそも構造的にできなくする）。

    ---
    English:
    Builds the cut-in reference model together with its abstraction rule
    (distance-band thresholds, lateral grid width) as a single bundle.

    `build_cutin_reference`'s `i_range` is always fixed to
    `logverify.zones`'s BEHIND(-1)/NEAR(0)/MEDIUM(1)/FAR(2) here
    (structurally ruling out the two ever drifting apart).
    """
    from logverify.zones import BEHIND, NEAR, MEDIUM, FAR, ZoneThresholds

    model, box_id_of = build_cutin_reference(
        i_range=(BEHIND, NEAR, MEDIUM, FAR),
        side_lanes=side_lanes,
        ego_lane=ego_lane,
        car=car,
        max_position_jump=max_position_jump,
    )
    thresholds = ZoneThresholds(near_max=near_max, medium_max=medium_max)
    return CutinReferenceScenario(
        model=model,
        box_id_of=box_id_of,
        thresholds=thresholds,
        gy=gy,
        ego_lane=ego_lane,
        side_lanes=tuple(side_lanes),
        car=car,
    )
