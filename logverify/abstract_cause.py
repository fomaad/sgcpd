"""ログの原因分析のための「抽象解釈」演算子。

## 経緯・狙い

これまでの方法A（12.1〜12.11節）は、rx・ryという連続値を格子で区切って
「箱」に離散化する抽象化だった。この格子は、細かくすればするほど元の値に
近づくが、それ自体は「速すぎる」「弱すぎる」といった**原因を直接語る
抽象値**にはならない（12.7節のEgoスイムレーンの「強い減速(-2.55)」は、
あくまで生の加速度値を固定の閾値でバケット分けしただけであり、
「なぜその減速では足りなかったのか」には答えていない）。

ユーザーからの依頼:
  「ログの抽象解釈をしたい．衝突の原因がわかる抽象度に自動的に抽象化
  したい．抽象化したモデルでは，それに現れている要素を見れば，原因が
  特定できるものが出現していてほしい．例えば，実際の速度ではなくて，
  速すぎる等の抽象的な値である．抽象的な値の上で演算が必要であれば，
  それを抽象解釈で実現する．咲川さんがcut-in演算子に関しては，抽象値で
  演算子に解釈を与えていると思います．同じような感じです．」

咲川氏のcut-in演算子（`sakikawa_relations.detect_cutin`）は、
「(lane, position)という抽象値の上で」隣接レーンからego車線への遷移
パターンを直接判定する演算子であり、生のrx/ryの上で判定しているわけ
ではない。本モジュールは、同じ発想を「原因」の軸に対して適用する:

  - 生の値（例: 加速度 -2.55 m/s^2）をそのまま使うのではなく、
  - **その状況で本来必要だった値**（例: 衝突を避けるのに必要な減速度）
    を物理的に計算し、
  - 実際の値をその必要値と比較した**抽象的な十分性の判定**
    （不要／適切／弱い／非常に弱い）を演算子として与える。

これにより、抽象化されたCPDの箱に現れる値そのもの（「弱い」「陳腐」
「接触可能」）を見るだけで、原因がどれであるかが直接読み取れることを
目指す。「弱い」の閾値自体も、9.5節以来のプロジェクトの一貫した方針
（分析用に選んだ恣意的な値ではなく、物理的・車両サイズ由来の基準を使う）
に従い、必要値からの比（`ratio`）で定義する。

## 3つの抽象解釈演算子

1. `classify_deceleration_adequacy`: 減速の十分性。
   「その状況で衝突を避けるために本来必要だった減速度」
   （`required_deceleration_magnitude`、教科書的な制動距離の式
   `v^2 / (2d)` で計算される、これも恣意的な値ではなく運動学的に
   導かれる値）に対して、実際に達成された減速度が何倍だったかを
   ratioとして評価し、{過剰, 適切, 弱い, 非常に弱い}に分類する。

2. `classify_prediction_reliability`: NPC予測経路の信頼性。
   Autowareの`predict_paths`が「今後こう動く」と予測した横方向の
   変化と、実際に起きた横方向の変化の符号・大きさを比較し、
   信頼度(confidence)と合わせて{的確, 低信頼, 陳腐（自信満々の誤り）}
   に分類する。「陳腐」は、高い信頼度を保ったまま実際の動きと逆の
   予測を続ける、最も危険な状態を指す。

3. `classify_contact_margin`: 横方向の接触余裕。
   12.8節の「接触境界」（両者の車体半幅の和）を単位とした比
   （`|ry| / (ego_half_width + npc_half_width)`）で、
   {余裕, 接近中, 接触可能, 接触（衝突）}の4段階に分類する
   （12.8節の2値のego車線帯/接触境界を、順序尺度に拡張したもの）。

---
English:
"Abstract interpretation" operators for root-cause analysis of a log.

## Background and goal

Method A so far (Sections 12.1-12.11) discretized the continuous values
rx/ry onto a grid, into "boxes". Making that grid finer approaches the
original values, but it never becomes an **abstract value that directly
names a cause** ("too fast", "too weak") -- Section 12.7's Ego swimlane
label "strong braking (-2.55)" is just the raw acceleration bucketed by a
fixed threshold; it does not answer "why was that deceleration not
enough?"

The user asked: "I want to abstractly interpret the log. I want it
automatically abstracted to a level of abstraction where the cause of the
collision can be seen. In the abstracted model, I want elements to appear
that, just by looking at them, let you pin down the cause -- for example,
not the actual speed, but an abstract value like 'too fast'. Where
operations on abstract values are needed, realize them via abstract
interpretation. I believe Mr. Sakikawa gives his cut-in operator its
interpretation over abstract values -- something similar."

Mr. Sakikawa's cut-in operator (`sakikawa_relations.detect_cutin`) judges
the transition pattern from the adjacent lane to the ego lane directly
"over the abstract values (lane, position)", not over raw rx/ry. This
module applies the same idea to the "cause" axis:

  - rather than using a raw value as-is (e.g. acceleration = -2.55 m/s^2),
  - physically compute **the value that situation actually required**
    (e.g. the deceleration needed to avoid the collision), and
  - give an operator that yields an **abstract adequacy judgment**
    (unnecessary / adequate / weak / very weak) by comparing the actual
    value against that required value.

The goal is that the values appearing in the abstracted CPD's boxes
themselves ("weak", "stale", "contact-possible") let the cause be read
off directly. Following this project's consistent policy since Section
9.5 (use a physical/vehicle-size-derived basis, not an arbitrary value
chosen for the analysis), even the "weak" threshold is defined as a ratio
to the required value.

## The three abstract-interpretation operators

1. `classify_deceleration_adequacy`: deceleration adequacy. Evaluates the
   ratio of the actually achieved deceleration to "the deceleration that
   situation actually required to avoid the collision"
   (`required_deceleration_magnitude`, computed from the textbook braking
   distance formula `v^2 / (2d)` -- itself not an arbitrary value but one
   derived kinematically), classifying it into {overkill, adequate, weak,
   very weak}.

2. `classify_prediction_reliability`: reliability of the NPC's predicted
   path. Compares the lateral change Autoware's `predict_paths` predicted
   ("it will move like this") against the sign and magnitude of what
   actually happened, together with the confidence, classifying into
   {accurate, low-confidence, stale (confidently wrong)}. "Stale" denotes
   the most dangerous state: continuing to predict the opposite of what
   is actually happening while confidence stays high.

3. `classify_contact_margin`: lateral contact margin. Using Section 12.8's
   "contact boundary" (the sum of both vehicles' half-widths) as the
   unit, classifies the ratio `|ry| / (ego_half_width + npc_half_width)`
   into 4 levels {clear, approaching, contact-possible, contact
   (colliding)} -- an ordinal extension of Section 12.8's binary ego-lane
   band / contact boundary.
"""

import math
from dataclasses import dataclass
from typing import Optional


def required_deceleration_magnitude(closing_speed: float, distance_to_contact: float) -> float:
    """教科書的な制動距離の式(v^2 / (2d))で、衝突を避けるのに必要な減速度の
    大きさ（m/s^2、常に非負）を求める。

    closing_speedが0以下（近づいていない）、またはdistance_to_contactが
    0以下（すでに接触範囲に入っている＝もはや減速では避けられない）の
    場合は、有限の値ではなく`math.inf`を返す
    （「間に合わない」という抽象値`classify_deceleration_adequacy`側で
    「非常に弱い」に直結させるため）。

    ---
    English:
    Computes the magnitude of deceleration (m/s^2, always non-negative)
    required to avoid a collision, from the textbook braking-distance
    formula v^2 / (2d).

    Returns `math.inf` (rather than a finite value) when closing_speed is
    not positive (not approaching) or distance_to_contact is not positive
    (already within contact range -- deceleration alone can no longer
    avoid it), so that `classify_deceleration_adequacy` maps this
    "cannot make it in time" case directly to "very weak".
    """
    if closing_speed <= 0 or distance_to_contact <= 0:
        return math.inf
    return (closing_speed ** 2) / (2.0 * distance_to_contact)


def classify_deceleration_adequacy(
    achieved_decel_magnitude: float,
    required_decel_magnitude: float,
    overkill_ratio: float = 1.5,
    adequate_ratio: float = 1.0,
    weak_ratio: float = 0.5,
) -> str:
    """実際に達成された減速度の大きさが、必要な減速度の大きさの何倍かで、
    {不要, 過剰, 適切, 弱い, 非常に弱い}に分類する。

    ---
    English:
    Classifies into {unnecessary, overkill, adequate, weak, very weak}
    based on what multiple of the required deceleration the actually
    achieved deceleration reached.
    """
    if not math.isfinite(required_decel_magnitude) or required_decel_magnitude <= 0:
        return "不要" if achieved_decel_magnitude < 0.05 else "過剰"
    if not math.isfinite(achieved_decel_magnitude):
        return "非常に弱い"
    ratio = achieved_decel_magnitude / required_decel_magnitude
    if ratio >= overkill_ratio:
        return "過剰"
    if ratio >= adequate_ratio:
        return "適切"
    if ratio >= weak_ratio:
        return "弱い"
    return "非常に弱い"


def classify_prediction_reliability(
    confidence: float,
    predicted_delta: float,
    actual_delta: float,
    confidence_threshold: float = 0.6,
    magnitude_ratio_ok: float = 0.4,
) -> str:
    """NPC予測経路の「今後の変化予測」(predicted_delta)と、実際に起きた
    変化(actual_delta)の符号・大きさを比較し、{的確, 低信頼, 陳腐}に
    分類する。

    - confidenceが低い場合は、予測の当たり外れによらず「低信頼」。
    - confidenceが高いのに符号が逆、または大きさの比が
      `magnitude_ratio_ok`未満（=予測が実際の変化をほとんど捉えて
      いない）場合は、最も危険な「陳腐」（自信満々の誤り）。
    - confidenceが高く、符号が一致し大きさも近い場合は「的確」。

    ---
    English:
    Compares the sign and magnitude of the NPC prediction's forecast
    change (predicted_delta) against the actually observed change
    (actual_delta), together with confidence, classifying into
    {accurate, low-confidence, stale}.

    - Low confidence -> "low-confidence" regardless of whether the
      prediction happened to be right.
    - High confidence but the sign is opposite, or the magnitude ratio is
      below `magnitude_ratio_ok` (the prediction barely captures the
      actual change) -> the most dangerous state, "stale" (confidently
      wrong).
    - High confidence, matching sign, and comparable magnitude ->
      "accurate".
    """
    if confidence < confidence_threshold:
        return "低信頼"
    if abs(actual_delta) < 1e-6:
        return "的確" if abs(predicted_delta) < 1e-6 else "陳腐"
    same_sign = (predicted_delta * actual_delta) > 0
    magnitude_ratio = abs(predicted_delta) / abs(actual_delta)
    if same_sign and magnitude_ratio >= magnitude_ratio_ok:
        return "的確"
    return "陳腐"


def classify_contact_margin(
    ry: float,
    ego_half_width: float,
    npc_half_width: float,
    is_colliding: bool = False,
    approach_ratio: float = 1.5,
    contact_ratio: float = 1.0,
) -> str:
    """横方向オフセットryを、両者の車体半幅の和（12.8節の接触境界）を単位
    とした比で、{余裕, 接近中, 接触可能, 接触}の4段階に分類する。

    ---
    English:
    Classifies the lateral offset ry, as a ratio to the sum of both
    vehicles' half-widths (Section 12.8's contact boundary), into 4
    levels: {clear, approaching, contact-possible, contact}.
    """
    if is_colliding:
        return "接触"
    contact_half = ego_half_width + npc_half_width
    if contact_half <= 0:
        return "接触可能"
    ratio = abs(ry) / contact_half
    if ratio >= approach_ratio:
        return "余裕"
    if ratio >= contact_ratio:
        return "接近中"
    return "接触可能"


@dataclass
class AbstractCauseLabels:
    decel_adequacy: Optional[str] = None
    pred_reliability: Optional[str] = None
    contact_margin: Optional[str] = None


def box_aggregated_deceleration_ratio(
    rxs, gk, cc, start_frame: int, end_frame: int, eh_l: float, nh_l: float, pad_frames: int = 10
) -> Optional[float]:
    """12.12/12.13節で見つかった「減速の十分性」演算子の不安定さ（`v^2/(2d)`
    が単一フレームの評価に極めて敏感なこと）への対処。単一フレームで
    評価するのではなく、**その値が属するCPDの箱（`GridState`）の区間全体**
    をclosing_speedと`achieved`の平均化ウィンドウとして使う。

    12.14節で導入した「スナップショット列＝CPDの箱列」という考え方と
    同じで、CPDの箱そのものが自然な時間積分の単位になっている、という
    発見に基づく（詳細は12.15節）。

    Returns:
        achieved/required の比。requiredが有限かつ正でない場合はNone
        （すでに接触範囲に入っている等、比では語れない状況）。

    ---
    English:
    Addresses the instability of the deceleration-adequacy operator found
    in Sections 12.12/12.13 (that `v^2/(2d)` is extremely sensitive to a
    single-frame evaluation). Rather than evaluating at a single frame,
    uses **the whole interval of the CPD box (`GridState`) that value
    belongs to** as the averaging window for both closing_speed and
    `achieved`.

    This follows directly from Section 12.14's "snapshot sequence = CPD
    box sequence" idea: the CPD box itself turns out to be a natural
    unit of time-integration (see Section 12.15 for details).

    Returns:
        The ratio achieved/required, or None if required is not finite
        and positive (e.g. already within contact range, where a ratio
        is not meaningful).
    """
    t0, t1 = gk[start_frame]["timestamp"], gk[end_frame]["timestamp"]
    dt = t1 - t0
    if dt > 0:
        i0, i1 = start_frame, end_frame
    else:
        i0, i1 = max(0, start_frame - pad_frames), min(len(rxs) - 1, end_frame + pad_frames)
        dt = gk[i1]["timestamp"] - gk[i0]["timestamp"]
    closing = (rxs[i0] - rxs[i1]) / dt if dt > 0 else 0.0
    mid = (start_frame + end_frame) // 2
    dist = rxs[mid] - (eh_l + nh_l)
    required = required_deceleration_magnitude(closing, dist)
    if not math.isfinite(required) or required <= 0:
        return None
    accs = [abs(e["longitudinal"]["acceleration"]) for e in cc if t0 <= e["timestamp"] <= t1]
    if not accs:
        nearest = min(cc, key=lambda e: abs(e["timestamp"] - gk[mid]["timestamp"]))
        accs = [abs(nearest["longitudinal"]["acceleration"])]
    achieved = sum(accs) / len(accs)
    return achieved / required
