"""
「参照CPDモデルに、離散化したログが適合する(membership)か」を判定する。

docs/log_to_cpd_verification_design.md 5.1節・9.3節で述べた
check_membership の実装:

  観測された (lane, position) の列（ステップ 0..T）を、参照モデル自身の
  Box/Pos/Lane 制約 (add_pos, add_lane, add_init, add_trans) に対する
  「追加の制約」として solver に投入する。具体的には、各ステップ t について
  「その (lane, position) を持つ箱のどれかがアクティブである」という
  論理和制約を足す。

  - SAT  -> ログの挙動は参照モデルが定義するシナリオ集合の要素として
            矛盾なく説明できる（適合）。
  - UNSAT -> 参照モデルが許さない遷移が起きている
             （例: 一度合流してから別のレーンへ戻る、蛇行的な動きなど）。

gcpd.py はモジュールレベルの可変状態 (solver, c2i) を持つため、
複数回 check_membership を呼ぶ際は明示的にリセットする。
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import gcpd
from gcpd import Box, Model, Or, sat


@dataclass
class MembershipResult:
    is_member: bool
    z3_result: str
    max_step: int
    num_steps_observed: int
    unmatched_step: Optional[int] = None  # 参照モデルに対応する箱が1つも無かった最初のステップ

    def __str__(self) -> str:
        if self.unmatched_step is not None:
            return (
                f"UNSAT（適合しない）: ステップ {self.unmatched_step} に対応する箱が"
                f"参照モデルに存在しません（観測された (lane, position) が"
                f"参照モデルの語彙の外にあります）"
            )
        verdict = "SAT（適合する）" if self.is_member else "UNSAT（適合しない）"
        return f"{verdict}  [z3: {self.z3_result}, steps={self.num_steps_observed}, max_step={self.max_step}]"


def reset_solver() -> None:
    """gcpd モジュールのグローバルな solver/c2i をリセットする。

    gcpd.py は `solver = Solver()` をモジュールロード時に一度だけ作る設計になっており、
    複数のモデルを別々に検証しようとすると制約が蓄積してしまう。
    """
    gcpd.solver = gcpd.Solver()
    gcpd.c2i = {}


def _candidate_boxes(model: Model, car: str, lane_val: int, position_val: int) -> List[int]:
    pos_boxes = {n for (c, n, p) in model.position if c == car and p == position_val}
    lane_boxes = {n for (c, n, l) in model.lane if c == car and l == lane_val}
    return sorted(pos_boxes & lane_boxes)


def check_membership(
    model: Model,
    observed_lane_position: Sequence[Tuple[int, int]],
    car: Optional[str] = None,
    start_offset: int = 0,
) -> MembershipResult:
    """observed_lane_position: [(lane_0, position_0), (lane_1, position_1), ...] の順序列。

    圧縮済みグリッド状態列 (logverify.grid_bridge.GridState のリスト) から
    そのまま作る場合は
        [(s.k, s.i) for s in states]
    を渡す。

    start_offset: 観測列の先頭がモデルのどのstepに対応するかのずれ。
    例えば reference_models.build_cutin_reference のようにモデルの
    step 0 が「実座標を持たないダミーの開始箱」に対応する場合は
    start_offset=1 を指定する（観測 t 番目は モデルの step t+1 に対応する）。
    """
    reset_solver()

    car_name = car or model.cars[0]
    num_steps = len(observed_lane_position)
    needed_max_step = num_steps - 1 + start_offset if num_steps > 0 else 0
    model.max_step = max(model.max_step, needed_max_step)

    gcpd.init(model)
    gcpd.add_pos(model)
    gcpd.add_lane(model)
    gcpd.add_init(model)
    gcpd.add_trans(model)

    for t, (lane_val, position_val) in enumerate(observed_lane_position):
        step = t + start_offset
        candidates = _candidate_boxes(model, car_name, lane_val, position_val)
        if not candidates:
            return MembershipResult(
                is_member=False,
                z3_result="unsat (no matching box)",
                max_step=model.max_step,
                num_steps_observed=num_steps,
                unmatched_step=t,
            )
        gcpd.solver.add(Or([Box(gcpd.c2i[car_name], n, step) for n in candidates]))

    result = gcpd.solver.check()
    return MembershipResult(
        is_member=(result == sat),
        z3_result=str(result),
        max_step=model.max_step,
        num_steps_observed=num_steps,
    )


def check_membership_cutin(
    model: Model,
    observed_lane_position: Sequence[Tuple[int, int]],
    car: Optional[str] = None,
) -> MembershipResult:
    """reference_models.build_cutin_reference が作るモデル専用のショートカット。
    ダミー開始箱の分のオフセット (start_offset=1) を自動で付与する。"""
    return check_membership(model, observed_lane_position, car=car, start_offset=1)
