"""
方法C: 複数のログを1つのCPDモデルに統合抽象化する。

これまでの方法との違い（3つとも別々の方法として併存させる）:

  - 方法A（既存, vendor/trajectory_abstraction/src/cpd_bridge.py）:
    咲川氏の15領域抽象化を使い、1本のログから、そのログだけを表す
    「インスタンスCPD」を作る。

  - 方法B（logverify/reference_models.py, logverify/zones.py）:
    特定のログに依存しない「シナリオ集合そのもの」（cut-inなど）を
    設計者が手で書き下し、格子/距離帯ベースに離散化した任意のログが
    それに適合するか（membership check）を判定する。

  - 方法C（このモジュール）: 複数本の具体的なログをまとめて、
    1つのCPDモデルに機械的に抽象化する。各ログを互いに区別できる
    程度に細かい格子を選び、各ログの箱列（(lane, position)の列）を
    「そのログが辿った経路」としてモデルに書き込む。
    複数のログが同じ箱を通れば、そこでモデルの中で経路が合流・分岐する
    ことになり、モデルからシナリオを列挙する (`gcpd.s_gen`) と、
    入力した全てのログの経路に加えて、それらの部分列を組み合わせた
    「入力にはなかった経路」も一般に列挙されうる
    （＝複数の具体例から、それらを包含するシナリオ集合を機械的に
    構築するという使い方）。

使い方の要点:
  1. `find_distinguishing_grid` で、与えられた全てのログが互いに異なる
     箱列に離散化されるような格子サイズ (gx, gy) を自動的に探す
     （粗い格子から始めて、重複がなくなるまで細かくしていく）。
  2. `build_union_model` で、その格子サイズを使って全ログの箱列を
     1つの `gcpd.Model` に統合する（ログごとの経路の合併＝グラフの union）。
  3. `verify_logs_included` で、統合したモデルに対して各ログの箱列が
     実際に membership check で SAT になる（＝そのログがモデルの
     シナリオ集合に含まれる）ことを確認する。
  4. 必要なら `count_scenarios` / `enumerate_scenarios` で、モデルから
     実際に列挙されるシナリオの総数・中身を確認する
     （入力したログの本数と一致すれば「一般化なしで再現された」、
     それより多ければ「入力にない経路も生成された」ことがわかる）。
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import gcpd
from gcpd import Model

from logverify.grid_bridge import compress_to_grid_states
from logverify.membership import MembershipResult, check_membership, reset_solver


BoxKey = Tuple[int, int]  # (lane, position)
START_BOX = -1  # 実座標を持たないダミーの開始箱（logverify.reference_models と同じ仕掛け）


def _sequences_from_grid(
    trajectories: Sequence[Sequence[Tuple[float, float]]], gx: float, gy: float
) -> List[List[BoxKey]]:
    sequences = []
    for traj in trajectories:
        rxs = [p[0] for p in traj]
        rys = [p[1] for p in traj]
        states = compress_to_grid_states(rxs, rys, gx, gy)
        sequences.append([(s.k, s.i) for s in states])
    return sequences


def _sequences_distinct(sequences: Sequence[Sequence[BoxKey]]) -> bool:
    seen = set()
    for seq in sequences:
        key = tuple(seq)
        if key in seen:
            return False
        seen.add(key)
    return True


def find_distinguishing_grid(
    trajectories: Sequence[Sequence[Tuple[float, float]]],
    gx0: float = 5.0,
    gy0: float = 3.5,
    shrink: float = 0.5,
    max_iters: int = 8,
) -> Tuple[float, float, List[List[BoxKey]]]:
    """全てのログが互いに異なる箱列に離散化されるまで、格子を細かくしていく。

    Returns:
        (gx, gy, sequences): 見つかった格子サイズと、その格子での各ログの箱列。
        max_iters 回試しても区別できなければ、最後に試した（最も細かい）
        格子でのsequencesをそのまま返す（呼び出し側で
        _sequences_distinct により重複の有無を確認できる）。
    """
    gx, gy = gx0, gy0
    sequences: List[List[BoxKey]] = []
    for _ in range(max_iters):
        sequences = _sequences_from_grid(trajectories, gx, gy)
        if _sequences_distinct(sequences):
            return gx, gy, sequences
        gx *= shrink
        gy *= shrink
    return gx, gy, sequences


@dataclass
class MultiLogModel:
    model: Model
    box_id_of: Dict[BoxKey, int]
    sequences: List[List[BoxKey]]  # 各ログの箱列（(lane, position) のタプル）
    gx: float
    gy: float
    id_of_box: Dict[int, BoxKey] = field(default_factory=dict)

    def __post_init__(self):
        self.id_of_box = {v: k for k, v in self.box_id_of.items()}


def build_union_model(
    trajectories: Sequence[Sequence[Tuple[float, float]]],
    gx: Optional[float] = None,
    gy: Optional[float] = None,
    car: str = "NPC",
    auto_grid: bool = True,
) -> MultiLogModel:
    """複数ログを1つのCPDモデルに統合する。

    gx/gy を省略した場合（auto_grid=True, デフォルト）は
    find_distinguishing_grid を使って全ログを区別できる格子を自動的に探す。
    """
    if gx is not None and gy is not None:
        sequences = _sequences_from_grid(trajectories, gx, gy)
    else:
        gx0 = gx if gx is not None else 5.0
        gy0 = gy if gy is not None else 3.5
        if auto_grid:
            gx, gy, sequences = find_distinguishing_grid(trajectories, gx0, gy0)
            if not _sequences_distinct(sequences):
                raise ValueError(
                    f"格子を (gx={gx}, gy={gy}) まで細かくしても全ログを区別できませんでした。"
                    "max_iters を増やすか、gx0/gy0 を小さくしてやり直してください。"
                )
        else:
            gx, gy = gx0, gy0
            sequences = _sequences_from_grid(trajectories, gx, gy)

    box_id_of: Dict[BoxKey, int] = {(None, None): START_BOX}
    boxes: List[Tuple[str, int]] = [(car, START_BOX)]
    position: List[Tuple[str, int, int]] = []
    lane: List[Tuple[str, int, int]] = []

    def get_or_create_box(key: BoxKey) -> int:
        if key not in box_id_of:
            bid = len(boxes) - 1  # boxes[0] は START_BOX なので、新規箱は 0 から連番
            box_id_of[key] = bid
            boxes.append((car, bid))
            lane_val, pos_val = key
            position.append((car, bid, pos_val))
            lane.append((car, bid, lane_val))
        return box_id_of[key]

    ntrans_set = set()
    max_len = 0
    for seq in sequences:
        max_len = max(max_len, len(seq))
        prev_box = START_BOX
        for key in seq:
            cur_box = get_or_create_box(key)
            if prev_box != cur_box:
                ntrans_set.add((car, prev_box, car, cur_box))
            prev_box = cur_box

    m = Model()
    m.set_car([car])
    m.set_box(boxes)
    m.set_position(position)
    m.set_lane(lane)
    m.set_ntrans(sorted(ntrans_set))
    m.set_init([(car, START_BOX)])
    m.max_step = max_len  # ダミー開始箱の分だけ +1 されているのでこれでよい

    return MultiLogModel(model=m, box_id_of=box_id_of, sequences=sequences, gx=gx, gy=gy)


def verify_logs_included(mlm: MultiLogModel, car: Optional[str] = None) -> List[MembershipResult]:
    """統合モデルに、元になった各ログの箱列が実際に含まれる(SAT)ことを確認する。"""
    results = []
    for seq in mlm.sequences:
        result = check_membership(mlm.model, seq, car=car, start_offset=1)
        results.append(result)
    return results


def count_scenarios(mlm: MultiLogModel) -> int:
    """統合モデルから列挙できるシナリオの総数を返す（入力ログの本数と比較するため）。"""
    reset_solver()
    m = mlm.model
    gcpd.init(m)
    gcpd.add_pos(m)
    gcpd.add_lane(m)
    gcpd.add_init(m)
    gcpd.add_trans(m)
    return gcpd.enum_count(m)


def enumerate_scenarios(mlm: MultiLogModel) -> List[List[Tuple[int, BoxKey]]]:
    """統合モデルから全シナリオを列挙し、各シナリオを [(step, (lane,position)), ...] の形で返す。
    （START_BOX に対応するstep 0は除く）"""
    reset_solver()
    m = mlm.model
    m.num_model = 10_000  # enum_ss は num_model 回までしか列挙しないため、十分大きくしておく
    gcpd.init(m)
    gcpd.add_pos(m)
    gcpd.add_lane(m)
    gcpd.add_init(m)
    gcpd.add_trans(m)
    history = gcpd.enum_ss(m)

    scenarios = []
    for h in history:
        # h は [(car, box, lane, pos, step), ...] のリスト（gcpd.enum_ss の形式）
        by_step = sorted({(s, l, p) for (c, n, l, p, s) in h if n != START_BOX}, key=lambda x: x[0])
        scenarios.append([(s, (l, p)) for (s, l, p) in by_step])
    return scenarios
