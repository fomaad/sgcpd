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
"""

from typing import Dict, List, Optional, Sequence, Tuple

from gcpd import Model


BoxKey = Tuple[int, int]  # (lane, position)


START_BOX = -1  # 「まだどこにも出現していない」ことを表すダミーの初期箱の番号


def build_cutin_reference(
    i_range: Sequence[int] = (-1, 0, 1, 2),  # logverify.zones の BEHIND/NEAR/MEDIUM/FAR に対応
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
    for lane_val in side_lanes:
        for i in i_values:
            add_edge((None, None), (lane_val, i))

    # 1. 隣接レーン内での縦方向移動（車線はまたがない。position の変化量は自由
    #    = 一定距離を保つ「並走」も、大きく詰める「加速」もこの中の経路として表現される）
    for lane_val in side_lanes:
        for i1 in i_values:
            for i2 in i_values:
                if i1 != i2 and jump_ok(i1, i2):
                    add_edge((lane_val, i1), (lane_val, i2))

    # 2. 合流（隣接レーン -> egoレーン）。position の変化量は自由。
    for lane_val in side_lanes:
        for i1 in i_values:
            for i2 in i_values:
                if jump_ok(i1, i2):
                    add_edge((lane_val, i1), (ego_lane, i2))

    # 3. 合流後は ego レーン内でのみ縦方向に移動できる（隣接レーンへは戻れない）
    for i1 in i_values:
        for i2 in i_values:
            if i1 != i2 and jump_ok(i1, i2):
                add_edge((ego_lane, i1), (ego_lane, i2))

    m.set_ntrans(ntrans)
    m.set_init([(car, START_BOX)])

    m.max_step = 0  # membership.check_membership が観測系列の長さに合わせて設定する
    return m, box_id_of


def describe(model: Model, box_id_of: Dict[BoxKey, int]) -> str:
    lines = [f"cars={model.cars}", f"#boxes={len(model.boxes)}", f"#ntrans={len(model.ntrans)}"]
    id_of_box = {v: k for k, v in box_id_of.items()}
    lines.append(f"inits={[id_of_box[n] for (_, n) in model.inits]}")
    return "\n".join(lines)
