"""
logverify のCPDモデルから列挙したシナリオを、「ワールド座標系」
（絶対座標系）でアニメーション化する。

## 経緯（v1 -> v2）

v1は、モデルが持つ (lane, position) を「Ego基準の相対座標」の格子と
みなして描画する gif_viz.py に対し、「Egoが前進する」という事実を
アニメーション側だけの後付け計算（ego_world_position = step * ego_speed）
として表現していた。しかしこれはCPDモデル自体にはEgoという car が
存在しない、可視化上のトリックに過ぎず、「オリジナルの英語の論文では
CPD自体がEgoの前進をモデル化しているはず」というユーザ指摘の通り、
論文（Fig.2/Fig.7）のCPDはEgoも他の車と同じく1つの car として、
自分の箱列を持つ形でモデル化されている。

v2では、logverify.ego_car.with_ego_track を使ってEgoを実際にCPDの
car として追加したモデルを構築し、gcpd.enum_ss で列挙されるシナリオ
そのものにEgoの箱列を含める。Egoの前進はNPCの遷移と同期遷移(strans)で
結び付けられており（詳細は ego_car.py のdocstring参照）、
「NPCが実際に距離帯・車線を変える瞬間には必ずEgoも1つ前進する」
「NPCが変化しないステップはEgoも変化しない」という形で、CPDが
表現するシナリオの一部として解かれる。

NPC側の position は（方法B・Cとも）Egoとの相対距離をそのまま表して
いるため、1枚の固定カメラの絵として描画する際には、
    NPCの絶対position = Ego(box).position（=これまでの同期前進の回数）
                         + NPCの相対position
として画面上の座標に変換する（この変換自体は見た目のためのもので、
CPDが表現するシナリオ集合の意味＝相対距離ベースを変えるものではない）。
"""

import os
from typing import Dict, List, Optional, Sequence, Tuple

import gcpd
from gcpd import Model
from gcpd_gif import VehicleGif

from logverify.ego_car import with_ego_track
from logverify.membership import reset_solver


HistoryEntry = Tuple[str, int, int, int, int]  # (car, box, lane, position, step)


def strip_pre_scenario_step(hs: Sequence[HistoryEntry]) -> List[HistoryEntry]:
    """ステップ0（NPCがダミー開始箱にいる助走区間であり、Egoにとっても
    まだ本番前の初期位置に相当する）を丸ごと取り除き、ステップ番号を1つ詰める。"""
    return [(c, n, l, p, s - 1) for (c, n, l, p, s) in hs if s != 0]


def render_world_frame_gif(
    model: Model,
    output_prefix: str,
    ego_car: str = "Ego",
    ego_lane: int = 0,
    ego_speed: float = 1.0,
    ego_color: Tuple[int, int, int] = (30, 110, 230),
    npc_colors: Optional[dict] = None,
    combined: bool = True,
    grid_scale: int = 120,
    a_res: int = 8,
    max_step: Optional[int] = None,
    num_model: int = 10_000,
) -> List[str]:
    """model に Ego を car として追加した上で gcpd.enum_ss でシナリオを列挙し、
    Egoが前進するワールド座標系のGIFアニメーションとして書き出す。

    Args:
        model: logverify.reference_models.build_cutin_reference や
            logverify.multi_log_model.build_union_model が返す gcpd.Model
            （NPCを car として持つ、Egoを含まないモデル）。この引数自体は
            変更されない（with_ego_track が新しいモデルを作る）。
        output_prefix: 出力ファイル名の接頭辞（拡張子 .gif は自動で付く）。
        ego_lane: Egoに割り当てるlaneの値（描画上の固定レーン、デフォルト0）。
        ego_speed: Egoの箱番号（=NPCの遷移と同期した前進回数）を画面上の
            格子セル数に変換する際のスケール（デフォルト1＝1回の同期前進=1セル）。
        combined: True なら全シナリオを1本のGIFにまとめる（gen_gif_all）。
            False ならシナリオごとに別ファイルにする。
        max_step: Egoの箱列の長さ、およびモデル全体で列挙するシナリオの
            長さの上限。logverify.reference_models のモデルは max_step=0 の
            まま構築される（membership check時に観測系列の長さに合わせて
            設定される想定のため）ので、可視化する際は明示的に指定する
            必要がある。None の場合は model.max_step をそのまま使う
            （方法Cの統合モデルはログの長さから自動設定済みなので通常は
            省略でよい）。
        num_model: 列挙するシナリオ数の上限（gcpd.enum_ss に渡す）。方法Bの
            参照CPDのように非決定性が大きいモデルでは全列挙が非常に大きく
            なりうるため、可視化用には小さい値（例: 6）に絞るとよい。

    Returns:
        生成されたGIFファイルのパスのリスト。
    """
    aug_model = with_ego_track(model, max_step=max_step, ego_lane=ego_lane, ego_car=ego_car)
    aug_model.num_model = num_model

    reset_solver()
    gcpd.init(aug_model)
    gcpd.add_pos(aug_model)
    gcpd.add_lane(aug_model)
    gcpd.add_init(aug_model)
    gcpd.add_trans(aug_model)
    history = gcpd.enum_ss(aug_model)

    stripped = [strip_pre_scenario_step(hs) for hs in history]
    stripped = [hs for hs in stripped if hs]
    if not stripped:
        raise ValueError("可視化できるシナリオがありません（列挙結果が空です）")

    npc_car = model.cars[0]

    # シナリオごとに、ステップ -> {car: (lane, position)} の対応を作る
    # （Ego, NPC 双方の実際に solve された箱がここに入る）
    scenarios_by_step: List[Dict[int, Dict[str, Tuple[int, int]]]] = []
    for hs in stripped:
        by_step: Dict[int, Dict[str, Tuple[int, int]]] = {}
        for (c, n, l, p, s) in hs:
            by_step.setdefault(s, {})[c] = (l, p)
        scenarios_by_step.append(by_step)

    # NPCのpositionはEgoとの相対距離なので、Ego自身の前進量（実際にCPDが
    # 解として持つEgo(box).position）を足し込んでワールド座標に変換する。
    world_scenarios: List[List[HistoryEntry]] = []
    for by_step in scenarios_by_step:
        max_s = max(by_step.keys())
        combined_hs: List[HistoryEntry] = []
        for s in range(max_s + 1):
            entries = by_step.get(s, {})
            _, ego_pos_v = entries.get(ego_car, (ego_lane, 0))
            ego_world_pos = ego_pos_v * ego_speed
            if npc_car in entries:
                npc_lane_v, npc_pos_rel = entries[npc_car]
                npc_world_pos = ego_world_pos + npc_pos_rel
                combined_hs.append((npc_car, 0, npc_lane_v, round(npc_world_pos), s))
            combined_hs.append((ego_car, 0, ego_lane, round(ego_world_pos), s))
        world_scenarios.append(combined_hs)

    # 全シナリオを通した lane/position の範囲を求め、グリッドサイズを共通化する
    # (gen_gif_all は1本のGIF内でグリッドサイズを変えられないため)
    all_lanes = [l for hs in world_scenarios for (c, n, l, p, s) in hs]
    all_positions = [p for hs in world_scenarios for (c, n, l, p, s) in hs]
    lane_min, lane_max = min(all_lanes), max(all_lanes)
    pos_min, pos_max = min(all_positions), max(all_positions)

    shifted = [
        [(c, n, l - lane_min, p - pos_min, s) for (c, n, l, p, s) in hs] for hs in world_scenarios
    ]

    vg = VehicleGif()
    vg.grid_scale = grid_scale
    vg.a_res = a_res
    vg.grid_x = (lane_max - lane_min) + 1
    vg.grid_y = (pos_max - pos_min) + 1
    vg.y_bup = 0

    colors = dict(npc_colors or {})
    default_palette = [(220, 30, 30), (30, 160, 60), (200, 140, 0), (140, 60, 200)]
    for i, c in enumerate(model.cars):
        colors.setdefault(c, default_palette[i % len(default_palette)])

    vg.car_color = {ego_car: ego_color, **colors}
    vg.x_margin = {ego_car: 15, **{c: 45 for c in model.cars}}
    vg.y_margin = {ego_car: 15, **{c: 60 for c in model.cars}}

    out_dir = os.path.dirname(output_prefix)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    if combined:
        vg.gen_gif_all(shifted, output_prefix)
        return [f"{output_prefix}.gif"]
    else:
        vg.gen_gif(shifted, output_prefix)
        return [f"{output_prefix}-{i}.gif" for i in range(len(shifted))]
