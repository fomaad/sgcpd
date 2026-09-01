"""
logverify のCPDモデルから列挙したシナリオを、「ワールド座標系」
（絶対座標系）でアニメーション化する。

gif_viz.py との違い:
    gif_viz.render_scenarios_gif は、モデルが持つ (lane, position) を
    そのまま「Ego基準の相対座標」の格子とみなして描画するため、
    Ego を lane=0, position=0 に静止した参照点として描いていた。
    しかし実際にはEgoも道路上を前進しており、position が変化しないのは
    「NPCとの相対距離」を状態として使っているからに過ぎない。

    このモジュールでは、
        1. Ego自身が一定速度で前進する（ego_world_position(s) = s * ego_speed）
        2. NPCの絶対位置 = Egoの絶対位置（前進分） + モデルが持つ相対オフセット
    として、固定カメラ（ワールド座標系に固定、Egoを追従しない）のまま、
    Egoが前進し、NPCがEgoに対して相対的に動く様子を1つのアニメーションで表す。

    lane（横方向）は元々Ego基準の相対値（-1, 0, 1 など）なので、そのまま
    「Egoからの左右オフセット」として使う（Ego自身は lane=0 に固定）。
    position（縦方向）だけ、Egoの前進量を足し込んで絶対座標に変換する。
"""

import os
from typing import Dict, List, Optional, Sequence, Tuple

import gcpd
from gcpd import Model
from gcpd_gif import VehicleGif

from logverify.membership import reset_solver


HistoryEntry = Tuple[str, int, int, int, int]  # (car, box, lane, position, step)


def strip_start_box(hs: Sequence[HistoryEntry], start_box: int = -1) -> List[HistoryEntry]:
    """ダミー開始箱（step 0 にのみ出現する）を取り除き、ステップ番号を1つ詰める。"""
    return [(c, n, l, p, s - 1) for (c, n, l, p, s) in hs if n != start_box]


def render_world_frame_gif(
    model: Model,
    output_prefix: str,
    start_box: int = -1,
    ego_car: str = "Ego",
    ego_color: Tuple[int, int, int] = (30, 110, 230),
    npc_colors: Optional[dict] = None,
    combined: bool = True,
    ego_speed: float = 1.0,
    grid_scale: int = 120,
    a_res: int = 8,
    max_step: Optional[int] = None,
    num_model: int = 10_000,
) -> List[str]:
    """model から gcpd.enum_ss でシナリオを列挙し、Egoが前進するワールド座標系の
    GIFアニメーションとして書き出す。

    Args:
        model: logverify.reference_models.build_cutin_reference や
            logverify.multi_log_model.build_union_model が返す gcpd.Model。
        output_prefix: 出力ファイル名の接頭辞（拡張子 .gif は自動で付く）。
        ego_speed: Egoが1ステップで前進する量（モデルのposition/gridと同じ単位の
            格子セル数）。相対距離側の変化量と同程度になるよう、デフォルトは1。
        combined: True なら全シナリオを1本のGIFにまとめる（gen_gif_all）。
            False ならシナリオごとに別ファイルにする。
        max_step: 列挙するシナリオの長さの上限。logverify.reference_models の
            モデルは max_step=0 のまま構築される（membership check時に観測系列の
            長さに合わせて設定される想定のため）ので、可視化する際は明示的に
            指定する必要がある。None の場合は model.max_step をそのまま使う
            （方法Cの統合モデルはログの長さから自動設定済みなので通常は省略でよい）。
        num_model: 列挙するシナリオ数の上限（gcpd.enum_ss に渡す）。方法Bの
            参照CPDのように非決定性が大きいモデルでは全列挙が非常に大きく
            なりうるため、可視化用には小さい値（例: 6）に絞るとよい。

    Returns:
        生成されたGIFファイルのパスのリスト。
    """
    if max_step is not None:
        model.max_step = max_step
    reset_solver()
    gcpd.init(model)
    gcpd.add_pos(model)
    gcpd.add_lane(model)
    gcpd.add_init(model)
    gcpd.add_trans(model)
    model.num_model = num_model
    history = gcpd.enum_ss(model)

    stripped = [strip_start_box(hs, start_box) for hs in history]
    stripped = [hs for hs in stripped if hs]
    if not stripped:
        raise ValueError("可視化できるシナリオがありません（列挙結果が空です）")

    # シナリオごとに、ステップ -> (lane, position) の対応を作る
    scenarios_by_step: List[Dict[int, Tuple[int, int]]] = []
    for hs in stripped:
        by_step: Dict[int, Tuple[int, int]] = {}
        for (c, n, l, p, s) in hs:
            by_step[s] = (l, p)
        scenarios_by_step.append(by_step)

    # 各シナリオについて、Egoの絶対前進位置を足し込んでワールド座標を計算する
    world_scenarios: List[List[HistoryEntry]] = []
    for by_step in scenarios_by_step:
        max_s = max(by_step.keys())
        combined_hs: List[HistoryEntry] = []
        last_lp = None
        for s in range(max_s + 1):
            ego_pos = s * ego_speed
            lane_rel, pos_rel = by_step.get(s, last_lp)
            last_lp = (lane_rel, pos_rel)
            npc_world_pos = ego_pos + pos_rel
            combined_hs.append((model.cars[0], 0, lane_rel, round(npc_world_pos), s))
            combined_hs.append((ego_car, 0, 0, round(ego_pos), s))
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
