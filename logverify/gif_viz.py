"""
gcpd_gif.py（既存のGIFアニメーション生成器）を使って、logverify/ で構築した
CPDモデル（方法B・方法Cのいずれでも可）から列挙したシナリオを可視化する。

gcpd_gif.VehicleGif は gcpd.enum_ss が返す history（[(car,box,lane,pos,step), ...]
のリストのリスト）を受け取り、(lane, position) の格子上を車がステップごとに
移動するアニメーションを作る（sample2.py が既存の使用例）。

このモジュールが埋める2つのギャップ:
  1. logverify のモデルは、実座標を持たないダミーの開始箱 (START_BOX, box番号-1)
     を使っている（複数の初期候補からの非決定的な出発を表現するため）。
     この箱には位置・車線の値が無いため、そのままgcpd_gifに渡すと壊れる。
     -> strip_start_box() で除去し、ステップ番号を1つ詰める。
  2. logverify のモデルはNPC（複数ログの場合はNPCという1台の車）の
     lane/position しか持たず、ego車線自体は登場しない
     （ego基準の相対座標だから）。可視化の見やすさのため、
     lane=0, position=0 に静止した "Ego" を参照点として追加する。

---
English:
Uses gcpd_gif.py (the existing GIF animation generator) to visualize scenarios
enumerated from a CPD model (either Method B or Method C) built in logverify/.

gcpd_gif.VehicleGif takes the history returned by gcpd.enum_ss (a list of lists
of [(car, box, lane, pos, step), ...]) and produces an animation where cars move
step by step over a (lane, position) grid (sample2.py is an existing usage
example).

Two gaps this module fills:
  1. logverify's models use a dummy start box (START_BOX, box number -1) that
     has no real coordinates (to represent a non-deterministic start from
     multiple initial candidates). This box has no position/lane values, so
     passing it straight to gcpd_gif would break.
     -> Removed with strip_start_box(), which also collapses the step numbers
     by one.
  2. logverify's models only carry lane/position for the NPC (in the
     multi-log case, a single car named NPC); the ego lane itself never
     appears (because coordinates are ego-relative). For clarity of
     visualization, a stationary "Ego" reference point is added at
     lane=0, position=0.
"""

import os
from typing import List, Optional, Sequence, Tuple

import gcpd
from gcpd import Model
from gcpd_gif import VehicleGif

from logverify.membership import reset_solver


HistoryEntry = Tuple[str, int, int, int, int]  # (car, box, lane, position, step)


def strip_start_box(hs: Sequence[HistoryEntry], start_box: int = -1) -> List[HistoryEntry]:
    """ダミー開始箱（step 0 にのみ出現する）を取り除き、ステップ番号を1つ詰める。

    ---
    English:
    Removes the dummy start box (which appears only at step 0) and collapses
    the step numbers by one.
    """
    return [(c, n, l, p, s - 1) for (c, n, l, p, s) in hs if n != start_box]


def render_scenarios_gif(
    model: Model,
    output_prefix: str,
    start_box: int = -1,
    ego_car: str = "Ego",
    ego_color: Tuple[int, int, int] = (30, 110, 230),
    npc_colors: Optional[dict] = None,
    combined: bool = True,
    grid_scale: int = 120,
    a_res: int = 8,
    num_model: int = 10_000,
) -> List[str]:
    """model から gcpd.enum_ss でシナリオを列挙し、GIFアニメーションとして書き出す。

    Args:
        model: logverify.reference_models.build_cutin_reference や
            logverify.multi_log_model.build_union_model が返す gcpd.Model。
        output_prefix: 出力ファイル名の接頭辞（拡張子 .gif は自動で付く）。
        combined: True なら全シナリオを1本のGIFにまとめる（gen_gif_all）。
            False ならシナリオごとに別ファイルにする（gen_gif、
            "{output_prefix}-{番号}.gif" が生成される）。

    Returns:
        生成されたGIFファイルのパスのリスト。

    ---
    English:
    Enumerates scenarios from `model` via gcpd.enum_ss and writes them out as
    GIF animations.

    Args:
        model: a gcpd.Model, such as one returned by
            logverify.reference_models.build_cutin_reference or
            logverify.multi_log_model.build_union_model.
        output_prefix: prefix for the output file name (the .gif extension is
            appended automatically).
        combined: if True, all scenarios are combined into a single GIF
            (gen_gif_all). If False, a separate file is produced per scenario
            (gen_gif, generating "{output_prefix}-{index}.gif").

    Returns:
        A list of paths to the generated GIF files.
    """
    reset_solver()
    gcpd.init(model)
    gcpd.add_pos(model)
    gcpd.add_lane(model)
    gcpd.add_init(model)
    gcpd.add_trans(model)
    model.num_model = num_model  # デフォルトはenum_ssが打ち切らないよう十分大きい値。
    # 大規模モデルで可視化に必要な分だけに絞りたい場合は小さい値を指定する。
    # (English) The default is large enough that enum_ss does not truncate
    # the enumeration. Pass a smaller value to limit enumeration to only
    # what is needed for visualization on a large model.
    history = gcpd.enum_ss(model)

    stripped = [strip_start_box(hs, start_box) for hs in history]
    stripped = [hs for hs in stripped if hs]  # 空（＝ダミー箱だけだった）シナリオは除外
    if not stripped:
        raise ValueError("可視化できるシナリオがありません（列挙結果が空です）")

    lanes = [l for hs in stripped for (_, _, l, _, _) in hs]
    positions = [p for hs in stripped for (_, _, _, p, _) in hs]
    lane_min, lane_max = min(lanes + [0]), max(lanes + [0])
    pos_min, pos_max = min(positions + [0]), max(positions + [0])
    max_step = max(s for hs in stripped for (_, _, _, _, s) in hs)

    # ego を lane=0, position=0 に固定した参照点として全ステップに追加する。
    # gcpd_gif.make_scenario はエントリがステップ順にグループ化されている
    # （同じステップの全車両分のタプルがまとまって並んでいる）ことを前提に
    # しているため、単純に末尾へ追記するのではなく、ステップごとにEgoの
    # エントリを挟み込む。
    #
    # (English)
    # Add Ego as a reference point fixed at lane=0, position=0 to every step.
    # gcpd_gif.make_scenario assumes entries are grouped in step order (all
    # cars' tuples for the same step appear together), so instead of simply
    # appending Ego's entries at the end, we interleave an Ego entry into each
    # step.
    with_ego = []
    for hs in stripped:
        by_step: dict = {}
        for entry in hs:
            by_step.setdefault(entry[4], []).append(entry)
        combined_hs = []
        for s in range(max_step + 1):
            combined_hs.extend(by_step.get(s, []))
            combined_hs.append((ego_car, 0, 0, 0, s))
        with_ego.append(combined_hs)

    # gcpd_gif は非負の格子座標を前提にしているため、lane/positionをシフトする
    # (English) gcpd_gif assumes non-negative grid coordinates, so shift lane/position.
    shifted = [
        [(c, n, l - lane_min, p - pos_min, s) for (c, n, l, p, s) in hs] for hs in with_ego
    ]

    vg = VehicleGif()
    vg.grid_scale = grid_scale
    vg.a_res = a_res
    vg.grid_x = (lane_max - lane_min) + 1
    vg.grid_y = (pos_max - pos_min) + 1
    vg.y_bup = 0  # lane/positionは既に非負へシフト済みなので追加のシフトは不要

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
