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

## 注意: 方法Bの距離帯（順序尺度）をそのまま座標として足すと衝突に見える

方法B（`reference_models.build_cutin_reference`）のpositionは、
`logverify/zones.py` が定義する BEHIND=-1 / NEAR=0 / MEDIUM=1 / FAR=2
という**順序尺度の距離帯カテゴリ**であり、実際のメートル距離ではない
（10.5節）。このうち NEAR=0 をそのまま「Egoの前進量 + 相対position」の
式に代入すると、Egoのワールド座標にちょうど0を足すことになり、
Egoとまったく同じ格子セルに描画されてしまう。これは「NPCがEgoに
極めて近い（NEAR）」という抽象的な分類が、たまたま0という数値で
表現されているために起きる見た目上のアーティファクトであり、
モデルが「衝突するシナリオ」を表しているわけではない
（衝突可能性の判定自体は9.4節の`ps_col`と組み合わせる別の話であり、
本節の参照CPDにはまだ組み込んでいない。11.5節今後の課題）。

これを避けるため、`zone_ahead_offset`引数で「0以上（NEAR/MEDIUM/FAR）の
距離帯にだけ一律のオフセットを足す」ことができるようにした
（BEHINDはEgoより手前で既に区別がつくため対象外）。方法Cのように
positionが実際の格子（メートル単位）であるモデルでは、0は本当に
「ほぼ同じ位置」を意味しうるため、デフォルト（0＝オフセットなし）の
ままにしておくこと。

---
English:
Animates the scenarios enumerated from logverify's CPD model in the
"world coordinate frame" (absolute coordinate system).

## Background (v1 -> v2)

In v1, gif_viz.py treated the (lane, position) held by the model as a
grid of "coordinates relative to Ego" and drew it that way, while the
fact that "Ego moves forward" was expressed only as an after-the-fact
computation on the animation side (ego_world_position = step *
ego_speed). However, this was merely a visualization trick, since the
CPD model itself has no car called Ego. As the user pointed out, "in
the original English paper, CPD itself must be modeling Ego's forward
motion", and indeed in the paper (Fig.2/Fig.7) the CPD models Ego as
just another car like the others, with its own sequence of boxes.

In v2, logverify.ego_car.with_ego_track is used to build a model in
which Ego is actually added to the CPD as a car, so that the scenarios
enumerated by gcpd.enum_ss themselves include Ego's box sequence.
Ego's forward motion is tied to the NPC's transitions via a
synchronized transition (strans) (see the docstring of ego_car.py for
details), so that "whenever the NPC actually changes its distance
zone/lane, Ego always advances by one" and "when the NPC does not
change, Ego does not change either" — this is solved as part of the
scenario that the CPD represents.

Since the NPC-side position (in both Method B and Method C) directly
represents the relative distance to Ego, when drawing it as a single
fixed-camera picture it is converted into on-screen coordinates as:
    NPC absolute position = Ego(box).position (= the number of
                             synchronized advances so far)
                             + NPC relative position
(this conversion itself is purely for the visuals and does not change
the meaning of the scenario set the CPD represents, i.e. it remains
relative-distance based).

## Note: adding Method B's distance zone (an ordinal scale) directly as a coordinate can look like a collision

The position in Method B (`reference_models.build_cutin_reference`) is
an **ordinal-scale distance-zone category** defined by
`logverify/zones.py` as BEHIND=-1 / NEAR=0 / MEDIUM=1 / FAR=2, not an
actual metric distance (Section 10.5). If NEAR=0 is substituted as-is
into the formula "Ego's forward amount + relative position", it amounts
to adding exactly 0 to Ego's world coordinate, so it gets drawn in
exactly the same grid cell as Ego. This is a visual artifact caused by
the abstract classification "the NPC is extremely close to Ego (NEAR)"
happening to be represented by the number 0 — it does not mean the
model represents "a colliding scenario" (collision-possibility
determination itself is a separate matter combined with `ps_col` in
Section 9.4, and has not yet been incorporated into the reference CPD
in this section — future work, Section 11.5).

To avoid this, the `zone_ahead_offset` argument was added so that "a
uniform offset can be added only to distance zones of 0 or greater
(NEAR/MEDIUM/FAR)" (BEHIND is excluded because it is already
distinguishable as being in front of Ego). For a model like Method C
where position is an actual grid (in meters), 0 can genuinely mean
"almost the same position", so leave it at the default (0 = no offset).
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
    まだ本番前の初期位置に相当する）を丸ごと取り除き、ステップ番号を1つ詰める。

    ---
    English:
    Removes step 0 entirely (the run-up interval during which the NPC is
    in the dummy starting box, which also corresponds to Ego's initial
    pre-scenario position), and shifts the step numbers down by one."""
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
    zone_ahead_offset: int = 0,
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
        zone_ahead_offset: NPCの相対position（0以上、＝Ego手前ではない側）
            に一律で足すオフセット。方法Bの距離帯（NEAR=0など）をそのまま
            Egoの絶対座標に足すと、NEARがちょうどEgoと同じ格子セルに
            描画され、衝突しているように見えてしまう（本ファイル冒頭の
            注意を参照）。方法Bを描画する場合は1以上を指定するとよい。
            方法C（positionが実際の格子＝メートル単位）ではデフォルトの
            0のままにしておくこと。

    Returns:
        生成されたGIFファイルのパスのリスト。

    ---
    English:
    Adds Ego to model as a car, enumerates scenarios with gcpd.enum_ss,
    and writes them out as a GIF animation in the world coordinate frame
    in which Ego advances.

    Args:
        model: A gcpd.Model as returned by
            logverify.reference_models.build_cutin_reference or
            logverify.multi_log_model.build_union_model (a model that
            holds the NPC as a car, without Ego). This argument itself
            is not modified (with_ego_track builds a new model).
        output_prefix: Prefix for the output file name (the .gif
            extension is added automatically).
        ego_lane: The lane value assigned to Ego (a fixed lane for
            drawing purposes; default 0).
        ego_speed: The scale used to convert Ego's box number (= the
            number of synchronized advances made together with the NPC's
            transitions) into a number of on-screen grid cells (default
            1 = one synchronized advance = one cell).
        combined: If True, all scenarios are combined into a single GIF
            (gen_gif_all). If False, a separate file is produced per
            scenario.
        max_step: The upper bound on the length of Ego's box sequence,
            and on the length of the scenarios enumerated for the whole
            model. Models from logverify.reference_models are built with
            max_step=0 left as-is (on the assumption that it will be set
            to match the length of the observed sequence at membership-
            check time), so it must be given explicitly when
            visualizing. If None, model.max_step is used as-is (the
            Method C union model is normally already set automatically
            from the log length, so it can usually be omitted).
        num_model: The upper bound on the number of scenarios enumerated
            (passed to gcpd.enum_ss). For a model with a lot of
            non-determinism, such as Method B's reference CPD, full
            enumeration can become very large, so it is a good idea to
            restrict this to a small value (e.g. 6) for visualization.
        zone_ahead_offset: A uniform offset added to the NPC's relative
            position when it is 0 or greater (i.e. not on the side
            behind Ego). Adding Method B's distance zone (e.g. NEAR=0)
            directly to Ego's absolute coordinate causes NEAR to be
            drawn in exactly the same grid cell as Ego, making it look
            like a collision (see the note at the top of this file). It
            is a good idea to specify 1 or more when drawing Method B.
            For Method C (where position is an actual grid, i.e. in
            meters), leave it at the default of 0.

    Returns:
        A list of paths to the generated GIF files.
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
    # (English) For each scenario, build a mapping from step -> {car:
    # (lane, position)} (this holds the actually-solved boxes for both
    # Ego and the NPC).
    scenarios_by_step: List[Dict[int, Dict[str, Tuple[int, int]]]] = []
    for hs in stripped:
        by_step: Dict[int, Dict[str, Tuple[int, int]]] = {}
        for (c, n, l, p, s) in hs:
            by_step.setdefault(s, {})[c] = (l, p)
        scenarios_by_step.append(by_step)

    # NPCのpositionはEgoとの相対距離なので、Ego自身の前進量（実際にCPDが
    # 解として持つEgo(box).position）を足し込んでワールド座標に変換する。
    #
    # 車線変更を伴う遷移は、そのまま(旧レーン,旧position)から(新レーン,新
    # position)へ直線で補間すると、斜めに突っ切る経路がEgoの位置をかすめ、
    # 衝突しているように見えてしまうことがある（ユーザ指摘、11.5節参照）。
    # これを避けるため、車線が変わる遷移では「まず現在のレーンのまま
    # 縦位置(position)だけ動き、その後にレーンだけを切り替える」という
    # 2段階の中間フレームを挿入する（実際の車線変更＝縦に詰めてから
    # 横に合流する、という一般的な運転動作にも近い）。これはCPDが検証
    # した状態そのものは変えず、あくまで描画上の補間経路を分割するだけ
    # である点に注意。
    #
    # (English) The NPC's position is a distance relative to Ego, so when
    # converting to world coordinates for the fixed-camera drawing, we add
    # Ego's own amount of forward advance (Ego(box).position, the value
    # actually held in the CPD's solution).
    #
    # If a transition that involves a lane change is interpolated as a
    # straight line directly from (old lane, old position) to (new lane,
    # new position), the diagonal path can graze Ego's position and make
    # it look like a collision (per the user's observation; see Section
    # 11.5). To avoid this, for transitions where the lane changes we
    # insert a two-stage set of intermediate frames: "first move only the
    # vertical position while staying in the current lane, then switch
    # only the lane" (this is also close to how an ordinary driving
    # maneuver actually performs a lane change: close the gap
    # longitudinally, then merge laterally). Note that this does not
    # change the state actually verified by the CPD itself — it only
    # splits the interpolated path used for drawing.
    world_scenarios: List[List[HistoryEntry]] = []
    for by_step in scenarios_by_step:
        max_s = max(by_step.keys())
        combined_hs: List[HistoryEntry] = []
        step_idx = 0
        prev_npc_lane: Optional[int] = None
        for s in range(max_s + 1):
            entries = by_step.get(s, {})
            _, ego_pos_v = entries.get(ego_car, (ego_lane, 0))
            ego_world_pos = ego_pos_v * ego_speed

            if npc_car in entries:
                npc_lane_v, npc_pos_rel = entries[npc_car]
                if npc_pos_rel >= 0:
                    npc_pos_rel = npc_pos_rel + zone_ahead_offset
                npc_world_pos = ego_world_pos + npc_pos_rel

                if prev_npc_lane is not None and prev_npc_lane != npc_lane_v:
                    # 中間フレーム: 旧レーンのまま新positionへ
                    combined_hs.append(
                        (npc_car, 0, prev_npc_lane, round(npc_world_pos), step_idx)
                    )
                    combined_hs.append((ego_car, 0, ego_lane, round(ego_world_pos), step_idx))
                    step_idx += 1

                combined_hs.append((npc_car, 0, npc_lane_v, round(npc_world_pos), step_idx))
                prev_npc_lane = npc_lane_v
            combined_hs.append((ego_car, 0, ego_lane, round(ego_world_pos), step_idx))
            step_idx += 1
        world_scenarios.append(combined_hs)

    # 全シナリオを通した lane/position の範囲を求め、グリッドサイズを共通化する
    # (gen_gif_all は1本のGIF内でグリッドサイズを変えられないため)
    # (English) Find the range of lane/position across all scenarios and
    # unify the grid size (because gen_gif_all cannot change the grid
    # size within a single GIF).
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
    # car_width/car_height（デフォルトはそれぞれ100/50）を、marginと合わせても
    # grid_scale（=1セルの幅）を超えないよう縮小する。これを怠ると、
    # 隣接レーンの車同士がセルの境界をまたいで実際にはレーンが違うのに
    # 矩形が視覚的に重なってしまう（衝突しているように誤解される原因の
    # 1つだった。11.5節参照）。
    # (English) Shrink car_width/car_height (default 100/50 respectively)
    # so that, even together with the margin, they do not exceed
    # grid_scale (the width of one cell). Failing to do this causes the
    # rectangles of cars in adjacent lanes to visually overlap across the
    # cell boundary even though they are actually in different lanes
    # (this was one cause of scenarios being mistaken for collisions; see
    # Section 11.5).
    vg.car_width = min(60, grid_scale - 20)
    vg.car_height = min(40, grid_scale - 20)

    colors = dict(npc_colors or {})
    default_palette = [(220, 30, 30), (30, 160, 60), (200, 140, 0), (140, 60, 200)]
    for i, c in enumerate(model.cars):
        colors.setdefault(c, default_palette[i % len(default_palette)])

    vg.car_color = {ego_car: ego_color, **colors}
    # x/y margin はセル内でEgoとNPCの矩形をずらして見やすくするためのもの。
    # margin + car_width/car_height が grid_scale を超えないようにする
    # （超えると上記と同じ理由で隣接レーンの車と誤って重なって見える）。
    # (English) x/y margin are used to offset the Ego and NPC rectangles
    # within a cell so they are easier to see. Make sure margin +
    # car_width/car_height does not exceed grid_scale (exceeding it
    # causes cars in adjacent lanes to appear to incorrectly overlap, for
    # the same reason as above).
    vg.x_margin = {ego_car: 10, **{c: grid_scale - vg.car_width - 10 for c in model.cars}}
    vg.y_margin = {ego_car: 10, **{c: grid_scale - vg.car_height - 10 for c in model.cars}}

    out_dir = os.path.dirname(output_prefix)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    if combined:
        vg.gen_gif_all(shifted, output_prefix)
        return [f"{output_prefix}.gif"]
    else:
        vg.gen_gif(shifted, output_prefix)
        return [f"{output_prefix}-{i}.gif" for i in range(len(shifted))]
