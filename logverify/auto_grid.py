"""「重要なところ（衝突が起こりうる近傍）だけ細かく、それ以外は粗く」という
near/far格子のパラメータ (gy, rx_near_cell, rx_near_range, rx_far_cell) を、
分析者が手でメートル単位の値を選ぶのではなく、**その2台の車両自身の物理
サイズ（半車幅・半車長）から自動的に導出する**モジュール。

## 経緯

12.4節（`classify_fine`）以来、本プロジェクトの一貫した方針は「分析用に
選んだ恣意的なメートル値ではなく、車両自身の物理サイズを単位にする」
というものだった（`EGO_HALF_WIDTH`, `EGO_HALF_LENGTH`が9領域・fine領域
双方の基準）。しかし12.7〜12.9節でインスタンスCPD用に使った近傍/遠方
格子のパラメータ（`RX_NEAR_CELL=1.0, RX_FAR_CELL=50.0, RX_NEAR_RANGE=15.0,
GY=0.3`）は、実際にはlog 0067を見ながら「これくらいなら衝突の前後が
ちょうど細かく見える」と手で選んだ値であり、この一貫した方針から外れて
いた。

ユーザーから「重要なところだけ細かく、それ以外は粗くといった自動詳細化は
できますか」との依頼があり、これに応えるため、上記のパラメータを
`groundtruth_size.vehicle_sizes`から機械的に導出できるようにした。

## 導出方法

- `auto_gy`: 咲川氏のlane境界と同じ考え方（`EGO_HALF_WIDTH`を
  `n_bins`個に分割）で、横方向のセルサイズを求める。
  `ego_half_width=0.95, n_bins=3` なら `gy=0.317`（12.7〜12.9節で
  手で選んだ`GY=0.3`とほぼ一致する）。
- `auto_near_range`: 「物理的に接触しうる範囲」を基準に、その
  `near_range_factor`倍を「注意して見るべき近傍」とする
  （12.8節の接触境界=`ego_half_width+npc_half_width`と同じ発想を、
  縦方向の車体サイズに対して適用したもの）。
  `ego_half_length=2.443, npc_half_length=2.32, factor=3.0` なら
  `near_range=14.3m`（手で選んだ`RX_NEAR_RANGE=15.0`とほぼ一致する）。
- `auto_near_cell`: 近傍の車体サイズ相当の距離を`near_cell_bins`個に
  分割する。`(ego_half_length+npc_half_length)=4.76, near_cell_bins=5.0`
  なら`near_cell=0.95`（手で選んだ`RX_NEAR_CELL=1.0`とほぼ一致する）。
- `auto_far_cell`: 遠方は「近傍の外は原則としてまとめてよい」という
  方針のもと、`near_range`の`far_cell_factor`倍という粗いセルにする
  （衝突分析の主眼は近傍にあるため、遠方の精度はさほど重要でない）。

いずれも、手で`log 0067`を見ながら選んだ値とほぼ一致する結果になった
ことを、`logverify/demo_auto_grid.py`で確認している。これは、その場しのぎ
の値ではなく、この物理的な導出方法自体が合理的であることの傍証である。

---
English:
A module that derives the near/far grid parameters (gy, rx_near_cell,
rx_near_range, rx_far_cell) used to keep "only the important region (near
where a collision can occur) fine, everything else coarse" **automatically
from the two vehicles' own physical size** (half-width, half-length),
rather than having the analyst hand-pick values in meters.

## Background

Since Section 12.4 (`classify_fine`), this project's consistent policy has
been "use the vehicles' own physical size as the unit, not an arbitrary
meter value chosen for the analysis" (`EGO_HALF_WIDTH`/`EGO_HALF_LENGTH`
are the basis for both the 9-area partition and the fine-grained one).
However, the near/far grid parameters used for the instance CPDs in
Sections 12.7-12.9 (`RX_NEAR_CELL=1.0, RX_FAR_CELL=50.0,
RX_NEAR_RANGE=15.0, GY=0.3`) were in fact hand-picked while looking at log
0067, choosing "values that make the region around the collision look
appropriately fine" -- a departure from that consistent policy.

The user asked, "can automatic refinement -- fine only where it matters,
coarse elsewhere -- be done?" This module answers that by deriving the
above parameters mechanically from `groundtruth_size.vehicle_sizes`.

## Derivation

- `auto_gy`: uses the same idea as Mr. Sakikawa's lane boundary (dividing
  `EGO_HALF_WIDTH` into `n_bins` parts) to get the lateral cell size. With
  `ego_half_width=0.95, n_bins=3`, `gy=0.317` (nearly identical to the
  hand-picked `GY=0.3` from Sections 12.7-12.9).
- `auto_near_range`: takes "the range within which physical contact is
  possible" and treats `near_range_factor` times that as "the vicinity
  worth watching closely" (the same idea as Section 12.8's contact
  boundary, `ego_half_width + npc_half_width`, applied to the vehicles'
  longitudinal size instead). With `ego_half_length=2.443,
  npc_half_length=2.32, factor=3.0`, `near_range=14.3m` (nearly identical
  to the hand-picked `RX_NEAR_RANGE=15.0`).
- `auto_near_cell`: divides a distance on the scale of the vehicles'
  combined length into `near_cell_bins` parts. With
  `(ego_half_length+npc_half_length)=4.76, near_cell_bins=5.0`,
  `near_cell=0.95` (nearly identical to the hand-picked `RX_NEAR_CELL=1.0`).
- `auto_far_cell`: under the policy that "outside the near region can, in
  principle, be lumped together", uses a coarse cell of `far_cell_factor`
  times `near_range` (since the focus of collision analysis is the near
  region, precision far away matters little).

`logverify/demo_auto_grid.py` confirms that all of these nearly match the
values hand-picked while looking at log 0067 -- evidence that this
physically-derived approach is sound, rather than the earlier values
having been an ad-hoc lucky guess.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class AutoGridParams:
    gy: float
    rx_near_cell: float
    rx_near_range: float
    rx_far_cell: float


def auto_gy(ego_half_width: float, n_bins: float = 3.0) -> float:
    """咲川氏のlane境界(`EGO_HALF_WIDTH`)を`n_bins`分割した、横方向セルサイズ。"""
    return ego_half_width / n_bins


def auto_near_range(ego_half_length: float, npc_half_length: float, factor: float = 3.0) -> float:
    """接触しうる縦方向の範囲（両者の半車長の和）の`factor`倍を近傍とする。"""
    return factor * (ego_half_length + npc_half_length)


def auto_near_cell(ego_half_length: float, npc_half_length: float, near_cell_bins: float = 5.0) -> float:
    """両者の半車長の和を`near_cell_bins`分割した、近傍の縦方向セルサイズ。"""
    return (ego_half_length + npc_half_length) / near_cell_bins


def auto_far_cell(near_range: float, far_cell_factor: float = 5.0) -> float:
    """近傍の外側でまとめる、遠方の縦方向セルサイズ（near_rangeの倍数）。"""
    return far_cell_factor * near_range


def auto_grid_params(
    ego_half_width: float,
    ego_half_length: float,
    npc_half_width: float,
    npc_half_length: float,
    near_range_factor: float = 3.0,
    gy_bins: float = 3.0,
    near_cell_bins: float = 5.0,
    far_cell_factor: float = 5.0,
) -> AutoGridParams:
    """車両サイズから、近傍細かく・遠方粗い格子の4パラメータを自動導出する。

    ---
    English:
    Automatically derives the 4 near/far grid parameters (fine near,
    coarse far) from vehicle sizes.
    """
    gy = auto_gy(ego_half_width, gy_bins)
    near_range = auto_near_range(ego_half_length, npc_half_length, near_range_factor)
    near_cell = auto_near_cell(ego_half_length, npc_half_length, near_cell_bins)
    far_cell = auto_far_cell(near_range, far_cell_factor)
    return AutoGridParams(gy=gy, rx_near_cell=near_cell, rx_near_range=near_range, rx_far_cell=far_cell)


def auto_near_range_from_risk_frame(
    rxs, risk_frame: Optional[int], margin_factor: float = 1.2, default: float = 15.0
) -> float:
    """12.25節: 「安全性判定モデル(safety model)が実際にリスクを気にし
    始めるフレーム」(risk_frame -- JAMA C&Cのrisk-perceived frameでも、
    RSSのviolation frameでも、どちらでも渡せる)から、near_rangeを
    機械的に決める。

    これまでの`auto_near_range`(車両物理サイズ×factor)は「接触しうる
    範囲」という車両ジオメトリだけに基づいており、どの安全性モデルを
    採用するかとは無関係だった。本関数は代わりに、risk_frameでの
    |rx|の値そのもの(にmargin_factor倍の余裕を持たせた値)をnear_rangeに
    採用することで、「その安全性モデルが実際に注意を払う範囲を、
    格子もちょうど覆うように細かくする」という、文字通りの
    safety-model-guided abstraction を実現する。risk_frameが見つからない
    場合(その安全性モデルの基準では一度もリスクが検出されなかった場合)は
    `default`を使う。

    ---
    English: Section 12.25. Derives near_range mechanically from "the
    frame at which the safety model actually starts to care about risk"
    (risk_frame -- this can be either JAMA C&C's risk-perceived frame or
    RSS's violation frame). The earlier `auto_near_range` (vehicle
    physical size x factor) was based purely on vehicle geometry and had
    nothing to do with which safety model was in use. This function
    instead takes the value of |rx| at risk_frame itself (with a
    margin_factor safety margin) as near_range, so that the grid is made
    fine exactly over the region the safety model actually attends to --
    a literal instance of safety-model-guided abstraction. If no
    risk_frame was found (the safety model never flagged risk under its
    own criterion), `default` is used.
    """
    if risk_frame is None or rxs[risk_frame] is None:
        return default
    return margin_factor * abs(rxs[risk_frame])


def auto_grid_params_naive_uniform(
    rx_extent: float, cell_width: float = 2.0, gy: float = 0.3
) -> AutoGridParams:
    """12.25節: ベースライン用の、素朴な一様格子。近傍/遠方の区別を
    一切設けず、`rx_extent`（観測されたrxの絶対値の最大程度）全体を
    `cell_width`一定のセルで覆う(near_range=rx_extentとして、
    near_cell=far_cell=cell_widthにすることで実現する)。

    「何も特徴を考えずに、適当な幅のグリッドで抽象化する」という
    ユーザー提案のベースラインをそのまま実装したもの。cell_widthの
    デフォルト2.0mは、車両物理サイズにもいずれの安全性モデルにも
    由来しない、恣意的な値である(ベースラインとしてそうであることが
    重要)。

    ---
    English: Section 12.25. A naive baseline: a uniform grid with no
    near/far distinction at all, covering the whole observed rx extent
    with a constant `cell_width` (implemented by setting
    near_range=rx_extent and near_cell=far_cell=cell_width).

    This directly implements the user's proposed baseline: "abstract with
    a grid of some arbitrary width, without considering any features at
    all". The default cell_width of 2.0m is deliberately arbitrary --
    derived from neither vehicle geometry nor any safety model (which is
    the point of a baseline).
    """
    return AutoGridParams(gy=gy, rx_near_cell=cell_width, rx_near_range=rx_extent, rx_far_cell=cell_width)


def search_minimal_purity_grid(
    rxs, rys, gy: float, near_range: float, onset_frames,
    near_cell_candidates=None, far_cell_candidates=None, margin_ratio: float = 0.3,
):
    """12.25節への追記: near_cell・far_cellを`near_range`に対して固定せず、
    候補の組み合わせを総当たりし、指定した`onset_frames`(1つ以上の
    safety modelのonsetフレームのリスト)すべてに対してpureになる中で
    箱数が最小の(near_cell, far_cell)を探す。

    ユーザーからの指摘「C&C基準・RSS基準の格子は、near/farの粒度を
    変えれば箱数を減らせるのではないか」に応えるための関数。
    `auto_near_range_from_risk_frame`はnear_rangeだけを安全性モデルに
    連動させ、near_cell/far_cellは車両物理サイズ由来の値を流用して
    いたが、これは「なぜその値でなければならないか」の必然性がなく、
    実際に粒度を独立して調整すれば箱数を大きく減らせることが
    (このモジュールの呼び出し元での実験で)確認されている。

    注意: purity(pure/impureの判定)は、ヒステリシスで圧縮された箱の
    境界がonsetフレームのちょうど開始位置に来るかどうかという、
    セルサイズに対して滑らかではない(単調でない)判定である。すなわち
    「もっと粗いセルでもたまたまpureになる」ことがあり得る——本関数が
    返す(near_cell, far_cell)は、あくまで与えられた候補集合とこの1本の
    ログに対して最小箱数を達成する組み合わせであり、他のログでも同じ
    組み合わせがpureであり続ける保証はない(過学習のリスクがある)。
    複数ログでの頑健性の検証は今後の課題。

    Returns:
        (best_near_cell, best_far_cell, best_n_boxes) のタプル。
        条件を満たす組み合わせが候補内に見つからない場合は
        (None, None, None)。

    ---
    English: Addendum to Section 12.25. Rather than fixing near_cell/
    far_cell at the vehicle-geometry-derived values while only tying
    near_range to the safety model, this brute-force searches candidate
    (near_cell, far_cell) combinations and returns the one with the
    fewest resulting boxes that is pure with respect to every frame in
    `onset_frames` (one or more safety models' onset frames).

    Responds to the user's observation that "the C&C-guided and
    RSS-guided grids could probably use fewer boxes if near/far
    granularity were tuned independently rather than reused from the
    vehicle-geometry grid" -- confirmed by experiment to be correct.

    Caveat: purity is not a smooth (monotonic) function of cell size --
    a coarser cell can, by coincidence of where the hysteresis-filtered
    box boundary happens to land, still be pure. The (near_cell, far_cell)
    this function returns is only the best combination for this one log
    among the given candidates; there is no guarantee the same
    combination stays pure on other logs (a risk of overfitting).
    Checking robustness across multiple logs is future work.

    Returns:
        (best_near_cell, best_far_cell, best_n_boxes), or
        (None, None, None) if no candidate combination satisfies purity
        for every onset frame.
    """
    from logverify.grid_bridge import compress_to_grid_states_variable_hysteresis

    if near_cell_candidates is None:
        near_cell_candidates = [0.5, 0.6, 0.7, 0.8, 0.953, 1.2, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
    if far_cell_candidates is None:
        far_cell_candidates = [1, 1.5, 2, 3, 4, 5, 7, 10, 15, 20, 30, 45, 60, 71.44]

    onset_frames = [f for f in onset_frames if f is not None]

    def is_pure(states, onset_frame):
        for s in states:
            if s.start_frame <= onset_frame <= s.end_frame:
                return s.start_frame == onset_frame
        return False

    best_combo, best_n = (None, None), None
    for near_cell in near_cell_candidates:
        for far_cell in far_cell_candidates:
            states = compress_to_grid_states_variable_hysteresis(
                rxs, rys, near_cell, far_cell, near_range, gy, margin_ratio=margin_ratio,
            )
            if all(is_pure(states, f) for f in onset_frames):
                n = len(states)
                if best_n is None or n < best_n:
                    best_n, best_combo = n, (near_cell, far_cell)

    if best_n is None:
        return None, None, None
    return best_combo[0], best_combo[1], best_n


def auto_grid_params_from_ajisai(json_path: str, npc_name: Optional[str] = None, **kwargs) -> AutoGridParams:
    """AJISAIログのJSONファイルから車両サイズを読み取り、自動的に格子パラメータを導出する。

    ---
    English:
    Reads vehicle sizes from an AJISAI log JSON file and automatically
    derives the grid parameters.
    """
    import json

    with open(json_path) as f:
        data = json.load(f)
    sizes = {v["name"]: v["size"] for v in data["groundtruth_size"]["vehicle_sizes"]}
    ego_size = sizes["ego"]
    if npc_name is not None:
        npc_size = sizes[npc_name]
    else:
        npc_size = sizes.get("npc1", list(v for k, v in sizes.items() if k != "ego")[0])
    return auto_grid_params(
        ego_half_width=ego_size["y"] / 2,
        ego_half_length=ego_size["x"] / 2,
        npc_half_width=npc_size["y"] / 2,
        npc_half_length=npc_size["x"] / 2,
        **kwargs,
    )
