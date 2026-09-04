"""
グリッドベースの抽象化アダプタ。

Autoware/AJISAIログ（あるいは任意の ego/NPC の時系列座標）を、
ego中心・進行方向基準の相対座標 (rx, ry) から、指定したセルサイズ
(gx, gy) の格子に離散化し、CPD の (position=i, lane=k) にそのまま
対応する整数列を作る。

咲川氏の名前付き領域抽象化 (vendor/trajectory_abstraction/src/abstraction_15area.py 等)
とは異なり、ここでは「参照CPDを書くときに使った粒度」をそのまま使う。
つまり、gx/gy は「1レーンの幅」「1箱に相当する縦方向の距離」を
CPDモデルの設計者が自分で選ぶためのパラメータである。

咲川氏のvendor/trajectory_abstraction配下のコード（座標の正規化や
JSON読み込みを含む）には一切依存していない（logverify全体の方針として、
方法B・方法Cはvendorのコードを使わず独立に実装している。vendorを使う
のは方法A＝vendor/trajectory_abstraction/src/cpd_bridge.pyのみ）。
実データ（AJISAIログJSON）からego基準の相対座標 (rx, ry) を取り出す
部分は、必要になった時点で本モジュールの中に独立に実装する
（`grid_states_from_relative_xy` はすでに相対座標が分かっている場合の
入口。JSON読み込み自体はまだ用意していない）。

---
English:
Grid-based abstraction adapter.

Takes Autoware/AJISAI logs (or the time-series coordinates of any
ego/NPC) expressed as ego-centered, heading-relative coordinates
(rx, ry), discretizes them onto a grid with the given cell size
(gx, gy), and produces an integer sequence that maps directly onto
the CPD's (position=i, lane=k).

Unlike Sakikawa's named-area abstraction
(vendor/trajectory_abstraction/src/abstraction_15area.py, etc.), this
module reuses the exact granularity that was used when writing the
reference CPD. That is, gx/gy are parameters that let the CPD model's
designer choose, on their own, "the width of one lane" and "the
longitudinal distance corresponding to one box".

This module has no dependency whatsoever on the code under Sakikawa's
vendor/trajectory_abstraction (including coordinate normalization and
JSON loading) — as a matter of policy for logverify as a whole, Method
B and Method C are implemented independently, without using the
vendor's code. Only Method A (vendor/trajectory_abstraction/src/cpd_bridge.py)
uses the vendor. The part that extracts ego-relative coordinates
(rx, ry) from real data (AJISAI log JSON) will be implemented
independently inside this module once it is actually needed
(`grid_states_from_relative_xy` is the entry point for when the
relative coordinates are already known; JSON loading itself has not
been implemented yet).
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class GridState:
    """圧縮後の1状態（イベント駆動: 連続して同じ格子セルに留まる区間を1つにまとめたもの）

    ---
    English:
    One state after compression (event-driven: a run of consecutive
    frames that stay in the same grid cell is collapsed into one).
    """
    index: int          # 0始まりの状態番号（CPDのbox番号にそのまま使う） (English) 0-based state number (used directly as the CPD box number)
    i: int               # 縦方向グリッド index（position） (English) longitudinal grid index (position)
    k: int               # 横方向グリッド index（lane） (English) lateral grid index (lane)
    start_frame: int      # この状態に対応する元データの開始フレーム (English) start frame of the source data for this state
    end_frame: int         # この状態に対応する元データの終了フレーム（inclusive） (English) end frame of the source data for this state (inclusive)


def grid_index_centered(value: float, cell_size: float) -> int:
    """0を中心にした対称な格子インデックスを返す (round-half-to-even ではなく四捨五入寄り)。

    floor() ベースだと k=0 のセルが [0, gy) のように非対称になり、
    「lane=0 は自車線」という直感（自車線中心 ry=0 の前後 ±gy/2）とずれる。
    ここでは round() を使い、k=0 が [-gy/2, +gy/2) になるようにする。

    ---
    English:
    Returns a grid index that is symmetric around 0 (rounds to the
    nearest integer, not round-half-to-even).

    With a floor()-based scheme, the k=0 cell would be asymmetric,
    e.g. [0, gy), which conflicts with the intuition that "lane=0 is
    the ego lane" (centered on ry=0, ±gy/2 on each side). Here round()
    is used instead, so that k=0 becomes [-gy/2, +gy/2).
    """
    return int(np.floor(value / cell_size + 0.5))


def to_grid_indices(
    rx: Optional[float], ry: Optional[float], gx: float, gy: float
) -> Tuple[Optional[int], Optional[int]]:
    if rx is None or ry is None or np.isnan(rx) or np.isnan(ry):
        return None, None
    return grid_index_centered(rx, gx), grid_index_centered(ry, gy)


def grid_index_variable(value: float, near_cell: float, far_cell: float, near_range: float) -> int:
    """Egoからの距離に応じて分解能を変える、非一様な格子インデックス。

    「Egoに近い部分は今まで通り細かく区別し、遠い部分はまとめてしまってよい」
    という考え方（11.6節の課題への対応）を実装したもの。

    - |value| <= near_range の範囲は、near_cell を使ったこれまで通りの
      grid_index_centered をそのまま使う（Ego近傍の区別能力は変えない）。
    - |value| > near_range の部分は、near_range の境界インデックスを基準に、
      far_cell（near_cell より大きい値を渡す想定）で追加のインデックスを足す。
      far_cell が大きいほど、遠方の複数のセルが同じインデックスにまとめられる。

    value・near_cell・far_cell・near_range の単位は呼び出し側で揃えること
    （例: メートル）。返り値は value について単調非減少（symmetricなので
    絶対値が大きいほど原点から離れたインデックスになる）。

    ---
    English:
    A non-uniform grid index whose resolution varies with distance
    from Ego.

    Implements the idea that "the region near Ego should keep being
    distinguished finely, as before, while the far region may be
    lumped together" (addressing the issue raised in section 11.6).

    - For |value| <= near_range, the same grid_index_centered as
      before is used with near_cell (the ability to distinguish
      positions near Ego is unchanged).
    - For |value| > near_range, additional index steps are added on
      top of the boundary index at near_range, using far_cell (a
      value larger than near_cell is expected). The larger far_cell
      is, the more far-away cells get merged into the same index.

    The caller must keep value, near_cell, far_cell, and near_range in
    consistent units (e.g. meters). The return value is monotonically
    non-decreasing in value (since it is symmetric, a larger absolute
    value means an index further from the origin).
    """
    if abs(value) <= near_range:
        return grid_index_centered(value, near_cell)
    sign = 1 if value > 0 else -1
    boundary_idx = grid_index_centered(sign * near_range, near_cell)
    remainder = value - sign * near_range
    return boundary_idx + sign * grid_index_centered(abs(remainder), far_cell)


def grid_index_variable_center(idx: int, near_cell: float, far_cell: float, near_range: float) -> float:
    """grid_index_variable の近似逆写像。与えられた格子インデックスに
    対応するセルの代表値（中心付近の値）を、可視化用に返す。

    12.22節: gcpd.Model は箱を (lane, position) という離散インデックスの
    組でしか持たない（実際の連続値rx/ryは、モデル構築の入力にしか使われず
    モデル自身には残らない）。「CPDモデルの図もEGO/NPCの相対位置が
    わかるように」というユーザーの依頼に応えるには、各箱のインデックスから
    「その箱が表す近似的な実座標」を逆算する必要がある——本関数はその
    ための、可視化専用の近似逆写像である（格子内での正確な位置は失われて
    いるため、あくまで代表値であることに注意）。

    ---
    English:
    Approximate inverse of grid_index_variable. Returns a representative
    (near-center) value for the cell corresponding to the given grid
    index, for visualization purposes.

    Section 12.22: a gcpd.Model only holds boxes as discrete (lane,
    position) index pairs -- the actual continuous rx/ry values are used
    only as input to building the model and are not retained by the
    model itself. To satisfy the user's request that "the CPD model
    diagram should also show the EGO/NPC relative position," each box's
    index must be inverted back into an approximate real-world
    coordinate -- this function does that, for visualization only (the
    exact position within the cell is lost, so this is only a
    representative value).
    """
    boundary_idx = grid_index_centered(near_range, near_cell)
    if abs(idx) <= boundary_idx:
        return idx * near_cell
    sign = 1 if idx > 0 else -1
    remainder_idx = abs(idx) - boundary_idx
    return sign * (near_range + remainder_idx * far_cell)


def to_grid_indices_variable(
    rx: Optional[float],
    ry: Optional[float],
    rx_near_cell: float,
    rx_far_cell: float,
    rx_near_range: float,
    gy: float,
) -> Tuple[Optional[int], Optional[int]]:
    """to_grid_indices の非一様版。rx（縦方向=Egoからの距離）だけを
    grid_index_variable で量子化し、ry（横方向=レーン）は従来通り
    一様な grid_index_centered を使う（レーン数はもともと少なく、
    遠方でまとめる恩恵が小さいため）。

    ---
    English:
    Non-uniform version of to_grid_indices. Only rx (longitudinal =
    distance from Ego) is quantized with grid_index_variable; ry
    (lateral = lane) still uses the uniform grid_index_centered as
    before (the number of lanes is already small, so there is little
    benefit to merging far-away lanes).
    """
    if rx is None or ry is None or np.isnan(rx) or np.isnan(ry):
        return None, None
    return (
        grid_index_variable(rx, rx_near_cell, rx_far_cell, rx_near_range),
        grid_index_centered(ry, gy),
    )


def compress_to_grid_states(
    rxs: Sequence[Optional[float]],
    rys: Sequence[Optional[float]],
    gx: float,
    gy: float,
) -> List[GridState]:
    """(rx, ry) の時系列を格子に離散化し、イベント駆動で圧縮する。

    「連続して同じ (i, k) に留まっている区間」を1つの状態にまとめる。
    これは docs/log_to_cpd_verification_design.md 4.3節「イベント駆動（第一選択）」の実装。

    ---
    English:
    Discretizes the (rx, ry) time series onto the grid and compresses
    it event-driven-style.

    A run of consecutive frames that stay in the same (i, k) is
    collapsed into a single state. This implements section 4.3,
    "Event-driven (first choice)", of
    docs/log_to_cpd_verification_design.md.
    """
    states: List[GridState] = []
    prev_ik: Optional[Tuple[int, int]] = None

    for frame, (rx, ry) in enumerate(zip(rxs, rys)):
        i, k = to_grid_indices(rx, ry, gx, gy)
        if i is None:
            continue
        if prev_ik is not None and (i, k) == prev_ik:
            states[-1].end_frame = frame
            continue
        states.append(GridState(index=len(states), i=i, k=k, start_frame=frame, end_frame=frame))
        prev_ik = (i, k)

    return states


def compress_to_grid_states_variable(
    rxs: Sequence[Optional[float]],
    rys: Sequence[Optional[float]],
    rx_near_cell: float,
    rx_far_cell: float,
    rx_near_range: float,
    gy: float,
) -> List[GridState]:
    """compress_to_grid_states の非一様版（rxにgrid_index_variableを使う）。

    ---
    English:
    Non-uniform version of compress_to_grid_states (uses
    grid_index_variable for rx).
    """
    states: List[GridState] = []
    prev_ik: Optional[Tuple[int, int]] = None

    for frame, (rx, ry) in enumerate(zip(rxs, rys)):
        i, k = to_grid_indices_variable(rx, ry, rx_near_cell, rx_far_cell, rx_near_range, gy)
        if i is None:
            continue
        if prev_ik is not None and (i, k) == prev_ik:
            states[-1].end_frame = frame
            continue
        states.append(GridState(index=len(states), i=i, k=k, start_frame=frame, end_frame=frame))
        prev_ik = (i, k)

    return states


def hysteresis_filter_indices(
    values: Sequence[float], idx_fn, margin: float
) -> List[int]:
    """`idx_fn`（値について単調非減少な、階段状の格子インデックス関数。
    `grid_index_centered`や`grid_index_variable`がこれにあたる）を、
    境界をまたいだ瞬間にすぐ切り替えるのではなく、**境界を`margin`だけ
    超えて初めて切り替える**、ヒステリシス（シュミットトリガー）付きで
    適用する。

    これは、12.9節で見つかった「格子が細かいと、測定ノイズやわずかな
    揺れ戻りだけで格子セルの境界をまたぎ、それが本物の分岐点として
    モデルに現れてしまう」という問題への対処である。境界ちょうどに
    値がとどまり続けると、ヒステリシスなしの量子化ではノイズによって
    セルをまたぐ→戻る→またぐ…を繰り返すが、本関数は「今のセルから
    margin分だけ余分に踏み出さない限り、セルを移らない」という制約を
    課すことで、この種の chattering（びびり）を吸収する。

    実装は、直近に「今のインデックスで安定していた」値
    （`last_stable_value`）と新しい値の間で、`idx_fn`が実際に切り替わる
    境界点を二分探索で求め、新しい値がその境界からさらに`margin`だけ
    先に進んでいる場合にのみインデックスの変更を確定する
    （`idx_fn`の具体的な形（一様格子か、near/far可変格子か）に依存しない
    汎用的な実装）。

    Args:
        values: 元の連続値の時系列（例: rx または ry）。
        idx_fn: 値について単調非減少な格子インデックス関数
            （例: `lambda v: grid_index_centered(v, gy)`）。
        margin: 境界を実際に越えたと判定するための、余分に必要な距離
            （valuesと同じ単位、例: メートル）。0にすると通常の
            （ヒステリシスなしの）量子化と同じ結果になる。

    Returns:
        各フレームに対応する、ヒステリシス適用後のインデックスのリスト。

    ---
    English:
    Applies `idx_fn` (a monotonically non-decreasing, step-shaped grid
    index function -- `grid_index_centered` and `grid_index_variable`
    both qualify) with hysteresis (a Schmitt-trigger characteristic):
    rather than switching the instant a boundary is crossed, it switches
    only once the value has moved **an extra `margin` past the
    boundary**.

    This addresses the problem found in Section 12.9: with a fine grid,
    measurement noise or a small real back-and-forth is enough to cross a
    grid-cell boundary, and that crossing then shows up in the model as a
    genuine branch point. If a value sits right at a boundary,
    hysteresis-free quantization repeatedly crosses back and forth as
    noise pushes it one way then the other; this function absorbs that
    chattering by requiring the value to step an extra `margin` past the
    current cell before it is allowed to leave that cell.

    Implementation: for the most recent value at which the index was
    stably `cur` (`last_stable_value`), and a new value whose raw index
    differs, the actual switching boundary between them is located by
    bisection (generic, independent of whether `idx_fn` is the uniform
    grid or the near/far variable grid), and the index change is
    accepted only once the new value has moved a further `margin` beyond
    that boundary.

    Args:
        values: the original continuous-valued time series (e.g. rx or ry).
        idx_fn: a monotonically non-decreasing grid index function (e.g.
            `lambda v: grid_index_centered(v, gy)`).
        margin: the extra distance required, in the same units as
            `values` (e.g. meters), before a boundary crossing is
            accepted as real. 0 reproduces ordinary (hysteresis-free)
            quantization.

    Returns:
        The list of hysteresis-applied indices, one per frame.
    """
    out: List[int] = []
    cur: Optional[int] = None
    last_stable_value: Optional[float] = None

    for v in values:
        raw = idx_fn(v)
        if cur is None:
            cur = raw
            last_stable_value = v
            out.append(cur)
            continue
        if raw == cur:
            last_stable_value = v
            out.append(cur)
            continue

        # raw != cur: idx_fn is monotonic, so the boundary crossed lies
        # strictly between last_stable_value (index == cur) and v
        # (index == raw). Bisect to find it. `a` is always anchored on the
        # "still cur" side and `b` on the "already raw" side -- NOT sorted
        # by numeric value, since last_stable_value can be either larger
        # or smaller than v depending on whether values are increasing or
        # decreasing.
        a, b = last_stable_value, v
        direction = 1 if v > last_stable_value else -1
        for _ in range(60):
            mid = (a + b) / 2.0
            if idx_fn(mid) == cur:
                a = mid
            else:
                b = mid
        boundary = b

        crossed = (v >= boundary + margin) if direction > 0 else (v <= boundary - margin)
        if crossed:
            cur = raw
            last_stable_value = v
        # else: treated as noise -- stay at `cur`, keep last_stable_value
        # anchored at its old (solidly-cur) value.
        out.append(cur)

    return out


def to_grid_indices_variable_hysteresis(
    rxs: Sequence[Optional[float]],
    rys: Sequence[Optional[float]],
    rx_near_cell: float,
    rx_far_cell: float,
    rx_near_range: float,
    gy: float,
    rx_margin: Optional[float] = None,
    ry_margin: Optional[float] = None,
    margin_ratio: float = 0.3,
) -> Tuple[List[int], List[int]]:
    """`to_grid_indices_variable`のヒステリシス付き版。rx・ryそれぞれの
    時系列全体に対して`hysteresis_filter_indices`を適用する。

    rx_margin・ry_marginを省略した場合は、それぞれ
    `margin_ratio * rx_near_cell`・`margin_ratio * gy`
    （デフォルトでセルサイズの30%）を使う。Egoに近い領域のセル
    （rx_near_cell）を基準にするのは、ノイズによる分岐が問題になるのは
    主にEgo近傍の細かいセルであり、遠方の粗いセル（rx_far_cell）の
    境界をたまたま跨いでも実害が小さいため。

    ---
    English:
    Hysteresis version of `to_grid_indices_variable`. Applies
    `hysteresis_filter_indices` to the whole time series of rx and of ry
    separately.

    If rx_margin/ry_margin are omitted, `margin_ratio * rx_near_cell` and
    `margin_ratio * gy` are used respectively (30% of the cell size by
    default). The near-Ego cell size (rx_near_cell) is used as the basis
    because noise-induced branching is mainly a problem in the fine cells
    near Ego; incidentally crossing a boundary in the coarse far-away
    cells (rx_far_cell) does little harm.
    """
    rx_margin = rx_margin if rx_margin is not None else margin_ratio * rx_near_cell
    ry_margin = ry_margin if ry_margin is not None else margin_ratio * gy

    valid = [
        (rx is not None and ry is not None and not np.isnan(rx) and not np.isnan(ry))
        for rx, ry in zip(rxs, rys)
    ]
    rxs_valid = [rx for rx, v in zip(rxs, valid) if v]
    rys_valid = [ry for ry, v in zip(rys, valid) if v]

    i_seq = hysteresis_filter_indices(
        rxs_valid, lambda v: grid_index_variable(v, rx_near_cell, rx_far_cell, rx_near_range), rx_margin
    )
    k_seq = hysteresis_filter_indices(rys_valid, lambda v: grid_index_centered(v, gy), ry_margin)

    is_ = iter(i_seq)
    ks_ = iter(k_seq)
    result_i: List[Optional[int]] = []
    result_k: List[Optional[int]] = []
    for v in valid:
        if v:
            result_i.append(next(is_))
            result_k.append(next(ks_))
        else:
            result_i.append(None)
            result_k.append(None)
    return result_i, result_k


def compress_to_grid_states_variable_hysteresis(
    rxs: Sequence[Optional[float]],
    rys: Sequence[Optional[float]],
    rx_near_cell: float,
    rx_far_cell: float,
    rx_near_range: float,
    gy: float,
    rx_margin: Optional[float] = None,
    ry_margin: Optional[float] = None,
    margin_ratio: float = 0.3,
) -> List[GridState]:
    """`compress_to_grid_states_variable`のヒステリシス付き版（ノイズ除去の
    抽象化）。境界ちょうどでの揺れ戻りによる余計な状態変化・分岐点を
    抑えた上で、イベント駆動の圧縮（連続して同じ(i,k)に留まる区間を
    1状態にまとめる）を行う。

    ---
    English:
    Hysteresis version of `compress_to_grid_states_variable` (a
    noise-removal abstraction). Suppresses spurious state changes/branch
    points caused by back-and-forth right at a boundary, then performs
    the same event-driven compression (a run of consecutive frames
    staying in the same (i, k) collapsed into one state).
    """
    i_seq, k_seq = to_grid_indices_variable_hysteresis(
        rxs, rys, rx_near_cell, rx_far_cell, rx_near_range, gy,
        rx_margin=rx_margin, ry_margin=ry_margin, margin_ratio=margin_ratio,
    )

    states: List[GridState] = []
    prev_ik: Optional[Tuple[int, int]] = None
    for frame, (i, k) in enumerate(zip(i_seq, k_seq)):
        if i is None:
            continue
        if prev_ik is not None and (i, k) == prev_ik:
            states[-1].end_frame = frame
            continue
        states.append(GridState(index=len(states), i=i, k=k, start_frame=frame, end_frame=frame))
        prev_ik = (i, k)

    return states


def grid_states_from_relative_xy(
    rel_xy: Sequence[Tuple[float, float]], gx: float, gy: float
) -> List[GridState]:
    """すでに ego 基準の相対座標 (rx, ry) が分かっている場合の簡易入口。

    実データが手元にない場合の合成トラジェクトリでのテストや、
    座標正規化を別途済ませている場合に使う。

    ---
    English:
    A simple entry point for when the ego-relative coordinates
    (rx, ry) are already known.

    Used for testing with synthetic trajectories when real data is
    not on hand, or when coordinate normalization has already been
    done separately.
    """
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    return compress_to_grid_states(rxs, rys, gx, gy)


def relative_xy_from_ajisai_groundtruth(
    json_path: str, npc_name: Optional[str] = None
) -> List[Tuple[float, float]]:
    """実際のAJISAIログ（JAMA-Traceable ADS Runtime Log Dataset）のJSONファイルから、
    ego基準の相対座標 (rx, ry) の時系列を取り出す。

    AJISAIのログ形式では、`groundtruth_kinematic` が
    `[{"timestamp": ..., "groundtruth_ego": {...}, "groundtruth_vehicles": [...]}, ...]`
    という、egoと全NPCの位置(x, y, z)・姿勢(rotation)がタイムスタンプごとに
    同期して記録された配列になっている（world座標系、単位は概ねメートル）。

    ここでは、各タイムスタンプについて
      1. egoの姿勢の `rotation.z`（度、water-levelのyaw角）から、
         ワールド座標系でのegoの前方単位ベクトル・左方単位ベクトルを求める
         （実データで検証済み: 連続する2フレームのego位置の変位ベクトルの
         向きと `rotation.z` が一致することを確認した — すなわち
         rotation.z は標準的な数学の角度（+x軸から反時計回りに測った角度、度）
         としてそのまま使ってよい）。
      2. 指定したNPC（`npc_name`。Noneの場合は、そのフレームに存在する唯一の
         NPCを自動選択する。複数いる場合はエラー）の位置とegoの位置の差分
         （ワールド座標系）を、上記の前方・左方ベクトルに射影することで
         rx（前方距離）・ry（左方向のオフセット。正が左隣接レーン側、
         負が右隣接レーン側 — logverify全体で使っているry符号の規約と一致）
         を求める。
    NPCがそのフレームに存在しない（未検出・視野外など）タイムスタンプは
    スキップする（欠測として扱う。補間はしない）。

    Args:
        json_path: AJISAIログのJSONファイルへのパス。
        npc_name: 相対座標を計算する対象のNPC名（例: "npc1"）。
            Noneの場合、各フレームで観測される名前の集合から一意に決まれば
            それを使う。複数のNPC名が観測される場合はValueErrorを送出する
            （その場合は npc_name を明示的に指定すること）。

    Returns:
        [(rx, ry), ...] のリスト（`grid_states_from_relative_xy` や
        `zones.zone_states_from_relative_xy` にそのまま渡せる）。

    ---
    English:
    Extract a time series of ego-relative coordinates (rx, ry) from an
    actual AJISAI log (JAMA-Traceable ADS Runtime Log Dataset) JSON file.

    In the AJISAI log format, `groundtruth_kinematic` is an array of
    `[{"timestamp": ..., "groundtruth_ego": {...}, "groundtruth_vehicles": [...]}, ...]`,
    where the position (x, y, z) and orientation (rotation) of ego and every
    NPC are recorded together, synchronized per timestamp (world coordinate
    frame, units roughly in meters).

    For each timestamp, this function:
      1. Computes ego's forward and left unit vectors in the world frame
         from ego's `rotation.z` (degrees, yaw angle) (verified against the
         real data: the direction of the displacement vector between two
         consecutive ego positions matches `rotation.z`, confirming that
         rotation.z can be used directly as a standard mathematical angle —
         degrees measured counterclockwise from the +x axis).
      2. Projects the world-frame difference between the specified NPC's
         (`npc_name`; if None, the single NPC observed in that frame is
         auto-selected, and an error is raised if more than one NPC name is
         observed) position and ego's position onto the forward/left
         vectors above, to obtain rx (forward distance) and ry (leftward
         offset — positive is the left-adjacent-lane side, negative is the
         right-adjacent-lane side, matching the ry sign convention used
         throughout logverify).
    Timestamps where the NPC is not present in that frame (not detected,
    out of view, etc.) are skipped (treated as missing data; no
    interpolation is performed).

    Args:
        json_path: path to the AJISAI log JSON file.
        npc_name: the name of the NPC to compute relative coordinates for
            (e.g. "npc1"). If None, the name observed across frames is used
            if it is unique; a ValueError is raised if more than one NPC
            name is observed (in that case, pass npc_name explicitly).

    Returns:
        A list of [(rx, ry), ...] (can be passed directly to
        `grid_states_from_relative_xy` or
        `zones.zone_states_from_relative_xy`).
    """
    import json
    import math

    with open(json_path) as f:
        data = json.load(f)

    gk = data["groundtruth_kinematic"]

    if npc_name is None:
        observed_names = set()
        for rec in gk:
            for v in rec.get("groundtruth_vehicles", []):
                observed_names.add(v["name"])
        if len(observed_names) != 1:
            raise ValueError(
                f"npc_name を指定してください（複数のNPC名が観測されました: {sorted(observed_names)}） "
                f"/ (English) please specify npc_name explicitly (multiple NPC names observed: "
                f"{sorted(observed_names)})"
            )
        npc_name = next(iter(observed_names))

    rel_xy: List[Tuple[float, float]] = []
    for rec in gk:
        ego = rec.get("groundtruth_ego")
        if ego is None:
            continue
        ex = ego["pose"]["position"]["x"]
        ey = ego["pose"]["position"]["y"]
        yaw = math.radians(ego["pose"]["rotation"]["z"])
        fwd_x, fwd_y = math.cos(yaw), math.sin(yaw)
        left_x, left_y = -math.sin(yaw), math.cos(yaw)

        npc = next(
            (v for v in rec.get("groundtruth_vehicles", []) if v["name"] == npc_name), None
        )
        if npc is None:
            continue
        nx = npc["pose"]["position"]["x"]
        ny = npc["pose"]["position"]["y"]
        dx, dy = nx - ex, ny - ey

        rx = dx * fwd_x + dy * fwd_y
        ry = dx * left_x + dy * left_y
        rel_xy.append((rx, ry))

    return rel_xy
