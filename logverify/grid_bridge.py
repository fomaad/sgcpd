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
