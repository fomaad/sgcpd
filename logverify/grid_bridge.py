"""
グリッドベースの抽象化アダプタ。

Autoware/AJISAIログ（あるいは任意の ego/NPC の時系列座標）を、
ego中心・進行方向基準に正規化した上で、指定したセルサイズ (gx, gy) の
格子に離散化し、CPD の (position=i, lane=k) にそのまま対応する
整数列を作る。

咲川氏の名前付き領域抽象化 (vendor/trajectory_abstraction/src/abstraction_15area.py 等) と
違い、ここでは「参照CPDを書くときに使った粒度」をそのまま使う。
つまり、gx/gy は「1レーンの幅」「1箱に相当する縦方向の距離」を
CPDモデルの設計者が自分で選ぶためのパラメータである。

座標の正規化 (ego進行方向基準への回転) は咲川氏の
vendor/trajectory_abstraction/src/abstraction_grid.py の
normalize_coordinates / extract_coordinates_from_json をそのまま再利用する。
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from vendor.trajectory_abstraction.src.abstraction_grid import (
    extract_coordinates_from_json,
    load_json_data,
    normalize_coordinates,
    normalize_velocities,
    calculate_relative_velocity,
)


@dataclass
class GridState:
    """圧縮後の1状態（イベント駆動: 連続して同じ格子セルに留まる区間を1つにまとめたもの）"""
    index: int          # 0始まりの状態番号（CPDのbox番号にそのまま使う）
    i: int               # 縦方向グリッド index（position）
    k: int               # 横方向グリッド index（lane）
    start_frame: int      # この状態に対応する元データの開始フレーム
    end_frame: int         # この状態に対応する元データの終了フレーム（inclusive）


def grid_index_centered(value: float, cell_size: float) -> int:
    """0を中心にした対称な格子インデックスを返す (round-half-to-even ではなく四捨五入寄り)。

    floor() ベースだと k=0 のセルが [0, gy) のように非対称になり、
    「lane=0 は自車線」という直感（自車線中心 ry=0 の前後 ±gy/2）とずれる。
    ここでは round() を使い、k=0 が [-gy/2, +gy/2) になるようにする。
    """
    return int(np.floor(value / cell_size + 0.5))


def to_grid_indices(
    rx: Optional[float], ry: Optional[float], gx: float, gy: float
) -> Tuple[Optional[int], Optional[int]]:
    if rx is None or ry is None or np.isnan(rx) or np.isnan(ry):
        return None, None
    return grid_index_centered(rx, gx), grid_index_centered(ry, gy)


def compress_to_grid_states(
    rxs: Sequence[Optional[float]],
    rys: Sequence[Optional[float]],
    gx: float,
    gy: float,
) -> List[GridState]:
    """(rx, ry) の時系列を格子に離散化し、イベント駆動で圧縮する。

    「連続して同じ (i, k) に留まっている区間」を1つの状態にまとめる。
    これは docs/log_to_cpd_verification_design.md 4.3節「イベント駆動（第一選択）」の実装。
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


def grid_states_from_json(input_path: str, gx: float, gy: float) -> List[GridState]:
    """AJISAI形式のログJSON（あるいは groundtruth_kinematic を持つ本体ログ）から
    圧縮済みグリッド状態列を作る。"""
    data = load_json_data(input_path)
    ego_c, npc_c, ego_v, npc_v, ts, ids = extract_coordinates_from_json(data)
    if len(ts) == 0:
        return []
    ego_n, npc_n, rot = normalize_coordinates(ego_c, npc_c)
    rxs = [None if c[0] is None or (isinstance(c[0], float) and np.isnan(c[0])) else c[0] for c in npc_n]
    rys = [None if c[1] is None or (isinstance(c[1], float) and np.isnan(c[1])) else c[1] for c in npc_n]
    return compress_to_grid_states(rxs, rys, gx, gy)


def grid_states_from_relative_xy(
    rel_xy: Sequence[Tuple[float, float]], gx: float, gy: float
) -> List[GridState]:
    """すでに ego 基準の相対座標 (rx, ry) が分かっている場合の簡易入口。

    実データが手元にない場合の合成トラジェクトリでのテストや、
    座標正規化を別途済ませている場合に使う。
    """
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    return compress_to_grid_states(rxs, rys, gx, gy)
