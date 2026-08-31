"""
縦方向の距離を「近距離・中距離・遠距離」といった少数の順序尺度
(ordinal scale) に分類するモジュール。

docs/log_to_cpd_verification_design.md 2.1節が述べている通り、CPDの
position は連続座標ではなく、モデル設計者が与える離散的な順序値である。
logverify.grid_bridge の生の格子インデックス（例: i_range=-2..20）を
そのまま参照CPDの箱として使うと、箱の数が距離レンジに比例して増え、
遷移をすべて張ると (箱数)^2 のオーダーで爆発してソルバが実用的な時間で
終わらなくなる。

そこで、参照CPD側は「近距離(0) / 中距離(1) / 遠距離(2)」（+ 後方(-1)）
という少数の順序値だけを position として持つようにし、縦方向距離(rx)を
直接この順序値に丸め込む。箱の数は距離レンジによらず一定になり、
「近距離cut-in」「中距離cut-in」「遠距離cut-in」といった質問にも
直接対応する。
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from logverify.grid_bridge import grid_index_centered


# 順序値 -> ラベル。実際の position の整数値としてそのままCPDに使う。
BEHIND = -1
NEAR = 0
MEDIUM = 1
FAR = 2

ZONE_LABELS = {BEHIND: "後方", NEAR: "近距離", MEDIUM: "中距離", FAR: "遠距離"}


@dataclass
class ZoneThresholds:
    """縦方向距離 rx (m, 前方が正) を順序値に丸めるためのしきい値。"""
    near_max: float = 5.0     # |rx| <= near_max を「近距離」（後方はbehindが優先）
    medium_max: float = 20.0  # near_max < rx <= medium_max を「中距離」
    # rx > medium_max は「遠距離」、rx < -near_max は「後方」


def classify_rx(rx: float, th: ZoneThresholds = ZoneThresholds()) -> int:
    if rx < -th.near_max:
        return BEHIND
    if rx <= th.near_max:
        return NEAR
    if rx <= th.medium_max:
        return MEDIUM
    return FAR


@dataclass
class ZoneState:
    index: int
    lane: int    # 横方向の格子インデックス（laneとしてそのまま使う）
    zone: int    # 縦方向の順序値（positionとしてそのまま使う）
    start_frame: int
    end_frame: int


def compress_to_zone_states(
    rxs: Sequence[Optional[float]],
    rys: Sequence[Optional[float]],
    gy: float,
    thresholds: ZoneThresholds = ZoneThresholds(),
) -> List[ZoneState]:
    """(rx, ry) の時系列を (lane=格子, position=距離帯) に丸め、
    イベント駆動で圧縮する。"""
    states: List[ZoneState] = []
    prev_key: Optional[Tuple[int, int]] = None

    for frame, (rx, ry) in enumerate(zip(rxs, rys)):
        if rx is None or ry is None:
            continue
        lane_val = grid_index_centered(ry, gy)
        zone_val = classify_rx(rx, thresholds)
        key = (lane_val, zone_val)
        if prev_key is not None and key == prev_key:
            states[-1].end_frame = frame
            continue
        states.append(
            ZoneState(index=len(states), lane=lane_val, zone=zone_val, start_frame=frame, end_frame=frame)
        )
        prev_key = key

    return states


def zone_states_from_relative_xy(
    rel_xy: Sequence[Tuple[float, float]], gy: float, thresholds: ZoneThresholds = ZoneThresholds()
) -> List[ZoneState]:
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    return compress_to_zone_states(rxs, rys, gy, thresholds)
