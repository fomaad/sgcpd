"""12.12節の3つの抽象解釈演算子（減速の十分性・NPC予測の信頼性・接触余裕）
の分類結果を、時間軸に沿ったスイムレーン（ガントチャート風）として
可視化する。

インスタンスCPD（12.7〜12.9節、`collision_cpd_diagram.py`）が「格子で
圧縮した箱」を列に並べるのに対し、本モジュールは「実時間」を横軸に
とり、3つの抽象値それぞれの分類結果が時間とともにどう遷移するかを、
色分けした帯として重ねて表示する。これにより、生データを見なくても
「どの抽象値が、いつ、どの順番で悪化したか」が一目でわかることを狙う。

一番上には文脈として、EGO周辺の細かい抽象化（12.4節
`compress_to_fine_relation_states`）によるNPCの箱の遷移も同じ時間軸で
重ねて表示する。

---
English:
Visualizes the classification results of Section 12.12's three
abstract-interpretation operators (deceleration adequacy, NPC prediction
reliability, contact margin) as swimlanes (Gantt-chart-like) along a
time axis.

Whereas the instance CPD (Sections 12.7-12.9, `collision_cpd_diagram.py`)
lays "boxes compressed by the grid" out in columns, this module puts
real time on the horizontal axis and overlays each of the three abstract
values' classification results as color-coded bands, so that "which
abstract value degraded, when, and in what order" is visible at a glance
without looking at the raw data.

At the top, for context, the transitions of the NPC's box under the
fine-grained abstraction near Ego (Section 12.4,
`compress_to_fine_relation_states`) are overlaid on the same time axis.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False


DECEL_COLORS = {
    "不要": "#cfd8dc",
    "過剰": "#90caf9",
    "適切": "#a5d6a7",
    "弱い": "#ffcc80",
    "非常に弱い": "#ef9a9a",
}
PRED_COLORS = {
    "的確": "#a5d6a7",
    "低信頼": "#ffcc80",
    "陳腐": "#ef9a9a",
}
CONTACT_COLORS = {
    "余裕": "#a5d6a7",
    "接近中": "#fff59d",
    "接触可能": "#ffab91",
    "接触": "#e53935",
}


@dataclass
class TimeSegment:
    t_start: float
    t_end: float
    label: str


def _compress_segments(samples: List[Tuple[float, str]], t_end_last: Optional[float] = None) -> List[TimeSegment]:
    """(timestamp, label) のリストを、連続して同じlabelが続く区間ごとに
    TimeSegmentへ圧縮する。各セグメントの終端は次のサンプルの開始時刻
    （最後のサンプルはt_end_last、省略時は最後のサンプル時刻+その前との
    平均間隔）とする。

    ---
    English:
    Compresses a list of (timestamp, label) samples into TimeSegments, one
    per run of consecutive identical labels. Each segment's end is the
    next sample's start time (the last sample's end is t_end_last, or if
    omitted, its own timestamp plus the average spacing before it).
    """
    if not samples:
        return []
    segs: List[TimeSegment] = []
    for i, (t, label) in enumerate(samples):
        if segs and segs[-1].label == label:
            continue
        segs.append(TimeSegment(t_start=t, t_end=t, label=label))
    # fill in end times from the next segment's start
    for i in range(len(segs) - 1):
        segs[i].t_end = segs[i + 1].t_start
    if t_end_last is not None:
        segs[-1].t_end = t_end_last
    elif len(samples) > 1:
        avg_dt = (samples[-1][0] - samples[0][0]) / max(1, len(samples) - 1)
        segs[-1].t_end = samples[-1][0] + avg_dt
    return segs


def plot_abstract_cause_timeline(
    output_path: str,
    npc_box_segments: List[TimeSegment],
    decel_segments: List[TimeSegment],
    pred_segments: List[TimeSegment],
    contact_segments: List[TimeSegment],
    onset_ts: Optional[float] = None,
    collision_window: Optional[Tuple[float, float]] = None,
    title: str = "",
) -> str:
    """4つの時系列（NPC箱・減速の十分性・NPC予測信頼性・接触余裕）を、
    共通の時間軸を持つスイムレーンとして重ねて描画する。

    ---
    English:
    Draws the four time series (NPC box, deceleration adequacy, NPC
    prediction reliability, contact margin) as swimlanes sharing a common
    time axis.
    """
    rows = [
        ("NPC位置（EGO近傍を細かく刻んだ抽象化）", npc_box_segments, None),
        ("減速の十分性", decel_segments, DECEL_COLORS),
        ("NPC予測経路の信頼性", pred_segments, PRED_COLORS),
        ("横方向の接触余裕", contact_segments, CONTACT_COLORS),
    ]

    all_ts = [s.t_start for _, segs, _ in rows for s in segs] + [s.t_end for _, segs, _ in rows for s in segs]
    t_min, t_max = min(all_ts), max(all_ts)

    row_h = 1.0
    fig_w = max(10.0, (t_max - t_min) * 6.0)
    fig_h = row_h * len(rows) + 1.6
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    label_colors_used = {}
    npc_palette = plt.get_cmap("tab20")

    for row_idx, (row_name, segs, cmap) in enumerate(rows):
        y = (len(rows) - 1 - row_idx) * row_h
        ax.text(t_min - (t_max - t_min) * 0.02, y + row_h / 2, row_name, ha="right", va="center", fontsize=10)
        for seg in segs:
            if cmap is not None:
                color = cmap.get(seg.label, "#dddddd")
            else:
                if seg.label not in label_colors_used:
                    label_colors_used[seg.label] = npc_palette(len(label_colors_used) % 20)
                color = label_colors_used[seg.label]
            rect = Rectangle(
                (seg.t_start, y + row_h * 0.08), max(seg.t_end - seg.t_start, 1e-3), row_h * 0.84,
                facecolor=color, edgecolor="#666666", linewidth=0.4, zorder=2,
            )
            ax.add_patch(rect)
            width = seg.t_end - seg.t_start
            if width > (t_max - t_min) * 0.012:
                ax.text(
                    (seg.t_start + seg.t_end) / 2, y + row_h / 2, seg.label,
                    ha="center", va="center", fontsize=7.5, zorder=3,
                )

    if collision_window is not None:
        ax.axvspan(collision_window[0], collision_window[1], color="#e53935", alpha=0.12, zorder=0)
        ax.text(
            (collision_window[0] + collision_window[1]) / 2, len(rows) * row_h + 0.15, "衝突ウィンドウ",
            ha="center", va="bottom", fontsize=9, color="#c62828", zorder=5,
        )
    if onset_ts is not None:
        ax.axvline(onset_ts, color="#555555", linestyle="--", linewidth=1.0, zorder=4)
        ax.text(onset_ts, len(rows) * row_h + 0.15, "カットイン検出", ha="center", va="bottom", fontsize=8, color="#555555")

    ax.set_xlim(t_min, t_max)
    ax.set_ylim(-0.2, len(rows) * row_h + 0.6)
    ax.set_xlabel("時刻 (s)", fontsize=10)
    ax.set_yticks([])
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)
    ax.set_title(title or "Abstract cause timeline", fontsize=12, pad=12)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path
