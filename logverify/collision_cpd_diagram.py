"""1本のログの衝突前後だけを細かく、それ以外は粗く抽象化した「インスタンス
CPD」を、論文（Scenario Modeling Language Camera-Ready Fig.2/Fig.4）に近い
スタイルで描画する。

これまでの`model_diagram.plot_model_with_ego_paper_style`は、
  - NPC側のスイムレーンは「(lane, position)の値の順位」で列を割り当てる
  - Ego側のスイムレーンは「ステップ番号」で列を割り当てる
という、**互いに意味の異なる列軸**を使っていたため、2つのスイムレーンを
見比べても「同じ瞬間に何が起きていたか」が分からなかった
（ユーザー指摘: 「EGOとNPCの振舞がCPDで離れていると関係がわからない」）。

本モジュールは、1本の観測ログ（インスタンス）専用に、
  - 列 = 時間順（このログで実際に起きた箱の遷移の順序）
  - 行 = 咲川氏のego車線境界を基準にした「隣接レーン」「ego車線」の
    2スイムレーン（NPC側）+ 「Ego自身の挙動」の1スイムレーン（Ego側）
という、**両方のスイムレーンが同じ列軸（=同じ瞬間）を共有する**描画を行う。
これにより、あるNPCの箱の真下（または真上）に、その瞬間のEgoの挙動
（巡航／減速開始／強い減速）が並んで表示され、両者の関係が一目で分かる。

さらに、衝突が起きた箱を赤枠で強調し、減速開始・強い減速到達などの
イベントをテキスト注記として直接CPD図に書き込む。

---
English:
Draws a single log's "instance CPD" -- fine-grained only around the
collision, coarse everywhere else -- in a style close to the paper
(Scenario Modeling Language Camera-Ready Fig.2/Fig.4).

The existing `model_diagram.plot_model_with_ego_paper_style` assigned
columns to the NPC-side swimlane by "rank of the (lane, position) value"
and to the Ego-side swimlane by "step number" -- two **axes with
different meanings** -- so comparing the two swimlanes could not reveal
"what was happening at the same moment" (the user's complaint: "with Ego
and NPC's behavior drawn apart in the CPD, the relationship is unclear").

This module instead draws, specifically for one observed log (an
instance):
  - columns = chronological order (the actual order of box transitions
    that occurred in this log)
  - rows = two swimlanes for the NPC side ("adjacent lane" / "ego lane",
    split by Mr. Sakikawa's ego-lane boundary) plus one swimlane for
    Ego's own behavior
so that **both swimlanes share the same column axis (the same instant)**.
This puts Ego's behavior at that moment (cruising / starting to
decelerate / braking hard) directly above or below the NPC's box for
that same moment, making the relationship between the two visible at a
glance.

It also highlights the box(es) where a collision occurred with a red
border, and writes events such as "deceleration onset" and "reaching
strong braking" directly onto the diagram as text annotations.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Circle

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False


@dataclass
class InstanceBox:
    """インスタンスCPDの1つの箱（時間順に並んだ、圧縮済み状態の1つ）。

    ---
    English:
    One box of the instance CPD (one compressed state, in chronological
    order).
    """

    label: str          # 箱に表示するラベル（例: "NPC(k=-6,i=4)"）
    lane_band: str       # スイムレーンのキー（例: "side", "ego", "far"）
    start_frame: int
    end_frame: int
    highlight: Optional[str] = None   # 例: "collision" -> 赤枠で強調
    note: Optional[str] = None        # 箱の下に書く短い注記
    ego_state: Optional[str] = None   # 同じ列に表示するEgoの状態ラベル
    ego_color: Optional[str] = None   # Ego状態の背景色


@dataclass
class EventAnnotation:
    """特定の箱（列）に紐付ける、テキストの吹き出し注記。

    ---
    English:
    A text call-out annotation attached to a specific box (column).
    """

    box_index: int
    text: str
    color: str = "#333333"


def plot_instance_cpd_annotated(
    boxes: List[InstanceBox],
    lane_band_order: List[Tuple[str, str]],  # [(band_key, display_label), ...] 上から順
    output_path: str,
    title: str = "",
    events: Optional[List[EventAnnotation]] = None,
    box_w: float = 0.85,
    box_h: float = 0.6,
    lane_gap: float = 2.0,
    col_gap: float = 1.05,
) -> str:
    """1本のログのインスタンスCPDを、時間順の共通列軸で、NPCの
    スイムレーンとEgoの挙動スイムレーンを揃えて描画する。

    ---
    English:
    Draws one log's instance CPD, aligning the NPC swimlanes and the
    Ego-behavior swimlane on a shared, chronologically-ordered column
    axis.
    """
    events = events or []
    n = len(boxes)

    row_of_band = {key: i for i, (key, _) in enumerate(lane_band_order)}
    n_bands = len(lane_band_order)
    ego_row_y = n_bands * lane_gap  # Egoの行は一番上

    def band_y(band_key: str) -> float:
        return (n_bands - 1 - row_of_band[band_key]) * lane_gap

    fig_w = max(8.0, 1.0 * n + 2.0)
    fig_h = max(4.0, lane_gap * (n_bands + 1) + 2.2)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # NPC側のスイムレーン帯とラベル
    for key, disp in lane_band_order:
        y = band_y(key)
        ax.axhspan(y - box_h * 0.85, y + box_h * 0.85, color="#f5f7fb", zorder=0)
        ax.text(
            -col_gap * 0.9, y, disp, ha="right", va="center", fontsize=10, color="#555555",
            bbox=dict(facecolor="#f5f7fb", edgecolor="none", pad=1.5), zorder=6,
        )

    # Ego行の帯とラベル
    ax.axhspan(ego_row_y - box_h * 0.85, ego_row_y + box_h * 0.85, color="#eaf6ea", zorder=0)
    ax.text(
        -col_gap * 0.9, ego_row_y, "Ego（自身の挙動）", ha="right", va="center", fontsize=10,
        color="#2a7a2a", bbox=dict(facecolor="#eaf6ea", edgecolor="none", pad=1.5), zorder=6,
    )

    def xy(idx: int, band_key: str) -> Tuple[float, float]:
        return idx * col_gap, band_y(band_key)

    # NPCの箱を時間順の列に描く
    prev_xy = None
    for idx, b in enumerate(boxes):
        x, y = xy(idx, b.lane_band)
        edge = "#333333"
        lw = 1.2
        face = "#ffffff"
        if b.highlight == "collision":
            edge = "#cc2222"
            lw = 2.2
            face = "#ffe9e9"
        box = FancyBboxPatch(
            (x - box_w / 2, y - box_h / 2), box_w, box_h,
            boxstyle="round,pad=0.02,rounding_size=0.08",
            linewidth=lw, edgecolor=edge, facecolor=face, zorder=3,
        )
        ax.add_patch(box)
        ax.text(x, y, b.label, ha="center", va="center", fontsize=7.5, zorder=4)
        if b.note:
            ax.text(
                x, y - box_h * 0.95, b.note, ha="center", va="top", fontsize=6.8,
                color="#a33", rotation=90 if len(b.note) > 6 else 0, zorder=4,
            )
        if prev_xy is not None:
            x1, y1 = prev_xy
            rad = 0.0 if y1 == y else (0.2 if y > y1 else -0.2)
            arrow = FancyArrowPatch(
                (x1 + box_w / 2, y1), (x - box_w / 2, y),
                arrowstyle="-|>", mutation_scale=10, color="#333333", linewidth=1.0,
                zorder=2, connectionstyle=f"arc3,rad={rad}", shrinkA=2, shrinkB=2,
            )
            ax.add_patch(arrow)
        prev_xy = (x, y)

        # Egoの状態を同じ列に描く（NPCと同じ瞬間であることが列で分かる）
        if b.ego_state is not None:
            ex, ey = xy(idx, None) if False else (x, ego_row_y)
            ego_box = FancyBboxPatch(
                (ex - box_w / 2, ey - box_h / 2), box_w, box_h,
                boxstyle="round,pad=0.02,rounding_size=0.08",
                linewidth=1.2, edgecolor="#2a7a2a", facecolor=b.ego_color or "#eafaea", zorder=3,
            )
            ax.add_patch(ego_box)
            ax.text(ex, ey, b.ego_state, ha="center", va="center", fontsize=7, zorder=4)
            # NPCとEgoの箱を、同じ瞬間であることを示す薄い縦の点線で結ぶ
            ax.plot([x, x], [y + box_h / 2, ego_row_y - box_h / 2], linestyle=":", color="#bbbbbb", linewidth=0.8, zorder=1)

    # イベント注記（吹き出し）。同じ箱に複数のイベントが付く場合は、
    # 重ならないよう縦方向にずらして積み上げる。
    # (English) Event call-outs. When more than one event attaches to the
    # same box, stack them vertically so they don't overlap.
    stack_count: dict = {}
    for ev in events:
        if not (0 <= ev.box_index < n):
            continue
        b = boxes[ev.box_index]
        x, y = xy(ev.box_index, b.lane_band)
        k = ev.box_index
        level = stack_count.get(k, 0)
        stack_count[k] = level + 1
        y_text = y + box_h / 2 + lane_gap * (0.32 + 0.34 * level)
        ax.annotate(
            ev.text,
            xy=(x, y + box_h / 2), xytext=(x, y_text),
            ha="center", va="bottom", fontsize=7.5, color=ev.color,
            arrowprops=dict(arrowstyle="->", color=ev.color, linewidth=1.0),
            bbox=dict(boxstyle="round,pad=0.25", facecolor="#ffffff", edgecolor=ev.color, linewidth=0.8, alpha=0.95),
            zorder=7,
        )

    ax.set_xlim(-col_gap * 1.5, n * col_gap)
    ax.set_ylim(-lane_gap * 0.8, ego_row_y + lane_gap * 1.3)
    ax.axis("off")
    ax.set_title(title or "Instance CPD (annotated)", fontsize=12, pad=10)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path
