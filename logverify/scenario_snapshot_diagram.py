"""シナリオベース分析のための「スナップショット列」可視化。

ユーザーからの依頼: 「シナリオベース分析なので、CPDのような位置関係の
上に抽象値を割り当ててほしい。EGOとNPCの位置関係（dx0, dy0, Ve0, Vy,
Vo0 のような図）をスナップショットとし、そのスナップショットが列を
成した感じのものを作ってほしい。CPDはそれをモデルとして表現したもの
であり、スナップショットの列がCPDで列挙したものになっている関係
（スナップショット列 = CPDの箱列の可視化）」。

すなわち本モジュールが描くのは、12.13節の「実時間を横軸にした
スイムレーン」とは違う軸を持つ図である。12.13節はサンプリング間隔
（control_cmds等）ごとに抽象値を並べたが、本モジュールは
**CPDの箱そのもの**（`grid_bridge.GridState` = ヒステリシス付き
near/far格子で圧縮した状態、12.9〜12.11節と同じもの）を1列に並べ、
各箱を「その箱を代表する瞬間のEGO/NPC位置関係の模式図
（Subject/Ego・NPCの矩形、Ve0・Vo0・Vyの矢印、dx0・dy0の寸法線）」
として描く。1つの箱=1つのスナップショット=1枚のパネルであり、
パネルを箱の順番に並べたものが、まさにCPDが列挙する状態遷移列の
絵になっている（gcpd.Modelのbox列と1対1に対応する）。

各パネルには、12.12節の3つの抽象解釈演算子（減速の十分性・NPC予測の
信頼性・接触余裕）による分類結果も、その箱の代表フレームにおける値
として併記する。位置関係の模式図（幾何）と、原因を語る抽象値（分類
ラベル）を、CPDの1箱の上に同時に載せる、という試みである。

---
English:
"Snapshot sequence" visualization for scenario-based analysis.

The user asked: since this is a scenario-based analysis, abstract
values should be assigned on top of a CPD-like positional relation.
Each Ego/NPC positional-relation diagram (in the style of the
attached figure with dx0, dy0, Ve0, Vy, Vo0) is one snapshot, and a
sequence of such snapshots should be produced -- because a CPD is a
formal *model* of exactly that: the sequence of snapshots is what a
CPD enumerates as its box sequence.

This module therefore uses a different axis than Section 12.13's
"swimlanes along real time". Section 12.13 plotted abstract values
at each raw sampling instant; this module instead lays out the
**CPD boxes themselves** (`grid_bridge.GridState`, i.e. the states
after hysteresis-filtered near/far-grid compression -- the same
boxes used in Sections 12.9-12.11) in a single row, and draws each
box as "a schematic positional diagram of Ego/NPC at the instant
that box represents" (Ego/NPC rectangles, Ve0/Vo0/Vy velocity
arrows, dx0/dy0 dimension lines). One box = one snapshot = one
panel, and the panels laid out in box order are literally a picture
of the state-transition sequence a CPD (`gcpd.Model`) enumerates --
a 1:1 correspondence with the box sequence of the gcpd.Model.

Each panel also carries the classification results of Section
12.12's three abstract-interpretation operators (deceleration
adequacy, NPC prediction reliability, contact margin), evaluated at
that box's representative frame -- an attempt to place both the
geometric positional relation and the cause-naming abstract values
on the same CPD box at once.
"""

from dataclasses import dataclass
from typing import List, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch, FancyArrowPatch, Rectangle

from logverify.abstract_cause_diagram import CONTACT_COLORS, DECEL_COLORS, PRED_COLORS

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

# 12.21節: JAMA C&Cモデルの反実仮想と実際のNPC位置とのずれ（縦方向の
# 「差」＝ rx_cc_ref - rx）を強調表示するための色分け。差が小さいほど
# 「有能で慎重な人間ドライバなら取ったはずの挙動に近い」ことを意味し、
# 差が大きいほど、実際の挙動がその基準から乖離している（＝危険側）
# ことを意味する。
#
# English: Section 12.21 — colors for emphasizing the deviation
# (longitudinal "gap" = rx_cc_ref - rx) between the JAMA C&C model's
# counterfactual and the actual NPC position. A small gap means the
# actual behavior is close to what a competent and careful human
# driver would have produced; a large gap means the actual behavior
# has diverged from that reference (i.e. toward the dangerous side).
CC_GAP_COLORS = {
    "match": "#a5d6a7",
    "deviate": "#ffcc80",
    "diverge": "#ef9a9a",
}


def _cc_gap_label(gap: float) -> str:
    if gap <= 1.0:
        return "match"
    if gap <= 5.0:
        return "deviate"
    return "diverge"


@dataclass
class ScenarioSnapshot:
    """CPDの1箱（GridState）を代表する、1枚のスナップショット。

    ---
    English:
    One snapshot representing a single CPD box (GridState).
    """
    box_index: int
    t: float                 # representative timestamp (s)
    rx: float                # longitudinal offset, NPC relative to Ego (m, +front)
    ry: float                # lateral offset, NPC relative to Ego (m, +left)
    ego_speed: float         # Ego's own speed, |v| (m/s)  -- "Ve0"
    npc_speed: float         # NPC's own speed, |v| (m/s)  -- "Vo0"
    npc_lateral_speed: float # d(ry)/dt (m/s, +left)       -- "Vy"
    decel_label: Optional[str] = None
    pred_label: Optional[str] = None
    contact_label: Optional[str] = None
    lane_k: Optional[int] = None
    pos_i: Optional[int] = None
    # 12.19節: JAMA C&Cモデルの反実仮想（risk知覚後のカウンターファク
    # チュアルなNPCとの縦方向距離）。Noneなら、そのゴースト矩形・
    # C&C差ラベルは描かれない（12.14節までの図との後方互換）。
    # 12.20節でTTCラベルを併記していたが、12.21節でTTCは抽象化に
    # 有用でないと判断し、本モジュールからは削除した（TTC自体は
    # jama_cc_model.find_risk_perceived_frame の内部トリガーとしては
    # 引き続き使われている）。
    #
    # English: Section 12.19's JAMA C&C model counterfactual (the NPC's
    # longitudinal distance under the reference driver, after risk is
    # perceived). None draws neither the ghost NPC box nor the C&C-gap
    # label (backward compatible with the diagrams through Section
    # 12.14). Section 12.20 additionally showed a TTC label here, but
    # Section 12.21 removed it from this module (TTC itself remains in
    # use as one of the internal triggers of
    # jama_cc_model.find_risk_perceived_frame).
    rx_cc_ref: Optional[float] = None


def _draw_velocity_arrow(ax, x0, y0, dx, dy, color, label):
    if abs(dx) < 1e-3 and abs(dy) < 1e-3:
        return
    arrow = FancyArrowPatch((x0, y0), (x0 + dx, y0 + dy), arrowstyle="-|>", mutation_scale=8,
                             color=color, linewidth=1.3, zorder=5)
    ax.add_patch(arrow)
    ax.text(x0 + dx * 1.12, y0 + dy * 1.12, label, fontsize=6.5, color=color, ha="center", va="center", zorder=6)


def _abstract_label_row(ax, y_frac, name, label, colors):
    if label is None:
        return
    color = colors.get(label, "#dddddd")
    ax.text(
        0.5, y_frac, f"{name}: {label}", transform=ax.transAxes, ha="center", va="center", fontsize=7.5,
        bbox=dict(boxstyle="round,pad=0.3", facecolor=color, edgecolor="#666666", linewidth=0.5),
    )


def plot_scenario_snapshot_sequence(
    snapshots: List[ScenarioSnapshot],
    output_path: str,
    ego_half_length: float,
    ego_half_width: float,
    npc_half_length: float,
    npc_half_width: float,
    title: str = "",
    t_ref: Optional[float] = None,
    speed_scale: float = 0.55,
    panel_w_in: float = 3.0,
    panel_h_in: float = 3.0,
    show_time: bool = True,
    transition_arrow_style: str = "panel",
) -> str:
    """CPDの箱列(`snapshots`, 箱の順に並んでいること)を、1箱=1パネルの
    横並びスナップショット列として描画する。

    transition_arrow_style: 隣接パネル間の「箱遷移」矢印の描き方。
    - "panel"（デフォルト、12.14〜12.22節までと同じ）: パネルとパネルの
      間の空白に、各パネル中央の高さで1本の矢印を描く。実測ログ由来の
      スナップショット列（このログ自身が辿った経路そのもの）を表示する
      用途を想定しており、CPDモデルから列挙されうる「シナリオ」を後で
      この上に重ねて示す拡張がしやすいよう、位置関係とは独立した単純な
      「次に進む」という記号として描いている。
    - "boxes"（12.23節で追加）: EGOを表す矩形どうし、NPCを表す矩形どうし
      を、`ConnectionPatch`で直接結ぶ（色を分けて区別）。CPDモデルの
      箱列（`gcpd.Model`のntrans）をそのまま可視化する用途を想定して
      おり、矢印そのものがEGO/NPCそれぞれの実座標上の経路を直接なぞる。

    ---
    English:
    Draws a CPD box sequence (`snapshots`, assumed already in box order)
    as a horizontal sequence of snapshots, one panel per box.

    transition_arrow_style: how the "box transition" arrow between
    adjacent panels is drawn.
    - "panel" (default, same as Sections 12.14-12.22): a single arrow
      drawn in the empty space between panels, at each panel's
      mid-height. Intended for the snapshot sequence built from an
      actual measured log (the path this log itself followed); it is
      drawn as a simple "advance to the next" symbol independent of
      position, so that "scenarios" enumerable from the CPD model can
      later be overlaid on top of it.
    - "boxes" (added in Section 12.23): directly connects the
      rectangles representing EGO to each other, and separately the
      rectangles representing NPC to each other, via `ConnectionPatch`
      (in different colors). Intended for visualizing a CPD model's own
      box sequence (a gcpd.Model's ntrans) directly, where the arrow
      itself traces EGO's and NPC's paths in real coordinates.
    """
    if transition_arrow_style not in ("panel", "boxes"):
        raise ValueError(f"transition_arrow_style must be 'panel' or 'boxes', got {transition_arrow_style!r}")
    n = len(snapshots)
    assert n > 0, "snapshots must be non-empty"

    all_rx = [s.rx for s in snapshots] + [s.rx_cc_ref for s in snapshots if s.rx_cc_ref is not None]
    x_min = min(min(all_rx) - npc_half_length, -ego_half_length) - 2.0
    x_max = max(max(all_rx) + npc_half_length, ego_half_length) + 2.0
    y_extent = max(max(abs(s.ry) for s in snapshots) + npc_half_width, ego_half_width) + 1.4
    y_min, y_max = -y_extent, y_extent

    label_h_in = panel_h_in * 0.30
    fig = plt.figure(figsize=(panel_w_in * n, panel_h_in + label_h_in))
    gs = fig.add_gridspec(2, n, height_ratios=[panel_h_in, label_h_in], hspace=0.08, wspace=0.15,
                           top=0.84, bottom=0.03, left=0.02, right=0.99)
    axes = [fig.add_subplot(gs[0, j]) for j in range(n)]
    label_axes = [fig.add_subplot(gs[1, j]) for j in range(n)]

    ego_rects: List[Rectangle] = []
    npc_rects: List[Rectangle] = []
    for ax, s in zip(axes, snapshots):
        ax.axhspan(-ego_half_width, ego_half_width, color="#c8e6c9", alpha=0.55, zorder=0)
        ax.axhline(0, color="#9e9e9e", lw=0.5, ls=":", zorder=1)

        ego_rect = Rectangle(
            (-ego_half_length, -ego_half_width), 2 * ego_half_length, 2 * ego_half_width,
            facecolor="#42a5f5", edgecolor="black", linewidth=0.6, zorder=3,
        )
        ax.add_patch(ego_rect)
        ax.text(0, 0, "Ego", ha="center", va="center", fontsize=6.5, color="white", zorder=4)
        ego_rects.append(ego_rect)

        is_colliding = s.contact_label == "接触"
        npc_color = "#e53935" if is_colliding else "#ffa726"
        npc_rect = Rectangle(
            (s.rx - npc_half_length, s.ry - npc_half_width), 2 * npc_half_length, 2 * npc_half_width,
            facecolor=npc_color, edgecolor="black", linewidth=0.6, zorder=3,
        )
        ax.add_patch(npc_rect)
        ax.text(s.rx, s.ry, "NPC", ha="center", va="center", fontsize=6.5, color="white", zorder=4)
        npc_rects.append(npc_rect)

        # 12.19/12.21節: JAMA C&Cモデルの反実仮想における、その箱の代表
        # フレームでのNPC縦方向位置を、実際のNPC矩形に重ねて「ゴースト」
        # 矩形として描く（risk知覚前、あるいは反実仮想が未定義の箱では
        # 描かない）。12.21節でユーザーから「C&C部分を強調してほしい」
        # との依頼を受け、太線・ハッチング・ラベル・実際位置との差を示す
        # 矢印を追加して、単なる細い破線枠より強く目立つようにした。
        #
        # English: Sections 12.19/12.21 — draw the NPC's longitudinal
        # position under the JAMA C&C model's counterfactual, at that
        # box's representative frame, as a "ghost" rectangle overlaid on
        # the actual NPC rectangle (omitted where risk has not yet been
        # perceived, or the counterfactual is undefined). Per the user's
        # Section 12.21 request to "emphasize the C&C part", this was
        # made more visually prominent than a thin dashed outline: a
        # thicker hatched rectangle, a text label, and an arrow showing
        # the deviation from the actual position.
        if s.rx_cc_ref is not None:
            gap = s.rx_cc_ref - s.rx
            ghost_rect = Rectangle(
                (s.rx_cc_ref - npc_half_length, s.ry - npc_half_width), 2 * npc_half_length, 2 * npc_half_width,
                facecolor="#1565c0", alpha=0.18, edgecolor="#0d47a1", linewidth=2.2, linestyle="--",
                hatch="////", zorder=3.6,
            )
            ax.add_patch(ghost_rect)
            ax.text(s.rx_cc_ref, s.ry - npc_half_width - 0.45, "C&C基準",
                    ha="center", va="top", fontsize=6.3, color="#0d47a1", fontweight="bold", zorder=4.1)
            if abs(gap) > 0.05:
                arrow_y = s.ry + npc_half_width + 0.55
                ax.annotate(
                    "", xy=(s.rx_cc_ref, arrow_y), xytext=(s.rx, arrow_y),
                    arrowprops=dict(arrowstyle="-|>", color="#0d47a1", linewidth=1.6), zorder=4.2,
                )
                ax.text((s.rx + s.rx_cc_ref) / 2, arrow_y + y_extent * 0.05, f"{gap:+.1f}m",
                        ha="center", va="bottom", fontsize=6.3, color="#0d47a1", fontweight="bold", zorder=4.2)

        # Ve0: Ego's own forward speed
        _draw_velocity_arrow(ax, ego_half_length, ego_half_width * 0.55, s.ego_speed * speed_scale, 0,
                              "#1565c0", "Ve0")
        # Vo0: NPC's own forward speed
        _draw_velocity_arrow(ax, s.rx + npc_half_length, s.ry + npc_half_width * 0.55, s.npc_speed * speed_scale, 0,
                              "#ef6c00", "Vo0")
        # Vy: NPC's lateral (lane-change) speed, drawn from the NPC box's rear-top corner
        # so it does not overlap the "NPC" label inside the box.
        vy_x0 = s.rx - npc_half_length * 0.7
        vy_y0 = s.ry + npc_half_width
        _draw_velocity_arrow(ax, vy_x0, vy_y0, 0, s.npc_lateral_speed * speed_scale * 3.0, "#6a1b9a", "Vy")

        # dx0: longitudinal gap, Ego front -> NPC rear (or Ego front -> NPC rear even if overlapping/negative)
        y_top = y_max * 0.86
        gap_x0, gap_x1 = ego_half_length, s.rx - npc_half_length
        ax.annotate(
            "", xy=(gap_x1, y_top), xytext=(gap_x0, y_top),
            arrowprops=dict(arrowstyle="<->", color="#555555", linewidth=0.8), zorder=5,
        )
        ax.text((gap_x0 + gap_x1) / 2, y_top + y_extent * 0.06, f"dx0={gap_x1 - gap_x0:+.1f}m",
                ha="center", va="bottom", fontsize=6.3, color="#333333")

        # dy0: lateral offset, Ego center line -> NPC center
        dy_x = x_min + (x_max - x_min) * 0.04
        ax.plot([dy_x, dy_x], [0, s.ry], ls="--", color="#555555", lw=0.8, zorder=2)
        ax.plot([dy_x - 0.3, dy_x + 0.3], [s.ry, s.ry], color="#555555", lw=0.8, zorder=2)
        ax.text(dy_x - 0.35, s.ry / 2, f"dy0={s.ry:+.2f}m", rotation=90, fontsize=6.3, ha="right", va="center",
                color="#333333")

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)

        box_label = f"box #{s.box_index}"
        if s.lane_k is not None and s.pos_i is not None:
            box_label += f" (k={s.lane_k},i={s.pos_i})"
        if show_time:
            t_label = f"t={s.t - t_ref:+.2f}s" if t_ref is not None else f"t={s.t:.2f}s"
            ax.set_title(f"{box_label}\n{t_label}", fontsize=7.5)
        else:
            ax.set_title(box_label, fontsize=7.5)

    for lax, s in zip(label_axes, snapshots):
        lax.set_xlim(0, 1)
        lax.set_ylim(0, 1)
        lax.axis("off")
        has_cc = s.rx_cc_ref is not None
        if has_cc:
            gap = s.rx_cc_ref - s.rx
            gap_label = _cc_gap_label(gap)
            color = CC_GAP_COLORS.get(gap_label, "#dddddd")
            lax.text(
                0.5, 0.87, f"C&C差: {gap:+.1f}m", transform=lax.transAxes, ha="center", va="center",
                fontsize=8.5, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.35", facecolor=color, edgecolor="#0d47a1", linewidth=1.8),
            )
            _abstract_label_row(lax, 0.58, "減速", s.decel_label, DECEL_COLORS)
            _abstract_label_row(lax, 0.33, "予測", s.pred_label, PRED_COLORS)
            _abstract_label_row(lax, 0.08, "余裕", s.contact_label, CONTACT_COLORS)
        else:
            _abstract_label_row(lax, 0.80, "減速", s.decel_label, DECEL_COLORS)
            _abstract_label_row(lax, 0.47, "予測", s.pred_label, PRED_COLORS)
            _abstract_label_row(lax, 0.14, "余裕", s.contact_label, CONTACT_COLORS)

    fig.suptitle(title or "Scenario snapshot sequence (= CPD box sequence)", fontsize=12, y=0.995)
    if any(s.rx_cc_ref is not None for s in snapshots):
        fig.text(0.5, 0.955,
                  "ハッチング矩形＝JAMA C&Cモデルの反実仮想NPC位置（12.19節）／ "
                  "「C&C差」＝有能で慎重な人間ドライバとの縦方向乖離量",
                  ha="center", va="top", fontsize=8.5, color="#0d47a1", fontweight="bold")

    # 箱遷移(CPDのntrans)を表す矢印の描き方は、用途に応じて2通りから選べる
    # （12.23節、`transition_arrow_style`）。
    #
    # ---
    # English: The way box-transition arrows (CPD's ntrans) are drawn is
    # selectable for the two intended use cases (Section 12.23,
    # `transition_arrow_style`).
    fig.canvas.draw()
    if transition_arrow_style == "boxes":
        # "boxes": パネルとパネルの間の宙に引くのではなく、EGOを表す矩形
        # どうし・NPCを表す矩形どうしを直接結ぶ（ユーザーからの依頼:
        # 「BOX遷移は，スナップショットではなくて，EGOを表現するボックス
        # の間にひいてもらえますか．NPCに関しても同様です」）。
        # ConnectionPatchはpatchA/patchBを指定すると、指定したパッチの
        # 境界で矢印を切り詰めてくれるため、パネル(axes)をまたいでも
        # 矩形の縁から縁への矢印になる。EGO用とNPC用で色を変え、どちらの
        # 遷移かが一目で分かるようにした。CPDモデル自身の箱列
        # （gcpd.Modelのntrans）を直接可視化する図（例:
        # jama_cc_cpd_model_positions.png）向け。
        #
        # English: "boxes" -- instead of a line floating between panels,
        # directly connect the rectangles representing EGO to each
        # other, and separately the rectangles representing NPC to each
        # other (per the user's request: "rather than between the
        # snapshots, could you draw the box transition between the
        # boxes representing EGO, and likewise for NPC"). ConnectionPatch,
        # given patchA/patchB, clips the arrow at the given patch's
        # boundary, so even across different panels (axes) the arrow
        # runs edge-to-edge between the rectangles. EGO and NPC arrows
        # use different colors so which transition is which is clear at
        # a glance. Intended for figures that directly visualize a CPD
        # model's own box sequence (a gcpd.Model's ntrans), e.g.
        # jama_cc_cpd_model_positions.png.
        for i in range(n - 1):
            ego_con = ConnectionPatch(
                xyA=(0, 0), coordsA="data", axesA=axes[i],
                xyB=(0, 0), coordsB="data", axesB=axes[i + 1],
                patchA=ego_rects[i], patchB=ego_rects[i + 1],
                arrowstyle="-|>", mutation_scale=11, color="#1565c0", linewidth=1.3, zorder=10,
            )
            fig.add_artist(ego_con)

            s_i, s_j = snapshots[i], snapshots[i + 1]
            npc_con = ConnectionPatch(
                xyA=(s_i.rx, s_i.ry), coordsA="data", axesA=axes[i],
                xyB=(s_j.rx, s_j.ry), coordsB="data", axesB=axes[i + 1],
                patchA=npc_rects[i], patchB=npc_rects[i + 1],
                arrowstyle="-|>", mutation_scale=11, color="#ef6c00", linewidth=1.3, zorder=10,
            )
            fig.add_artist(npc_con)
    else:
        # "panel"（デフォルト、12.14〜12.22節までと同じ）: パネルとパネルの
        # 間の空白に、各パネル中央の高さで1本の矢印を描くだけの、
        # 位置関係とは独立した単純な「次に進む」という記号。実測ログ由来の
        # スナップショット列（このログ自身が辿った経路そのもの。将来的に
        # CPDモデルから列挙されうる「シナリオ」をこの上に重ねて示す拡張が
        # しやすいよう、あえて位置と切り離した表現にしている）向け。
        #
        # English: "panel" (default, same as Sections 12.14-12.22) --
        # just a single arrow drawn in the empty space between panels,
        # at each panel's mid-height: a simple "advance to the next"
        # symbol, independent of position. Intended for the snapshot
        # sequence built from an actual measured log (the path this log
        # itself followed; deliberately kept independent of position so
        # that "scenarios" enumerable from the CPD model can later be
        # overlaid on top of it).
        for i in range(n - 1):
            bbox_l = axes[i].get_position()
            bbox_r = axes[i + 1].get_position()
            y_mid = (bbox_l.y0 + bbox_l.y1) / 2
            fig.patches.append(
                FancyArrowPatch(
                    (bbox_l.x1, y_mid), (bbox_r.x0, y_mid), transform=fig.transFigure,
                    arrowstyle="-|>", mutation_scale=12, color="#424242", linewidth=1.2, zorder=10,
                )
            )

    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path
