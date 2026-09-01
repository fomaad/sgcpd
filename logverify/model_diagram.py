"""
gcpd.Model の構造（箱と遷移のグラフ）を、論文（Scenario Modeling Language
Camera-Readyの Fig.2, Fig.4 など）やUMLアクティビティ図に近いスタイルで
静止画として可視化する。

- レーンごとに横一列のスイムレーンを作り（論文Fig.2の "Left Lane"/"Right Lane"
  に相当）、各レーン内では position の昇順に箱を左から右へ並べる。
- 箱は角丸の長方形で描き、"NPC(3)" のように car(box_id) のラベルを付ける
  （論文の "LCar(0)" 表記に合わせている）。
- ダミーの開始箱（複数の初期候補からの非決定的な出発を表すためのもの、
  10.2節参照）は、そのまま箱として描かず、UMLアクティビティ図の
  初期ノード（黒い塗りつぶし円）として描き、実際の初期候補箱へ矢印を出す。
"""

from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Circle

from gcpd import Model

# 日本語ラベル（タイトル等）が豆腐にならないよう、CJK対応フォントを優先する
matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

BoxKey = Tuple[int, int]  # (lane, position)


def plot_model_paper_style(
    model: Model,
    box_id_of: Dict[BoxKey, int],
    output_path: str,
    start_box: int = -1,
    car: Optional[str] = None,
    ego_lane: Optional[int] = None,
    title: str = "",
    box_w: float = 0.8,
    box_h: float = 0.6,
    lane_gap: float = 1.6,
    col_gap: float = 1.3,
) -> str:
    """論文のCPD図（Fig.2, Fig.4）に近いスイムレーン形式で model を描画する。"""
    car = car or model.cars[0]

    lane_of = {n: l for (c, n, l) in model.lane if c == car}
    pos_of = {n: p for (c, n, p) in model.position if c == car}

    lanes = sorted(set(lane_of.values()), reverse=True)  # 上から降順（左レーンが上に来やすい）
    row_of_lane = {l: i for i, l in enumerate(lanes)}

    # 各レーン内で position 昇順に等間隔の列位置(col)を割り当てる
    # (position の値そのものではなく順位を使うことで、間隔が空いていても詰めて描く)
    col_of_box: Dict[int, int] = {}
    for l in lanes:
        boxes_in_lane = sorted([n for n, lv in lane_of.items() if lv == l], key=lambda n: pos_of[n])
        for col, n in enumerate(boxes_in_lane):
            col_of_box[n] = col

    def xy(n: int) -> Tuple[float, float]:
        return col_of_box[n] * col_gap, -row_of_lane[lane_of[n]] * lane_gap

    n_cols = max(col_of_box.values(), default=0) + 1
    fig_w = max(6.0, 1.2 * n_cols)
    fig_h = max(3.0, 1.6 * len(lanes) + 1.5)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # スイムレーンの帯とラベル
    for l in lanes:
        y = -row_of_lane[l] * lane_gap
        ax.axhspan(y - lane_h_half(box_h), y + lane_h_half(box_h), color="#f5f7fb", zorder=0)
        label = f"lane={l}"
        if ego_lane is not None and l == ego_lane:
            label += " (ego lane)"
        ax.text(
            -col_gap * 0.9, y, label, ha="right", va="center", fontsize=10, color="#555555",
            bbox=dict(facecolor="#f5f7fb", edgecolor="none", pad=1.5), zorder=6,
        )

    # 箱を描く
    for n in col_of_box:
        x, y = xy(n)
        box = FancyBboxPatch(
            (x - box_w / 2, y - box_h / 2), box_w, box_h,
            boxstyle="round,pad=0.02,rounding_size=0.08",
            linewidth=1.2, edgecolor="#333333", facecolor="#ffffff", zorder=3,
        )
        ax.add_patch(box)
        ax.text(x, y, f"{car}({n})", ha="center", va="center", fontsize=8.5, zorder=4)

    # 遷移を矢印で描く（同じレーン内は水平、レーンをまたぐ場合はカーブさせる）
    for (c1, n1, c2, n2) in model.ntrans:
        if c1 != car or c2 != car or n1 == start_box or n2 == start_box:
            continue
        x1, y1 = xy(n1)
        x2, y2 = xy(n2)
        same_row = row_of_lane[lane_of[n1]] == row_of_lane[lane_of[n2]]
        rad = 0.0 if same_row else (0.15 if y2 > y1 else -0.15)
        arrow = FancyArrowPatch(
            (x1 + box_w / 2, y1), (x2 - box_w / 2, y2),
            arrowstyle="-|>", mutation_scale=11, color="#333333",
            linewidth=1.0, zorder=2, connectionstyle=f"arc3,rad={rad}",
            shrinkA=2, shrinkB=2,
        )
        ax.add_patch(arrow)

    # 初期状態: UMLアクティビティ図の初期ノード（黒丸）から、実際の初期候補箱へ矢印。
    # ダミー開始箱を使うモデル（10.2節）では model.inits は START_BOX 自身しか
    # 持たないため、実際の初期候補は「START_BOXから出ているntransの行き先」から求める。
    init_boxes = sorted({
        n2 for (c1, n1, c2, n2) in model.ntrans
        if c1 == car and c2 == car and n1 == start_box
    })
    if not init_boxes:
        init_boxes = [n for (c, n) in model.inits if c == car and n != start_box]
    if init_boxes:
        ix, iy = -col_gap * 3.6, sum(xy(n)[1] for n in init_boxes) / len(init_boxes)
        ax.add_patch(Circle((ix, iy), 0.09, color="black", zorder=5))
        for n in init_boxes:
            x, y = xy(n)
            arrow = FancyArrowPatch(
                (ix + 0.09, iy), (x - box_w / 2, y),
                arrowstyle="-|>", mutation_scale=11, color="#333333", linewidth=1.0, zorder=2,
                connectionstyle="arc3,rad=0.1", shrinkA=2, shrinkB=2,
            )
            ax.add_patch(arrow)
    elif start_box in col_of_box:
        pass  # (通常は start_box は col_of_box に含めていない)

    ax.set_xlim(-col_gap * 4.2, n_cols * col_gap)
    ax.set_ylim(-len(lanes) * lane_gap + lane_gap * 0.5, lane_gap * 0.5)
    ax.axis("off")
    ax.set_title(title or f"CPD model (car={car})", fontsize=12)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def plot_model_with_ego_paper_style(
    model: Model,
    box_id_of: Dict[BoxKey, int],
    output_path: str,
    start_box: int = -1,
    car: Optional[str] = None,
    ego_lane: Optional[int] = None,
    ego_car: str = "Ego",
    ego_max_step: Optional[int] = None,
    title: str = "",
    box_w: float = 0.8,
    box_h: float = 0.6,
    lane_gap: float = 1.6,
    col_gap: float = 1.3,
) -> str:
    """plot_model_paper_style に、11.5節で追加した Ego の箱列
    （logverify.ego_car.with_ego_track が同期遷移で結び付けているもの）を
    一番上のスイムレーンとして書き加えたもの。

    Ego(0)->Ego(1)->...->Ego(ego_max_step) という単純な一本道を、NPC側の
    スイムレーンとは別の帯（緑）として描く。実際の同期遷移（strans）は
    「NPCのntrans1本ごとにEgoの箱番号すべてとの組」を機械的に生成した
    ものなので、1本ずつ描くと膨大かつ視覚的に無意味な線になる。そのため
    ここでは同期の「事実」を注記テキストで示すに留め、矢印同士を線で
    結ぶことはしない。
    """
    car = car or model.cars[0]
    ms = ego_max_step if ego_max_step is not None else model.max_step

    lane_of = {n: l for (c, n, l) in model.lane if c == car}
    pos_of = {n: p for (c, n, p) in model.position if c == car}

    lanes = sorted(set(lane_of.values()), reverse=True)
    row_of_lane = {l: i for i, l in enumerate(lanes)}

    col_of_box: Dict[int, int] = {}
    for l in lanes:
        boxes_in_lane = sorted([n for n, lv in lane_of.items() if lv == l], key=lambda n: pos_of[n])
        for col, n in enumerate(boxes_in_lane):
            col_of_box[n] = col

    def xy(n: int) -> Tuple[float, float]:
        return col_of_box[n] * col_gap, -row_of_lane[lane_of[n]] * lane_gap

    ego_row_y = lane_gap  # NPC側の全レーンより1段上に配置する

    def ego_xy(i: int) -> Tuple[float, float]:
        return i * col_gap, ego_row_y

    n_cols = max(max(col_of_box.values(), default=0), ms) + 1
    fig_w = max(6.0, 1.2 * n_cols)
    fig_h = max(3.0, 1.6 * (len(lanes) + 1) + 1.5)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # Egoのスイムレーン（NPCとは別の色にして区別する）
    ax.axhspan(
        ego_row_y - lane_h_half(box_h), ego_row_y + lane_h_half(box_h), color="#eaf6ea", zorder=0
    )
    ax.text(
        -col_gap * 0.9, ego_row_y, f"{ego_car}（NPCと同期して前進）",
        ha="right", va="center", fontsize=10, color="#2a7a2a",
        bbox=dict(facecolor="#eaf6ea", edgecolor="none", pad=1.5), zorder=6,
    )
    for i in range(ms + 1):
        x, y = ego_xy(i)
        box = FancyBboxPatch(
            (x - box_w / 2, y - box_h / 2), box_w, box_h,
            boxstyle="round,pad=0.02,rounding_size=0.08",
            linewidth=1.2, edgecolor="#2a7a2a", facecolor="#eafaea", zorder=3,
        )
        ax.add_patch(box)
        ax.text(x, y, f"{ego_car}({i})", ha="center", va="center", fontsize=8.5, zorder=4)
    for i in range(ms):
        x1, y1 = ego_xy(i)
        x2, y2 = ego_xy(i + 1)
        arrow = FancyArrowPatch(
            (x1 + box_w / 2, y1), (x2 - box_w / 2, y2),
            arrowstyle="-|>", mutation_scale=11, color="#2a7a2a",
            linewidth=1.0, zorder=2, shrinkA=2, shrinkB=2,
        )
        ax.add_patch(arrow)
    ego_ix, ego_iy = -col_gap * 4.4, ego_row_y
    ax.add_patch(Circle((ego_ix, ego_iy), 0.09, color="black", zorder=5))
    ax.add_patch(FancyArrowPatch(
        (ego_ix + 0.09, ego_iy), (ego_xy(0)[0] - box_w / 2, ego_iy),
        arrowstyle="-|>", mutation_scale=11, color="#2a7a2a", linewidth=1.0, zorder=2,
        shrinkA=2, shrinkB=2,
    ))

    # --- ここから下は plot_model_paper_style と同じ（NPC側のスイムレーン） ---
    for l in lanes:
        y = -row_of_lane[l] * lane_gap
        ax.axhspan(y - lane_h_half(box_h), y + lane_h_half(box_h), color="#f5f7fb", zorder=0)
        label = f"lane={l}"
        if ego_lane is not None and l == ego_lane:
            label += " (ego lane)"
        ax.text(
            -col_gap * 0.9, y, label, ha="right", va="center", fontsize=10, color="#555555",
            bbox=dict(facecolor="#f5f7fb", edgecolor="none", pad=1.5), zorder=6,
        )

    for n in col_of_box:
        x, y = xy(n)
        box = FancyBboxPatch(
            (x - box_w / 2, y - box_h / 2), box_w, box_h,
            boxstyle="round,pad=0.02,rounding_size=0.08",
            linewidth=1.2, edgecolor="#333333", facecolor="#ffffff", zorder=3,
        )
        ax.add_patch(box)
        ax.text(x, y, f"{car}({n})", ha="center", va="center", fontsize=8.5, zorder=4)

    for (c1, n1, c2, n2) in model.ntrans:
        if c1 != car or c2 != car or n1 == start_box or n2 == start_box:
            continue
        x1, y1 = xy(n1)
        x2, y2 = xy(n2)
        same_row = row_of_lane[lane_of[n1]] == row_of_lane[lane_of[n2]]
        rad = 0.0 if same_row else (0.15 if y2 > y1 else -0.15)
        arrow = FancyArrowPatch(
            (x1 + box_w / 2, y1), (x2 - box_w / 2, y2),
            arrowstyle="-|>", mutation_scale=11, color="#333333",
            linewidth=1.0, zorder=2, connectionstyle=f"arc3,rad={rad}",
            shrinkA=2, shrinkB=2,
        )
        ax.add_patch(arrow)

    init_boxes = sorted({
        n2 for (c1, n1, c2, n2) in model.ntrans
        if c1 == car and c2 == car and n1 == start_box
    })
    if not init_boxes:
        init_boxes = [n for (c, n) in model.inits if c == car and n != start_box]
    if init_boxes:
        ix, iy = -col_gap * 4.4, sum(xy(n)[1] for n in init_boxes) / len(init_boxes)
        ax.add_patch(Circle((ix, iy), 0.09, color="black", zorder=5))
        for n in init_boxes:
            x, y = xy(n)
            arrow = FancyArrowPatch(
                (ix + 0.09, iy), (x - box_w / 2, y),
                arrowstyle="-|>", mutation_scale=11, color="#333333", linewidth=1.0, zorder=2,
                connectionstyle="arc3,rad=0.1", shrinkA=2, shrinkB=2,
            )
            ax.add_patch(arrow)

    fig.text(
        0.5, 0.008,
        f"※ {ego_car}の遷移は{car}の各遷移と同期（synchronous transition）しており、"
        f"{car}が距離帯・車線を変える瞬間に必ず1つ前進する（11.5節）。",
        ha="center", va="bottom", fontsize=8, color="#444444",
    )

    ax.set_xlim(-col_gap * 5.0, n_cols * col_gap)
    ax.set_ylim(-len(lanes) * lane_gap + lane_gap * 0.5, ego_row_y + lane_gap * 0.6)
    ax.axis("off")
    ax.set_title(title or f"CPD model ({car} + {ego_car}, synchronized)", fontsize=12, pad=14)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def lane_h_half(box_h: float) -> float:
    return box_h * 0.9
