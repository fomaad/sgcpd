"""12.25節: compare_safety_model_abstractions.pyの結果を可視化する。

左: 箱数(抽象化の度合い、少ないほど粗い)。
右: 各safety model(JAMA C&C / RSS)のonsetフレームでのsmear量
   （箱がpre-onsetとpost-onsetを混在させているrx方向の広がり、m。
   pureなら0）。

---
English: Visualizes the results of compare_safety_model_abstractions.py.
Left: box count (degree of abstraction; fewer = coarser).
Right: smear amount (meters of rx-span over which a box mixes pre- and
post-onset frames) at each safety model's (JAMA C&C / RSS) onset frame;
0 if pure.

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.plot_safety_model_abstraction_comparison
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from logverify.compare_safety_model_abstractions import run

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

OUT_PATH = "out_gif/safety_model_abstraction_comparison.png"


def plot(results, output_path=OUT_PATH):
    labels = [r.label_ja for r in results]
    n_boxes = [r.n_boxes for r in results]
    smear_cc = [r.purity["jama_cc"].get("box_rx_span_m", 0.0) if not r.purity["jama_cc"].get("pure", True) else 0.0
                for r in results]
    smear_rss = [r.purity["rss"].get("box_rx_span_m", 0.0) if not r.purity["rss"].get("pure", True) else 0.0
                 for r in results]

    colors = ["#78909c", "#1565c0", "#ef6c00", "#757575"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.8))

    ax1.bar(labels, n_boxes, color=colors)
    for i, v in enumerate(n_boxes):
        ax1.text(i, v + 2, str(v), ha="center", fontsize=9)
    ax1.set_ylabel("箱(スナップショット)の数")
    ax1.set_title("抽象化の度合い（少ないほど粗い）", fontsize=10.5)
    ax1.tick_params(axis="x", labelrotation=20, labelsize=8)

    x = np.arange(len(labels))
    w = 0.35
    ax2.bar(x - w / 2, smear_cc, width=w, color="#1565c0", label="JAMA C&C onsetでのsmear")
    ax2.bar(x + w / 2, smear_rss, width=w, color="#ef6c00", label="RSS onsetでのsmear")
    for i, v in enumerate(smear_cc):
        if v > 0:
            ax2.text(i - w / 2, v + 0.5, f"{v:.1f}", ha="center", fontsize=7.5)
    for i, v in enumerate(smear_rss):
        if v > 0:
            ax2.text(i + w / 2, v + 0.5, f"{v:.1f}", ha="center", fontsize=7.5)
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=20, fontsize=8)
    ax2.set_ylabel("smear量 (m, rx方向。0=pure)")
    ax2.set_title("大事な境界をつぶしていないか（0m=その安全性モデルの\nonsetを箱の境界がちょうど分離できている）", fontsize=10.5)
    ax2.legend(fontsize=8)

    fig.suptitle("safety-model-guided abstractionの比較（ログ0067、1本のみの試行）", fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output_path


if __name__ == "__main__":
    results = run()
    path = plot(results)
    print(f"図を書き出しました: {path}")
