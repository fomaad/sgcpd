"""12.25節の最終まとめ: (1)〜(5)それぞれについて「大事な境界（安全性
モデルのonsetフレーム）をつぶしていないか」を、smear量（onsetを含む
箱がpre-onset/post-onsetのフレームを混在させているrx方向の広がり、m。
0=つぶれていない）のグラフにする。

12.25節での訂正（ユーザーからの指摘）を反映し、"自分自身の安全性モデルに
対して"のsmearだけを意味のある比較として示す:
  - (1)(2) は特定の安全性モデルに紐付いていない汎用格子なので、C&C・RSS
    どちらのonsetに対するsmearも意味を持つ（そしてどちらに対しても
    大きくつぶれていることが、汎用格子の弱点として示される）。
  - (3) はC&C述語抽象化なので、C&C onsetに対するsmearだけを示す
    （構造上必ず0）。RSS onsetに対する値は、別モデルの文脈であり
    比較対象として意味がないため描かない。
  - (4) はRSS述語抽象化なので、RSS onsetに対するsmearだけを示す。
  - (5) は（訂正前の）C&C基準の格子なので、C&C onsetに対するsmearだけを
    示す。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.plot_five_abstractions_purity
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from logverify.plot_five_abstractions_summary import collect

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

OUT_PATH = "out_gif/five_abstractions_purity.png"


def plot(results, output_path=OUT_PATH):
    labels = [r[0] for r in results]
    cc_smear = [r[2] for r in results]   # None = 表示しない(別モデルの文脈で無意味)
    rss_smear = [r[3] for r in results]

    x = np.arange(len(labels))
    w = 0.35

    fig, ax = plt.subplots(figsize=(9.5, 5.5))

    cc_vals = [v if v is not None else 0 for v in cc_smear]
    rss_vals = [v if v is not None else 0 for v in rss_smear]
    cc_mask = [v is not None for v in cc_smear]
    rss_mask = [v is not None for v in rss_smear]

    bars_cc = ax.bar(
        [xi - w / 2 for xi, m in zip(x, cc_mask) if m], [v for v, m in zip(cc_vals, cc_mask) if m],
        width=w, color="#1565c0", label="JAMA C&C onsetでのsmear",
    )
    bars_rss = ax.bar(
        [xi + w / 2 for xi, m in zip(x, rss_mask) if m], [v for v, m in zip(rss_vals, rss_mask) if m],
        width=w, color="#ef6c00", label="RSS onsetでのsmear",
    )

    for xi, v, m in zip(x, cc_vals, cc_mask):
        if m:
            label = "0 (PURE)" if v == 0 else f"{v:.1f}m"
            ax.text(xi - w / 2, v + max(cc_vals + rss_vals) * 0.02, label, ha="center", fontsize=8.5)
    for xi, v, m in zip(x, rss_vals, rss_mask):
        if m:
            label = "0 (PURE)" if v == 0 else f"{v:.1f}m"
            ax.text(xi + w / 2, v + max(cc_vals + rss_vals) * 0.02, label, ha="center", fontsize=8.5)

    # (3)(4)で描かなかった側に「対象外」の注記
    for xi, m in zip(x, cc_mask):
        if not m:
            ax.text(xi - w / 2, max(cc_vals + rss_vals) * 0.02, "対象外", ha="center", fontsize=7.5, color="#9e9e9e", rotation=90, va="bottom")
    for xi, m in zip(x, rss_mask):
        if not m:
            ax.text(xi + w / 2, max(cc_vals + rss_vals) * 0.02, "対象外", ha="center", fontsize=7.5, color="#9e9e9e", rotation=90, va="bottom")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8.5)
    ax.set_ylabel("smear量 (m, rx方向。0=大事な境界をつぶしていない)")
    ax.set_title("(1)〜(5) 「大事な境界(onset)をつぶしていないか」の比較\n（自分自身が対象とする安全性モデルのonsetのみ評価。ログ0067）", fontsize=11.5)
    ax.legend(fontsize=9, loc="upper right")

    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output_path


if __name__ == "__main__":
    results = collect()
    # (5)はC&C基準の格子なので、C&C onsetに対するsmearのみを評価対象と
    # する（RSS onsetに対する値はcollect()内では参考として計算されて
    # いるが、(3)(4)と同じ理由で「対象外」として表示から外す）。
    label5, boxes5, cc5, rss5 = results[4]
    results[4] = (label5, boxes5, cc5, None)

    for label, boxes, cc, rss in results:
        print(label.replace("\n", " "), "boxes=", boxes, "cc_smear=", cc, "rss_smear=", rss)
    path = plot(results)
    print(f"図を書き出しました: {path}")
