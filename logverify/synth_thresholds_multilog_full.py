"""94本のcut-inログ全体（AJISAIデータセット、TD-NI-AR-SD-N04-CI-0001~0094）を
教師データとして、12.16節の分離しきい値合成を再実行する（規模拡大版）。

## 経緯

12.16節では、セッションにたまたま存在した6本のログ（衝突1本・非衝突5本）
だけで、Z3による分離しきい値合成が「きれいに」成功した（しきい値0.2006、
マージン0.1964）。しかしユーザーからの指摘「AJISAIにアクセスして、もう
少し、規模の大きい実験はできますか？」を受け、cut-inシナリオの全94本
（AJISAIデータセット全体、衝突15本・16.0%）を対象に同じ分析を再実行した。

本スクリプトは、94本分の生ログ（合計2.1GB、クラウドサンドボックスの
Box egress制限のためユーザーのローカル計算機上でのみアクセス可能）を
このコンテナに転送するのではなく、`compute_ratios.py`
（`logverify/synth_thresholds_multilog.py`の`log_level_deceleration_ratio`と
完全に同一のロジックを移植したスタンドアロン版）をユーザーのローカル
計算機上で実行し、ログ1本につき1つの数値（減速の十分性比、衝突フラグ）
だけを抽出した小さなJSON（94エントリ、約15KB）をこのコンテナに転送して
分析する、という設計にした。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.synth_thresholds_multilog_full
"""

import json

import matplotlib
import z3

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from logverify.synth_thresholds_multilog import synthesize_separating_threshold

matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

RATIOS_PATH = "/mnt/user-data/uploads/Downloads/cutin/cutin_ratios_full.json"
OUT_PATH = "out_gif/synth_thresholds_multilog_full.png"


def plot_separation_full(rows, synth_threshold, output_path=OUT_PATH, xlim=None) -> str:
    fig, ax = plt.subplots(figsize=(10, 4.2))
    for name, ratio, is_collision in rows:
        if ratio is None:
            continue
        if xlim is not None and not (xlim[0] <= ratio <= xlim[1]):
            continue
        color = "#e53935" if is_collision else "#43a047"
        marker = "X" if is_collision else "o"
        ax.scatter([ratio], [0], s=90, color=color, marker=marker, zorder=3,
                   edgecolor="black", linewidth=0.5, alpha=0.85)
    if xlim is not None:
        ax.set_xlim(*xlim)

    if synth_threshold is not None:
        ax.axvline(synth_threshold, color="#1565c0", linestyle="-", linewidth=1.4, zorder=2)
        ax.text(synth_threshold, 0.55, f"Z3合成: {synth_threshold:.4f}", color="#1565c0", fontsize=8,
                ha="center", rotation=90, va="bottom")
    ax.axvline(0.5, color="#9e9e9e", linestyle="--", linewidth=1.0, zorder=1)
    ax.text(0.5, -0.55, "既定 weak_ratio=0.5", color="#757575", fontsize=8, ha="center", rotation=90, va="top")
    ax.axvline(1.0, color="#9e9e9e", linestyle=":", linewidth=1.0, zorder=1)
    ax.text(1.0, -0.55, "既定 adequate_ratio=1.0", color="#757575", fontsize=8, ha="center", rotation=90, va="top")

    ax.set_xscale("log")
    ax.set_xlabel("減速の十分性比 achieved/required (対数軸)")
    ax.set_yticks([])
    ax.set_ylim(-1.0, 1.0)
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)
    n_coll = sum(1 for _, r, c in rows if r is not None and c)
    n_safe = sum(1 for _, r, c in rows if r is not None and not c)
    ax.set_title(
        f"cut-inシナリオ全94本（AJISAI、衝突{n_coll}本・非衝突{n_safe}本）における減速比の分布",
        fontsize=11)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def run() -> None:
    with open(RATIOS_PATH) as f:
        data = json.load(f)
    print(f"対象ログ: {len(data)}本")

    rows = [(r["name"], r["ratio"], r["is_collision"]) for r in data]
    collision_ratios = [r["ratio"] for r in data if r["is_collision"] and r["ratio"] is not None]
    safe_ratios = [r["ratio"] for r in data if not r["is_collision"] and r["ratio"] is not None]
    n_collision_total = sum(1 for r in data if r["is_collision"])
    undefined_collision = [r["name"] for r in data if r["is_collision"] and r["ratio"] is None]

    print(f"衝突ログ数: {n_collision_total}本 (うち比が定義できたもの: {len(collision_ratios)}本)")
    if undefined_collision:
        print(f"  比が定義できなかった衝突ログ: {undefined_collision}")
    print(f"非衝突ログ数: {len(safe_ratios)}本")
    print()

    print(f"衝突ログの比の範囲: [{min(collision_ratios):.6f}, {max(collision_ratios):.6f}]")
    print(f"非衝突ログの比の範囲: [{min(safe_ratios):.6f}, {max(safe_ratios):.6f}]")
    print()

    # Sort both lists and report the overlap region explicitly.
    overlap = [(n, r, c) for n, r, c in rows if r is not None and r <= max(collision_ratios)
               and not c]
    print(f"衝突ログの最大比({max(collision_ratios):.6f})以下の非衝突ログ: "
          f"{[(n, round(r, 6)) for n, r, c in overlap]}")
    print()

    print("=== Z3で、衝突ログと非衝突ログを分離するしきい値を合成 ===")
    result, error = synthesize_separating_threshold(collision_ratios, safe_ratios)
    if result is None:
        print(f"分離失敗: {error}")
        print("-> 6本のログ(12.16節)では「きれいに」分離できていたが、94本全体では"
              "単一の特徴量（減速の十分性比）だけでは衝突/非衝突を完全には分離できない"
              "ことが分かった。これは12.16節の結果が小標本（1衝突例）に対する過学習で"
              "あったことを意味し、重要な負の結果である。")
        path = plot_separation_full(rows, synth_threshold=None)
        print(f"図を書き出しました: {path}")
        zoom_path = plot_separation_full(
            rows, synth_threshold=None, output_path="out_gif/synth_thresholds_multilog_full_zoom.png",
            xlim=(1e-4, 1e2))
        print(f"拡大図を書き出しました: {zoom_path}")
        return

    print(f"合成されたしきい値: {result['threshold']:.4f} (マージン={result['margin']:.4f})")
    path = plot_separation_full(rows, synth_threshold=result["threshold"])
    print(f"図を書き出しました: {path}")

    zoom_path = plot_separation_full(
        rows, synth_threshold=None, output_path="out_gif/synth_thresholds_multilog_full_zoom.png",
        xlim=(1e-4, 1e2))
    print(f"拡大図を書き出しました: {zoom_path}")


if __name__ == "__main__":
    run()
