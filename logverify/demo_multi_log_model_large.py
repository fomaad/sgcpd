"""
方法C（logverify/multi_log_model.py）を、より大きな規模（約20本のログ）で
試すデモ。

demo_multi_log_model.py の4本はどれも rx（縦方向の距離）の範囲が
互いに重ならないように選んであり、格子の自動細分化があまり試されて
いなかった。本デモでは、
  - cut-inの距離帯（近距離/中距離/遠距離）×左右レーン
  - 自車線に留まり続けるログ（複数本、あえて rx の範囲を重複させる）
  - 隣接レーンで並走してから加速して合流するログ
  - 一度合流してから再度隣接レーンへ抜けるログ（cut-in + cut-out）
  - 蛇行(swerve)のようなログ
を組み合わせて約20本の合成ログを作り、`find_distinguishing_grid` が
実際にどこまで格子を細かくする必要があるか、モデルのサイズ（箱数）が
どう増えるか、シナリオ列挙数が入力本数とどれだけ一致する／しないかを
確認する。

実データ（AJISAIログ）はこの環境では未取得のため、引き続き合成
トラジェクトリを使っている（11.2節の今後の課題）。

実行方法:
    cd sgcpd && python3 -m logverify.demo_multi_log_model_large

---
English:
A demo that exercises Method C (logverify/multi_log_model.py) at a larger
scale (about 20 logs).

The 4 logs in demo_multi_log_model.py were all chosen so that their rx
(longitudinal distance) ranges never overlap, so the grid's automatic
subdivision was not exercised much. This demo combines:
  - cut-ins at different distance bands (near/medium/far) x left/right lanes
  - logs that stay in the ego lane the whole time (several of them, with
    their rx ranges deliberately made to overlap)
  - logs that drive in parallel in the adjacent lane and then accelerate
    to merge in
  - logs that merge in and then cut back out to the adjacent lane again
    (cut-in + cut-out)
  - swerve-like logs
into roughly 20 synthetic logs, and checks how far `find_distinguishing_grid`
actually needs to refine the grid, how the model size (number of boxes)
grows, and how closely the number of enumerated scenarios matches (or
doesn't match) the number of input logs.

Real data (AJISAI logs) has not been obtained in this environment, so
synthetic trajectories are still used (a topic for future work, Section 11.2).

How to run:
    cd sgcpd && python3 -m logverify.demo_multi_log_model_large
"""

import os
import time

from logverify.multi_log_model import (
    build_union_model,
    build_union_model_near_far_grid,
    find_distinguishing_near_far_grid,
    verify_logs_included,
    count_scenarios,
    enumerate_scenarios,
)
from logverify.model_diagram import plot_model_with_ego_paper_style
from logverify.world_frame_gif import render_world_frame_gif

OUT_DIR = "out_gif"


def _cutin(start_rx, rx_step, side_ry, length=6, merge_at=3):
    """start_rxから始まり、merge_atステップ目でego車線(ry=0)へ合流するログ。

    (English) A log that starts at start_rx and merges into the ego lane
    (ry=0) at step merge_at.
    """
    rxs = [start_rx + i * rx_step for i in range(length)]
    rys = []
    for i in range(length):
        if i < merge_at:
            rys.append(side_ry)
        elif i == merge_at:
            rys.append(side_ry / 2)
        else:
            rys.append(0.0)
    return list(zip(rxs, rys))


def _stays_in_lane(start_rx, rx_step, length=5):
    rxs = [start_rx + i * rx_step for i in range(length)]
    return [(rx, 0.0) for rx in rxs]


def _parallel_then_merge(start_rx, rx_step, side_ry, parallel_len=4, merge_at=5, length=7):
    rxs = [start_rx + i * rx_step for i in range(length)]
    rys = []
    for i in range(length):
        if i < parallel_len:
            rys.append(side_ry)
        elif i < merge_at:
            rys.append(side_ry / 2)
        else:
            rys.append(0.0)
    return list(zip(rxs, rys))


def _cutin_then_cutout(start_rx, rx_step, side_ry, length=8):
    rxs = [start_rx + i * rx_step for i in range(length)]
    # 隣接レーン -> 合流 -> 自車線 -> 再度隣接レーンへ抜ける
    # (English) adjacent lane -> merge in -> ego lane -> cut back out to the adjacent lane
    pattern = [side_ry, side_ry, side_ry / 2, 0.0, 0.0, side_ry / 2, side_ry, side_ry]
    rys = pattern[:length]
    return list(zip(rxs, rys))


def _swerve(start_rx, rx_step, amplitude, length=7):
    rxs = [start_rx + i * rx_step for i in range(length)]
    pattern = [0.0, amplitude, amplitude, 0.0, -amplitude, -amplitude, 0.0]
    rys = pattern[:length]
    return list(zip(rxs, rys))


def build_large_log_set():
    logs = {}

    # 右隣接レーンからのcut-in（近距離/中距離/遠距離、それぞれ2本ずつ、
    # 速度〈rx_step〉やmerge_atのタイミングを変えてバリエーションを作る）
    # (English) Cut-ins from the right adjacent lane (near/medium/far distance,
    # two logs each, varying speed (rx_step) and merge_at timing for variety)
    logs["cutin_right_near_1"] = _cutin(2.0, 2.0, -3.5, merge_at=2)
    logs["cutin_right_near_2"] = _cutin(2.0, 2.0, -3.5, merge_at=3)
    logs["cutin_right_medium_1"] = _cutin(20.0, 2.5, -3.5, merge_at=2)
    logs["cutin_right_medium_2"] = _cutin(20.0, 3.0, -3.5, merge_at=3)
    logs["cutin_right_far_1"] = _cutin(50.0, 3.0, -3.5, merge_at=2)
    logs["cutin_right_far_2"] = _cutin(50.0, 4.0, -3.5, merge_at=3)

    # 左隣接レーンからのcut-in（同様に近距離/中距離/遠距離）
    # (English) Cut-ins from the left adjacent lane (near/medium/far distance, same as above)
    logs["cutin_left_near_1"] = _cutin(2.0, 2.0, 3.5, merge_at=2)
    logs["cutin_left_near_2"] = _cutin(2.0, 2.0, 3.5, merge_at=3)
    logs["cutin_left_medium_1"] = _cutin(20.0, 2.5, 3.5, merge_at=2)
    logs["cutin_left_far_1"] = _cutin(50.0, 3.0, 3.5, merge_at=2)

    # 自車線に留まり続けるログ（あえてrxの範囲をcut-inログと重複させる）
    # (English) Logs that stay in the ego lane the whole time (rx ranges are
    # deliberately made to overlap with the cut-in logs)
    logs["stays_in_lane_1"] = _stays_in_lane(2.0, 2.0)
    logs["stays_in_lane_2"] = _stays_in_lane(20.0, 2.5)
    logs["stays_in_lane_3"] = _stays_in_lane(50.0, 3.0)

    # 隣接レーンで並走してから加速して合流するログ
    # (English) Logs that drive in parallel in the adjacent lane and then accelerate to merge in
    logs["parallel_then_merge_right"] = _parallel_then_merge(2.0, 2.0, -3.5)
    logs["parallel_then_merge_left"] = _parallel_then_merge(20.0, 2.5, 3.5)

    # 合流後、再び隣接レーンへ抜けるログ（cut-in + cut-out）
    # (English) Logs that merge in and then cut back out to the adjacent lane (cut-in + cut-out)
    logs["cutin_then_cutout_right"] = _cutin_then_cutout(2.0, 2.0, -3.5)
    logs["cutin_then_cutout_left"] = _cutin_then_cutout(20.0, 2.5, 3.5)

    # 蛇行(swerve)のようなログ
    # (English) Swerve-like logs
    logs["swerve_1"] = _swerve(2.0, 2.0, 1.75)
    logs["swerve_2"] = _swerve(20.0, 2.5, 1.5)

    return logs


if __name__ == "__main__":
    logs = build_large_log_set()
    print(f"Number of input logs: {len(logs)}")
    print()

    t0 = time.time()
    mlm = build_union_model(list(logs.values()))
    t1 = time.time()
    print(f"Time to auto-select grid + build union model: {t1 - t0:.2f}s")
    print(f"Selected grid size: gx={mlm.gx}, gy={mlm.gy}")
    print(f"Number of boxes: {len(mlm.model.boxes)}  (including the dummy start box)")
    print(f"max_step: {mlm.model.max_step}")
    print()

    print("--- Box sequence (discretization result) for each log ---")
    for name, seq in zip(logs.keys(), mlm.sequences):
        print(f"{name:24s}: {seq}")
    print()

    t2 = time.time()
    results = verify_logs_included(mlm)
    t3 = time.time()
    print(f"--- Whether each log is included in the union model (membership check, {t3 - t2:.2f}s) ---")
    n_ok = 0
    for name, r in zip(logs.keys(), results):
        ok = r.is_member
        n_ok += int(ok)
        print(f"{name:24s}: {'included (SAT)' if ok else 'not included (UNSAT) <- unexpected'}")
    print(f"-> {n_ok}/{len(logs)} logs confirmed included")
    print()

    t4 = time.time()
    n_scenarios = count_scenarios(mlm)
    t5 = time.time()
    print(f"--- Total number of scenarios enumerated from the model: {n_scenarios} (input logs: {len(logs)}, {t5 - t4:.2f}s) ---")
    if n_scenarios == len(logs):
        print("Each input log is reproduced as exactly one scenario (no generalization)")
    else:
        print(
            f"Subsequences of the input logs merged with other paths, producing "
            f"{n_scenarios - len(logs)} new paths not present in the input (generalization occurred)"
        )
    print()

    # --- Egoに近い距離帯は今まで通り(5m)、遠い距離帯はまとめる(10m)、
    # 非一様な格子での統合モデル（11.6節：格子設計の見直し） ---
    # (English) Union model on a non-uniform grid: distance bands close to
    # ego stay as before (5m), while far distance bands are merged (10m)
    # (Section 11.6: revisiting the grid design)
    print("=== Union model on a non-uniform grid: fine near ego, coarse far from ego ===")
    t6 = time.time()
    # rx_far_cell は指定しない（デフォルトのrx_near_cell*2から出発し、
    # auto_grid=Trueにより全ログを区別できるまで自動的に細分化される）。
    # rx_near_range=45m は、中距離帯のログ同士が区別できなくなる境界を
    # 実際に確認しながら選んだ値（11.7節）。
    # (English) rx_far_cell is left unspecified (it starts from the default
    # rx_near_cell*2, and auto_grid=True automatically refines it until all
    # logs can be distinguished). rx_near_range=45m was chosen by actually
    # checking where medium-distance logs stop being distinguishable from
    # one another (Section 11.7).
    far_cell_used, _ = find_distinguishing_near_far_grid(
        list(logs.values()), rx_near_cell=5.0, rx_near_range=45.0, gy=3.5
    )
    mlm_nf = build_union_model_near_far_grid(
        list(logs.values()),
        rx_near_cell=5.0,
        rx_far_cell=far_cell_used,
        rx_near_range=45.0,
        gy=3.5,
        auto_grid=False,  # 上ですでに区別できる遠方セルサイズを見つけているので再探索不要
        # (English) no need to re-search: a distinguishing far-cell size was already found above
    )
    t7 = time.time()
    print(f"Time to build the union model: {t7 - t6:.2f}s")
    print(f"Auto-selected far cell size: {far_cell_used}m  (near cell size: 5.0m, boundary: 45.0m)")
    print(f"Number of boxes: {len(mlm_nf.model.boxes)}  (was {len(mlm.model.boxes)} with the uniform grid)")
    print(f"max_step: {mlm_nf.model.max_step}  (was {mlm.model.max_step} with the uniform grid)")

    t8 = time.time()
    results_nf = verify_logs_included(mlm_nf)
    t9 = time.time()
    n_ok_nf = sum(r.is_member for r in results_nf)
    print(f"membership check: {n_ok_nf}/{len(logs)} logs SAT ({t9 - t8:.2f}s)")

    t10 = time.time()
    n_scenarios_nf = count_scenarios(mlm_nf)
    t11 = time.time()
    print(f"Number of enumerated scenarios: {n_scenarios_nf} (was {n_scenarios} with the uniform grid, {t11 - t10:.2f}s)")
    print()

    # --- Egoを同期遷移(strans)で実際にCPDのcarとして追加したワールド座標系
    # アニメーション。列挙する必要があるのは「絵にする分の数個」だけでよく、
    # num_modelを小さく絞ることで、この規模のモデルでも実用的な時間で描画できる
    # （11.6節：全列挙ではなくmembership checkだけならそもそも列挙不要、という
    # 指摘を受けての見直し。アニメーションは可視化目的で数個あれば十分）。---
    # (English) A world-frame animation with ego actually added as a CPD car
    # via a synchronized transition (strans). Only as many scenarios as are
    # needed for the pictures need to be enumerated, and keeping num_model
    # small lets even a model of this size render in a practical amount of
    # time (Section 11.6: revised after noting that enumeration isn't needed
    # at all if only a membership check is required, not full enumeration;
    # a handful of scenarios is enough for visualization purposes).
    print("=== World-frame animation with ego synchronized (enumerating only a few) ===")
    os.makedirs(OUT_DIR, exist_ok=True)
    t12 = time.time()
    paths = render_world_frame_gif(
        mlm_nf.model,
        os.path.join(OUT_DIR, "multilog_large_with_ego"),
        combined=True,
        ego_speed=1.0,
        num_model=5,
    )
    t13 = time.time()
    print(f"Ego-synchronized animation (num_model=5): {t13 - t12:.2f}s -> {paths}")

    path_diagram = plot_model_with_ego_paper_style(
        mlm_nf.model,
        mlm_nf.box_id_of,
        os.path.join(OUT_DIR, "model_multilog_large_near_far_with_ego.png"),
        car="NPC",
        ego_lane=0,
        ego_max_step=mlm_nf.model.max_step,
        title="Unified CPD from 19 logs (near/far grid) + Ego (synchronized) - Method C",
    )
    print("Model structure diagram:", path_diagram)
