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
"""

import time

from logverify.multi_log_model import (
    build_union_model,
    verify_logs_included,
    count_scenarios,
    enumerate_scenarios,
)


def _cutin(start_rx, rx_step, side_ry, length=6, merge_at=3):
    """start_rxから始まり、merge_atステップ目でego車線(ry=0)へ合流するログ。"""
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
    logs["cutin_right_near_1"] = _cutin(2.0, 2.0, -3.5, merge_at=2)
    logs["cutin_right_near_2"] = _cutin(2.0, 2.0, -3.5, merge_at=3)
    logs["cutin_right_medium_1"] = _cutin(20.0, 2.5, -3.5, merge_at=2)
    logs["cutin_right_medium_2"] = _cutin(20.0, 3.0, -3.5, merge_at=3)
    logs["cutin_right_far_1"] = _cutin(50.0, 3.0, -3.5, merge_at=2)
    logs["cutin_right_far_2"] = _cutin(50.0, 4.0, -3.5, merge_at=3)

    # 左隣接レーンからのcut-in（同様に近距離/中距離/遠距離）
    logs["cutin_left_near_1"] = _cutin(2.0, 2.0, 3.5, merge_at=2)
    logs["cutin_left_near_2"] = _cutin(2.0, 2.0, 3.5, merge_at=3)
    logs["cutin_left_medium_1"] = _cutin(20.0, 2.5, 3.5, merge_at=2)
    logs["cutin_left_far_1"] = _cutin(50.0, 3.0, 3.5, merge_at=2)

    # 自車線に留まり続けるログ（あえてrxの範囲をcut-inログと重複させる）
    logs["stays_in_lane_1"] = _stays_in_lane(2.0, 2.0)
    logs["stays_in_lane_2"] = _stays_in_lane(20.0, 2.5)
    logs["stays_in_lane_3"] = _stays_in_lane(50.0, 3.0)

    # 隣接レーンで並走してから加速して合流するログ
    logs["parallel_then_merge_right"] = _parallel_then_merge(2.0, 2.0, -3.5)
    logs["parallel_then_merge_left"] = _parallel_then_merge(20.0, 2.5, 3.5)

    # 合流後、再び隣接レーンへ抜けるログ（cut-in + cut-out）
    logs["cutin_then_cutout_right"] = _cutin_then_cutout(2.0, 2.0, -3.5)
    logs["cutin_then_cutout_left"] = _cutin_then_cutout(20.0, 2.5, 3.5)

    # 蛇行(swerve)のようなログ
    logs["swerve_1"] = _swerve(2.0, 2.0, 1.75)
    logs["swerve_2"] = _swerve(20.0, 2.5, 1.5)

    return logs


if __name__ == "__main__":
    logs = build_large_log_set()
    print(f"入力ログ本数: {len(logs)}")
    print()

    t0 = time.time()
    mlm = build_union_model(list(logs.values()))
    t1 = time.time()
    print(f"格子の自動選定 + 統合モデル構築にかかった時間: {t1 - t0:.2f}秒")
    print(f"選ばれた格子サイズ: gx={mlm.gx}, gy={mlm.gy}")
    print(f"箱の数: {len(mlm.model.boxes)}  (ダミー開始箱を含む)")
    print(f"max_step: {mlm.model.max_step}")
    print()

    print("--- 各ログの箱列（離散化結果） ---")
    for name, seq in zip(logs.keys(), mlm.sequences):
        print(f"{name:24s}: {seq}")
    print()

    t2 = time.time()
    results = verify_logs_included(mlm)
    t3 = time.time()
    print(f"--- 統合モデルに、各ログが含まれるか（membership check、{t3 - t2:.2f}秒） ---")
    n_ok = 0
    for name, r in zip(logs.keys(), results):
        ok = r.is_member
        n_ok += int(ok)
        print(f"{name:24s}: {'含まれる (SAT)' if ok else '含まれない (UNSAT) ← 想定外'}")
    print(f"-> {n_ok}/{len(logs)} 本が含まれることを確認")
    print()

    t4 = time.time()
    n_scenarios = count_scenarios(mlm)
    t5 = time.time()
    print(f"--- モデルから列挙されるシナリオの総数: {n_scenarios} (入力ログ数: {len(logs)}, {t5 - t4:.2f}秒) ---")
    if n_scenarios == len(logs):
        print("入力した各ログがそのまま1本ずつのシナリオとして再現されている（一般化なし）")
    else:
        print(
            f"入力ログの部分列が別の経路と合流し、入力にはなかった新しい経路も "
            f"{n_scenarios - len(logs)}本 生成された（一般化が起きた）"
        )
