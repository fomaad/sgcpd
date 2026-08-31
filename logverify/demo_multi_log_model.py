"""
方法C（logverify/multi_log_model.py）のデモ。

複数の合成ログを、それぞれを区別できる格子で1つのCPDモデルに統合し、
そのモデルからシナリオを列挙すると、入力した全てのログが
（元の経路として）含まれていることを確認する。

実行方法:
    cd sgcpd && python3 -m logverify.demo_multi_log_model
"""

from logverify.multi_log_model import (
    build_union_model,
    verify_logs_included,
    count_scenarios,
    enumerate_scenarios,
)


def log_cutin_right_near():
    # 右隣接レーン、近距離から合流
    return [(2.0, -3.5), (4.0, -3.5), (6.0, -1.75), (8.0, 0.0), (10.0, 0.0)]


def log_cutin_left_medium():
    # 左隣接レーン、中距離から合流
    return [(15.0, 3.5), (18.0, 3.5), (21.0, 1.75), (24.0, 0.0), (27.0, 0.0)]


def log_stays_in_lane():
    # ずっと自車線
    return [(5.0, 0.0), (10.0, 0.0), (15.0, 0.0), (20.0, 0.0)]


def log_cutin_right_far():
    # 右隣接レーン、遠距離から合流（log_cutin_right_near と方向は同じだが距離が違う）
    return [(30.0, -3.5), (34.0, -3.5), (38.0, -1.75), (42.0, 0.0), (46.0, 0.0)]


if __name__ == "__main__":
    logs = {
        "cutin_right_near": log_cutin_right_near(),
        "cutin_left_medium": log_cutin_left_medium(),
        "stays_in_lane": log_stays_in_lane(),
        "cutin_right_far": log_cutin_right_far(),
    }

    mlm = build_union_model(list(logs.values()))
    print(f"選ばれた格子サイズ: gx={mlm.gx}, gy={mlm.gy}")
    print(f"箱の数: {len(mlm.model.boxes)}  (ダミー開始箱を含む)")
    print()

    print("--- 各ログの箱列（離散化結果） ---")
    for name, seq in zip(logs.keys(), mlm.sequences):
        print(f"{name:20s}: {seq}")
    print()

    print("--- 統合モデルに、各ログが含まれるか（membership check） ---")
    results = verify_logs_included(mlm)
    for name, r in zip(logs.keys(), results):
        print(f"{name:20s}: {'含まれる (SAT)' if r.is_member else '含まれない (UNSAT) ← 想定外'}")
    print()

    n_scenarios = count_scenarios(mlm)
    print(f"--- モデルから列挙されるシナリオの総数: {n_scenarios} (入力ログ数: {len(logs)}) ---")
    if n_scenarios == len(logs):
        print("入力した各ログがそのまま1本ずつのシナリオとして再現されている（一般化なし）")
    else:
        print("入力ログの部分列が別の経路と合流し、入力にはなかった新しい経路も生成された可能性がある")
    print()

    scenarios = enumerate_scenarios(mlm)
    print("--- 列挙された各シナリオ（(lane, position)の列） ---")
    print("(注: gcpd.Modelは全ログ共通の1つのmax_stepを持つため、元のログより短い経路は")
    print(" 末尾の箱で足踏み（同じ箱が連続）して長さを揃える。これはモデルの制約上の")
    print(" 見た目上の重複であり、実際に2回その状態を通ったという意味ではない。)")
    for idx, sc in enumerate(scenarios, 1):
        print(f"  シナリオ{idx}: {[bk for (_, bk) in sc]}")
