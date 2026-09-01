"""方法C（logverify/multi_log_model.py）のデモ。

複数の合成ログを、それぞれを区別できる格子で1つのCPDモデルに統合し、
そのモデルからシナリオを列挙すると、入力した全てのログが
（元の経路として）含まれていることを確認する。

実行方法:
    cd sgcpd && python3 -m logverify.demo_multi_log_model

---
English:
Demo for Method C (logverify/multi_log_model.py).

Multiple synthetic logs are unified into a single CPD model using a grid
fine enough to distinguish each of them, and enumerating scenarios from
that model confirms that every input log is included (as one of the
original paths).

How to run:
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
    # (English) Merges in from the right-adjacent lane, starting at a near distance.
    return [(2.0, -3.5), (4.0, -3.5), (6.0, -1.75), (8.0, 0.0), (10.0, 0.0)]


def log_cutin_left_medium():
    # 左隣接レーン、中距離から合流
    # (English) Merges in from the left-adjacent lane, starting at a medium distance.
    return [(15.0, 3.5), (18.0, 3.5), (21.0, 1.75), (24.0, 0.0), (27.0, 0.0)]


def log_stays_in_lane():
    # ずっと自車線
    # (English) Stays in the ego lane the whole time.
    return [(5.0, 0.0), (10.0, 0.0), (15.0, 0.0), (20.0, 0.0)]


def log_cutin_right_far():
    # 右隣接レーン、遠距離から合流（log_cutin_right_near と方向は同じだが距離が違う）
    # (English) Merges in from the right-adjacent lane, starting at a far distance
    # (English) (same direction as log_cutin_right_near, but a different distance).
    return [(30.0, -3.5), (34.0, -3.5), (38.0, -1.75), (42.0, 0.0), (46.0, 0.0)]


if __name__ == "__main__":
    logs = {
        "cutin_right_near": log_cutin_right_near(),
        "cutin_left_medium": log_cutin_left_medium(),
        "stays_in_lane": log_stays_in_lane(),
        "cutin_right_far": log_cutin_right_far(),
    }

    mlm = build_union_model(list(logs.values()))
    print(f"Selected grid size: gx={mlm.gx}, gy={mlm.gy}")
    print(f"Number of boxes: {len(mlm.model.boxes)}  (including the dummy start box)")
    print()

    print("--- Box sequence for each log (discretization result) ---")
    for name, seq in zip(logs.keys(), mlm.sequences):
        print(f"{name:20s}: {seq}")
    print()

    print("--- Whether each log is included in the union model (membership check) ---")
    results = verify_logs_included(mlm)
    for name, r in zip(logs.keys(), results):
        print(f"{name:20s}: {'included (SAT)' if r.is_member else 'NOT included (UNSAT) <- unexpected'}")
    print()

    n_scenarios = count_scenarios(mlm)
    print(f"--- Total number of scenarios enumerated from the model: {n_scenarios} (number of input logs: {len(logs)}) ---")
    if n_scenarios == len(logs):
        print("Each input log is reproduced as exactly one scenario (no generalization occurred)")
    else:
        print("A subsequence of an input log merged with another path, possibly producing new paths not present in the input")
    print()

    scenarios = enumerate_scenarios(mlm)
    print("--- Enumerated scenarios (sequence of (lane, position)) ---")
    print("(Note: since gcpd.Model uses a single max_step shared by all logs, a path")
    print(" shorter than the others is padded by staying in its final box (the same box")
    print(" repeated) to equalize lengths. This is an apparent duplication caused by the")
    print(" model's constraints, not an indication that the state was actually visited twice.)")
    for idx, sc in enumerate(scenarios, 1):
        print(f"  Scenario {idx}: {[bk for (_, bk) in sc]}")
