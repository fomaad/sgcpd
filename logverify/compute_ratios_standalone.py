"""12.17節: AJISAI cut-inシナリオ全94本のような大量の生ログを、そのまま
クラウドコンテナへ転送せずに集計するためのスタンドアロン・スクリプト。

`logverify/synth_thresholds_multilog.py`の`log_level_deceleration_ratio`
（および内部で使う`vehicle_sizes`, `relative_xy`, `closest_approach_frame`,
`cutin_onset_frame`, `required_deceleration_magnitude`）と完全に同一の
ロジックを、他のモジュールに依存しない形で移植したもの。z3やmatplotlibは
使わない（それらはコンテナ側の`synth_thresholds_multilog_full.py`が担当
する）ため、生ログが置かれているマシン（クラウドサンドボックスのegress
制限でBoxに直接アクセスできない場合は、ユーザーのローカル計算機）上で
単独で実行できる。

出力は、ログ1本につき`{name, ratio, is_collision, closest_frame,
onset_frame}`だけを持つ小さなJSON配列であり、これをコンテナへ転送すれば
`synth_thresholds_multilog_full.py`でZ3による分離しきい値合成・可視化が
行える。

How to run / 実行方法:
    python3 compute_ratios_standalone.py <ログが入ったディレクトリ> [出力先.json]

    例 (12.17節で実際に使ったコマンド):
        python3 compute_ratios_standalone.py ~/mnt/Downloads/cutin/cutin \\
            ~/mnt/Downloads/cutin/cutin_ratios_full.json
"""

import glob
import json
import math
import sys


def _load(p):
    with open(p) as f:
        return json.load(f)


def vehicle_sizes(data):
    sizes = {v["name"]: v["size"] for v in data["groundtruth_size"]["vehicle_sizes"]}
    ego = sizes["ego"]
    npc = sizes.get("npc1", list(v for k, v in sizes.items() if k != "ego")[0])
    return (ego["x"] / 2, ego["y"] / 2), (npc["x"] / 2, npc["y"] / 2)


def relative_xy(data):
    gk = data["groundtruth_kinematic"]
    rxs, rys = [], []
    for rec in gk:
        ego = rec["groundtruth_ego"]
        ex, ey = ego["pose"]["position"]["x"], ego["pose"]["position"]["y"]
        yaw = math.radians(ego["pose"]["rotation"]["z"])
        fwd = (math.cos(yaw), math.sin(yaw))
        left = (-math.sin(yaw), math.cos(yaw))
        npc = next((v for v in rec.get("groundtruth_vehicles", []) if v["name"] == "npc1"), None)
        if npc is None:
            rxs.append(None)
            rys.append(None)
            continue
        dx = npc["pose"]["position"]["x"] - ex
        dy = npc["pose"]["position"]["y"] - ey
        rxs.append(dx * fwd[0] + dy * fwd[1])
        rys.append(dx * left[0] + dy * left[1])
    return rxs, rys


def closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w, near_rx=20.0):
    best_i, best_risk = None, None
    first_contact_i = None
    for i, (rx, ry) in enumerate(zip(rxs, rys)):
        if rx is None or abs(rx) > near_rx:
            continue
        risk = max(abs(rx) / (eh_l + nh_l), abs(ry) / (eh_w + nh_w))
        if risk < 1.0 and first_contact_i is None:
            first_contact_i = i
        if best_risk is None or risk < best_risk:
            best_risk, best_i = risk, i
    if first_contact_i is not None:
        return max(0, first_contact_i - 1), best_risk
    return best_i, best_risk


def cutin_onset_frame(rys, closest_frame, lookback=400):
    start = max(0, closest_frame - lookback)
    baseline = rys[start]
    for i in range(start, closest_frame):
        if abs(rys[i] - baseline) > 0.3 and all(
            abs(rys[j] - rys[j - 1]) < 0.05 or (rys[j] - rys[j - 1]) * (rys[i] - baseline) > 0
            for j in range(max(0, i - 5), i + 1)
        ):
            return i
    return max(0, closest_frame - 100)


def required_deceleration_magnitude(closing_speed, distance_to_contact):
    if closing_speed <= 0 or distance_to_contact <= 0:
        return math.inf
    return (closing_speed ** 2) / (2.0 * distance_to_contact)


def log_level_deceleration_ratio(json_path):
    data = _load(json_path)
    gk = data["groundtruth_kinematic"]
    cc = data["control_cmds"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)

    closest_frame, risk = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    if closest_frame is None:
        return None, False, None, None
    is_collision = risk is not None and risk < 1.0
    onset_frame = cutin_onset_frame(rys, closest_frame)

    eval_frame = closest_frame
    while eval_frame > onset_frame and rxs[eval_frame] - (eh_l + nh_l) <= 0:
        eval_frame -= 1

    t0, t1 = gk[onset_frame]["timestamp"], gk[eval_frame]["timestamp"]
    dt = t1 - t0
    if dt <= 0:
        return None, is_collision, closest_frame, onset_frame
    closing = (rxs[onset_frame] - rxs[eval_frame]) / dt
    dist = rxs[eval_frame] - (eh_l + nh_l)
    required = required_deceleration_magnitude(closing, dist)
    if not math.isfinite(required) or required <= 0:
        return None, is_collision, closest_frame, onset_frame
    accs = [abs(e["longitudinal"]["acceleration"]) for e in cc if t0 <= e["timestamp"] <= t1]
    if not accs:
        return None, is_collision, closest_frame, onset_frame
    achieved = sum(accs) / len(accs)
    return achieved / required, is_collision, closest_frame, onset_frame


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <log_dir> [output.json]", file=sys.stderr)
        sys.exit(1)
    log_dir = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else log_dir.rstrip("/") + "/../ratios.json"

    paths = sorted(glob.glob(log_dir.rstrip("/") + "/TD-NI-AR-SD-N04-CI-*.json"))
    paths = [p for p in paths if not p.endswith(".jama.json")]
    print(f"found {len(paths)} body logs", file=sys.stderr)
    results = []
    for i, p in enumerate(paths):
        name = p.split("/")[-1]
        try:
            ratio, is_collision, closest_frame, onset_frame = log_level_deceleration_ratio(p)
        except Exception as e:
            print(f"  ERROR {name}: {e}", file=sys.stderr)
            continue
        results.append(dict(name=name, ratio=ratio, is_collision=is_collision,
                             closest_frame=closest_frame, onset_frame=onset_frame))
        print(f"  [{i + 1}/{len(paths)}] {name}: ratio={ratio} collision={is_collision}", file=sys.stderr)
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"done: wrote {out_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
