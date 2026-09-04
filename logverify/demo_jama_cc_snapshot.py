"""12.19節のJAMA C&Cモデルによる抽象化を、12.14節と同じ「スナップショット
列 = CPDの箱列」の形式で可視化する。

ユーザーからの依頼: 「抽象化した後のログ，つまり，モデルが気になるので，
そちらを可視化してください」。すなわち、12.19節が数値・グラフ（TTCの
時系列、反実仮想の縦距離の時系列）として示した結果を、12.14節のCPD箱列
スタイルの図——各箱がEGO/NPCの位置関係を模式的に描き、その箱を代表する
時刻での抽象値（ラベル）を併記するという形式——に落とし込んだものが、
本モジュールが出力する図である。

各パネル（=CPDの箱）には以下を重ねて描く:

- 実際のEGO・NPCの位置関係（12.14節と同じ、Ve0・Vo0・Vy・dx0・dy0付き）。
- **青い破線のゴースト矩形**: risk知覚後の箱について、JAMA C&Cモデルの
  反実仮想（もし有能で慎重な人間ドライバだったら、その箱の代表時刻で
  NPCとの縦方向距離はどこにあったか）。実際のNPC矩形との重なり具合が、
  「実際の挙動は基準からどれだけ乖離していたか」を一目で示す。
- **TTCラベル**: 12.18節のTTCによる抽象値(safe/caution/danger)。
- 12.12/12.15節の減速・予測・余裕ラベル（従来通り）。

risk知覚フレームより前の箱では、反実仮想はまだ実際の軌道と同一
（両者とも未反応）と定義されるため、ゴースト矩形は実際のNPC矩形と重なり
描き分けられない——これも「まだリスクが知覚されていない」ことを暗に
示している。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_jama_cc_snapshot \
        [path-to-TD-NI-AR-SD-N04-CI-0067.json]
"""

import sys

from logverify.abstract_cause import (
    classify_contact_margin,
    required_deceleration_magnitude,
)
from logverify.auto_grid import auto_grid_params_from_ajisai
from logverify.demo_scenario_snapshot import (
    _load,
    decel_label_at,
    detect_cutin_onset_frame,
    find_collision_frames,
    lateral_speed_at,
    npc_speed_at,
    pred_label_at,
    speed_at,
    vehicle_sizes,
)
from logverify.grid_bridge import (
    compress_to_grid_states_variable_hysteresis,
    relative_xy_from_ajisai_groundtruth,
)
from logverify.jama_cc_model import find_risk_perceived_frame, simulate_cc_reference
from logverify.reference_model_comparison import compute_ttc, ego_speed_series, ttc_zone
from logverify.scenario_snapshot_diagram import ScenarioSnapshot, plot_scenario_snapshot_sequence

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_PATH = "out_gif/jama_cc_scenario_snapshots.png"


def run(json_path: str) -> None:
    print(f"Loading: {json_path}")
    data = _load(json_path)
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path)
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    gk = data["groundtruth_kinematic"]
    cc = data["control_cmds"]
    po = data["perception_objects"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    timestamps = [rec["timestamp"] for rec in gk]

    coll_frames = find_collision_frames(rel_xy, eh_l, eh_w, nh_l, nh_w)
    coll_set = set(coll_frames)
    window_first, window_last = coll_frames[0], coll_frames[-1]
    onset_frame = detect_cutin_onset_frame(rys, window_first)
    onset_ts = gk[onset_frame]["timestamp"]
    window_ts0, window_ts1 = gk[window_first]["timestamp"], gk[window_last]["timestamp"]

    # 12.19節と同じく、TTC=2.0秒境界・横方向0.72m境界のいずれか早い方で
    # risk知覚フレームを求め、そこからJAMA C&Cモデルの反実仮想軌道を
    # シミュレートする。
    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)
    ego_speed_full = ego_speed_series(gk)
    risk_frame, lateral_frame, ttc_frame = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    print(f"risk知覚フレーム: {risk_frame} (横方向={lateral_frame}, TTC={ttc_frame})")
    rx_ref = simulate_cc_reference(gk, rxs, ego_speed_full, risk_frame)

    t_lo = min(onset_ts, timestamps[risk_frame]) - 1.0
    t_hi = window_ts1 + 1.0
    frame_lo = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - t_lo))
    frame_hi = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - t_hi))
    print(f"表示範囲: t={t_lo:.2f}s〜{t_hi:.2f}s (frame {frame_lo}-{frame_hi})")

    print("=== 車両サイズから格子パラメータを自動導出（12.11節） ===")
    auto = auto_grid_params_from_ajisai(json_path)
    states = compress_to_grid_states_variable_hysteresis(
        rxs, rys, auto.rx_near_cell, auto.rx_far_cell, auto.rx_near_range, auto.gy, margin_ratio=0.3,
    )
    sub_states = [s for s in states if s.end_frame >= frame_lo and s.start_frame <= frame_hi]
    print(f"CPDの箱数（全体）: {len(states)}, 表示範囲内の箱数: {len(sub_states)}")

    snapshots = []
    for s in sub_states:
        frame = (s.start_frame + s.end_frame) // 2
        ts = gk[frame]["timestamp"]
        rx, ry = rel_xy[frame]
        ego_speed = speed_at(gk[frame], "groundtruth_ego")
        npc_speed = npc_speed_at(gk[frame])
        vy = lateral_speed_at(rys, gk, frame)
        decel_label = decel_label_at(rxs, gk, cc, frame, eh_l, nh_l)
        pred_label = pred_label_at(po, gk, rys, ts)
        contact_label = classify_contact_margin(ry, eh_w, nh_w, is_colliding=(frame in coll_set))
        ttc_label = ttc_zone(ttcs[frame]) if frame < len(ttcs) else None
        rx_cc_ref = rx_ref[frame] if frame >= risk_frame and rx_ref[frame] is not None else None
        snapshots.append(ScenarioSnapshot(
            box_index=s.index, t=ts, rx=rx, ry=ry,
            ego_speed=ego_speed, npc_speed=npc_speed, npc_lateral_speed=vy,
            decel_label=decel_label, pred_label=pred_label, contact_label=contact_label,
            lane_k=s.k, pos_i=s.i,
            ttc_label=ttc_label, rx_cc_ref=rx_cc_ref,
        ))
        marker = " <- risk知覚後" if frame >= risk_frame else ""
        print(f"  box#{s.index} (k={s.k},i={s.i}) frame={frame} t={ts - onset_ts:+.2f}s "
              f"rx={rx:.2f} rx_ref={rx_cc_ref if rx_cc_ref is not None else '-'} "
              f"TTC:{ttc_label} 減速:{decel_label} 余裕:{contact_label}{marker}")

    path = plot_scenario_snapshot_sequence(
        snapshots, OUT_PATH,
        ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
        title="TD-NI-AR-SD-N04-CI-0067: 抽象化後のモデル（TTC + JAMA C&C反実仮想、= CPDの箱列）",
        t_ref=onset_ts,
    )
    print(f"図を書き出しました: {path}")


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
