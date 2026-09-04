"""12.19節のJAMA C&Cモデルによる抽象化を、12.14節と同じ「スナップショット
列 = CPDの箱列」の形式で可視化する（12.21節で改訂）。

ユーザーからの依頼の変遷:

1. 「抽象化した後のログ，つまり，モデルが気になるので，そちらを
   可視化してください」（12.20節）— 12.19節が数値・グラフとして示した
   結果を、12.14節のCPD箱列スタイルの図に落とし込む。
2. 「TTCを消してください．C&C部分を強調してください．また，対応する
   CPDモデルも出力できるようにしてください．」（12.21節、本節）—
   (a) TTCラベルの表示をやめる（TTCはJAMA C&Cモデル自身のrisk知覚
       トリガーの一部として内部的には使われ続けるが、図の抽象値としては
       もう表示しない。12.20節末の分析で、TTCは衝突直後に「safe」へ
       戻ってしまうという弱点が単独の抽象値としては誤解を招くと判断
       したため）。
   (b) JAMA C&Cモデルの反実仮想（ゴースト矩形）をより強調する（太線・
       ハッチング・ラベル・実際位置との差を示す矢印などを
       scenario_snapshot_diagram.py 側に追加）。
   (c) 「対応するCPDモデル」——すなわち模式図ではなく、実際に
       gcpd.Model として構築された形式的なCPD（Z3のBox/Pos/Lane関数と
       遷移関係）——も出力できるようにする。スナップショット列と同じ
       格子・同じヒステリシス処理（`auto_grid_params_from_ajisai` +
       `compress_to_grid_states_variable_hysteresis`, margin_ratio=0.3）
       で箱列を作っているログ1本から、`multi_log_model.
       build_single_log_model_hysteresis` を使って同じ箱列を持つ
       `gcpd.Model` を構築し、`model_diagram.
       plot_model_with_ego_paper_style` で12.7/12.8/12.14節と同じ
       箱矢印図として描く。スナップショット列の各パネルの
       box_index・(k,i) は、このCPDモデル図の箱番号と1対1に対応する
       （「対応する」の意図はここにある）。

各パネル（=CPDの箱）には以下を重ねて描く:

- 実際のEGO・NPCの位置関係（12.14節と同じ、Ve0・Vo0・Vy・dx0・dy0付き）。
- **ハッチング入りのゴースト矩形**: risk知覚後の箱について、JAMA C&C
  モデルの反実仮想（もし有能で慎重な人間ドライバだったら、その箱の
  代表時刻でNPCとの縦方向距離はどこにあったか）。実際のNPC矩形との
  重なり具合、および「C&C差」ラベル（縦方向のずれ量）が、「実際の挙動は
  基準からどれだけ乖離していたか」を一目で示す。
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
from logverify.model_diagram import plot_model_with_ego_paper_style
from logverify.multi_log_model import build_single_log_model_hysteresis, verify_logs_included
from logverify.reference_model_comparison import compute_ttc, ego_speed_series
from logverify.scenario_snapshot_diagram import ScenarioSnapshot, plot_scenario_snapshot_sequence

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_PATH = "out_gif/jama_cc_scenario_snapshots.png"
MODEL_OUT_PATH = "out_gif/jama_cc_cpd_model.png"


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
        rx_cc_ref = rx_ref[frame] if frame >= risk_frame and rx_ref[frame] is not None else None
        snapshots.append(ScenarioSnapshot(
            box_index=s.index, t=ts, rx=rx, ry=ry,
            ego_speed=ego_speed, npc_speed=npc_speed, npc_lateral_speed=vy,
            decel_label=decel_label, pred_label=pred_label, contact_label=contact_label,
            lane_k=s.k, pos_i=s.i,
            rx_cc_ref=rx_cc_ref,
        ))
        marker = " <- risk知覚後" if frame >= risk_frame else ""
        gap_str = f"{rx_cc_ref - rx:+.1f}m" if rx_cc_ref is not None else "-"
        print(f"  box#{s.index} (k={s.k},i={s.i}) frame={frame} t={ts - onset_ts:+.2f}s "
              f"rx={rx:.2f} rx_ref={rx_cc_ref if rx_cc_ref is not None else '-'} (差{gap_str}) "
              f"減速:{decel_label} 余裕:{contact_label}{marker}")

    path = plot_scenario_snapshot_sequence(
        snapshots, OUT_PATH,
        ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
        title="TD-NI-AR-SD-N04-CI-0067: 抽象化後のモデル（JAMA C&C反実仮想 = CPDの箱列）",
        t_ref=onset_ts,
    )
    print(f"図を書き出しました: {path}")

    # 12.21節 (c): 「対応するCPDモデル」——スナップショット列と同じ格子・
    # 同じヒステリシス処理で作った箱列を、実際にgcpd.Modelとして構築し、
    # 12.7/12.8/12.14節と同じ箱矢印図で可視化する。box_index・(k,i)は
    # 上のスナップショット列のものと1対1に対応する。
    #
    # English: Section 12.21 (c) -- build the "corresponding CPD model":
    # the same box sequence used for the snapshot sequence above (same
    # grid, same hysteresis processing), but actually constructed as a
    # gcpd.Model, rendered as the same box-and-arrow diagram used in
    # Sections 12.7/12.8/12.14. box_index/(k,i) here correspond 1:1 with
    # the snapshot sequence above.
    print("=== 対応するCPDモデル (gcpd.Model) を構築 ===")
    mlm = build_single_log_model_hysteresis(
        rel_xy, auto.rx_near_cell, auto.rx_far_cell, auto.rx_near_range, auto.gy, margin_ratio=0.3,
    )
    membership = verify_logs_included(mlm)
    print(f"  箱数(ダミー開始箱含む): {len(mlm.model.boxes)}, max_step: {mlm.model.max_step}")
    print(f"  元ログの箱列がモデルに含まれるか(SAT): {[r.is_member for r in membership]}")
    model_path = plot_model_with_ego_paper_style(
        mlm.model, mlm.box_id_of, MODEL_OUT_PATH,
        car="NPC", ego_lane=0, ego_max_step=mlm.model.max_step,
        title="TD-NI-AR-SD-N04-CI-0067: 対応するgcpd.Model（スナップショット列と同一の箱列）",
    )
    print(f"CPDモデル図を書き出しました: {model_path}")


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
