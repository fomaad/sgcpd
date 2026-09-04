"""12.24節: AJISAIカットインの衝突ログ5本・非衝突ログ5本に対して、
12.19〜12.23節で確立したJAMA C&Cモデルによる抽象化・可視化パイプライン
（`demo_jama_cc_snapshot.py`と同じ処理）を一括で適用する。

ユーザーからの依頼: 「少し，実験の対象を増やしたいと思います．AJISAIから，
cutinに関して，衝突しているログを5つ，衝突していないログを5つ拾い上げて，
それぞれ，同じ分析をして，可視化して，結果を教えてください．」

`demo_jama_cc_snapshot.py`のrun()は、衝突ログ(TD-NI-AR-SD-N04-CI-0067)
専用に書かれており、`find_collision_frames`が空リストを返す非衝突ログでは
そのままでは動かない（`coll_frames[0]`でIndexError）。本モジュールは、
12.17節で94本全体の解析に使った`compute_ratios_standalone.py`の
`closest_approach_frame`/`cutin_onset_frame`（衝突の有無によらず「最接近
フレーム」を求められる）を使って表示ウィンドウを決める一般化版を実装し、
衝突・非衝突どちらのログにも同じパイプラインを適用できるようにした。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.batch_jama_cc_analysis
"""

import json
import os

from logverify.abstract_cause import classify_contact_margin
from logverify.auto_grid import auto_grid_params_from_ajisai
from logverify.compute_ratios_standalone import closest_approach_frame, cutin_onset_frame
from logverify.demo_scenario_snapshot import (
    _load,
    decel_label_at,
    find_collision_frames,
    lateral_speed_at,
    npc_speed_at,
    pred_label_at,
    speed_at,
    vehicle_sizes,
)
from logverify.grid_bridge import (
    compress_to_grid_states_variable_hysteresis,
    grid_index_variable_center,
    relative_xy_from_ajisai_groundtruth,
)
from logverify.jama_cc_model import find_risk_perceived_frame, simulate_cc_reference
from logverify.model_diagram import plot_model_with_ego_paper_style
from logverify.multi_log_model import build_single_log_model_hysteresis, verify_logs_included
from logverify.reference_model_comparison import compute_ttc, ego_speed_series
from logverify.scenario_snapshot_diagram import ScenarioSnapshot, plot_scenario_snapshot_sequence

LOG_DIR_CANDIDATES = [
    "/mnt/user-data/uploads/Downloads/cutin/cutin",
    "/mnt/user-data/uploads/Downloads",
]
OUT_DIR = "out_gif/batch12_24"

# ユーザー指示: 衝突ログ5本・非衝突ログ5本。cutin_ratios_full.json（12.17節、
# 94本全体のratio/is_collisionサマリ）から、既存の分析対象0067(衝突)・
# 0030(非衝突)を含みつつ、番号が偏らないよう分散して選定した。
COLLISION_LOGS = [
    "TD-NI-AR-SD-N04-CI-0002.json",
    "TD-NI-AR-SD-N04-CI-0036.json",
    "TD-NI-AR-SD-N04-CI-0067.json",
    "TD-NI-AR-SD-N04-CI-0071.json",
    "TD-NI-AR-SD-N04-CI-0093.json",
]
NON_COLLISION_LOGS = [
    "TD-NI-AR-SD-N04-CI-0001.json",
    "TD-NI-AR-SD-N04-CI-0030.json",
    "TD-NI-AR-SD-N04-CI-0044.json",  # 12.17節のニアミスログ(ratio=0.012048、衝突ログの範囲に食い込む)
    "TD-NI-AR-SD-N04-CI-0065.json",
    "TD-NI-AR-SD-N04-CI-0090.json",
]


def _find_log_path(name: str) -> str:
    for d in LOG_DIR_CANDIDATES:
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    raise FileNotFoundError(f"{name} not found in {LOG_DIR_CANDIDATES}")


def analyze_one(json_path: str, log_id: str, out_dir: str) -> dict:
    """demo_jama_cc_snapshot.run()と同じ分析・可視化を1本のログに適用する。

    find_collision_frames()が空になりうる非衝突ログにも対応するため、
    表示ウィンドウの中心はcompute_ratios_standalone.closest_approach_frame
    （衝突の有無によらず「最接近フレーム」を返す）を使って決める。

    ---
    English:
    Applies the same analysis/visualization as demo_jama_cc_snapshot.run()
    to a single log. Since find_collision_frames() can return an empty
    list for a non-collision log, the display window is centered using
    compute_ratios_standalone.closest_approach_frame (which returns the
    "closest approach frame" regardless of whether a collision occurs).
    """
    print(f"\n=== {log_id} ===")
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

    closest_frame, closest_risk = closest_approach_frame(rxs, rys, eh_l, eh_w, nh_l, nh_w)
    is_collision = closest_risk is not None and closest_risk < 1.0
    onset_frame = cutin_onset_frame(rys, closest_frame)
    onset_ts = gk[onset_frame]["timestamp"]
    print(f"最接近フレーム: {closest_frame} (2Dリスク値={closest_risk:.4f}, "
          f"{'衝突' if is_collision else '非衝突'}), カットイン開始フレーム: {onset_frame}")

    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)
    ego_speed_full = ego_speed_series(gk)
    risk_frame, lateral_frame, ttc_frame = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    print(f"risk知覚フレーム: {risk_frame} (横方向={lateral_frame}, TTC={ttc_frame})")
    rx_ref = simulate_cc_reference(gk, rxs, ego_speed_full, risk_frame)

    # 予防可能性判定(12.19節と同じロジック): risk知覚後、最接近から30フレーム
    # 先までの範囲で、反実仮想の2Dリスク値の最小値を求める。
    window_end = min(len(rxs), closest_frame + 30)
    min_risk_ref, min_risk_frame = None, None
    for i in range(risk_frame, window_end):
        if rx_ref[i] is None or rys[i] is None:
            continue
        r = max(abs(rx_ref[i]) / (eh_l + nh_l), abs(rys[i]) / (eh_w + nh_w))
        if min_risk_ref is None or r < min_risk_ref:
            min_risk_ref, min_risk_frame = r, i
    preventable = min_risk_ref is not None and min_risk_ref >= 1.0
    print(f"JAMA C&C反実仮想の最小2Dリスク値: {min_risk_ref:.4f} (実際: {closest_risk:.4f}) "
          f"-> {'予防可能' if preventable else '予防不可能'}")

    t_lo = min(onset_ts, timestamps[risk_frame]) - 1.0
    t_hi = timestamps[window_end - 1] + 1.0
    frame_lo = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - t_lo))
    frame_hi = min(range(len(gk)), key=lambda i: abs(gk[i]["timestamp"] - t_hi))

    auto = auto_grid_params_from_ajisai(json_path)
    states = compress_to_grid_states_variable_hysteresis(
        rxs, rys, auto.rx_near_cell, auto.rx_far_cell, auto.rx_near_range, auto.gy, margin_ratio=0.3,
    )
    sub_states = [s for s in states if s.end_frame >= frame_lo and s.start_frame <= frame_hi]
    # 可視化上の都合による打ち切り: risk知覚フレームが（TTCのノイズ的な
    # 振動などにより）最接近よりかなり早く求まるログでは、表示ウィンドウ
    # が数十箱に及ぶことがある（本節の10本中、0002が該当）。予防可能性
    # 判定などの数値はウィンドウを絞る前の`risk_frame`に基づいて既に
    # 計算済みであり影響を受けないが、図の可読性のため、パネル数が
    # `MAX_BOXES`を超える場合は最接近側の直近`MAX_BOXES`箱に絞る。
    #
    # English: A visualization-only truncation. For logs where the
    # risk-perceived frame ends up much earlier than the closest
    # approach (e.g. due to TTC's noisy oscillation), the display
    # window can span dozens of boxes (log 0002 among these 10). The
    # preventability verdict etc. are already computed from the
    # un-truncated `risk_frame` and are unaffected; only the figure is
    # truncated to the most recent `MAX_BOXES` boxes before the closest
    # approach, for readability.
    MAX_BOXES = 25
    truncated = len(sub_states) > MAX_BOXES
    if truncated:
        sub_states = sub_states[-MAX_BOXES:]
    print(f"CPDの箱数（全体）: {len(states)}, 表示範囲内の箱数: {len(sub_states)}"
          + (" (表示用に直近{}箱へ打ち切り)".format(MAX_BOXES) if truncated else ""))

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

    final_gap = None
    for s in reversed(snapshots):
        if s.rx_cc_ref is not None:
            final_gap = s.rx_cc_ref - s.rx
            break

    snap_path = f"{out_dir}/{log_id}_snapshots.png"
    plot_scenario_snapshot_sequence(
        snapshots, snap_path,
        ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
        title=f"{log_id}: 抽象化後のモデル（JAMA C&C反実仮想 = CPDの箱列）",
        t_ref=onset_ts,
    )
    print(f"図を書き出しました: {snap_path}")

    # build_single_log_model_hysteresis自体はPythonのみで軽いが、
    # verify_logs_included（Z3によるmembership check）は箱数が多い
    # ログ（非衝突ログはカットイン後も長く記録が続くため箱数が数百に
    # なることがある）では数分単位で重くなることがある(11.6/11.7節で
    # 報告した「分岐が多いほどSAT求解が重くなる」傾向と同じ)。本バッチは
    # 可視化が目的であり、0067個別の詳細解析(12.19〜12.22節)で
    # membership=SATであることは既に確認済みのため、ここでは省略する。
    #
    # English: build_single_log_model_hysteresis itself is pure Python
    # and cheap, but verify_logs_included (a Z3 membership check) can
    # take minutes for logs with many boxes (non-collision logs keep
    # recording well past the cut-in, so box counts can run into the
    # hundreds) -- the same "more branching makes SAT solving heavier"
    # trend reported in Sections 11.6/11.7. This batch is for
    # visualization, and the detailed single-log analysis (Sections
    # 12.19-12.22) already confirmed membership=SAT for 0067, so it is
    # skipped here.
    mlm = build_single_log_model_hysteresis(
        rel_xy, auto.rx_near_cell, auto.rx_far_cell, auto.rx_near_range, auto.gy, margin_ratio=0.3,
    )
    is_member = None

    sub_indices = {s.index for s in sub_states}
    model_snapshots = []
    for idx, key in enumerate(mlm.sequences[0]):
        if idx not in sub_indices:
            continue
        lane_k, pos_i = key
        box_id = mlm.box_id_of[key]
        rx_c = grid_index_variable_center(pos_i, auto.rx_near_cell, auto.rx_far_cell, auto.rx_near_range)
        ry_c = lane_k * auto.gy
        model_snapshots.append(ScenarioSnapshot(
            box_index=box_id, t=0.0, rx=rx_c, ry=ry_c,
            ego_speed=0.0, npc_speed=0.0, npc_lateral_speed=0.0,
            lane_k=lane_k, pos_i=pos_i,
        ))
    model_pos_path = f"{out_dir}/{log_id}_cpd_model_positions.png"
    if model_snapshots:
        plot_scenario_snapshot_sequence(
            model_snapshots, model_pos_path,
            ego_half_length=eh_l, ego_half_width=eh_w, npc_half_length=nh_l, npc_half_width=nh_w,
            title=f"{log_id}: 対応するgcpd.Model（箱ごとの代表位置、EGO/NPC相対位置版）",
            show_time=False,
            transition_arrow_style="boxes",
        )
        print(f"CPDモデル位置関係図を書き出しました: {model_pos_path}")
    else:
        model_pos_path = None

    return {
        "log_id": log_id,
        "dataset_label_collision": None,  # 呼び出し側で埋める(cutin_ratios_full.jsonのラベル)
        "recomputed_collision": is_collision,
        "closest_risk": closest_risk,
        "risk_frame": risk_frame,
        "min_risk_ref": min_risk_ref,
        "preventable": preventable,
        "final_cc_gap_m": final_gap,
        "n_boxes_total": len(states),
        "n_boxes_window": len(sub_states),
        "is_member": is_member,
        "snapshot_path": snap_path,
        "model_positions_path": model_pos_path,
    }


def run() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)

    ratios_by_name = {}
    for cand in ("/mnt/user-data/uploads/Downloads/cutin/cutin_ratios_full.json",):
        if os.path.exists(cand):
            with open(cand) as f:
                for entry in json.load(f):
                    ratios_by_name[entry["name"]] = entry

    results = []
    for name in COLLISION_LOGS + NON_COLLISION_LOGS:
        log_id = name.replace(".json", "")
        json_path = _find_log_path(name)
        r = analyze_one(json_path, log_id, OUT_DIR)
        r["dataset_label_collision"] = ratios_by_name.get(name, {}).get("is_collision")
        results.append(r)

    print("\n\n=== まとめ ===")
    header = (
        f"{'log':<28}{'label':<8}{'再計算':<8}{'2Dリスク':>10}{'risk_frame':>12}"
        f"{'C&C最小risk':>12}{'予防可能':>10}{'C&C差(末端)':>12}{'箱(窓/全)':>10}"
    )
    print(header)
    for r in results:
        label = "衝突" if r["dataset_label_collision"] else "非衝突"
        recomputed = "衝突" if r["recomputed_collision"] else "非衝突"
        gap_str = f"{r['final_cc_gap_m']:+.1f}m" if r["final_cc_gap_m"] is not None else "-"
        boxes_str = f"{r['n_boxes_window']}/{r['n_boxes_total']}"
        print(
            f"{r['log_id']:<28}{label:<8}{recomputed:<8}{r['closest_risk']:>10.4f}{r['risk_frame']:>12d}"
            f"{r['min_risk_ref']:>12.4f}{'YES' if r['preventable'] else 'no':>10}"
            f"{gap_str:>12}{boxes_str:>10}"
        )

    summary_path = f"{OUT_DIR}/summary.json"
    with open(summary_path, "w") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"\nサマリJSONを書き出しました: {summary_path}")


if __name__ == "__main__":
    run()
