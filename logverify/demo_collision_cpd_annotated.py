"""衝突ログ（既定: TD-NI-AR-SD-N04-CI-0067.json）について、
  - 衝突の前後だけを細かく、それ以外（遠方）は粗く抽象化し、
  - egoの減速状態を各箱にアノテーションとして書き込み、
  - EgoとNPCを同じ列（=同じ瞬間）で揃えて1枚のCPD図に描く
デモ。`logverify/collision_cpd_diagram.py`を使う。

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.demo_collision_cpd_annotated \
        [path-to-TD-NI-AR-SD-N04-CI-0067.json]
"""

import json
import sys

from logverify.collision_cpd_diagram import (
    EventAnnotation,
    InstanceBox,
    plot_instance_cpd_annotated,
    plot_instance_cpd_lateral,
)
from logverify.grid_bridge import compress_to_grid_states_variable, relative_xy_from_ajisai_groundtruth

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"
OUT_PATH = "out_gif/collision_0067_cpd_annotated.png"
OUT_PATH_LATERAL = "out_gif/collision_0067_cpd_lateral.png"

# 衝突の近辺だけを細かくするための格子。rx（縦方向）はego近傍
# (|rx|<=RX_NEAR_RANGE)だけ1mセルで細かく、それより遠くは50mセルで粗く
# まとめる。ry（横方向）は変動範囲がもともと数mしかないため、一様に
# 0.3mセルで細かいままにしておく（遠方でまとめる恩恵が小さいため）。
# (English) Grid used to keep only the region around the collision fine.
# rx (longitudinal) is fine (1m cells) only within RX_NEAR_RANGE of ego,
# coarse (50m cells) beyond that. ry (lateral) stays uniformly fine at
# 0.3m cells since its range is only a few meters to begin with (little
# benefit to coarsening it far away).
RX_NEAR_CELL = 1.0
RX_FAR_CELL = 50.0
RX_NEAR_RANGE = 15.0
GY = 0.3

EGO_HALF_WIDTH = 1.9 / 2.0  # logverify.sakikawa_relations と同じ値
EGO_LANE_K_BOUND = EGO_HALF_WIDTH / GY  # このk以内なら「ego車線」帯とみなす


def _load(json_path):
    with open(json_path) as f:
        return json.load(f)


def find_collision_window(data, rel_xy):
    sizes = {v["name"]: v["size"] for v in data["groundtruth_size"]["vehicle_sizes"]}
    ego_size = sizes["ego"]
    npc_size = sizes.get("npc1", list(sizes.values())[1])
    eh_l, eh_w = ego_size["x"] / 2, ego_size["y"] / 2
    nh_l, nh_w = npc_size["x"] / 2, npc_size["y"] / 2
    frames = [
        i for i, (rx, ry) in enumerate(rel_xy)
        if (eh_l + nh_l) - abs(rx) > 0 and (eh_w + nh_w) - abs(ry) > 0
    ]
    if not frames:
        return None
    return frames[0], frames[-1]


def ego_state_at(data, frame_start, frame_end, use_endpoint: bool = False):
    """[frame_start, frame_end] の時間区間でのegoの加速度から状態ラベル・色を返す。

    use_endpoint=False（既定、細かい箱向け）: 区間内の最小値（最も強い減速）を使う。
    use_endpoint=True（粗く1つにまとめた箱向け）: 区間の"直前"の値ではなく、
    区間の終わり（次の箱に引き継がれる瞬間）に最も近い値を使う。粗い箱は
    長い時間区間をまとめているため、区間内のどこかにあるカットインとは
    無関係な一時的な減速スパイク（例: ログ冒頭の初期整定）を最小値として
    拾ってしまうのを避けるため。

    ---
    English:
    Returns a state label and color from ego's acceleration over the
    window [frame_start, frame_end].

    use_endpoint=False (default, for fine-grained boxes): uses the minimum
    value in the window (the strongest deceleration).
    use_endpoint=True (for boxes that collapse a long span into one coarse
    box): uses the value nearest the *end* of the window (the moment
    handed off to the next box), rather than the minimum over the whole
    span -- this avoids picking up a transient deceleration spike
    unrelated to the cut-in that happens to occur somewhere within that
    long collapsed span (e.g. initial settling at the very start of the
    log).
    """
    gk = data["groundtruth_kinematic"]
    ts0 = gk[frame_start]["timestamp"]
    ts1 = gk[frame_end]["timestamp"]
    cc = data["control_cmds"]
    accels = [e["longitudinal"]["acceleration"] for e in cc if ts0 - 0.05 <= e["timestamp"] <= ts1 + 0.05]
    if not accels:
        # フォールバック: 最も近い1点
        nearest = min(cc, key=lambda e: abs(e["timestamp"] - (ts0 + ts1) / 2))
        accels = [nearest["longitudinal"]["acceleration"]]
    if use_endpoint:
        near_end = [e["longitudinal"]["acceleration"] for e in cc if ts1 - 0.5 <= e["timestamp"] <= ts1 + 0.05]
        a_min = near_end[-1] if near_end else accels[-1]
    else:
        a_min = min(accels)
    if a_min >= -0.05:
        return f"巡航\n({a_min:+.2f})", "#eafaea"
    if a_min >= -1.0:
        return f"減速開始\n({a_min:+.2f})", "#fff6d8"
    if a_min >= -2.0:
        return f"減速中\n({a_min:+.2f})", "#ffe0b0"
    return f"強い減速\n({a_min:+.2f})", "#ffb3b3"


def vehicle_half_widths(data):
    """ego・NPCそれぞれの車体半幅(m)を groundtruth_size から取り出す。

    ---
    English:
    Returns ego's and the NPC's own vehicle half-width (m) from
    groundtruth_size.
    """
    sizes = {v["name"]: v["size"] for v in data["groundtruth_size"]["vehicle_sizes"]}
    ego_size = sizes["ego"]
    npc_size = sizes.get("npc1", list(sizes.values())[1])
    return ego_size["y"] / 2, npc_size["y"] / 2


def build_boxes(json_path: str):
    data = _load(json_path)
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path)
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    states = compress_to_grid_states_variable(rxs, rys, RX_NEAR_CELL, RX_FAR_CELL, RX_NEAR_RANGE, GY)

    window = find_collision_window(data, rel_xy)
    assert window is not None, "collision window not found"
    coll_start, coll_end = window

    # 遠方（rxがnear_rangeの外）にある区間と、合流後しばらく安定してからの
    # 区間は、1つの粗い箱にまとめる。境界は「near_rangeに最初に入った箱」と
    # 「合流後、最初の安定した長い箱が終わった箱」で決める。
    # (English) Collapse the far-away region (rx outside near_range) and the
    # region well after settling into the ego lane into single coarse
    # boxes. Boundaries are picked at "the box where rx first enters
    # near_range" and "the box where the first long, stable post-merge box
    # ends".
    near_start_idx = next(i for i, s in enumerate(states) if abs(rxs[s.start_frame]) <= RX_NEAR_RANGE + 2)
    # 合流後最初の長時間安定した箱（k=0まで到達した箱）の直後で打ち切る
    merge_stable_idx = next(i for i, s in enumerate(states) if s.k == 0 and (s.end_frame - s.start_frame) > 100)

    far_box = states[:near_start_idx]
    fine_boxes = states[near_start_idx : merge_stable_idx + 1]
    post_box = states[merge_stable_idx + 1 :]

    boxes = []

    if far_box:
        ego_label, ego_color = ego_state_at(data, far_box[0].start_frame, far_box[-1].end_frame, use_endpoint=True)
        boxes.append(
            InstanceBox(
                label="NPC\n(遠方走行中)",
                lane_band="side",
                start_frame=far_box[0].start_frame,
                end_frame=far_box[-1].end_frame,
                note="粗い抽象化\n(rx>15m)",
                ego_state=ego_label,
                ego_color=ego_color,
                ry_m=rys[far_box[-1].end_frame],
            )
        )

    for s in fine_boxes:
        band = "ego" if abs(s.k) <= EGO_LANE_K_BOUND else "side"
        is_collision = s.start_frame <= coll_end and s.end_frame >= coll_start
        ego_label, ego_color = ego_state_at(data, s.start_frame, s.end_frame)
        mid_frame = (s.start_frame + s.end_frame) // 2
        boxes.append(
            InstanceBox(
                label=f"NPC\n(k={s.k},i={s.i})",
                lane_band=band,
                start_frame=s.start_frame,
                end_frame=s.end_frame,
                highlight="collision" if is_collision else None,
                note="衝突" if is_collision else None,
                ego_state=ego_label,
                ego_color=ego_color,
                ry_m=rys[mid_frame],
            )
        )

    if post_box:
        ego_label, ego_color = ego_state_at(data, post_box[0].start_frame, post_box[-1].end_frame, use_endpoint=True)
        boxes.append(
            InstanceBox(
                label="NPC\n(合流後、安定)",
                lane_band="ego",
                start_frame=post_box[0].start_frame,
                end_frame=post_box[-1].end_frame,
                note="粗い抽象化\n(以後の車線変更等は割愛)",
                ego_state=ego_label,
                ego_color=ego_color,
                ry_m=rys[post_box[0].start_frame],
            )
        )

    return boxes, data


def build_events(boxes, data):
    """減速開始・強い減速到達などのイベントを、それが最初に起きた箱に
    紐付けたEventAnnotationのリストにする。

    ---
    English:
    Turns events such as deceleration onset / reaching strong braking into
    a list of EventAnnotation objects, attached to the box in which they
    first occur.
    """
    cc = data["control_cmds"]
    gk = data["groundtruth_kinematic"]

    def box_index_at_ts(ts):
        for idx, b in enumerate(boxes):
            if gk[b.start_frame]["timestamp"] - 0.05 <= ts <= gk[b.end_frame]["timestamp"] + 0.05:
                return idx
        return None

    # ログ冒頭には、カットインとは無関係な初期整定の一時的な減速スパイク
    # （t=130.48s付近、-1.5m/s^2）が記録されているため、遠方の粗い箱
    # （boxes[0]、まだEgoに近づいていない区間）より後だけを探索対象にする。
    # (English) The very start of the log contains a transient deceleration
    # spike (around t=130.48s, -1.5 m/s^2) unrelated to the cut-in, so we
    # only search after the far/coarse box (boxes[0], the region not yet
    # close to ego).
    search_from_ts = gk[boxes[0].end_frame]["timestamp"] if boxes and "遠方" in boxes[0].label else gk[0]["timestamp"]
    onset_ts = next((e["timestamp"] for e in cc if e["timestamp"] >= search_from_ts and e["longitudinal"]["acceleration"] < 0), None)
    strong_ts = next((e["timestamp"] for e in cc if e["timestamp"] >= search_from_ts and e["longitudinal"]["acceleration"] <= -2.0), None)

    events = []
    if onset_ts is not None:
        idx = box_index_at_ts(onset_ts)
        if idx is not None:
            events.append(EventAnnotation(idx, f"減速指令開始\n(t={onset_ts:.2f}s)", color="#c98a00"))
    if strong_ts is not None:
        idx = box_index_at_ts(strong_ts)
        if idx is not None:
            events.append(EventAnnotation(idx, f"強い減速(-2m/s²超)\n(t={strong_ts:.2f}s)", color="#cc2222"))

    # NPCの予測軌道のゆるさに関する注記は、衝突箱の少し手前に付ける
    collision_idx = next((i for i, b in enumerate(boxes) if b.highlight == "collision"), None)
    if collision_idx is not None and collision_idx >= 2:
        events.append(
            EventAnnotation(
                collision_idx - 2,
                "NPC予測: 信頼度最大の経路は\nこの時点でも「直進継続」を予測\n(信頼度1.0→0.2に低下)",
                color="#555599",
            )
        )
    return events


def run(json_path: str) -> None:
    print(f"Loading: {json_path}")
    boxes, data = build_boxes(json_path)
    events = build_events(boxes, data)

    print(f"箱数: {len(boxes)}")
    for i, b in enumerate(boxes):
        mark = " <-- 衝突" if b.highlight == "collision" else ""
        print(f"  [{i}] {b.label.strip()!r:24s} band={b.lane_band:4s} frames {b.start_frame}-{b.end_frame} ego={b.ego_state.strip()!r}{mark}")

    lane_band_order = [("side", "隣接レーン（側方）"), ("ego", "ego車線")]
    path = plot_instance_cpd_annotated(
        boxes,
        lane_band_order,
        OUT_PATH,
        title="TD-NI-AR-SD-N04-CI-0067: 衝突前後を細かく、他は粗いインスタンスCPD",
        events=events,
    )
    print(f"\n図を書き出しました: {path}")

    ego_half_width, npc_half_width = vehicle_half_widths(data)
    path_lateral = plot_instance_cpd_lateral(
        boxes,
        OUT_PATH_LATERAL,
        ego_half_width=ego_half_width,
        npc_half_width=npc_half_width,
        title="TD-NI-AR-SD-N04-CI-0067: 横方向オフセット(ry)を縦位置で表したインスタンスCPD",
        events=events,
        y_scale=1.4,
    )
    print(f"図を書き出しました: {path_lateral}")


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
