"""12.25節への訂正・再設計: 「近傍/遠方」の本来の意味を反映した、
安全性モデルによる述語的な抽象化 (predicate abstraction)。

## これまでの実装の何が間違っていたか

`compare_safety_model_abstractions.py`（および元になった`auto_grid.py`）は、
「near/far」を**距離に応じてセルサイズを変える、非一様な計量格子**
（`grid_bridge.grid_index_variable`）として実装していた。すなわち、
近傍は細かいセルサイズ(near_cell)、遠方は粗いセルサイズ(far_cell)で
区切るが、遠方であっても位置が変われば別の箱になる（`far_cell`の倍数
おきに新しい箱番号が割り当てられる）——遠くにいくらでも多くの区別が
残る、単に「粗い」格子だった。

また、「車両物理サイズ基準」の格子(`auto_grid_params_from_ajisai`)は、
near_cell（車両サイズ由来）とfar_cell（near_rangeの倍数）とで異なる
セルサイズを使っていたが、near_rangeの「係数3倍」・far_cellの
「near_rangeの5倍」自体は車両サイズそのものではなく、その上に載せた
追加の設計判断（ヒューリスティック）だった。「車両の物理サイズを基準に
する」という原則だけからは、near/farの区別をする理由もセルサイズを
変える理由も出てこない——徹底するなら、near/farの区別なしに、
車両サイズ由来の1つのセルサイズを全域に一様に使うべきである（これは
`auto_grid_params_naive_uniform`が表す「一様格子」ファミリーの、
特にセルサイズを車両サイズから機械的に決めた特殊ケースに他ならない）。

ユーザーからの指摘は以上の2点で、いずれも正しい。

## 本来意図されていた「near/far」

ユーザーの説明: 「near/farといっていたのは、セルサイズを変更するのでは
ない。(1) C&Cドライバモデルで抽象化する。(2) 遠い部分の箱で関係ない
ものは1つの箱にする。という意味」。

これは、計量格子（メートル単位のセルサイズ）ではなく、**述語的な抽象化
(predicate abstraction)**である。すなわち:

  1. 「近傍」（安全性モデルが実際に注意を払う範囲）は、その安全性モデル
     自身が使っている状態変数——JAMA C&Cなら「risk知覚フレームより前か
     後か」「接触しているかどうか」——によって区切る。メートル単位の
     セルサイズという概念そのものが登場しない。
  2. 「遠方」（安全性モデルが注意を払わない範囲）は、位置によらず
     **文字通り単一の箱**にまとめる。遠方でも位置に応じて別の箱に
     なる、ということはない。

さらに重要な技術的補足として、`gcpd.Model`の箱の同一性は
`(lane, position)`という離散indexのペアであり、`multi_log_model.
_model_from_sequences`は同じindexペアへの再訪問を自動的に同じ箱として
扱う（`box_id_of`は初出のときだけ新しいidを割り当てる）。したがって
これまで報告していた「箱数」(`len(states)`、`compress_to_grid_states_
variable_hysteresis`の戻り値の長さ)は、実際には**連続して同じ箱に
留まっている区間(run)の数**であり、`gcpd.Model`が実際に持つ**相異なる
箱の数**(`len(box_id_of)`)とは異なる（travelが同じ箱に複数回戻れば
run数は箱数より多くなる）。本モジュールでは`len(box_id_of)`（真の
状態空間サイズ）の方を報告する。

---
English:
Section 12.25 correction/redesign: a predicate-abstraction implementation
that reflects the originally-intended meaning of "near/far".

## What the earlier implementation got wrong

`compare_safety_model_abstractions.py` (and the underlying `auto_grid.py`)
implemented "near/far" as a **non-uniform metric grid** whose cell size
varies with distance (`grid_bridge.grid_index_variable`): near uses a
fine cell size (near_cell), far uses a coarse one (far_cell) -- but even
in the far region, a different position still gets a different box (a new
box index every far_cell meters). That is merely "coarser", not
"collapsed" -- arbitrarily many distinct far-away boxes remain possible.

Also, the "vehicle physical size" grid (`auto_grid_params_from_ajisai`)
used different cell sizes for near_cell (vehicle-size-derived) and
far_cell (a multiple of near_range) -- but the "x3" factor for near_range
and the "x5 of near_range" for far_cell are not themselves derived from
vehicle size; they are additional heuristics layered on top. The
principle "base it on the vehicle's physical size" alone gives no reason
to distinguish near from far, or to use different cell sizes at all -- to
be consistent, it should instead use a single vehicle-size-derived cell
size uniformly across the whole domain (exactly a special case of the
"uniform grid" family in `auto_grid_params_naive_uniform`, with the cell
width mechanically fixed from vehicle size instead of picked arbitrarily).

Both of the user's points above are correct.

## What "near/far" was actually meant to be

The user's own clarification: "near/far" doesn't mean changing cell size.
It means: (1) abstract using the C&C driver model itself, and (2) merge
irrelevant far-away boxes into a single box.

This is **predicate abstraction**, not a metric grid:

  1. "Near" (the region the safety model actually attends to) is
     partitioned by that safety model's OWN state variables -- for JAMA
     C&C, "before or after the risk-perceived frame", "in contact or
     not". The notion of a metric cell size never enters.
  2. "Far" (the region the safety model does not attend to) is collapsed
     into **literally one box**, regardless of position. A far-away
     position never gets a different box just because it's further away.

An important further technical point: a `gcpd.Model`'s box identity is
the discrete `(lane, position)` index pair, and
`multi_log_model._model_from_sequences` automatically treats a revisit to
the same index pair as the same box (`box_id_of` only assigns a new id
the first time a key is seen). So the "box counts" reported earlier
(`len(states)`, the length of
`compress_to_grid_states_variable_hysteresis`'s return value) actually
counted the number of **runs** (maximal stretches spent continuously in
the same box), not the number of **distinct boxes**
(`len(box_id_of)`) the `gcpd.Model` actually has -- if the trajectory
revisits the same box more than once, the run count exceeds the true box
count. This module reports `len(box_id_of)` (the true state-space size)
instead.

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.safety_predicate_abstraction
"""

from dataclasses import dataclass
from typing import Callable, Dict, Hashable, List, Optional, Sequence, Tuple

from logverify.grid_bridge import grid_index_centered
from logverify.synth_thresholds_multilog import _load, vehicle_sizes, relative_xy
from logverify.reference_model_comparison import compute_ttc, ego_speed_series
from logverify.jama_cc_model import find_risk_perceived_frame
from logverify.rss_model import npc_speed_series, find_rss_risk_frame

LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"


@dataclass
class LabelRun:
    box_id: int
    label: Hashable
    start_frame: int
    end_frame: int


def compress_by_label(
    n_frames: int, valid: Sequence[bool], label_fn: Callable[[int], Hashable]
) -> Tuple[List[LabelRun], Dict[Hashable, int]]:
    """述語（ラベル）による圧縮。`label_fn(frame)`が同じ値を返す連続区間を
    1つのrunにまとめつつ、ラベルが以前に出現していれば同じbox_idを再利用
    する（gcpd.Modelの箱の同一性と同じ意味論）。

    ---
    English: Label(predicate)-driven compression. Consecutive frames with
    the same `label_fn(frame)` value are merged into one run; a label seen
    before reuses its existing box_id (matching the semantics of box
    identity in a gcpd.Model).
    """
    label_of_box: Dict[Hashable, int] = {}
    runs: List[LabelRun] = []
    prev_label: Optional[Hashable] = None

    for frame in range(n_frames):
        if not valid[frame]:
            prev_label = None
            continue
        label = label_fn(frame)
        if label not in label_of_box:
            label_of_box[label] = len(label_of_box)
        bid = label_of_box[label]
        if prev_label is not None and label == prev_label:
            runs[-1].end_frame = frame
        else:
            runs.append(LabelRun(box_id=bid, label=label, start_frame=frame, end_frame=frame))
        prev_label = label

    return runs, label_of_box


def cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, risk_frame, near_rx, gy):
    """JAMA C&Cモデル自身の状態変数（risk知覚フレームの前後・接触の有無）
    と、near_rxの外側を単一のFAR箱に潰す、というルールで各フレームの
    ラベルを決める。

    ---
    English: labels each frame using JAMA C&C's own state variables
    (before/after the risk-perceived frame, in contact or not), collapsing
    everything outside near_rx into a single FAR box.
    """
    def label_fn(frame):
        rx, ry = rxs[frame], rys[frame]
        risk2d = max(abs(rx) / (eh_l + nh_l), abs(ry) / (eh_w + nh_w))
        if risk2d < 1.0:
            return ("CONTACT",)
        if abs(rx) > near_rx:
            return ("FAR",)
        lane_k = grid_index_centered(ry, gy)
        if risk_frame is not None and frame >= risk_frame:
            return ("RISK", lane_k)
        return ("SAFE", lane_k)
    return label_fn


def rss_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, risk_frame, near_rx, gy):
    """RSS版。risk_frameはRSS違反(|rx|<d_min)が最初に持続するフレーム。

    ---
    English: RSS counterpart. risk_frame is the first persistent RSS
    violation frame.
    """
    def label_fn(frame):
        rx, ry = rxs[frame], rys[frame]
        risk2d = max(abs(rx) / (eh_l + nh_l), abs(ry) / (eh_w + nh_w))
        if risk2d < 1.0:
            return ("CONTACT",)
        if abs(rx) > near_rx:
            return ("FAR",)
        lane_k = grid_index_centered(ry, gy)
        if risk_frame is not None and frame >= risk_frame:
            return ("VIOLATION", lane_k)
        return ("SAFE", lane_k)
    return label_fn


def _purity_of_predicate_abstraction(runs: List[LabelRun], onset_frame: Optional[int]) -> dict:
    """述語抽象化の場合のpurity判定（compare_safety_model_abstractions.
    _purity_for_onsetと同じ定義）。"""
    if onset_frame is None:
        return {"applicable": False}
    for r in runs:
        if r.start_frame <= onset_frame <= r.end_frame:
            pure = r.start_frame == onset_frame
            return {
                "applicable": True, "pure": pure, "onset_frame": onset_frame,
                "run_start_frame": r.start_frame, "run_end_frame": r.end_frame,
                "n_frames_smeared_before_onset": onset_frame - r.start_frame,
            }
    return {"applicable": False}


def run():
    data = _load(LOG_PATH)
    gk = data["groundtruth_kinematic"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)
    rxs, rys = relative_xy(data)
    timestamps = [rec["timestamp"] for rec in gk]
    ego_speed = ego_speed_series(gk)
    npc_speed = npc_speed_series(data)
    ttcs = compute_ttc(rxs, timestamps, eh_l, nh_l)

    cc_risk_frame, _, _ = find_risk_perceived_frame(rxs, rys, ttcs, eh_w, nh_w)
    rss_risk_frame, _ = find_rss_risk_frame(rxs, ego_speed, npc_speed)

    gy = 0.364
    near_rx = 40.0
    n = len(rxs)
    valid = [rxs[i] is not None and rys[i] is not None for i in range(n)]

    cc_label_fn = cc_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, cc_risk_frame, near_rx, gy)
    rss_label_fn = rss_predicate_label_fn(rxs, rys, eh_l, eh_w, nh_l, nh_w, rss_risk_frame, near_rx, gy)

    cc_runs, cc_boxes = compress_by_label(n, valid, cc_label_fn)
    rss_runs, rss_boxes = compress_by_label(n, valid, rss_label_fn)

    print(f"JAMA C&C risk-perceived frame: {cc_risk_frame}")
    print(f"RSS violation-onset frame:     {rss_risk_frame}")
    print()
    print("=== C&C述語抽象化 (predicate abstraction) ===")
    print(f"  真の箱数(distinct boxes) = {len(cc_boxes)}   (参考: run数 = {len(cc_runs)})")
    print(f"  箱の内訳: {sorted(cc_boxes.keys(), key=str)}")
    p_own = _purity_of_predicate_abstraction(cc_runs, cc_risk_frame)
    p_other = _purity_of_predicate_abstraction(cc_runs, rss_risk_frame)
    print(f"  purity(自分自身=C&C onset): {'PURE' if p_own.get('pure') else 'IMPURE'} (構造上必然的にPURE)")
    print(f"  purity(RSS onsetに対して): {'PURE' if p_other.get('pure') else 'IMPURE'}"
          + ("" if p_other.get('pure', True) else f" -- {p_other['n_frames_smeared_before_onset']}フレーム分混在"))

    print()
    print("=== RSS述語抽象化 (predicate abstraction) ===")
    print(f"  真の箱数(distinct boxes) = {len(rss_boxes)}   (参考: run数 = {len(rss_runs)})")
    print(f"  箱の内訳: {sorted(rss_boxes.keys(), key=str)}")
    p_own2 = _purity_of_predicate_abstraction(rss_runs, rss_risk_frame)
    p_other2 = _purity_of_predicate_abstraction(rss_runs, cc_risk_frame)
    print(f"  purity(自分自身=RSS onset): {'PURE' if p_own2.get('pure') else 'IMPURE'} (構造上必然的にPURE)")
    print(f"  purity(C&C onsetに対して): {'PURE' if p_other2.get('pure') else 'IMPURE'}"
          + ("" if p_other2.get('pure', True) else f" -- {p_other2['n_frames_smeared_before_onset']}フレーム分混在"))

    return {
        "cc_boxes": len(cc_boxes), "cc_runs": len(cc_runs),
        "rss_boxes": len(rss_boxes), "rss_runs": len(rss_runs),
    }


if __name__ == "__main__":
    run()
