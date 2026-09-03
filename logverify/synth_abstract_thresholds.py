"""抽象解釈演算子のしきい値（`abstract_cause.classify_deceleration_adequacy`の
`overkill_ratio`・`adequate_ratio`・`weak_ratio`）を、既知の原因（ドメイン
分析で確立済みの結論）から、Z3（SMT）で自動的に求める。

## 経緯

ユーザーからの質問: 「これらの抽象的な値は，速度とかの具体的な値を
データマッピングを用いて抽象化していると思います．そのようなデータ
マッピングする抽象化関数を自動的に求めることは可能でしょうか？」
（添付論文 Bensalem, Lakhnech, Owre "Computing Abstractions of Infinite
State Systems Compositionally and Automatically" [CAV'98] を参考に、
PVSの代わりにSMTを使う）。

論文を読んだ上での結論（詳細は12.15節）: 論文のelimination methodは
「抽象化関数αはすでに与えられている」という前提で、そのαに関して
具象系をシミュレートすることが保証される抽象遷移関係を自動計算する
方法である（Hoare tripleの妥当性判定にPVSを使っており、これはZ3の
UNSATチェックに直接置き換えられる）。しかし、α自体（しきい値という
データマッピング）を新規に合成する部分は論文の範囲外だった。

そこでユーザーへの追加質問「衝突を起こす要因が与えられれば、それらから
自動的に求まりますか？」への回答として、本モジュールを実装した:
**すでに分かっている原因（ドメイン分析の結論）を、Z3の制約として
与え、その制約を満たす中で「現在の手動しきい値からの変化が最小」な
しきい値をZ3のOptimizeで求める**、という制約ベースのしきい値合成
（CEGIS的な発想の簡易版）である。

## 使う「既知の原因」

12.5〜12.6節で確立済みの結論: log 0067の衝突は、NPCのカットインへの
EGOの減速が終始「弱いまま」であり(緩やかな立ち上がり)、衝突直前に
急に「過剰」になったわけではない。したがって、衝突ウィンドウ直前
までのCPDの箱（12.14節のbox#8〜#13）はすべて「不要／適切／過剰」
ではなく「弱い」または「非常に弱い」に分類されなければならない、
という制約を立てる。

---
English:
Automatically derives the thresholds of the deceleration-adequacy
operator (`overkill_ratio`, `adequate_ratio`, `weak_ratio` in
`abstract_cause.classify_deceleration_adequacy`) from an already-known
cause (an established conclusion from domain analysis), using Z3 (SMT).

## Background

The user asked: "I believe these abstract values abstract concrete
values (like speed) via a data mapping. Is it possible to automatically
find the abstraction function that performs that data mapping?" --
referencing the attached paper (Bensalem, Lakhnech, Owre, CAV'98),
using SMT in place of PVS.

Having read the paper (see Section 12.15 for the full discussion), the
paper's elimination method assumes the abstraction function alpha is
already GIVEN, and automatically computes an abstract transition
relation guaranteed to simulate the concrete system w.r.t. that alpha
(the Hoare-triple validity check uses PVS, which maps directly onto a
Z3 UNSAT check). Synthesizing alpha itself (the threshold-based data
mapping) is outside the paper's scope.

In response to the user's follow-up question, "if the factors causing
the collision are given, can [the mapping] be derived automatically
from them?", this module implements exactly that: **encode an
already-known cause (a domain-analysis conclusion) as Z3 constraints,
and use Z3's Optimize to find thresholds that satisfy those constraints
while deviating minimally from the current hand-picked thresholds** -- a
constraint-based threshold synthesis (a lightweight CEGIS-style
approach).

## The "known cause" used here

The conclusion already established in Sections 12.5-12.6: in log 0067,
Ego's deceleration response to the NPC's cut-in stayed "weak" throughout
(a slow ramp-up), rather than being adequate and then suddenly
"overkill" right before the collision. Hence the constraint: every CPD
box before the collision window (Section 12.14's box #8-#13) must be
classified as "weak" or "very weak" -- never "unnecessary", "adequate",
or "overkill".

How to run / 実行方法:
    cd sgcpd && python3 -m logverify.synth_abstract_thresholds \
        [path-to-TD-NI-AR-SD-N04-CI-0067.json]
"""

import json
import sys

import z3

from logverify.abstract_cause import box_aggregated_deceleration_ratio, classify_deceleration_adequacy
from logverify.auto_grid import auto_grid_params_from_ajisai
from logverify.grid_bridge import (
    compress_to_grid_states_variable_hysteresis,
    relative_xy_from_ajisai_groundtruth,
)

DEFAULT_LOG_PATH = "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0067.json"

DEFAULT_THRESHOLDS = dict(overkill_ratio=1.5, adequate_ratio=1.0, weak_ratio=0.5)


def _load(json_path):
    with open(json_path) as f:
        return json.load(f)


def vehicle_sizes(data):
    sizes = {v["name"]: v["size"] for v in data["groundtruth_size"]["vehicle_sizes"]}
    ego = sizes["ego"]
    npc = sizes.get("npc1", list(v for k, v in sizes.items() if k != "ego")[0])
    return (ego["x"] / 2, ego["y"] / 2), (npc["x"] / 2, npc["y"] / 2)


def find_collision_frames(rel_xy, eh_l, eh_w, nh_l, nh_w):
    return [
        i for i, (rx, ry) in enumerate(rel_xy)
        if (eh_l + nh_l) - abs(rx) > 0 and (eh_w + nh_w) - abs(ry) > 0
    ]


def synthesize_thresholds(known_constraints, defaults=DEFAULT_THRESHOLDS, margin: float = 1e-3):
    """`known_constraints`（既知の原因から分かっている、achieved/requiredの
    比とそのラベル制約の対のリスト）を満たしつつ、既定のしきい値からの
    変化が最小になるしきい値をZ3のOptimizeで求める。

    `known_constraints`の各要素は`(ratio, kind)`で、kindは:
      - "not_adequate": ratio は「適切」以上であってはならない
        (ratio < adequate_ratio)
      - "very_weak": ratio は「非常に弱い」でなければならない
        (ratio < weak_ratio)

    ---
    English:
    Uses Z3's Optimize to find thresholds that satisfy `known_constraints`
    (pairs of an achieved/required ratio and a label constraint implied by
    a known cause), while deviating as little as possible from `defaults`.

    Each element of `known_constraints` is `(ratio, kind)`, where kind is:
      - "not_adequate": the ratio must not be classified "adequate" or
        higher (ratio < adequate_ratio)
      - "very_weak": the ratio must be classified "very weak"
        (ratio < weak_ratio)
    """
    overkill_ratio = z3.Real("overkill_ratio")
    adequate_ratio = z3.Real("adequate_ratio")
    weak_ratio = z3.Real("weak_ratio")

    opt = z3.Optimize()
    # Physical sanity: the four regions must stay ordered and positive.
    opt.add(overkill_ratio > adequate_ratio)
    opt.add(adequate_ratio > weak_ratio)
    opt.add(weak_ratio > 0)

    # A strict "<" leaves the feasible region open, so its infimum need not
    # be attained -- Z3's optimizer can then settle on an arbitrary feasible
    # point rather than the true minimum-distance one. Using `ratio + margin
    # <= threshold` instead makes the region closed (and the LP well-posed)
    # while still implying the strict inequality for any margin > 0.
    for r, kind in known_constraints:
        if kind == "not_adequate":
            opt.add(z3.RealVal(r) + margin <= adequate_ratio)
        elif kind == "very_weak":
            opt.add(z3.RealVal(r) + margin <= weak_ratio)
        else:
            raise ValueError(f"unknown constraint kind: {kind}")

    # Objective: minimize the (L1) distance from the hand-picked defaults --
    # i.e., find the smallest correction to the existing thresholds that is
    # consistent with the known cause, rather than an arbitrary solution.
    d_overkill = z3.Real("d_overkill")
    d_adequate = z3.Real("d_adequate")
    d_weak = z3.Real("d_weak")
    opt.add(d_overkill >= overkill_ratio - defaults["overkill_ratio"])
    opt.add(d_overkill >= defaults["overkill_ratio"] - overkill_ratio)
    opt.add(d_adequate >= adequate_ratio - defaults["adequate_ratio"])
    opt.add(d_adequate >= defaults["adequate_ratio"] - adequate_ratio)
    opt.add(d_weak >= weak_ratio - defaults["weak_ratio"])
    opt.add(d_weak >= defaults["weak_ratio"] - weak_ratio)
    opt.minimize(d_overkill + d_adequate + d_weak)

    result = opt.check()
    if result != z3.sat:
        return None, result

    m = opt.model()

    def val(v):
        r = m.eval(v, model_completion=True)
        return float(r.as_fraction())

    return dict(
        overkill_ratio=val(overkill_ratio),
        adequate_ratio=val(adequate_ratio),
        weak_ratio=val(weak_ratio),
    ), result


def run(json_path: str) -> None:
    print(f"Loading: {json_path}")
    data = _load(json_path)
    rel_xy = relative_xy_from_ajisai_groundtruth(json_path)
    rxs = [p[0] for p in rel_xy]
    rys = [p[1] for p in rel_xy]
    gk = data["groundtruth_kinematic"]
    cc = data["control_cmds"]
    (eh_l, eh_w), (nh_l, nh_w) = vehicle_sizes(data)

    coll_frames = find_collision_frames(rel_xy, eh_l, eh_w, nh_l, nh_w)
    window_first, window_last = coll_frames[0], coll_frames[-1]

    auto = auto_grid_params_from_ajisai(json_path)
    states = compress_to_grid_states_variable_hysteresis(
        rxs, rys, auto.rx_near_cell, auto.rx_far_cell, auto.rx_near_range, auto.gy, margin_ratio=0.3,
    )
    # Boxes strictly before the collision window: this is where Sections
    # 12.5-12.6 established the known cause ("weak deceleration throughout").
    pre_collision_boxes = [s for s in states if s.end_frame < window_first and s.start_frame >= window_first - 200]

    print("=== 12.14節のCPDの箱ごとに、区間全体で平均化した減速比を計算 ===")
    box_ratios = []
    for s in pre_collision_boxes:
        ratio = box_aggregated_deceleration_ratio(rxs, gk, cc, s.start_frame, s.end_frame, eh_l, nh_l)
        if ratio is not None:
            box_ratios.append((s, ratio))
        print(f"  box#{s.index} frames {s.start_frame}-{s.end_frame}: "
              f"ratio={ratio if ratio is not None else 'N/A (すでに接触範囲)'}")
    ratios = [r for _, r in box_ratios]
    print(f"既知の原因（12.5〜12.6節）: これら{len(ratios)}個の比はすべて「適切」以上であってはならない "
          f"(= 一貫して弱い減速だった)")
    print(f"現在の手動しきい値との比較: max(ratio)={max(ratios):.3f}, 既定のadequate_ratio={DEFAULT_THRESHOLDS['adequate_ratio']}")
    print()

    print("=== Z3で、既知の原因を満たす、既定値から最小変化のしきい値を合成 ===")
    constraints = [(r, "not_adequate") for r in ratios]
    thresholds, result = synthesize_thresholds(constraints)
    if thresholds is None:
        print(f"合成失敗: {result}")
        return
    print(f"既定のしきい値:   {DEFAULT_THRESHOLDS}")
    print(f"合成されたしきい値: {{'overkill_ratio': {thresholds['overkill_ratio']:.4f}, "
          f"'adequate_ratio': {thresholds['adequate_ratio']:.4f}, 'weak_ratio': {thresholds['weak_ratio']:.4f}}}")
    changed = any(abs(thresholds[k] - DEFAULT_THRESHOLDS[k]) > 1e-6 for k in DEFAULT_THRESHOLDS)
    if changed:
        print("-> 既定のしきい値では既知の原因と矛盾していたため、Z3が最小限の補正を行った。")
    else:
        print("-> 既定のしきい値は、箱単位で平均化した比を使えばすでに既知の原因と矛盾していなかった"
              "（12.12/12.13節の不安定さは、しきい値の選び方の問題ではなく、単一フレームで評価していた"
              "ことが原因だったことがここで裏付けられる）。")
    print()

    print("=== 合成されたしきい値で、実際に各箱を再分類 ===")
    for s, r in box_ratios:
        label = classify_deceleration_adequacy(r, 1.0, overkill_ratio=thresholds["overkill_ratio"],
                                                 adequate_ratio=thresholds["adequate_ratio"],
                                                 weak_ratio=thresholds["weak_ratio"])
        print(f"  box#{s.index}: ratio={r:.3f} -> 【{label}】")
    print()

    print("=== What-if: 既定のしきい値と矛盾する既知の原因が与えられた場合（架空の例） ===")
    print("仮に、box#13(ratio=0.476)は「弱い」ではなく厳密に「非常に弱い」でなければならない、"
          "という(架空の)追加のドメイン知識が別途得られたとする。既定のweak_ratio=0.5では"
          "0.476<0.5を満たすので実は既定値のままでも問題ないが、より厳しい仮の制約"
          "(ratio=0.55のケースが「非常に弱い」でなければならない、というまだ見ぬログの事例)"
          "を与えると、既定のweak_ratio=0.5とは矛盾するため、Z3が最小限の補正を行う。")
    hypothetical_constraints = constraints + [(0.55, "very_weak")]
    thresholds2, result2 = synthesize_thresholds(hypothetical_constraints)
    if thresholds2 is None:
        print(f"  合成失敗（この制約集合は矛盾している）: {result2}")
    else:
        print(f"  合成されたしきい値: {{'overkill_ratio': {thresholds2['overkill_ratio']:.4f}, "
              f"'adequate_ratio': {thresholds2['adequate_ratio']:.4f}, 'weak_ratio': {thresholds2['weak_ratio']:.4f}}}")
        print(f"  -> weak_ratioが既定の0.5から{thresholds2['weak_ratio']:.4f}へ、"
              f"矛盾を解消する最小限だけ自動的に補正された。")


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    run(json_path)
