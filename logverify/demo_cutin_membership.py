"""参照CPD（cut-in）に対する適合性検証 + シナリオ分類のデモ。

実際のAJISAIログ（Boxで配布、リポジトリには同梱しない）が手元にない環境でも
パイプライン全体を検証できるよう、ego基準の相対座標 (rx, ry) を
手作りした合成トラジェクトリで試す。

v0.4 で追加したバリエーション:
  - 近距離 / 中距離 / 遠距離 それぞれで始まり、合流するcut-in
  - 隣接レーンで速度を合わせてしばらく並走してから、加速して合流するcut-in
  これらはすべて同じ1つの参照CPD (build_cutin_reference の1回の呼び出し) の
  異なる充足解（witness）として表現される。これがCPDの強み:
  「距離帯・並走・加速といったバリエーション」を1つのモデルで書ける。

  position を生の格子ではなく logverify.zones の
  近距離/中距離/遠距離(+後方) という少数の順序値に丸めることで、
  箱の数を距離レンジによらず一定に保ち、ソルバを高速に保っている
  （生の格子のままだと箱の数が距離レンジに比例して増え、遷移が
  箱数の2乗のオーダーで爆発してしまう）。

実データに対しては logverify/zones.py の
zone_states_from_relative_xy(rel_xy, gy, thresholds) に、
座標正規化済みの (rx, ry) 列を渡せばそのまま使える。

実行方法:
    cd sgcpd && python3 -m logverify.demo_cutin_membership

---
English:
Demo for conformance checking against a reference CPD (cut-in) plus scenario
classification.

Since a real AJISAI log (distributed via Box, not bundled with this
repository) is not always available, this demo exercises the whole pipeline
using hand-crafted synthetic trajectories of ego-relative coordinates
(rx, ry).

Variations added in v0.4:
  - cut-ins starting at a near / medium / far distance and then merging in
  - a cut-in that first matches speed and drives alongside in the adjacent
    lane for a while, then accelerates and merges in
  All of these are expressed as different satisfying solutions (witnesses)
  of the very same reference CPD (a single call to build_cutin_reference).
  This is the strength of CPD: variations such as "distance band, parallel
  driving, acceleration" can all be written into one model.

  Rounding position down to the small set of ordered values from
  logverify.zones (near/medium/far(+behind)) instead of using the raw grid
  keeps the number of boxes constant regardless of the distance range,
  which keeps the solver fast (with the raw grid, the number of boxes grows
  proportionally to the distance range and the number of transitions
  explodes on the order of the square of the box count).

For real data, simply pass a coordinate-normalized (rx, ry) sequence to
zone_states_from_relative_xy(rel_xy, gy, thresholds) in logverify/zones.py.

How to run:
    cd sgcpd && python3 -m logverify.demo_cutin_membership
"""

from logverify.membership import check_membership_cutin
from logverify.reference_models import build_cutin_reference
from logverify.report import summarize_trace, describe_scenario
from logverify.zones import zone_states_from_relative_xy, ZoneThresholds

GY = 3.5  # 車線幅(m)。 lane=0 が自車線、lane=+-1 が隣接レーン。
# (English) Lane width (m). lane=0 is the ego lane, lane=+-1 are the adjacent lanes.
THRESHOLDS = ZoneThresholds(near_max=5.0, medium_max=20.0)


def _lane_change_points(rx0, ry0, ry1, rx_step, n):
    pts = []
    rx = rx0
    for t in range(n):
        pts.append((rx, ry0 + (ry1 - ry0) * (t + 1) / n))
        rx += rx_step
    return pts, rx


def make_cutin_near():
    # ego のすぐ横（近距離）から合流する
    # (English) Merges in from right beside ego (near distance).
    pts = [(2.0, -3.5), (3.0, -3.5)]
    lane_pts, rx = _lane_change_points(4.0, -3.5, 0.0, 1.0, 4)
    pts += lane_pts
    pts += [(rx, 0.0), (rx + 1.0, 0.0)]
    return pts


def make_cutin_medium():
    pts = [(12.0, -3.5), (14.0, -3.5), (16.0, -3.5)]
    lane_pts, rx = _lane_change_points(18.0, -3.5, 0.0, 2.0, 5)
    pts += lane_pts
    pts += [(rx, 0.0), (rx + 2.0, 0.0)]
    return pts


def make_cutin_far():
    pts = [(30.0, -3.5), (33.0, -3.5), (36.0, -3.5)]
    lane_pts, rx = _lane_change_points(39.0, -3.5, 0.0, 3.0, 5)
    pts += lane_pts
    pts += [(rx, 0.0), (rx + 3.0, 0.0), (rx + 6.0, 0.0)]
    return pts


def make_cutin_parallel_then_accelerate():
    # 隣接レーンでegoとほぼ横並びのまま速度を合わせて並走 -> 加速して前方へ抜けつつ合流
    # (English) Drives alongside ego in the adjacent lane at matched speed, staying
    # (English) roughly level -> then accelerates, pulls ahead, and merges in.
    pts = []
    rx, ry = 1.0, -3.5
    for _ in range(6):  # 並走区間（ほぼ同じ rx のまま複数状態） (English) parallel-driving segment (multiple states at nearly the same rx)
        pts.append((rx, ry)); rx += 0.3
    # 加速して一気に前方（遠距離）へ抜けながら合流（distance zoneが大きく飛ぶ）
    # (English) Accelerates and shoots forward (to a far distance) while merging in
    # (English) (the distance zone jumps by a large amount).
    lane_pts, rx = _lane_change_points(rx, -3.5, 0.0, 8.0, 3)
    pts += lane_pts
    pts += [(rx, 0.0), (rx + 25.0, 0.0)]
    return pts


def make_stays_in_own_lane():
    pts = []
    rx, ry = 10.0, 0.0
    for _ in range(15):
        pts.append((rx, ry)); rx += 1.5
    return pts


def make_swerve_like():
    pts = []
    rx, ry = 12.0, -3.5
    for t in range(6):
        pts.append((rx, ry + (t + 1) * (3.5 / 6))); rx += 2.0
    ry = 0.0
    for _ in range(3):
        pts.append((rx, ry)); rx += 2.0
    for t in range(6):
        pts.append((rx, ry - (t + 1) * (3.5 / 6))); rx += 2.0
    ry = -3.5
    for _ in range(3):
        pts.append((rx, ry)); rx += 2.0
    return pts


def run_case(name, points, model):
    states = zone_states_from_relative_xy(points, GY, THRESHOLDS)
    observed = [(s.lane, s.zone) for s in states]
    durations = [s.end_frame - s.start_frame + 1 for s in states]
    result = check_membership_cutin(model, observed)
    print(f"=== {name} ===")
    print(f"Compressed state sequence (lane, position=zone), frame durations: {list(zip(observed, durations))}")
    print(result)
    if result.is_member:
        steps = summarize_trace(observed, durations=durations, parallel_min_frames=6)
        print(describe_scenario(steps))
    print()
    return result


if __name__ == "__main__":
    # 1つの参照モデルで、近距離/中距離/遠距離/並走+加速のすべてのcut-inをカバーする。
    # position は logverify.zones の BEHIND(-1)/NEAR(0)/MEDIUM(1)/FAR(2) の4値のみ。
    # (English) A single reference model covers all cut-in variants: near, medium,
    # (English) far, and parallel-then-accelerate. position takes only the four
    # (English) values BEHIND(-1)/NEAR(0)/MEDIUM(1)/FAR(2) from logverify.zones.
    model, box_id_of = build_cutin_reference(
        i_range=(-1, 0, 1, 2), side_lanes=(-1, 1), ego_lane=0
    )

    results = {}
    results["near-distance cut-in"] = run_case("near-distance cut-in", make_cutin_near(), model)
    results["medium-distance cut-in"] = run_case("medium-distance cut-in", make_cutin_medium(), model)
    results["far-distance cut-in"] = run_case("far-distance cut-in", make_cutin_far(), model)
    results["parallel-then-accelerate cut-in"] = run_case(
        "parallel-then-accelerate cut-in", make_cutin_parallel_then_accelerate(), model
    )
    results["stays_in_own_lane (negative case)"] = run_case(
        "stays_in_own_lane (negative case)", make_stays_in_own_lane(), model
    )
    results["swerve_like (negative case)"] = run_case("swerve_like (negative case)", make_swerve_like(), model)

    print("--- Summary (verdicts against the same reference model) ---")
    for name, r in results.items():
        print(f"{name:32s}: {'SAT' if r.is_member else 'UNSAT'}")
