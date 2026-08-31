"""
参照CPD（cut-in）に対する適合性検証のデモ。

実際のAJISAIログ（Boxで配布、リポジトリには同梱しない）が手元にない環境でも
パイプライン全体を検証できるよう、ego基準の相対座標 (rx, ry) を
手作りした3種類の合成トラジェクトリで試す:

  1. cutin_like       : 隣接レーンからegoレーンへ merge して、そのまま前方に留まる
                         （典型的な cut-in）。 -> SAT が期待される。
  2. stays_in_own_lane: 最初からずっと ego レーンにいる（隣接レーンから来ていない）。
                         -> 参照モデルの init（隣接レーンのみ）に一致する箱がなく、
                            ステップ0で unmatched になることが期待される。
  3. swerve_like       : 隣接レーン -> egoレーン -> 隣接レーンに戻る、を繰り返す
                         （蛇行）。 -> 合流後に隣接レーンへ戻る遷移が参照モデルに
                            存在しないため UNSAT が期待される。

実データに対しては logverify.grid_bridge.grid_states_from_json(path, gx, gy) で
同じ (gx, gy) を使って離散化すればそのまま流用できる。

実行方法:
    cd sgcpd && python3 -m logverify.demo_cutin_membership
"""

from logverify.grid_bridge import grid_states_from_relative_xy
from logverify.membership import check_membership_cutin
from logverify.reference_models import build_cutin_reference, describe

# 参照モデルと同じ粒度で離散化する。
# gy=3.5: 一般的な車線幅(m)。 lane=0 が自車線、lane=+-1 が隣接レーン。
# gx=5.0: 縦方向の1箱あたりの距離(m)。position=0 は「ほぼ横並び」、
#         正が前方、負が後方。
GX, GY = 5.0, 3.5


def make_cutin_like():
    # NPCは右隣接レーン(ry=-3.5m)のやや前方(rx=15m)から始まり、
    # 徐々に前方へ進みつつ、途中で ego レーンへ merge し、その後も前方へ進み続ける。
    pts = []
    rx, ry = 15.0, -3.5
    for _ in range(6):
        pts.append((rx, ry)); rx += 2.0
    # merge (ryが0へ)
    for t in range(6):
        pts.append((rx, ry + (t + 1) * (3.5 / 6))); rx += 2.0
    ry = 0.0
    for _ in range(6):
        pts.append((rx, ry)); rx += 2.0
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
    # 右隣接レーンから ego レーンへ
    for t in range(6):
        pts.append((rx, ry + (t + 1) * (3.5 / 6))); rx += 2.0
    ry = 0.0
    for _ in range(3):
        pts.append((rx, ry)); rx += 2.0
    # ego レーンから再び右隣接レーンへ戻る（蛇行）
    for t in range(6):
        pts.append((rx, ry - (t + 1) * (3.5 / 6))); rx += 2.0
    ry = -3.5
    for _ in range(3):
        pts.append((rx, ry)); rx += 2.0
    return pts


def run_case(name, points):
    states = grid_states_from_relative_xy(points, GX, GY)
    observed = [(s.k, s.i) for s in states]
    model, box_id_of = build_cutin_reference(i_range=range(-2, 12), side_lanes=(-1, 1), ego_lane=0)
    result = check_membership_cutin(model, observed)
    print(f"=== {name} ===")
    print(f"圧縮後の状態列 (lane, position): {observed}")
    print(result)
    print()
    return result


if __name__ == "__main__":
    r1 = run_case("cutin_like（典型的なカットイン）", make_cutin_like())
    r2 = run_case("stays_in_own_lane（最初からego車線）", make_stays_in_own_lane())
    r3 = run_case("swerve_like（合流後に元のレーンへ戻る＝蛇行）", make_swerve_like())

    print("--- 期待される結果 ---")
    print("cutin_like        : SAT   (実際:", "SAT" if r1.is_member else "UNSAT", ")")
    print("stays_in_own_lane : UNSAT (実際:", "SAT" if r2.is_member else "UNSAT", ")")
    print("swerve_like       : UNSAT (実際:", "SAT" if r3.is_member else "UNSAT", ")")
