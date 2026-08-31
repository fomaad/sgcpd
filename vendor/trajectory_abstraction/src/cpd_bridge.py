"""
cpd_bridge.py

咲川氏の15領域モデルによる抽象化（abstraction_15area.py）の出力を、
gcpd.py の Model（CPDの箱・レーン・位置）へ変換するアダプタ。

設計ドキュメント docs/log_to_cpd_verification_design.md の 9.2節「CPDとの対応づけ」
を実装したもの。

領域名 → (lane, position) の対応（15領域モデル）:
    lane:      left=+1 (自車から見て左隣接車線) / 0 (自車線) / right=-1 (右隣接車線)
    position:  far-lead=+2 / lead=+1 / ego=0 / follow=-1 / far-rear=-2
"""

import json
from pathlib import Path
import abstraction_15area as ab

# 領域名 -> (lane, position)
REGION_TO_LANE_POS = {
    "far-left":        (1, 2),
    "far-front":       (0, 2),
    "far-right":       (-1, 2),
    "lead-left":       (1, 1),
    "lead0":           (0, 1),
    "lead-right":      (-1, 1),
    "left":            (1, 0),
    "ego":             (0, 0),
    "right":           (-1, 0),
    "follow-left":     (1, -1),
    "follow_0":        (0, -1),
    "follow-right":    (-1, -1),
    "far-rear-left":   (1, -2),
    "far-rear":        (0, -2),
    "far-rear-right":  (-1, -2),
}


def abstract_trajectory(json_path):
    """1件のAJISAIログ（本体.json）を咲川氏の15領域モデルで抽象化し、
    圧縮後の状態遷移列（region名のリスト）を返す。
    """
    data = ab.load_json_data(json_path)
    ego_coords, npc_coords, ego_vel, npc_vel, timestamps, npc_ids = ab.extract_coordinates_from_json(data)
    ego_n, npc_n, rot = ab.normalize_coordinates(ego_coords, npc_coords)
    ego_vn, npc_vn = ab.normalize_velocities(ego_vel, npc_vel, rot)

    rel_v = ab.calculate_relative_velocity(ego_vn, npc_vn)

    regions = []
    prev_region = None
    for i in range(len(timestamps)):
        if npc_n[i][0] is None:
            continue
        region, rx, ry = ab.get_relative_position_with_values(ego_n[i], npc_n[i])
        if region != prev_region:
            regions.append(region)
            prev_region = region
    return regions


def build_cpd_model_snippet(regions, car="npc1"):
    """抽象状態列 (region名のリスト) から、gcpd.Model に登録するための
    append_box / append_position / append_lane / add_ntrans の引数を組み立てる。
    実際に gcpd.Model インスタンスを構築するには、これらをそのまま
    my_model.append_box([...]) のように渡せばよい。
    """
    boxes = []
    positions = []
    lanes = []
    ntrans = []
    for k, region in enumerate(regions):
        if region not in REGION_TO_LANE_POS:
            raise ValueError(f"未知の領域名です（対応表に追加してください）: {region}")
        lane, pos = REGION_TO_LANE_POS[region]
        boxes.append((car, k))
        positions.append((car, k, pos))
        lanes.append((car, k, lane))
        if k > 0:
            ntrans.append((car, k - 1, car, k))
    return {
        "boxes": boxes,
        "positions": positions,
        "lanes": lanes,
        "ntrans": ntrans,
        "init": [(car, 0)],
        "max_step": max(len(regions) - 1, 0),
    }


if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else "/mnt/user-data/uploads/Downloads/TD-NI-AR-SD-N04-CI-0035.json"
    regions = abstract_trajectory(path)
    print("抽象状態遷移列:", regions)
    snippet = build_cpd_model_snippet(regions)
    print()
    print("CPD Model 構築用データ:")
    for key, val in snippet.items():
        print(f"  {key}: {val}")
