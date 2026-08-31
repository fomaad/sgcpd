import json
import numpy as np
from math import cos, sin, atan2, floor
import os
from pathlib import Path
from tqdm import tqdm
import pandas as pd # 追加

# ==========================================
# 定数定義
# ==========================================
VEHICLE_WIDTH = 1.9   # 車幅 (m)
VEHICLE_LENGTH = 5.3  # 車長 (m)

EGO_HALF_WIDTH = VEHICLE_WIDTH / 2.0
EGO_HALF_LENGTH = VEHICLE_LENGTH / 2.0

V_MAX = 120.0  # 最大相対速度 (m/s)

# ==========================================
# グリッド計算・相対位置判定
# ==========================================

def get_grid_indices(rx, ry, grid_size_x, grid_size_y):
    """rx, ry に対して個別のグリッドサイズを適用"""
    if rx is None or ry is None:
        return None, None
    i = floor(rx / grid_size_x)
    k = floor(ry / grid_size_y)
    return i, k

def get_grid_label_str(i, k):
    if i is None or k is None:
        return "データなし"
    return f"i{i}_k{k}"

def concrete_same_lane(rx, ry):
    return abs(ry) < EGO_HALF_WIDTH

def concrete_other_lane(rx, ry):
    return abs(ry) >= EGO_HALF_WIDTH

def concrete_lead(rx, ry):
    return rx >= EGO_HALF_LENGTH

# ==========================================
# 相対速度計算・ラベル判定
# ==========================================

def calculate_relative_velocity(ego_velocities, npc_velocities):
    rel_velocities = []
    for i in range(len(ego_velocities)):
        if (ego_velocities[i][0] is not None and npc_velocities[i][0] is not None):
            rel_v = npc_velocities[i][0] - ego_velocities[i][0]
            rel_velocities.append(rel_v)
        else:
            rel_velocities.append(None)
    return np.array(rel_velocities)

def abstract_rel_vel_label(rel_v, threshold=0.0):
    if rel_v is None:
        return None
    if rel_v > threshold:
        return "abstract_faster" 
    elif rel_v < -threshold:
        return "abstract_slower" 
    else:
        return None

def concrete_rel_vel_label(rel_v, threshold=0):
    if rel_v is None:
        return None
    if rel_v > threshold:
        return "concrete_faster"
    elif rel_v < threshold:
        return "concrete_slower"
    else:
        return "concrete_equal"

# ==========================================
# メイン処理関数群
# ==========================================

def get_relative_position_with_values(ego_pos, npc_pos, grid_size_x, grid_size_y):
    if npc_pos[0] is None or npc_pos[1] is None:
        return "データなし", None, None, None, None
    rx = npc_pos[0] - ego_pos[0]
    ry = npc_pos[1] - ego_pos[1]
    i, k = get_grid_indices(rx, ry, grid_size_x, grid_size_y)
    label = get_grid_label_str(i, k)
    return label, rx, ry, i, k

def get_concrete_space_labels(rx, ry, rel_v):
    if rx is None or ry is None:
        return None, None, None
    
    lane_label = "same-lane" if concrete_same_lane(rx, ry) else "other-lane"
    is_lead = concrete_lead(rx, ry)
    rv_label = concrete_rel_vel_label(rel_v)
    
    return lane_label, is_lead, rv_label

def load_json_data(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        data = json.load(file)
    return data

def extract_coordinates_from_json(data):
    ego_coords = []
    npc_coords = []
    ego_velocities = []
    npc_velocities = []
    timestamps = []
    npc_ids = []

    if isinstance(data, dict) and 'groundtruth_kinematic' in data:
        states = data['groundtruth_kinematic']
    elif isinstance(data, list):
        states = data
    elif isinstance(data, dict) and 'states' in data:
        states = data['states']
    else:
        raise ValueError("JSONデータの構造が不明です。")

    for state in states:
        timestamp = state.get('timestamp') or state.get('timeStamp')
        timestamps.append(timestamp)

        ego_data = state.get('groundtruth_ego') or state.get('ego')
        if ego_data:
            ego_pos = ego_data['pose']['position']
            ego_coords.append([ego_pos['x'], ego_pos['y']]) 
            ego_twist = ego_data['twist']['linear']
            ego_velocities.append([ego_twist['x'], ego_twist['y']])
        else:
            ego_coords.append([None, None])
            ego_velocities.append([None, None])

        npc_data = state.get('groundtruth_vehicles') or state.get('groundtruth_NPCs')
        if npc_data and len(npc_data) > 0:
            npc_pos = npc_data[0]['pose']['position']
            npc_coords.append([npc_pos['x'], npc_pos['y']])
            npc_twist = npc_data[0]['twist']['linear']
            npc_velocities.append([npc_twist['x'], npc_twist['y']])
            npc_id = npc_data[0].get('id') or npc_data[0].get('name')
            npc_ids.append(npc_id)
        else:
            npc_coords.append([None, None])
            npc_velocities.append([None, None])
            npc_ids.append(None)

    return (np.array(ego_coords), np.array(npc_coords), 
            np.array(ego_velocities), np.array(npc_velocities), 
            np.array(timestamps), np.array(npc_ids))

def normalize_coordinates(ego_coords, npc_coords):
    start_point = ego_coords[0].copy()
    ego_normalized = ego_coords - start_point
    npc_normalized = npc_coords - start_point

    direction_vector = None
    for i in range(1, min(10, len(ego_normalized))):
        if ego_normalized[i, 0] is not None:
            movement = ego_normalized[i] - ego_normalized[0]
            if np.linalg.norm(movement) > 0.1:
                direction_vector = movement / np.linalg.norm(movement)
                break

    if direction_vector is None:
        direction_vector = np.array([1, 0])

    current_angle = atan2(direction_vector[1], direction_vector[0])
    rotation_angle = -current_angle
    cos_theta, sin_theta = cos(rotation_angle), sin(rotation_angle)
    rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

    def rotate_points(coords):
        rotated = []
        for c in coords:
            if c[0] is not None: rotated.append(np.dot(c, rotation_matrix.T))
            else: rotated.append([None, None])
        return np.array(rotated)

    return rotate_points(ego_normalized), rotate_points(npc_normalized), rotation_matrix

def normalize_velocities(ego_vel, npc_vel, rotation_matrix):
    def rotate_vels(vels):
        rotated = []
        for v in vels:
            if v[0] is not None: rotated.append(np.dot(v, rotation_matrix.T))
            else: rotated.append([None, None])
        return np.array(rotated)
    return rotate_vels(ego_vel), rotate_vels(npc_vel)

def generate_output_text(ego_coords, npc_coords, ego_vel, npc_vel, timestamps, npc_ids, grid_size_x, grid_size_y):
    output_lines = []
    npc_rel_velocities = calculate_relative_velocity(ego_vel, npc_vel)
    
    processed_data = []
    prev_state_key = None
    start_time = None
    
    # --- 具体空間の検出用フラグ ---
    concrete_cutin_files = []
    concrete_cutout_files = []
    concrete_accel_frames = []
    concrete_decel_frames = []
    
    # 判定用の履歴保持
    prev_rx = None
    prev_lane = None
    prev_lead = None

    for i in range(len(timestamps)):
        # 現在の値を計算
        rel_v = npc_rel_velocities[i]
        curr_id = npc_ids[i]
        abs_rv = abstract_rel_vel_label(rel_v)
        
        if (i < len(npc_coords) and npc_coords[i][0] is not None and not np.isnan(npc_coords[i][0])):
            grid_label, rx, ry, gi, gk = get_relative_position_with_values(ego_coords[i], npc_coords[i], grid_size_x, grid_size_y)
            con_lane, con_lead, con_rv = get_concrete_space_labels(rx, ry, rel_v)
        else:
            grid_label, rx, ry, gi, gk = "データなし", None, None, None, None
            con_lane, con_lead, con_rv = None, None, None

        # --- 具体空間のシナリオ判定 ---
        if i > 0 and rx is not None and prev_rx is not None:
            # 減速
            if rx >= 0 and rx < prev_rx and prev_rx >=0:
                concrete_decel_frames.append(i + 1)
            # 加速
            if rx < 0 and rx > prev_rx and prev_rx < 0:
                concrete_accel_frames.append(i + 1)
            # カットイン
            if prev_lane == "other-lane" and con_lane == "same-lane" and con_lead:
                concrete_cutin_files.append(i + 1)
            # カットアウト
            if prev_lane == "same-lane" and con_lane == "other-lane" and prev_lead:
                concrete_cutout_files.append(i + 1)

        prev_rx, prev_lane, prev_lead = rx, con_lane, con_lead

        # --- 状態圧縮ロジック ---
        current_key = (grid_label, con_lane, con_lead, abs_rv, curr_id)
        
        if current_key != prev_state_key:
            if start_time is not None:
                processed_data[-1]['time_range'] = (start_time, i)
            
            start_time = i + 1
            processed_data.append({
                'index': len(processed_data) + 1,
                'time_range': (start_time, start_time),
                'grid_label': grid_label,
                'grid_i': gi, 'grid_k': gk,
                'con_lane': con_lane, 'con_lead': con_lead,
                'abs_rv': abs_rv, 'npc_id': curr_id,
                'rx': rx,
                'state_events': [] # ★追加: この状態でのイベントを記録するリストを初期化
            })
            prev_state_key = current_key
    
    if processed_data:
        processed_data[-1]['time_range'] = (start_time, len(timestamps))

    # --- 抽象空間の検出（圧縮データから判定） ---
    abs_cutin, abs_cutout, abs_accel, abs_decel = [], [], [], []
    for i in range(1, len(processed_data)):
        p, c = processed_data[i-1], processed_data[i]
        
        # もし c['state_events'] が未定義なら初期化（念のため）
        if 'state_events' not in c: c['state_events'] = []

        if p['grid_i'] is not None and c['grid_i'] is not None:
            # カットイン: 隣接グリッド(k!=0)から自車線グリッド(k=0)へ
            if p['grid_i'] >= 2 and (p['grid_k'] >= 1 or p['grid_k'] < -1) and (-1 <= c['grid_k'] < 1) and c['grid_i'] >= 3:
                event_msg = f"{p['grid_label']} -> {c['grid_label']}(状態{c['index']})"
                abs_cutin.append(event_msg)
                c['state_events'].append("【カットイン】") # ★追加: 表示用にタグ付け

            # カットアウト: 自車線グリッド(k=0)から隣接グリッド(k!=0)へ
            if (-1 <= p['grid_k'] < 1) and p['grid_i'] >=3 and c['grid_i'] >=2 and (c['grid_k'] >=1 or c['grid_k'] < -1):
                event_msg = f"{p['grid_label']} -> {c['grid_label']}(状態{c['index']})"
                abs_cutout.append(event_msg)
                c['state_events'].append("【カットアウト】") # ★追加: 表示用にタグ付け

            # 加速: 後方(i<0)でグリッド番号が増加
            if c['grid_i'] < 0 and p['grid_i'] < 0  and c['grid_i'] > p['grid_i']:
                abs_accel.append(f"{p['grid_label']} -> {c['grid_label']}(状態{c['index']})")
                # 必要であればここにも c['state_events'].append("【抽象加速】") を追加可能

            # 減速: 前方(i>=0)でグリッド番号が減少
            if c['grid_i'] >= 0 and p['grid_i'] >= 0 and c['grid_i'] < p['grid_i']:
                abs_decel.append(f"{p['grid_label']} -> {c['grid_label']}(状態{c['index']})")
                # 必要であればここにも c['state_events'].append("【抽象減速】") を追加可能

    # --- テキスト出力の生成 ---
    header = f"{'Time':<20} {'抽象空間(Grid)':<20} {'具体空間':<25} {'備考':<20}"
    output_lines.append(header); output_lines.append("-" * len(header))
    
    for d in processed_data:
        s, e = d['time_range']
        t_str = f"{d['index']}({s}-{e})" if s != e else f"{d['index']}({s})"
        con_str = f"{d['con_lane']}+{'lead' if d['con_lead'] else 'follow'}" if d['con_lane'] else "N/A"
        
        # ★修正箇所: シナリオ検出結果と相対速度判定をマージして表示
        remarks = d.get('state_events', [])[:] # 検出されたイベント（カットイン/アウト）を取得
        
        if d['abs_rv'] == "abstract_faster": remarks.append("【加速中】")
        if d['abs_rv'] == "abstract_slower": remarks.append("【減速中】")
        
        output_lines.append(f"{t_str:<20} {d['grid_label']:<20} {con_str:<25} {' '.join(remarks)}")

    # サマリー
    output_lines.append("\n" + "="*60 + "\n【抽象空間での検出結果（グリッド不変）】\n" + "="*60)
    output_lines.append(f"カットイン: {'あり' if abs_cutin else 'なし'} ({len(abs_cutin)}回)")
    output_lines.append(f"カットアウト: {'あり' if abs_cutout else 'なし'} ({len(abs_cutout)}回)")
    output_lines.append(f"加速シナリオ: {'あり' if abs_accel else 'なし'} ({len(abs_accel)}フレームで検出)")
    output_lines.append(f"減速シナリオ: {'あり' if abs_decel else 'なし'} ({len(abs_decel)}フレームで検出)")

    # --- 検出結果の辞書を作成 ---
    det_res = {
        'abstract_cutin': bool(abs_cutin),
        'abstract_cutout': bool(abs_cutout),
        'abstract_accel': bool(abs_accel),
        'abstract_decel': bool(abs_decel),
        'concrete_cutin': bool(concrete_cutin_files),
        'concrete_cutout': bool(concrete_cutout_files),
        'concrete_accel': bool(concrete_accel_frames),
        'concrete_decel': bool(concrete_decel_frames),
        
        'num_abstract_states': len(processed_data),
        'total_frames': len(timestamps)
    }

    return "\n".join(output_lines), det_res

def process_file(input_path, output_path, gx, gy):
    try:
        data = load_json_data(input_path)
        ego_c, npc_c, ego_v, npc_v, ts, ids = extract_coordinates_from_json(data)
        if len(ts) == 0: return False, "No Data", None
        ego_n, npc_n, rot = normalize_coordinates(ego_c, npc_c)
        ego_vn, npc_vn = normalize_velocities(ego_v, npc_v, rot)
        out_txt, det = generate_output_text(ego_n, npc_n, ego_vn, npc_vn, ts, ids, gx, gy)
        with open(output_path, 'w', encoding='utf-8') as f: f.write(out_txt)
        return True, None, det
    except Exception as e:
        return False, str(e), None

def process_folder_batch(input_folder, base_output_folder, grid_size_pairs):
    input_path = Path(input_folder)
    base_out = Path(base_output_folder)
    json_files = list(input_path.glob("*.json"))
    if not json_files: return

    # ★追加: 全データを保持するリスト
    all_records = []

    for (gx, gy) in grid_size_pairs:
        output_dir = base_out / f"wx-{gx:.1f}_wy-{gy:.1f}"
        output_dir.mkdir(parents=True, exist_ok=True)
        print(f"\n--- 実行中: wx={gx}m, wy={gy}m ---")
        
        detection_stats = {k: {'count': 0, 'files': []} for k in [
            'abstract_cutin', 'abstract_cutout', 'abstract_accel', 'abstract_decel', 
            'concrete_cutin', 'concrete_cutout', 'concrete_accel', 'concrete_decel'
        ]}

        # --- 差分記録用の辞書を追加 ---
        discrepancies = {
            'cutin': [],
            'cutout': [],
            'accel': [],
            'decel': []
        }

        for json_file in tqdm(json_files, desc=f"x={gx},y={gy}"):
            out_file = output_dir / f"{json_file.stem}_result.txt"
            success, err, det = process_file(json_file, out_file, gx, gy)
            
            if success and det:
                # ★追加: 分析用レコードの作成
                record = {
                    'filename': json_file.name,
                    'grid_x': gx,
                    'grid_y': gy,
                    'total_frames': det['total_frames'],
                    'num_abstract_states': det['num_abstract_states'],
                    # ブール値を 1/0 または True/False で保存
                    'abs_cutin': det['abstract_cutin'],
                    'con_cutin': det['concrete_cutin'],
                    'abs_cutout': det['abstract_cutout'],
                    'con_cutout': det['concrete_cutout'],
                    'abs_accel': det['abstract_accel'],
                    'con_accel': det['concrete_accel'],
                    'abs_decel': det['abstract_decel'],
                    'con_decel': det['concrete_decel']
                }
                all_records.append(record)
            
                for k in detection_stats:
                    if det[k]:
                        detection_stats[k]['count'] += 1
                        detection_stats[k]['files'].append(json_file.name)

                # --- 抽象と具体の差分をチェック ---
                # カットイン
                if det['abstract_cutin'] != det['concrete_cutin']:
                    discrepancies['cutin'].append({
                        'file': json_file.name,
                        'abs': det['abstract_cutin'],
                        'con': det['concrete_cutin']
                    })
                # カットアウト
                if det['abstract_cutout'] != det['concrete_cutout']:
                    discrepancies['cutout'].append({
                        'file': json_file.name,
                        'abs': det['abstract_cutout'],
                        'con': det['concrete_cutout']
                    })
                # 加速
                if det['abstract_accel'] != det['concrete_accel']:
                    discrepancies['accel'].append({
                        'file': json_file.name,
                        'abs': det['abstract_accel'],
                        'con': det['concrete_accel']
                    })
                # 減速
                if det['abstract_decel'] != det['concrete_decel']:
                    discrepancies['decel'].append({
                        'file': json_file.name,
                        'abs': det['abstract_decel'],
                        'con': det['concrete_decel']
                    })

        if all_records:
            df = pd.DataFrame(all_records)
            csv_path = base_out / "analysis_dataset.csv"
            df.to_csv(csv_path, index=False, encoding='utf-8-sig')
            print(f"分析用CSVを保存しました: {csv_path}")

if __name__ == "__main__":
    # 基本となるパス設定
    BASE_INPUT_DIR = Path("../data/generated_trajectories")
    BASE_OUTPUT_DIR = Path("../out")

    # 処理したいシナリオフォルダ名のリスト
    TARGET_SCENARIOS = [
        "accel_scenario",
        "deccel_scenario",
        "cutout_scenario",
        "cutin_scenario"
    ]

    print("=== 全シナリオの一括処理を開始します ===")
    print(f"Base Input : {BASE_INPUT_DIR}")
    print(f"Base Output: {BASE_OUTPUT_DIR}")

    for scenario_name in TARGET_SCENARIOS:
        # 入力パスと出力パスの構築
        input_folder = BASE_INPUT_DIR / scenario_name
        
        output_folder = BASE_OUTPUT_DIR / scenario_name
        
        print("\n" + "*" * 60)
        print(f"シナリオ処理開始: {scenario_name}")
        print("*" * 60)
        print(f"  Input : {input_folder}")
        print(f"  Output: {output_folder}")

        # フォルダが存在するか確認
        if not input_folder.exists():
            print(f"  [警告] 入力フォルダが見つからないためスキップします: {input_folder}")
            continue

        GRID_SIZE_PAIRS = [(1.0,1.0),(1.75,1.75),(3.5,3.5),(EGO_HALF_LENGTH,EGO_HALF_WIDTH),(VEHICLE_LENGTH,VEHICLE_WIDTH),(2.0,2.0),(4.0,4.0)]
        # 処理実行
        process_folder_batch(input_folder, output_folder, GRID_SIZE_PAIRS)

    print("\n" + "=" * 60)
    print("全シナリオの処理が完了しました。")
    print("=" * 60)

     
    