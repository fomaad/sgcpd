import json
import numpy as np
from math import cos, sin, atan2
import os
from pathlib import Path
from tqdm import tqdm
import pandas as pd  # CSV出力用

# ==========================================
# 定数定義
# ==========================================
VEHICLE_WIDTH = 1.9   # 車幅 (m)
VEHICLE_LENGTH = 5.3  # 車長 (m)

EGO_HALF_WIDTH = VEHICLE_WIDTH / 2.0
EGO_HALF_LENGTH = VEHICLE_LENGTH / 2.0

V_MAX = 120.0  # 最大相対速度 (m/s)
FAR_DISTANCE = 40.0 # 遠方判定の距離 (m)

# ==========================================
# 相対位置を判定する関数群 
# (rxを前後方向、ryを左右方向として定義)
# ==========================================

# --- Front (前方) ---
def lead_left(rx, ry):
    return ry >= EGO_HALF_WIDTH and EGO_HALF_LENGTH <= rx < FAR_DISTANCE

def lead_0(rx, ry):
    return -EGO_HALF_WIDTH < ry < EGO_HALF_WIDTH and EGO_HALF_LENGTH <= rx < FAR_DISTANCE

def lead_right(rx, ry):
    return ry <= -EGO_HALF_WIDTH and EGO_HALF_LENGTH <= rx < FAR_DISTANCE

# --- Side (自車横) ---
def ego_zone(rx, ry):
    return -EGO_HALF_WIDTH < ry < EGO_HALF_WIDTH and -EGO_HALF_LENGTH <= rx < EGO_HALF_LENGTH

def right(rx, ry):
    return ry <= -EGO_HALF_WIDTH and -EGO_HALF_LENGTH <= rx < EGO_HALF_LENGTH

def left(rx, ry):
    return ry >= EGO_HALF_WIDTH and -EGO_HALF_LENGTH <= rx < EGO_HALF_LENGTH

# --- Follow (後方近傍) : -40 < rx < -L/2 ---
def follow_0(rx, ry):
    return -EGO_HALF_WIDTH < ry < EGO_HALF_WIDTH and -FAR_DISTANCE < rx < -EGO_HALF_LENGTH

def follow_right(rx, ry):
    return ry <= -EGO_HALF_WIDTH and -FAR_DISTANCE < rx < -EGO_HALF_LENGTH

def follow_left(rx, ry):
    return ry >= EGO_HALF_WIDTH and -FAR_DISTANCE < rx < -EGO_HALF_LENGTH

# --- Far Front (遠方前方) : rx >= 40 ---
def far_front(rx, ry):
    return  -EGO_HALF_WIDTH < ry < EGO_HALF_WIDTH and rx >= FAR_DISTANCE 

def far_left(rx, ry):
    return ry >= EGO_HALF_WIDTH and rx >= FAR_DISTANCE

def far_right(rx, ry):
    return ry <= -EGO_HALF_WIDTH and rx >= FAR_DISTANCE

# --- Far Rear (遠方後方) : rx <= -40 ---
def far_rear(rx, ry):
    return -EGO_HALF_WIDTH < ry < EGO_HALF_WIDTH and rx <= -FAR_DISTANCE

def far_rear_left(rx, ry):
    return ry >= EGO_HALF_WIDTH and rx <= -FAR_DISTANCE

def far_rear_right(rx, ry):
    return ry <= -EGO_HALF_WIDTH and rx <= -FAR_DISTANCE


# 具体空間のラベル判定関数 (rxを前後方向、ryを左右方向として定義)
def concrete_same_lane(rx, ry):
    """same-lane: |ry| (左右) < EGO_HALF_WIDTH"""
    return abs(ry) < EGO_HALF_WIDTH

def concrete_other_lane(rx, ry):
    """other-lane: |ry| (左右) >= EGO_HALF_WIDTH"""
    return abs(ry) >= EGO_HALF_WIDTH

def concrete_lead(rx, ry):
    """lead: rx (前後) >= EGO_HALF_LENGTH"""
    return rx >= EGO_HALF_LENGTH

# 判定ラベルと関数の対応（優先順位順）
POSITION_LABELS = [
    # Far Front
    ("far-front", far_front),
    ("far-left", far_left),
    ("far-right", far_right),
    
    # Lead
    ("lead0", lead_0),
    ("lead-right", lead_right),
    ("lead-left", lead_left),
    
    # Side
    ("ego", ego_zone),
    ("right", right),
    ("left", left),
    
    # Follow (Near)
    ("follow_0", follow_0),
    ("follow-right", follow_right),
    ("follow-left", follow_left),
    
    # Far Rear
    ("far-rear", far_rear),
    ("far-rear-right", far_rear_right),
    ("far-rear-left", far_rear_left),
]

# ==========================================
# 相対速度計算・ラベル判定
# ==========================================

def calculate_relative_velocity(ego_velocities, npc_velocities):
    """
    相対速度を計算 (NPC - Ego)
    正の値: NPCがEgoより速い（遠ざかる、あるいは後ろから追いつく）
    負の値: NPCがEgoより遅い（近づく、あるいは後ろへ離れる）
    """
    rel_velocities = []
    
    for i in range(len(ego_velocities)):
        if (ego_velocities[i][0] is not None and npc_velocities[i][0] is not None):
            # x軸(Longitudinal)の差分
            rel_v = npc_velocities[i][0] - ego_velocities[i][0]
            rel_velocities.append(rel_v)
        else:
            rel_velocities.append(None)
    
    return np.array(rel_velocities)

def abstract_rel_vel_label(rel_v, threshold=0.0):
    """抽象空間の相対速度ラベル判定"""
    if rel_v is None:
        return None
    
    if rel_v > threshold:
        return "abstract_faster" 
    elif rel_v < -threshold:
        return "abstract_slower" 
    else:
        return None

def concrete_rel_vel_label(rel_v, threshold=0):
    """相対速度から具体空間のラベルを判定"""
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

def get_relative_position_with_values(ego_pos, npc_pos):
    """egoとnpcの相対位置と実数値を判定"""
    if npc_pos[0] is None or npc_pos[1] is None:
        return "データなし", None, None

    rx = npc_pos[0] - ego_pos[0]  # 前後方向
    ry = npc_pos[1] - ego_pos[1]  # 左右方向

    for label, func in POSITION_LABELS:
        if func(rx, ry):
            return label, rx, ry

    return "その他", rx, ry

def get_concrete_space_labels(rx, ry, rel_v):
    """具体空間のラベルを取得"""
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
            
            npc_id = npc_data[0].get('id')
            if npc_id is None:
                npc_id = npc_data[0].get('name')
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
        if ego_normalized[i, 0] is not None and ego_normalized[0, 0] is not None:
            movement = ego_normalized[i] - ego_normalized[0]
            if np.linalg.norm(movement) > 0.1:
                direction_vector = movement / np.linalg.norm(movement)
                break

    if direction_vector is None:
        if ego_normalized[-1, 0] is not None and ego_normalized[0, 0] is not None:
            total_movement = ego_normalized[-1] - ego_normalized[0]
            if np.linalg.norm(total_movement) > 0.1:
                direction_vector = total_movement / np.linalg.norm(total_movement)
            else:
                direction_vector = np.array([1, 0])
        else:
             direction_vector = np.array([1, 0])

    current_angle = atan2(direction_vector[1], direction_vector[0])
    rotation_angle = -current_angle

    cos_theta = cos(rotation_angle)
    sin_theta = sin(rotation_angle)
    rotation_matrix = np.array([[cos_theta, -sin_theta],
                               [sin_theta, cos_theta]])

    ego_rotated = []
    for coord in ego_normalized:
        if coord[0] is not None and coord[1] is not None:
            rotated = np.dot(coord, rotation_matrix.T)
            ego_rotated.append(rotated)
        else:
            ego_rotated.append([None, None])
    ego_rotated = np.array(ego_rotated)

    npc_rotated = []
    for coord in npc_normalized:
        if coord[0] is not None and coord[1] is not None:
            rotated = np.dot(coord, rotation_matrix.T)
            npc_rotated.append(rotated)
        else:
            npc_rotated.append([None, None])
    npc_rotated = np.array(npc_rotated)
    
    return ego_rotated, npc_rotated, rotation_matrix

def normalize_velocities(ego_vel, npc_vel, rotation_matrix):
    ego_vel_rotated = []
    for vel in ego_vel:
        if vel[0] is not None and vel[1] is not None:
            rotated = np.dot(vel, rotation_matrix.T)
            ego_vel_rotated.append(rotated)
        else:
            ego_vel_rotated.append([None, None])
    
    npc_vel_rotated = []
    for vel in npc_vel:
        if vel[0] is not None and vel[1] is not None:
            rotated = np.dot(vel, rotation_matrix.T)
            npc_vel_rotated.append(rotated)
        else:
            npc_vel_rotated.append([None, None])
    
    return np.array(ego_vel_rotated), np.array(npc_vel_rotated)

def generate_output_text(ego_coords, npc_coords, ego_vel, npc_vel, timestamps, npc_ids):
    """
    座標と相対位置の出力テキストを生成
    """
    output_lines = []
    npc_rel_velocities = calculate_relative_velocity(ego_vel, npc_vel)
    
    processed_data = []
    prev_relative_pos = None
    prev_concrete_lane = None
    prev_concrete_lead = None
    prev_concrete_rv = None   
    prev_abstract_rv = None   
    prev_npc_id = None
    prev_npc_switched = False
    start_time = None
    
    rx_values = []
    ry_values = []
    
    for i in range(len(timestamps)):
        time_index = i + 1
        current_npc_id = npc_ids[i]
        current_abstract_rv = abstract_rel_vel_label(npc_rel_velocities[i])
        
        if (i < len(npc_coords) and npc_coords[i][0] is not None and not np.isnan(npc_coords[i][0])):
            relative_pos, rx, rz = get_relative_position_with_values(ego_coords[i], npc_coords[i])
            rx_values.append(rx)
            ry_values.append(rz)
            concrete_lane, concrete_lead, concrete_rv = get_concrete_space_labels(rx, rz, npc_rel_velocities[i])
        else:
            relative_pos = "データなし"
            concrete_lane = concrete_lead = concrete_rv = current_npc_id = None
            rx_values.append(None)
            ry_values.append(None)
        
        npc_switched = i > 0 and current_npc_id != prev_npc_id and (current_npc_id is not None or prev_npc_id is not None)

        if (relative_pos == prev_relative_pos and concrete_lane == prev_concrete_lane and
            concrete_lead == prev_concrete_lead and concrete_rv == prev_concrete_rv and
            current_abstract_rv == prev_abstract_rv and not npc_switched):
            continue
        else:
            if start_time is not None:
                processed_data.append({
                    'time_range': (start_time, i),
                    'relative_pos': prev_relative_pos,
                    'concrete_lane': prev_concrete_lane,
                    'concrete_lead': prev_concrete_lead,
                    'concrete_rv': prev_concrete_rv,
                    'abstract_rv': prev_abstract_rv,
                    'npc_id': prev_npc_id,
                    'npc_switched': prev_npc_switched
                })
            start_time, prev_relative_pos, prev_concrete_lane = time_index, relative_pos, concrete_lane
            prev_concrete_lead, prev_concrete_rv, prev_abstract_rv = concrete_lead, concrete_rv, current_abstract_rv
            prev_npc_id, prev_npc_switched = current_npc_id, npc_switched
    
    # 最後のデータを追加
    if start_time is not None:
        processed_data.append({
            'time_range': (start_time, len(timestamps)),
            'relative_pos': prev_relative_pos,
            'concrete_lane': prev_concrete_lane,
            'concrete_lead': prev_concrete_lead,
            'concrete_rv': prev_concrete_rv,
            'abstract_rv': prev_abstract_rv,
            'npc_id': prev_npc_id,
            'npc_switched': prev_npc_switched
        })

    # ヘッダーの定義
    header = f"{'Time':<20} {'抽象空間':<15} {'具体空間':<30} {'備考':<30}"
    output_lines.append(header)
    output_lines.append("-" * len(header))

    # テーブル行の生成
    state_id = 1
    for data in processed_data:
        start_t, end_t = data['time_range']
        time_str = f"{state_id}({start_t})" if start_t == end_t else f"{state_id}({start_t}-{end_t})"
        
        abstract_space = data['relative_pos']
        
        # 具体空間の文字列整形
        if data['concrete_lane'] is not None:
            lead_str = "lead" if data['concrete_lead'] else "follow"
            rv_sym = "+RV" if data['concrete_rv'] == "concrete_faster" else "-RV" if data['concrete_rv'] == "concrete_slower" else "=RV"
            concrete_str = f"{data['concrete_lane']}+{lead_str}({rv_sym})"
        else:
            concrete_str = "データなし"
        
        # 備考欄の構築
        remarks_list = []
        if data['abstract_rv'] == "abstract_faster":
            remarks_list.append("【加速】")
        elif data['abstract_rv'] == "abstract_slower":
            remarks_list.append("【減速】")
            
        if data['npc_switched']:
            remarks_list.append(f"NPC切替({data['npc_id']})")
        
        remarks_str = " ".join(remarks_list)
        
        line = f"{time_str:<20} {abstract_space:<15} {concrete_str:<30} {remarks_str:<30}"
        output_lines.append(line)
        state_id += 1
    
    # カットイン・カットアウト判定
    abstract_cutin_detected = False
    abstract_cutin_info = []
    for i in range(1, len(processed_data)):
        prev_pos = processed_data[i-1]['relative_pos']
        curr_pos = processed_data[i]['relative_pos']
        
        # カットイン: 側方/遠方側方 -> 中央(lead/follow/far-front/far-rear)
        if (prev_pos == "lead-right" and curr_pos in ["lead0"]) or \
            (prev_pos == "lead-left" and curr_pos in ["lead0"]) or \
            (prev_pos == "right" and curr_pos in ["lead0"]) or \
            (prev_pos == "left" and curr_pos in ["lead0"]) or \
            (prev_pos == "far-right" and curr_pos in ["far-front"]) or \
            (prev_pos == "far-left" and curr_pos in ["far-front"]):
            
            abstract_cutin_detected = True
            abstract_cutin_info.append(f"{prev_pos}(状態{i}) → {curr_pos}(状態{i+1})")

    abstract_cutout_detected = False
    abstract_cutout_info = []
    for i in range(1, len(processed_data)):
        prev_pos = processed_data[i-1]['relative_pos']
        curr_pos = processed_data[i]['relative_pos']
        
        # カットアウト: 中央 -> 側方
        if (prev_pos == "lead0" and curr_pos in ["lead-right", "lead-left"]) or \
            (prev_pos == "far-front" and curr_pos in ["far-right", "far-left"]) or \
            (prev_pos == "far-rear" and curr_pos in ["far-rear-right", "far-rear-left"]):
            
            abstract_cutout_detected = True
            abstract_cutout_info.append(f"{prev_pos}(状態{i}) → {curr_pos}(状態{i+1})")
    
    concrete_cutin_detected = False
    concrete_cutin_info = []
    for i in range(1, len(processed_data)):
        prev_lane = processed_data[i-1]['concrete_lane']
        curr_lane = processed_data[i]['concrete_lane']
        curr_lead = processed_data[i]['concrete_lead']
        if (prev_lane == "other-lane" and curr_lane == "same-lane" and curr_lead == True):
            concrete_cutin_detected = True
            concrete_cutin_info.append(f"other-lane(状態{i}) → same-lane+lead(状態{i+1})")

    concrete_cutout_detected = False
    concrete_cutout_info = []
    for i in range(1, len(processed_data)):
        prev_lane = processed_data[i-1]['concrete_lane']
        curr_lane = processed_data[i]['concrete_lane']
        prev_lead = processed_data[i-1]['concrete_lead']
        if (prev_lane == "same-lane" and curr_lane == "other-lane" and prev_lead == True):
            concrete_cutout_detected = True
            concrete_cutout_info.append(f"same-lane+lead(状態{i}) → other-lane(状態{i+1})")

    # ==========================================
    # 加速シナリオ・減速シナリオの判定
    # ==========================================

    # 1. 抽象空間での加速シナリオ判定
    abstract_accel_detected = False
    abstract_accel_info = []
    
    # 2. 抽象空間での減速シナリオ判定
    abstract_decel_detected = False
    abstract_decel_info = []

    for i in range(len(processed_data)):
        curr_abs_rv = processed_data[i]['abstract_rv']
        curr_pos = processed_data[i]['relative_pos']
        
        # 加速シナリオ (Abstract): Faster かつ 後方(Follow/Far-Rear)
        if curr_abs_rv == "abstract_faster":
            if curr_pos in ["follow_0", "follow-left", "follow-right", 
                            "far-rear", "far-rear-left", "far-rear-right"]:
                abstract_accel_detected = True
                abstract_accel_info.append(f"状態{i+1}({curr_pos})")
        
        # 減速シナリオ (Abstract): Slower かつ 前方(Lead/Far-Front)
        elif curr_abs_rv == "abstract_slower":
            if curr_pos in ["lead0", "lead-left", "lead-right",
                            "far-front", "far-left", "far-right"]: # Far-Frontも念のため追加
                abstract_decel_detected = True
                abstract_decel_info.append(f"状態{i+1}({curr_pos})")

    # 3. 具体空間での加速・減速シナリオ判定 (rxの変化による定義に変更)
    
    concrete_accel_frames = []
    concrete_decel_frames = []
    
    # 全フレームを走査して判定
    for i in range(1, len(rx_values)):
        rx = rx_values[i]
        prev_rx = rx_values[i-1]
        
        if rx is not None and prev_rx is not None:
            # 減速 (前方で距離が縮まる)
            if rx >= 0 and rx < prev_rx and prev_rx >= 0:
                concrete_decel_frames.append(i + 1)
            
            # 加速 (後方で距離が縮まる)
            if rx < 0 and rx > prev_rx and prev_rx < 0:
                concrete_accel_frames.append(i + 1)

    # 連続するフレームを範囲テキスト（例："10-15"）に変換する関数
    def get_frame_ranges(frames):
        if not frames:
            return []
        ranges = []
        start = frames[0]
        prev = frames[0]
        for f in frames[1:]:
            if f == prev + 1:
                prev = f
            else:
                ranges.append(f"{start}-{prev}" if start != prev else f"{start}")
                start = f
                prev = f
        ranges.append(f"{start}-{prev}" if start != prev else f"{start}")
        return ranges

    concrete_accel_info = [f"時刻 {r}" for r in get_frame_ranges(concrete_accel_frames)]
    concrete_accel_detected = len(concrete_accel_info) > 0

    concrete_decel_info = [f"時刻 {r}" for r in get_frame_ranges(concrete_decel_frames)]
    concrete_decel_detected = len(concrete_decel_info) > 0

    # 圧縮率を計算
    original_count = len(timestamps)
    compressed_count = len(processed_data)
    compression_ratio = (1 - compressed_count / original_count) * 100 if original_count > 0 else 0

    # 検出結果を辞書にまとめる
    detection_results = {
        'abstract_cutin': abstract_cutin_detected,
        'abstract_cutout': abstract_cutout_detected,
        'abstract_accel': abstract_accel_detected, 
        'abstract_decel': abstract_decel_detected, 
        'concrete_cutin': concrete_cutin_detected,
        'concrete_cutout': concrete_cutout_detected,
        'concrete_accel': concrete_accel_detected, 
        'concrete_decel': concrete_decel_detected,
        'num_abstract_states': compressed_count,  # 圧縮後の状態数
        'total_frames': original_count            # 元のフレーム数
    }

    output_lines.append("-" * len(header))
    output_lines.append(f"元データ数: {original_count}, 圧縮後: {compressed_count}, 圧縮率: {compression_ratio:.1f}%")
    
    # サマリー出力    
    output_lines.append("")
    output_lines.append("=" * 60)
    output_lines.append("【抽象空間での検出結果】")
    output_lines.append("=" * 60)
    
    output_lines.append("")
    output_lines.append("カットイン検出:")
    if abstract_cutin_detected:
        for info in abstract_cutin_info:
            output_lines.append(f"  ✓ {info}")
    else:
        output_lines.append("  なし")

    output_lines.append("")
    output_lines.append("カットアウト検出:")
    if abstract_cutout_detected:
        for info in abstract_cutout_info:
            output_lines.append(f"  ✓ {info}")
    else:
        output_lines.append("  なし")

    output_lines.append("")
    output_lines.append("加速シナリオ検出 (Faster & Follow/Far-Rear):")
    if abstract_accel_detected:
        for info in abstract_accel_info:
            output_lines.append(f"  ✓ {info}")
    else:
        output_lines.append("  なし")
    
    output_lines.append("")
    output_lines.append("減速シナリオ検出 (Slower & Lead/Far-Front):")
    if abstract_decel_detected:
        for info in abstract_decel_info:
            output_lines.append(f"  ✓ {info}")
    else:
        output_lines.append("  なし")
    
    output_lines.append("")
    output_lines.append("=" * 60)
    output_lines.append("【具体空間での検出結果】")
    output_lines.append("=" * 60)
    
    output_lines.append("")
    output_lines.append("カットイン検出:")
    if concrete_cutin_detected:
        for info in concrete_cutin_info:
            output_lines.append(f"  ✓ {info}")
    else:
        output_lines.append("  なし")

    output_lines.append("")
    output_lines.append("カットアウト検出:")
    if concrete_cutout_detected:
        for info in concrete_cutout_info:
            output_lines.append(f"  ✓ {info}")
    else:
        output_lines.append("  なし")

    output_lines.append("")
    output_lines.append("加速シナリオ検出 (後方接近):")
    if concrete_accel_detected:
        output_lines.append(f"  検出数: {len(concrete_accel_info)}区間")
        for info in concrete_accel_info:
            output_lines.append(f"  ✓ {info}")
    else:
        output_lines.append("  なし")
    
    output_lines.append("")
    output_lines.append("減速シナリオ検出 (前方接近):")
    if concrete_decel_detected:
        output_lines.append(f"  検出数: {len(concrete_decel_info)}区間")
        for info in concrete_decel_info:
            output_lines.append(f"  ✓ {info}")
    else:
        output_lines.append("  なし")

    return "\n".join(output_lines), detection_results

def process_file(input_path, output_path):
    """単一ファイルを処理"""
    try:
        data = load_json_data(input_path)
        
        ego_coords, npc_coords, ego_vel, npc_vel, timestamps, npc_ids = extract_coordinates_from_json(data)
        
        if len(timestamps) == 0:
            return False, "データ(タイムスタンプ)が抽出できませんでした。", None
        
        ego_normalized, npc_normalized, rotation_matrix = normalize_coordinates(ego_coords, npc_coords)
        ego_vel_normalized, npc_vel_normalized = normalize_velocities(ego_vel, npc_vel, rotation_matrix)
        
        output_text, detection_results = generate_output_text(
            ego_normalized, 
            npc_normalized, 
            ego_vel_normalized, 
            npc_vel_normalized, 
            timestamps, 
            npc_ids
        )
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(f"=== {input_path.name} の処理結果 ===\n\n")
            f.write(output_text)
        
        return True, None, detection_results
    
    except Exception as e:
        return False, str(e), None

def process_folder(input_folder, output_folder):
    """フォルダ内の全JSONファイルを処理 (CSV出力付き)"""
    input_path = Path(input_folder)
    output_path = Path(output_folder)
    
    if not input_path.exists():
        print(f"エラー: 入力フォルダ '{input_folder}' が見つかりません。")
        return [] # リストを返す
    
    output_path.mkdir(parents=True, exist_ok=True)
    
    json_files = list(input_path.glob("*.json"))
    
    if not json_files:
        print(f"警告: '{input_folder}' 内にJSONファイルが見つかりません。")
        return [] # リストを返す
    
    print(f"\n処理対象ファイル数: {len(json_files)}")
    print("=" * 60)
    
    success_count = 0
    error_count = 0
    
    detection_stats = {
        'abstract_cutin': {'count': 0, 'files': []},
        'abstract_cutout': {'count': 0, 'files': []},
        'abstract_accel': {'count': 0, 'files': []}, 
        'abstract_decel': {'count': 0, 'files': []}, 
        'concrete_cutin': {'count': 0, 'files': []},
        'concrete_cutout': {'count': 0, 'files': []},
        'concrete_accel': {'count': 0, 'files': []}, 
        'concrete_decel': {'count': 0, 'files': []}   
    }
    
    discrepancy_stats = {
        'cutin': {'label': 'カットイン', 'abs_only': [], 'con_only': []},
        'cutout': {'label': 'カットアウト', 'abs_only': [], 'con_only': []},
        'accel': {'label': '加速シナリオ', 'abs_only': [], 'con_only': []}, 
        'decel': {'label': '減速シナリオ', 'abs_only': [], 'con_only': []}   
    }
    
    results = []
    tasks = []
    error_files = []
    
    # CSV分析用のデータ蓄積リスト
    all_csv_records = []
    
    # プログレスバーで処理
    for json_file in tqdm(json_files, desc=f"処理中: {input_path.name}", unit="file"):
        output_file = output_path / f"{json_file.stem}_result.txt"
        tasks.append((json_file, output_file))
        
        success, error, detection_results = process_file(json_file, output_file)
        results.append((success, error, detection_results))
        
        if not success:
            error_files.append((json_file.name, error))
        else:
            # 成功時にCSV用レコードを作成
            if detection_results:
                record = {
                    'filename': json_file.name,
                    'total_frames': detection_results['total_frames'],
                    'num_abstract_states': detection_results['num_abstract_states'],
                    'compression_ratio': (1 - detection_results['num_abstract_states']/detection_results['total_frames']) * 100 if detection_results['total_frames'] > 0 else 0,
                    'abstract_cutin': detection_results['abstract_cutin'],
                    'concrete_cutin': detection_results['concrete_cutin'],
                    'abstract_cutout': detection_results['abstract_cutout'],
                    'concrete_cutout': detection_results['concrete_cutout'],
                    'abstract_accel': detection_results['abstract_accel'],
                    'concrete_accel': detection_results['concrete_accel'],
                    'abstract_decel': detection_results['abstract_decel'],
                    'concrete_decel': detection_results['concrete_decel']
                }
                all_csv_records.append(record)
    
    # 集計処理
    for i, (success, error, detection_results) in enumerate(results):
        original_file_name = tasks[i][0].name 
        
        if success and detection_results:
            success_count += 1
            
            for key in detection_stats.keys():
                if detection_results[key]:
                    detection_stats[key]['files'].append(original_file_name)
                    detection_stats[key]['count'] += 1
            
            checks = [
                ('cutin', 'abstract_cutin', 'concrete_cutin'),
                ('cutout', 'abstract_cutout', 'concrete_cutout'),
                ('accel', 'abstract_accel', 'concrete_accel'),
                ('decel', 'abstract_decel', 'concrete_decel') 
            ]
            
            for key, abs_key, con_key in checks:
                abs_val = detection_results[abs_key]
                con_val = detection_results[con_key]
                
                if abs_val and not con_val:
                    discrepancy_stats[key]['abs_only'].append(original_file_name)
                elif con_val and not abs_val:
                    discrepancy_stats[key]['con_only'].append(original_file_name)

        elif not success:
            error_count += 1
    
    # フォルダごとのCSVも保存（維持）
    if all_csv_records:
        csv_path = output_path / "summary_all.csv"
        df = pd.DataFrame(all_csv_records)
        df.to_csv(csv_path, index=False, encoding='utf-8-sig')
        print(f"\n★フォルダ別分析用CSVデータを保存しました: {csv_path}")

    # テキストサマリー出力用リスト
    summary_lines = []
    def add_summary(line=""):
        summary_lines.append(str(line))
    
    # コンソール出力とファイル書き込み用サマリー作成
    print("\n" + "=" * 80)
    print("【処理完了サマリー】")
    print("=" * 80)
    print(f"処理完了: 成功 {success_count}件, 失敗 {error_count}件")
    
    add_summary("\n" + "=" * 80)
    add_summary("【処理完了サマリー】")
    add_summary("=" * 80)
    add_summary(f"処理完了: 成功 {success_count}件, 失敗 {error_count}件")

    if error_files:
        print("\n【エラー詳細】")
        for filename, error in error_files:
            print(f"  ✗ {filename}: {error}")
    
    add_summary("\n" + "=" * 80)
    add_summary("【抽象空間での検出統計】")
    add_summary("=" * 80)
    
    for key, label in [('abstract_cutin', 'カットイン'), ('abstract_cutout', 'カットアウト'), 
                       ('abstract_accel', '加速シナリオ'), ('abstract_decel', '減速シナリオ')]:
        count = detection_stats[key]['count']
        rate = (count/success_count*100) if success_count > 0 else 0
        add_summary(f"\n{label}検出:")
        add_summary(f"   検出ファイル数: {count}件")
        if success_count > 0:
            add_summary(f"   検出率: {rate:.1f}%")
    
    add_summary("\n" + "=" * 80)
    add_summary("【具体空間での検出統計】")
    add_summary("=" * 80)
    
    for key, label in [('concrete_cutin', 'カットイン'), ('concrete_cutout', 'カットアウト'), 
                       ('concrete_accel', '加速シナリオ'), ('concrete_decel', '減速シナリオ')]:
        count = detection_stats[key]['count']
        rate = (count/success_count*100) if success_count > 0 else 0
        add_summary(f"\n{label}検出:")
        add_summary(f"   検出ファイル数: {count}件")
        if success_count > 0:
            add_summary(f"   検出率: {rate:.1f}%")

    add_summary("\n" + "=" * 80)
    add_summary("【検出結果の差異サマリー (抽象 vs 具体)】")
    add_summary("=" * 80)
    
    for key, data in discrepancy_stats.items():
        label = data['label']
        add_summary(f"\n■ {label}の差異:")
        
        abs_files = data['abs_only']
        if abs_files:
            add_summary(f"  [抽象○ / 具体×] ({len(abs_files)}件):")
            for f in abs_files:
                add_summary(f"    - {f}")
        else:
            add_summary("  [抽象○ / 具体×]: なし")
            
        con_files = data['con_only']
        if con_files:
            add_summary(f"  [抽象× / 具体○] ({len(con_files)}件):")
            for f in con_files:
                add_summary(f"    - {f}")
        else:
            add_summary("  [抽象× / 具体○]: なし")

    add_summary("\n" + "=" * 80)
    add_summary("【詳細ファイルリスト】")
    add_summary("=" * 80)
    
    for key, label in [
        ('abstract_cutin', '抽象空間カットイン'),
        ('abstract_cutout', '抽象空間カットアウト'),
        ('abstract_accel', '抽象空間加速シナリオ'),
        ('abstract_decel', '抽象空間減速シナリオ'),
        ('concrete_cutin', '具体空間カットイン'),
        ('concrete_cutout', '具体空間カットアウト'),
        ('concrete_accel', '具体空間加速シナリオ'),
        ('concrete_decel', '具体空間減速シナリオ')
    ]:
        add_summary(f"\n{label}検出ファイル ({detection_stats[key]['count']}件):")
        if detection_stats[key]['count'] > 0 and detection_stats[key]['count'] <= 10:
            for filename in sorted(detection_stats[key]['files']):
                add_summary(f"  • {filename}")
        elif detection_stats[key]['count'] > 10:
            for filename in sorted(detection_stats[key]['files'])[:5]:
                add_summary(f"  • {filename}")
            add_summary(f"  ... (他 {detection_stats[key]['count'] - 5}件)")
    
    add_summary("\n" + "=" * 80)

    summary_path = output_path / "summary.txt"
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("\n".join(summary_lines))

    print(f"\nテキストサマリーを '{summary_path}' に保存しました。")
    
    # 呼び出し元で集計するためにデータを返す
    return all_csv_records

# メイン処理
if __name__ == "__main__":
    BASE_INPUT_DIR = Path("../data/generated_trajectories")
    BASE_OUTPUT_DIR = Path("../out/15area")

    all_scenario_data = []

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
        input_folder = BASE_INPUT_DIR / scenario_name
        output_folder = BASE_OUTPUT_DIR / scenario_name
        
        print("\n" + "*" * 60)
        print(f"シナリオ処理開始: {scenario_name}")
        print("*" * 60)
        
        # 処理結果のリストを受け取る
        scenario_records = process_folder(input_folder, output_folder)
        
        # シナリオ名を追加して統合リストに追加
        if scenario_records:
            for record in scenario_records:
                record['scenario'] = scenario_name
            all_scenario_data.extend(scenario_records)

    # 全体サマリーCSVの保存
    if all_scenario_data:
        print("\n" + "=" * 60)
        print("全体サマリーCSVを生成しています...")
        
        csv_output_path = BASE_OUTPUT_DIR / "summary_all.csv"
        # 親ディレクトリがない場合は作成
        csv_output_path.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            df = pd.DataFrame(all_scenario_data)
            # カラムの順序を整える: scenario, filename を先頭に
            cols = ['scenario', 'filename'] + [c for c in df.columns if c not in ['scenario', 'filename']]
            df = df[cols]
            
            df.to_csv(csv_output_path, index=False, encoding='utf-8-sig')
            print(f"全体サマリーCSV保存完了: {csv_output_path}")
        except Exception as e:
            print(f"CSV保存エラー: {e}")
    else:
        print("\n処理されたデータがないため、全体サマリーCSVは生成されませんでした。")

    print("\n" + "=" * 60)
    print("全シナリオの処理が完了しました。")
    print("=" * 60)