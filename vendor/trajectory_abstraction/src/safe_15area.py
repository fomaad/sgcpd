import json
import numpy as np
from math import cos, sin, atan2
import os
from pathlib import Path
from tqdm import tqdm
import pandas as pd

# ==========================================
# 定数定義
# ==========================================
VEHICLE_WIDTH = 1.9
VEHICLE_LENGTH = 5.3
EGO_HALF_WIDTH = VEHICLE_WIDTH / 2.0
EGO_HALF_LENGTH = VEHICLE_LENGTH / 2.0
V_MAX = 33.33  # 約120 km/h
TTC_THRESHOLD = 2.0
FAR_DISTANCE = 40.0 # 遠方判定距離

# ==========================================
# 領域判定関数 (15領域版)
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


def concrete_same_lane(rx, ry): return abs(ry) < EGO_HALF_WIDTH
def concrete_lead(rx, ry): return rx >= EGO_HALF_LENGTH

# 判定優先順位定義
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
    
    # Far Rear (追加)
    ("far-rear", far_rear),
    ("far-rear-right", far_rear_right),
    ("far-rear-left", far_rear_left),
]

# ==========================================
# 安全性判定ロジック
# ==========================================

def calculate_ttc(rx, rel_v):
    ttc = float('inf')
    if rx is not None and rel_v is not None and rel_v != 0:
        # 衝突コース判定: 前方(rx>0)で遅い(rel_v<0) or 後方(rx<0)で速い(rel_v>0) ※rel_vは相対速度
        if (rx > 0 and rel_v < 0) or (rx < 0 and rel_v > 0):
            ttc = abs(rx / rel_v)
    return ttc

def is_concrete_safe(rx, ry, rel_v):
    ttc = calculate_ttc(rx, rel_v)
    # 具体空間における安全判定: 横距離が2.78m超 または TTCが2.0s超
    is_safe = abs(ry) > 2.78 or ttc > 2.0
    return is_safe, ttc

def is_abstract_safe(position_label, abstract_rv):
    if position_label is None or abstract_rv is None: return True
    
    # 安全性定義のためのグループ分け
    leads = ["lead0", "lead-left", "lead-right"]
    # Far Rearも「後方」として扱い、Slower(離れる)なら安全とする
    follows = ["follow_0", "follow-left", "follow-right", "far-rear", "far-rear-left", "far-rear-right"]
    far_fronts = ["far-front", "far-right", "far-left"]
    far_rears = ["far-rear","far-rear-right","far-rear-left"]
    
    is_acpos = (abstract_rv == "abstract_faster") # 遠ざかる(前方)、近づく(後方)
    is_acneg = (abstract_rv == "abstract_slower") # 近づく(前方)、遠ざかる(後方)

    # 安全性プロパティ (Safe条件)
    # 1. 前方(Lead/FarFront)にいて、Faster(自分より速い=離れていく)なら安全
    # 2. 後方(Follow/FarRear)にいて、Slower(自分より遅い=離れていく)なら安全
    
    far_safe = (position_label in far_fronts) and is_acpos
    far_rear_safe = (position_label in far_rears) and is_acneg
    lead_safe = (position_label in leads) and is_acpos
    follow_safe = (position_label in follows) and is_acneg
    
    return follow_safe or lead_safe or far_safe or far_rear_safe

# ==========================================
# データ処理関数群
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
    if rel_v is None: return None
    if rel_v > threshold: return "abstract_faster" 
    elif rel_v < -threshold: return "abstract_slower" 
    else: return "abstract_equal"

def concrete_rel_vel_label(rel_v, threshold=0):
    if rel_v is None: return None
    if rel_v > threshold: return "concrete_faster"
    elif rel_v < threshold: return "concrete_slower"
    else: return "concrete_equal"

def get_relative_position_with_values(ego_pos, npc_pos):
    if npc_pos[0] is None: return "データなし", None, None
    rx = npc_pos[0] - ego_pos[0]
    ry = npc_pos[1] - ego_pos[1]
    for label, func in POSITION_LABELS:
        if func(rx, ry): return label, rx, ry
    return "その他", rx, ry

def get_concrete_space_labels(rx, ry, rel_v):
    if rx is None: return None, None, None
    lane = "same-lane" if concrete_same_lane(rx, ry) else "other-lane"
    is_lead = concrete_lead(rx, ry)
    rv = concrete_rel_vel_label(rel_v)
    return lane, is_lead, rv

def load_json_data(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        data = json.load(file)
    return data

def extract_coordinates_from_json(data):
    ego_coords, npc_coords = [], []
    ego_vels, npc_vels = [], []
    timestamps, npc_ids = [], []

    if isinstance(data, dict) and 'groundtruth_kinematic' in data: states = data['groundtruth_kinematic']
    elif isinstance(data, list): states = data
    elif isinstance(data, dict) and 'states' in data: states = data['states']
    else: raise ValueError("JSONデータの構造が不明です。")

    for state in states:
        timestamps.append(state.get('timestamp') or state.get('timeStamp'))
        ego = state.get('groundtruth_ego') or state.get('ego')
        if ego:
            ep, et = ego['pose']['position'], ego['twist']['linear']
            ego_coords.append([ep['x'], ep['y']])
            ego_vels.append([et['x'], et['y']])
        else:
            ego_coords.append([None, None]); ego_vels.append([None, None])
            
        npcs = state.get('groundtruth_vehicles') or state.get('groundtruth_NPCs')
        if npcs and len(npcs) > 0:
            np_pos, np_twist = npcs[0]['pose']['position'], npcs[0]['twist']['linear']
            npc_coords.append([np_pos['x'], np_pos['y']])
            npc_vels.append([np_twist['x'], np_twist['y']])
            npc_ids.append(npcs[0].get('id') or npcs[0].get('name'))
        else:
            npc_coords.append([None, None]); npc_vels.append([None, None]); npc_ids.append(None)
            
    return (np.array(ego_coords), np.array(npc_coords), np.array(ego_vels), np.array(npc_vels), np.array(timestamps), np.array(npc_ids))

def normalize_coordinates(ego_coords, npc_coords):
    start = ego_coords[0].copy()
    ego_n = ego_coords - start
    npc_n = npc_coords - start
    vec = np.array([1, 0])
    for i in range(1, min(10, len(ego_n))):
        diff = ego_n[i] - ego_n[0]
        if np.linalg.norm(diff) > 0.1:
            vec = diff / np.linalg.norm(diff)
            break
    angle = -atan2(vec[1], vec[0])
    rot_mat = np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
    def rot(arr): return np.array([np.dot(p, rot_mat.T) if p[0] is not None else [None, None] for p in arr])
    return rot(ego_n), rot(npc_n), rot_mat

def normalize_velocities(ego_vel, npc_vel, rot_mat):
    def rot(arr): return np.array([np.dot(p, rot_mat.T) if p[0] is not None else [None, None] for p in arr])
    return rot(ego_vel), rot(npc_vel)

def generate_output_text(ego_coords, npc_coords, ego_vel, npc_vel, timestamps, npc_ids):
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
    
    # 区間内の安全性（初期値True、一つでも違反があればFalseになる）
    current_state_conc_safe = True
    current_state_abs_safe = True
    
    start_time = None
    rx_values = []
    conc_violation_info = []
    abs_violation_states = []

    for i in range(len(timestamps)):
        time_index = i + 1
        current_npc_id = npc_ids[i]
        rel_v = npc_rel_velocities[i]
        current_abstract_rv = abstract_rel_vel_label(rel_v)
        
        is_conc_safe_now = True
        is_abs_safe_now = True
        
        if (i < len(npc_coords) and npc_coords[i][0] is not None):
            relative_pos, rx, rz = get_relative_position_with_values(ego_coords[i], npc_coords[i])
            rx_values.append(rx)
            concrete_lane, concrete_lead, concrete_rv = get_concrete_space_labels(rx, rz, rel_v)
            
            is_conc_safe_now, ttc_val = is_concrete_safe(rx, rz, rel_v)
            is_abs_safe_now = is_abstract_safe(relative_pos, current_abstract_rv)
            
            if not is_conc_safe_now:
                conc_violation_info.append(f"時刻{time_index}: TTC={ttc_val:.2f}s")
        else:
            relative_pos = "データなし"; concrete_lane = None; concrete_lead = None; concrete_rv = None
            rx_values.append(None)
        
        npc_switched = False
        if i > 0 and current_npc_id != prev_npc_id:
             if current_npc_id is not None or prev_npc_id is not None: npc_switched = True

        if (relative_pos == prev_relative_pos and 
            concrete_lane == prev_concrete_lane and
            concrete_lead == prev_concrete_lead and
            concrete_rv == prev_concrete_rv and
            current_abstract_rv == prev_abstract_rv and
            not npc_switched):
            
            current_state_conc_safe = current_state_conc_safe and is_conc_safe_now
            current_state_abs_safe = current_state_abs_safe and is_abs_safe_now
            continue
        else:
            if start_time is not None:
                end_time = i
                processed_data.append({
                    'time_range': (start_time, end_time),
                    'relative_pos': prev_relative_pos,
                    'concrete_lane': prev_concrete_lane,
                    'concrete_lead': prev_concrete_lead,
                    'concrete_rv': prev_concrete_rv,
                    'abstract_rv': prev_abstract_rv,
                    'npc_id': prev_npc_id,
                    'npc_switched': prev_npc_switched,
                    'is_conc_safe': current_state_conc_safe,
                    'is_abs_safe': current_state_abs_safe
                })
                if not current_state_abs_safe:
                    abs_violation_states.append(f"状態{len(processed_data)}({prev_relative_pos}, {prev_abstract_rv})")
            
            start_time = time_index
            prev_relative_pos = relative_pos
            prev_concrete_lane = concrete_lane
            prev_concrete_lead = concrete_lead
            prev_concrete_rv = concrete_rv
            prev_abstract_rv = current_abstract_rv
            prev_npc_id = current_npc_id
            prev_npc_switched = npc_switched
            current_state_conc_safe = is_conc_safe_now
            current_state_abs_safe = is_abs_safe_now
    
    if start_time is not None:
        end_time = len(timestamps)
        processed_data.append({
            'time_range': (start_time, end_time),
            'relative_pos': prev_relative_pos,
            'concrete_lane': prev_concrete_lane,
            'concrete_lead': prev_concrete_lead,
            'concrete_rv': prev_concrete_rv,
            'abstract_rv': prev_abstract_rv,
            'npc_id': prev_npc_id,
            'npc_switched': prev_npc_switched,
            'is_conc_safe': current_state_conc_safe,
            'is_abs_safe': current_state_abs_safe
        })
        if not current_state_abs_safe:
            abs_violation_states.append(f"状態{len(processed_data)}({prev_relative_pos}, {prev_abstract_rv})")

    # 統計集計
    total_trajectory_conc_safe = all([d['is_conc_safe'] for d in processed_data])
    total_trajectory_abs_safe = all([d['is_abs_safe'] for d in processed_data])

    detection_results = {
        'abstract_safe': total_trajectory_abs_safe,
        'concrete_safe': total_trajectory_conc_safe
    }

    # テキスト出力生成
    header = f"{'Time':<20} {'抽象空間':<15} {'具体空間':<30} {'相対速度':<15} {'Safety(Abs/Conc)':<18} {'NPC_ID':<10}"
    output_lines.append(header)
    output_lines.append("-" * len(header))

    state_id = 1
    for data in processed_data:
        start_t, end_t = data['time_range']
        time_str = f"{state_id}({start_t})" if start_t == end_t else f"{state_id}({start_t}-{end_t})"
        abstract_space = data['relative_pos']
        if data['concrete_lane'] is not None:
            lead_str = "lead" if data['concrete_lead'] else "follow"
            rv_sym = "+RV" if data['concrete_rv'] == "concrete_faster" else ("-RV" if data['concrete_rv'] == "concrete_slower" else "=RV")
            concrete_str = f"{data['concrete_lane']}+{lead_str}({rv_sym})"
        else: concrete_str = "データなし"
        
        if data['abstract_rv'] == "abstract_faster": rv_label = "遠(>0)"
        elif data['abstract_rv'] == "abstract_slower": rv_label = "近(<0)"
        else: rv_label = "-"
        
        abs_safe_mark = "Safe" if data['is_abs_safe'] else "Danger"
        conc_safe_mark = "Safe" if data['is_conc_safe'] else "Danger"
        safety_str = f"{abs_safe_mark} / {conc_safe_mark}"
        npc_id_str = f"{data['npc_id']}" if data['npc_id'] is not None else "N/A"
        
        line = f"{time_str:<20} {abstract_space:<15} {concrete_str:<30} {rv_label:<15} {safety_str:<18} {npc_id_str:<10}"
        output_lines.append(line)
        state_id += 1

    output_lines.append("-" * len(header))
    output_lines.append("")
    output_lines.append("=" * 60)
    output_lines.append("【安全性プロパティ検証結果】")
    output_lines.append("=" * 60)
    output_lines.append(f"軌道全体の抽象安全性 (Abstract Safe): {'[Safe]' if total_trajectory_abs_safe else '[DANGER]'}")
    if not total_trajectory_abs_safe:
        for info in abs_violation_states: output_lines.append(f"    - {info}")
    output_lines.append("")
    output_lines.append(f"軌道全体の具体安全性 (Concrete Safe): {'[Safe]' if total_trajectory_conc_safe else '[DANGER]'}")
    if not total_trajectory_conc_safe:
        for info in conc_violation_info[:5]: output_lines.append(f"    - {info}")

    return "\n".join(output_lines), detection_results

def process_file(input_path, output_path):
    try:
        data = load_json_data(input_path)
        ego_coords, npc_coords, ego_vel, npc_vel, timestamps, npc_ids = extract_coordinates_from_json(data)
        if len(timestamps) == 0: return False, "データ(タイムスタンプ)なし", None
        ego_n, npc_n, rot_mat = normalize_coordinates(ego_coords, npc_coords)
        ego_vn, npc_vn = normalize_velocities(ego_vel, npc_vel, rot_mat)
        
        output_text, detection_results = generate_output_text(ego_n, npc_n, ego_vn, npc_vn, timestamps, npc_ids)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(f"=== {input_path.name} の処理結果 ===\n\n")
            f.write(output_text)
        return True, None, detection_results
    except Exception as e:
        return False, str(e), None

def process_folder(input_folder, output_folder, scenario_name):
    input_path = Path(input_folder)
    output_path = Path(output_folder)
    if not input_path.exists(): print(f"入力フォルダ不明: {input_folder}"); return []
    output_path.mkdir(parents=True, exist_ok=True)
    json_files = list(input_path.glob("*.json"))
    
    print(f"処理対象: {len(json_files)} ファイル")
    
    stats_vel = {'TP': 0, 'TN': 0, 'FP': 0, 'FN': 0}
    stats_acc = {'TP': 0, 'TN': 0, 'FP': 0, 'FN': 0} 

    total_scenarios = 0
    folder_records = []

    for json_file in tqdm(json_files, desc="Processing"):
        output_file = output_path / f"{json_file.stem}_result.txt"
        success, error, res = process_file(json_file, output_file)
        
        if success:
            total_scenarios += 1
            
            abs_safe = res['abstract_safe']
            conc_safe = res['concrete_safe']
            
            # [VEL] Relative Velocity Based Logic
            classification_vel = "Unknown"
            if not abs_safe and not conc_safe:
                stats_vel['TP'] += 1 
                classification_vel = "TP"
            elif abs_safe and conc_safe:
                stats_vel['TN'] += 1 
                classification_vel = "TN"
            elif not abs_safe and conc_safe:
                stats_vel['FP'] += 1 
                classification_vel = "FP"
            elif abs_safe and not conc_safe:
                stats_vel['FN'] += 1 
                classification_vel = "FN"
                
            folder_records.append({
                "scenario": scenario_name,
                "filename": json_file.name,
                "abstract_safe": abs_safe,
                "concrete_safe": conc_safe,
                "result_type": classification_vel
            })

    summary_lines = []
    def add(line=""): summary_lines.append(str(line))
    
    add(f"Grid Size: 15-Region (Lead/Follow/Far-Rear definitions)") 
    add(f"Total Scenarios: {total_scenarios}")
    add()
    
    add("=== Result [VEL]: Relative Velocity Based ===")
    add(f"TP: {stats_vel['TP']}, TN: {stats_vel['TN']}, FP: {stats_vel['FP']}, FN: {stats_vel['FN']}")

    print("\n".join(summary_lines))
    
    with open(output_path / "batch_summary.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(summary_lines))

    return folder_records

if __name__ == "__main__":
    BASE_INPUT_DIR = Path("D:/generated_trajectories")
    BASE_OUTPUT_DIR = Path("D:/datamapping")

    TARGET_SCENARIOS = [
        "accel_scenario_ver2",
        "deccel_scenario_ver2",
        "cutout_scenario_ver2",
        "cutin_scenario_ver2"
    ]

    print("=== 全シナリオの一括処理を開始します ===")
    print(f"Base Input : {BASE_INPUT_DIR}")
    print(f"Base Output: {BASE_OUTPUT_DIR}")

    all_scenario_data = []

    for scenario_name in TARGET_SCENARIOS:
        input_folder = BASE_INPUT_DIR / scenario_name
        # 15領域版なので出力パスを変更
        output_folder = BASE_OUTPUT_DIR / "15area" /"property"/ scenario_name
        
        print("\n" + "*" * 60)
        print(f"シナリオ処理開始: {scenario_name}")
        print("*" * 60)
        
        if not input_folder.exists():
            print(f"  [警告] 入力フォルダが見つからないためスキップします: {input_folder}")
            continue

        scenario_records = process_folder(input_folder, output_folder, scenario_name)
        
        if scenario_records:
            all_scenario_data.extend(scenario_records)

    if all_scenario_data:
        print("\n" + "=" * 60)
        print("全体サマリーCSVを生成しています...")
        
        csv_output_path = BASE_OUTPUT_DIR / "15area" /"property"/ "summary_all.csv"
        csv_output_path.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            df = pd.DataFrame(all_scenario_data)
            cols = ['scenario', 'filename'] + [c for c in df.columns if c not in ['scenario', 'filename']]
            df = df[cols]
            
            df.to_csv(csv_output_path, index=False, encoding='utf-8-sig')
            print(f"CSV保存完了: {csv_output_path}")
        except Exception as e:
            print(f"CSV保存エラー: {e}")
    else:
        print("\n処理されたデータがないため、CSVは生成されませんでした。")

    print("\n" + "=" * 60)
    print("全シナリオの処理が完了しました。")
    print("=" * 60)