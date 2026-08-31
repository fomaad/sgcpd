import json
import numpy as np
from math import cos, sin, atan2, floor
import os
from pathlib import Path
from tqdm import tqdm
import csv  # 追加: CSV出力用

# ==========================================
# 1. 定数定義
# ==========================================
VEHICLE_WIDTH = 1.9
VEHICLE_LENGTH = 5.3
EGO_HALF_WIDTH = VEHICLE_WIDTH / 2.0
EGO_HALF_LENGTH = VEHICLE_LENGTH / 2.0

V_MAX = 120.0
TTC_THRESHOLD = 2.0   # 具体安全性の閾値 (秒)
ACC_THRESHOLD = 0.0   # 加速度判定の閾値 (m/s^2)
VEL_THRESHOLD = 0.0   # 速度差判定の閾値 (m/s)

# 実験対象のグリッドサイズペア (Grid X, Grid Y)
GRID_SIZE_PAIRS = [
    (1.0, 1.0),
    (1.75, 1.75),
    (3.5, 3.5),
    (EGO_HALF_LENGTH, EGO_HALF_WIDTH), # 車体サイズ半分
    (VEHICLE_LENGTH, VEHICLE_WIDTH),   # 車体サイズ
    (2.0, 2.0),
    (4.0, 4.0)
]

# ==========================================
# 2. 物理量計算・ラベル関数
# ==========================================

def get_grid_indices(rx, ry, gx, gy):
    """指定されたグリッドサイズ(gx, gy)でインデックスを計算"""
    if rx is None or ry is None:
        return None, None
    i = floor(rx / gx)
    k = floor(ry / gy)
    return i, k

def get_grid_label_str(i, k):
    if i is None or k is None: return "NoData"
    return f"i{i}_k{k}"

def get_abstract_acc_label(acc):
    """加速度の抽象化"""
    if acc is None: return "-"
    if acc < -ACC_THRESHOLD: return "NEG"
    elif acc > ACC_THRESHOLD: return "POS"
    else: return "ZERO"

def get_rel_acc_label(rel_acc_val):
    """相対加速度の抽象化"""
    if rel_acc_val is None: return "-"
    if rel_acc_val < -ACC_THRESHOLD: return "NEG"
    elif rel_acc_val > ACC_THRESHOLD: return "POS"
    else: return "ZERO"

def get_rel_vel_label(rel_v_val, rx):
    """相対速度の抽象化 (接近判定)"""
    if rel_v_val is None or rx is None: return "-"
    # rel_v_val = v_npc - v_ego
    if rel_v_val > VEL_THRESHOLD: return "POS"    # NPCの方が速い (離れる or 追いつく)
    elif rel_v_val < -VEL_THRESHOLD: return "NEG" # NPCの方が遅い (詰まる or 離れる)
    else: return "ZERO"

def get_concrete_space_labels(rx, ry, rel_v):
    # (表示用) 具体空間の簡易ラベル
    if rx is None: return None, None, None
    lane = "Same" if abs(ry) <= EGO_HALF_WIDTH else "Other"
    lead = "Front" if rx >= 0 else "Back"
    rv = "Faster" if rel_v > 0 else "Slower"
    return lane, lead, rv

# ==========================================
# 3. 安全性判定ロジック
# ==========================================

def is_abstract_unsafe_rule(i, k, npc_acc_label, rel_label, mode="ACC"):
    """
    抽象安全性ルール
    mode="ACC": 相対加速度を使用 / mode="VEL": 相対速度を使用
    """
    if i is None or k is None: return False
    danger = False
    
    # 車線内または隣接車線 (-3 <= k <= 2) を監視範囲とする
    if -3 <= k <= 2:
        if i >= 0: # 前方
            # 前方にいて、相手が減速(NEG)または速度差がマイナス(接近)
            if mode == "ACC" and npc_acc_label in ["NEG", "ZERO"]: danger = True
            if mode == "VEL" and rel_label in ["NEG", "ZERO"]: danger = True
        else: # 後方
            # 後方にいて、相手が加速(POS)または速度差がプラス(接近)
            if mode == "ACC" and npc_acc_label in ["POS", "ZERO"]: danger = True
            if mode == "VEL" and rel_label in ["POS", "ZERO"]: danger = True
        
    return danger

def calculate_acceleration(velocities, timestamps):
    accels = []
    for i in range(len(velocities)):
        if i == 0:
            accels.append(0.0)
            continue
        try:
            dt = timestamps[i] - timestamps[i-1]
            dv = velocities[i][0] - velocities[i-1][0]
            if dt > 0 and velocities[i][0] is not None and velocities[i-1][0] is not None:
                accels.append(dv / dt)
            else:
                accels.append(0.0)
        except:
            accels.append(None)
    return np.array(accels)

# ==========================================
# 4. データ抽出・正規化
# ==========================================

def load_json_data(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        return json.load(file)

def extract_coordinates_from_json(data):
    ego_coords, npc_coords = [], []
    ego_velocities, npc_velocities = [], []
    timestamps, npc_ids = [], []
    
    states = data.get('groundtruth_kinematic') or data.get('states') or (data if isinstance(data, list) else [])
    
    if not states:
        return np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([])

    for state in states:
        t = state.get('timestamp') if state.get('timestamp') is not None else state.get('timeStamp')
        timestamps.append(t)
        
        e = state.get('groundtruth_ego') or state.get('ego')
        if e:
            p, v = e['pose']['position'], e['twist']['linear']
            ego_coords.append([p['x'], p['y']])
            ego_velocities.append([v['x'], v['y']])
        else:
            ego_coords.append([None, None]); ego_velocities.append([None, None])
            
        ns = state.get('groundtruth_vehicles') or state.get('groundtruth_NPCs')
        if ns and len(ns) > 0:
            n = ns[0]
            p, v = n['pose']['position'], n['twist']['linear']
            npc_coords.append([p['x'], p['y']])
            npc_velocities.append([v['x'], v['y']])
            npc_ids.append(n.get('id') or n.get('name'))
        else:
            npc_coords.append([None, None]); npc_velocities.append([None, None]); npc_ids.append(None)
            
    return np.array(ego_coords), np.array(npc_coords), np.array(ego_velocities), np.array(npc_velocities), np.array(timestamps), np.array(npc_ids)

def normalize_coordinates(ego_coords, npc_coords):
    valid_idx = next((i for i, c in enumerate(ego_coords) if c[0] is not None), 0)
    if len(ego_coords) == 0: return ego_coords, npc_coords, np.eye(2)
    
    origin = ego_coords[valid_idx].astype(float)
    ego_n = []
    npc_n = []
    for c in ego_coords:
        ego_n.append(c - origin if c[0] is not None else [None, None])
    for c in npc_coords:
        npc_n.append(c - origin if c[0] is not None else [None, None])
    
    ego_n = np.array(ego_n)
    npc_n = np.array(npc_n)
    
    rot_mat = np.eye(2)
    direction_found = False
    for i in range(valid_idx + 1, min(valid_idx + 20, len(ego_n))):
        if ego_n[i][0] is not None and ego_n[valid_idx][0] is not None:
            move = ego_n[i].astype(float) - ego_n[valid_idx].astype(float)
            if np.linalg.norm(move) > 0.1:
                angle = -atan2(move[1], move[0])
                rot_mat = np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
                direction_found = True
                break
    
    if not direction_found:
        rot_mat = np.eye(2)

    def rotate(coords):
        rotated = []
        for c in coords:
            if c[0] is not None:
                rotated.append(np.dot(rot_mat, c.astype(float)))
            else:
                rotated.append([None, None])
        return np.array(rotated)

    return rotate(ego_n), rotate(npc_n), rot_mat

def normalize_velocities(ego_vel, npc_vel, rot_mat):
    def rotate(vels):
        rotated = []
        for v in vels:
            if v[0] is not None:
                rotated.append(np.dot(rot_mat, v))
            else:
                rotated.append([None, None])
        return np.array(rotated)
    return rotate(ego_vel), rotate(npc_vel)

# ==========================================
# 5. メイン処理 (データ生成・判定)
# ==========================================

def generate_output_text(ego_coords, npc_coords, ego_vel, npc_vel, timestamps, npc_ids, gx, gy):
    ego_accels = calculate_acceleration(ego_vel, timestamps)
    npc_accels = calculate_acceleration(npc_vel, timestamps)
    
    rel_vs = []
    for ev, nv in zip(ego_vel, npc_vel):
        if ev[0] is not None and nv[0] is not None:
            rel_vs.append(nv[0] - ev[0])
        else:
            rel_vs.append(None)
            
    rel_accels = []
    for ea, na in zip(ego_accels, npc_accels):
        if ea is not None and na is not None:
            rel_accels.append(na - ea)
        else:
            rel_accels.append(None)

    processed_segments = []
    prev_state_key = None
    start_idx = 1
    
    conc_unsafe_any = False
    abs_acc_unsafe_any = False
    abs_vel_unsafe_any = False
    
    current_segment_data = None

    for i in range(len(timestamps)):
        gi, gk = None, None
        n_acc_label = "-"
        r_acc_label = "-"
        r_vel_label = "-"
        safe_c = True
        unsafe_acc = False
        unsafe_vel = False
        
        if (i < len(npc_coords) and npc_coords[i][0] is not None and ego_coords[i][0] is not None):
            rx = npc_coords[i][0] - ego_coords[i][0]
            ry = npc_coords[i][1] - ego_coords[i][1]
            gi, gk = get_grid_indices(rx, ry, gx, gy)
            grid_label = get_grid_label_str(gi, gk)
            n_acc_label = get_abstract_acc_label(npc_accels[i])
            r_acc_label = get_rel_acc_label(rel_accels[i])
            r_vel_label = get_rel_vel_label(rel_vs[i], rx)
            
            ttc = float('inf')
            if rel_vs[i] is not None and rel_vs[i] != 0:
                if (rx > 0 and rel_vs[i] < 0) or (rx < 0 and rel_vs[i] > 0):
                    ttc = abs(rx / rel_vs[i])
            
            if abs(ry) < 2.78 and ttc < TTC_THRESHOLD:
                safe_c = False
                conc_unsafe_any = True
                
            unsafe_acc = is_abstract_unsafe_rule(gi, gk, n_acc_label, r_acc_label, mode="ACC")
            unsafe_vel = is_abstract_unsafe_rule(gi, gk, n_acc_label, r_vel_label, mode="VEL")
            
            if unsafe_acc: abs_acc_unsafe_any = True
            if unsafe_vel: abs_vel_unsafe_any = True
            
        else:
            grid_label = "NoData"

        current_key = (grid_label, n_acc_label, r_acc_label, r_vel_label)
        
        if i == 0:
            prev_state_key = current_key
            current_segment_data = {
                'safe_c': safe_c, 'unsafe_acc': unsafe_acc, 'unsafe_vel': unsafe_vel
            }
            continue

        npc_switched = False 
        if i < len(npc_ids) and npc_ids[i] != npc_ids[i-1]: npc_switched = True

        if current_key != prev_state_key or npc_switched:
            processed_segments.append({
                'range': (start_idx, i),
                'grid': prev_state_key[0],
                'n_acc': prev_state_key[1],
                'r_acc': prev_state_key[2],
                'r_vel': prev_state_key[3],
                'safe_c': current_segment_data['safe_c'],
                'unsafe_acc': current_segment_data['unsafe_acc'],
                'unsafe_vel': current_segment_data['unsafe_vel']
            })
            start_idx = i + 1
            prev_state_key = current_key
            current_segment_data = {
                'safe_c': safe_c, 'unsafe_acc': unsafe_acc, 'unsafe_vel': unsafe_vel
            }
        else:
            if not safe_c: current_segment_data['safe_c'] = False
            if unsafe_acc: current_segment_data['unsafe_acc'] = True
            if unsafe_vel: current_segment_data['unsafe_vel'] = True

    if current_segment_data is not None:
        processed_segments.append({
            'range': (start_idx, len(timestamps)),
            'grid': prev_state_key[0],
            'n_acc': prev_state_key[1],
            'r_acc': prev_state_key[2],
            'r_vel': prev_state_key[3],
            'safe_c': current_segment_data['safe_c'],
            'unsafe_acc': current_segment_data['unsafe_acc'],
            'unsafe_vel': current_segment_data['unsafe_vel']
        })

    def get_res_type(is_conc_unsafe, is_abs_unsafe):
        if is_conc_unsafe and is_abs_unsafe: return "TP"
        if not is_conc_unsafe and not is_abs_unsafe: return "TN"
        if not is_conc_unsafe and is_abs_unsafe: return "FP"
        if is_conc_unsafe and not is_abs_unsafe: return "FN"
        return "Error"

    res_acc = get_res_type(conc_unsafe_any, abs_acc_unsafe_any)
    res_vel = get_res_type(conc_unsafe_any, abs_vel_unsafe_any)
    
    detection_results = {"acc": res_acc, "vel": res_vel}

    header = f"{'Time':<12} {'Grid':<10} {'N_Acc':<6} {'R_Acc':<6} {'R_Vel':<6} {'Conc':<6} {'Abs(A)':<8} {'Abs(V)':<8}"
    lines = []
    lines.append(f"Scenario Result -> ACC-Rule: {res_acc}, VEL-Rule: {res_vel}")
    lines.append(f"(Concrete Unsafe: {conc_unsafe_any})")
    lines.append(f"(Abs Unsafe [ACC]: {abs_acc_unsafe_any}, [VEL]: {abs_vel_unsafe_any})")
    lines.append("")
    lines.append(header)
    lines.append("-" * len(header))
    
    for s in processed_segments:
        r_str = f"{s['range'][0]}-{s['range'][1]}"
        conc_str = "OK" if s['safe_c'] else "NG"
        abs_a_str = "NG" if s['unsafe_acc'] else "OK"
        abs_v_str = "NG" if s['unsafe_vel'] else "OK"
        lines.append(f"{r_str:<12} {s['grid']:<10} {s['n_acc']:<6} {s['r_acc']:<6} {s['r_vel']:<6} {conc_str:<6} {abs_a_str:<8} {abs_v_str:<8}")

    return "\n".join(lines), detection_results

# ==========================================
# 6. ファイル・実験管理
# ==========================================

def process_file(input_path, output_path, gx, gy):
    try:
        data = load_json_data(input_path)
        ego_coords, npc_coords, ego_vel, npc_vel, timestamps, npc_ids = extract_coordinates_from_json(data)
        if len(timestamps) == 0: return False, "No Data", None
        
        ego_n, npc_n, rot_mat = normalize_coordinates(ego_coords, npc_coords)
        ego_vn, npc_vn = normalize_velocities(ego_vel, npc_vel, rot_mat)
        
        output_text, res = generate_output_text(ego_n, npc_n, ego_vn, npc_vn, timestamps, npc_ids, gx, gy)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(f"=== {input_path.name} (Grid: {gx:.2f}x{gy:.2f}) Result ===\n\n{output_text}")
        return True, None, res
    except Exception as e:
        import traceback
        traceback.print_exc()
        return False, str(e), None

def run_experiment(input_folder, base_output_folder, scenario_name):
    """
    指定フォルダ内の実験を実行し、統計結果のリストを返すように変更
    """
    input_path = Path(input_folder)
    if not input_path.exists():
        print(f"Error: Input folder {input_folder} does not exist.")
        return []

    # CSV集計用リスト (1行分のデータを格納した辞書またはリスト)
    all_grid_results = []

    # グリッドサイズごとのループ
    for gx, gy in GRID_SIZE_PAIRS:
        grid_folder_name = f"grid_{gx:.2f}_{gy:.2f}"
        current_output_dir = Path(base_output_folder) / grid_folder_name
        current_output_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"\n>>> Running Experiment: Grid Size X={gx:.2f}, Y={gy:.2f}")
        
        json_files = list(input_path.glob("*.json"))
        
        # 統計用辞書 (ACCベースとVELベースで分別)
        stats_acc = {'FN': [], 'FP': [], 'TP': [], 'TN': [], 'Error': []}
        stats_vel = {'FN': [], 'FP': [], 'TP': [], 'TN': [], 'Error': []}
        
        for json_file in tqdm(json_files):
            out_file = current_output_dir / f"{json_file.stem}_result.txt"
            success, err, res = process_file(json_file, out_file, gx, gy)
            if success and res:
                stats_acc[res['acc']].append(json_file.name)
                stats_vel[res['vel']].append(json_file.name)
        
        # 個別テキストサマリー保存 (既存機能維持)
        summary_path = current_output_dir / "summary_diff.txt"
        with open(summary_path, "w", encoding='utf-8') as f:
            f.write(f"Grid Size: X={gx:.2f}, Y={gy:.2f}\n")
            f.write(f"Total Scenarios: {len(json_files)}\n\n")
            f.write("=== Result [ACC]: Relative Acceleration Based ===\n")
            f.write(f"TP: {len(stats_acc['TP'])}, TN: {len(stats_acc['TN'])}, FP: {len(stats_acc['FP'])}, FN: {len(stats_acc['FN'])}\n")
            f.write("\n=== Result [VEL]: Relative Velocity Based ===\n")
            f.write(f"TP: {len(stats_vel['TP'])}, TN: {len(stats_vel['TN'])}, FP: {len(stats_vel['FP'])}, FN: {len(stats_vel['FN'])}\n")
        
        # --- CSV用データ収集 ---
        # ACCルールの行
        all_grid_results.append({
            "Scenario": scenario_name,
            "Grid_X": gx,
            "Grid_Y": gy,
            "Rule_Type": "ACC",
            "Total": len(json_files),
            "TP": len(stats_acc['TP']),
            "TN": len(stats_acc['TN']),
            "FP": len(stats_acc['FP']),
            "FN": len(stats_acc['FN'])
        })
        # VELルールの行
        all_grid_results.append({
            "Scenario": scenario_name,
            "Grid_X": gx,
            "Grid_Y": gy,
            "Rule_Type": "VEL",
            "Total": len(json_files),
            "TP": len(stats_vel['TP']),
            "TN": len(stats_vel['TN']),
            "FP": len(stats_vel['FP']),
            "FN": len(stats_vel['FN'])
        })

    return all_grid_results

if __name__ == "__main__":
    # 基本となるパス設定
    BASE_INPUT_DIR = Path("D:/generated_trajectories")
    BASE_OUTPUT_DIR = Path("D:/datamapping")

    # 処理したいシナリオフォルダ名のリスト
    # ユーザー指定のフォルダ名に変更（必要に応じて修正してください）
    TARGET_SCENARIOS = [
        "cutout_scenario_ver2",
    ]

    # 全結果を格納するリスト
    total_csv_data = []

    for scenario_name in TARGET_SCENARIOS:
        # 入力パス: D:/generated_trajectories/{scenario_name}
        input_folder = BASE_INPUT_DIR / scenario_name
        
        # 出力パス: D:/datamapping/{scenario_name}/property
        output_folder = BASE_OUTPUT_DIR / scenario_name / "property"

        print(f"\n{'='*50}")
        print(f"Processing Target: {scenario_name}")
        print(f"Input : {input_folder}")
        print(f"Output: {output_folder}")
        print(f"{'='*50}\n")

        # 実行結果を受け取り、全体リストに追加
        scenario_results = run_experiment(str(input_folder), str(output_folder), scenario_name)
        total_csv_data.extend(scenario_results)

    # --- 全ての処理完了後にまとめてCSV出力 ---
    if total_csv_data:
        csv_file_path = BASE_OUTPUT_DIR / "all_scenarios_summary.csv"
        print(f"\nWriting total summary to: {csv_file_path}")
        
        fieldnames = ["Scenario", "Grid_X", "Grid_Y", "Rule_Type", "Total", "TP", "TN", "FP", "FN"]
        
        try:
            with open(csv_file_path, mode='w', newline='', encoding='utf-8') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(total_csv_data)
            print("CSV export completed successfully.")
        except Exception as e:
            print(f"Failed to write CSV: {e}")