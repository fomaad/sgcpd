import json
import math
import numpy as np
import glob
import os
import csv

# ==========================================
# 1. 簡易Lanelet2マップハンドラ
# ==========================================
class SimpleLaneletMap:
    def __init__(self, lane_width=3.5):
        self.lane_width = lane_width
        # 3車線道路の定義 (y座標で判定)
        self.lanes = {
            "left_lane": (1.75, 5.25),
            "center_lane": (-1.75, 1.75),
            "right_lane": (-5.25, -1.75)
        }
        
        # 隣接関係の定義 (Graph)
        self.adjacency = {
            "left_lane": ["center_lane"],
            "center_lane": ["left_lane", "right_lane"],
            "right_lane": ["center_lane"]
        }

    def get_lanelet_id(self, x, y):
        """座標(x, y)からLanelet IDを返す"""
        for lane_id, (min_y, max_y) in self.lanes.items():
            if min_y <= y <= max_y:
                return lane_id
        return None # レーン外

    def is_adjacent(self, lane_a, lane_b):
        """lane_a と lane_b が隣接しているか判定"""
        if lane_a is None or lane_b is None:
            return False
        return lane_b in self.adjacency.get(lane_a, [])

# ==========================================
# 2. STL評価ロジック
# ==========================================
def evaluate_cutin_scenario(trajectory_data, map_handler, time_window=5.0, dt=0.1):
    timestamps = []
    predicates = {
        "is_adj": [],       # 隣接車線にいるか
        "completed": []     # カットイン完了条件 (自車線 && 前方)
    }

    last_ego_lane = None
    last_npc_lane = None

    for frame in trajectory_data:
        t = frame["timestamp"]
        timestamps.append(t)
        
        # 1. 座標取得
        # データ構造のエラーハンドリング
        try:
            ego_pos = frame["groundtruth_ego"]["pose"]["position"]
            # NPCは配列の最初の1台を対象とする
            if len(frame["groundtruth_vehicles"]) > 0:
                npc_pos = frame["groundtruth_vehicles"][0]["pose"]["position"]
            else:
                # NPCがいないフレームは判定不可としてFalse扱い
                predicates["is_adj"].append(False)
                predicates["completed"].append(False)
                continue
        except KeyError:
            # 必要なキーがない場合
            continue

        # 2. マップマッチング
        ego_lane_id = map_handler.get_lanelet_id(ego_pos["x"], ego_pos["y"])
        npc_lane_id = map_handler.get_lanelet_id(npc_pos["x"], npc_pos["y"])
        
        last_ego_lane = ego_lane_id
        last_npc_lane = npc_lane_id

        # 3. 述語の計算
        is_adjacent_lane = map_handler.is_adjacent(ego_lane_id, npc_lane_id)
        is_same_lane = (ego_lane_id == npc_lane_id) and (ego_lane_id is not None)
        is_ahead = npc_pos["x"] > ego_pos["x"]

        predicates["is_adj"].append(is_adjacent_lane)
        predicates["completed"].append(is_same_lane and is_ahead)

    # STL評価
    detected = False
    detection_time = None
    window_frames = int(time_window / dt)

    for i in range(len(timestamps)):
        if predicates["is_adj"][i]:
            start_idx = i
            end_idx = min(i + window_frames, len(timestamps))
            
            if any(predicates["completed"][start_idx:end_idx]):
                detected = True
                detection_time = timestamps[i]
                break 

    return {
        "detected": detected,
        "time": detection_time,
        "ego_lane": last_ego_lane,
        "npc_lane": last_npc_lane
    }

# ==========================================
# 3. バッチ処理メイン部
# ==========================================
def main():
    # --- 設定 ---
    TARGET_DIR = "."  # JSONファイルがあるフォルダ (現在のフォルダなら ".")
    OUTPUT_CSV = "stl_analysis_result.csv"
    
    # ----------------
    
    lanelet_map = SimpleLaneletMap(lane_width=3.5)
    
    # JSONファイル一覧を取得
    json_files = glob.glob(os.path.join(TARGET_DIR, "*.json"))
    total_files = len(json_files)
    
    print(f"フォルダ '{TARGET_DIR}' 内の {total_files} 個のJSONファイルを処理します...")

    results = []

    for i, file_path in enumerate(json_files):
        file_name = os.path.basename(file_path)
        print(f"[{i+1}/{total_files}] Processing: {file_name} ...", end="\r")

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # --- データ構造の正規化 (辞書 -> リスト) ---
            target_data = None
            if isinstance(data, list):
                target_data = data
            elif isinstance(data, dict):
                # リストが入っているキーを探す
                for k in data.keys():
                    if isinstance(data[k], list):
                        target_data = data[k]
                        break
            
            if target_data is None:
                # リストが見つからない場合
                results.append({
                    "Filename": file_name,
                    "Detected": "Error",
                    "Time": "",
                    "Note": "Valid list data not found in JSON"
                })
                continue

            # --- 判定実行 ---
            res = evaluate_cutin_scenario(target_data, lanelet_map)
            
            results.append({
                "Filename": file_name,
                "Detected": "TRUE" if res["detected"] else "FALSE",
                "Time": f"{res['time']:.2f}" if res["time"] is not None else "",
                "LastEgoLane": res["ego_lane"],
                "LastNpcLane": res["npc_lane"],
                "Note": ""
            })

        except Exception as e:
            # 読み込みエラーや予期せぬエラー
            results.append({
                "Filename": file_name,
                "Detected": "Error",
                "Time": "",
                "Note": str(e)
            })

    print(f"\n処理完了。集計結果を書き込み中: {OUTPUT_CSV}")

    # CSV書き込み
    csv_header = ["Filename", "Detected", "Time", "LastEgoLane", "LastNpcLane", "Note"]
    
    try:
        with open(OUTPUT_CSV, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=csv_header)
            writer.writeheader()
            writer.writerows(results)
        print("完了しました。")
        
        # 簡易集計表示
        true_count = sum(1 for r in results if r["Detected"] == "TRUE")
        print(f"--- Summary ---\nTotal: {total_files}\nDetected: {true_count}\nNot Detected: {total_files - true_count}")
        
    except IOError as e:
        print(f"CSV書き込みエラー: {e}")

if __name__ == "__main__":
    main()