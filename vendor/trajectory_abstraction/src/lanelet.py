import math

# XMLテンプレートのヘッダーとフッター
OSM_HEADER = """<?xml version='1.0' encoding='UTF-8'?>
<osm version='0.6' generator='Lanelet2Generator'>"""
OSM_FOOTER = """</osm>"""

# 設定
ORIGIN_LAT = 36.0  # 基準緯度
ORIGIN_LON = 136.0 # 基準経度
LENGTH = 300.0     # 長さ(m)
LANE_WIDTH = 3.5   # 車線幅(m)
NUM_LANES = 3      # 車線数

# 定数 (簡易変換用)
METERS_PER_DEG_LAT = 111319.9
METERS_PER_DEG_LON = 111319.9 * math.cos(math.radians(ORIGIN_LAT))

def get_geo(x, y):
    """ローカル座標(m)を緯度経度に変換"""
    lat = ORIGIN_LAT + (y / METERS_PER_DEG_LAT)
    lon = ORIGIN_LON + (x / METERS_PER_DEG_LON)
    return lat, lon

def create_osm():
    nodes = []
    ways = []
    relations = []
    
    node_id = 1
    way_id = 1
    rel_id = 1

    # 1. ノードとWay（境界線）の作成
    # ラインは左から右へ順に作成 (Line 0 = 左端, Line 3 = 右端)
    line_ids = []
    
    for i in range(NUM_LANES + 1):
        y_pos = 0  # 直線なのでyは固定（あるいはx固定でyを伸ばす）
        # ここでは道路が「東（X軸プラス方向）」に向かって伸びると仮定
        # 車線は「北（Y軸プラス方向）」に並べる（左側通行の場合の並び順に注意が必要だが、まずは座標配置）
        
        # 左端をY=0とし、右へ向かってYが増える配置（車線1, 2, 3）
        # ※Lanelet2のright/left定義に合わせるため
        offset_y = i * LANE_WIDTH * -1 # 左側通行を想定し、Y軸マイナス方向へ並べる（上がCenter）
        
        # 始点ノード
        lat_start, lon_start = get_geo(-100, offset_y)
        nodes.append(f"  <node id='{node_id}' lat='{lat_start}' lon='{lon_start}' version='1' visible='true' />")
        start_node = node_id
        node_id += 1
        
        # 終点ノード
        lat_end, lon_end = get_geo(LENGTH, offset_y)
        nodes.append(f"  <node id='{node_id}' lat='{lat_end}' lon='{lon_end}' version='1' visible='true' />")
        end_node = node_id
        node_id += 1
        
        # Way作成 (線の種類：実線か破線か)
        subtype = "solid" if i == 0 or i == NUM_LANES else "dashed"
        ways.append(f"""  <way id='{way_id}' version='1' visible='true'>
    <nd ref='{start_node}' />
    <nd ref='{end_node}' />
    <tag k='type' v='line_thin' />
    <tag k='subtype' v='{subtype}' />
  </way>""")
        line_ids.append(way_id)
        way_id += 1

    # 2. Relation（Lanelet）の作成
    # 日本の左側通行を前提：
    # レーン1（一番左）: Left=Line0, Right=Line1
    for i in range(NUM_LANES):
        left_bound = line_ids[i]
        right_bound = line_ids[i+1]
        
        relations.append(f"""  <relation id='{rel_id}' version='1' visible='true'>
    <member type='way' ref='{left_bound}' role='left' />
    <member type='way' ref='{right_bound}' role='right' />
    <tag k='type' v='lanelet' />
    <tag k='subtype' v='road' />
    <tag k='speed_limit' v='60' />
    <tag k='location' v='urban' />
    <tag k='one_way' v='yes' />
  </relation>""")
        rel_id += 1

    # ファイル書き込み
    with open("3lane_straight_300m.osm", "w", encoding="utf-8") as f:
        f.write(OSM_HEADER + "\n")
        f.write("\n".join(nodes) + "\n")
        f.write("\n".join(ways) + "\n")
        f.write("\n".join(relations) + "\n")
        f.write(OSM_FOOTER)
        
    print("生成完了: 3lane_straight_300m.osm")

if __name__ == "__main__":
    create_osm()