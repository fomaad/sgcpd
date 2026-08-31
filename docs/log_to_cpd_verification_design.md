# ログ→CPD対応 検証設計ドキュメント（Draft v0.3）

> **v0.2 更新について**: 咲川麻尋氏の修士論文「自動運転における車両軌道の抽象化および分析手法の提案」（2026年3月, 主指導教員: 青木利晃）を踏まえ、v0.1で「未決事項」としていたログの離散化（4節・7節）に対する具体的な解を第9節にまとめた。以下の第1〜8節はv0.1の内容をほぼそのまま残し、第9節でそれを咲川論文の成果と接続する形にしている。
>
> **v0.3 更新について**: 第9節までの方式は「1本のログから、そのログだけを表すインスタンスCPDを機械的に生成する」ものであり、「cut-inというシナリオ集合そのもの」を表す参照CPDとの突き合わせ（5.1節・9.3節で述べた membership check 本来の目的）はまだ実装していなかった。v0.3では、(a) cut-inを直接表す参照CPDモデルを咲川氏の名前付き領域ではなく格子(grid)ベースの抽象化に基づいて書き下し、(b) 任意のログをその格子と同じ粒度で離散化して参照モデルに対するSAT/UNSATを判定する、という当初の目的を第10節で実装した（`fomaad/sgcpd` の `logverify/` パッケージ）。

対象リポジトリ: `fomaad/sgcpd`（`toshiaki-jaist/rprd` のフォーク、拡張プロジェクト）
関連論文: *Scenario Modeling Language*（CPD / GCPD の提案論文）
対象データセット: JAMA-Traceable ADS Runtime Log Dataset "AJISAI"
（Box: https://jstorage.app.box.com/s/1q19y57rztfpvh1t3u8fzschvxcxu1nu ）

## 1. 目的

Autoware を実走（シミュレーション）させて得られたランタイムログを、既存の **CPD (Car Position Diagram) / GCPD** モデルと突き合わせ、次の2点を検証できるようにする。

1. **適合性検証（membership check）**：観測されたログの挙動が、CPDモデルが表現するシナリオ集合（`gcpd.py` の SAT/SMT 制約が許す `(car, box, lane, position, step)` の割当て空間）に含まれるか。
2. **性質の突合（property cross-check）**：CPDモデル上で成立する性質（衝突可能性など）と、ログから実際に観測される事象（衝突の有無、目標速度への到達など）が一致するか。

これは論文中で今後の課題として挙げられている「CPDモデルを具体的な地図・交通データと統合する」方向性の第一歩であり、CPDを**シナリオ生成**だけでなく**シナリオベース検証**（実行ログの適合性チェック）に使う拡張にあたる。

## 2. 入力データの整理

### 2.1 CPD/GCPDモデル（`gcpd.py`）

`Model` クラスが持つ要素（`sample1.py` より）:

| 要素 | 型 | 意味 |
|---|---|---|
| `cars` | `[str]` | 車両名のリスト（例: `"LCar"`, `"RCar"`） |
| `boxes` | `[(car, box_id)]` | 車両ごとの「箱」＝シナリオの離散状態 |
| `position` | `[(car, box_id, pos:int)]` | 箱の縦方向位置（整数の順序尺度） |
| `lane` | `[(car, box_id, lane:int)]` | 箱が属するレーン番号 |
| `inits` | `[(car, box_id)]` | 各車両の初期箱 |
| `ntrans` / `ctrans` / `netrans` / `cstrans` / `strans` | 遷移 | 通常・条件付き・非存在条件付き・カスタム条件・同期遷移 |
| `max_step` | `int` | シナリオの最大ステップ数 |

`Box(car_index, box_id, step): Bool` が「時刻stepにその箱がアクティブか」を表す一階述語で、`Pos`/`Lane` はその箱の位置・レーンを表す関数。**position と lane は連続座標ではなく、モデル設計者が与える離散的な順序値**である点が、ログとの対応付けで最も注意すべき点になる。

### 2.2 AJISAI データセット（Autowareランタイムログ）

Box 上で確認した実際の構成（README との差分は「flat layout」「metadata_fields.txt が今回の公開版には未同梱」の2点）。

```
AJISAI/
├── README.md
├── jama_index.json         # 生成物: 全432インスタンスのカタログ
├── jama_summary.csv        # 生成物: 上記の表形式版
├── parameter_ranges.txt    # 生成物: 行動ごとの値レンジ
├── schema/jama_sidecar.schema.json
├── scripts/{build_index.py, calc_ranges.py}
└── data/{cutin, cutout, deceleration, uturn, swerve}/
     ├── <scenario_id>.json        … 本体（実行ログ本体、数MB〜数十MB）
     └── <scenario_id>.jama.json   … サイドカー（JAMA 4軸ラベル＋導出根拠、~2KB）
```

**JAMA 4軸ラベル**（サイドカーの必須5キー）:

| 軸 | 値 |
|---|---|
| `jama_road_geometry` | `non_intersection`（本データセットは直線道路のみ） |
| `jama_npc_position` | `ahead` / `ahead_left` / `ahead_right` |
| `jama_npc_direction` | `same_direction` / `oncoming` |
| `jama_npc_behavior` | `cut_in` / `cut_out` / `deceleration` / `u_turn` / `swerve` |

内訳（432件）: cut_in 94, cut_out 72, deceleration 77, u_turn 59, swerve 130。

**サイドカーの `derivation` ブロック**（実測値、実例 `TD-NI-AR-SD-N04-CI-0010` より抜粋）:

```jsonc
{
  "reference_frame_timestamp": 479.905,   // ラベル判定を行った基準時刻
  "npc_lateral_m": -2.65,                 // egoレーン中心からのNPC横方向オフセット [m]
  "npc_longitudinal_m": 23.16,            // ego基準のNPC縦方向距離 [m]
  "delta_lateral_m": 3.9,                 // 横移動量（cut_in/out系）
  "ego_target_speed_kmh": 30.0, "npc_target_speed_kmh": 15.0,
  "measured_ego_speed_kmh": 30.0, "measured_npc_speed_kmh": 15.0,
  "derived_behavior": "cut_in", "consistency_ok": true
}
```

**本体ログ（`<id>.json`）のトップレベルキー**（README記載、`ego_estimated_kinematic` は実データで構造確認済み）:

- `ego_estimated_kinematic` … 実測: `[{timestamp, pose:{position:{x,y,z}, rotation:{x,y,z}}, twist:{linear,angular}, acceleration:{linear,angular}}, ...]`（約0.1秒刻みの時系列。x/yは大きな値＝ローカル絶対座標系〈UTM風〉で、原点はワールド固定）
- `groundtruth_kinematic`, `groundtruth_size` … NPC含む全物体の真値
- `perception_objects`, `boundingbox_perception_objects` … Autowareの認識結果
- `planning_trajectory`, `control_cmds` … Autowareの計画・制御出力
- `metadata` … シミュレーション設定

## 3. ギャップ分析

| | CPDモデル | Autowareログ |
|---|---|---|
| 空間 | `lane`（離散整数）・`position`（離散整数、順序尺度） | 連続座標 `x, y, z`（メートル） |
| 時間 | `step`（離散整数、0..max_step） | 連続時刻 `timestamp`（秒、~0.1s刻み） |
| 状態の存在 | 箱＝「その状態が成立している区間」を表す真偽値 `Box(c,n,s)` | 車両は常に存在し続ける連続軌跡 |
| 対象 | 抽象化されたシナリオの等価類 | 1回のシミュレーション実行の具体的トレース |

→ **ログを CPD のボキャブラリ（car, box, lane, position, step）に離散化する変換**が必要。この変換の設計が本ドキュメントの中心課題。

## 4. 提案するマッピング

### 4.1 車両の対応

- Autoware側 `ego` → CPD側 `"Ego"`（既存モデルでの `EgoCar` 系命名に合わせる）
- Autoware側 NPC（本データセットは基本 ego + NPC1台の想定）→ CPD側 `"RCar"` 等、検証対象のCPDモデルで使われている命名に合わせる

### 4.2 空間の離散化

- **lane**: サイドカーの `npc_lateral_m` を用い、egoレーンを `lane=0`、隣接レーンを符号に応じて `lane=1`（右）/`lane=-1`（左）のように量子化する。レーン境界（車線幅）は `jama_road_geometry=non_intersection` 前提のもと、AJISAI側のシミュレーション設定（`metadata`）から車線幅を取得して閾値化する。
- **position**: CPDのpositionは「箱の登場順」を表す順序尺度であり、メートル値そのものではない。したがって、`npc_longitudinal_m` や `delta_lateral_m` の**変化点**（イベント発生前後）を箱の切り替わり点として扱い、position はモデル定義側（検証対象のCPDモデル）にあらかじめ与えられている整数系列にそのまま対応させる。つまり「ログのどの時刻区間がモデルのどの箱に相当するか」を決める問題に帰着させる。

### 4.3 時間（step）の離散化

2方式を候補として提示する。

1. **イベント駆動（第一選択）**：サイドカーの `derivation.reference_frame_timestamp`（ラベル判定時刻）や、速度・横位置の変化が閾値を超えた時刻を「ステップの境界」とする。例えば cut_in なら `[割込み前（走行中）] → [割込み後]` の2ステップに対応し、`max_step=1` のモデルと対応させやすい。
2. **固定間隔（代替案）**：一定Δt（例: 1秒）ごとにサンプリングしてステップ列を作る。より忠実だが `max_step` が大きくなり、SAT制約も比例して増える。まずは方式1でPoCを行い、必要に応じて方式2に拡張する。

## 5. 検証の2モード

### 5.1 適合性検証（Conformance / Membership Check）

離散化して得た `(car, box, lane, position, step)` の割当てを、検証対象の `gcpd.Model` に対する追加制約として solver に投入し（`Box`/`Pos`/`Lane` の値を固定する等式制約を `add`）、モデル本来の制約（`add_pos`, `add_lane`, `add_init`, `add_trans`）と連立させて `solver.check()` を呼ぶ。

- **sat** → ログの挙動はモデルが表現するシナリオ集合の要素として矛盾なく説明できる（適合）。
- **unsat** → モデルが想定していない挙動（モデル外の振る舞い、またはモデルの不備、あるいは離散化の誤り）としてフラグを立てる。

### 5.2 性質の突合（Property Cross-check）

`gcpd.py` の `ps_col`（衝突述語）等を使い、CPDモデル上で「衝突が起こりうる」とされる箱の組と、ログ側で実際に観測される最小車間距離（`groundtruth_kinematic` から計算）やサイドカーの `consistency_ok` を突き合わせる。既にAJISAI側で行われている「filenameベースのラベル」対「trajectoryから導出したラベル」の整合性チェックと同様の考え方を、CPDモデル対ログの整合性チェックに応用する。

### 5.3 ラベルからモデルを自動選択

サイドカーの4軸ラベル（`jama_npc_position` / `jama_npc_direction` / `jama_npc_behavior`）を使い、どの既存CPDモデル（論文中のJAMA用モデル、または `experiment/` 配下のモデル）を検証対象として選ぶかを決めるルックアップテーブルを用意する。

## 6. 実装アーキテクチャ（提案）

`fomaad/sgcpd` 内に新規モジュール `logverify/` を追加する案。

```
logverify/
├── jama_log.py     # 本体+サイドカーの読み込み → 内部表現 Instance に変換
├── discretize.py   # DiscretizationSpec を受け取り Instance → (car,box,lane,pos,step) 集合
├── verify.py       # check_membership(model, discretized), check_collision_consistency(model, instance)
├── models/         # 検証対象のCPDモデル定義（既存 experiment/ を整理・移設）
└── cli.py          # 例: python -m logverify verify --model models/cutin.py --scenario TD-NI-AR-SD-N04-CI-0010
```

- `jama_log.py`：`<id>.json`（本体、大きいので必要な系列だけストリーム的に読む）と `<id>.jama.json`（サイドカー）をロードし、`Instance`（`scenario_id`, 4軸ラベル, `ego_traj`, `npc_trajs`, `derivation`）にまとめる。
- `discretize.py`：behaviorごとに異なる `DiscretizationSpec`（レーン境界、ステップ境界の決め方）をプラガブルに用意する。
- `verify.py`：上記5.1・5.2のロジックを実装。
- データセット本体（9.7GB）はリポジトリに含めず、外部パス／環境変数でBoxからの取得先ディレクトリを指定する運用とする。

## 7. 未決事項（要相談）

1. **position の解像度**：1箱を「車体1台分」とするか「レーン変更が完了するまで」とするか。
2. **複数NPCの扱い**：AJISAIは基本 ego + NPC1台と推測されるが、要確認（`groundtruth_kinematic` に複数エントリがあるかを次のPoCで確認する）。
3. **box数・max_step の決め方**：behaviorごとに手動で設計するか、ログから自動推定するか。
4. **座標系**：`pose.position` はワールド固定のローカル座標（値が8万台）に見える。ego相対変換が必要だが、サイドカーの `derivation.npc_lateral_m` / `npc_longitudinal_m` が既に相対値として提供されているため、まずはこれを直接利用するのが近道。
5. **検証対象のCPDモデルをどう用意するか**：論文中のJAMA関連モデル（cut-in等）が `experiment/` にある場合はそれを再利用し、なければ本ドキュメントの対応表に基づき新規に定義する。
6. **スコープ**：432件全体をバッチ検証する前に、まず1件（例: `TD-NI-AR-SD-N04-CI-0010`, cut_in）でPoCを行う。

## 8. 次のステップ（提案・v0.1時点）

1. `experiment/` 配下の既存モデルを確認し、cut_in に対応するCPDモデルがあるか調査する。
2. `TD-NI-AR-SD-N04-CI-0010` を対象に、手動で離散化した `(car,box,lane,position,step)` を作り、`check_membership` をスクリプトとして素朴に実装（PoC）。
3. PoCの結果を踏まえて `DiscretizationSpec` を一般化し、cut_in以外の4行動（cut_out, deceleration, u_turn, swerve）に展開する。
4. `jama_index.json` / `jama_summary.csv` を使い、94件（cut_in）程度からバッチ検証パイプラインに拡張する。

→ 上記のうち2〜4（離散化の具体的な方式）は、第9節で述べる咲川論文の手法を使うことで大幅に具体化できる。以降はそちらを参照。

---

## 9. 咲川論文に基づく具体化（v0.2）

### 9.1 咲川論文の要点

咲川氏の研究は、本ドキュメントが「4. ギャップ分析」で述べた「CPDの離散な語彙」と「Autowareログの連続な物理量」の間のギャップと**まったく同じ問題**（論文中では「抽象度のギャップ」）を、CPDとは独立に、抽象解釈（データマッピング）の枠組みで扱ったものである。提案されている2手法のうち、特に**車両周辺の領域分割（15領域モデル）**が本プロジェクトとの親和性が高い。

- **9領域モデル**: egoを中心に縦方向3分割（lead / ego / following）× 横方向3分割（left / ego車線 / right）＝9領域（lead-left, lead, lead-right, left, ego, right, follow-left, following, follow-right）。
- **15領域モデル**: 縦方向をさらに5分割（far-lead / lead / ego / follow / far-follow、閾値距離 `Dth = v_ego × 2.0s`）×横方向3分割＝15領域。9領域モデルより高い距離での接近予兆を捉えられ、実験ではFN=0を保ったままTNが改善（誤検知が減少）した。
- **時間の抽象化**：連続する同一の抽象状態を1つに集約するイベント駆動型の圧縮（本ドキュメントv0.1の「4.3 イベント駆動（第一選択）」で提案していたものと同一の考え方）。実データ（AWSIM）で平均状態数10個前後、圧縮率99%超を達成。
- **座標正規化**：ワールド座標系のego・NPC位置を、egoの進行方向を基準とした相対座標系へ回転変換する実装（`normalize_coordinates`）を持つ。これは本ドキュメントの「7. 未決事項 (4) 座標系」に対する具体的な答えを与える。
- **安全性の定義**：相対速度の符号＋領域（前方でPOS＝遠ざかる、後方でNEG＝遠ざかる、なら安全）による安全性判定を提案し、「抽象空間で安全と判定されたら具体空間でも安全」という健全性（over-approximation）を証明している（定理4.1相当）。実験でFN（危険の見逃し）= 0 を達成。
- **既知の限界**（論文7.4節）：(a) 蛇行（swerve）・Uターンのように「向き」の変化が本質的なシナリオは、位置ベースの領域分割だけでは検出できない。(b) 距離と速度を独立に抽象化しているため、安全性判定のFP（過検知）が多い。改善案として「述語抽象化」（例: `TTC<2.0` に相当する述語 `x - 2.0v < 0` を直接抽象状態に持たせる）を提案している。

### 9.2 CPDとの対応づけ（本ドキュメント4節の具体化）

咲川氏の15領域モデルの各シンボルは、**縦方向インデックス×横方向インデックス**の直積として素直に分解でき、これはCPDの `(lane, position)` とほぼそのまま対応する。

| 領域名 | lane（横方向） | position（縦方向、15領域） |
|---|---|---|
| far-left / lead-left / left / follow-left / far-rear-left | +1（左隣接車線） | far-lead=+2 / lead=+1 / ego=0 / follow=-1 / far-rear=-2 |
| far-front / lead / ego / following / far-rear | 0（自車線） | 同上 |
| far-right / lead-right / right / follow-right / far-rear-right | -1（右隣接車線） | 同上 |

（9領域モデルの場合は縦方向を {lead=+1, ego=0, following=-1} の3値に単純化すればよい。）

これにより、**咲川氏の抽象軌道 `T̂rj = ⟨ŝ'_0, ŝ'_1, …, ŝ'_m⟩` は、そのままCPDの箱列に変換できる**：

1. 各抽象状態 `ŝ'_k = (p̂_k, v̂_k)` の領域 `p̂_k` を上表で `(lane_k, position_k)` に変換する。
2. NPC車両について、CPDの箱を `("npc", k)`（`k = 0, …, m`）として生成し、`append_position([("npc", k, position_k)])`、`append_lane([("npc", k, lane_k)])` を登録する。
3. 連続する箱の間に通常遷移を追加する：`add_ntrans(("npc", k, "npc", k+1))`（すべての `k`）。
4. egoは自車中心の相対座標系の原点にあたるため、別途 `("ego", 0)` の固定箱として置くか、モデルの目的に応じて省略する。
5. `set_init([("npc", 0)])`、`max_step = m` として `Model` インスタンスを構成する。

この手順により、**1件のログから、そのログの挙動だけを表す「インスタンスCPDモデル」が機械的に生成できる**。これは咲川氏の「抽象化ツール」の出力形式（`follow-left → lead-left → lead` のような遷移系列）を、そのままCPDのモデル構築APIに流し込むアダプタに相当する。

### 9.3 適合性検証への接続（本ドキュメント5.1節の具体化）

「5.1 適合性検証」で述べた `check_membership` は、次のように具体化される。

1. **参照モデル**：JAMA行動セル（例: cut_in, ahead_right, same_direction）ごとに、許容される箱の遷移パターンを表す「参照CPDモデル」を用意する（`experiment/` にある論文Table Iのレーン変更モデルを参考に、`ntrans`/`ctrans` として「left→ego」「right→ego」等の遷移のみを許可する形で定義）。
2. **インスタンスモデル**：9.2節の手順で、検証したいログ（例: `TD-NI-AR-SD-N04-CI-0010`）から自動生成する。
3. **検証**：インスタンスモデルの `(box, lane, position, step)` 割当てを、参照モデルの制約（`add_pos`, `add_lane`, `add_init`, `add_trans`）に対する追加の等式制約として solver に投入し `check()` する。
   - **sat** → このログの挙動は参照モデルが定義する cut_in シナリオ集合の要素として矛盾なく説明できる。AJISAI側のサイドカーラベル（`jama_npc_behavior="cut_in"`, `consistency_ok=true`）との突合も行い、両者が一致するかを報告する。
   - **unsat** → 参照モデルが想定していない遷移（例：一度自車線に入ってからまた元の車線へ戻る、といった蛇行的な動き）が発生している。これは咲川論文が指摘した「swerve/u_turnを位置ベースの領域分割だけでは検出できない」という限界と表裏一体であり、**CPD側の視点では「参照モデルにない遷移＝新規シナリオ候補（unsat）として自動的に検出される」**という利点に転化できる可能性がある。

### 9.4 安全性判定への接続（本ドキュメント5.2節の具体化）

咲川氏の安全性定義（相対速度の符号＋領域）は、CPDの衝突述語 `ps_col(c1, c2, bx, t)`（同時刻に同じ `lane` かつ同じ `position` にいたら衝突）と両立する。咲川論文が課題として挙げた「距離と速度を独立に抽象化したことによるFP過多」は、まさに論文が提案する「述語抽象化」に相当するものをCPDが最初から備えている、と捉えることができる。すなわち、`Pos` と `Lane` を同時に扱う `ps_col` の条件式自体が、咲川論文で言う「距離と速度（正確には位置と位置）の結合条件」であるため、**CPDに衝突判定を委ねることで、咲川氏の手法単体よりもFPを抑制できる可能性がある**。この仮説はPoCで検証する価値がある。

### 9.5 未解決の課題（AJISAIデータセット特有の懸念）

AJISAIデータセットの内訳（cut_in 94, cut_out 72, deceleration 77, u_turn 59, **swerve 130**）を踏まえると、**最大のクラスであるswerveが、咲川氏の位置ベース領域分割が最も苦手とするシナリオ**であることに注意が必要である（論文7.4.3節、図7.1）。対応案：

- AJISAIの本体ログには `pose.rotation`（向き）が含まれているため、咲川氏の抽象化関数に「向きの変化量」または「横方向移動の往復回数」を表す次元を追加する拡張を検討する（論文が今後の課題として提言している内容と一致）。
- CPD側では、この拡張された抽象状態をどう箱に反映するか（例えば `position` とは別に `heading_bin` を持つ「複合ラベル付き箱」を導入するか、あるいは同一 `position` 内での往復を検出する専用の `ctrans`/`cstrans` を定義するか）を別途設計する必要がある。

### 9.6 次のステップ（更新・PoC実施済み）

1. ~~咲川氏の抽象化ツールの所在を確認~~ → **完了**。咲川氏の実装（`github.com/fomaad/Trajectory-Abstraction`, private repo）を取得し、`fomaad/sgcpd` の `vendor/trajectory_abstraction/` に組み込んだ（`abstraction_15area.py`, `abstraction_9area.py`, `abstraction_grid.py`, `safe_15area.py`, `safe_grid.py`, `case_study.py`, `lanelet.py`, `lanelet_stl.py`）。
2. ~~9.2節の変換パイプラインを最小実装~~ → **完了・動作確認済み**。AJISAIの実データ（`TD-NI-AR-SD-N04-CI-0035.json`, cut_in, 1697フレーム）を対象に、以下をエンドツーエンドで確認した。
   - AJISAIの `groundtruth_kinematic`（`{timestamp, groundtruth_ego, groundtruth_vehicles, groundtruth_pedestrians}` の配列）は、咲川氏のツールが元々想定していたキー名（`groundtruth_ego`/`groundtruth_vehicles`）と**そのまま一致**しており、コード変更なしで動作した（9.5節で懸念していたスキーマ差異は cut_in については問題にならなかった）。
   - `abstraction_15area.py` を適用した結果、1697フレーム → 8状態（圧縮率99.5%）に圧縮され、抽象空間・具体空間の両方で「lead-right → lead0」の遷移としてカットインが正しく検出された（サイドカーのラベル `jama_npc_behavior="cut_in"` と整合）。
   - 新規に実装した `vendor/trajectory_abstraction/src/cpd_bridge.py` により、領域名の遷移列 `['far-right', 'lead-right', 'lead0', 'far-front']` を CPDの `(lane, position)` 割当て（9.2節の対応表）に変換し、`gcpd.Model` のインスタンス（`npc1` の箱4つ、`ntrans` 3本）を機械的に構築した。
   - このインスタンスモデルを `gcpd.py` の `s_gen`（SATベースのシナリオ列挙）にそのまま渡したところ、`(npc1 0) @ (-1, 2) at 0 → (npc1 1) @ (-1, 1) at 1 → (npc1 2) @ (0, 1) at 2 → (npc1 3) @ (0, 2) at 3` という、抽象化結果と一致する唯一のシナリオが得られ、**咲川氏の抽象化出力がCPDの語彙にそのまま乗ることを実証した**。
3. 次の作業：
   a. 参照CPDモデル（cut_inセルの許容遷移パターン）を明示的に定義し、9.3節の「参照モデルに対するsat/unsat判定」（今回はインスタンスモデル単体の自己無矛盾性しか見ていない）を実装する。
   b. cut_in 94件全件、および cut_out・deceleration・u_turn の残り3behaviorでも同様にパイプラインを流し、`jama_index.json`/`jama_summary.csv` のラベルとの一致率を集計する。
   c. 9.4節の仮説（CPDのps_colによるFP抑制効果）を検証する。
   d. swerve（130件、最大クラス）・u_turnについては、9.5節で述べた「向き情報の欠如」の影響を実データで確認し、必要な拡張を設計する。

---

## 10. cut-in の参照CPDモデルと格子ベース抽象化（v0.3）

### 10.1 動機：なぜ咲川氏の名前付き領域ではなく格子ベースにするのか

第9節までの `cpd_bridge.py` は、咲川氏の15領域抽象化ツールの出力（`lead-right`, `far-front` のような領域名の列）を、`REGION_TO_LANE_POS` という固定のルックアップ表で `(lane, position)` に変換していた。この方式は「1本のログを要約し、そのログ専用のインスタンスCPDを作る」用途には向いているが、次の点で「cut-inというシナリオ集合そのものを表す参照CPD」を**自分たちで直接書き下す**用途には合わない。

- 15領域モデルの境界（`Dth = v_ego × 2.0s` など）は、咲川氏が別の目的（安全性判定）のために選んだ閾値であり、参照CPDを設計する際にこちらが選びたい粒度（例えば「1箱=5m」）とは独立に決まってしまっている。
- 領域名は9〜15種類の固定カテゴリしかなく、参照CPD側で「隣接レーンのどこから出発してもよい」といった**任意の範囲**を表現しようとすると、結局は領域名を経由せずに `(lane, position)` を直接扱いたくなる。

そこで、**参照CPDを設計するときに使う格子のセルサイズ (gx, gy) を先に決め、ログの離散化にも同じ (gx, gy) を使う**、という順番に変更する。ログのどの時刻についても `position = round(rx/gx)`, `lane = round(ry/gy)` という整数がそのまま得られ（`logverify/grid_bridge.py`）、これは参照CPDモデルの `(lane, position)` の語彙と最初から一致する。咲川氏の抽象化との違いは「固定の意味的カテゴリ」対「設計者が選ぶ物差しの目盛り」であり、後者の方が参照CPDの直接記述と相性が良い。

なお、咲川氏の `vendor/trajectory_abstraction/src/abstraction_grid.py` / `safe_grid.py` も内部的に格子ベースの抽象化を実装済みだが、境界が `floor(ry/gy)` で `k=0` が `[0, gy)` と非対称になっている（自車線中心 `ry=0` を跨がない）。`logverify/grid_bridge.py` では `round()` ベースの対称な離散化（`k=0` が `[-gy/2, +gy/2)`）に変更し、「lane=0 は自車線」という直感と一致させている。

### 10.2 cut-in の参照CPDモデル（`logverify/reference_models.py`）

`build_cutin_reference(i_range, side_lanes=(-1,1), ego_lane=0)` が、次の形の `gcpd.Model` を機械的に生成する。

1. **状態空間**：`side_lanes ∪ {ego_lane}` の各レーンと、`i_range` の各縦方向位置の直積を、それぞれ1つの箱として列挙する（`i_range` を広く取るほど、扱える縦方向レンジが広がる）。
2. **隣接レーン内の移動**：レーンを跨がずに `position` が ±1 だけ変わる遷移（接近・離反）。
3. **合流（cut-inの核心）**：隣接レーンから `ego_lane` へ、`position` が高々1変化して移る遷移。
4. **合流後**：`ego_lane` 内でのみ `position` が変化できる。**隣接レーンへ戻る遷移は一切定義しない。**
5. **非決定的な出発点**：実座標を持たないダミーの開始箱（`START_BOX`）を1つ用意し、そこから隣接レーンの任意の箱へ遷移できるようにする。こうすることで「隣接レーンのどこから出発してもよい」「左右どちらの隣接レーンから来てもよい」という非決定性を、`gcpd.Model` が前提とする「各車について常にちょうど1つの箱がアクティブ」という不変条件を壊さずに表現できる（詳細はモジュール内のコメントを参照）。

4番目の制約が「カットイン」と「蛇行(swerve)」を区別する核である。一度合流したあとに元のレーンへ戻る、あるいは往復するようなトラジェクトリは、対応する遷移が存在しないため、この参照モデルでは構造的に受理されない。

### 10.3 適合性検証（`logverify/membership.py`）

`check_membership_cutin(model, observed)` は、観測された `(lane, position)` の列（`logverify/grid_bridge.py` が出力する圧縮済みグリッド状態列からそのまま作れる）を、参照モデル自身の制約（`add_pos`, `add_lane`, `add_init`, `add_trans`）に対する追加の等式制約として solver に投入し、`solver.check()` を呼ぶ（5.1節・9.3節で述べた方式の実装）。

- **観測の各ステップ** t について「その `(lane, position)` を持つ箱のどれかがアクティブ」という論理和制約を追加する（該当する箱が1つも無ければ、その時点で構造的にUNSATと確定する）。
- ダミー開始箱の分、観測列の先頭はモデルの step 1 に対応する（`start_offset=1`）。
- `gcpd.py` はモジュールレベルの可変な `solver`/`c2i` を持つため、`check_membership` は呼び出しのたびに明示的にリセットする（`reset_solver()`）。

`logverify/demo_cutin_membership.py` で、実データが手元になくても検証できる3種類の合成トラジェクトリ（典型的なcut-in／最初からego車線／合流後に元のレーンへ戻る蛇行）に対してこのパイプラインを流し、期待通り SAT / UNSAT / UNSAT が得られることを確認済みである。

```
$ python3 -m logverify.demo_cutin_membership
cutin_like        : SAT
stays_in_own_lane : UNSAT
swerve_like       : UNSAT
```

### 10.4 未検証・今後の課題

1. **実データでの検証**：本節のパイプラインは合成トラジェクトリでのみ検証済みであり、AJISAIの実ログ（cut_in 94件）に対してはまだ流していない。`logverify/grid_bridge.py` の `grid_states_from_json(path, gx, gy)` はAJISAIの `<id>.json`（`groundtruth_kinematic`）にそのまま使える設計だが、外部（Box）からデータセットを取得できる環境で実行する必要がある。
2. **セルサイズ (gx, gy) の選び方**：本節のデモでは `gx=5.0m, gy=3.5m`（車線幅相当）を暫定的に使ったが、これがAJISAIの実データに対して適切か（粗すぎて別々の状態を同一視してしまわないか、細かすぎて `i_range` が発散しないか）はPoCで確認する必要がある。咲川論文・`safe_grid.py` が実験しているセルサイズの候補（`(1.0,1.0), (1.75,1.75), (3.5,3.5)` 等）を横展開して比較するとよい。
3. **cut_out・deceleration との切り分け**：本節の参照モデルは「隣接レーン→egoレーン」の合流のみを許すため、cut_out（逆方向）や lane change を伴わない加減速シナリオは自動的にUNSATになる（＝別の参照モデルが必要、という結果自体は意図通り）。cut_out用の参照モデル（`ego_lane`→隣接レーンの合流のみ許す、対称な定義）を同様に用意し、`jama_npc_behavior` ラベルとの一致率を集計するのが次の一歩になる。
4. **咲川氏の安全性判定との統合**：9.4節で述べた `ps_col`（CPDの衝突述語）による突き合わせは、本節の格子ベースの参照モデルに対してもそのまま適用できるはずであり、まだ試していない。
5. **蛇行(swerve)の扱いの一般化**：本節のUNSAT判定は「一度合流したら戻れない」という単一の制約に基づく。実際のswerveはもっと多様なパターン（合流せずに近づいたり離れたりを繰り返す等）を含みうるため、`side_lanes` 内の往復も含めて「何が蛇行としてUNSATになるべきか」をAJISAIのswerveラベル（130件）で確認し、必要なら参照モデルをさらに洗練する。
