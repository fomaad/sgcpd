### 1. 環境構築
以下のライブラリがインストールされていることを確認してください。
- numpy
- pandas
- tqdm

インストールコマンド:
pip install numpy pandas tqdm

### 2. データの準備
- 軌道データ（JSON形式）を任意の入力フォルダに配置してください。
- 各スクリプト内の `BASE_INPUT_DIR` および `BASE_OUTPUT_DIR` を、ご自身の実行環境に合わせて修正してください。

### 3. 軌道の抽象化と分析の実行
各手法に対応するスクリプトを実行することで、抽象化された状態遷移ログと統計データが生成されます。

| 実行コマンド | 内容 |
| :--- | :--- |
| python src/abstractor_grid.py | 可変グリッドサイズによる抽象化とシナリオ検出 |
| python src/abstractor_9area.py | 固定9領域モデルによる抽象化と分析 |
| python src/abstractor_15area.py | 遠方判定を含む固定15領域モデルによる抽象化 |

### 4. 安全性プロパティの検証
抽象化された状態に基づき、具体空間との整合性を含めた安全性を検証します。
以下の式に基づくTTC（Time-to-Collision）計算などが含まれます。

$$TTC = \left| \frac{rx}{rel\_v} \right|$$

- グリッドベース検証: `python src/verify_safety_grid.py` を実行
- 15領域モデル検証: `python src/verify_safety_15area.py` を実行

### 5. ケーススタディ
特定のシナリオに対して詳細なステップ分析を行う場合は、以下を実行します。
python src/case_study_analysis.py

---

## 出力ファイルの構成
実行後、指定した出力ディレクトリに以下のファイルが生成されます。

- **`*_result.txt`**: 各フレームの抽象状態、具体空間ラベル、検出イベント（カットイン等）のログ。
- **`summary_all.csv`**: 全シナリオの検出結果や圧縮率をまとめた統計データ。
- **`analysis_dataset.csv`**: 機械学習や詳細分析に利用可能な統合データセット。