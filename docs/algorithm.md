# ReKep アルゴリズム解説

本ドキュメントは、リポジトリに含まれる論文 *"ReKep: Spatio-Temporal Reasoning of Relational Keypoint Constraints for Robotic Manipulation"*（arXiv:2409.01652v2）とソースコードを対応付けながら、ReKep アルゴリズムを日本語で解説したものです。

---

## 1. ReKep とは

ReKep（**Re**lational **Kep**oint Constraints、関係的キーポイント制約）は、ロボットマニピュレーションにおける多段階・自由形式・反応的な動作を、

1. **シーン内の3Dキーポイント間の関係** を Python 関数として表現された制約で定式化し、
2. **大規模視覚モデル（LVM）と視覚言語モデル（VLM）** によってその制約を自動生成し、
3. **階層的最適化問題** として実時間（約10 Hz）で解く、

という3つの要素から構成される手法です。代表的なデモタスクは「ペンを把持 → 直立させる → ペン立てに落とす」の3段階動作で、`main.py` の `task_list['pen']` として定義されています。

---

## 2. ReKep 制約の定義

### 2.1 キーポイントと制約関数

- キーポイント $k_i \in \mathbb{R}^3$ はシーン表面上の意味のある3D点（例：把手、口、エンドエフェクタ）です。
- ReKep の1インスタンスは次の関数です。

$$
f : \mathbb{R}^{K \times 3} \rightarrow \mathbb{R}, \quad f(\bm{k}) \le 0 \;\Rightarrow\; \text{制約を満たす}
$$

- 実装は **NumPy 演算を含むステートレスな Python 関数** です。キャッシュ例 `vlm_query/pen/stage2_subgoal_constraints.txt` では次のように書かれています。

```python
def stage2_subgoal_constraint1(end_effector, keypoints):
    """ペンを直立させる: keypoints[0]→keypoints[1] ベクトルを z 軸に揃える"""
    pen_vector = keypoints[1] - keypoints[0]
    z_axis = np.array([0, 0, 1])
    cost = np.linalg.norm(np.cross(pen_vector, z_axis))
    return cost
```

### 2.2 ステージごとの2種類の制約

タスクを $N$ ステージに分解し、各ステージ $i$ に2種類の制約集合を与えます（論文 Sec. 3.1）。

- **サブゴール制約** $\mathcal{C}_{\text{sub-goal}}^{(i)}$：ステージ $i$ の**終端**で満たすべき関係（例：注ぎ口がコップの上にある）。
- **経路制約** $\mathcal{C}_{\text{path}}^{(i)}$：ステージ $i$ の**全時刻**で満たすべき関係（例：搬送中はティーポットを直立に保つ）。

コード上では、VLM 出力を `constraint_generation.py:_parse_and_save_constraints` が `stage{i}_subgoal_constraints.txt` と `stage{i}_path_constraints.txt` に分割保存し、`main.py:_execute` で `self.constraint_fns[stage]['subgoal' | 'path']` として読み込みます。

---

## 3. 制約付き最適化としての定式化

全軌道 $\mathbf{e}_{1:T}$（エンドエフェクタ姿勢 $SE(3)$）とステージ遷移時刻 $g_{1:N}$ を同時に決める制約付き最適化（論文 Eq. 1）:

$$
\begin{aligned}
\min_{\mathbf{e}_{1:T}, g_{1:N}} &\; \sum_{i=1}^N \Big[ \lambda_{\text{sub-goal}}^{(i)}(\mathbf{e}_{g_i}) + \sum_{t=g_{i-1}}^{g_i} \lambda_{\text{path}}^{(i)}(\mathbf{e}_t) \Big] \\
\text{s.t.} &\; \mathbf{e}_1 = \mathbf{e}_{\text{init}},\; 0 < g_i < g_{i+1} \\
&\; f(\bm{k}_{g_i}) \le 0,\; \forall f \in \mathcal{C}_{\text{sub-goal}}^{(i)} \\
&\; f(\bm{k}_t) \le 0,\; \forall f \in \mathcal{C}_{\text{path}}^{(i)},\; t = g_{i-1}, \ldots, g_i \\
&\; \bm{k}_{t+1} = h(\bm{k}_t, \mathbf{e}_t)
\end{aligned}
$$

- $\lambda$ は補助コスト（衝突回避、到達可能性、姿勢正則化、解の一貫性 等）。
- $h$ はキーポイントの前進モデルで、**把持対象のキーポイント群にはエンドエフェクタと剛体一体で動く**と仮定します。

---

## 4. 階層的分解による実時間解法

全体を同時に解くのは高コストなので、**その場のサブゴールとそこへの経路のみを解く**形に分解します（論文 Sec. 3.3、Algorithm 1）。

### 4.1 サブゴール最適化（`subgoal_solver.py`）

$$
\min_{\mathbf{e}_{g_i}} \lambda_{\text{sub-goal}}^{(i)}(\mathbf{e}_{g_i}) \quad \text{s.t.}\; f(\bm{k}_{g_i}) \le 0,\; \forall f \in \mathcal{C}_{\text{sub-goal}}^{(i)}
$$

`subgoal_solver.py:objective` に対応する補助コストの内訳：

| 項 | 重み | 役割 |
| --- | --- | --- |
| `collision_cost` | 0.8 | SDF を通じたシーン衝突回避 |
| `init_pose_cost` | 1.0 | 現在姿勢との一貫性（`consistency`） |
| `ik_cost` | 20.0 | IK 反復回数に基づく到達可能性 |
| `reset_reg_cost` | 0.2 | 初期関節姿勢からの正則化 |
| `grasp_cost` | 10.0 | 把持ステージで上からの把持を促進 |
| `subgoal_constraint_cost` | 200.0 | ReKep サブゴール制約違反 |
| `path_constraint_cost` | 200.0 | ReKep 経路制約違反 |

### 4.2 経路最適化（`path_solver.py`）

得られた $\mathbf{e}_{g_i}$ に向けて、現在姿勢から続く軌道 $\mathbf{e}_{t:g_i}$ を解きます。

$$
\min_{\mathbf{e}_{t:g_i}, g_i} \lambda_{\text{path}}^{(i)}(\mathbf{e}_{t:g_i}) \quad \text{s.t.}\; f(\bm{k}_{\hat{t}}) \le 0,\; \forall f \in \mathcal{C}_{\text{path}}^{(i)}
$$

`path_solver.py:objective` の補助コスト：

| 項 | 重み | 役割 |
| --- | --- | --- |
| `collision_cost` | 0.5 | 中間ポーズに対する衝突ペナルティ |
| `path_length_cost` | 4.0 | 位置＋回転の経路長ペナルティ |
| `ik_cost` / `reset_reg_cost` | 20.0 / 0.2 | 制御点ごとの到達可能性と正則化 |
| `path_constraint_cost` | 200.0 | 各サンプル点での経路制約違反 |

決定変数は中間制御点（3〜6点、`get_linear_interpolation_steps` により決定）。解を `get_samples_jitted` で高密度サンプルに展開した上でコスト計算を行います。

### 4.3 最適化器

両ソルバーとも SciPy を利用し、変数を $[-1, 1]$ に正規化して解きます（`normalize_vars` / `unnormalize_vars`）。

- **初回**（`from_scratch=True`）：`scipy.optimize.dual_annealing`（Dual Annealing）の中で SLSQP を局所最適化器として使用。約1秒。
- **以降**：`scipy.optimize.minimize`（SLSQP）で前回解から継続。約10 Hz。

解は `self.last_opt_result` にキャッシュされ、`from_scratch=False` 時の初期値に使われます。

### 4.4 バックトラッキング

`main.py:_execute` のメインループは、各ステップで現ステージの経路制約を再評価し、違反があれば（例：グリッパーからペンが落ちた）**制約が満たされる最も近い過去ステージまで戻る**ことで反応的に再計画します:

```python
if violation > self.config['constraint_tolerance']:
    backtrack = True
# ...
for new_stage in range(self.stage - 1, 0, -1):
    if all_constraints_satisfied:
        break
self._update_stage(new_stage)
```

### 4.5 キーポイント前進モデル

最適化中は視覚追跡を呼べないので、**把持物体に属するキーポイントのみ** エンドエフェクタと剛体一体とみなして変換します（`utils.transform_keypoints`、`keypoint_movable_mask` により識別）。実行時の実座標は視覚系が 20 Hz で再追跡し、次サイクルの最適化入力とします。

---

## 5. キーポイント提案（`keypoint_proposal.py`）

RGB 画像と深度点群から ReKep に渡すキーポイント候補を生成する LVM パイプラインです。

1. **DINOv2 特徴抽出** (`_get_features`): `dinov2_vits14` でパッチ特徴を取得し、双線形補間で元解像度に拡大。
2. **SAM マスク** を入力として各マスクごとに処理。サイズ過大なマスク（`max_mask_ratio` 超）は除外。
3. **特徴クラスタリング** (`_cluster_features`): マスク内特徴を PCA で3次元に縮約し、3D座標を連結してから k-means（`num_candidates_per_mask`、論文では $k=5$）でクラスタリング。各クラスタの重心に最も近いパッチを候補キーポイントとする。
4. **ワークスペース外を除外** (`filter_points_by_bounds`)。
5. **近傍統合** (`_merge_clusters`): `sklearn.MeanShift` を使い、指定半径 `min_dist_bt_keypoints`（論文 8 cm）以内の候補を1点に統合。
6. **番号付き画像の生成** (`_project_keypoints_to_img`): 各候補にインデックスを描画した画像を VLM 入力として返す。

出力：候補キーポイントの3D座標配列と、番号ラベル付き RGB 画像。

---

## 6. ReKep 生成（`constraint_generation.py`）

ラベル付き画像と自然言語命令を GPT-4o に与え、Python 関数として制約を生成します。

1. `prompt_template.txt` を `instruction` で整形し、画像を base64 で埋め込んで `OpenAI.chat.completions.create` に投入（`generate`）。
2. **関数抽出** (`_parse_and_save_constraints`): `def ` で始まる行から `return` までをブロック化し、関数名のプレフィックス（例 `stage2_subgoal_`）でグルーピングし、`stage{i}_{subgoal|path}_constraints.txt` に保存。
3. **メタデータ解析** (`_parse_other_metadata`): 出力中の `num_stages = ...`、`grasp_keypoints = [...]`、`release_keypoints = [...]` を抽出し `metadata.json` として保存。
4. **実行時バインド**: `main.py:_execute` が `load_functions_from_txt` で Python として読み込み、`get_grasping_cost_fn`（把持判定のための補助関数）を注入して呼び出し可能にする。

VLM はキーポイントの**数値**を直接扱わず、「L2 距離」や「内積」など**算術演算**としての空間関係のみを書くため、3D 回転を含む関係も実行時の実測座標で評価できます。

---

## 7. 実行ループ全体像（`main.py`）

`Main.perform_task` の流れ：

1. 環境リセットと RGB-D + マスク取得。
2. `KeypointProposer.get_keypoints` でキーポイント候補と注釈画像を取得。
3. `ConstraintGenerator.generate` で各ステージの制約関数群とメタデータを生成・保存（`--use_cached_query` で `vlm_query/pen` の既成ファイルを利用可能）。
4. `_execute` で：
   - キーポイントを `env.register_keypoints` に登録（視覚追跡）。
   - ステージごとの制約を読み込み。
   - `keypoint_movable_mask` は常にインデックス0（エンドエフェクタ）を True とし、把持判定された物体のキーポイントを True にする。
   - メインループで：
     - 現キーポイント、現エンドエフェクタ姿勢、SDFボクセル、衝突点群を取得。
     - 経路制約違反があればバックトラック。
     - 違反がなければ `SubgoalSolver.solve` → `PathSolver.solve` を順に呼び、結果の経路を `spline_interpolate_poses` で密化し `env.execute_action` で実行。
     - キューが尽きたら把持／リリース動作を実行してステージを進める。
5. 全ステージ完了で動画を保存して終了。

---

## 8. 設定とチューニング

- `configs/config.yaml` に `subgoal_solver` / `path_solver` の `sampling_maxfun`、`minimizer_options`、`constraint_tolerance`、`bounds_min/max` などが定義されています。
- 論文の Known Limitations 節に記されている通り、重いコスト（特に IK 到達可能性）は削減可能で、実時間化には各ソルバーを別プロセスで走らせることが推奨されます。
- 実機展開時は `environment.py` の公開関数（キーポイント追跡、SDF 再構成、IK、把持検出器）を差し替える必要があります（README の "Real-World Deployment" 節）。

---

## 9. まとめ

ReKep の核心は以下の3点に集約されます。

1. **表現**: 3Dキーポイントに対する Python 関数として、意味のある空間関係を自由に書ける統一的な制約表現。
2. **生成**: DINOv2+SAM によるキーポイント候補提案と、GPT-4o による視覚プロンプトからの制約コード生成で、タスクごとの人手設計を排除。
3. **求解**: ステージ単位のサブゴール／経路ソルバを Dual Annealing + SLSQP で解き、キャッシュによる暖機起動とバックトラックで 10 Hz 級の閉ループ制御を実現。

この設計により、事前に物体モデルや専用データセットを用意せずに、多段階・両腕協調・反応的なマニピュレーションを実行できます。
