"""
============================================================================
ReKep 最小自己完結実装 -- 「ペンをペン立てに入れる」ユースケース
============================================================================

このファイルは ReKep (Relational Keypoint Constraints) の "システム全体の
ロジックの流れ" を1ファイルで理解するための教育用実装です。元のリポジトリ
(main.py / subgoal_solver.py / path_solver.py / keypoint_proposal.py /
constraint_generation.py / environment.py / utils.py) の複数ファイルにまた
がる処理を、1つのファイル・他ファイル非依存(NumPy/SciPy のみ)にまとめました。

【このファイルで "本物" として実装している重要部分】
  - 制約付き最適化の階層分解: サブゴール求解 -> 経路求解 -> 実行 のメインループ
  - サブゴールソルバの目的関数(各コスト項と ReKep 制約違反コスト)
  - 経路ソルバの目的関数(経路長 + 経路制約)
  - キーポイント前進モデル(把持物体のみエンドエフェクタと剛体一体で動く)
  - ステージ遷移とバックトラッキング(過去ステージへの反応的な再計画)
  - ReKep 制約 = キーポイントに対する Python 関数(元コードの pen タスクを忠実再現)

【ダミー化(固定値を返す)している重要でない部分】
  - DINOv2 + SAM によるキーポイント提案 -> 固定のキーポイント座標を返す
  - GPT-4o による制約コード生成      -> ハードコードした制約関数を使う
  - IK ソルバ / SDF 衝突 / 物理シミュレータ -> 簡易な近似・固定値

実行: python rekep_minimal.py
============================================================================
"""
import numpy as np
from scipy.optimize import dual_annealing, minimize

# ============================================================================
# 1. 幾何ユーティリティ (transform_utils.py / utils.py 相当)
#    姿勢は教育目的のため [x, y, z, roll, pitch, yaw] の 6 次元で表現する
#    (元コードはクォータニオンを使うが、流れの理解にはオイラー角で十分)
# ============================================================================

def euler2mat(rpy):
    """ZYX オイラー角 -> 3x3 回転行列"""
    rx, ry, rz = rpy
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def pose2mat(pose6):
    """6 次元姿勢 -> 4x4 同次変換行列"""
    m = np.eye(4)
    m[:3, :3] = euler2mat(pose6[3:])
    m[:3, 3] = pose6[:3]
    return m


def normalize_vars(vars, bounds):
    """元の値域 bounds の変数を最適化用に [-1, 1] へ正規化 (utils.normalize_vars)"""
    b = np.asarray(bounds, dtype=float)
    return (vars - b[:, 0]) / (b[:, 1] - b[:, 0]) * 2 - 1


def unnormalize_vars(nvars, bounds):
    """[-1, 1] の変数を元の値域へ戻す (utils.unnormalize_vars)"""
    b = np.asarray(bounds, dtype=float)
    return (nvars + 1) / 2 * (b[:, 1] - b[:, 0]) + b[:, 0]


def transform_keypoints(transform, keypoints, movable_mask):
    """
    キーポイント前進モデル h の核心 (utils.transform_keypoints)。
    movable_mask が True のキーポイント(=把持物体 + エンドエフェクタ)だけに
    剛体変換 transform を適用し、それ以外は静止しているとみなす。
    """
    out = keypoints.copy()
    if movable_mask.sum() > 0:
        out[movable_mask] = keypoints[movable_mask] @ transform[:3, :3].T + transform[:3, 3]
    return out


def consistency(pose_mat, ref_mat, rot_weight=1.5):
    """現在姿勢からの近さ(位置 + 重み付き回転距離)。元の consistency を簡略化。"""
    pos = np.linalg.norm(pose_mat[:3, 3] - ref_mat[:3, 3])
    rot = np.linalg.norm(pose_mat[:3, :3] - ref_mat[:3, :3])  # フロベニウスノルムで回転差を近似
    return pos + rot_weight * rot


def linear_interpolate_poses(start6, end6, n):
    """始点-終点間を線形補間して n 個の 6 次元姿勢を返す(密な軌道生成用)"""
    ts = np.linspace(0, 1, n)
    return np.array([(1 - t) * start6 + t * end6 for t in ts])


# ============================================================================
# 2. ダミーのコスト源 (IK / SDF など。重要でないので固定値を返す)
# ============================================================================

def reachability_cost(pose_mat):
    """
    到達可能性コスト。元コードは IK ソルバの降下反復回数で近似するが、
    ここでは「ワークスペース中心に近いほど到達しやすい」という固定ヒューリスティック。
    """
    dist = np.linalg.norm(pose_mat[:3, 3] - np.array([-0.3, 0.0, 0.75]))
    return float(np.clip(dist, 0.0, 3.0))  # IK 詳細は割愛(ダミー)


def collision_cost(pose_mat):
    """SDF ボクセルによるシーン衝突コスト。ここでは常に 0 を返すダミー。"""
    return 0.0  # SDF 再構成は割愛(ダミー)


def grasp_metric(pose_mat):
    """把持しやすさ。上から(ee の x 軸が -z 方向)に近いほど低コスト [0, 2]。"""
    preferred_dir = np.array([0, 0, -1])
    return float(-np.dot(pose_mat[:3, 0], preferred_dir) + 1)


# ============================================================================
# 3. キーポイント提案(ダミー)  -- 元 keypoint_proposal.py 相当
#    本来は DINOv2 特徴 + SAM マスク + k-means クラスタリングで提案するが、
#    ここでは pen タスクの metadata.json に保存された固定値を返す。
# ============================================================================

def propose_keypoints():
    """7 個のキーポイント(0,1=ペン / 2..6=ペン立て)の 3D 座標を返す(固定)。"""
    return np.array([
        [-0.258, -0.236, 0.691],  # 0: ペン先(下)
        [-0.267, -0.094, 0.712],  # 1: ペンの把持点(上) <- grasp/release 対象
        [-0.272,  0.084, 0.715],  # 2: ペン立て
        [-0.266,  0.082, 0.810],  # 3: ペン立て開口部
        [-0.361,  0.186, 0.718],  # 4: ペン立て開口部
        [-0.262,  0.196, 0.687],  # 5: ペン立て開口部
        [-0.318,  0.218, 0.791],  # 6: ペン立て開口部
    ])


# キーポイント -> 所属オブジェクト(剛体グループ)。把持判定に使う。
KEYPOINT_OBJECT = {0: 'pen', 1: 'pen', 2: 'holder', 3: 'holder',
                   4: 'holder', 5: 'holder', 6: 'holder'}


# ============================================================================
# 4. ReKep 制約生成(ダミー)  -- 元 constraint_generation.py 相当
#    本来は GPT-4o が画像 + 命令から Python 関数を生成するが、ここでは pen
#    タスクのキャッシュ(vlm_query/pen/*.txt)と同じ制約を直接定義する。
#    各制約は (end_effector, keypoints) -> cost で、cost <= 0 で充足とみなす。
#    keypoints はエンドエフェクタを除いたシーンキーポイント配列。
# ============================================================================

# 把持コスト関数。本来は環境が提供(get_callable_grasping_cost_fn)。
# 指定キーポイントを把持していれば 0、していなければ正の大きな値を返す。
GRASP_COST_FN = None  # メインループで env のものを束縛する


def stage1_subgoal_1(end_effector, keypoints):
    """ステージ1: エンドエフェクタをペンの把持点(keypoint 1)に合わせる。"""
    return np.linalg.norm(end_effector - keypoints[1])


def stage2_subgoal_1(end_effector, keypoints):
    """ステージ2: keypoint0->1 のベクトルを z 軸に揃え、ペンを直立させる。"""
    pen_vector = keypoints[1] - keypoints[0]
    return np.linalg.norm(np.cross(pen_vector, np.array([0, 0, 1])))


def stage2_path_1(end_effector, keypoints):
    """ステージ2(経路): ペン(keypoint 1)を把持し続けていること。"""
    return GRASP_COST_FN(1)


def stage3_subgoal_1(end_effector, keypoints):
    """ステージ3: ペン上端をペン立て開口部(keypoints 3..6 の平均)の20cm上へ。"""
    holder_opening = np.mean(keypoints[3:7], axis=0)
    above_holder = holder_opening + np.array([0, 0, 0.2])
    return np.linalg.norm(keypoints[1] - above_holder)


def stage3_subgoal_2(end_effector, keypoints):
    """ステージ3: ペンを直立に保つ。"""
    pen_vector = keypoints[1] - keypoints[0]
    return np.linalg.norm(np.cross(pen_vector, np.array([0, 0, 1])))


def stage3_path_1(end_effector, keypoints):
    """ステージ3(経路): 放すまではペンを把持し続けていること。"""
    return GRASP_COST_FN(1)


# VLM 出力に相当するプログラム情報(metadata.json と同じ構造)
PROGRAM_INFO = {
    'num_stages': 3,
    'grasp_keypoints':   [1, -1, -1],   # 各ステージで把持するキーポイント(-1=なし)
    'release_keypoints': [-1, -1, 1],   # 各ステージで放すキーポイント(-1=なし)
}
CONSTRAINTS = {
    1: {'subgoal': [stage1_subgoal_1],                  'path': []},
    2: {'subgoal': [stage2_subgoal_1],                  'path': [stage2_path_1]},
    3: {'subgoal': [stage3_subgoal_1, stage3_subgoal_2],'path': [stage3_path_1]},
}


# ============================================================================
# 5. 環境(ダミーの物理シミュレータ)  -- 元 environment.py 相当
#    キーポイントの真値を保持し、execute_action でエンドエフェクタを動かす。
#    把持中のオブジェクトのキーポイントはエンドエフェクタと剛体一体で追従する。
# ============================================================================

class DummyEnv:
    def __init__(self):
        self.keypoints = propose_keypoints()         # シーンキーポイントの真値 [7, 3]
        # エンドエフェクタ初期姿勢: 上から掴む向き(x 軸が下向き = roll で反転)
        self.ee_pose = np.array([-0.3, -0.4, 0.9, np.pi, 0.0, 0.0])
        self.grasped_object = None                   # 把持中オブジェクト名 or None
        self._grasp_offsets = {}                     # 把持時のキーポイントの ee ローカル座標

    # ---- 観測 ----
    def get_ee_pose(self):
        return self.ee_pose.copy()

    def get_keypoint_positions(self):
        return self.keypoints.copy()

    def is_grasping(self, obj):
        return self.grasped_object == obj

    # ---- 行動 ----
    def execute_action(self, target_pose6):
        """エンドエフェクタを目標姿勢へ移動(簡略化のため瞬時移動)。
        把持中なら、対象キーポイントを ee に剛体追従させる。"""
        self.ee_pose = target_pose6.copy()
        if self.grasped_object is not None:
            ee_mat = pose2mat(self.ee_pose)
            for idx, local in self._grasp_offsets.items():
                self.keypoints[idx] = (ee_mat @ np.append(local, 1.0))[:3]

    def grasp(self):
        """現在 ee 付近のオブジェクトを把持。キーポイントの ee ローカル座標を記録。"""
        self.grasped_object = 'pen'
        ee_inv = np.linalg.inv(pose2mat(self.ee_pose))
        self._grasp_offsets = {
            idx: (ee_inv @ np.append(self.keypoints[idx], 1.0))[:3]
            for idx, obj in KEYPOINT_OBJECT.items() if obj == 'pen'
        }

    def release(self):
        self.grasped_object = None
        self._grasp_offsets = {}

    def grasping_cost_fn(self, scene_keypoint_idx):
        """指定シーンキーポイントの所属オブジェクトを把持していれば 0、なければ 5。"""
        obj = KEYPOINT_OBJECT[scene_keypoint_idx]
        return 0.0 if self.grasped_object == obj else 5.0


# ============================================================================
# 6. サブゴールソルバ  -- 元 subgoal_solver.py 相当
#    現ステージのサブゴール(次に到達すべき ee 姿勢)を制約付き最適化で求める。
# ============================================================================

# 最適化変数(ee の 6 次元姿勢)の値域。位置はワークスペース、回転は ±pi。
POS_BOUNDS = [(-0.45, -0.05), (-0.45, 0.40), (0.65, 1.05)]
ROT_BOUNDS = [(-np.pi, np.pi)] * 3
SUBGOAL_BOUNDS = POS_BOUNDS + ROT_BOUNDS


def subgoal_objective(x, init_mat, kps_centered, movable, subgoal_cs, path_cs, is_grasp):
    """サブゴール最適化の目的関数。各補助コスト + ReKep 制約違反コストの総和。"""
    pose = unnormalize_vars(x, SUBGOAL_BOUNDS)
    pose_mat = pose2mat(pose)

    cost = 0.0
    cost += 0.8 * collision_cost(pose_mat)                  # シーン衝突回避
    cost += 1.0 * consistency(pose_mat, init_mat)           # 現在姿勢との一貫性
    cost += 1.0 * reachability_cost(pose_mat)               # 到達可能性(IK 近似)
    if is_grasp:
        cost += 10.0 * grasp_metric(pose_mat)               # 把持ステージのみ: 上から把持

    # ReKep 制約: 前進モデルでキーポイントを動かしてから違反量を評価
    tk = transform_keypoints(pose_mat, kps_centered, movable)
    for f in subgoal_cs:
        cost += 200.0 * np.clip(f(tk[0], tk[1:]), 0, np.inf)  # サブゴール制約違反
    for f in path_cs:
        cost += 200.0 * np.clip(f(tk[0], tk[1:]), 0, np.inf)  # 経路制約違反
    return cost


def solve_subgoal(ee_pose, keypoints, movable, subgoal_cs, path_cs, is_grasp,
                  last_x, from_scratch):
    """サブゴール姿勢を求める。初回は Dual Annealing(大域)、以降は SLSQP(局所)。"""
    ee_mat = pose2mat(ee_pose)
    # キーポイントを現在 ee 基準に中心化 -> 最適化中は opt 姿勢で剛体変換するだけでよい
    kps_centered = transform_keypoints(np.linalg.inv(ee_mat), keypoints, movable)
    args = (ee_mat, kps_centered, movable, subgoal_cs, path_cs, is_grasp)
    bounds = [(-1, 1)] * 6

    if from_scratch or last_x is None:
        x0 = normalize_vars(np.concatenate([ee_pose[:3], ee_pose[3:]]), SUBGOAL_BOUNDS)
        res = dual_annealing(subgoal_objective, bounds=bounds, args=args,
                             maxfun=200, x0=x0, no_local_search=False,
                             minimizer_kwargs={'method': 'SLSQP'})
    else:
        res = minimize(subgoal_objective, x0=last_x, args=args,
                       bounds=bounds, method='SLSQP')
    return unnormalize_vars(res.x, SUBGOAL_BOUNDS), res.x


# ============================================================================
# 7. 経路ソルバ  -- 元 path_solver.py 相当
#    現在 ee 姿勢からサブゴール姿勢までの中間制御点を最適化し、密な軌道を返す。
# ============================================================================

PATH_BOUNDS = SUBGOAL_BOUNDS  # 各中間制御点も同じ値域


def path_length(control_points6):
    """制御点列の経路長(位置 + 回転)。短いほど低コスト。"""
    pos = np.sum(np.linalg.norm(np.diff(control_points6[:, :3], axis=0), axis=1))
    rot = np.sum(np.linalg.norm(np.diff(control_points6[:, 3:], axis=0), axis=1))
    return pos + rot


def path_objective(x, start6, end6, kps_centered, movable, path_cs, n_inter):
    """経路最適化の目的関数。経路長 + 到達可能性 + 経路制約違反の総和。"""
    bounds = PATH_BOUNDS * n_inter
    inter = unnormalize_vars(x, bounds).reshape(n_inter, 6)
    ctrl = np.vstack([start6, inter, end6])

    cost = 4.0 * path_length(ctrl)                          # 経路長ペナルティ
    for p in ctrl:
        cost += 0.2 * reachability_cost(pose2mat(p))        # 各制御点の到達可能性
    # 中間制御点で経路制約を評価
    for p in ctrl[1:-1]:
        tk = transform_keypoints(pose2mat(p), kps_centered, movable)
        for f in path_cs:
            cost += 200.0 * np.clip(f(tk[0], tk[1:]), 0, np.inf)
    return cost


def solve_path(start6, end6, keypoints, movable, path_cs):
    """中間制御点を最適化し、線形補間した密な軌道(6次元姿勢の列)を返す。"""
    n_inter = 2  # 中間制御点数(元コードは経路長に応じ 1..4。ここでは固定)
    start_mat = pose2mat(start6)
    kps_centered = transform_keypoints(np.linalg.inv(start_mat), keypoints, movable)
    bounds_full = PATH_BOUNDS * n_inter
    # 初期値は始点-終点の線形補間
    init = linear_interpolate_poses(start6, end6, n_inter + 2)[1:-1].flatten()
    x0 = normalize_vars(init, bounds_full)
    args = (start6, end6, kps_centered, movable, path_cs, n_inter)
    res = minimize(path_objective, x0=x0, args=args,
                   bounds=[(-1, 1)] * len(x0), method='SLSQP')
    inter = unnormalize_vars(res.x, bounds_full).reshape(n_inter, 6)
    ctrl = np.vstack([start6, inter, end6])
    # 制御点列を密な実行軌道へ補間
    dense = []
    for a, b in zip(ctrl[:-1], ctrl[1:]):
        dense.extend(linear_interpolate_poses(a, b, 5)[:-1])
    dense.append(ctrl[-1])
    return np.array(dense)


# ============================================================================
# 8. メイン実行ループ  -- 元 main.py:Main._execute 相当
#    ステージごとに「サブゴール求解 -> 経路求解 -> 実行」を繰り返し、
#    経路制約違反時には過去ステージへバックトラックして反応的に再計画する。
# ============================================================================

class ReKepRunner:
    def __init__(self, env, program_info, constraints):
        self.env = env
        self.program_info = program_info
        self.constraints = constraints
        self.constraint_tolerance = 0.10  # この値以下の違反は充足とみなす
        # 把持コスト関数を制約から参照できるように束縛(元コードの注入に相当)
        global GRASP_COST_FN
        GRASP_COST_FN = env.grasping_cost_fn

    def _get_keypoints(self):
        """ee を先頭(index 0)に連結した全キーポイント配列を返す。"""
        scene = self.env.get_keypoint_positions()
        ee_pos = self.env.get_ee_pose()[:3]
        return np.concatenate([[ee_pos], scene], axis=0)  # [1 + 7, 3]

    def _movable_mask(self):
        """最適化で動かせるキーポイント。index0(ee)は常に可動、把持物体も可動。"""
        mask = np.zeros(1 + len(KEYPOINT_OBJECT), dtype=bool)
        mask[0] = True  # エンドエフェクタ
        for i, obj in KEYPOINT_OBJECT.items():
            mask[i + 1] = self.env.is_grasping(obj)
        return mask

    def _stage_flags(self, stage):
        is_grasp = self.program_info['grasp_keypoints'][stage - 1] != -1
        is_release = self.program_info['release_keypoints'][stage - 1] != -1
        return is_grasp, is_release

    def _check_path_violation(self, stage, keypoints):
        """指定ステージの経路制約の最大違反量を返す。"""
        max_v = 0.0
        for f in self.constraints[stage]['path']:
            max_v = max(max_v, f(keypoints[0], keypoints[1:]))
        return max_v

    def run(self):
        stage = 1
        last_subgoal_x = None
        first_iter = True
        while True:
            keypoints = self._get_keypoints()
            movable = self._movable_mask()
            is_grasp, is_release = self._stage_flags(stage)

            # --- バックトラッキング: 過去ステージの経路制約が破れていたら戻る ---
            if stage > 1 and self._check_path_violation(stage, keypoints) > self.constraint_tolerance:
                new_stage = stage
                for j in range(stage - 1, 0, -1):
                    if self._check_path_violation(j, keypoints) <= self.constraint_tolerance:
                        new_stage = j
                        break
                print(f"  [backtrack] stage {stage} -> {new_stage} (経路制約違反)")
                stage = new_stage
                last_subgoal_x, first_iter = None, True
                continue

            # --- サブゴール求解 ---
            ee_pose = self.env.get_ee_pose()
            subgoal, last_subgoal_x = solve_subgoal(
                ee_pose, keypoints, movable,
                self.constraints[stage]['subgoal'],
                self.constraints[stage]['path'],
                is_grasp, last_subgoal_x, from_scratch=first_iter)

            # --- 経路求解 ---
            path = solve_path(ee_pose, subgoal, keypoints, movable,
                              self.constraints[stage]['path'])
            first_iter = False

            # --- 実行 ---
            for pose in path:
                self.env.execute_action(pose)

            # サブゴール充足判定
            reached = np.linalg.norm(self.env.get_ee_pose()[:3] - subgoal[:3])
            print(f"  [stage {stage}] サブゴール到達 (残差={reached:.3f}, "
                  f"grasp={is_grasp}, release={is_release})")

            # 把持 / リリース動作(ステージ末)
            if is_grasp:
                self.env.grasp()
                print(f"  [stage {stage}] ペンを把持")
            elif is_release:
                self.env.release()
                print(f"  [stage {stage}] ペンを放す")

            # 次ステージへ。全ステージ完了で終了。
            if stage == self.program_info['num_stages']:
                print("タスク完了")
                return
            stage += 1
            last_subgoal_x, first_iter = None, True


# ============================================================================
# 9. エントリポイント
# ============================================================================

if __name__ == "__main__":
    print("ReKep デモ: 白いペンを直立させて黒いペン立てに入れる")
    print("=" * 60)
    env = DummyEnv()
    runner = ReKepRunner(env, PROGRAM_INFO, CONSTRAINTS)
    runner.run()
