"""mini_rekep.py — ReKep 簡易再実装。重い依存なしでコアロジックを1ファイルで再現。
  python mini_rekep.py          # キャッシュ済み制約
  python mini_rekep.py --live   # LLM呼び出し (OPENAI_API_KEY 必要)
"""
import numpy as np, json, os, re, argparse
from scipy.optimize import minimize

# === 1. 模擬環境 — OmniGibson の代わりにシンプルな3D状態を管理 ===
class MockEnv:
    """ペンとホルダーの3Dキーポイント位置だけを保持する模擬環境。"""
    def __init__(self):
        self.ee_pos = np.array([0.0, -0.4, 0.85])   # エンドエフェクタ初期位置
        self.grasped = False
        # pen-in-holder タスクの初期キーポイント (本家 vlm_query/pen/metadata.json 相当)
        self.keypoints = np.array([
            [-0.258, -0.236, 0.691],  # 0: ペン先端
            [-0.267, -0.094, 0.712],  # 1: ペン把持点
            [-0.272,  0.084, 0.715],  # 2: ペン根元
            [-0.266,  0.082, 0.810],  # 3: ホルダー縁1
            [-0.361,  0.186, 0.718],  # 4: ホルダー縁2
            [-0.262,  0.196, 0.687],  # 5: ホルダー縁3
            [-0.318,  0.218, 0.791],  # 6: ホルダー縁4
        ])

    def get_ee_pos(self):
        return self.ee_pos.copy()

    def get_keypoints(self):
        return self.keypoints.copy()

    def move_ee(self, target_pos, steps=5):
        """エンドエフェクタを線形補間で移動。把持中はキーポイントも追従。"""
        for i in range(1, steps + 1):
            alpha = i / steps
            new_pos = self.ee_pos * (1 - alpha) + target_pos * alpha
            if self.grasped:
                delta = new_pos - self.ee_pos
                self.keypoints += delta  # 剛体仮定: 把持物体はEEと一体で動く
            self.ee_pos = new_pos

    def grasp(self):  self.grasped = True
    def release(self): self.grasped = False

# === 2. LLM 制約生成 — VLMにキーポイント+指示を送り制約コードを得る ===
PROMPT_TEMPLATE = """\
You are controlling a robot to perform manipulation tasks by writing Python constraint functions.
Keypoints in the scene (index: 3D position):
{keypoints_description}

Task instruction: "{instruction}"

For each stage, write "sub-goal constraints" (satisfied at end of stage) and "path constraints" (satisfied during stage).
Each function signature: def stageN_TYPE_constraintM(end_effector, keypoints) -> float
  - end_effector: np.array shape (3,)  — the end-effector position
  - keypoints: np.array shape (K, 3)   — the keypoint positions
  - return a cost value; constraint satisfied when cost <= 0
For grasping cost, call: get_grasping_cost_by_keypoint_idx(i)

Output a single ```python block containing:
  num_stages = N
  (constraint functions)
  grasp_keypoints = [...]   # length = num_stages; keypoint idx for grasp stages, -1 otherwise
  release_keypoints = [...]  # length = num_stages; keypoint idx to release at end, -1 otherwise
"""

def call_llm(instruction: str, keypoints: np.ndarray) -> str:
    """OpenAI API を呼び出してReKep制約コードを生成する。"""
    from openai import OpenAI
    client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])
    kp_desc = "\n".join(f"  {i}: {kp.round(3).tolist()}" for i, kp in enumerate(keypoints))
    prompt_text = PROMPT_TEMPLATE.format(
        keypoints_description=kp_desc,
        instruction=instruction,
    )
    print("[LLM] Sending prompt to OpenAI...")
    # --- API呼び出し (本家: constraint_generation.py L120-L129 相当) ---
    stream = client.chat.completions.create(
        model="gpt-4o",
        messages=[{"role": "user", "content": prompt_text}],
        temperature=0.0,
        max_tokens=2048,
        stream=True,
    )
    output = ""
    for chunk in stream:
        if chunk.choices[0].delta.content:
            output += chunk.choices[0].delta.content
    print(f"[LLM] Received {len(output)} chars")
    return output

# === 3. 制約パーサー — LLM出力からPython関数とメタデータを抽出 ===
def parse_constraints(raw_output: str):
    """LLM出力（またはキャッシュ文字列）から制約関数群とメタデータを抽出する。
    本家: constraint_generation.py  _parse_and_save_constraints + _parse_other_metadata 相当。
    """
    m = re.search(r"```python\s*\n(.*?)```", raw_output, re.DOTALL)
    code = m.group(1) if m else raw_output
    # メタデータ抽出 (本家: parse.parse テンプレートマッチング)
    meta = {}
    for line in code.split("\n"):
        for key in ("num_stages", "grasp_keypoints", "release_keypoints"):
            pat = rf"^{key}\s*=\s*(.+)"
            match = re.match(pat, line.strip())
            if match:
                meta[key] = eval(match.group(1))  # noqa: S307
    # 制約関数の安全な実行 (本家: utils.py exec_safe 相当)
    banned = ["import", "__"]
    for b in banned:
        if b in code:
            raise ValueError(f"Banned keyword '{b}' found in LLM output")

    _mock_grasping_cost = lambda idx: 0.0  # ミニ版では常に把持成功とみなす
    local_ns = {}
    exec(code, {"np": np, "get_grasping_cost_by_keypoint_idx": _mock_grasping_cost}, local_ns)
    # 関数名からステージ別・種別別に分類 (本家: _parse_and_save_constraints のgrouping)
    constraints = {}  # {stage: {"subgoal": [fn,...], "path": [fn,...]}}
    for name, obj in local_ns.items():
        if not callable(obj):
            continue
        m = re.match(r"stage(\d+)_(subgoal|path)_constraint\d+", name)
        if m:
            stage = int(m.group(1))
            kind = m.group(2)
            constraints.setdefault(stage, {"subgoal": [], "path": []})
            constraints[stage][kind].append(obj)

    return meta, constraints

# === 4. 簡易サブゴールソルバー — EE位置の最適化 (本家はSE(3), ここはR^3に簡略化) ===
def solve_subgoal(ee_pos, keypoints, movable_mask, subgoal_fns, path_fns):
    """現在のEE位置から、制約を最小化する次のサブゴール位置を求める。
    本家: subgoal_solver.py  objective + SubgoalSolver.solve 相当。
    本家ではSE(3)姿勢をDual Annealing+SLSQPで最適化するが、ミニ版ではR^3のL-BFGS-Bに簡略化。
    """
    def objective(x):
        cost = 0.0
        # 仮想的なキーポイント変換: 可動キーポイントをEEの変位分だけ移動 (本家: transform_keypoints)
        kp = keypoints.copy()
        delta = x - ee_pos
        kp[movable_mask] += delta

        # サブゴール制約コスト (本家の weight=200.0 相当)
        for fn in subgoal_fns:
            violation = fn(x, kp)
            cost += 200.0 * max(violation, 0.0)

        # パス制約コスト
        for fn in path_fns:
            violation = fn(x, kp)
            cost += 200.0 * max(violation, 0.0)

        # 初期姿勢からの逸脱コスト (本家: init_pose_cost, weight=1.0)
        cost += 1.0 * np.linalg.norm(x - ee_pos)
        return cost

    bounds = [(-1.0, 1.0), (-1.0, 1.0), (0.5, 1.5)]
    result = minimize(objective, ee_pos, method="L-BFGS-B", bounds=bounds)
    return result.x, result.fun

# === 5. 実行ループ — ステージ遷移・バックトラック (本家: Main._execute) ===
def execute(env: MockEnv, meta: dict, constraints: dict):
    num_stages = meta["num_stages"]
    grasp_kps = meta["grasp_keypoints"]
    release_kps = meta["release_keypoints"]

    # 可動マスク: EE(index 0) は常に可動。把持後は把持物体のキーポイントも可動になる
    movable = np.zeros(len(env.get_keypoints()), dtype=bool)

    stage = 1
    while stage <= num_stages:
        print(f"\n{'='*50}")
        print(f"  STAGE {stage}/{num_stages}")
        is_grasp = grasp_kps[stage - 1] != -1
        is_release = release_kps[stage - 1] != -1
        if is_grasp:
            env.release()
            print(f"  [Grasp stage] target keypoint = {grasp_kps[stage-1]}")
        print(f"{'='*50}")

        subgoal_fns = constraints.get(stage, {}).get("subgoal", [])
        path_fns = constraints.get(stage, {}).get("path", [])

        for iteration in range(20):  # 最大反復 (本家の while True ループ)
            ee = env.get_ee_pos()
            kp = env.get_keypoints()

            # --- バックトラック判定 (本家: main.py L113-L133) ---
            if stage > 1:
                prev_path_fns = constraints.get(stage, {}).get("path", [])
                violated = any(fn(ee, kp) > 0.10 for fn in prev_path_fns)
                if violated:
                    stage = max(1, stage - 1)
                    print(f"  !! Path constraint violated → backtrack to stage {stage}")
                    break

            # --- サブゴール求解 (本家: _get_next_subgoal) ---
            subgoal, cost = solve_subgoal(ee, kp, movable, subgoal_fns, path_fns)
            dist = np.linalg.norm(subgoal - ee)
            print(f"  iter {iteration}: ee={ee.round(3)}, subgoal={subgoal.round(3)}, "
                  f"dist={dist:.4f}, cost={cost:.4f}")

            # --- アクション実行 (本家: action_queue の消費) ---
            env.move_ee(subgoal, steps=3)

            # --- ステージ完了判定 ---
            if dist < 0.02 or cost < 0.01:
                print(f"  ✓ Stage {stage} subgoal reached")
                if is_grasp:
                    env.grasp()
                    movable[:] = False
                    # 把持物体のキーポイントを可動にする (本家: _update_keypoint_movable_mask)
                    # ミニ版では全キーポイントを可動にする簡略化
                    movable[:] = True
                    print(f"  ✓ Grasped keypoint {grasp_kps[stage-1]}")
                if is_release:
                    env.release()
                    movable[:] = False
                    print(f"  ✓ Released keypoint {release_kps[stage-1]}")
                stage += 1
                break
        else:
            print(f"  ✗ Stage {stage} did not converge, moving on")
            stage += 1

    print(f"\n{'='*50}")
    print("  TASK COMPLETE")
    print(f"  Final EE pos:      {env.get_ee_pos().round(3)}")
    print(f"  Final keypoints:\n{env.get_keypoints().round(3)}")
    print(f"{'='*50}")

# === 6. キャッシュ済み制約 (本家: vlm_query/pen/ 相当) ===
CACHED_CONSTRAINTS = """\
num_stages = 3

def stage1_subgoal_constraint1(end_effector, keypoints):
    cost = np.linalg.norm(end_effector - keypoints[1])
    return cost

def stage2_subgoal_constraint1(end_effector, keypoints):
    pen_vec = keypoints[1] - keypoints[0]
    z_axis = np.array([0, 0, 1])
    cost = np.linalg.norm(np.cross(pen_vec, z_axis))
    return cost

def stage2_path_constraint1(end_effector, keypoints):
    return get_grasping_cost_by_keypoint_idx(1)

def stage3_subgoal_constraint1(end_effector, keypoints):
    holder_opening = np.mean(keypoints[3:7], axis=0)
    above = holder_opening + np.array([0, 0, 0.2])
    cost = np.linalg.norm(keypoints[1] - above)
    return cost

def stage3_subgoal_constraint2(end_effector, keypoints):
    pen_vec = keypoints[1] - keypoints[0]
    z_axis = np.array([0, 0, 1])
    cost = np.linalg.norm(np.cross(pen_vec, z_axis))
    return cost

def stage3_path_constraint1(end_effector, keypoints):
    return get_grasping_cost_by_keypoint_idx(1)

grasp_keypoints = [1, -1, -1]
release_keypoints = [-1, -1, 1]
"""

# === 7. エントリーポイント ===
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mini ReKep — 簡易再実装")
    parser.add_argument("--live", action="store_true",
                        help="LLM (GPT-4o) を実際に呼び出して制約を生成する")
    parser.add_argument("--instruction", default="put the pen into the holder",
                        help="タスク指示文 (--live 時に使用)")
    args = parser.parse_args()

    env = MockEnv()

    if args.live:
        # --- ライブLLM呼び出しフロー ---
        raw = call_llm(args.instruction, env.get_keypoints())
        print("\n--- LLM Raw Output ---")
        print(raw)
        print("--- End LLM Output ---\n")
        meta, constraints = parse_constraints(raw)
    else:
        # --- キャッシュ済み制約を使用 ---
        print("[Using cached constraints (no LLM call)]")
        meta, constraints = parse_constraints(CACHED_CONSTRAINTS)

    print(f"Parsed metadata: {json.dumps(meta, indent=2)}")
    print(f"Constraint stages: {sorted(constraints.keys())}")
    for s in sorted(constraints.keys()):
        print(f"  stage {s}: {len(constraints[s]['subgoal'])} subgoal, "
              f"{len(constraints[s]['path'])} path constraints")

    execute(env, meta, constraints)
