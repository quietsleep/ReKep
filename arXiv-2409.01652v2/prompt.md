## Instructions
Suppose you are controlling a robot to perform manipulation tasks by writing constraint functions in Python. The manipulation task is given as an image of the environment, overlayed with keypoints marked with their indices, along with a text instruction. The instruction starts with a parenthesis indicating whether the robot has a single arm or is bimanual. For each given task, please perform the following steps:
- Determine how many stages are involved in the task. Grasping must be an independent stage. Some examples:
  - "(single-arm) pouring tea from teapot":
    - 3 stages: "grasp teapot", "align teapot with cup opening", and "pour liquid"
  - "(single-arm) put red block on top of blue block":
    - 3 stages: "grasp red block", "align red block on top of blue block", and "release red block"
  - "(bimanual) fold sleeves to the center":
    - 2 stages: "left arm grasps left sleeve and right arm grasps right sleeve" and "both arms fold sleeves to the center"
  - "(bimanual) fold a jacket":
    - 3 stages: "left arm grasps left sleeve and right arm grasps right sleeve", "both arms fold sleeves to the center", and "grasp the neck with one arm (the other arm stays in place)", and "align the neck with the bottom"
- For each stage, write two kinds of constraints, "sub-goal constraints" and "path constraints". The "sub-goal constraints" are constraints that must be satisfied **at the end of the stage**, while the "path constraints" are constraints that must be satisfied **within the stage**. Some examples:
  - "(single-arm) pouring liquid from teapot":
    - "grasp teapot" stage:
      - sub-goal constraints: "align the end-effector with the teapot handle"
      - path constraints: None
    - "align teapot with cup opening" stage:
      - sub-goal constraints: "the teapot spout needs to be 10cm above the cup opening"
      - path constraints: "robot is grasping the teapot", and "the teapot must stay upright to avoid spilling"
    - "pour liquid" stage:
      - sub-goal constraints: "the teapot spout needs to be 5cm above the cup opening", "the teapot spout must be tilted to pour liquid"
      - path constraints: "the teapot spout is directly above the cup opening"
  - "(bimanual) fold sleeves to the center":
    - "left arm grasps left sleeve and right arm grasps right sleeve" stage:
      - sub-goal constraints: "left arm grasps left sleeve", "right arm grasps right sleeve"
      - path constraints: None
    - "both arms fold sleeves to the center" stage:
      - sub-goal constraints: "left sleeve aligns with the center", "right sleeve aligns with the center"
      - path constraints: None

Note:
- Each constraint takes a dummy end-effector point and a set of keypoints as input and returns a numerical cost, where the constraint is satisfied if the cost is smaller than or equal to zero.
- For each stage, you may write 0 or more sub-goal constraints and 0 or more path constraints.
- Avoid using "if" statements in your constraints.
- Avoid using path constraints when manipulating deformable objects (e.g., clothing, towels).
- You do not need to consider collision avoidance. Focus on what is necessary to complete the task.
- Inputs to the constraints are as follows:
  - `end_effector`: np.array of shape `(3,)` representing the end-effector position.
  - `keypoints`: np.array of shape `(K, 3)` representing the keypoint positions.
- Inside of each function, you may use native Python functions and NumPy functions.
- For grasping stage, you should only write one sub-goal constraint that associates the end-effector with a keypoint. No path constraints are needed.
- For non-grasping stage, you should not refer to the end-effector position.
- In order to move a keypoint, its associated object must be grasped in one of the previous stages.
- The robot can only grasp one object at a time.
- Grasping must be an independent stage from other stages.
- You may use two keypoints to form a vector, which can be used to specify a rotation (by specifying the angle between the vector and a fixed axis).
- You may use multiple keypoints to specify a surface or volume.
- You may also use the center of multiple keypoints to specify a position.
- A single folding action should consist of two stages: one grasp and one place.

Structure your output in a single python code block as follows for single-arm robot:
```python

# Your explanation of how many stages are involved in the task and what each stage is about.
# ...

num_stages = ?

### stage 1 sub-goal constraints (if any)
def stage1_subgoal_constraint1(end_effector, keypoints):
    """Put your explanation here."""
    ...
    return cost
# Add more sub-goal constraints if needed

### stage 1 path constraints (if any)
def stage1_path_constraint1(end_effector, keypoints):
    """Put your explanation here."""
    ...
    return cost
# Add more path constraints if needed

# repeat for more stages
...
```

Structure your output in a single python code block as follows for bimanual robot:
```python

# Your explanation of how many stages are involved in the task and what each stage is about.
# ...

num_stages = ?

### left-arm stage 1 sub-goal constraints (if any)
def left_stage1_subgoal_constraint1(end_effector, keypoints):
    """Put your explanation here."""
    ...
    return cost

### right-arm stage 1 sub-goal constraints (if any)
def right_stage1_subgoal_constraint1(end_effector, keypoints):
    """Put your explanation here."""
    ...
    return cost
# Add more sub-goal constraints if needed

### left stage 1 path constraints (if any)
def left_stage1_path_constraint1(end_effector, keypoints):
    """Put your explanation here."""
    ...
    return cost
### right stage 1 path constraints (if any)
def right_stage1_path_constraint1(end_effector, keypoints):
    """Put your explanation here."""
    ...
    return cost
# Add more path constraints if needed

# repeat for more stages
...
```

## Query
Query Task: "[INSTRUCTION]"
Query Image: [IMAGE WITH KEYPOINTS]