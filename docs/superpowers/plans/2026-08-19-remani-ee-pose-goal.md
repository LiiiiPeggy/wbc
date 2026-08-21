# REMANI EE 6D Pose Goal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add RViz 6D EE Marker → whole-body IK terminal → existing REMANI plan/execute → actual FK reach verification, without changing Hybrid A* / RRT / optimizer / time scaling or 2D Nav Goal semantics.

**Architecture:** Extend `mm_config` with validated CR10 EE FK (6×9 Jacobian), numerical IK (`FixedBaseArmIk` + `WholeBodyIkSolver` Stage B/C), then wire `REMANIReplanFSM` for readiness/two-phase commit and EE-only completion. Marker is a separate UI node; planner publishes `/ee_current_pose`.

**Tech Stack:** ROS1 catkin, C++14, Eigen, REMANI `MMConfig` / `GridMap` ESDF, `interactive_markers`.

**Spec:** `docs/superpowers/specs/2026-08-19-remani-ee-pose-goal-design.md`

## Global Constraints

- Whole-body IK variable: `ξ = [x,y,yaw,q0..q5] ∈ R^9`; Jacobian `6×9`; REMANI terminal stays `end_pt=[x,y,q0..q5]` + `end_yaw`.
- `/ee_goal` only when `exec_state_ == WAIT_TARGET && !have_target_ && have_odom_ && have_joint_state_` (V1: no EE preemption — covers the short window after `planNextWaypoint` success before the FSM timer leaves `WAIT_TARGET`).
- V1: no EE preemption; no `AsyncSpinner`; no background IK worker.
- EE active state committed **only after** `planNextWaypoint()` returns true.
- `planNextWaypoint()` failure → remain `WAIT_TARGET`; do **not** enter `GEN_NEW_TRAJ`.
- FK chain: `T_car * T_q_0_ * ∏_{i=0}^{5} getAJointTran(i,q_i) * T_tcp`; **do not** use unverified `getJointTMat()` for EE FK.
- Error/Jacobian/FD: world/spatial convention (`e_p = p_des - p_now`, `e_R = Log(R_des * R_now^T)`).
- Hard safety: `checkcollision(car, q, /*safe=*/true)` at terminal; clearance ranking uses new read-only ESDF min API, not collision `min_dist` on no-hit.
- TCP param: `mm/ee_tcp_xyz_rpy` (default identity = flange).
- IK params / reach params: `fsm/ee_*` in `remani_planner_param.yaml`.
- Do not modify: `kino_astar.cpp`, `rrt.cpp` search cores, `poly_traj_optimizer.cpp` costs/time scaling, 2D `waypointCallback` planning body, wheel hard limits.
- New/changed C++ blocks: single banner comment before block (`// ################################` + description), per repo user rule.
- **Const-correctness (V1 option A):** New EE read APIs **do not** declare `const` because existing helpers (`CarState2T`, `getAJointTran`, `getJointTrans`, `getCarPts`) are non-const members. Do not leave a mixed const/non-const state that fails to compile.

---

## Current Source Audit

| Item | Current state (main) |
|------|----------------------|
| Namespace | `remani_planner` |
| `MMConfig` | `mm_config/include/mm_config/mm_config.hpp`, `src/mm_config.cpp`; no `getEePose`, no joint-limit getters, no TCP |
| `getAJointTran` | `void getAJointTran(int joint_num, double theta, Matrix4d &T, Matrix4d &T_grad)` — **non-const** |
| `CarState2T`, `getJointTrans`, `getCarPts` | **non-const** |
| `getJointTMat` | Loops `i=0..manipulator_dof_-2`, prepends `Identity` — **not** full flange chain |
| Collision | `checkcollision(Vector3d car, VectorXd q, bool safe)`; car obs uses `getPreciseDistance` when `precise=true` |
| Obstacle sample geometry (CR10) | `checkCarObsCollision` → `getCarPts`; `checkManiObsCollision` → `manipulator_base_link_pts_` pedestal + `manipulator_link_pts_[i]` for `i=0..5` |
| `GridMap` | `isInMap(Vector2d/3d)`, `getPreciseDistance(Vector3d)` |
| `MMConfig::setParam` | Overload `setParam(nh, grid_map)` binds shared `grid_map_` |
| `REMANIReplanFSM` | `have_odom_` yes; **no** `have_joint_state_`; `init()` calls `planner_manager_->initPlanModules(nh, visualization_)` at `:84` |
| `planNextWaypoint` | Sets `end_pt_`, `end_yaw_`, `have_target_` **only if** `planGlobalTrajWaypoints` succeeds |
| `EXEC_TRAJ` completion | Config-space `touch_the_goal` + `reach goal`; `reached_pub_` publishes `std_msgs::Bool` with `msg.data=true` |
| `GEN_NEW_TRAJ` abort | Invalid terminal or max failures → `have_target_=false`, `WAIT_TARGET` — **no EE metadata cleanup today** |
| `mm_config` CMake | Library from `mm_config.cpp` only |
| `remani_planner` CMake | **No** `mm_config` in `find_package` / `catkin_package` / `package.xml`; must add all three |
| `visMM` / markers | Public `getMMMarkerArray(...)`; private `getManiMarkerArray(...)` — FSM ghost must use public API |
| FSM gripper | `gripper_state_` = actual state; `gripper_flag_` = operation trigger — ghost uses `gripper_state_` |

**Spec vs source:** No blocking conflict. Must **add** EE APIs, `have_joint_state_`, explicit `mm_config` dep, clearance read-only API, EE metadata lifecycle.

---

## Architecture / Data Flow

(Frozen — spec §2.2.) EE callback gate: `WAIT_TARGET && !have_target_ && ready`. Then: pending → IK → ghost on commit path → local `candidate_end_pt/yaw` → `resetFailureCount()` → `planNextWaypoint` → on success only: `active_goal_source_=EE_POSE`. FSM timer `WAIT_TARGET→GEN_NEW_TRAJ`. After `EXEC_TRAJ` terminal: if `EE_POSE`, actual `getEePose` vs `ee_reach_*`; PASS publishes `planning/finish`, FAIL does not. GoalSource lifecycle: `NONE → EE_POSE → NONE` and `NONE → NAV_2D → NONE`.

**FSM init order (mandatory):**
```cpp
planner_manager_->initPlanModules(nh, visualization_);  // existing, first
WholeBodyIkParams ik_params = WholeBodyIkParams::loadFromRosParam(nh);
whole_body_ik_ = std::make_shared<WholeBodyIkSolver>(
    planner_manager_->mm_config_,
    planner_manager_->grid_map_,
    ik_params);
// Do NOT construct a second MMConfig or GridMap.
```

---

## File Creation Order (mandatory)

Execute in this order — **do not reference `whole_body_ik.cpp` before Task 6 creates it**:

| Step | Files created | When |
|------|---------------|------|
| 1 | `ee_kinematics_utils.hpp` | Task 3 |
| 2 | `fixed_base_arm_ik.hpp`, `fixed_base_arm_ik.cpp` | Task 5 |
| 3 | **`whole_body_ik_types.hpp`, `whole_body_ik.hpp`, `whole_body_ik.cpp`** | **Task 6 (scaffold)** |
| 4 | Stage B/C functions inside existing `whole_body_ik.cpp` | Tasks 7, 10–11 |
| 5 | `test_ee_pose_ik.cpp` | Task 4 (FK), extended in later tasks |
| 6 | `ee_goal_marker_node.cpp` | Task 19 |
| 7 | FSM modifications | Tasks 13–18 |

---

## File Structure (new/modified)

| Path | Responsibility |
|------|----------------|
| `mm_config/include/mm_config/mm_config.hpp` | EE FK/Jacobian/TCP/getters/clearance (**non-const** EE methods) |
| `mm_config/src/mm_config.cpp` | Implementations |
| `mm_config/include/mm_config/ee_kinematics_utils.hpp` | `wrapToPi`, SO(3) log, pose error, vee/skew (Eigen only — no ROS msgs) |
| `mm_config/include/mm_config/fixed_base_arm_ik.hpp` | Fixed-base 6-DoF IK primitive |
| `mm_config/src/fixed_base_arm_ik.cpp` | Weighted DLS/LM arm IK |
| `mm_config/include/mm_config/whole_body_ik_types.hpp` | `WholeBodyIkParams`, `WholeBodyGoalCandidate`, `WholeBodyIkResult`, `CandidateSource` |
| `mm_config/include/mm_config/whole_body_ik.hpp` | `WholeBodyIkSolver` + `using Ptr = std::shared_ptr<WholeBodyIkSolver>` |
| `mm_config/src/whole_body_ik.cpp` | Stage B, Stage C, orchestration |
| `mm_config/src/test_ee_pose_ik.cpp` | Milestones A–G unit tests + GridMap fixture |
| `plan_manage/include/plan_manage/remani_replan_fsm.h` | EE FSM state, `clearEeGoalState()`, ghost pub |
| `plan_manage/src/remani_replan_fsm.cpp` | EE lifecycle, completion, abort cleanup |
| `plan_manage/src/ee_goal_marker_node.cpp` | Interactive Marker UI only |
| `plan_manage/config/mm_param_ranger_cr10.yaml` | `mm/ee_tcp_xyz_rpy` |
| `plan_manage/config/remani_planner_param.yaml` | `fsm/ee_*` |
| `plan_manage/launch/remani_sim.launch` | Start marker when ranger+rviz |
| `plan_manage/launch/remani_ranger_cr10.rviz` | InteractiveMarkers on `/ee_goal_marker/update` |

---

## Numerical IK Engineering (Phase 2–3, all weighted solvers)

### Angle / joint update rules

- **Yaw only:** `yaw = wrapToPi(yaw)`; displacement `Δyaw = wrapToPi(yaw - yaw0)`.
- **CR10 joints q0..q5:** bounded joints — **never** `wrapToPi(q_i)`; **never** shortest-angle joint difference.
  - Update: `q_i = clamp(q_i, qmin_i, qmax_i)`.
  - Displacement metric: `Δq_i = q_i - q0_i` (raw subtraction after clamp).
- Trust-region checks use the same rules.

### Task-space weighting (consistent on error **and** Jacobian rows)

Let `e = [e_p; e_R]` (6×1, unscaled for gates).

Internal task-space scaling matrix:
```text
S = diag(1, 1, 1, w_rot, w_rot, w_rot)
A = S * J          # J is 6×9 analytic Jacobian
b = S * e
```
`w_rot` default **0.05** m/rad (5 cm ↔ 1 rad) — fixed constant in solver unless exposed as `WholeBodyIkParams::task_rot_weight`. **Gates** always use raw `||e_p||` and `||e_R||`, not scaled norms.

`FixedBaseArmIk` is a **separate 6×6** solver on `q ∈ R^6` only — see **FixedBaseArmIk weighted DLS** below (no 9-DOF slicing).

### Weighted DLS step (Stage B whole-body IK)

Configuration weight matrix:
```text
W = diag(w_xy, w_xy, w_yaw, w_q, ..., w_q)   # 9×9, from WholeBodyIkParams Stage B weights
M = W^{-2}                                    # diagonal, no matrix inverse call
```

Each iteration:
```text
solve:  (A * M * A^T + λ² I) y = b     # 6×6, Eigen::LDLT — NEVER .inverse()
Δξ = M * A^T * y
```

Apply trust-region clamp on `Δξ`, then project `ξ_new = ξ + Δξ` to **projectable** box only (local xy/yaw trust, joint limits). **Map bounds are not projected** — see Stage B map handling in Task 7.

### FixedBaseArmIk weighted DLS (6×6 arm-only)

Optimization variable: `q ∈ R^6` (not 9-DOF ξ).

```text
J_arm ∈ R^(6×6)   # from getEeJacobian → block(0, 3, 6, 6) at fixed base

S = diag(1, 1, 1, w_rot, w_rot, w_rot)
A_arm = S * J_arm
b = S * e

W_arm = joint_weight * I_6   # FixedBaseArmIkParams::joint_weight > 0; V1 uniform arm weight
M_arm = W_arm^{-2}           # diagonal, no matrix inverse
# Future Work: per-joint diag(w_q0..w_q5) — not V1

solve:  (A_arm * M_arm * A_arm^T + λ² I) y = b   # Eigen::LDLT<Matrix<double,6,6>>
Δq = M_arm * A_arm^T * y   # 6×1 directly — NEVER (M A^T y)(3:8)
```

`joint_weight` comes from `FixedBaseArmIkParams` (mapped from `WholeBodyIkParams::b_weight_joint` via `makeArmIkParams()`). LM accept/reject identical to whole-body solver. Joint update: clamp only.

### LM adaptive damping (trial accept/reject)

`λ` init = `lambda_init`, clamp `[lambda_min, lambda_max]` (from `WholeBodyIkParams` / `FixedBaseArmIkParams`).

**Per iteration:**
1. Build `A, b` from current `ξ`.
2. Solve trial step `Δξ` with current `λ`.
3. Compute trial `ξ_trial`: apply projectable clamps (trust box, joint limits). If `!map->isInMap(x,y)` → **reject trial** (Stage B only), increase λ, retry.
4. Compute trial error norm `||S*e||` for LM comparison.
5. **If** `||S*e_trial|| < ||S*e||`: **accept** `ξ ← ξ_trial`, `λ ← max(λ/3, λ_min)`.
6. **Else:** **reject** trial (keep `ξ`), `λ ← min(λ*10, λ_max)`, **retry same iteration** with new λ (do not advance ξ on rejected step).
7. Max inner λ retries per outer iter: `lambda_retry_max` (default 5); then treat as stagnation exit.

### Convergence semantics (strict)

**Only success condition (inside IK loop or at exit):**
```text
||e_p|| ≤ pos_tol  AND  ||e_R|| ≤ rot_tol
```
Then run hard validity (joint limits, map bounds, `checkcollision(..., true)`).

**These are failure / stagnation exits only — never success:**
- `||Δξ|| < step_tol` with gates not met
- 5 consecutive iterations without `||e||` improvement
- `max_iters` exhausted
- wall-clock timeout

Small step **without** pose gates → exit with `fail_reason` e.g. `stagnation` or `step_too_small`, `success=false`.

### Wall time

`ros::WallTime` budgets: `ee_ik_b_max_ms`, `ee_ik_c_max_ms`, `ee_ik_c_arm_max_ms`.

### NaN guards (IK layer)

- If any `ξ`, `e`, `J`, or `rotationLog(R)` non-finite → abort candidate with `fail_reason = "non_finite"` or `"non_finite_rotation_log"`.
- Do not let NaN enter LDLT.

### ROS quaternion input (FSM layer — not mm_config)

`bool sanitizePoseQuaternion(const geometry_msgs::Quaternion &msg_q, Eigen::Quaterniond &q_out)` lives in `remani_replan_fsm.cpp` anonymous namespace (Task 15). Copies + normalizes into Eigen — **never mutates ConstPtr**. **Not** in `ee_kinematics_utils.hpp`.

### SO(3) log (`rotationLog` in ee_kinematics_utils.hpp)

- Input `R` from FK: use directly.
- Input from ROS quaternion: normalize in FSM first, then build `R`.
- Near zero rotation: `Eigen::AngleAxisd` / first-order OK.
- Near π (trace ≈ −1): stable axis from `(R + I)` column with largest norm.
- **Must return finite**; if not → caller sets `fail_reason = "non_finite_rotation_log"`.

---

## WholeBodyIkParams — complete struct + ROS mapping

Defined in `whole_body_ik_types.hpp`. Loaded via `static WholeBodyIkParams loadFromRosParam(ros::NodeHandle &nh)` reading **`fsm/`** prefix (same node as FSM).

**`WholeBodyIkParams` = unique EE IK parameter source of truth.** `FixedBaseArmIk` never reads ROS params; Stage C builds `FixedBaseArmIkParams` via `makeArmIkParams()`.

```cpp
struct WholeBodyIkParams {
  // --- Topics (FSM uses directly; stored here for single source) ---
  std::string ee_goal_topic;           // fsm/ee_goal_topic, default "/ee_goal"
  std::string ee_current_pose_topic;   // fsm/ee_current_pose_topic, default "/ee_current_pose"

  // --- IK hard gates (solver terminal pose check) ---
  double ik_pos_tol;                   // fsm/ee_ik_pos_tol, 0.01 m
  double ik_rot_tol_rad;               // deg2rad(fsm/ee_ik_rot_tol_deg), 2.0°

  // --- Actual FK reach (FSM may duplicate as ee_reach_* members; same YAML keys) ---
  double reach_pos_tol;                // fsm/ee_reach_pos_tol, 0.02 m
  double reach_rot_tol_rad;            // deg2rad(fsm/ee_reach_rot_tol_deg), 4.0°

  // --- Stage B trust region ---
  double local_xy;                     // fsm/ee_ik_local_xy, 0.40 m
  double local_yaw_rad;                // deg2rad(fsm/ee_ik_local_yaw_deg), 20°

  // --- Stage B iteration / time ---
  int b_max_iters;                     // fsm/ee_ik_b_max_iters, 60
  double b_max_ms;                     // fsm/ee_ik_b_max_ms, 40.0

  // --- Stage B configuration weights (W diagonal) ---
  double b_weight_xy;                  // fsm/ee_ik_b_weight_xy, 5.0
  double b_weight_yaw;                 // fsm/ee_ik_b_weight_yaw, 2.0
  double b_weight_joint;               // fsm/ee_ik_b_weight_joint, 1.0  (also → FixedBaseArmIkParams::joint_weight)

  // --- Stage B fast-path (skip Stage C when hard-valid B passes all) ---
  double b_accept_joint_margin;        // fsm/ee_ik_b_accept_joint_margin, 0.15 rad
  double b_accept_clearance;           // fsm/ee_ik_b_accept_clearance, 0.20 m
  double b_accept_max_xy;              // fsm/ee_ik_b_accept_max_xy, 0.15 m
  double b_accept_max_yaw_rad;         // deg2rad(fsm/ee_ik_b_accept_max_yaw_deg), 10°
  double b_accept_max_dq;              // fsm/ee_ik_b_accept_max_dq, 0.50 rad

  // --- Stage C search geometry ---
  std::vector<double> c_radii;         // fsm/ee_ik_c_radii, {0.50,0.75,1.00,1.25}
  int c_yaw_bins;                      // fsm/ee_ik_c_yaw_bins, 12
  std::vector<double> c_yaw_offsets_rad; // deg2rad(fsm/ee_ik_c_yaw_offsets_deg), {0,-15,15}

  // --- Stage C arm IK per base candidate ---
  int c_arm_max_iters;                 // fsm/ee_ik_c_arm_iters, 40
  double c_arm_max_ms;                 // fsm/ee_ik_c_arm_max_ms, 5.0  (shared deadline per candidate)

  // --- Stage C total wall clock (ONLY cutoff — no early exit, no hidden cap) ---
  double c_max_ms;                     // fsm/ee_ik_c_max_ms, 150.0

  // --- Soft ranking weights ---
  double rank_w_xy;                    // fsm/ee_ik_w_xy, 1.0
  double rank_w_yaw;                   // fsm/ee_ik_w_yaw, 0.3
  double rank_w_q;                     // fsm/ee_ik_w_q, 0.1
  double rank_w_limit;                 // fsm/ee_ik_w_limit, 1.0
  double rank_w_gap;                   // fsm/ee_ik_w_gap, 2.0
  double rank_gap_ref;                 // fsm/ee_ik_gap_ref, 0.20
  double rank_joint_margin_req;        // fsm/ee_ik_joint_margin_req, 0.087 rad

  // --- Task-space rotation row weight (S matrix) ---
  double task_rot_weight;              // fsm/ee_ik_task_rot_weight or internal default 0.05

  // --- LM / convergence numeric (must exist in struct — used by Stage B & FixedBaseArmIk) ---
  double step_tol;                     // fsm/ee_ik_step_tol or internal default 1e-4
  int stagnation_iters;                // fsm/ee_ik_stagnation_iters or internal default 5
  double lambda_init;                  // fsm/ee_ik_lambda_init or internal default 1e-3
  double lambda_min;                   // fsm/ee_ik_lambda_min or internal default 1e-6
  double lambda_max;                   // fsm/ee_ik_lambda_max or internal default 1e6
  int lambda_retry_max;                // fsm/ee_ik_lambda_retry_max or internal default 5

  static WholeBodyIkParams loadFromRosParam(ros::NodeHandle &nh);
};
```

**Numeric field source:** Prefer YAML keys above when present; otherwise set **internal numerical defaults** in `loadFromRosParam()`. Do not leave algorithm fields absent from the struct.

**`loadFromRosParam()` validation (end of load — fail hard, no silent dangerous fallback):**
```cpp
// Require all scalars finite; positive weights; positive tolerances/times/iters;
// lambda_max >= lambda_init && lambda_max >= lambda_min;
// every c_radii[i] finite && > 0; every c_yaw_offsets_rad[i] finite;
// task_rot_weight > 0; b_weight_xy/yaw/joint > 0  (zero → Inf/NaN in M = W^{-2})
if (!valid) {
  ROS_FATAL("[EE IK] invalid WholeBodyIkParams: ...");
  throw std::runtime_error("invalid WholeBodyIkParams");
}
```

**Reach tolerances:** FSM stores `ee_reach_pos_tol_`, `ee_reach_rot_tol_rad_` loaded from same YAML keys in `init()` for completion branch; may copy from `WholeBodyIkParams` after load.

---

## Safety Hierarchy (all phases)

1. Map bounds (`GridMap::isInMap`)
2. IK pose tolerance + joint limits
3. `checkcollision(..., safe=true)` — **do not** weaken thresholds
4. Soft ranking: movement, joint margin, true obstacle clearance

Never accept EE tolerance fail or collision for lower cost.

---

## GridMap Test Fixture (Milestones C–G)

**All tests** (including pure FK) start with CR10 self-init matching `test_cr10_fk.cpp`:

```cpp
// test_ee_pose_ik.cpp
void ensureCr10TestParams(ros::NodeHandle &nh) {
  if (nh.hasParam("mm/manipulator_type")) return;
  // Values MUST match validated test_cr10_fk.cpp:
  nh.setParam("mm/manipulator_type", std::string("cr10"));
  nh.setParam("mm/mobile_base_dof", 2);
  nh.setParam("mm/mobile_base_length", 1.10);
  nh.setParam("mm/mobile_base_width", 0.90);
  nh.setParam("mm/mobile_base_height", 0.40);
  nh.setParam("mm/mobile_base_check_radius", 0.20);
  nh.setParam("mm/mobile_base_wheel_base", 0.56);
  nh.setParam("mm/mobile_base_wheel_radius", 0.125);
  nh.setParam("mm/mobile_base_max_wheel_omega", 4.0);
  nh.setParam("mm/mobile_base_max_wheel_alpha", 8.0);
  nh.setParam("mm/manipulator_dof", 6);
  nh.setParam("mm/manipulator_thickness", 0.06);
  nh.setParam("mm/manipulator_config",
              std::vector<double>{0.1765, 0.607, 0.568, 0.191, 0.125, 0.1084});
  nh.setParam("mm/manipulator_min_pos",
              std::vector<double>{-3, -1.5, -2.8, -3, -3, -3});
  nh.setParam("mm/manipulator_max_pos",
              std::vector<double>{3, 1.5, 2.8, 3, 3, 3});
  nh.setParam("mm/base_mani_fixed_joint_xyz_ypr",
              std::vector<double>{0.2462, 0.0, 0.1, 0.0, 0.0, 0.0});
  nh.setParam("optimization/safe_margin", 0.05);
  nh.setParam("optimization/safe_margin_mani", 0.05);
  nh.setParam("optimization/self_safe_margin", 0.02);
  nh.setParam("optimization/ground_safe_dis", 0.1);
  nh.setParam("mm/ee_tcp_xyz_rpy",
              std::vector<double>{0, 0, 0, 0, 0, 0});
}
```

Pure FK/Jacobian (A–B): `ensureCr10TestParams(nh)` then `MMConfig::setParam(nh)` — `grid_map_` may be null.

**Any test touching** `checkcollision`, clearance, Stage B terminal collision, Stage C, or map bounds **must**:

```cpp
ensureCr10TestParams(nh);
// Then set GridMap params required by initMap (origin, map_size, resolution, ESDF params)
nh.setParam("grid_map//*", ...);  // match GridMap::initMap expectations

auto grid = std::make_shared<GridMap>();
grid->initMap(nh);

MMConfig::Ptr cfg(new MMConfig);
cfg->setParam(nh, grid);  // SAME grid instance → cfg->grid_map_

WholeBodyIkSolver solver(cfg, grid, params);
```

**Controlled obstacle insertion (public GridMap API only):**
1. `ensureCr10TestParams` + map size/origin/resolution params.
2. `grid->initMap(nh)` — empty fixture.
3. `grid->setOccupied(Eigen::Vector3d(ox, oy, oz));`
4. `grid->updateESDF3d();`
5. Verify `grid->getPreciseDistance(pt_near) < grid->getPreciseDistance(pt_far)`.
6. Run clearance / collision / Stage B/C tests.

If `setOccupied` / `updateESDF3d` require additional buffer init, follow real `GridMap::initMap` — **do not** use undocumented cloud helpers.

**Forbidden:** Stage C or clearance tests with `cfg->grid_map_ == nullptr`.

---

## Implementation Phases & Tasks

### Phase 1 — EE FK / TCP / 6×9 Jacobian

**Milestone A:** FK/Jacobian tests pass before any IK code.

---

#### Task 1: TCP param + joint-limit getters in MMConfig

**Files:**
- Modify: `mm_config/include/mm_config/mm_config.hpp`
- Modify: `mm_config/src/mm_config.cpp` (`setParam`)

**Add members:** `Eigen::Matrix4d T_tcp_;`

**Add APIs (non-const where calling non-const helpers internally — getters stay const):**
```cpp
const Eigen::VectorXd& getManipulatorMinPos() const;
const Eigen::VectorXd& getManipulatorMaxPos() const;
Eigen::Matrix4d getTcpTransform() const;
```

**Load:** `mm/ee_tcp_xyz_rpy: [x,y,z,r,p,y]` default zeros → Identity via `urdfRpyToRotation`.

- [ ] **Step 1:** Add getters + `T_tcp_` load.
- [ ] **Step 2:** `catkin_make --pkg mm_config -DCMAKE_BUILD_TYPE=Release`.
- [ ] **Step 3:** Done when test inject sets TCP and getter matches.

---

#### Task 2: `MMConfig::getEePose` (full 6-joint + TCP chain)

**Files:** `mm_config.hpp`, `mm_config.cpp`

**API (V1 option A — no trailing const):**
```cpp
Eigen::Matrix4d getEePose(const Eigen::Vector3d &car_state,
                          const Eigen::VectorXd &q);
```

**Algorithm:**
```cpp
CarState2T(car_state, T_car);
T = T_car * T_q_0_;
for (int i = 0; i < manipulator_dof_; ++i) {
  getAJointTran(i, q(i), Ti, Tgrad_unused);
  T = T * Ti;
}
T = T * T_tcp_;
return T;
```

- [ ] **Step 1:** Implement.
- [ ] **Step 2:** Random base+q vs reference loop in `test_ee_pose_ik.cpp`.
- [ ] **Step 3:** max pos err `< 1e-3` m, rot `< 0.1`° over 100 samples.

---

#### Task 3: `MMConfig::getEeJacobian` (6×9, full TCP chain)

**Files:** `mm_config.hpp`, `mm_config.cpp`, `ee_kinematics_utils.hpp`

**API:**
```cpp
void getEeJacobian(const Eigen::Vector3d &car_state,
                   const Eigen::VectorXd &q,
                   Eigen::Matrix<double, 6, 9> &J);
```

**Algorithm — must include TCP as rightmost constant factor in every column:**

Forward pass: accumulate prefix transforms through car, `T_q_0_`, `T_0..T_{i-1}`.

For each joint column `i ∈ {3..8}` (q0..q5):
```text
dT_ee/dq_i =
  T_car * T_q_0 * T_0 * ... * T_{i-1} * dT_i/dq_i * T_{i+1} * ... * T_5 * T_tcp
```
`T_tcp` is the **rightmost** constant factor in the full chain — do not describe it as "multiply T_tcp on the left of joint partial".

Use `getAJointTran(i, q(i), Ti, dTi_dqi)` for `dTi/dqi`.

**Linear columns:** `J.block<3,1>(0, i) = (dT_ee/dξ_i).block<3,1>(0, 3)`.

**Spatial angular columns (WORLD convention — `R_ee^T` appears exactly once):**

For variable `ξ_i`, with `T_ee = getEePose(...)`:
```text
Rdot_i = (dT_ee/dξ_i).block<3,3>(0, 0)
R_ee   = T_ee.block<3,3>(0, 0)

Omega_i = Rdot_i * R_ee.transpose()
Omega_i = 0.5 * (Omega_i - Omega_i.transpose())   // enforce skew-symmetry
omega_i = vee(Omega_i)

J.block<3,1>(3, i) = omega_i
```

**Forbidden:** `Omega = (dR * R_ee^T) * R_ee^T` or any double application of `R_ee^T`.

**Base columns 0–1 (x, y):**
- `J(0:2, 0) = [1, 0, 0]^T`, `J(0:2, 1) = [0, 1, 0]^T`
- Angular rows `= 0`

**Base column 2 (yaw):** derivative of full `T_car(yaw) * T_q_0 * arm * T_tcp`. Recommended geometric form:
```text
omega_yaw = [0, 0, 1]^T
v_yaw = z_world × (p_ee - p_base_origin)
J.block<3,1>(0, 2) = v_yaw
J.block<3,1>(3, 2) = omega_yaw
```
Must match analytic `dT/dyaw` and FD verification.

**TCP effect on Jacobian (clarification):**
- Non-zero TCP **translation** changes EE position and **linear** Jacobian rows (lever arm).
- For a rigidly attached TCP, **spatial angular velocity** does not change solely due to TCP translation offset.
- TCP FD smoke verifies: (1) `getEePose` position shift, (2) linear rows change with TCP offset, (3) angular rows still match world/spatial FD.

**Utils (Eigen only — no geometry_msgs):**
```cpp
double wrapToPi(double a);
Eigen::Matrix3d skew(const Eigen::Vector3d &v);
Eigen::Vector3d vee(const Eigen::Matrix3d &S);
Eigen::Vector3d rotationLog(const Eigen::Matrix3d &R);  // must return finite or signal failure
void poseError(const Eigen::Matrix4d &T_now, const Eigen::Matrix4d &T_des,
               Eigen::Matrix<double,6,1> &e);
// e_p = p_des - p_now; e_R = Log(R_des * R_now^T)
bool allFinite(const Eigen::Matrix<double,6,1> &e);
bool allFinite(const Eigen::Matrix<double,6,9> &J);
```

- [ ] **Step 1:** Implement utils + Jacobian with full TCP chain.
- [ ] **Step 2:** FD all 9 columns; report **absolute and relative** error per column:
```cpp
abs_err = ||J_analytic.col(i) - J_fd.col(i)||
rel_err = abs_err / max(1.0, ||J_fd.col(i)||)
// PASS if abs_err < 1e-5 (or 1e-4) OR rel_err < 1e-4 — avoids false fail on near-zero columns
```
- [ ] **Step 3:** TCP offset smoke: position + linear rows change; angular rows match FD.

---

#### Task 4: FK/Jacobian test executable (Milestone A)

**Files:** Create `test_ee_pose_ik.cpp`; modify `mm_config/CMakeLists.txt`

Tests 1–6: random whole-body FK, q5 sensitivity, 6×9 FD (abs+rel per column), q5 column, TCP identity, TCP offset smoke (position + linear rows; angular FD consistency).

**Param init:** call `ensureCr10TestParams(nh)` before `MMConfig::setParam` (see GridMap fixture section).

**CMake at Task 4 (only files that exist now):**
```cmake
add_library(mm_config
  src/mm_config.cpp
)
add_executable(test_ee_pose_ik src/test_ee_pose_ik.cpp)
target_link_libraries(test_ee_pose_ik mm_config ${catkin_LIBRARIES})
```

- [ ] **Step 1:** Add executable + `ensureCr10TestParams`.
- [ ] **Step 2:** `rosrun mm_config test_ee_pose_ik` — all PASS.

---

### Phase 2 — Fixed-base CR10 numerical IK primitive

**Milestone B**

---

#### Task 5: `FixedBaseArmIk` class

**Files:** `fixed_base_arm_ik.hpp`, `fixed_base_arm_ik.cpp`

**CMake (Task 5 — add one source only):**
```cmake
add_library(mm_config
  src/mm_config.cpp
  src/fixed_base_arm_ik.cpp
)
```

**Includes for `fixed_base_arm_ik.hpp`:**
```cpp
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <limits>
#include <string>
#include <mm_config/mm_config.hpp>
```

**Types in `fixed_base_arm_ik.hpp`:**
```cpp
struct FixedBaseArmIkParams {
  double pos_tol;
  double rot_tol_rad;
  int max_iters;

  double task_rot_weight;   // w_rot for S matrix
  double joint_weight;      // W_arm = joint_weight * I_6; MUST be > 0

  double lambda_init;
  double lambda_min;
  double lambda_max;
  int lambda_retry_max;

  double step_tol;
  int stagnation_iters;

  // max_ms NOT stored here — absolute WallTime deadline passed to solve()
};

struct FixedBaseArmIkResult {
  bool success{false};

  Eigen::VectorXd q;

  double pos_err{std::numeric_limits<double>::infinity()};
  double rot_err_rad{std::numeric_limits<double>::infinity()};

  int iters{0};

  std::string fail_reason;
};

class FixedBaseArmIk {
 public:
  explicit FixedBaseArmIk(const MMConfig::Ptr &cfg);
  FixedBaseArmIkResult solve(
      const Eigen::Vector3d &car_xyyaw,
      const Eigen::Matrix4d &T_goal,
      const Eigen::VectorXd &q_seed,
      const FixedBaseArmIkParams &p,
      const ros::WallTime &deadline);  // absolute deadline
 private:
  MMConfig::Ptr cfg_;
};
```

**No ROS param load inside FixedBaseArmIk.** Params always come from caller (`makeArmIkParams()` in Stage C / unit tests built from `WholeBodyIkParams`).

**Algorithm:**
- Fixed base; `getEeJacobian` → `J_arm = J.block(0, 3, 6, 6)`.
- **6×6 weighted DLS:** `W_arm = joint_weight * I_6`, `M_arm = W_arm^{-2}`, `Δq = M_arm A_arm^T y` — **no** 9-DOF slicing.
- LM trial accept/reject using `p.lambda_*` / `p.lambda_retry_max`.
- Joint update: clamp only, no wrap.
- Success **only** if pose gates pass and all values finite.

**Multi-seed (Stage C):** see Task 11 — shared `candidate_deadline` capped by `stage_deadline`.

**Tests (forward-generated ground truth — pose-based PASS, not ||q||):**
```cpp
q_goal = random legal joints within limits;
T_goal = cfg->getEePose(car_xyyaw, q_goal);
q_seed = q_goal + small perturbation; clamp;
FixedBaseArmIkParams arm_p = /* from WholeBodyIkParams fields */;
result = arm_ik.solve(car_xyyaw, T_goal, q_seed, arm_p, deadline);

EXPECT result.success;
EXPECT q within joint limits;
poseError(getEePose(...), T_goal, e);
EXPECT pos/rot within ik_*_tol; all finite;
// OPTIONAL diagnostic: ||result.q - q_goal||
```

**Failure test (deterministic — do NOT use "bad orientation"):**
```cpp
// Fixed base; place p_goal far beyond CR10 geometric reach from arm mount
// (max link sum from manipulator_config + safety margin)
T_unreachable.translation = far point; orientation = Identity;
result = arm_ik.solve(..., T_unreachable, q_seed, arm_p, deadline);
EXPECT !result.success;
EXPECT result.fail_reason in {timeout, stagnation, pose_not_reached, ...};
EXPECT no NaN; q stays within joint limits; exits by deadline
```

- [ ] **Step 1:** Implement with 6×6 LDLT + LM accept/reject.
- [ ] **Step 2:** Forward FK goal test PASS (pose gates).
- [ ] **Step 3:** Unreachable-position failure test PASS.

---

### Phase 3 — Whole-body IK scaffold (types + empty solver shell)

**Must complete before Stage B.**

---

#### Task 6: Create `whole_body_ik_types.hpp`, `whole_body_ik.hpp`, `whole_body_ik.cpp`

**Files:**
- Create: `mm_config/include/mm_config/whole_body_ik_types.hpp`
- Create: `mm_config/include/mm_config/whole_body_ik.hpp`
- Create: `mm_config/src/whole_body_ik.cpp`
- Modify: `mm_config/CMakeLists.txt` — append **`whole_body_ik.cpp` only**

**CMake after Task 6 (final expected list — each source once):**
```cmake
add_library(mm_config
  src/mm_config.cpp
  src/fixed_base_arm_ik.cpp
  src/whole_body_ik.cpp
)
```

**Includes for `whole_body_ik_types.hpp`:**
```cpp
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <limits>
#include <string>
#include <vector>
```

**Types (with defaults — avoid uninitialized stub returns):**
```cpp
enum class CandidateSource { B, C };

struct WholeBodyGoalCandidate {
  Eigen::Vector3d base_xyyaw = Eigen::Vector3d::Zero();
  Eigen::VectorXd q;
  double pos_err = std::numeric_limits<double>::infinity();
  double rot_err_rad = std::numeric_limits<double>::infinity();
  double base_xy_disp = 0.0;
  double yaw_disp = 0.0;
  double q_disp_norm = 0.0;
  double min_joint_margin = 0.0;
  double obstacle_clearance = 0.0;
  bool hard_valid{false};
  double cost = std::numeric_limits<double>::infinity();
  CandidateSource source{CandidateSource::B};
  std::string fail_reason;
};

struct WholeBodyIkResult {
  bool success{false};
  Eigen::Matrix<double,9,1> xi_best = Eigen::Matrix<double,9,1>::Zero();
  WholeBodyGoalCandidate best;
  std::string fail_reason;
};

struct WholeBodyIkParams { /* full fields in WholeBodyIkParams section above */
  static WholeBodyIkParams loadFromRosParam(ros::NodeHandle &nh);
};
```

**Class shell:**
```cpp
#include <mm_config/whole_body_ik_types.hpp>
#include <mm_config/fixed_base_arm_ik.hpp>
#include <memory>

class WholeBodyIkSolver {
 public:
  using Ptr = std::shared_ptr<WholeBodyIkSolver>;

  WholeBodyIkSolver(MMConfig::Ptr cfg, GridMap::Ptr map, WholeBodyIkParams params);

  WholeBodyIkResult solve(const Eigen::Matrix<double,9,1> &xi_start,
                          const Eigen::Matrix4d &T_goal);

 private:
  MMConfig::Ptr cfg_;
  GridMap::Ptr map_;
  WholeBodyIkParams params_;
  FixedBaseArmIk arm_ik_;

  FixedBaseArmIkParams makeArmIkParams() const;

  bool runStageB(..., WholeBodyGoalCandidate &out);
  bool runStageC(..., std::vector<WholeBodyGoalCandidate> &out);
};
```

**`makeArmIkParams()` — sole WholeBodyIkParams → FixedBaseArmIkParams mapping:**
```cpp
FixedBaseArmIkParams WholeBodyIkSolver::makeArmIkParams() const {
  FixedBaseArmIkParams p;
  p.pos_tol = params_.ik_pos_tol;
  p.rot_tol_rad = params_.ik_rot_tol_rad;
  p.max_iters = params_.c_arm_max_iters;
  p.task_rot_weight = params_.task_rot_weight;
  p.joint_weight = params_.b_weight_joint;  // W_arm = joint_weight * I_6
  p.lambda_init = params_.lambda_init;
  p.lambda_min = params_.lambda_min;
  p.lambda_max = params_.lambda_max;
  p.lambda_retry_max = params_.lambda_retry_max;
  p.step_tol = params_.step_tol;
  p.stagnation_iters = params_.stagnation_iters;
  return p;
}
```

**Initial `whole_body_ik.cpp`:** Constructor + stub `solve` returning `WholeBodyIkResult{}` with `success=false`, `fail_reason="not implemented"` (defaults protect uninit doubles).

- [ ] **Step 1:** Create files + CMake (`whole_body_ik.cpp` only).
- [ ] **Step 2:** `catkin_make --pkg mm_config` links cleanly.
- [ ] **Step 3:** `loadFromRosParam` + validation smoke (reject zero weights).

---

### Phase 4 — Stage B weighted whole-body IK

**Milestone C**

---

#### Task 7: Implement `runStageB` in `whole_body_ik.cpp`

**Files:** Modify existing `whole_body_ik.cpp` (created Task 6)

**Function:**
```cpp
bool runStageB(const Eigen::Matrix<double,9,1> &xi0,
               const Eigen::Matrix4d &T_goal,
               WholeBodyGoalCandidate &out_cand);
```

**Behavior:**
- Weighted DLS on 9-DOF per Numerical IK section (`A=S*J`, `M=W^{-2}`, LDLT).
- Trust region: `ee_ik_local_xy`, `ee_ik_local_yaw_deg` on **Δ** from `xi0`.
- Yaw wrap on state; joint clamp only.
- LM accept/reject; stagnation/small-step are **failures**, not success.

**Projectable constraints (clamp after trial step):**
- Base local xy trust box
- Yaw trust (wrapped)
- Joint limits

**Non-projectable validity (hard check — do NOT clamp to map boundary):**
After forming `xi_trial` and applying projectable clamps:
```cpp
if (!map_->isInMap(Eigen::Vector2d(x, y))) {
  // reject this LM trial — do not accept step
  increase lambda; continue;
}
```
GridMap has no terminal-state projection semantics — never "project base to map edge".

- After loop: pose gates → joint limits → `map->isInMap` → `cfg->checkcollision(..., true)`.
- Fill candidate metrics; `source=B`, `hard_valid` from gates+collision.

**Tests (forward-generated):**
```cpp
// Near test
xi_goal = xi_start + small perturbation within trust region;
T_goal = getEePose(xi_goal);
EXPECT runStageB success; small base motion.

// B→C handoff (do NOT assume base displacement > local_xy implies B failure)
// 1. Choose known-valid xi_goal; T_goal = getEePose(xi_goal)
// 2. Choose xi_start where Stage B trust region + sampled checks show
//    no accepted B solution (obstacle/joint-limit exclusion near current base OK)
// 3. Ensure some Stage C base candidate can reach T_goal via fixed-base arm IK
// Goal: Stage B cannot provide accepted solution → Stage C can
```

Uses **GridMap fixture** for collision terminal test.

- [ ] **Step 1:** Implement Stage B.
- [ ] **Step 2:** Near-current goal test PASS.
- [ ] **Step 3:** Terminal collision occupied voxel → `hard_valid=false`.

---

### Phase 5 — Candidate metrics + clearance API

**Milestone D**

---

#### Task 8: Ranking helpers

**Files:** `whole_body_ik_types.hpp` or `whole_body_ik.cpp`

**Functions:**
```cpp
double computeJointLimitMargin(MMConfig &cfg, const Eigen::VectorXd &q);
double computeCandidateCost(const WholeBodyIkParams &p, const WholeBodyGoalCandidate &c);
bool passesFastPathQuality(const WholeBodyIkParams &p, const WholeBodyGoalCandidate &c);
```

Cost formula per spec §5.6 (movement + limit + gap terms).

- [ ] **Step 1:** Implement.
- [ ] **Step 2:** Two hard-valid candidates — worse margin → higher cost.

---

#### Task 9: Read-only obstacle clearance API

**Files:** `mm_config.hpp`, `mm_config.cpp`

**API (V1 option A — non-const):**
```cpp
double getWholeBodyObstacleClearance(const Eigen::Vector3d &car_state,
                                     const Eigen::VectorXd &q);
double getCarObstacleClearance(const Eigen::Vector3d &car_state);
```

**Implementation — scan all CR10 obstacle sample points (same geometry as hard collision, read-only min ESDF):**

1. **Ranger base:** all points from `getCarPts(car_state, car_pts)` → min `grid_map_->getPreciseDistance(pt)`.
2. **Manipulator pedestal:** `manipulator_base_link_pts_` transformed by `T_q * T_q_0_` (same as `checkManiObsCollision` prefix loop).
3. **Arm links q0..q5:** for each link `i`, transform `manipulator_link_pts_[i]` through accumulated `T_now` (same loop structure as `checkManiObsCollision`).

**Exclude:** self-collision pairs (`checkCarManiCollision` geometry) — clearance is **obstacle ESDF only**.

**Do not** change `checkCarObsCollision`, `checkManiObsCollision`, or safe-margin thresholds.

**Cheap base variant:** car samples only (Stage C pre-filter lower bound).

- [ ] **Step 1:** Implement min-distance scanners.
- [ ] **Step 2:** GridMap fixture: empty → large clearance; obstacle at `(1,0,0.5)` → small clearance at nearby config.
- [ ] **Step 3:** `checkcollision` bool unchanged on same states.

---

### Phase 6 — Stage C base candidate search

**Milestone E (partial)**

---

#### Task 10: Base candidate generation + cheap filter

**Files:** `whole_body_ik.cpp`

**Generate:** `radii × yaw_bins × yaw_offsets` + current base (front) → **145** max.

**Cheap filter:**
- `map->isInMap(x,y)`
- `checkCarObsCollision(car_state, precise=true, safe=true, min_dist)` → discard if **true** (hit). Matches terminal safety margin.

**Cheap score:** `rank_w_xy * ||Δp_base|| + rank_w_yaw * |Δyaw|` with wrapped yaw delta; sort ascending (proximity to current base, not fixed world θ index).

**V1 cutoff:** only `ee_ik_c_max_ms` stops search — **no** early exit on cost, **no** hidden candidate cap.

- [ ] **Step 1:** Generator + filter + sort.
- [ ] **Step 2:** Map-edge bases filtered; order varies with robot pose.

---

#### Task 11: Stage C arm IK loop

**Files:** `whole_body_ik.cpp`

**Loop (explicit for — advances candidate index; total time never exceeds stage deadline):**
```cpp
const FixedBaseArmIkParams arm_params = makeArmIkParams();
const ros::WallTime stage_deadline =
    stage_start + ros::WallDuration(params_.c_max_ms / 1000.0);

for (const auto &base_candidate : sorted_bases) {
  if (ros::WallTime::now() >= stage_deadline) break;

  const ros::WallTime now = ros::WallTime::now();
  const ros::WallTime candidate_deadline = std::min(
      stage_deadline,
      now + ros::WallDuration(params_.c_arm_max_ms / 1000.0));
  if (candidate_deadline <= now) break;

  // 1. arm_ik_.solve(car, T_goal, q_current, arm_params, candidate_deadline)
  // 2. If fail && now < candidate_deadline:
  //       arm_ik_.solve(car, T_goal, q_mid, arm_params, candidate_deadline)
  // 3. On first IK success → pose gates → checkcollision(..., true)
  //    → clearance + margins → push pool
}
```

**B→C integration test:** Obstacle/joint-limit scenario (Task 7) — Stage B no accepted solution, Stage C finds one.

**Stage C success test (forward-generated — not "wrong orientation"):**
```cpp
base_goal, q_goal = known-valid;
T_goal = getEePose(base_goal, q_goal);
// arrange so Stage C evaluates a candidate near base_goal
result = runStageC / solve(...);
EXPECT hard_valid solution with pos_err <= ik_pos_tol, rot_err <= ik_rot_tol;
```

**Separate unit test for `poseError` rotation gate:**
```cpp
// same position, orientation error > rot_tol → gate detects failure
// (do NOT assume "wrong orientation ⇒ no IK solution")
```

- [ ] **Step 1:** Implement for-loop with `makeArmIkParams()` + capped deadlines.
- [ ] **Step 2:** B→C handoff PASS.
- [ ] **Step 3:** Forward Stage C pose gates + `poseError` unit test PASS.

---

### Phase 7 — WholeBodyIkSolver orchestration

**Milestone E (complete)**

---

#### Task 12: `WholeBodyIkSolver::solve` orchestration only

**Files:** Modify existing `whole_body_ik.hpp`, `whole_body_ik.cpp` — **do not recreate files**

**Logic (spec §5.3):**
```text
runStageB → candB
if !candB.hard_valid → runStageC only
else if passesFastPathQuality(candB) → return candB
else → runStageC; pool = {candB if hard_valid} ∪ C_valid; pick min cost
```

**Must NOT touch:** FSM, ROS, `have_target_`, `planNextWaypoint`.

- [ ] **Step 1:** Wire orchestrator in existing cpp.
- [ ] **Step 2:** Integration tests: unreachable goal, collision terminal rejected, B+C ranking.

---

### Phase 8 — FSM lifecycle / readiness / two-phase commit

**Milestone F**

---

#### Task 13: FSM state members + joint readiness

**Files:** `remani_replan_fsm.h`, `remani_replan_fsm.cpp`

**Add:**
```cpp
enum class GoalSource { NONE, NAV_2D, EE_POSE };

bool have_joint_state_{false};
GoalSource active_goal_source_{GoalSource::NONE};
bool pending_ee_goal_{false};
bool active_ee_goal_{false};
Eigen::Matrix4d T_world_ee_goal_;
Eigen::Matrix4d T_world_ee_pending_;

WholeBodyIkSolver::Ptr whole_body_ik_;

ros::Subscriber ee_goal_sub_;
ros::Publisher ee_current_pose_pub_;
ros::Publisher ee_ik_terminal_marker_pub_;  // MarkerArray, topic /ee_ik_terminal_markers
ros::Time last_ee_current_pose_pub_;

// clearEeGoalState helpers — see Task 15
void clearPendingEeGoal();     // pending_ee_goal_=false only
void clearActiveEeGoal();      // active_ee_goal_=false; active_goal_source_=NONE; delete ghost
void clearEeGoalState();       // both pending + active
void publishEeTerminalGhost(const Eigen::Vector3d &car, const Eigen::VectorXd &q);
void deleteEeTerminalGhost();
```

`clearActiveEeGoal()` always sets `active_goal_source_ = GoalSource::NONE` (closes EE lifecycle). NAV success path also sets `NONE` after reach (Task 17).

**`mmManiOdomCallback`:** full joint received → `have_joint_state_ = true`.

- [ ] **Step 1:** Add members.
- [ ] **Step 2:** Short joint state → `have_joint_state_` stays false.

---

#### Task 14: `/ee_current_pose` publisher

**Files:** `remani_replan_fsm.cpp`

At ≤20 Hz when `have_odom_ && have_joint_state_`:
```cpp
T = planner_manager_->mm_config_->getEePose({x,y,yaw}, q);
// geometry_msgs/PoseStamped, frame_id=world, topic from params
```

- [ ] **Step 1:** Advertise + publish.
- [ ] **Step 2:** Sim smoke after joint_state.

---

#### Task 15: `eeGoalCallback`, two-phase commit, 2D replacement, ghost viz

**Files:** `remani_replan_fsm.h/cpp`

**Includes (FSM):**
```cpp
#include <mm_config/whole_body_ik.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
```

**Final `/ee_goal` acceptance gate (V1 no preemption):**
```cpp
if (exec_state_ != WAIT_TARGET || have_target_) {
  ROS_WARN("[EE GOAL] ignored: planner is not idle");
  return;
}
if (!(have_odom_ && have_joint_state_)) {
  ROS_WARN("[EE GOAL] robot state not ready");
  return;
}
// then: frame_id == world; CR10; target_type_ != PRESET
```

Accept only when:
```text
exec_state_ == WAIT_TARGET
AND have_target_ == false
AND have_odom_ == true
AND have_joint_state_ == true
AND frame valid AND CR10 AND not PRESET
```

**`eeGoalCallback` (pending only — does not touch active until plan succeeds):**

**Local helpers in `remani_replan_fsm.cpp` (anonymous namespace) — do NOT mutate ConstPtr:**
```cpp
bool sanitizePoseQuaternion(const geometry_msgs::Quaternion &msg_q,
                            Eigen::Quaterniond &q_out) {
  // 1. x/y/z/w finite check → false if not
  // 2. Eigen::Quaterniond q(msg_q.w, msg_q.x, msg_q.y, msg_q.z);
  // 3. if (q.norm() < 1e-12) return false;
  // 4. if (|norm - 1| large) ROS_WARN only
  // 5. q.normalize(); q_out = q; return true
  // NEVER modify msg_q / ConstPtr message
}
```

**Inline pose → Matrix4d (no undefined `poseToMatrix`):**
```cpp
Eigen::Quaterniond q_goal;
if (!sanitizePoseQuaternion(msg->pose.orientation, q_goal)) {
  ROS_WARN("[EE GOAL] invalid quaternion");
  return;
}
Eigen::Matrix4d T_goal = Eigen::Matrix4d::Identity();
T_goal.block<3,3>(0,0) = q_goal.toRotationMatrix();
T_goal(0,3) = msg->pose.position.x;
T_goal(1,3) = msg->pose.position.y;
T_goal(2,3) = msg->pose.position.z;

pending_ee_goal_ = true;
T_world_ee_pending_ = T_goal;
result = whole_body_ik_->solve(xi_start, T_world_ee_pending_);
if (!result.success) {
  clearPendingEeGoal();
  deleteEeTerminalGhost();
  return;
}
// build candidate_end_pt / candidate_end_yaw from result.xi_best
planner_manager_->resetFailureCount();
bool ok = planNextWaypoint(candidate_end_pt, candidate_end_yaw);
if (!ok) {
  ROS_ERROR("[EE GOAL] terminal found but global trajectory generation failed");
  clearPendingEeGoal();   // pending only — active unchanged
  return;               // WAIT_TARGET, no GEN_NEW_TRAJ
}
// planNextWaypoint success: end_pt_/have_target_ already set inside
active_goal_source_ = GoalSource::EE_POSE;
active_ee_goal_ = true;
T_world_ee_goal_ = T_world_ee_pending_;
pending_ee_goal_ = false;
publishEeTerminalGhost(...);  // REQUIRED — spec §10
// do NOT changeFSMExecState — timer WAIT_TARGET→GEN_NEW_TRAJ
```

**Ghost visualization (required, not optional):**
- Publisher: `ee_ik_terminal_marker_pub_` → topic **`/ee_ik_terminal_markers`**, type `visualization_msgs/MarkerArray`.
- Use **public** API only (`getManiMarkerArray` is private):
```cpp
void REMANIReplanFSM::publishEeTerminalGhost(
    const Eigen::Vector3d &car_state, const Eigen::VectorXd &q) {
  visualization_msgs::MarkerArray marker_array;
  planner_manager_->mm_config_->getMMMarkerArray(
      marker_array,
      "ee_ik_terminal",
      0,
      0.5,
      car_state,
      q,
      gripper_state_);   // actual gripper state — NOT gripper_flag_
  ee_ik_terminal_marker_pub_.publish(marker_array);
}
```
`gripper_state_` = actual current gripper state. `gripper_flag_` = "need gripper operation" — wrong for ghost.
- **Show:** after successful EE commit (above).
- **DELETE:** `deleteEeTerminalGhost()` on — IK failure before commit; accepted replacement 2D goal; new EE goal replacing old; preset if applicable.
- Does not alter existing trajectory mesh publishers.

**2D Nav Goal metadata (two-phase — do NOT clear EE at callback entry):**

In `waypointCallback`, **after existing** end_pt construction, **replace only the final commit hook**:

```cpp
bool ok = planNextWaypoint(end_pt_, end_yaw_);
if (ok) {
  if (active_goal_source_ == GoalSource::EE_POSE) {
    clearActiveEeGoal();  // active metadata + ghost, NOT pending
  }
  active_goal_source_ = GoalSource::NAV_2D;
}
// if !ok: leave active_goal_source_, T_world_ee_goal_, active_ee_goal_ unchanged
```

Same two-phase rule for **preset** acceptance: clear active EE only after successful `planNextWaypoint`.

**`clearEeGoalState()` usage matrix:**

| Event | clearPending | clearActive | clear ghost |
|-------|-------------|-------------|-------------|
| EE IK fail | yes | no | yes |
| EE planNextWaypoint fail | yes | no | no |
| EE execution PASS | no | yes | yes |
| EE execution FAIL | no | yes | yes |
| GEN_NEW_TRAJ abort (EE committed) | no | yes | yes |
| 2D/preset planNextWaypoint success | no | yes | yes |

- [ ] **Step 1:** Implement callback + ghost pub + `!have_target_` gate.
- [ ] **Step 2:** Mock planNextWaypoint fail → no GEN_NEW_TRAJ, active EE untouched if was executing.
- [ ] **Step 3:** 2D plan fail while EE active → EE metadata preserved.
- [ ] **Step 4:** Second `/ee_goal` while `have_target_==true` still in WAIT_TARGET → ignored.

---

#### Task 16: Explicit `mm_config` dependency in plan_manage

**Files:** `plan_manage/CMakeLists.txt`, `plan_manage/package.xml`

**CMake — all three places:**
```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  mm_config
)

catkin_package(
  ...
  CATKIN_DEPENDS ... mm_config ...
)

target_link_libraries(remani_planner_node
  ${catkin_LIBRARIES}
  ...
)
```

Use `${catkin_LIBRARIES}` for `mm_config` — do **not** duplicate bare `mm_config` target if already in catkin deps.

**package.xml:**
```xml
<depend>mm_config</depend>
```

- [ ] **Step 1:** Update both files.
- [ ] **Step 2:** Clean build `remani_planner`.

---

### Phase 9 — Actual EE completion + abort cleanup

**Milestone G**

---

#### Task 17: EE-only EXEC_TRAJ completion branch

**Files:** `remani_replan_fsm.cpp` (`EXEC_TRAJ`, ~`:249`)

**Insert before existing NAV completion block:**

When `t_cur > duration - 1e-2 && touch_the_goal && active_goal_source_==EE_POSE`:

```cpp
T_actual = planner_manager_->mm_config_->getEePose(
    {mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_},
    mm_state_pos_.tail(manipulator_dim_));

Eigen::Matrix<double, 6, 1> e;
poseError(T_actual, T_world_ee_goal_, e);  // T_now, T_des — same API as IK layer
double pos_err = e.head<3>().norm();
double rot_err = e.tail<3>().norm();

have_target_ = false;
have_trigger_ = false;
clearActiveEeGoal();  // active EE metadata + ghost
changeFSMExecState(WAIT_TARGET, "FSM");

if (pos_err <= ee_reach_pos_tol_ && rot_err <= ee_reach_rot_tol_rad_) {
  ROS_INFO("[EE GOAL] reached");
  std_msgs::Bool msg;
  msg.data = true;
  reached_pub_.publish(msg);
} else {
  ROS_WARN("[EE GOAL] execution finished but pose tolerance not met: pos=%.3f rot=%.1f deg",
           pos_err, rot_err * 180.0 / M_PI);
  // V1: do NOT publish planning/finish; no corrective replan
}
goto force_return;
```

**NAV_2D / preset:** keep existing `reach goal` + `reached_pub_` path. On that **success** path only, add GoalSource closure (metadata only — no planning change):

```cpp
// existing NAV reach-goal block (~:249):
have_target_ = false;
have_trigger_ = false;
std::cout << "reach goal\n";
active_goal_source_ = GoalSource::NONE;  // NAV_2D → NONE; EE already cleared if was EE branch
changeFSMExecState(WAIT_TARGET, "FSM");
std_msgs::Bool msg; msg.data = true;
reached_pub_.publish(msg);
```

GoalSource lifecycle:
```text
NONE → NAV_2D → NONE
NONE → EE_POSE → NONE
```

- [ ] **Step 1:** Implement EE branch.
- [ ] **Step 2:** Injected state tests: PASS publishes Bool true; FAIL does not publish.
- [ ] **Step 3:** NAV completion resets `active_goal_source_` to NONE.

---

#### Task 18: EE metadata cleanup on GEN_NEW_TRAJ abort

**Files:** `remani_replan_fsm.cpp` (`GEN_NEW_TRAJ` failure branches ~`:164-174`)

When aborting to `WAIT_TARGET` due to **invalid terminal** or **max continuous failures**, add:

```cpp
if (active_goal_source_ == GoalSource::EE_POSE) {
  clearActiveEeGoal();
}
```

**Do not** change retry thresholds or failure-count logic.

- [ ] **Step 1:** Hook both abort paths.
- [ ] **Step 2:** Test: EE committed → GEN_NEW_TRAJ abort → `active_goal_source_==NONE`.

---

### Phase 10 — Interactive Marker node

**Milestone H**

---

#### Task 19: `ee_goal_marker_node`

**Files:** Create `ee_goal_marker_node.cpp`; modify `CMakeLists.txt`, `package.xml`

**Dependencies:** `interactive_markers`, `geometry_msgs`, `visualization_msgs`, `roscpp` — only what includes require.

**Includes:**
```cpp
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <geometry_msgs/PoseStamped.h>
```

**Interactive marker server name (fixed):** `ee_goal_marker`  
**RViz update topic:** `/ee_goal_marker/update`

**UI policy:** **Do not insert marker until first `/ee_current_pose`.** Until then: spin idle, `ROS_INFO_THROTTLE` "waiting for current EE pose". On first pose: insert 6DOF marker at that pose. MOUSE_UP / menu "Plan" → publish `/ee_goal` (`PoseStamped`, `world`). "Reset" → snap to latest `/ee_current_pose` only.

**Forbidden:** `#include <mm_config/...>`, odom/joint subs, FK, collision.

- [ ] **Step 1:** Implement with server name `ee_goal_marker`.
- [ ] **Step 2:** Task 20 RViz display uses `/ee_goal_marker/update`.

---

### Phase 11 — Launch / RViz / regression

**Milestone I**

---

#### Task 20: YAML parameters

**Files:** `mm_param_ranger_cr10.yaml`, `remani_planner_param.yaml`

Add all `fsm/ee_*` keys per spec §9 verbatim defaults.

- [ ] **Step 1:** Params added.

---

#### Task 21: Launch + RViz + full regression

**Files:** `remani_sim.launch`, `remani_ranger_cr10.rviz`

**Launch:** when `robot_model==ranger_cr10 && use_rviz`:
```xml
<node pkg="remani_planner" type="ee_goal_marker_node" name="ee_goal_marker_node" output="screen"/>
```

**RViz:** InteractiveMarkers display → `/ee_goal_marker/update`; optional Pose display → `/ee_current_pose`.

**Regression checklist:**

| ID | Checkpoint |
|----|------------|
| 9 | 2D Nav Goal unchanged (arm held, time scaling, abort) |
| 10 | Marker during EXEC ignored |
| 11 | EE reach PASS → `planning/finish` true |
| 12 | EE reach FAIL → no finish pub |
| B/C | Stage B/C in sim |
| safety | time scaling, wheel limits, RRT timeout, failure-count unchanged |
| 2D replace | EE executing → 2D click fail → EE continues; 2D success → EE cleared |

```bash
cd remani_planner && catkin_make -DCMAKE_BUILD_TYPE=Release --pkg mm_config remani_planner
rosrun mm_config test_ee_pose_ik
```

FastArmer build; non-CR10 `/ee_goal` rejected.

- [ ] **Step 1:** Launch + RViz updated.
- [ ] **Step 2:** Full checklist logged to `logs/run_YYYYMMDD_*.log`.

---

## Parameter Additions (summary)

| File | Keys |
|------|------|
| `mm_param_ranger_cr10.yaml` | `mm/ee_tcp_xyz_rpy` |
| `remani_planner_param.yaml` | All `fsm/ee_*` per spec §9 + optional numeric keys: `ee_ik_task_rot_weight`, `ee_ik_step_tol`, `ee_ik_stagnation_iters`, `ee_ik_lambda_init/min/max`, `ee_ik_lambda_retry_max` (or internal defaults in `loadFromRosParam`) |

---

## Failure Handling (implementation mapping)

| Case | Action |
|------|--------|
| Not idle (`!WAIT_TARGET` or `have_target_`) | WARN `ignored: planner is not idle`, return |
| Not ready | WARN, return |
| Bad quaternion (FSM) | `sanitizePoseQuaternion` false → WARN, return |
| IK fail | `clearPendingEeGoal()`, delete stale ghost |
| `planNextWaypoint` false (EE) | ERROR, `clearPendingEeGoal()` only; active EE untouched |
| GEN_NEW_TRAJ abort (EE active) | existing abort + `clearActiveEeGoal()` |
| EE tolerance fail | WARN, `clearActiveEeGoal()`, no `planning/finish` |
| 2D planNextWaypoint false | no EE metadata change |

---

## Logging / Visualization

- `[EE IK]` solver; `[EE GOAL]` FSM (spec messages).
- Ghost: **`/ee_ik_terminal_markers`** (required).
- Marker server: **`ee_goal_marker`** → `/ee_goal_marker/update`.
- RViz: `/ee_current_pose` optional display.

---

## Test Execution Order (Milestones)

```text
A: Tasks 1–4   FK/Jacobian (no GridMap)
B: Task 5      fixed-base IK (no GridMap)
C: Task 6–7    scaffold + Stage B (GridMap fixture)
D: Tasks 8–9   candidate + clearance (GridMap fixture)
E: Tasks 10–12 Stage C + orchestrator (GridMap fixture)
F: Tasks 13–16 FSM + CMake
G: Tasks 17–18 EE completion + abort cleanup
H: Task 19     Marker
I: Tasks 20–21 launch + regression
```

**Stop rule:** Milestone failure → do not stack next layer.

**Ground-truth policy:** Stage B/C/solver tests generate `T_goal = getEePose(xi_goal)` from known legal `xi_goal` rather than arbitrary poses.

---

## Risks

| Risk | Mitigation |
|------|------------|
| Stage C 145 bases × arm IK exceeds 150 ms | Cheap base sort; **`ee_ik_c_max_ms` only** wall-clock cutoff (no early-exit heuristic) |
| Numeric IK singularities | LM trial reject + λ increase; LDLT |
| `plan_manage` missing `mm_config` | Task 16 explicit find_package + catkin_package + package.xml |
| Marker before joint_state | Defer insert until `/ee_current_pose` |
| Clearance duplication | Reuse collision sample point loops; separate min-only aggregation |
| Non-const MMConfig EE APIs | V1 option A — consistent non-const |

---

## Explicit Non-Goals (V1)

Continuous EE trajectory; scan/multi-pose; MoveIt/TRAC-IK/IKFast; AsyncSpinner/worker; TopAY; optimizer EE constraints; corrective replan; hardware bridge; Stage C cost early exit; hidden candidate cap.

---

## Final Verification Checklist

- [ ] `test_ee_pose_ik` all cases pass
- [ ] `test_ee_pose_ik` self-initializes CR10 test params via `ensureCr10TestParams`
- [ ] Weighted DLS: Stage B `(A M A^T + λ²I)y=b`, `Δξ=M A^T y`; Arm `(A_arm M_arm A_arm^T + λ²I)y=b`, `Δq=M_arm A_arm^T y`
- [ ] FixedBaseArmIk `W_arm` is 6×6 (`joint_weight * I_6`)
- [ ] FixedBaseArmIk gets explicit `FixedBaseArmIkParams` via `makeArmIkParams()`
- [ ] WholeBodyIkParams validation rejects zero/non-finite weights
- [ ] Jacobian angular column: `Omega = Rdot * R_ee^T` (once)
- [ ] FixedBaseArmIkParams/Result defined in fixed_base_arm_ik.hpp
- [ ] Ghost uses `getMMMarkerArray` + `gripper_state_`
- [ ] EE completion uses `poseError(T_now, T_des, e)` — no `poseErrorMetrics`
- [ ] No undefined `poseToMatrix` helper
- [ ] ConstPtr quaternion is copied/normalized, not mutated
- [ ] Success only on pose gates + hard validity
- [ ] `/ee_goal` gate is `WAIT_TARGET && !have_target_ && state ready`
- [ ] Two-phase: IK ok + planNextWaypoint fail → no GEN_NEW_TRAJ
- [ ] 2D plan fail preserves active EE metadata
- [ ] NAV_2D normal completion returns `GoalSource` to `NONE`
- [ ] EE FAIL does not publish `planning/finish`
- [ ] GEN_NEW_TRAJ abort clears EE metadata
- [ ] Ghost on `/ee_ik_terminal_markers` after EE commit
- [ ] Stage C for-loop cannot exceed total stage deadline
- [ ] 2D Nav regression pass
- [ ] non-CR10 rejects `/ee_goal`

---

## Self-Review (plan vs spec)

| Spec requirement | Task |
|------------------|------|
| 9-DoF IK, 6×9 J + TCP | 2–3 |
| Weighted Stage B | 6–7 |
| Stage C 145 + wall clock only | 10–11 |
| Clearance (base+pedestal+links) | 9 |
| WholeBodyIkSolver | 12 |
| WholeBodyIkParams + shared grid/mm | 6, 13 |
| have_joint_state_ | 13 |
| Two-phase after planNextWaypoint | 15 |
| 2D two-phase metadata | 15 |
| EE completion PASS/FAIL pub | 17 |
| EE abort cleanup | 18 |
| Ghost terminal viz | 15 |
| Marker no FK | 19 |
| Regression | 21 |

No placeholder TBD. API names consistent. File creation order: Task 6 before Task 7. CMake: Task 4=`mm_config.cpp` only → Task 5+=`fixed_base_arm_ik.cpp` → Task 6+=`whole_body_ik.cpp`.

---

**Plan complete.** Execution-ready for Milestone A / Task 1. Execution choice offered after user review (subagent-driven vs inline).
