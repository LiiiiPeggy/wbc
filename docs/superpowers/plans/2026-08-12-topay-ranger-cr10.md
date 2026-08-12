# TopAY Ranger+CR10 Phase A Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make TopAY default to Ranger geometry + CR10 (6-DOF) + AG95 fixed-closed envelope under **TopAY virtual-differential** base semantics, while keeping `robot:=tracer7` as a full regression path.

**Architecture:** Config-driven dual kinematics backends in a real `fake_moma` shared library (`topay_alt` | `cr10`); one finalized `shared_ptr<const MomaParam>` injected into **all** planner instances (including 8 parallel traj/RRT workers); sim/vis load the same global `/moma/*` profile and pose meshes only via typed `getLinkTransforms()`. CR10 mount is \(T^{base}_{mount}=T(0.2462,0,0.1)\) and **must not** stack on `chassis_height`.

**Tech Stack:** ROS1 Noetic, catkin, Eigen, TopAY `fake_moma` / `planner` / `map`, REMANI CR10 FK oracle (`mm_config` / `test_cr10_fk.cpp`).

**Spec:** `docs/superpowers/specs/2026-08-11-topay-ranger-cr10-design.md` (revised 2026-08-12).

## Global Constraints

- Phase A only: no EE 6D goal interface; no AGX hardware; no real Ranger 4WS / Dual-Ackermann.
- Always say **Ranger geometry + TopAY virtual differential**, never “full Ranger model”.
- Robot profile ROS namespace is **global** `/moma/*` (not `~/moma/*`).
- CR10 FK must not use `now_p.z = chassis_height; now_p += relative_t` with mount z=0.1.
- `getFKPose` / `getEEGrads` / `getColliPts` / `getColliGrads` for `cr10` must all use the **same** CR10 transform chain (no mixed alternating-axis grads).
- FK / EE-grad / collision-grad gates: relative error &lt; 1e-4 (and FK pos &lt; 1e-3 m, rot &lt; 0.1 deg) over ≥100 random q.
- Smoke must **plan + MPC-execute + converge** with quantitative tolerances (`local_mode: true` + explicit `/move_base_simple/goal`).
- Legacy Python `mpc_demo.py` / `moma_param.py` are **Phase A Non-Goals** (must not be launched by Ranger smoke).
- New/changed code blocks: put a single language-appropriate comment banner **before** the block only, e.g. `// ################################` + `// C++: description` + `// ################################`. Do **not** add matching `end` banners.
- Every task commit must `git add` **all** new/modified files explicitly (`git commit -am` is forbidden for tasks that create files).

### Phase boundary (unchanged)

```text
Phase A  Ranger geometry + CR10 6DOF + AG95 fixed envelope + TopAY virtual-diff  ← this plan
Phase B  EE 6D pose → whole-body terminal state
Phase C  real Ranger motion model + hardware
```

---

## File Structure

| Path | Role |
|------|------|
| `TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h` | Typed kinematics/collision APIs, layered finalize |
| `TopAY/src/simulator/fake_moma/src/moma_param_load.cpp` (new) | `/moma/*` load + layered finalize |
| `TopAY/src/simulator/fake_moma/src/moma_param_cr10.cpp` (new) | CR10 FK, EE grads, colli pts/grads, sphere sampling |
| `TopAY/src/simulator/fake_moma/src/moma_param_topay_alt.cpp` (new, optional split) | Existing Tracer FK/grads moved out of header if needed |
| `TopAY/src/simulator/fake_moma/src/test_topay_cr10_fk.cpp` (new) | FK + EE-grad + (Task4) colli-grad gates |
| `TopAY/src/simulator/fake_moma/CMakeLists.txt` | Real `libfake_moma` + node executables |
| `TopAY/src/planner/params/robot_tracer7.yaml` (new) | Tracer profile under top-level `moma:` |
| `TopAY/src/planner/params/robot_ranger_cr10.yaml` (new) | Ranger/CR10/AG95 profile |
| `TopAY/src/planner/params/agent_ranger_cr10_smoke.yaml` (new) | `local_mode: true` smoke agent overlay |
| `TopAY/src/planner/launch/run_all.launch` | `robot` arg; load `/moma` via global rosparam |
| `TopAY/src/planner/launch/run_ranger_cr10_smoke.launch` (new) | Deterministic execute-to-goal gate |
| `TopAY/src/map/.../grid_map.*` | `setMomaParam`; dual-radius whole-body checks |
| `TopAY/src/planner/...` | Shared profile injection for all instances; state-dim audit |
| `TopAY/src/simulator/fake_moma/src/{moma_sim,moma_vis}.cpp` | Root `NodeHandle` for profile; mesh from typed transforms |
| `TopAY/src/simulator/fake_moma/meshes/ranger_cr10/` (new) | Vendored meshes |

Do **not** fork a second fake_moma package.

---

### Task 1: Global `/moma/*` contract + tracer7 YAML + loader + real `fake_moma` library

**Files:**
- Create: `TopAY/src/planner/params/robot_tracer7.yaml`
- Create: `TopAY/src/simulator/fake_moma/src/moma_param_load.cpp`
- Modify: `TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h`
- Modify: `TopAY/src/simulator/fake_moma/CMakeLists.txt`
- Modify: `TopAY/src/planner/launch/run_all.launch` (load robot yaml at **global** scope)
- Modify if needed: `TopAY/src/planner/CMakeLists.txt`, `TopAY/src/map/CMakeLists.txt` (already depend on `fake_moma` / link `${catkin_LIBRARIES}` — verify they resolve `libfake_moma` without duplicate `moma_sim` symbols)

**Interfaces:**
- Consumes: global params `/moma/*`
- Produces:
  - `enum class KinematicsType { TopayAlt, Cr10 };`
  - Typed result structs (see below)
  - `static MomaParam fromRos(const ros::NodeHandle& root_nh);` — **must** be called with **root** handle (`ros::NodeHandle root_nh;`), never `~`
  - Layered finalize:
    - `finalizeKinematics()`
    - `finalizeCollision()` (may no-op / soft-skip until Task 4)
    - `finalizeVisualization()` (may no-op until Task 6)
    - `validateCore()` / `validateKinematics()` / `validateCollision()` / `validateVisualization()` / `validateAll()`

**ROS namespace contract (P0):**

```text
robot profile (shared):     /moma/*
planner-private params:     /planner_node/agent/*, /planner_node/... (via ~)
fake_moma private:          /fake_moma_node/sim/*, etc. (via ~)
```

Launch must load robot YAML **without** putting it under a node ns:

```xml
<arg name="robot" default="tracer7"/>  <!-- Task1 regression default; Task7 flips to ranger_cr10 -->
<rosparam command="load" file="$(find planner)/params/robot_$(arg robot).yaml"/>
<!-- file top-level key is `moma:` → params appear as /moma/... -->
```

Node pattern:

```cpp
ros::NodeHandle pnh("~");
ros::NodeHandle root_nh;
MomaParam profile = MomaParam::fromRos(root_nh); // reads /moma/*
```

Planner may become `init(pnh, root_nh)` or equivalent.

**Typed kinematics result (P1, define now):**

```cpp
struct KinematicResult {
  Eigen::Matrix4d base_T = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d arm_base_T = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Matrix4d> arm_link_T; // size == dof_num
  Eigen::Matrix4d ee_T = Eigen::Matrix4d::Identity(); // CR10 tool/flange (NOT AG95 mesh frame)
};
```

**CMake library split (P0):**

```cmake
# ################################
# CMake: real shared MomaParam library
# ################################
add_library(fake_moma
  src/moma_param_load.cpp
  # Task3+: src/moma_param_cr10.cpp
)
add_dependencies(fake_moma fake_moma_gencpp)
target_link_libraries(fake_moma ${catkin_LIBRARIES})

add_executable(fake_moma_node src/moma_sim.cpp)
target_link_libraries(fake_moma_node fake_moma ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(moma_vis_node src/moma_vis.cpp)
target_link_libraries(moma_vis_node fake_moma ${catkin_LIBRARIES} ${PCL_LIBRARIES})
```

**Forbidden:** `add_library(fake_moma src/moma_sim.cpp)` (node/main must not be the library).

Keep default `MomaParam()` ctor behavior identical to today’s Tracer numbers for interim compile, **or** ensure all call sites use `fromRos` before use by end of Task 1 smoke.

- [ ] **Step 1: Write `robot_tracer7.yaml`** with exact current ctor values under top-level `moma:`.

- [ ] **Step 2: Add header APIs + layered finalize stubs; implement `fromRos`/`validateCore`/`finalizeKinematics` for `topay_alt` in `moma_param_load.cpp`.**

Comment style before each new block:

```cpp
// ################################
// C++: MomaParam::fromRos reads global /moma/*
// ################################
```

- [ ] **Step 3: Restructure CMake as above; ensure `catkin_package(LIBRARIES fake_moma ...)` and planner/map still link.**

- [ ] **Step 4: Point `run_all.launch` at global robot yaml; temporarily make `fake_moma_node` / planner load via `root_nh`.**

Smoke:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch planner run_all.launch robot:=tracer7 rviz:=false
```

Expected: no FATAL; `/moma/dof_num` exists; startup logs from planner and fake_moma print matching `robot_name/dof_num/kinematics/mount`.

- [ ] **Step 5: Commit (explicit add)**

```bash
git add \
  TopAY/src/planner/params/robot_tracer7.yaml \
  TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h \
  TopAY/src/simulator/fake_moma/src/moma_param_load.cpp \
  TopAY/src/simulator/fake_moma/CMakeLists.txt \
  TopAY/src/planner/launch/run_all.launch \
  TopAY/src/planner/CMakeLists.txt \
  TopAY/src/map/CMakeLists.txt \
  TopAY/src/simulator/fake_moma/src/moma_sim.cpp \
  TopAY/src/planner/include/planner/planner.h \
  TopAY/src/planner/src/planner.cpp \
  TopAY/src/planner/src/planner_node.cpp
git commit -m "$(cat <<'EOF'
Add global /moma profile loader and restructure fake_moma library.

EOF
)"
```

(Only add files actually touched.)

---

### Task 2: Repo-wide whole-body state / DOF / marker audit + tracer7 regression

**Files:**
- Modify: all justified hits under `TopAY/src` from the audit grep below
- Especially: `planner.cpp` (`Zero(10)`, `safeCallback` temp state, `boundary_vel`/`boundary_acc`), `moma_traj_opt*.cpp/.h`, `ompc.cpp`, `moma_sim.cpp`, `moma_vis.cpp`

**Interfaces:**
- Consumes: `moma_param.dof_num`
- Produces: whole-body width always `state_dim = 3 + dof_num`; role-based marker indices

**Audit grep (minimum):**

```bash
rg -n \
  'tail\(7\)|markers\[8\]|Zero\(\s*10|resize\(\s*10|VectorXd.*\b10\b|MatrixXd.*\b10\b|3\s*\+\s*7|q1-q7|link1-7' \
  TopAY/src --glob '*.{cpp,h,hpp}'
```

**Do not** blindly replace every `7`/`10`. Manually classify each hit as whole-body state dim, arm dof, marker index, or unrelated constant.

**Required pattern:**

```cpp
// ################################
// C++: whole-body state dimension from dof_num
// ################################
const int state_dim = 3 + static_cast<int>(moma_param.dof_num);
Eigen::MatrixXd boundary_vel = Eigen::MatrixXd::Zero(state_dim, 2);
Eigen::MatrixXd boundary_acc = Eigen::MatrixXd::Zero(state_dim, 2);
```

Also fix:

```cpp
// Planner::safeCallback
Eigen::VectorXd temp_state = Eigen::VectorXd::Zero(state_dim);
```

and `.tail(7)` → `.tail(moma_param.dof_num)`.

Marker role indices:

```cpp
const size_t kBase = 0;
const size_t kArmBase = 1;
const size_t kJoint0 = 2;
const size_t kGripper = 2 + moma_param.dof_num;
```

- [ ] **Step 1: Run audit grep; produce a short checklist of hits to change vs leave.**

- [ ] **Step 2: Apply justified replacements; build.**

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
rg -n 'tail\(7\)|markers\[8\]|Zero\(\s*10' TopAY/src --glob '*.{cpp,h,hpp}'
# expected: no remaining whole-body hardcodes (comments OK)
```

- [ ] **Step 3: Tracer7 regression smoke**

```bash
roslaunch planner run_all.launch robot:=tracer7 rviz:=false
```

- [ ] **Step 4: Commit**

```bash
git add -u TopAY/src
# if any new helper headers were added, git add them explicitly too
git commit -m "$(cat <<'EOF'
Audit TopAY whole-body state dimensions to follow dof_num.

EOF
)"
```

---

### Task 3: CR10 exact FK + `getLinkTransforms` + `getFKPose` + `getEEGrads` (+ EE FD gate)

**Files:**
- Create: `TopAY/src/simulator/fake_moma/src/moma_param_cr10.cpp`
- Create: `TopAY/src/simulator/fake_moma/src/test_topay_cr10_fk.cpp`
- Create: `TopAY/src/planner/params/robot_ranger_cr10.yaml` (kinematics/mount/limits; collision section may be incomplete stubs)
- Modify: `moma_param.h`, `moma_param_load.cpp`, `fake_moma/CMakeLists.txt`

**Interfaces:**
- Consumes: `/moma/manipulator_config`, `/moma/base_mani_fixed_joint_xyz_ypr` (or `T_base_mount`), dof=6
- Produces:
  - `KinematicResult getLinkTransforms(const Eigen::VectorXd& state) const;`
  - `Eigen::VectorXd getFKPose(const Eigen::VectorXd& state);` dispatch
  - `Eigen::VectorXd getEEGrads(const Eigen::VectorXd& state, const Eigen::VectorXd& ee_grad);` dispatch
- **Task 3 gate does NOT require** non-empty collision spheres / complete meshes (`validateCollision`/`validateVisualization` not enforced yet).

**Mount rule (P0):**

```cpp
// T_W_base from (x,y,yaw) only — chassis_height does NOT enter arm FK
T_W_mount = T_W_base * T_base_mount; // z = 0.1
```

CR10 fixed frames (REMANI `test_cr10_fk.cpp`):

```cpp
F[0]=makeFixed(0,0,0.1765,0,0,0);
F[1]=makeFixed(0,0,0,1.5708,1.5708,0);
F[2]=makeFixed(-0.607,0,0,0,0,0);
F[3]=makeFixed(-0.568,0,0.191,0,0,-1.5708);
F[4]=makeFixed(0,-0.125,0,1.5708,0,0);
F[5]=makeFixed(0,0.1084,0,-1.5708,0,0);
// T *= F[i] * Rz(q[i])
```

**Forbidden:** CR10 path calling TopAY alternating-axis internals for FK/EE grads.

- [ ] **Step 1: Add `robot_ranger_cr10.yaml` kinematics core + CMake `moma_param_cr10.cpp` + failing `test_topay_cr10_fk`.**

Test must cover:

1. Mount equals `(0.2462,0,0.1)` (pos err &lt; 1e-6)
2. ≥100 random q: FK vs analytic `F[i]*Rz` — pos &lt; 1e-3 m, rot &lt; 0.1 deg
3. **EE gradient FD gate:** random `q`, random EE-space `g`, `f(q)=gᵀ FK(q)`; analytic `getEEGrads` vs central FD; relative error &lt; 1e-4

```cmake
add_executable(test_topay_cr10_fk src/test_topay_cr10_fk.cpp)
target_link_libraries(test_topay_cr10_fk fake_moma ${catkin_LIBRARIES})
```

- [ ] **Step 2: Run — expect FAIL**

```bash
catkin_make --pkg fake_moma
rosrun fake_moma test_topay_cr10_fk
```

- [ ] **Step 3: Implement CR10 transforms + EE analytic grads; `topay_alt` keeps prior math.**

- [ ] **Step 4: Run — expect PASS** (kinematics / EE-grad only; collision section of test skipped or not yet compiled).

- [ ] **Step 5: Commit**

```bash
git add \
  TopAY/src/simulator/fake_moma/src/moma_param_cr10.cpp \
  TopAY/src/simulator/fake_moma/src/test_topay_cr10_fk.cpp \
  TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h \
  TopAY/src/simulator/fake_moma/src/moma_param_load.cpp \
  TopAY/src/simulator/fake_moma/CMakeLists.txt \
  TopAY/src/planner/params/robot_ranger_cr10.yaml
git commit -m "$(cat <<'EOF'
Add TopAY CR10 FK/EE analytic gradients with finite-difference gates.

EOF
)"
```

---

### Task 4: CR10 variable spheres + dual radii + ignore topology + `getColliPts`/`getColliGrads` (+ colli FD gate)

**Files:**
- Modify: `moma_param.h`, `moma_param_cr10.cpp`, `moma_param_load.cpp`, `robot_ranger_cr10.yaml`
- Modify: `TopAY/src/map/include/map/grid_map.h`, `TopAY/src/map/src/grid_map.cpp`
- Modify: `test_topay_cr10_fk.cpp` (add collision gradient section) **or** add `test_topay_cr10_colli_grad.cpp`

**Interfaces:**
- Produces:
  - `struct CollisionSphere { Eigen::Vector3d local_offset; double obstacle_radius; double self_radius; int link_id; };`
  - `std::vector<Eigen::Vector4d> getColliPts(state)` — `w` = **obstacle_radius**; centers from CR10 `getLinkTransforms`
  - `Eigen::VectorXd getColliGrads(const Eigen::VectorXd& state, const std::vector<Eigen::Vector3d>& pos_grads);`
    - `topay_alt` → existing implementation
    - `cr10` → analytic grads on **same** CR10 chain (P0 — required)
  - `void GridMap::setMomaParam(const std::shared_ptr<const MomaParam>&);`
- Enables `finalizeCollision()` + `validateCollision()`.

**Forbidden:** `getColliPts` correct on CR10 while `getColliGrads` still uses alternating-axis FK.

**YAML sampling (example):**

```yaml
moma:
  obstacle_thickness: 0.10
  self_thickness: 0.045
  sphere_spacing_factor: 1.0  # N_i = ceil(L_i / (factor * r_obs)); long 0.607/0.568 => N>2
  chassis_ignore_link_ids: [0, 1]  # arm_base / early links near mount — policy tunable
  ag95_spheres:
    - {xyz: [0.0, 0.0, 0.06], obstacle_radius: 0.05, self_radius: 0.04}
    - {xyz: [0.04, 0.03, 0.10], obstacle_radius: 0.03, self_radius: 0.025}
    - {xyz: [0.04, -0.03, 0.10], obstacle_radius: 0.03, self_radius: 0.025}
```

**Ignore topology (link-based, not zero-pose distance guessing):**

```text
same link sphere pairs          → ignore
adjacent arm links              → ignore
link6 ↔ AG95                    → ignore
non-adjacent links              → check
arm↔chassis: use chassis_ignore_link_ids / explicit pair table
```

**Collision gradient FD gate (≥100 random q):**

```text
pick sphere index + random 3D g
f(q) = gᵀ p_sphere(q)
analytic getColliGrads vs central FD
relative error < 1e-4
```

Also assert mid-span of long links has a sphere within ~5 cm (coverage).

- [ ] **Step 1: Extend YAML + `buildCollisionProxies` / ignore matrix; implement CR10 `getColliPts`/`getColliGrads`.**

- [ ] **Step 2: Wire GridMap dual-radius env vs self checks; `setMomaParam`.**

- [ ] **Step 3: Expand test binary; run — expect PASS including colli-grad FD.**

```bash
rosrun fake_moma test_topay_cr10_fk
```

- [ ] **Step 4: Collision regression poses**

```text
home/open pose → no false positive self-collision
folded unsafe pose → collision
```

- [ ] **Step 5: Commit**

```bash
git add \
  TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h \
  TopAY/src/simulator/fake_moma/src/moma_param_cr10.cpp \
  TopAY/src/simulator/fake_moma/src/moma_param_load.cpp \
  TopAY/src/simulator/fake_moma/src/test_topay_cr10_fk.cpp \
  TopAY/src/planner/params/robot_ranger_cr10.yaml \
  TopAY/src/map/include/map/grid_map.h \
  TopAY/src/map/src/grid_map.cpp
git commit -m "$(cat <<'EOF'
Add CR10 dual-radius collision spheres and analytic getColliGrads.

EOF
)"
```

---

### Task 5: Inject one shared finalized profile into ALL planner instances

**Files:**
- Modify: `planner.h` / `planner.cpp`
- Modify: `birrts`, `mcrrts`, `ompls`, `mpc`/`ompc`, `moma_traj_opt` (+ any GraphSearch ctor using chassis radius)
- Modify: GridMap (already `setMomaParam`)

**Interfaces:**
- Produces: single `std::shared_ptr<const MomaParam> moma_param_shared_` from `MomaParam::fromRos(root_nh)`
- Injects into:
  - `GridMap`
  - `BiRRTs`
  - `MCRRTs` (primary)
  - `OMPLPlanner`
  - `MomaTrajOpt` (primary)
  - `OMPC` / MPC
  - **all** `traj_opters[0..7]`
  - **all** `mc_rrtsers[0..7]`
- Forbids: any child `init()` constructing a default Tracer `MomaParam` and ignoring shared profile

```cpp
// ################################
// C++: inject shared MomaParam into parallel workers
// ################################
moma_param_shared_ = std::make_shared<const MomaParam>(MomaParam::fromRos(root_nh));
grid_map->setMomaParam(moma_param_shared_);
birrts->setMomaParam(moma_param_shared_);
mcrrts->setMomaParam(moma_param_shared_);
ompl_planner->setMomaParam(moma_param_shared_);
traj_opter->setMomaParam(moma_param_shared_);
mpc->setMomaParam(moma_param_shared_);
for (int i = 0; i < 8; ++i) {
  traj_opters[i]->setMomaParam(moma_param_shared_);
  traj_opters[i]->init(pnh);
  mc_rrtsers[i]->setMomaParam(moma_param_shared_);
  mc_rrtsers[i]->init(pnh);
}
ROS_ASSERT(static_cast<int>(moma_param_shared_->dof_num) + 3 == /* expected state width in use */);
```

Startup log (planner): `robot_name`, `dof_num`, `kinematics`, mount xyz — must match fake_moma/moma_vis later.

- [ ] **Step 1: Add `setMomaParam` to every submodule; remove per-module silent default profile use.**

- [ ] **Step 2: Wire planner injection including parallel arrays; build.**

- [ ] **Step 3: Tracer7 regression with shared injection**

```bash
roslaunch planner run_all.launch robot:=tracer7 rviz:=false
```

- [ ] **Step 4: Commit**

```bash
git add -u TopAY/src/planner TopAY/src/map
git commit -m "$(cat <<'EOF'
Share one finalized MomaParam across all TopAY planner workers.

EOF
)"
```

---

### Task 6: Unified sim/vis transforms + mesh visual offsets + AG95 + vendored meshes

**Files:**
- Modify: `moma_sim.cpp`, `moma_vis.cpp`
- Create: `TopAY/src/simulator/fake_moma/meshes/ranger_cr10/**` (from `agx/rangerboxcr10lidar_description/meshes/`)
- Extend: `robot_*.yaml` `mesh_parts` (link + file + xyz/rpy visual offset)
- Enable: `finalizeVisualization()` / `validateVisualization()`

**Interfaces:**
- Consumes: `getLinkTransforms(state)`
- AG95: `T_W_ag95 = ee_T * T_ee_ag95` (ee is flange/tool; **not** gripper mesh frame by default)
- Mesh: `T_W_mesh = T_W_link * T_link_visual` from YAML offsets (preserve Tracer visual offsets for regression)

```yaml
# ################################
# YAML: mesh_parts with visual offsets
# ################################
moma:
  mesh_parts:
    - role: base
      file: "package://fake_moma/meshes/ranger_cr10/...."
      xyz: [0,0,0]
      rpy: [0,0,0]
    - role: arm_link
      index: 0
      file: "..."
      xyz: [0,0,0]
      rpy: [0,0,0]
    - role: ag95
      file: "..."
      xyz: [...]
      rpy: [...]
```

Both nodes:

```cpp
ros::NodeHandle pnh("~");
ros::NodeHandle root_nh;
auto profile = MomaParam::fromRos(root_nh);
```

Log the same four fields as planner. **Must match.**

Delete CR10-path private alternating-axis FK loops.

- [ ] **Step 1: Vendor meshes; extend YAML mesh_parts.**

- [ ] **Step 2: Rewrite sim/vis pose updates from typed `KinematicResult`.**

- [ ] **Step 3: Visual smoke**

```bash
roslaunch planner run_all.launch robot:=ranger_cr10 rviz:=true
# arm base world Z ~ 0.1 m (not ~0.25 m); AG95 attached at flange*T_ee_ag95
```

Also re-check `robot:=tracer7` mesh alignment regression.

- [ ] **Step 4: Commit**

```bash
git add \
  TopAY/src/simulator/fake_moma/src/moma_sim.cpp \
  TopAY/src/simulator/fake_moma/src/moma_vis.cpp \
  TopAY/src/simulator/fake_moma/meshes/ranger_cr10 \
  TopAY/src/planner/params/robot_tracer7.yaml \
  TopAY/src/planner/params/robot_ranger_cr10.yaml \
  TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h \
  TopAY/src/simulator/fake_moma/src/moma_param_load.cpp
git commit -m "$(cat <<'EOF'
Drive TopAY sim/vis from typed link transforms with mesh offsets.

EOF
)"
```

---

### Task 7: Launch switch + deterministic smoke (plan + MPC execute + reach goal)

**Files:**
- Modify: `run_all.launch` — default `robot:=ranger_cr10`
- Create: `TopAY/src/planner/params/agent_ranger_cr10_smoke.yaml`
- Create: `TopAY/src/planner/params/map_ranger_cr10_smoke.yaml` (wide open free space)
- Create: `TopAY/src/planner/launch/run_ranger_cr10_smoke.launch`
- Optional: short TopAY README note

**Phase A Non-Goal (explicit):**

```text
Legacy Python mpc_demo.py / moma_param.py remain 7DOF/10-state and are unused by Ranger launches.
run_ranger_cr10_smoke.launch MUST NOT start those demos.
```

**Smoke agent overlay (P0 — local_mode required for cmd/replan/safe threads):**

```yaml
# ################################
# YAML: ranger_cr10 smoke agent — execute with MPC
# ################################
planner_node:
  agent:
    mode: "planner"
    planner: "moma"
    local_mode: true
    random_ee: true
    fixed_sequence: true
    fixed_startgoal: false
    # keep scene/map pointing at open smoke map
```

**Goal trigger (verified against `planner.cpp`):**

- Subscriber: `/move_base_simple/goal` (`geometry_msgs/PoseStamped`)
- If `header.frame_id == "target"` → waypoint list path (not used for Phase A smoke)
- Else → base SE(2) from pose + random feasible arm terminal (`random_ee: true`)

Use `frame_id: world` (any non-`target` frame works; document `world`):

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

**Quantitative reach-goal acceptance (all required):**

1. Planner prints successful optimization / accepts trajectory  
2. MPC receives trajectory (`local_mode` path)  
3. `/moma_cmd` publishes continuously during tracking  
4. `fake_moma` state evolves on `/moma_odom` (or configured state topic)  
5. Final convergence (tunable but must be numeric), suggested initial gates:
   - base position error &lt; 0.10 m  
   - base yaw error &lt; 5 deg  
   - arm joint max |error| &lt; 0.05 rad vs planned terminal arm q  

**PLAN SUCCESS alone does not pass.**

- [ ] **Step 1: Write smoke launch/map/agent; ensure Python demos are absent; default `run_all` robot=`ranger_cr10`.**

- [ ] **Step 2: Run smoke procedure**

```bash
roslaunch planner run_ranger_cr10_smoke.launch rviz:=true
# then publish /move_base_simple/goal as above
# verify cmd stream + convergence tolerances
```

- [ ] **Step 3: Confirm profile logs identical across planner / fake_moma / moma_vis.**

- [ ] **Step 4: Commit**

```bash
git add \
  TopAY/src/planner/launch/run_all.launch \
  TopAY/src/planner/launch/run_ranger_cr10_smoke.launch \
  TopAY/src/planner/params/agent_ranger_cr10_smoke.yaml \
  TopAY/src/planner/params/map_ranger_cr10_smoke.yaml \
  TopAY/README.md
git commit -m "$(cat <<'EOF'
Add Ranger+CR10 smoke launch with MPC execute-to-goal acceptance.

EOF
)"
```

---

### Task 8: Full tracer regression + collision regression + richer Ranger scenes

**Files:**
- Mostly test/docs; fix only if regressions found
- Optional: note known narrow-map limitations of conservative chassis disk in README

**Gates:**

1. `robot:=tracer7` full `run_all` behavioral regression (vis + plan + optional local track)  
2. Re-run `rosrun fake_moma test_topay_cr10_fk` (FK + EE-grad + colli-grad)  
3. Collision regression: open/home pass; folded fail; mid-link obstacle reject; no wrist false positive  
4. Attempt richer/random Ranger scenes — **non-blocking** if conservative footprint fails corridors; document outcomes  

- [ ] **Step 1: Execute checklist; file any blocking bugs as follow-ups only if Phase A gates fail.**

- [ ] **Step 2: Commit doc/notes if any**

```bash
git add TopAY/README.md  # if updated
git commit -m "$(cat <<'EOF'
Record TopAY Phase A regression results for tracer7 and ranger_cr10.

EOF
)"
```

---

## Spec / Review Coverage Self-Check

| Requirement | Task |
|-------------|------|
| Global `/moma/*` namespace contract | 1, 6, 7 |
| Real `libfake_moma` (not `moma_sim.cpp` as lib) | 1 |
| Layered finalize (no Task3↔4 deadlock) | 1, 3, 4, 6 |
| Repo-wide state-dim audit incl. `Zero(10)` | 2 |
| CR10 mount not stacked on chassis_height | 3 |
| Typed `KinematicResult` frames | 1, 3, 6 |
| `getEEGrads` + EE FD gate | 3 |
| `getColliPts`/`getColliGrads` same CR10 chain + colli FD | 4 |
| Dual radii + variable spheres + link ignore topology | 4 |
| Shared profile → all 8 parallel workers | 5 |
| sim/vis unified FK + mesh offsets + AG95 | 6 |
| Smoke: `local_mode` + goal pub + MPC + quantitative reach | 7 |
| Python demos Non-Goal | 7 |
| Explicit `git add` (no `-am` for new files) | all |
| Comment banners before blocks only (no `end`) | all |
| Phase B/C out of scope | global |

## Type Consistency

- `KinematicsType`, `KinematicResult{base_T, arm_base_T, arm_link_T, ee_T}`, `CollisionSphere`, `MomaParam::fromRos(root_nh)`, `getLinkTransforms`, `getColliGrads(state, pos_grads)`, `setMomaParam(shared_ptr<const MomaParam>)`.
- Whole-body state width always `3 + dof_num`.
- Profile identity: planner / fake_moma / moma_vis logs must match on `robot_name`, `dof_num`, `kinematics`, mount xyz.
