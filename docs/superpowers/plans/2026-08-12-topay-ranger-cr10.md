# TopAY Ranger+CR10 Phase A Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make TopAY default to Ranger geometry + CR10 (6-DOF) + AG95 closed envelope under virtual-differential base semantics, while keeping `robot:=tracer7` as a full regression path.

**Architecture:** Config-driven dual kinematics backends inside `MomaParam` (`topay_alt` | `cr10`), with a single finalized `shared_ptr` injected across the planner process; sim/vis load the same YAML and pose meshes only via `getLinkTransforms()`. CR10 mount is \(T^{base}_{mount}=T(0.2462,0,0.1)\) and must not stack on `chassis_height`.

**Tech Stack:** ROS1 Noetic, catkin, Eigen, TopAY `fake_moma`/`planner`/`map`, REMANI CR10 FK oracle (`mm_config` / `test_cr10_fk.cpp`).

**Spec:** `docs/superpowers/specs/2026-08-11-topay-ranger-cr10-design.md` (revised 2026-08-12).

## Global Constraints

- Phase A only: no EE 6D goal interface; no AGX hardware; no real Ranger 4WS.
- Always describe motion as **Ranger geometry + TopAY virtual differential**, not “full Ranger model”.
- CR10 FK must not use `now_p.z = chassis_height; now_p += relative_t` with mount z=0.1.
- All planner submodules must share one finalized `MomaParam` profile; sim/vis must load the identical `robot_*.yaml`.
- Obstacle vs self collision radii are separate; CR10 long links use variable-count spheres.
- FK acceptance: pos &lt; 1e-3 m, rot &lt; 0.1 deg, grad rel &lt; 1e-4 over ≥100 random q.
- Smoke plan must **SUCCESS** (timeout alone is failure).
- Code edits: wrap new/changed blocks with language-appropriate comments and `################################` delimiters per repo user rule.
- Prefer frequent small commits after each task gate.

---

## File Structure

| Path | Role |
|------|------|
| `TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h` | `KinematicResult`, `CollisionSphere`, `fromRos`, backends, `getLinkTransforms` |
| `TopAY/src/simulator/fake_moma/src/moma_param_load.cpp` (new) | YAML load + finalize pipeline |
| `TopAY/src/simulator/fake_moma/src/moma_param_cr10.cpp` (new) | CR10 transforms + grads + sphere sampling |
| `TopAY/src/simulator/fake_moma/src/test_topay_cr10_fk.cpp` (new) | Standalone FK/grad gate binary |
| `TopAY/src/planner/params/robot_tracer7.yaml` (new) | Extracted Tracer profile |
| `TopAY/src/planner/params/robot_ranger_cr10.yaml` (new) | REMANI-mapped Ranger/CR10/AG95 |
| `TopAY/src/planner/params/agent_ranger_cr10.yaml` (new) | 9-D pick/mid/place states |
| `TopAY/src/planner/params/map_ranger_cr10_smoke.yaml` (new) | Open smoke map params |
| `TopAY/src/planner/launch/run_all.launch` | `robot` arg + yaml select |
| `TopAY/src/planner/launch/run_ranger_cr10_smoke.launch` (new) | Deterministic success gate |
| `TopAY/src/map/include/map/grid_map.h` (+ `.cpp`) | `setMomaParam`, dual-radius whole-body checks |
| `TopAY/src/planner/include/planner/{planner,birrts,mcrrts,ompls,mpc,moma_traj_opt}.h` (+ src) | Inject shared profile; kill `.tail(7)` |
| `TopAY/src/simulator/fake_moma/src/{moma_sim,moma_vis}.cpp` | Mesh poses from `getLinkTransforms`; no `markers[8]` |
| `TopAY/src/simulator/fake_moma/meshes/ranger_cr10/` (new) | Vendored ranger/cr10/ag95 STL |

Do **not** fork a second `fake_moma` package.

---

### Task 1: Tracer profile YAML + `MomaParam::fromRos` finalize (no behavior change)

**Files:**
- Create: `TopAY/src/planner/params/robot_tracer7.yaml`
- Create: `TopAY/src/simulator/fake_moma/src/moma_param_load.cpp`
- Modify: `TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h`
- Modify: `TopAY/src/simulator/fake_moma/CMakeLists.txt`
- Modify: `TopAY/src/planner/launch/run_all.launch` (load tracer yaml explicitly for this task’s test)

**Interfaces:**
- Produces:
  - `enum class KinematicsType { TopayAlt, Cr10 };`
  - `static MomaParam fromRos(const ros::NodeHandle& nh);`
  - `void validateAndFinalize();` (called inside `fromRos`)
  - Fields: `KinematicsType kinematics; std::string robot_name; Eigen::Vector3d T_base_mount_xyz; Eigen::Matrix3d T_base_mount_R;` (Tracer may mirror old `relative_t/R`)

- [ ] **Step 1: Extract current constructor defaults into `robot_tracer7.yaml`**

Under prefix `moma/` (stick to it):

```yaml
moma:
  robot_name: tracer7
  kinematics: topay_alt
  dof_num: 7
  chassis_length: 0.685
  chassis_width: 0.57
  chassis_height: 0.155
  chassis_colli_radius: 0.4
  max_v: 1.0
  max_a: 0.8
  max_w: 1.25
  max_dw: 1.0
  # ... all arrays currently hard-coded in MomaParam() ...
  relative_t: [0.0, 0.115, 0.016]
  relative_R_row_major: [0.7071068, 0.7071068, 0.0, -0.7071068, 0.7071068, 0.0, 0.0, 0.0, 1.0]
  mesh_prefix: "package://fake_moma/meshes/"
```

Values must match `moma_param.h` constructor exactly.

- [ ] **Step 2: Add factory API to header**

```cpp
// ################################
// C++: MomaParam profile load API begin
// ################################
enum class KinematicsType { TopayAlt = 0, Cr10 = 1 };

struct KinematicResult {
  std::vector<Eigen::Matrix4d> link_T;
  Eigen::Matrix4d ee_T;
};

struct CollisionSphere {
  Eigen::Vector3d local_offset;
  double obstacle_radius = 0.0;
  double self_radius = 0.0;
  int link_id = -1;
};

static MomaParam fromRos(const ros::NodeHandle& nh);
void validateAndFinalize();
KinematicsType kinematics = KinematicsType::TopayAlt;
std::string robot_name = "tracer7";
// ################################
// C++: MomaParam profile load API end
// ################################
```

Keep default constructor building tracer7 identically so existing nodes still compile in this task.

- [ ] **Step 3: Implement `fromRos` + `validateAndFinalize` in `moma_param_load.cpp`**

Pipeline:

```text
read scalars/arrays
→ set dof_num / kinematics
→ resize dynamic members
→ if TopayAlt: rebuild collision_matrix / colli_link_map exactly as old ctor
→ validate sizes (throw std::runtime_error on mismatch)
```

Wire into `fake_moma` CMake sources list.

- [ ] **Step 4: Load `robot_tracer7.yaml` from launch and smoke-run**

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch planner run_all.launch robot:=tracer7 rviz:=false
```

Expected: starts without FATAL; planning still works as before.

- [ ] **Step 5: Commit**

```bash
git add TopAY/src/planner/params/robot_tracer7.yaml \
  TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h \
  TopAY/src/simulator/fake_moma/src/moma_param_load.cpp \
  TopAY/src/simulator/fake_moma/CMakeLists.txt \
  TopAY/src/planner/launch/run_all.launch
git commit -m "$(cat <<'EOF'
Add TopAY tracer7 robot profile loader without behavior change.

EOF
)"
```

---

### Task 2: Repo-wide DOF / marker / vector-size de-hardcoding

**Files:**
- Modify: all TopAY hits from `rg 'tail\(7\)|markers\[8\]|Zero\(10\)' TopAY --glob '*.{cpp,h,hpp}'`
- Especially: `moma_traj_opt.cpp`, `moma_traj_opt_falm.cpp`, `moma_traj_opt_relax.cpp`, `ompc.cpp`, `moma_traj_opt.h`, `moma_sim.cpp`, `moma_vis.cpp`

**Interfaces:**
- Consumes: `moma_param.dof_num`
- Produces: no remaining arm-width literal `7` in state tails; marker indices by role

- [ ] **Step 1: Replace `.tail(7)` with `.tail(moma_param.dof_num)`**

```cpp
// ################################
// C++: use dof_num instead of literal 7 begin
// ################################
state_wb.tail(moma_param.dof_num) = init_path[i].tail(moma_param.dof_num);
// ################################
// C++: use dof_num instead of literal 7 end
// ################################
```

Rename `state12d` locals to `state_wb` sized `3+dof_num` where the name implies 3+7.

- [ ] **Step 2: Fix `ompc.cpp` command extraction**

```cpp
VectorXd next_q = traj.getState(t_cur+1.0/ctrl_freq).tail(moma_param.dof_num);
VectorXd next_dq = traj.getDState(t_cur+1.0/ctrl_freq).tail(moma_param.dof_num);
```

- [ ] **Step 3: Replace `moma_marker.markers[8]` with role indices**

```cpp
const size_t gripper_idx = 2 + moma_param.dof_num; // 0 base, 1 arm_base, 2..1+dof joints
```

- [ ] **Step 4: Build and grep gate**

```bash
rg -n 'tail\(7\)|markers\[8\]' TopAY --glob '*.{cpp,h,hpp}'
# expected: no hits (or only comments)
catkin_make -DCMAKE_BUILD_TYPE=Release
```

- [ ] **Step 5: Commit**

```bash
git commit -am "$(cat <<'EOF'
Remove TopAY hardcoded 7-DOF state and marker assumptions.

EOF
)"
```

---

### Task 3: CR10 `getLinkTransforms` + analytic grads (math only)

**Files:**
- Create: `TopAY/src/simulator/fake_moma/src/moma_param_cr10.cpp`
- Modify: `TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h`
- Create: `TopAY/src/simulator/fake_moma/src/test_topay_cr10_fk.cpp`
- Modify: `TopAY/src/simulator/fake_moma/CMakeLists.txt`
- Create: `TopAY/src/planner/params/robot_ranger_cr10.yaml` (kinematics + mount + config; collision filled in Task 4)

**Interfaces:**
- Consumes: `manipulator_config[6]`, `T_base_mount` xyz/ypr, joint vector `q` length 6, base SE2 in `state.head(3)`
- Produces:
  - `KinematicResult getLinkTransforms(const Eigen::VectorXd& state) const;`
  - `getFKPose` / `getEEGrads` dispatch for `Cr10`

**Mount rule (must implement literally):**

```cpp
// T_W_base from (x,y,yaw); chassis_height NOT applied to arm FK
Eigen::Matrix4d T_W_mount = T_W_base * T_base_mount; // translation z = 0.1
```

Fixed CR10 frames (from REMANI `test_cr10_fk.cpp`):

```cpp
F[0] = makeFixed(0, 0, 0.1765, 0, 0, 0);
F[1] = makeFixed(0, 0, 0, 1.5708, 1.5708, 0);
F[2] = makeFixed(-0.607, 0, 0, 0, 0, 0);
F[3] = makeFixed(-0.568, 0, 0.191, 0, 0, -1.5708);
F[4] = makeFixed(0, -0.125, 0, 1.5708, 0, 0);
F[5] = makeFixed(0, 0.1084, 0, -1.5708, 0, 0);
// joint i: T *= F[i] * Rz(q[i])
```

Prefer YAML `manipulator_config` lengths with the same fixed rpy.

- [ ] **Step 1: Write failing FK test binary `test_topay_cr10_fk.cpp`**

Mirror REMANI gates: 100 samples; mount `(0.2462,0,0.1)`; require `max_pos < 1e-3`, `max_rot < 0.1` deg, `max_grad_rel < 1e-4`.

```cmake
add_executable(test_topay_cr10_fk src/test_topay_cr10_fk.cpp src/moma_param_cr10.cpp src/moma_param_load.cpp)
target_link_libraries(test_topay_cr10_fk ${catkin_LIBRARIES})
```

- [ ] **Step 2: Run test — expect FAIL**

```bash
catkin_make --pkg fake_moma
rosrun fake_moma test_topay_cr10_fk
```

- [ ] **Step 3: Implement CR10 backend**

```cpp
KinematicResult MomaParam::getLinkTransforms(const Eigen::VectorXd& state) const {
  if (kinematics == KinematicsType::Cr10) return linkTransformsCr10(state);
  return linkTransformsTopayAlt(state);
}
```

`topay_alt` must reproduce existing Tracer FK.

- [ ] **Step 4: Run test — expect PASS**

```bash
rosrun fake_moma test_topay_cr10_fk
# Expected: CR10 FK/grad validation PASSED
```

- [ ] **Step 5: Commit**

```bash
git commit -am "$(cat <<'EOF'
Add TopAY CR10 getLinkTransforms with REMANI-aligned FK tests.

EOF
)"
```

---

### Task 4: CR10 collision spheres (variable count, dual radii) + GridMap wiring

**Files:**
- Modify: `moma_param.h`, `moma_param_cr10.cpp`, `moma_param_load.cpp`
- Modify: `TopAY/src/map/include/map/grid_map.h`, `TopAY/src/map/src/grid_map.cpp`
- Modify: `robot_ranger_cr10.yaml` collision section

**Interfaces:**
- Produces: `std::vector<CollisionSphere> collision_spheres_` built in finalize
- Produces: `getColliPts(state)` uses **obstacle_radius** in `w` for env
- Produces: self-check uses **self_radius**
- GridMap: `void setMomaParam(const std::shared_ptr<const MomaParam>& p);`

- [ ] **Step 1: Define sampling rule in YAML**

```yaml
moma:
  obstacle_thickness: 0.10
  self_thickness: 0.045
  sphere_spacing_factor: 1.0   # N_i = ceil(L_i / (factor * r_obs))
  ag95_spheres:
    - {xyz: [0.0, 0.0, 0.06], obstacle_radius: 0.05, self_radius: 0.04}
    - {xyz: [0.04, 0.03, 0.10], obstacle_radius: 0.03, self_radius: 0.025}
    - {xyz: [0.04, -0.03, 0.10], obstacle_radius: 0.03, self_radius: 0.025}
```

Long links 0.607 / 0.568 must get \(N>2\).

- [ ] **Step 2: Implement `buildCollisionProxies()` + ignore matrix for new sphere graph**

- [ ] **Step 3: Update GridMap shared profile + dual-radius checks**

```cpp
void GridMap::setMomaParam(const std::shared_ptr<const MomaParam>& p) {
  moma_param_shared_ = p;
  moma_param = *p; // temporary compat if inlines still use value member
}
```

Env checks keep obstacle radius; self / car-mani use `self_radius`.

- [ ] **Step 4: Collision smoke cases**

1. open pose → not self-colliding  
2. folded pose → self or car-mani colliding  
3. mid-span of link2/link3 has a sphere center within ~5 cm (coverage check)

- [ ] **Step 5: Commit**

```bash
git commit -am "$(cat <<'EOF'
Add CR10 variable collision spheres with separate obstacle/self radii.

EOF
)"
```

---

### Task 5: Inject shared `MomaParam` through planner stack

**Files:**
- Modify: `planner.h`, `planner.cpp`
- Modify: `birrts`, `mcrrts`, `ompls`, `mpc`, `moma_traj_opt` headers/sources
- Modify: GridMap (already has `setMomaParam`)

**Interfaces:**
- Produces: every submodule uses the same `shared_ptr<const MomaParam>`
- Forbids: child `init()` constructing a fresh default `MomaParam()` and ignoring planner profile

- [ ] **Step 1: Planner loads once and injects**

```cpp
void Planner::init(ros::NodeHandle& nh) {
  moma_param_shared_ = std::make_shared<MomaParam>(MomaParam::fromRos(nh));
  moma_param = *moma_param_shared_;
  grid_map.reset(new GridMap);
  grid_map->init(nh);
  grid_map->setMomaParam(moma_param_shared_);
  birrts = std::make_shared<BiRRTs>(grid_map);
  birrts->setMomaParam(moma_param_shared_);
  birrts->init(nh);
  // same for mcrrts, ompl_planner, traj_opter, mpc, ...
}
```

- [ ] **Step 2: Children only assign from shared_ptr (never re-fromRos)**

```cpp
void BiRRTs::setMomaParam(const std::shared_ptr<const MomaParam>& p) {
  moma_param_shared_ = p;
  moma_param = *p;
}
```

- [ ] **Step 3: Build + `robot:=tracer7` regression**

```bash
catkin_make
roslaunch planner run_all.launch robot:=tracer7 rviz:=false
```

- [ ] **Step 4: Commit**

```bash
git commit -am "$(cat <<'EOF'
Inject a single finalized MomaParam profile across TopAY planner modules.

EOF
)"
```

---

### Task 6: Point `fake_moma` / `moma_vis` at `getLinkTransforms` + vendor meshes

**Files:**
- Modify: `moma_sim.cpp`, `moma_vis.cpp`
- Create: `TopAY/src/simulator/fake_moma/meshes/ranger_cr10/**` from `agx/rangerboxcr10lidar_description/meshes/`
- YAML `mesh_files` list

**Interfaces:**
- Consumes: `getLinkTransforms(state)`
- Produces: marker poses from `link_T[i]` / AG95 from `ee_T`

- [ ] **Step 1: Vendor meshes into `meshes/ranger_cr10/`**

- [ ] **Step 2: Rewrite vis loop**

```cpp
MomaParam moma_param = MomaParam::fromRos(nh);
auto kin = moma_param.getLinkTransforms(moma_pos);
for (size_t i = 0; i < kin.link_T.size(); ++i) {
  setMarkerPose(markers[i], kin.link_T[i]);
  markers[i].mesh_resource = mesh_files[i];
}
```

Delete CR10-path alternating-axis `euler2rotation(joint_offset)` chains.

- [ ] **Step 3: Startup profile log**

```cpp
ROS_INFO("fake_moma robot=%s dof=%zu kinematics=%d mount=[%.4f %.4f %.4f] chassis_h=%.3f (mount not stacked for cr10)",
  moma_param.robot_name.c_str(), moma_param.dof_num, (int)moma_param.kinematics,
  moma_param.T_base_mount_xyz.x(), moma_param.T_base_mount_xyz.y(), moma_param.T_base_mount_xyz.z(),
  moma_param.chassis_height);
```

- [ ] **Step 4: Visual smoke**

```bash
roslaunch planner run_all.launch robot:=ranger_cr10 rviz:=true
# Expected: Ranger+CR10+AG95; arm base ~0.1 m world Z, not ~0.25 m
```

- [ ] **Step 5: Commit**

```bash
git commit -am "$(cat <<'EOF'
Drive TopAY sim/vis meshes from unified CR10/Tracer link transforms.

EOF
)"
```

---

### Task 7: Launch switch, agent overlay, smoke scene, acceptance

**Files:**
- Modify: `run_all.launch` (+ other daily entry launches if needed)
- Create: `agent_ranger_cr10.yaml`, `map_ranger_cr10_smoke.yaml`, `run_ranger_cr10_smoke.launch`
- Optional: short `TopAY/README.md` run notes

**Interfaces:**
- `arg robot` default `ranger_cr10`
- Loads `robot_$(arg robot).yaml` + matching agent/map overlays

- [ ] **Step 1: Launch wiring**

```xml
<arg name="robot" default="ranger_cr10"/>
<rosparam command="load" file="$(find planner)/params/robot_$(arg robot).yaml"/>
```

Keep current `agent.yaml` for tracer; add `agent_ranger_cr10.yaml` for CR10 (9-D states).

- [ ] **Step 2: Deterministic smoke start/goal (9-D) in open map**

```yaml
agent:
  fixed_startgoal: true
  random_ee: false
  pick_state:  [0.0, 0.0, 0.0,  0, -0.4, 0.8, 0, 0.4, 0]
  place_state: [3.0, 0.0, 0.0,  0, -0.4, 0.8, 0, 0.4, 0]
```

Tune joints via collision tests so both ends are free.

- [ ] **Step 3: Run acceptance checklist**

1. `robot:=tracer7` regression  
2. `rosrun fake_moma test_topay_cr10_fk` PASS  
3. `roslaunch planner run_ranger_cr10_smoke.launch` → PLAN SUCCESS + reach goal  
4. Collision regression (open/fold/mid-link)  
5. Launch-arg switch only  

- [ ] **Step 4: Commit**

```bash
git commit -am "$(cat <<'EOF'
Default TopAY launch to ranger_cr10 with smoke acceptance scene.

EOF
)"
```

---

## Spec Coverage Self-Review

| Spec requirement | Task |
|------------------|------|
| Dual YAML profiles + default ranger_cr10 | 1, 7 |
| `fromRos` finalize pipeline | 1 |
| Repo-wide de-hardcode | 2 |
| CR10 mount not stacked on chassis_height | 3 |
| `getLinkTransforms` + analytic grads | 3 |
| FK math gates | 3 |
| Dual obstacle/self radii | 4 |
| Variable spheres + long-link coverage | 4 |
| Shared profile injection | 5 |
| sim/vis unified FK | 6 |
| AG95 multi-sphere + meshes | 4, 6 |
| Virtual-diff / speed approx wording | YAML comments in 1/3/7 |
| Smoke SUCCESS acceptance | 7 |
| EE 6D / hardware Non-Goals | intentionally omitted |

## Type Consistency

- Cross-task contracts: `KinematicsType`, `KinematicResult`, `CollisionSphere`, `MomaParam::fromRos`, `getLinkTransforms`, `setMomaParam(shared_ptr<const MomaParam>)`.
- `dof_num` is `size_t`; whole-body state width is always `3 + dof_num`.
