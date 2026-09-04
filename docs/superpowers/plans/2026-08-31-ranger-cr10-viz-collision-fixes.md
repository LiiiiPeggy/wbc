# Ranger+CR10 轮胎 / 碰撞可视化 / Box 环境碰撞修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 smoke 验收中剩余三类问题：轮胎 mesh 穿地、CR10 碰撞体 RViz 显示与 CAD 坐标系混淆、Ranger box 相对环境障碍物的 **planning false negative**（含 discrete + continuous collision）。

**Architecture:** 保持 **planning frame（base z=0）** 与 **visual frame（+0.275 CAD root）** 严格分离。绿色 `/sphere` 继续表示 **arm planning collision truth**（`getColliPtsCr10()`）；新增 `/sphere_visual` 仅用于 CAD 对齐调试，alignment gate 基于 **owner-link transform × proxy.local_offset**，不得与 link origin 比较。Box 环境碰撞通过 **独立 `base_obstacle_proxies_`** + `getBaseObstaclePts()` / `getBaseObstacleGrads()` 接入 **GridMap 离散碰撞** 与 **MomaTrajOpt 连续 collision cost** 两层。

**Tech Stack:** ROS1 Noetic, TopAY `fake_moma`, `map/grid_map`, `planner/moma_traj_opt*`, Eigen, URDF `rangercr10lidar.urdf`, `visual_transform_utils`（`f792a3a`）, `test_topay_cr10_fk`, `test_topay_ranger_visual`.

**Spec:** 用户 smoke 反馈（2026-08-31）+ 代码基线 `f792a3a` + Phase A 约束 `docs/superpowers/plans/2026-08-12-topay-ranger-cr10.md`

## Global Constraints

- **禁止**修改 CR10 臂架数学 FK：`getLinkTransformsCr10()`、`getFKPoseCr10()`、`getEEGradsCr10()`、`mount.relative_t = [0.2462, 0, 0.1]`、CR10 joint fixed transforms。
- **原则上不得修改**现有 arm collision gradient logic（`getColliGradsCr10()` 中 `link_id >= 1` 的 joint/x/y/yaw 链）。
- **允许**为 base obstacle proxy 新增独立 API：`getBaseObstacleGrads()` / `getBaseObstacleGradsCr10()`，仅含 base 的 `∂p/∂x, ∂p/∂y, ∂p/∂yaw`；**q1~q6 对 base box proxy gradient 严格为 0**。
- Planning base **z=0**；visual root **+0.275** 仅用于 CAD mesh 与 **RViz visual collision overlay**；**禁止**把 visual root 写入 `getColliPtsCr10()` / `getBaseObstaclePts()` / GridMap / traj-opt。
- **禁止**通过调整 random map 障碍生成高度来规避 box 漏检。
- **禁止** `fake_moma` test 依赖 `map` package（会形成 `fake_moma ↔ map` cycle）；GridMap 集成测试放 `map` package。
- 新增/修改 C++ 代码块前仅加：`// ################################` + `// C++: description` + `// ################################`（YAML/XML 同理）；**不要** end 注释。
- 每个 task 独立可测；下文 **Final Regression Gates** 全部 PASS 方可收尾。

---

## 代码基线事实（`f792a3a`，已核对）

### RViz 默认显示

```text
/fake_moma_node/sphere    Enabled: true   → getColliMarkerArray()
/fake_moma_node/cylinder  Enabled: false  → getColliCylinderArray()
```

用户当前看到的「绿色碰撞球不对」**主要对应** `getColliMarkerArray()`，**不是** cylinder。

### CR10 planning collision 与 owner link

```text
p_plan = T_planning_owner_link * [proxy.local_offset, 1]^T
```

Owner link 映射（须复用 `cr10LinkTransformAt()` 或等价 helper，**禁止**测试重新实现一套）：

```text
link_id == 1           → arm_base_T
link_id >= 2 && < 2+dof → arm_link_T[link_id - 2]
AG95 / EE              → ee_T
```

Visual overlay 同构关系：

```text
T_visual_owner = applyVisualRoot(base_T, T_planning_owner)
expected_visual_sphere = T_visual_owner * [proxy.local_offset, 1]^T
```

### Planning vs Visual 分层（`f792a3a` 后）

```text
CAD mesh:  applyVisualRoot(base_T, link_T) * link_T_visual     → 含 +0.275
Collision: getColliPtsCr10()                                   → planning z=0，无 visual root
```

### GridMap chassis 碰撞模型（box false negative 根因）

```cpp
// grid_map.cpp — 建 2D occ 层时
if (pc.points[i].z < moma_param->chassis_height)  // 0.15
    occ_buffer_2d[...] = 1;
// grid_map.h — isWholeBodyCollision
if (isCollision2d(state.head(2), chassis_colli_radius))  // r = 0.711
```

实质：**XY 外接圆 + Z 仅覆盖 z∈[0, 0.15) 障碍点**；box 中高处无 3D envelope → **planning false negative**。

### 连续优化同样漏检

`MomaTrajOpt::eeCostCallback()`（`moma_traj_opt.cpp:66-139`）仅对 `getColliPts()` 做 ESDF + self collision cost。**仅改 GridMap 不够**——RRT/waypoint 可能安全，trajectory optimizer 仍可能优化到 box 穿障碍。

---

## 根因判断（修订后）

| 现象 | 修订根因 | 首要验证 |
|------|---------|---------|
| 绿色球「位置不对」 | planning vs visual frame 混淆；或 overlay 缺失 | Task 2A：planning truth；Task 2B：`local_offset` transform gate |
| 绿色球「太大」 | RViz 显示 `obstacle_radius=0.10` | Task 2A 同时记录 obstacle/self 半径 |
| box 穿障碍 planner 不报 | chassis 2D + z 过滤；**无 base 3D envelope** | Task 3B + 3C2 discrete + 3C3 continuous |
| 轮胎穿地 | wheel `Rx(90°)`；须 world-space STL bounds | Task 0 全顶点变换 |

---

## Planning vs Visual 碰撞语义

```text
Planning collision truth (arm)
------------------------------
来源:   getColliPtsCr10() / collision_proxies_
坐标系: planning world / base z=0
用途:   GridMap arm 3D, traj-opt arm ESDF, self collision
禁止:   加入 visual root

Planning collision truth (base box)
-----------------------------------
来源:   getBaseObstaclePts() / base_obstacle_proxies_
坐标系: planning world / base z=0
用途:   GridMap base 3D, traj-opt base ESDF
禁止:   加入 collision_matrix / self collision / visual root

RViz visual collision overlay (debug only)
------------------------------------------
来源:   getColliPtsCr10() 每颗 proxy
显示:   T_visual_owner * local_offset（完整 applyVisualRoot）
用途:   RViz 调试；禁止反馈 planner
```

### 方案 A：`/sphere` + `/sphere_visual`

| Topic | 语义 |
|-------|------|
| `/fake_moma_node/sphere` | arm **planning truth** |
| `/fake_moma_node/sphere_visual` | arm **visual overlay**（local_offset transform gate） |

Base box obstacle spheres 可选单独 topic `/base_obstacle_sphere`（planning truth），不与 arm overlay 混用。

---

## 数据结构决策（优先方案）

**首选：分离向量（推荐）**

```cpp
std::vector<CollisionSphere> collision_proxies_;       // CR10 arm + AG95
std::vector<CollisionSphere> base_obstacle_proxies_;   // Ranger upper box

// Arm API（语义不变）
getColliPtsCr10()
getColliGradsCr10()

// Base obstacle API（新增）
getBaseObstaclePts()      // 或 getBaseObstaclePtsCr10()
getBaseObstacleGrads()    // 或 getBaseObstacleGradsCr10()
```

语义：

```text
collision_proxies_:
  environment + self collision
  collision_matrix / colli_self_radii_ / colli_link_map 仅覆盖此向量

base_obstacle_proxies_:
  environment only
  NO self collision
  NO collision_matrix
  base x/y/yaw gradient only
```

**备选（仅当分离向量不可行时）：** `CollisionSphere { check_environment; check_self; }` 统一向量。若采用，**必须**保证：

```text
collision_matrix.rows() == collision_proxies_.size()
getColliPts().size() 不得 > collision_matrix.rows()
isSelfCollision() 必须 filter check_self == false 的 proxy
```

**禁止** `getColliPts().size() > collision_matrix.rows()` 且下游仍 `collision_matrix(i,j)`。

---

## File Structure

| Path | 职责 |
|------|------|
| `scripts/audit_ranger_stl_bounds.py` | 真实 YAML + `package://` STL；binary/ASCII；world bounds |
| `docs/superpowers/plans/2026-08-31-ranger-geometry-collision-audit.txt` | Task 0 统一审计报告 |
| `docs/superpowers/plans/2026-08-31-collision-consumer-audit.txt` | Task 3C0 consumer 表 |
| `TopAY/src/planner/params/robot_ranger_cr10.yaml` | wheel visual；box obstacle layout（Task 3A 后） |
| `TopAY/src/simulator/fake_moma/include/fake_moma/moma_param.h` | `base_obstacle_proxies_`；sphere_visual API |
| `TopAY/src/simulator/fake_moma/src/moma_param_cr10.cpp` | `getBaseObstaclePts/Grads` |
| `TopAY/src/simulator/fake_moma/src/moma_sim.cpp` | `/sphere_visual`；cylinder ≠ GridMap 注释 |
| `TopAY/src/simulator/fake_moma/src/colli_visual_utils.cpp` | CR10 cylinder exact chain |
| `TopAY/src/simulator/fake_moma/src/test_topay_cr10_colli_frame.cpp` | Task 2A/2B frame + overlay gates |
| `TopAY/src/simulator/fake_moma/src/test_topay_cr10_fk.cpp` | arm grad 100-sample + base obstacle FD grad |
| `TopAY/src/map/src/grid_map.cpp` | discrete base obstacle 3D 查询 |
| `TopAY/src/map/src/test_topay_box_collision.cpp` | Case A/B/C discrete GridMap |
| `TopAY/src/map/CMakeLists.txt` | 注册 map box collision test |
| `TopAY/src/planner/src/moma_traj_opt.cpp` | continuous base obstacle ESDF cost |
| `TopAY/src/planner/src/moma_traj_opt_falm.cpp` | 同上（若 CR10 smoke 路径使用） |
| `TopAY/src/planner/src/moma_traj_opt_relax.cpp` | 同上（若 CR10 smoke 路径使用） |
| `TopAY/src/planner/src/test_topay_box_traj_collision.cpp` | continuous trajectory obstacle-cost gate |

**删除（原计划错误路径）：**

```text
TopAY/src/simulator/fake_moma/src/test_topay_box_collision.cpp  # 会造成 fake_moma↔map cycle
```

---

### Task 0: 统一 Geometry / Frame / Collision 审计

**Files:**
- Create: `scripts/audit_ranger_stl_bounds.py`
- Create: `docs/superpowers/plans/2026-08-31-ranger-geometry-collision-audit.txt`

- [ ] **Step 1: 实现审计脚本**

必须包含：

```python
def resolve_package_uri(uri: str, repo_root: Path) -> Path:
    if uri.startswith("package://fake_moma/"):
        rel = uri[len("package://fake_moma/"):]
        return repo_root / "TopAY/src/simulator/fake_moma" / rel
    raise ValueError(f"unsupported uri: {uri}")

REPO_ROOT = Path(__file__).resolve().parents[1]  # 或 --repo-root CLI

def load_stl_vertices(path: Path):
    data = path.read_bytes()
    if len(data) >= 84 and data[:5] != b"solid":
        return parse_binary_stl(data)   # format=binary
    return parse_ascii_stl(data)          # format=ascii
```

Ground 语义（**禁止** `vmax[2] - 0.0`）：

```python
ground_z = 0.0
world_zmin = vmin[2]
ground_clearance = world_zmin - ground_z
# ground_clearance < 0 → penetrates ground
# ground_clearance = 0 → tangent
# ground_clearance > 0 → floating
penetration_depth = max(0.0, -ground_clearance)
```

每个 mesh 输出：

```text
mesh | format=binary/ascii | vertex_count
local_bounds | world_bounds
ground_clearance | penetration_depth | recommended_delta_z
```

- [ ] **Step 2: 审计范围**

1. Ranger base STL  
2. box_link.STL — local + planning + visual bounds  
3. 四轮 wheel — 完整 `T_world = T_base * T_visual_root * T_mesh_part`  
4. CR10 collision proxies — index, link_id, local_offset, planning xyz, radii  
5. Chassis collision 模型  
6. Box smoke false negative 实例  

Run: `python3 scripts/audit_ranger_stl_bounds.py | tee docs/superpowers/plans/2026-08-31-ranger-geometry-collision-audit.txt`

- [ ] **Step 3: 基线 regression**

```bash
./devel/lib/fake_moma/test_topay_cr10_fk
./devel/lib/fake_moma/test_topay_ranger_visual
```

- [ ] **Step 4: Commit**

```bash
git commit -m "chore: add unified Ranger geometry/collision audit tooling"
```

---

### Task 1: Wheel Visual-Only Ground Correction

**Files:** `robot_ranger_cr10.yaml`, `scripts/audit_ranger_stl_bounds.py`

- [ ] **Step 1: Python wheel gate（真实 YAML + 真实 STL + package://）**

```python
assert -0.01 <= ground_clearance <= 0.01  # 四轮
```

- [ ] **Step 2: 按 Task 0 报告修正 wheel xyz（仅 visual）**

- [ ] **Step 3: Gate PASS + CR10 FK 不变**

- [ ] **Step 4: Commit**

---

### Task 2A: Collision Sphere Frame Diagnosis

**Files:** `test_topay_cr10_colli_frame.cpp`（或扩展 `test_topay_cr10_fk.cpp`）

- [ ] **Step 1: 诊断报告（零位 + 非零 state）**

```cpp
// 复用 cr10LinkTransformAt() 获取 T_planning_owner
// 验证: pts[i].head<3>() ≈ (T_owner * homog(local_offset)).head<3>()
// 记录: index, link_id, local_offset, planning xyz, r_obs, r_self
// 判定: 位置问题? 半径问题? 两者?
```

测试 state：

```text
state0: x=0, y=0, yaw=0, q=zeros
state1: x=1.0, y=-0.5, yaw=0.6, q=non-zero
```

- [ ] **Step 2: 写入审计报告 §5**

- [ ] **Step 3: Commit**

---

### Task 2B: Visual-Aligned Collision Overlay（方案 A）

**Files:** `moma_param.h`, `moma_sim.cpp`, `moma_vis.cpp`, `default.rviz`, `test_topay_cr10_colli_frame.cpp`

**Alignment gate（P0 修正）**

```cpp
// 禁止: |p_visual.z - p_cad_link.z| < tol
// 正确:
Eigen::Matrix4d T_plan_owner = cr10LinkTransformAt(proxy.link_id, links);
Eigen::Vector4d local = proxy.local_offset.homogeneous();
Eigen::Vector3d expected_plan = (T_plan_owner * local).head<3>();

Eigen::Matrix4d T_vis_owner = applyVisualRoot(base_T, T_plan_owner);
Eigen::Vector3d expected_visual = (T_vis_owner * local).head<3>();

Eigen::Vector3d actual_visual = getColliVisualMarkerCenter(i);
ASSERT_LT((actual_visual - expected_visual).norm(), 1e-6);  // 或 1e-9~1e-6
```

**禁止实现：**

```cpp
p_visual.z += 0.275;  // 即使当前配置碰巧如此
```

- [ ] **Step 1: overlay gate — state0 + state1（含非零 base yaw）**

- [ ] **Step 2: 实现 `/sphere_visual` + rviz 显示项**

- [ ] **Step 3: `/sphere` = planning truth 文档化**

- [ ] **Step 4: Commit**

---

### Task 2C: CR10 Cylinder Debug Visualization

（内容同前版；cylinder ≠ planner truth ≠ GridMap）

- [ ] **Step 1–3:** exact chain test + CR10 dispatch + commit

---

### Task 3A: Box Geometry Audit

**Files:** `scripts/audit_ranger_stl_bounds.py`

- [ ] **Step 1: box_link.STL — local + planning + visual bounds**

输出 XY footprint, z_min, z_max。

- [ ] **Step 2: 验证 0.711 m 圆 vs 实测 box XY footprint**

- [ ] **Step 3: 设计 multi-sphere / OBB layout 建议**

**禁止**未经审计直接单球 `obstacle_radius: 0.05`。

Task 3A 必须输出：

```text
recommended layout: Nx × Ny × Nz (或 OBB)
sphere center spacing
radius
coverage error estimate
margin (e.g. 0.02 m)
目标: planning envelope ⊇ physical box STL
```

验证：box STL 关键表面被 planning envelope 覆盖；physical box 不得明显在 envelope 外部。

---

### Task 3B: Current Collision-Model Diagnosis

- [ ] **Step 1: 文档化 chassis 2D + z<0.15 + arm 3D 模型**

- [ ] **Step 2: 记录 smoke false negative 实例（障碍 xyz/z-range vs planner result）**

---

### Task 3C0: Collision Consumer Audit

**Files:**
- Create: `docs/superpowers/plans/2026-08-31-collision-consumer-audit.txt`
- Read-only: 全仓 `getColliPts*`, `getColliGrads*`, `collision_matrix`, `isWholeBodyCollision`, `isSelfCollision`

**必须在实现 box proxy 前完成。** 全仓搜索并填表：

| Consumer | 文件 | arm env | self | collision_matrix | getColliGrads | must see box env | must NOT see box self |
|----------|------|---------|------|------------------|---------------|------------------|----------------------|
| `GridMap::isWholeBodyCollision` | `grid_map.h:621` | yes (arm pts) | yes | yes | no | **yes (3C2)** | **yes** |
| `MomaTrajOpt::eeCostCallback` | `moma_traj_opt.cpp:66` | yes + ESDF | yes | yes | yes | **yes (3C3)** | **yes** |
| `MomaTrajOpt` inner loop | `moma_traj_opt.cpp:1504` | yes | yes | yes | yes | **yes (3C3)** | **yes** |
| `MomaTrajOptFalm` | `moma_traj_opt_falm.cpp:991` | yes | yes | yes | yes | **yes (3C3)** | **yes** |
| `MomaTrajOptRelax` | `moma_traj_opt_relax.cpp:989` | yes | yes | yes | yes | **yes (3C3)** | **yes** |
| `MomaParam::isSelfCollision` | `moma_param.h:306` | no | yes | yes | no | **no** | **yes** |
| `moma_sim.cpp` self + cylinder KD-tree | `moma_sim.cpp:275` | debug only | yes | — | no | no | yes |
| `moma_vis.cpp` self | `moma_vis.cpp:111` | no | yes | — | no | no | yes |
| `BiRRT::isValid` | `birrts.h:418` | via GridMap | via GridMap | — | no | via 3C2 | yes |
| `MCRRT::isValid` | `mcrrts.h:243,254,380` | via GridMap | via GridMap | — | no | via 3C2 | yes |
| `OMPL::isValid` | `ompls.h:514`, `ompls.cpp:44` | via GridMap | via GridMap | — | no | via 3C2 | yes |
| `planner.cpp` sampling | `planner.cpp:265,335,513` | via GridMap | via GridMap | — | no | via 3C2 | yes |
| `planner.cpp` gripper pts | `planner.cpp:1912` | arm pts only | — | — | no | no | yes |
| `moma_traj_opt.h` min_dist | `moma_traj_opt.h:1002` | arm pts | — | — | no | **评估** | yes |

- [ ] **Step 1: 完成 consumer 表并标注 3C2/3C3 接入点**

- [ ] **Step 2: Commit audit doc**

---

### Task 3C1: Base Upper-Box Obstacle Data Model

**Files:** `robot_ranger_cr10.yaml`, `moma_param.h`, `moma_param_cr10.cpp`

- [ ] **Step 1: 实现 `base_obstacle_proxies_` + YAML layout（来自 Task 3A）**

```yaml
box_obstacle:
  enabled: true
  layout: multi_sphere   # 或 obb — 来自 3A audit
  spheres:
    - local_offset: [dx, dy, dz]   # 至少一个 sphere 的 dx或dy != 0（供 yaw grad 测试）
      obstacle_radius: R
  margin: 0.02
```

- [ ] **Step 2: 实现 `getBaseObstaclePts()` / `getBaseObstacleGrads()` 声明**

- [ ] **Step 3: 确认 `collision_proxies_` / `collision_matrix` / `colli_self_radii_` 尺寸不变**

- [ ] **Step 4: Commit**

---

### Task 3D: Base Obstacle Analytic Gradient（x/y/yaw only）

**依赖:** Task 3C1 完成后；**必须在 3C3 前完成**（continuous integration 需要正确 gradient）

**Files:** `moma_param_cr10.cpp`, `test_topay_cr10_fk.cpp`

- [ ] **Step 1: `getBaseObstacleGrads()` — base x/y/yaw only**

```cpp
// p = base_T * local_offset
// ∂p/∂x, ∂p/∂y, ∂p/∂yaw 非零
// ∂p/∂q1..q6 == 0
```

- [ ] **Step 2: 定向 FD gate（禁止 sample % N 碰巧命中）**

必须选择 **`local_offset.x != 0` 或 `local_offset.y != 0`** 的 base sphere（如 corner/perimeter sphere）：

```cpp
// yaw=0.0 和 yaw=0.7 两个 state
// 分别验证 x, y, yaw 三个 derivative
// relative error < 1e-4
// yaw gradient 必须非零（证明 analytic yaw branch 正确）
```

- [ ] **Step 3: 保留 arm 100-sample collision gradient regression PASS**

- [ ] **Step 4: Commit**

---

### Task 3C2: Discrete GridMap Integration

**Files:** `grid_map.h`, `grid_map.cpp`

- [ ] **Step 1: `isWholeBodyCollision()` 增加 base obstacle 3D 查询**

```cpp
// 在 chassis 2D + arm spheres 之后:
for (const auto& pt : moma_param->getBaseObstaclePts(state)) {
    if (isCollision3d(pt.head(3), pt(3)))
        return true;
}
// base obstacle 不参与 self collision loop
// base obstacle 不进入 collision_matrix
```

- [ ] **Step 2: Commit**

---

### Task 3C3: Continuous Trajectory-Optimization Integration

**Files:** `moma_traj_opt.cpp`, `moma_traj_opt_falm.cpp`, `moma_traj_opt_relax.cpp`

**P0 requirement:** Box envelope 必须进入连续 collision cost，不能只改 GridMap。

- [ ] **Step 1: 在 eeCostCallback / 等价 collision cost loop 增加 base obstacle ESDF**

```cpp
for (const auto& pt : obj.moma_param->getBaseObstaclePts(now_pos)) {
    obj.grid_map->getDisWithGradI3d(pt.head(3), sdf_value, grad_pc);
    if (violaPos > 0) {
        sdf_cost += ...;
        base_pos_grads.push_back(...);
    }
}
moma_grad += obj.moma_param->getBaseObstacleGrads(now_pos, base_pos_grads);
// ∂cost/∂x, ∂cost/∂y, ∂cost/∂yaw 必须进入 optimizer gradient
// ∂cost/∂q1..q6 对 base proxy == 0
```

- [ ] **Step 2: 同步 falm / relax 变体（若 smoke 路径使用）**

- [ ] **Step 3: Commit**

---

### Task 3E: Box Collision Regression

**测试职责拆分（避免 package cycle）：**

```text
fake_moma/test_topay_cr10_fk.cpp
  base obstacle FK
  base obstacle x/y/yaw FD gradient（nonzero local xy, nonzero yaw state）
  arm 100-sample collision gradient

map/test_topay_box_collision.cpp
  GridMap discrete Case A/B/C

planner/test_topay_box_traj_collision.cpp
  continuous trajectory obstacle-cost gate
```

**Files:**
- Create: `TopAY/src/map/src/test_topay_box_collision.cpp`
- Modify: `TopAY/src/map/CMakeLists.txt`
- Create: `TopAY/src/planner/src/test_topay_box_traj_collision.cpp`
- Modify: `TopAY/src/planner/CMakeLists.txt`

- [ ] **Case A — 低障碍（z ∈ [0, 0.15)，chassis 盘内）→ collision == true**

- [ ] **Case B — 中高障碍（z > 0.15，与 box envelope 相交）→ 修复后 collision == true**

- [ ] **Case C — 高于 box 顶部 → collision == false**

- [ ] **Continuous gate — `test_topay_box_traj_collision.cpp`**

```cpp
// start safe, end safe
// mid trajectory: base box envelope intersects obstacle
// discrete endpoints collision-free
// midpoint: trajectory collision cost > 0
// x/y/yaw gradient finite and non-zero in expected direction
// (optional) optimized trajectory no longer intersects
```

- [ ] **Commit**

---

### Task 4: Full Regression + Smoke

- [ ] **Step 1: 跑完全部 Final Regression Gates**

- [ ] **Step 2: Smoke — `roslaunch planner run_ranger_cr10_smoke.launch rviz:=true`**

RViz 目视仅辅助。

- [ ] **Step 3: Commit docs**

---

## Final Regression Gates（全部必须 PASS）

```text
CR10 FK/EE-grad validation PASSED

CR10 arm collision-gradient 100-sample validation PASSED

CR10 collision pose regression PASSED
  (home/open, folded, mid-link obstacle, wrist no-FP)

Ranger visual-root / LiDAR / D435 regression PASSED

Four-wheel actual-YAML + actual-STL world-zmin gate PASSED

CR10 planning sphere numeric truth gate PASSED

CR10 visual collision overlay exact local-offset transform gate PASSED
  - zero state
  - nonzero x/y/yaw/q

CR10 cylinder exact-chain debug visualization PASSED

Base-box obstacle proxy geometry coverage PASSED

Base-box x/y/yaw analytic gradient FD PASSED
  - nonzero local xy offset
  - nonzero yaw state

Ranger box low-obstacle collision PASSED

Ranger box upper-body obstacle collision PASSED

Ranger above-box clearance PASSED

Ranger box continuous trajectory obstacle-cost gate PASSED

TopAY Ranger+CR10 smoke planning PASSED
```

---

## 建议执行顺序

```text
Task 0   统一 geometry/frame/collision audit
   ↓
Task 1   wheel visual-only ground correction
   ↓
Task 2A  sphere planning truth validation
   ↓
Task 2B  visual-aligned collision overlay (/sphere_visual)
   ↓
Task 2C  CR10 cylinder debug visualization
   ↓
Task 3A  box geometry audit (+ sphere layout design)
   ↓
Task 3B  current collision-model diagnosis
   ↓
Task 3C0 collision consumer audit
   ↓
Task 3C1 base obstacle data model (base_obstacle_proxies_)
   ↓
Task 3D  base obstacle analytic gradient
   ↓
Task 3C2 discrete GridMap integration
   ↓
Task 3C3 continuous trajectory-optimization integration
   ↓
Task 3E  regression (map discrete + planner continuous)
   ↓
Task 4   full regression + smoke
```

**可直接进入执行，无需第三轮 plan review。**

---

## Self-Review（第二轮审核对照）

| 审核项 | 计划中的处理 |
|--------|-------------|
| Task 2B alignment gate | `expected = T_visual_owner * local_offset`；禁止 vs link origin |
| Collision Consumer Audit | Task 3C0 + consumer 表 + 3C2/3C3 标注 |
| separate vs flags | **优先 `base_obstacle_proxies_`**；flags 为备选 + index contract |
| Box discrete | Task 3C2 GridMap |
| Box continuous | Task 3C3 MomaTrajOpt* ESDF + base grad |
| test 放 map package | `map/test_topay_box_collision.cpp`；continuous 放 planner |
| Python package URI + STL | Task 0 `resolve_package_uri` + binary/ASCII |
| Base yaw grad 真实测试 | Task 3D：nonzero local xy + yaw=0/0.7 |
| Continuous trajectory gate | Task 3E + Final Regression Gates |
| wheel ground_clearance | 修正语义；禁止 penetration 混用 |

---

## 风险

| 风险 | 缓解 |
|------|------|
| `/sphere_visual` 被误当作 planner 输入 | 命名 + README + rviz 默认区分 |
| base obstacle 仅接入 GridMap，continuous optimizer 仍穿 box | Task 3C0 consumer audit + 3C3 + continuous gate |
| unified proxy vector 破坏 collision_matrix indexing | **优先 separate `base_obstacle_proxies_`**；否则 assert size contract |
| visual overlay 测试比较 sphere vs link origin | gate = `T_visual_owner * local_offset` |
| base box proxy 破坏 arm self-collision | 不进 `collision_proxies_` / `collision_matrix` |
| base grad 改动污染 arm grad | 独立 API + 定向 FD + 100-sample arm 回归 |
| wheel YAML 与 audit 脚本漂移 | gate parse 生产 YAML |
| fake_moma ↔ map dependency cycle | GridMap tests 在 map package |
| 单 sphere box envelope 覆盖不足 | Task 3A multi-sphere layout + coverage gate |

---

## Execution Handoff

**Plan complete. 可直接按上述顺序执行 Task 0 → Task 4。**

**1. Subagent-Driven (recommended)** — Fresh subagent per task, review between tasks

**2. Inline Execution** — Execute tasks in this session using executing-plans
