# REMANI 末端 6D 位姿目标设计

**Date:** 2026-08-19  
**Revised:** 2026-08-19 (review: 9-DoF IK, WAIT_TARGET-only, actual FK completion, planNextWaypoint commit order, have_joint_state_, full CR10 FK chain)  
**Status:** Revised — awaiting re-approval before implementation planning  
**Scope:** 在当前 Ranger+CR10 REMANI 仿真上，增加「RViz 6D Marker → 单个 EE 6D Pose → 求一个 whole-body terminal → 现有 REMANI 规划执行 → 用实际 odom+关节 FK 验证末端是否到位」。不改 Hybrid A* / RRT / 多项式优化 / time scaling，不改 2D Nav Goal 语义。

## 0. Review Delta

相对初稿必须改掉的错误与语义：

1. Whole-body IK 变量是 **9 维** `[x, y, yaw, q0..q5]`，Jacobian 是 **6×9**。现有 REMANI trajectory state 仍是 **8 维 `end_pt` + 单独 `end_yaw`**，本功能不得改 trajectory representation。
2. `/ee_goal` **只在 `WAIT_TARGET` 接受**。V1 不做执行中抢占。
3. `remani_planner_node` 当前是单线程 `ros::spin()`。Marker 独立节点只保护 RViz UI，不让 planner 内部并行。V1 不改成 `AsyncSpinner`。
4. EE 任务 two-phase commit：IK 失败或 `planNextWaypoint` 失败都不得 commit active EE state；EE-specific active state **仅在** `planNextWaypoint()` 成功接受 terminal 后才设置。
5. EE readiness 需要 `have_odom_ && have_joint_state_`（当前源码只有 `have_odom_`）。
6. `getEePose()` / `getEeJacobian()` 必须用完整 CR10 `q0..q5` 的 `getAJointTran` 链；不得未经验证直接使用 `getJointTMat()`。
7. EE 任务完成必须用**实际** `mm_state_pos_` + `mm_car_yaw_` 再 FK，不得把构型 `reach goal` 当成末端到位。
8. IK 门闩与执行到位门闩拆开：`ee_ik_*` vs `ee_reach_*`。
9. 误差与 Jacobian 统一 **world / spatial** convention。
10. Stage B 用 **weighted DLS**（冗余 3 DoF），不是普通 minimum-norm DLS。
11. 关节限位：硬 clipping + 软 margin 打分；限位数据只从 `MMConfig` 读。
12. Stage B→C：hard-invalid 走 C；legal-but-poor 保留 B 再跑 C，**共用打分**。
13. Clearance **不得**用现有 `check*Collision` 无碰时写回的 `min_dist`（那是 `safe_dist` 阈值，不是真实最小 ESDF）。
14. Stage C 先 cheap filter + cheap score 排序，再跑臂 IK，避免 `c_max_ms` 造成方向偏置。
15. TCP 在 `mm/ee_tcp_xyz_rpy`，不在 `fsm/`。
16. Marker 节点不维护第二套 FK；planner 发布 `/ee_current_pose`（需 odom+joint ready）。

## 1. Goals and Non-Goals

### Goals

- 仿真中用 RViz Interactive Marker 指定一次世界系 6D 末端位姿，规划一次，执行后用实际状态 FK 验证末端是否到位。
- 底盘可以动。Stage B：当前整机附近加权 whole-body IK；hard-invalid 或质量差则 Stage C：目标附近采样 Ranger 底盘 + CR10 臂 IK，硬过滤后按软代价选一个 terminal。
- 只服务**单个终点**。不把末端位姿当笛卡尔轨迹，不做执行中笛卡尔伺服。
- 现有 2D Nav Goal 保留且语义不变。`target_type=1` 时：点 2D Goal 只改底盘并保持当前臂角；仅在 `WAIT_TARGET` 时拖 EE Marker 走末端位姿。
- 末端默认是 **CR10 法兰盘**。TCP 偏置在 `mm/ee_tcp_xyz_rpy`，默认单位阵。

### Non-Goals

- 多段末端路点、笛卡尔直线/圆弧、阻抗/伺服、continuous Cartesian tracking。
- EE goal preemption、corrective IK / corrective replan。
- 把 `ros::spin()` 改成 `AsyncSpinner`，或引入 background IK worker / 多线程 GridMap。
- 解析 UR 式 8 解 IK、逆可达性数据库、离线 IRM。
- FastArmer / UR5 的 EE 目标（`manipulator_type != cr10` 时拒绝 `/ee_goal`）。
- 把 `target_type=2` 预设构型路点改成末端位姿。
- 改规划后端、轮速硬限制、time scaling、FSM 对 2D/预设任务的重试上限。
- 把 EE task-space constraint 加入 MINCO / L-BFGS。
- 实机 / Ranger 四轮转向；TopAY `/ee_goal`（另一条栈）。

## 2. Architecture

### 2.1 两套状态维数（不得混淆）

| 层 | 变量 | 维数 | 用途 |
|----|------|------|------|
| Whole-body IK | `ξ = [x, y, yaw, q0, q1, q2, q3, q4, q5]` | **R^9** | 数值 IK / Jacobian |
| 现有 REMANI trajectory | `end_pt = [x, y, q0..q5]`，`end_yaw` 单独 | **R^8 + scalar** | `planNextWaypoint` 及之后整条规划 |

当前代码依据（不得为本功能改掉）：

- `traj_dim_ = mobile_base_dim_ + manipulator_dim_` → Ranger+CR10 为 `2+6=8`
- `end_pt_` 是 goal 的 8 维构型；`end_yaw_` 单独
- `planNextWaypoint(const Eigen::VectorXd next_wp, const double next_yaw)`
- `mm_state_pos_` 是 8 维实际构型；底盘 yaw 在 `mm_car_yaw_`

**转换（Architecture 与 IK 共用，唯一出口）：**

```text
IK result ξ* = [x, y, yaw, q0..q5]     ∈ R^9
        ↓
end_pt  = [x, y, q0..q5]               ∈ R^8
end_yaw = yaw
        ↓
planNextWaypoint(end_pt, end_yaw)
```

不要为了 EE Goal 把 trajectory state 扩成 9 维，也不要把 yaw 塞进 `end_pt`。

### 2.2 数据流

```text
RViz 2D Nav Goal
  --> /move_base_simple/goal
  --> waypointCallback                 [现有语义，本 spec 不改]
  --> 清 active EE goal state
  --> planNextWaypoint(end_pt, end_yaw)
  --> Hybrid A* / RRT / 优化 / time scaling / EXEC_TRAJ
  --> 现有构型空间 "reach goal"

RViz EE Interactive Marker (MOUSE_UP)
        ↓
      /ee_goal
        ↓
   WAIT_TARGET ?
        ↓ YES
 robot state ready ?
 (have_odom_ && have_joint_state_)
        ↓ YES
 pending EE goal + pending T_world_ee_goal
        ↓
 Stage B / C  →  ξ* = [x,y,yaw,q0..q5]
        ↓
 construct local only:
   candidate_end_pt  = [x,y,q0..q5]
   candidate_end_yaw = yaw
        ↓
 resetFailureCount()
        ↓
 planNextWaypoint(candidate_end_pt, candidate_end_yaw)
        ↓
 success ?
 ├─ NO  → clear pending; active_goal_source_=NONE;
 │        active_ee_goal_=false; remain WAIT_TARGET
 │        (不进入 GEN_NEW_TRAJ)
 └─ YES → planNextWaypoint 已写入 end_pt_/end_yaw_/have_target_
          commit EE metadata:
            active_goal_source_ = EE_POSE
            active_ee_goal_ = true
            T_world_ee_goal_ = pending
            pending_ee_goal_ = false
        ↓
 existing REMANI lifecycle
        ↓
 WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ
        ↓
 actual odom + actual joints (complete CR10 FK q0..q5)
        ↓
 actual EE pose tolerance check
        ↓
 REACHED / FAILED
```

**EE-specific active state is committed only after `planNextWaypoint()` successfully accepts the generated terminal configuration.** IK success alone is **not** EE task committed.

当前 `planNextWaypoint()` 内部先调 `planGlobalTrajWaypoints(...)`，**仅**在 `success==true` 时设置 `end_pt_`、`end_yaw_`、`have_target_`、`have_new_target_`。EE callback **不得**在调用 `planNextWaypoint()` 之前写这些字段，也不得重复实现 `planNextWaypoint` 已负责的状态写入。

Planner 另发 `/ee_current_pose`（`geometry_msgs/PoseStamped`，frame=`world`），仅当 `have_odom_ && have_joint_state_` 时用 `MMConfig::getEePose`（完整 CR10 六轴链）计算。Marker 节点只订阅该话题做 UI，第一帧 `/ee_current_pose` 到达前不自行猜 flange pose。

### 2.3 模块

| 单元 | 进程 | 职责 | 接口 |
|------|------|------|------|
| `ee_goal_marker_node` | 独立节点 | 6DOF Interactive Marker；MOUSE_UP 发 `/ee_goal`；Reset 跟 `/ee_current_pose` | **禁止**依赖 `MMConfig` / GridMap / CR10 FK |
| `WholeBodyIkSolver` | `remani_planner_node` | Stage B→C，输出一个碰撞合法 `ξ*` | `bool solve(ξ_start, T_ee, ξ_out)`，`ξ ∈ R^9` |
| `MMConfig` 增补 | 同上 | 世界系 EE FK、6×9 Jacobian、TCP、关节限位 getter、真实 obstacle clearance | 见第 9 节 |
| `REMANIReplanFSM` | 同上 | `/ee_goal` 生命周期、goal-source、实际 EE reach、`/ee_current_pose` | 见第 3、5 节 |

**进程划分：** Marker 独立节点只保证 **RViz Interactive Marker UI** 不会因 planner 内同步 IK（最坏约 40+150 ms）而卡死。它**不能**让 `remani_planner_node` 的 callback/timer 并行。

当前 `remani_planner_node.cpp` 是：

```text
remani_replan.init(nh);
ros::spin();
```

FSM `exec_timer_` 与 `safety_timer_` 均为 0.01 s（100 Hz），与 odom / joint / waypoint subscriber 同线程。若在 `EXEC_TRAJ` 的 subscriber 里跑 Stage B/C，会阻塞这些回调，并有 stale-start state 风险。因此：

- V1 允许 `WholeBodyIkSolver` **同步**运行，但**仅**在 `WAIT_TARGET`（此时没有活动轨迹）。
- **不**把 `ros::spin()` 改成 `AsyncSpinner`。
- **不**引入 background IK worker 或多线程 GridMap / FSM 同步。这些留 Future Work。

**坐标系：** 全部 `world`，与 `grid_map/frame_id`、RViz Fixed Frame、现有碰撞一致。`/ee_goal` 的 `header.frame_id` 必须是 `world`。

## 3. 输入与 FSM

### 3.1 Goal source

最小状态，避免 EE 完成逻辑作用到 2D/预设任务：

```text
enum GoalSource { NONE, NAV_2D, EE_POSE };
GoalSource active_goal_source_;
```

实现可用等价的 `bool` + 姿态缓存，但语义必须能区分这三种。建议另存：

- `bool pending_ee_goal_`
- `bool active_ee_goal_`（当且仅当 `active_goal_source_ == EE_POSE`）
- `T_world_ee_goal_` 或 `(goal_pos, goal_quat)`（class 已有 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`）

| 模式 | 2D Nav Goal | `/ee_goal` |
|------|-------------|------------|
| `target_type=1` 且 `WAIT_TARGET` 且 robot ready | 现有语义；合法接受后 `active_goal_source_=NAV_2D`；清 EE active/pending | Stage B→C；`planNextWaypoint` 成功后才 `EE_POSE` |
| `target_type=1` 且非 `WAIT_TARGET` | **保持现有 2D 行为不改** | WARN + ignore，不 IK |
| `target_type=2` | 仍作 preset trigger | WARN + ignore EE；清 stale EE state |

**Goal source 设置时机（生命周期）：**

| `active_goal_source_` | 何时设置 | 何时清回 `NONE` |
|----------------------|----------|-----------------|
| `NAV_2D` | 现有合法 2D Goal 被 `waypointCallback` 接受时 | 任务完成 / abort（现有逻辑） |
| `EE_POSE` | **仅当** `planNextWaypoint(candidate_terminal)` 返回 `true` 之后 | EE IK 失败；`planNextWaypoint` 失败；EE 执行 FK 通过/未通过；收到新 2D/preset 时清 stale EE |
| `NONE` | 默认；上述清 EE 后 | — |

**禁止：** 收到 Marker 时设 `EE_POSE`；IK success 时设 `EE_POSE`。Preset 模式不得继承 stale EE source。

收到**已接受**的新 2D Nav Goal：必须 `active_goal_source_=NAV_2D`，清 `active_ee_goal_` / `pending_ee_goal_` / `T_world_ee_goal_`，避免上一次 EE 的 FK completion 作用到新的 2D 轨迹。Preset trigger 同理，不得继承 stale EE state。

### 3.2 `/ee_goal` 只在 WAIT_TARGET 接受

V1 **不支持**执行中 Marker 抢占。

`exec_state_ != WAIT_TARGET` 时收到 `/ee_goal`（含 `GEN_NEW_TRAJ`、`REPLAN_TRAJ`、`EXEC_TRAJ`、`EMERGENCY_STOP`、`INIT`）：

- `ROS_WARN("[EE GOAL] ignore new marker goal: FSM state is %s", ...)`
- **不**覆盖当前任务
- **不** reset 当前 trajectory
- **不**修改 `have_target_`
- **不**触发 IK
- **不**进入新的 planning

此限制**只针对**新增的 `/ee_goal`。不要修改当前 2D Nav Goal 的既有行为（今日 `waypointCallback` 在执行中点新目标仍会 `resetFailureCount` 并 `planNextWaypoint`）。

### 3.2.1 EE-specific robot state readiness

当前源码（`remani_replan_fsm.cpp`）只有 `have_odom_`；`mmManiOdomCallback()` 在 `position.size() < manipulator_dim_` 时 WARN 并 return，但**没有** `have_joint_state_` 标志。

本功能新增：

```text
bool have_joint_state_{false};
```

在 `mmManiOdomCallback()` 中，保留现有 short JointState guard；**仅当**成功更新全部 `manipulator_dim_` 个 joint position 后：

```text
valid JointState received
  → update mm_state_pos_.tail(manipulator_dim_)
  → have_joint_state_ = true
```

EE readiness gate（**只针对 EE 功能**，不改整个 FSM 的 INIT 行为）：

```text
/ee_goal 允许进入 IK 当且仅当：
  exec_state_ == WAIT_TARGET
  AND have_odom_ == true
  AND have_joint_state_ == true
```

否则：

```text
[EE GOAL] ignore: robot state not ready (odom_ready=... joint_ready=...)
return;   # 不 IK，remain WAIT_TARGET
```

`/ee_current_pose` 同样要求 `have_odom_ && have_joint_state_` 后才计算并发布。Marker 节点在第一帧 `/ee_current_pose` 之前不自行猜 flange pose（默认不可交互或等待 first pose；具体 UI 实现计划再定）。

2D Nav Goal 现有初始化语义尽量不变；此 gate **不**强加给 `waypointCallback`（除非实现阶段发现必须，本 spec 不要求改 2D）。

### 3.3 Two-phase commit

```text
收到 /ee_goal
且 exec_state_ == WAIT_TARGET
且 have_odom_ && have_joint_state_
        ↓
pending_ee_goal_ = true
保存 pending T_world_ee_goal_
（不改 active、不改 end_pt_/end_yaw_/have_target_/active_goal_source_）
        ↓
Stage B / Stage C
        ↓
IK 失败 → 只清 pending；remain WAIT_TARGET；不调用 planNextWaypoint
        ↓
IK terminal ξ* 成功
        ↓
仅构造局部变量：
  candidate_end_pt  = [x, y, q0..q5]
  candidate_end_yaw = yaw
        ↓
planner_manager_->resetFailureCount()
        ↓
bool success = planNextWaypoint(candidate_end_pt, candidate_end_yaw)
        ↓
success == false ?
├─ YES
│    ↓
│  clear pending
│  active_goal_source_ 保持 NONE
│  active_ee_goal_ = false
│  不写 active EE target
│  have_target_ 仍为 false（planNextWaypoint 未成功）
│  remain WAIT_TARGET
│  不进入 GEN_NEW_TRAJ
│
└─ NO
     ↓
  此时 planNextWaypoint 已成功设置：
  end_pt_, end_yaw_, have_target_, have_new_target_
     ↓
  再 commit EE-specific metadata：
  active_goal_source_ = EE_POSE
  active_ee_goal_ = true
  T_world_ee_goal_ = pending goal
  pending_ee_goal_ = false
     ↓
  现有 FSM：WAIT_TARGET 见 have_target_ → GEN_NEW_TRAJ → ...
```

**定义：**

- `IK success` ≠ `EE task committed`
- 真正 commit 条件 = **IK success AND `planNextWaypoint` success**
- **EE-specific active state is committed only after `planNextWaypoint()` successfully accepts the generated terminal configuration.**

EE callback **不要**在 `planNextWaypoint()` 之前写 `end_pt_` / `end_yaw_` / `have_target_` / `active_ee_goal_` / `active_goal_source_=EE_POSE`。

### 3.4 Marker 触发

- 控件：`interactive_markers` 6DOF（3 平移 + 3 旋转）。
- **只在 MOUSE_UP 发布 `/ee_goal`。** 拖动中的 `POSE_UPDATE` 只更新显示。
- 启动 / 「Reset to current EE」：收到 `/ee_current_pose` 后把 Marker 放到该位姿，不规划。
- 规划成功后 Marker 留在用户指定位姿。
- 右键「Plan to this pose」= 发一次 `/ee_goal`（仍受 WAIT_TARGET 门闩约束，门闩在 planner 侧）。

Marker 节点订阅 `/ee_current_pose`，**不**订阅 raw odom/joint 自己做 FK。

## 4. 末端帧、FK、Jacobian

### 4.1 运动学链

与 `test_cr10_fk.cpp` 已验证 reference 一致，**完整 CR10 六关节**全部参与 flange/TCP FK：

```text
T = T_car(x, y, yaw)
T = T * T_q_0_
for i = 0 .. 5:
    T = T * getAJointTran(i, q[i], T_i, T_grad_i)
T = T * T_tcp

即：
T_world_tcp =
  T_car * T_q_0_ * T_j0 * T_j1 * T_j2 * T_j3 * T_j4 * T_j5 * T_tcp
```

- `T_car`：现有 `CarState2T`（平面 yaw，平移 z=0）。
- `T_q_0_`：现有 `base_mani_fixed_joint_xyz_ypr`（Ranger 上 CR10 安装）。
- `T_ji`：**必须**用 `getAJointTran(i, q(i), ...)`，`i = 0..5` 全部循环（CR10：`T_fixed * RotZ(θ)`）。与 `test_cr10_fk.cpp` 中 `for (i = 0; i < 6; ++i) { getAJointTran(...); T_mm = T_mm * Ti; }` 相同。
- `T_tcp`：`mm/ee_tcp_xyz_rpy`，默认 `[0,0,0,0,0,0]` → 单位阵。

**禁止（V1）：** 未经验证直接把 `getJointTMat()` 当作完整 flange FK。当前 `getJointTMat()` 先 push `Identity`，再只对 `i = 0 .. manipulator_dof_-2` 调用 `getAJointTran`，**并不显然等价**于完整 `q0..q5` 法兰链。除非将来单独证明等价，否则 `getEePose()` / `getEeJacobian()` / 有限差分 reference **必须**走上述 `getAJointTran` 六轴链。

**链一致性（硬约束）：**

```text
Pose chain  ==  Jacobian chain  ==  finite-difference reference chain
```

默认 TCP 为单位阵时，`T_world_tcp = T_world_flange`。

`getEePose(car_xyyaw, q_arm) → T_ee` 与 `getEeJacobian(...) → 6×9` 必须对**同一条**六轴链。

### 4.2 World / spatial 误差与 Jacobian（三者必须一致）

全部使用 **WORLD / spatial** convention，禁止 position 在 world、orientation 在 body、Jacobian 在第三套 frame。

```text
e_p = p_des - p_now                         ∈ R^3   (world)
e_R = Log( R_des * R_now^T )                ∈ R^3   (world axis-angle)
e   = [e_p; e_R]                            ∈ R^6
```

`Log(R)` 取主值，角度 ∈ `[-π, π]`。`||e_p||` 与 `||e_R||` 分别对应位置门闩与姿态门闩。

`getEeJacobian()` 输出 `J ∈ R^{6×9}`，行是 world 线速度 / world 角速度，列顺序：

```text
[ ∂/∂x, ∂/∂y, ∂/∂yaw, ∂/∂q0, ∂/∂q1, ∂/∂q2, ∂/∂q3, ∂/∂q4, ∂/∂q5 ]
```

臂部列基于与 `getEePose` **相同**的 `getAJointTran(i=0..5)` 链，用 `T_joint_grad` 链式法则变到 world；底盘三列对 `T_car` 解析求导，同样变到 world。

有限差分门闩必须用同一套 `e`：

```text
J_fd[:, i] ≈ e(ξ + ε e_i) 的差分
  线部分: (p(ξ+ε) - p(ξ)) / ε
  角部分: Log( R(ξ+ε) * R(ξ)^T ) / ε
```

DLS 一律解 `J Δξ ≈ e`（符号与 `e = e_des - e_now` 一致，使一步更新减小误差）。

## 5. Whole-body IK

`ξ ∈ R^9`。任务空间 `e ∈ R^6`。冗余 3 DoF。

### 5.1 关节限位 source of truth

`WholeBodyIkSolver` **不得**再从 YAML 读 `mm/manipulator_min_pos` / `max_pos`。当前它们是 `MMConfig` 私有成员 `manipulator_min_pos_` / `manipulator_max_pos_`，**没有 getter**。本功能给 `MMConfig` 增加只读 getter（名字按现有 `getManiDof` 风格，例如）：

- `getManipulatorMinPos() const → const Eigen::VectorXd&`
- `getManipulatorMaxPos() const → const Eigen::VectorXd&`

硬约束：全程 `qmin ≤ q ≤ qmax`（投影/拒绝）。软打分见 5.5。

### 5.2 Stage B — weighted whole-body IK + 信任域

目的：在当前整机附近找满足 `T_ee` 的 9 维解，正则化倾向少动底盘，但 **EE 门闩是硬条件**，不得用更大末端误差换更少移动。

- 初值：当前 `ξ`（`[mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_, mm_state_pos_.tail(6)]`）。
- 问题（加权阻尼最小二乘 / 等价 regularized LS），实现可选等价 closed-form，不必本 spec 锁死公式：

```text
min_Δξ  ||J Δξ - e||²  +  λ² ||W Δξ||²

W = diag(w_bx, w_by, w_byaw, w_q0, …, w_q5)
```

YAML 提供三档权重（xy 共用、关节共用）：`ee_ik_b_weight_xy` / `ee_ik_b_weight_yaw` / `ee_ik_b_weight_joint`。初值为仿真猜测，验证阶段再调。当前臂能完成时，更大的 base 权重应让优化器倾向不动底盘；需要协同时底盘仍可在信任域内动。

- **信任域（硬投影）：**
  - `|x-x0| ≤ ee_ik_local_xy`（默认 0.40 m）
  - `|y-y0| ≤ ee_ik_local_xy`
  - `|wrap(yaw-yaw0)| ≤ ee_ik_local_yaw_deg`（默认 20°）
  - `q` 投影到 `getManipulatorMinPos/MaxPos`
- 底盘必须在地图内。
- 最多 `ee_ik_b_max_iters`（60）或 `ee_ik_b_max_ms`（40 ms）。

**IK 硬门闩（B 与 C 共用）：**

- `||e_p|| < ee_ik_pos_tol`（默认 0.01 m）
- `||e_R|| < ee_ik_rot_tol_deg`（默认 2°）
- 关节不越限
- `checkcollision(car, q, /*safe=*/true)` 为无碰（**不改**现有 collision API / `safe=true` 行为）

### 5.3 Stage B → C 判定（统一，无矛盾）

```text
Stage B
  ↓
hard-invalid?
  ├─ YES → 丢弃 B，进入 Stage C
  └─ NO
       ↓
     quality good?（fast-path）
       ├─ YES → 直接采用 B，不跑 C
       └─ NO  → 保留 B candidate + 跑 Stage C
                 → B∪C 合法解共用 ranking → 取 cost 最小
```

**Hard-invalid** 任一即是：

- 未收敛 / 超时且未达 IK 门闩
- `||e_p||` 或 `||e_R||` 超 IK 门闩
- 底盘出图
- 关节越限（投影后仍不满足位姿门闩视为 invalid）
- `checkcollision(..., true)` 碰撞

**Soft quality poor**（hard 已过，但不够好，需与 C 比）任一即是：

- `min_i m_i < ee_ik_b_accept_joint_margin`
- `d_clear < ee_ik_b_accept_clearance`
- `||Δp_base|| > ee_ik_b_accept_max_xy`
- `|Δyaw| > ee_ik_b_accept_max_yaw_deg`
- `||Δq||_2 > ee_ik_b_accept_max_dq`

`m_i = min(q_i - qmin_i, qmax_i - q_i)`。上述 accept 阈值为仿真初值，验证阶段可调。

### 5.4 Stage C — 底盘候选 + 臂 IK（无方向超时偏置）

仅当 B hard-invalid，或 B legal-but-poor 时运行。只产单个终点，不产底盘路径。

**1. 生成全部底盘候选（heuristic，不是硬约束）：**

```text
for r in ee_ik_c_radii:          # 默认 {0.50, 0.75, 1.00, 1.25} m
  for k in 0 .. (yaw_bins-1):    # 默认 12 → 30°
    θ = k * 2π / yaw_bins
    for Δ in ee_ik_c_yaw_offsets_deg:   # 默认 {0, -15, 15}
      x = p_ee.x - r * cos(θ)
      y = p_ee.y - r * sin(θ)
      yaw = θ + Δ
```

`yaw = θ` 表示 Ranger +x 大致指向 EE 水平投影，这是 **candidate-generation heuristic**，不是唯一允许的 yaw。`±15°` offset 避免「车头正对目标但腕部姿态极差」。默认 offset 实现阶段可测完再改。

并入 **当前底盘位姿** 作为一个优先候选（即使超出 B 信任域）。

**2. Cheap filter（不做臂 IK）：** 出图、`checkCarObsCollision` 碰撞的丢掉。

**3. Cheap lower-bound score（仍不做臂 IK），从好到坏排序：** 例如 `w_xy * ||Δp_base|| + w_yaw * |Δyaw|`。当前底盘候选排在最前。

**4. 按该顺序逐个做 6-DoF 臂 IK**（底盘固定，world 误差与 6×6 Jacobian 同第 4.2 节 convention）。种子：当前臂角；再 `(qmin+qmax)/2`。投影到限位。每候选最多 `ee_ik_c_arm_iters` / `ee_ik_c_arm_max_ms`。

**5.** 位姿 IK 门闩 + 整机 `checkcollision(..., true)` 通过才进入合法池。

**6.** 到达 `ee_ik_c_max_ms`（默认 150 ms）停止。最终 ranking 只在**已经解完**的合法候选（含保留的 B 解）上进行。超时 ≠ 「只检查了世界系某几个固定 θ」。

### 5.5 真实 obstacle clearance（新只读查询）

现有 `checkCarObsCollision` / `checkManiObsCollision` 在**无碰**时把 `min_dist` 设成 `safe_dist`（阈值），不是扫描到的真实最小 ESDF。见 `mm_config.cpp`：无碰分支 `min_dist = safe_dist; return false;`。因此 **不能**用于 ranking。

新增只读查询（名字实现时可按代码风格微调），例如：

- `getCarMinObstacleDistance(car_state) → double`
- `getManiMinObstacleDistance(car_state, q) → double`
- 或合一 `getWholeBodyObstacleClearance(car_state, q) → double`（取车、臂球体 ESDF 的真实最小）

约束：

1. 不改变现有 collision API。
2. 不改变现有 collision threshold。
3. 不改变 `safe=true` 行为。
4. 仅用于 IK candidate quality / ranking。
5. 返回实际检测到的最小 ESDF。自碰不计入该 `d`。

硬过滤仍必须 `checkcollision(car, q, true)`。clearance 只是软 ranking。

### 5.6 最终 ranking

合法 candidate **必须先**满足 IK 位姿硬门闩。`e_p` / `e_R` **不是**软 ranking 主项。禁止「末端误差更大但底盘少动」的解胜出。

```text
cost = w_xy    * ||Δp_base||
     + w_yaw   * |wrap(Δyaw)|
     + w_q     * ||Δq||_2
     + w_limit * joint_limit_penalty
     + w_gap   * max(0, gap_ref - d_clear)
```

`joint_limit_penalty`：对每个关节 `max(0, m_req - m_i)` 的平方和（或等价归一化惩罚），`m_req` 为期望余量（仿真初值）。`d_clear` 来自第 5.5 节新查询。

权重 YAML 见第 8 节；无标定依据的标为 **initial simulation value / tune in validation**。

日志（每次被接受的 `/ee_goal` 必打）：

```text
[EE IK] stage=B|C|B+C  success|fail  pos_err=.. rot_err_deg=..
        ξ is 9-DoF; mapped to end_pt(8)+end_yaw
        base_xy=(..) yaw_deg=..  Δxy=.. Δyaw_deg=.. Δq=.. margin=.. clear=..
        candidates=n_ok/n_total  time_ms=..
        fail_reason=...
```

## 6. EE 任务完成：实际 FK 验证

现有 `EXEC_TRAJ` 终端判定是构型空间：

```text
t_cur > duration && (local_target_pt_ - end_pt_).norm() < 1e-2
→ cout "reach goal" → have_target_=false → WAIT_TARGET → planning/finish
```

这**不是**实测末端到位。该语义 **只保留给** `NAV_2D` 与 preset。

**仅当** `active_goal_source_ == EE_POSE`（`active_ee_goal_ == true`）时，在同样的「轨迹已到终端」时刻追加实际 FK：

```text
actual_base = [mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_]
actual_arm  = mm_state_pos_.tail(manipulator_dim_)   # q0..q5
T_world_ee_actual = MMConfig::getEePose(actual_base, actual_arm)  # 完整六轴链

pos_err = ||p_actual - p_goal||
rot_err = ||Log(R_goal * R_actual^T)||     # 与第 4.2 节同一 world convention
```

| 结果 | 行为 |
|------|------|
| `pos_err ≤ ee_reach_pos_tol` **且** `rot_err ≤ ee_reach_rot_tol` | `[EE GOAL] reached`；`have_target_=false`；`have_trigger_=false`；`active_ee_goal_=false`；`active_goal_source_=NONE`；`planning/finish=true`；`WAIT_TARGET` |
| 轨迹已结束但超门闩 | `[EE GOAL] execution finished but pose tolerance not met: pos_err=... rot_err_deg=...`；清 EE/target/trigger；`WAIT_TARGET`。**V1 不做** corrective IK / replan / Cartesian servo，**不**因此自动规划 10～20 次 |

执行到位门闩与 IK 门闩分开（仿真跟踪误差，初值可略宽，**不是**机械臂硬件精度）：

| 参数 | 默认初值 | 用途 |
|------|----------|------|
| `ee_ik_pos_tol` | 0.01 m | solver 硬门闩 |
| `ee_ik_rot_tol_deg` | 2.0° | solver 硬门闩 |
| `ee_reach_pos_tol` | 0.02 m | 实际 FK 完成检查 |
| `ee_reach_rot_tol_deg` | 4.0° | 实际 FK 完成检查 |

最终以测试调参为准。不要写「允许 2 倍门闩」这类模糊倍率。

`/ee_current_pose` 建议 10–20 Hz（在 100 Hz FSM timer 里节流，或 odom/joint 更新节流）。**仅** `have_odom_ && have_joint_state_` 时发布。用于 Marker 初始化/Reset，以及 RViz 对比 target vs actual。

## 7. Error handling

失败类型必须区分，不要统一叫 “plan failed”：

1. **EE terminal generation failure**（Stage B/C 无解）
2. **Global trajectory waypoint generation failure**（`planNextWaypoint` / `planGlobalTrajWaypoints` 失败）
3. **GEN_NEW_TRAJ / frontend-backend planning failure**（已有 Hybrid A* / RRT / optimizer 重试链）

| 情况 | 行为 |
|------|------|
| 无 odom（`have_odom_==false`） | `[EE GOAL] robot state not ready: odom missing`；ignore；`WAIT_TARGET` |
| 尚无合法 JointState（`have_joint_state_==false`） | `[EE GOAL] robot state not ready: joint state missing`；ignore；`WAIT_TARGET` |
| `JointState.position.size() < manipulator_dim_` | 现有 WARN+skip；`have_joint_state_` 保持 false；EE goal 不执行 |
| `frame_id != world` | WARN，reject |
| `manipulator_type != cr10` | ERROR，reject |
| `target_type == 2` | WARN，ignore EE；不继承 stale EE state |
| `exec_state_ != WAIT_TARGET` | `[EE GOAL] ignore new marker goal: FSM state is ...`；不 IK、不改轨迹 |
| B/C 均无合法 terminal | `[EE IK] no valid terminal configuration`；clear pending；`WAIT_TARGET`；**不**调用 `planNextWaypoint`；**不**进 Hybrid A* |
| **Case A：** IK terminal valid，但 `planNextWaypoint` 返回 false | `[EE GOAL] terminal found but global trajectory generation failed`；clear pending；`active_goal_source_=NONE`；`active_ee_goal_=false`；`have_target_` 仍为 false；**remain `WAIT_TARGET`**；**不**进入 `GEN_NEW_TRAJ`；**不**手工设 `have_target_=true` 去“继续尝试” |
| **Case B：** `planNextWaypoint` 成功，之后 `GEN_NEW_TRAJ` 内 Hybrid A* / RRT / backend 失败 | 走**现有** REMANI retry / abort / invalid terminal / RRT timeout 等。这是 **trajectory planning failure**，不是 IK failure |
| Stage B fail，Stage C success → `planNextWaypoint` 成功 | 正常进入现有 REMANI lifecycle |
| B legal-but-poor，C 有更好解 → `planNextWaypoint` 成功 | 共用 ranking 取优后进入现有 REMANI |
| 轨迹结束但实际 EE FK 超 `ee_reach_*` | `[EE GOAL] execution finished but pose tolerance not met`；清 target/trigger/EE active；`WAIT_TARGET`；V1 不 corrective replan |
| 执行中碰撞 / 超轮速 | 现有 replan / time scaling；与 2D 任务相同。期间新的 `/ee_goal` 仍 ignore |

## 8. Visualization

- Target：Interactive Marker 坐标系（轴长约 0.15 m）+ 半透明盒，表示用户指定的 `T_ee`。
- Actual：RViz 显示 `/ee_current_pose`（可用较小坐标轴）。最少能对比 **target flange/TCP vs actual flange/TCP**。
- IK 成功：现有 `visMM` / `getManiMarkerArray` 画半透明幽灵整机（ns `ee_ik_terminal`）。新的已提交 EE 目标或已接受的 2D Goal 到来时 DELETE 旧幽灵。
- IK 失败：幽灵删除；Marker 保持用户位姿。
- 规划成功后的全局路径、后端 mesh：现有显示不改。避免视觉过载：target 轴、current 轴、一帧幽灵即可。

## 9. Configuration

TCP 是运动学，进 `mm_param_ranger_cr10.yaml`：

```yaml
mm:
  ee_tcp_xyz_rpy: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # identity → EE goal = CR10 flange
```

IK / reach / FSM 进 `remani_planner_param.yaml` 的 `fsm:`：

```yaml
fsm:
  ee_goal_topic: /ee_goal
  ee_current_pose_topic: /ee_current_pose

  # IK hard gate (solver). Simulation initial values; tune in validation.
  ee_ik_pos_tol: 0.01          # m
  ee_ik_rot_tol_deg: 2.0

  # Actual FK reach gate after EXEC. Wider than IK; not hardware accuracy.
  ee_reach_pos_tol: 0.02       # m
  ee_reach_rot_tol_deg: 4.0

  ee_ik_local_xy: 0.40         # m, Stage B trust region
  ee_ik_local_yaw_deg: 20.0
  ee_ik_b_max_iters: 60
  ee_ik_b_max_ms: 40.0
  # Stage B regularization (larger → more penalty). Initial simulation values.
  ee_ik_b_weight_xy: 5.0
  ee_ik_b_weight_yaw: 2.0
  ee_ik_b_weight_joint: 1.0

  # Fast-path accept (hard-valid B skips C). Initial simulation values.
  ee_ik_b_accept_joint_margin: 0.15   # rad
  ee_ik_b_accept_clearance: 0.20      # m
  ee_ik_b_accept_max_xy: 0.15         # m
  ee_ik_b_accept_max_yaw_deg: 10.0
  ee_ik_b_accept_max_dq: 0.50         # rad, ||Δq||_2

  ee_ik_c_radii: [0.50, 0.75, 1.00, 1.25]
  ee_ik_c_yaw_bins: 12
  ee_ik_c_yaw_offsets_deg: [0.0, -15.0, 15.0]  # heuristic, not unique yaw
  ee_ik_c_arm_iters: 40
  ee_ik_c_arm_max_ms: 5.0
  ee_ik_c_max_ms: 150.0

  # Soft ranking weights. Initial simulation values / tune in validation.
  ee_ik_w_xy: 1.0
  ee_ik_w_yaw: 0.3
  ee_ik_w_q: 0.1
  ee_ik_w_limit: 1.0
  ee_ik_w_gap: 2.0
  ee_ik_gap_ref: 0.20
  ee_ik_joint_margin_req: 0.087   # rad (~5 deg), for joint_limit_penalty
```

Launch：只在 `robot_model == ranger_cr10` **且** `use_rviz == true` 时启动 `ee_goal_marker_node`（改 `remani_sim.launch` 里已有的 ranger+rviz group）。RViz 配置文件是现有 `plan_manage/launch/remani_ranger_cr10.rviz`（**不是**不存在的 launch 名）。仿真入口：

- 包装：`remani_ranger_cr10_sim.launch` → include `remani_sim.launch`（`robot_model:=ranger_cr10`，默认 `target_type:=1`）
- 或直接 `remani_sim.launch`

`advanced_param_sim.xml` 负责起 `remani_planner_node`；EE 参数走 yaml rosparam，不必为每个 IK 参数加 launch arg。

## 10. 文件与改动边界

路径均相对 `remani_planner/src/REMANI-Planner/remani_planner/`。

### mm_config

| 文件 | 改动 |
|------|------|
| `include/mm_config/mm_config.hpp` | `getEePose`（`getAJointTran` 六轴链）、`getEeJacobian`（6×9，同链）、TCP API、`getManipulatorMinPos/MaxPos`、真实 clearance 查询 |
| `src/mm_config.cpp` | 对应实现；**禁止**用未验证的 `getJointTMat()` 作完整 EE FK；**不改**现有 `checkcollision` 语义 |
| `include/mm_config/whole_body_ik.hpp` + `src/whole_body_ik.cpp` | `WholeBodyIkSolver`（ξ∈R^9，B→C） |
| `src/test_ee_pose_ik.cpp` | 第 11 节离线测例（风格对齐现有 `test_cr10_fk.cpp`） |
| `CMakeLists.txt` | `whole_body_ik.cpp` 加入 `mm_config` library；增加 `test_ee_pose_ik` executable |
| `package.xml` | 若无新 ROS 依赖则 **不**加。FK/IK 只用现有 Eigen + `plan_env` |

### plan_manage

| 文件 | 改动 |
|------|------|
| `include/plan_manage/remani_replan_fsm.h` | `bool have_joint_state_`；EE subscriber；pending/active EE metadata；`GoalSource`；实际 EE reach helper；`/ee_current_pose` publisher |
| `src/remani_replan_fsm.cpp` | `mmManiOdomCallback` 成功读满 joint 后 `have_joint_state_=true`；EE callback 要求 `WAIT_TARGET && have_odom_ && have_joint_state_`；two-phase commit（commit 在 `planNextWaypoint` 成功之后）；FK completion |
| `src/ee_goal_marker_node.cpp` | Interactive Marker UI only |
| `CMakeLists.txt` | marker executable；`interactive_markers` |
| `package.xml` | 增加 **真实需要**的 `interactive_markers`（及 marker 节点实际用到的 `visualization_msgs` / `geometry_msgs`）。不要为未使用项加 depend |
| `config/mm_param_ranger_cr10.yaml` | `mm/ee_tcp_xyz_rpy` |
| `config/remani_planner_param.yaml` | 第 9 节 `fsm/ee_*` |
| `launch/remani_sim.launch` | ranger_cr10 ∧ use_rviz 时起 marker 节点 |
| `launch/remani_ranger_cr10.rviz` | InteractiveMarkers display；可选 Pose display 订 `/ee_current_pose` |

`remani_ranger_cr10_sim.launch` 只是 thin wrapper，一般不必改（参数已由 `remani_sim.launch` 传入）。`remani_planner_node.cpp` **不改** `ros::spin()`。

**禁止改：** `kino_astar.cpp` 搜索核、`rrt.cpp` 搜索核、`poly_traj_optimizer.cpp` 代价与 time scaling、2D `waypointCallback` 主语义、轮速硬限制、现有 collision 阈值。

## 11. Testing

离线测例在 `mm_config` 中注入 Ranger+CR10 参数（对齐 `test_cr10_fk.cpp`）。仿真入口：`remani_ranger_cr10_sim.launch`（或 `remani_sim.launch robot_model:=ranger_cr10`），`target_type:=1`。

| ID | 内容 | 期望 |
|----|------|------|
| 1 Whole-body FK | 随机 base `x,y,yaw` + 随机 `q0..q5`；`getEePose` 使用**全部六个** `getAJointTran(i)` | 与 `test_cr10_fk` 同量级误差 |
| 1b FK q5 sensitivity | 固定其它关节，**只扰动 q5** | flange/TCP orientation 发生正确变化（抓「漏最后一个 joint」） |
| 2 Jacobian 6×9 FD | 分别扰动 `x,y,yaw,q0..q5` 共 9 列 | 与第 4.2 节 world FD 一致 |
| 2b Jacobian q5 column | `∂EE/∂q5` 为最后一列 | 与 FD 一致；**不得**为零列或漏列 |
| 3 Stage B near | 当前 EE 附近 5 cm / 5° | B success 且 fast-accept；底盘移动小 |
| 4 B trust region → C | 目标需移动底盘超过 `local_xy` | B hard-invalid 或 poor；C success |
| 5 C orientation | 同位置、不同 EE 姿态 | 必须解姿态，不能只解 position |
| 6 Joint-limit quality | 两合法 IK，其一贴限位、其一余量大 | ranking 选 margin 更好者 |
| 7 Collision | IK 几何可达但 `checkcollision` 碰 | reject |
| 8 Unreachable | 明显超出 workspace / 地图 | B/C 超时受控；`WAIT_TARGET` |
| 9 2D Nav Goal regression | 与现在相同 | 臂角保持；无 EE FK completion |
| 10 Marker while EXEC | `EXEC_TRAJ` 中 MOUSE_UP | ignore；轨迹不变 |
| 11 Actual EE reach PASS | 执行结束，实际 odom+q 完整 FK 满足 `ee_reach_*` | `[EE GOAL] reached` |
| 12 Actual EE reach FAIL | 时间到但 FK 超限 | tolerance-not-met；非 `reached` |
| **R-A Readiness** | odom 已到，joint 未到，发 `/ee_goal` | WARN；不 IK；`WAIT_TARGET` |
| **R-B Readiness** | `JointState.position.size() < manipulator_dim_` | `have_joint_state_` 仍为 false；EE 不执行 |
| **R-C Readiness** | 合法 JointState 到达后 | `have_joint_state_=true`；EE 可正常求解 |
| **T Two-phase** | IK 成功但 `planGlobalTrajWaypoints` 人为失败 | `active_goal_source_!=EE_POSE`；`active_ee_goal_==false`；`have_target_==false`；FSM 保持 `WAIT_TARGET`；**绝不能** `WAIT_TARGET→GEN_NEW_TRAJ` |

Case 9 回归不得变差。Case 10 是 V1 抢占禁令的硬验收。

## 12. 明确取舍

1. V1 只处理 **single EE terminal pose**。
2. `/ee_goal` **只在 `WAIT_TARGET` 接受**。
3. V1 **不支持** EE goal preemption。
4. **不**把 `remani_planner_node` 改成 `AsyncSpinner`。
5. **不**引入 background IK worker。
6. **不做** continuous Cartesian tracking。
7. **不做** corrective Cartesian replan。
8. 实际 EE FK verification **只在任务结束**（现有 EXEC 终端时刻）执行。
9. 当前 2D Nav Goal 语义 **不改**。
10. 当前 Hybrid A* / RRT / optimizer / time scaling **不改**。
11. EE task-space constraint **不**加入 MINCO/L-BFGS。
12. Whole-body IK 是 **9 DoF**；REMANI terminal representation 仍是 **8D `end_pt` + `end_yaw`**。
13. TCP 默认 identity，当前目标表示 CR10 flange。
14. continuous observation / object scanning 留 Future Work。
15. 碰撞硬过滤用现有 `checkcollision`；clearance ranking 用新只读 ESDF 最小值查询。
16. Marker 节点不维护第二套 CR10 FK。
