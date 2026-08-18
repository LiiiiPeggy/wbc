# REMANI 末端 6D 位姿目标设计

**Date:** 2026-08-19  
**Status:** Draft — awaiting review  
**Scope:** 在当前 Ranger+CR10 REMANI 仿真上，增加「指定末端 6D 位姿 → 解单个整机终点构型 → 现有规划执行」。不改 Hybrid A* / RRT / 多项式优化 / time scaling。

## 1. Goals and Non-Goals

### Goals

- 仿真中用 RViz Interactive Marker 指定一次世界系 6D 末端位姿，规划一次，执行到位。
- 底盘可以动。先在当前整机附近求 whole-body IK（尽量少动）；质量不够再在目标附近采样 Ranger 底盘候选，对每个候选求 CR10 IK，过滤限位与碰撞，按代价选一个 terminal。
- 只服务**单个终点**。不把末端位姿当笛卡尔轨迹，不在执行中做笛卡尔伺服。
- 现有 2D Nav Goal 保留。`target_type=1` 时两个工具都能用：点 2D Goal 只改底盘并保持当前臂角；拖 EE Marker 走末端位姿。
- 末端默认是 **CR10 法兰盘**。TCP 偏置做成配置，默认单位阵，本阶段不强制标定夹爪 TCP。

### Non-Goals

- 多段末端路点、笛卡尔直线/圆弧、阻抗/伺服。
- 解析 UR 式 8 解 IK、逆可达性数据库、离线 IRM。
- FastArmer / UR5 的 EE 目标（本阶段 `manipulator_type != cr10` 时拒绝 `/ee_goal`）。
- `target_type=2` 预设构型路点改成末端位姿。
- 改规划后端、轮速硬限制、time scaling、FSM 重试上限。
- 实机 / Ranger 四轮转向。
- TopAY `/ee_goal`（那是另一条栈；本 spec 只改 REMANI）。

## 2. Architecture

逆解只发生在规划入口之前。成功则得到与今天 YAML 路点同构的 terminal：`end_pt = [x, y, q0..q5]`，`end_yaw`。之后调用现有 `planNextWaypoint`。

```text
RViz 2D Nav Goal
  --> /move_base_simple/goal (PoseStamped)
  --> waypointCallback          [语义不变]
  --> planNextWaypoint(end_pt, end_yaw)

RViz EE Interactive Marker (mouse-up)
  --> /ee_goal (PoseStamped, frame=world)
  --> eeGoalCallback
        WholeBodyIkSolver::solve(current_state, T_ee)
          Stage B: trust-region numerical whole-body IK
          Stage C: if B fails / poor  →  base samples + arm IK + rank
        fail → 立即 abort（不进入 GEN_NEW_TRAJ）
        ok   → planNextWaypoint(end_pt, end_yaw)   [同一入口]
  --> 现有 Hybrid A* / RRT / 优化 / time scaling / EXEC_TRAJ
```

| 单元 | 职责 | 接口 |
|------|------|------|
| `ee_goal_marker_node` | 6DOF Interactive Marker；仅在鼠标松开时发一次目标 | 订阅底盘 odom + 关节；发布 `/ee_goal`；广播 marker |
| `WholeBodyIkSolver` | Stage B→C，输出单个碰撞合法 terminal | `bool solve(start, T_ee, out_cfg)` |
| `MMConfig` 增补 | 世界系 EE FK / 6×8 Jacobian；TCP 偏置 | `getEePose`, `getEeJacobian` |
| `REMANIReplanFSM::eeGoalCallback` | 收 `/ee_goal`，调 solver，成功则走现有规划 | 与 `waypointCallback` 并列 |

**进程划分：** Marker 独立节点，避免规划阻塞时 Marker 反馈卡死。Solver 在 `remani_planner_node` 内，直接用已有 `MMConfig` + `GridMap`，不做跨进程碰撞查询。

**坐标系：** 全部 `world`，与现有 RViz Fixed Frame、碰撞检测、2D Nav Goal 一致。`/ee_goal` 的 `header.frame_id` 必须是 `world`；其它 frame 直接拒绝。

## 3. 输入与 FSM

### 3.1 双工具并存

不新增互斥的 `target_type=3`。行为按**消息来源**区分：

| 模式 | 2D Nav Goal | `/ee_goal` |
|------|-------------|------------|
| `target_type=1`（手动） | 现有语义：改 `xy/yaw`，臂角保持当前 | Stage B→C 逆解后规划 |
| `target_type=2`（预设路点） | 仍作「开始走 waypoint0」的 trigger | **忽略**，打 WARN |

执行中再点 2D Goal 或再拖 Marker：与今天手动目标相同，视为新任务——`resetFailureCount()`，用当前实际状态为起点重新 `planNextWaypoint`。

### 3.2 Marker 触发

- 控件：`interactive_markers` 6DOF（3 平移 + 3 旋转）。
- **只在 MOUSE_UP 发布 `/ee_goal`。** 拖动过程中的 `POSE_UPDATE` 只更新 Marker 显示，不规划。
- 启动后 Marker 放到当前法兰盘位姿（有 odom + 关节之后）。规划成功后 Marker 留在用户指定位姿；执行结束不强制吸回，避免和用户下一次拖动抢。
- 右键菜单提供同等操作：「Plan to this pose」= 发一次 `/ee_goal`；「Reset to current EE」= 把 Marker 移回当前法兰盘，不规划。

### 3.3 成功与到达判定

- 逆解成功后的执行到达判定**仍用现有构型空间** `reach goal`（底盘 xy/yaw + 关节）。terminal 是 IK 解，构型到达即末端到达。
- 本阶段不在 `EXEC_TRAJ` 里再查 EE 误差。

## 4. 末端帧与 FK

世界系末端位姿：

```text
T_ee = T_car(x, y, yaw) * T_q_0_ * T_j0 * T_j1 * T_j2 * T_j3 * T_j4 * T_j5 * T_tcp
```

- `T_car`：现有 `CarState2T`（平面 yaw）。
- `T_q_0_`：现有 `base_mani_fixed_joint_xyz_ypr`（Ranger 上 CR10 安装）。
- `T_ji`：现有 `getAJointTran` / `getJointTrans`。
- `T_tcp`：配置 `fsm/ee_tcp_xyz_rpy`，默认 `[0,0,0,0,0,0]` → 单位阵（法兰盘）。rpy 与安装座相同：`Rz(y)*Ry(p)*Rx(r)`。

`getEePose(car_xyz_yaw, q_arm) → T_ee` 与 `getEeJacobian(...) → 6×8` 必须对同一条运动学链。Jacobian 列顺序：`[∂/∂x, ∂/∂y, ∂/∂yaw, ∂/∂q0, …, ∂/∂q5]`。臂部列用已有 `T_joint_grad` 链式法则；底盘三列对 `T_car` 解析求导。实现后用有限差分门闩（与 `test_cr10_fk` 同类）。

## 5. Whole-body IK

状态向量 `ξ = [x, y, yaw, q0, q1, q2, q3, q4, q5] ∈ R^8`。  
位姿误差 `e ∈ R^6`：`e_p = p_now - p_des`，`e_r = Log_SO3(R_now^T R_des)`（或等价的轴角，角度取 `[-π, π]`）。

### 5.1 Stage B — 当前附近数值 IK

目的：在当前整机附近找一个满足 `T_ee` 的解，尽量少动。

- 初值：当前 `ξ`。
- 算法：阻尼最小二乘（DLS / Levenberg-Marquardt），`Δξ = J^T (J J^T + λ I)^{-1} (-e)`。`λ` 随误差增大，奇异时加大。
- **信任域（「附近」的硬定义）：** 迭代中投影
  - `|x - x0| ≤ local_xy`（默认 0.40 m）
  - `|y - y0| ≤ local_xy`
  - `|wrap(yaw - yaw0)| ≤ local_yaw`（默认 20°）
  - `q` 投影到 `manipulator_min_pos` / `max_pos`
- 底盘必须在地图内，否则该步失败。
- 最多 `b_max_iters`（默认 60）或耗时 `b_max_ms`（默认 40 ms）。
- **B 成功：** 位姿门闩通过 **且** 终点 `checkcollision(car, q, safe=true)` 为无碰。
- **B 失败 / 质量差（进入 C）：** 位姿未收敛；或收敛但碰撞；或信任域挡死导致位姿达不到门闩。B 不在信任域外「走出很远」充数——那是 C 的职责。

位姿门闩（B 与 C 共用，可配）：

- 位置：`||e_p|| < ee_pos_tol`（默认 0.01 m）
- 姿态：`||e_r|| < ee_rot_tol`（默认 2°）

### 5.2 Stage C — 底盘候选 + 臂 IK

仅当 B 失败时运行。服务**单个终点**，不生成底盘路径。

**底盘采样（世界系 XY 在末端位置周围，车头朝向末端）：**

```text
for r in {0.50, 0.75, 1.00, 1.25} m:          # 法兰盘到车原点的水平距离
  for k in 0..11:                             # 12 个朝向
    θ = k * 30°
    x = p_ee.x - r * cos(θ)
    y = p_ee.y - r * sin(θ)
    yaw = θ                                   # Ranger +x 指向末端
```

共 48 个底盘候选。丢弃：超出地图、`checkCarObsCollision` 碰撞。再并入一个「当前底盘」候选（即使超出信任域），避免 C 完全丢掉现在站位。

**每个合法底盘上的臂 IK（6 维）：** 底盘固定，DLS 解 `q`。两个种子，先成功先用：

1. 当前臂角  
2. 中间姿态：各关节取 `(qmin+qmax)/2`

臂角投影到限位。最多 `c_arm_iters`（默认 40）/ 单候选 `c_arm_max_ms`（默认 5 ms）。  
整机再跑一次 `checkcollision(..., safe=true)`。碰撞或未达位姿门闩则丢弃。

**总预算：** Stage C 上限 `c_max_ms`（默认 150 ms）。超时则在已评估候选里选最好的；一个都没有则整体失败。

### 5.3 打分（B 成功的解与 C 候选同一公式）

代价越小越好：

```text
cost = w_xy   * ||Δp_base||           # 底盘平移，m
     + w_yaw  * |wrap(Δyaw)|          # rad
     + w_q    * ||Δq||_2              # rad
     + w_gap  * max(0, gap_ref - d)   # m
```

`d` 是终点处底盘与臂碰撞球在 ESDF 上的最小距离（调用已有 `checkCarObsCollision` / `checkManiObsCollision` 的 `min_dist` 输出；自碰不计入 `d`）。默认：`w_xy=1.0`，`w_yaw=0.3`，`w_q=0.1`，`w_gap=2.0`，`gap_ref=0.20`。  
B 只有 0 或 1 个合法解，合法即采用，不必和 C 比。C 在合法候选中取 `cost` 最小。

日志（每次 `/ee_goal` 必打）：

```text
[EE IK] stage=B|C success|fail  pos_err=.. rot_err_deg=..
        base_xy=(..) yaw_deg=..  Δxy=.. Δyaw_deg=.. Δq=..
        candidates=n_ok/n_total  time_ms=..
        fail_reason=...
```

## 6. Error handling

| 情况 | 行为 |
|------|------|
| 无 odom / 关节 | 忽略本次 `/ee_goal`，WARN |
| `frame_id != world` | 拒绝，WARN |
| `manipulator_type != cr10` | 拒绝，ERROR |
| `target_type=2` | 忽略，WARN |
| B 和 C 都无合法解 | `[EE IK] fail`；`have_target_=false`；停在 `WAIT_TARGET`。**不**进 `GEN_NEW_TRAJ`，**不**占连续失败计数 |
| IK 成功但后续 `planGlobalTrajWaypoints` / Hybrid A* 失败 | 走**现有** FSM 失败与 abort（terminal 合法，失败在路径）。非法终点仍走现有 `GOAL_COLLISION` 通道（IK 已过滤则不应出现） |
| 执行中碰撞 / 超轮速 | 现有 replan / time scaling，与 2D Goal 任务相同 |

IK 失败视为「这个末端位姿当前不可解」，不是规划器内部噪声，因此立即 abort，避免 10～20 次空转。

## 7. Visualization

- Marker 本体：坐标系（轴长约 0.15 m）+ 半透明盒子，表示用户指定的 `T_ee`。
- IK 成功：用现有 `visMM` / `getManiMarkerArray` 画一帧**半透明幽灵整机**在 terminal 构型上（namespace `ee_ik_terminal`）。新的 `/ee_goal` 或 2D Goal 到来时 DELETE 旧幽灵。
- IK 失败：幽灵删除；Marker 保持用户位姿；WARN。
- 规划成功后的全局路径、后端 mesh：现有显示，不改。

## 8. Configuration

新增块，放在 `remani_planner_param.yaml` 的 `fsm:` 下（仅手动仿真需要的 IK 参数；运动学仍读 `mm/`）：

```yaml
fsm:
  ee_goal_topic: /ee_goal
  ee_tcp_xyz_rpy: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # flange; TCP later
  ee_pos_tol: 0.01          # m
  ee_rot_tol_deg: 2.0
  ee_ik_local_xy: 0.40      # m, Stage B trust region
  ee_ik_local_yaw_deg: 20.0
  ee_ik_b_max_iters: 60
  ee_ik_b_max_ms: 40.0
  ee_ik_c_radii: [0.50, 0.75, 1.00, 1.25]
  ee_ik_c_yaw_bins: 12
  ee_ik_c_max_ms: 150.0
  ee_ik_w_xy: 1.0
  ee_ik_w_yaw: 0.3
  ee_ik_w_q: 0.1
  ee_ik_w_gap: 2.0
  ee_ik_gap_ref: 0.20
```

Launch：`remani_sim.launch` 在 `use_rviz:=true` 时启动 `ee_goal_marker_node`。RViz 配置增加 InteractiveMarkers display（topic 与节点 `server` 一致，例如 `/ee_goal_marker/update`）。默认 `target_type` 仍由现有 launch arg 决定；Ranger+CR10 仿真继续 `target_type:=1` 才能同时用两个工具。

## 9. 文件与改动边界

| 路径 | 改动 |
|------|------|
| `mm_config/include/mm_config/mm_config.hpp` + `src/mm_config.cpp` | `getEePose`、`getEeJacobian`、TCP 偏置 |
| `mm_config/include/mm_config/whole_body_ik.hpp` + `src/whole_body_ik.cpp` | `WholeBodyIkSolver`（B→C） |
| `mm_config/src/test_ee_pose_ik.cpp` | FK/Jacobian 门闩 + 可达/不可达/碰撞用例 |
| `plan_manage/src/remani_replan_fsm.cpp` + `.h` | 订阅 `/ee_goal`，`eeGoalCallback` |
| `plan_manage/src/ee_goal_marker_node.cpp` | Interactive Marker 节点 |
| `plan_manage/config/remani_planner_param.yaml` | 第 8 节参数 |
| `plan_manage/launch/remani_sim.launch` | 启动 marker 节点 |
| `plan_manage/launch/remani_ranger_cr10.rviz` | InteractiveMarkers display |

**禁止改：** `kino_astar.cpp` 搜索核、`rrt.cpp` 搜索核、`poly_traj_optimizer.cpp` 代价与 time scaling、2D `waypointCallback` 主语义、轮速硬限制。

## 10. Testing

### 10.1 离线（无 RViz）

在 `mm_config` 测例中构造 Ranger+CR10 参数（复用 `test_cr10_fk.cpp` 的 param 注入）：

1. **FK 往返：** 随机合法 `ξ`，`T = getEePose(ξ)`，从附近初值 IK 回 `ξ'`，要求位姿门闩通过；Jacobian 与有限差分最大相对误差 `< 1e-4`。
2. **信任域：** 目标只需底盘移动 1 m 才能到，则 Stage B 必须失败，Stage C 必须给出解（空地图）。
3. **近处少动：** 目标在当前法兰盘 5 cm / 5° 内，Stage B 成功，`||Δp_base|| < 0.05 m`。
4. **不可达：** `z = 3.5 m`（CR10 够不着），B 和 C 都失败，不抛异常。
5. **碰撞过滤：** 在终点法兰盘处放一个占体素障碍，C 不得返回碰撞 terminal。

### 10.2 仿真（`remani_ranger_cr10_sim.launch`，`target_type:=1`）

| Case | 操作 | 期望 |
|------|------|------|
| A | 2D Nav Goal，与现在相同 | 臂角保持；time scaling 行为不变；`/ee_goal` 未用 |
| B | Marker 拖到当前法兰盘附近松手 | `[EE IK] stage=B success` → `GEN_NEW_TRAJ` → `EXEC_TRAJ` → `reach goal` |
| C | Marker 拖到数米外、高度仍在臂工作空间 | `[EE IK] stage=C success`，幽灵底盘在末端附近 → 规划执行到位 |
| D | Marker 拖进障碍物内部 / 过高 | `[EE IK] fail`，停在 `WAIT_TARGET`，无连续 abort 刷屏 |
| E | 先 2D Goal 走一程，再拖 Marker | 第二次按 EE 路径规划，从当前实际状态起步 |
| F | 先拖 Marker，再点 2D Goal | 第二次按底盘路径，臂角为**当时**关节，不回到 Marker 姿态 |

成功标准：Case B/C 执行结束时法兰盘与 Marker 位姿误差在 `ee_pos_tol` / `ee_rot_tol` 量级（允许控制跟踪误差到 2 倍门闩）。Case A 回归不得变差。

## 11. 明确取舍

- **不做解析臂 IK：** CR10 现有链是 `T_fixed * RotZ`，与标准 UR 解不完全同一套。数值 DLS + 现有 Jacobian 足够本阶段单个终点。
- **B 有信任域：** 避免「数值 IK 把底盘滑出很远」冒充少动解。远处由 C 的站位采样负责。
- **碰撞只卡终点：** 路径碰撞仍由现有规划负责。IK 不代替 Hybrid A*。
- **TCP 后置：** 法兰盘与现有 FK 测试一致；夹爪偏置只改配置，不改求解器。
