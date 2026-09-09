# REMANI Ranger+CR10 实机部署与人工确认执行设计

**Date:** 2026-09-01
**Revised:** 2026-09-09（cross-host control-plane boundary：laptop plan-only + remote AGX black-box endpoint）
**Status:** Implementation-plan ready — source-consistency review blockers closed
**Chosen approach:** 方案 1，规划器与实机执行器分离
**Scope:** 保留现有 REMANI 仿真模式，新增 Ranger+CR10 实机模式；在 RViz 中设置末端目标、仅规划并预览全身轨迹，待操作者确认后再执行；执行期间支持 Pause、Resume 和 Abort。

## 0. 本轮修订结论

本设计使用一个且仅一个 execution owner：

| 启动模式 | execution_owner | 执行生命周期所有者 |
|---|---|---|
| sim | internal | REMANI 原 FSM，保留 EXEC_TRAJ、现有 mm_controller 和现有完成逻辑 |
| real | external | Trajectory Gate + Real Executor；REMANI 严格 PLAN-ONLY |

实机模式不允许 REMANI 的 EXEC_TRAJ 与 Real Executor 的 EXECUTING 同时推进。real 下 REMANI 只读取实际状态、规划、发布候选事务，然后退出该候选的执行生命周期。

### 0.1 跨主机部署边界（2026-09-09）

当前已确认的部署拓扑：

- 笔记本只运行 REMANI planner、Gate、State Bridge、dry-run Executor、RViz Panel 等控制面。
- `agx/` 在另一台主机运行，是远端黑盒 ROS endpoint。笔记本通过 ROS 网络交换标准话题、Action 和状态。
- 不得在笔记本上修改、编译、source 或启动任何 `agx/` 包，也不得依赖本机 `agx/build` 或 `agx/devel`。
- 笔记本 launch 不启动 Ranger、CR10 或其他硬件节点；远端 runtime remap/config 属于远端部署契约。
- 规范 Ranger 硬件命令话题是 `/remani/hardware/ranger/cmd_vel`。dry-run 不得 advertise/publish 该话题；dry-run 诊断只用 `/remani/dry_run/ranger_cmd_vel_preview`。
- planner raw transaction 话题是 `/remani/planner_candidate`，不得使用过时的 `/remani/candidate_trajectory`。
- planner 状态名是 `HANDOFF`，不是 `HANDED_OFF`。Gate acknowledgement 必须保留并回传 `raw_transaction_stamp`。
- 正式 non-dry 硬件输出在远端 watchdog、CR10 安全 Action proxy、可靠 `/remani/cr10_status`、两机 ROS/时间同步和 shared-T0 可观测反馈单独验证前保持硬阻塞。当前 Phase 2 只允许完成 zero-output dry-run control plane。

本轮已根据源码确认并修正以下事实：

- 当前 GEN_NEW_TRAJ 成功后直接进入 EXEC_TRAJ，并用 ros::Time::now() 减 trajectory.start_time 推进虚拟执行时间。
- 当前 checkCollisionCallback() 在非 WAIT_TARGET 状态下使用同一虚拟时间做未来碰撞检查、局部重规划和 EMERGENCY_STOP。
- 当前 sendPolyTrajROSMsg() 只发布 ACTION_ADD；START / FINAL 虽已定义在消息中，但 planner 尚未使用。
- PolynomialTraj.trajectory_id 是同一 SingulTraj 内的 segment 序号，不是用户 Plan 的候选版本。
- 当前 planner 机械臂回调直接读取 position[0..5]，不按 JointState.name 重排。
- 当前 Ranger 驱动直接订阅绝对话题 /cmd_vel，且没有 ROS 侧命令超时保护。
- 当前 CR10 Action callback 会阻塞执行整条轨迹，单线程 spinner 下 cancel 不能保证及时执行。
- 当前 dobot_v4_bringup/RobotStatus.msg 只包含 is_enable 和 is_connected；虽然驱动实时数据已有 ErrorStatus 和 robot_mode，但尚未通过该状态消息暴露，不能把现有 RobotStatus 直接当作完整 fault 信号。笔记本控制面必须消费项目自有 `/remani/cr10_status`，不得为订阅旧 AGX RobotStatus 引入 AGX generated message 编译依赖。

## 1. 目标、固定决策与非目标

### 1.1 目标

- 使用一个启动参数选择现有仿真模式或新增实机模式。
- mode:=sim 保持当前仿真入口、内部执行状态机和自动执行语义。
- mode:=real 在笔记本上启动 REMANI plan-only、Trajectory Gate、Real Executor、RViz 和状态桥；Ranger/CR10 硬件节点只在远端 AGX 主机按远端部署契约运行，不由笔记本 launch 启动。
- 实机启动后，RViz RobotModel 由当前实际 odom 和 CR10 关节反馈初始化并持续更新。
- RViz 的 Plan 只生成候选全身轨迹；只有人工 Execute 才能进入实机执行。
- real 模式由外部执行层统一拥有 PLANNED、EXECUTING、PAUSED、SUCCEEDED 和 ERROR。
- 执行中支持 Pause、严格容差内 Resume，以及使旧候选失效的 Abort。
- dry_run 完整运行规划、验证、采样、时序和状态机，但在硬件 ownership 边界阻断运动输出。
- 第一阶段使用显式 static-empty GridMap，不使用 D435 或 LiDAR 在线感知。

### 1.2 固定决策

- 默认 mode:=sim。
- real 必须 execution_owner:=external；sim 必须 execution_owner:=internal。
- 非法组合，例如 mode:=real + execution_owner:=internal，启动时直接拒绝。
- 第一阶段 global_plan:=true，不做执行期 periodic replan。
- 不引入 MoveIt 执行层；CR10 由 Real Executor 直接调用 FollowJointTrajectory。
- 当前启动位置定义为 world 原点，不引入外部定位。
- 实机使用保守且可参数化的速度和误差限制。
- 物理急停是实机运行前置条件。

### 1.3 非目标

- 不提供动态障碍物检测、在线避障或执行中动态重规划。
- 不改 REMANI 的 Hybrid A*、RRT、多项式优化和 whole-body terminal 算法。
- 不增加笛卡尔伺服、阻抗控制、MPC 或新的 whole-body planner。
- 不实现执行中的新目标抢占。
- V1 不自动生成 Pause 到剩余轨迹之间的 whole-body connector。
- 不用软件按钮替代物理急停。

## 2. 启动方式与 execution ownership

统一入口：

~~~bash
# 现有仿真行为
./remani_planner/run_remani.sh mode:=sim

# 连接实机并运行完整 dry-run
./remani_planner/run_remani.sh \
  mode:=real \
  dry_run:=true \
  robot_ip:=192.168.5.1 \
  can_interface:=can0

# 正式实机输出
./remani_planner/run_remani.sh \
  mode:=real \
  dry_run:=false \
  robot_ip:=192.168.5.1 \
  can_interface:=can0
~~~

launch 内部锁死：

~~~text
mode=sim
  execution_owner=internal
  environment_mode=simulated

mode=real
  execution_owner=external
  environment_mode=static_empty
~~~

### 2.1 sim：REMANI owns execution lifecycle

保持现有行为：

~~~text
actual simulated state
  -> planning
  -> planning success
  -> REMANI EXEC_TRAJ
  -> mm_controller automatic execution
  -> existing periodic replan / emergency / completion behavior
~~~

sim 继续保留：

- GEN_NEW_TRAJ -> EXEC_TRAJ。
- planner trajectory.start_time 与 ros::Time::now() 的内部时间推进。
- mm_controller 对现有 ACTION_ADD 的消费。
- planner 的 REPLAN_TRAJ、EMERGENCY_STOP、planning/finish 和 EE actual-FK completion。

实机功能不得改变上述语义或默认启动路径。

### 2.2 real：external layer owns execution lifecycle

real 下 REMANI 严格 PLAN-ONLY：

~~~text
actual current state
  -> planning
  -> START / ADD... / FINAL candidate transaction
  -> candidate handoff
  -> planner returns to planning idle
~~~

real 的执行生命周期为：

~~~text
Trajectory Gate
  -> PLANNED
  -> explicit Execute
  -> Real Executor EXECUTING
  -> PAUSED / SUCCEEDED / ERROR
~~~

禁止增加一个继续驱动内部轨迹时间的 WAIT_EXTERNAL_EXECUTE，然后再与外部 Executor 双向同步。real 下 planner 不拥有任何执行时钟。

## 3. 三层架构与数据流

~~~text
================================================
UI / Human
================================================

EE Marker
  -> cached EE target

RViz Panel
  |- Plan
  |- Execute(candidate_id)
  |- Pause
  |- Resume
  '- Abort


================================================
Planning layer
================================================

REMANI Planner

mode=sim:
  owns normal EXEC_TRAJ lifecycle

mode=real:
  PLAN ONLY
  -> /remani/planner_candidate
     ACTION_WARN_START
     ACTION_ADD trajectory_id=1..N
     ACTION_WARN_FINAL


================================================
Real execution layer
================================================

Trajectory Gate
  -> assemble and validate
  -> assign monotonic candidate_id
  -> freeze candidate
  -> /remani/frozen_candidate
  -> publish isolated preview
  -> explicit Execute(candidate_id)

Real Executor
  |- /remani/hardware/ranger/cmd_vel -> Ranger driver
  '- /cr10_robot/joint_controller/follow_joint_trajectory

actual /odom + actual CR10 q
  -> State Bridge
  -> readiness / tracking / completion

Real Executor
  -> final base/joint checks
  -> actual EE FK verification
  -> SUCCEEDED or ERROR
~~~

Planner、Gate 与 Executor 使用两级接口，不共享一条“candidate trajectory”通道：

~~~text
REMANI Planner
  -> /remani/planner_candidate
  -> raw START / ADD... / FINAL transaction

Trajectory Gate
  -> assemble / validate / safety and speed checks
  -> assign candidate_id / manage version
  -> freeze immutable candidate
  -> /remani/frozen_candidate

Real Executor
  -> consume frozen candidate only
~~~

Trajectory Gate 是 planning ownership 与 execution ownership 的边界。Real Executor 永远不得订阅或直接消费 `/remani/planner_candidate`；所有 validation、speed check、safety filtering、freeze 和 version management 必须先在 Gate 完成。

实际 RobotModel 和候选预览使用不同数据通路。预览不得发布到真实 /joint_states，也不得发布 world -> base_link。

## 4. REMANI real PLAN-ONLY 语义

### 4.1 planner-side 抽象生命周期

real 下 planner 使用独立的 planner_state：

~~~text
IDLE
PLANNING
HANDOFF
~~~

语义：

- IDLE：没有正在进行的规划或 handoff；外部 Executor 可以同时处于 PLANNED、EXECUTING、PAUSED、SUCCEEDED 或 ERROR。
- PLANNING：正在根据最新实际状态生成 raw candidate transaction。
- HANDOFF：START / ADD... / FINAL 已发布，等待 Gate 对完整事务给出 commit success / failure 和 candidate_id correlation；这是短暂的接口交接状态，不是等待执行状态。

这三个是独立于 REMANI 原 FSM 的设计语义，不要求把 real 执行重新塞入原 FSM，也不允许 HANDOFF 演变成 WAIT_EXTERNAL_EXECUTE。

### 4.2 成功 Plan

~~~text
planner_state=IDLE
  -> accept one permitted Plan
  -> planner_state=PLANNING
  -> candidate finalized
  -> publish /remani/planner_candidate START / ADD... / FINAL
  -> planner_state=HANDOFF
  -> Gate commit acknowledgement supplies candidate_id or failure
  -> release planner execution ownership
  -> planner_state=IDLE
~~~

candidate handoff 后，planner：

- 清除 active trajectory ownership。
- 清除任何内部 EXEC_TRAJ state / latch；该 candidate 不得进入原 EXEC_TRAJ。
- 清除 planner execution timer state，包括 trajectory.start_time 对该 candidate 的执行期语义。
- 保留 Gate 返回的 candidate_id、external execution metadata 和 execution-result correlation information，用于日志与结果关联。
- 不进入当前基于 wall-clock 的 EXEC_TRAJ。
- 不继续按 candidate duration 推进时间。
- 不执行 real 的 EE completion。
- 不发布 planning/finish。
- 不启动 periodic local replan。
- 不因 candidate duration 到期而切回 WAIT_TARGET。
- 不认为机器人已运动或已到达目标。
- 不拥有 PLANNED、EXECUTING、PAUSED、SUCCEEDED 或 ERROR。

保留 correlation metadata 不代表保留 execution ownership。candidate handoff 后，该 candidate 的执行状态只存在于 Gate / Real Executor；planner 不得据此重启内部时钟或重新进入 EXEC_TRAJ。

### 4.3 失败 Plan

- 普通 IK failure、无可行路径或 optimizer failure：发布明确的 planning failure / ACTION_WARN_IMPOSSIBLE，Real Deployment State Machine 从 PLANNING 回到 READY。
- malformed transaction、内部协议损坏或实际反馈安全故障：进入 ERROR。
- 失败不得让 planner 进入 EXEC_TRAJ 或按虚拟轨迹时间做完成判断。

### 4.4 execution result / reset

Real Executor 在 Success、Abort 或 Error 后发布显式 execution result，至少包含 candidate_id、result code 和终点误差。

planner 可以订阅该结果用于清理日志或任务元数据，但该接口：

- 不启动内部 EXEC_TRAJ。
- 不恢复 planner wall-clock。
- 不驱动 REPLAN_TRAJ 或 EMERGENCY_STOP。
- 不改变外部层对执行结果的所有权。

V1 的下一次 Plan 由 Real Deployment State Machine 的 READY / PLANNED 权限控制，而不是通过内部执行状态机解锁。

### 4.5 real 下 planner safety timer

execution_owner=external 时，现有 checkCollisionCallback() 的执行期逻辑整体短路，不允许使用一个未真实执行的 t_cur。

real candidate handoff 后，下列 planner 执行态不参与硬件控制：

- EXEC_TRAJ
- REPLAN_TRAJ
- EMERGENCY_STOP
- dynamic trajectory collision scan
- depth/cloud loss 引起的执行期 replan 或 emergency transition

仍保留规划阶段的：

- terminal collision check
- whole-body self collision
- car-arm 和 arm-arm collision
- static-empty map boundary collision
- optimizer collision and feasibility checks
- Gate 对完整 candidate 的离线验证

real 执行后的反馈超时、跟踪误差、停止和完成判定全部归 Real Executor。

## 5. Real Deployment State Machine

以下状态属于外部实机部署状态机，不是 REMANI 原 FSM：

~~~text
NOT_READY
READY
PLANNING
PLANNED
EXECUTING
PAUSED
SUCCEEDED
ERROR
~~~

主要转换：

~~~text
NOT_READY --actual feedback synchronized--> READY

READY --Panel Plan--> PLANNING
PLANNED --Panel Plan--> PLANNING

PLANNING --complete START/ADD/FINAL + Gate validation pass--> PLANNED
PLANNING --IK/no-path/optimizer failure--> READY
PLANNING --protocol corruption or feedback safety fault--> ERROR

PLANNED --Execute(candidate_id) + readiness recheck--> EXECUTING

EXECUTING --Pause + both devices confirmed stopped--> PAUSED
PAUSED --strict tolerance pass--> EXECUTING
PAUSED --strict tolerance fail--> PAUSED, Resume rejected

EXECUTING --trajectory complete + final checks pass--> SUCCEEDED
EXECUTING --tracking/safety/completion fail--> ERROR

PLANNED/EXECUTING/PAUSED --Abort--> READY
ERROR --health restored + Abort reset--> READY
~~~

### 5.1 planner_state 与 executor_state

ExecutionState 必须分别报告两个正交状态空间：

~~~text
planner_state:
  IDLE
  PLANNING
  HANDOFF

executor_state:
  NONE
  PLANNED
  EXECUTING
  PAUSED
  SUCCEEDED
  ERROR
~~~

典型合法组合：

| Real Deployment State | planner_state | executor_state |
|---|---|---|
| NOT_READY / READY | IDLE | NONE |
| PLANNING，planner 计算中 | PLANNING | NONE |
| PLANNING，等待 Gate commit | HANDOFF | NONE |
| PLANNED | IDLE | PLANNED |
| EXECUTING | IDLE | EXECUTING |
| PAUSED | IDLE | PAUSED |
| SUCCEEDED | IDLE | SUCCEEDED |
| ERROR | IDLE | ERROR |

其中 `planner_state=IDLE + executor_state=EXECUTING` 是 real 模式的正常执行状态，明确表示 planner 已完成 handoff 且没有执行 ownership。Panel 不得使用 planner_busy、planner 是否空闲或 planner wall-clock 推断 executor_state。HANDOFF 只等待 Gate commit acknowledgement，不等待人工 Execute 或硬件完成。

按钮权限：

| 外部状态 | 可用操作 |
|---|---|
| NOT_READY | 无 |
| READY | Plan |
| PLANNING | Abort |
| PLANNED | Plan、Execute、Abort |
| EXECUTING | Pause、Abort |
| PAUSED | Resume、Abort |
| SUCCEEDED | Plan |
| ERROR | Abort，仅用于健康恢复后的安全复位 |

## 6. Candidate Transaction Protocol

### 6.1 当前事实与新增要求

PolynomialTraj.msg 已定义 ACTION_WARN_START 和 ACTION_WARN_FINAL，但当前 planner 只发送 ACTION_ADD。START / FINAL 是 real external-execution mode 必须新增的正式协议，不得描述成现有行为。

不修改 PolynomialTraj.msg 定义；action 的解释由 mode 和接收边界决定：

| 模式 | 接收者 | action 语义 |
|---|---|---|
| sim / internal | 现有 mm_controller | 保持原 PolynomialTraj 语义和 ACTION_ADD-only 行为，不引入 transaction control |
| real / external | Trajectory Gate | ACTION_WARN_START = begin transaction |
| real / external | Trajectory Gate | ACTION_ADD = append one ordered segment |
| real / external | Trajectory Gate | ACTION_WARN_FINAL = commit transaction；只有 commit 后才可成为 complete candidate |
| real / external | Trajectory Gate | ACTION_ABORT = invalidate assembling/cached transaction |
| real / external | Trajectory Gate | ACTION_WARN_IMPOSSIBLE = planning failure notification；不得 commit |

real/external 的 raw transaction 只发布在 `/remani/planner_candidate`：

~~~text
ACTION_WARN_START
  -> ACTION_ADD trajectory_id=1
  -> ACTION_ADD trajectory_id=2
  -> ...
  -> ACTION_ADD trajectory_id=N
  -> ACTION_WARN_FINAL
~~~

控制消息 START、FINAL、ABORT 和 IMPOSSIBLE 的 trajectory_id 固定为 0，trajectory 数组为空；只有 ADD 携带 segment 数据。

### 6.2 START

当一个 Plan 被外部状态机从 READY 或 PLANNED 合法启动后，状态进入 PLANNING，并允许该 planning session 接收一个 START。

Gate 接受 START 时：

- 立即使旧的未完成 assembly 失效。
- 如果原来存在未执行的 PLANNED candidate，立即使其失效；新 Plan 失败时不恢复旧 candidate。
- 清空 assembly buffer。
- 分配新的 uint64 单调 candidate_id。
- 将 expected next trajectory_id 设为 1。
- 启动 assembly timeout。

未经合法 Plan session 的 START 被拒绝。EXECUTING 或 PAUSED 时收到任何新 START 都拒绝且不得影响正在执行的 frozen candidate。

### 6.3 ADD

每个 ACTION_ADD：

- 必须属于当前唯一 assembling transaction。
- trajectory_id 从 1 开始严格连续。
- 不允许 duplicate、回退或跳号。
- PolynomialMatrix 数量必须非零。
- 每个 piece 的维数、阶数和 duration 必须合法。
- coefficient、duration 和所有数值必须 finite。
- segment 间维数、时间和状态连续性必须满足 Gate 参数。

任一 ADD 非法，整个 assembling transaction 失效并进入 protocol ERROR。

### 6.4 FINAL

只有收到 FINAL 后，candidate 才是 complete。Gate 随后执行完整验证：

- segment 序号完整。
- 总 duration 有限且大于零。
- position、velocity、acceleration 连续性。
- 轨迹维数与 Ranger+CR10 模型一致。
- 起点与 Plan 时实际状态一致。
- 底盘、关节速度和加速度限制。
- self collision、car-arm、arm-arm 和 static-empty boundary。

验证通过后：

- 缓存 immutable frozen candidate。
- 计算候选终点 base、joint 和 expected EE FK。
- 生成实际状态隔离的 RViz preview。
- candidate_complete=true。
- candidate_valid=true。
- 外部状态进入 PLANNED。
- Enable Execute(candidate_id)。

FINAL 之前 Execute 始终禁用。

### 6.5 ABORT、IMPOSSIBLE、timeout 与新 Plan

- ACTION_ABORT：当前 assembling 和 cached candidate 都失效。
- ACTION_WARN_IMPOSSIBLE：当前 assembling 和 cached candidate 都失效；普通规划失败返回 READY。
- assembly timeout：视为内部 transaction failure，candidate 失效并进入 ERROR。默认 timeout 为 60 s，可参数化。
- Plan 只允许从 READY 或 PLANNED 发起。
- EXECUTING / PAUSED 禁止新 planning transaction 覆盖 frozen executing candidate。

### 6.6 candidate_id 与 trajectory_id

两者语义严格分离：

| 字段 | 类型 | 所有者 | 语义 |
|---|---|---|---|
| PolynomialTraj.trajectory_id | uint32 | planner | 当前 transaction 内 SingulTraj segment 序号，1..N |
| Gate candidate_id | uint64 | Trajectory Gate | 每个接受 START 分配的全局单调候选版本 |

trajectory_id 在每个 transaction 中从 1 重新开始。candidate_id 在 Gate 进程生命周期内单调递增，不因失败回退或复用。

Execute 不再使用 std_srvs/Trigger，而使用带 uint64 candidate_id 的 ExecuteCandidate 服务。请求中的 candidate_id 必须等于当前 PLANNED frozen candidate；旧 UI 请求、延迟请求或已失效 ID 全部拒绝。

### 6.7 Planner -> Gate -> Executor 两级接口

| 接口 | 唯一 publisher | 唯一 consumer | 内容与 ownership |
|---|---|---|---|
| /remani/planner_candidate | REMANI real plan-only planner | Trajectory Gate | 未验证的 PolynomialTraj START/ADD/FINAL transaction；仍属于 planning side |
| /remani/frozen_candidate | Trajectory Gate | Real Executor | 含 Gate candidate_id、完整已验证 segment 集、duration 和 validation result 的 immutable frozen candidate；属于 external execution side；preview 由 Gate 从同一 frozen cache 独立生成 |

Gate 在 START 时分配 candidate_id，在 FINAL 验证成功后随 `/remani/frozen_candidate` 发布 commit acknowledgement；planner 由该 acknowledgement 保留 candidate_id correlation 后回到 IDLE。验证失败只发布 failure/result，不得产生 frozen candidate。

Real Executor：

- 永远不订阅 `/remani/planner_candidate`。
- 只缓存 Gate 发布的 `/remani/frozen_candidate`。
- Execute(candidate_id) 只可选择当前缓存且未失效的 frozen version。
- 不自行补齐缺段、跳过 Gate validation 或重新解释 raw PolynomialTraj action。

## 7. State Bridge 与启动实际姿态

### 7.1 驱动原始输入

CR10 驱动原始 /joint_states 在 real launch 中先重映射为：

~~~text
/remani/cr10_joint_states_raw
~~~

launch 语义等价于：

~~~xml
<remap from="/joint_states" to="/remani/cr10_joint_states_raw"/>
~~~

State Bridge 按 name 查找且只接受：

~~~text
joint1
joint2
joint3
joint4
joint5
joint6
~~~

输入数组顺序不可信。缺失、重复、NaN、Inf 或陈旧样本均无效。

CR10 原始消息若没有 velocity，State Bridge 使用带时间戳的位置差分和限幅滤波生成 qd_estimated，并单独发布 velocity_valid。停止与完成判定只有在估计窗口有效后才允许进行；不得把缺失 velocity 静默当作零速度。

当前 CR10 驱动的 RobotStatus 只有 connected / enabled 两个布尔量，而驱动内部实时数据另有 ErrorStatus 和 robot_mode。real V1 必须通过只读 telemetry 扩展或独立适配器输出统一状态：

~~~text
/remani/cr10_status
  connected
  enabled
  error_status
  robot_mode
  stamp / feedback_age
~~~

readiness 和运行时 fault 检查只消费这个规范化状态。不得用“Action Server 存在”“connected=true”或“enabled=true”推断无报警。error_status 非零、robot_mode 不在配置的可执行集合、字段未知或状态超时都视为 not ready；执行中出现则进入 ERROR。具体复用现有实时数据还是扩展驱动消息属于实现落点，但完整只读 fault telemetry 是正式实机输出的硬前置条件。

### 7.2 A：RobotModel JointState

输出：

~~~text
/joint_states
~~~

消费者：

- robot_state_publisher
- RViz RobotModel

可以包含：

- CR10 六轴实际反馈，名字为 cr10_joint1..cr10_joint6。
- Ranger steering / wheel display joints。
- gripper display joints。
- URDF 显示需要的其他关节。

没有真实反馈的显示关节必须标注为 static display default / not measured。它们不能参与 planner、tracking、Pause、Resume 或完成判定。

### 7.3 B：REMANI planning JointState

输出：

~~~text
/remani/cr10_joint_states
~~~

该消息只包含六个关节，固定顺序为：

~~~text
position[0] = cr10_joint1 = raw joint1
position[1] = cr10_joint2 = raw joint2
position[2] = cr10_joint3 = raw joint3
position[3] = cr10_joint4 = raw joint4
position[4] = cr10_joint5 = raw joint5
position[5] = cr10_joint6 = raw joint6
~~~

name 数组同样固定为 cr10_joint1..cr10_joint6。planner 即使仍按 position[0..5] 读取，也能得到正确六轴顺序。

禁止把完整 /joint_states 直接 remap 给 planner 的 joint_state 输入。

### 7.4 启动同步与 RViz 初始化

real 启动保持 NOT_READY，直到：

1. Ranger /odom 有连续、时间新鲜且数值有限的样本；消息 frame_id 固定为 world，child_frame_id 固定为 base_link。
2. world -> base_link TF 新鲜且唯一。real launch 固定 Ranger driver 的 publish_odom_tf=false，由 State Bridge 独占根据同一份 /odom 发布该动态 TF；State Bridge 不对 /odom 再积分、归零或重标定。
3. /remani/cr10_status 新鲜、connected=true、error_status=0 且 robot_mode 合法；正式 Execute 还要求 enabled=true。
4. 至少三个连续完整的 CR10 六轴实际样本已通过名称映射。
5. CR10 velocity estimate 已建立有效窗口。
6. robot_state_publisher 已形成 world 到 CR10 末端的完整 TF。
7. CR10 FollowJointTrajectory Action Server 存在。
8. static-empty GridMap 与 ESDF 已 ready。

同步前：

- Plan、Execute、Resume 禁用。
- 末端目标 Marker 隐藏。
- 不允许 candidate transaction。

同步后：

- Ranger driver 的内部 odom 积分状态在进程启动时为 x=0、y=0、yaw=0，因此启动瞬间实际位置定义为 world 原点；后续只使用该 /odom，不建立第二套原点或积分器。
- RViz RobotModel 使用实际底盘相对位姿和实际 CR10 q。
- REMANI 使用同一份 /odom 和 /remani/cr10_joint_states 初始化。
- planner 发布实际 /ee_current_pose。
- EE Marker 首次放置在实际 FK 末端位姿。
- 外部状态进入 READY。

### 7.5 实际状态与 preview 隔离

- 实际 RobotModel 只由 /odom、TF 和 /joint_states 驱动。
- candidate 全身动画使用独立 visualization_msgs/MarkerArray namespace。
- 底盘和 EE 路径使用独立 nav_msgs/Path。
- preview 不得覆盖 /joint_states、/odom 或 world -> base_link。

## 8. remani_real.launch 的强制 remap 与 topic ownership

### 8.1 planner 实际状态输入

real launch 锁死：

~~~xml
<remap from="~odom_world" to="/odom"/>
<remap from="~joint_state" to="/remani/cr10_joint_states"/>
~~~

语义：

| 话题 | 语义 |
|---|---|
| /odom | Ranger 实际相对里程计；planner、Executor 和 readiness 使用 |
| /remani/cr10_joint_states_raw | CR10 驱动原始 joint1..joint6 |
| /remani/cr10_joint_states | 固定顺序六轴规划/执行反馈 |
| /joint_states | 完整 RobotModel 显示状态，不直接给 planner |

grid_map 的 odom 输入同样 remap 到 /odom。

同一 launch 还锁死 Ranger odom_frame=world、base_frame=base_link、publish_odom_tf=false。world -> base_link 只由 State Bridge 发布，避免 Ranger driver 与 State Bridge 双重广播。

### 8.2 Ranger 硬件 command topic 隔离

所有通过 ROS topic 进入真实 hardware driver 的 command topic，必须由 real launch 放入 `/remani/hardware/<device>/...` 隔离 namespace。不得让通用控制 namespace 直接连接真实硬件。

当前 Ranger 驱动直接订阅绝对 /cmd_vel。real launch 必须将该订阅 remap 为唯一规范硬件接口：

~~~text
/remani/hardware/ranger/cmd_vel
~~~

launch 语义等价于：

~~~xml
<remap from="/cmd_vel" to="/remani/hardware/ranger/cmd_vel"/>
~~~

最终链路：

~~~text
Real Executor
  -> /remani/hardware/ranger/cmd_vel
  -> Ranger driver remapped /cmd_vel subscription
~~~

普通 /cmd_vel 不连接真实 Ranger。teleop、navigation 或其他节点即使发布 /cmd_vel，也不能直接驱动实机。

Real Executor 是 `/remani/hardware/ranger/cmd_vel` 的唯一合法 publisher。若硬件 topic 出现第二个 publisher，进入 ERROR。publisher ownership check 只是第二层保护；real launch namespace / topic isolation 才是第一层保护。设计中不提供旧 shorthand 的并行别名，避免形成第二条可达硬件路径。

### 8.3 Ranger command watchdog

Ranger 驱动边界必须补齐：

- 全零或近零 Twist 走显式停止分支，禁止 0/0 半径和 NaN。
- angular.z 为零的直线命令显式使用零转角。
- cmd_vel_timeout 默认 0.20 s；超时主动发送零运动命令。
- 启动、正常退出和通讯异常发送显式停止命令。
- watchdog 订阅的是 remap 后硬件 topic。
- ExecutionState 暴露 watchdog ready / timed_out。

watchdog 状态语义锁定为：

- ranger_watchdog_ready 表示驱动侧 watchdog 已启用、参数合法且健康状态可观测，不表示当前必须收到运动命令。
- READY / PLANNED 的 timed_out=true 是“无命令且保持停止”的正常安全空闲，不阻止 Plan。
- dry_run 不创建硬件 command publisher，timed_out 只作为诊断显示，不能为消除 timeout 而发布零命令，也不能因此使 dry-run 失败。
- dry_run=false 的 Execute readiness 通过后，Executor 先以唯一 publisher 连续发送零命令；只有 driver 报告 command channel fresh 后才接受 Execute、建立 T0 并发送 CR10 goal。
- 从 Execute 被接受、pre-start hold、EXECUTING 到受控停止完成，timed_out=true 都是执行安全故障，立即停止并进入 ERROR。

watchdog 不替代 Ranger 固件保护和物理急停。

## 9. dry_run 的硬件输出边界

dry_run:=true 允许完整执行：

- candidate sampling
- Gate assembly / validation
- readiness 和反馈监控
- Real Deployment State Machine
- monotonic timing 和 requested T0
- Ranger desired command calculation
- CR10 JointTrajectory generation
- limit / tracking calculation
- preview
- Pause / Resume / Abort 的 dry simulation

dry_run:=true 禁止：

- 向 /remani/hardware/ranger/cmd_vel 发布任何消息，包括“测试零命令”。
- 发送任何 CR10 FollowJointTrajectory goal，包括零轨迹或 hold 测试。
- 调用 ServoJ、Stop、Pause、Continue、EmergencyStop、EnableRobot、DisableRobot。
- 调用任何其他会改变机械臂运动或使能状态的服务。

只读操作允许：

- 检查 Action Server 是否存在。
- 读取 RobotStatus、connected、enabled。
- 读取规范化 CR10 error_status、robot_mode 和 feedback age。
- 读取 joint feedback、odom 和 TF。
- 发布 diagnostics、ExecutionState 和 preview。
- 将计算出的 Ranger 命令发布到 diagnostics-only 的 /remani/dry_run/ranger_cmd_vel_preview。
- 将生成的 CR10 trajectory 发布为 preview/diagnostics，但不发送 Action goal。

hardware_output_enabled 在 launch 和 Executor 输出适配层由 dry_run 锁死。dry-run 不能依靠“业务逻辑应该不会调用”来保证安全。

## 10. environment_mode=static_empty

real V1 固定：

~~~text
environment_mode=static_empty
~~~

`static_empty` 的范围与分辨率必须参数化，不允许在实现中 hard-code：

~~~text
environment/static_empty/size_x:     16.0   # m, V1 default
environment/static_empty/size_y:     12.0   # m, V1 default
environment/static_empty/size_z:      3.0   # m, V1 default
environment/static_empty/resolution:  0.05  # m, V1 default
~~~

所有值必须 finite 且大于零，并在初始化 GridMap 前完成参数校验。修改参数只改变 empty map 的边界和离散精度，不改变 static-empty / no-online-sensing 的安全语义。

该模式必须：

- 按上述配置创建范围明确的 free GridMap；V1 默认范围为 16 m × 12 m × 3 m、默认分辨率为 0.05 m，中心为启动 world 原点。
- 将范围内体素明确初始化为 free，而不是 unknown。
- 初始化完成后至少计算一次可查询 ESDF，并发布 map ready。
- 将地图范围外视为 hard-invalid，保留 map boundary。
- 不启动在线 depth/cloud fusion。
- 不等待 D435/LiDAR topic。
- 不产生 odom/depth timeout。
- 不因没有深度传感器进入 planner EMERGENCY_STOP。
- 保留 whole-body self collision、car-arm、arm-arm 和静态边界检查。

该模式不检测现实环境中的桌椅、墙、人或移动物体。RViz Panel 必须持续显示：

~~~text
ENVIRONMENT: STATIC EMPTY / NO ONLINE OBSTACLE SENSING
~~~

没有 D435/LiDAR 不等于忽略 GridMap；它表示使用显式初始化、可查询且有硬边界的 empty GridMap。

## 11. RViz Panel 与候选预览

Panel 显示：

- mode、execution_owner 和 dry_run。
- planner_state 和 executor_state，分别按第 5.1 节显示。
- Real Deployment State。
- transaction assembly 状态。
- candidate_id、candidate_complete、candidate_valid。
- environment_mode 和醒目的 static-empty 警告。
- odom、CR10 joint、TF、RobotStatus、Action、GridMap 和 watchdog readiness。
- 反馈年龄和最后错误。
- candidate duration、最大底盘线/角速度、最大关节速度。
- candidate 起点与最新实际状态误差。
- execution progress、pause_param_time 和 start skew。
- final base / joint / EE errors。
- Plan、Execute、Pause、Resume、Abort。

Plan：

- Marker 拖动只缓存目标。
- Panel Plan 复用 /ee_goal_plan，使 Marker 发布缓存的 /ee_goal。
- 只有 READY / PLANNED 可触发。
- 进入 PLANNING 后等待正式 candidate transaction。

Preview：

- 底盘路径、航向和正向/倒车 singul 分段。
- EE 路径。
- CR10 关节动画。
- 稀疏全身姿态。
- 起点、终点、时长和 Gate 检查结果。
- 无效段显示红色且 Execute 禁用。

Execute 必须携带当前 candidate_id，不增加隐式自动执行。

## 12. Real Executor 的唯一安全职责

execution_owner=external 时，Real Executor 是唯一 real execution safety controller，负责：

- odom timeout。
- CR10 joint feedback / velocity estimate timeout。
- TF 和 RobotStatus fault。
- 规范化 CR10 error_status、robot_mode 与状态 timeout。
- Action availability 和 Action state。
- Ranger command watchdog health。
- candidate 起点 recheck。
- base / joint tracking error。
- Ranger command limit 和实际速度 limit。
- CR10 joint velocity limit。
- shared T0 与 start skew。
- Pause、Resume 和 Abort。
- execution completion。
- actual whole-body / EE final verification。

planner 在 candidate handoff 后不再充当执行安全控制器。

## 13. 执行前与运行时限制

第一阶段默认限制：

| 项目 | 默认值 |
|---|---:|
| Ranger 最大线速度 | 0.10 m/s |
| Ranger 最大角速度 | 0.15 rad/s |
| CR10 各关节最大速度 | 0.10 rad/s |
| Execute 起点底盘位置容差 | 0.05 m |
| Execute 起点底盘航向容差 | 5 deg |
| Execute 起点各关节容差 | 3 deg |
| Resume 底盘位置容差 | 0.02 m |
| Resume 底盘航向容差 | 2 deg |
| Resume 各关节容差 | 1 deg |

加速度、tracking error、feedback timeout、stop velocity、completion tolerance 和 stop timeout 全部参数化。

Gate 在 PLANNED 前检查整条 candidate。Executor 在 Execute 前和运行时再次检查。

超限 candidate 必须拒绝，不允许独立 clamp 底盘或机械臂速度，因为独立截断会破坏全身同步；需要调整 planner/time-scaling 后重新 Plan。

Execute(candidate_id) 前必须重新确认：

- 请求 candidate_id 等于当前 frozen candidate。
- candidate complete 且 valid。
- feedback、TF、规范化 CR10 status、Action、watchdog capability 和 GridMap ready。
- 最新实际状态仍在 candidate 起点容差内。
- 没有第二个硬件 command publisher。
- dry_run 与 hardware_output_enabled 一致。

对于 dry_run=false，以上静态 readiness 通过后还必须完成 Ranger 零命令 freshness handshake；handshake 未完成时 Execute 请求不被接受、CR10 goal 不发送、T0 不建立。dry_run=true 只模拟同一状态转换，不创建硬件 publisher，也不要求硬件 command age 变 fresh。

## 14. Shared monotonic T0

Real Executor 拥有唯一单调执行时钟。点击 Execute、通过 readiness，并在 non-dry-run 完成 Ranger 零命令 freshness handshake 后，Execute 才被接受；接受瞬间：

~~~text
T_request = steady_now
T0 = T_request + start_lead_time
~~~

start_lead_time 可参数化，V1 默认 1.0 s。

### 14.1 Ranger

以下硬件 publish 仅在 dry_run=false 时启用；dry_run=true 只计算并发布 diagnostics preview。

~~~text
steady_now < T0
  -> continuously command zero

steady_now >= T0
  -> t_remani = steady_now - T0
  -> sample base trajectory(t_remani)
  -> closed-loop /odom correction
  -> publish /remani/hardware/ranger/cmd_vel
~~~

### 14.2 CR10

dry_run=false 时，Executor 在计算 T0 后立即提交 JointTrajectory；dry_run=true 时只生成相同轨迹用于检查和 preview，不发送 Action goal。轨迹包含：

~~~text
point time_from_start=0:
  current actual q
  zero velocity

hold until start_lead_time:
  current actual q
  zero velocity

for every original REMANI arm sample:
  time_from_start = start_lead_time + t_remani
~~~

以上是逻辑时间段，不允许在 time_from_start=start_lead_time 生成两个同时间戳的 JointTrajectoryPoint。序列化与 driver 执行规则锁定为：

- time_from_start=0 的首点保存 Execute 时的 current actual q，driver 在 pre-start hold 状态持续发送该 q。
- start_lead_time 之前不在首点与原轨迹起点之间做插值。
- 原 REMANI t=0 的点是 time_from_start=start_lead_time 的唯一一点；其后所有时间戳严格递增。
- Execute 时 current actual q 与原 REMANI q(0) 的最大关节误差必须小于 arm_hold_handoff_tol，V1 默认 0.02 rad；超限拒绝 Execute 并要求重新 Plan，不能用 hold 区间偷偷生成 connector。

Ranger 的 T0 对应 REMANI t=0；CR10 的第一个 non-hold 点同样对应 REMANI t=0。pre-start hold 是 CR10 driver 的明确状态，不依赖通用 JointTrajectory 插值器猜测其语义。

Action 必须在 T0 前被接受。arm_accept_guard 默认 0.20 s；若在 T0 - arm_accept_guard 前仍未接受，则保持 Ranger 为零、取消 goal 并进入 ERROR，不允许底盘单独启动。

### 14.3 时间记录与验收

Executor / driver 记录：

- requested T0。
- Action goal send 和 accept timestamp。
- first post-T0 Ranger trajectory-sampling timestamp。
- first non-zero Ranger command timestamp。
- first non-hold CR10 ServoJ send timestamp。
- ranger_start_error。
- cr10_start_error。
- cross-device start_skew。

cross-device start_skew 使用 first post-T0 Ranger trajectory-sampling timestamp 与 first non-hold CR10 ServoJ send timestamp 计算。first non-zero Ranger command 仍单独记录；如果 candidate 的底盘起始段或整段为零速度，该字段明确为 unavailable，不能把“没有非零命令”误判为同步失败。

初始同步验收目标为 absolute start_skew <= 100 ms，目标优化区间为 50–100 ms。最终收紧值依据 CR10 单机测试结果配置，但不能删除记录和验收。

不再使用“Action active 后再确定共同起始时刻”的模糊语义。

## 15. CR10 FollowJointTrajectory V1

### 15.1 严格 goal validation

Goal callback 在 accept 前必须检查：

- trajectory.points.size() >= 2。
- joint_names 恰好包含且只包含 joint1..joint6，每个名字唯一。
- 允许输入 joint_names 重排，但 driver 必须按名字转换为内部固定 q1..q6。
- 每个 point.positions.size() == 6。
- 每个 point.velocities.size() == 6；V1 对缺失 velocity 直接 reject。
- positions 和 velocities 全部 finite。
- 每个 time_from_start finite 且 >= 0。
- time_from_start 严格递增。
- 如果 accelerations 非空，size 必须为 6 且 finite；V1 不依赖 accelerations。
- 同时只允许一个 active goal；未完成时的新 goal 拒绝。

任一非法 goal 使用 Action REJECTED，不能发送任何 ServoJ。

### 15.2 非阻塞可抢占状态机

Goal callback：

~~~text
validate
-> reorder/cache
-> record steady receive time
-> accept
-> return
~~~

Timer callback 每次只允许：

~~~text
read steady elapsed time
-> locate one segment
-> sample one q/qd
-> send one ServoJ
-> update/publish feedback
-> return
~~~

禁止在 timer callback 内出现：

- 遍历整条轨迹的 for。
- 覆盖整个 segment 的 while。
- ros::Rate.sleep()。

servoj_period 参数化，V1 初始值 0.10 s；dry-run 和单机测试通过前不能用于组合实机运动。禁止保留隐藏的 0.40 s 常量。

### 15.3 Cancel

Cancel callback：

1. 设置 cancel/preempt flag。
2. 立即停止未来 timer sampling。
3. 调用已单机验证的 Dobot Stop()。
4. 通过实际 q 变化率确认机械臂停稳。
5. Action 返回 CANCELED / PREEMPTED，不能返回 SUCCEEDED。

Stop() 失败、cancel 未及时处理或停止超时进入 ERROR；受控停止失败后才能调用配置的 EmergencyStop 升级路径。

Resume 不调用当前 Continue 服务，而是重新提交剩余 JointTrajectory。

### 15.4 Completion

不能只按 elapsed >= trajectory duration 返回 SUCCEEDED。至少要求：

- elapsed 已到轨迹终点。
- final max joint position error <= cr10_goal_joint_tol。
- estimated max joint velocity <= cr10_stop_velocity_tol。
- 连续三个有效反馈样本满足上述门限。

V1 初始参数为 cr10_goal_joint_tol=0.02 rad、cr10_stop_velocity_tol=0.01 rad/s、completion_settle_timeout=2.0 s；参数可在 CR10 单机验收后收紧。超过 completion_settle_timeout 仍不满足则 Action ABORTED。Action 的成功仅表示 CR10 关节轨迹完成，不等于 whole-body 或 EE 任务成功。

## 16. Pause / Resume / Abort V1

本节的 Ranger 硬件命令、CR10 cancel 和 Stop() 只适用于 dry_run=false。dry_run=true 只模拟时间冻结、状态转换和容差判断，禁止发布硬件命令或调用机械臂写服务。

### 16.1 Pause

pause_param_time 只来自 Real Executor command timeline：

~~~text
pause_param_time = executor.last_successfully_commanded_trajectory_parameter
~~~

它不是 REMANI planner wall-clock、`ros::Time - trajectory.start_time` 或 planner 对执行进度的估计。Executor 的 monotonic timeline 只在一次 whole-body command cycle 通过输出检查并成功发出后更新该参数；收到 Pause 时冻结最近一次成功 commanded 的值，停止继续推进。

Pause 时记录：

~~~text
pause_param_time
expected_pause_state = original candidate state(pause_param_time)
actual_stop_state
~~~

操作顺序：

1. 将 Executor command timeline 冻结在 last successfully commanded trajectory parameter，并记为 pause_param_time。
2. Ranger 立即、持续发布零速度。
3. cancel CR10 Action，并调用经验证的 Stop()。
4. 用 /odom 和 CR10 q 变化率确认两部分实际停稳。
5. 保存 actual_stop_state。
6. 两部分都确认停止后进入 PAUSED。

### 16.2 Resume

比较 actual_stop_state 与 frozen candidate(pause_param_time)：

- base position <= 0.02 m。
- base yaw <= 2 deg。
- 每个 arm joint <= 1 deg。

满足时：

- 从 pause_param_time 截取原 frozen candidate 的剩余部分。
- 不修改原路径几何、singul 或 whole-body synchronization。
- 不生成新的 whole-body smooth connector。
- 重新按第 14 节建立新的 T0。
- Ranger 闭环跟踪消除允许范围内的小起始误差。
- CR10 新 JointTrajectory 的 hold 使用当前实际 q；第一个原轨迹 q 必须仍在严格 joint tolerance 内。

超限时：

~~~text
Resume rejected
-> remain PAUSED
-> user Abort
-> Plan again from current actual state
~~~

### 16.3 Abort

Abort：

- Ranger 持续零速度直到确认停止。
- cancel CR10 Action 并调用经验证的 Stop()。
- 清除 assembling、cached 和 remaining candidate。
- candidate_id 失效且不可复用。
- 返回 READY 后必须重新 Plan。

### 16.4 Future Work

自动 smooth reconnect trajectory 移至 Future Work。若未来实现，必须单独设计 nonholonomic base path、collision check、singul、wheel speed、arm speed、acceleration continuity 和 synchronization；V1 不包含。

## 17. Real execution completion

sim 继续使用 REMANI 原 EXEC_TRAJ 和 EE actual-FK completion。

real 不使用 planner 的 EXEC_TRAJ completion，也不把 CR10 Action SUCCEEDED 当作任务成功。

Real Executor 到达 candidate 末端后，使用最新：

~~~text
Ranger /odom
+
CR10 actual q
~~~

计算：

- final_base_position_error。
- final_base_yaw_error。
- final_max_joint_error。
- actual EE FK。
- final_ee_pos_error。
- final_ee_rot_error。

expected final EE pose 由 Gate 对 frozen candidate 最终 base + q 做 FK 得到。

Real execution success 是显式 AND 条件：

~~~text
cr10_trajectory_completion
AND ranger_trajectory_completion
AND base_tolerance_satisfied
AND joint_tolerance_satisfied
AND ee_fk_tolerance_satisfied
AND feedback_and_robot_health_valid
~~~

其中：

- cr10_trajectory_completion：CR10 Action 完成，且实际关节误差与停止速度满足第 15.4 节。
- ranger_trajectory_completion：Executor 已按自身 command timeline 完整消费 frozen base trajectory，最终 Ranger command 已归零，并由实际 odom 速度确认停止。
- 后三项 tolerance 必须使用最新实际 odom / q 计算，不使用 candidate 期望值代替实际反馈。

Executor 不得只等待 CR10 Action result 后直接返回成功。上述任一布尔条件为 false、unknown 或 timeout，whole-body 结果均为 ERROR。

V1 初始终点门限：

| 项目 | 默认值 |
|---|---:|
| final base position | 0.05 m |
| final base yaw | 5 deg |
| final max arm joint | 0.02 rad |
| ee_reach_pos_tol | 0.02 m |
| ee_reach_rot_tol | 4 deg |

只有同时满足：

- CR10 trajectory completion。
- Ranger trajectory completion。
- base final tolerance。
- joint final tolerance。
- CR10 actual joint velocity stop tolerance。
- ee_reach_pos_tol。
- ee_reach_rot_tol。
- feedback 仍然新鲜且 RobotStatus 正常。

才进入 SUCCEEDED。

任一失败进入 ERROR，error code 为 TERMINAL_TOLERANCE_FAILURE 或对应安全错误。Action succeeded 只是一项输入条件。

Real Executor 发布 execution result；planner 可记录，但不得恢复内部 EXEC_TRAJ。

## 18. ROS 接口与 ExecutionState

### 18.1 控制与数据接口

| 接口 | 类型/语义 |
|---|---|
| /ee_goal_plan | std_msgs/Empty；Panel 让 Marker 发布缓存目标 |
| /ee_goal | geometry_msgs/PoseStamped；real plan-only planner 输入 |
| /remani/planner_candidate | quadrotor_msgs/PolynomialTraj；Planner -> Gate raw transaction control protocol |
| /remani/frozen_candidate | Gate-owned immutable frozen candidate；含 candidate_id、完整已验证轨迹与 validation result；Executor 唯一轨迹输入 |
| /remani/execute | ExecuteCandidate.srv；request 含 uint64 candidate_id |
| /remani/pause | std_srvs/Trigger |
| /remani/resume | std_srvs/Trigger |
| /remani/abort | std_srvs/Trigger |
| /remani/execution_result | 新增 ExecutionResult.msg；candidate_id、result code、final errors |
| /remani/execution_state | 统一只读状态；Panel 不自行推导 |
| /remani/candidate_robot | visualization_msgs/MarkerArray；隔离预览 |
| /remani/candidate_base_path | nav_msgs/Path |
| /remani/candidate_ee_path | nav_msgs/Path |
| /odom | Ranger 实际相对里程计 |
| /remani/cr10_joint_states_raw | CR10 驱动原始六轴状态 |
| /remani/cr10_joint_states | 固定 q1..q6 的 planner / Executor 状态 |
| /joint_states | RobotModel 显示状态 |
| /dobot_v4_bringup/msg/RobotStatus | 当前 CR10 驱动原始 connected/enabled 状态；不是完整 fault 状态 |
| /remani/cr10_status | 规范化 CR10 connected/enabled/error_status/robot_mode/age，只读安全状态 |
| /remani/hardware/ranger/cmd_vel | real launch namespace 隔离后的 Ranger 硬件命令；Executor 唯一合法 publisher |
| /remani/dry_run/ranger_cmd_vel_preview | dry-run diagnostics-only |
| /cr10_robot/joint_controller/follow_joint_trajectory | 正式 CR10 Action，仅 non-dry-run 可发送 |

### 18.2 ExecutionState 最小字段

~~~text
mode
execution_owner
dry_run
environment_mode

planner_state  # IDLE / PLANNING / HANDOFF

transaction_state
candidate_id
candidate_complete
candidate_valid

executor_state # NONE / PLANNED / EXECUTING / PAUSED / SUCCEEDED / ERROR

odom_ready
cr10_joint_ready
cr10_velocity_valid
tf_ready
robot_status_ready
robot_connected
robot_enabled
robot_error_status
robot_mode
action_server_ready
cr10_action_state
grid_map_ready
ranger_watchdog_ready
ranger_watchdog_timed_out

ranger_feedback_age
cr10_joint_feedback_age
tf_age
robot_status_age

base_tracking_error
joint_tracking_error

candidate_start_base_error
candidate_start_joint_error

requested_t0
ranger_trajectory_start_time
ranger_first_motion_time
cr10_first_motion_time
start_skew

final_base_error
final_base_position_error
final_base_yaw_error
final_joint_error
final_ee_pos_error
final_ee_rot_error

last_error_code
last_error
~~~

Panel 只展示该统一状态和服务响应，不根据 topic 数量、本地计时、planner_busy 或 planner 是否 IDLE 自行推导执行状态。planner_state 与 executor_state 必须作为两个独立字段传输。

## 19. 测试与分阶段验收

### 19.1 单元测试

- execution_owner 组合校验；real 永不进入内部 EXEC_TRAJ。
- real candidate handoff 后清除 planner execution ownership/timer state，同时保留 candidate_id correlation；不发布 planning/finish、不运行 periodic replan、不运行 planner execution safety timer。
- START / ADD / FINAL 正常、缺段、乱序、重复、跳号、ABORT、IMPOSSIBLE 和 timeout。
- sim 继续 ADD-only；real action 只由 Gate 按 transaction control protocol 解释。
- candidate_id 单调性、失效和 Execute 绑定；trajectory_id 每事务从 1 开始。
- `/remani/planner_candidate` 只到 Gate，Executor 只接受 `/remani/frozen_candidate`；绕过 Gate 的 raw candidate 被拒绝。
- planner_state 与 executor_state 的合法组合；planner IDLE 不得被解释成 execution idle。
- State Bridge name reorder、缺失、重复、NaN、velocity estimate 和两路输出。
- planner 只消费 /remani/cr10_joint_states 的六轴固定顺序。
- Ranger 零 Twist、直线除零、watchdog timeout 和 hardware topic ownership。
- Ranger watchdog 的 idle、dry-run、pre-start freshness handshake 和执行期 timeout 语义。
- dry_run 对 Ranger publisher、CR10 Action goal 和所有写服务的阻断。
- CR10 规范化状态对 ErrorStatus、robot_mode、unknown 和 timeout 的 fail-closed 行为。
- static-empty 参数 default、override、非法值拒绝、free GridMap、ESDF ready、边界 hard-invalid 和无 depth timeout。
- CR10 goal validation 的所有 reject 路径。
- CR10 timer callback 单步采样、cancel、Stop failure、正确 Action 状态和 completion settle。
- Pause 使用 Executor last successfully commanded parameter；Resume 使用 frozen candidate(pause_param_time)，确认没有 planner wall-clock 和 V1 connector。
- shared T0、hold mapping、accept deadline 和 start skew 记录。
- CR10 completion AND Ranger completion AND base/joint/EE tolerance；任一 false/unknown/timeout 均 ERROR。

### 19.2 集成测试

使用 fake Ranger、fake CR10 Action、fake RobotStatus 和实际 planner：

- sim 仍是 internal owner 和现有自动执行。
- real planner 只产生 transaction，Gate 拦住时 planner 不虚拟执行。
- real 执行时 planner_state=IDLE、executor_state=EXECUTING。
- real Plan 不产生硬件命令。
- FINAL 前 Execute 禁用。
- 旧 candidate_id Execute 被拒绝。
- EXECUTING / PAUSED 的新 START 不覆盖 frozen candidate。
- /cmd_vel 发布不能到达 Ranger hardware topic。
- dry-run 运行完整状态机但硬件输出计数为零。
- actual RobotModel 与 candidate preview 不互相污染。
- world -> base_link 只有 State Bridge 一个发布者，且与 /odom 数值一致。
- Pause 同时停止两条链；Resume 只恢复原剩余轨迹。
- final EE tolerance failure 进入 ERROR。
- CR10 Action succeeded 但 Ranger 未完成时进入 ERROR。

### 19.3 实机分阶段验证

1. dry_run 只读连接：实际 odom、q、TF、static-empty map、Marker 和 preview。
2. Ranger watchdog 与隔离 topic：无 Executor 时超时停机，普通 /cmd_vel 不驱动硬件。
3. Ranger-only 极短、低速执行和 Pause/Abort。
4. CR10 Action validation、非阻塞 sampling、cancel、Stop 和 completion 单机验证。
5. CR10-only 小范围、低速轨迹执行。
6. shared T0 synchronized dry-run，核对 hold 和 timestamp。
7. synchronized real short trajectory，测量 start_skew。
8. 早期、中段、末段 Pause/Resume；超容差必须拒绝。
9. final base/joint/EE completion 和故障注入。
10. 保持空场地与物理急停，逐步增加轨迹长度。

## 20. 实施顺序约束

本节只是设计依赖顺序，不是 implementation plan。

1. Planner real plan-only / candidate transaction protocol。
2. State Bridge + actual TF/readiness。
3. Trajectory Gate transaction assembly/validation。
4. RViz candidate preview + Panel。
5. dry_run Real Executor + Real Deployment State Machine。
6. Ranger command topic isolation + watchdog。
7. Ranger-only low-speed execution。
8. CR10 Action non-blocking refactor + strict validation/cancel。
9. CR10-only low-speed execution。
10. shared monotonic T0 + synchronized dry-run。
11. synchronized real execution。
12. Pause/Resume/Abort。
13. real final FK completion。
14. unified remani_real.launch。
15. sim regression + staged real acceptance。

硬性依赖：

~~~text
Planner real plan-only
  must complete before
hardware Real Executor implementation
~~~

CR10 non-blocking cancel 和 Ranger watchdog 未验证前，不允许组合实机运动。

## 21. 第一阶段验收标准

- mode:=sim 默认且行为不回归。
- mode:=real 强制 execution_owner=external。
- Gate 拦住 candidate 时 REMANI 不进入 EXEC_TRAJ、不虚拟推进、不发布 finish。
- 正式 transaction 为 START / ADD 1..N / FINAL。
- candidate_id 与 trajectory_id 不混用。
- /joint_states 只服务 RobotModel；planner 只接固定六轴 /remani/cr10_joint_states。
- ordinary /cmd_vel 无法驱动 Ranger；硬件只接 /remani/hardware/ranger/cmd_vel。
- dry_run 对 Ranger hardware topic、CR10 Action goal 和机械臂写服务实现零输出。
- static-empty GridMap 参数可配置且 V1 default 为 16 m × 12 m × 3 m / 0.05 m；有可查询 ESDF 和硬边界，但 UI 明示没有现实障碍感知。
- Execute 前无硬件运动；Execute 必须绑定当前 candidate_id。
- Pause 可停止；Resume 只在严格容差内恢复原剩余轨迹；超限 Abort + Replan。
- CR10 Action 可抢占、非阻塞、输入严格校验，cancel 不伪报 SUCCEEDED。
- shared T0 明确，start_skew 被记录并初始验收为 <=100 ms。
- real 成功需要 CR10 completion AND Ranger completion AND actual base/joint/EE FK 全部通过，Action succeeded 不等于任务成功。
- 物理急停和空场地是 V1 实机运行前置条件。

## 22. Source consistency self-check

修改后的正确语义：

| 检查项 | 最终语义 |
|---|---|
| Gate 拦住后 planner 是否执行 | 否；real planner plan-only，禁止进入内部 EXEC_TRAJ |
| handoff 后 planner 保留什么 | 清除 execution ownership、EXEC_TRAJ/timer state；仅保留 candidate_id、external metadata 和 result correlation，随后 planner_state=IDLE |
| START / FINAL 是否为当前行为 | 否；这是 real candidate protocol 必须新增的行为 |
| real PolynomialTraj action 由谁解释 | 仅 Trajectory Gate 按 begin/append/commit/invalidate/failure transaction control protocol 解释；sim 保持 ADD-only |
| trajectory_id 是否为 candidate_id | 否；前者是 segment 序号，后者是 Gate 单调 candidate version |
| Executor 是否消费 planner raw trajectory | 否；数据流固定为 Planner -> /remani/planner_candidate -> Gate -> /remani/frozen_candidate -> Executor |
| planner 是否用 busy 表示执行状态 | 否；planner_state 与 executor_state 分离，IDLE + EXECUTING 是合法组合 |
| planner 是否接完整 /joint_states | 否；只接 /remani/cr10_joint_states |
| rogue publisher 检测是否足够 | 否；先做 /remani/hardware/ranger/cmd_vel launch namespace 隔离，再做唯一 publisher 检查 |
| Resume 是否生成 connector | 否；V1 小误差恢复原剩余轨迹，超限 Abort + Replan |
| Pause 参数时间来自哪里 | 仅 Executor last successfully commanded timeline，不来自 planner wall-clock 或 trajectory.start_time |
| Action SUCCEEDED 是否等于 whole-body succeeded | 否；需要 CR10 AND Ranger completion AND actual base/joint/EE tolerance |
| 无 depth/cloud 是否可忽略 GridMap | 否；real V1 使用 explicit static-empty GridMap + ESDF + boundary |
| static-empty 尺寸是否固定 | 否；16 m × 12 m × 3 m / 0.05 m 是 V1 default，必须可配置且禁止 hard-code |
| real 谁负责 execution safety | 仅 Real Executor |
| sim 谁负责 execution lifecycle | REMANI 原 FSM |
| 当前 RobotStatus 是否能完整表示 fault | 否；必须增加规范化只读 error_status / robot_mode telemetry，不能从 connected/enabled 推断 |
| READY 或 dry-run 的 watchdog timeout 是否等于执行故障 | 否；安全空闲允许 timeout，正式 Execute 接受前必须完成零命令 freshness handshake，执行期 timeout 才进入 ERROR |
| world -> base_link 谁发布 | 仅 State Bridge；Ranger driver 的 publish_odom_tf=false，且两者使用同一 /odom |

全文不再依赖以下错误假设：

- REMANI EXEC_TRAJ 和 Real Executor EXECUTING 可以同时存在。
- planner wall-clock 可以代表未执行 candidate 的实际进度。
- 完整 RobotModel JointState 的前六项天然是 CR10。
- 检测 /cmd_vel publisher 数量就能阻止其他节点驱动底盘。
- 简单 joint-wise quintic 可以作为非完整约束底盘的恢复段。
- CR10 elapsed duration 到期即可成功。

## 23. Remaining risk 与 plan-ready 判定

设计层面已经关闭 implementation-blocking ownership、protocol、topic、state、timing、fault telemetry 和 completion 歧义。

仍需在实现/验收阶段验证的硬件事实：

- Ranger SDK/固件停止响应和 watchdog 实际停机时间。
- Dobot Stop() 对 ServoJ 的受控停止效果。
- servoj_period=0.10 s 的实机可用性。
- CR10 Action accept latency 和最终 start_skew。
- 实机反馈噪声对应的 velocity、tracking 和 completion threshold。

这些是已定义失败处理与阶段门槛的硬件验证项，不是未定义的架构决策。当前 spec 已经 implementation-plan ready；本轮不生成 implementation plan。
