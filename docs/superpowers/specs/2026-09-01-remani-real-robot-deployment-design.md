# REMANI Ranger+CR10 实机部署与人工确认执行设计

**Date:** 2026-09-01
**Status:** Draft — awaiting user review before implementation planning
**Chosen approach:** 方案 1，规划器与实机执行器分离
**Scope:** 保留现有 REMANI 仿真模式，新增可选择的 Ranger+CR10 实机模式；在 RViz 中设置末端目标、仅规划并预览全身轨迹，待操作者确认后再执行；执行期间支持 Pause、Resume 和 Abort。

## 1. 背景与现状

当前仓库已经具备以下基础：

- REMANI 接收 Ranger 里程计和 CR10 六轴关节状态，以底盘平面状态和六关节状态规划全身轨迹。
- `ee_goal_marker_node` 已实现显式 `Plan` 语义：拖动末端 Marker 只缓存目标，执行 Plan 操作后才发布 `/ee_goal`。
- 规划器发布 `quadrotor_msgs/PolynomialTraj` 到 `planning/trajectory`。
- 当前仿真中的 `mm_controller` 会立即消费该轨迹并执行，不存在实机所需的人工确认门。
- Ranger 驱动接收 `geometry_msgs/Twist`，发布 `/odom`，并可发布 `odom_frame -> base_link` TF。
- CR10 驱动发布名为 `joint1 ... joint6` 的 `/joint_states`，并提供 `/cr10_robot/joint_controller/follow_joint_trajectory` Action。
- CR10 已验证过通过 MoveIt 设置末端目标后运动，但尚未完成 REMANI 全身轨迹、Action 取消、暂停保持和底盘同步的实机验证。
- 当前 CR10 Action 的轨迹回调在单线程 callback queue 中阻塞执行整条轨迹；现有 cancel 仅停止 ROS timer 并返回 `SUCCEEDED`，不能证明可在运动中及时抢占。该问题必须在启用 Pause 前修复。

当前仿真会根据模拟 `/odom` 和模拟关节状态更新 RViz。仓库还没有本设计中的统一实机 launch，因此**现状不能保证实机启动时 RViz 已按实际姿态初始化**。本设计将实际状态同步列为实机进入 `READY` 的硬门槛，详见第 6 节。

## 2. 目标与非目标

### 2.1 目标

- 使用一个启动参数选择现有仿真模式或新增实机模式。
- `mode:=sim` 保持当前仿真入口和执行语义，不因实机功能发生行为回归。
- `mode:=real` 一次启动 Ranger 驱动、CR10 bringup、REMANI、RViz 和实机执行链路。
- 实机启动后，RViz 中的底盘和机械臂模型由当前实际反馈初始化并持续更新。
- 在 RViz 设置末端目标后，`Plan` 只生成和显示候选全身轨迹，绝不驱动硬件。
- 操作者确认轨迹后点击 `Execute`，才允许向 Ranger 和 CR10 输出运动指令。
- 执行中可随时 `Pause`，确认停稳后可在容差内 `Resume`；也可 `Abort` 并清除轨迹。
- 实机执行默认使用保守、参数化的速度与误差限制。
- 第一阶段在清空场地中运行，不依赖 D435 或激光雷达在线更新碰撞环境。

### 2.2 非目标

- 第一阶段不提供动态障碍物检测、在线避障或执行中重规划。
- 不改 REMANI 的 Hybrid A*、RRT、多项式优化和 whole-body terminal 求解算法。
- 不把 CR10 改成笛卡尔伺服或阻抗控制。
- 不实现跨 Pause 状态的大偏差自动纠偏；偏差超限时必须重新规划。
- 不用软件按钮替代物理急停。
- 第一阶段不引入外部定位。启动时 Ranger 当前物理位置被定义为 `world` 原点，之后由轮式里程计积分。

## 3. 已确认的使用方式

统一入口：

```bash
# 保持当前仿真行为
./remani_planner/run_remani.sh mode:=sim

# 连接实机、检查反馈和完整执行链路，但屏蔽运动输出
./remani_planner/run_remani.sh \
  mode:=real \
  dry_run:=true \
  robot_ip:=192.168.5.1 \
  can_interface:=can0

# 正式向实机输出命令
./remani_planner/run_remani.sh \
  mode:=real \
  dry_run:=false \
  robot_ip:=192.168.5.1 \
  can_interface:=can0
```

`mode` 默认值为 `sim`。`mode:=sim` 继续启动现有 `remani_sim.launch`，不强制经过实机 Trajectory Gate。`mode:=real` 启动新的 `remani_real.launch`。

操作者工作流：

```text
启动实机系统
  -> 等待真实状态同步，面板显示 READY
  -> 在 RViz 拖动末端目标 Marker
  -> 点击 Plan
  -> 查看候选全身轨迹动画、路径、时长和限制检查
  -> 人工确认轨迹正确
  -> 点击 Execute
  -> 执行中可 Pause / Resume / Abort
```

拖动 Marker 只更新目标，不发布运动命令，也不自动触发规划。

## 4. 总体架构

采用规划器和实机执行器分离的结构：

```text
RViz EE Marker
      │ cached target
      ▼
RViz Panel: Plan
      │ /ee_goal
      ▼
REMANI Planner
      │ candidate PolynomialTraj transaction
      ▼
Trajectory Gate ───────────────► RViz candidate preview
      │ cached and validated
      │
      └── Execute confirmed ───► Real Executor
                                   ├──► Ranger velocity command
                                   └──► CR10 FollowJointTrajectory

Actual /odom + CR10 joint feedback
      ├──► State Bridge / readiness monitor
      ├──► REMANI initial and current state
      ├──► Real Executor feedback control
      └──► TF + RobotModel actual pose in RViz
```

### 4.1 REMANI Planner

保留规划器的核心职责：根据当前实际底盘、机械臂状态和末端目标生成完整的 current-to-goal 全身轨迹。

实机 launch 将规划器原 `planning/trajectory` 输出重映射为候选轨迹话题，防止现有 `mm_controller` 或其他消费者直接执行。第一阶段保持 `global_plan:=true`，一条获批轨迹代表从当前状态到目标的完整轨迹；执行期间不做周期性重规划。

### 4.2 Trajectory Gate

Gate 是规划与硬件之间的人工确认边界，职责为：

- 按一次规划事务接收并组装候选多项式轨迹。
- 使用 `ACTION_WARN_START`、一个或多个轨迹数据消息以及 `ACTION_WARN_FINAL` 确认事务完整。
- 校验维数、时间单调性、数值有限性、连续性、起点和速度/加速度限制。
- 缓存候选轨迹并发布独立预览数据。
- 只有收到显式 `Execute` 且所有 readiness 条件仍然满足时，才把冻结轨迹交给 Real Executor。
- 空闲时新 `Plan` 会替换旧候选；执行中不接受新轨迹覆盖。

候选轨迹不得直接连接硬件控制话题。

### 4.3 Real Executor

Executor 负责：

- 从同一条冻结轨迹和同一单调时间轴生成 Ranger 与 CR10 指令。
- 用实际 `/odom` 对底盘做闭环跟踪。
- 生成并提交 CR10 `JointTrajectory`。
- 执行运行期限制、反馈超时和跟踪误差检查。
- 实现 Pause、Resume、Abort 和异常停机。
- 在 `dry_run:=true` 时运行完整状态机和采样逻辑，但禁止输出硬件运动指令。

### 4.4 State Bridge

State Bridge 隔离驱动接口差异：

- 按 `JointState.name` 抽取 `joint1 ... joint6`，映射为模型需要的 `cr10_joint1 ... cr10_joint6`。
- 输出给 REMANI 的六轴状态顺序固定且经过完整性检查，不能依赖输入数组顺序。
- 为 `robot_state_publisher` 提供组合 `/joint_states`。CR10 六轴使用实际反馈；当前没有反馈的转向、车轮和夹爪显示关节使用明确的静态默认值，不得冒充实际测量值。
- 转发时间戳和健康状态，检测缺关节、重复关节、NaN 和超时。

### 4.5 Ranger command watchdog

现有 Ranger ROS 驱动没有可见的 `/cmd_vel` 超时停机逻辑，而且全零 Twist 会经过 `linear/angular = 0/0` 的转弯半径计算路径。实机部署前必须在驱动边界补齐：

- 全零或近零 Twist 走显式停止分支，不计算半径，不得向 SDK 传入 NaN。
- 纯直线命令 `angular.z == 0` 显式使用零转角，不做除零。
- 增加可配置 `cmd_vel_timeout`，默认 `0.20 s`。超过时限未收到新命令时，驱动主动发送零运动命令。
- 启动时、退出时和通讯异常时发送一次显式停止命令。
- 单元测试覆盖零速度、纯直线、纯旋转和超时；实机单独验证底盘确实停下。

该驱动看门狗保护 Executor 进程崩溃或 ROS 指令流中断的情况，但不替代 Ranger 自身固件保护和物理急停。

## 5. 执行状态机

状态定义：

```text
NOT_READY
READY
PLANNING
PLANNED
EXECUTING
PAUSED
SUCCEEDED
ERROR
```

主要转换：

```text
NOT_READY --all feedback synchronized--> READY
READY --Plan--> PLANNING
PLANNING --valid complete candidate--> PLANNED
PLANNING --no feasible path/Abort--> READY
PLANNING --malformed transaction or safety fault--> ERROR
PLANNED --Plan--> PLANNING       # 替换旧候选
PLANNED --Execute--> EXECUTING
EXECUTING --Pause and stop confirmed--> PAUSED
PAUSED --Resume and tolerance passed--> EXECUTING
EXECUTING --completed--> SUCCEEDED
PLANNED/EXECUTING/PAUSED --Abort--> READY
any active state --safety fault--> ERROR
ERROR --health restored and Abort--> READY
```

普通规划无解不是硬件故障：显示规划失败原因并返回 `READY`。轨迹事务损坏、反馈故障或其他安全异常进入 `ERROR`。`Abort` 清除候选与剩余轨迹，之后必须重新 Plan；`ERROR` 中旧轨迹同样不可恢复。

按钮权限：

| 状态 | 可用操作 |
|---|---|
| `NOT_READY` | 无 |
| `READY` | Plan |
| `PLANNING` | Abort |
| `PLANNED` | Plan、Execute、Abort |
| `EXECUTING` | Pause、Abort |
| `PAUSED` | Resume、Abort |
| `SUCCEEDED` | Plan |
| `ERROR` | Abort，仅用于健康恢复后的安全复位 |

## 6. 启动时按实际姿态初始化 RViz

### 6.1 明确结论

实机模式必须根据启动时收到的实际反馈初始化 RViz，而不是使用 YAML 中的仿真初值或全零关节角。

对底盘而言，Ranger 驱动提供的是相对轮式里程计，不是房间中的绝对定位。因此第一阶段采用：

```text
启动瞬间的实际 Ranger 位姿 := world 原点 (0, 0, 0)
之后的实际相对运动 := Ranger /odom 积分结果
```

这表示 RViz 中底盘的**当前相对姿态**是实际反馈，但不表示已知道机器人在房间地图中的绝对坐标。未来接入 SLAM/定位时，再增加 `map -> odom`，本阶段不引入。

对 CR10 而言，RViz 必须使用驱动读回的实际 `joint1 ... joint6`，经名称映射后显示，不使用预设 home pose。

### 6.2 TF 与 JointState 链路

实机 launch 配置：

- Ranger 驱动：`odom_frame:=world`、`base_frame:=base_link`、`publish_odom_tf:=true`。
- Ranger 驱动发布 `world -> base_link`，其数值来自 `/odom`。
- `robot_state_publisher` 加载组合 Ranger+CR10 URDF。
- State Bridge 发布映射后的 `/joint_states`，从而产生 `base_link -> ... -> cr10_Link6` 的整机 TF。
- RViz Fixed Frame 设为 `world`。

禁止并存第二个 `world/odom -> base_link` 发布者，避免 TF 跳变。

### 6.3 首帧同步门控

实机启动保持 `NOT_READY`，直到同时满足：

1. Ranger `/odom` 已收到连续有效样本，时间戳新鲜，位置和四元数有限。
2. TF 中存在新鲜的 `world -> base_link`。
3. Dobot RobotStatus 表示已连接；正式执行还要求机械臂已使能。
4. 已收到至少三个连续、完整且有限的 CR10 六轴实际关节样本。
5. State Bridge 完成名称映射，六个模型关节均可更新。
6. `robot_state_publisher` 已形成从 `world` 到 CR10 末端的完整 TF 链。
7. CR10 FollowJointTrajectory Action Server 可用；`dry_run` 仍检查其可用性，但不提交运动目标。

在上述条件满足前：

- Panel 明确显示等待项和反馈年龄。
- `Plan`、`Execute`、`Resume` 全部禁用。
- 末端目标 Marker 不使用猜测姿态；在同步完成前隐藏。
- 候选轨迹不能生成。

状态同步完成后：

- RViz RobotModel 立即显示实际底盘相对位姿和 CR10 实际关节姿态。
- REMANI 用同一份实际状态初始化 `mm_state_pos_` 和 `mm_car_yaw_`。
- 规划器计算并发布实际 `/ee_current_pose`。
- 目标 Marker 首次放置在实际末端位姿，而不是固定默认点。
- Panel 才进入 `READY`。

### 6.4 实际模型与候选预览隔离

实际 RobotModel 始终由真实 `/odom`、TF 和 `/joint_states` 驱动。候选轨迹预览使用 `visualization_msgs/MarkerArray` 的独立 namespace 播放采样后的全身模型，同时使用两条 `nav_msgs/Path` 显示底盘和末端路径；不得向真实 `/joint_states` 或 `world -> base_link` 写入预览状态。

因此在 RViz 中应能同时区分：

- 不透明或固定配色的当前实际机器人；
- 半透明的候选轨迹动画/稀疏全身姿态；
- 底盘路径和末端路径。

在点击 Execute 前，再次比较最新实际状态和候选轨迹起点；超过配置容差即使画面曾经正确，也必须拒绝执行并要求重新 Plan。

## 7. RViz 交互与候选轨迹检查

新增专用 RViz Panel，显示：

- 模式：`SIM`、`REAL-DRY-RUN` 或 `REAL`。
- 当前状态机状态。
- Ranger odom/TF、CR10 joint/status/action、planner 和 executor readiness。
- 当前反馈年龄和最后错误。
- 候选轨迹总时长、最大底盘线速度、最大底盘角速度、最大关节速度。
- 候选起点与最新实际状态的差值。
- 执行进度和暂停点。
- `Plan / Execute / Pause / Resume / Abort` 按钮。

点击 `Plan` 后，RViz 自动显示：

- 底盘路径、航向和正向/倒车分段。
- 末端执行器路径。
- CR10 关节运动动画。
- 采样后的稀疏全身姿态。
- 起点、终点和轨迹时长。
- Gate 检查结果。无效或超限部分显示红色，且 `Execute` 禁用。

轨迹完整且通过检查后进入 `PLANNED`。操作者可以反复查看预览；只有显式点击 `Execute` 才视为确认。设计不要求 Marker 移动后自动重规划，也不要求执行前再弹第二个确认框。

## 8. 实机执行与时间同步

### 8.1 共享时间轴

Gate 冻结获批轨迹后，Executor 从同一条多项式轨迹生成底盘和机械臂指令。

CR10 `JointTrajectory` 开头增加可配置静止前导时间。Executor 先提交机械臂 Action；Action 进入活动状态后确定共同起始时刻。前导时间结束前底盘持续发送零速度，结束时底盘与机械臂共同进入轨迹 `t=0`。后续使用单调时钟计算经过时间，避免 ROS 时间跳变造成错位。

不得让底盘先启动后再等待机械臂 Action。

### 8.2 Ranger 跟踪

Executor 以默认 50 Hz 采样底盘轨迹：

- 从轨迹得到期望世界系位置、速度、加速度、航向和奇异方向。
- 根据 `singul` 正确处理正向与倒车，不能只根据瞬时速度符号猜测。
- 结合实际 `/odom` 做位置和航向反馈修正。
- 转换为 Ranger 可接受的 `linear.x` 和 `angular.z`。
- 在线监视反馈年龄、跟踪误差、命令速度和实际速度。
- 以高于驱动 `cmd_vel_timeout` 要求的固定频率持续刷新指令；退出 `EXECUTING` 后持续刷新全零命令，直到驱动确认停止。

Real Executor 是本 launch 中唯一运动命令拥有者。启动或运行中检测到未授权 `/cmd_vel` 发布源时，保持零速度并进入 `ERROR`。

### 8.3 CR10 跟踪

Executor 按可配置周期采样六关节位置和速度，构造名称为 `joint1 ... joint6` 的 `trajectory_msgs/JointTrajectory`，提交到：

```text
/cr10_robot/joint_controller/follow_joint_trajectory
```

现有 CR10 驱动内部 `ServoJ` 调度周期存在硬编码值，且 `moveHandle()` 在一个 callback 中循环和 sleep 完成整条轨迹。节点只运行一个 spinner 线程，因此运动期间 cancel callback 不能保证及时执行。实现阶段必须先把它重构为可抢占的非阻塞 Action 状态机：

- Goal callback 只校验并缓存轨迹、记录起始时间，然后返回。
- 高频 timer 每次 callback 只采样和发送一个 `ServoJ` 点，然后立即返回。
- `servoj_period` 参数默认 `0.10 s`，真实启用前必须通过 CR10 单机测试；禁止保留隐藏的 `0.40 s` 常量。
- Cancel callback 设置抢占标志、停止后续采样、调用 Dobot `/dobot_v4_bringup/srv/Stop`，并把 Action 置为 `CANCELED/PREEMPTED`，不能伪报 `SUCCEEDED`。
- Resume 不调用 Dobot `Continue` 服务，而是由 Executor 从当前反馈重新提交带平滑衔接段的剩余 FollowJointTrajectory。

完成上述重构后，在 CR10 单机低速测试中验证：

- 时间戳是否按预期执行；
- cancel 是否能在一个 `servoj_period` 加 ROS 调度裕量内停止继续发送轨迹点；
- 机械臂是否实际减速并保持；
- Action 返回状态是否与实际关节速度一致。

不能只根据 Action 返回 `SUCCEEDED` 或 cancel 已接受来判断机械臂已停稳。

## 9. 速度、起点和运行时限制

第一阶段默认限制：

| 项目 | 默认值 |
|---|---:|
| Ranger 最大线速度 | `0.10 m/s` |
| Ranger 最大角速度 | `0.15 rad/s` |
| CR10 各关节最大速度 | `0.10 rad/s` |
| Resume 底盘位置容差 | `0.05 m` |
| Resume 底盘航向容差 | `5 deg` |
| Resume 各关节容差 | `3 deg` |

加速度、跟踪误差、反馈超时、停止确认速度和停止超时同样参数化。

限制执行两次：

1. Gate 在 `PLANNED` 前检查整条候选轨迹。
2. Executor 在运行中监控命令和反馈。

轨迹超限时必须拒绝执行，不能简单截断单个底盘或关节速度，因为独立截断会破坏全身轨迹的时间同步。需要降低规划/time-scaling 参数后重新规划。

每次 Execute 前重新检查：

- 反馈仍然新鲜；
- Action Server 和 RobotStatus 正常；
- 候选轨迹起点与当前实机状态在容差内；
- 没有未授权命令发布者；
- 轨迹仍是当前 Gate 缓存的版本。

## 10. Pause、Resume、Abort 与异常停机

### 10.1 Pause

点击 Pause 后立即：

1. Ranger 连续发布零速度。
2. 取消 CR10 当前 FollowJointTrajectory goal，停止继续下发轨迹点，并调用已单机验证的 Dobot `Stop()` 受控停止。
3. 通过 `/odom` 和关节状态确认底盘、机械臂速度均低于停止阈值。
4. 保存暂停时刻、剩余轨迹和实际停止状态。
5. 两部分均确认停止后才进入 `PAUSED`。

普通 Pause 使用受控停止，不把 EmergencyStop 当作日常暂停接口。

### 10.2 Resume

Resume 比较实际停止状态和原轨迹暂停点：

- 底盘位置误差不超过 `0.05 m`；
- 航向误差不超过 `5 deg`；
- 每个机械臂关节误差不超过 `3 deg`。

满足容差时：

- 截取暂停点之后的剩余轨迹；
- 从当前实机状态加入短暂、限速限加速度的平滑衔接段；
- 对剩余轨迹重新计时；
- 按第 8 节重新建立底盘与机械臂共同时间轴。

不满足容差时拒绝 Resume，清楚显示偏差，并要求 Abort 后重新 Plan。Resume 不能从原轨迹起点重放。

### 10.3 Abort

Abort：

- 底盘持续零速度；
- 取消 CR10 Action 并确认停止；
- 清除候选和剩余轨迹；
- 返回 `READY` 后必须重新 Plan。

### 10.4 自动进入 ERROR 的条件

- `/odom`、CR10 joint state、RobotStatus 或 TF 超时。
- CR10 Action Server 消失、拒绝目标或报告失败。
- CR10 cancel 未在规定时间内被 Action Server 接受，或 Dobot `Stop()` 调用失败。
- 起点或运行中跟踪误差持续超限。
- 命令或实际速度超过安全阈值。
- Pause/Abort 后未在配置时间内确认停稳。
- 出现未授权 `/cmd_vel` 发布者。
- 候选轨迹不完整、数值异常或版本不一致。

发生错误时，底盘保持零速度并取消机械臂 Action。如果受控停止在规定时间内失败，则调用经过单机验证的、可配置的 CR10 紧急停止升级路径，进入需要人工检查和复位的 `ERROR`。物理急停始终是实机运行前置条件。

## 11. Launch、话题与服务边界

### 11.1 `mode:=sim`

- 启动现有 `remani_sim.launch`。
- 保留现有模拟里程计、模拟关节状态、`mm_controller` 和 fake MM 链路。
- 不改变当前自动执行语义。

### 11.2 `mode:=real`

新 `remani_real.launch` 一次启动：

- Ranger CAN 驱动，参数 `can_interface`。
- Dobot v4 bringup，环境/参数选择 CR10，参数 `robot_ip`。
- Ranger+CR10 `robot_description` 和 `robot_state_publisher`。
- State Bridge 和 startup readiness monitor。
- 第一阶段空环境地图。
- REMANI Planner，候选轨迹输出被重映射。
- Trajectory Gate 和 Real Executor。
- RViz 及专用 Panel。

### 11.3 逻辑接口

控制接口类型固定如下：

| 接口 | 语义 |
|---|---|
| `/ee_goal_plan` (`std_msgs/Empty`) | Panel 的 Plan 命令现有 Marker 发布缓存目标 |
| `/ee_goal` (`geometry_msgs/PoseStamped`) | Marker 将缓存的末端目标交给规划器 |
| `/remani/candidate_trajectory` (`quadrotor_msgs/PolynomialTraj`) | 规划器到 Gate 的候选事务 |
| `/remani/execution_state`（新增 `ExecutionState.msg`） | 状态、readiness、候选编号、进度、反馈年龄、最大速度和错误文本 |
| `/remani/execute` (`std_srvs/Trigger`) | 显式人工执行确认 |
| `/remani/pause` (`std_srvs/Trigger`) | 请求受控暂停 |
| `/remani/resume` (`std_srvs/Trigger`) | 请求按容差恢复；失败响应携带超限原因 |
| `/remani/abort` (`std_srvs/Trigger`) | 停止并清除轨迹，或在健康恢复后复位错误 |
| `/remani/candidate_robot` (`visualization_msgs/MarkerArray`) | 与实际状态隔离的全身轨迹动画 |
| `/remani/candidate_base_path` (`nav_msgs/Path`) | 候选底盘路径 |
| `/remani/candidate_ee_path` (`nav_msgs/Path`) | 候选末端路径 |
| `/odom` | Ranger 实际相对里程计 |
| `/joint_states` | 映射后的整机 RobotModel 状态 |
| `/remani/cr10_joint_states` | 固定顺序六轴规划/执行反馈 |
| `/cmd_vel` | 仅由实机命令所有者输出给 Ranger |
| CR10 FollowJointTrajectory | Executor 到机械臂的轨迹执行 |

`ExecutionState.msg` 至少包含：枚举状态、模式、`dry_run`、候选轨迹编号、各 readiness 布尔值、各反馈年龄、总时长、执行进度、候选最大速度、起点误差和最后错误文本。Panel 只展示该状态，不自己推导安全状态。

## 12. 测试设计

### 12.1 单元测试

- Gate 对完整、缺段、乱序、重复和超限事务的处理。
- 状态机全部合法/非法转换。
- 正向、倒车和奇异段底盘命令计算。
- JointState 按名称映射、乱序、缺失、NaN 和陈旧消息。
- 启动同步门控，尤其是禁止全零默认姿态误判为已同步。
- Ranger 全零 Twist 不产生 NaN，纯直线不除零，命令超时触发显式停止。
- 候选起点和最新实际状态检查。
- Pause 剩余轨迹截取、Resume 重新计时和平滑衔接。
- CR10 非阻塞 Action sampler、cancel 抢占、正确 canceled 状态和 `Stop()` 失败路径。
- `dry_run` 屏蔽所有硬件运动输出。

### 12.2 集成测试

使用假的 Ranger odom/TF、Dobot RobotStatus 和 CR10 Action Server，验证：

- 首帧同步完成前 RobotModel/Marker/readiness 的行为。
- Plan 不产生硬件命令。
- Execute 前没有底盘或机械臂运动输出。
- Pause 同时停止两条执行链。
- Resume 偏差超限时拒绝继续。
- Abort 后旧轨迹不能再次执行。
- 反馈、TF 或 Action 中断时进入 `ERROR`。
- 候选预览不污染真实 TF 和 `/joint_states`。
- `mode:=sim` 原有规划和仿真执行通过回归测试。

### 12.3 实机分阶段调试

1. **只连接反馈：** `dry_run:=true`，核对底盘启动原点、CR10 实际关节、TF、实际末端 Marker 和预览方向。
2. **只验证 Ranger：** 禁用 CR10 输出，在清空区域执行极短直线和小角度旋转，验证速度、方向、Pause 和 Abort。
3. **只验证 CR10：** 禁用底盘输出，从安全姿态执行小范围单关节及末端轨迹，重点测量 Action cancel、停止和保持。
4. **验证同步：** 执行低速、短距离的全身轨迹，比较共同起始时刻和进度。
5. **验证恢复：** 在早期、中段和末段测试 Pause/Resume；人工制造超容差偏差，确认拒绝恢复。
6. **组合验收：** 逐步增加轨迹长度，但第一阶段始终保持场地清空并配备物理急停。

## 13. 第一阶段验收标准

- 一条命令可选择仿真或实机，且 `mode:=sim` 行为不回归。
- 实机启动时，RViz 在实际状态同步前保持 `NOT_READY`，同步后显示真实底盘相对位姿和 CR10 当前关节姿态。
- 末端 Marker 初始位置来自实际 FK，而非固定默认值。
- 点击 Execute 前，实机不会产生任何运动。
- 候选轨迹可在 RViz 中检查底盘、机械臂和末端的同步运动。
- 轨迹不完整、超限、反馈不健康或起点偏差过大时不能执行。
- Pause 请求后，Ranger 零速度命令在一个控制周期内发出；CR10 cancel/hold 请求延迟和实际停机时间分别记录并验收。
- Resume 从剩余轨迹继续，并对超容差偏差拒绝恢复。
- Abort 和任何安全错误都使旧轨迹失效。
- 第一阶段不宣称具备动态障碍物安全能力。

## 14. 实施顺序约束

后续实现计划应按以下依赖顺序拆分：

1. State Bridge、真实 TF/JointState 初始化和 readiness。
2. 候选轨迹事务与 Trajectory Gate。
3. RViz Panel 和只读候选预览。
4. `dry_run` Real Executor 及状态机。
5. Ranger 单独执行与停止。
6. CR10 单独执行、取消、保持和驱动周期参数化。
7. 同步执行、Pause/Resume/Abort。
8. 统一 real launch、回归测试和实机分阶段验收。

在前一阶段验证通过前，不进入下一阶段的组合实机运动。
