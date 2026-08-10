# WBC：移动机械臂全身控制实验仓库

本仓库面向 **Ranger 底盘 + CR10 机械臂** 的全身控制（Whole-Body Control）研究与集成。

目标能力：

- 给定末端 **6D 位姿**
- 协调 **底盘 + 机械臂** 运动到达
- 逐步加入 **自碰 / 环境避障**

当前已在本机验证：

- **OCS2** Mobile Manipulator 官方 demo（ROS1 / Noetic）
- **Holistic MM** 反应式全身伺服（conda 环境 `holistic`）
- **REMANI** Ranger+CR10 仿真规划闭环（ROS1 / Noetic，`remani_sim.launch`）

## 目录结构

| 路径 | 说明 |
|------|------|
| [`agx/`](agx/) | 真机相关：Ranger、CR10、整机 URDF、传感器、夹爪等 |
| [`ocs2_ws/`](ocs2_ws/) | OCS2（ROS1）独立 catkin 工作空间 |
| [`remani_planner/`](remani_planner/) | REMANI-Planner：在线全身轨迹规划（含 Ranger+CR10 仿真） |
| [`robotics-toolbox-python/`](robotics-toolbox-python/) | Robotics Toolbox for Python 源码（含 Holistic/NEO 相关示例） |
| [`scripts/`](scripts/) | 本仓库适配脚本（如 Holistic RTB 1.3 演示） |

## 设备模型（agx）

整机 URDF：

- [`agx/rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf`](agx/rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf)

关键链路：

- 底盘：`base_link`，控制入口多为 `/cmd_vel`
- 机械臂：`cr10_base_link` → `cr10_joint1..6` → `cr10_Link6` / `gripper_base_link`
- 真机臂：Dobot bringup / `FollowJointTrajectory`（关节名可能是 `joint1..6`，与整机 `cr10_joint*` 需桥接）

## 方案对比（简况）

| 方案 | 末端 6D 输入 | 全身底盘+臂 | 避障 | 形态 | 本仓库状态 |
|------|-------------|-------------|------|------|------------|
| **OCS2 Mobile Manipulator** | 支持 | 支持（SE(2)+臂） | 自碰可配；环境障碍需扩展 | ROS1 MPC | 官方 demo 已跑通 |
| **Holistic MM（RTB）** | 支持 | 支持（反应式 QP） | 限位/可扩展；非全局规划 | Python + Swift | conda 已测通 |
| **REMANI** | 弱（偏 `[x,y,q]` / 2D Nav Goal） | 支持 | 在线 ESDF/重规划较强 | ROS1 规划器 | Ranger+CR10 仿真已跑通；部分场景仍有穿模 |
| **RTB 单独作库** | IK/轨迹强 | 需自建 | 视用法 | Python 库 | 源码在库 |

### 本机测试结论一览

| 方案 | 场景 | 结果 | 备注 |
|------|------|------|------|
| **OCS2** | Ridgeback+UR5 interactive Goal | 通过 | 底盘+臂协同跟踪 6D；默认自碰关闭，可见臂–底盘穿模 |
| **Holistic** | Frankie 非全向，预设/自定义 `Tep` | 通过 | ~234 步到位，末端位置误差约 4.16→0.05；无复杂全局绕障 |
| **REMANI** | Ranger+CR10 方块图 + 过桥 | 部分通过 | 短/中程多次 `Success` 并执行完成；难路径会 Hybrid A* 超时后切 RRT；第三次远点仍观测到臂–障碍穿模 |

---

## 1. OCS2

### 1.1 环境

- Ubuntu 20.04
- ROS Noetic
- 工作空间：[`ocs2_ws/`](ocs2_ws/)

源码（`ocs2_ws/src/`）：

- `ocs2`（`main`，ROS1）
- `pinocchio`
- `hpp-fcl`
- `ocs2_robotic_assets`

系统依赖：

```bash
sudo apt-get update
sudo apt-get install -y \
  libglpk-dev \
  ros-noetic-pybind11-catkin \
  ros-noetic-rqt-multiplot \
  liburdfdom-dev liboctomap-dev libassimp-dev \
  python3-catkin-tools
```

### 1.2 编译

```bash
cd /home/gzz/Codes/wbc/ocs2_ws

# 首次初始化示例（已配置可跳过）
# catkin init
# catkin config --extend /opt/ros/noetic
# catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo

source /opt/ros/noetic/setup.bash
catkin build ocs2_mobile_manipulator_ros
source devel/setup.bash
```

主要节点：

- `mobile_manipulator_mpc_node`
- `mobile_manipulator_dummy_mrt_node`
- `mobile_manipulator_target`

### 1.3 运行 Demo

每个新终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/gzz/Codes/wbc/ocs2_ws/devel/setup.bash
```

推荐（轮式底盘 + 6 轴臂，最接近移动臂场景）：

```bash
roslaunch ocs2_mobile_manipulator_ros manipulator_ridgeback_ur5.launch
```

操作：

1. RViz 中找到 interactive marker **Goal**
2. 拖动目标位姿
3. 右键 **Send target pose**
4. 观察底盘与臂是否协同运动

相关文件：

- URDF：`ocs2_ws/src/ocs2_robotic_assets/resources/mobile_manipulator/ridgeback_ur5/urdf/ridgeback_ur5.urdf`
- 任务：`ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/ridgeback_ur5/task.info`
  - `manipulatorModelType: 1`（Wheel-based / SE(2)）
  - `eeFrame: ur_arm_tool0`
  - 输入：底盘 `v, ω` + 臂关节速度

对比（固定基臂）：

```bash
roslaunch ocs2_mobile_manipulator_ros manipulator_franka.launch
```

其他：

```bash
roslaunch ocs2_mobile_manipulator_ros manipulator_mabi_mobile.launch
roslaunch ocs2_mobile_manipulator_ros manipulator_kinova_j2n6.launch
roslaunch ocs2_mobile_manipulator_ros manipulator_pr2.launch
```

关闭 RViz：

```bash
roslaunch ocs2_mobile_manipulator_ros manipulator_ridgeback_ur5.launch rviz:=false
```

### 1.4 自碰 / 穿模

OCS2 使用 URDF `<collision>` + `task.info` 中的 `selfCollision`：

| 字段 | 含义 |
|------|------|
| `activate` | 是否启用自碰 |
| `collisionLinkPairs` | 要检查的 link 对 |
| `minimumDistance` | 最小安全距离（米） |
| `mu` / `delta` | soft constraint 参数 |

**Ridgeback+UR5 默认 `selfCollision.activate: false`**，因此可能出现臂与底盘穿模。开启时建议：

1. 将 `activate` 改为 `true`
2. 配置底盘–臂关键 `collisionLinkPairs`
3. 复杂 mesh 建议换成 box/cylinder（官方 TODO 也提到这一点）

这是自碰，不是环境障碍；环境避障需另行扩展。

### 1.5 测试结果（本机）

| 能力 | Ridgeback+UR5 | 实测 |
|------|---------------|------|
| 末端 6D 跟踪 | 是 | interactive marker Goal → Send target pose 后可跟踪 |
| 底盘+臂全身协同 | 是 | 轮式底盘与 UR5 同步运动 |
| 实时 MPC | 约 100 Hz | demo 可稳定运行 |
| 自碰 | 默认关闭 | `activate: false` 时可见臂–底盘穿模 |
| 环境障碍 | 非开箱 | 未在本仓库做障碍扩展验证 |
| 真机 Ranger+CR10 | 需移植与桥接 | 未接入 |

说明：RViz 可能提示 `Interactive marker 'Goal' contains unnormalized quaternions`，来自 marker 控件四元数未归一化，一般可忽略。

---

## 2. Holistic MM

官网：https://jhavl.github.io/holistic/

基于 Robotics Toolbox for Python 的 **QP 反应式**全身控制器（底盘+臂一体），适合给定末端位姿的快速伺服。不做复杂全局规划，环境绕障能力弱于 OCS2/REMANI。

### 2.1 conda 环境

已创建环境名：**`holistic`**（Python 3.11）

```bash
conda activate holistic

# 若需重装：
pip install "roboticstoolbox-python[swift,qp]"
# Swift 与 NumPy 2.x 不兼容，需固定 1.x
pip install "numpy<2"
```

当前实测版本示例：

- `roboticstoolbox-python` 1.3.1
- `numpy` 1.26.4
- `swift-sim` + `qpsolvers` / `quadprog`

### 2.2 运行（可选目标位姿）

Holistic 跟踪任意给定的末端目标 `Tep`（`SE3`）。脚本已支持预设 / 自定义：

```bash
conda activate holistic
cd /home/gzz/Codes/wbc

# 默认目标（官网类似：身后远处）
python scripts/holistic_mm_non_holonomic_rtb13.py

# 预设：front / left / right / up / near / default
python scripts/holistic_mm_non_holonomic_rtb13.py --preset front
python scripts/holistic_mm_non_holonomic_rtb13.py --preset left --swift

# 自定义世界系位置（米）+ 姿态 RPY（度）
python scripts/holistic_mm_non_holonomic_rtb13.py --xyz 1.0 -0.5 0.6 --rpy 180 0 0
python scripts/holistic_mm_non_holonomic_rtb13.py --xyz 0.8 0.3 0.5 --rpy 180 0 90 --swift
```

适配脚本：[`scripts/holistic_mm_non_holonomic_rtb13.py`](scripts/holistic_mm_non_holonomic_rtb13.py)

上游示例也可参考：

- `robotics-toolbox-python/examples/holistic_mm_non_holonomic.py`
- `robotics-toolbox-python/examples/holistic_mm_omni.py`

### 2.3 RTB 1.3 API 注意

官网示例中的：

```python
r.joint_velocity_damper(ps, pi, r.n)  # 旧写法，在 RTB 1.3 会报错
```

需改为：

```python
r.joint_velocity_damper(ps=0.1, pi=0.9, n=r.n)
```

另外 `qp.solve_qp(..., solver="quadprog")` 建议显式指定求解器。

### 2.4 测试结果（本机）

| 项目 | 结果 |
|------|------|
| 环境 | conda `holistic`，RTB 1.3.1 + NumPy 1.26.4 |
| Frankie 非全向回路 | 约 234 步到达目标 |
| 末端位置误差 | 约 4.16 → 约 0.05 |
| Swift | 可 `launch` 并短步进 |
| 预设目标 | `--preset front/left/right/up/near/default` 可用 |
| 环境全局绕障 | 弱（反应式 QP，非规划器） |

---

## 3. REMANI（Ranger + CR10 仿真）

工作空间：[`remani_planner/`](remani_planner/)（ROS1 Noetic catkin）。

当前阶段：**仿真优先**。保留虚拟差分底盘与原生 `car_cmd` / `joint_cmd` / `gripper_cmd`；真机桥接后置。臂安装位姿固定：

```yaml
base_mani_fixed_joint_xyz_ypr: [0.2462, 0.0, 0.1, 0.0, 0.0, 0.0]
```

### 3.1 编译与运行

```bash
cd /home/gzz/Codes/wbc/remani_planner
source /opt/ros/noetic/setup.bash
export CPATH=/usr/include/opencv4:/usr/include/pcl-1.10:$CPATH
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# 统一入口（默认 robot_model:=ranger_cr10）
roslaunch remani_planner remani_sim.launch
```

CR10 FK 冒烟：

```bash
roslaunch remani_planner test_cr10_fk.launch
```

RViz 中用 **2D Nav Goal** 触发规划。相关配置：

- [`mm_param_ranger_cr10.yaml`](remani_planner/src/REMANI-Planner/remani_planner/plan_manage/config/mm_param_ranger_cr10.yaml)
- [`exp_ranger_cr10_param.yaml`](remani_planner/src/REMANI-Planner/remani_planner/plan_manage/config/exp_ranger_cr10_param.yaml)
- [`remani_planner_param.yaml`](remani_planner/src/REMANI-Planner/remani_planner/plan_manage/config/remani_planner_param.yaml)

### 3.2 已做适配摘要

- CR10 运动学 / 碰撞采样 / mesh 可视化；自碰厚度与腕部邻接规则收紧
- Ranger 底盘几何 + 虚拟差分参数；地图过桥与走廊清空
- Hybrid A* 失败后：在 `try_astar_times` 内拒绝直线回退；累计失败后切 whole-body RRT
- 前端失败时递增 `continous_failures_count`（否则会永远停在 `failures=0/5`）
- 轨迹安全校验：启动段软 margin，随后加严；提高臂–障碍权重与采样密度

### 3.3 测试结果（本机）

| 项目 | 结果 |
|------|------|
| CR10 FK / 梯度 | 通过（位置误差 ~0，旋转误差 ~3e-6°） |
| 仿真闭环 | `fake_mm` + `mm_controller` + 规划器可启动并执行轨迹 |
| 短/中程多点导航 | 多次优化 `Success`，FSM `GEN_NEW_TRAJ → EXEC_TRAJ → STAY` |
| 难路径（Hybrid A* 超时） | 累计失败后切 RRT，仍可出轨迹并执行 |
| 臂–障碍穿模 | **仍有**：远点/第三次给点等场景可见机械臂穿障碍（见下方日志） |
| `rospack too many positional options` | 偶发，多为 RobotModel/URDF 路径解析噪声，通常非致命 |

一次连续三连点实测摘录（原始日志：[`remani_planner/src/REMANI-Planner/bug.sh`](remani_planner/src/REMANI-Planner/bug.sh)）：

| 次序 | start → goal（约） | 前端 | 优化 | 执行 |
|------|-------------------|------|------|------|
| 1 | (-5.00, 0.00) → (5.59, -1.65) | 正常 | Success ~1.73 s | 完成 |
| 2 | (5.59, -1.65) → (2.63, 4.52) | 正常 | Success ~2.36 s | 完成 |
| 3 | (2.63, 4.52) → (-3.88, 4.88) | Hybrid A* NO_PATH → 5 次后 RRT | Success ~0.81 s | 完成；**仍观测到臂–障碍穿模** |

---

## 4. 后续方向（Ranger + CR10）

无论选 OCS2、Holistic 还是 REMANI，真机接入都需要：

1. 整机 URDF / TCP 定义（建议先定 `gripper_base_link` 或 `cr10_Link6`）
2. 观测桥：`odom` + `joint_state`（`joint1..6` ↔ `cr10_joint1..6`）
3. 执行桥：底盘速度 → `/cmd_vel`；臂速度/轨迹 → CR10
4. 自碰：配置 collision 几何与检查对
5. 环境避障：按所选框架扩展（OCS2 约束 / REMANI ESDF / 感知距离场）

近期优先（REMANI 仿真）：

- 继续压低臂–障碍穿模（硬校验门控、采样厚度、前端路径质量）
- 稳定难路径下 Hybrid A* / RRT 切换与失败计数
- 再考虑 `remani_agx_bridge` 真机接入

## 参考链接

- OCS2：https://github.com/leggedrobotics/ocs2
- OCS2 安装：https://leggedrobotics.github.io/ocs2/installation.html
- OCS2 Mobile Manipulator：https://leggedrobotics.github.io/ocs2/robotic_examples.html
- Holistic MM：https://jhavl.github.io/holistic/
- Robotics Toolbox for Python：https://github.com/petercorke/robotics-toolbox-python
- REMANI-Planner：https://github.com/SYSU-STAR/REMANI-Planner
