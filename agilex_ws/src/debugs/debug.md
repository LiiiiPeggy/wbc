# 调试记录与解决方案

本文档整理了设备调试过程中遇到的问题及其解决方案。

---

## 1. 相机话题冲突（bug_camera.md）

### 问题

系统中有两个 RealSense 相机（一个装在雷达上方，一个装在机械臂夹爪上）。同时启动时，两者发布相同的话题名（如 `/camera/color/image_raw`），导致只能显示一个相机的图像。

### 解决方案

通过 ROS launch 参数为新相机指定不同的命名空间（namespace）或话题前缀。保留原相机话题名称不变，修改新相机的发布话题。

**原相机（雷达上方，保持默认）：**
```bash
roslaunch realsense2_camera rs_camera.launch
```

**新相机（夹爪上，加 `camera_name` 参数）：**
```bash
roslaunch realsense2_camera rs_camera.launch camera_name:=gripper_camera
```

这样新相机的话题会变为：
- `/gripper_camera/color/image_raw`
- `/gripper_camera/depth/image_rect_raw`
- `/gripper_camera/infra1/image_rect_raw`
- 等等

如果需要同时启动两个相机，也可以写一个统一的 launch 文件：

```xml
<!-- dual_camera.launch -->
<launch>
  <!-- 雷达上方相机（默认话题名） -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="camera" value="camera"/>
    <arg name="serial_no" value="<原相机序列号>"/>
  </include>

  <!-- 夹爪相机（独立话题名） -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="camera" value="gripper_camera"/>
    <arg name="serial_no" value="<新相机序列号>"/>
  </include>
</launch>
```

**查看相机序列号：**
```bash
rs-enumerate-devices | grep "Serial Number"
```

---

## 2. CR10 机械臂 MoveIt 执行失败（bug_cr10.md 问题1）

### 问题

启动 `dobot_moveit moveit.launch` 后，MoveIt 能成功规划路径，但执行时报错：

```
[ERROR] Unable to identify any set of controllers that can actuate the specified joints: [ joint1 joint2 joint3 joint4 joint5 joint6 ]
[ERROR] Known controllers and their joints:
[ERROR] Action client not connected: cr10_robot/joint_controller/follow_joint_trajectory
ABORTED: Solution found but controller failed during execution
```

### 原因

MoveIt 需要一个 `FollowJointTrajectory` action server 来执行轨迹，这个 action server 由 `dobot_v4_bringup` 提供。**必须先启动 `dobot_v4_bringup`，再启动 `moveit.launch`。**

### 解决方案

按正确顺序启动（两个终端）：

```bash
# 终端 1：先启动机械臂驱动（提供 action server）
export DOBOT_TYPE=cr10
roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.8.188

# 终端 2：再启动 MoveIt（消费 action server）
export DOBOT_TYPE=cr10
roslaunch dobot_moveit moveit.launch
```

**验证 action server 是否就绪：**
```bash
rostopic list | grep follow_joint_trajectory
# 应该看到:
# /cr10_robot/joint_controller/follow_joint_trajectory/cancel
# /cr10_robot/joint_controller/follow_joint_trajectory/feedback
# /cr10_robot/joint_controller/follow_joint_trajectory/goal
# /cr10_robot/joint_controller/follow_joint_trajectory/result
# /cr10_robot/joint_controller/follow_joint_trajectory/status
```

---

## 3. CR10 机械臂基本操作（bug_cr10.md 问题2）

### 问题

启动 `dobot_v4_bringup` 后，如何操作机械臂？

### 解决方案

启动 `dobot_v4_bringup` 后，机械臂进入待命状态。可以通过以下方式操作：

**方式一：ROS Service 命令行**

```bash
# 使能机械臂（必须先执行）
rosservice call /dobot_v4_bringup/srv/EnableRobot

# 关节运动（移动到指定关节角度，单位：度）
rosservice call /dobot_v4_bringup/srv/JointMovJ "{j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0}"

# 笛卡尔运动（移动到指定位姿）
rosservice call /dobot_v4_bringup/srv/MovJ "{x: 200, y: 0, z: 200, rx: 180, ry: 0, rz: 0}"

# 直线运动
rosservice call /dobot_v4_bringup/srv/MovL "{x: 200, y: 0, z: 300, rx: 180, ry: 0, rz: 0}"

# 急停
rosservice call /dobot_v4_bringup/srv/EmergencyStop

# 清除错误
rosservice call /dobot_v4_bringup/srv/ClearError

# 下使能
rosservice call /dobot_v4_bringup/srv/DisableRobot
```

**方式二：MoveIt（需同时启动 moveit.launch）**

启动 MoveIt 后，在 RViz 中拖拽机械臂末端目标，点击 "Plan and Execute"。

**方式三：Python 脚本（通过 FollowJointTrajectoryAction）**

```python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

client = actionlib.SimpleActionClient(
    '/cr10_robot/joint_controller/follow_joint_trajectory',
    FollowJointTrajectoryAction
)
client.wait_for_server()

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

point = JointTrajectoryPoint()
point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 弧度
point.time_from_start = rospy.Duration(2.0)
goal.trajectory.points = [point]

client.send_goal(goal)
client.wait_for_result()
```

---

## 4. Ranger 底盘移动操作（bug_ranger.md 问题1）

### 问题

启动 `ranger.launch` 后，如何控制底盘移动？

```bash
# 先使能 CAN 口（每次开机执行一次）
rosrun ranger_bringup bringup_can2usb.bash

# 启动底盘驱动
roslaunch ranger_bringup ranger.launch
```

### 解决方案

底盘节点订阅 `/cmd_vel` 话题（`geometry_msgs/Twist` 类型）。发送速度命令即可控制移动。

**方式一：命令行直接发送（可用）**

```bash
# 前进（线速度 0.3 m/s）
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# 左转（角速度 0.5 rad/s）
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"

# 停止
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

**方式二：键盘遥控（可行）**
**遥控器第二根杆需要在最上：指令控制模式**

```bash
# 安装（如果未安装）
sudo apt install ros-melodic-teleop-twist-keyboard

# 启动
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```

使用按键 `i` 前进、`,` 后退、`j` 左转、`l` 右转、`k` 停止。

**方式三：手柄控制**

```bash
sudo apt install ros-melodic-joy
rosrun joy joy_node
# 需要编写或使用现成的 joy -> cmd_vel 节点
```

---

## 5. 导航操作流程（bug_ranger.md 问题2）

### 问题

启动导航栈后，如何进行导航？是手动给点还是自动导航？

### 解决方案

导航是通过 RViz 设置目标点，`move_base` 自动规划路径并控制底盘到达目标。

**完整导航流程：**

```bash
# 终端 1：启动雷达
roslaunch rslidar_sdk start.launch

# 终端 2：启动导航栈（含地图、AMCL、move_base、RViz）
roslaunch ranger_bringup navigation_4wd.launch
```

**在 RViz 中操作：**

1. 确认激光雷达点云和地图已正确显示
2. 点击工具栏的 **"2D Nav Goal"** 按钮
3. 在地图上点击目标位置，拖拽设置朝向
4. `move_base` 会自动规划路径，底盘开始移动
5. 到达目标后自动停止

**多目标导航（使用 RViz 插件）：**

如果需要设置多个目标点顺序巡航：
1. 在 RViz 中加载插件：`Panels -> Add New Panel -> MultiNaviGoalsPanel`
2. 重配置 2D Nav Goal 工具：`Tool Property` 中将 topic 改为 `/move_base_simple/goal_temp`
3. 在地图上依次点击设置多个目标点
4. 点击 "Start navigation" 开始顺序巡航
5. 勾选 "Cycle" 可循环巡航

**调试导航问题：**

```bash
# 查看 move_base 状态
rostopic echo /move_base/status

# 查看当前位姿（AMCL 定位结果）
rostopic echo /amcl_pose

# 重置代价地图（如果规划异常）
rosservice call /move_base/clear_costmaps
```

---

## 6. IMU 相关（bug_ranger.md 问题3）

### 问题

没有找到 IMU 启动相关的内容。

### 解决方案

当前 Ranger 底盘驱动 `ranger_base_node` **不包含独立的 IMU 节点**。Ranger 全尺寸底盘的 CAN 协议中包含 IMU 数据，但当前 `ranger_ros` 驱动未将其作为独立话题发布。

**当前状态：**
- `ranger_base_node` 通过 CAN 总线读取底盘状态，发布 `/odom`（航迹推算里程计）
- 里程计数据来自轮式编码器，不包含 IMU 融合
- 如果需要 IMU 数据用于导航或 SLAM，有以下选项：

**选项一：外接独立 IMU 模块**

```bash
# 常见 IMU 模块（如 MPU9250、BNO055）通过 USB-UART 连接
# 使用现有的 imu_driver 包，例如：
roslaunch mpu9250_serial mpu9250_serial.launch
```

外接 IMU 后发布 `/imu/data` 话题，可在 AMCL 或 EKF 中融合使用。

**选项二：使用 robot_localization 包融合 odom + IMU**

```bash
sudo apt install ros-melodic-robot-localization
```

配置 `ekf` 节点融合轮式里程计和 IMU，输出更精确的定位。

**选项三：确认底盘是否内置 IMU**

部分 Ranger 底盘硬件内置 IMU，但需要检查 CAN 协议文档确认。如果有内置 IMU 且需要使用，需要在 `ugv_sdk` 层面解析对应的 CAN 帧并发布 ROS 话题。

---

## 附录：包名命名规范警告（已处理）

历史版本中包名 `RangerCR10LiDAR_description`、URDF 机器人名 `RangerCR10LiDAR` 含大写字母，会触发 rospack 命名规范警告。

### 当前命名（已统一为小写）

| 项 | 新名称 |
|----|--------|
| 目录 / ROS 包 | `rangerboxcr10lidar_description` |
| 整机 URDF | `urdf/rangercr10lidar.urdf` |
| `<robot name="...">` | `rangercr10lidar` |
| 关节配置 | `config/joint_names_rangerboxcr10lidar_description.yaml` |

重新 `catkin_make` 并 `source devel/setup.bash` 后，上述警告应不再出现。
