# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 1 (catkin) workspace for a **Loco-Manipulation robot** platform consisting of:
- **Ranger** mobile base (AgileX, Ackermann steering, CAN bus)
- **Dobot CR10** 6-axis arm (TCP/IP, port 29999 for commands, port 30004 for realtime feedback)
- **DH-Robotics AG95** gripper (Modbus RTU over serial, 1 DOF)
- RoboSense 16-line LiDAR + Intel RealSense depth camera
- SLAM (GMapping) + Navigation (move_base/AMCL) + RViz multi-goal plugin

## Build Commands

```bash
# Full workspace build (from workspace root)
cd ~/catkin_ws && catkin_make

# Build specific package only
catkin_make --pkg <package_name>

# Source the workspace after building
source devel/setup.bash
```

**Build dependency order**: `bt_task_msgs` must be built before `lifting_ctrl` (the latter imports message definitions from the former).

## Hardware Bring-up Sequence

```bash
# 1. CAN bus (once per boot)
rosrun ranger_bringup bringup_can2usb.bash

# 2. Ranger base
roslaunch ranger_bringup ranger.launch

# 3. Dobot CR10 arm
export DOBOT_TYPE=cr10
roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.5.1

# 4. AG95 gripper
roslaunch dh_gripper_driver dh_gripper.launch GripperModel:=AG95_MB Connectport:=/dev/ttyUSB0

# 5. MoveIt (for arm planning)
export DOBOT_TYPE=cr10
roslaunch dobot_moveit moveit.launch
```

The `DOBOT_TYPE` env var selects the arm model at runtime and must be set before launching any Dobot-related node.

## Architecture

### Package Map (src/)

| Package | Language | Role |
|---------|----------|------|
| `ranger_ros/ranger_base` | C++ | Ranger base driver node (`ranger_base_node`), publishes odom/state, subscribes `/cmd_vel` |
| `ranger_ros/ranger_msgs` | msg | Custom messages: SystemState, MotionState, ActuatorStateArray |
| `ranger_ros/ranger_bringup` | launch | Navigation stack (DWA+AMCL), gmapping, lidar launch, CAN scripts |
| `ugv_sdk` | C++ lib | Low-level CAN protocol for all AgileX UGVs (no ROS nodes, pure library) |
| `TCP-IP-ROS-6AXis/dobot_v4_bringup` | C++ | V4 arm driver: TCP socket bridge, 90+ ROS services, FollowJointTrajectoryAction |
| `TCP-IP-ROS-6AXis/dobot_moveit` | launch | MoveIt dispatcher: routes to `$(DOBOT_TYPE)_moveit` package |
| `TCP-IP-ROS-6AXis/dobot_description` | urdf | URDF models for 9 arm variants |
| `gripper/dh_gripper_driver` | C++ | AG95/RGI/DH3 gripper driver, 50Hz control loop, Modbus/CAN protocols |
| `gripper/dh_gripper_msgs` | msg | GripperCtrl, GripperState, GripperRotCtrl, GripperRotState |
| `lifting_ctrl` | Python | Electric lifting column control via RS-485 Modbus RTU |
| `bt_task_msgs` | msg/srv | LiftMotorMsg, LiftMotorSrv, LiftInterfaceSrv (shared by lifting_ctrl) |
| `rslidar_sdk` | C++ | RoboSense LiDAR driver, publishes `/rslidar_points` (PointCloud2) |
| `realsense-ros` | C++ | Intel RealSense camera wrapper (depth/IR/color/IMU) |
| `SLAM/slam_gmapping` | C++ | GMapping 2D SLAM (particle filter + grid map) |
| `SLAM/rf2o_laser_odometry` | C++ | Range-flow laser odometry (lightweight, 0.9ms/frame) |
| `SLAM/laser_scan_matcher` | C++ | ICP-based scan-to-scan matching |
| `plugin/...` | C++/Qt | RViz panel for multi-goal sequential navigation with cycling |

### Key Communication Flow

```
Ranger base <--CAN--> ugv_sdk <--catkin_link--> ranger_base_node --> /cmd_vel, /odom, /tf
Dobot CR10 <--TCP/IP--> dobot_v4_bringup --> /joint_states, FollowJointTrajectoryAction
AG95 gripper <--Modbus/Serial--> dh_gripper_driver --> /gripper/states, /gripper/joint_states
RoboSense LiDAR <--UDP--> rslidar_sdk --> /rslidar_points
RealSense <--USB--> realsense2_camera --> /camera/depth/*, /camera/color/*
```

### Ranger Motion Modes (auto-selected by ranger_base_node from /cmd_vel)

- **DUAL_ACKERMAN**: default, standard steering
- **PARALLEL**: `linear.y != 0` (crab walk)
- **SPINNING**: turn radius < min_turn_radius
- **SIDE_SLIP**: `linear.y != 0` and `linear.x == 0` on Ranger Mini V1 only

### Dobot Arm Architecture

`dobot_v4_bringup` opens two TCP connections to the controller:
- Port 29999: dashboard commands (request/response, single client)
- Port 30004: 1440-byte binary RealTimeData struct at 8ms (joints, TCP pose, torques, temperatures)

All Dobot commands are exposed as ROS services under `/dobot_v4_bringup/srv/`. The `FollowJointTrajectoryAction` server enables MoveIt integration using `servoj()` for real-time trajectory following.

### AG95 Gripper Details

- 1 DOF (grip only, no rotation)
- Joint name: `gripper_finger1_joint` (range: 0 ~ 0.637 rad, mapped from raw 0-1000)
- Default launch args: `GripperModel:=AG95_MB`, `Connectport:=/dev/DH_hand`, `Baudrate:=115200`
- Control via `/gripper/ctrl` topic (GripperCtrl msg: position, force, speed, initialize)
- URDF package `dh_robotics_ag95_description` is NOT in this workspace (external dependency)

## Key Topics

| Topic | Type | Source |
|-------|------|--------|
| `/cmd_vel` | geometry_msgs/Twist | Input to ranger_base_node |
| `/odom` | nav_msgs/Odometry | ranger_base_node (dead reckoning) |
| `/joint_states` | sensor_msgs/JointState | dobot_v4_bringup (arm) + dh_gripper_driver (gripper) |
| `/gripper/ctrl` | GripperCtrl | Input to gripper driver |
| `/gripper/states` | GripperState | Gripper feedback at 50Hz |
| `/rslidar_points` | PointCloud2 | rslidar_sdk |
| `/map` | OccupancyGrid | slam_gmapping output |
| `/move_base_simple/goal` | PoseStamped | Navigation goal (used by plugin) |

## Notes

- The workspace targets **ROS 1 (Kinetic/Melodic/Noetic)** on Linux. The serial/POSIX code in gripper and lifting_ctrl is Linux-only.
- `rslidar_sdk` config is in `src/rslidar_sdk/config/config.yaml` -- must match the actual lidar model (currently set to RSHELIOS_16P).
- The `src/README.md` contains a comprehensive subproject guide (written in Chinese).
- `librealsense-2.50.0.zip` in src/ is the LibRealSense SDK source needed by `realsense-ros`.
- The `ranger_ros/sensor_msgs` package is a bundled local copy of the standard sensor_msgs (for version pinning).
