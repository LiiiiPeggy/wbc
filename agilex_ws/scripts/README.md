# 实机脚本（Robot host）— `agilex_ws/scripts`

标准 catkin 布局：

```text
agilex_ws/
  scripts/     ← 本目录
  src/         ← 驱动与传感器包
  build/
  devel/
```

笔记本侧在 `remani_planner/scripts/`（不要拷到实机）。
仓库内路径即 `wbc/agilex_ws/`；拷到车上仍是 `~/agilex_ws/`。

## 底盘 / 机械臂

| 脚本 | 作用 |
|------|------|
| `build_agx_drivers.sh` | 编译 `ugv_sdk;ranger_msgs;ranger_base;dobot_v4_bringup` |
| `bringup_can.sh` | 拉起 `can0` @ 500 kbit/s |
| `run_real_dry_run.sh` | 仅启动 Ranger+CR10 驱动（REMANI remap） |
| `watch_ranger_cmd.sh` | 只读 `/remani/hardware/ranger/cmd_vel` |
| `watch_cr10_status.sh` | 只读驱动侧关节 |

## 激光雷达 / 相机

| 脚本 | 作用 |
|------|------|
| `build_agx_sensors.sh` | 编译 `rslidar_sdk;realsense2_camera`（需本机 `librealsense2`） |
| `run_lidar.sh` | 启动 RoboSense → `/rslidar_points`（默认不启本机 RViz；`WITH_RVIZ=1` 可开） |
| `run_camera.sh` | 启动 RealSense D435 → `/camera/...` |
| `watch_lidar.sh` | `rostopic hz /rslidar_points` |
| `watch_camera.sh` | `rostopic hz` 彩色/深度图 |

> `build_agx_drivers.sh` 与 `build_agx_sensors.sh` 使用不同白名单。切换编译目标时若包找不到，先 `rm -rf build devel` 再编。

## 实机一次会话

```bash
cd ~/agilex_ws   # 或仓库内 wbc/agilex_ws
./scripts/build_agx_drivers.sh
./scripts/build_agx_sensors.sh          # 需要雷达/相机时

./scripts/bringup_can.sh
export ROS_IP=<robot_ip>
export ROS_MASTER_URI=http://<master>:11311

./scripts/run_real_dry_run.sh
./scripts/run_lidar.sh
./scripts/run_camera.sh
```

然后到**笔记本**跑 `remani_planner/scripts/run_real_dry_run_control_plane.sh`
（同一 `ROS_MASTER_URI`）。

注意：本目录不启动 `remani_real_node` / planner / RViz；也不发运动指令。
启动后前几帧 `/odom` 须贴近世界原点（≤1 mm、≤0.1°），否则笔记本报 `START_ODOM_NOT_ZERO`。
当前 REMANI V1 `static_empty` 不用雷达/相机做避障；传感器脚本便于现场标定与后续感知接入。
