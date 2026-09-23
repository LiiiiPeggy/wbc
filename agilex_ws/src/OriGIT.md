# OriGIT 项目适配修改整理

本文整理 `origit/` 中原始 Git 仓库与同级 `../src/` 当前工程之间的主要差异，重点说明为了适配本套移动操作设备所做的针对性修改。

当前设备栈按 `../src/README.md` 和 `../src/CLAUDE.md` 归纳为：

- 移动底盘：AgileX Ranger / Ranger 系列底盘，CAN 总线通信。
- 机械臂：Dobot CR10，TCP/IP 通信，常用控制器 IP 为 `192.168.5.1`。
- 夹爪：DH-Robotics AG95，Modbus RTU 串口通信。
- 感知：RoboSense RS-Helios-16P 激光雷达，Intel RealSense D435 系列深度相机。
- 建图导航：RoboSense 点云转 2D LaserScan，RF2O / GMapping / AMCL / move_base，RViz 多目标点巡航插件。

注意：`origit/` 中除已恢复源码的 `ugv_sdk` 外，多个仓库当前工作树为空，但 `.git` 仍保留了原始版本信息。本文以 Git HEAD 导出的原始文件和 `../src/` 的当前工程做对照；本次复核时，`ugv_sdk` 排除 `.git` 后两侧均为 89 个文件，文件哈希完全一致。`ugv_sdk` 的 `.git` 目录仍不能作为正常 Git 工作树打开，因此只用其源码文件参与对照。

GitHub 链接优先取自 `origit/*/.git/config`、`../src/README.md`、`.gitmodules` 和各包 `package.xml` 中能确认的上游记录；未在本地内容中记录来源的新增设备包，不额外猜测上游仓库。

## 总览

| 项目 | 当前工程位置 | 适配结论 |
| --- | --- | --- |
| `ranger_ros` | `../src/ranger_ros` | 改动最大：新增建图、导航、雷达、地图、RViz、DWA/AMCL 参数，形成 Ranger 底盘的完整导航入口。 |
| `ugv_sdk` | `../src/ugv_sdk` | 源码、配置、脚本和文档均未发现相对 `origit/ugv_sdk` 的改动；作为 Ranger 底层 CAN SDK 原样使用。 |
| `TCP-IP-ROS-6AXis` | `../src/TCP-IP-ROS-6AXis` | 与原始仓库基本一致；设备适配主要通过运行时选择 `DOBOT_TYPE=cr10` 和 `robotIp:=192.168.5.1` 完成。 |
| `rslidar_sdk` | `../src/rslidar_sdk` | 修改雷达型号和近距离过滤；同时把原本的 `rs_driver` 子模块实体化，方便工作空间直接编译。 |
| `realsense-ros` | `../src/realsense-ros` | 只补了 OpenCV 的 CMake 查找、头文件和库链接；未看到针对具体相机型号的 launch 参数改动。 |
| `rf2o_laser_odometry` | `../src/SLAM/localizer/rf2o_laser_odometry` | 改成真实 `/scan` 输入和 `/odom` 输出，启用 TF 发布，移除仿真 ground truth 初始化，并增加无效距离值保护。 |
| `rviz_navi_multi_goals_pub_plugin` | `../src/plugin/rviz_navi_multi_goals_pub_plugin` | 在多目标导航面板上加入建图/地图管理/导航启动流程，可一键调起雷达、GMapping、map_server、RF2O、move_base。 |
| `SLAM` | `../src/SLAM` | `origit/SLAM` 实际指向 RViz 插件仓库，不是有效 SLAM 基线；当前 `SLAM` 是算法包集合，配合 Ranger 雷达导航栈使用。 |

## 1. ranger_ros

GitHub：本地 `origit/ranger_ros/.git/config` 指向 <https://github.com/lagrangeluo/ranger_ros.git>；当前 `../src/README.md` 记录的来源为 <https://github.com/agilexrobotics/ranger_ros.git>。

原始项目只提供 Ranger 底盘 ROS 驱动、消息和基础 bringup。当前工程在 `ranger_bringup` 下补齐了实车建图和导航所需的完整配置。

主要适配点：

- 新增 `launch/open_lidar.launch`，把 `rslidar_sdk`、`pointcloud_to_laserscan` 和 `rf2o_laser_odometry` 串起来，将 RoboSense 3D 点云转换为 2D `/scan`，再输出 `/odom`。
- 在 `open_lidar.launch` 中增加静态 TF：`rslidar -> dummy_base_link`，位移为 `-0.54 0 0.059972`，用于对齐雷达与底盘/建图坐标。
- 新增 `launch/gmapping.launch`，设置 `odom_frame: odom`、`base_frame: dummy_base_link`、`map_frame: map`，并把 GMapping 参数调成更适合室内建图的值，例如 `maxUrange: 10.0`、`particles: 100`、`linearUpdate: 0.3`、`delta: 0.05`。
- 新增 `launch/navigation_4wd.launch`，集成 `map_server`、`amcl`、`move_base` 和 RViz，默认加载 `maps/map11.yaml`。
- 新增 `launch/volecity_smoother.launch` 和 `param/velocity_smoother.yaml`，限制线速度/角速度为 `0.5`，加速度较低，用于减小实车速度突变。
- 新增 `param/4wd/*` 导航参数，使用 DWA 局部规划器；底盘 footprint 设置为约 `1.10m x 0.90m`，即 `[[-0.55,-0.45], [-0.55,0.45], [0.55,0.45], [0.55,-0.45]]`。
- AMCL 参数改为 `base_frame_id: rslidar`、`global_frame_id: world`，并使用较多粒子数：`in_particles: 300`、`max_particles: 1500`。
- 增加多张实测地图及 RViz 配置：`maps/*.pgm`、`maps/*.yaml`、`rviz/mapping.rviz`、`rviz/navigation.rviz`、`rviz/slam.rviz`。
- README 中补充了实车常见话题列表，例如 `/cmd_vel`、`/odom`、`/system_state`、`/battery_state`、`/tf`。

需要留意：

- `navigation_4wd.launch` 末尾包含 `ranger_mini_v2.launch`，而总 README 中写的是当前底盘为 Ranger 全尺寸。这里可能是历史调试配置，真机部署前建议确认实际底盘型号对应的 launch。
- 若后续统一包名，应把历史文档中的 `agilexpro`、`scout_bringup` 说法同步到当前 `ranger_bringup`。

## 2. ugv_sdk

GitHub：本地 `origit/ugv_sdk/.git/config` 指向 <https://github.com/agilexrobotics/ugv_sdk.git>；当前 `../src/README.md` 和 `.github/workflows` 记录/拉取的是 <https://github.com/westonrobot/ugv_sdk.git>。

`ugv_sdk` 是 Ranger 底盘通信的底层 C++ SDK。用户恢复相关文件后，已重新逐文件核对 `origit/ugv_sdk` 与 `../src/ugv_sdk`：排除 `.git` 后两边均为 89 个文件，SHA256 哈希差异为 0。

主要情况：

- `ranger_ros` 依赖它完成 CAN 通信，实际设备适配主要发生在 `ranger_ros` 的 launch 和导航参数中。
- 当前 `../src/ugv_sdk` 没有相对 `origit/ugv_sdk` 的源码、配置、脚本或文档改动。
- 代码层仍保留上游对 Ranger、Scout、Hunter、Bunker、Tracer 等多种 UGV 的支持。
- `.github/workflows` 中保留的是上游 CI 配置，用于 GitHub Actions 自动构建检查，不参与本地 ROS 运行。

结论：本项目把 `ugv_sdk` 作为底层依赖原样使用，没有实车定制修改。

## 3. TCP-IP-ROS-6AXis

GitHub：<https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git>

与 `origit/TCP-IP-ROS-6AXis` 的原始内容相比，当前 `../src/TCP-IP-ROS-6AXis` 基本没有文件级差异。

设备适配方式：

- 使用 Dobot V4 bringup 控制真实 CR10：`roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.5.1`。
- 通过环境变量选择机械臂型号：`export DOBOT_TYPE=cr10`。
- 通过 `dobot_moveit` 根据 `DOBOT_TYPE` 分发到 `cr10_moveit` 等对应 MoveIt 包。
- 上游项目已包含 CR10 的描述、MoveIt 配置、V4 控制器服务和 FollowJointTrajectoryAction，因此本工作空间没有额外改源码。

结论：CR10 适配主要是运行参数和启动流程，不是仓库源码修改。

## 4. rslidar_sdk

GitHub：<https://github.com/RoboSense-LiDAR/rslidar_sdk.git>；其中 `rs_driver` 子模块/实体化驱动来源为 <https://github.com/RoboSense-LiDAR/rs_driver.git>。

当前工程对 RoboSense 激光雷达做了明确的型号适配。

主要改动：

- `config/config.yaml` 中将默认雷达型号从 `RSM1` 改为 `RSHELIOS_16P`。
- 最小有效距离从 `0.2` 调整为 `0.45`，用于过滤车体/近距离噪声点。
- 保持在线雷达模式 `msg_source: 1`，并发布 ROS 点云：`send_point_cloud_ros: true`。
- 坐标系和话题沿用导航链路：`ros_frame_id: rslidar`、`/rslidar_points`、`/rslidar_packets`。
- `src/rs_driver` 在当前工程中从 Git 子模块变成了实体源码目录，方便没有子模块初始化的 catkin 工作空间直接编译。

结论：这是针对本机 RS-Helios-16P 雷达和底盘安装环境的配置级适配。

## 5. realsense-ros

GitHub：<https://github.com/IntelRealSense/realsense-ros.git>

当前工程只对编译依赖做了轻量补充。

主要改动：

- 在 `realsense2_camera/CMakeLists.txt` 中增加 `find_package(OpenCV REQUIRED)`。
- 将 `${OpenCV_INCLUDE_DIRS}` 加入 include path。
- 将 `${OpenCV_LIBRARIES}` 链接到 `realsense2_camera` 目标。

结论：这是为了适配当前 ROS/OpenCV 编译环境的构建修改；未看到针对 D435/D435i 的 launch 参数或话题重映射改动。相机型号和滤波配置仍通过原始 `rs_camera.launch` 等入口运行时指定。

## 6. rf2o_laser_odometry

GitHub：<https://github.com/MAPIRlab/rf2o_laser_odometry.git>

当前 RF2O 放在 `../src/SLAM/localizer/rf2o_laser_odometry`，相对原始 `origit/rf2o_laser_odometry` 有少量但关键的实车改动。

主要改动：

- 输入话题从 `/laser_scan` 改为 `/scan`，接上 `pointcloud_to_laserscan` 的输出。
- 输出里程计从 `/odom_rf2o` 改为 `/odom`，直接供 GMapping、AMCL 和 move_base 使用。
- `publish_tf` 从 `false` 改为 `true`，由 RF2O 发布 odom TF。
- `base_frame_id` 去掉前导 `/`，从 `/base_link` 改为 `base_link`，符合 ROS TF2 推荐写法。
- `init_pose_from_topic` 从 `/base_pose_ground_truth` 改为空，移除仿真依赖，适配真机启动。
- 运行频率从 `6.0` 提高到 `10`。
- `CLaserOdometry2D.cpp` 在构建金字塔时加入 `std::isfinite(dcenter)` 判断，避免激光数据中的 NaN/Inf 进入计算。

结论：RF2O 被从仿真/示例配置改成了可以直接接真实 `/scan` 的里程计节点。

## 7. rviz_navi_multi_goals_pub_plugin

GitHub：<https://github.com/autolaborcenter/rviz_navi_multi_goals_pub_plugin.git>

当前插件不再只是原始的多目标点导航面板，而是加入了面向建图、地图保存、地图切换和导航启动的一体化控制。

主要改动：

- `CMakeLists.txt` 新增依赖 `roslib`、`rosnode`，并启用 `-std=c++11`。
- 新增 `getDirPath()`，通过 `ros::package::getPath(...)` 推导工作空间路径。
- Mapping 页加入地图列表和 `record` 按钮，用于启动/停止建图并保存地图。
- 增加 `clearMap()`、`mapTableSlot()` 等逻辑，点击地图后可启动 `map_server`、`rf2o_laser_odometry`、导航 launch，并切换到对应地图。
- `saveFileBtnSlot()` 会调用 `map_saver` 保存地图，并在保存后停止 `slam_gmapping` 和 `rf2o_laser_odometry_node`。
- 建图流程中会按需 `killall move_base`、`amcl`、`bunker_base_node`、`rf2o_laser_odometry_node` 等节点，避免建图和导航同时运行。
- 原始多目标点逻辑仍保留：订阅临时目标点、按队列发布到 `/move_base_simple/goal`、监听 `/move_base/status`、绘制 marker。

需要留意：

- 插件源码中仍有不少历史包名和路径，例如 `agilexpro`、`src/agilexpro/maps`、`/opt/ros/kinetic`、`bunker_base_node`。如果当前工程统一使用 `ranger_bringup`，这些路径需要同步修正，否则插件一键建图/导航功能可能找不到包。

## 8. SLAM

GitHub：`origit/SLAM` 的 Git remote 实际也指向 <https://github.com/autolaborcenter/rviz_navi_multi_goals_pub_plugin.git>，不能代表当前 `../src/SLAM` 集合。当前集合能从本地元数据确认或对应到的上游包括：

- `openslam_gmapping`：<https://github.com/ros-perception/openslam_gmapping>
- `slam_gmapping`：<https://github.com/ros-perception/slam_gmapping>
- `pointcloud_to_laserscan`：本地 `package.xml` 记录为 <https://github.com/ros-perception/perception_pcl>
- `laser_scan_matcher`：<https://github.com/ccny-ros-pkg/scan_tools>
- `rf2o_laser_odometry`：<https://github.com/MAPIRlab/rf2o_laser_odometry.git>

`origit/SLAM` 的 Git 信息实际指向 `rviz_navi_multi_goals_pub_plugin`，不能作为 `../src/SLAM` 的有效原始基线。当前 `../src/SLAM` 更像是把多个算法包集中到一起，服务于 Ranger + RoboSense 的 2D 建图导航。

当前集合内容：

- `openslam_gmapping` 和 `slam_gmapping`：提供 GMapping 2D SLAM。
- `pointcloud_to_laserscan`：把 RoboSense 3D 点云转换为 `/scan`。
- `laser_scan_matcher`：保留基于 CSM 的扫描匹配能力。
- `localizer/rf2o_laser_odometry`：提供真实 `/scan` 到 `/odom` 的激光里程计。

结论：SLAM 目录的适配核心不是单个源码补丁，而是把点云转激光、RF2O、GMapping 和导航参数接成一条适合本车雷达安装方式的链路。

## 9. gripper

GitHub：本地内容未记录可确认的 GitHub 上游；该包不在 `origit/` 原始项目列表中，按当前工程新增设备包记录。

`gripper` 不在 `origit/` 原始项目列表中，是当前工作空间额外引入的设备驱动包，用于 DH-Robotics AG95。

主要适配点：

- 默认夹爪型号为 `AG95_MB`，即 AG95 的 Modbus RTU 模式。
- 默认启动端口在 `dh_gripper.launch` 中为 `/dev/DH_hand`，README 示例中也可用 `/dev/ttyUSB0`，波特率 `115200`。
- 订阅 `/gripper/ctrl`，发布 `/gripper/states` 和 `/gripper/joint_states`。
- `dh_gripper_driver.cpp` 将原始 0-1000 位置映射到 `gripper_finger1_joint`，角度范围约 `0 ~ 0.637 rad`，方便与 URDF/MoveIt 的关节状态合流。
- 保留测试节点 `dh_gripper_driver_test`，可通过 `test_run:=true` 自动开合验证。

结论：这是针对 AG95 实物夹爪串口控制和 ROS 关节状态发布的新增适配包。

## 10. lifting_ctrl 与 bt_task_msgs

GitHub：本地内容未记录可确认的 GitHub 上游；两者不在 `origit/` 原始项目列表中，按当前工程新增控制/消息包记录。

这两个包也不在 `origit/` 原始项目列表中，是为升降柱/线性执行器新增的控制与消息定义。

`bt_task_msgs`：

- 定义 `LiftMotorMsg`，包含电机 ID、初始化状态、控制模式、电压、电流、速度、位置、高度、上下限位、错误状态、目标位置等字段。
- 定义 `LiftMotorSrv`：请求包含 `val` 和 `mode`，响应为 `state`。
- 定义 `LiftInterfaceSrv`：在 `val`、`mode` 基础上增加 `id`，响应 `message/success/code`，便于上层按电机 ID 路由。

`lifting_ctrl`：

- 支持 `830ABS` 绝对值编码器和 `850pro` 增量式编码器两类电机节点。
- 使用 RS-485 / Modbus RTU，配置文件位于 `scripts/config/`。
- `2_lifting_motor_config.json` 和 `4_lifting_motor_config.json` 适配 `/dev/lifter_2`、`/dev/lifter_4`，波特率 `38400`，行程 `0 ~ 230 mm`，最大转速 `500 rpm`，堵转电流阈值 `5 A`。
- `1_lifting_motor_config.json` 配置了另一套行程方向：`upLimitVal: 0`、`downLimitVal: -610`、最大转速 `1500 rpm`、堵转电流阈值 `16 A`。
- 通过 `start_motor.launch` 和 `start_850pro_motor.launch` 暴露 `motor_id_` 与命名空间参数。

结论：这部分是为移动操作平台上的升降机构新增的完整 ROS 控制接口。

## 11. rangerboxcr10lidar_description

GitHub：本地内容未记录可确认的 GitHub 上游；该包不在 `origit/` 原始项目列表中，按当前工程新增整机描述包记录。

该描述包不在 `origit/` 中，是当前工作空间新增的整机 URDF/mesh 集成包（ROS 包名与目录均为小写 `rangerboxcr10lidar_description`）。

主要适配点：

- 将 Ranger 底盘、CR10 机械臂、AG95 夹爪、RoboSense 雷达、RealSense D435 和上装箱体合并到同一个 `urdf/rangercr10lidar.urdf`（`<robot name="rangercr10lidar">`）。
- `cr10_mount_joint` 将 `cr10_base_link` 固定到底盘 `base_link`。
- AG95 固定到 `cr10_Link6`，并保留多组 mimic joint，使单个 `gripper_finger1_joint` 能驱动夹爪连杆联动。
- 雷达通过 `lidar_joint0` 固定在 `base_link` 上。
- RealSense D435 通过 `d435_joint` 固定到 `lidar_link0` 上方，并展开了 D435 的 depth/infra/color optical frame。
- `config/joint_names_rangerboxcr10lidar_description.yaml` 整合了底盘转向/轮子关节和 CR10 六轴关节。

结论：这是为了仿真、可视化和整机 TF/URDF 对齐新增的设备描述包。

## 结论

当前工作空间的真正设备适配主要集中在四类地方：

1. `ranger_ros/ranger_bringup`：把底盘、雷达、RF2O、GMapping、AMCL、move_base、地图和 RViz 串成实车导航栈。
2. `rslidar_sdk`、`rf2o_laser_odometry`、`SLAM`：把 RS-Helios-16P 的点云变成 2D `/scan`，再生成 `/odom` 和地图/定位所需 TF。
3. `plugin/rviz_navi_multi_goals_pub_plugin`：把原本的多目标导航面板扩展为建图、保存地图、选择地图、启动导航的一体化操作面板。
4. 新增 `gripper`、`lifting_ctrl`、`bt_task_msgs`、`rangerboxcr10lidar_description`：补齐 AG95 夹爪、升降机构、消息接口和整机 URDF。

`TCP-IP-ROS-6AXis` 和 `ugv_sdk` 基本原样使用，更多依赖运行参数或上层 launch 完成接入；`realsense-ros` 只有编译依赖补充，没有明显的设备定制逻辑。
