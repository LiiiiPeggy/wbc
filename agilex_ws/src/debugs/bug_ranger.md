> **注（命名已统一）：** 整机描述包现为 `rangerboxcr10lidar_description`，URDF 机器人名为 `rangercr10lidar`。下文终端输出为历史记录，其中的 `RangerCR10LiDAR_description` 为旧包名，重编译后该 WARNING 应消失。

### 问题1：启动底盘驱动之后，如何进行移动操作？

```bash
agilex@agilex-desktop:~/agilex_ws$ roslaunch ranger_bringup ranger.launch
WARNING: Package name "RangerCR10LiDAR_description" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
... logging to /home/agilex/.ros/log/ee091eee-5387-11f1-8567-68eda460820c/roslaunch-agilex-desktop-4270.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

WARNING: Package name "RangerCR10LiDAR_description" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
started roslaunch server http://agilex-desktop:44891/

SUMMARY
========

PARAMETERS
 * /ranger_base_node/base_frame: base_link
 * /ranger_base_node/odom_frame: odom
 * /ranger_base_node/odom_topic_name: odom
 * /ranger_base_node/port_name: can0
 * /ranger_base_node/publish_odom_tf: False
 * /ranger_base_node/robot_model: ranger
 * /ranger_base_node/update_rate: 50
 * /rosdistro: melodic
 * /rosversion: 1.14.13

NODES
  /
    ranger_base_node (ranger_base/ranger_base_node)

auto-starting new master
process[master]: started with pid [4280]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to ee091eee-5387-11f1-8567-68eda460820c
WARNING: Package name "RangerCR10LiDAR_description" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
process[rosout-1]: started with pid [4291]
started core service [/rosout]
process[ranger_base_node-2]: started with pid [4295]
[ INFO] [1779197870.972572472]: Successfully loaded the following parameters: 
 port_name: can0
 robot_model: ranger
 odom_frame: odom
 base_frame: base_link
 update_rate: 50
 odom_topic_name: odom
 publish_odom_tf: 0

Start listening to port: can0

```

### 问题2：启动导航节点之后，是如何进行导航操作？是由我来过给点然后底盘走过去吗？

```bash
roslaunch rslidar_sdk start.launch
roslaunch ranger_bringup navigation_4wd.launch
```



### 问题3：没有找到imu启动相关的，帮我看看

```bash
agilex@agilex-desktop:~/agilex_ws$ roslaunch 
Display all 335 possibilities? (y or n)
actionlib                                        openslam_gmapping
actionlib_msgs                                   orocos_kdl
actionlib_tutorials                              pcl_conversions
amcl                                             pcl_msgs
angles                                           pcl_ros
apriltag_ros                                     pluginlib
base_local_planner                               pluginlib_tutorials
bond                                             pointcloud_to_laserscan
bondcpp                                          polled_camera
bondpy                                           position_controllers
bt_task_msgs                                     pr2_controllers_msgs
build/                                           pr2_description
camera_calibration                               pr2_mechanism_msgs
camera_calibration_parsers                       pr2_moveit_config
camera_info_manager                              pr2_moveit_plugins
carrot_planner                                   pybind11_catkin
catkin                                           python_orocos_kdl
chomp_motion_planner                             python_qt_binding
class_loader                                     qt_dotgraph
clear_costmap_recovery                           qt_gui
cmake_modules                                    qt_gui_cpp
compressed_depth_image_transport                 qt_gui_py_common
compressed_image_transport                       qwt_dependency
controller_interface                             random_numbers
controller_manager                               ranger_base
controller_manager_msgs                          ranger_bringup
control_msgs                                     RangerCR10LiDAR_description
control_toolbox                                  ranger_msgs
costmap_2d                                       realsense2_camera
cpp_common                                       realsense2_description
cr10_moveit                                      realtime_tools
cr12_moveit                                      resource_retriever
cr16_moveit                                      rf2o_laser_odometry
cr3_moveit                                       robot_state_publisher
cr5_moveit                                       rosbag
cr7_moveit                                       rosbag_migration_rule
cv_bridge                                        rosbag_storage
ddynamic_reconfigure                             rosbash
depth_image_proc                                 rosboost_cfg
devel/                                           rosbuild
dh_gripper_driver                                rosclean
dh_gripper_msgs                                  rosconsole
diagnostic_aggregator                            rosconsole_bridge
diagnostic_analysis                              ros_control_boilerplate
diagnostic_common_diagnostics                    roscpp
diagnostic_msgs                                  roscpp_serialization
diagnostic_updater                               roscpp_traits
diff_drive_controller                            roscpp_tutorials
dobot_bringup                                    roscreate
dobot_description                                rosdemo_v3
dobot_gazebo                                     rosdemo_v4
dobot_moveit                                     ros_environment
dobot_v4_bringup                                 rosgraph
dwa_local_planner                                rosgraph_msgs
dynamic_reconfigure                              roslang
eigen_conversions                                roslaunch
eigenpy                                          roslib
eigen_stl_containers                             roslint
fake_localization                                roslisp
filters                                          roslz4
find_object_2d                                   rosmake
forward_command_controller                       rosmaster
gazebo_dev                                       rosmsg
gazebo_msgs                                      rosnode
gazebo_plugins                                   rosout
gazebo_ros                                       rospack
gazebo_ros_control                               rosparam
gencpp                                           rosparam_shortcuts
geneus                                           rospy
genlisp                                          rospy_tutorials
genmsg                                           rosservice
gennodejs                                        rostest
genpy                                            rostime
geometric_shapes                                 rostopic
geometry_msgs                                    rosunit
gl_dependency                                    roswtf
global_planner                                   rotate_recovery
gmapping                                         rqt_action
graph_msgs                                       rqt_bag
hardware_interface                               rqt_bag_plugins
image_geometry                                   rqt_console
image_proc                                       rqt_dep
image_publisher                                  rqt_graph
image_rotate                                     rqt_gui
image_transport                                  rqt_gui_cpp
image_view                                       rqt_gui_py
interactive_markers                              rqt_image_view
interactive_marker_tutorials                     rqt_launch
joint_limits_interface                           rqt_logger_level
joint_state_controller                           rqt_moveit
joint_state_publisher                            rqt_msg
joint_state_publisher_gui                        rqt_nav_view
joy_teleop                                       rqt_plot
kdl_conversions                                  rqt_pose_view
kdl_parser                                       rqt_publisher
kdl_parser_py                                    rqt_py_common
laser_assembler                                  rqt_py_console
laser_filters                                    rqt_reconfigure
laser_geometry                                   rqt_robot_dashboard
laser_scan_matcher                               rqt_robot_monitor
libg2o                                           rqt_robot_steering
libnabo                                          rqt_runtime_monitor
libpointmatcher                                  rqt_rviz
librviz_tutorial                                 rqt_service_caller
lifting_ctrl                                     rqt_shell
locomotor_msgs                                   rqt_srv
map_msgs                                         rqt_tf_tree
map_server                                       rqt_top
me6_moveit                                       rqt_topic
media_export                                     rqt_web
message_filters                                  rslidar_sdk
message_generation                               rtabmap
message_runtime                                  rtabmap_ros
mk                                               rviz
move_base                                        rviz_dobot_control
move_base_msgs                                   rviz_plugin_tutorials
moveit_chomp_optimizer_adapter                   rviz_python_tutorial
moveit_commander                                 rviz_visual_tools
moveit_controller_manager_example                self_test
moveit_core                                      sensor_msgs
moveit_experimental                              serial
moveit_fake_controller_manager                   shape_msgs
moveit_kinematics                                smach
moveit_msgs                                      smach_msgs
moveit_opw_kinematics_plugin                     smach_ros
moveit_planners_chomp                            smclib
moveit_planners_ompl                             spacenav_node
moveit_python                                    src/
moveit_resources_fanuc_description               srdfdom
moveit_resources_fanuc_moveit_config             stage
moveit_resources_panda_description               stage_ros
moveit_resources_panda_moveit_config             std_msgs
moveit_resources_pr2_description                 std_srvs
moveit_resources_prbt_ikfast_manipulator_plugin  stereo_image_proc
moveit_resources_prbt_moveit_config              stereo_msgs
moveit_resources_prbt_pg70_support               teleop_tools_msgs
moveit_resources_prbt_support                    tf
moveit_ros_benchmarks                            tf2
moveit_ros_control_interface                     tf2_eigen
moveit_ros_manipulation                          tf2_geometry_msgs
moveit_ros_move_group                            tf2_kdl
moveit_ros_occupancy_map_monitor                 tf2_msgs
moveit_ros_perception                            tf2_py
moveit_ros_planning                              tf2_ros
moveit_ros_planning_interface                    tf2_sensor_msgs
moveit_ros_robot_interaction                     tf_conversions
moveit_ros_visualization                         theora_image_transport
moveit_ros_warehouse                             topic_tools
moveit_servo                                     trajectory_msgs
moveit_setup_assistant                           transmission_interface
moveit_sim_controller                            turtle_actionlib
moveit_simple_controller_manager                 turtlesim
moveit_visual_tools                              turtle_tf
move_slow_and_clear                              turtle_tf2
nav_2d_msgs                                      ugv_sdk
nav_core                                         urdf
navfn                                            urdfdom_py
navi_multi_goals_pub_rviz_plugin                 urdf_parser_plugin
nav_msgs                                         urdf_sim_tutorial
nodelet                                          urdf_tutorial
nodelet_topic_tools                              visualization_marker_tutorials
nodelet_tutorial_math                            visualization_msgs
nova2_moveit                                     voxel_grid
nova5_moveit                                     warehouse_ros
object_recognition_msgs                          webkit_dependency
octomap                                          xacro
octomap_msgs                                     xmlrpcpp
ompl                                         
```