> **注（命名已统一）：** 整机描述包现为 `rangerboxcr10lidar_description`，URDF 机器人名为 `rangercr10lidar`。下文终端输出为历史记录，其中的 `RangerCR10LiDAR_description` 为旧包名。

### 问题1：运行如下命令之后的作用是什么，同时运行真实CR10控制出现了如下报错

```bash
export DOBOT_TYPE=cr10
roslaunch dobot_moveit moveit.launch

```

NODES
  /
    move_group (moveit_ros_move_group/move_group)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rviz_agilex_desktop_32143_7416681206341658944 (rviz/rviz)

auto-starting new master
process[master]: started with pid [32153]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 466b9b20-5383-11f1-8567-68eda460820c
WARNING: Package name "RangerCR10LiDAR_description" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
process[rosout-1]: started with pid [32164]
started core service [/rosout]
process[robot_state_publisher-2]: started with pid [32171]
process[move_group-3]: started with pid [32172]
process[rviz_agilex_desktop_32143_7416681206341658944-4]: started with pid [32173]
[ INFO] [1779195871.861408456]: Loading robot model 'cr10_robot'...
[ INFO] [1779195871.902708724]: rviz version 1.13.30
[ INFO] [1779195871.902755014]: compiled against Qt version 5.9.5
[ INFO] [1779195871.902761704]: compiled against OGRE version 1.9.0 (Ghadamon)
[ INFO] [1779195871.909480673]: Forcing OpenGl version 0.
[ INFO] [1779195872.015592523]: Stereo is NOT SUPPORTED
[ INFO] [1779195872.015648202]: OpenGL device: Mesa DRI Intel(R) UHD Graphics 630 (CFL GT2)
[ INFO] [1779195872.015661351]: OpenGl version: 3.0 (GLSL 1.3).
[ INFO] [1779195872.098142388]: Publishing maintained planning scene on 'monitored_planning_scene'
[ INFO] [1779195872.099491978]: MoveGroup debug mode is ON
Starting planning scene monitors...
[ INFO] [1779195872.099507117]: Starting planning scene monitor
[ INFO] [1779195872.100832720]: Listening to '/planning_scene'
[ INFO] [1779195872.100847903]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[ INFO] [1779195872.102086001]: Listening to '/collision_object'
[ INFO] [1779195872.103316402]: Listening to '/planning_scene_world' for planning scene world geometry
[ERROR] [1779195872.103883755]: No sensor plugin specified for octomap updater 0; ignoring.
[ INFO] [1779195872.233977235]: Listening to '/attached_collision_object' for attached collision objects
Planning scene monitors started.
[ INFO] [1779195872.249077021]: Initializing OMPL interface using ROS parameters
[ INFO] [1779195872.261059926]: Using planning interface 'OMPL'
[ INFO] [1779195872.264074056]: Param 'default_workspace_bounds' was not set. Using default value: 10
[ INFO] [1779195872.264630031]: Param 'start_state_max_bounds_error' was set to 0.1
[ INFO] [1779195872.264932195]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1779195872.265469217]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1779195872.265722102]: Param 'jiggle_fraction' was set to 0.05
[ INFO] [1779195872.265972068]: Param 'max_sampling_attempts' was not set. Using default value: 100
[ INFO] [1779195872.266012062]: Using planning request adapter 'Add Time Parameterization'
[ INFO] [1779195872.266021240]: Using planning request adapter 'Fix Workspace Bounds'
[ INFO] [1779195872.266028377]: Using planning request adapter 'Fix Start State Bounds'
[ INFO] [1779195872.266055854]: Using planning request adapter 'Fix Start State In Collision'
[ INFO] [1779195872.266081396]: Using planning request adapter 'Fix Start State Path Constraints'
[ INFO] [1779195875.249611262]: Loading robot model 'cr10_robot'...
[ INFO] [1779195875.555310049]: Starting planning scene monitor
[ INFO] [1779195875.557281956]: Listening to '/move_group/monitored_planning_scene'
[ INFO] [1779195875.637306868]: waitForService: Service [/get_planning_scene] has not been advertised, waiting...
[ WARN] [1779195877.279120093]: Waiting for cr10_robot/joint_controller/follow_joint_trajectory to come up
[ INFO] [1779195880.647006609]: Failed to call service get_planning_scene, have you launched move_group or called psm.providePlanningSceneService()?
[ INFO] [1779195880.647589308]: Constructing new MoveGroup connection for group 'arm' in namespace ''
[ WARN] [1779195883.279373244]: Waiting for cr10_robot/joint_controller/follow_joint_trajectory to come up
[ERROR] [1779195889.279749927]: Action client not connected: cr10_robot/joint_controller/follow_joint_trajectory
[ INFO] [1779195889.299959277]: Returned 0 controllers in list
[ INFO] [1779195889.310992664]: Trajectory execution is managing controllers
Loading 'move_group/ApplyPlanningSceneService'...
Loading 'move_group/ClearOctomapService'...
Loading 'move_group/MoveGroupCartesianPathService'...
Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
Loading 'move_group/MoveGroupGetPlanningSceneService'...
Loading 'move_group/MoveGroupKinematicsService'...
Loading 'move_group/MoveGroupMoveAction'...
Loading 'move_group/MoveGroupPickPlaceAction'...
Loading 'move_group/MoveGroupPlanService'...
Loading 'move_group/MoveGroupQueryPlannersService'...
Loading 'move_group/MoveGroupStateValidationService'...
[ INFO] [1779195889.347630738]: 

********************************************************
* MoveGroup using: 
*     - ApplyPlanningSceneService
*     - ClearOctomapService
*     - CartesianPathService
*     - ExecuteTrajectoryAction
*     - GetPlanningSceneService
*     - KinematicsService
*     - MoveAction
*     - PickPlaceAction
*     - MotionPlanService
*     - QueryPlannersService
*     - StateValidationService
********************************************************

[ INFO] [1779195889.347733210]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[ INFO] [1779195889.347747050]: MoveGroup context initialization complete

You can start planning now!

[ INFO] [1779195890.423433689]: Ready to take commands for planning group arm.
[ INFO] [1779195890.423580109]: Looking around: no
[ INFO] [1779195890.423658135]: Replanning: no
[ INFO] [1779195899.925797993]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[ INFO] [1779195899.929480803]: Planner configuration 'arm[RRT]' will use planner 'geometric::RRT'. Additional configuration parameters will be set when the planner is constructed.
[ INFO] [1779195899.931604453]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.931751284]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.931916208]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.932101980]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.940372192]: arm[RRT]: Created 6 states
[ INFO] [1779195899.957563886]: arm[RRT]: Created 25 states
[ INFO] [1779195899.958044819]: arm[RRT]: Created 28 states
[ INFO] [1779195899.963205311]: arm[RRT]: Created 39 states
[ INFO] [1779195899.963377359]: ParallelPlan::solve(): Solution found by one or more threads in 0.032454 seconds
[ INFO] [1779195899.963615000]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.963641688]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.963717137]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.963748932]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.963945447]: arm[RRT]: Created 2 states
[ INFO] [1779195899.970487790]: arm[RRT]: Created 15 states
[ INFO] [1779195899.972072972]: arm[RRT]: Created 17 states
[ INFO] [1779195899.972874772]: arm[RRT]: Created 21 states
[ INFO] [1779195899.972972462]: ParallelPlan::solve(): Solution found by one or more threads in 0.009422 seconds
[ INFO] [1779195899.973126997]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.973154144]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195899.979756340]: arm[RRT]: Created 20 states
[ INFO] [1779195900.001638169]: arm[RRT]: Created 84 states
[ INFO] [1779195900.001870150]: ParallelPlan::solve(): Solution found by one or more threads in 0.028789 seconds
[ INFO] [1779195900.002038039]: SimpleSetup: Path simplification took 0.000009 seconds and changed from 2 to 2 states
[ INFO] [1779195901.615731497]: Execution request received
[ INFO] [1779195901.615878977]: Returned 0 controllers in list
[ERROR] [1779195901.615981754]: Unable to identify any set of controllers that can actuate the specified joints: [ joint1 joint2 joint3 joint4 joint5 joint6 ]
[ERROR] [1779195901.616050830]: Known controllers and their joints:

[ INFO] [1779195901.631578573]: ABORTED: Solution found but controller failed during execution
[ INFO] [1779195931.825654813]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[ INFO] [1779195931.825998524]: Planning attempt 1 of at most 1
[ INFO] [1779195931.827022181]: Planner configuration 'arm[RRT]' will use planner 'geometric::RRT'. Additional configuration parameters will be set when the planner is constructed.
[ INFO] [1779195931.828126770]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.828347189]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.828590582]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.828764121]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.852885098]: arm[RRT]: Created 15 states
[ INFO] [1779195931.853626355]: arm[RRT]: Created 13 states
[ INFO] [1779195931.854623987]: arm[RRT]: Created 15 states
[ INFO] [1779195931.855834434]: arm[RRT]: Created 15 states
[ INFO] [1779195931.856007119]: ParallelPlan::solve(): Solution found by one or more threads in 0.028270 seconds
[ INFO] [1779195931.856486581]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.856566049]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.856692790]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.856790224]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.866442633]: arm[RRT]: Created 11 states
[ INFO] [1779195931.875447367]: arm[RRT]: Created 28 states
[ INFO] [1779195931.885824019]: arm[RRT]: Created 45 states
[ INFO] [1779195931.887438772]: arm[RRT]: Created 56 states
[ INFO] [1779195931.887505022]: ParallelPlan::solve(): Solution found by one or more threads in 0.031162 seconds
[ INFO] [1779195931.887627737]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.887663649]: arm[RRT]: Starting planning with 1 states already in datastructure
[ INFO] [1779195931.900550644]: arm[RRT]: Created 40 states
[ INFO] [1779195931.901943846]: arm[RRT]: Created 39 states
[ INFO] [1779195931.902076122]: ParallelPlan::solve(): Solution found by one or more threads in 0.014464 seconds
[ INFO] [1779195931.902126280]: SimpleSetup: Path simplification took 0.000001 seconds and changed from 2 to 2 states
[ INFO] [1779195931.902667568]: Returned 0 controllers in list
[ERROR] [1779195931.902700301]: Unable to identify any set of controllers that can actuate the specified joints: [ joint1 joint2 joint3 joint4 joint5 joint6 ]
[ERROR] [1779195931.902732200]: Known controllers and their joints:

[ERROR] [1779195931.902768820]: Apparently trajectory initialization failed
[ INFO] [1779195931.934594504]: ABORTED: Solution found but controller failed during execution


### 问题2：运行如下命令之后如何操作机械臂？
```bash
export DOBOT_TYPE=cr10
roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.5.1

```
