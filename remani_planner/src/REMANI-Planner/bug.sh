在第三次给点的时候还是会出现机械臂与障碍物穿模的情况，[FSM]: from INIT to WAIT_TARGET
[INFO] [1786373373.286693374]: [MMctrl] AUTO_STAY(L1) --> CMD_CTRL(L2)
[INFO] [1786373373.287620013]: [MMctrl] TRIGGER sent, allow user command.
[rospack] Error: failed to parse command-line options: too many positional options have been specified on the command line
[librospack]: error while executing command
[rospack] Error: failed to parse command-line options: too many positional options have been specified on the command line
[librospack]: error while executing command
[rospack] Error: failed to parse command-line options: too many positional options have been specified on the command line
[librospack]: error while executing command
Triggered! traget type: 1
[INFO] [1786373377.349803667]: FSM goal click: goal_xy=(5.592, -1.646) start_xy=(-5.000, 0.000) yaw=0.0 deg
[FSM]: from WAIT_TARGET to GEN_NEW_TRAJ

[replan 0]==============================================
[INFO] [1786373377.362214408]: continous_failures_count: 0
[INFO] [1786373379.044395856]: The optimization result is : Success: met stopping criteria (delta).
total time: 1730.48, init: 388.245, optimize: 1342.24, avg_time: 1730.48, count_success: 1
[INFO] [1786373379.096555201]: [MMctrl] Receive the trajectory. STAY --> POLY_TRAJ
[FSM]: from GEN_NEW_TRAJ to EXEC_TRAJ
[INFO] [1786373405.646535903]: [MMctrl] Stop execute the trajectory. POLY_TRAJ --> STAY
Triggered! traget type: 1
[INFO] [1786373412.029538832]: FSM goal click: goal_xy=(2.632, 4.517) start_xy=(5.592, -1.646) yaw=-17.6 deg
[TRIG]: from EXEC_TRAJ to GEN_NEW_TRAJ

[replan 1]==============================================
[INFO] [1786373412.036560748]: continous_failures_count: 0
[INFO] [1786373414.348940275]: The optimization result is : Success: met stopping criteria (delta).
total time: 2355.13, init: 1412.1, optimize: 943.034, avg_time: 2042.81, count_success: 2
[INFO] [1786373414.396517468]: [MMctrl] Receive the trajectory. STAY --> POLY_TRAJ
[FSM]: from GEN_NEW_TRAJ to EXEC_TRAJ
[INFO] [1786373437.936509947]: [MMctrl] Stop execute the trajectory. POLY_TRAJ --> STAY
Triggered! traget type: 1
[INFO] [1786373444.060021737]: FSM goal click: goal_xy=(-3.883, 4.884) start_xy=(2.632, 4.517) yaw=-160.6 deg
[TRIG]: from EXEC_TRAJ to GEN_NEW_TRAJ

[replan 2]==============================================
[INFO] [1786373444.067581904]: continous_failures_count: 0
[kino astar] open set empty, no path.
[WARN] [1786373444.067953305]: KinoAstar: Hybrid A* NO_PATH; refuse straight/RRT fallback (failures=0/5). start=(2.63,4.52) goal=(-3.88,4.88)
[kino astar] open set empty, no path.
[kino astar] open set empty, no path.
[kino astar] open set empty, no path.
[kino astar] open set empty, no path.
[WARN] [1786373444.069321282]: KinoAstar: switching to whole-body RRT after 5 failures. start=(2.63,4.52) goal=(-3.88,4.88)
[INFO] [1786373444.853302147]: The optimization result is : Success: met stopping criteria (delta).
total time: 813.268, init: 313.809, optimize: 499.459, avg_time: 1632.96, count_success: 3
[INFO] [1786373444.886509091]: [MMctrl] Receive the trajectory. STAY --> POLY_TRAJ
[FSM]: from GEN_NEW_TRAJ to EXEC_TRAJ
[INFO] [1786373460.996634866]: [MMctrl] Stop execute the trajectory. POLY_TRAJ --> STAY

