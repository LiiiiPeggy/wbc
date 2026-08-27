#include <plan_manage/remani_replan_fsm.h>
#include <mm_config/ee_kinematics_utils.hpp>
#include <cmath>

// ################################
// C++: EE goal quaternion sanitizer (does not mutate ConstPtr) begin
// ################################
namespace {
bool sanitizePoseQuaternion(const geometry_msgs::Quaternion &msg_q,
                            Eigen::Quaterniond &q_out) {
  if (!std::isfinite(msg_q.x) || !std::isfinite(msg_q.y) ||
      !std::isfinite(msg_q.z) || !std::isfinite(msg_q.w)) {
    return false;
  }
  Eigen::Quaterniond q(msg_q.w, msg_q.x, msg_q.y, msg_q.z);
  const double n = q.norm();
  if (n < 1e-12) {
    return false;
  }
  if (std::abs(n - 1.0) > 1e-3) {
    ROS_WARN("[EE GOAL] quaternion not unit (norm=%.6f), normalizing", n);
  }
  q.normalize();
  q_out = q;
  return true;
}
}  // namespace
// ################################
// C++: EE goal quaternion sanitizer (does not mutate ConstPtr) end
// ################################

namespace remani_planner
{
  REMANIReplanFSM::~REMANIReplanFSM(){}
  void REMANIReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    try_plan_after_emergency_ = false;
    flag_relan_astar_ = false;
    have_local_traj_ = false;
    replan_fail_time_ = 0;

    /*  fsm param  */
    nh.param("fsm/target_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0);
    nh.param("fsm/time_for_gripper", time_for_gripper_, -1.0);
    nh.param("fsm/global_plan", global_plan_, false);
    nh.param("fsm/max_continuous_plan_failures", max_continuous_plan_failures_, 20);
    if(max_continuous_plan_failures_ < 1) max_continuous_plan_failures_ = 1;
    if(global_plan_) planning_horizen_ = 1.0e3;

    nh.param("mm/mobile_base_dof", mobile_base_dim_, -1);
    nh.param("mm/manipulator_dof", manipulator_dim_, -1);
    nh.param("mm/mobile_base_non_singul_vel", mobile_base_non_singul_vel_, -1.0);
    

    traj_dim_ = mobile_base_dim_ + manipulator_dim_;

    mm_state_pos_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_vel_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_acc_ = Eigen::VectorXd::Zero(traj_dim_);

    gripper_flag_ = true;

    start_pos_.resize(traj_dim_);
    start_vel_.resize(traj_dim_);
    start_acc_.resize(traj_dim_);
    start_jer_.resize(traj_dim_);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);

    waypoints_.clear();
    waypoints_yaw_.clear();
    Eigen::VectorXd wp = Eigen::VectorXd::Zero(traj_dim_);
    double yaw_temp;
    bool gripper_close;
    for (int i = 0; i < waypoint_num_; i++){
      nh.param("fsm/waypoint" + to_string(i) + "_yaw", yaw_temp, -1.0);
      waypoints_yaw_.push_back(yaw_temp * M_PI / 180.0);

      nh.param("fsm/waypoint" + to_string(i) + "_gripper_close", gripper_close, true);
      waypoint_gripper_close_.push_back(gripper_close);

      std::vector<double> waypoints_temp;
      nh.getParam("fsm/waypoint" + to_string(i), waypoints_temp);
      for(unsigned int j = 0; j < waypoints_temp.size(); j++){
        wp(j) = waypoints_temp[j];
        if((int)j >= mobile_base_dim_) wp(j) = wp(j) * M_PI / 180.0;
      }
      waypoints_.push_back(wp);
    }
    
    init_time_list_.clear();
    opt_time_list_.clear();
    total_time_list_.clear();

    rcv_gripper_state_ = false;
    gripper_state_ = false;
    map_state_ = 0;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new MMPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    // ################################
    // C++: WholeBodyIkSolver + EE I/O init begin
    // ################################
    have_joint_state_ = false;
    active_goal_source_ = GoalSource::NONE;
    pending_ee_goal_ = false;
    active_ee_goal_ = false;
    last_ee_current_pose_pub_ = ros::Time(0);

    nh.param("fsm/ee_reach_pos_tol", ee_reach_pos_tol_, 0.02);
    double ee_reach_rot_tol_deg = 4.0;
    nh.param("fsm/ee_reach_rot_tol_deg", ee_reach_rot_tol_deg, 4.0);
    ee_reach_rot_tol_rad_ = ee_reach_rot_tol_deg * M_PI / 180.0;

    try{
      WholeBodyIkParams ik_params = WholeBodyIkParams::loadFromRosParam(nh);
      whole_body_ik_ = std::make_shared<WholeBodyIkSolver>(
          planner_manager_->mm_config_,
          planner_manager_->grid_map_,
          ik_params);
      if(!ik_params.ee_goal_topic.empty()){
        ee_goal_sub_ = nh.subscribe(ik_params.ee_goal_topic, 1,
                                    &REMANIReplanFSM::eeGoalCallback, this);
      }else{
        ee_goal_sub_ = nh.subscribe("/ee_goal", 1,
                                    &REMANIReplanFSM::eeGoalCallback, this);
      }
      const std::string ee_cur_topic = ik_params.ee_current_pose_topic.empty()
          ? "/ee_current_pose"
          : ik_params.ee_current_pose_topic;
      ee_current_pose_pub_ =
          nh.advertise<geometry_msgs::PoseStamped>(ee_cur_topic, 10);
    }catch(const std::exception &ex){
      ROS_ERROR("[EE IK] failed to init WholeBodyIkSolver: %s", ex.what());
      whole_body_ik_.reset();
    }
    ee_ik_terminal_marker_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>("/ee_ik_terminal_markers", 1);
    // ################################
    // C++: WholeBodyIkSolver + EE I/O init end
    // ################################

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &REMANIReplanFSM::mmCarOdomCallback, this);
    joint_state_sub_ = nh.subscribe("joint_state", 1, &REMANIReplanFSM::mmManiOdomCallback, this);
    gripper_state_sub_ = nh.subscribe("gripper_state", 1, &REMANIReplanFSM::gripperCallback, this);

    poly_traj_pub_ = nh.advertise<quadrotor_msgs::PolynomialTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    gripper_cmd_pub_ = nh.advertise<std_msgs::Bool>("gripper_cmd", 100);
    map_state_pub_ = nh.advertise<std_msgs::Int32>("/map_generator/map_state", 100);

    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &REMANIReplanFSM::waypointCallback, this);
    
  }

  void REMANIReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    // ################################
    // C++: publish /ee_current_pose at ≤20 Hz begin
    // ################################
    maybePublishEeCurrentPose();
    // ################################
    // C++: publish /ee_current_pose at ≤20 Hz end
    // ################################

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100){
      fsm_num = 0;
      // printFSMExecState();
    }

    switch (exec_state_){
    case INIT:
    {
      if (!have_odom_){
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // return;
      else{
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if(try_plan_after_emergency_){
        std::cout << "emergency stop mm pos: " << mm_state_pos_.transpose() << std::endl;
        std::cout << "emergency stop mm vel: " << mm_state_vel_.transpose() << std::endl;
        std::cout << "emergency stop mm acc: " << mm_state_acc_.transpose() << std::endl;
        std::cout << "emergency stop mm yaw: " << mm_car_yaw_ << std::endl;
      }
      // std::cout << "gen new traj 1\n";
      have_local_traj_ = false;
      bool success = planFromGlobalTraj(10);
      // std::cout << "gen new traj 2\n";
      if (success){
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        try_plan_after_emergency_ = false;
      }
      else
      {
        // ROS_ERROR("Failed to generate new trajectory!!!");
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        // ################################
        // C++: abort unreachable / invalid new-task goal begin
        // ################################
        const int kino_st = planner_manager_->getLastKinoStatus();
        const bool invalid_terminal =
            (kino_st == KinoAstar::GOAL_COLLISION || kino_st == KinoAstar::START_COLLISION);
        if(invalid_terminal){
          ROS_ERROR("[FSM] abort current goal: terminal/start state invalid (kino status=%d)", kino_st);
          have_target_ = false;
          have_trigger_ = false;
          // ################################
          // C++: EE metadata cleanup on GEN_NEW_TRAJ abort begin
          // ################################
          if(active_goal_source_ == GoalSource::EE_POSE){
            clearActiveEeGoal();
          }
          // ################################
          // C++: EE metadata cleanup on GEN_NEW_TRAJ abort end
          // ################################
          changeFSMExecState(WAIT_TARGET, "FSM");
        }else if(planner_manager_->getFailureCount() >= max_continuous_plan_failures_){
          ROS_ERROR("[FSM] abort current goal after %d consecutive planning failures",
                    planner_manager_->getFailureCount());
          have_target_ = false;
          have_trigger_ = false;
          // ################################
          // C++: EE metadata cleanup on GEN_NEW_TRAJ abort begin
          // ################################
          if(active_goal_source_ == GoalSource::EE_POSE){
            clearActiveEeGoal();
          }
          // ################################
          // C++: EE metadata cleanup on GEN_NEW_TRAJ abort end
          // ################################
          changeFSMExecState(WAIT_TARGET, "FSM");
        }else{
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        // ################################
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      
      if(planFromLocalTraj(flag_relan_astar_)){
        replan_fail_time_ = 0;
        flag_relan_astar_ = false;
        if((ros::Time::now() - t_last_Astar_ ).toSec() > 1.0){
          std::cout << "cal front end next time" << std::endl;
          flag_relan_astar_ = true;
          t_last_Astar_ = ros::Time::now();
        }
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else{
        replan_fail_time_++;
        flag_relan_astar_ = true;
        t_last_Astar_ = ros::Time::now();
        if(replan_fail_time_ >= 20){
          replan_fail_time_ = 0;
          ROS_ERROR("[FSM]:REPLAN fail over 20 times!!!");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else{
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
      // LocalTrajData *info = &planner_manager_->traj_container_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      bool need_to_plan_next = ((t_cur - info->duration) > time_for_gripper_);
      bool need_to_gripper = (t_cur > info->duration + 0.01);
      t_cur = min(info->duration, t_cur);

      Eigen::VectorXd pos = info->getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);
      bool close_to_no_replan_thresh = ((end_pt_ - pos).head(2).norm() < no_replan_thresh_);

      if((target_type_ == TARGET_TYPE::PRESET_TARGET) && close_to_no_replan_thresh){
        if((wpt_id_ < waypoint_num_ - 1) && need_to_plan_next){
          ++wpt_id_;
          planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
          gripper_flag_ = true;
        }else if(need_to_gripper && gripper_flag_){
          ++map_state_;
          std_msgs::Bool gripper_cmd;
          gripper_cmd.data = waypoint_gripper_close_[wpt_id_]; // true: close gripper; false: open
          gripper_cmd_pub_.publish(gripper_cmd);

          std::string gripper_cmd_str = waypoint_gripper_close_[wpt_id_] ? "close gripper" : "open gripper";
          ROS_INFO(gripper_cmd_str.c_str());

          std_msgs::Int32 map_state;
          map_state.data = map_state_;
          // map_state_pub_.publish(map_state);

          // planner_manager_->grid_map_->md_.has_cloud_ = false;

          gripper_flag_ = false;
        }
        
      }else if(t_cur > info->duration - 1e-2 && touch_the_goal){

        // ################################
        // C++: EE-only EXEC_TRAJ completion branch begin
        // ################################
        if(active_goal_source_ == GoalSource::EE_POSE){
          Eigen::Vector3d car_now(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
          Eigen::VectorXd q_now = mm_state_pos_.tail(manipulator_dim_);
          const Eigen::Matrix4d T_actual =
              planner_manager_->mm_config_->getEePose(car_now, q_now);
          Eigen::Matrix<double, 6, 1> e;
          poseError(T_actual, T_world_ee_goal_, e);
          const double pos_err = e.head<3>().norm();
          const double rot_err = e.tail<3>().norm();

          have_target_ = false;
          have_trigger_ = false;
          clearActiveEeGoal();
          changeFSMExecState(WAIT_TARGET, "FSM");

          if(pos_err <= ee_reach_pos_tol_ && rot_err <= ee_reach_rot_tol_rad_){
            ROS_INFO("[EE GOAL] reached");
            std_msgs::Bool msg;
            msg.data = true;
            reached_pub_.publish(msg);
          }else{
            ROS_WARN("[EE GOAL] execution finished but pose tolerance not met: "
                     "pos=%.3f rot=%.1f deg",
                     pos_err, rot_err * 180.0 / M_PI);
          }
          goto force_return;
        }
        // ################################
        // C++: EE-only EXEC_TRAJ completion branch end
        // ################################
        
        if(target_type_ != TARGET_TYPE::PRESET_TARGET && wpt_id_ >= waypoint_num_ - 1){
          have_target_ = false;
          have_trigger_ = false;
          /* The navigation task completed */
          std::cout << "reach goal\n";
          // ################################
          // C++: NAV GoalSource closure begin
          // ################################
          active_goal_source_ = GoalSource::NONE;
          // ################################
          // C++: NAV GoalSource closure end
          // ################################
          changeFSMExecState(WAIT_TARGET, "FSM");

          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }
        
      }else if(!close_to_no_replan_thresh && t_cur > replan_thresh_ && (!global_plan_)){
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if(flag_escape_emergency_){ // Avoiding repeated calls
        callEmergencyStop(mm_state_pos_, mm_car_yaw_, mm_car_singul_);
      }
      else{
        if(enable_fail_safe_ && mm_state_vel_.head(2).norm() < 0.1){
          try_plan_after_emergency_ = true;
          have_local_traj_ = false;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }

      flag_escape_emergency_ = false;

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;
    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout()){
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }
    // std::cout << "check 3" << std::endl;
    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    Eigen::VectorXd p_cur = info->getPos(t_cur);
    double t_1_2 = info->duration * 1 / 2;
    double t_2_3 = info->duration * 2 / 3;
    double t_temp;
    bool occ = false;
    // std::cout << "check 4" << std::endl;
    int coll_type;
    for (double t = t_cur; t < info->duration; t += time_step){
      // If t_cur < t_1_2, only the first 2/3 partition of the trajectory is considered valid and will get checked.
      if (t_cur < t_1_2 && t >= t_2_3)
        break;
        
      if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t, coll_type)){
        if(coll_type == 0){
          ROS_WARN("car collision at relative time %f!", t / info->duration);
        }else if (coll_type == 1){
          ROS_WARN("mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 2){
          ROS_WARN("car-mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 3){
          ROS_WARN("mani-mani collision at relative time %f!", t / info->duration);
        }
        
        t_temp = t;
        occ = true;
        break;
      }
    }

    if (occ){
      /* Handle the collided case immediately */
      ROS_INFO("Try to replan a safe trajectory");
      if (planFromLocalTraj(false)){ // Make a chance
        ROS_INFO("Plan success when detect collision.");
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }else{
        // if(planFromLocalTraj(true))
        // {
        //   ROS_INFO("Plan success when detect collision.");
        //   changeFSMExecState(EXEC_TRAJ, "SAFETY");
        //   return;
        // }
        if (t_temp - t_cur < emergency_time_){ // 1.0s of emergency time
          ROS_WARN("Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }else{
          ROS_WARN("current traj in collision, replan.");
          if(planFromLocalTraj(true))
          {
            ROS_INFO("Plan success when detect collision.");
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  bool REMANIReplanFSM::planNextWaypoint(const Eigen::VectorXd next_wp, const double next_yaw)
  {
    std::vector<Eigen::VectorXd> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    bool success = planner_manager_->planGlobalTrajWaypoints(
        mm_state_pos_, mm_car_yaw_, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_),
        one_pt_wps, next_yaw, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_));

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;
      end_yaw_ = next_yaw;
      have_local_traj_ = false;
      start_singul_ = 0;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_container_.global_traj.duration / step_size_t);
      vector<Eigen::Vector2d> global_traj(i_end);
      for (int i = 0; i < i_end; i++){
        global_traj[i] = planner_manager_->traj_container_.global_traj.traj.getPos(i * step_size_t).head(mobile_base_dim_);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGoalPoint(end_pt_.head(2), Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalTraj(global_traj, 0.05, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  // manual waypoint
  void REMANIReplanFSM::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    
    if (target_type_ == TARGET_TYPE::PRESET_TARGET){
      have_trigger_ = true;
      cout << "Triggered! traget type: " << target_type_ << endl;

      // ################################
      // C++: preset trigger also resets failure count begin
      // ################################
      planner_manager_->resetFailureCount();
      // ################################
      // C++: preset trigger also resets failure count end
      // ################################

      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      start_pub_.publish(flag_msg);
      wpt_id_ = 0;
      // ################################
      // C++: preset two-phase EE metadata commit begin
      // ################################
      const bool preset_ok =
          planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
      if(preset_ok){
        if(active_goal_source_ == GoalSource::EE_POSE){
          clearActiveEeGoal();
        }
        active_goal_source_ = GoalSource::NAV_2D;
      }
      // ################################
      // C++: preset two-phase EE metadata commit end
      // ################################
      return;
    }

    if(msg->pose.position.z < -0.1)
      return;
    cout << "Triggered! traget type: " << target_type_ << endl;

    if(target_type_ != TARGET_TYPE::MANUAL_TARGET){
      ROS_ERROR("wrong target type: %d", target_type_);
      return;
    }

    // ################################
    // C++: reject out-of-map 2D Nav Goal via GridMap begin
    // ################################
    {
      const double gx = msg->pose.position.x;
      const double gy = msg->pose.position.y;
      Eigen::Vector3d map_ori, map_size;
      planner_manager_->grid_map_->getRegion(map_ori, map_size);
      if(!planner_manager_->grid_map_->isInMap(Eigen::Vector2d(gx, gy))){
        ROS_WARN("[GOAL] Reject out-of-map goal: (%.3f, %.3f), valid x=[%.3f, %.3f], y=[%.3f, %.3f]",
                 gx, gy,
                 map_ori(0), map_ori(0) + map_size(0),
                 map_ori(1), map_ori(1) + map_size(1));
        return;
      }
    }
    // ################################
    // C++: reject out-of-map 2D Nav Goal via GridMap end
    // ################################

    // ################################
    // C++: new manual goal resets failure count begin
    // ################################
    // Only on a fresh user 2D Nav Goal — not on auto-retry / mid-traj replan.
    planner_manager_->resetFailureCount();
    ROS_INFO("FSM new goal: reset continous_failures_count to 0");
    // ################################
    // C++: new manual goal resets failure count end
    // ################################

    // ################################
    // C++: manual goal also sets trigger begin
    // ################################
    have_trigger_ = true;
    // ################################
    // C++: manual goal also sets trigger end
    // ################################
    init_state_ = mm_state_pos_;
    end_pt_ = Eigen::VectorXd::Zero(traj_dim_);

    // ################################
    // C++: manual goal keeps current arm joints begin
    // ################################
    end_pt_ = mm_state_pos_;
    end_pt_(0) = msg->pose.position.x;
    end_pt_(1) = msg->pose.position.y;
    // ################################
    // C++: log clicked goal + current pose begin
    // ################################
    ROS_INFO("FSM goal click: goal_xy=(%.3f, %.3f) start_xy=(%.3f, %.3f) yaw=%.1f deg",
             end_pt_(0), end_pt_(1),
             mm_state_pos_(0), mm_state_pos_(1),
             mm_car_yaw_ * 180.0 / M_PI);
    // ################################
    // C++: log clicked goal + current pose end
    // ################################
    end_yaw_ = tf::getYaw(msg->pose.orientation);
    // ################################
    // C++: manual goal keeps current arm joints end
    // ################################

    // ################################
    // C++: 2D Nav Goal two-phase EE metadata commit begin
    // ################################
    const bool ok = planNextWaypoint(end_pt_, end_yaw_);
    if(ok){
      if(active_goal_source_ == GoalSource::EE_POSE){
        clearActiveEeGoal();
      }
      active_goal_source_ = GoalSource::NAV_2D;
    }
    // ################################
    // C++: 2D Nav Goal two-phase EE metadata commit end
    // ################################
  }

  void REMANIReplanFSM::mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // std::cout << "odom: " << mm_state_pos_.transpose() << "\n";
    mm_state_pos_(0) = msg->pose.pose.position.x;
    mm_state_pos_(1) = msg->pose.pose.position.y;
    mm_car_yaw_ = tf::getYaw(msg->pose.pose.orientation);

    mm_car_orient_.w() = msg->pose.pose.orientation.w;
    mm_car_orient_.x() = msg->pose.pose.orientation.x;
    mm_car_orient_.y() = msg->pose.pose.orientation.y;
    mm_car_orient_.z() = msg->pose.pose.orientation.z;

    mm_state_vel_(0) = msg->twist.twist.linear.x;
    mm_state_vel_(1) = msg->twist.twist.linear.y;
    if(mm_state_vel_.head(2).norm() < mobile_base_non_singul_vel_){
      mm_state_vel_(0) = mobile_base_non_singul_vel_ * cos(mm_car_yaw_);
      mm_state_vel_(1) = mobile_base_non_singul_vel_ * sin(mm_car_yaw_);
      mm_car_singul_ = 0;
    }else{
      Eigen::Vector2d car_head(cos(mm_car_yaw_), sin(mm_car_yaw_));
      mm_car_singul_ = 1 ? car_head.dot(mm_state_vel_.head(2)) >= 0 : -1;
    }

    mm_car_yaw_rate_ = msg->twist.twist.angular.z;

    have_odom_ = true;
  }

  void REMANIReplanFSM::mmManiOdomCallback(const sensor_msgs::JointStateConstPtr &msg){
    // ################################
    // C++: guard short JointState begin
    // ################################
    if((int)msg->position.size() < manipulator_dim_){
      ROS_WARN_THROTTLE(1.0, "joint_state position size %zu < %d, skip", msg->position.size(), manipulator_dim_);
      return;
    }
    for(int i = 0; i < manipulator_dim_; ++i){
      mm_state_pos_(mobile_base_dim_ + i) = msg->position[i];
      mm_state_vel_(mobile_base_dim_ + i) = ((int)msg->velocity.size() > i) ? msg->velocity[i] : 0.0;
      mm_state_acc_(mobile_base_dim_ + i) = ((int)msg->effort.size() > i) ? msg->effort[i] : 0.0;
    }
    have_joint_state_ = true;
    // ################################
    // C++: guard short JointState end
    // ################################
  }

  void REMANIReplanFSM::gripperCallback(const std_msgs::Bool::ConstPtr &msg){
    if(gripper_state_ != msg->data || (!rcv_gripper_state_)){
      rcv_gripper_state_ = true;
      gripper_state_ = msg->data;
      planner_manager_->mm_config_->setGripperPoint(gripper_state_);
    }
  }

  void REMANIReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    // ################################
    // C++: throttle same-state FSM spam begin
    // ################################
    if(pre_s == int(new_state)){
      ROS_INFO_THROTTLE(2.0, "[%s]: stay in %s (x%d)", pos_call.c_str(),
                        state_str[int(new_state)].c_str(), continously_called_times_);
    }else{
      cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    }
    // ################################
    // C++: throttle same-state FSM spam end
    // ################################
  }

  void REMANIReplanFSM::printFSMExecState(){
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> REMANIReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void REMANIReplanFSM::sendPolyTrajROSMsg(){
    auto data = &planner_manager_->traj_container_.singul_traj_data;
    
    for(unsigned int i = 0; i < data->singul_traj.size(); ++i){
      quadrotor_msgs::PolynomialTraj msg;
      msg.trajectory_id = data->singul_traj[i].traj_id;
      msg.header.stamp = ros::Time(data->start_time);
      msg.action = msg.ACTION_ADD;
      msg.singul = data->singul_traj[i].singul;
      int piece_num = data->singul_traj[i].traj.getPieceNum();
      for (int j = 0; j < piece_num; ++j)
      {
        quadrotor_msgs::PolynomialMatrix piece;
        piece.num_dim = data->singul_traj[i].traj.getPiece(j).getDim();
        piece.num_order = data->singul_traj[i].traj.getPiece(j).getDegree();
        piece.duration = data->singul_traj[i].traj.getPiece(j).getDuration();
        auto cMat = data->singul_traj[i].traj.getPiece(j).getCoeffMat();
        piece.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
        msg.trajectory.emplace_back(piece);
      }
      poly_traj_pub_.publish(msg);
    }

  }

  bool REMANIReplanFSM::planFromGlobalTraj(const int trial_times /*= 1*/){
    start_pos_ = mm_state_pos_;
    start_vel_ = mm_state_vel_;
    start_acc_.setZero();
    start_jer_.setZero();
    start_yaw_ = mm_car_yaw_;
    start_singul_ = mm_car_singul_;
    bool flag_random_poly_init;
    if(timesOfConsecutiveStateCalls().first == 1) flag_random_poly_init = false;
    else flag_random_poly_init = true;
    for(int i = 0; i < trial_times; i++){
      if(callReboundReplan(true, flag_random_poly_init)){
        return true;
      }
    }
    return false;
  }

  bool REMANIReplanFSM::planFromLocalTraj(bool flag_use_poly_init){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    double t_cur = ros::Time::now().toSec() - info->start_time + replan_trajectory_time_;
    t_cur = min(info->duration, t_cur);

    start_pos_     = info->getPos(t_cur);
    start_vel_    = info->getVel(t_cur);
    start_acc_    = info->getAcc(t_cur);
    start_jer_   = info->getJer(t_cur);
    start_singul_ = info->getSingul(t_cur);
    if(start_vel_.norm() >= mobile_base_non_singul_vel_) start_yaw_ = atan2(start_singul_ * start_vel_(1), start_singul_ * start_vel_(0));
    else start_yaw_ = mm_car_yaw_;

    bool success = callReboundReplan(flag_use_poly_init, false);
    if (!success){
      for (int i = 0; i < 1; i++){
        success = callReboundReplan(true, true);
        if (success)
          break;
      }
      if (!success)
      {
        return false;
      }
    }

    return true;
  }

  bool REMANIReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj){
    bool reach_horizon;
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pos_, start_yaw_, end_pt_, end_yaw_,
        local_target_pt_, local_target_vel_, local_target_acc_, reach_horizon);
    bool local_target_gripper;
    if(reach_horizon){
      local_target_gripper = gripper_state_;
    }else{
      local_target_gripper = waypoint_gripper_close_[wpt_id_];
    }
    local_target_acc_.setZero();
    double local_target_yaw = atan2(local_target_vel_(1), local_target_vel_(0)); // global traj is foreward, no need to take singul into account
    local_target_vel_.setZero();
    local_target_vel_.head(2) = mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(local_target_yaw), sin(local_target_yaw));

    Eigen::VectorXd desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk;
    int desired_start_singul;
    double desired_start_yaw;
    double desired_start_time, start_time_dura;
    
    if(have_local_traj_)
    {
      desired_start_time = ros::Time::now().toSec() + replan_trajectory_time_;
      start_time_dura = desired_start_time - planner_manager_->traj_container_.singul_traj_data.start_time;
      start_time_dura = min(start_time_dura, planner_manager_->traj_container_.singul_traj_data.duration);
      
      desired_start_pt = planner_manager_->traj_container_.singul_traj_data.getPos(start_time_dura);
      desired_start_vel = planner_manager_->traj_container_.singul_traj_data.getVel(start_time_dura);
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_singul = planner_manager_->traj_container_.singul_traj_data.getSingul(start_time_dura);
      desired_start_acc = planner_manager_->traj_container_.singul_traj_data.getAcc(start_time_dura);
      desired_start_jerk = planner_manager_->traj_container_.singul_traj_data.getJer(start_time_dura);
      desired_start_yaw = atan2(desired_start_singul * desired_start_vel(1), desired_start_singul * desired_start_vel(0));
    }else{
      desired_start_time = ros::Time::now().toSec();
      desired_start_pt = start_pos_;
      desired_start_vel = start_vel_;
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jer_;
      desired_start_yaw = start_yaw_;
      desired_start_singul = start_singul_;
    }
    // std::cout << "desired_start_singul: " << desired_start_singul << std::endl;
    double init_time, opt_time;
    
    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc,desired_start_jerk, desired_start_yaw, desired_start_singul, gripper_state_,
        desired_start_time, local_target_pt_, local_target_vel_, local_target_acc_, local_target_yaw, local_target_gripper,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, have_local_traj_, init_time, opt_time);
    have_new_target_ = false;

    if (plan_success){
      init_time_list_.push_back(init_time);
      opt_time_list_.push_back(opt_time);
      total_time_list_.push_back(init_time + opt_time);
      sendPolyTrajROSMsg();
      have_local_traj_ = true;

      // vis local traj
      int i_end = floor(planner_manager_->traj_container_.singul_traj_data.duration / 0.02);
      std::vector<Eigen::Vector2d> local_path_list;
      Eigen::Vector2d local_traj_pt;
      for(int i = 0; i < i_end; ++i){
        local_traj_pt = planner_manager_->traj_container_.singul_traj_data.getPos(i * 0.02).head(2);
        local_path_list.push_back(local_traj_pt);
      }
      visualization_->displayGlobalTraj(local_path_list, 0.05, 0);
      planner_manager_->ploy_traj_opt_->displayBackEndMesh(planner_manager_->traj_container_.singul_traj_data, false, gripper_state_);
    }

    return plan_success;
  }

  bool REMANIReplanFSM::callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul){
    std::cout << "\033[31mcall EmergencyStop\033[0m" << std::endl;
    planner_manager_->EmergencyStop(stop_pos, stop_yaw, singul);
    quadrotor_msgs::PolynomialTraj msg;
    msg.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT;
    poly_traj_pub_.publish(msg);

    return true;
  }

  // ################################
  // C++: EE pose goal FSM helpers begin
  // ################################
  void REMANIReplanFSM::clearPendingEeGoal(){
    pending_ee_goal_ = false;
  }

  void REMANIReplanFSM::clearActiveEeGoal(){
    active_ee_goal_ = false;
    active_goal_source_ = GoalSource::NONE;
    deleteEeTerminalGhost();
  }

  void REMANIReplanFSM::clearEeGoalState(){
    clearPendingEeGoal();
    clearActiveEeGoal();
  }

  void REMANIReplanFSM::publishEeTerminalGhost(const Eigen::Vector3d &car,
                                               const Eigen::VectorXd &q){
    visualization_msgs::MarkerArray marker_array;
    planner_manager_->mm_config_->getMMMarkerArray(
        marker_array, "ee_ik_terminal", 0, 0.5, car, q, gripper_state_);
    ee_ik_terminal_marker_pub_.publish(marker_array);
  }

  void REMANIReplanFSM::deleteEeTerminalGhost(){
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);
    ee_ik_terminal_marker_pub_.publish(arr);
  }

  void REMANIReplanFSM::maybePublishEeCurrentPose(){
    if(!(have_odom_ && have_joint_state_)){
      return;
    }
    const ros::Time now = ros::Time::now();
    if((now - last_ee_current_pose_pub_).toSec() < 0.05){
      return;  // ≤20 Hz
    }
    last_ee_current_pose_pub_ = now;

    Eigen::Vector3d car(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
    Eigen::VectorXd q = mm_state_pos_.tail(manipulator_dim_);
    const Eigen::Matrix4d T =
        planner_manager_->mm_config_->getEePose(car, q);

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = "world";
    msg.pose.position.x = T(0, 3);
    msg.pose.position.y = T(1, 3);
    msg.pose.position.z = T(2, 3);
    const Eigen::Quaterniond quat(T.block<3, 3>(0, 0));
    msg.pose.orientation.w = quat.w();
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    ee_current_pose_pub_.publish(msg);
  }

  void REMANIReplanFSM::eeGoalCallback(
      const geometry_msgs::PoseStamped::ConstPtr &msg){
    if(exec_state_ != WAIT_TARGET || have_target_){
      ROS_WARN("[EE GOAL] ignored: planner is not idle");
      return;
    }
    if(!(have_odom_ && have_joint_state_)){
      ROS_WARN("[EE GOAL] robot state not ready");
      return;
    }
    if(msg->header.frame_id != "world" && !msg->header.frame_id.empty()){
      ROS_WARN("[EE GOAL] rejected: frame_id must be world (got '%s')",
               msg->header.frame_id.c_str());
      return;
    }
    if(planner_manager_->mm_config_->getManipulatorType()
       != MMConfig::ManipulatorType::CR10){
      ROS_WARN("[EE GOAL] rejected: EE goal requires CR10");
      return;
    }
    if(target_type_ == TARGET_TYPE::PRESET_TARGET){
      ROS_WARN("[EE GOAL] rejected: PRESET mode");
      return;
    }
    if(!whole_body_ik_){
      ROS_ERROR("[EE GOAL] WholeBodyIkSolver not initialized");
      return;
    }

    Eigen::Quaterniond q_goal;
    if(!sanitizePoseQuaternion(msg->pose.orientation, q_goal)){
      ROS_WARN("[EE GOAL] invalid quaternion");
      return;
    }
    Eigen::Matrix4d T_goal = Eigen::Matrix4d::Identity();
    T_goal.block<3, 3>(0, 0) = q_goal.toRotationMatrix();
    T_goal(0, 3) = msg->pose.position.x;
    T_goal(1, 3) = msg->pose.position.y;
    T_goal(2, 3) = msg->pose.position.z;

    pending_ee_goal_ = true;
    T_world_ee_pending_ = T_goal;

    Eigen::Matrix<double, 9, 1> xi_start;
    xi_start << mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_,
                mm_state_pos_.tail(manipulator_dim_);

    const WholeBodyIkResult result =
        whole_body_ik_->solve(xi_start, T_world_ee_pending_);
    if(!result.success || result.best.q.size() != 6){
      ROS_WARN("[EE GOAL] IK failed: %s", result.fail_reason.c_str());
      clearPendingEeGoal();
      deleteEeTerminalGhost();
      return;
    }

    Eigen::VectorXd candidate_end_pt = Eigen::VectorXd::Zero(traj_dim_);
    candidate_end_pt(0) = result.xi_best(0);
    candidate_end_pt(1) = result.xi_best(1);
    candidate_end_pt.tail(manipulator_dim_) = result.xi_best.tail(6);
    const double candidate_end_yaw = result.xi_best(2);

    planner_manager_->resetFailureCount();
    const bool ok = planNextWaypoint(candidate_end_pt, candidate_end_yaw);
    if(!ok){
      ROS_ERROR("[EE GOAL] terminal found but global trajectory generation failed");
      clearPendingEeGoal();
      return;
    }

    active_goal_source_ = GoalSource::EE_POSE;
    active_ee_goal_ = true;
    T_world_ee_goal_ = T_world_ee_pending_;
    pending_ee_goal_ = false;
    publishEeTerminalGhost(result.best.base_xyyaw, result.best.q);
    ROS_INFO("[EE GOAL] committed EE_POSE terminal (Δxy=%.3f Δyaw=%.1fdeg)",
             result.best.base_xy_disp,
             result.best.yaw_disp * 180.0 / M_PI);
  }
  // ################################
  // C++: EE pose goal FSM helpers end
  // ################################

} // namespace remani_planner
