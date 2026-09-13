#include <cmath>
#include <memory>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <remani_real_msgs/Cr10Status.h>
#include <remani_real_msgs/ExecuteCandidate.h>
#include <remani_real_msgs/ExecutionState.h>
#include <remani_real_msgs/PlannerStatus.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>

#include <mm_config/mm_config.hpp>
#include <plan_env/grid_map.h>

#include <remani_real/actual_state.hpp>
#include <remani_real/base_tracking_controller.hpp>
#include <remani_real/candidate_assembler.hpp>
#include <remani_real/candidate_validator.hpp>
#include <remani_real/deployment_state_machine.hpp>
#include <remani_real/dry_run_motion_output.hpp>
#include <remani_real/mm_config_validation_environment.hpp>
#include <remani_real/preview_publisher.hpp>

namespace remani_real {
namespace {

// ################################
// C++: remani_real_node dry-run control plane begin
// ################################
using FollowJointTrajectoryClient =
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

double loadAssemblyTimeoutSec() {
  // ################################
  // C++: private assembly_timeout_sec for rostest begin
  // ################################
  double timeout_sec = 60.0;
  ros::NodeHandle private_nh("~");
  private_nh.param("assembly_timeout_sec", timeout_sec, 60.0);
  return timeout_sec;
  // ################################
  // C++: private assembly_timeout_sec for rostest end
  // ################################
}

class RemaniRealNode {
 public:
  RemaniRealNode()
      : private_nh_("~"),
        assembler_(loadAssemblyTimeoutSec()),
        action_client_("/cr10_robot/joint_controller/follow_joint_trajectory",
                       true) {
    std::string mode;
    std::string owner;
    std::string environment_mode;
    bool dry_run = false;
    nh_.param<std::string>("mode", mode, "sim");
    nh_.param<std::string>("execution_owner", owner, "internal");
    nh_.param<std::string>("environment_mode", environment_mode, "simulated");
    nh_.param("dry_run", dry_run, false);
    if (mode != "real" || owner != "external" ||
        environment_mode != "static_empty" || !dry_run) {
      throw std::runtime_error(
          "remani_real_node Phase-2 requires mode=real, "
          "execution_owner=external, environment_mode=static_empty, dry_run=true");
    }

    configureEnvironment();
    ValidationLimits limits;
    validator_.reset(new CandidateValidator(limits, environment_.get()));
    preview_.reset(new PreviewPublisher(nh_));
    // ################################
    // C++: load base tracking config begin
    // ################################
    BaseTrackingConfig tracking_config;
    nh_.param("base_tracking/k_x", tracking_config.k_x, 1.0);
    nh_.param("base_tracking/k_y", tracking_config.k_y, 1.0);
    nh_.param("base_tracking/k_yaw", tracking_config.k_yaw, 1.0);
    nh_.param("base_tracking/max_linear", tracking_config.max_linear, 0.10);
    nh_.param("base_tracking/max_angular", tracking_config.max_angular, 0.15);
    nh_.param("base_tracking/max_linear_correction",
              tracking_config.max_linear_correction, 0.03);
    nh_.param("base_tracking/max_angular_correction",
              tracking_config.max_angular_correction, 0.05);
    nh_.param("base_tracking/max_position_error",
              tracking_config.max_position_error, 0.20);
    nh_.param("base_tracking/max_yaw_error", tracking_config.max_yaw_error,
              0.20);
    base_tracker_.reset(new BaseTrackingController(tracking_config));
    // ################################
    // C++: load base tracking config end
    // ################################

    state_pub_ = nh_.advertise<remani_real_msgs::ExecutionState>(
        "/remani/execution_state", 1, true);
    dry_preview_pub_ = nh_.advertise<geometry_msgs::Twist>(
        "/remani/dry_run/ranger_cmd_vel_preview", 1, false);

    plan_sub_ = nh_.subscribe("/ee_goal_plan", 1, &RemaniRealNode::onPlanSignal,
                              this);
    candidate_sub_ = nh_.subscribe("/remani/planner_candidate", 10,
                                   &RemaniRealNode::onPlannerCandidate, this);
    planner_status_sub_ = nh_.subscribe("/remani/planner_status", 1,
                                        &RemaniRealNode::onPlannerStatus, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &RemaniRealNode::onOdom, this);
    joint_sub_ = nh_.subscribe("/remani/cr10_joint_states", 10,
                               &RemaniRealNode::onJoints, this);
    status_sub_ = nh_.subscribe("/remani/cr10_status", 10,
                                &RemaniRealNode::onStatus, this);
    watchdog_sub_ = nh_.subscribe("/remani/ranger_watchdog_ready", 1,
                                  &RemaniRealNode::onWatchdog, this);
    watchdog_timeout_sub_ = nh_.subscribe(
        "/remani/ranger_watchdog_timed_out", 1, &RemaniRealNode::onWatchdogTimeout,
        this);

    execute_srv_ = nh_.advertiseService("/remani/execute",
                                        &RemaniRealNode::onExecute, this);
    pause_srv_ =
        nh_.advertiseService("/remani/pause", &RemaniRealNode::onPause, this);
    resume_srv_ =
        nh_.advertiseService("/remani/resume", &RemaniRealNode::onResume, this);
    abort_srv_ =
        nh_.advertiseService("/remani/abort", &RemaniRealNode::onAbort, this);

    timer_ = nh_.createTimer(ros::Duration(0.05), &RemaniRealNode::onTimer, this);
    ROS_WARN("remani_real_node dry-run control plane online");
  }

 private:
  void configureEnvironment() {
    ros::NodeHandle mm_nh(nh_, "remani_real_validation");
    mm_nh.setParam("mm/manipulator_type", std::string("cr10"));
    mm_nh.setParam("mm/mobile_base_dof", 2);
    mm_nh.setParam("mm/mobile_base_length", 1.10);
    mm_nh.setParam("mm/mobile_base_width", 0.90);
    mm_nh.setParam("mm/mobile_base_height", 0.40);
    mm_nh.setParam("mm/mobile_base_check_radius", 0.20);
    mm_nh.setParam("mm/mobile_base_wheel_base", 0.56);
    mm_nh.setParam("mm/mobile_base_wheel_radius", 0.125);
    mm_nh.setParam("mm/mobile_base_max_wheel_omega", 4.0);
    mm_nh.setParam("mm/mobile_base_max_wheel_alpha", 8.0);
    mm_nh.setParam("mm/manipulator_dof", 6);
    mm_nh.setParam("mm/manipulator_thickness", 0.06);
    mm_nh.setParam("mm/manipulator_config",
                   std::vector<double>{0.1765, 0.607, 0.568, 0.191, 0.125, 0.1084});
    mm_nh.setParam("mm/manipulator_min_pos", std::vector<double>(6, -3.0));
    mm_nh.setParam("mm/manipulator_max_pos", std::vector<double>(6, 3.0));
    mm_nh.setParam("mm/base_mani_fixed_joint_xyz_ypr",
                   std::vector<double>{0.2462, 0.0, 0.1, 0.0, 0.0, 0.0});
    mm_nh.setParam("mm/ee_tcp_xyz_rpy", std::vector<double>(6, 0.0));
    mm_nh.setParam("optimization/safe_margin", 0.05);
    mm_nh.setParam("optimization/safe_margin_mani", 0.05);
    mm_nh.setParam("optimization/self_safe_margin", 0.02);
    mm_nh.setParam("optimization/ground_safe_dis", 0.1);
    mm_nh.setParam("grid_map/resolution", 0.05);
    mm_nh.setParam("environment_mode", "static_empty");
    mm_nh.setParam("environment/static_empty/resolution", 0.05);
    mm_nh.setParam("environment/static_empty/size_x", 16.0);
    mm_nh.setParam("environment/static_empty/size_y", 12.0);
    mm_nh.setParam("environment/static_empty/size_z", 3.0);
    mm_nh.setParam("grid_map/ground_height", 0.0);

    grid_map_.reset(new GridMap());
    grid_map_->initMap(mm_nh);
    mm_config_.reset(new remani_planner::MMConfig());
    mm_config_->setParam(mm_nh, grid_map_);
    environment_.reset(
        new MmConfigValidationEnvironment(grid_map_, mm_config_));
  }

  ReadinessSnapshot readinessSnapshot() const {
    ReadinessSnapshot ready;
    ready.odom = have_odom_;
    ready.cr10_joints = have_joints_;
    ready.cr10_velocity = true;  // Phase-2 dry-run: do not block on velocity LPF.
    ready.tf = have_odom_;
    ready.robot_status = have_status_ && status_ok_;
    ready.action_server = action_client_.isServerConnected();
    ready.grid_map = grid_map_ && grid_map_->isReady();
    ready.ranger_watchdog = watchdog_ready_ && !watchdog_timed_out_;
    return ready;
  }

  void publishState() {
    remani_real_msgs::ExecutionState msg;
    msg.header.stamp = ros::Time::now();
    msg.mode = remani_real_msgs::ExecutionState::MODE_REAL;
    msg.execution_owner = remani_real_msgs::ExecutionState::OWNER_EXTERNAL;
    msg.dry_run = true;
    msg.environment_mode = remani_real_msgs::ExecutionState::ENV_STATIC_EMPTY;
    msg.planner_state = planner_state_;

    switch (fsm_.state()) {
      case State::NotReady:
      case State::Ready:
      case State::Planning:
        msg.executor_state = remani_real_msgs::ExecutionState::EXECUTOR_NONE;
        break;
      case State::Planned:
        msg.executor_state = remani_real_msgs::ExecutionState::EXECUTOR_PLANNED;
        break;
      case State::Executing:
        msg.executor_state =
            remani_real_msgs::ExecutionState::EXECUTOR_EXECUTING;
        break;
      case State::Paused:
        msg.executor_state = remani_real_msgs::ExecutionState::EXECUTOR_PAUSED;
        break;
      case State::Succeeded:
        msg.executor_state =
            remani_real_msgs::ExecutionState::EXECUTOR_SUCCEEDED;
        break;
      case State::Error:
        msg.executor_state = remani_real_msgs::ExecutionState::EXECUTOR_ERROR;
        break;
    }

    if (fsm_.state() == State::Planning) {
      msg.transaction_state =
          remani_real_msgs::ExecutionState::TRANSACTION_ASSEMBLING;
    } else if (frozen_ && report_.valid) {
      msg.transaction_state =
          remani_real_msgs::ExecutionState::TRANSACTION_COMPLETE;
    } else {
      msg.transaction_state =
          remani_real_msgs::ExecutionState::TRANSACTION_IDLE;
    }

    msg.candidate_id = fsm_.plannedCandidateId();
    if (frozen_) {
      msg.raw_transaction_stamp = frozen_->rawTransactionStamp();
      msg.candidate_complete = true;
      msg.candidate_valid = report_.valid;
      msg.candidate_duration = report_.duration;
      msg.candidate_max_base_linear_speed = report_.max_base_linear_speed;
      msg.candidate_max_base_angular_speed = report_.max_base_angular_speed;
      msg.candidate_max_joint_speed = report_.max_joint_speed;
    }
    const ReadinessSnapshot ready = readinessSnapshot();
    msg.odom_ready = ready.odom;
    msg.cr10_joint_ready = ready.cr10_joints;
    msg.cr10_velocity_valid = ready.cr10_velocity;
    msg.tf_ready = ready.tf;
    msg.robot_status_ready = ready.robot_status;
    msg.robot_connected = status_connected_;
    msg.robot_enabled = status_enabled_;
    msg.robot_error_status = status_error_;
    msg.robot_mode = status_mode_;
    msg.action_server_ready = ready.action_server;
    msg.grid_map_ready = ready.grid_map;
    msg.ranger_watchdog_ready = ready.ranger_watchdog;
    msg.ranger_watchdog_timed_out = watchdog_timed_out_;
    msg.execution_progress = execution_progress_;
    msg.pause_param_time = pause_param_time_;
    // ################################
    // C++: prefer FSM fault code over stale validation report begin
    // ################################
    if (!fsm_.lastErrorCode().empty()) {
      msg.last_error_code = fsm_.lastErrorCode();
      msg.last_error = fsm_.lastErrorCode();
    } else if (!report_.valid) {
      msg.last_error_code = report_.error_code;
      msg.last_error = report_.detail;
    } else {
      msg.last_error_code.clear();
      msg.last_error.clear();
    }
    // ################################
    // C++: prefer FSM fault code over stale validation report end
    // ################################
    state_pub_.publish(msg);
  }

  void onPlanSignal(const std_msgs::Empty::ConstPtr&) {
    fsm_.updateReadiness(readinessSnapshot());
    const CommandResult result = fsm_.requestPlan();
    if (!result.accepted) {
      ROS_WARN("Plan rejected: %s (%s)", result.error_code.c_str(),
               result.detail.c_str());
    } else {
      // ################################
      // C++: capture plan session and isolate prior assembler begin
      // ################################
      active_plan_session_ = fsm_.planSessionId();
      assembler_.invalidate("NEW_PLAN");
      // ################################
      // C++: capture plan session and isolate prior assembler end
      // ################################
    }
    publishState();
  }

  void onPlannerStatus(const remani_real_msgs::PlannerStatus::ConstPtr& msg) {
    planner_state_ = msg->state;
  }

  static bool isProtocolCorruption(const std::string& error_code) {
    // ################################
    // C++: protocol-corruption codes for Planning -> Error begin
    // ################################
    return error_code == "SEGMENT_SEQUENCE" ||
           error_code == "UNKNOWN_ACTION" ||
           error_code == "BAD_CONTROL" ||
           error_code == "ASSEMBLY_TIMEOUT" ||
           error_code == "INVALID_FINAL" ||
           error_code == "INVALID_PIECE" ||
           error_code == "INVALID_ABORT" ||
           error_code == "INVALID_IMPOSSIBLE" ||
           error_code == "INVALID_START" ||
           error_code == "INVALID_START_YAW" ||
           error_code == "EMPTY_ADD" ||
           error_code == "INVALID_SINGUL" ||
           error_code == "FINAL_WITHOUT_ADD" ||
           error_code == "CANDIDATE_CONSTRUCT" ||
           error_code == "DURATION_OVERFLOW";
    // ################################
    // C++: protocol-corruption codes for Planning -> Error end
    // ################################
  }

  void applyPlanningFailure(const AssemblyEvent& event) {
    if (event.error_code.empty()) {
      return;
    }
    if (event.state != AssemblyState::Invalid && event.accepted) {
      return;
    }
    // ################################
    // C++: ignore idle/stale rejects after invalidate begin
    // ################################
    // After Plan/Abort invalidate, late ADD/FINAL must not kill the new session.
    if (event.error_code == "ADD_WITHOUT_START" ||
        event.error_code == "FINAL_WITHOUT_START" ||
        event.error_code == "START_NOT_ALLOWED") {
      return;
    }
    // ################################
    // C++: ignore idle/stale rejects after invalidate end
    // ################################
    if (fsm_.state() != State::Planning) {
      return;
    }
    fsm_.onPlanningFailure(event.error_code,
                           isProtocolCorruption(event.error_code),
                           active_plan_session_);
  }

  void onPlannerCandidate(const quadrotor_msgs::PolynomialTraj::ConstPtr& msg) {
    fsm_.updateReadiness(readinessSnapshot());
    const bool allow = fsm_.newTransactionAllowed();
    const AssemblyEvent event = assembler_.consume(
        *msg, ros::SteadyTime::now(), allow, actual_.base_yaw);
    // ################################
    // C++: planning failure uses captured plan session begin
    // ################################
    applyPlanningFailure(event);
    // ################################
    // C++: planning failure uses captured plan session end
    // ################################
    if (event.accepted &&
        event.state == AssemblyState::Complete) {
      frozen_ = assembler_.completedCandidate();
      report_ = validator_->validate(frozen_, actual_);
      preview_->publish(frozen_, report_, environment_.get());
      fsm_.onCandidateValidated(frozen_->id(), report_.valid,
                                active_plan_session_);
      if (!report_.valid) {
        ROS_WARN("Candidate validation failed: %s", report_.error_code.c_str());
      }
    }
    publishState();
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    actual_.base_xy.x() = msg->pose.pose.position.x;
    actual_.base_xy.y() = msg->pose.pose.position.y;
    const double z = msg->pose.pose.orientation.z;
    const double w = msg->pose.pose.orientation.w;
    actual_.base_yaw = std::atan2(2.0 * w * z, 1.0 - 2.0 * z * z);
    actual_.odom_valid = true;
    have_odom_ = true;
  }

  void onJoints(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->position.size() < 6) {
      return;
    }
    for (int i = 0; i < 6; ++i) {
      actual_.q(i) = msg->position[i];
    }
    actual_.joints_valid = true;
    have_joints_ = true;
  }

  void onStatus(const remani_real_msgs::Cr10Status::ConstPtr& msg) {
    have_status_ = true;
    status_connected_ = msg->connected;
    status_enabled_ = msg->enabled;
    status_error_ = msg->error_status;
    status_mode_ = msg->robot_mode;
    // ################################
    // C++: formal status requires connected+enabled+no-error begin
    // ################################
    status_ok_ =
        msg->connected && msg->enabled && msg->error_status == 0;
    // ################################
    // C++: formal status requires connected+enabled+no-error end
    // ################################
  }

  void onWatchdog(const std_msgs::Bool::ConstPtr& msg) {
    watchdog_ready_ = msg->data;
  }

  void onWatchdogTimeout(const std_msgs::Bool::ConstPtr& msg) {
    watchdog_timed_out_ = msg->data;
  }

  bool onExecute(remani_real_msgs::ExecuteCandidate::Request& req,
                 remani_real_msgs::ExecuteCandidate::Response& res) {
    fsm_.updateReadiness(readinessSnapshot());
    const CommandResult result = fsm_.requestExecute(req.candidate_id);
    res.accepted = result.accepted;
    res.message = result.accepted ? "accepted" : result.error_code;
    if (result.accepted) {
      execute_start_ = ros::SteadyTime::now();
      pause_param_time_ = 0.0;
      paused_ = false;
      execution_progress_ = 0.0;
    }
    publishState();
    return true;
  }

  bool onPause(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    const CommandResult result = fsm_.requestPause();
    res.success = result.accepted;
    res.message = result.accepted ? "pause requested" : result.error_code;
    if (result.accepted) {
      pause_param_time_ =
          (ros::SteadyTime::now() - execute_start_).toSec();
      fsm_.onPauseConfirmed();
      paused_ = true;
    }
    publishState();
    return true;
  }

  bool onResume(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    const CommandResult result = fsm_.requestResume(true);
    res.success = result.accepted;
    res.message = result.accepted ? "resumed" : result.error_code;
    if (result.accepted) {
      // Continue the frozen pause parameter without wall-clock rebaseline tricks.
      execute_start_ = ros::SteadyTime::now();
      execute_start_.fromSec(execute_start_.toSec() - pause_param_time_);
      paused_ = false;
    }
    publishState();
    return true;
  }

  bool onAbort(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    fsm_.updateReadiness(readinessSnapshot());
    const CommandResult result = fsm_.requestAbort();
    res.success = result.accepted;
    res.message = result.accepted ? "aborted" : result.error_code;
    if (result.accepted) {
      // ################################
      // C++: Abort isolates assembler from late FINAL begin
      // ################################
      assembler_.invalidate("ABORT");
      // ################################
      // C++: Abort isolates assembler from late FINAL end
      // ################################
      paused_ = false;
      execution_progress_ = 0.0;
      dry_output_.stop();
    }
    publishState();
    return true;
  }

  void onTimer(const ros::TimerEvent&) {
    fsm_.updateReadiness(readinessSnapshot());
    // ################################
    // C++: poll assembler timeout while Planning begin
    // ################################
    if (fsm_.state() == State::Planning) {
      applyPlanningFailure(assembler_.pollTimeout(ros::SteadyTime::now()));
    }
    // ################################
    // C++: poll assembler timeout while Planning end
    // ################################
    if (fsm_.state() == State::Executing && frozen_ && !paused_) {
      const double t =
          std::max(0.0, (ros::SteadyTime::now() - execute_start_).toSec());
      const double duration = frozen_->duration();
      const double sample_t = std::min(t, duration);
      execution_progress_ = duration > 0.0 ? sample_t / duration : 1.0;
      try {
        const WholeBodySample sample = frozen_->sample(sample_t);
        // ################################
        // C++: dry-run uses base tracker for command shaping begin
        // ################################
        const BaseTrackingResult tracked =
            base_tracker_->compute(sample, actual_);
        geometry_msgs::Twist twist;
        if (tracked.valid) {
          twist = tracked.command;
        } else {
          // Keep diagnostics zeroed and surface fault without hardware path.
          fsm_.onExecutionFault(tracked.error_code.empty()
                                    ? "BASE_TRACKING_ERROR"
                                    : tracked.error_code);
        }
        dry_output_.publish(twist);
        dry_preview_pub_.publish(twist);
        // ################################
        // C++: dry-run uses base tracker for command shaping end
        // ################################
      } catch (const std::exception& ex) {
        fsm_.onExecutionFault(ex.what());
      }
      if (t >= duration) {
        fsm_.onExecutionSucceeded();
      }
    }
    publishState();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  DeploymentStateMachine fsm_;
  CandidateAssembler assembler_;
  std::unique_ptr<CandidateValidator> validator_;
  std::unique_ptr<PreviewPublisher> preview_;
  std::unique_ptr<MmConfigValidationEnvironment> environment_;
  std::unique_ptr<BaseTrackingController> base_tracker_;
  std::shared_ptr<GridMap> grid_map_;
  std::shared_ptr<remani_planner::MMConfig> mm_config_;
  DryRunMotionOutput dry_output_;
  FrozenCandidate frozen_;
  ValidationReport report_;
  ActualStateSnapshot actual_;

  FollowJointTrajectoryClient action_client_;
  ros::Publisher state_pub_;
  ros::Publisher dry_preview_pub_;
  ros::Subscriber plan_sub_;
  ros::Subscriber candidate_sub_;
  ros::Subscriber planner_status_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber joint_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber watchdog_sub_;
  ros::Subscriber watchdog_timeout_sub_;
  ros::ServiceServer execute_srv_;
  ros::ServiceServer pause_srv_;
  ros::ServiceServer resume_srv_;
  ros::ServiceServer abort_srv_;
  ros::Timer timer_;

  uint8_t planner_state_{remani_real_msgs::PlannerStatus::IDLE};
  // ################################
  // C++: captured plan session for Gate callbacks begin
  // ################################
  uint64_t active_plan_session_{0};
  // ################################
  // C++: captured plan session for Gate callbacks end
  // ################################
  bool have_odom_{false};
  bool have_joints_{false};
  bool have_status_{false};
  bool status_ok_{false};
  bool status_connected_{false};
  bool status_enabled_{false};
  int8_t status_error_{0};
  uint16_t status_mode_{0};
  bool watchdog_ready_{false};
  bool watchdog_timed_out_{false};
  bool paused_{false};
  double execution_progress_{0.0};
  double pause_param_time_{0.0};
  ros::SteadyTime execute_start_;
};
// ################################
// C++: remani_real_node dry-run control plane end
// ################################

}  // namespace
}  // namespace remani_real

int main(int argc, char** argv) {
  ros::init(argc, argv, "remani_real_node");
  try {
    remani_real::RemaniRealNode node;
    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL("%s", ex.what());
    return 1;
  }
  return 0;
}
