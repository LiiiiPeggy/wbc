#pragma once

#include <cstdint>
#include <string>

#include <remani_real_msgs/ExecutionState.h>

namespace remani_real_rviz {

// ################################
// C++: PanelViewModel pure mapping begin
// ################################
struct PanelViewModel {
  bool plan_enabled{false};
  bool execute_enabled{false};
  bool pause_enabled{false};
  bool resume_enabled{false};
  bool abort_enabled{false};
  uint64_t execute_candidate_id{0};
  std::string environment_warning;
  std::string last_error_code;
  std::string last_error;
  double candidate_duration{0.0};
  double execution_progress{0.0};
  double pause_param_time{0.0};
  double start_skew{0.0};
  bool start_skew_available{false};
  double final_base_error{0.0};
  double final_joint_error{0.0};
  double final_ee_pos_error{0.0};
  double final_ee_rot_error{0.0};
  double ranger_feedback_age{0.0};
  double cr10_joint_feedback_age{0.0};
  double tf_age{0.0};
  double robot_status_age{0.0};

  static PanelViewModel from(const remani_real_msgs::ExecutionState& msg);
};

inline bool readinessComplete(const remani_real_msgs::ExecutionState& msg) {
  return msg.odom_ready && msg.cr10_joint_ready && msg.cr10_velocity_valid &&
         msg.tf_ready && msg.robot_status_ready && msg.action_server_ready &&
         msg.grid_map_ready && msg.ranger_watchdog_ready;
}

inline PanelViewModel PanelViewModel::from(
    const remani_real_msgs::ExecutionState& msg) {
  PanelViewModel view;
  view.execute_candidate_id = msg.candidate_id;
  view.candidate_duration = msg.candidate_duration;
  view.execution_progress = msg.execution_progress;
  view.pause_param_time = msg.pause_param_time;
  view.start_skew = msg.start_skew;
  view.start_skew_available = msg.start_skew_available;
  view.final_base_error = msg.final_base_error;
  view.final_joint_error = msg.final_joint_error;
  view.final_ee_pos_error = msg.final_ee_pos_error;
  view.final_ee_rot_error = msg.final_ee_rot_error;
  view.ranger_feedback_age = msg.ranger_feedback_age;
  view.cr10_joint_feedback_age = msg.cr10_joint_feedback_age;
  view.tf_age = msg.tf_age;
  view.robot_status_age = msg.robot_status_age;
  view.last_error_code = msg.last_error_code;
  view.last_error = msg.last_error;

  if (msg.environment_mode == remani_real_msgs::ExecutionState::ENV_STATIC_EMPTY) {
    view.environment_warning =
        "ENVIRONMENT: STATIC EMPTY / NO ONLINE OBSTACLE SENSING";
  }

  // Button enablement mirrors DeploymentStateMachine permissions, derived only
  // from published ExecutionState fields (no wall-clock inference).
  const bool ready = readinessComplete(msg);
  const uint8_t exec = msg.executor_state;
  const uint8_t planner = msg.planner_state;

  if (!ready) {
    return view;
  }

  if (exec == remani_real_msgs::ExecutionState::EXECUTOR_ERROR) {
    view.abort_enabled = true;
    return view;
  }
  if (exec == remani_real_msgs::ExecutionState::EXECUTOR_EXECUTING) {
    view.pause_enabled = true;
    view.abort_enabled = true;
    return view;
  }
  if (exec == remani_real_msgs::ExecutionState::EXECUTOR_PAUSED) {
    view.resume_enabled = true;
    view.abort_enabled = true;
    return view;
  }
  if (exec == remani_real_msgs::ExecutionState::EXECUTOR_PLANNED) {
    view.plan_enabled = true;
    view.abort_enabled = true;
    if (msg.candidate_complete && msg.candidate_valid) {
      view.execute_enabled = true;
    }
    return view;
  }
  if (exec == remani_real_msgs::ExecutionState::EXECUTOR_SUCCEEDED) {
    view.plan_enabled = true;
    return view;
  }
  if (planner == remani_real_msgs::ExecutionState::PLANNER_PLANNING ||
      planner == remani_real_msgs::ExecutionState::PLANNER_HANDOFF) {
    view.abort_enabled = true;
    return view;
  }

  // READY: executor NONE + planner IDLE + readiness complete.
  view.plan_enabled = true;
  return view;
}
// ################################
// C++: PanelViewModel pure mapping end
// ################################

}  // namespace remani_real_rviz
