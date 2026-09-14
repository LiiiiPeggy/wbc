#pragma once

#include <string>
#include <vector>

#include <remani_real/motion_output.hpp>

namespace remani_real {

// ################################
// C++: RuntimeSafetyMonitor types begin
// ################################
struct RuntimeSafetyConfig {
  bool dry_run{true};
  std::string expected_ranger_publisher;
  double odom_timeout{0.30};
  double joint_timeout{0.30};
  double tf_timeout{0.30};
  double max_base_linear_speed{0.10};
  double max_base_angular_speed{0.15};
  double max_joint_speed{0.10};
  double max_base_tracking_error{0.20};
  double max_joint_tracking_error{0.10};
};

struct RuntimeSafetyInput {
  double odom_age{0.0};
  double joint_age{0.0};
  double tf_age{0.0};
  bool joint_velocity_valid{false};
  bool robot_connected{false};
  bool robot_enabled{false};
  bool robot_fault{false};
  ArmGoalState action_state{ArmGoalState::Unavailable};
  bool ranger_watchdog_ready{false};
  bool ranger_watchdog_timed_out{false};
  std::vector<std::string> hardware_topic_publishers;
  double base_tracking_error{0.0};
  double joint_tracking_error{0.0};
  double desired_base_linear_speed{0.0};
  double desired_base_angular_speed{0.0};
  double desired_max_joint_speed{0.0};
};

struct SafetyDecision {
  bool safe{false};
  std::string error_code;
  std::string detail;
};

class RuntimeSafetyMonitor {
 public:
  explicit RuntimeSafetyMonitor(RuntimeSafetyConfig config);
  SafetyDecision evaluate(const RuntimeSafetyInput& input) const;

 private:
  static bool isFiniteNonNeg(double value);
  RuntimeSafetyConfig config_;
};
// ################################
// C++: RuntimeSafetyMonitor types end
// ################################

}  // namespace remani_real
