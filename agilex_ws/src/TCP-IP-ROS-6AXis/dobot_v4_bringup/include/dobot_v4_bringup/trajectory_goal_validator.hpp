#pragma once

#include <array>
#include <string>
#include <vector>

#include <trajectory_msgs/JointTrajectory.h>

namespace dobot_v4_bringup {

// ################################
// C++: CR10 trajectory goal validation types begin
// ################################
struct CanonicalTrajectoryPoint {
  std::array<double, 6> positions{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> velocities{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> accelerations{{0, 0, 0, 0, 0, 0}};
  bool has_accelerations{false};
  double time_from_start{0.0};
};

struct CanonicalTrajectory {
  std::array<std::string, 6> joint_names{{"joint1", "joint2", "joint3",
                                          "joint4", "joint5", "joint6"}};
  std::vector<CanonicalTrajectoryPoint> points;
};

struct GoalValidationResult {
  bool valid{false};
  std::string error_code;
  std::string detail;
  CanonicalTrajectory trajectory;
};

class TrajectoryGoalValidator {
 public:
  static GoalValidationResult validate(
      const trajectory_msgs::JointTrajectory& input);
};
// ################################
// C++: CR10 trajectory goal validation types end
// ################################

}  // namespace dobot_v4_bringup
