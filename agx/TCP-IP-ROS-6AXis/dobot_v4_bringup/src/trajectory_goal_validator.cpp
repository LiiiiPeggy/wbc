#include <dobot_v4_bringup/trajectory_goal_validator.hpp>

#include <cmath>
#include <set>
#include <unordered_map>

namespace dobot_v4_bringup {
namespace {

const std::array<std::string, 6> kCanonicalNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

GoalValidationResult reject(const char* code, const char* detail) {
  GoalValidationResult out;
  out.error_code = code;
  out.detail = detail;
  return out;
}

bool allFinite(const std::vector<double>& values) {
  for (double v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

}  // namespace

// ################################
// C++: TrajectoryGoalValidator::validate begin
// ################################
GoalValidationResult TrajectoryGoalValidator::validate(
    const trajectory_msgs::JointTrajectory& input) {
  if (input.points.size() < 2) {
    return reject("POINT_COUNT", "trajectory requires at least two points");
  }
  if (input.joint_names.size() != 6) {
    return reject("NAME_COUNT", "joint_names must contain exactly six entries");
  }

  std::set<std::string> unique_names(input.joint_names.begin(),
                                    input.joint_names.end());
  if (unique_names.size() != 6) {
    return reject("NAME_DUPLICATE", "joint_names contain duplicates");
  }
  for (const std::string& required : kCanonicalNames) {
    if (!unique_names.count(required)) {
      return reject("NAME_SET", "joint_names must be joint1..joint6");
    }
  }

  std::unordered_map<std::string, std::size_t> input_index;
  for (std::size_t i = 0; i < input.joint_names.size(); ++i) {
    input_index[input.joint_names[i]] = i;
  }

  CanonicalTrajectory canonical;
  canonical.joint_names = kCanonicalNames;
  canonical.points.reserve(input.points.size());

  double previous_time = -1.0;
  for (std::size_t p = 0; p < input.points.size(); ++p) {
    const trajectory_msgs::JointTrajectoryPoint& point = input.points[p];
    if (point.positions.size() != 6) {
      return reject("POSITION_SIZE", "each point needs six positions");
    }
    if (point.velocities.size() != 6) {
      return reject("VELOCITY_SIZE", "each point needs six velocities");
    }
    if (!(point.accelerations.empty() || point.accelerations.size() == 6)) {
      return reject("ACCEL_SIZE", "accelerations must be empty or size six");
    }
    if (!allFinite(point.positions)) {
      return reject("POSITION_NONFINITE", "positions must be finite");
    }
    if (!allFinite(point.velocities)) {
      return reject("VELOCITY_NONFINITE", "velocities must be finite");
    }
    if (!point.accelerations.empty() && !allFinite(point.accelerations)) {
      return reject("ACCEL_NONFINITE", "accelerations must be finite");
    }

    const double t = point.time_from_start.toSec();
    if (!std::isfinite(t) || t < 0.0) {
      return reject("TIME_INVALID", "time_from_start must be finite and >= 0");
    }
    if (!(t > previous_time)) {
      return reject("TIME_NOT_STRICT", "time_from_start must strictly increase");
    }
    previous_time = t;

    CanonicalTrajectoryPoint out_point;
    out_point.time_from_start = t;
    out_point.has_accelerations = !point.accelerations.empty();
    for (std::size_t j = 0; j < 6; ++j) {
      const std::size_t src = input_index[kCanonicalNames[j]];
      out_point.positions[j] = point.positions[src];
      out_point.velocities[j] = point.velocities[src];
      if (out_point.has_accelerations) {
        out_point.accelerations[j] = point.accelerations[src];
      }
    }
    canonical.points.push_back(out_point);
  }

  GoalValidationResult ok;
  ok.valid = true;
  ok.trajectory = canonical;
  return ok;
}
// ################################
// C++: TrajectoryGoalValidator::validate end
// ################################

}  // namespace dobot_v4_bringup
