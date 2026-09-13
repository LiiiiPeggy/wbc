#pragma once

#include <string>

#include <Eigen/Core>
#include <trajectory_msgs/JointTrajectory.h>

#include <remani_real/candidate_trajectory.hpp>

namespace remani_real {

// ################################
// C++: ArmTrajectoryBuilder types begin
// ################################
struct ArmTrajectoryBuildResult {
  bool valid{false};
  trajectory_msgs::JointTrajectory trajectory;
  std::string error_code;
  std::string detail;
};

class ArmTrajectoryBuilder {
 public:
  // current_q: six CR10 joints (rad). sample_period and hold_tol are seconds/radians.
  static ArmTrajectoryBuildResult build(const FrozenCandidate& candidate,
                                        const Eigen::Matrix<double, 6, 1>& current_q,
                                        double start_lead_time,
                                        double sample_period,
                                        double hold_tol);
};
// ################################
// C++: ArmTrajectoryBuilder types end
// ################################

}  // namespace remani_real
