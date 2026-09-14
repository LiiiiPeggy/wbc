#pragma once

#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <remani_real/actual_state.hpp>

namespace remani_planner {
class MMConfig;
}

namespace remani_real {

// ################################
// C++: CompletionVerifier types begin
// ################################
struct CompletionThresholds {
  double base_position{0.05};
  double base_yaw{5.0 * M_PI / 180.0};
  double base_linear_velocity{0.01};
  double base_yaw_rate{0.02};
  double arm_joint{0.02};
  double arm_velocity{0.01};
  double ee_position{0.02};
  double ee_rotation{4.0 * M_PI / 180.0};
};

class EeKinematics {
 public:
  virtual ~EeKinematics() = default;
  virtual Eigen::Matrix4d pose(const Eigen::Vector3d& car,
                               const Eigen::Matrix<double, 6, 1>& q) const = 0;
};

class MmConfigEeKinematics : public EeKinematics {
 public:
  explicit MmConfigEeKinematics(remani_planner::MMConfig* config);
  Eigen::Matrix4d pose(const Eigen::Vector3d& car,
                       const Eigen::Matrix<double, 6, 1>& q) const override;

 private:
  remani_planner::MMConfig* config_;
};

struct CompletionInput {
  Eigen::Vector2d expected_base_xy{Eigen::Vector2d::Zero()};
  double expected_base_yaw{0.0};
  Eigen::Matrix<double, 6, 1> expected_q{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix4d expected_ee{Eigen::Matrix4d::Identity()};
  ActualStateSnapshot actual;
  bool arm_action_succeeded{false};
  bool feedback_fresh{false};
  bool robot_status_healthy{false};
};

struct CompletionDecision {
  bool succeeded{false};
  double final_base_position_error{0.0};
  double final_base_yaw_error{0.0};
  double final_joint_error{0.0};
  double final_ee_pos_error{0.0};
  double final_ee_rot_error{0.0};
  std::string error_code;
  std::string detail;
};

class CompletionVerifier {
 public:
  CompletionVerifier(CompletionThresholds thresholds,
                     const EeKinematics* kinematics);
  CompletionDecision verify(const CompletionInput& input) const;

 private:
  static double yawAbsError(double a, double b);
  static double rotationAngle(const Eigen::Matrix3d& r_expected,
                              const Eigen::Matrix3d& r_actual);
  CompletionThresholds thresholds_;
  const EeKinematics* kinematics_;
};
// ################################
// C++: CompletionVerifier types end
// ################################

}  // namespace remani_real
