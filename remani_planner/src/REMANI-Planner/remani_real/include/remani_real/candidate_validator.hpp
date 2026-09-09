#pragma once

#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Core>

#include <remani_real/actual_state.hpp>
#include <remani_real/candidate_trajectory.hpp>

namespace remani_real {

// ################################
// C++: CandidateValidator types begin
// ################################
struct ValidationReport {
  bool valid{false};
  std::string error_code;
  std::string detail;
  double duration{0.0};
  double max_base_linear_speed{0.0};
  double max_base_angular_speed{0.0};
  double max_joint_speed{0.0};
  Eigen::Vector2d final_base_xy{Eigen::Vector2d::Zero()};
  double final_base_yaw{0.0};
  Eigen::Matrix<double, 6, 1> final_q{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix4d expected_final_ee{Eigen::Matrix4d::Identity()};
};

struct ValidationLimits {
  double validation_dt{0.01};
  double start_base_xy_tol{0.05};
  double start_base_yaw_tol{5.0 * 3.14159265358979323846 / 180.0};
  double start_joint_tol{3.0 * 3.14159265358979323846 / 180.0};
  double max_base_linear_speed{0.10};
  double max_base_angular_speed{0.15};
  double max_joint_speed{0.10};
  double continuity_pos_tol{1e-4};
  double continuity_vel_tol{1e-3};
  double continuity_acc_tol{1e-2};
};

/* ---------- Map / collision / FK environment used by Gate validation. ---------- */
class ValidationEnvironment {
 public:
  virtual ~ValidationEnvironment() = default;
  virtual bool samplesInMap(const Eigen::Vector3d& car_state,
                            const Eigen::VectorXd& q) const = 0;
  virtual bool inCollision(const Eigen::Vector3d& car_state,
                           const Eigen::VectorXd& q,
                           int* coll_type) const = 0;
  virtual Eigen::Matrix4d eePose(const Eigen::Vector3d& car_state,
                                 const Eigen::VectorXd& q) const = 0;
};

class CandidateValidator {
 public:
  CandidateValidator(ValidationLimits limits,
                     const ValidationEnvironment* environment);

  ValidationReport validate(const FrozenCandidate& candidate,
                            const ActualStateSnapshot& actual) const;

 private:
  ValidationLimits limits_;
  const ValidationEnvironment* environment_;
};
// ################################
// C++: CandidateValidator types end
// ################################

}  // namespace remani_real
