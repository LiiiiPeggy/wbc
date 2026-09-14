#include <remani_real/completion_verifier.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <mm_config/mm_config.hpp>

namespace remani_real {

namespace {

CompletionDecision fail(CompletionDecision out, const std::string& code,
                        const std::string& detail) {
  out.succeeded = false;
  out.error_code = code;
  out.detail = detail;
  return out;
}

}  // namespace

// ################################
// C++: CompletionVerifier::verify begin
// ################################
MmConfigEeKinematics::MmConfigEeKinematics(remani_planner::MMConfig* config)
    : config_(config) {
  if (config_ == nullptr) {
    throw std::invalid_argument("MmConfigEeKinematics requires MMConfig");
  }
}

Eigen::Matrix4d MmConfigEeKinematics::pose(
    const Eigen::Vector3d& car, const Eigen::Matrix<double, 6, 1>& q) const {
  return config_->getEePose(car, q);
}

CompletionVerifier::CompletionVerifier(CompletionThresholds thresholds,
                                       const EeKinematics* kinematics)
    : thresholds_(thresholds), kinematics_(kinematics) {
  if (kinematics_ == nullptr) {
    throw std::invalid_argument("CompletionVerifier requires EeKinematics");
  }
}

double CompletionVerifier::yawAbsError(double a, double b) {
  double err = a - b;
  while (err > M_PI) {
    err -= 2.0 * M_PI;
  }
  while (err < -M_PI) {
    err += 2.0 * M_PI;
  }
  return std::abs(err);
}

double CompletionVerifier::rotationAngle(const Eigen::Matrix3d& r_expected,
                                         const Eigen::Matrix3d& r_actual) {
  const Eigen::Matrix3d r = r_expected.transpose() * r_actual;
  const double c = std::max(-1.0, std::min(1.0, (r.trace() - 1.0) / 2.0));
  return std::acos(c);
}

CompletionDecision CompletionVerifier::verify(
    const CompletionInput& input) const {
  CompletionDecision out;
  const Eigen::Vector3d actual_car(input.actual.base_xy.x(),
                                   input.actual.base_xy.y(),
                                   input.actual.base_yaw);
  const Eigen::Matrix4d actual_ee =
      kinematics_->pose(actual_car, input.actual.q);

  out.final_base_position_error =
      (input.actual.base_xy - input.expected_base_xy).norm();
  out.final_base_yaw_error =
      yawAbsError(input.actual.base_yaw, input.expected_base_yaw);
  out.final_joint_error =
      (input.actual.q - input.expected_q).cwiseAbs().maxCoeff();
  out.final_ee_pos_error =
      (actual_ee.block<3, 1>(0, 3) - input.expected_ee.block<3, 1>(0, 3))
          .norm();
  out.final_ee_rot_error = rotationAngle(input.expected_ee.block<3, 3>(0, 0),
                                         actual_ee.block<3, 3>(0, 0));

  if (!input.arm_action_succeeded) {
    return fail(out, "CR10_ACTION_NOT_SUCCEEDED",
                "Action success is required but not sufficient");
  }
  if (!input.feedback_fresh) {
    return fail(out, "TERMINAL_FEEDBACK_STALE", "terminal feedback is stale");
  }
  if (!input.robot_status_healthy) {
    return fail(out, "TERMINAL_ROBOT_STATUS",
                "RobotStatus is not healthy at terminal");
  }
  if (!input.actual.velocity_valid) {
    return fail(out, "TERMINAL_VELOCITY_INVALID",
                "terminal joint/base velocity is invalid");
  }

  const double base_speed = std::hypot(input.actual.base_velocity_world.x(),
                                       input.actual.base_velocity_world.y());
  if (out.final_base_position_error > thresholds_.base_position) {
    return fail(out, "TERMINAL_BASE_POSITION",
                "terminal base position exceeds tolerance");
  }
  if (out.final_base_yaw_error > thresholds_.base_yaw) {
    return fail(out, "TERMINAL_BASE_YAW", "terminal base yaw exceeds tolerance");
  }
  if (base_speed > thresholds_.base_linear_velocity) {
    return fail(out, "TERMINAL_BASE_LINEAR_VELOCITY",
                "terminal base linear speed exceeds stop limit");
  }
  if (std::abs(input.actual.base_yaw_rate) > thresholds_.base_yaw_rate) {
    return fail(out, "TERMINAL_BASE_YAW_RATE",
                "terminal yaw rate exceeds stop limit");
  }
  if (out.final_joint_error > thresholds_.arm_joint) {
    return fail(out, "TERMINAL_ARM_JOINT",
                "terminal joint error exceeds tolerance");
  }
  if (input.actual.qd.cwiseAbs().maxCoeff() > thresholds_.arm_velocity) {
    return fail(out, "TERMINAL_ARM_VELOCITY",
                "terminal joint speed exceeds stop limit");
  }
  if (out.final_ee_pos_error > thresholds_.ee_position) {
    return fail(out, "TERMINAL_EE_POSITION",
                "terminal EE position exceeds tolerance");
  }
  if (out.final_ee_rot_error > thresholds_.ee_rotation) {
    return fail(out, "TERMINAL_EE_ROTATION",
                "terminal EE rotation exceeds tolerance");
  }

  out.succeeded = true;
  return out;
}
// ################################
// C++: CompletionVerifier::verify end
// ################################

}  // namespace remani_real
