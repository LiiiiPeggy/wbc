#include <remani_real/base_tracking_controller.hpp>

#include <cmath>
#include <limits>

namespace remani_real {
namespace {

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double clampAbs(double value, double limit) {
  if (!(limit >= 0.0) || !std::isfinite(limit)) {
    return value;
  }
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

}  // namespace

// ################################
// C++: BaseTrackingController implementation begin
// ################################
BaseTrackingController::BaseTrackingController(BaseTrackingConfig config)
    : config_(config) {}

BaseTrackingResult BaseTrackingController::compute(
    const WholeBodySample& desired, const ActualStateSnapshot& actual) const {
  BaseTrackingResult out;
  if (!actual.odom_valid) {
    out.error_code = "ODOM_INVALID";
    return out;
  }
  if (desired.position.size() < 2 || desired.velocity.size() < 2 ||
      !desired.position.head<2>().allFinite() ||
      !desired.velocity.head<2>().allFinite() ||
      !std::isfinite(desired.base_yaw) ||
      !std::isfinite(desired.base_angular_velocity) ||
      !actual.base_xy.allFinite() || !std::isfinite(actual.base_yaw) ||
      desired.singul == 0) {
    out.error_code = "NON_FINITE";
    return out;
  }

  const Eigen::Vector2d desired_xy = desired.position.head<2>();
  const double yaw_d = desired.base_yaw;
  const Eigen::Rotation2Dd R_d(yaw_d);
  const Eigen::Vector2d e_xy =
      R_d.inverse() * (desired_xy - actual.base_xy);
  const double e_yaw = normalizeAngle(yaw_d - actual.base_yaw);
  out.position_error = e_xy.norm();
  out.yaw_error = e_yaw;

  if (out.position_error > config_.max_position_error ||
      std::abs(e_yaw) > config_.max_yaw_error) {
    out.error_code = "BASE_TRACKING_ERROR";
    return out;
  }

  const double v_ff =
      static_cast<double>(desired.singul) * desired.velocity.head<2>().norm();
  const double w_ff = desired.base_angular_velocity;
  if (!std::isfinite(v_ff) || !std::isfinite(w_ff) ||
      std::abs(v_ff) > config_.max_linear + 1e-9 ||
      std::abs(w_ff) > config_.max_angular + 1e-9) {
    out.error_code = "NOMINAL_LIMIT";
    return out;
  }

  const double v_corr =
      clampAbs(config_.k_x * e_xy.x(), config_.max_linear_correction);
  const double w_corr = clampAbs(
      config_.k_y * e_xy.y() + config_.k_yaw * e_yaw,
      config_.max_angular_correction);

  double v_cmd = v_ff + v_corr;
  double w_cmd = w_ff + w_corr;
  v_cmd = clampAbs(v_cmd, config_.max_linear);
  w_cmd = clampAbs(w_cmd, config_.max_angular);
  if (!std::isfinite(v_cmd) || !std::isfinite(w_cmd)) {
    out.error_code = "NON_FINITE";
    return out;
  }

  out.valid = true;
  out.command.linear.x = v_cmd;
  out.command.angular.z = w_cmd;
  return out;
}
// ################################
// C++: BaseTrackingController implementation end
// ################################

}  // namespace remani_real
