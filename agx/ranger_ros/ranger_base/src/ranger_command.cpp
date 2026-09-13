#include <ranger_base/ranger_command.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <ranger_msgs/MotionState.h>

namespace westonrobot {
namespace {

bool isFiniteTwist(const geometry_msgs::Twist& msg) {
  return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
         std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
         std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
}

double clampAbs(double value, double limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

double centralSteerFromRadius(double signed_radius, double wheelbase,
                              double track) {
  const double radius = std::abs(signed_radius);
  const double half_l = wheelbase * 0.5;
  const double x = std::sqrt(radius * radius + half_l * half_l);
  const double denom = x - track * 0.5;
  if (!(denom > 0.0) || !std::isfinite(denom)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double phi = std::atan(half_l / denom);
  return (signed_radius >= 0.0) ? phi : -phi;
}

}  // namespace

// ################################
// C++: ComputeRangerCommand finite branches begin
// ################################
RangerCommandDecision ComputeRangerCommand(const geometry_msgs::Twist& msg,
                                           const RangerCommandLimits& limits,
                                           double zero_epsilon) {
  RangerCommandDecision out;
  if (!isFiniteTwist(msg) || !(zero_epsilon >= 0.0) ||
      !std::isfinite(zero_epsilon) || !std::isfinite(limits.track) ||
      !std::isfinite(limits.wheelbase) ||
      !std::isfinite(limits.min_turn_radius)) {
    return out;
  }

  const double vx = msg.linear.x;
  const double vy = msg.linear.y;
  const double wz = msg.angular.z;
  const double eps = zero_epsilon;

  if (std::abs(vx) <= eps && std::abs(vy) <= eps && std::abs(wz) <= eps) {
    out.valid = true;
    out.stop = true;
    out.motion_mode = ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN;
    out.linear = 0.0;
    out.steering = 0.0;
    out.angular = 0.0;
    out.turn_radius = 0.0;
    return out;
  }

  if (std::abs(vy) > eps) {
    if (std::abs(vx) <= eps && limits.allow_side_slip) {
      out.valid = true;
      out.motion_mode = ranger_msgs::MotionState::MOTION_MODE_SIDE_SLIP;
      out.linear = 0.0;
      out.steering = 0.0;
      out.angular = clampAbs(vy, limits.max_linear_speed);
      out.turn_radius = 0.0;
      return out;
    }

    // Parallel: never divide before confirming |vx| > eps.
    out.valid = true;
    out.motion_mode = ranger_msgs::MotionState::MOTION_MODE_PARALLEL;
    if (std::abs(vx) <= eps) {
      out.steering =
          (vy >= 0.0) ? limits.max_steer_angle_parallel
                      : -limits.max_steer_angle_parallel;
      out.linear = clampAbs(vy, limits.max_linear_speed);
    } else {
      out.steering = clampAbs(std::atan(vy / vx), limits.max_steer_angle_parallel);
      const double speed = std::sqrt(vx * vx + vy * vy);
      out.linear = (vx >= 0.0) ? speed : -speed;
      out.linear = clampAbs(out.linear, limits.max_linear_speed);
    }
    out.angular = 0.0;
    out.turn_radius = 0.0;
    return out;
  }

  // |vy| <= eps: Ackermann or spinning.
  if (std::abs(wz) <= eps) {
    out.valid = true;
    out.motion_mode = ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN;
    out.linear = clampAbs(vx, limits.max_linear_speed);
    out.steering = 0.0;
    out.angular = 0.0;
    out.turn_radius = 0.0;  // diagnostic; never vx/wz
    return out;
  }

  // Curved / spin selection uses finite radius only when |wz| > eps.
  const double radius = std::abs(vx) / std::abs(wz);
  if (!std::isfinite(radius)) {
    return out;
  }
  out.turn_radius = radius;

  if (radius < limits.min_turn_radius) {
    out.valid = true;
    out.motion_mode = ranger_msgs::MotionState::MOTION_MODE_SPINNING;
    out.linear = 0.0;
    out.steering = 0.0;
    out.angular = clampAbs(wz, limits.max_angular_speed);
    return out;
  }

  const int k = (wz * vx) >= 0.0 ? 1 : -1;
  const double steer =
      centralSteerFromRadius(static_cast<double>(k) * radius, limits.wheelbase,
                             limits.track);
  if (!std::isfinite(steer)) {
    return out;
  }

  out.valid = true;
  out.motion_mode = ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN;
  out.linear = clampAbs(vx, limits.max_linear_speed);
  out.steering = clampAbs(steer, limits.max_steer_angle_central);
  out.angular = 0.0;
  return out;
}
// ################################
// C++: ComputeRangerCommand finite branches end
// ################################

}  // namespace westonrobot
