#include <remani_real/runtime_safety_monitor.hpp>

#include <cmath>

namespace remani_real {

namespace {

SafetyDecision fail(const std::string& code, const std::string& detail) {
  SafetyDecision out;
  out.safe = false;
  out.error_code = code;
  out.detail = detail;
  return out;
}

bool finiteNonNeg(double value) {
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

// ################################
// C++: RuntimeSafetyMonitor::evaluate begin
// ################################
RuntimeSafetyMonitor::RuntimeSafetyMonitor(RuntimeSafetyConfig config)
    : config_(std::move(config)) {}

bool RuntimeSafetyMonitor::isFiniteNonNeg(double value) {
  return finiteNonNeg(value);
}

SafetyDecision RuntimeSafetyMonitor::evaluate(
    const RuntimeSafetyInput& input) const {
  if (!finiteNonNeg(input.odom_age) || !finiteNonNeg(input.joint_age) ||
      !finiteNonNeg(input.tf_age) ||
      !std::isfinite(input.base_tracking_error) ||
      !std::isfinite(input.joint_tracking_error) ||
      !std::isfinite(input.desired_base_linear_speed) ||
      !std::isfinite(input.desired_base_angular_speed) ||
      !std::isfinite(input.desired_max_joint_speed)) {
    return fail("NON_FINITE_SAFETY_INPUT", "safety input is non-finite");
  }

  if (input.odom_age > config_.odom_timeout) {
    return fail("ODOM_TIMEOUT", "odom age exceeds timeout");
  }
  if (input.joint_age > config_.joint_timeout) {
    return fail("JOINT_TIMEOUT", "joint age exceeds timeout");
  }
  if (input.tf_age > config_.tf_timeout) {
    return fail("TF_TIMEOUT", "tf age exceeds timeout");
  }
  if (!input.joint_velocity_valid) {
    return fail("JOINT_VELOCITY_INVALID", "joint velocity estimate is invalid");
  }
  if (!input.robot_connected) {
    return fail("CR10_NOT_CONNECTED", "RobotStatus connected is false");
  }
  if (!input.robot_enabled) {
    return fail("CR10_NOT_ENABLED", "RobotStatus enabled is false");
  }
  if (input.robot_fault) {
    return fail("CR10_FAULT", "RobotStatus reports a fault");
  }
  if (!config_.dry_run) {
    if (input.action_state == ArmGoalState::Aborted) {
      return fail("CR10_ACTION_ABORTED", "CR10 Action aborted");
    }
    if (input.action_state == ArmGoalState::Unavailable) {
      return fail("CR10_ACTION_UNAVAILABLE", "CR10 Action unavailable");
    }
  }

  if (config_.dry_run) {
    if (!input.hardware_topic_publishers.empty()) {
      return fail("RANGER_PUBLISHER_OWNERSHIP",
                  "dry-run requires zero hardware publishers");
    }
    if (!input.ranger_watchdog_ready) {
      return fail("RANGER_WATCHDOG_NOT_READY", "watchdog ready is false");
    }
  } else {
    if (input.hardware_topic_publishers.size() != 1 ||
        input.hardware_topic_publishers.front() !=
            config_.expected_ranger_publisher) {
      return fail("RANGER_PUBLISHER_OWNERSHIP",
                  "hardware topic must have exactly the expected publisher");
    }
    if (!input.ranger_watchdog_ready) {
      return fail("RANGER_WATCHDOG_NOT_READY", "watchdog ready is false");
    }
    if (input.ranger_watchdog_timed_out) {
      return fail("RANGER_WATCHDOG_TIMEOUT", "watchdog timed out");
    }
  }

  if (std::abs(input.desired_base_linear_speed) >
          config_.max_base_linear_speed ||
      std::abs(input.desired_base_angular_speed) >
          config_.max_base_angular_speed ||
      std::abs(input.desired_max_joint_speed) > config_.max_joint_speed) {
    return fail("NOMINAL_TRAJECTORY_LIMIT",
                "desired speed exceeds V1 nominal limits");
  }
  if (input.base_tracking_error > config_.max_base_tracking_error) {
    return fail("BASE_TRACKING_ERROR", "base tracking error exceeds limit");
  }
  if (input.joint_tracking_error > config_.max_joint_tracking_error) {
    return fail("JOINT_TRACKING_ERROR", "joint tracking error exceeds limit");
  }

  SafetyDecision ok;
  ok.safe = true;
  return ok;
}
// ################################
// C++: RuntimeSafetyMonitor::evaluate end
// ################################

}  // namespace remani_real
