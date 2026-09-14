#include <gtest/gtest.h>

#include <remani_real/runtime_safety_monitor.hpp>

using remani_real::ArmGoalState;
using remani_real::RuntimeSafetyConfig;
using remani_real::RuntimeSafetyInput;
using remani_real::RuntimeSafetyMonitor;

namespace {

RuntimeSafetyConfig safetyConfig() {
  RuntimeSafetyConfig config;
  config.dry_run = false;
  config.expected_ranger_publisher = "/remani_real_node";
  config.odom_timeout = 0.30;
  config.joint_timeout = 0.30;
  config.tf_timeout = 0.30;
  config.max_base_linear_speed = 0.10;
  config.max_base_angular_speed = 0.15;
  config.max_joint_speed = 0.10;
  config.max_base_tracking_error = 0.20;
  config.max_joint_tracking_error = 0.10;
  return config;
}

RuntimeSafetyInput nominalRuntimeSafetyInput() {
  RuntimeSafetyInput input;
  input.odom_age = input.joint_age = input.tf_age = 0.0;
  input.joint_velocity_valid = true;
  input.robot_connected = input.robot_enabled = true;
  input.robot_fault = false;
  input.action_state = ArmGoalState::Active;
  input.ranger_watchdog_ready = true;
  input.ranger_watchdog_timed_out = false;
  input.hardware_topic_publishers = {"/remani_real_node"};
  input.base_tracking_error = input.joint_tracking_error = 0.0;
  input.desired_base_linear_speed = 0.05;
  input.desired_base_angular_speed = 0.05;
  input.desired_max_joint_speed = 0.05;
  return input;
}

}  // namespace

// ################################
TEST(RuntimeSafetyMonitor, ReportsTheFirstStableFaultCode) {
  RuntimeSafetyMonitor monitor(safetyConfig());
  RuntimeSafetyInput input = nominalRuntimeSafetyInput();
  input.odom_age = 0.31;
  EXPECT_EQ("ODOM_TIMEOUT", monitor.evaluate(input).error_code);

  input = nominalRuntimeSafetyInput();
  input.hardware_topic_publishers = {"/remani_real_node", "/rogue"};
  EXPECT_EQ("RANGER_PUBLISHER_OWNERSHIP", monitor.evaluate(input).error_code);

  input = nominalRuntimeSafetyInput();
  input.action_state = ArmGoalState::Aborted;
  EXPECT_EQ("CR10_ACTION_ABORTED", monitor.evaluate(input).error_code);
}

TEST(RuntimeSafetyMonitor, DryRunRequiresZeroPublishers) {
  RuntimeSafetyConfig config = safetyConfig();
  config.dry_run = true;
  RuntimeSafetyMonitor monitor(config);
  RuntimeSafetyInput input = nominalRuntimeSafetyInput();
  input.hardware_topic_publishers.clear();
  input.ranger_watchdog_timed_out = true;
  EXPECT_TRUE(monitor.evaluate(input).safe);

  input.hardware_topic_publishers = {"/remani_real_node"};
  EXPECT_EQ("RANGER_PUBLISHER_OWNERSHIP", monitor.evaluate(input).error_code);
}

TEST(RuntimeSafetyMonitor, RejectsNominalLimitWithoutClamping) {
  RuntimeSafetyMonitor monitor(safetyConfig());
  RuntimeSafetyInput input = nominalRuntimeSafetyInput();
  input.desired_base_linear_speed = 0.11;
  const auto decision = monitor.evaluate(input);
  EXPECT_FALSE(decision.safe);
  EXPECT_EQ("NOMINAL_TRAJECTORY_LIMIT", decision.error_code);
}

TEST(RuntimeSafetyMonitor, NominalInputIsSafe) {
  RuntimeSafetyMonitor monitor(safetyConfig());
  EXPECT_TRUE(monitor.evaluate(nominalRuntimeSafetyInput()).safe);
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
