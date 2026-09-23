#include <gtest/gtest.h>

#include <dobot_v4_bringup/follow_joint_trajectory_adapter.hpp>

#include "fake_commander.hpp"
#include "trajectory_test_fixtures.hpp"

using dobot_v4_bringup::FakeCommander;
using dobot_v4_bringup::FollowJointTrajectoryAdapter;
using dobot_v4_bringup::RunnerConfig;
using dobot_v4_bringup::RunnerState;
using dobot_v4_bringup::test_fixtures::canonicalNames;
using dobot_v4_bringup::test_fixtures::validTrajectory;

namespace {

RunnerConfig runnerConfig() {
  RunnerConfig config;
  config.servoj_period = 0.10;
  config.goal_joint_tol = 0.02;
  config.stop_velocity_tol = 0.01;
  config.settle_timeout = 2.0;
  config.required_settle_samples = 3;
  config.remani_prestart_hold_mode = false;
  return config;
}

}  // namespace

// ################################
TEST(FollowJointTrajectoryAdapter, AcceptsWithoutSendingWholeTrajectory) {
  FakeCommander sink;
  FollowJointTrajectoryAdapter adapter(&sink, runnerConfig());
  const auto result = adapter.accept(validTrajectory(canonicalNames()), 1.0);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(0u, sink.servoJCalls());
  adapter.timerTick(1.1);
  EXPECT_EQ(1u, sink.servoJCalls());
}

TEST(FollowJointTrajectoryAdapter, CancelStopsFutureServoJ) {
  FakeCommander sink;
  FollowJointTrajectoryAdapter adapter(&sink, runnerConfig());
  ASSERT_TRUE(adapter.accept(validTrajectory(canonicalNames()), 1.0).accepted);
  adapter.timerTick(1.1);
  const std::size_t before = sink.servoJCalls();
  adapter.requestCancel(1.2);
  EXPECT_EQ(1u, sink.stopCalls());
  adapter.timerTick(1.3);
  adapter.timerTick(1.4);
  EXPECT_EQ(before, sink.servoJCalls());
}

TEST(FollowJointTrajectoryAdapter, RejectsSecondGoalWhileActive) {
  FakeCommander sink;
  FollowJointTrajectoryAdapter adapter(&sink, runnerConfig());
  ASSERT_TRUE(adapter.accept(validTrajectory(canonicalNames()), 1.0).accepted);
  const auto second = adapter.accept(validTrajectory(canonicalNames()), 1.1);
  EXPECT_FALSE(second.accepted);
  EXPECT_EQ("GOAL_ACTIVE", second.error_code);
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
