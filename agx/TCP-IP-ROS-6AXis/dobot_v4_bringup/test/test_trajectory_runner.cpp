#include <gtest/gtest.h>

#include <dobot_v4_bringup/trajectory_runner.hpp>

using dobot_v4_bringup::CanonicalTrajectory;
using dobot_v4_bringup::CanonicalTrajectoryPoint;
using dobot_v4_bringup::RunnerActualState;
using dobot_v4_bringup::RunnerConfig;
using dobot_v4_bringup::RunnerState;
using dobot_v4_bringup::TrajectoryRunner;

namespace {

// ################################
RunnerConfig runnerConfig(bool remani_hold = false) {
  RunnerConfig config;
  config.servoj_period = 0.10;
  config.goal_joint_tol = 0.02;
  config.stop_velocity_tol = 0.01;
  config.settle_timeout = 2.0;
  config.required_settle_samples = 3;
  config.remani_prestart_hold_mode = remani_hold;
  return config;
}

CanonicalTrajectoryPoint point(double q, double time) {
  CanonicalTrajectoryPoint p;
  p.positions.fill(q);
  p.velocities.fill(0.0);
  p.accelerations.fill(0.0);
  p.time_from_start = time;
  return p;
}

CanonicalTrajectory twoSecondTrajectory() {
  CanonicalTrajectory trajectory;
  trajectory.joint_names = {"joint1", "joint2", "joint3",
                            "joint4", "joint5", "joint6"};
  trajectory.points = {point(0.0, 0.0), point(0.02, 2.0)};
  return trajectory;
}

CanonicalTrajectory remaniHoldTrajectory(double lead) {
  CanonicalTrajectory trajectory;
  trajectory.joint_names = {"joint1", "joint2", "joint3",
                            "joint4", "joint5", "joint6"};
  trajectory.points = {point(0.0, 0.0), point(0.01, lead),
                       point(0.02, lead + 1.0)};
  return trajectory;
}

RunnerActualState actualAt(double q) {
  RunnerActualState actual;
  actual.q.fill(q);
  actual.qd.fill(0.0);
  return actual;
}
// ################################

}  // namespace

TEST(TrajectoryRunner, TickReturnsExactlyOneCommand) {
  TrajectoryRunner runner(runnerConfig());
  ASSERT_TRUE(runner.start(twoSecondTrajectory(), 10.0).accepted);
  const auto tick = runner.tick(10.1, actualAt(0.0));
  EXPECT_TRUE(tick.has_command);
  EXPECT_EQ(6u, tick.command_rad.size());
  EXPECT_EQ(1u, runner.tickCount());
}

TEST(TrajectoryRunner, RemaniModeHoldsPointZeroUntilPointOneTime) {
  TrajectoryRunner runner(runnerConfig(true));
  ASSERT_TRUE(runner.start(remaniHoldTrajectory(1.0), 20.0).accepted);
  const auto before = runner.tick(20.9, actualAt(0.0));
  EXPECT_EQ(RunnerState::Holding, before.state);
  EXPECT_EQ(point(0.0, 0.0).positions, before.command_rad);
  const auto at_start = runner.tick(21.0, actualAt(0.0));
  EXPECT_EQ(RunnerState::Running, at_start.state);
  EXPECT_EQ(point(0.01, 1.0).positions, at_start.command_rad);
}

TEST(TrajectoryRunner, ExactFinalTimeEntersSettling) {
  TrajectoryRunner runner(runnerConfig());
  ASSERT_TRUE(runner.start(twoSecondTrajectory(), 0.0).accepted);
  const auto tick = runner.tick(2.0, actualAt(0.02));
  EXPECT_EQ(RunnerState::Settling, tick.state);
  EXPECT_TRUE(tick.has_command);
  EXPECT_EQ(point(0.02, 2.0).positions, tick.command_rad);
}

TEST(TrajectoryRunner, SettlesToSucceeded) {
  TrajectoryRunner runner(runnerConfig());
  ASSERT_TRUE(runner.start(twoSecondTrajectory(), 0.0).accepted);
  runner.tick(2.0, actualAt(0.02));
  EXPECT_FALSE(runner.tick(2.1, actualAt(0.02)).terminal);
  EXPECT_FALSE(runner.tick(2.2, actualAt(0.02)).terminal);
  const auto done = runner.tick(2.3, actualAt(0.02));
  EXPECT_TRUE(done.terminal);
  EXPECT_EQ(RunnerState::Succeeded, done.state);
}

TEST(TrajectoryRunner, CancelProducesNoFurtherCommands) {
  TrajectoryRunner runner(runnerConfig());
  ASSERT_TRUE(runner.start(twoSecondTrajectory(), 0.0).accepted);
  runner.tick(0.1, actualAt(0.0));
  runner.requestCancel();
  const auto tick = runner.tick(0.2, actualAt(0.0));
  EXPECT_EQ(RunnerState::Canceling, tick.state);
  EXPECT_FALSE(tick.has_command);
  EXPECT_FALSE(runner.tick(0.3, actualAt(0.0)).has_command);
  EXPECT_FALSE(runner.tick(0.4, actualAt(0.0)).has_command);
  const auto canceled = runner.tick(0.5, actualAt(0.0));
  EXPECT_TRUE(canceled.terminal);
  EXPECT_EQ(RunnerState::Canceled, canceled.state);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
