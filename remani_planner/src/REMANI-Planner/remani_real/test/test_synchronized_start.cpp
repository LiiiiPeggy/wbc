#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <remani_real/synchronized_executor.hpp>

using remani_real::ActualStateSnapshot;
using remani_real::ArmCommandChannel;
using remani_real::ArmGoalState;
using remani_real::CandidateSegment;
using remani_real::CandidateTrajectory;
using remani_real::ExecutorStepState;
using remani_real::FrozenCandidate;
using remani_real::RangerCommandChannel;
using remani_real::SynchronizedExecutor;
using remani_real::SynchronizedExecutorConfig;

namespace {

ros::SteadyTime steadyAt(double sec) {
  ros::SteadyTime value;
  value.fromSec(sec);
  return value;
}

SynchronizedExecutorConfig testExecutorConfig() {
  SynchronizedExecutorConfig config;
  config.dry_run = false;
  config.start_lead_time = 1.0;
  config.arm_accept_guard = 0.20;
  config.max_start_skew = 0.10;
  return config;
}

ActualStateSnapshot actualAtOrigin() {
  ActualStateSnapshot actual;
  actual.base_xy.setZero();
  actual.base_yaw = 0.0;
  actual.q << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60;
  actual.qd.setZero();
  actual.odom_valid = actual.joints_valid = actual.velocity_valid = true;
  actual.tf_valid = actual.robot_connected = actual.robot_enabled = true;
  return actual;
}

FrozenCandidate candidateAtOrigin() {
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  coeff.col(7).tail<6>() = actualAtOrigin().q;
  coeff(0, 6) = 0.05;
  MMController::Trajectory trajectory;
  trajectory.emplace_back(2.0, coeff);
  CandidateSegment segment;
  segment.trajectory_id = 1;
  segment.singul = 1;
  segment.trajectory = trajectory;
  segment.start_time = 0.0;
  segment.duration = 2.0;
  return std::make_shared<const CandidateTrajectory>(
      1, std::vector<CandidateSegment>{segment}, 0.0);
}

class FakeRangerChannel : public RangerCommandChannel {
 public:
  bool publish(const geometry_msgs::Twist& value) override {
    commands.push_back(value);
    return true;
  }
  bool hardwareOutputEnabled() const override { return true; }
  bool onlyZeroCommands() const {
    return std::all_of(commands.begin(), commands.end(),
                       [](const geometry_msgs::Twist& cmd) {
                         return cmd.linear.x == 0.0 && cmd.linear.y == 0.0 &&
                                cmd.angular.z == 0.0;
                       });
  }
  std::vector<geometry_msgs::Twist> commands;
};

class FakeArmChannel : public ArmCommandChannel {
 public:
  bool send(const trajectory_msgs::JointTrajectory&) override {
    ++send_count;
    return send_ok;
  }
  bool cancel() override {
    ++cancel_count;
    return true;
  }
  bool stop() override {
    ++stop_count;
    return true;
  }
  ArmGoalState state() const override { return state_value; }
  bool hardwareOutputEnabled() const override { return true; }
  void setAcceptState(ArmGoalState value) { state_value = value; }
  std::size_t cancelCount() const { return cancel_count; }
  ArmGoalState state_value{ArmGoalState::Pending};
  std::size_t send_count{0}, cancel_count{0}, stop_count{0};
  bool send_ok{true};
};

}  // namespace

// ################################
TEST(SynchronizedStart, NeverStartsRangerWithoutAcceptedArmGoal) {
  FakeRangerChannel ranger;
  FakeArmChannel arm;
  arm.setAcceptState(ArmGoalState::Pending);
  SynchronizedExecutor executor(testExecutorConfig(), &ranger, &arm);

  ASSERT_TRUE(executor
                  .prepare(candidateAtOrigin(), actualAtOrigin(),
                           steadyAt(10.0))
                  .accepted);
  EXPECT_EQ(ExecutorStepState::HoldingForT0,
            executor.tick(steadyAt(10.7), actualAtOrigin()).state);
  EXPECT_TRUE(ranger.onlyZeroCommands());

  const auto late = executor.tick(steadyAt(10.81), actualAtOrigin());
  EXPECT_EQ(ExecutorStepState::Error, late.state);
  EXPECT_EQ("ARM_ACCEPT_DEADLINE", late.error_code);
  EXPECT_EQ(1u, arm.cancelCount());
  EXPECT_TRUE(ranger.onlyZeroCommands());
}

TEST(SynchronizedStart, AcceptsBeforeDeadlineAndHoldsT0) {
  FakeRangerChannel ranger;
  FakeArmChannel arm;
  SynchronizedExecutor executor(testExecutorConfig(), &ranger, &arm);
  ASSERT_TRUE(executor
                  .prepare(candidateAtOrigin(), actualAtOrigin(),
                           steadyAt(10.0))
                  .accepted);
  EXPECT_EQ(1u, arm.send_count);
  arm.setAcceptState(ArmGoalState::Active);

  const auto hold =
      executor.tick(steadyAt(10.5), actualAtOrigin());
  EXPECT_EQ(ExecutorStepState::HoldingForT0, hold.state);
  EXPECT_TRUE(ranger.onlyZeroCommands());
  EXPECT_DOUBLE_EQ(11.0, executor.epoch().t0.toSec());

  const auto run = executor.tick(steadyAt(11.0), actualAtOrigin());
  EXPECT_EQ(ExecutorStepState::Running, run.state);
  EXPECT_DOUBLE_EQ(0.0, run.parameter_time);
  EXPECT_FALSE(ranger.commands.empty());
  EXPECT_DOUBLE_EQ(11.0, executor.timing().ranger_trajectory_start_at->toSec());
}

TEST(SynchronizedStart, FixedT0AfterAccept) {
  FakeRangerChannel ranger;
  FakeArmChannel arm;
  SynchronizedExecutor executor(testExecutorConfig(), &ranger, &arm);
  ASSERT_TRUE(executor
                  .prepare(candidateAtOrigin(), actualAtOrigin(),
                           steadyAt(20.0))
                  .accepted);
  const double t0 = executor.epoch().t0.toSec();
  arm.setAcceptState(ArmGoalState::Accepted);
  executor.tick(steadyAt(20.3), actualAtOrigin());
  EXPECT_DOUBLE_EQ(t0, executor.epoch().t0.toSec());
  EXPECT_DOUBLE_EQ(21.0, t0);
}

TEST(SynchronizedStart, ActionSendFailureRejectsPrepare) {
  FakeRangerChannel ranger;
  FakeArmChannel arm;
  arm.send_ok = false;
  SynchronizedExecutor executor(testExecutorConfig(), &ranger, &arm);
  const auto decision = executor.prepare(candidateAtOrigin(), actualAtOrigin(),
                                         steadyAt(1.0));
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ("ARM_SEND_FAILED", decision.error_code);
  EXPECT_TRUE(ranger.commands.empty());
}

TEST(SynchronizedStart, StationaryBaseSkewUsesTrajectoryStart) {
  FakeRangerChannel ranger;
  FakeArmChannel arm;
  arm.setAcceptState(ArmGoalState::Active);
  SynchronizedExecutorConfig config = testExecutorConfig();
  // Zero base motion candidate still records ranger_trajectory_start_at.
  FrozenCandidate candidate = candidateAtOrigin();
  // Rebuild with zero base velocity.
  {
    MMController::Piece::CoefficientMat coeff =
        MMController::Piece::CoefficientMat::Zero(8, 8);
    coeff.col(7).tail<6>() = actualAtOrigin().q;
    MMController::Trajectory trajectory;
    trajectory.emplace_back(1.0, coeff);
    CandidateSegment segment;
    segment.trajectory_id = 1;
    segment.singul = 1;
    segment.trajectory = trajectory;
    segment.start_time = 0.0;
    segment.duration = 1.0;
    candidate = std::make_shared<const CandidateTrajectory>(
        2, std::vector<CandidateSegment>{segment}, 0.0);
  }
  SynchronizedExecutor executor(config, &ranger, &arm);
  ASSERT_TRUE(
      executor.prepare(candidate, actualAtOrigin(), steadyAt(0.0)).accepted);
  executor.noteCr10FirstMotionSteady(1.05);
  const auto run = executor.tick(steadyAt(1.0), actualAtOrigin());
  EXPECT_EQ(ExecutorStepState::Running, run.state);
  ASSERT_TRUE(executor.timing().ranger_trajectory_start_at.is_initialized());
  ASSERT_TRUE(executor.timing().start_skew.is_initialized());
  EXPECT_NEAR(0.05, *executor.timing().start_skew, 1e-9);
  EXPECT_FALSE(executor.timing().ranger_first_motion_at.is_initialized());
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
