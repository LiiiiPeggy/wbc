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
  std::vector<geometry_msgs::Twist> commands;
};

class FakeArmChannel : public ArmCommandChannel {
 public:
  bool send(const trajectory_msgs::JointTrajectory&) override {
    ++send_count;
    return true;
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
  ArmGoalState state_value{ArmGoalState::Pending};
  std::size_t send_count{0}, cancel_count{0}, stop_count{0};
};

}  // namespace

// ################################
class PauseResumeTest : public ::testing::Test {
 protected:
  PauseResumeTest() : executor(testExecutorConfig(), &ranger, &arm) {}

  ActualStateSnapshot actualAtParameter(double parameter_time, double base_speed,
                                        double joint_speed) const {
    ActualStateSnapshot actual = actualAtOrigin();
    actual.base_xy.x() = 0.05 * parameter_time;
    actual.base_velocity_world.x() = base_speed;
    actual.qd.setConstant(joint_speed);
    return actual;
  }

  void startAndPauseAt075() {
    arm.setAcceptState(ArmGoalState::Accepted);
    ASSERT_TRUE(executor
                    .prepare(candidateAtOrigin(), actualAtOrigin(),
                             steadyAt(10.0))
                    .accepted);
    ASSERT_EQ(ExecutorStepState::Running,
              executor.tick(steadyAt(11.75), actualAtParameter(0.75, 0.05, 0.01))
                  .state);
    ASSERT_TRUE(executor.requestPause().accepted);
  }

  FakeRangerChannel ranger;
  FakeArmChannel arm;
  SynchronizedExecutor executor;
};

TEST_F(PauseResumeTest, EntersPausedOnlyAfterBothActualStops) {
  startAndPauseAt075();
  EXPECT_EQ(ExecutorStepState::Stopping,
            executor.tick(steadyAt(11.76), actualAtParameter(0.75, 0.05, 0.01))
                .state);
  EXPECT_EQ(ExecutorStepState::Stopping,
            executor.tick(steadyAt(11.90), actualAtParameter(0.75, 0.0, 0.01))
                .state);
  EXPECT_EQ(ExecutorStepState::Paused,
            executor.tick(steadyAt(12.00), actualAtParameter(0.75, 0.0, 0.0))
                .state);
  EXPECT_EQ(1u, arm.cancel_count);
  EXPECT_EQ(1u, arm.stop_count);
}

TEST_F(PauseResumeTest, RejectsErrorAboveStrictToleranceWithoutConnector) {
  startAndPauseAt075();
  ASSERT_EQ(ExecutorStepState::Paused,
            executor.tick(steadyAt(12.00), actualAtParameter(0.75, 0.0, 0.0))
                .state);
  ActualStateSnapshot displaced = actualAtParameter(0.75, 0.0, 0.0);
  displaced.base_xy.x() += 0.021;
  const auto decision = executor.requestResume(displaced, steadyAt(13.0));
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ("RESUME_BASE_POSITION_TOLERANCE", decision.error_code);
  EXPECT_EQ(ExecutorStepState::Paused, executor.state());
  EXPECT_EQ(0u, executor.generatedConnectorCount());
}

TEST_F(PauseResumeTest, AbortInvalidatesCandidate) {
  startAndPauseAt075();
  ASSERT_EQ(ExecutorStepState::Paused,
            executor.tick(steadyAt(12.00), actualAtParameter(0.75, 0.0, 0.0))
                .state);
  ASSERT_TRUE(executor.requestAbort(actualAtOrigin()).accepted);
  EXPECT_EQ(ExecutorStepState::Idle, executor.state());
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
