#include <gtest/gtest.h>

#include <remani_real/deployment_state_machine.hpp>
#include <remani_real/motion_output.hpp>

namespace remani_real {
namespace {

// ################################
// C++: DeploymentStateMachine test fixtures begin
// ################################
ReadinessSnapshot readySnapshot() {
  ReadinessSnapshot ready;
  ready.odom = true;
  ready.cr10_joints = true;
  ready.cr10_velocity = true;
  ready.tf = true;
  ready.robot_status = true;
  ready.action_server = true;
  ready.grid_map = true;
  ready.ranger_watchdog = true;
  return ready;
}

class RecordingRangerChannel : public RangerCommandChannel {
 public:
  bool publish(const geometry_msgs::Twist& command) override {
    last = command;
    ++count;
    return true;
  }
  bool hardwareOutputEnabled() const override { return enabled; }

  geometry_msgs::Twist last;
  std::size_t count{0};
  bool enabled{false};
};

class RecordingArmChannel : public ArmCommandChannel {
 public:
  bool send(const trajectory_msgs::JointTrajectory& trajectory) override {
    last = trajectory;
    ++send_count;
    goal_state = ArmGoalState::Pending;
    return true;
  }
  bool cancel() override {
    ++cancel_count;
    goal_state = ArmGoalState::Canceled;
    return true;
  }
  bool stop() override {
    ++stop_count;
    goal_state = ArmGoalState::Aborted;
    return true;
  }
  ArmGoalState state() const override { return goal_state; }
  bool hardwareOutputEnabled() const override { return enabled; }

  trajectory_msgs::JointTrajectory last;
  ArmGoalState goal_state{ArmGoalState::Unavailable};
  std::size_t send_count{0};
  std::size_t cancel_count{0};
  std::size_t stop_count{0};
  bool enabled{false};
};
// ################################
// C++: DeploymentStateMachine test fixtures end
// ################################

TEST(DeploymentStateMachine, StartsNotReadyUntilAllFeedbackReady) {
  DeploymentStateMachine fsm;
  EXPECT_EQ(State::NotReady, fsm.state());
  EXPECT_FALSE(fsm.permissions().plan);

  ReadinessSnapshot partial = readySnapshot();
  partial.ranger_watchdog = false;
  fsm.updateReadiness(partial);
  EXPECT_EQ(State::NotReady, fsm.state());

  fsm.updateReadiness(readySnapshot());
  EXPECT_EQ(State::Ready, fsm.state());
  EXPECT_TRUE(fsm.permissions().plan);
  EXPECT_FALSE(fsm.permissions().execute);
  EXPECT_FALSE(fsm.permissions().abort);
}

TEST(DeploymentStateMachine, RequiresExplicitExecuteAfterValidFinal) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  EXPECT_EQ(State::Ready, fsm.state());
  EXPECT_TRUE(fsm.requestPlan().accepted);
  EXPECT_EQ(State::Planning, fsm.state());
  fsm.onCandidateValidated(7, true);
  EXPECT_EQ(State::Planned, fsm.state());
  EXPECT_TRUE(fsm.requestExecute(7).accepted);
  EXPECT_EQ(State::Executing, fsm.state());
}

TEST(DeploymentStateMachine, RejectsStaleIdAndNewStartWhileExecuting) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onCandidateValidated(8, true);
  EXPECT_FALSE(fsm.requestExecute(7).accepted);
  ASSERT_TRUE(fsm.requestExecute(8).accepted);
  EXPECT_FALSE(fsm.newTransactionAllowed());
  EXPECT_FALSE(fsm.requestPlan().accepted);
}

TEST(DeploymentStateMachine, OrdinaryPlanningFailureReturnsReady) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onPlanningFailure("NO_PATH", false);
  EXPECT_EQ(State::Ready, fsm.state());
  EXPECT_TRUE(fsm.permissions().plan);
}

TEST(DeploymentStateMachine, ProtocolCorruptionEntersError) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onPlanningFailure("SEGMENT_SEQUENCE", true);
  EXPECT_EQ(State::Error, fsm.state());
  EXPECT_TRUE(fsm.permissions().abort);
  EXPECT_FALSE(fsm.permissions().plan);
}

TEST(DeploymentStateMachine, PlannedPermissionsAndAbort) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onCandidateValidated(3, true);
  const CommandPermissions perms = fsm.permissions();
  EXPECT_TRUE(perms.plan);
  EXPECT_TRUE(perms.execute);
  EXPECT_TRUE(perms.abort);
  EXPECT_FALSE(perms.pause);
  EXPECT_FALSE(perms.resume);
  ASSERT_TRUE(fsm.requestAbort().accepted);
  EXPECT_EQ(State::Ready, fsm.state());
  EXPECT_EQ(0u, fsm.plannedCandidateId());
}

TEST(DeploymentStateMachine, PauseResumeAbortLifecycle) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onCandidateValidated(4, true);
  ASSERT_TRUE(fsm.requestExecute(4).accepted);
  EXPECT_TRUE(fsm.permissions().pause);
  ASSERT_TRUE(fsm.requestPause().accepted);
  EXPECT_EQ(State::Executing, fsm.state());
  fsm.onPauseConfirmed();
  EXPECT_EQ(State::Paused, fsm.state());
  EXPECT_TRUE(fsm.permissions().resume);
  EXPECT_FALSE(fsm.requestResume(false).accepted);
  EXPECT_EQ(State::Paused, fsm.state());
  ASSERT_TRUE(fsm.requestResume(true).accepted);
  EXPECT_EQ(State::Executing, fsm.state());
  ASSERT_TRUE(fsm.requestAbort().accepted);
  EXPECT_EQ(State::Ready, fsm.state());
}

TEST(DeploymentStateMachine, ExecutionSuccessAndReplan) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onCandidateValidated(5, true);
  ASSERT_TRUE(fsm.requestExecute(5).accepted);
  fsm.onExecutionSucceeded();
  EXPECT_EQ(State::Succeeded, fsm.state());
  EXPECT_TRUE(fsm.permissions().plan);
  EXPECT_FALSE(fsm.permissions().execute);
  ASSERT_TRUE(fsm.requestPlan().accepted);
  EXPECT_EQ(State::Planning, fsm.state());
}

TEST(DeploymentStateMachine, ReadinessLossDuringExecutionEntersError) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onCandidateValidated(6, true);
  ASSERT_TRUE(fsm.requestExecute(6).accepted);
  ReadinessSnapshot lost = readySnapshot();
  lost.odom = false;
  fsm.updateReadiness(lost);
  EXPECT_EQ(State::Error, fsm.state());
  EXPECT_FALSE(fsm.requestAbort().accepted);
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestAbort().accepted);
  EXPECT_EQ(State::Ready, fsm.state());
}

TEST(DeploymentStateMachine, InvalidValidationReturnsReady) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onCandidateValidated(9, false);
  EXPECT_EQ(State::Ready, fsm.state());
  EXPECT_EQ(0u, fsm.plannedCandidateId());
}

TEST(MotionOutputBoundary, ChannelsExposeHardwareEnableFlag) {
  RecordingRangerChannel ranger;
  RecordingArmChannel arm;
  EXPECT_FALSE(ranger.hardwareOutputEnabled());
  EXPECT_FALSE(arm.hardwareOutputEnabled());
  geometry_msgs::Twist twist;
  twist.linear.x = 0.05;
  EXPECT_TRUE(ranger.publish(twist));
  EXPECT_EQ(1u, ranger.count);
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = {"cr10_joint1", "cr10_joint2", "cr10_joint3",
                      "cr10_joint4", "cr10_joint5", "cr10_joint6"};
  EXPECT_TRUE(arm.send(traj));
  EXPECT_EQ(1u, arm.send_count);
  EXPECT_EQ(ArmGoalState::Pending, arm.state());
}

}  // namespace
}  // namespace remani_real

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
