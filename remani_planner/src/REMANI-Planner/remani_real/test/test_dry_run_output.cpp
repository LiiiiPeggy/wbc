#include <gtest/gtest.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <remani_real/dry_run_motion_output.hpp>

namespace remani_real {
namespace {

// ################################
// C++: DryRunMotionOutput gtests begin
// ################################
TEST(DryRunMotionOutput, RecordsDiagnosticsWithoutHardwareSideEffects) {
  DryRunMotionOutput output;

  geometry_msgs::Twist twist;
  twist.linear.x = 0.05;
  ASSERT_TRUE(output.publish(twist));

  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = {"cr10_joint1", "cr10_joint2", "cr10_joint3",
                            "cr10_joint4", "cr10_joint5", "cr10_joint6"};
  trajectory_msgs::JointTrajectoryPoint p0;
  p0.positions.assign(6, 0.0);
  p0.velocities.assign(6, 0.0);
  p0.time_from_start = ros::Duration(0.0);
  trajectory_msgs::JointTrajectoryPoint p1 = p0;
  p1.positions[0] = 0.1;
  p1.velocities[0] = 0.05;
  p1.time_from_start = ros::Duration(1.0);
  trajectory.points.push_back(p0);
  trajectory.points.push_back(p1);

  ASSERT_TRUE(output.send(trajectory));
  EXPECT_EQ(1u, output.rangerDiagnostics().size());
  EXPECT_DOUBLE_EQ(0.05, output.rangerDiagnostics().front().linear.x);
  EXPECT_EQ(1u, output.armDiagnostics().size());
  EXPECT_EQ(6u, output.armDiagnostics().front().joint_names.size());
  EXPECT_EQ(2u, output.armDiagnostics().front().points.size());
  EXPECT_EQ(ArmGoalState::Pending, output.state());

  EXPECT_FALSE(output.hardwareOutputEnabled());
  EXPECT_EQ(0u, output.hardwarePublishCount());
  EXPECT_EQ(0u, output.actionGoalCount());
  EXPECT_EQ(0u, output.writeServiceCount());
}

TEST(DryRunMotionOutput, CancelAndStopOnlyAffectDiagnosticArmState) {
  DryRunMotionOutput output;
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = {"cr10_joint1", "cr10_joint2", "cr10_joint3",
                            "cr10_joint4", "cr10_joint5", "cr10_joint6"};
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.assign(6, 0.0);
  point.velocities.assign(6, 0.0);
  trajectory.points.push_back(point);
  ASSERT_TRUE(output.send(trajectory));
  ASSERT_TRUE(output.cancel());
  EXPECT_EQ(ArmGoalState::Canceled, output.state());
  ASSERT_TRUE(output.stop());
  EXPECT_EQ(ArmGoalState::Aborted, output.state());
  EXPECT_EQ(0u, output.hardwarePublishCount());
  EXPECT_EQ(0u, output.actionGoalCount());
  EXPECT_EQ(0u, output.writeServiceCount());
}
// ################################
// C++: DryRunMotionOutput gtests end
// ################################

}  // namespace
}  // namespace remani_real

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
