#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <dobot_v4_bringup/trajectory_goal_validator.hpp>

#include "trajectory_test_fixtures.hpp"

using dobot_v4_bringup::TrajectoryGoalValidator;
using dobot_v4_bringup::test_fixtures::canonicalNames;
using dobot_v4_bringup::test_fixtures::validTrajectory;

// ################################
TEST(TrajectoryGoalValidator, ReordersNamesAndAllPointArrays) {
  auto input = validTrajectory(
      {"joint3", "joint1", "joint6", "joint2", "joint5", "joint4"});
  const auto result = TrajectoryGoalValidator::validate(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ("joint1", result.trajectory.joint_names[0]);
  EXPECT_EQ("joint2", result.trajectory.joint_names[1]);
  EXPECT_EQ("joint3", result.trajectory.joint_names[2]);
  EXPECT_EQ("joint4", result.trajectory.joint_names[3]);
  EXPECT_EQ("joint5", result.trajectory.joint_names[4]);
  EXPECT_EQ("joint6", result.trajectory.joint_names[5]);
  EXPECT_DOUBLE_EQ(input.points[0].positions[1],
                   result.trajectory.points[0].positions[0]);
}

TEST(TrajectoryGoalValidator, RejectsMissingVelocity) {
  auto input = validTrajectory(canonicalNames());
  input.points[1].velocities.clear();
  const auto result = TrajectoryGoalValidator::validate(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ("VELOCITY_SIZE", result.error_code);
}

TEST(TrajectoryGoalValidator, RejectsNonIncreasingTime) {
  auto input = validTrajectory(canonicalNames());
  input.points[1].time_from_start = input.points[0].time_from_start;
  EXPECT_EQ("TIME_NOT_STRICT",
            TrajectoryGoalValidator::validate(input).error_code);
}

TEST(TrajectoryGoalValidator, RejectsSinglePoint) {
  auto input = validTrajectory(canonicalNames());
  input.points.pop_back();
  EXPECT_EQ("POINT_COUNT", TrajectoryGoalValidator::validate(input).error_code);
}

TEST(TrajectoryGoalValidator, RejectsDuplicateName) {
  auto input = validTrajectory(canonicalNames());
  input.joint_names[5] = "joint1";
  EXPECT_EQ("NAME_DUPLICATE",
            TrajectoryGoalValidator::validate(input).error_code);
}

TEST(TrajectoryGoalValidator, RejectsNaNPosition) {
  auto input = validTrajectory(canonicalNames());
  input.points[0].positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ("POSITION_NONFINITE",
            TrajectoryGoalValidator::validate(input).error_code);
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
