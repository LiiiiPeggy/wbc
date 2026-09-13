#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <geometry_msgs/Twist.h>
#include <ranger_msgs/MotionState.h>

#include <ranger_base/ranger_command.hpp>
#include <ranger_base/ranger_params.hpp>

using westonrobot::ComputeRangerCommand;
using westonrobot::RangerCommandDecision;
using westonrobot::RangerCommandLimits;
using westonrobot::RangerParams;

namespace {

// ################################
RangerCommandLimits rangerLimits() {
  RangerCommandLimits limits;
  limits.track = RangerParams::track;
  limits.wheelbase = RangerParams::wheelbase;
  limits.max_linear_speed = RangerParams::max_linear_speed;
  limits.max_angular_speed = RangerParams::max_angular_speed;
  limits.max_speed_cmd = RangerParams::max_speed_cmd;
  limits.max_steer_angle_central = RangerParams::max_steer_angle_central;
  limits.max_steer_angle_parallel = RangerParams::max_steer_angle_parallel;
  limits.max_round_angle = RangerParams::max_round_angle;
  limits.min_turn_radius = RangerParams::min_turn_radius;
  limits.allow_side_slip = false;
  return limits;
}
// ################################

}  // namespace

TEST(RangerCommand, ZeroTwistIsExplicitStop) {
  geometry_msgs::Twist msg;
  const RangerCommandDecision out =
      ComputeRangerCommand(msg, rangerLimits(), 1e-4);
  ASSERT_TRUE(out.valid);
  EXPECT_TRUE(out.stop);
  EXPECT_DOUBLE_EQ(0.0, out.linear);
  EXPECT_DOUBLE_EQ(0.0, out.steering);
  EXPECT_DOUBLE_EQ(0.0, out.angular);
}

TEST(RangerCommand, StraightTwistNeverDividesByZero) {
  geometry_msgs::Twist msg;
  msg.linear.x = 0.05;
  const RangerCommandDecision out =
      ComputeRangerCommand(msg, rangerLimits(), 1e-4);
  ASSERT_TRUE(out.valid);
  EXPECT_FALSE(out.stop);
  EXPECT_EQ(ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN, out.motion_mode);
  EXPECT_DOUBLE_EQ(0.0, out.steering);
  EXPECT_TRUE(std::isfinite(out.turn_radius));
  EXPECT_DOUBLE_EQ(0.0, out.turn_radius);
}

TEST(RangerCommand, RejectsAnyNonFiniteInput) {
  geometry_msgs::Twist msg;
  msg.angular.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ComputeRangerCommand(msg, rangerLimits(), 1e-4).valid);
}

TEST(RangerCommand, CurvedAckermannIsFinite) {
  geometry_msgs::Twist msg;
  msg.linear.x = 0.20;
  msg.angular.z = 0.10;
  const RangerCommandDecision out =
      ComputeRangerCommand(msg, rangerLimits(), 1e-4);
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN, out.motion_mode);
  EXPECT_TRUE(std::isfinite(out.steering));
  EXPECT_TRUE(std::isfinite(out.turn_radius));
  // C++11: avoid ODR-use of static constexpr via gtest reference args.
  const double min_turn_radius = 0.810330349;
  EXPECT_GT(out.turn_radius, min_turn_radius);
}

TEST(RangerCommand, TightRadiusSelectsSpinning) {
  geometry_msgs::Twist msg;
  msg.linear.x = 0.01;
  msg.angular.z = 1.0;
  const RangerCommandDecision out =
      ComputeRangerCommand(msg, rangerLimits(), 1e-4);
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(ranger_msgs::MotionState::MOTION_MODE_SPINNING, out.motion_mode);
  EXPECT_DOUBLE_EQ(0.0, out.linear);
  // Clamped to RangerParams::max_angular_speed (0.7853).
  EXPECT_NEAR(0.7853, out.angular, 1e-9);
}

TEST(RangerCommand, LateralParallelAvoidsDivideByZero) {
  geometry_msgs::Twist msg;
  msg.linear.y = 0.05;
  const RangerCommandDecision out =
      ComputeRangerCommand(msg, rangerLimits(), 1e-4);
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(ranger_msgs::MotionState::MOTION_MODE_PARALLEL, out.motion_mode);
  EXPECT_TRUE(std::isfinite(out.steering));
  EXPECT_TRUE(std::isfinite(out.linear));
}

TEST(RangerCommand, SideSlipWhenAllowed) {
  RangerCommandLimits limits = rangerLimits();
  limits.allow_side_slip = true;
  geometry_msgs::Twist msg;
  msg.linear.y = 0.05;
  const RangerCommandDecision out = ComputeRangerCommand(msg, limits, 1e-4);
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(ranger_msgs::MotionState::MOTION_MODE_SIDE_SLIP, out.motion_mode);
  EXPECT_NEAR(0.05, out.angular, 1e-9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
