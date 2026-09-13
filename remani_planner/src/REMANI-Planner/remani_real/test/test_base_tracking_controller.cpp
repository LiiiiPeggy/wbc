#include <cmath>

#include <gtest/gtest.h>

#include <remani_real/base_tracking_controller.hpp>

using remani_real::ActualStateSnapshot;
using remani_real::BaseTrackingConfig;
using remani_real::BaseTrackingController;
using remani_real::WholeBodySample;

namespace {

// ################################
BaseTrackingConfig trackingConfig() {
  BaseTrackingConfig config;
  config.k_x = 1.0;
  config.k_y = 1.0;
  config.k_yaw = 1.0;
  config.max_linear = 0.10;
  config.max_angular = 0.15;
  config.max_linear_correction = 0.03;
  config.max_angular_correction = 0.05;
  config.max_position_error = 0.20;
  config.max_yaw_error = 0.20;
  return config;
}

WholeBodySample desiredSample(double speed, int singul) {
  WholeBodySample desired;
  desired.position = Eigen::VectorXd::Zero(8);
  desired.velocity = Eigen::VectorXd::Zero(8);
  desired.acceleration = Eigen::VectorXd::Zero(8);
  desired.velocity.x() = std::abs(speed);
  desired.base_yaw = 0.0;
  desired.base_angular_velocity = 0.0;
  desired.singul = singul;
  return desired;
}

ActualStateSnapshot matchingActual() {
  ActualStateSnapshot actual;
  actual.base_xy.setZero();
  actual.base_yaw = 0.0;
  actual.odom_valid = true;
  return actual;
}
// ################################

}  // namespace

TEST(BaseTrackingController, FollowsStraightCandidateAtNominalSpeed) {
  BaseTrackingController controller(trackingConfig());
  const auto result = controller.compute(desiredSample(0.05, 1), matchingActual());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(0.05, result.command.linear.x, 1e-6);
  EXPECT_NEAR(0.0, result.command.angular.z, 1e-6);
}

TEST(BaseTrackingController, PreservesReverseSingularity) {
  BaseTrackingController controller(trackingConfig());
  const auto result =
      controller.compute(desiredSample(0.05, -1), matchingActual());
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.command.linear.x, 0.0);
}

TEST(BaseTrackingController, ExcessTrackingErrorFailsInsteadOfTimeScaling) {
  auto actual = matchingActual();
  actual.base_xy.x() += 0.25;
  BaseTrackingController controller(trackingConfig());
  const auto result = controller.compute(desiredSample(0.05, 1), actual);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ("BASE_TRACKING_ERROR", result.error_code);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
