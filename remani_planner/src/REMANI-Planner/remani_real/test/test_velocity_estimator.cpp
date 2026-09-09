#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <remani_real/velocity_estimator.hpp>

namespace remani_real {
namespace {

TEST(VelocityEstimator, StaysInvalidUntilThreeStrictlyIncreasingSamples) {
  VelocityEstimator estimator(/*min_samples=*/3, /*max_abs_velocity=*/3.0,
                              /*alpha=*/0.5);
  Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  EXPECT_FALSE(estimator.update(ros::Time(1.0), q).valid);
  q << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
  EXPECT_FALSE(estimator.update(ros::Time(1.1), q).valid);
  q << 0.2, 0.0, 0.0, 0.0, 0.0, 0.0;
  const auto third = estimator.update(ros::Time(1.2), q);
  ASSERT_TRUE(third.valid);
  EXPECT_NEAR(1.0, third.qd(0), 1e-9);
}

TEST(VelocityEstimator, RejectsNonPositiveOrTooLargeDtAndClamps) {
  VelocityEstimator estimator(/*min_samples=*/3, /*max_abs_velocity=*/1.0,
                              /*alpha=*/1.0);
  Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  ASSERT_FALSE(estimator.update(ros::Time(1.0), q).valid);
  q << 0.05, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(estimator.update(ros::Time(1.05), q).valid);
  q << 0.10, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_TRUE(estimator.update(ros::Time(1.10), q).valid);

  // Non-positive dt invalidates.
  EXPECT_FALSE(estimator.update(ros::Time(1.10), q).valid);

  estimator.reset();
  q.setZero();
  ASSERT_FALSE(estimator.update(ros::Time(2.0), q).valid);
  q << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(estimator.update(ros::Time(2.05), q).valid);
  // Too-large dt (default max_dt=0.2) keeps estimate invalid.
  q << 0.2, 0.0, 0.0, 0.0, 0.0, 0.0;
  EXPECT_FALSE(estimator.update(ros::Time(2.5), q).valid);

  estimator.reset();
  q.setZero();
  ASSERT_FALSE(estimator.update(ros::Time(3.0), q).valid);
  q << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(estimator.update(ros::Time(3.05), q).valid);
  // Raw finite difference is 10 rad/s; clamp to max_abs_velocity=1.0.
  q << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  const auto clamped = estimator.update(ros::Time(3.10), q);
  ASSERT_TRUE(clamped.valid);
  EXPECT_NEAR(1.0, clamped.qd(0), 1e-9);
}

TEST(VelocityEstimator, AppliesLowPassAfterClamp) {
  VelocityEstimator estimator(/*min_samples=*/3, /*max_abs_velocity=*/10.0,
                              /*alpha=*/0.5);
  Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  ASSERT_FALSE(estimator.update(ros::Time(1.0), q).valid);
  q << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(estimator.update(ros::Time(1.1), q).valid);
  q << 0.2, 0.0, 0.0, 0.0, 0.0, 0.0;
  const auto first = estimator.update(ros::Time(1.2), q);
  ASSERT_TRUE(first.valid);
  EXPECT_NEAR(1.0, first.qd(0), 1e-9);

  // Next FD is 2.0; LPF with alpha=0.5 => 0.5*2 + 0.5*1 = 1.5.
  q << 0.4, 0.0, 0.0, 0.0, 0.0, 0.0;
  const auto filtered = estimator.update(ros::Time(1.3), q);
  ASSERT_TRUE(filtered.valid);
  EXPECT_NEAR(1.5, filtered.qd(0), 1e-9);
}

TEST(VelocityEstimator, RejectsNonFinitePositions) {
  VelocityEstimator estimator(/*min_samples=*/3, /*max_abs_velocity=*/3.0,
                              /*alpha=*/1.0);
  Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  ASSERT_FALSE(estimator.update(ros::Time(1.0), q).valid);
  q << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
  ASSERT_FALSE(estimator.update(ros::Time(1.1), q).valid);
  q << std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.0, 0.0;
  EXPECT_FALSE(estimator.update(ros::Time(1.2), q).valid);
}

}  // namespace
}  // namespace remani_real

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
