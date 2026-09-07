#include <cmath>
#include <limits>
#include <type_traits>

#include <gtest/gtest.h>

#include <remani_real/actual_state.hpp>
#include <remani_real/candidate_trajectory.hpp>

namespace remani_real {
namespace {

/* ---------- Test fixtures build polynomial pieces in MMController coefficient order. ---------- */
CandidateSegment constantVelocitySegment(uint32_t id, int singul,
                                         double start_time, double duration,
                                         double vx) {
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  coeff.col(7) << start_time * vx, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  coeff(0, 6) = vx;

  CandidateSegment segment;
  segment.trajectory_id = id;
  segment.singul = singul;
  segment.trajectory.emplace_back(duration, coeff);
  segment.start_time = start_time;
  segment.duration = duration;
  return segment;
}

CandidateSegment stoppedSegment(uint32_t id, double start_time, double duration) {
  CandidateSegment segment = constantVelocitySegment(id, 1, start_time, duration, 0.0);
  return segment;
}

CandidateSegment deceleratingSegment(uint32_t id, double start_time, double duration) {
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  coeff.col(7) << 0.0, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  coeff(0, 6) = 1.0;
  coeff(0, 5) = -0.5;

  CandidateSegment segment;
  segment.trajectory_id = id;
  segment.singul = 1;
  segment.trajectory.emplace_back(duration, coeff);
  segment.start_time = start_time;
  segment.duration = duration;
  return segment;
}

CandidateSegment narrowMultiIntervalSegment(uint32_t id) {
  constexpr double kThreshold = 1e-6;
  constexpr double kPeakOffset = 1e-12;
  constexpr double kShape = 4e-5;
  constexpr double kLateralSlope = 1e-9;
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);

  // vx = threshold + peak_offset - shape * (t - 0.3)^2 * (t - 0.7)^2.
  // Its two valid intervals are narrower than the former 1/128 search grid.
  coeff(0, 6) = kThreshold + kPeakOffset - kShape * 0.0441;
  coeff(0, 5) = kShape * 0.42 / 2.0;
  coeff(0, 4) = -kShape * 1.42 / 3.0;
  coeff(0, 3) = kShape * 2.0 / 4.0;
  coeff(0, 2) = -kShape / 5.0;
  coeff(1, 6) = -kLateralSlope * 0.5;
  coeff(1, 5) = kLateralSlope / 2.0;

  CandidateSegment segment;
  segment.trajectory_id = id;
  segment.singul = 1;
  segment.trajectory.emplace_back(1.0, coeff);
  segment.start_time = 0.0;
  segment.duration = 1.0;
  return segment;
}

TEST(CandidateTrajectory, SamplesAcrossSegmentBoundary) {
  const CandidateSegment first = constantVelocitySegment(1, 1, 0.0, 1.0, 0.1);
  const CandidateSegment second = constantVelocitySegment(2, -1, 1.0, 2.0, 0.1);
  const CandidateTrajectory candidate(42, {first, second}, 0.0);

  EXPECT_EQ(42u, candidate.id());
  EXPECT_DOUBLE_EQ(3.0, candidate.duration());
  EXPECT_EQ(1, candidate.sample(0.5).singul);
  EXPECT_EQ(-1, candidate.sample(1.0).singul);
  EXPECT_EQ(-1, candidate.sample(1.5).singul);
  EXPECT_EQ(8, candidate.sample(1.5).position.size());
  EXPECT_EQ(-1, candidate.sample(3.0).singul);
}

TEST(CandidateTrajectory, PreservesRawTransactionStamp) {
  const ros::Time stamp(123, 456);
  const CandidateTrajectory candidate(
      7, {constantVelocitySegment(1, 1, 0.0, 1.0, 0.1)}, 0.0, stamp);

  EXPECT_EQ(stamp, candidate.rawTransactionStamp());
}

TEST(CandidateTrajectory, UsesMostRecentYawWhenStopped) {
  const CandidateTrajectory candidate(
      9, {constantVelocitySegment(1, 1, 0.0, 1.0, 0.1),
          stoppedSegment(2, 1.0, 1.0)},
      0.7);

  EXPECT_NEAR(0.0, candidate.sample(0.5).base_yaw, 1e-12);
  EXPECT_NEAR(0.0, candidate.sample(1.5).base_yaw, 1e-12);
  EXPECT_DOUBLE_EQ(0.0, candidate.sample(1.5).base_angular_velocity);
}

TEST(CandidateTrajectory, UsesConstructorYawUntilMotionIsRecoverable) {
  const CandidateTrajectory candidate(10, {stoppedSegment(1, 0.0, 1.0)}, 0.7);

  EXPECT_NEAR(0.7, candidate.sample(0.5).base_yaw, 1e-12);
}

TEST(CandidateTrajectory, RecoversHeadingFromInsideStoppedCurrentPiece) {
  const CandidateTrajectory candidate(
      16, {deceleratingSegment(1, 0.0, 1.0)}, 0.73);

  const WholeBodySample sample = candidate.sample(1.0);
  EXPECT_NEAR(0.0, sample.base_yaw, 1e-12);
  EXPECT_DOUBLE_EQ(0.0, sample.base_angular_velocity);
}

TEST(CandidateTrajectory, RecoversHeadingFromInsideStoppedPriorPiece) {
  CandidateSegment segment = deceleratingSegment(1, 0.0, 1.0);
  segment.trajectory.emplace_back(
      1.0, MMController::Piece::CoefficientMat::Zero(8, 8));
  segment.duration = 2.0;
  const CandidateTrajectory candidate(17, {segment}, 0.73);

  const WholeBodySample sample = candidate.sample(1.5);
  EXPECT_NEAR(0.0, sample.base_yaw, 1e-12);
  EXPECT_DOUBLE_EQ(0.0, sample.base_angular_velocity);
}

TEST(CandidateTrajectory, RecoversLatestNarrowMultiIntervalHeading) {
  const CandidateTrajectory candidate(18, {narrowMultiIntervalSegment(1)}, 0.73);

  const WholeBodySample sample = candidate.sample(1.0);
  EXPECT_GT(sample.base_yaw, 0.0);
  EXPECT_LT(sample.base_yaw, 0.1);
  EXPECT_DOUBLE_EQ(0.0, sample.base_angular_velocity);
}

TEST(CandidateTrajectory, ComputesAngularVelocityFromPlanarAcceleration) {
  CandidateSegment segment = constantVelocitySegment(1, 1, 0.0, 2.0, 1.0);
  MMController::Piece::CoefficientMat coeff = segment.trajectory[0].getCoeffMat();
  coeff(1, 5) = 1.0;
  segment.trajectory.clear();
  segment.trajectory.emplace_back(2.0, coeff);
  const CandidateTrajectory candidate(11, {segment}, 0.0);

  const WholeBodySample sample = candidate.sample(1.0);
  EXPECT_NEAR(std::atan2(2.0, 1.0), sample.base_yaw, 1e-12);
  EXPECT_NEAR(0.4, sample.base_angular_velocity, 1e-12);
}

TEST(CandidateTrajectory, RejectsInvalidSampleTimes) {
  const CandidateTrajectory candidate(
      12, {constantVelocitySegment(1, 1, 0.0, 1.0, 0.1)}, 0.0);

  EXPECT_THROW(candidate.sample(-0.01), std::out_of_range);
  EXPECT_THROW(candidate.sample(1.01), std::out_of_range);
  EXPECT_THROW(candidate.sample(std::numeric_limits<double>::quiet_NaN()),
               std::out_of_range);
}

TEST(CandidateTrajectory, RejectsInvalidCandidateInvariants) {
  EXPECT_THROW(CandidateTrajectory(13, {}, 0.0), std::invalid_argument);

  CandidateSegment discontinuous = constantVelocitySegment(1, 1, 0.1, 1.0, 0.1);
  EXPECT_THROW(CandidateTrajectory(14, {discontinuous}, 0.0), std::invalid_argument);

  CandidateSegment wrong_dimension = constantVelocitySegment(1, 1, 0.0, 1.0, 0.1);
  wrong_dimension.trajectory.clear();
  wrong_dimension.trajectory.emplace_back(
      1.0, MMController::Piece::CoefficientMat::Zero(7, 8));
  EXPECT_THROW(CandidateTrajectory(15, {wrong_dimension}, 0.0), std::invalid_argument);

  CandidateSegment zero_duration = constantVelocitySegment(1, 1, 0.0, 0.0, 0.1);
  EXPECT_THROW(CandidateTrajectory(19, {zero_duration}, 0.0), std::invalid_argument);

  const double huge_duration = std::numeric_limits<double>::max() * 0.75;
  CandidateSegment first = constantVelocitySegment(1, 1, 0.0, huge_duration, 0.1);
  CandidateSegment second =
      constantVelocitySegment(2, 1, huge_duration, huge_duration, 0.1);
  EXPECT_THROW(CandidateTrajectory(20, {first, second}, 0.0), std::invalid_argument);
}

TEST(CandidateTrajectory, RejectsNonFinitePolynomialEvaluation) {
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  coeff(0, 7) = std::numeric_limits<double>::max();
  coeff(0, 6) = std::numeric_limits<double>::max();
  CandidateSegment segment;
  segment.trajectory_id = 1;
  segment.singul = 1;
  segment.trajectory.emplace_back(1.0, coeff);
  segment.duration = 1.0;

  const CandidateTrajectory candidate(21, {segment}, 0.0);
  EXPECT_THROW(candidate.sample(1.0), std::domain_error);
}

TEST(CandidateTrajectory, ExposesOnlyConstSegments) {
  using SegmentsReference = decltype(std::declval<const CandidateTrajectory&>().segments());
  static_assert(std::is_same<SegmentsReference,
                             const std::vector<CandidateSegment>&>::value,
                "Candidate segments must not be mutable through public accessors");
  SUCCEED();
}

TEST(ActualStateSnapshot, StartsFailClosed) {
  const ActualStateSnapshot snapshot;
  EXPECT_FALSE(snapshot.odom_valid);
  EXPECT_FALSE(snapshot.joints_valid);
  EXPECT_FALSE(snapshot.velocity_valid);
  EXPECT_FALSE(snapshot.tf_valid);
  EXPECT_FALSE(snapshot.robot_connected);
  EXPECT_FALSE(snapshot.robot_enabled);
  EXPECT_FALSE(snapshot.robot_fault);
  EXPECT_DOUBLE_EQ(0.0, snapshot.base_xy.norm());
  EXPECT_DOUBLE_EQ(0.0, snapshot.q.norm());
}

}  // namespace
}  // namespace remani_real

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
