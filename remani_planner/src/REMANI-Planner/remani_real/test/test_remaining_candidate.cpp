#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <remani_real/actual_state.hpp>
#include <remani_real/remaining_candidate.hpp>

using remani_real::ActualStateSnapshot;
using remani_real::CandidateSegment;
using remani_real::CandidateTrajectory;
using remani_real::FrozenCandidate;
using remani_real::RemainingCandidate;
using remani_real::WholeBodySample;

namespace {

ActualStateSnapshot actualAtOrigin() {
  ActualStateSnapshot actual;
  actual.base_xy.setZero();
  actual.base_yaw = 0.0;
  actual.q << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60;
  actual.qd.setZero();
  actual.odom_valid = actual.joints_valid = true;
  return actual;
}

FrozenCandidate twoSegmentCandidate() {
  const auto actual = actualAtOrigin();
  MMController::Piece::CoefficientMat c1 =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  c1.col(7).tail<6>() = actual.q;
  c1(0, 6) = 0.05;
  MMController::Trajectory t1;
  t1.emplace_back(1.0, c1);

  MMController::Piece::CoefficientMat c2 = c1;
  c2(0, 7) = 0.05;
  MMController::Trajectory t2;
  t2.emplace_back(1.0, c2);

  CandidateSegment s1;
  s1.trajectory_id = 1;
  s1.singul = 1;
  s1.trajectory = t1;
  s1.start_time = 0.0;
  s1.duration = 1.0;
  CandidateSegment s2;
  s2.trajectory_id = 2;
  s2.singul = 1;
  s2.trajectory = t2;
  s2.start_time = 1.0;
  s2.duration = 1.0;
  return std::make_shared<const CandidateTrajectory>(
      4, std::vector<CandidateSegment>{s1, s2}, 0.0);
}

void expectWholeBodyNear(const WholeBodySample& expected,
                         const WholeBodySample& actual, double tolerance) {
  EXPECT_TRUE(expected.position.isApprox(actual.position, tolerance));
  EXPECT_TRUE(expected.velocity.isApprox(actual.velocity, tolerance));
  EXPECT_TRUE(expected.acceleration.isApprox(actual.acceleration, tolerance));
  EXPECT_NEAR(expected.base_yaw, actual.base_yaw, tolerance);
  EXPECT_NEAR(expected.base_angular_velocity, actual.base_angular_velocity,
              tolerance);
  EXPECT_EQ(expected.singul, actual.singul);
}

}  // namespace

// ################################
TEST(RemainingCandidate, RebasesOnlyTheOriginalSuffix) {
  const FrozenCandidate original = twoSegmentCandidate();
  const WholeBodySample expected = original->sample(1.25);
  const auto suffix = RemainingCandidate::slice(original, 1.25);
  ASSERT_TRUE(suffix.valid) << suffix.error_code << " " << suffix.detail;
  EXPECT_DOUBLE_EQ(original->duration() - 1.25, suffix.candidate->duration());
  expectWholeBodyNear(expected, suffix.candidate->sample(0.0), 1e-12);
  EXPECT_EQ(expected.singul, suffix.candidate->sample(0.0).singul);
  EXPECT_EQ(original->sample(2.0).singul, suffix.candidate->sample(0.75).singul);
}

TEST(RemainingCandidate, RejectsOutOfRange) {
  const FrozenCandidate original = twoSegmentCandidate();
  EXPECT_FALSE(RemainingCandidate::slice(original, -0.1).valid);
  EXPECT_FALSE(RemainingCandidate::slice(original, 2.0).valid);
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
