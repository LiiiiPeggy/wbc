#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <remani_real/arm_trajectory_builder.hpp>
#include <remani_real/candidate_trajectory.hpp>

using remani_real::ArmTrajectoryBuilder;
using remani_real::CandidateSegment;
using remani_real::CandidateTrajectory;
using remani_real::FrozenCandidate;

namespace {

// ################################
Eigen::Matrix<double, 6, 1> currentQ() {
  Eigen::Matrix<double, 6, 1> q;
  q << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60;
  return q;
}

std::vector<double> currentQVector() {
  const auto q = currentQ();
  return std::vector<double>(q.data(), q.data() + q.size());
}

FrozenCandidate armCandidate(double initial_offset) {
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  coeff.col(7).head<2>().setZero();
  coeff.col(7).tail<6>() = (currentQ().array() + initial_offset).matrix();
  coeff(2, 6) = 0.01;
  MMController::Trajectory trajectory;
  trajectory.emplace_back(1.0, coeff);
  CandidateSegment segment;
  segment.trajectory_id = 1;
  segment.singul = 1;
  segment.trajectory = trajectory;
  segment.start_time = 0.0;
  segment.duration = 1.0;
  return std::make_shared<const CandidateTrajectory>(
      9, std::vector<CandidateSegment>{segment}, 0.0);
}
// ################################

}  // namespace

// ################################
TEST(ArmTrajectoryBuilder, AddsOneHoldPointAndStrictTimes) {
  const auto result = ArmTrajectoryBuilder::build(
      armCandidate(0.0), currentQ(), 1.0, 0.10, 0.02);
  ASSERT_TRUE(result.valid);
  const auto& trajectory = result.trajectory;
  ASSERT_GE(trajectory.points.size(), 3u);
  EXPECT_DOUBLE_EQ(0.0, trajectory.points[0].time_from_start.toSec());
  EXPECT_EQ(currentQVector(), trajectory.points[0].positions);
  EXPECT_EQ(std::vector<double>(6, 0.0), trajectory.points[0].velocities);
  EXPECT_DOUBLE_EQ(1.0, trajectory.points[1].time_from_start.toSec());
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    EXPECT_LT(trajectory.points[i - 1].time_from_start,
              trajectory.points[i].time_from_start);
    EXPECT_EQ(6u, trajectory.points[i].positions.size());
    EXPECT_EQ(6u, trajectory.points[i].velocities.size());
  }
}

TEST(ArmTrajectoryBuilder, RejectsLargeHoldHandoffError) {
  EXPECT_EQ("ARM_HOLD_HANDOFF_TOLERANCE",
            ArmTrajectoryBuilder::build(armCandidate(0.03), currentQ(), 1.0,
                                        0.10, 0.02)
                .error_code);
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
