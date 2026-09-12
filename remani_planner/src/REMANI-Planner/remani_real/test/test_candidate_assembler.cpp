#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <quadrotor_msgs/PolynomialTraj.h>

#include <remani_real/candidate_assembler.hpp>

namespace remani_real {
namespace {

// ################################
// C++: CandidateAssembler test fixtures begin
// ################################
ros::SteadyTime steadyAt(double sec) {
  ros::SteadyTime value;
  value.fromSec(sec);
  return value;
}

quadrotor_msgs::PolynomialTraj controlMessage(uint32_t action) {
  quadrotor_msgs::PolynomialTraj msg;
  msg.header.stamp = ros::Time(10.0);
  msg.action = action;
  msg.trajectory_id = 0;
  msg.singul = 0;
  return msg;
}

quadrotor_msgs::PolynomialTraj validAdd(uint32_t id) {
  quadrotor_msgs::PolynomialTraj msg;
  msg.header.stamp = ros::Time(10.0);
  msg.action = quadrotor_msgs::PolynomialTraj::ACTION_ADD;
  msg.trajectory_id = id;
  msg.singul = 1;
  quadrotor_msgs::PolynomialMatrix piece;
  piece.num_dim = 8;
  piece.num_order = 7;
  piece.duration = 1.0;
  piece.data.assign(64, 0.0);
  piece.data[56] = 0.1 * static_cast<double>(id - 1);
  piece.data[48] = 0.1;
  msg.trajectory.push_back(piece);
  return msg;
}
// ################################
// C++: CandidateAssembler test fixtures end
// ################################

TEST(CandidateAssembler, AssignsCandidateIdOnEachAcceptedStart) {
  CandidateAssembler gate(60.0);
  const AssemblyEvent first = gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
      steadyAt(0), true, 0.0);
  EXPECT_TRUE(first.accepted);
  EXPECT_EQ(1u, first.candidate_id);
  EXPECT_EQ(ros::Time(10.0), first.raw_transaction_stamp);

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_ABORT),
               steadyAt(1), true, 0.0);
  const AssemblyEvent second = gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
      steadyAt(2), true, 0.0);
  EXPECT_TRUE(second.accepted);
  EXPECT_EQ(2u, second.candidate_id);
}

TEST(CandidateAssembler, RequiresStrictContiguousSegmentsAndFinal) {
  CandidateAssembler gate(60.0);
  gate.consume(controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_START), steadyAt(0), true, 0.0);
  EXPECT_TRUE(gate.consume(validAdd(1), steadyAt(1), true, 0.0).accepted);
  EXPECT_FALSE(gate.consume(validAdd(1), steadyAt(2), true, 0.0).accepted);

  gate.consume(controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_START), steadyAt(3), true, 0.0);
  EXPECT_FALSE(gate.consume(validAdd(2), steadyAt(4), true, 0.0).accepted);
}

TEST(CandidateAssembler, FinalIsTheOnlyCompletionBoundary) {
  CandidateAssembler gate(60.0);
  gate.consume(controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_START), steadyAt(0), true, 0.0);
  gate.consume(validAdd(1), steadyAt(1), true, 0.0);
  EXPECT_EQ(nullptr, gate.completedCandidate());
  const AssemblyEvent final_event = gate.consume(controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL),
      steadyAt(2), true, 0.0);
  ASSERT_TRUE(final_event.accepted);
  EXPECT_EQ(ros::Time(10.0), final_event.raw_transaction_stamp);
  ASSERT_NE(nullptr, gate.completedCandidate());
  EXPECT_EQ(1u, gate.completedCandidate()->id());
  EXPECT_EQ(ros::Time(10.0), gate.completedCandidate()->rawTransactionStamp());
  EXPECT_DOUBLE_EQ(1.0, gate.completedCandidate()->duration());
}

TEST(CandidateAssembler, RejectsMalformedAddsAndOutOfOrderControl) {
  CandidateAssembler gate(60.0);
  EXPECT_FALSE(gate.consume(validAdd(1), steadyAt(0), true, 0.0).accepted);

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(1), true, 0.0);

  auto empty_add = validAdd(1);
  empty_add.trajectory.clear();
  EXPECT_FALSE(gate.consume(empty_add, steadyAt(2), true, 0.0).accepted);

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(3), true, 0.0);
  auto nan_add = validAdd(1);
  nan_add.trajectory[0].data[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gate.consume(nan_add, steadyAt(4), true, 0.0).accepted);

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(5), true, 0.0);
  auto zero_duration = validAdd(1);
  zero_duration.trajectory[0].duration = 0.0;
  EXPECT_FALSE(gate.consume(zero_duration, steadyAt(6), true, 0.0).accepted);

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(7), true, 0.0);
  auto wrong_dim = validAdd(1);
  wrong_dim.trajectory[0].num_dim = 7;
  EXPECT_FALSE(gate.consume(wrong_dim, steadyAt(8), true, 0.0).accepted);

  EXPECT_FALSE(gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL),
      steadyAt(9), true, 0.0).accepted);
}

TEST(CandidateAssembler, AbortImpossibleTimeoutAndPermission) {
  CandidateAssembler gate(60.0);
  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(0), true, 0.0);
  gate.consume(validAdd(1), steadyAt(1), true, 0.0);
  ASSERT_TRUE(gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL),
      steadyAt(2), true, 0.0).accepted);
  ASSERT_NE(nullptr, gate.completedCandidate());

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_ABORT),
               steadyAt(3), true, 0.0);
  EXPECT_EQ(nullptr, gate.completedCandidate());

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(4), true, 0.0);
  gate.consume(validAdd(1), steadyAt(5), true, 0.0);
  gate.consume(controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_IMPOSSIBLE),
               steadyAt(6), true, 0.0);
  EXPECT_EQ(nullptr, gate.completedCandidate());

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(10), true, 0.0);
  const AssemblyEvent timed_out = gate.consume(
      validAdd(1), steadyAt(71), true, 0.0);
  EXPECT_FALSE(timed_out.accepted);
  EXPECT_EQ(AssemblyState::Invalid, timed_out.state);
  EXPECT_EQ("ASSEMBLY_TIMEOUT", timed_out.error_code);

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(100), true, 0.0);
  gate.consume(validAdd(1), steadyAt(101), true, 0.0);
  ASSERT_TRUE(gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL),
      steadyAt(102), true, 0.0).accepted);
  const uint64_t frozen_id = gate.completedCandidate()->id();
  const AssemblyEvent rejected_start = gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
      steadyAt(103), false, 0.0);
  EXPECT_FALSE(rejected_start.accepted);
  ASSERT_NE(nullptr, gate.completedCandidate());
  EXPECT_EQ(frozen_id, gate.completedCandidate()->id());
}

TEST(CandidateAssembler, NewStartInvalidatesPlannedWithoutRestoring) {
  CandidateAssembler gate(60.0);
  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(0), true, 0.0);
  gate.consume(validAdd(1), steadyAt(1), true, 0.0);
  ASSERT_TRUE(gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL),
      steadyAt(2), true, 0.0).accepted);
  ASSERT_NE(nullptr, gate.completedCandidate());

  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(3), true, 0.0);
  EXPECT_EQ(nullptr, gate.completedCandidate());
  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_ABORT),
               steadyAt(4), true, 0.0);
  EXPECT_EQ(nullptr, gate.completedCandidate());
}

// ################################
// C++: Regression for timeout recovery START and unknown-action freeze safety.
// ################################
TEST(CandidateAssembler, TimedOutConsumeStillAcceptsRecoveryStart) {
  CandidateAssembler gate(60.0);
  ASSERT_TRUE(gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
      steadyAt(0), true, 0.0).accepted);
  ASSERT_TRUE(gate.consume(validAdd(1), steadyAt(1), true, 0.0).accepted);

  const AssemblyEvent recovered = gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
      steadyAt(70), true, 0.25);
  EXPECT_TRUE(recovered.accepted);
  EXPECT_EQ(AssemblyState::Assembling, recovered.state);
  EXPECT_EQ(2u, recovered.candidate_id);
  EXPECT_TRUE(gate.consume(validAdd(1), steadyAt(71), true, 0.25).accepted);
  ASSERT_TRUE(gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL),
      steadyAt(72), true, 0.25).accepted);
  ASSERT_NE(nullptr, gate.completedCandidate());
  EXPECT_EQ(2u, gate.completedCandidate()->id());
}

TEST(CandidateAssembler, UnknownActionDoesNotClearFrozenCandidate) {
  CandidateAssembler gate(60.0);
  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(0), true, 0.0);
  gate.consume(validAdd(1), steadyAt(1), true, 0.0);
  ASSERT_TRUE(gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL),
      steadyAt(2), true, 0.0).accepted);
  ASSERT_NE(nullptr, gate.completedCandidate());
  const uint64_t frozen_id = gate.completedCandidate()->id();

  quadrotor_msgs::PolynomialTraj garbage = controlMessage(99u);
  const AssemblyEvent rejected =
      gate.consume(garbage, steadyAt(3), true, 0.0);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ("UNKNOWN_ACTION", rejected.error_code);
  ASSERT_NE(nullptr, gate.completedCandidate());
  EXPECT_EQ(frozen_id, gate.completedCandidate()->id());
}

TEST(CandidateAssembler, RejectsMalformedControlMessages) {
  CandidateAssembler gate(60.0);
  auto bad_start = controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_START);
  bad_start.trajectory_id = 7;
  EXPECT_FALSE(gate.consume(bad_start, steadyAt(0), true, 0.0).accepted);

  auto payload_abort = controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_ABORT);
  payload_abort.trajectory.push_back(quadrotor_msgs::PolynomialMatrix());
  EXPECT_FALSE(gate.consume(payload_abort, steadyAt(1), true, 0.0).accepted);
}

// ################################
TEST(CandidateAssembler, PollTimeoutWithoutCandidateMessage) {
  CandidateAssembler gate(60.0);
  EXPECT_TRUE(gate.pollTimeout(steadyAt(0)).error_code.empty());
  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
               steadyAt(0), true, 0.0);
  gate.consume(validAdd(1), steadyAt(1), true, 0.0);
  EXPECT_TRUE(gate.pollTimeout(steadyAt(30)).error_code.empty());
  const AssemblyEvent timed_out = gate.pollTimeout(steadyAt(61));
  EXPECT_FALSE(timed_out.accepted);
  EXPECT_EQ(AssemblyState::Invalid, timed_out.state);
  EXPECT_EQ("ASSEMBLY_TIMEOUT", timed_out.error_code);
  EXPECT_TRUE(gate.pollTimeout(steadyAt(62)).error_code.empty());
}
// ################################

}  // namespace
}  // namespace remani_real

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
