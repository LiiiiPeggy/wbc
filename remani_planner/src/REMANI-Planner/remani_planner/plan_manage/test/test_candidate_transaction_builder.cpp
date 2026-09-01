#include <gtest/gtest.h>

#include <stdexcept>

#include "plan_manage/candidate_transaction_builder.hpp"

namespace remani_planner {
namespace {

SingulTrajData oneSegmentTrajectory() {
  poly_traj::Piece<7>::CoefficientMat coeff;
  coeff.setZero();
  coeff.col(7) << 0.0, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  poly_traj::Trajectory<7> traj;
  traj.emplace_back(1.25, coeff, 1);

  SingulTrajData data;
  data.addSingulTraj(traj, 10.0);
  return data;
}

TEST(CandidateTransactionBuilder, ExternalWrapsContiguousAdds) {
  const auto msgs = CandidateTransactionBuilder::buildExternal(
      oneSegmentTrajectory(), ros::Time(10.0));

  ASSERT_EQ(3u, msgs.size());
  EXPECT_EQ(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START, msgs[0].action);
  EXPECT_EQ(0u, msgs[0].trajectory_id);
  EXPECT_EQ(10.0, msgs[0].header.stamp.toSec());
  EXPECT_EQ(0, msgs[0].singul);
  EXPECT_TRUE(msgs[0].trajectory.empty());

  EXPECT_EQ(quadrotor_msgs::PolynomialTraj::ACTION_ADD, msgs[1].action);
  EXPECT_EQ(1u, msgs[1].trajectory_id);
  EXPECT_EQ(10.0, msgs[1].header.stamp.toSec());
  EXPECT_EQ(1, msgs[1].singul);
  ASSERT_EQ(1u, msgs[1].trajectory.size());
  EXPECT_EQ(8u, msgs[1].trajectory[0].num_dim);
  EXPECT_EQ(7u, msgs[1].trajectory[0].num_order);
  EXPECT_DOUBLE_EQ(1.25, msgs[1].trajectory[0].duration);
  ASSERT_EQ(64u, msgs[1].trajectory[0].data.size());
  EXPECT_DOUBLE_EQ(0.0, msgs[1].trajectory[0].data[0]);
  EXPECT_DOUBLE_EQ(0.1, msgs[1].trajectory[0].data[58]);

  EXPECT_EQ(quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL, msgs[2].action);
  EXPECT_EQ(0u, msgs[2].trajectory_id);
  EXPECT_EQ(10.0, msgs[2].header.stamp.toSec());
  EXPECT_EQ(0, msgs[2].singul);
  EXPECT_TRUE(msgs[2].trajectory.empty());
}

TEST(CandidateTransactionBuilder, InternalRemainsAddOnly) {
  const auto msgs = CandidateTransactionBuilder::buildInternalAdds(
      oneSegmentTrajectory(), ros::Time(10.0));

  ASSERT_EQ(1u, msgs.size());
  EXPECT_EQ(quadrotor_msgs::PolynomialTraj::ACTION_ADD, msgs[0].action);
  EXPECT_EQ(1u, msgs[0].trajectory_id);
  EXPECT_EQ(10.0, msgs[0].header.stamp.toSec());
  EXPECT_EQ(1, msgs[0].singul);
  ASSERT_EQ(1u, msgs[0].trajectory.size());
  EXPECT_DOUBLE_EQ(0.1, msgs[0].trajectory[0].data[58]);
}

TEST(CandidateTransactionBuilder, ExternalRejectsAnEmptyTransaction) {
  SingulTrajData data;

  EXPECT_THROW(CandidateTransactionBuilder::buildExternal(data, ros::Time(10.0)),
               std::invalid_argument);
}

TEST(CandidateTransactionBuilder, ControlMessageHasNoTrajectoryPayload) {
  const auto msg = CandidateTransactionBuilder::buildControl(
      quadrotor_msgs::PolynomialTraj::ACTION_ABORT, ros::Time(10.0));

  EXPECT_EQ(quadrotor_msgs::PolynomialTraj::ACTION_ABORT, msg.action);
  EXPECT_EQ(0u, msg.trajectory_id);
  EXPECT_EQ(10.0, msg.header.stamp.toSec());
  EXPECT_EQ(0, msg.singul);
  EXPECT_TRUE(msg.trajectory.empty());
}

}  // namespace
}  // namespace remani_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
