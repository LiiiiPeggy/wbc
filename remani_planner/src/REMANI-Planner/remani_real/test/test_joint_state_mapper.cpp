#include <array>
#include <limits>

#include <gtest/gtest.h>
#include <sensor_msgs/JointState.h>

#include <remani_real/joint_state_mapper.hpp>

namespace remani_real {
namespace {

/* ---------- Build a shuffled but complete raw CR10 JointState fixture. ---------- */
sensor_msgs::JointState validRawJointState() {
  sensor_msgs::JointState raw;
  raw.header.stamp = ros::Time(1.0);
  raw.name = {"joint3", "joint1", "joint6", "joint2", "joint5", "joint4"};
  raw.position = {3, 1, 6, 2, 5, 4};
  return raw;
}

TEST(JointStateMapper, ReordersRawNamesToCr10ModelOrder) {
  const sensor_msgs::JointState raw = validRawJointState();
  const auto mapped = JointStateMapper::map(raw);
  ASSERT_TRUE(mapped.valid);
  EXPECT_EQ((std::array<double, 6>{1, 2, 3, 4, 5, 6}), mapped.position);
  ASSERT_EQ(6u, mapped.planning_msg.name.size());
  EXPECT_EQ("cr10_joint1", mapped.planning_msg.name[0]);
  EXPECT_EQ("cr10_joint6", mapped.planning_msg.name[5]);
  ASSERT_EQ(6u, mapped.planning_msg.position.size());
  EXPECT_DOUBLE_EQ(1.0, mapped.planning_msg.position[0]);
  EXPECT_DOUBLE_EQ(6.0, mapped.planning_msg.position[5]);
}

TEST(JointStateMapper, RejectsMissingDuplicateAndNonFiniteNames) {
  auto missing = validRawJointState();
  missing.name.pop_back();
  missing.position.pop_back();
  auto duplicate = validRawJointState();
  duplicate.name[0] = "joint1";
  auto nonfinite = validRawJointState();
  nonfinite.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(JointStateMapper::map(missing).valid);
  EXPECT_FALSE(JointStateMapper::map(duplicate).valid);
  EXPECT_FALSE(JointStateMapper::map(nonfinite).valid);
}

}  // namespace
}  // namespace remani_real

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
