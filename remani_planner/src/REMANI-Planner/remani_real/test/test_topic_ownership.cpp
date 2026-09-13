#include <gtest/gtest.h>

#include <remani_real/topic_ownership_monitor.hpp>

using remani_real::IsUniqueOwner;

// ################################
TEST(TopicOwnership, RequiresExactlyTheExpectedPublisher) {
  EXPECT_TRUE(IsUniqueOwner({"/remani_real_node"}, "/remani_real_node"));
  EXPECT_FALSE(IsUniqueOwner({}, "/remani_real_node"));
  EXPECT_FALSE(IsUniqueOwner({"/remani_real_node", "/rogue_teleop"},
                             "/remani_real_node"));
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
