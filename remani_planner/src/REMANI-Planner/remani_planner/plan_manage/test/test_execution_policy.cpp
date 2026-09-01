#include <gtest/gtest.h>

#include <stdexcept>

#include "plan_manage/execution_policy.hpp"

namespace remani_planner {
namespace {

TEST(ExecutionPolicy, AcceptsOnlyLockedCombinations) {
  EXPECT_NO_THROW(ExecutionPolicy::fromStrings("sim", "internal"));
  EXPECT_NO_THROW(ExecutionPolicy::fromStrings("real", "external"));
  EXPECT_THROW(ExecutionPolicy::fromStrings("real", "internal"),
               std::invalid_argument);
  EXPECT_THROW(ExecutionPolicy::fromStrings("sim", "external"),
               std::invalid_argument);
  EXPECT_THROW(ExecutionPolicy::fromStrings("hardware", "external"),
               std::invalid_argument);
  EXPECT_THROW(ExecutionPolicy::fromStrings("", "internal"),
               std::invalid_argument);
  EXPECT_THROW(ExecutionPolicy::fromStrings("Sim", "internal"),
               std::invalid_argument);
  EXPECT_THROW(ExecutionPolicy::fromStrings("sim ", "internal"),
               std::invalid_argument);
}

TEST(ExecutionPolicy, DefaultsRemainSimulationInternal) {
  const auto policy = ExecutionPolicy::fromStrings("sim", "internal");

  EXPECT_TRUE(policy.ownsInternalExecution());
  EXPECT_FALSE(policy.isRealPlanOnly());
}

}  // namespace
}  // namespace remani_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
