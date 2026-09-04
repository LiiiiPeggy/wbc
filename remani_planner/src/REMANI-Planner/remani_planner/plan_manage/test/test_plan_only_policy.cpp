#include <gtest/gtest.h>

#include "plan_manage/remani_replan_fsm.h"

namespace remani_planner {
namespace {

TEST(PlanOnlyPolicy, RealSuccessHandsOffInsteadOfExecuting) {
  EXPECT_EQ(PlanSuccessDisposition::ExternalHandoff,
            planSuccessDisposition(
                ExecutionPolicy::fromStrings("real", "external")));
}

TEST(PlanOnlyPolicy, SimulationSuccessKeepsInternalExecution) {
  EXPECT_EQ(PlanSuccessDisposition::EnterInternalExec,
            planSuccessDisposition(
                ExecutionPolicy::fromStrings("sim", "internal")));
}

}  // namespace
}  // namespace remani_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
