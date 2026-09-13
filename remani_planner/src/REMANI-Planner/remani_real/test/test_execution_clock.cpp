#include <gtest/gtest.h>

#include <remani_real/execution_clock.hpp>

using remani_real::ExecutionClock;
using remani_real::ExecutionEpoch;

namespace {

ros::SteadyTime steadyAt(double sec) {
  ros::SteadyTime value;
  value.fromSec(sec);
  return value;
}

}  // namespace

// ################################
TEST(ExecutionClock, MapsOneRequestedT0ToCandidateTime) {
  const ExecutionEpoch epoch = ExecutionClock::begin(steadyAt(100.0), 1.0);
  EXPECT_DOUBLE_EQ(101.0, epoch.t0.toSec());
  EXPECT_FALSE(
      ExecutionClock::parameterTime(epoch, steadyAt(100.9)).is_initialized());
  EXPECT_DOUBLE_EQ(
      0.0, ExecutionClock::parameterTime(epoch, steadyAt(101.0)).get());
  EXPECT_DOUBLE_EQ(
      0.25, ExecutionClock::parameterTime(epoch, steadyAt(101.25)).get());
}

TEST(ExecutionClock, RejectsNonPositiveLead) {
  EXPECT_THROW(ExecutionClock::begin(steadyAt(1.0), 0.0),
               std::invalid_argument);
  EXPECT_THROW(ExecutionClock::begin(steadyAt(1.0), -1.0),
               std::invalid_argument);
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
