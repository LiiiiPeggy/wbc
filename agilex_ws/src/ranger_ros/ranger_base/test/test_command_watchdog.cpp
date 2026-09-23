#include <gtest/gtest.h>

#include <ranger_base/command_watchdog.hpp>

using westonrobot::CommandWatchdog;

// ################################
TEST(CommandWatchdog, TimesOutOnceUntilNewCommand) {
  CommandWatchdog watchdog(0.20);
  watchdog.arm(1.0);
  EXPECT_FALSE(watchdog.update(1.19).request_stop);
  EXPECT_TRUE(watchdog.update(1.21).request_stop);
  EXPECT_FALSE(watchdog.update(1.22).request_stop);
  EXPECT_TRUE(watchdog.timedOut());
  watchdog.noteCommand(1.23);
  EXPECT_FALSE(watchdog.timedOut());
  EXPECT_FALSE(watchdog.update(1.30).request_stop);
}

TEST(CommandWatchdog, InvalidTimeoutFallsBackToDefault) {
  CommandWatchdog watchdog(-1.0);
  watchdog.arm(0.0);
  EXPECT_FALSE(watchdog.update(0.19).request_stop);
  EXPECT_TRUE(watchdog.update(0.21).request_stop);
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
