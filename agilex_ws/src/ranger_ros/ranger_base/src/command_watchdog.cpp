#include <ranger_base/command_watchdog.hpp>

#include <cmath>

namespace westonrobot {

// ################################
// C++: CommandWatchdog implementation begin
// ################################
CommandWatchdog::CommandWatchdog(double timeout_sec)
    : timeout_sec_(timeout_sec) {
  if (!(timeout_sec_ > 0.0) || !std::isfinite(timeout_sec_)) {
    timeout_sec_ = 0.20;
  }
}

void CommandWatchdog::arm(double steady_now_sec) {
  armed_ = true;
  timed_out_ = false;
  stop_latched_ = false;
  last_command_sec_ = steady_now_sec;
}

void CommandWatchdog::noteCommand(double steady_now_sec) {
  if (!armed_) {
    return;
  }
  last_command_sec_ = steady_now_sec;
  timed_out_ = false;
  stop_latched_ = false;
}

WatchdogUpdate CommandWatchdog::update(double steady_now_sec) {
  WatchdogUpdate out;
  if (!armed_ || !(timeout_sec_ > 0.0)) {
    return out;
  }

  const double age = steady_now_sec - last_command_sec_;
  if (age > timeout_sec_) {
    out.timed_out = true;
    timed_out_ = true;
    if (!stop_latched_) {
      stop_latched_ = true;
      out.request_stop = true;
    }
  }
  return out;
}

bool CommandWatchdog::timedOut() const { return timed_out_; }
// ################################
// C++: CommandWatchdog implementation end
// ################################

}  // namespace westonrobot
