#pragma once

namespace westonrobot {

// ################################
// C++: steady-clock command watchdog types begin
// ################################
struct WatchdogUpdate {
  bool request_stop{false};
  bool timed_out{false};
};

// Pure monotonic timeout state. Callers pass steady seconds at the boundary.
class CommandWatchdog {
 public:
  explicit CommandWatchdog(double timeout_sec);

  void arm(double steady_now_sec);
  void noteCommand(double steady_now_sec);
  WatchdogUpdate update(double steady_now_sec);
  bool timedOut() const;

 private:
  double timeout_sec_{0.20};
  bool armed_{false};
  bool timed_out_{false};
  bool stop_latched_{false};
  double last_command_sec_{0.0};
};
// ################################
// C++: steady-clock command watchdog types end
// ################################

}  // namespace westonrobot
