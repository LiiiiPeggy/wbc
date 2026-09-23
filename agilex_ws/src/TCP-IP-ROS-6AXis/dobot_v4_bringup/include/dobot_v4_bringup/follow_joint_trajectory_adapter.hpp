#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>

#include <trajectory_msgs/JointTrajectory.h>

#include <dobot_v4_bringup/trajectory_runner.hpp>

namespace dobot_v4_bringup {

// ################################
// C++: FollowJointTrajectoryAdapter types begin
// ################################
class Cr10CommandSink {
 public:
  virtual ~Cr10CommandSink() = default;
  virtual bool sendServoJ(const std::array<double, 6>& q_rad,
                          double duration_sec) = 0;
  virtual bool stop() = 0;
  virtual std::array<double, 6> actualQ() = 0;
};

struct AdapterDecision {
  bool accepted{false};
  RunnerState state{RunnerState::Idle};
  bool publish_feedback{false};
  bool terminal{false};
  bool first_non_hold_servoj{false};
  double first_non_hold_steady_sec{0.0};
  std::string error_code;
  std::string detail;
};

class FollowJointTrajectoryAdapter {
 public:
  FollowJointTrajectoryAdapter(Cr10CommandSink* sink, RunnerConfig config);

  AdapterDecision accept(const trajectory_msgs::JointTrajectory& trajectory,
                         double steady_now_sec);
  AdapterDecision timerTick(double steady_now_sec);
  AdapterDecision requestCancel(double steady_now_sec);
  bool hasActiveGoal() const;

 private:
  RunnerActualState readActual(double steady_now_sec);

  Cr10CommandSink* sink_;
  RunnerConfig config_;
  TrajectoryRunner runner_;
  bool active_{false};
  bool stop_called_{false};
  bool published_first_non_hold_{false};
  double last_q_time_{0.0};
  std::array<double, 6> last_q_{};
  bool have_last_q_{false};
};
// ################################
// C++: FollowJointTrajectoryAdapter types end
// ################################

}  // namespace dobot_v4_bringup
