#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <dobot_v4_bringup/trajectory_goal_validator.hpp>

namespace dobot_v4_bringup {

// ################################
// C++: TrajectoryRunner types begin
// ################################
enum class RunnerState {
  Idle,
  Holding,
  Running,
  Settling,
  Canceling,
  Succeeded,
  Canceled,
  Aborted
};

struct RunnerTick {
  RunnerState state{RunnerState::Idle};
  bool has_command{false};
  std::array<double, 6> command_rad{};
  std::array<double, 6> desired_velocity{};
  bool terminal{false};
  std::string error_code;
};

struct RunnerConfig {
  double servoj_period{0.10};
  double goal_joint_tol{0.02};
  double stop_velocity_tol{0.01};
  double settle_timeout{2.0};
  int required_settle_samples{3};
  bool remani_prestart_hold_mode{false};
};

struct RunnerActualState {
  std::array<double, 6> q{};
  std::array<double, 6> qd{};
};

struct StartDecision {
  bool accepted{false};
  std::string error_code;
  std::string detail;
};

class TrajectoryRunner {
 public:
  explicit TrajectoryRunner(RunnerConfig config);

  StartDecision start(const CanonicalTrajectory& trajectory,
                      double steady_start_sec);
  RunnerTick tick(double steady_now_sec, const RunnerActualState& actual);
  void requestCancel();
  std::size_t tickCount() const { return tick_count_; }
  RunnerState state() const { return state_; }

 private:
  void sampleAt(double traj_time, std::array<double, 6>* q,
                std::array<double, 6>* qd) const;
  bool withinGoal(const RunnerActualState& actual) const;
  bool withinStopVelocity(const RunnerActualState& actual) const;

  RunnerConfig config_;
  RunnerState state_{RunnerState::Idle};
  CanonicalTrajectory trajectory_;
  std::vector<double> point_times_;
  double steady_start_sec_{0.0};
  double settle_start_sec_{0.0};
  int settle_good_samples_{0};
  int cancel_good_samples_{0};
  bool cancel_requested_{false};
  bool stop_issued_{false};
  std::size_t tick_count_{0};
  std::array<double, 6> final_q_{};
};
// ################################
// C++: TrajectoryRunner types end
// ################################

}  // namespace dobot_v4_bringup
