#pragma once

#include <cmath>
#include <string>

#include <boost/optional.hpp>

#include <remani_real/actual_state.hpp>
#include <remani_real/candidate_trajectory.hpp>
#include <remani_real/execution_clock.hpp>
#include <remani_real/motion_output.hpp>

namespace remani_real {

// ################################
// C++: SynchronizedExecutor types begin
// ################################
struct StartTimingRecord {
  ros::SteadyTime requested_t0;
  ros::SteadyTime arm_goal_sent_at;
  boost::optional<ros::SteadyTime> arm_goal_accepted_at;
  boost::optional<ros::SteadyTime> ranger_trajectory_start_at;
  boost::optional<ros::SteadyTime> ranger_first_motion_at;
  boost::optional<ros::SteadyTime> cr10_first_motion_at;
  double ranger_start_error{0.0};
  double cr10_start_error{0.0};
  boost::optional<double> start_skew;
};

struct PauseRecord {
  double pause_param_time{0.0};
  WholeBodySample expected_pause_state;
  ActualStateSnapshot actual_stop_state;
  bool ranger_stopped{false};
  bool cr10_stopped{false};
};

struct ResumeTolerance {
  double base_position{0.02};
  double base_yaw{2.0 * M_PI / 180.0};
  double arm_joint{1.0 * M_PI / 180.0};
};

enum class ExecutorStepState {
  Idle,
  HoldingForT0,
  Running,
  Stopping,
  Paused,
  Finished,
  Error
};

struct ExecutorStepResult {
  ExecutorStepState state{ExecutorStepState::Idle};
  double parameter_time{0.0};
  std::string error_code;
  std::string detail;
};

struct ExecuteDecision {
  bool accepted{false};
  std::string error_code;
  std::string detail;
};

struct SynchronizedExecutorConfig {
  bool dry_run{true};
  double start_lead_time{1.0};
  double arm_accept_guard{0.20};
  double max_start_skew{0.10};
  double arm_sample_period{0.10};
  double arm_hold_tol{0.02};
  double command_zero_epsilon{1e-4};
  double stop_base_speed{1e-3};
  double stop_joint_speed{1e-3};
  ResumeTolerance resume_tol;
};

class SynchronizedExecutor {
 public:
  SynchronizedExecutor(SynchronizedExecutorConfig config,
                       RangerCommandChannel* ranger, ArmCommandChannel* arm);

  ExecuteDecision prepare(FrozenCandidate candidate,
                          const ActualStateSnapshot& actual,
                          const ros::SteadyTime& request_time);
  ExecutorStepResult tick(const ros::SteadyTime& now,
                          const ActualStateSnapshot& actual);
  ExecuteDecision requestPause();
  ExecuteDecision requestResume(const ActualStateSnapshot& actual,
                                const ros::SteadyTime& request_time);
  ExecuteDecision requestAbort(const ActualStateSnapshot& actual);

  const StartTimingRecord& timing() const { return timing_; }
  const PauseRecord& pauseRecord() const { return pause_record_; }
  ExecutorStepState state() const { return state_; }
  const ExecutionEpoch& epoch() const { return epoch_; }
  std::size_t generatedConnectorCount() const { return 0; }
  uint64_t executionGeneration() const { return execution_generation_; }

  void noteCr10FirstMotionSteady(double steady_sec);

 private:
  bool armAccepted() const;
  bool publishRangerZero();
  bool devicesStopped(const ActualStateSnapshot& actual) const;
  ExecuteDecision checkResumeTolerance(const ActualStateSnapshot& actual) const;
  ExecutorStepResult fail(const std::string& code, const std::string& detail);

  SynchronizedExecutorConfig config_;
  RangerCommandChannel* ranger_;
  ArmCommandChannel* arm_;
  FrozenCandidate original_candidate_;
  FrozenCandidate active_candidate_;
  ExecutionEpoch epoch_;
  StartTimingRecord timing_;
  PauseRecord pause_record_;
  ExecutorStepState state_{ExecutorStepState::Idle};
  bool prepared_{false};
  bool recorded_ranger_start_{false};
  bool pause_arm_cancel_issued_{false};
  bool pause_arm_stop_issued_{false};
  uint64_t execution_generation_{0};
  double last_parameter_time_{0.0};
  bool have_last_parameter_time_{false};
};
// ################################
// C++: SynchronizedExecutor types end
// ################################

}  // namespace remani_real
