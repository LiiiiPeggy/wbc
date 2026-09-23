#include <dobot_v4_bringup/follow_joint_trajectory_adapter.hpp>

#include <cmath>

#include <dobot_v4_bringup/trajectory_goal_validator.hpp>

namespace dobot_v4_bringup {

// ################################
// C++: FollowJointTrajectoryAdapter implementation begin
// ################################
FollowJointTrajectoryAdapter::FollowJointTrajectoryAdapter(Cr10CommandSink* sink,
                                                           RunnerConfig config)
    : sink_(sink), config_(config), runner_(config) {}

bool FollowJointTrajectoryAdapter::hasActiveGoal() const { return active_; }

RunnerActualState FollowJointTrajectoryAdapter::readActual(double steady_now_sec) {
  RunnerActualState actual;
  actual.q = sink_->actualQ();
  actual.qd.fill(0.0);
  if (have_last_q_) {
    const double dt = steady_now_sec - last_q_time_;
    if (dt > 1e-6) {
      for (std::size_t i = 0; i < 6; ++i) {
        actual.qd[i] = (actual.q[i] - last_q_[i]) / dt;
      }
    }
  }
  last_q_ = actual.q;
  last_q_time_ = steady_now_sec;
  have_last_q_ = true;
  return actual;
}

AdapterDecision FollowJointTrajectoryAdapter::accept(
    const trajectory_msgs::JointTrajectory& trajectory, double steady_now_sec) {
  AdapterDecision out;
  if (active_) {
    out.error_code = "GOAL_ACTIVE";
    out.detail = "reject while another goal is active";
    return out;
  }
  const GoalValidationResult validated = TrajectoryGoalValidator::validate(trajectory);
  if (!validated.valid) {
    out.error_code = validated.error_code;
    out.detail = validated.detail;
    return out;
  }
  const StartDecision started =
      runner_.start(validated.trajectory, steady_now_sec);
  if (!started.accepted) {
    out.error_code = started.error_code;
    out.detail = started.detail;
    return out;
  }
  active_ = true;
  stop_called_ = false;
  published_first_non_hold_ = false;
  have_last_q_ = false;
  out.accepted = true;
  out.state = runner_.state();
  return out;
}

AdapterDecision FollowJointTrajectoryAdapter::timerTick(double steady_now_sec) {
  AdapterDecision out;
  if (!active_) {
    out.state = RunnerState::Idle;
    return out;
  }
  const RunnerActualState actual = readActual(steady_now_sec);
  const RunnerTick tick = runner_.tick(steady_now_sec, actual);
  out.state = tick.state;
  out.publish_feedback = true;
  out.error_code = tick.error_code;

  if (tick.has_command) {
    if (!sink_->sendServoJ(tick.command_rad, config_.servoj_period * 1.5)) {
      active_ = false;
      out.terminal = true;
      out.state = RunnerState::Aborted;
      out.error_code = "SERVOJ_FAILED";
      return out;
    }
    if (!published_first_non_hold_ && tick.state == RunnerState::Running) {
      published_first_non_hold_ = true;
      out.first_non_hold_servoj = true;
      out.first_non_hold_steady_sec = steady_now_sec;
    }
  } else if (tick.state == RunnerState::Canceling && !stop_called_) {
    stop_called_ = true;
    if (!sink_->stop()) {
      active_ = false;
      out.terminal = true;
      out.state = RunnerState::Aborted;
      out.error_code = "STOP_FAILED";
      return out;
    }
  }

  if (tick.terminal) {
    active_ = false;
    out.terminal = true;
    out.state = tick.state;
  }
  return out;
}

AdapterDecision FollowJointTrajectoryAdapter::requestCancel(double steady_now_sec) {
  AdapterDecision out;
  if (!active_) {
    out.error_code = "NO_ACTIVE_GOAL";
    return out;
  }
  runner_.requestCancel();
  out.state = RunnerState::Canceling;
  if (!stop_called_) {
    stop_called_ = true;
    if (!sink_->stop()) {
      active_ = false;
      out.terminal = true;
      out.state = RunnerState::Aborted;
      out.error_code = "STOP_FAILED";
      return out;
    }
  }
  // Do not send ServoJ on cancel path.
  (void)steady_now_sec;
  return out;
}
// ################################
// C++: FollowJointTrajectoryAdapter implementation end
// ################################

}  // namespace dobot_v4_bringup
