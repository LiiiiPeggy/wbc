#include <remani_real/synchronized_executor.hpp>

#include <cmath>
#include <stdexcept>

#include <geometry_msgs/Twist.h>

#include <remani_real/arm_trajectory_builder.hpp>
#include <remani_real/base_tracking_controller.hpp>

namespace remani_real {
namespace {

geometry_msgs::Twist zeroTwist() {
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;
  return cmd;
}

}  // namespace

// ################################
// C++: SynchronizedExecutor implementation begin
// ################################
SynchronizedExecutor::SynchronizedExecutor(SynchronizedExecutorConfig config,
                                           RangerCommandChannel* ranger,
                                           ArmCommandChannel* arm)
    : config_(config), ranger_(ranger), arm_(arm) {
  if (ranger_ == nullptr || arm_ == nullptr) {
    throw std::invalid_argument("SynchronizedExecutor requires ranger and arm");
  }
}

bool SynchronizedExecutor::armAccepted() const {
  const ArmGoalState s = arm_->state();
  return s == ArmGoalState::Accepted || s == ArmGoalState::Active ||
         s == ArmGoalState::Succeeded;
}

bool SynchronizedExecutor::publishRangerZero() {
  if (config_.dry_run) {
    return true;
  }
  return ranger_->publish(zeroTwist());
}

ExecutorStepResult SynchronizedExecutor::fail(const std::string& code,
                                              const std::string& detail) {
  state_ = ExecutorStepState::Error;
  ExecutorStepResult out;
  out.state = state_;
  out.error_code = code;
  out.detail = detail;
  return out;
}

ExecuteDecision SynchronizedExecutor::prepare(
    FrozenCandidate candidate, const ActualStateSnapshot& actual,
    const ros::SteadyTime& request_time) {
  ExecuteDecision out;
  if (prepared_ && state_ != ExecutorStepState::Idle &&
      state_ != ExecutorStepState::Finished &&
      state_ != ExecutorStepState::Error) {
    out.error_code = "EXECUTOR_BUSY";
    return out;
  }
  if (!candidate) {
    out.error_code = "CANDIDATE_NULL";
    return out;
  }
  if (!actual.odom_valid || !actual.joints_valid) {
    out.error_code = "ACTUAL_NOT_READY";
    return out;
  }

  try {
    epoch_ = ExecutionClock::begin(request_time, config_.start_lead_time);
  } catch (const std::exception& ex) {
    out.error_code = "EPOCH_INVALID";
    out.detail = ex.what();
    return out;
  }

  const ArmTrajectoryBuildResult arm_traj = ArmTrajectoryBuilder::build(
      candidate, actual.q, config_.start_lead_time, config_.arm_sample_period,
      config_.arm_hold_tol);
  if (!arm_traj.valid) {
    out.error_code = arm_traj.error_code.empty() ? "ARM_TRAJ_BUILD"
                                                 : arm_traj.error_code;
    out.detail = arm_traj.detail;
    return out;
  }

  timing_ = StartTimingRecord();
  timing_.requested_t0 = epoch_.t0;
  timing_.arm_goal_sent_at = request_time;
  candidate_ = candidate;
  recorded_ranger_start_ = false;

  if (config_.dry_run) {
    timing_.arm_goal_accepted_at = request_time;
  } else {
    if (!arm_->send(arm_traj.trajectory)) {
      out.error_code = "ARM_SEND_FAILED";
      return out;
    }
  }

  prepared_ = true;
  state_ = ExecutorStepState::HoldingForT0;
  out.accepted = true;
  return out;
}

void SynchronizedExecutor::noteCr10FirstMotionSteady(double steady_sec) {
  if (!timing_.cr10_first_motion_at) {
    ros::SteadyTime stamp;
    stamp.fromSec(steady_sec);
    timing_.cr10_first_motion_at = stamp;
    if (timing_.ranger_trajectory_start_at) {
      timing_.start_skew = timing_.cr10_first_motion_at->toSec() -
                           timing_.ranger_trajectory_start_at->toSec();
    }
  }
}

ExecutorStepResult SynchronizedExecutor::tick(
    const ros::SteadyTime& now, const ActualStateSnapshot& actual) {
  ExecutorStepResult out;
  if (!prepared_ || state_ == ExecutorStepState::Idle) {
    return fail("NOT_PREPARED", "call prepare before tick");
  }
  if (state_ == ExecutorStepState::Error ||
      state_ == ExecutorStepState::Finished ||
      state_ == ExecutorStepState::Paused) {
    out.state = state_;
    return out;
  }

  const double now_sec = now.toSec();
  const double t0_sec = epoch_.t0.toSec();
  const double accept_deadline =
      t0_sec - config_.arm_accept_guard;

  if (now_sec < t0_sec) {
    out.state = ExecutorStepState::HoldingForT0;
    state_ = ExecutorStepState::HoldingForT0;
    if (!publishRangerZero()) {
      return fail("RANGER_ZERO_FAILED", "failed to publish pre-T0 zero");
    }

    if (!config_.dry_run) {
      if (armAccepted() && !timing_.arm_goal_accepted_at) {
        timing_.arm_goal_accepted_at = now;
      }
      if (now_sec + 1e-12 >= accept_deadline && !armAccepted()) {
        arm_->cancel();
        publishRangerZero();
        return fail("ARM_ACCEPT_DEADLINE",
                    "CR10 Action not accepted before T0-arm_accept_guard");
      }
    }
    return out;
  }

  // now >= T0
  if (!config_.dry_run && !armAccepted()) {
    arm_->cancel();
    publishRangerZero();
    return fail("ARM_NOT_ACCEPTED_AT_T0",
                "CR10 Action must be accepted at or before T0");
  }
  if (config_.dry_run && !timing_.arm_goal_accepted_at) {
    timing_.arm_goal_accepted_at = epoch_.requested_at;
  }
  if (!config_.dry_run && !timing_.arm_goal_accepted_at && armAccepted()) {
    // Accepted observed at/after T0 still records once; T0 itself is fixed.
    timing_.arm_goal_accepted_at = now;
  }

  const boost::optional<double> param =
      ExecutionClock::parameterTime(epoch_, now);
  if (!param) {
    return fail("PARAM_TIME", "parameter time unavailable at/after T0");
  }
  out.parameter_time = *param;

  if (!recorded_ranger_start_) {
    timing_.ranger_trajectory_start_at = now;
    recorded_ranger_start_ = true;
    if (timing_.cr10_first_motion_at) {
      timing_.start_skew = timing_.cr10_first_motion_at->toSec() -
                           timing_.ranger_trajectory_start_at->toSec();
    }
  }

  WholeBodySample desired;
  try {
    const double sample_t =
        std::min(*param, std::max(0.0, candidate_->duration()));
    desired = candidate_->sample(sample_t);
  } catch (const std::exception& ex) {
    publishRangerZero();
    return fail("SAMPLE_FAIL", ex.what());
  }

  BaseTrackingConfig tracking_config;
  tracking_config.max_linear = 0.10;
  tracking_config.max_angular = 0.15;
  BaseTrackingController tracker(tracking_config);
  const BaseTrackingResult tracked = tracker.compute(desired, actual);
  if (!tracked.valid) {
    publishRangerZero();
    return fail(tracked.error_code.empty() ? "TRACKING_FAIL" : tracked.error_code,
                "base tracking failed");
  }

  if (!config_.dry_run) {
    if (!ranger_->publish(tracked.command)) {
      return fail("RANGER_PUBLISH_FAILED", "failed to publish Ranger command");
    }
  }

  const double speed = std::hypot(tracked.command.linear.x,
                                  tracked.command.angular.z);
  if (!timing_.ranger_first_motion_at &&
      speed > config_.command_zero_epsilon) {
    timing_.ranger_first_motion_at = now;
  }

  if (timing_.start_skew &&
      std::abs(*timing_.start_skew) > config_.max_start_skew) {
    publishRangerZero();
    if (!config_.dry_run) {
      arm_->cancel();
    }
    return fail("START_SKEW", "abs(start_skew) exceeds max_start_skew");
  }

  state_ = ExecutorStepState::Running;
  out.state = state_;

  if (*param + 1e-12 >= candidate_->duration()) {
    state_ = ExecutorStepState::Finished;
    out.state = state_;
  }
  return out;
}
// ################################
// C++: SynchronizedExecutor implementation end
// ################################

}  // namespace remani_real
