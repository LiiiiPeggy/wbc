#include <remani_real/synchronized_executor.hpp>

#include <cmath>
#include <stdexcept>

#include <geometry_msgs/Twist.h>

#include <remani_real/arm_trajectory_builder.hpp>
#include <remani_real/base_tracking_controller.hpp>
#include <remani_real/remaining_candidate.hpp>

namespace remani_real {
namespace {

geometry_msgs::Twist zeroTwist() {
  geometry_msgs::Twist cmd;
  return cmd;
}

double yawAbsError(double a, double b) {
  double err = a - b;
  while (err > M_PI) {
    err -= 2.0 * M_PI;
  }
  while (err < -M_PI) {
    err += 2.0 * M_PI;
  }
  return std::abs(err);
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

bool SynchronizedExecutor::devicesStopped(
    const ActualStateSnapshot& actual) const {
  const double base_speed =
      std::hypot(actual.base_velocity_world.x(), actual.base_velocity_world.y());
  const double joint_speed = actual.qd.cwiseAbs().maxCoeff();
  return base_speed <= config_.stop_base_speed &&
         joint_speed <= config_.stop_joint_speed;
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
      state_ != ExecutorStepState::Error &&
      state_ != ExecutorStepState::Paused) {
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
  if (!original_candidate_) {
    original_candidate_ = candidate;
  }
  active_candidate_ = candidate;
  recorded_ranger_start_ = false;
  pause_arm_cancel_issued_ = false;
  pause_arm_stop_issued_ = false;
  have_last_parameter_time_ = false;
  last_parameter_time_ = 0.0;
  ++execution_generation_;

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

ExecuteDecision SynchronizedExecutor::requestPause() {
  ExecuteDecision out;
  if (state_ != ExecutorStepState::Running &&
      state_ != ExecutorStepState::HoldingForT0) {
    out.error_code = "PAUSE_NOT_ALLOWED";
    return out;
  }
  pause_record_ = PauseRecord();
  if (have_last_parameter_time_) {
    pause_record_.pause_param_time = last_parameter_time_;
  } else {
    pause_record_.pause_param_time = 0.0;
  }
  try {
    pause_record_.expected_pause_state = original_candidate_->sample(std::min(
        pause_record_.pause_param_time, original_candidate_->duration()));
  } catch (const std::exception& ex) {
    out.error_code = "PAUSE_SAMPLE";
    out.detail = ex.what();
    return out;
  }
  state_ = ExecutorStepState::Stopping;
  out.accepted = true;
  return out;
}

ExecuteDecision SynchronizedExecutor::checkResumeTolerance(
    const ActualStateSnapshot& actual) const {
  ExecuteDecision out;
  const WholeBodySample& expected = pause_record_.expected_pause_state;
  const double base_err =
      (actual.base_xy - expected.position.head<2>()).norm();
  if (base_err > config_.resume_tol.base_position) {
    out.error_code = "RESUME_BASE_POSITION_TOLERANCE";
    return out;
  }
  if (yawAbsError(actual.base_yaw, expected.base_yaw) >
      config_.resume_tol.base_yaw) {
    out.error_code = "RESUME_BASE_YAW_TOLERANCE";
    return out;
  }
  double joint_err = 0.0;
  for (int i = 0; i < 6; ++i) {
    joint_err = std::max(joint_err, std::abs(actual.q(i) - expected.position(2 + i)));
  }
  if (joint_err > config_.resume_tol.arm_joint) {
    out.error_code = "RESUME_ARM_JOINT_TOLERANCE";
    return out;
  }
  out.accepted = true;
  return out;
}

ExecuteDecision SynchronizedExecutor::requestResume(
    const ActualStateSnapshot& actual, const ros::SteadyTime& request_time) {
  ExecuteDecision out;
  if (state_ != ExecutorStepState::Paused) {
    out.error_code = "RESUME_NOT_PAUSED";
    return out;
  }
  out = checkResumeTolerance(actual);
  if (!out.accepted) {
    return out;
  }
  const RemainingCandidateResult suffix = RemainingCandidate::slice(
      original_candidate_, pause_record_.pause_param_time);
  if (!suffix.valid) {
    out.accepted = false;
    out.error_code = suffix.error_code.empty() ? "RESUME_SLICE" : suffix.error_code;
    out.detail = suffix.detail;
    return out;
  }
  // Retain original candidate_id via suffix candidate (slice keeps id()).
  return prepare(suffix.candidate, actual, request_time);
}

ExecuteDecision SynchronizedExecutor::requestAbort(
    const ActualStateSnapshot& actual) {
  ExecuteDecision out;
  publishRangerZero();
  if (!config_.dry_run) {
    arm_->cancel();
    arm_->stop();
  }
  prepared_ = false;
  active_candidate_.reset();
  original_candidate_.reset();
  state_ = ExecutorStepState::Idle;
  out.accepted = true;
  (void)actual;
  return out;
}

ExecutorStepResult SynchronizedExecutor::tick(
    const ros::SteadyTime& now, const ActualStateSnapshot& actual) {
  ExecutorStepResult out;
  if (!prepared_ && state_ != ExecutorStepState::Stopping &&
      state_ != ExecutorStepState::Paused) {
    if (state_ == ExecutorStepState::Idle) {
      out.state = state_;
      return out;
    }
    return fail("NOT_PREPARED", "call prepare before tick");
  }
  if (state_ == ExecutorStepState::Error ||
      state_ == ExecutorStepState::Finished) {
    out.state = state_;
    return out;
  }
  if (state_ == ExecutorStepState::Paused) {
    out.state = state_;
    out.parameter_time = pause_record_.pause_param_time;
    return out;
  }

  if (state_ == ExecutorStepState::Stopping) {
    publishRangerZero();
    if (!config_.dry_run) {
      if (!pause_arm_cancel_issued_) {
        arm_->cancel();
        pause_arm_cancel_issued_ = true;
      }
      if (!pause_arm_stop_issued_) {
        if (!arm_->stop()) {
          return fail("PAUSE_STOP_FAILED", "arm Stop failed");
        }
        pause_arm_stop_issued_ = true;
      }
    }
    if (config_.dry_run || devicesStopped(actual)) {
      pause_record_.ranger_stopped = true;
      pause_record_.cr10_stopped = true;
      pause_record_.actual_stop_state = actual;
      state_ = ExecutorStepState::Paused;
      out.state = state_;
      out.parameter_time = pause_record_.pause_param_time;
      return out;
    }
    out.state = ExecutorStepState::Stopping;
    out.parameter_time = pause_record_.pause_param_time;
    return out;
  }

  const double now_sec = now.toSec();
  const double t0_sec = epoch_.t0.toSec();
  const double accept_deadline = t0_sec - config_.arm_accept_guard;

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
    timing_.arm_goal_accepted_at = now;
  }

  const boost::optional<double> param =
      ExecutionClock::parameterTime(epoch_, now);
  if (!param) {
    return fail("PARAM_TIME", "parameter time unavailable at/after T0");
  }
  out.parameter_time = *param;
  last_parameter_time_ = *param;
  have_last_parameter_time_ = true;

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
        std::min(*param, std::max(0.0, active_candidate_->duration()));
    desired = active_candidate_->sample(sample_t);
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

  const double speed =
      std::hypot(tracked.command.linear.x, tracked.command.angular.z);
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
  if (*param + 1e-12 >= active_candidate_->duration()) {
    state_ = ExecutorStepState::Finished;
    out.state = state_;
  }
  return out;
}
// ################################
// C++: SynchronizedExecutor implementation end
// ################################

}  // namespace remani_real
