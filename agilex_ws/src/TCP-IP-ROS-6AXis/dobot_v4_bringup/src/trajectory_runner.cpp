#include <dobot_v4_bringup/trajectory_runner.hpp>

#include <algorithm>
#include <cmath>

namespace dobot_v4_bringup {
namespace {

bool allNearZero(const std::array<double, 6>& values, double eps) {
  for (double v : values) {
    if (std::abs(v) > eps) {
      return false;
    }
  }
  return true;
}

double hermiteQ(double q0, double q1, double v0, double v1, double T,
                double t) {
  const double c = (-3.0 * q0 + 3.0 * q1 - 2.0 * T * v0 - T * v1) / (T * T);
  const double d = (2.0 * q0 - 2.0 * q1 + T * v0 + T * v1) / (T * T * T);
  return q0 + v0 * t + c * t * t + d * t * t * t;
}

double hermiteV(double q0, double q1, double v0, double v1, double T,
                double t) {
  const double c = (-3.0 * q0 + 3.0 * q1 - 2.0 * T * v0 - T * v1) / (T * T);
  const double d = (2.0 * q0 - 2.0 * q1 + T * v0 + T * v1) / (T * T * T);
  return v0 + 2.0 * c * t + 3.0 * d * t * t;
}

}  // namespace

// ################################
// C++: TrajectoryRunner implementation begin
// ################################
TrajectoryRunner::TrajectoryRunner(RunnerConfig config) : config_(config) {
  if (!(config_.required_settle_samples > 0)) {
    config_.required_settle_samples = 3;
  }
}

StartDecision TrajectoryRunner::start(const CanonicalTrajectory& trajectory,
                                      double steady_start_sec) {
  StartDecision out;
  if (trajectory.points.size() < 2) {
    out.error_code = "POINT_COUNT";
    out.detail = "runner requires at least two points";
    return out;
  }
  if (!std::isfinite(steady_start_sec)) {
    out.error_code = "START_TIME";
    out.detail = "steady_start_sec must be finite";
    return out;
  }

  if (config_.remani_prestart_hold_mode) {
    if (trajectory.points.size() < 3) {
      out.error_code = "REMANI_HOLD_POINTS";
      out.detail = "remani hold mode requires hold + at least two motion points";
      return out;
    }
    if (!(std::abs(trajectory.points[0].time_from_start) <= 1e-12) ||
        !allNearZero(trajectory.points[0].velocities, 1e-12)) {
      out.error_code = "REMANI_HOLD_POINT0";
      out.detail = "point[0] must be t=0 with zero velocity";
      return out;
    }
    if (!(trajectory.points[1].time_from_start > 0.0)) {
      out.error_code = "REMANI_HOLD_LEAD";
      out.detail = "point[1].time must be the positive start_lead_time";
      return out;
    }
  }

  trajectory_ = trajectory;
  point_times_.clear();
  point_times_.reserve(trajectory_.points.size());
  for (const CanonicalTrajectoryPoint& p : trajectory_.points) {
    point_times_.push_back(p.time_from_start);
  }
  final_q_ = trajectory_.points.back().positions;
  steady_start_sec_ = steady_start_sec;
  settle_start_sec_ = 0.0;
  settle_good_samples_ = 0;
  cancel_good_samples_ = 0;
  cancel_requested_ = false;
  stop_issued_ = false;
  tick_count_ = 0;

  if (config_.remani_prestart_hold_mode) {
    state_ = RunnerState::Holding;
  } else {
    state_ = RunnerState::Running;
  }
  out.accepted = true;
  return out;
}

void TrajectoryRunner::requestCancel() {
  if (state_ == RunnerState::Idle || state_ == RunnerState::Succeeded ||
      state_ == RunnerState::Canceled || state_ == RunnerState::Aborted) {
    return;
  }
  cancel_requested_ = true;
  state_ = RunnerState::Canceling;
  cancel_good_samples_ = 0;
}

bool TrajectoryRunner::withinGoal(const RunnerActualState& actual) const {
  for (std::size_t i = 0; i < 6; ++i) {
    if (std::abs(actual.q[i] - final_q_[i]) > config_.goal_joint_tol) {
      return false;
    }
  }
  return true;
}

bool TrajectoryRunner::withinStopVelocity(
    const RunnerActualState& actual) const {
  for (std::size_t i = 0; i < 6; ++i) {
    if (std::abs(actual.qd[i]) > config_.stop_velocity_tol) {
      return false;
    }
  }
  return true;
}

void TrajectoryRunner::sampleAt(double traj_time, std::array<double, 6>* q,
                                std::array<double, 6>* qd) const {
  const double t_end = point_times_.back();
  if (traj_time <= point_times_.front()) {
    *q = trajectory_.points.front().positions;
    *qd = trajectory_.points.front().velocities;
    return;
  }
  if (traj_time >= t_end) {
    *q = trajectory_.points.back().positions;
    *qd = trajectory_.points.back().velocities;
    return;
  }

  const auto upper =
      std::upper_bound(point_times_.begin(), point_times_.end(), traj_time);
  const std::size_t i1 =
      static_cast<std::size_t>(std::distance(point_times_.begin(), upper));
  const std::size_t i0 = i1 - 1;
  const CanonicalTrajectoryPoint& p0 = trajectory_.points[i0];
  const CanonicalTrajectoryPoint& p1 = trajectory_.points[i1];
  const double T = p1.time_from_start - p0.time_from_start;
  const double t = traj_time - p0.time_from_start;
  for (std::size_t j = 0; j < 6; ++j) {
    (*q)[j] = hermiteQ(p0.positions[j], p1.positions[j], p0.velocities[j],
                       p1.velocities[j], T, t);
    (*qd)[j] = hermiteV(p0.positions[j], p1.positions[j], p0.velocities[j],
                        p1.velocities[j], T, t);
  }
}

RunnerTick TrajectoryRunner::tick(double steady_now_sec,
                                  const RunnerActualState& actual) {
  RunnerTick out;
  out.state = state_;
  if (state_ == RunnerState::Idle || state_ == RunnerState::Succeeded ||
      state_ == RunnerState::Canceled || state_ == RunnerState::Aborted) {
    out.terminal = (state_ != RunnerState::Idle);
    return out;
  }
  if (!std::isfinite(steady_now_sec)) {
    state_ = RunnerState::Aborted;
    out.state = state_;
    out.terminal = true;
    out.error_code = "NON_FINITE_TIME";
    return out;
  }

  ++tick_count_;
  const double elapsed = steady_now_sec - steady_start_sec_;

  if (cancel_requested_ || state_ == RunnerState::Canceling) {
    state_ = RunnerState::Canceling;
    out.state = state_;
    out.has_command = false;
    stop_issued_ = true;
    if (withinStopVelocity(actual)) {
      ++cancel_good_samples_;
    } else {
      cancel_good_samples_ = 0;
    }
    if (cancel_good_samples_ >= config_.required_settle_samples) {
      state_ = RunnerState::Canceled;
      out.state = state_;
      out.terminal = true;
    }
    return out;
  }

  if (state_ == RunnerState::Holding) {
    const double hold_until = trajectory_.points[1].time_from_start;
    out.has_command = true;
    out.command_rad = trajectory_.points[0].positions;
    out.desired_velocity = trajectory_.points[0].velocities;
    if (elapsed + 1e-12 >= hold_until) {
      state_ = RunnerState::Running;
      out.state = state_;
      out.command_rad = trajectory_.points[1].positions;
      out.desired_velocity = trajectory_.points[1].velocities;
    } else {
      out.state = RunnerState::Holding;
    }
    return out;
  }

  if (state_ == RunnerState::Running) {
    const double t_end = point_times_.back();
    sampleAt(elapsed, &out.command_rad, &out.desired_velocity);
    out.has_command = true;
    out.state = RunnerState::Running;
    if (elapsed + 1e-12 >= t_end) {
      state_ = RunnerState::Settling;
      settle_start_sec_ = steady_now_sec;
      settle_good_samples_ = 0;
      out.state = state_;
      out.command_rad = final_q_;
      out.desired_velocity.fill(0.0);
    }
    return out;
  }

  if (state_ == RunnerState::Settling) {
    out.has_command = true;
    out.command_rad = final_q_;
    out.desired_velocity.fill(0.0);
    out.state = RunnerState::Settling;
    if (withinGoal(actual) && withinStopVelocity(actual)) {
      ++settle_good_samples_;
    } else {
      settle_good_samples_ = 0;
    }
    if (settle_good_samples_ >= config_.required_settle_samples) {
      state_ = RunnerState::Succeeded;
      out.state = state_;
      out.terminal = true;
      return out;
    }
    if ((steady_now_sec - settle_start_sec_) > config_.settle_timeout) {
      state_ = RunnerState::Aborted;
      out.state = state_;
      out.terminal = true;
      out.error_code = "SETTLE_TIMEOUT";
    }
    return out;
  }

  return out;
}
// ################################
// C++: TrajectoryRunner implementation end
// ################################

}  // namespace dobot_v4_bringup
