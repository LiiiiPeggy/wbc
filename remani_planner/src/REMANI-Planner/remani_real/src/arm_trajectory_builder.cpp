#include <remani_real/arm_trajectory_builder.hpp>

#include <cmath>
#include <vector>

namespace remani_real {
namespace {

constexpr double kMaxJointSpeed = 0.10;
constexpr int kArmOffset = 2;
constexpr int kArmDof = 6;

bool extractArm(const WholeBodySample& sample, std::vector<double>* q,
                std::vector<double>* qd, std::string* error_code) {
  if (sample.position.size() < kArmOffset + kArmDof ||
      sample.velocity.size() < kArmOffset + kArmDof) {
    *error_code = "ARM_SAMPLE_DIM";
    return false;
  }
  q->resize(static_cast<std::size_t>(kArmDof));
  qd->resize(static_cast<std::size_t>(kArmDof));
  for (int i = 0; i < kArmDof; ++i) {
    (*q)[static_cast<std::size_t>(i)] = sample.position(kArmOffset + i);
    (*qd)[static_cast<std::size_t>(i)] = sample.velocity(kArmOffset + i);
    if (!std::isfinite((*q)[static_cast<std::size_t>(i)]) ||
        !std::isfinite((*qd)[static_cast<std::size_t>(i)])) {
      *error_code = "ARM_SAMPLE_NONFINITE";
      return false;
    }
    if (std::abs((*qd)[static_cast<std::size_t>(i)]) > kMaxJointSpeed) {
      *error_code = "ARM_SPEED_LIMIT";
      return false;
    }
  }
  return true;
}

trajectory_msgs::JointTrajectoryPoint makePoint(double time_from_start,
                                                const std::vector<double>& q,
                                                const std::vector<double>& qd) {
  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(time_from_start);
  point.positions = q;
  point.velocities = qd;
  return point;
}

}  // namespace

// ################################
// C++: ArmTrajectoryBuilder::build begin
// ################################
ArmTrajectoryBuildResult ArmTrajectoryBuilder::build(
    const FrozenCandidate& candidate,
    const Eigen::Matrix<double, 6, 1>& current_q, double start_lead_time,
    double sample_period, double hold_tol) {
  ArmTrajectoryBuildResult out;
  if (!candidate) {
    out.error_code = "ARM_CANDIDATE_NULL";
    out.detail = "frozen candidate is null";
    return out;
  }
  if (!(start_lead_time > 0.0) || !(sample_period > 0.0) || !(hold_tol > 0.0) ||
      !std::isfinite(start_lead_time) || !std::isfinite(sample_period) ||
      !std::isfinite(hold_tol)) {
    out.error_code = "ARM_BUILD_PARAMS";
    out.detail = "lead/period/tol must be finite and positive";
    return out;
  }
  if (!current_q.allFinite()) {
    out.error_code = "ARM_CURRENT_NONFINITE";
    return out;
  }

  WholeBodySample at_zero;
  try {
    at_zero = candidate->sample(0.0);
  } catch (const std::exception& ex) {
    out.error_code = "ARM_SAMPLE_FAIL";
    out.detail = ex.what();
    return out;
  }

  std::vector<double> q0;
  std::vector<double> qd0;
  if (!extractArm(at_zero, &q0, &qd0, &out.error_code)) {
    return out;
  }

  double max_handoff = 0.0;
  for (int i = 0; i < kArmDof; ++i) {
    max_handoff =
        std::max(max_handoff, std::abs(current_q(i) - q0[static_cast<std::size_t>(i)]));
  }
  if (max_handoff > hold_tol) {
    out.error_code = "ARM_HOLD_HANDOFF_TOLERANCE";
    out.detail = "current q exceeds hold handoff tolerance vs candidate q(0)";
    return out;
  }

  std::vector<double> remani_times;
  remani_times.push_back(0.0);
  const double duration = candidate->duration();
  for (double t = sample_period; t + 1e-12 < duration; t += sample_period) {
    remani_times.push_back(t);
  }
  if (std::abs(remani_times.back() - duration) > 1e-12) {
    remani_times.push_back(duration);
  }

  out.trajectory.joint_names = {"joint1", "joint2", "joint3",
                                "joint4", "joint5", "joint6"};
  std::vector<double> hold_q(static_cast<std::size_t>(kArmDof));
  for (int i = 0; i < kArmDof; ++i) {
    hold_q[static_cast<std::size_t>(i)] = current_q(i);
  }
  std::vector<double> zero_qd(static_cast<std::size_t>(kArmDof), 0.0);
  out.trajectory.points.push_back(makePoint(0.0, hold_q, zero_qd));

  for (double remani_t : remani_times) {
    WholeBodySample sample;
    try {
      sample = candidate->sample(remani_t);
    } catch (const std::exception& ex) {
      out.error_code = "ARM_SAMPLE_FAIL";
      out.detail = ex.what();
      out.trajectory.points.clear();
      return out;
    }
    std::vector<double> q;
    std::vector<double> qd;
    if (!extractArm(sample, &q, &qd, &out.error_code)) {
      out.trajectory.points.clear();
      return out;
    }
    out.trajectory.points.push_back(
        makePoint(start_lead_time + remani_t, q, qd));
  }

  out.valid = true;
  return out;
}
// ################################
// C++: ArmTrajectoryBuilder::build end
// ################################

}  // namespace remani_real
