#include <remani_real/candidate_assembler.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace remani_real {
namespace {

bool allFinite(const std::vector<double>& values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace

// ################################
// C++: CandidateAssembler implementation begin
// ################################
CandidateAssembler::CandidateAssembler(double timeout_sec)
    : timeout_sec_(timeout_sec) {
  if (!std::isfinite(timeout_sec_) || timeout_sec_ <= 0.0) {
    throw std::invalid_argument("CandidateAssembler timeout must be finite and > 0");
  }
}

AssemblyEvent CandidateAssembler::makeEvent(bool accepted, AssemblyState state,
                                            const std::string& error_code,
                                            const std::string& detail) const {
  AssemblyEvent event;
  event.accepted = accepted;
  event.state = state;
  event.candidate_id = assembling_candidate_id_;
  event.raw_transaction_stamp = raw_transaction_stamp_;
  event.error_code = error_code;
  event.detail = detail;
  return event;
}

bool CandidateAssembler::timedOut(const ros::SteadyTime& now) const {
  if (state_ != AssemblyState::Assembling) {
    return false;
  }
  const double age = (now - assembly_started_at_).toSec();
  return !std::isfinite(age) || age > timeout_sec_;
}

void CandidateAssembler::invalidate(const std::string& reason) {
  assembling_segments_.clear();
  expected_trajectory_id_ = 1;
  completed_.reset();
  state_ = AssemblyState::Invalid;
  // Keep last candidate_id/stamp for correlation on the invalidation event.
  (void)reason;
}

bool CandidateAssembler::parseAdd(const quadrotor_msgs::PolynomialTraj& msg,
                                  CandidateSegment* segment,
                                  std::string* error_code,
                                  std::string* detail) const {
  if (msg.singul != 1 && msg.singul != -1) {
    *error_code = "INVALID_SINGUL";
    *detail = "singul must be +1 or -1";
    return false;
  }
  if (msg.trajectory.empty()) {
    *error_code = "EMPTY_ADD";
    *detail = "ADD trajectory array must be nonempty";
    return false;
  }

  MMController::Trajectory trajectory;
  double segment_duration = 0.0;
  for (const quadrotor_msgs::PolynomialMatrix& piece_msg : msg.trajectory) {
    if (piece_msg.num_dim != 8 || piece_msg.num_order != 7 ||
        piece_msg.data.size() != 64u ||
        !std::isfinite(piece_msg.duration) || piece_msg.duration <= 0.0 ||
        !allFinite(piece_msg.data)) {
      *error_code = "INVALID_PIECE";
      *detail = "piece must be 8D order-7 finite duration>0 with 64 coeffs";
      return false;
    }
    if (segment_duration >
        std::numeric_limits<double>::max() - piece_msg.duration) {
      *error_code = "DURATION_OVERFLOW";
      *detail = "segment duration overflows";
      return false;
    }
    Eigen::Map<const Eigen::Matrix<double, 8, 8, Eigen::ColMajor>> coeff(
        piece_msg.data.data());
    trajectory.emplace_back(piece_msg.duration, coeff);
    segment_duration += piece_msg.duration;
  }

  segment->trajectory_id = msg.trajectory_id;
  segment->singul = msg.singul;
  segment->trajectory = trajectory;
  segment->duration = segment_duration;
  return true;
}

AssemblyEvent CandidateAssembler::consume(const quadrotor_msgs::PolynomialTraj& msg,
                                          const ros::SteadyTime& now,
                                          bool new_transaction_allowed,
                                          double actual_start_yaw) {
  // ################################
  // C++: Timeout invalidates assembly, but still allows a recovery START in-call.
  // ################################
  if (timedOut(now)) {
    invalidate("ASSEMBLY_TIMEOUT");
    const bool recovery_start =
        msg.action == quadrotor_msgs::PolynomialTraj::ACTION_WARN_START &&
        new_transaction_allowed;
    if (!recovery_start) {
      return makeEvent(false, AssemblyState::Invalid, "ASSEMBLY_TIMEOUT",
                       "assembly exceeded timeout before FINAL");
    }
  }

  const auto requireEmptyControl =
      [&](const char* error_code) -> AssemblyEvent {
    if (msg.trajectory_id != 0 || !msg.trajectory.empty()) {
      if (state_ == AssemblyState::Assembling) {
        invalidate(error_code);
        return makeEvent(false, AssemblyState::Invalid, error_code,
                         "control message must use trajectory_id=0 and empty trajectory");
      }
      return makeEvent(false, state_, error_code,
                       "control message must use trajectory_id=0 and empty trajectory");
    }
    return AssemblyEvent{};  // accepted sentinel unused
  };

  switch (msg.action) {
    case quadrotor_msgs::PolynomialTraj::ACTION_WARN_START: {
      const AssemblyEvent control_error = requireEmptyControl("INVALID_START");
      if (!control_error.error_code.empty()) {
        return control_error;
      }
      if (!new_transaction_allowed) {
        return makeEvent(false, state_ == AssemblyState::Complete
                                    ? AssemblyState::Complete
                                    : state_,
                         "START_NOT_ALLOWED",
                         "new START rejected while execution owns candidate");
      }
      if (!std::isfinite(actual_start_yaw)) {
        invalidate("INVALID_START_YAW");
        return makeEvent(false, AssemblyState::Invalid, "INVALID_START_YAW",
                         "actual_start_yaw must be finite");
      }
      if (next_candidate_id_ == 0) {
        throw std::overflow_error("candidate_id overflow");
      }
      completed_.reset();
      assembling_segments_.clear();
      assembling_candidate_id_ = next_candidate_id_++;
      raw_transaction_stamp_ = msg.header.stamp;
      assembly_started_at_ = now;
      start_yaw_ = actual_start_yaw;
      expected_trajectory_id_ = 1;
      state_ = AssemblyState::Assembling;
      return makeEvent(true, AssemblyState::Assembling);
    }

    case quadrotor_msgs::PolynomialTraj::ACTION_ADD: {
      if (state_ != AssemblyState::Assembling) {
        return makeEvent(false, state_, "ADD_WITHOUT_START",
                         "ADD requires an active assembling transaction");
      }
      if (msg.trajectory_id != expected_trajectory_id_) {
        invalidate("SEGMENT_SEQUENCE");
        return makeEvent(false, AssemblyState::Invalid, "SEGMENT_SEQUENCE",
                         "trajectory_id must be contiguous from 1");
      }

      CandidateSegment segment;
      std::string error_code;
      std::string detail;
      if (!parseAdd(msg, &segment, &error_code, &detail)) {
        invalidate(error_code);
        return makeEvent(false, AssemblyState::Invalid, error_code, detail);
      }
      double start_time = 0.0;
      if (!assembling_segments_.empty()) {
        const CandidateSegment& previous = assembling_segments_.back();
        start_time = previous.start_time + previous.duration;
      }
      segment.start_time = start_time;
      assembling_segments_.push_back(segment);
      ++expected_trajectory_id_;
      return makeEvent(true, AssemblyState::Assembling);
    }

    case quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL: {
      if (state_ != AssemblyState::Assembling) {
        return makeEvent(false, state_, "FINAL_WITHOUT_START",
                         "FINAL requires an active assembling transaction");
      }
      if (assembling_segments_.empty()) {
        invalidate("FINAL_WITHOUT_ADD");
        return makeEvent(false, AssemblyState::Invalid, "FINAL_WITHOUT_ADD",
                         "FINAL requires at least one ADD segment");
      }
      if (msg.trajectory_id != 0 || !msg.trajectory.empty()) {
        invalidate("INVALID_FINAL");
        return makeEvent(false, AssemblyState::Invalid, "INVALID_FINAL",
                         "FINAL control message must have empty trajectory");
      }
      try {
        completed_ = std::make_shared<const CandidateTrajectory>(
            assembling_candidate_id_, assembling_segments_, start_yaw_,
            raw_transaction_stamp_);
      } catch (const std::exception& ex) {
        invalidate("CANDIDATE_CONSTRUCT");
        return makeEvent(false, AssemblyState::Invalid, "CANDIDATE_CONSTRUCT",
                         ex.what());
      }
      assembling_segments_.clear();
      expected_trajectory_id_ = 1;
      state_ = AssemblyState::Complete;
      return makeEvent(true, AssemblyState::Complete);
    }

    case quadrotor_msgs::PolynomialTraj::ACTION_ABORT:
    case quadrotor_msgs::PolynomialTraj::ACTION_WARN_IMPOSSIBLE: {
      const char* error_code =
          msg.action == quadrotor_msgs::PolynomialTraj::ACTION_ABORT
              ? "ABORT"
              : "IMPOSSIBLE";
      const AssemblyEvent control_error = requireEmptyControl(
          msg.action == quadrotor_msgs::PolynomialTraj::ACTION_ABORT
              ? "INVALID_ABORT"
              : "INVALID_IMPOSSIBLE");
      if (!control_error.error_code.empty()) {
        return control_error;
      }
      invalidate(error_code);
      return makeEvent(true, AssemblyState::Invalid, error_code,
                       "transaction invalidated");
    }

    default: {
      // ################################
      // C++: Unknown actions must not clear an already frozen Complete candidate.
      // ################################
      if (state_ == AssemblyState::Assembling) {
        invalidate("UNKNOWN_ACTION");
        return makeEvent(false, AssemblyState::Invalid, "UNKNOWN_ACTION",
                         "unsupported PolynomialTraj action");
      }
      return makeEvent(false, state_, "UNKNOWN_ACTION",
                       "unsupported PolynomialTraj action");
    }
  }
}

FrozenCandidate CandidateAssembler::completedCandidate() const {
  return completed_;
}
// ################################
// C++: CandidateAssembler implementation end
// ################################

}  // namespace remani_real
