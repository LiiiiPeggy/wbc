#include <remani_real/candidate_validator.hpp>

#include <cmath>
#include <limits>
#include <sstream>

namespace remani_real {
namespace {

// ################################
// C++: CandidateValidator helpers begin
// ################################
constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool allFinite(const Eigen::VectorXd& value) {
  return value.size() > 0 && value.allFinite();
}

double maxAbsDiff(const Eigen::VectorXd& left, const Eigen::VectorXd& right) {
  if (left.size() != right.size() || left.size() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return (left - right).cwiseAbs().maxCoeff();
}

const char* collisionTypeName(int coll_type) {
  switch (coll_type) {
    case 0:
      return "car-obs";
    case 1:
      return "mani-obs";
    case 2:
      return "car-mani";
    case 3:
      return "mani-mani";
    default:
      return "unknown";
  }
}

ValidationReport failReport(const std::string& code, const std::string& detail) {
  ValidationReport report;
  report.valid = false;
  report.error_code = code;
  report.detail = detail;
  return report;
}

bool checkContinuityPair(const Eigen::VectorXd& left_pos,
                         const Eigen::VectorXd& left_vel,
                         const Eigen::VectorXd& left_acc,
                         const Eigen::VectorXd& right_pos,
                         const Eigen::VectorXd& right_vel,
                         const Eigen::VectorXd& right_acc,
                         const ValidationLimits& limits,
                         std::string* detail) {
  if (!allFinite(left_pos) || !allFinite(left_vel) || !allFinite(left_acc) ||
      !allFinite(right_pos) || !allFinite(right_vel) || !allFinite(right_acc)) {
    *detail = "non-finite continuity samples";
    return false;
  }
  if (maxAbsDiff(left_pos, right_pos) > limits.continuity_pos_tol) {
    *detail = "position discontinuity";
    return false;
  }
  if (maxAbsDiff(left_vel, right_vel) > limits.continuity_vel_tol) {
    *detail = "velocity discontinuity";
    return false;
  }
  if (maxAbsDiff(left_acc, right_acc) > limits.continuity_acc_tol) {
    *detail = "acceleration discontinuity";
    return false;
  }
  return true;
}
// ################################
// C++: CandidateValidator helpers end
// ################################

}  // namespace

// ################################
// C++: CandidateValidator implementation begin
// ################################
CandidateValidator::CandidateValidator(ValidationLimits limits,
                                       const ValidationEnvironment* environment)
    : limits_(limits), environment_(environment) {}

ValidationReport CandidateValidator::validate(
    const FrozenCandidate& candidate,
    const ActualStateSnapshot& actual) const {
  if (!candidate) {
    return failReport("EMPTY_CANDIDATE", "frozen candidate is null");
  }
  if (environment_ == nullptr) {
    return failReport("MISSING_ENVIRONMENT", "validation environment is null");
  }
  if (candidate->segments().empty() || !(candidate->duration() > 0.0)) {
    return failReport("EMPTY_CANDIDATE", "candidate has no positive duration");
  }

  ValidationReport report;
  report.duration = candidate->duration();

  std::string continuity_detail;
  const std::vector<CandidateSegment>& segments = candidate->segments();
  for (std::size_t segment_index = 0; segment_index < segments.size();
       ++segment_index) {
    const CandidateSegment& segment = segments[segment_index];
    const MMController::Trajectory& trajectory = segment.trajectory;
    for (int piece_index = 0; piece_index + 1 < trajectory.getPieceNum();
         ++piece_index) {
      const MMController::Piece& left = trajectory[piece_index];
      const MMController::Piece& right = trajectory[piece_index + 1];
      if (!checkContinuityPair(left.getPos(left.getDuration()),
                               left.getVel(left.getDuration()),
                               left.getAcc(left.getDuration()),
                               right.getPos(0.0), right.getVel(0.0),
                               right.getAcc(0.0), limits_,
                               &continuity_detail)) {
        return failReport("SEGMENT_CONTINUITY",
                          "piece boundary: " + continuity_detail);
      }
    }
    if (segment_index + 1 < segments.size()) {
      const CandidateSegment& next = segments[segment_index + 1];
      const MMController::Piece& left =
          trajectory[trajectory.getPieceNum() - 1];
      const MMController::Piece& right = next.trajectory[0];
      if (!checkContinuityPair(left.getPos(left.getDuration()),
                               left.getVel(left.getDuration()),
                               left.getAcc(left.getDuration()),
                               right.getPos(0.0), right.getVel(0.0),
                               right.getAcc(0.0), limits_,
                               &continuity_detail)) {
        return failReport("SEGMENT_CONTINUITY",
                          "ADD boundary: " + continuity_detail);
      }
    }
  }

  WholeBodySample start;
  try {
    start = candidate->sample(0.0);
  } catch (const std::exception& ex) {
    return failReport("NON_FINITE_SAMPLE", ex.what());
  }
  if (!allFinite(start.position) || !allFinite(start.velocity) ||
      !allFinite(start.acceleration) || start.position.size() != 8) {
    return failReport("NON_FINITE_SAMPLE", "start sample is invalid");
  }

  const Eigen::Vector2d start_xy = start.position.head<2>();
  if ((start_xy - actual.base_xy).norm() > limits_.start_base_xy_tol) {
    return failReport("START_STATE_MISMATCH", "base xy exceeds tolerance");
  }
  if (std::abs(wrapAngle(start.base_yaw - actual.base_yaw)) >
      limits_.start_base_yaw_tol) {
    return failReport("START_STATE_MISMATCH", "base yaw exceeds tolerance");
  }
  const Eigen::Matrix<double, 6, 1> start_q = start.position.segment<6>(2);
  if ((start_q - actual.q).cwiseAbs().maxCoeff() > limits_.start_joint_tol) {
    return failReport("START_STATE_MISMATCH", "joint error exceeds tolerance");
  }

  const double dt = limits_.validation_dt > 0.0 ? limits_.validation_dt : 0.01;
  double t = 0.0;
  bool include_final = true;
  while (include_final) {
    const bool is_final = !(t + 0.5 * dt < report.duration);
    const double sample_t = is_final ? report.duration : t;
    WholeBodySample sample;
    try {
      sample = candidate->sample(sample_t);
    } catch (const std::exception& ex) {
      return failReport("NON_FINITE_SAMPLE", ex.what());
    }
    if (!allFinite(sample.position) || !allFinite(sample.velocity) ||
        !allFinite(sample.acceleration) || sample.position.size() != 8 ||
        !std::isfinite(sample.base_yaw) ||
        !std::isfinite(sample.base_angular_velocity)) {
      return failReport("NON_FINITE_SAMPLE", "sample contains non-finite values");
    }

    const double base_linear =
        std::hypot(sample.velocity(0), sample.velocity(1));
    report.max_base_linear_speed =
        std::max(report.max_base_linear_speed, base_linear);
    report.max_base_angular_speed =
        std::max(report.max_base_angular_speed,
                 std::abs(sample.base_angular_velocity));
    const double joint_speed = sample.velocity.segment<6>(2).cwiseAbs().maxCoeff();
    report.max_joint_speed = std::max(report.max_joint_speed, joint_speed);

    if (base_linear > limits_.max_base_linear_speed) {
      return failReport("BASE_SPEED_LIMIT", "base linear speed exceeded");
    }
    if (std::abs(sample.base_angular_velocity) >
        limits_.max_base_angular_speed) {
      return failReport("BASE_SPEED_LIMIT", "base angular speed exceeded");
    }
    if (joint_speed > limits_.max_joint_speed) {
      return failReport("JOINT_SPEED_LIMIT", "joint speed exceeded");
    }

    Eigen::Vector3d car_state;
    car_state << sample.position(0), sample.position(1), sample.base_yaw;
    const Eigen::VectorXd q = sample.position.segment(2, 6);
    if (!environment_->samplesInMap(car_state, q)) {
      return failReport("MAP_BOUNDARY", "collision samples leave GridMap");
    }
    int coll_type = -1;
    if (environment_->inCollision(car_state, q, &coll_type)) {
      std::ostringstream detail;
      detail << "collision type=" << collisionTypeName(coll_type);
      return failReport("WHOLE_BODY_COLLISION", detail.str());
    }

    if (is_final) {
      report.final_base_xy = sample.position.head<2>();
      report.final_base_yaw = sample.base_yaw;
      report.final_q = sample.position.segment<6>(2);
      report.expected_final_ee = environment_->eePose(car_state, q);
      include_final = false;
    } else {
      t += dt;
      if (!(t < report.duration)) {
        t = report.duration;
      }
    }
  }

  report.valid = true;
  report.error_code.clear();
  report.detail.clear();
  return report;
}
// ################################
// C++: CandidateValidator implementation end
// ################################

}  // namespace remani_real
