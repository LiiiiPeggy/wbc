#include <remani_real/candidate_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <utility>

#include <unsupported/Eigen/Polynomials>

namespace remani_real {
namespace {

constexpr double kMinimumBaseSpeed = 1e-6;
constexpr double kMinimumBaseSpeedSquared =
    kMinimumBaseSpeed * kMinimumBaseSpeed;
constexpr double kPolynomialRootTolerance = 1e-9;
constexpr std::size_t kHeadingBisectionIterations = 64;

/* ---------- Validate timing comparisons without accepting arbitrary gaps. ---------- */
bool timingsMatch(double expected, double actual) {
  const double scale = std::max(1.0, std::max(std::abs(expected), std::abs(actual)));
  return std::abs(expected - actual) <= 16.0 * std::numeric_limits<double>::epsilon() * scale;
}

/* ---------- Evaluate the base-heading terms associated with a polynomial piece. ---------- */
bool headingFromPiece(const MMController::Piece& piece, double local_time,
                      int singul, double* yaw, double* angular_velocity) {
  const Eigen::VectorXd velocity = piece.getVel(local_time);
  const Eigen::VectorXd acceleration = piece.getAcc(local_time);
  const double vx = velocity(0);
  const double vy = velocity(1);
  const double squared_speed = vx * vx + vy * vy;
  if (!std::isfinite(squared_speed) || squared_speed < kMinimumBaseSpeedSquared) {
    return false;
  }

  *yaw = std::atan2(static_cast<double>(singul) * vy,
                    static_cast<double>(singul) * vx);
  *angular_velocity = (vx * acceleration(1) - vy * acceleration(0)) / squared_speed;
  return std::isfinite(*yaw) && std::isfinite(*angular_velocity);
}

/* ---------- Form the bounded-degree speed threshold polynomial in ascending powers. ---------- */
Eigen::VectorXd speedThresholdPolynomial(const MMController::Piece& piece) {
  const int position_degree = piece.getDegree();
  const int velocity_degree = std::max(0, position_degree - 1);
  Eigen::VectorXd vx = Eigen::VectorXd::Zero(velocity_degree + 1);
  Eigen::VectorXd vy = Eigen::VectorXd::Zero(velocity_degree + 1);
  const MMController::Piece::CoefficientMat& coefficients = piece.getCoeffMat();
  for (int exponent = 1; exponent <= position_degree; ++exponent) {
    const int coefficient_column = position_degree - exponent;
    vx(exponent - 1) = exponent * coefficients(0, coefficient_column);
    vy(exponent - 1) = exponent * coefficients(1, coefficient_column);
  }

  Eigen::VectorXd speed_squared =
      Eigen::VectorXd::Zero(2 * velocity_degree + 1);
  for (int left = 0; left <= velocity_degree; ++left) {
    for (int right = 0; right <= velocity_degree; ++right) {
      const double x_product = vx(left) * vx(right);
      const double y_product = vy(left) * vy(right);
      const double sum = speed_squared(left + right) + x_product + y_product;
      if (!std::isfinite(x_product) || !std::isfinite(y_product) ||
          !std::isfinite(sum)) {
        throw std::domain_error("candidate speed polynomial is non-finite");
      }
      speed_squared(left + right) = sum;
    }
  }
  speed_squared(0) -= kMinimumBaseSpeedSquared;
  if (!std::isfinite(speed_squared(0))) {
    throw std::domain_error("candidate speed threshold is non-finite");
  }
  return speed_squared;
}

/* ---------- Find all real speed-threshold roots in a piece with scale-aware filtering. ---------- */
std::vector<double> speedThresholdRoots(const MMController::Piece& piece,
                                        double upper_time) {
  Eigen::VectorXd polynomial = speedThresholdPolynomial(piece);
  double scale = 0.0;
  for (Eigen::Index index = 0; index < polynomial.size(); ++index) {
    scale = std::max(scale, std::abs(polynomial(index)));
  }
  if (scale == 0.0) {
    return {};
  }
  const double coefficient_tolerance =
      64.0 * std::numeric_limits<double>::epsilon() * scale;
  Eigen::Index degree = polynomial.size() - 1;
  while (degree > 0 && std::abs(polynomial(degree)) <= coefficient_tolerance) {
    --degree;
  }
  if (degree == 0) {
    return {};
  }

  Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
  solver.compute(polynomial.head(degree + 1));

  std::vector<double> roots;
  for (Eigen::Index index = 0; index < solver.roots().size(); ++index) {
    const std::complex<double> root = solver.roots()(index);
    if (!std::isfinite(root.real()) || !std::isfinite(root.imag()) ||
        std::abs(root.imag()) >
            kPolynomialRootTolerance * std::max(1.0, std::abs(root.real()))) {
      continue;
    }
    const double root_tolerance =
        kPolynomialRootTolerance * std::max(1.0, std::abs(root.real()));
    if (root.real() < -root_tolerance || root.real() > upper_time + root_tolerance) {
      continue;
    }
    roots.push_back(std::min(std::max(0.0, root.real()), upper_time));
  }
  std::sort(roots.begin(), roots.end());
  roots.erase(std::unique(roots.begin(), roots.end(), [](double left, double right) {
    return std::abs(left - right) <=
        kPolynomialRootTolerance * std::max(1.0, std::max(std::abs(left), std::abs(right)));
  }), roots.end());
  return roots;
}

/* ---------- Recover the latest valid heading by real-root interval partition, not grid probing. ---------- */
bool recoverHeadingAtOrBefore(const MMController::Piece& piece, double upper_time,
                              int singul, double* yaw) {
  if (upper_time < 0.0) {
    return false;
  }

  double ignored_angular_velocity = 0.0;
  if (headingFromPiece(piece, upper_time, singul, yaw, &ignored_angular_velocity)) {
    return true;
  }
  if (upper_time == 0.0) {
    return false;
  }

  std::vector<double> boundaries = speedThresholdRoots(piece, upper_time);
  boundaries.push_back(0.0);
  boundaries.push_back(upper_time);
  std::sort(boundaries.begin(), boundaries.end());
  boundaries.erase(std::unique(boundaries.begin(), boundaries.end(),
                               [](double left, double right) {
    return std::abs(left - right) <=
        kPolynomialRootTolerance * std::max(1.0, std::max(std::abs(left), std::abs(right)));
  }), boundaries.end());

  for (std::size_t index = boundaries.size(); index > 1; --index) {
    const double left = boundaries[index - 2];
    const double right = boundaries[index - 1];
    if (headingFromPiece(piece, right, singul, yaw, &ignored_angular_velocity)) {
      return true;
    }
    const double midpoint = 0.5 * (left + right);
    if (!headingFromPiece(piece, midpoint, singul, yaw,
                          &ignored_angular_velocity)) {
      continue;
    }

    double lower_bound = midpoint;
    double upper_bound = right;
    for (std::size_t iteration = 0;
         iteration < kHeadingBisectionIterations; ++iteration) {
      const double candidate_time = 0.5 * (lower_bound + upper_bound);
      if (headingFromPiece(piece, candidate_time, singul, yaw,
                           &ignored_angular_velocity)) {
        lower_bound = candidate_time;
      } else {
        upper_bound = candidate_time;
      }
    }
    return headingFromPiece(piece, lower_bound, singul, yaw,
                            &ignored_angular_velocity);
  }
  return headingFromPiece(piece, 0.0, singul, yaw, &ignored_angular_velocity);
}

/* ---------- Select a segment with later ownership at exact internal boundaries. ---------- */
std::size_t segmentIndexAt(const std::vector<CandidateSegment>& segments,
                           double total_duration, double t) {
  if (t == total_duration) {
    return segments.size() - 1;
  }

  for (std::size_t index = 0; index < segments.size(); ++index) {
    const CandidateSegment& segment = segments[index];
    if (t < segment.start_time + segment.duration) {
      return index;
    }
  }
  throw std::logic_error("validated candidate has no segment for sample time");
}

/* ---------- Select a polynomial piece with later ownership at exact internal boundaries. ---------- */
std::size_t pieceIndexAt(const MMController::Trajectory& trajectory,
                         double segment_time) {
  const int piece_count = trajectory.getPieceNum();
  double elapsed = 0.0;
  for (int index = 0; index < piece_count; ++index) {
    const double next = elapsed + trajectory[index].getDuration();
    if (segment_time < next) {
      return static_cast<std::size_t>(index);
    }
    elapsed = next;
  }
  return static_cast<std::size_t>(piece_count - 1);
}

/* ---------- Calculate local time within an already selected trajectory piece. ---------- */
double localPieceTime(const MMController::Trajectory& trajectory,
                      std::size_t piece_index, double segment_time) {
  double elapsed = 0.0;
  for (std::size_t index = 0; index < piece_index; ++index) {
    elapsed += trajectory[static_cast<int>(index)].getDuration();
  }
  return std::min(std::max(0.0, segment_time - elapsed),
                  trajectory[static_cast<int>(piece_index)].getDuration());
}

}  // namespace

/* ---------- Construct and validate an immutable, Gate-owned candidate transaction. ---------- */
CandidateTrajectory::CandidateTrajectory(uint64_t candidate_id,
                                         std::vector<CandidateSegment> segments,
                                         double start_yaw,
                                         const ros::Time& raw_transaction_stamp)
    : candidate_id_(candidate_id),
      segments_(std::move(segments)),
      start_yaw_(start_yaw),
      raw_transaction_stamp_(raw_transaction_stamp) {
  if (!std::isfinite(start_yaw_) || segments_.empty()) {
    throw std::invalid_argument("candidate must have finite start yaw and nonempty segments");
  }

  double cumulative_duration = 0.0;
  for (const CandidateSegment& segment : segments_) {
    if (!std::isfinite(segment.start_time) || !std::isfinite(segment.duration) ||
        segment.duration <= 0.0 || !timingsMatch(cumulative_duration, segment.start_time)) {
      throw std::invalid_argument("candidate segments must have contiguous finite timing");
    }
    if (segment.trajectory.getPieceNum() <= 0 || segment.trajectory.getDim() != 8) {
      throw std::invalid_argument("candidate segments require nonempty eight-dimensional trajectories");
    }

    double trajectory_duration = 0.0;
    for (int piece_index = 0; piece_index < segment.trajectory.getPieceNum(); ++piece_index) {
      const MMController::Piece& piece = segment.trajectory[piece_index];
      if (piece.getDim() != 8 || piece.getCoeffMat().cols() <= 0 ||
          !piece.getCoeffMat().allFinite() || !std::isfinite(piece.getDuration()) ||
          piece.getDuration() <= 0.0 ||
          trajectory_duration > std::numeric_limits<double>::max() - piece.getDuration()) {
        throw std::invalid_argument("candidate polynomial piece is invalid");
      }
      trajectory_duration += piece.getDuration();
    }
    if (!std::isfinite(trajectory_duration) ||
        !timingsMatch(segment.duration, trajectory_duration)) {
      throw std::invalid_argument("segment duration must equal its polynomial duration");
    }
    if (cumulative_duration > std::numeric_limits<double>::max() - segment.duration) {
      throw std::invalid_argument("candidate duration overflows");
    }
    cumulative_duration += segment.duration;
  }
  if (!std::isfinite(cumulative_duration) || cumulative_duration <= 0.0) {
    throw std::invalid_argument("candidate must have finite positive duration");
  }
  duration_ = cumulative_duration;
}

uint64_t CandidateTrajectory::id() const {
  return candidate_id_;
}

double CandidateTrajectory::duration() const {
  return duration_;
}

const std::vector<CandidateSegment>& CandidateTrajectory::segments() const {
  return segments_;
}

const ros::Time& CandidateTrajectory::rawTransactionStamp() const {
  return raw_transaction_stamp_;
}

/* ---------- Sample position, derivatives, and heading without mutating the frozen candidate. ---------- */
WholeBodySample CandidateTrajectory::sample(double t) const {
  if (!std::isfinite(t) || t < 0.0 || t > duration_) {
    throw std::out_of_range("candidate sample time is outside its duration");
  }

  const std::size_t segment_index = segmentIndexAt(segments_, duration_, t);
  const CandidateSegment& segment = segments_[segment_index];
  const double segment_time = std::min(std::max(0.0, t - segment.start_time),
                                       segment.duration);
  const std::size_t piece_index = pieceIndexAt(segment.trajectory, segment_time);
  const MMController::Piece& piece = segment.trajectory[static_cast<int>(piece_index)];
  const double piece_time = localPieceTime(segment.trajectory, piece_index, segment_time);

  WholeBodySample result;
  result.position = piece.getPos(piece_time);
  result.velocity = piece.getVel(piece_time);
  result.acceleration = piece.getAcc(piece_time);
  if (!result.position.allFinite() || !result.velocity.allFinite() ||
      !result.acceleration.allFinite()) {
    throw std::domain_error("candidate polynomial evaluation is non-finite");
  }
  result.singul = segment.singul;

  if (headingFromPiece(piece, piece_time, segment.singul, &result.base_yaw,
                       &result.base_angular_velocity)) {
    return result;
  }

  if (recoverHeadingAtOrBefore(piece, piece_time, segment.singul,
                               &result.base_yaw)) {
    result.base_angular_velocity = 0.0;
    return result;
  }
  for (std::size_t prior_piece = piece_index; prior_piece > 0; --prior_piece) {
    const MMController::Piece& candidate_piece =
        segment.trajectory[static_cast<int>(prior_piece - 1)];
    if (recoverHeadingAtOrBefore(candidate_piece, candidate_piece.getDuration(),
                                 segment.singul, &result.base_yaw)) {
      result.base_angular_velocity = 0.0;
      return result;
    }
  }
  for (std::size_t prior_segment = segment_index; prior_segment > 0; --prior_segment) {
    const CandidateSegment& candidate_segment = segments_[prior_segment - 1];
    for (int prior_piece = candidate_segment.trajectory.getPieceNum(); prior_piece > 0;
         --prior_piece) {
      const MMController::Piece& candidate_piece =
          candidate_segment.trajectory[prior_piece - 1];
      if (recoverHeadingAtOrBefore(candidate_piece, candidate_piece.getDuration(),
                                   candidate_segment.singul, &result.base_yaw)) {
        result.base_angular_velocity = 0.0;
        return result;
      }
    }
  }

  result.base_yaw = start_yaw_;
  result.base_angular_velocity = 0.0;
  return result;
}

}  // namespace remani_real
