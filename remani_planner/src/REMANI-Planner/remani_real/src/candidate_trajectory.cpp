#include <remani_real/candidate_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace remani_real {
namespace {

constexpr double kMinimumBaseSpeed = 1e-6;
constexpr double kMinimumBaseSpeedSquared =
    kMinimumBaseSpeed * kMinimumBaseSpeed;
constexpr long double kRootTimeTolerance = 1e-12L;
constexpr long double kRootValueTolerance =
    4096.0L * std::numeric_limits<long double>::epsilon();
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
  // ################################
  // Accept tangent/repeated threshold roots that undershoot by double roundoff only.
  // ################################
  constexpr double kSpeedThresholdAcceptSlack =
      256.0 * std::numeric_limits<double>::epsilon() * kMinimumBaseSpeedSquared;
  if (!std::isfinite(squared_speed) ||
      squared_speed + kSpeedThresholdAcceptSlack < kMinimumBaseSpeedSquared) {
    return false;
  }

  *yaw = std::atan2(static_cast<double>(singul) * vy,
                    static_cast<double>(singul) * vx);
  *angular_velocity = (vx * acceleration(1) - vy * acceleration(0)) / squared_speed;
  return std::isfinite(*yaw) && std::isfinite(*angular_velocity);
}

using Polynomial = std::vector<long double>;

/* ---------- Trim only numerically irresolvable trailing terms from a normalized polynomial. ---------- */
void trimPolynomial(Polynomial* polynomial) {
  long double scale = 0.0L;
  for (const long double coefficient : *polynomial) {
    scale = std::max(scale, std::abs(coefficient));
  }
  const long double tolerance = kRootValueTolerance * scale;
  while (polynomial->size() > 1 &&
         std::abs(polynomial->back()) <= tolerance) {
    polynomial->pop_back();
  }
}

/* ---------- Evaluate an ascending-power polynomial on the normalized unit interval. ---------- */
long double evaluatePolynomial(const Polynomial& polynomial, long double value) {
  long double result = 0.0L;
  for (Polynomial::const_reverse_iterator it = polynomial.rbegin();
       it != polynomial.rend(); ++it) {
    result = result * value + *it;
  }
  if (!std::isfinite(result)) {
    throw std::domain_error("normalized candidate speed polynomial is non-finite");
  }
  return result;
}

/* ---------- Differentiate an ascending-power polynomial for recursive root isolation. ---------- */
Polynomial derivativePolynomial(const Polynomial& polynomial) {
  if (polynomial.size() <= 1) {
    return {0.0L};
  }
  Polynomial derivative(polynomial.size() - 1, 0.0L);
  for (std::size_t index = 1; index < polynomial.size(); ++index) {
    derivative[index - 1] = static_cast<long double>(index) * polynomial[index];
    if (!std::isfinite(derivative[index - 1])) {
      throw std::domain_error("normalized candidate speed derivative is non-finite");
    }
  }
  trimPolynomial(&derivative);
  return derivative;
}

/* ---------- Bisect one monotonic normalized interval with a strict endpoint sign change. ---------- */
long double bisectNormalizedRoot(const Polynomial& polynomial, long double left,
                                 long double right) {
  long double left_value = evaluatePolynomial(polynomial, left);
  for (std::size_t iteration = 0;
       iteration < kHeadingBisectionIterations * 2; ++iteration) {
    const long double midpoint = 0.5L * (left + right);
    const long double midpoint_value = evaluatePolynomial(polynomial, midpoint);
    if (std::abs(midpoint_value) <= kRootValueTolerance ||
        right - left <= kRootTimeTolerance) {
      return midpoint;
    }
    if ((left_value < 0.0L && midpoint_value > 0.0L) ||
        (left_value > 0.0L && midpoint_value < 0.0L)) {
      right = midpoint;
    } else {
      left = midpoint;
      left_value = midpoint_value;
    }
  }
  return 0.5L * (left + right);
}

/* ---------- Isolate all real roots on [0,1] through derivative critical intervals. ---------- */
std::vector<long double> isolateNormalizedRoots(Polynomial polynomial) {
  trimPolynomial(&polynomial);
  if (polynomial.size() <= 1) {
    return {};
  }

  std::vector<long double> boundaries = derivativePolynomial(polynomial).size() <= 1
      ? std::vector<long double>{0.0L, 1.0L}
      : isolateNormalizedRoots(derivativePolynomial(polynomial));
  boundaries.push_back(0.0L);
  boundaries.push_back(1.0L);
  std::sort(boundaries.begin(), boundaries.end());
  boundaries.erase(std::unique(boundaries.begin(), boundaries.end(),
                               [](long double left, long double right) {
    return std::abs(left - right) <= kRootTimeTolerance;
  }), boundaries.end());

  std::vector<long double> roots;
  for (const long double boundary : boundaries) {
    if (std::abs(evaluatePolynomial(polynomial, boundary)) <= kRootValueTolerance) {
      roots.push_back(boundary);
    }
  }
  for (std::size_t index = 1; index < boundaries.size(); ++index) {
    const long double left = boundaries[index - 1];
    const long double right = boundaries[index];
    const long double left_value = evaluatePolynomial(polynomial, left);
    const long double right_value = evaluatePolynomial(polynomial, right);
    if ((left_value < 0.0L && right_value > 0.0L) ||
        (left_value > 0.0L && right_value < 0.0L)) {
      roots.push_back(bisectNormalizedRoot(polynomial, left, right));
    }
  }
  std::sort(roots.begin(), roots.end());
  roots.erase(std::unique(roots.begin(), roots.end(),
                          [](long double left, long double right) {
    return std::abs(left - right) <= kRootTimeTolerance;
  }), roots.end());
  return roots;
}

/* ---------- Form speed squared minus threshold after safely normalizing t by piece duration. ---------- */
Polynomial normalizedSpeedThresholdPolynomial(const MMController::Piece& piece,
                                              double duration) {
  if (!std::isfinite(duration) || duration <= 0.0) {
    throw std::domain_error("candidate piece duration is invalid for root isolation");
  }
  const int position_degree = piece.getDegree();
  const int velocity_degree = std::max(0, position_degree - 1);
  Polynomial vx(velocity_degree + 1, 0.0L);
  Polynomial vy(velocity_degree + 1, 0.0L);
  const MMController::Piece::CoefficientMat& coefficients = piece.getCoeffMat();
  for (int exponent = 1; exponent <= position_degree; ++exponent) {
    const int coefficient_column = position_degree - exponent;
    const int velocity_power = exponent - 1;
    const long double domain_scale = std::pow(static_cast<long double>(duration),
                                              velocity_power);
    vx[velocity_power] = static_cast<long double>(exponent) *
        static_cast<long double>(coefficients(0, coefficient_column)) * domain_scale;
    vy[velocity_power] = static_cast<long double>(exponent) *
        static_cast<long double>(coefficients(1, coefficient_column)) * domain_scale;
    if (!std::isfinite(vx[velocity_power]) || !std::isfinite(vy[velocity_power])) {
      throw std::domain_error("candidate normalized speed coefficient overflows");
    }
  }

  Polynomial speed_squared(2 * velocity_degree + 1, 0.0L);
  for (int left = 0; left <= velocity_degree; ++left) {
    for (int right = 0; right <= velocity_degree; ++right) {
      const long double x_product = vx[left] * vx[right];
      const long double y_product = vy[left] * vy[right];
      const long double sum = speed_squared[left + right] + x_product + y_product;
      if (!std::isfinite(x_product) || !std::isfinite(y_product) ||
          !std::isfinite(sum)) {
        throw std::domain_error("candidate normalized speed polynomial is non-finite");
      }
      speed_squared[left + right] = sum;
    }
  }
  speed_squared[0] -= static_cast<long double>(kMinimumBaseSpeedSquared);
  if (!std::isfinite(speed_squared[0])) {
    throw std::domain_error("candidate normalized speed threshold is non-finite");
  }
  long double scale = 0.0L;
  for (const long double coefficient : speed_squared) {
    scale = std::max(scale, std::abs(coefficient));
  }
  if (scale == 0.0L) {
    return {0.0L};
  }
  for (long double& coefficient : speed_squared) {
    coefficient /= scale;
  }
  trimPolynomial(&speed_squared);
  return speed_squared;
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

  // ################################
  // Normalized root isolation recovers yaw without sampling past the query time.
  // ################################
  const double piece_duration = piece.getDuration();
  if (!std::isfinite(piece_duration) || piece_duration <= 0.0) {
    throw std::domain_error("candidate piece duration is invalid for heading recovery");
  }
  const long double upper_u = static_cast<long double>(upper_time) /
      static_cast<long double>(piece_duration);
  if (!std::isfinite(upper_u)) {
    throw std::domain_error("candidate normalized query time is non-finite");
  }
  std::vector<long double> boundaries = isolateNormalizedRoots(
      normalizedSpeedThresholdPolynomial(piece, piece_duration));
  // Never keep a root after the query, even when it exceeds upper_u by only a tolerance.
  boundaries.erase(std::remove_if(boundaries.begin(), boundaries.end(),
                                  [upper_u](long double root) {
    return root > upper_u;
  }), boundaries.end());
  boundaries.push_back(0.0L);
  boundaries.push_back(std::min(1.0L, std::max(0.0L, upper_u)));
  std::sort(boundaries.begin(), boundaries.end());
  boundaries.erase(std::unique(boundaries.begin(), boundaries.end(),
                               [](long double left, long double right) {
    return std::abs(left - right) <= kRootTimeTolerance;
  }), boundaries.end());

  for (std::size_t index = boundaries.size(); index > 1; --index) {
    const long double left = boundaries[index - 2];
    const long double right = boundaries[index - 1];
    if (headingFromPiece(piece, static_cast<double>(right * piece_duration), singul,
                         yaw, &ignored_angular_velocity)) {
      return true;
    }
    const long double midpoint = 0.5L * (left + right);
    if (!headingFromPiece(piece, static_cast<double>(midpoint * piece_duration), singul, yaw,
                          &ignored_angular_velocity)) {
      continue;
    }

    long double lower_bound = midpoint;
    long double upper_bound = right;
    for (std::size_t iteration = 0;
         iteration < kHeadingBisectionIterations; ++iteration) {
      const long double candidate_time = 0.5L * (lower_bound + upper_bound);
      if (headingFromPiece(piece,
                           static_cast<double>(candidate_time * piece_duration), singul, yaw,
                           &ignored_angular_velocity)) {
        lower_bound = candidate_time;
      } else {
        upper_bound = candidate_time;
      }
    }
    return headingFromPiece(piece, static_cast<double>(lower_bound * piece_duration), singul, yaw,
                            &ignored_angular_velocity);
  }
  // ################################
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
