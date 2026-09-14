#include <remani_real/remaining_candidate.hpp>

#include <cmath>
#include <utility>
#include <vector>

namespace remani_real {
namespace {

double binomial(int n, int k) {
  if (k < 0 || k > n) {
    return 0.0;
  }
  if (k == 0 || k == n) {
    return 1.0;
  }
  double result = 1.0;
  for (int i = 1; i <= k; ++i) {
    result *= static_cast<double>(n - k + i);
    result /= static_cast<double>(i);
  }
  return result;
}

// coeff.col(DEGREE-k) multiplies t^k in Piece::getPos.
MMController::Piece::CoefficientMat shiftCoefficients(
    const MMController::Piece::CoefficientMat& coeff, double shift) {
  const int degree = static_cast<int>(coeff.cols()) - 1;
  MMController::Piece::CoefficientMat out =
      MMController::Piece::CoefficientMat::Zero(coeff.rows(), coeff.cols());
  for (int j = 0; j <= degree; ++j) {
    Eigen::VectorXd new_aj = Eigen::VectorXd::Zero(coeff.rows());
    for (int k = j; k <= degree; ++k) {
      const double weight =
          binomial(k, j) * std::pow(shift, static_cast<double>(k - j));
      new_aj += weight * coeff.col(degree - k);
    }
    out.col(degree - j) = new_aj;
  }
  return out;
}

std::size_t segmentIndexAt(const std::vector<CandidateSegment>& segments,
                           double total_duration, double t) {
  if (t == total_duration) {
    return segments.size() - 1;
  }
  for (std::size_t index = 0; index < segments.size(); ++index) {
    if (t < segments[index].start_time + segments[index].duration) {
      return index;
    }
  }
  return segments.size() - 1;
}

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

// ################################
// C++: RemainingCandidate::slice begin
// ################################
RemainingCandidateResult RemainingCandidate::slice(FrozenCandidate original,
                                                   double pause_param_time) {
  RemainingCandidateResult out;
  if (!original) {
    out.error_code = "REMAINING_NULL";
    return out;
  }
  const double duration = original->duration();
  if (!std::isfinite(pause_param_time) || pause_param_time < 0.0 ||
      !(pause_param_time < duration)) {
    out.error_code = "REMAINING_TIME";
    out.detail = "pause_param_time must be in [0, duration)";
    return out;
  }

  WholeBodySample at_pause;
  try {
    at_pause = original->sample(pause_param_time);
  } catch (const std::exception& ex) {
    out.error_code = "REMAINING_SAMPLE";
    out.detail = ex.what();
    return out;
  }

  const std::vector<CandidateSegment>& segments = original->segments();
  const std::size_t seg_i = segmentIndexAt(segments, duration, pause_param_time);
  const CandidateSegment& cut_segment = segments[seg_i];
  const double segment_time = pause_param_time - cut_segment.start_time;
  const std::size_t piece_i =
      pieceIndexAt(cut_segment.trajectory, segment_time);
  const double piece_time =
      localPieceTime(cut_segment.trajectory, piece_i, segment_time);
  const MMController::Piece& cut_piece =
      cut_segment.trajectory[static_cast<int>(piece_i)];
  const double remaining_piece_duration = cut_piece.getDuration() - piece_time;
  if (!(remaining_piece_duration > 0.0)) {
    out.error_code = "REMAINING_EMPTY";
    return out;
  }

  std::vector<CandidateSegment> remaining;
  CandidateSegment first;
  first.trajectory_id = cut_segment.trajectory_id;
  first.singul = cut_segment.singul;
  first.start_time = 0.0;
  MMController::Trajectory first_traj;
  first_traj.emplace_back(
      remaining_piece_duration,
      shiftCoefficients(cut_piece.getCoeffMat(), piece_time));
  for (int p = static_cast<int>(piece_i) + 1;
       p < cut_segment.trajectory.getPieceNum(); ++p) {
    const MMController::Piece& piece = cut_segment.trajectory[p];
    first_traj.emplace_back(piece.getDuration(), piece.getCoeffMat());
  }
  first.trajectory = first_traj;
  first.duration = first_traj.getTotalDuration();
  remaining.push_back(first);

  double cursor = first.duration;
  for (std::size_t s = seg_i + 1; s < segments.size(); ++s) {
    CandidateSegment next = segments[s];
    next.start_time = cursor;
    remaining.push_back(next);
    cursor += next.duration;
  }

  try {
    out.candidate = std::make_shared<const CandidateTrajectory>(
        original->id(), std::move(remaining), at_pause.base_yaw,
        original->rawTransactionStamp());
  } catch (const std::exception& ex) {
    out.error_code = "REMAINING_BUILD";
    out.detail = ex.what();
    return out;
  }

  // Invariant: first sample matches original at pause time.
  try {
    const WholeBodySample got = out.candidate->sample(0.0);
    if (!(got.position.isApprox(at_pause.position, 1e-9) &&
          got.velocity.isApprox(at_pause.velocity, 1e-9))) {
      out.error_code = "REMAINING_MISMATCH";
      out.candidate.reset();
      return out;
    }
  } catch (const std::exception& ex) {
    out.error_code = "REMAINING_VERIFY";
    out.detail = ex.what();
    out.candidate.reset();
    return out;
  }

  out.valid = true;
  return out;
}
// ################################
// C++: RemainingCandidate::slice end
// ################################

}  // namespace remani_real
