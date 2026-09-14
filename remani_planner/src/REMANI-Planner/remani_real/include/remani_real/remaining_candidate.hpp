#pragma once

#include <string>

#include <remani_real/candidate_trajectory.hpp>

namespace remani_real {

// ################################
// C++: RemainingCandidate suffix slicer begin
// ################################
struct RemainingCandidateResult {
  bool valid{false};
  FrozenCandidate candidate;
  std::string error_code;
  std::string detail;
};

class RemainingCandidate {
 public:
  // pause_param_time in [0, duration). Analytic piece restriction; no connector.
  static RemainingCandidateResult slice(FrozenCandidate original,
                                        double pause_param_time);
};
// ################################
// C++: RemainingCandidate suffix slicer end
// ################################

}  // namespace remani_real
