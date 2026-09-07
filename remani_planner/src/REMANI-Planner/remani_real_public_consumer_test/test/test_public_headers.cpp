#include <remani_real/actual_state.hpp>
#include <remani_real/candidate_trajectory.hpp>

int main() {
  remani_real::ActualStateSnapshot snapshot;
  remani_real::FrozenCandidate candidate;
  return snapshot.odom_valid || candidate ? 1 : 0;
}
