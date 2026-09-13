#include <remani_real/execution_clock.hpp>

#include <cmath>
#include <stdexcept>

namespace remani_real {

// ################################
// C++: ExecutionClock implementation begin
// ################################
ExecutionEpoch ExecutionClock::begin(const ros::SteadyTime& requested_at,
                                     double start_lead_time) {
  if (!std::isfinite(start_lead_time) || !(start_lead_time > 0.0)) {
    throw std::invalid_argument("start_lead_time must be finite and > 0");
  }
  ExecutionEpoch epoch;
  epoch.requested_at = requested_at;
  epoch.start_lead_time = start_lead_time;
  epoch.t0.fromSec(requested_at.toSec() + start_lead_time);
  return epoch;
}

boost::optional<double> ExecutionClock::parameterTime(
    const ExecutionEpoch& epoch, const ros::SteadyTime& now) {
  if (now.toSec() < epoch.t0.toSec()) {
    return boost::none;
  }
  return std::max(0.0, now.toSec() - epoch.t0.toSec());
}
// ################################
// C++: ExecutionClock implementation end
// ################################

}  // namespace remani_real
