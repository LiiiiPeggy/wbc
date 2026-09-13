#pragma once

#include <boost/optional.hpp>
#include <ros/time.h>

namespace remani_real {

// ################################
// C++: ExecutionClock / ExecutionEpoch begin
// ################################
struct ExecutionEpoch {
  ros::SteadyTime requested_at;
  ros::SteadyTime t0;
  double start_lead_time{0.0};
};

class ExecutionClock {
 public:
  static ExecutionEpoch begin(const ros::SteadyTime& requested_at,
                              double start_lead_time);
  static boost::optional<double> parameterTime(const ExecutionEpoch& epoch,
                                               const ros::SteadyTime& now);
};
// ################################
// C++: ExecutionClock / ExecutionEpoch end
// ################################

}  // namespace remani_real
