#pragma once

#include <string>

#include <geometry_msgs/Twist.h>

#include <remani_real/actual_state.hpp>
#include <remani_real/candidate_trajectory.hpp>

namespace remani_real {

// ################################
// C++: BaseTrackingController types begin
// ################################
struct BaseTrackingConfig {
  double k_x{0.0};
  double k_y{0.0};
  double k_yaw{0.0};
  double max_linear{0.10};
  double max_angular{0.15};
  double max_linear_correction{0.03};
  double max_angular_correction{0.05};
  double max_position_error{0.20};
  double max_yaw_error{0.20};
};

struct BaseTrackingResult {
  bool valid{false};
  geometry_msgs::Twist command;
  double position_error{0.0};
  double yaw_error{0.0};
  std::string error_code;
};

class BaseTrackingController {
 public:
  explicit BaseTrackingController(BaseTrackingConfig config);

  BaseTrackingResult compute(const WholeBodySample& desired,
                             const ActualStateSnapshot& actual) const;

 private:
  BaseTrackingConfig config_;
};
// ################################
// C++: BaseTrackingController types end
// ################################

}  // namespace remani_real
