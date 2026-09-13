#pragma once

#include <cstdint>

#include <geometry_msgs/Twist.h>

namespace westonrobot {

// ################################
// C++: finite Ranger Twist decision types begin
// ################################
struct RangerCommandLimits {
  double track{0.0};
  double wheelbase{0.0};
  double max_linear_speed{0.0};
  double max_angular_speed{0.0};
  double max_speed_cmd{0.0};
  double max_steer_angle_central{0.0};
  double max_steer_angle_parallel{0.0};
  double max_round_angle{0.0};
  double min_turn_radius{0.0};
  // Mini V1 may treat vx~0 + vy as SIDE_SLIP; others keep PARALLEL.
  bool allow_side_slip{false};
};

struct RangerCommandDecision {
  bool valid{false};
  bool stop{false};
  uint8_t motion_mode{0};  // ranger_msgs::MotionState constants
  double linear{0.0};
  double steering{0.0};
  double angular{0.0};
  double turn_radius{0.0};
};

// Pure Twist -> command decision. Never touches hardware / RangerRobot.
RangerCommandDecision ComputeRangerCommand(const geometry_msgs::Twist& msg,
                                           const RangerCommandLimits& limits,
                                           double zero_epsilon);
// ################################
// C++: finite Ranger Twist decision types end
// ################################

}  // namespace westonrobot
