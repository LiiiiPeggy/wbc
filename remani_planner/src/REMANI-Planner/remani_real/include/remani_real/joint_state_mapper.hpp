#pragma once

#include <array>
#include <string>

#include <sensor_msgs/JointState.h>

namespace remani_real {

// ################################
// C++: JointStateMapper interface begin
// ################################
struct MappedJointState {
  bool valid{false};
  std::array<double, 6> position{{0, 0, 0, 0, 0, 0}};
  sensor_msgs::JointState planning_msg;
};

class JointStateMapper {
 public:
  static MappedJointState map(const sensor_msgs::JointState& raw);
};
// ################################
// C++: JointStateMapper interface end
// ################################

}  // namespace remani_real
