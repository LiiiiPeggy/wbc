#include <remani_real/joint_state_mapper.hpp>

#include <cmath>
#include <unordered_map>

namespace remani_real {
namespace {

// ################################
// C++: JointStateMapper name tables begin
// ################################
const std::array<std::string, 6> kRawNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
const std::array<std::string, 6> kModelNames = {
    "cr10_joint1", "cr10_joint2", "cr10_joint3",
    "cr10_joint4", "cr10_joint5", "cr10_joint6"};
// ################################
// C++: JointStateMapper name tables end
// ################################

}  // namespace

// ################################
// C++: JointStateMapper::map begin
// ################################
MappedJointState JointStateMapper::map(const sensor_msgs::JointState& raw) {
  MappedJointState out;
  if (raw.name.size() != raw.position.size()) {
    return out;
  }

  std::unordered_map<std::string, std::size_t> index_by_name;
  index_by_name.reserve(raw.name.size());
  for (std::size_t i = 0; i < raw.name.size(); ++i) {
    const auto& name = raw.name[i];
    if (index_by_name.count(name) != 0u) {
      return out;  // duplicate name
    }
    index_by_name.emplace(name, i);
  }

  std::array<double, 6> positions{};
  for (std::size_t j = 0; j < kRawNames.size(); ++j) {
    const auto it = index_by_name.find(kRawNames[j]);
    if (it == index_by_name.end()) {
      return out;  // missing required joint
    }
    const double value = raw.position[it->second];
    if (!std::isfinite(value)) {
      return out;
    }
    positions[j] = value;
  }

  out.valid = true;
  out.position = positions;
  out.planning_msg.header = raw.header;
  out.planning_msg.name.assign(kModelNames.begin(), kModelNames.end());
  out.planning_msg.position.assign(positions.begin(), positions.end());
  // Velocity left empty until VelocityEstimator reports valid.
  out.planning_msg.velocity.clear();
  out.planning_msg.effort.clear();
  return out;
}
// ################################
// C++: JointStateMapper::map end
// ################################

}  // namespace remani_real
