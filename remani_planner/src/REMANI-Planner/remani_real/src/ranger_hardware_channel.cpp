#include <remani_real/ranger_hardware_channel.hpp>

#include <ros/master.h>

namespace remani_real {
namespace {

std::vector<std::string> publishersForTopic(const std::string& topic) {
  std::vector<std::string> nodes;
  ros::master::V_TopicInfo topics;
  // Use getSystemState for publisher node names.
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", args, result, payload, true)) {
    return nodes;
  }
  if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 1) {
    return nodes;
  }
  XmlRpc::XmlRpcValue& publishers = payload[0];
  if (publishers.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return nodes;
  }
  for (int i = 0; i < publishers.size(); ++i) {
    XmlRpc::XmlRpcValue& entry = publishers[i];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2) {
      continue;
    }
    const std::string name = static_cast<std::string>(entry[0]);
    if (name != topic) {
      continue;
    }
    XmlRpc::XmlRpcValue& pubs = entry[1];
    if (pubs.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      continue;
    }
    for (int j = 0; j < pubs.size(); ++j) {
      nodes.push_back(static_cast<std::string>(pubs[j]));
    }
  }
  return nodes;
}

}  // namespace

// ################################
// C++: RangerHardwareChannel ownershipHealthy begin
// ################################
bool RangerHardwareChannel::ownershipHealthy() const {
  return ownership_monitor_.isHealthy(
      publishersForTopic(ownership_monitor_.topic()));
}
// ################################
// C++: RangerHardwareChannel ownershipHealthy end
// ################################

}  // namespace remani_real
