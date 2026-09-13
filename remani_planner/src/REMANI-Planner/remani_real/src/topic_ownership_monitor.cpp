#include <remani_real/topic_ownership_monitor.hpp>

namespace remani_real {

// ################################
// C++: topic ownership implementation begin
// ################################
bool IsUniqueOwner(const std::vector<std::string>& publisher_nodes,
                   const std::string& expected_node) {
  if (publisher_nodes.size() != 1) {
    return false;
  }
  return publisher_nodes.front() == expected_node;
}

TopicOwnershipMonitor::TopicOwnershipMonitor(std::string topic,
                                             std::string expected_node)
    : topic_(std::move(topic)), expected_node_(std::move(expected_node)) {}

bool TopicOwnershipMonitor::isHealthy(
    const std::vector<std::string>& publisher_nodes) const {
  return IsUniqueOwner(publisher_nodes, expected_node_);
}
// ################################
// C++: topic ownership implementation end
// ################################

}  // namespace remani_real
