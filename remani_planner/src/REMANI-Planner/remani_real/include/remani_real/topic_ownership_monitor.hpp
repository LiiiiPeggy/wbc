#pragma once

#include <string>
#include <vector>

namespace remani_real {

// ################################
// C++: topic ownership decision helpers begin
// ################################
// Pure decision: exactly one publisher and it matches expected_node.
bool IsUniqueOwner(const std::vector<std::string>& publisher_nodes,
                   const std::string& expected_node);

class TopicOwnershipMonitor {
 public:
  TopicOwnershipMonitor(std::string topic, std::string expected_node);

  const std::string& topic() const { return topic_; }
  const std::string& expectedNode() const { return expected_node_; }

  // publisher_nodes come from ros::master::getSystemState callers.
  bool isHealthy(const std::vector<std::string>& publisher_nodes) const;

 private:
  std::string topic_;
  std::string expected_node_;
};
// ################################
// C++: topic ownership decision helpers end
// ################################

}  // namespace remani_real
