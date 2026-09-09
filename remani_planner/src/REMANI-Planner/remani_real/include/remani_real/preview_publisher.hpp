#pragma once

#include <string>
#include <vector>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <remani_real/candidate_validator.hpp>

namespace remani_real {

// ################################
// C++: PreviewPublisher interface begin
// ################################
class PreviewPublisher {
 public:
  explicit PreviewPublisher(ros::NodeHandle& nh);

  void publish(const FrozenCandidate& candidate,
               const ValidationReport& report,
               const ValidationEnvironment* environment);

  const std::vector<std::string>& advertisedTopics() const;

 private:
  ros::Publisher robot_pub_;
  ros::Publisher base_path_pub_;
  ros::Publisher ee_path_pub_;
  std::vector<std::string> advertised_topics_;
};
// ################################
// C++: PreviewPublisher interface end
// ################################

}  // namespace remani_real
