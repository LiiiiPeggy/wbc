#pragma once

#include <stdexcept>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <remani_real/motion_output.hpp>
#include <remani_real/topic_ownership_monitor.hpp>

namespace remani_real {

// ################################
// C++: RangerHardwareChannel non-dry adapter begin
// ################################
constexpr char kRangerHardwareCmdVelTopic[] = "/remani/hardware/ranger/cmd_vel";

class RangerHardwareChannel : public RangerCommandChannel {
 public:
  RangerHardwareChannel(ros::NodeHandle& nh, const std::string& hardware_topic,
                        const std::string& expected_node)
      : ownership_monitor_(hardware_topic, expected_node) {
    if (hardware_topic != kRangerHardwareCmdVelTopic) {
      throw std::invalid_argument(
          "RangerHardwareChannel rejects topic other than "
          "/remani/hardware/ranger/cmd_vel");
    }
    publisher_ = nh.advertise<geometry_msgs::Twist>(hardware_topic, 1, false);
  }

  bool publish(const geometry_msgs::Twist& command) override {
    if (!ownershipHealthy()) {
      return false;
    }
    publisher_.publish(command);
    return true;
  }

  bool hardwareOutputEnabled() const override { return true; }

  bool ownershipHealthy() const;

 private:
  ros::Publisher publisher_;
  TopicOwnershipMonitor ownership_monitor_;
};
// ################################
// C++: RangerHardwareChannel non-dry adapter end
// ################################

}  // namespace remani_real
