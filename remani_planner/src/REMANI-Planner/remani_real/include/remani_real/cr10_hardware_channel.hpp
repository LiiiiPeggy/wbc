#pragma once

#include <string>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <remani_real/motion_output.hpp>
#include <remani_real_msgs/Cr10EmergencyStop.h>
#include <remani_real_msgs/Cr10Stop.h>

namespace remani_real {

// ################################
// C++: CR10 readiness + hardware Action channel begin
// ################################
class Cr10ReadinessClient {
 public:
  explicit Cr10ReadinessClient(const std::string& action_name);
  bool serverReady(const ros::Duration& timeout) const;

 private:
  mutable actionlib::SimpleActionClient<
      control_msgs::FollowJointTrajectoryAction>
      client_;
};

class Cr10HardwareChannel : public ArmCommandChannel {
 public:
  Cr10HardwareChannel(ros::NodeHandle& nh, const std::string& action_name,
                      const std::string& stop_service,
                      const std::string& emergency_stop_service,
                      bool emergency_stop_on_stop_failure);

  bool send(const trajectory_msgs::JointTrajectory& trajectory) override;
  bool cancel() override;
  bool stop() override;
  ArmGoalState state() const override;
  bool hardwareOutputEnabled() const override { return true; }

 private:
  void doneCb(const actionlib::SimpleClientGoalState& goal_state,
              const control_msgs::FollowJointTrajectoryResultConstPtr& result);
  void activeCb();

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      client_;
  ros::ServiceClient stop_client_;
  ros::ServiceClient emergency_stop_client_;
  bool emergency_stop_on_stop_failure_{false};
  ArmGoalState state_{ArmGoalState::Unavailable};
};
// ################################
// C++: CR10 readiness + hardware Action channel end
// ################################

}  // namespace remani_real
