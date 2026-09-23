#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace dobot_v4_bringup {
namespace test_fixtures {

// ################################
inline std::vector<std::string> canonicalNames() {
  return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

inline trajectory_msgs::JointTrajectory validTrajectory(
    const std::vector<std::string>& names) {
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = names;
  trajectory_msgs::JointTrajectoryPoint p0;
  p0.positions = {0.30, 0.10, 0.60, 0.20, 0.50, 0.40};
  p0.velocities.assign(6, 0.0);
  p0.time_from_start = ros::Duration(0.0);
  trajectory_msgs::JointTrajectoryPoint p1 = p0;
  for (double& q : p1.positions) {
    q += 0.01;
  }
  p1.time_from_start = ros::Duration(1.0);
  trajectory.points = {p0, p1};
  return trajectory;
}
// ################################

}  // namespace test_fixtures
}  // namespace dobot_v4_bringup
