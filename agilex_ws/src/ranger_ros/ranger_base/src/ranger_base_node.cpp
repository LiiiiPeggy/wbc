/**
* @file ranger_base_node.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "ranger_base/ranger_messenger.hpp"
#include "ugv_sdk/details/robot_base/ranger_base.hpp"

using namespace westonrobot;

void SignalHandler(int s)
{
  // ################################
  // C++: request ros::shutdown so messenger destructor can stop begin
  // ################################
  printf("Caught signal %d, requesting ros::shutdown\n", s);
  ros::shutdown();
  // ################################
  // C++: request ros::shutdown so messenger destructor can stop end
  // ################################
}

void controlSingal()
{
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char** argv)
{
  // setup ROS node
  ros::init(argc, argv, "ranger_node");
  ros::NodeHandle node("~");

  controlSingal();

  RangerROSMessenger messenger(&node);
  messenger.Run();

  return 0;
}
