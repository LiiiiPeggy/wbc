#pragma once

#include <Eigen/Core>
#include <ros/time.h>

namespace remani_real {

/* ---------- Immutable actual-state snapshot shared by Gate and Executor. ---------- */
struct ActualStateSnapshot {
  ros::SteadyTime captured_at;
  ros::Time ros_stamp;
  Eigen::Vector2d base_xy{Eigen::Vector2d::Zero()};
  double base_yaw{0.0};
  Eigen::Vector2d base_velocity_world{Eigen::Vector2d::Zero()};
  double base_yaw_rate{0.0};
  Eigen::Matrix<double, 6, 1> q{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> qd{Eigen::Matrix<double, 6, 1>::Zero()};
  bool odom_valid{false};
  bool joints_valid{false};
  bool velocity_valid{false};
  bool tf_valid{false};
  bool robot_connected{false};
  bool robot_enabled{false};
  bool robot_fault{false};
};

}  // namespace remani_real
