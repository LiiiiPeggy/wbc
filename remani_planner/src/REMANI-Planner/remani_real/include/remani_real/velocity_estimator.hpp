#pragma once

#include <cstddef>
#include <deque>

#include <Eigen/Core>
#include <ros/time.h>

namespace remani_real {

// ################################
// C++: VelocityEstimator interface begin
// ################################
struct VelocityEstimate {
  Eigen::Matrix<double, 6, 1> qd{Eigen::Matrix<double, 6, 1>::Zero()};
  bool valid{false};
};

class VelocityEstimator {
 public:
  VelocityEstimator(std::size_t min_samples, double max_abs_velocity,
                    double alpha, double max_dt = 0.2);

  VelocityEstimate update(const ros::Time& stamp,
                          const Eigen::Matrix<double, 6, 1>& q);
  void reset();

 private:
  struct Sample {
    ros::Time stamp;
    Eigen::Matrix<double, 6, 1> q{Eigen::Matrix<double, 6, 1>::Zero()};
  };

  std::size_t min_samples_{3};
  double max_abs_velocity_{0.0};
  double alpha_{1.0};
  double max_dt_{0.2};
  std::deque<Sample> samples_;
  VelocityEstimate filtered_;
  bool has_filtered_{false};
};
// ################################
// C++: VelocityEstimator interface end
// ################################

}  // namespace remani_real
