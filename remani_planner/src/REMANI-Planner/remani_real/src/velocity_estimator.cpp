#include <remani_real/velocity_estimator.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace remani_real {

// ################################
// C++: VelocityEstimator implementation begin
// ################################
VelocityEstimator::VelocityEstimator(std::size_t min_samples,
                                     double max_abs_velocity, double alpha,
                                     double max_dt)
    : min_samples_(min_samples),
      max_abs_velocity_(max_abs_velocity),
      alpha_(alpha),
      max_dt_(max_dt) {
  if (min_samples_ < 3 || !std::isfinite(max_abs_velocity_) ||
      max_abs_velocity_ <= 0.0 || !std::isfinite(alpha_) || alpha_ <= 0.0 ||
      alpha_ > 1.0 || !std::isfinite(max_dt_) || max_dt_ <= 0.0) {
    throw std::invalid_argument("VelocityEstimator parameters are invalid");
  }
}

void VelocityEstimator::reset() {
  samples_.clear();
  filtered_ = VelocityEstimate{};
  has_filtered_ = false;
}

VelocityEstimate VelocityEstimator::update(
    const ros::Time& stamp, const Eigen::Matrix<double, 6, 1>& q) {
  VelocityEstimate invalid;
  if (stamp.isZero() || !q.allFinite()) {
    reset();
    return invalid;
  }

  if (!samples_.empty()) {
    const double dt = (stamp - samples_.back().stamp).toSec();
    if (!(dt > 0.0)) {
      reset();
      return invalid;
    }
    if (dt > max_dt_) {
      reset();
      return invalid;
    }
  }

  samples_.push_back(Sample{stamp, q});
  while (samples_.size() > min_samples_) {
    samples_.pop_front();
  }
  if (samples_.size() < min_samples_) {
    return invalid;
  }

  const Sample& previous = samples_[samples_.size() - 2];
  const Sample& current = samples_.back();
  const double dt = (current.stamp - previous.stamp).toSec();
  if (!(dt > 0.0) || dt > max_dt_) {
    reset();
    return invalid;
  }

  Eigen::Matrix<double, 6, 1> raw = (current.q - previous.q) / dt;
  if (!raw.allFinite()) {
    reset();
    return invalid;
  }
  for (int index = 0; index < 6; ++index) {
    raw(index) = std::max(-max_abs_velocity_,
                          std::min(max_abs_velocity_, raw(index)));
  }

  VelocityEstimate estimate;
  if (!has_filtered_) {
    estimate.qd = raw;
  } else {
    estimate.qd = alpha_ * raw + (1.0 - alpha_) * filtered_.qd;
  }
  if (!estimate.qd.allFinite()) {
    reset();
    return invalid;
  }
  estimate.valid = true;
  filtered_ = estimate;
  has_filtered_ = true;
  return estimate;
}
// ################################
// C++: VelocityEstimator implementation end
// ################################

}  // namespace remani_real
