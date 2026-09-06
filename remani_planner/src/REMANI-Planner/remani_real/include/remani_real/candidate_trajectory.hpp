#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <ros/time.h>

#include <mm_controller/polynomial_trajectory.h>

namespace remani_real {

/* ---------- Immutable sample of an eight-dimensional whole-body candidate. ---------- */
struct WholeBodySample {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  double base_yaw{0.0};
  double base_angular_velocity{0.0};
  int singul{0};
};

/* ---------- One Gate-assembled polynomial candidate segment. ---------- */
struct CandidateSegment {
  uint32_t trajectory_id{0};
  int singul{0};
  MMController::Trajectory trajectory;
  double start_time{0.0};
  double duration{0.0};
};

/* ---------- Gate-owned, externally immutable candidate trajectory. ---------- */
class CandidateTrajectory {
 public:
  CandidateTrajectory(uint64_t candidate_id,
                      std::vector<CandidateSegment> segments,
                      double start_yaw,
                      const ros::Time& raw_transaction_stamp = ros::Time(0));

  uint64_t id() const;
  double duration() const;
  const std::vector<CandidateSegment>& segments() const;
  const ros::Time& rawTransactionStamp() const;
  WholeBodySample sample(double t) const;

 private:
  uint64_t candidate_id_{0};
  std::vector<CandidateSegment> segments_;
  double start_yaw_{0.0};
  double duration_{0.0};
  ros::Time raw_transaction_stamp_;
};

using FrozenCandidate = std::shared_ptr<const CandidateTrajectory>;

}  // namespace remani_real
