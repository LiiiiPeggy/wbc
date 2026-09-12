#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <quadrotor_msgs/PolynomialTraj.h>
#include <ros/time.h>

#include <remani_real/candidate_trajectory.hpp>

namespace remani_real {

// ################################
// C++: CandidateAssembler protocol types begin
// ################################
enum class AssemblyState { Idle, Assembling, Complete, Invalid };

struct AssemblyEvent {
  bool accepted{false};
  AssemblyState state{AssemblyState::Idle};
  uint64_t candidate_id{0};
  ros::Time raw_transaction_stamp;
  std::string error_code;
  std::string detail;
};

class CandidateAssembler {
 public:
  explicit CandidateAssembler(double timeout_sec);

  AssemblyEvent consume(const quadrotor_msgs::PolynomialTraj& msg,
                        const ros::SteadyTime& now,
                        bool new_transaction_allowed,
                        double actual_start_yaw);
  FrozenCandidate completedCandidate() const;
  void invalidate(const std::string& reason);
  // ################################
  // C++: poll assembly timeout without a candidate message begin
  // ################################
  AssemblyEvent pollTimeout(const ros::SteadyTime& now);
  // ################################
  // C++: poll assembly timeout without a candidate message end
  // ################################

 private:
  AssemblyEvent makeEvent(bool accepted, AssemblyState state,
                          const std::string& error_code = std::string(),
                          const std::string& detail = std::string()) const;
  bool timedOut(const ros::SteadyTime& now) const;
  bool parseAdd(const quadrotor_msgs::PolynomialTraj& msg,
                CandidateSegment* segment, std::string* error_code,
                std::string* detail) const;

  double timeout_sec_{60.0};
  uint64_t next_candidate_id_{1};
  AssemblyState state_{AssemblyState::Idle};
  uint64_t assembling_candidate_id_{0};
  ros::Time raw_transaction_stamp_;
  ros::SteadyTime assembly_started_at_;
  double start_yaw_{0.0};
  uint32_t expected_trajectory_id_{1};
  std::vector<CandidateSegment> assembling_segments_;
  FrozenCandidate completed_;
};
// ################################
// C++: CandidateAssembler protocol types end
// ################################

}  // namespace remani_real
