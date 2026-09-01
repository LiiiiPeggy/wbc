#ifndef PLAN_MANAGE_CANDIDATE_TRANSACTION_BUILDER_HPP_
#define PLAN_MANAGE_CANDIDATE_TRANSACTION_BUILDER_HPP_

#include <cstdint>
#include <vector>

#include <ros/time.h>

#include <quadrotor_msgs/PolynomialTraj.h>
#include <traj_utils/plan_container.hpp>

namespace remani_planner {

class CandidateTransactionBuilder {
 public:
  static std::vector<quadrotor_msgs::PolynomialTraj> buildExternal(
      const SingulTrajData& data, const ros::Time& transaction_stamp);

  static std::vector<quadrotor_msgs::PolynomialTraj> buildInternalAdds(
      const SingulTrajData& data, const ros::Time& transaction_stamp);

  static quadrotor_msgs::PolynomialTraj buildControl(
      uint32_t action, const ros::Time& transaction_stamp);
};

}  // namespace remani_planner

#endif  // PLAN_MANAGE_CANDIDATE_TRANSACTION_BUILDER_HPP_
