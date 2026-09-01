#include "plan_manage/candidate_transaction_builder.hpp"

#include <stdexcept>

namespace remani_planner {
namespace {

std::vector<quadrotor_msgs::PolynomialTraj> buildAdds(
    const SingulTrajData& data, const ros::Time& transaction_stamp,
    bool use_segment_sequence) {
  std::vector<quadrotor_msgs::PolynomialTraj> messages;
  messages.reserve(data.singul_traj.size());

  for (size_t i = 0; i < data.singul_traj.size(); ++i) {
    const auto& segment = data.singul_traj[i];
    quadrotor_msgs::PolynomialTraj message;
    message.trajectory_id = use_segment_sequence ? i + 1 : segment.traj_id;
    message.header.stamp = transaction_stamp;
    message.action = quadrotor_msgs::PolynomialTraj::ACTION_ADD;
    message.singul = segment.singul;

    const int piece_num = segment.traj.getPieceNum();
    message.trajectory.reserve(piece_num);
    for (int j = 0; j < piece_num; ++j) {
      const auto& piece = segment.traj.getPiece(j);
      quadrotor_msgs::PolynomialMatrix piece_msg;
      piece_msg.num_dim = piece.getDim();
      piece_msg.num_order = piece.getDegree();
      piece_msg.duration = piece.getDuration();
      const auto coeff = piece.getCoeffMat();
      piece_msg.data.assign(coeff.data(), coeff.data() + coeff.size());
      message.trajectory.emplace_back(piece_msg);
    }

    messages.emplace_back(message);
  }

  return messages;
}

}  // namespace

std::vector<quadrotor_msgs::PolynomialTraj>
CandidateTransactionBuilder::buildExternal(const SingulTrajData& data,
                                           const ros::Time& transaction_stamp) {
  if (data.singul_traj.empty()) {
    throw std::invalid_argument("external candidate transaction is empty");
  }

  auto messages = buildAdds(data, transaction_stamp, true);
  messages.insert(messages.begin(),
                  buildControl(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START,
                               transaction_stamp));
  messages.emplace_back(
      buildControl(quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL,
                   transaction_stamp));
  return messages;
}

std::vector<quadrotor_msgs::PolynomialTraj>
CandidateTransactionBuilder::buildInternalAdds(const SingulTrajData& data,
                                               const ros::Time& transaction_stamp) {
  return buildAdds(data, transaction_stamp, false);
}

quadrotor_msgs::PolynomialTraj CandidateTransactionBuilder::buildControl(
    const uint32_t action, const ros::Time& transaction_stamp) {
  quadrotor_msgs::PolynomialTraj message;
  message.trajectory_id = 0;
  message.header.stamp = transaction_stamp;
  message.action = action;
  message.singul = 0;
  return message;
}

}  // namespace remani_planner
