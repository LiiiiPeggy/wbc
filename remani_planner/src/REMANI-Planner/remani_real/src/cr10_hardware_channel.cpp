#include <remani_real/cr10_hardware_channel.hpp>

#include <boost/bind.hpp>

namespace remani_real {

// ################################
// C++: Cr10ReadinessClient / Cr10HardwareChannel begin
// ################################
Cr10ReadinessClient::Cr10ReadinessClient(const std::string& action_name)
    : client_(action_name, true) {}

bool Cr10ReadinessClient::serverReady(const ros::Duration& timeout) const {
  return client_.waitForServer(timeout);
}

Cr10HardwareChannel::Cr10HardwareChannel(
    ros::NodeHandle& nh, const std::string& action_name,
    const std::string& stop_service, const std::string& emergency_stop_service,
    bool emergency_stop_on_stop_failure)
    : client_(action_name, true),
      emergency_stop_on_stop_failure_(emergency_stop_on_stop_failure) {
  stop_client_ = nh.serviceClient<remani_real_msgs::Cr10Stop>(stop_service);
  emergency_stop_client_ =
      nh.serviceClient<remani_real_msgs::Cr10EmergencyStop>(
          emergency_stop_service);
  state_ = ArmGoalState::Unavailable;
}

bool Cr10HardwareChannel::send(
    const trajectory_msgs::JointTrajectory& trajectory) {
  if (!client_.isServerConnected()) {
    state_ = ArmGoalState::Unavailable;
    return false;
  }
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  state_ = ArmGoalState::Pending;
  client_.sendGoal(
      goal,
      boost::bind(&Cr10HardwareChannel::doneCb, this, _1, _2),
      boost::bind(&Cr10HardwareChannel::activeCb, this),
      actionlib::SimpleActionClient<
          control_msgs::FollowJointTrajectoryAction>::SimpleFeedbackCallback());
  return true;
}

bool Cr10HardwareChannel::cancel() {
  if (!client_.isServerConnected()) {
    return false;
  }
  client_.cancelGoal();
  if (state_ == ArmGoalState::Pending || state_ == ArmGoalState::Accepted ||
      state_ == ArmGoalState::Active) {
    state_ = ArmGoalState::Canceled;
  }
  return true;
}

bool Cr10HardwareChannel::stop() {
  remani_real_msgs::Cr10Stop stop_srv;
  const bool stop_ok =
      stop_client_.exists() && stop_client_.call(stop_srv) && stop_srv.response.res == 0;
  if (stop_ok) {
    state_ = ArmGoalState::Aborted;
    return true;
  }
  if (emergency_stop_on_stop_failure_) {
    remani_real_msgs::Cr10EmergencyStop estop_srv;
    estop_srv.request.value = 1;
    if (emergency_stop_client_.exists() &&
        emergency_stop_client_.call(estop_srv) &&
        estop_srv.response.res == 0) {
      state_ = ArmGoalState::Aborted;
      return false;
    }
  }
  state_ = ArmGoalState::Aborted;
  return false;
}

ArmGoalState Cr10HardwareChannel::state() const { return state_; }

void Cr10HardwareChannel::doneCb(
    const actionlib::SimpleClientGoalState& goal_state,
    const control_msgs::FollowJointTrajectoryResultConstPtr& /*result*/) {
  switch (goal_state.state_) {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      state_ = ArmGoalState::Succeeded;
      break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
    case actionlib::SimpleClientGoalState::RECALLED:
      state_ = ArmGoalState::Canceled;
      break;
    default:
      state_ = ArmGoalState::Aborted;
      break;
  }
}

void Cr10HardwareChannel::activeCb() {
  if (state_ == ArmGoalState::Pending) {
    state_ = ArmGoalState::Active;
  } else if (state_ == ArmGoalState::Unavailable) {
    state_ = ArmGoalState::Accepted;
  }
}
// ################################
// C++: Cr10ReadinessClient / Cr10HardwareChannel end
// ################################

}  // namespace remani_real
