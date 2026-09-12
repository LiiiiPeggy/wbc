#include <remani_real/dry_run_motion_output.hpp>

namespace remani_real {

// ################################
// C++: DryRunMotionOutput implementation begin
// ################################
bool DryRunMotionOutput::publish(const geometry_msgs::Twist& command) {
  ranger_diagnostics_.push_back(command);
  return true;
}

bool DryRunMotionOutput::send(const trajectory_msgs::JointTrajectory& trajectory) {
  arm_diagnostics_.push_back(trajectory);
  arm_state_ = ArmGoalState::Pending;
  return true;
}

bool DryRunMotionOutput::cancel() {
  arm_state_ = ArmGoalState::Canceled;
  return true;
}

bool DryRunMotionOutput::stop() {
  arm_state_ = ArmGoalState::Aborted;
  return true;
}

ArmGoalState DryRunMotionOutput::state() const { return arm_state_; }

bool DryRunMotionOutput::hardwareOutputEnabled() const { return false; }

const std::vector<geometry_msgs::Twist>& DryRunMotionOutput::rangerDiagnostics()
    const {
  return ranger_diagnostics_;
}

const std::vector<trajectory_msgs::JointTrajectory>&
DryRunMotionOutput::armDiagnostics() const {
  return arm_diagnostics_;
}

std::size_t DryRunMotionOutput::hardwarePublishCount() const { return 0; }

std::size_t DryRunMotionOutput::actionGoalCount() const { return 0; }

std::size_t DryRunMotionOutput::writeServiceCount() const { return 0; }
// ################################
// C++: DryRunMotionOutput implementation end
// ################################

}  // namespace remani_real
