#pragma once

#include <cstddef>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <remani_real/motion_output.hpp>

namespace remani_real {

// ################################
// C++: DryRunMotionOutput zero-hardware adapter begin
// ################################
class DryRunMotionOutput : public RangerCommandChannel,
                           public ArmCommandChannel {
 public:
  bool publish(const geometry_msgs::Twist& command) override;
  bool send(const trajectory_msgs::JointTrajectory& trajectory) override;
  bool cancel() override;
  bool stop() override;
  ArmGoalState state() const override;
  bool hardwareOutputEnabled() const override;

  const std::vector<geometry_msgs::Twist>& rangerDiagnostics() const;
  const std::vector<trajectory_msgs::JointTrajectory>& armDiagnostics() const;
  std::size_t hardwarePublishCount() const;
  std::size_t actionGoalCount() const;
  std::size_t writeServiceCount() const;

 private:
  std::vector<geometry_msgs::Twist> ranger_diagnostics_;
  std::vector<trajectory_msgs::JointTrajectory> arm_diagnostics_;
  ArmGoalState arm_state_{ArmGoalState::Unavailable};
};
// ################################
// C++: DryRunMotionOutput zero-hardware adapter end
// ################################

}  // namespace remani_real
