#pragma once

#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace remani_real {

// ################################
// C++: motion output channel boundaries begin
// ################################
class RangerCommandChannel {
 public:
  virtual ~RangerCommandChannel() = default;
  virtual bool publish(const geometry_msgs::Twist& command) = 0;
  virtual bool hardwareOutputEnabled() const = 0;
};

enum class ArmGoalState {
  Unavailable,
  Pending,
  Accepted,
  Active,
  Succeeded,
  Canceled,
  Aborted
};

class ArmCommandChannel {
 public:
  virtual ~ArmCommandChannel() = default;
  virtual bool send(const trajectory_msgs::JointTrajectory& trajectory) = 0;
  virtual bool cancel() = 0;
  virtual bool stop() = 0;
  virtual ArmGoalState state() const = 0;
  virtual bool hardwareOutputEnabled() const = 0;
};
// ################################
// C++: motion output channel boundaries end
// ################################

}  // namespace remani_real
