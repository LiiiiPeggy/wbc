#pragma once

#include <array>
#include <cstddef>

#include <dobot_v4_bringup/follow_joint_trajectory_adapter.hpp>

namespace dobot_v4_bringup {

// ################################
// C++: FakeCommander for adapter gtests begin
// ################################
class FakeCommander : public Cr10CommandSink {
 public:
  bool sendServoJ(const std::array<double, 6>& q_rad,
                  double /*duration_sec*/) override {
    ++servoj_calls_;
    last_command_ = q_rad;
    actual_q_ = q_rad;
    return true;
  }

  bool stop() override {
    ++stop_calls_;
    actual_qd_.fill(0.0);
    return true;
  }

  std::array<double, 6> actualQ() override { return actual_q_; }

  std::size_t servoJCalls() const { return servoj_calls_; }
  std::size_t stopCalls() const { return stop_calls_; }

 private:
  std::size_t servoj_calls_{0};
  std::size_t stop_calls_{0};
  std::array<double, 6> actual_q_{};
  std::array<double, 6> actual_qd_{};
  std::array<double, 6> last_command_{};
};
// ################################
// C++: FakeCommander for adapter gtests end
// ################################

}  // namespace dobot_v4_bringup
