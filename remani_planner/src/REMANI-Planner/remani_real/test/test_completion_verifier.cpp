#include <gtest/gtest.h>

#include <remani_real/completion_verifier.hpp>

using remani_real::ActualStateSnapshot;
using remani_real::CompletionInput;
using remani_real::CompletionThresholds;
using remani_real::CompletionVerifier;
using remani_real::EeKinematics;

namespace {

class FakeEeKinematics : public EeKinematics {
 public:
  Eigen::Matrix4d pose(const Eigen::Vector3d& car,
                       const Eigen::Matrix<double, 6, 1>& q) const override {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(car.z() + q(5), Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    transform.block<3, 1>(0, 3) << car.x() + q.sum(), car.y(), 1.0;
    return transform;
  }
};

ActualStateSnapshot actualAtOrigin() {
  ActualStateSnapshot actual;
  actual.base_xy.setZero();
  actual.base_yaw = 0.0;
  actual.q << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60;
  actual.qd.setZero();
  actual.odom_valid = actual.joints_valid = actual.velocity_valid = true;
  actual.tf_valid = actual.robot_connected = actual.robot_enabled = true;
  return actual;
}

CompletionInput exactCompletionInput(const EeKinematics& kinematics) {
  CompletionInput input;
  input.expected_base_xy.setZero();
  input.expected_base_yaw = 0.0;
  input.expected_q << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60;
  input.expected_ee =
      kinematics.pose(Eigen::Vector3d::Zero(), input.expected_q);
  input.actual = actualAtOrigin();
  input.arm_action_succeeded = true;
  input.feedback_fresh = true;
  input.robot_status_healthy = true;
  return input;
}

}  // namespace

// ################################
TEST(CompletionVerifier, ActionSuccessAloneCannotSucceed) {
  FakeEeKinematics kinematics;
  CompletionVerifier verifier(CompletionThresholds(), &kinematics);
  CompletionInput input = exactCompletionInput(kinematics);
  input.arm_action_succeeded = true;
  input.actual.base_xy.x() += 0.051;
  const auto result = verifier.verify(input);
  EXPECT_FALSE(result.succeeded);
  EXPECT_EQ("TERMINAL_BASE_POSITION", result.error_code);
  EXPECT_NEAR(0.051, result.final_base_position_error, 1e-9);
}

TEST(CompletionVerifier, RequiresActualEeFkTolerance) {
  FakeEeKinematics kinematics;
  CompletionVerifier verifier(CompletionThresholds(), &kinematics);
  CompletionInput input = exactCompletionInput(kinematics);
  input.actual.q(5) += 0.03;
  const auto result = verifier.verify(input);
  EXPECT_FALSE(result.succeeded);
  EXPECT_GT(result.final_ee_rot_error, 0.0);
}

TEST(CompletionVerifier, ExactMatchSucceeds) {
  FakeEeKinematics kinematics;
  CompletionVerifier verifier(CompletionThresholds(), &kinematics);
  EXPECT_TRUE(verifier.verify(exactCompletionInput(kinematics)).succeeded);
}
// ################################

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
