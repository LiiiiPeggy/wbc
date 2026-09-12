#include <gtest/gtest.h>

#include <remani_real_rviz/panel_view_model.hpp>

namespace remani_real_rviz {
namespace {

// ################################
// C++: PanelViewModel tests begin
// ################################
remani_real_msgs::ExecutionState readyBase() {
  remani_real_msgs::ExecutionState msg;
  msg.mode = remani_real_msgs::ExecutionState::MODE_REAL;
  msg.execution_owner = remani_real_msgs::ExecutionState::OWNER_EXTERNAL;
  msg.dry_run = true;
  msg.environment_mode = remani_real_msgs::ExecutionState::ENV_STATIC_EMPTY;
  msg.planner_state = remani_real_msgs::ExecutionState::PLANNER_IDLE;
  msg.executor_state = remani_real_msgs::ExecutionState::EXECUTOR_NONE;
  msg.odom_ready = true;
  msg.cr10_joint_ready = true;
  msg.cr10_velocity_valid = true;
  msg.tf_ready = true;
  msg.robot_status_ready = true;
  msg.action_server_ready = true;
  msg.grid_map_ready = true;
  msg.ranger_watchdog_ready = true;
  return msg;
}

TEST(PanelViewModel, PlannedEnablesPlanExecuteAbortOnly) {
  remani_real_msgs::ExecutionState msg = readyBase();
  msg.executor_state = remani_real_msgs::ExecutionState::EXECUTOR_PLANNED;
  msg.candidate_id = 19;
  msg.candidate_complete = true;
  msg.candidate_valid = true;
  const auto view = PanelViewModel::from(msg);
  EXPECT_TRUE(view.plan_enabled);
  EXPECT_TRUE(view.execute_enabled);
  EXPECT_FALSE(view.pause_enabled);
  EXPECT_FALSE(view.resume_enabled);
  EXPECT_TRUE(view.abort_enabled);
  EXPECT_EQ(19u, view.execute_candidate_id);
}

TEST(PanelViewModel, StaticEmptyAlwaysShowsWarning) {
  remani_real_msgs::ExecutionState msg;
  msg.environment_mode = remani_real_msgs::ExecutionState::ENV_STATIC_EMPTY;
  EXPECT_EQ("ENVIRONMENT: STATIC EMPTY / NO ONLINE OBSTACLE SENSING",
            PanelViewModel::from(msg).environment_warning);
}

TEST(PanelViewModel, NotReadyDisablesAllButtons) {
  remani_real_msgs::ExecutionState msg = readyBase();
  msg.odom_ready = false;
  msg.executor_state = remani_real_msgs::ExecutionState::EXECUTOR_PLANNED;
  msg.candidate_complete = true;
  msg.candidate_valid = true;
  const auto view = PanelViewModel::from(msg);
  EXPECT_FALSE(view.plan_enabled);
  EXPECT_FALSE(view.execute_enabled);
  EXPECT_FALSE(view.pause_enabled);
  EXPECT_FALSE(view.resume_enabled);
  EXPECT_FALSE(view.abort_enabled);
}

TEST(PanelViewModel, ReadyEnablesPlanOnly) {
  const auto view = PanelViewModel::from(readyBase());
  EXPECT_TRUE(view.plan_enabled);
  EXPECT_FALSE(view.execute_enabled);
  EXPECT_FALSE(view.abort_enabled);
}

TEST(PanelViewModel, PlanningEnablesAbortOnly) {
  remani_real_msgs::ExecutionState msg = readyBase();
  msg.planner_state = remani_real_msgs::ExecutionState::PLANNER_PLANNING;
  const auto view = PanelViewModel::from(msg);
  EXPECT_FALSE(view.plan_enabled);
  EXPECT_TRUE(view.abort_enabled);
}

TEST(PanelViewModel, ExecutingEnablesPauseAbort) {
  remani_real_msgs::ExecutionState msg = readyBase();
  msg.executor_state = remani_real_msgs::ExecutionState::EXECUTOR_EXECUTING;
  const auto view = PanelViewModel::from(msg);
  EXPECT_TRUE(view.pause_enabled);
  EXPECT_TRUE(view.abort_enabled);
  EXPECT_FALSE(view.plan_enabled);
  EXPECT_FALSE(view.execute_enabled);
}

TEST(PanelViewModel, CopiesFeedbackFieldsWithoutInference) {
  remani_real_msgs::ExecutionState msg = readyBase();
  msg.execution_progress = 0.42;
  msg.pause_param_time = 1.5;
  msg.start_skew_available = true;
  msg.start_skew = 0.03;
  msg.last_error_code = "X";
  msg.last_error = "detail";
  msg.final_ee_pos_error = 0.01;
  const auto view = PanelViewModel::from(msg);
  EXPECT_DOUBLE_EQ(0.42, view.execution_progress);
  EXPECT_DOUBLE_EQ(1.5, view.pause_param_time);
  EXPECT_TRUE(view.start_skew_available);
  EXPECT_DOUBLE_EQ(0.03, view.start_skew);
  EXPECT_EQ("X", view.last_error_code);
  EXPECT_EQ("detail", view.last_error);
  EXPECT_DOUBLE_EQ(0.01, view.final_ee_pos_error);
}
// ################################
// C++: PanelViewModel tests end
// ################################

}  // namespace
}  // namespace remani_real_rviz

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
