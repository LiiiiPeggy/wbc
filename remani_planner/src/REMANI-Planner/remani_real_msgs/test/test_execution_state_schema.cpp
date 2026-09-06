#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace {

TEST(ExecutionStateSchema, FreezesEveryFieldNameTypeAndOrder) {
  std::ifstream schema(EXECUTION_STATE_MSG_PATH);
  ASSERT_TRUE(schema.good()) << EXECUTION_STATE_MSG_PATH;
  std::ostringstream actual;
  actual << schema.rdbuf();

  const std::string expected = R"SCHEMA(std_msgs/Header header

uint8 MODE_SIM=0
uint8 MODE_REAL=1
uint8 OWNER_INTERNAL=0
uint8 OWNER_EXTERNAL=1
uint8 ENV_SIMULATED=0
uint8 ENV_STATIC_EMPTY=1
uint8 mode
uint8 execution_owner
bool dry_run
uint8 environment_mode

uint8 PLANNER_IDLE=0
uint8 PLANNER_PLANNING=1
uint8 PLANNER_HANDOFF=2
uint8 planner_state

uint8 TRANSACTION_IDLE=0
uint8 TRANSACTION_ASSEMBLING=1
uint8 TRANSACTION_COMPLETE=2
uint8 TRANSACTION_INVALID=3
uint8 transaction_state
uint64 candidate_id
time raw_transaction_stamp
bool candidate_complete
bool candidate_valid
float64 candidate_duration
float64 candidate_max_base_linear_speed
float64 candidate_max_base_angular_speed
float64 candidate_max_joint_speed

uint8 EXECUTOR_NONE=0
uint8 EXECUTOR_PLANNED=1
uint8 EXECUTOR_EXECUTING=2
uint8 EXECUTOR_PAUSED=3
uint8 EXECUTOR_SUCCEEDED=4
uint8 EXECUTOR_ERROR=5
uint8 executor_state

bool odom_ready
bool cr10_joint_ready
bool cr10_velocity_valid
bool tf_ready
bool robot_status_ready
bool robot_connected
bool robot_enabled
int8 robot_error_status
uint16 robot_mode
bool action_server_ready
uint8 cr10_action_state
bool grid_map_ready
bool ranger_watchdog_ready
bool ranger_watchdog_timed_out

float64 ranger_feedback_age
float64 cr10_joint_feedback_age
float64 tf_age
float64 robot_status_age
float64 base_tracking_error
float64 joint_tracking_error
float64 candidate_start_base_error
float64 candidate_start_joint_error

float64 execution_progress
float64 pause_param_time
float64 requested_t0
float64 ranger_trajectory_start_time
bool ranger_first_motion_available
float64 ranger_first_motion_time
float64 cr10_first_motion_time
bool start_skew_available
float64 start_skew

float64 final_base_error
float64 final_base_position_error
float64 final_base_yaw_error
float64 final_joint_error
float64 final_ee_pos_error
float64 final_ee_rot_error

string last_error_code
string last_error
)SCHEMA";

  EXPECT_EQ(expected, actual.str());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
