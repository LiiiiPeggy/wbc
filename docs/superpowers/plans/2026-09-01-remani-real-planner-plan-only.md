# REMANI Real Planner PLAN-ONLY Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the shared real-deployment interfaces, strict `mode=real` planner PLAN-ONLY behavior, the formal START/ADD/FINAL candidate transaction, and a queryable static-empty GridMap without changing default simulation execution.

**Architecture:** `plan_manage` receives an explicit execution policy: internal ownership preserves the current FSM, while external ownership publishes one complete transaction and returns to planner idle. `plan_env::GridMap` gains a deterministic `static_empty` initialization path that keeps ESDF and hard map bounds but creates no depth/cloud subscriptions. A small `remani_real_msgs` package freezes the cross-plan ROS contract before any hardware executor is written.

**Tech Stack:** ROS1 Noetic, catkin, C++14, Eigen, gtest/rostest, existing `plan_manage`, `plan_env`, `quadrotor_msgs`, and `mm_config` packages.

**Spec:** `docs/superpowers/specs/2026-09-01-remani-real-robot-deployment-design.md`

## Global Constraints

- `mode:=sim` is the default and must keep current `GEN_NEW_TRAJ -> EXEC_TRAJ -> mm_controller` behavior.
- `mode:=real` requires `execution_owner:=external`; invalid mode/owner combinations fail at startup.
- Real planner is PLAN-ONLY: no internal `EXEC_TRAJ`, wall-clock progress, `planning/finish`, periodic replan, execution EE completion, or execution safety timer after handoff.
- Real candidate protocol is `ACTION_WARN_START`, contiguous `ACTION_ADD trajectory_id=1..N`, then `ACTION_WARN_FINAL`; sim remains ADD-only.
- `trajectory_id` is a segment sequence number, never a Plan/candidate version.
- Real V1 uses `environment_mode:=static_empty`, `global_plan:=true`, a 16 m × 12 m × 3 m map at 0.05 m resolution, and no depth/cloud timeout.
- Do not change Hybrid A*, RRT, optimizer cost/time scaling, MoveIt, Cartesian servo, impedance, MPC, or the existing default sim launch semantics.
- New/changed C++ blocks follow the repository convention: one comment banner before the block, without a matching end banner.
- Each commit explicitly stages every changed/new file; do not use `git commit -am` when a task creates files.

## Plan-set boundary

This is phase 1 of five and must land before hardware Executor work:

1. This plan: shared messages, planner PLAN-ONLY, transaction protocol, static-empty environment.
2. `2026-09-01-remani-real-control-plane.md`: State Bridge, Gate, preview, panel, dry-run state machine.
3. `2026-09-01-remani-real-ranger-execution.md`: Ranger isolation, watchdog, low-speed base execution.
4. `2026-09-01-remani-real-cr10-action.md`: non-blocking/preemptible CR10 Action and arm-only execution.
5. `2026-09-01-remani-real-integration.md`: shared T0, Pause/Resume/Abort, final FK, unified launch.

---

## File Structure

| Path | Responsibility |
|---|---|
| `remani_planner/src/REMANI-Planner/remani_real_msgs/` | Shared messages/services only; no runtime node |
| `remani_real_msgs/msg/PlannerStatus.msg` | Planner-side idle/planning/handoff/failure truth |
| `remani_real_msgs/msg/ExecutionState.msg` | Unified deployment state consumed by the future Panel |
| `remani_real_msgs/msg/ExecutionResult.msg` | External execution terminal result and actual final errors |
| `remani_real_msgs/srv/ExecuteCandidate.srv` | Execute request bound to a Gate-owned `candidate_id` |
| `plan_manage/include/plan_manage/execution_policy.hpp` | Strict mode/owner parsing and startup validation |
| `plan_manage/include/plan_manage/candidate_transaction_builder.hpp` | Pure START/ADD/FINAL message construction |
| `plan_manage/src/candidate_transaction_builder.cpp` | Transaction serialization from `SingulTrajData` |
| `plan_manage/src/remani_replan_fsm.cpp` | Ownership branch, planner status, handoff/idle behavior |
| `plan_env/include/plan_env/grid_map.h` | Environment mode/readiness API |
| `plan_env/src/grid_map.cpp` | Static-empty initialization and online-sensing separation |
| `plan_manage/launch/remani_ranger_cr10_real_planner.launch` | Planner-only real-mode launch used before Executor exists |
| `plan_manage/config/remani_ranger_cr10_real.yaml` | Locked real planner/environment parameters |
| `plan_manage/test/` and `plan_env/test/` | gtest and rostest gates |

---

### Task 1: Freeze the shared ROS interface package

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real_msgs/CMakeLists.txt`
- Create: `remani_planner/src/REMANI-Planner/remani_real_msgs/package.xml`
- Create: `remani_planner/src/REMANI-Planner/remani_real_msgs/msg/PlannerStatus.msg`
- Create: `remani_planner/src/REMANI-Planner/remani_real_msgs/msg/ExecutionState.msg`
- Create: `remani_planner/src/REMANI-Planner/remani_real_msgs/msg/ExecutionResult.msg`
- Create: `remani_planner/src/REMANI-Planner/remani_real_msgs/srv/ExecuteCandidate.srv`

**Interfaces:**
- Consumes: ROS `std_msgs/Header`.
- Produces: `remani_real_msgs/PlannerStatus`, `ExecutionState`, `ExecutionResult`, and `ExecuteCandidate` used unchanged by all later plans.

- [ ] **Step 1: Create the exact message and service definitions**

`PlannerStatus.msg`:

```text
std_msgs/Header header
uint8 IDLE=0
uint8 PLANNING=1
uint8 HANDED_OFF=2
uint8 FAILED=3
uint8 state
bool ready
bool busy
string last_error_code
string last_error
```

`ExecutionResult.msg`:

```text
std_msgs/Header header
uint8 SUCCEEDED=0
uint8 ABORTED=1
uint8 ERROR=2
uint8 TERMINAL_TOLERANCE_FAILURE=3
uint64 candidate_id
uint8 result
float64 final_base_position_error
float64 final_base_yaw_error
float64 final_joint_error
float64 final_ee_pos_error
float64 final_ee_rot_error
string message
```

`ExecuteCandidate.srv`:

```text
uint64 candidate_id
---
bool accepted
string message
```

`ExecutionState.msg` must be created with these exact names so later phases do not invent parallel status topics:

```text
std_msgs/Header header

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

bool planner_ready
bool planner_busy

uint8 TRANSACTION_IDLE=0
uint8 TRANSACTION_ASSEMBLING=1
uint8 TRANSACTION_COMPLETE=2
uint8 TRANSACTION_INVALID=3
uint8 transaction_state
uint64 candidate_id
bool candidate_complete
bool candidate_valid
float64 candidate_duration
float64 candidate_max_base_linear_speed
float64 candidate_max_base_angular_speed
float64 candidate_max_joint_speed

uint8 NOT_READY=0
uint8 READY=1
uint8 PLANNING=2
uint8 PLANNED=3
uint8 EXECUTING=4
uint8 PAUSED=5
uint8 SUCCEEDED=6
uint8 ERROR=7
uint8 executor_state

bool odom_ready
bool cr10_joint_ready
bool cr10_velocity_valid
bool tf_ready
bool robot_status_ready
bool action_server_ready
bool grid_map_ready
bool ranger_watchdog_ready
bool ranger_watchdog_timed_out

float64 ranger_feedback_age
float64 cr10_joint_feedback_age
float64 tf_age
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
float64 final_joint_error
float64 final_ee_pos_error
float64 final_ee_rot_error

string last_error_code
string last_error
```

- [ ] **Step 2: Wire message generation**

Use a minimal `CMakeLists.txt` contract:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(remani_real_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES PlannerStatus.msg ExecutionState.msg ExecutionResult.msg)
add_service_files(FILES ExecuteCandidate.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
```

`package.xml` must declare `catkin`, `message_generation`, `message_runtime`, and `std_msgs` for build/export/exec as appropriate.

- [ ] **Step 3: Build just the interface package**

Run:

```bash
catkin_make -C remani_planner --pkg remani_real_msgs
```

Expected: message/service generation succeeds and `rossrv show remani_real_msgs/ExecuteCandidate` contains `uint64 candidate_id`.

- [ ] **Step 4: Commit the interface freeze**

```bash
git add remani_planner/src/REMANI-Planner/remani_real_msgs/CMakeLists.txt \
        remani_planner/src/REMANI-Planner/remani_real_msgs/package.xml \
        remani_planner/src/REMANI-Planner/remani_real_msgs/msg/PlannerStatus.msg \
        remani_planner/src/REMANI-Planner/remani_real_msgs/msg/ExecutionState.msg \
        remani_planner/src/REMANI-Planner/remani_real_msgs/msg/ExecutionResult.msg \
        remani_planner/src/REMANI-Planner/remani_real_msgs/srv/ExecuteCandidate.srv
git commit -m "feat: define REMANI real deployment interfaces"
```

---

### Task 2: Add a strict execution policy with startup validation

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/include/plan_manage/execution_policy.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/test_execution_policy.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/CMakeLists.txt`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/package.xml`

**Interfaces:**
- Consumes: private ROS params `mode` and `execution_owner`.
- Produces:
  - `enum class RuntimeMode { Sim, Real };`
  - `enum class ExecutionOwner { Internal, External };`
  - `ExecutionPolicy::load(const ros::NodeHandle&)` throwing `std::invalid_argument` on invalid combinations.
  - `ownsInternalExecution()` and `isRealPlanOnly()`.

- [ ] **Step 1: Write the failing policy test**

```cpp
TEST(ExecutionPolicy, AcceptsOnlyLockedCombinations) {
  EXPECT_NO_THROW(ExecutionPolicy::fromStrings("sim", "internal"));
  EXPECT_NO_THROW(ExecutionPolicy::fromStrings("real", "external"));
  EXPECT_THROW(ExecutionPolicy::fromStrings("real", "internal"), std::invalid_argument);
  EXPECT_THROW(ExecutionPolicy::fromStrings("sim", "external"), std::invalid_argument);
  EXPECT_THROW(ExecutionPolicy::fromStrings("hardware", "external"), std::invalid_argument);
}

TEST(ExecutionPolicy, DefaultsRemainSimulationInternal) {
  const auto policy = ExecutionPolicy::fromStrings("sim", "internal");
  EXPECT_TRUE(policy.ownsInternalExecution());
  EXPECT_FALSE(policy.isRealPlanOnly());
}
```

- [ ] **Step 2: Run the test and confirm the missing-header failure**

```bash
catkin_make -C remani_planner --pkg remani_planner --make-args run_tests_remani_planner_gtest_test_execution_policy
```

Expected: FAIL because `plan_manage/execution_policy.hpp` does not exist.

- [ ] **Step 3: Implement the minimal immutable policy**

```cpp
class ExecutionPolicy {
public:
  static ExecutionPolicy fromStrings(const std::string& mode,
                                     const std::string& owner);
  static ExecutionPolicy load(const ros::NodeHandle& nh);
  bool ownsInternalExecution() const { return owner_ == ExecutionOwner::Internal; }
  bool isRealPlanOnly() const {
    return mode_ == RuntimeMode::Real && owner_ == ExecutionOwner::External;
  }
  RuntimeMode mode() const { return mode_; }
  ExecutionOwner owner() const { return owner_; }

private:
  ExecutionPolicy(RuntimeMode mode, ExecutionOwner owner)
      : mode_(mode), owner_(owner) {}
  RuntimeMode mode_;
  ExecutionOwner owner_;
};
```

`load()` reads defaults `mode="sim"`, `execution_owner="internal"`; `fromStrings()` accepts exactly the two locked pairs.

- [ ] **Step 4: Register and rerun the gtest**

Add under `if(CATKIN_ENABLE_TESTING)`:

```cmake
catkin_add_gtest(test_execution_policy test/test_execution_policy.cpp)
if(TARGET test_execution_policy)
  target_link_libraries(test_execution_policy ${catkin_LIBRARIES})
endif()
```

Run:

```bash
catkin_make -C remani_planner --pkg remani_planner --make-args run_tests_remani_planner_gtest_test_execution_policy
```

Expected: PASS, 2 tests.

- [ ] **Step 5: Commit the policy gate**

```bash
git add remani_planner/src/REMANI-Planner/remani_planner/plan_manage/include/plan_manage/execution_policy.hpp \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/test_execution_policy.cpp \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/CMakeLists.txt \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/package.xml
git commit -m "feat: lock planner execution ownership by mode"
```

---

### Task 3: Build the formal candidate transaction serializer

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/include/plan_manage/candidate_transaction_builder.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/src/candidate_transaction_builder.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/test_candidate_transaction_builder.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/CMakeLists.txt`

**Interfaces:**
- Consumes: `const remani_planner::SingulTrajData&` and one transaction timestamp.
- Produces:
  - `buildExternal(const SingulTrajData&, const ros::Time&) -> std::vector<quadrotor_msgs::PolynomialTraj>`.
  - `buildInternalAdds(const SingulTrajData&, const ros::Time&)` preserving current ADD-only sim serialization.
  - `buildControl(uint32_t action, const ros::Time&)` with `trajectory_id=0`, `singul=0`, and empty `trajectory`.

- [ ] **Step 1: Write a failing serializer test using a real 8D degree-7 piece**

```cpp
static SingulTrajData oneSegmentTrajectory() {
  poly_traj::Piece<7>::CoefficientMat coeff;
  coeff.setZero();
  coeff.col(7) << 0.0, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  poly_traj::Trajectory<7> traj;
  traj.emplace_back(1.25, coeff, 1);
  SingulTrajData data;
  data.addSingulTraj(traj, 10.0);
  return data;
}

TEST(CandidateTransactionBuilder, ExternalWrapsContiguousAdds) {
  const auto msgs = CandidateTransactionBuilder::buildExternal(
      oneSegmentTrajectory(), ros::Time(10.0));
  ASSERT_EQ(3u, msgs.size());
  EXPECT_EQ(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START, msgs[0].action);
  EXPECT_EQ(0u, msgs[0].trajectory_id);
  EXPECT_EQ(quadrotor_msgs::PolynomialTraj::ACTION_ADD, msgs[1].action);
  EXPECT_EQ(1u, msgs[1].trajectory_id);
  ASSERT_EQ(1u, msgs[1].trajectory.size());
  EXPECT_EQ(8u, msgs[1].trajectory[0].num_dim);
  EXPECT_EQ(7u, msgs[1].trajectory[0].num_order);
  EXPECT_EQ(64u, msgs[1].trajectory[0].data.size());
  EXPECT_EQ(quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL, msgs[2].action);
  EXPECT_EQ(0u, msgs[2].trajectory_id);
}

TEST(CandidateTransactionBuilder, InternalRemainsAddOnly) {
  const auto msgs = CandidateTransactionBuilder::buildInternalAdds(
      oneSegmentTrajectory(), ros::Time(10.0));
  ASSERT_EQ(1u, msgs.size());
  EXPECT_EQ(quadrotor_msgs::PolynomialTraj::ACTION_ADD, msgs[0].action);
}
```

- [ ] **Step 2: Run and confirm the missing-builder failure**

```bash
catkin_make -C remani_planner --pkg remani_planner --make-args run_tests_remani_planner_gtest_test_candidate_transaction_builder
```

Expected: FAIL because the builder is undefined.

- [ ] **Step 3: Implement serialization once and expose the two protocols**

The ADD construction must preserve existing column-major coefficient layout:

```cpp
quadrotor_msgs::PolynomialMatrix piece_msg;
piece_msg.num_dim = piece.getDim();
piece_msg.num_order = piece.getDegree();
piece_msg.duration = piece.getDuration();
const auto coeff = piece.getCoeffMat();
piece_msg.data.assign(coeff.data(), coeff.data() + coeff.size());
```

`buildExternal()` pushes one START, all ADDs with `trajectory_id` taken from the segment sequence, then one FINAL. It rejects empty `singul_traj` by throwing `std::invalid_argument`; the FSM converts that internal serialization failure into planner failure instead of publishing a partial transaction.

- [ ] **Step 4: Link the builder library and rerun tests**

```cmake
add_library(candidate_transaction_builder src/candidate_transaction_builder.cpp)
target_link_libraries(candidate_transaction_builder ${catkin_LIBRARIES})
add_dependencies(candidate_transaction_builder ${catkin_EXPORTED_TARGETS})

target_link_libraries(remani_planner_node
  candidate_transaction_builder ${catkin_LIBRARIES})

catkin_add_gtest(test_candidate_transaction_builder test/test_candidate_transaction_builder.cpp)
target_link_libraries(test_candidate_transaction_builder
  candidate_transaction_builder ${catkin_LIBRARIES})
```

Run the named target again. Expected: PASS, including exact START/ADD/FINAL ordering and ADD-only sim behavior.

- [ ] **Step 5: Commit the serializer**

```bash
git add remani_planner/src/REMANI-Planner/remani_planner/plan_manage/include/plan_manage/candidate_transaction_builder.hpp \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/src/candidate_transaction_builder.cpp \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/test_candidate_transaction_builder.cpp \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/CMakeLists.txt
git commit -m "feat: serialize real candidate transactions"
```

---

### Task 4: Make the real planner strictly PLAN-ONLY

**Files:**
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/include/plan_manage/remani_replan_fsm.h:82-203`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/src/remani_replan_fsm.cpp:35-511,806-968`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/test_plan_only_policy.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/remani_plan_only.test`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/fake_plan_only_actual_state.py`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/test_remani_plan_only.py`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/CMakeLists.txt`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/package.xml`

**Interfaces:**
- Consumes: `ExecutionPolicy`, `/odom`, fixed-order six-axis `joint_state`, and `/ee_goal`.
- Produces: `/remani/candidate_trajectory`, `/remani/planner_status`, and no real execution lifecycle.
- Preserves: sim publication on `planning/trajectory` and current internal FSM behavior.

- [ ] **Step 1: Add a failing policy-level handoff test**

Introduce a pure decision used by the FSM:

```cpp
enum class PlanSuccessDisposition { EnterInternalExec, HandoffAndIdle };
PlanSuccessDisposition planSuccessDisposition(const ExecutionPolicy& policy);
```

Test:

```cpp
TEST(PlanOnlyPolicy, RealSuccessHandsOffInsteadOfExecuting) {
  EXPECT_EQ(PlanSuccessDisposition::HandoffAndIdle,
            planSuccessDisposition(
                ExecutionPolicy::fromStrings("real", "external")));
  EXPECT_EQ(PlanSuccessDisposition::EnterInternalExec,
            planSuccessDisposition(
                ExecutionPolicy::fromStrings("sim", "internal")));
}
```

The implementation is deliberately one branch and has no ROS/FSM side effect:

```cpp
PlanSuccessDisposition planSuccessDisposition(const ExecutionPolicy& policy) {
  return policy.ownsInternalExecution()
      ? PlanSuccessDisposition::EnterInternalExec
      : PlanSuccessDisposition::HandoffAndIdle;
}
```

Run the named gtest target and confirm it fails before implementation.

- [ ] **Step 2: Load ownership and publish planner truth**

Add members:

```cpp
ExecutionPolicy execution_policy_;
ros::Publisher planner_status_pub_;
ros::Publisher candidate_traj_pub_;
void publishPlannerStatus(uint8_t state,
                          const std::string& code = "",
                          const std::string& detail = "");
void finishExternalHandoff();
void publishExternalImpossible(const std::string& code,
                               const std::string& detail);
```

In `init()`, call `ExecutionPolicy::load(nh)` before timers/subscribers are created. Advertise the new real topics only when `isRealPlanOnly()`; keep the existing `planning/trajectory` publisher for sim.

- [ ] **Step 3: Route successful publication through exactly one protocol**

Replace direct publication in the successful `callReboundReplan()` tail with:

```cpp
ros::Time stamp;
stamp.fromSec(planner_manager_->traj_container_.singul_traj_data.start_time);
const auto& data = planner_manager_->traj_container_.singul_traj_data;
const auto messages = execution_policy_.ownsInternalExecution()
    ? CandidateTransactionBuilder::buildInternalAdds(data, stamp)
    : CandidateTransactionBuilder::buildExternal(data, stamp);
ros::Publisher& pub = execution_policy_.ownsInternalExecution()
    ? poly_traj_pub_ : candidate_traj_pub_;
for (const auto& message : messages) {
  pub.publish(message);
}
```

Do not publish START/FINAL in sim.

- [ ] **Step 4: Branch `GEN_NEW_TRAJ` success before `EXEC_TRAJ`**

The success branch must have this shape:

```cpp
if (success) {
  flag_escape_emergency_ = true;
  try_plan_after_emergency_ = false;
  if (execution_policy_.ownsInternalExecution()) {
    changeFSMExecState(EXEC_TRAJ, "FSM");
  } else {
    finishExternalHandoff();
    changeFSMExecState(WAIT_TARGET, "PLAN_ONLY");
  }
}
```

`finishExternalHandoff()` clears `have_target_`, `have_trigger_`, `have_local_traj_`, and active EE request metadata, publishes `PlannerStatus::HANDED_OFF`, then publishes `PlannerStatus::IDLE/ready=true` on the next FSM tick. It must not publish `planning/finish`.

- [ ] **Step 5: Disable all planner execution-time safety semantics for external ownership**

At the first line of `checkCollisionCallback()`:

```cpp
if (execution_policy_.isRealPlanOnly()) {
  return;
}
```

Also guard accidental transitions into `REPLAN_TRAJ`, `EXEC_TRAJ`, or `EMERGENCY_STOP` in real mode with a fatal log and return to `WAIT_TARGET`. Do not add `WAIT_EXTERNAL_EXECUTE`.

- [ ] **Step 6: Publish one explicit planning failure**

For terminal collision, IK/no-path exhaustion, optimizer failure exhaustion, and transaction serialization failure, publish one `ACTION_WARN_IMPOSSIBLE` control message on `/remani/candidate_trajectory` plus `PlannerStatus::FAILED`. Ordinary planning failure returns planner idle; malformed internal serialization reports code `TRANSACTION_SERIALIZATION_ERROR`.

- [ ] **Step 7: Add a rostest proving no real internal execution**

`remani_plan_only.test` launches the real planner with `target_type=2`, the existing Ranger+CR10 `exp_ranger_cr10_param.yaml`, and `fake_plan_only_actual_state.py`. The fake publishes 50 Hz finite `/odom` at base `(0,0,0)` and six fixed joints `[0,-40,130,0,30,0]` degrees converted to radians, matching existing waypoint 0. Existing waypoint 1 is the deterministic static-empty goal. `test_remani_plan_only.py` subscribes to:

```text
/remani/candidate_trajectory
/planning/finish
/remani/planner_status
```

Assertions after FINAL and after waiting longer than candidate duration:

```python
self.assertEqual(
    [PolynomialTraj.ACTION_WARN_START,
     PolynomialTraj.ACTION_ADD,
     PolynomialTraj.ACTION_WARN_FINAL],
    actions)
self.assertEqual(0, finish_publish_count)
self.assertIn(PlannerStatus.HANDED_OFF, planner_states)
self.assertEqual(PlannerStatus.IDLE, planner_states[-1])
self.assertTrue(planner_ready[-1])
```

After FINAL, keep publishing unchanged actual feedback and wait `candidate_duration+1.0 s`; assert no new transaction, no `planning/finish`, and planner status remains IDLE. The pure `PlanSuccessDisposition` unit test is the direct proof that this success cannot select `EXEC_TRAJ`; do not add a test-only production debug topic or parse console logs.

- [ ] **Step 8: Run the planner unit and rostest gates**

```bash
catkin_make -C remani_planner --pkg remani_planner
catkin_make -C remani_planner --pkg remani_planner --make-args run_tests_remani_planner
catkin_test_results remani_planner/build/remani_planner/test_results
```

Expected: all plan_manage tests pass; real transaction finishes without `planning/finish` or internal execution.

- [ ] **Step 9: Commit PLAN-ONLY ownership**

```bash
git add remani_planner/src/REMANI-Planner/remani_planner/plan_manage/include/plan_manage/remani_replan_fsm.h \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/src/remani_replan_fsm.cpp \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/test_plan_only_policy.cpp \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/remani_plan_only.test \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/fake_plan_only_actual_state.py \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/test_remani_plan_only.py \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/CMakeLists.txt \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/package.xml
git commit -m "feat: make REMANI real mode plan only"
```

---

### Task 5: Add deterministic static-empty GridMap mode

**Files:**
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_env/include/plan_env/grid_map.h:55-258`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_env/src/grid_map.cpp:6-217`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_env/test/test_static_empty_grid_map.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_env/CMakeLists.txt`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_env/package.xml`

**Interfaces:**
- Consumes: `environment_mode` string plus existing `grid_map/*` size/resolution parameters.
- Produces: `GridMap::EnvironmentMode`, `isReady()`, `usesOnlineSensing()`, and existing ESDF query APIs over known-free in-bounds voxels.

- [ ] **Step 1: Write the failing static-empty test**

```cpp
TEST(StaticEmptyGridMap, IsKnownFreeQueryableAndBounded) {
  ros::NodeHandle nh("~static_empty");
  nh.setParam("environment_mode", "static_empty");
  nh.setParam("grid_map/resolution", 0.05);
  nh.setParam("grid_map/map_size_x", 16.0);
  nh.setParam("grid_map/map_size_y", 12.0);
  nh.setParam("grid_map/map_size_z", 3.0);
  nh.setParam("grid_map/ground_height", 0.0);

  GridMap map;
  map.initMap(nh);
  Eigen::Vector3i center_index;
  map.posToIndex(Eigen::Vector3d(0.0, 0.0, 1.0), center_index);

  EXPECT_TRUE(map.isReady());
  EXPECT_FALSE(map.usesOnlineSensing());
  EXPECT_TRUE(map.isKnownFree(center_index));
  EXPECT_TRUE(std::isfinite(map.getDistance(Eigen::Vector3d(0.0, 0.0, 1.0))));
  EXPECT_FALSE(map.isInMap(Eigen::Vector3d(8.01, 0.0, 1.0)));
  EXPECT_FALSE(map.getOdomDepthTimeout());
}
```

- [ ] **Step 2: Run and confirm the missing-mode API failure**

```bash
catkin_make -C remani_planner --pkg plan_env --make-args run_tests_plan_env_gtest_test_static_empty_grid_map
```

Expected: FAIL because `isReady()` and `usesOnlineSensing()` do not exist.

- [ ] **Step 3: Split common buffer allocation from sensing setup**

Add:

```cpp
enum class EnvironmentMode { Simulated, StaticEmpty };
EnvironmentMode environment_mode_{EnvironmentMode::Simulated};
bool map_ready_{false};
void initializeStaticEmpty();
void setupOnlineSensing();
bool isReady() const { return map_ready_; }
bool usesOnlineSensing() const {
  return environment_mode_ != EnvironmentMode::StaticEmpty;
}
```

Keep parameter loading and buffer allocation common. In `static_empty`:

```cpp
std::fill(md_.occupancy_buffer_.begin(),
          md_.occupancy_buffer_.end(),
          mp_.clamp_min_log_);
std::fill(md_.occupancy_buffer_inflate_.begin(),
          md_.occupancy_buffer_inflate_.end(), 0);
md_.local_bound_min_.setZero();
md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
md_.flag_depth_odom_timeout_ = false;
md_.flag_use_depth_fusion = false;
updateESDF3d();
map_ready_ = true;
```

Do not construct depth/cloud subscribers, synchronization objects, occupancy timers, or depth-timeout behavior in this branch. Keep ESDF visualization publishing optional, but not required for readiness.

- [ ] **Step 4: Make out-of-bounds queries hard-invalid at validation call sites**

Do not clamp an out-of-map point into an in-map voxel. Preserve `isInMap()` checks and require Gate/planner collision call sites to reject out-of-bounds base/arm samples before calling interpolated ESDF APIs.

- [ ] **Step 5: Run map tests and a no-sensor timeout check**

```bash
catkin_make -C remani_planner --pkg plan_env
catkin_make -C remani_planner --pkg plan_env --make-args run_tests_plan_env
catkin_test_results remani_planner/build/plan_env/test_results
```

Expected: static-empty remains ready for at least 15 seconds with no depth/cloud publishers, and every in-bounds test voxel is known-free.

- [ ] **Step 6: Commit static-empty mode**

```bash
git add remani_planner/src/REMANI-Planner/remani_planner/plan_env/include/plan_env/grid_map.h \
        remani_planner/src/REMANI-Planner/remani_planner/plan_env/src/grid_map.cpp \
        remani_planner/src/REMANI-Planner/remani_planner/plan_env/test/test_static_empty_grid_map.cpp \
        remani_planner/src/REMANI-Planner/remani_planner/plan_env/CMakeLists.txt \
        remani_planner/src/REMANI-Planner/remani_planner/plan_env/package.xml
git commit -m "feat: add static empty REMANI environment"
```

---

### Task 6: Add the planner-only real launch and lock sim regression

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/config/remani_ranger_cr10_real.yaml`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/launch/remani_ranger_cr10_real_planner.launch`
- Create: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/remani_sim_owner.test`
- Modify: `remani_planner/src/REMANI-Planner/remani_planner/plan_manage/CMakeLists.txt`

**Interfaces:**
- Consumes: `/odom`, `/remani/cr10_joint_states`, `/ee_goal`; later plans provide the real State Bridge.
- Produces: a hardware-free planner launch that is safe to use while Gate/Executor do not yet exist.

- [ ] **Step 1: Create the locked real planner YAML**

The file must include these exact values, with existing Ranger+CR10 planner parameters loaded alongside it:

```yaml
mode: real
execution_owner: external
environment_mode: static_empty
fsm:
  global_plan: true
grid_map:
  resolution: 0.05
  map_size_x: 16.0
  map_size_y: 12.0
  map_size_z: 3.0
  frame_id: world
```

- [ ] **Step 2: Create the planner-only real launch**

The planner node remaps must be explicit:

```xml
<remap from="~odom_world" to="/odom"/>
<remap from="~joint_state" to="/remani/cr10_joint_states"/>
<remap from="~grid_map/odom" to="/odom"/>
```

Do not launch `mm_controller`, Ranger, CR10, Real Executor, depth, cloud, or MoveIt in this phase launch.

- [ ] **Step 3: Add the sim ownership regression test**

Launch existing `remani_ranger_cr10_sim.launch` without new mode arguments and assert:

```python
self.assertEqual("sim", rospy.get_param("/remani_planner_node/mode", "sim"))
self.assertEqual("internal", rospy.get_param(
    "/remani_planner_node/execution_owner", "internal"))
self.assertGreater(received_add_count, 0)
self.assertEqual(0, received_start_count)
self.assertEqual(0, received_final_count)
```

Also assert the `mm_controller_node` exists and receives `planning/trajectory`.

- [ ] **Step 4: Run phase-1 verification**

```bash
catkin_make -C remani_planner --pkg remani_real_msgs plan_env remani_planner
catkin_make -C remani_planner --pkg plan_env remani_planner --make-args run_tests
catkin_test_results remani_planner/build
roslaunch --nodes remani_planner remani_ranger_cr10_real_planner.launch
roslaunch --nodes remani_planner remani_ranger_cr10_sim.launch
```

Expected real node list: planner and its visualization/marker dependencies only, with no hardware driver or `mm_controller`. Expected sim node list: unchanged internal controller path.

- [ ] **Step 5: Commit the phase launch and regression gate**

```bash
git add remani_planner/src/REMANI-Planner/remani_planner/plan_manage/config/remani_ranger_cr10_real.yaml \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/launch/remani_ranger_cr10_real_planner.launch \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/test/remani_sim_owner.test \
        remani_planner/src/REMANI-Planner/remani_planner/plan_manage/CMakeLists.txt
git commit -m "test: gate real plan-only and sim ownership"
```

## Phase Exit Gate

Do not start either hardware execution plan until all are true:

```text
real planning success -> START / ADD 1..N / FINAL
real planning success !-> EXEC_TRAJ
real candidate duration expiry !-> planning/finish
real checkCollisionCallback -> immediate no-op
sim planning success -> existing EXEC_TRAJ + mm_controller
static_empty -> known-free ESDF + hard boundary + no sensor timeout
```
