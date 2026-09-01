# REMANI CR10 Non-blocking Action Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the blocking CR10 FollowJointTrajectory implementation with strict goal validation, one-sample-per-timer non-blocking execution, prompt cancel/Stop behavior, measured completion, and a tested REMANI arm trajectory adapter.

**Architecture:** `dobot_v4_bringup` extracts validation and time sampling into hardware-free C++ classes. The existing `CRRobot` ActionServer becomes a thin adapter: goal callback validates/caches/accepts and returns; timer callback reads one steady elapsed time, computes one sample, sends one ServoJ, publishes feedback, and returns. `remani_real` adds an arm trajectory builder and a `Cr10HardwareChannel`; neither is instantiated in dry-run.

**Tech Stack:** ROS1 Noetic, catkin, C++14, actionlib `ActionServer<control_msgs::FollowJointTrajectoryAction>`, `trajectory_msgs`, Dobot V4 commander, gtest/rostest.

**Spec:** `docs/superpowers/specs/2026-09-01-remani-real-robot-deployment-design.md`

## Global Constraints

- Modify `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup`, because the verified CR10 hardware path is V4; do not implement against the unused V3 `dobot_bringup` Action.
- Goal must have at least two points and exactly the unique names `joint1..joint6`; input order may differ but internal order is fixed.
- Every point has six finite positions and six finite velocities. Missing velocities are rejected in V1.
- `time_from_start` is finite, non-negative, and strictly increasing.
- Goal callback validates/caches/accepts/returns. Timer performs one sample/send/feedback/return with no full-trajectory `for`, segment `while`, or `ros::Rate::sleep()`.
- Cancel stops future sampling, calls validated `Stop()`, confirms actual stop, and returns CANCELED/PREEMPTED—not SUCCEEDED.
- Action completion requires elapsed duration, joint error, joint velocity, and three consecutive valid samples.
- Default `servoj_period=0.10 s`, `cr10_goal_joint_tol=0.02 rad`, `cr10_stop_velocity_tol=0.01 rad/s`, and settle timeout 2.0 s.
- `dry_run=true` sends no goal and invokes no CR10 write service.
- No combined Ranger+CR10 real movement in this phase.

---

## File Structure

| Path | Responsibility |
|---|---|
| `dobot_v4_bringup/include/dobot_v4_bringup/trajectory_goal_validator.hpp` | Strict JointTrajectory schema/name/value validation |
| `dobot_v4_bringup/include/dobot_v4_bringup/trajectory_runner.hpp` | Pure steady-time hold/run/settle/cancel state machine |
| `dobot_v4_bringup/src/cr5_v4_robot.cpp` | ROS Action/commander adapter only |
| `dobot_v4_bringup/test/` | Validator, interpolation, one-tick, cancel, completion tests |
| `remani_real/include/remani_real/arm_trajectory_builder.hpp` | Candidate arm samples → complete position+velocity JointTrajectory |
| `remani_real/include/remani_real/cr10_hardware_channel.hpp` | Non-dry action client/cancel/Stop adapter |
| `remani_real/launch/cr10_only_low_speed.launch` | Explicit single-device validation launch |

---

### Task 1: Reject malformed CR10 goals before accepting motion

**Files:**
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/include/dobot_v4_bringup/trajectory_goal_validator.hpp`
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/trajectory_goal_validator.cpp`
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/trajectory_test_fixtures.hpp`
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/test_trajectory_goal_validator.cpp`
- Modify: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/CMakeLists.txt`
- Modify: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/package.xml`

**Interfaces:**
- Consumes: `trajectory_msgs::JointTrajectory`.
- Produces a canonical fixed-order trajectory or a stable reject code; no Action or hardware calls.

- [ ] **Step 1: Write failing happy-path and rejection tests**

```cpp
static std::vector<std::string> canonicalNames() {
  return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

static trajectory_msgs::JointTrajectory validTrajectory(
    const std::vector<std::string>& names) {
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = names;
  trajectory_msgs::JointTrajectoryPoint p0;
  p0.positions = {0.30, 0.10, 0.60, 0.20, 0.50, 0.40};
  p0.velocities.assign(6, 0.0);
  p0.time_from_start = ros::Duration(0.0);
  trajectory_msgs::JointTrajectoryPoint p1 = p0;
  for (double& q : p1.positions) q += 0.01;
  p1.time_from_start = ros::Duration(1.0);
  trajectory.points = {p0, p1};
  return trajectory;
}

TEST(TrajectoryGoalValidator, ReordersNamesAndAllPointArrays) {
  auto input = validTrajectory(
      {"joint3", "joint1", "joint6", "joint2", "joint5", "joint4"});
  const auto result = TrajectoryGoalValidator::validate(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ((std::array<std::string, 6>{
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}),
      result.trajectory.joint_names);
  EXPECT_DOUBLE_EQ(input.points[0].positions[1],
                   result.trajectory.points[0].positions[0]);
}

TEST(TrajectoryGoalValidator, RejectsMissingVelocity) {
  auto input = validTrajectory(canonicalNames());
  input.points[1].velocities.clear();
  const auto result = TrajectoryGoalValidator::validate(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ("VELOCITY_SIZE", result.error_code);
}

TEST(TrajectoryGoalValidator, RejectsNonIncreasingTime) {
  auto input = validTrajectory(canonicalNames());
  input.points[1].time_from_start = input.points[0].time_from_start;
  EXPECT_EQ("TIME_NOT_STRICT", TrajectoryGoalValidator::validate(input).error_code);
}
```

- [ ] **Step 2: Run and confirm the missing-validator failure**

```bash
catkin_make -C agx --pkg dobot_v4_bringup --make-args run_tests_dobot_v4_bringup_gtest_test_trajectory_goal_validator
```

- [ ] **Step 3: Implement exact canonical types and validation**

```cpp
struct CanonicalTrajectoryPoint {
  std::array<double, 6> positions;
  std::array<double, 6> velocities;
  std::array<double, 6> accelerations;
  bool has_accelerations{false};
  double time_from_start{0.0};
};

struct CanonicalTrajectory {
  std::array<std::string, 6> joint_names;
  std::vector<CanonicalTrajectoryPoint> points;
};

struct GoalValidationResult {
  bool valid{false};
  std::string error_code;
  std::string detail;
  CanonicalTrajectory trajectory;
};
```

Validation order is deterministic:

```text
points.size >= 2
joint_names.size == 6
set(joint_names) == {joint1..joint6}, no duplicates
positions.size == 6 for every point
velocities.size == 6 for every point
accelerations empty or size == 6
all present numeric values finite
time finite, >=0, strictly increasing
```

- [ ] **Step 4: Add explicit dependencies and pass the full matrix**

Place `canonicalNames()` and `validTrajectory()` in `test/trajectory_test_fixtures.hpp` so later driver tests use the same valid baseline. Add `control_msgs`, `trajectory_msgs`, and `actionlib_msgs` to CMake/package dependencies rather than relying on transitive includes. Add one mutation test for each of: 0/1 point, extra/missing/duplicate name, positions size, velocities size, acceleration size, NaN/Inf in positions/velocities/accelerations/time, negative time, and duplicate/decreasing time. Each mutation starts from `validTrajectory(canonicalNames())`, asserts its exact reject code, and asserts no command sink was constructed.

```bash
catkin_make -C agx --pkg dobot_v4_bringup --make-args run_tests_dobot_v4_bringup_gtest_test_trajectory_goal_validator
```

Expected: all validation cases pass without constructing a commander.

- [ ] **Step 5: Commit strict input validation**

```bash
git add agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/include/dobot_v4_bringup/trajectory_goal_validator.hpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/trajectory_goal_validator.cpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/trajectory_test_fixtures.hpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/test_trajectory_goal_validator.cpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/CMakeLists.txt \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/package.xml
git commit -m "feat: validate CR10 trajectory goals strictly"
```

---

### Task 2: Implement one-tick Hermite sampling and explicit pre-start hold

**Files:**
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/include/dobot_v4_bringup/trajectory_runner.hpp`
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/trajectory_runner.cpp`
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/test_trajectory_runner.cpp`
- Modify: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/CMakeLists.txt`

**Interfaces:**
- Consumes: `CanonicalTrajectory`, steady elapsed time, and actual q/qd.
- Produces at most one six-axis radian command and one state transition per `tick()`.

- [ ] **Step 1: Write failing one-tick and hold tests**

```cpp
static RunnerConfig runnerConfig(bool remani_hold = false) {
  RunnerConfig config;
  config.servoj_period = 0.10;
  config.goal_joint_tol = 0.02;
  config.stop_velocity_tol = 0.01;
  config.settle_timeout = 2.0;
  config.required_settle_samples = 3;
  config.remani_prestart_hold_mode = remani_hold;
  return config;
}

static CanonicalTrajectoryPoint point(double q, double time) {
  CanonicalTrajectoryPoint p;
  p.positions.fill(q);
  p.velocities.fill(0.0);
  p.accelerations.fill(0.0);
  p.time_from_start = time;
  return p;
}

static CanonicalTrajectory twoSecondTrajectory() {
  CanonicalTrajectory trajectory;
  trajectory.joint_names = {"joint1", "joint2", "joint3",
                            "joint4", "joint5", "joint6"};
  trajectory.points = {point(0.0, 0.0), point(0.02, 2.0)};
  return trajectory;
}

static CanonicalTrajectory remaniHoldTrajectory(double lead) {
  CanonicalTrajectory trajectory;
  trajectory.joint_names = {"joint1", "joint2", "joint3",
                            "joint4", "joint5", "joint6"};
  trajectory.points = {point(0.0, 0.0), point(0.01, lead),
                       point(0.02, lead + 1.0)};
  return trajectory;
}

static RunnerActualState actualAt(double q) {
  RunnerActualState actual;
  actual.q.fill(q);
  actual.qd.fill(0.0);
  return actual;
}

TEST(TrajectoryRunner, TickReturnsExactlyOneCommand) {
  TrajectoryRunner runner(runnerConfig());
  ASSERT_TRUE(runner.start(twoSecondTrajectory(), 10.0).accepted);
  const auto tick = runner.tick(10.1, actualAt(0.0));
  EXPECT_TRUE(tick.has_command);
  EXPECT_EQ(6u, tick.command_rad.size());
  EXPECT_EQ(1u, runner.tickCount());
}

TEST(TrajectoryRunner, RemaniModeHoldsPointZeroUntilPointOneTime) {
  TrajectoryRunner runner(runnerConfig(true));
  runner.start(remaniHoldTrajectory(1.0), 20.0);
  const auto before = runner.tick(20.9, actualAt(0.0));
  EXPECT_EQ(RunnerState::Holding, before.state);
  EXPECT_EQ(point(0.0, 0.0).positions, before.command_rad);
  const auto at_start = runner.tick(21.0, actualAt(0.0));
  EXPECT_EQ(RunnerState::Running, at_start.state);
  EXPECT_EQ(point(0.01, 1.0).positions, at_start.command_rad);
}
```

- [ ] **Step 2: Define runner state and tick result**

```cpp
enum class RunnerState {
  Idle, Holding, Running, Settling, Canceling,
  Succeeded, Canceled, Aborted
};

struct RunnerTick {
  RunnerState state{RunnerState::Idle};
  bool has_command{false};
  std::array<double, 6> command_rad{};
  std::array<double, 6> desired_velocity{};
  bool terminal{false};
  std::string error_code;
};

struct RunnerConfig {
  double servoj_period{0.10};
  double goal_joint_tol{0.02};
  double stop_velocity_tol{0.01};
  double settle_timeout{2.0};
  int required_settle_samples{3};
  bool remani_prestart_hold_mode{false};
};

struct RunnerActualState {
  std::array<double, 6> q{};
  std::array<double, 6> qd{};
};
```

The public runner methods are `StartDecision start(const CanonicalTrajectory&, double steady_start_sec)`, `RunnerTick tick(double steady_now_sec, const RunnerActualState&)`, `void requestCancel()`, and `std::size_t tickCount() const`.

- [ ] **Step 3: Implement Hermite sampling without callback loops**

Precompute the point-time vector on `start()`. `tick()` uses `std::upper_bound` to select one segment and samples the six joints once:

```cpp
q(t) = q0 + v0*t + c*t*t + d*t*t*t;
c = (-3*q0 + 3*q1 - 2*T*v0 - T*v1)/(T*T);
d = ( 2*q0 - 2*q1 +   T*v0 + T*v1)/(T*T*T);
```

No `for` over trajectory points, `while` over a segment, `ros::Rate`, or sleep may appear in `tick()`. A fixed six-joint loop is allowed for the six scalar polynomials.

- [ ] **Step 4: Lock REMANI hold semantics without breaking MoveIt default behavior**

`remani_prestart_hold_mode` defaults false in the Dobot bringup launch. The unified REMANI real launch sets it true. In true mode:

```text
point[0].time == 0 and velocity == 0
hold point[0] while elapsed < point[1].time
point[1] is the sole original REMANI t=0 sample
sample point[1] onward normally
```

Reject a tagged REMANI goal if these conditions are not satisfied. Generic/MoveIt goals retain normal interpolation when the parameter is false.

- [ ] **Step 5: Run runner tests**

Include segment-boundary, exact final time, pre-start hold, default interpolation, and no-command-after-cancel tests.

```bash
catkin_make -C agx --pkg dobot_v4_bringup --make-args run_tests_dobot_v4_bringup_gtest_test_trajectory_runner
```

- [ ] **Step 6: Commit non-blocking sampling core**

```bash
git add agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/include/dobot_v4_bringup/trajectory_runner.hpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/trajectory_runner.cpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/test_trajectory_runner.cpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/CMakeLists.txt
git commit -m "feat: sample CR10 goals without blocking callbacks"
```

---

### Task 3: Replace the blocking CRRobot Action callbacks

**Files:**
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/include/dobot_v4_bringup/follow_joint_trajectory_adapter.hpp`
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/follow_joint_trajectory_adapter.cpp`
- Modify: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/include/dobot_v4_bringup/cr5_v4_robot.h:124-332`
- Modify: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/cr5_v4_robot.cpp:20-39,491-611`
- Modify: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/launch/bringup_v4.launch:9-18`
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/fake_commander.hpp`
- Create: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/test_action_adapter.cpp`
- Modify: `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/CMakeLists.txt`

**Interfaces:**
- Consumes: Action goals/cancels, timer ticks, `getJointState()`, and commander `motionDoCmd`/`Stop()`.
- Produces: accepted/rejected/canceled/aborted/succeeded Action states with prompt callback return.

- [ ] **Step 1: Introduce a narrow command sink for hardware-free adapter tests**

```cpp
class Cr10CommandSink {
public:
  virtual ~Cr10CommandSink() = default;
  virtual bool sendServoJ(const std::array<double, 6>& q_rad,
                          double duration_sec) = 0;
  virtual bool stop() = 0;
  virtual std::array<double, 6> actualQ() = 0;
};

struct AdapterDecision {
  bool accepted{false};
  RunnerState state{RunnerState::Idle};
  bool publish_feedback{false};
  bool terminal{false};
  std::string error_code;
  std::string detail;
};

class FollowJointTrajectoryAdapter {
public:
  FollowJointTrajectoryAdapter(Cr10CommandSink* sink, RunnerConfig config);
  AdapterDecision accept(const trajectory_msgs::JointTrajectory& trajectory,
                         double steady_now_sec);
  AdapterDecision timerTick(double steady_now_sec);
  AdapterDecision requestCancel(double steady_now_sec);
  bool hasActiveGoal() const;
};
```

The production sink converts radians to degrees only when formatting:

```text
servoj(q1_deg,...,q6_deg,t=<servoj_period*1.5>)
```

Check `err_id==0`; exceptions or nonzero errors return false.
The current `commander_->Stop()` returns `void`; the production `stop()` wrapper returns true after a no-exception call and false on any caught transport/commander exception. Stop success is still not treated as stopped until three actual-q velocity samples meet the threshold.

- [ ] **Step 2: Write a failing callback-latency/one-send test**

```cpp
TEST(FollowJointTrajectoryAdapter, AcceptsWithoutSendingWholeTrajectory) {
  FakeCommander sink;
  FollowJointTrajectoryAdapter adapter(&sink, runnerConfig());
  const auto result = adapter.accept(
      validTrajectory(canonicalNames()), 1.0);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(0u, sink.servoJCalls());
  adapter.timerTick(1.1);
  EXPECT_EQ(1u, sink.servoJCalls());
}
```

`test_action_adapter.cpp` includes `trajectory_test_fixtures.hpp` and defines the same `runnerConfig()` values from Task 2. `FakeCommander` initializes actual q/qd to zero, increments `servoJCalls()` in `sendServoJ()`, and makes `stop()` set actual qd to zero and increment `stopCalls()`.

- [ ] **Step 3: Replace current `moveHandle()` blocking body**

Delete the current trajectory-wide `for`, segment `while`, `ros::Rate timer`, and `timer.sleep()`. `goalHandle()` must:

```text
reject if an active goal exists
validate and reorder
runner.start(canonical, steady_now)
cache active GoalHandle
setAccepted
start servoj timer
return
```

`CRRobot` owns one `FollowJointTrajectoryAdapter`; its timer callback must:

```text
read current q
estimate qd from timestamped q
runner.tick(steady_now, q, qd)
if has_command: send one ServoJ
publish one feedback
apply at most one terminal Action state
return
```

- [ ] **Step 4: Implement cancel/Stop/actual-stop confirmation**

`cancelHandle()` only marks cancel requested, stops future trajectory sampling, calls `sink.stop()` once, and returns. While `RunnerState::Canceling`, timer ticks read actual q/qd but never send ServoJ. After three samples with max `|qd|<=0.01 rad/s`, call `active_handle_.setCanceled(result, "stopped")`. Stop exception/failure or timeout calls `setAborted` and exposes an error.

- [ ] **Step 5: Implement measured success and settle timeout**

After elapsed reaches final duration, enter Settling. Succeed only after three consecutive samples satisfy:

```text
max |q_actual - q_goal| <= 0.02 rad
max |qd_actual| <= 0.01 rad/s
```

After 2.0 s without meeting both, call `setAborted(..., "completion tolerance")`. Never call `setSucceeded()` from cancel.

The adapter reports terminal state only; `CRRobot` is the sole owner of `GoalHandle::setAccepted/setRejected/setCanceled/setAborted/setSucceeded`, so a terminal status is emitted once. The driver also publishes the first `RunnerState::Running` ServoJ steady timestamp once per goal on `/remani/cr10_first_non_hold_servoj_steady` as `std_msgs/Float64` for shared-T0 diagnostics.

- [ ] **Step 6: Expose parameters and keep one active goal**

Add to `bringup_v4.launch`:

```xml
<arg name="servoj_period" default="0.10"/>
<arg name="cr10_goal_joint_tol" default="0.02"/>
<arg name="cr10_stop_velocity_tol" default="0.01"/>
<arg name="completion_settle_timeout" default="2.0"/>
<arg name="remani_prestart_hold_mode" default="false"/>
```

Replace the hidden `SERVOJ_DURATION=0.4`; no 0.40 s execution constant remains.

Retain the `RobotStatus.has_error` and `robot_mode` read-only fields introduced by the control-plane phase. The Action refactor must not remove or repurpose them, and State Bridge/Executor never calls `GetErrorID` as a readiness probe.

- [ ] **Step 7: Run static scan, build, and adapter tests**

```bash
rg -n "ros::Rate|\.sleep\(|while \(|for \(" \
  agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/cr5_v4_robot.cpp
catkin_make -C agx --pkg dobot_v4_bringup
catkin_make -C agx --pkg dobot_v4_bringup --make-args run_tests_dobot_v4_bringup
catkin_test_results agx/build/dobot_v4_bringup/test_results
```

Review the scan manually: background feedback thread loops may remain, but the FollowJointTrajectory timer path must have no full-trajectory loop/sleep.

- [ ] **Step 8: Commit the Action refactor**

```bash
git add agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/include/dobot_v4_bringup/cr5_v4_robot.h \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/include/dobot_v4_bringup/follow_joint_trajectory_adapter.hpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/follow_joint_trajectory_adapter.cpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/cr5_v4_robot.cpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/launch/bringup_v4.launch \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/fake_commander.hpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/test/test_action_adapter.cpp \
        agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/CMakeLists.txt
git commit -m "feat: make CR10 trajectory action preemptible"
```

---

### Task 4: Generate complete REMANI arm trajectories and add the action channel

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/arm_trajectory_builder.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/arm_trajectory_builder.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/cr10_hardware_channel.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/cr10_hardware_channel.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_arm_trajectory_builder.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/package.xml`

**Interfaces:**
- Consumes: `FrozenCandidate`, actual six-axis q, `start_lead_time`, and sample period.
- Produces: one valid `trajectory_msgs::JointTrajectory` and non-dry Action/cancel/Stop channel.

- [ ] **Step 1: Write failing hold/timestamp/velocity tests**

```cpp
static Eigen::Matrix<double, 6, 1> currentQ() {
  Eigen::Matrix<double, 6, 1> q;
  q << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60;
  return q;
}

static std::vector<double> currentQVector() {
  const auto q = currentQ();
  return std::vector<double>(q.data(), q.data() + q.size());
}

static FrozenCandidate armCandidate(double initial_offset) {
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  coeff.col(7).head<2>().setZero();
  coeff.col(7).tail<6>() =
      (currentQ().array() + initial_offset).matrix();
  coeff(2, 6) = 0.01;
  MMController::Trajectory trajectory;
  trajectory.emplace_back(1.0, coeff);
  CandidateSegment segment;
  segment.trajectory_id = 1;
  segment.singul = 1;
  segment.trajectory = trajectory;
  segment.start_time = 0.0;
  segment.duration = 1.0;
  return std::make_shared<const CandidateTrajectory>(
      9, std::vector<CandidateSegment>{segment}, 0.0);
}

TEST(ArmTrajectoryBuilder, AddsOneHoldPointAndStrictTimes) {
  const auto result = ArmTrajectoryBuilder::build(
      armCandidate(0.0), currentQ(), 1.0, 0.10, 0.02);
  ASSERT_TRUE(result.valid);
  const auto& trajectory = result.trajectory;
  ASSERT_GE(trajectory.points.size(), 3u);
  EXPECT_DOUBLE_EQ(0.0, trajectory.points[0].time_from_start.toSec());
  EXPECT_EQ(currentQVector(), trajectory.points[0].positions);
  EXPECT_EQ(std::vector<double>(6, 0.0), trajectory.points[0].velocities);
  EXPECT_DOUBLE_EQ(1.0, trajectory.points[1].time_from_start.toSec());
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    EXPECT_LT(trajectory.points[i-1].time_from_start,
              trajectory.points[i].time_from_start);
    EXPECT_EQ(6u, trajectory.points[i].positions.size());
    EXPECT_EQ(6u, trajectory.points[i].velocities.size());
  }
}

TEST(ArmTrajectoryBuilder, RejectsLargeHoldHandoffError) {
  EXPECT_EQ("ARM_HOLD_HANDOFF_TOLERANCE",
            ArmTrajectoryBuilder::build(
                armCandidate(0.03), currentQ(),
                1.0, 0.10, 0.02).error_code);
}
```

- [ ] **Step 2: Implement exact sampling/time mapping**

Names are always `joint1..joint6`. Point zero is actual q/zero velocity at t=0. Original candidate arm sample at `t_remani=0` is the sole point at `time_from_start=start_lead_time`; subsequent points map to `start_lead_time+t_remani`. Include exact candidate final time even when it is not a sample-period multiple. Reject max `|current_q-candidate_q(0)|>0.02 rad` and any generated joint speed over 0.10 rad/s.

- [ ] **Step 3: Implement the non-dry CR10 channel**

```cpp
class Cr10ReadinessClient {
public:
  Cr10ReadinessClient(const std::string& action_name);
  bool serverReady(const ros::Duration& timeout) const;
};

class Cr10HardwareChannel : public ArmCommandChannel {
public:
  Cr10HardwareChannel(ros::NodeHandle& nh,
                      const std::string& action_name,
                      const std::string& stop_service,
                      const std::string& emergency_stop_service,
                      bool emergency_stop_on_stop_failure);
  bool send(const trajectory_msgs::JointTrajectory& trajectory) override;
  bool cancel() override;
  bool stop() override;
  ArmGoalState state() const override;
  bool hardwareOutputEnabled() const override { return true; }
};
```

`Cr10ReadinessClient` may be constructed in dry-run and exposes no send/cancel/Stop method. `Cr10HardwareChannel` is constructed only when `dry_run=false`; the composition layer enforces that invariant before construction. `cancel()` requests Action cancellation; `stop()` calls `/dobot_v4_bringup/srv/Stop` only through this non-dry channel. `emergency_stop_on_stop_failure` defaults false; after the EmergencyStop service is separately validated, setting it true permits one `/dobot_v4_bringup/srv/EmergencyStop` call only after Stop returns failure or actual-stop timeout. Either path still returns failure and leaves Executor in ERROR. Dry-run cannot construct or invoke either client.

- [ ] **Step 4: Run builder/channel unit tests**

```bash
catkin_make -C remani_planner --pkg remani_real
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_arm_trajectory_builder
```

- [ ] **Step 5: Commit arm generation and channel**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/arm_trajectory_builder.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/arm_trajectory_builder.cpp \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/cr10_hardware_channel.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/cr10_hardware_channel.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_arm_trajectory_builder.cpp \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt \
        remani_planner/src/REMANI-Planner/remani_real/package.xml
git commit -m "feat: adapt REMANI candidates to CR10 action"
```

---

### Task 5: Gate CR10-only execution and cancel on a fake server, then hardware

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/fake_cr10_action_server.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/cr10_only_execution.test`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_cr10_only_execution.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/launch/cr10_only_low_speed.launch`
- Create: `remani_planner/src/REMANI-Planner/remani_real/docs/cr10_only_acceptance.md`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: frozen candidates with stationary base and arm speed ≤0.05 rad/s for staged validation.
- Produces: fake-server evidence and a separately authorized CR10-only physical gate.

- [ ] **Step 1: Build a fake server that records every goal and cancel**

The fake server validates six names/full position+velocity arrays, publishes deterministic joint feedback, honors the pre-start hold, and exposes counters for goal, cancel, Stop, ServoJ-equivalent samples, and terminal state.

- [ ] **Step 2: Test accept deadline, hold, cancel, and completion**

The rostest asserts:

```python
self.assertEqual(0, servo_samples_before_hold_end)
self.assertLess(cancel_callback_latency, 0.20)
self.assertEqual(GoalStatus.PREEMPTED, canceled_goal_status)
self.assertNotEqual(GoalStatus.SUCCEEDED, canceled_goal_status)
self.assertEqual(GoalStatus.ABORTED, tolerance_failure_status)
self.assertEqual(GoalStatus.SUCCEEDED, settled_goal_status)
```

Also send every malformed-goal case from Task 1 and assert REJECTED with zero samples.

- [ ] **Step 3: Create a locked CR10-only launch**

Require `enable_staged_test=true` for `dry_run=false`; set base trajectory identically stationary, arm max speed 0.05 rad/s, `remani_prestart_hold_mode=true`, physical move range per joint ≤0.10 rad, and no Ranger hardware node/publisher.

- [ ] **Step 4: Run all software gates**

```bash
catkin_make -C agx --pkg dobot_v4_bringup
catkin_make -C agx --pkg dobot_v4_bringup --make-args run_tests_dobot_v4_bringup
catkin_make -C remani_planner --pkg remani_real
rostest remani_real cr10_only_execution.test
catkin_test_results agx/build/dobot_v4_bringup/test_results
catkin_test_results remani_planner/build/remani_real/test_results
```

- [ ] **Step 5: Execute the physical checklist only after dry-run approval**

The acceptance document requires physical E-stop, enabled/connected/fault-free RobotStatus, a stationary Ranger, a ≤0.10 rad per-joint trajectory, measured Action accept latency, early/mid/late cancel, Stop verification, actual velocity settling, and Action result evidence. Any cancel/Stop/completion failure blocks synchronized execution.

- [ ] **Step 6: Commit CR10 staged validation assets**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/test/fake_cr10_action_server.py \
        remani_planner/src/REMANI-Planner/remani_real/test/cr10_only_execution.test \
        remani_planner/src/REMANI-Planner/remani_real/test/test_cr10_only_execution.py \
        remani_planner/src/REMANI-Planner/remani_real/launch/cr10_only_low_speed.launch \
        remani_planner/src/REMANI-Planner/remani_real/docs/cr10_only_acceptance.md \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "test: gate CR10 only low speed execution"
```

## Phase Exit Gate

```text
malformed goal -> REJECTED, zero ServoJ
accepted goal callback -> returns before first ServoJ
timer tick -> at most one sample/send
cancel -> no future ServoJ + Stop + actual stop + PREEMPTED/CANCELED
elapsed only -> never sufficient for SUCCEEDED
settled q/qd for 3 samples -> SUCCEEDED
CR10-only physical validation -> passed before synchronized motion
```
