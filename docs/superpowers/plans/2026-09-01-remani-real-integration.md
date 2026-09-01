# REMANI Real Synchronized Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate the already-gated Ranger and CR10 channels behind one monotonic execution epoch, add strict Pause/Resume/Abort and actual-state terminal verification, then expose the complete real deployment through one safe launch while preserving the default simulation path.

**Architecture:** `remani_real_node` remains the only real execution owner. A pure `ExecutionClock` defines `T0`; a pure `RemainingCandidate` slicer preserves the original whole-body path on Resume; a `RuntimeSafetyMonitor` and `CompletionVerifier` consume actual feedback. Hardware adapters are composed only when `dry_run=false`. The unified launch remaps raw hardware topics at their source and supplies actual odom/joints to both planner and Executor.

**Tech Stack:** ROS1 Noetic, catkin, C++14, Eigen, actionlib, TF2, RViz/Qt5, rostest, existing `mm_config`, `remani_real`, `remani_real_msgs`, `ranger_base`, and `dobot_v4_bringup`.

**Spec:** `docs/superpowers/specs/2026-09-01-remani-real-robot-deployment-design.md`

## Global Constraints

- Prerequisites: all four earlier phase plans pass their phase exit gates; do not start synchronized hardware motion before Ranger-only and CR10-only acceptance.
- The Planner remains PLAN-ONLY in real mode. This plan does not add `WAIT_EXTERNAL_EXECUTE`, reactivate planner `EXEC_TRAJ`, or synchronize planner wall time with the Executor.
- `remani_real_node` owns the only monotonic execution clock and both hardware outputs.
- `dry_run=true` exercises timing, readiness, sampling, state transitions, completion calculations, and preview, but constructs no write-capable hardware adapter.
- Pause/Resume V1 never creates a connector. Resume uses the unmodified remaining suffix only when strict actual-state tolerances pass.
- Action success is necessary but not sufficient for real whole-body success; actual base, arm, velocity, feedback, RobotStatus, and EE FK must all pass.
- `mode:=sim` remains the no-argument default of `run_remani.sh` and launches the existing `remani_sim.launch` path unchanged.
- Physical E-stop, clear static-empty field, conservative limits, and operator confirmation remain mandatory.
- New/changed C++ blocks use one repository-style comment banner before the block.

## Plan-set boundary

This is phase 5 of five. It consumes these interfaces without renaming them:

1. `2026-09-01-remani-real-planner-plan-only.md`: messages, PLAN-ONLY planner, transaction, static-empty map.
2. `2026-09-01-remani-real-control-plane.md`: actual state, Gate, state machine, preview, Panel, output interfaces.
3. `2026-09-01-remani-real-ranger-execution.md`: finite Ranger control, isolated channel, watchdog.
4. `2026-09-01-remani-real-cr10-action.md`: non-blocking Action, arm builder, cancel/Stop channel.
5. This plan: shared T0, synchronized ownership, Pause/Resume/Abort, terminal FK, unified launch and regression.

---

## File Structure

| Path | Responsibility |
|---|---|
| `remani_real/include/remani_real/execution_clock.hpp` | Pure monotonic T0 and parameter-time mapping |
| `remani_real/include/remani_real/synchronized_executor.hpp` | Single-owner Ranger/CR10 start, tick, pause, resume and abort orchestration |
| `remani_real/include/remani_real/remaining_candidate.hpp` | Exact suffix extraction without geometric connector |
| `remani_real/include/remani_real/runtime_safety_monitor.hpp` | Feedback/action/watchdog/ownership/tracking runtime fault aggregation |
| `remani_real/include/remani_real/completion_verifier.hpp` | Actual base/joint/EE FK terminal decision |
| `remani_real/src/remani_real_node.cpp` | ROS composition and services; no secondary execution state machine |
| `remani_real/config/remani_real.yaml` | Locked V1 timing, resume, safety and completion thresholds |
| `remani_real/launch/remani_real.launch` | Unified hardware/dry-run real launch and hard topic remaps |
| `remani_real/test/fake_whole_body_system.py` | Fake odom, raw joints, RobotStatus, Action and Ranger sink |
| `remani_real/test/synchronized_dry_run.test` | T0/hold/no-output integration gate |
| `remani_real/test/pause_resume_abort.test` | Original-suffix and stop semantics gate |
| `remani_real/test/final_completion.test` | Actual-state/FK terminal gate |
| `remani_real/test/real_launch_contract.test` | Remap, initialization, warning and process contract |
| `remani_real/docs/synchronized_acceptance.md` | Authorized staged hardware checklist and evidence fields |
| `remani_planner/run_remani.sh` | Default sim / explicit real launcher selection |

---

### Task 1: Give the Executor one monotonic epoch and Action acceptance deadline

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/execution_clock.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/execution_clock.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/synchronized_executor.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/synchronized_executor.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_execution_clock.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_synchronized_start.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: frozen candidate, actual state, `RangerCommandChannel`, `ArmCommandChannel`, steady time, `start_lead_time`, and `arm_accept_guard`.
- Produces: exactly one `ExecutionEpoch`, parameter-time samples, start records, and an error before either device is allowed to run alone.

- [ ] **Step 1: Write the failing epoch boundary test**

```cpp
TEST(ExecutionClock, MapsOneRequestedT0ToCandidateTime) {
  const ExecutionEpoch epoch = ExecutionClock::begin(
      ros::SteadyTime(100, 0), 1.0);
  EXPECT_DOUBLE_EQ(101.0, epoch.t0.toSec());
  EXPECT_FALSE(ExecutionClock::parameterTime(
      epoch, ros::SteadyTime(100, 900000000)).has_value());
  EXPECT_DOUBLE_EQ(0.0, ExecutionClock::parameterTime(
      epoch, ros::SteadyTime(101, 0)).value());
  EXPECT_DOUBLE_EQ(0.25, ExecutionClock::parameterTime(
      epoch, ros::SteadyTime(101, 250000000)).value());
}
```

Use `boost::optional<double>` because the workspace is C++14; do not use `std::optional`.

- [ ] **Step 2: Implement the immutable execution epoch**

```cpp
struct ExecutionEpoch {
  ros::SteadyTime requested_at;
  ros::SteadyTime t0;
  double start_lead_time{0.0};
};

class ExecutionClock {
public:
  static ExecutionEpoch begin(const ros::SteadyTime& requested_at,
                              double start_lead_time);
  static boost::optional<double> parameterTime(
      const ExecutionEpoch& epoch, const ros::SteadyTime& now);
};
```

Reject non-finite lead time and values `<=0`. `parameterTime()` returns none before `T0`, and `max(0, now-T0)` at/after `T0`. It never reads ROS wall/sim time.

- [ ] **Step 3: Define exact start records and Executor result types**

```cpp
struct StartTimingRecord {
  ros::SteadyTime requested_t0;
  ros::SteadyTime arm_goal_sent_at;
  boost::optional<ros::SteadyTime> arm_goal_accepted_at;
  boost::optional<ros::SteadyTime> ranger_trajectory_start_at;
  boost::optional<ros::SteadyTime> ranger_first_motion_at;
  boost::optional<ros::SteadyTime> cr10_first_motion_at;
  double ranger_start_error{0.0};
  double cr10_start_error{0.0};
  boost::optional<double> start_skew;
};

enum class ExecutorStepState {
  HoldingForT0, Running, Stopping, Paused, Finished, Error
};

struct ExecutorStepResult {
  ExecutorStepState state{ExecutorStepState::HoldingForT0};
  double parameter_time{0.0};
  std::string error_code;
  std::string detail;
};
```

`requested_t0` is a steady timestamp serialized to `ExecutionState.requested_t0` with `.toSec()` only for diagnostics. It is never compared to `ros::Time`.

- [ ] **Step 4: Write the failing Action deadline test with strict fakes**

```cpp
static SynchronizedExecutorConfig testExecutorConfig() {
  SynchronizedExecutorConfig config;
  config.dry_run = false;
  config.start_lead_time = 1.0;
  config.arm_accept_guard = 0.20;
  config.max_start_skew = 0.10;
  return config;
}

static ActualStateSnapshot actualAtOrigin() {
  ActualStateSnapshot actual;
  actual.base_xy.setZero();
  actual.base_yaw = 0.0;
  actual.q << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60;
  actual.qd.setZero();
  actual.odom_valid = actual.joints_valid = actual.velocity_valid = true;
  actual.tf_valid = actual.robot_connected = actual.robot_enabled = true;
  return actual;
}

static FrozenCandidate candidateAtOrigin() {
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  coeff.col(7).tail<6>() = actualAtOrigin().q;
  coeff(0, 6) = 0.05;
  MMController::Trajectory trajectory;
  trajectory.emplace_back(2.0, coeff);
  CandidateSegment segment;
  segment.trajectory_id = 1;
  segment.singul = 1;
  segment.trajectory = trajectory;
  segment.start_time = 0.0;
  segment.duration = 2.0;
  return std::make_shared<const CandidateTrajectory>(
      1, std::vector<CandidateSegment>{segment}, 0.0);
}

class FakeRangerChannel : public RangerCommandChannel {
public:
  bool publish(const geometry_msgs::Twist& value) override {
    commands.push_back(value);
    return true;
  }
  bool hardwareOutputEnabled() const override { return true; }
  bool onlyZeroCommands() const {
    return std::all_of(commands.begin(), commands.end(), [](const auto& cmd) {
      return cmd.linear.x == 0.0 && cmd.linear.y == 0.0 &&
             cmd.angular.z == 0.0;
    });
  }
  std::vector<geometry_msgs::Twist> commands;
};

class FakeArmChannel : public ArmCommandChannel {
public:
  bool send(const trajectory_msgs::JointTrajectory&) override {
    ++send_count;
    return true;
  }
  bool cancel() override { ++cancel_count; return true; }
  bool stop() override { ++stop_count; return true; }
  ArmGoalState state() const override { return state_value; }
  bool hardwareOutputEnabled() const override { return true; }
  void setAcceptState(ArmGoalState value) { state_value = value; }
  std::size_t cancelCount() const { return cancel_count; }
  ArmGoalState state_value{ArmGoalState::Pending};
  std::size_t send_count{0}, cancel_count{0}, stop_count{0};
};

TEST(SynchronizedStart, NeverStartsRangerWithoutAcceptedArmGoal) {
  FakeRangerChannel ranger;
  FakeArmChannel arm;
  arm.setAcceptState(ArmGoalState::Pending);
  SynchronizedExecutor executor(testExecutorConfig(), &ranger, &arm);

  ASSERT_TRUE(executor.prepare(candidateAtOrigin(), actualAtOrigin(),
                               ros::SteadyTime(10, 0)).accepted);
  EXPECT_EQ(ExecutorStepState::HoldingForT0,
            executor.tick(ros::SteadyTime(10, 700000000), actualAtOrigin()).state);
  EXPECT_TRUE(ranger.onlyZeroCommands());

  const auto late = executor.tick(
      ros::SteadyTime(10, 810000000), actualAtOrigin());
  EXPECT_EQ(ExecutorStepState::Error, late.state);
  EXPECT_EQ("ARM_ACCEPT_DEADLINE", late.error_code);
  EXPECT_EQ(1u, arm.cancelCount());
  EXPECT_TRUE(ranger.onlyZeroCommands());
}
```

Keep these helpers in `test_synchronized_start.cpp`, not in production headers. Add accepted-before-deadline, exact-T0, fixed-T0-after-accept, and Action-send-failure cases using the same fakes.

- [ ] **Step 5: Implement synchronized prepare/tick ownership**

```cpp
struct SynchronizedExecutorConfig {
  bool dry_run{true};
  double start_lead_time{1.0};
  double arm_accept_guard{0.20};
  double max_start_skew{0.10};
};

class SynchronizedExecutor {
public:
  SynchronizedExecutor(SynchronizedExecutorConfig config,
                       RangerCommandChannel* ranger,
                       ArmCommandChannel* arm);
  ExecuteDecision prepare(FrozenCandidate candidate,
                          const ActualStateSnapshot& actual,
                          const ros::SteadyTime& request_time);
  ExecutorStepResult tick(const ros::SteadyTime& now,
                          const ActualStateSnapshot& actual);
  const StartTimingRecord& timing() const;
};
```

`prepare()` performs final readiness/start checks, builds the arm hold trajectory, creates one epoch, and immediately submits the CR10 goal only when non-dry. `tick()` follows this order:

```text
now < T0:
  dry-run -> calculate/store zero command only
  non-dry -> Ranger publish zero
  if Action accepted -> retain hold
  if now >= T0-arm_accept_guard and not accepted -> cancel, zero, ERROR

now >= T0:
  require accepted Action in non-dry
  set t_remani=steady_now-T0
  record first post-T0 Ranger sample timestamp once
  sample both devices from the same t_remani
  publish Ranger command only in non-dry
```

Do not shift `T0` after the Action accepts. In dry-run, use a deterministic synthetic accepted-at time equal to `requested_at` for timing state only; do not create an Action client or send a goal.

- [ ] **Step 6: Record skew without treating a stationary base as failure**

The CR10 driver publishes `/remani/cr10_first_non_hold_servoj_steady` as `std_msgs/Float64`; the Ranger Executor records its own first post-T0 sampling time. Compute:

```cpp
start_skew = cr10_first_motion_at->toSec()
           - ranger_trajectory_start_at->toSec();
```

`ranger_first_motion_at` is a separate optional field set only when `hypot(v,w)>command_zero_epsilon`. A missing first non-zero Ranger command on a stationary-base trajectory remains unavailable and does not invalidate `start_skew`.
When publishing `ExecutionState`, set `ranger_first_motion_available` and `start_skew_available` from the optionals; leave the associated numeric field at zero when unavailable rather than serializing NaN.

- [ ] **Step 7: Build and run the focused start tests**

```bash
catkin_make -C remani_planner --pkg remani_real
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_execution_clock
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_synchronized_start
```

Expected: exact pre/post-T0 boundary, fixed T0 after accept, late-accept failure, stationary-base timing, and `abs(start_skew)<=0.10` cases pass.

- [ ] **Step 8: Commit the shared execution clock**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/execution_clock.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/execution_clock.cpp \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/synchronized_executor.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/synchronized_executor.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_execution_clock.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_synchronized_start.cpp \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "feat: synchronize real execution on monotonic T0"
```

---

### Task 2: Implement Pause, strict suffix-only Resume, and Abort

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/remaining_candidate.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/remaining_candidate.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_remaining_candidate.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_pause_resume.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/synchronized_executor.hpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/src/synchronized_executor.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: `/remani/pause`, `/remani/resume`, `/remani/abort` (`std_srvs/Trigger`), actual stop state, and frozen original candidate.
- Produces: `PAUSED` only after both devices stop, a time-rebased original suffix on valid Resume, or candidate invalidation on Abort.

- [ ] **Step 1: Write the failing suffix preservation test**

```cpp
static FrozenCandidate twoSegmentCandidate() {
  const auto actual = actualAtOrigin();
  MMController::Piece::CoefficientMat c1 =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  c1.col(7).tail<6>() = actual.q;
  c1(0, 6) = 0.05;
  MMController::Trajectory t1;
  t1.emplace_back(1.0, c1);

  MMController::Piece::CoefficientMat c2 = c1;
  c2(0, 7) = 0.05;
  MMController::Trajectory t2;
  t2.emplace_back(1.0, c2);

  CandidateSegment s1;
  s1.trajectory_id = 1; s1.singul = 1; s1.trajectory = t1;
  s1.start_time = 0.0; s1.duration = 1.0;
  CandidateSegment s2;
  s2.trajectory_id = 2; s2.singul = 1; s2.trajectory = t2;
  s2.start_time = 1.0; s2.duration = 1.0;
  return std::make_shared<const CandidateTrajectory>(
      4, std::vector<CandidateSegment>{s1, s2}, 0.0);
}

static void expectWholeBodyNear(const WholeBodySample& expected,
                                const WholeBodySample& actual,
                                double tolerance) {
  EXPECT_TRUE(expected.position.isApprox(actual.position, tolerance));
  EXPECT_TRUE(expected.velocity.isApprox(actual.velocity, tolerance));
  EXPECT_TRUE(expected.acceleration.isApprox(actual.acceleration, tolerance));
  EXPECT_NEAR(expected.base_yaw, actual.base_yaw, tolerance);
  EXPECT_NEAR(expected.base_angular_velocity,
              actual.base_angular_velocity, tolerance);
  EXPECT_EQ(expected.singul, actual.singul);
}

TEST(RemainingCandidate, RebasesOnlyTheOriginalSuffix) {
  const FrozenCandidate original = twoSegmentCandidate();
  const WholeBodySample expected = original->sample(1.25);
  const auto suffix = RemainingCandidate::slice(original, 1.25);
  ASSERT_TRUE(suffix.valid);
  EXPECT_DOUBLE_EQ(original->duration() - 1.25,
                   suffix.candidate->duration());
  expectWholeBodyNear(expected, suffix.candidate->sample(0.0), 1e-12);
  EXPECT_EQ(expected.singul, suffix.candidate->sample(0.0).singul);
  EXPECT_EQ(original->sample(2.0).singul,
            suffix.candidate->sample(0.75).singul);
}
```

The test helper compares all 8 positions, all 8 velocities, yaw, angular velocity, and singul. It also asserts the first suffix polynomial is the exact analytic restriction of the original piece, not a newly fitted connector.

- [ ] **Step 2: Implement analytic polynomial restriction**

```cpp
struct RemainingCandidateResult {
  bool valid{false};
  FrozenCandidate candidate;
  std::string error_code;
};

class RemainingCandidate {
public:
  static RemainingCandidateResult slice(FrozenCandidate original,
                                        double pause_param_time);
};
```

For the containing polynomial piece, translate its local polynomial origin with the binomial shift `p_shifted(t)=p_original(t+t_cut)` and shorten its duration. Copy all later pieces byte-for-byte except rebased start times. Preserve `singul`, 8D synchronization and velocities. Reject non-finite time or values outside `[0,duration)`; do not fit a quintic, interpolate state dimensions independently, or collision-check a new path because no new geometry is introduced.

- [ ] **Step 3: Define exact Pause records and strict tolerances**

```cpp
struct PauseRecord {
  double pause_param_time{0.0};
  WholeBodySample expected_pause_state;
  ActualStateSnapshot actual_stop_state;
  bool ranger_stopped{false};
  bool cr10_stopped{false};
};

struct ResumeTolerance {
  double base_position{0.02};
  double base_yaw{2.0 * M_PI / 180.0};
  double arm_joint{1.0 * M_PI / 180.0};
};
```

Use `max(abs(actual.q-expected.q))` for arm error, Euclidean XY error for base, and normalized absolute yaw error.

- [ ] **Step 4: Write failure-first Pause/Resume tests**

```cpp
class PauseResumeTest : public ::testing::Test {
protected:
  PauseResumeTest()
      : executor(testExecutorConfig(), &ranger, &arm) {}

  ros::SteadyTime steady(double value) const {
    ros::SteadyTime time;
    time.fromSec(value);
    return time;
  }

  ActualStateSnapshot actualAtParameter(double parameter_time,
                                        double base_speed,
                                        double joint_speed) const {
    ActualStateSnapshot actual = actualAtOrigin();
    actual.base_xy.x() = 0.05 * parameter_time;
    actual.base_velocity_world.x() = base_speed;
    actual.qd.setConstant(joint_speed);
    return actual;
  }

  void startAndPauseAt075() {
    arm.setAcceptState(ArmGoalState::Accepted);
    ASSERT_TRUE(executor.prepare(candidateAtOrigin(), actualAtOrigin(),
                                 steady(10.0)).accepted);
    ASSERT_EQ(ExecutorStepState::Running,
              executor.tick(steady(11.75),
                            actualAtParameter(0.75, 0.05, 0.01)).state);
    executor.requestPause();
  }

  FakeRangerChannel ranger;
  FakeArmChannel arm;
  SynchronizedExecutor executor;
};

TEST_F(PauseResumeTest, EntersPausedOnlyAfterBothActualStops) {
  startAndPauseAt075();
  EXPECT_EQ(ExecutorStepState::Stopping,
            executor.tick(steady(11.76),
                          actualAtParameter(0.75, 0.05, 0.01)).state);
  EXPECT_EQ(ExecutorStepState::Stopping,
            executor.tick(steady(11.90),
                          actualAtParameter(0.75, 0.0, 0.01)).state);
  EXPECT_EQ(ExecutorStepState::Paused,
            executor.tick(steady(12.00),
                          actualAtParameter(0.75, 0.0, 0.0)).state);
}

TEST_F(PauseResumeTest, RejectsErrorAboveStrictToleranceWithoutConnector) {
  startAndPauseAt075();
  ASSERT_EQ(ExecutorStepState::Paused,
            executor.tick(steady(12.00),
                          actualAtParameter(0.75, 0.0, 0.0)).state);
  ActualStateSnapshot displaced = actualAtParameter(0.75, 0.0, 0.0);
  displaced.base_xy.x() += 0.021;
  const auto decision = executor.requestResume(displaced, steady(13.0));
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ("RESUME_BASE_POSITION_TOLERANCE", decision.error_code);
  EXPECT_EQ(ExecutorStepState::Paused, executor.state());
  EXPECT_EQ(0u, executor.generatedConnectorCount());
}
```

Define all fixture constructors in `test_pause_resume.cpp`; `generatedConnectorCount()` is a test-visible invariant that always returns zero in V1.

- [ ] **Step 5: Implement Pause ordering**

On Pause request:

```text
freeze t_remani once as pause_param_time
save original.sample(pause_param_time)
dry-run: simulate both stopped on next tick without hardware calls
non-dry: Ranger zero continuously; arm cancel once; arm Stop once
read odom speed and qd until both below configured stop thresholds
save actual_stop_state
transition to PAUSED
```

Timeout, Action cancel failure, Stop failure, or a Ranger watchdog fault transitions to ERROR. No CR10 `Pause` service or current `Continue` service is called.

- [ ] **Step 6: Implement Resume as a new epoch over the suffix**

`requestResume()` re-reads current actual state; stale stop records cannot authorize Resume. If all three strict errors pass, call `RemainingCandidate::slice(original, pause_param_time)`, rebuild the arm trajectory with the current q hold, and call the same `prepare()` path to create a new `T0`. It must retain the original `candidate_id` for status/audit but mark the internal execution generation monotonically increasing. The first suffix sample must equal the original candidate at `pause_param_time` within `1e-9`.

- [ ] **Step 7: Implement Abort as terminal candidate invalidation**

Abort is valid from PLANNING, PLANNED, EXECUTING, PAUSED, SUCCEEDED, and ERROR:

```text
stop Ranger until confirmed
cancel Action if active
call Stop once if non-dry and arm may be moving
invalidate assembler and frozen/remaining candidate
publish candidate_complete=false and candidate_valid=false
return READY only when feedback is ready and both devices are stopped
```

Any delayed Execute request for the aborted `candidate_id` must return `accepted=false` and `message="candidate id is not current"`.

- [ ] **Step 8: Run focused Pause/Resume/Abort tests**

```bash
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_remaining_candidate
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_pause_resume
```

Expected: early/mid/late Pause, dual-stop confirmation, accepted strict Resume, all three tolerance failures, no connector, and Abort invalidation pass.

- [ ] **Step 9: Commit V1 stop/resume semantics**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/remaining_candidate.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/remaining_candidate.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_remaining_candidate.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_pause_resume.cpp \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/synchronized_executor.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/synchronized_executor.cpp \
        remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "feat: add strict pause resume and abort semantics"
```

---

### Task 3: Centralize runtime safety under the Real Executor

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/runtime_safety_monitor.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/runtime_safety_monitor.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_runtime_safety_monitor.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: actual-state ages, RobotStatus, Action state, Ranger ownership/watchdog, desired command, candidate limits, and tracking errors.
- Produces: one deterministic safety decision used by the Executor before every hardware output.

- [ ] **Step 1: Write a table-driven failure test**

```cpp
static RuntimeSafetyConfig safetyConfig() {
  RuntimeSafetyConfig config;
  config.dry_run = false;
  config.expected_ranger_publisher = "/remani_real_node";
  config.odom_timeout = 0.30;
  config.joint_timeout = 0.30;
  config.tf_timeout = 0.30;
  config.max_base_linear_speed = 0.10;
  config.max_base_angular_speed = 0.15;
  config.max_joint_speed = 0.10;
  config.max_base_tracking_error = 0.20;
  config.max_joint_tracking_error = 0.10;
  return config;
}

static RuntimeSafetyInput nominalRuntimeSafetyInput() {
  RuntimeSafetyInput input;
  input.odom_age = input.joint_age = input.tf_age = 0.0;
  input.joint_velocity_valid = true;
  input.robot_connected = input.robot_enabled = true;
  input.robot_fault = false;
  input.action_state = ArmGoalState::Active;
  input.ranger_watchdog_ready = true;
  input.ranger_watchdog_timed_out = false;
  input.hardware_topic_publishers = {"/remani_real_node"};
  input.base_tracking_error = input.joint_tracking_error = 0.0;
  input.desired_base_linear_speed = 0.05;
  input.desired_base_angular_speed = 0.05;
  input.desired_max_joint_speed = 0.05;
  return input;
}

TEST(RuntimeSafetyMonitor, ReportsTheFirstStableFaultCode) {
  RuntimeSafetyMonitor monitor(safetyConfig());
  RuntimeSafetyInput input = nominalRuntimeSafetyInput();
  input.odom_age = 0.31;
  EXPECT_EQ("ODOM_TIMEOUT", monitor.evaluate(input).error_code);

  input = nominalRuntimeSafetyInput();
  input.hardware_topic_publishers = {"/remani_real_node", "/rogue"};
  EXPECT_EQ("RANGER_PUBLISHER_OWNERSHIP",
            monitor.evaluate(input).error_code);

  input = nominalRuntimeSafetyInput();
  input.action_state = ArmGoalState::Aborted;
  EXPECT_EQ("CR10_ACTION_ABORTED", monitor.evaluate(input).error_code);
}
```

- [ ] **Step 2: Implement exact input/decision types and priority**

```cpp
struct RuntimeSafetyConfig {
  bool dry_run{true};
  std::string expected_ranger_publisher;
  double odom_timeout{0.30};
  double joint_timeout{0.30};
  double tf_timeout{0.30};
  double max_base_linear_speed{0.10};
  double max_base_angular_speed{0.15};
  double max_joint_speed{0.10};
  double max_base_tracking_error{0.20};
  double max_joint_tracking_error{0.10};
};

struct RuntimeSafetyInput {
  double odom_age{0.0};
  double joint_age{0.0};
  double tf_age{0.0};
  bool joint_velocity_valid{false};
  bool robot_connected{false};
  bool robot_enabled{false};
  bool robot_fault{false};
  ArmGoalState action_state{ArmGoalState::Unavailable};
  bool ranger_watchdog_ready{false};
  bool ranger_watchdog_timed_out{false};
  std::vector<std::string> hardware_topic_publishers;
  double base_tracking_error{0.0};
  double joint_tracking_error{0.0};
  double desired_base_linear_speed{0.0};
  double desired_base_angular_speed{0.0};
  double desired_max_joint_speed{0.0};
};

struct SafetyDecision {
  bool safe{false};
  std::string error_code;
  std::string detail;
};

class RuntimeSafetyMonitor {
public:
  explicit RuntimeSafetyMonitor(RuntimeSafetyConfig config);
  SafetyDecision evaluate(const RuntimeSafetyInput& input) const;
};
```

Evaluate in fixed priority: non-finite → feedback/TF timeout → velocity invalid → RobotStatus → Action → ownership → watchdog → nominal trajectory limit → tracking error. In dry-run, ownership is healthy only when the hardware topic has zero publishers and watchdog `ready` is true; an idle driver's `timed_out` flag is displayed but is not a dry-run motion fault. In non-dry EXECUTING/HoldingForT0, ownership requires the single expected publisher and `timed_out` is a fault after the first commanded zero has reset it. This stable ordering makes `last_error_code` testable.

- [ ] **Step 3: Lock V1 configuration values and units**

Add exact YAML keys:

```yaml
executor:
  tick_rate: 50.0
  start_lead_time: 1.0
  arm_accept_guard: 0.20
  max_start_skew: 0.10
  odom_timeout: 0.30
  joint_feedback_timeout: 0.30
  tf_timeout: 0.30
  stop_timeout: 2.0
  ranger_stop_linear_velocity: 0.01
  ranger_stop_angular_velocity: 0.02
  cr10_stop_velocity: 0.01
  max_base_linear_speed: 0.10
  max_base_angular_speed: 0.15
  max_joint_speed: 0.10
  max_base_tracking_error: 0.20
  max_joint_tracking_error: 0.10
  execute_start_base_position: 0.05
  execute_start_base_yaw: 0.0872664626
  execute_start_arm_joint: 0.0523598776
  resume_base_position: 0.02
  resume_base_yaw: 0.0349065850
  resume_arm_joint: 0.0174532925
```

Never silently clamp a candidate that violates nominal limits; reject before execution.

- [ ] **Step 4: Apply the monitor before output and stop on failure**

`remani_real_node` builds `RuntimeSafetyInput` from the latest coherent snapshot. Every EXECUTING tick evaluates safety before Ranger publish. Failure freezes parameter time, commands/cancels the same stop sequence as Abort, transitions to ERROR, and preserves the candidate for diagnosis until explicit Abort. The planner collision timer is not used here.

- [ ] **Step 5: Run safety tests**

```bash
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_runtime_safety_monitor
```

Expected: every input fault has one stable error code and no unsafe decision permits a hardware publish.

- [ ] **Step 6: Commit real execution safety ownership**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/runtime_safety_monitor.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/runtime_safety_monitor.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_runtime_safety_monitor.cpp \
        remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp \
        remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "feat: centralize real execution safety monitoring"
```

---

### Task 4: Verify terminal whole-body and EE state from actual feedback

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/completion_verifier.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/completion_verifier.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_completion_verifier.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/package.xml`

**Interfaces:**
- Consumes: frozen candidate end sample, latest actual base/q/qd, Action result, freshness/RobotStatus, and `MMConfig::getEePose`.
- Produces: `CompletionDecision`, `ExecutionResult`, and `ExecutionState` final metrics.

- [ ] **Step 1: Write the failing Action-is-not-enough test**

```cpp
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

static CompletionInput exactCompletionInput(const EeKinematics& kinematics) {
  CompletionInput input;
  input.expected_base_xy.setZero();
  input.expected_base_yaw = 0.0;
  input.expected_q << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60;
  input.expected_ee = kinematics.pose(
      Eigen::Vector3d::Zero(), input.expected_q);
  input.actual = actualAtOrigin();
  input.arm_action_succeeded = true;
  input.feedback_fresh = true;
  input.robot_status_healthy = true;
  return input;
}

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
```

The fixture constructs `MMConfig` with the same Ranger+CR10 config loaded by the planner, sets expected final pose using `getEePose(expected_car, expected_q)`, and makes every non-mutated readiness flag valid.

- [ ] **Step 2: Implement completion metrics with explicit rotation distance**

```cpp
struct CompletionThresholds {
  double base_position{0.05};
  double base_yaw{5.0 * M_PI / 180.0};
  double base_linear_velocity{0.01};
  double base_yaw_rate{0.02};
  double arm_joint{0.02};
  double arm_velocity{0.01};
  double ee_position{0.02};
  double ee_rotation{4.0 * M_PI / 180.0};
};

class EeKinematics {
public:
  virtual ~EeKinematics() = default;
  virtual Eigen::Matrix4d pose(
      const Eigen::Vector3d& car,
      const Eigen::Matrix<double, 6, 1>& q) const = 0;
};

class MmConfigEeKinematics : public EeKinematics {
public:
  explicit MmConfigEeKinematics(remani_planner::MMConfig* config);
  Eigen::Matrix4d pose(
      const Eigen::Vector3d& car,
      const Eigen::Matrix<double, 6, 1>& q) const override;
};

struct CompletionInput {
  Eigen::Vector2d expected_base_xy{Eigen::Vector2d::Zero()};
  double expected_base_yaw{0.0};
  Eigen::Matrix<double, 6, 1> expected_q{
      Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix4d expected_ee{Eigen::Matrix4d::Identity()};
  ActualStateSnapshot actual;
  bool arm_action_succeeded{false};
  bool feedback_fresh{false};
  bool robot_status_healthy{false};
};

struct CompletionDecision {
  bool succeeded{false};
  double final_base_position_error{0.0};
  double final_base_yaw_error{0.0};
  double final_joint_error{0.0};
  double final_ee_pos_error{0.0};
  double final_ee_rot_error{0.0};
  std::string error_code;
  std::string detail;
};

class CompletionVerifier {
public:
  CompletionVerifier(CompletionThresholds thresholds,
                     const EeKinematics* kinematics);
  CompletionDecision verify(const CompletionInput& input) const;
};
```

Compute EE rotation error as the angle of `R_expected.transpose()*R_actual`, clamping `(trace(R)-1)/2` to `[-1,1]` before `acos`. Base yaw uses normalized angle; joint error uses max absolute norm.

- [ ] **Step 3: Require all terminal predicates**

Verification runs only after candidate duration and CR10 Action SUCCEEDED. It still requires fresh odom/joints/TF, valid qd, connected+enabled+fault-free RobotStatus, base tolerances, base speed `<=0.01 m/s`, base yaw rate `<=0.02 rad/s`, joint tolerance, `max|qd|<=0.01`, and both EE tolerances. A failed Action maps to its Action/safety code, not terminal tolerance failure.

- [ ] **Step 4: Publish result without reviving planner execution**

On PASS publish `ExecutionResult::SUCCEEDED` and state `SUCCEEDED`. On a final actual-state tolerance failure publish `ExecutionResult::TERMINAL_TOLERANCE_FAILURE` and state `ERROR`. Populate all error fields; `ExecutionState.final_base_error` stores final base position error, while `ExecutionResult` keeps base position and yaw errors separately. Publish `/remani/execution_result` latched for audit. Do not send a planner FSM transition or call planner completion code.

- [ ] **Step 5: Run completion tests**

```bash
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_completion_verifier
```

Expected: exact pass, each base/joint/velocity/EE/freshness failure, and Action-success-only failure pass independently.

- [ ] **Step 6: Commit actual-state completion**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/completion_verifier.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/completion_verifier.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_completion_verifier.cpp \
        remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp \
        remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt \
        remani_planner/src/REMANI-Planner/remani_real/package.xml
git commit -m "feat: verify real whole body completion from feedback"
```

---

### Task 5: Compose the unified real launch and actual startup RobotModel

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/launch/remani_real.launch`
- Create: `remani_planner/src/REMANI-Planner/remani_real/rviz/remani_real.rviz`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/real_launch_contract.test`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_real_launch_contract.py`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/src/remani_state_bridge_node.cpp`
- Modify: `remani_planner/run_remani.sh`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: `mode:=real`, `dry_run`, `robotIp`, `port_name`, real `/odom`, raw CR10 joints, and RobotStatus.
- Produces: one launch containing hardware drivers when non-dry, planner PLAN-ONLY, State Bridge, Gate/Executor, RobotModel, Marker, Panel and RViz.

- [ ] **Step 1: Lock launch arguments and reject unsafe combinations**

The root launch declares:

```xml
<arg name="mode" default="real"/>
<arg name="execution_owner" default="external"/>
<arg name="dry_run" default="true"/>
<arg name="environment_mode" default="static_empty"/>
<arg name="robotIp" default="192.168.5.1"/>
<arg name="port_name" default="can0"/>
<arg name="start_hardware_drivers" default="true"/>
<arg name="start_rviz" default="true"/>
```

`remani_real_node` exits nonzero unless mode/owner/environment are exactly `real/external/static_empty`. It derives `hardware_output_enabled=!dry_run` internally; a launch argument cannot override that invariant.

- [ ] **Step 2: Apply hard source-level remaps in launch**

Planner node remaps:

```xml
<remap from="~odom_world" to="/odom"/>
<remap from="~joint_state" to="/remani/cr10_joint_states"/>
```

GridMap odom uses `/odom`. CR10 driver remaps its absolute raw output:

```xml
<remap from="/joint_states" to="/remani/cr10_joint_states_raw"/>
```

Ranger driver remaps its absolute input:

```xml
<remap from="/cmd_vel" to="/remani/ranger_cmd_vel_hw"/>
```

Do not create a relay from ordinary `/cmd_vel`. Only `RangerHardwareChannel` may advertise the hardware topic, and only in non-dry mode.

- [ ] **Step 3: Separate driver feedback from Executor write capability**

With the production default `start_hardware_drivers=true`, include Ranger and Dobot V4 drivers in both dry and non-dry launches so dry-run can inspect actual odom, joints, RobotStatus, Action-server presence, and watchdog health. Apply the same hard remaps in both cases. The Ranger driver may issue its own validated startup/watchdog stop at the SDK boundary, but there is no `/remani/ranger_cmd_vel_hw` publisher in dry-run; the CR10 driver receives no goal and no write service call.

For `dry_run=true`, construct only `DryRunMotionOutput` plus `Cr10ReadinessClient`; do not construct either hardware channel. For `dry_run=false`, set `DOBOT_TYPE=cr10`, set `remani_prestart_hold_mode=true`, and construct both hardware channels only after readiness. CI sets `start_hardware_drivers=false` and launches the exact fake feedback/Action nodes from the tests.

- [ ] **Step 4: Establish actual startup pose before enabling Plan**

Launch Ranger with `odom_frame=world`, `base_frame=base_link`, and `publish_odom_tf=false`. The existing driver initializes `position_x_`, `position_y_`, and `theta_` to zero on process start, so `/odom` is already the required startup-relative actual state. State Bridge consumes that same `/odom` without republishing or renormalizing it and becomes the only publisher of `world -> base_link`. Reject readiness with `START_ODOM_NOT_ZERO` if the first three finite samples exceed 1 mm or 0.1 degree; this detects an incompatible externally persisted odom source instead of silently creating a second frame convention.

State Bridge publishes actual CR10 q from the first complete name-mapped feedback to both fixed-order `/remani/cr10_joint_states` and RobotModel `/joint_states`. Display-only Ranger steering/wheel and gripper joints use configured static defaults and are marked `not measured`; they do not enter readiness, planning, tracking, Resume, or completion.

Keep deployment state `NOT_READY` and disable Plan/Execute/Resume until consecutive finite odom, complete CR10 q, valid qd, TF, RobotStatus, Action server, GridMap, and watchdog readiness meet configured counts/ages. The actual RobotModel may appear as soon as odom and q arrive; candidate preview remains on Marker/Path topics and never publishes actual-state topics or TF.

Retain the existing planner `/ee_current_pose` publication from actual odom+q and the existing `ee_goal_marker_node` behavior that creates its marker only after that first pose. The launch contract test must assert the first interactive-marker pose matches actual `MMConfig::getEePose` before Plan becomes enabled; do not initialize the marker from zeros or candidate preview.

- [ ] **Step 5: Configure RViz as an operator console**

`remani_real.rviz` contains RobotModel, EE interactive marker, candidate robot MarkerArray, base/EE paths, and the `remani_real_rviz/Panel`. The panel always displays:

```text
MODE: REAL / OWNER: EXTERNAL
ENVIRONMENT: STATIC EMPTY / NO ONLINE OBSTACLE SENSING
DRY RUN: ON|OFF
```

Plan publishes `/ee_goal_plan`; Execute sends the displayed `candidate_id`; Pause/Resume/Abort call their services. Button permissions come only from `/remani/execution_state`.

- [ ] **Step 6: Update `run_remani.sh` without changing its default**

Remove exactly one `mode:=...` assignment from the forwarded ROS arguments; default is `sim` when absent and reject duplicate mode assignments. Preserve every other ROS launch argument:

```bash
RUN_MODE="sim"
MODE_SEEN="false"
FORWARD_ARGS=()
for ARGUMENT in "$@"; do
  if [[ "${ARGUMENT}" == mode:=* ]]; then
    [[ "${MODE_SEEN}" == "false" ]] || {
      echo "mode may be specified once" >&2
      exit 2
    }
    RUN_MODE="${ARGUMENT#mode:=}"
    MODE_SEEN="true"
  else
    FORWARD_ARGS+=("${ARGUMENT}")
  fi
done

case "${RUN_MODE}" in
  sim)  LAUNCH_FILE="remani_sim.launch" ;;
  real) LAUNCH_FILE="remani_real.launch" ;;
  *) echo "unsupported mode: ${RUN_MODE}" >&2; exit 2 ;;
esac
```

For real only, reject `execution_owner:=internal` in `FORWARD_ARGS` and invoke `DOBOT_TYPE=cr10 roslaunch remani_real "${LAUNCH_FILE}" "${FORWARD_ARGS[@]}"`. For sim, invoke `roslaunch remani_planner "${LAUNCH_FILE}" "${FORWARD_ARGS[@]}"`. Continue tee logging with a mode-prefixed filename. Do not alter the existing `remani_sim.launch` contents.

- [ ] **Step 7: Test launch graph and startup initialization**

`real_launch_contract.test` runs with fake feedback and `dry_run=true`. Assert:

```python
def publishers(topic):
    published, _, _ = rosgraph.Master('/real_launch_contract').getSystemState()
    for name, nodes in published:
        if name == topic:
            return nodes
    return []

self.assertEqual(0, len(publishers('/remani/ranger_cmd_vel_hw')))
self.assertEqual(['cr10_joint1','cr10_joint2','cr10_joint3',
                  'cr10_joint4','cr10_joint5','cr10_joint6'],
                 planner_joint_state.name)
self.assertAlmostEqual(0.0, first_world_base_tf.transform.translation.x, 1e-6)
self.assertAlmostEqual(0.0, first_world_base_tf.transform.translation.y, 1e-6)
self.assertEqual(raw_positions_reordered, planner_joint_state.position)
self.assertEqual(ExecutionState.ENV_STATIC_EMPTY,
                 execution_state.environment_mode)
self.assertEqual(ee_current_pose.header.frame_id,
                 first_marker_pose.header.frame_id)
self.assertAlmostEqual(ee_current_pose.pose.position.x,
                       first_marker_pose.pose.position.x, 1e-6)
```

In the test, `publishers(topic)` is a five-line helper around `rosgraph.Master.getSystemState()` that returns the publisher list for an exact topic; define it in `test_real_launch_contract.py`. Use the planner node's resolved subscriptions from ROS master to assert `/odom` and `/remani/cr10_joint_states`, and use the State Bridge's resolved subscription to assert `/remani/cr10_joint_states_raw`. Phase 3's isolation rostest remains the proof that ordinary `/cmd_vel` cannot reach a non-dry Ranger driver.

- [ ] **Step 8: Run launch contract and shell checks**

```bash
bash -n remani_planner/run_remani.sh
rostest remani_real real_launch_contract.test
catkin_test_results remani_planner/build/remani_real/test_results
```

Expected: startup actual pose appears at world origin, q matches hardware feedback, Planner receives only six ordered joints, and dry-run has zero write-capable outputs.

- [ ] **Step 9: Commit the unified launch**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/launch/remani_real.launch \
        remani_planner/src/REMANI-Planner/remani_real/rviz/remani_real.rviz \
        remani_planner/src/REMANI-Planner/remani_real/test/real_launch_contract.test \
        remani_planner/src/REMANI-Planner/remani_real/test/test_real_launch_contract.py \
        remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp \
        remani_planner/src/REMANI-Planner/remani_real/src/remani_state_bridge_node.cpp \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt \
        remani_planner/run_remani.sh
git commit -m "feat: add unified REMANI real deployment launch"
```

---

### Task 6: Prove synchronized dry-run, Pause/Resume, completion, and sim regression

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/fake_whole_body_system.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/fake_candidate_planner.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/synchronized_dry_run.test`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_synchronized_dry_run.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/pause_resume_abort.test`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_pause_resume_abort.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/final_completion.test`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_final_completion.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/docs/synchronized_acceptance.md`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: the same public topics/services as the Panel and launch.
- Produces: deterministic evidence for no-output dry-run, synchronized start, strict Resume, actual-FK completion, and unchanged sim ownership.

- [ ] **Step 1: Build deterministic fakes with observable write counters**

`fake_whole_body_system.py` publishes raw odom, shuffled `joint1..joint6`, healthy RobotStatus, watchdog flags, and an Action server. It exposes read-only counters on `/test/write_counts` for Ranger commands, arm goals, cancel, Stop, and non-hold samples. In dry-run, all five counters must remain zero.

`fake_candidate_planner.py` listens to `/ee_goal` and publishes one fixed valid START/ADD/FINAL transaction plus `PlannerStatus`. Use it for control/execution determinism; the actual planner transaction and PLAN-ONLY semantics are already exercised by phase 1 rostests.

- [ ] **Step 2: Test the complete dry-run operator sequence**

Sequence:

```text
NOT_READY -> feedback ready -> READY
Panel-equivalent Plan -> PLANNING
START/ADD -> Execute still rejected
FINAL + Gate validation -> PLANNED
Execute(candidate_id) -> EXECUTING dry simulation
Pause -> PAUSED
Resume within tolerance -> EXECUTING with new T0 over suffix
candidate duration + actual terminal state -> SUCCEEDED
Abort -> READY and candidate invalid
```

Assert start lead `1.0±0.02 s`, `abs(start_skew)<=0.10 s`, no hardware topic publisher, and all write counters zero.

- [ ] **Step 3: Test rejection and fault paths**

Run separate cases for stale candidate ID, FINAL missing, new START during EXECUTING/PAUSED, Resume position/yaw/joint over tolerance, odom timeout, joint timeout, RobotStatus fault, Action abort, rogue Ranger publisher, watchdog timeout, tracking error, start skew, and terminal EE error. Each case asserts the exact `last_error_code`, zero/stop behavior, and required explicit Abort before replanning when applicable.

- [ ] **Step 4: Run all real software tests**

```bash
catkin_make -C remani_planner --pkg remani_real_msgs remani_planner plan_env remani_real remani_real_rviz
catkin_make -C remani_planner --pkg remani_real_msgs remani_planner plan_env remani_real remani_real_rviz --make-args run_tests
rostest remani_real synchronized_dry_run.test
rostest remani_real pause_resume_abort.test
rostest remani_real final_completion.test
catkin_test_results remani_planner/build
```

Expected: all tests pass and no dry-run write counter or hardware command publisher appears.

- [ ] **Step 5: Run the separate hardware package gates**

```bash
catkin_make -C agx --pkg ranger_base dobot_v4_bringup
catkin_make -C agx --pkg ranger_base dobot_v4_bringup --make-args run_tests
catkin_test_results agx/build
```

Expected: finite Ranger conversion/watchdog and strict non-blocking CR10 Action tests pass.

- [ ] **Step 6: Run the existing simulation regression**

```bash
rostest remani_planner remani_sim_owner.test
timeout 20s roslaunch remani_planner remani_sim.launch robot_model:=ranger_cr10 use_rviz:=false
```

Expected: default `mode=sim`, `execution_owner=internal`, current ADD-only simulator path, planner `EXEC_TRAJ`, and `mm_controller` auto-execution remain active. No real Gate/Executor or hardware command topic is required.

- [ ] **Step 7: Execute physical stages only after explicit authorization**

Follow `synchronized_acceptance.md` in this order:

```text
1 read-only dry_run feedback/TF/RobotModel/preview
2 Ranger isolation + watchdog
3 Ranger-only <=0.05 m/s, <=0.20 m
4 CR10 Action validation/cancel/Stop
5 CR10-only <=0.05 rad/s, <=0.10 rad/joint
6 shared-T0 synchronized dry-run
7 synchronized short real trajectory, |start_skew|<=0.10 s
8 early/mid/late Pause/Resume and over-tolerance rejection
9 terminal base/joint/EE checks and injected faults
10 gradually longer paths in cleared static-empty field
```

Each record captures candidate ID, config hash, requested T0, Action send/accept, both start timestamps, skew, stop latencies, max commands, tracking errors, final errors, physical E-stop operator, and pass/fail. A failed stage blocks the next.

- [ ] **Step 8: Commit integration tests and acceptance gate**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/test/fake_whole_body_system.py \
        remani_planner/src/REMANI-Planner/remani_real/test/fake_candidate_planner.py \
        remani_planner/src/REMANI-Planner/remani_real/test/synchronized_dry_run.test \
        remani_planner/src/REMANI-Planner/remani_real/test/test_synchronized_dry_run.py \
        remani_planner/src/REMANI-Planner/remani_real/test/pause_resume_abort.test \
        remani_planner/src/REMANI-Planner/remani_real/test/test_pause_resume_abort.py \
        remani_planner/src/REMANI-Planner/remani_real/test/final_completion.test \
        remani_planner/src/REMANI-Planner/remani_real/test/test_final_completion.py \
        remani_planner/src/REMANI-Planner/remani_real/docs/synchronized_acceptance.md \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "test: gate synchronized REMANI real execution"
```

## Final Release Gate

```text
SIM:
  mode=sim -> owner=internal -> existing EXEC_TRAJ/mm_controller behavior

REAL planning:
  actual odom/q -> planner PLAN-ONLY -> START/ADD/FINAL -> frozen candidate

REAL execution:
  explicit candidate-bound Execute -> one steady T0 -> Ranger + CR10
  Pause -> both actually stopped -> PAUSED
  Resume -> strict tolerance -> original suffix + new T0, no connector
  Abort -> both stopped + candidate invalid
  completion -> Action success + actual base/q/qd/EE/freshness/RobotStatus

SAFETY:
  dry_run -> no hardware publisher, Action goal, or write service
  ordinary /cmd_vel -> cannot reach Ranger driver
  runtime safety -> Real Executor only; planner execution timer inactive
  environment -> explicit static empty warning, no online obstacle claim
```
