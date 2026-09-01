# REMANI Ranger Real Execution Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Isolate the real Ranger command topic, add a monotonic command watchdog and safe zero handling at the driver boundary, then execute frozen REMANI base trajectories at conservative speed with actual odom feedback.

**Architecture:** The Ranger driver keeps its public ROS shape but moves Twist interpretation into a pure tested decision function and enforces a steady-clock watchdog in its main update loop. Real launch remaps the driver's absolute `/cmd_vel` subscription to `/remani/ranger_cmd_vel_hw`; a `RangerHardwareChannel` in `remani_real` is the only legitimate publisher and is created only when `dry_run=false`. A tested nonholonomic tracking controller converts frozen base samples plus `/odom` error into bounded linear/angular commands.

**Tech Stack:** ROS1 Noetic, catkin, C++11 in `ranger_base`, C++14 in `remani_real`, AgileX `ugv_sdk`, Eigen, ROS master API, gtest/rostest.

**Spec:** `docs/superpowers/specs/2026-09-01-remani-real-robot-deployment-design.md`

## Global Constraints

- Prerequisites: planner PLAN-ONLY and control-plane plans are complete and passing.
- `/cmd_vel` must not connect to hardware in real launch; only `/remani/ranger_cmd_vel_hw` reaches the Ranger driver.
- Real Executor is the sole legal hardware-topic publisher; a second publisher causes ERROR and a commanded stop.
- `cmd_vel_timeout` defaults to 0.20 s and uses monotonic/steady time.
- All-zero/near-zero Twist takes an explicit stop path; no `0/0`, NaN radius, or accidental spinning mode.
- V1 base limits are 0.10 m/s linear and 0.15 rad/s angular.
- `dry_run=true` creates no Ranger hardware publisher, including no zero-command publisher.
- A software watchdog does not replace Ranger firmware protection or the physical E-stop.
- No synchronized Ranger+CR10 movement is permitted in this phase.

---

## File Structure

| Path | Responsibility |
|---|---|
| `agx/ranger_ros/ranger_base/include/ranger_base/ranger_command.hpp` | Pure Twist→Ranger command decision and finite checks |
| `agx/ranger_ros/ranger_base/include/ranger_base/command_watchdog.hpp` | Steady-clock timeout state |
| `agx/ranger_ros/ranger_base/src/ranger_messenger.cpp` | Hardware application, explicit stop, watchdog polling |
| `agx/ranger_ros/ranger_base/test/` | Zero/straight/turn/timeout/destructor tests |
| `remani_real/include/remani_real/base_tracking_controller.hpp` | Frozen sample + odom → bounded Twist |
| `remani_real/include/remani_real/topic_ownership_monitor.hpp` | ROS-master unique-publisher check |
| `remani_real/include/remani_real/ranger_hardware_channel.hpp` | Non-dry hardware topic adapter |
| `remani_real/launch/ranger_only_low_speed.launch` | Explicitly staged single-device validation |

---

### Task 1: Extract and test finite Ranger command decisions

**Files:**
- Create: `agx/ranger_ros/ranger_base/include/ranger_base/ranger_command.hpp`
- Create: `agx/ranger_ros/ranger_base/src/ranger_command.cpp`
- Create: `agx/ranger_ros/ranger_base/test/test_ranger_command.cpp`
- Modify: `agx/ranger_ros/ranger_base/CMakeLists.txt`

**Interfaces:**
- Consumes: `geometry_msgs::Twist`, a public value-only `RangerCommandLimits`, and zero epsilon. `RangerROSMessenger` copies its private `RobotParams` fields into this type at the callback boundary.
- Produces: a finite `RangerCommandDecision` without touching `RangerRobot`.

- [ ] **Step 1: Write failing zero and straight-line tests**

```cpp
static RangerCommandLimits rangerLimits() {
  RangerCommandLimits limits;
  limits.track = RangerParams::track;
  limits.wheelbase = RangerParams::wheelbase;
  limits.max_linear_speed = RangerParams::max_linear_speed;
  limits.max_angular_speed = RangerParams::max_angular_speed;
  limits.max_speed_cmd = RangerParams::max_speed_cmd;
  limits.max_steer_angle_central = RangerParams::max_steer_angle_central;
  limits.max_steer_angle_parallel = RangerParams::max_steer_angle_parallel;
  limits.max_round_angle = RangerParams::max_round_angle;
  limits.min_turn_radius = RangerParams::min_turn_radius;
  return limits;
}

TEST(RangerCommand, ZeroTwistIsExplicitStop) {
  geometry_msgs::Twist msg;
  const auto out = ComputeRangerCommand(msg, rangerLimits(), 1e-4);
  ASSERT_TRUE(out.valid);
  EXPECT_TRUE(out.stop);
  EXPECT_DOUBLE_EQ(0.0, out.linear);
  EXPECT_DOUBLE_EQ(0.0, out.steering);
  EXPECT_DOUBLE_EQ(0.0, out.angular);
}

TEST(RangerCommand, StraightTwistNeverDividesByZero) {
  geometry_msgs::Twist msg;
  msg.linear.x = 0.05;
  const auto out = ComputeRangerCommand(msg, rangerLimits(), 1e-4);
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN, out.motion_mode);
  EXPECT_DOUBLE_EQ(0.0, out.steering);
  EXPECT_TRUE(std::isfinite(out.turn_radius));
}

TEST(RangerCommand, RejectsAnyNonFiniteInput) {
  geometry_msgs::Twist msg;
  msg.angular.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ComputeRangerCommand(msg, rangerLimits(), 1e-4).valid);
}
```

- [ ] **Step 2: Run and confirm the missing implementation failure**

```bash
catkin_make -C agx --pkg ranger_base --make-args run_tests_ranger_base_gtest_test_ranger_command
```

- [ ] **Step 3: Implement the decision structure and explicit branches**

```cpp
struct RangerCommandLimits {
  double track{0.0};
  double wheelbase{0.0};
  double max_linear_speed{0.0};
  double max_angular_speed{0.0};
  double max_speed_cmd{0.0};
  double max_steer_angle_central{0.0};
  double max_steer_angle_parallel{0.0};
  double max_round_angle{0.0};
  double min_turn_radius{0.0};
};

struct RangerCommandDecision {
  bool valid{false};
  bool stop{false};
  uint8_t motion_mode{ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN};
  double linear{0.0};
  double steering{0.0};
  double angular{0.0};
  double turn_radius{0.0};
};
```

Branch order:

```text
non-finite -> invalid
|vx|,|vy|,|wz| <= epsilon -> stop
|vy| > epsilon -> existing parallel/side-slip logic
|wz| <= epsilon -> dual Ackermann, steering=0, diagnostic radius=0
otherwise -> finite radius/steering and existing Ackermann/spin selection
```

Do not perform `linear/angular` or `linear_y/linear_x` before the corresponding denominator branch.

- [ ] **Step 4: Link the pure library and pass tests**

```cmake
add_library(ranger_command src/ranger_command.cpp)
target_link_libraries(ranger_command ${catkin_LIBRARIES})
catkin_add_gtest(test_ranger_command test/test_ranger_command.cpp)
target_link_libraries(test_ranger_command ranger_command ${catkin_LIBRARIES})
```

Run the named test again. Expected: zero, straight, curved, spin, lateral, clamp, and non-finite cases pass.

- [ ] **Step 5: Commit finite command conversion**

```bash
git add agx/ranger_ros/ranger_base/include/ranger_base/ranger_command.hpp \
        agx/ranger_ros/ranger_base/src/ranger_command.cpp \
        agx/ranger_ros/ranger_base/test/test_ranger_command.cpp \
        agx/ranger_ros/ranger_base/CMakeLists.txt
git commit -m "fix: make Ranger Twist conversion finite"
```

---

### Task 2: Add a driver-boundary watchdog and stop lifecycle

**Files:**
- Create: `agx/ranger_ros/ranger_base/include/ranger_base/command_watchdog.hpp`
- Create: `agx/ranger_ros/ranger_base/src/command_watchdog.cpp`
- Create: `agx/ranger_ros/ranger_base/test/test_command_watchdog.cpp`
- Modify: `agx/ranger_ros/ranger_base/include/ranger_base/ranger_messenger.hpp:38-103`
- Modify: `agx/ranger_ros/ranger_base/src/ranger_messenger.cpp:38-153,377-453`
- Modify: `agx/ranger_ros/ranger_base/src/ranger_base_node.cpp:24-70`
- Modify: `agx/ranger_ros/ranger_base/launch/include/ranger_robot_base.launch`
- Modify: `agx/ranger_ros/ranger_base/CMakeLists.txt`

**Interfaces:**
- Consumes: accepted finite command timestamps and private `cmd_vel_timeout` (default 0.20 s).
- Produces: explicit hardware stop, `/remani/ranger_watchdog_ready`, and `/remani/ranger_watchdog_timed_out`.

- [ ] **Step 1: Write a failing steady-clock watchdog test**

```cpp
TEST(CommandWatchdog, TimesOutOnceUntilNewCommand) {
  CommandWatchdog watchdog(0.20);
  watchdog.arm(1.0);
  EXPECT_FALSE(watchdog.update(1.19).request_stop);
  EXPECT_TRUE(watchdog.update(1.21).request_stop);
  EXPECT_FALSE(watchdog.update(1.22).request_stop);
  watchdog.noteCommand(1.23);
  EXPECT_FALSE(watchdog.timedOut());
}
```

- [ ] **Step 2: Implement watchdog state without ROS wall time**

```cpp
struct WatchdogUpdate {
  bool request_stop{false};
  bool timed_out{false};
};

class CommandWatchdog {
public:
  explicit CommandWatchdog(double timeout_sec);
  void arm(double steady_now_sec);
  void noteCommand(double steady_now_sec);
  WatchdogUpdate update(double steady_now_sec);
  bool timedOut() const;
};
```

Use `ros::SteadyTime::now().toSec()` only at the Messenger integration boundary.

- [ ] **Step 3: Centralize hardware stop in `RangerROSMessenger`**

Add:

```cpp
~RangerROSMessenger();
void StopRobot(const char* reason);
void EnforceCommandWatchdog();
RangerCommandLimits CurrentCommandLimits() const;
```

`StopRobot()` always calls a finite SDK stop command:

```cpp
robot_->SetMotionMode(ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN);
robot_->SetMotionCommand(0.0, 0.0);
```

Call it on startup after commanded mode is enabled, on invalid Twist, on the first timeout edge, on communication exception, on ROS shutdown, and from the destructor. Replace the signal handler's direct `exit()` with `ros::shutdown()` so destructor cleanup runs.

- [ ] **Step 4: Apply pure command decisions in `TwistCmdCallback`**

The callback becomes:

```cpp
const auto decision = ComputeRangerCommand(
    *msg, CurrentCommandLimits(), command_zero_epsilon_);
if (!decision.valid) {
  StopRobot("invalid cmd_vel");
  return;
}
watchdog_.noteCommand(ros::SteadyTime::now().toSec());
ApplyRangerCommand(decision);
```

`Run()` invokes `EnforceCommandWatchdog()` on every update loop, independent of incoming callbacks.

- [ ] **Step 5: Publish watchdog status and expose launch parameters**

Add private params:

```xml
<arg name="cmd_vel_timeout" default="0.20"/>
<arg name="command_zero_epsilon" default="0.0001"/>
<param name="cmd_vel_timeout" value="$(arg cmd_vel_timeout)"/>
<param name="command_zero_epsilon" value="$(arg command_zero_epsilon)"/>
```

Publish `std_msgs/Bool` latched ready after commanded mode/startup stop, and `std_msgs/Bool` timed-out state on every edge.

- [ ] **Step 6: Run driver tests**

```bash
catkin_make -C agx --pkg ranger_base
catkin_make -C agx --pkg ranger_base --make-args run_tests_ranger_base
catkin_test_results agx/build/ranger_base/test_results
```

- [ ] **Step 7: Commit watchdog behavior**

```bash
git add agx/ranger_ros/ranger_base/include/ranger_base/command_watchdog.hpp \
        agx/ranger_ros/ranger_base/src/command_watchdog.cpp \
        agx/ranger_ros/ranger_base/test/test_command_watchdog.cpp \
        agx/ranger_ros/ranger_base/include/ranger_base/ranger_messenger.hpp \
        agx/ranger_ros/ranger_base/src/ranger_messenger.cpp \
        agx/ranger_ros/ranger_base/src/ranger_base_node.cpp \
        agx/ranger_ros/ranger_base/launch/include/ranger_robot_base.launch \
        agx/ranger_ros/ranger_base/CMakeLists.txt
git commit -m "feat: stop Ranger on command timeout"
```

---

### Task 3: Isolate topic ownership and detect a second hardware publisher

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/topic_ownership_monitor.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/topic_ownership_monitor.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/ranger_hardware_channel.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/ranger_hardware_channel.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_topic_ownership.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/ranger_topic_isolation.test`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_ranger_topic_isolation.py`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: ROS master system state for `/remani/ranger_cmd_vel_hw`.
- Produces: `RangerHardwareChannel::publish(const Twist&)` and a unique-owner health result.

- [ ] **Step 1: Write the ownership decision test**

```cpp
TEST(TopicOwnership, RequiresExactlyTheExpectedPublisher) {
  EXPECT_TRUE(IsUniqueOwner({"/remani_real_node"}, "/remani_real_node"));
  EXPECT_FALSE(IsUniqueOwner({}, "/remani_real_node"));
  EXPECT_FALSE(IsUniqueOwner(
      {"/remani_real_node", "/rogue_teleop"}, "/remani_real_node"));
}
```

- [ ] **Step 2: Implement hardware-channel construction guards**

```cpp
class RangerHardwareChannel : public RangerCommandChannel {
public:
  RangerHardwareChannel(ros::NodeHandle& nh,
                        const std::string& hardware_topic);
  bool publish(const geometry_msgs::Twist& command) override;
  bool hardwareOutputEnabled() const override { return true; }
  bool ownershipHealthy() const;
private:
  ros::Publisher publisher_;
  TopicOwnershipMonitor ownership_monitor_;
};
```

The composition layer constructs this class only when `dry_run=false` and passes `/remani/ranger_cmd_vel_hw`; the constructor rejects any other topic. The dry-run code path continues to use `DryRunMotionOutput` and never instantiates this publisher.

- [ ] **Step 3: Add the launch-level remap**

In the Ranger node include used by real launch:

```xml
<remap from="/cmd_vel" to="/remani/ranger_cmd_vel_hw"/>
```

Do not add a relay from ordinary `/cmd_vel`.

- [ ] **Step 4: Prove ordinary `/cmd_vel` is disconnected**

The rostest launches a fake Ranger subscriber under the same remap, publishes 10 messages on `/cmd_vel`, and asserts hardware-received count remains zero. It then publishes through `RangerHardwareChannel` and asserts one message arrives. Starting `/rogue_teleop` on the hardware topic must make `ownershipHealthy()==false` and the deployment state enter ERROR.

- [ ] **Step 5: Run and commit topic isolation**

```bash
rostest remani_real ranger_topic_isolation.test
```

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/topic_ownership_monitor.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/topic_ownership_monitor.cpp \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/ranger_hardware_channel.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/ranger_hardware_channel.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_topic_ownership.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/ranger_topic_isolation.test \
        remani_planner/src/REMANI-Planner/remani_real/test/test_ranger_topic_isolation.py \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "feat: isolate Ranger hardware command ownership"
```

---

### Task 4: Track frozen base trajectories with actual odom

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/base_tracking_controller.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/base_tracking_controller.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_base_tracking_controller.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: one `WholeBodySample`, one `ActualStateSnapshot`, and configured gains/limits.
- Produces: finite bounded `geometry_msgs::Twist` plus tracking metrics or an explicit safety failure.

- [ ] **Step 1: Write failing straight/reverse/error tests**

```cpp
static BaseTrackingConfig trackingConfig() {
  BaseTrackingConfig config;
  config.k_x = 1.0;
  config.k_y = 1.0;
  config.k_yaw = 1.0;
  config.max_linear = 0.10;
  config.max_angular = 0.15;
  config.max_linear_correction = 0.03;
  config.max_angular_correction = 0.05;
  config.max_position_error = 0.20;
  config.max_yaw_error = 0.20;
  return config;
}

static WholeBodySample desiredSample(double speed, int singul) {
  WholeBodySample desired;
  desired.position = Eigen::VectorXd::Zero(8);
  desired.velocity = Eigen::VectorXd::Zero(8);
  desired.acceleration = Eigen::VectorXd::Zero(8);
  desired.velocity.x() = std::abs(speed);
  desired.base_yaw = 0.0;
  desired.base_angular_velocity = 0.0;
  desired.singul = singul;
  return desired;
}

static ActualStateSnapshot matchingActual() {
  ActualStateSnapshot actual;
  actual.base_xy.setZero();
  actual.base_yaw = 0.0;
  actual.odom_valid = true;
  return actual;
}

TEST(BaseTrackingController, FollowsStraightCandidateAtNominalSpeed) {
  BaseTrackingController controller(trackingConfig());
  const auto result = controller.compute(desiredSample(0.05, 1), matchingActual());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(0.05, result.command.linear.x, 1e-6);
  EXPECT_NEAR(0.0, result.command.angular.z, 1e-6);
}

TEST(BaseTrackingController, PreservesReverseSingularity) {
  BaseTrackingController controller(trackingConfig());
  const auto result = controller.compute(desiredSample(0.05, -1), matchingActual());
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.command.linear.x, 0.0);
}

TEST(BaseTrackingController, ExcessTrackingErrorFailsInsteadOfTimeScaling) {
  auto actual = matchingActual();
  actual.base_xy.x() += 0.25;
  BaseTrackingController controller(trackingConfig());
  const auto result = controller.compute(desiredSample(0.05, 1), actual);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ("BASE_TRACKING_ERROR", result.error_code);
}
```

- [ ] **Step 2: Implement a nonholonomic feedback law**

Define the configuration/result boundary exactly:

```cpp
struct BaseTrackingConfig {
  double k_x{0.0};
  double k_y{0.0};
  double k_yaw{0.0};
  double max_linear{0.10};
  double max_angular{0.15};
  double max_linear_correction{0.03};
  double max_angular_correction{0.05};
  double max_position_error{0.20};
  double max_yaw_error{0.20};
};

struct BaseTrackingResult {
  bool valid{false};
  geometry_msgs::Twist command;
  double position_error{0.0};
  double yaw_error{0.0};
  std::string error_code;
};
```

Use desired-frame errors and bounded correction:

```cpp
e_xy = R(yaw_desired).transpose() * (desired_xy - actual_xy);
e_yaw = normalizeAngle(yaw_desired - actual_yaw);
v_ff = singul * desired.velocity.head<2>().norm();
w_ff = desired.base_angular_velocity;
v_cmd = v_ff + k_x * e_xy.x();
w_cmd = w_ff + k_y * e_xy.y() + k_yaw * e_yaw;
```

Reject non-finite values, nominal trajectory limit violations, and tracking errors over configured thresholds. Allow only a configured correction budget; clamp corrections inside that budget, then enforce absolute 0.10/0.15 limits. Never alter arm timing or candidate sampling time to compensate.

- [ ] **Step 3: Wire runtime monitoring**

At every Executor tick verify odom age, actual speed, computed command limits, watchdog health, and topic ownership before publishing. On any failure, publish zero until the driver confirms stop and transition to ERROR.

- [ ] **Step 4: Run the base controller suite**

```bash
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_base_tracking_controller
```

- [ ] **Step 5: Commit base execution logic**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/base_tracking_controller.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/base_tracking_controller.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_base_tracking_controller.cpp \
        remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml \
        remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "feat: track REMANI base trajectories on Ranger"
```

---

### Task 5: Gate Ranger-only low-speed validation before CR10 work

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/launch/ranger_only_low_speed.launch`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/fake_ranger_driver_node.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/ranger_only_execution.test`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_ranger_only_execution.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/docs/ranger_only_acceptance.md`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: a frozen candidate with constant arm joints and total base travel ≤0.20 m.
- Produces: an explicitly staged Ranger-only evidence gate; it is not the production whole-body launch.

- [ ] **Step 1: Create a locked staged-test launch**

Require all three explicit arguments:

```xml
<arg name="enable_staged_test" default="false"/>
<arg name="dry_run" default="true"/>
<arg name="max_test_duration" default="3.0"/>
```

The node refuses `dry_run=false` unless `enable_staged_test=true`, candidate arm velocity is identically zero, base speed limit is 0.05 m/s, angular limit is 0.10 rad/s, and candidate displacement is ≤0.20 m.

- [ ] **Step 2: Test watchdog and Abort against a fake driver**

The fake driver integrates commanded Twist into odom and records arrival times. Test sequence:

```text
Execute -> commands follow path
stop publishing Executor ticks -> watchdog zero within 0.20 s
Execute again -> Abort -> zero command and READY
publish rogue hardware command -> ERROR and zero
```

Assertions use measured messages, not sleeps alone:

```python
self.assertLessEqual(watchdog_stop_latency, 0.25)
self.assertLessEqual(max_abs_linear_command, 0.05)
self.assertEqual(0, command_after_abort_nonzero_count)
```

- [ ] **Step 3: Run software validation before any CAN access**

```bash
catkin_make -C agx --pkg ranger_base
catkin_make -C remani_planner --pkg remani_real
rostest remani_real ranger_only_execution.test
catkin_test_results remani_planner/build/remani_real/test_results
```

- [ ] **Step 4: Execute the physical checklist only with authorization**

The acceptance document requires: clear field, lifted-wheel/stand test first where mechanically safe, physical E-stop operator, CAN interface verified, ordinary `/cmd_vel` disconnection checked, 0.05 m/s command, ≤0.20 m path, Pause/Abort/watchdog measured, and logs attached. Failure of any stop gate blocks CR10 integration.

- [ ] **Step 5: Commit Ranger staged validation assets**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/launch/ranger_only_low_speed.launch \
        remani_planner/src/REMANI-Planner/remani_real/test/fake_ranger_driver_node.py \
        remani_planner/src/REMANI-Planner/remani_real/test/ranger_only_execution.test \
        remani_planner/src/REMANI-Planner/remani_real/test/test_ranger_only_execution.py \
        remani_planner/src/REMANI-Planner/remani_real/docs/ranger_only_acceptance.md \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "test: gate Ranger only low speed execution"
```

## Phase Exit Gate

```text
ordinary /cmd_vel -> no Ranger hardware subscriber
/remani/ranger_cmd_vel_hw -> exactly one expected publisher
zero/straight Twist -> finite SDK command
missing command >0.20 s -> explicit stop
Ranger-only path -> <=0.05 m/s, actual odom feedback, Pause/Abort/timeout measured
CR10 -> not moving and not commanded in this phase
```
