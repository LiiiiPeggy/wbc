# REMANI Real Control Plane Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the real-mode State Bridge, transaction Gate, frozen candidate/preview pipeline, unified deployment state machine, zero-output dry-run executor, and RViz confirmation Panel.

**Architecture:** A dedicated `remani_real` package contains testable core classes and two ROS nodes: `remani_state_bridge_node` owns actual joint/TF normalization, while `remani_real_node` composes Gate, readiness, deployment state, preview, and Executor. A separate focused `remani_real_rviz` package displays only the unified `ExecutionState` and sends explicit Plan/Execute/Pause/Resume/Abort commands. Ranger and arm writes are separated behind `RangerCommandChannel` and `ArmCommandChannel`; this phase instantiates one `DryRunMotionOutput` implementing both interfaces without advertising or calling a motion-changing hardware endpoint.

**Tech Stack:** ROS1 Noetic, catkin, C++14, Eigen, actionlib read-only readiness checks, TF2, RViz/Qt5, gtest/rostest, `quadrotor_msgs`, `mm_config`, `plan_env`, and `remani_real_msgs`.

**Spec:** `docs/superpowers/specs/2026-09-01-remani-real-robot-deployment-design.md`

## Global Constraints

- Prerequisite: complete `2026-09-01-remani-real-planner-plan-only.md`; do not reintroduce planner execution ownership.
- Cross-host topology: the laptop runs only the REMANI control plane (planner, Gate, State Bridge, dry-run Executor, RViz Panel). `agx/` runs on a remote host as a black-box ROS endpoint. Do not modify, compile, source, or start any `agx/` package from this laptop. Do not depend on local `agx/build` or `agx/devel` artifacts.
- Laptop real/control-plane launch must not start Ranger, CR10, or other hardware driver nodes. Remote runtime remaps and driver config are a remote deployment contract, not laptop launch content.
- Canonical Ranger hardware command topic is `/remani/hardware/ranger/cmd_vel`. `dry_run:=true` must not advertise or publish that topic. Dry-run diagnostics use only `/remani/dry_run/ranger_cmd_vel_preview`.
- Planner raw transaction topic is `/remani/planner_candidate`. Do not use the obsolete `/remani/candidate_trajectory` name.
- Planner state name is `HANDOFF` (not `HANDED_OFF`). Gate acknowledgements must preserve and return `raw_transaction_stamp`.
- `/joint_states` is RobotModel display data; `/remani/cr10_joint_states` is exactly six CR10 joints in fixed order.
- Raw driver input is `/remani/cr10_joint_states_raw` as standard `sensor_msgs/JointState`; raw array order is never trusted. `/odom` is standard `nav_msgs/Odometry`.
- Formal CR10 fault/readiness uses project-owned `/remani/cr10_status`. Do not introduce an AGX-generated `RobotStatus` compile dependency. Until a formal status producer is deployed, remain `NOT_READY`; never infer healthy from connected/enabled or topic presence alone.
- Gate owns monotonic `uint64 candidate_id`; `PolynomialTraj.trajectory_id` remains transaction-local segment order.
- Execute is disabled until a complete, validated START/ADD/FINAL transaction is frozen.
- New START may replace READY/PLANNED candidates, but must not affect EXECUTING/PAUSED candidates.
- Actual RobotModel and candidate preview never publish to each other's `/joint_states`, `/odom`, or `world -> base_link` channels.
- `dry_run:=true` must not advertise `/remani/hardware/ranger/cmd_vel`, send a FollowJointTrajectory goal, or call ServoJ/Stop/Pause/Continue/EmergencyStop/Enable/Disable.
- Panel displays the unified state; it does not infer safety from local timers, publisher counts, or topic presence.
- Environment warning text is exactly `ENVIRONMENT: STATIC EMPTY / NO ONLINE OBSTACLE SENSING`.
- New/changed C++ blocks use one repository-style comment banner before the block.
- Phase 2 may only complete the zero-output dry-run control plane. Formal non-dry hardware output remains blocked until remote Ranger watchdog/stop, CR10 safety Action proxy, reliable `/remani/cr10_status`, two-host ROS/time sync, and shared-T0 observability are separately designed, implemented, and verified.

Build only the laptop control-plane packages under `remani_planner`. Do not run `catkin_make -C agx` or `source agx/devel/setup.bash`.

---

## File Structure

| Path | Responsibility |
|---|---|
| `remani_real/include/remani_real/joint_state_mapper.hpp` | Raw-name validation and fixed q1..q6 mapping |
| `remani_real/include/remani_real/velocity_estimator.hpp` | Timestamped bounded finite-difference qd estimate |
| `remani_real/include/remani_real/actual_state.hpp` | Immutable state/readiness snapshots shared by Gate/Executor |
| `remani_real/src/remani_state_bridge_node.cpp` | `/odom`, raw joints → TF and two JointState outputs |
| `remani_real/include/remani_real/candidate_trajectory.hpp` | Immutable segment model and whole-body sampling |
| `remani_real/include/remani_real/candidate_assembler.hpp` | START/ADD/FINAL/ABORT/IMPOSSIBLE protocol state |
| `remani_real/include/remani_real/candidate_validator.hpp` | Numeric, continuity, limit, collision, and boundary validation |
| `remani_real/include/remani_real/deployment_state_machine.hpp` | Pure external state transition table and button permissions |
| `remani_real/include/remani_real/motion_output.hpp` | Separate Ranger/arm output boundaries implemented by dry-run and later hardware adapters |
| `remani_real/src/remani_real_node.cpp` | ROS composition, services, unified state publication |
| `remani_real/src/preview_publisher.cpp` | Isolated MarkerArray/base Path/EE Path previews |
| `remani_real/config/remani_real.yaml` | Timeouts, limits, Gate sampling, display defaults |
| `remani_real/launch/remani_real_control_plane.launch` | Planner + State Bridge + Gate + dry-run Executor, no hardware |
| `remani_real_rviz/src/remani_real_panel.*` | RViz Panel widget and ROS I/O |
| `remani_real_rviz/include/remani_real_rviz/panel_view_model.hpp` | Pure button/label mapping for unit tests |

---

### Task 1: Scaffold focused core packages and immutable shared types

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`
- Create: `remani_planner/src/REMANI-Planner/remani_real/package.xml`
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/actual_state.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/candidate_trajectory.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/candidate_trajectory.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_candidate_sampling.cpp`

**Interfaces:**
- Consumes: existing polynomial matrix layout from `quadrotor_msgs/PolynomialTraj`.
- Produces the exact cross-task C++ types below; later tasks must not rename fields or substitute a second candidate model.

- [ ] **Step 1: Define actual-state snapshots**

```cpp
struct ActualStateSnapshot {
  ros::SteadyTime captured_at;
  ros::Time ros_stamp;
  Eigen::Vector2d base_xy{Eigen::Vector2d::Zero()};
  double base_yaw{0.0};
  Eigen::Vector2d base_velocity_world{Eigen::Vector2d::Zero()};
  double base_yaw_rate{0.0};
  Eigen::Matrix<double, 6, 1> q{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> qd{Eigen::Matrix<double, 6, 1>::Zero()};
  bool odom_valid{false};
  bool joints_valid{false};
  bool velocity_valid{false};
  bool tf_valid{false};
  bool robot_connected{false};
  bool robot_enabled{false};
  bool robot_fault{false};
};
```

- [ ] **Step 2: Write the failing candidate sampling test with its local fixture**

```cpp
static CandidateSegment constantVelocitySegment(uint32_t id, int singul,
                                                double start_time,
                                                double duration,
                                                double vx) {
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  coeff.col(7) << start_time * vx, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  coeff(0, 6) = vx;
  MMController::Trajectory trajectory;
  trajectory.emplace_back(duration, coeff);
  CandidateSegment segment;
  segment.trajectory_id = id;
  segment.singul = singul;
  segment.trajectory = trajectory;
  segment.start_time = start_time;
  segment.duration = duration;
  return segment;
}

TEST(CandidateTrajectory, SamplesAcrossSegmentBoundary) {
  CandidateSegment first = constantVelocitySegment(1, 1, 0.0, 1.0, 0.1);
  CandidateSegment second = constantVelocitySegment(2, -1, 1.0, 2.0, 0.1);
  CandidateTrajectory candidate(42, {first, second}, 0.0);

  EXPECT_EQ(42u, candidate.id());
  EXPECT_DOUBLE_EQ(3.0, candidate.duration());
  EXPECT_EQ(1, candidate.sample(0.5).singul);
  EXPECT_EQ(-1, candidate.sample(1.5).singul);
  EXPECT_EQ(8, candidate.sample(1.5).position.size());
  EXPECT_THROW(candidate.sample(-0.01), std::out_of_range);
  EXPECT_THROW(candidate.sample(3.01), std::out_of_range);
}
```

- [ ] **Step 3: Implement the immutable candidate contract**

```cpp
struct WholeBodySample {
  Eigen::VectorXd position;      // [x, y, q1..q6]
  Eigen::VectorXd velocity;      // same order
  Eigen::VectorXd acceleration;  // same order
  double base_yaw{0.0};
  double base_angular_velocity{0.0};
  int singul{0};
};

struct CandidateSegment {
  uint32_t trajectory_id{0};
  int singul{0};
  MMController::Trajectory trajectory;
  double start_time{0.0};
  double duration{0.0};
};

class CandidateTrajectory {
public:
  CandidateTrajectory(uint64_t candidate_id,
                      std::vector<CandidateSegment> segments,
                      double start_yaw);
  uint64_t id() const;
  double duration() const;
  const std::vector<CandidateSegment>& segments() const;
  WholeBodySample sample(double t) const;
};

using FrozenCandidate = std::shared_ptr<const CandidateTrajectory>;
```

Use the previous valid yaw when base speed is below `1e-6`; otherwise compute `atan2(singul*vy, singul*vx)`. Compute angular velocity as `(vx*ay-vy*ax)/(vx²+vy²)` when speed is valid, else zero.

- [ ] **Step 4: Add package dependencies and test target**

`remani_real` depends on `roscpp`, `std_msgs`, `std_srvs`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `trajectory_msgs`, `visualization_msgs`, `tf2_ros`, `actionlib`, `control_msgs`, `quadrotor_msgs`, `remani_real_msgs`, `mm_controller`, `mm_config`, and `plan_env`; declare `robot_state_publisher` as an exec dependency. Do not add a compile dependency on `dobot_v4_bringup` or any other `agx/` package.

Run:

```bash
catkin_make -C remani_planner --pkg remani_real
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_candidate_sampling
```

Expected: build and sampling test pass.

- [ ] **Step 5: Commit the core type boundary**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt \
        remani_planner/src/REMANI-Planner/remani_real/package.xml \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/actual_state.hpp \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/candidate_trajectory.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/candidate_trajectory.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_candidate_sampling.cpp
git commit -m "feat: add immutable real candidate model"
```

---

### Task 2: Implement State Bridge mapping, velocity validity, and actual TF

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/joint_state_mapper.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/joint_state_mapper.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/velocity_estimator.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/velocity_estimator.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/remani_state_bridge_node.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_joint_state_mapper.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/state_bridge.test`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_state_bridge.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml`
- Create: project-owned `/remani/cr10_status` message/types inside `remani_planner` (not AGX)
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

Do **not** modify remote AGX sources, including:
- `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/msg/RobotStatus.msg`
- `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/include/dobot_v4_bringup/cr5_v4_robot.h`
- `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/cr5_v4_robot.cpp`
- `agx/TCP-IP-ROS-6AXis/dobot_v4_bringup/src/main.cpp`

**Interfaces:**
- Consumes: `/remani/cr10_joint_states_raw` (`sensor_msgs/JointState`), `/odom` (`nav_msgs/Odometry`), optional display joint inputs, and fake or remote `/remani/cr10_status`.
- Produces: `/remani/cr10_joint_states`, `/joint_states`, and the only `world -> base_link` dynamic TF in the laptop real/control-plane launch.
- State Bridge is implemented only inside `remani_planner`. Tests use fake normalized status. If formal `/remani/cr10_status` is absent or stale, readiness stays fail-closed `NOT_READY`.

- [ ] **Step 1: Write failing name-order and invalid-input tests**

```cpp
static sensor_msgs::JointState validRawJointState() {
  sensor_msgs::JointState raw;
  raw.header.stamp = ros::Time(1.0);
  raw.name = {"joint3", "joint1", "joint6", "joint2", "joint5", "joint4"};
  raw.position = {3, 1, 6, 2, 5, 4};
  return raw;
}

TEST(JointStateMapper, ReordersRawNamesToCr10ModelOrder) {
  const sensor_msgs::JointState raw = validRawJointState();
  const auto mapped = JointStateMapper::map(raw);
  ASSERT_TRUE(mapped.valid);
  EXPECT_EQ((std::array<double, 6>{1, 2, 3, 4, 5, 6}), mapped.position);
  EXPECT_EQ("cr10_joint1", mapped.planning_msg.name[0]);
  EXPECT_EQ("cr10_joint6", mapped.planning_msg.name[5]);
}

TEST(JointStateMapper, RejectsMissingDuplicateAndNonFiniteNames) {
  auto missing = validRawJointState();
  missing.name.pop_back();
  missing.position.pop_back();
  auto duplicate = validRawJointState();
  duplicate.name[0] = "joint1";
  auto nonfinite = validRawJointState();
  nonfinite.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(JointStateMapper::map(missing).valid);
  EXPECT_FALSE(JointStateMapper::map(duplicate).valid);
  EXPECT_FALSE(JointStateMapper::map(nonfinite).valid);
}
```

- [ ] **Step 2: Implement exact raw-to-model mapping**

`JointStateMapper::map()` searches exactly `joint1` through `joint6`, rejects missing/duplicate/non-finite values, and creates a planning message with exactly six names:

```cpp
static const std::array<std::string, 6> kRawNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
static const std::array<std::string, 6> kModelNames = {
    "cr10_joint1", "cr10_joint2", "cr10_joint3",
    "cr10_joint4", "cr10_joint5", "cr10_joint6"};
```

- [ ] **Step 3: Implement velocity estimation without silent zeros**

Define:

```cpp
struct VelocityEstimate {
  Eigen::Matrix<double, 6, 1> qd;
  bool valid{false};
};

class VelocityEstimator {
public:
  VelocityEstimator(std::size_t min_samples, double max_abs_velocity,
                    double alpha);
  VelocityEstimate update(const ros::Time& stamp,
                          const Eigen::Matrix<double, 6, 1>& q);
  void reset();
};
```

Require at least three strictly increasing timestamped samples. Compute finite differences, reject non-positive/too-large `dt`, limit magnitude before applying the configured low-pass `alpha`. Until valid, publish an empty velocity array on `/remani/cr10_joint_states` and set readiness `velocity_valid=false`; do not publish six zeros as measured velocity.

- [ ] **Step 4: Implement the State Bridge node**

The node:

```text
sub /remani/cr10_joint_states_raw
sub /odom
pub /remani/cr10_joint_states (exactly six mapped joints)
pub /joint_states (CR10 actual + configured static display defaults)
tf  world -> base_link from latest /odom
```

Default display joints in YAML must include Ranger steering/wheels and `gripper_finger1_joint`, each documented under `display_joint_defaults` as `measured: false`. Never copy these display defaults into the planning topic.

Define a project-owned normalized status interface (not AGX `RobotStatus`):

```text
/remani/cr10_status
  connected
  enabled
  error_status
  robot_mode
  stamp / feedback_age
```

Consume only that interface for readiness/fault. Do not compile against AGX-generated messages, do not modify remote driver sources, and do not infer healthy from connected/enabled or topic presence alone. Tests publish fake normalized status.

- [ ] **Step 5: Add a rostest for both outputs and TF ownership**

`test_state_bridge.py` publishes shuffled raw joints and odom, then asserts:

```python
self.assertEqual(6, len(planning.position))
self.assertEqual(
    ["cr10_joint1", "cr10_joint2", "cr10_joint3",
     "cr10_joint4", "cr10_joint5", "cr10_joint6"],
    planning.name)
self.assertGreater(len(robot_model.name), 6)
self.assertEqual("world", transform.header.frame_id)
self.assertEqual("base_link", transform.child_frame_id)
```

Publish a message with duplicate `joint1` and confirm neither output advances.

- [ ] **Step 6: Build and run State Bridge tests**

```bash
source /opt/ros/noetic/setup.bash
catkin_make -C remani_planner --pkg remani_real
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real
catkin_test_results remani_planner/build/test_results/remani_real
```

Do not build or source `agx/`.

- [ ] **Step 7: Commit State Bridge**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/joint_state_mapper.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/joint_state_mapper.cpp \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/velocity_estimator.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/velocity_estimator.cpp \
        remani_planner/src/REMANI-Planner/remani_real/src/remani_state_bridge_node.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_joint_state_mapper.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/state_bridge.test \
        remani_planner/src/REMANI-Planner/remani_real/test/test_state_bridge.py \
        remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
# plus any project-owned /remani/cr10_status message files created in remani_planner
git commit -m "feat: bridge actual state into REMANI ordering"
```

---

### Task 3: Assemble START/ADD/FINAL transactions with Gate-owned IDs

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/candidate_assembler.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/candidate_assembler.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_candidate_assembler.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: `/remani/planner_candidate`, current deployment state permission, current actual base yaw, and `ros::SteadyTime`.
- Produces: an assembly event and, only after FINAL, `FrozenCandidate` for validation. Gate acknowledgements must preserve and return `raw_transaction_stamp`.

- [ ] **Step 1: Write failing protocol tests**

```cpp
static ros::SteadyTime steadyAt(double sec) {
  ros::SteadyTime value;
  value.fromSec(sec);
  return value;
}

static quadrotor_msgs::PolynomialTraj controlMessage(uint32_t action) {
  quadrotor_msgs::PolynomialTraj msg;
  msg.action = action;
  msg.trajectory_id = 0;
  msg.singul = 0;
  return msg;
}

static quadrotor_msgs::PolynomialTraj validAdd(uint32_t id) {
  quadrotor_msgs::PolynomialTraj msg;
  msg.action = quadrotor_msgs::PolynomialTraj::ACTION_ADD;
  msg.trajectory_id = id;
  msg.singul = 1;
  quadrotor_msgs::PolynomialMatrix piece;
  piece.num_dim = 8;
  piece.num_order = 7;
  piece.duration = 1.0;
  piece.data.assign(64, 0.0);
  piece.data[56] = 0.1 * static_cast<double>(id - 1);
  piece.data[48] = 0.1;
  msg.trajectory.push_back(piece);
  return msg;
}

TEST(CandidateAssembler, AssignsCandidateIdOnEachAcceptedStart) {
  CandidateAssembler gate(60.0);
  EXPECT_EQ(1u, gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
      steadyAt(0), true, 0.0).candidate_id);
  gate.consume(controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_ABORT),
               steadyAt(1), true, 0.0);
  EXPECT_EQ(2u, gate.consume(
      controlMessage(quadrotor_msgs::PolynomialTraj::ACTION_WARN_START),
      steadyAt(2), true, 0.0).candidate_id);
}

TEST(CandidateAssembler, RequiresStrictContiguousSegmentsAndFinal) {
  CandidateAssembler gate(60.0);
  gate.consume(controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_START), steadyAt(0), true, 0.0);
  EXPECT_TRUE(gate.consume(validAdd(1), steadyAt(1), true, 0.0).accepted);
  EXPECT_FALSE(gate.consume(validAdd(1), steadyAt(2), true, 0.0).accepted);

  gate.consume(controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_START), steadyAt(3), true, 0.0);
  EXPECT_FALSE(gate.consume(validAdd(2), steadyAt(4), true, 0.0).accepted);
}

TEST(CandidateAssembler, FinalIsTheOnlyCompletionBoundary) {
  CandidateAssembler gate(60.0);
  gate.consume(controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_START), steadyAt(0), true, 0.0);
  gate.consume(validAdd(1), steadyAt(1), true, 0.0);
  EXPECT_EQ(nullptr, gate.completedCandidate());
  ASSERT_TRUE(gate.consume(controlMessage(
      quadrotor_msgs::PolynomialTraj::ACTION_WARN_FINAL),
      steadyAt(2), true, 0.0).accepted);
  EXPECT_NE(nullptr, gate.completedCandidate());
}
```

- [ ] **Step 2: Implement the assembly state and result types**

```cpp
enum class AssemblyState { Idle, Assembling, Complete, Invalid };

struct AssemblyEvent {
  bool accepted{false};
  AssemblyState state{AssemblyState::Idle};
  uint64_t candidate_id{0};
  std::string error_code;
  std::string detail;
};

class CandidateAssembler {
public:
  explicit CandidateAssembler(double timeout_sec);
  AssemblyEvent consume(const quadrotor_msgs::PolynomialTraj& msg,
                        const ros::SteadyTime& now,
                        bool new_transaction_allowed,
                        double actual_start_yaw);
  FrozenCandidate completedCandidate() const;
  void invalidate(const std::string& reason);
};
```

START increments a private `uint64_t next_candidate_id_` with overflow treated as fatal. ABORT/IMPOSSIBLE invalidate assembly and cached completion. START with `new_transaction_allowed=false` returns rejected without changing the current frozen candidate.

- [ ] **Step 3: Parse and validate each ADD before storing it**

For V1 every `PolynomialMatrix` must satisfy:

```text
num_dim == 8
num_order == 7
data.size() == 8 * 8
duration finite and > 0
all coefficients finite
trajectory array non-empty
singul is +1 or -1
```

Reconstruct the matrix with column-major layout:

```cpp
Eigen::Map<const Eigen::Matrix<double, 8, 8, Eigen::ColMajor>> coeff(
    piece_msg.data.data());
MMController::Piece piece(piece_msg.duration, coeff);
```

Store one `CandidateSegment` per ADD and preserve its `singul` separately.

- [ ] **Step 4: Enforce timeout and replacement semantics**

At every `consume()` and state publication, compare monotonic `now` with START time. More than 60 seconds before FINAL changes state to `Invalid` with `ASSEMBLY_TIMEOUT`. A new permitted START invalidates incomplete or PLANNED cache and never restores the old candidate if the new Plan fails.

- [ ] **Step 5: Run all assembler cases**

Add explicit tests for empty ADD, NaN coefficient, zero/negative duration, wrong dimension/order, ADD without START, FINAL without ADD, ABORT, IMPOSSIBLE, timeout, and START rejected while execution permission is false.

```bash
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_candidate_assembler
```

Expected: all protocol cases pass and `candidate_id` never equals/reuses `trajectory_id` semantics.

- [ ] **Step 6: Commit Gate assembly**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/candidate_assembler.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/candidate_assembler.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_candidate_assembler.cpp \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "feat: assemble versioned REMANI candidates"
```

---

### Task 4: Validate and freeze candidates, then publish isolated preview

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/candidate_validator.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/candidate_validator.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/preview_publisher.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/preview_publisher.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_candidate_validator.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: completed candidate, `ActualStateSnapshot`, shared static-empty `GridMap`, and `MMConfig`.
- Produces: `ValidationReport`, immutable frozen candidate, expected final base/q/EE, and isolated preview topics.

- [ ] **Step 1: Define and test a complete validation report**

```cpp
struct ValidationReport {
  bool valid{false};
  std::string error_code;
  std::string detail;
  double duration{0.0};
  double max_base_linear_speed{0.0};
  double max_base_angular_speed{0.0};
  double max_joint_speed{0.0};
  Eigen::Vector2d final_base_xy{Eigen::Vector2d::Zero()};
  double final_base_yaw{0.0};
  Eigen::Matrix<double, 6, 1> final_q;
  Eigen::Matrix4d expected_final_ee{Eigen::Matrix4d::Identity()};
};
```

Failing tests must cover segment continuity, start mismatch, base/joint speed limits, self collision, car-arm/arm-arm collision, and map boundary.

- [ ] **Step 2: Implement deterministic sampling validation**

Use `validation_dt=0.01` seconds and always include the exact final time. Validate:

```text
position/velocity/acceleration finite
position/velocity/acceleration continuity at every piece and ADD boundary
start base position <= 0.05 m
start base yaw <= 5 deg
start max joint error <= 3 deg
base linear speed <= 0.10 m/s
base angular speed <= 0.15 rad/s
each arm speed <= 0.10 rad/s
base and all MMConfig collision samples in GridMap bounds
MMConfig::checkcollision(..., safe=true) == false
```

Do not clamp a failing trajectory. Return a stable error code such as `START_STATE_MISMATCH`, `BASE_SPEED_LIMIT`, `JOINT_SPEED_LIMIT`, `MAP_BOUNDARY`, or `WHOLE_BODY_COLLISION`.

- [ ] **Step 3: Freeze only after validation passes**

The ROS composition may expose `candidate_complete=true` after FINAL, but `candidate_valid=true`, preview, `PLANNED`, and Execute enablement occur only after `ValidationReport::valid` is true. Cache `FrozenCandidate` and report together; never expose a mutable assembler vector to Executor.

- [ ] **Step 4: Publish preview without touching actual-state topics**

`PreviewPublisher::publish(const FrozenCandidate&, const ValidationReport&)` publishes:

```text
/remani/candidate_robot      visualization_msgs/MarkerArray
/remani/candidate_base_path  nav_msgs/Path
/remani/candidate_ee_path    nav_msgs/Path
```

Sample paths at 0.05 s and sparse robot poses at 0.25 s. Use namespace `candidate_<candidate_id>`, distinct colors for `singul=+1/-1`, and red markers only for rejected validation diagnostics. It must not advertise `/joint_states`, `/odom`, or TF.

- [ ] **Step 5: Run validator and preview publisher tests**

```bash
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real_gtest_test_candidate_validator
```

Add a ROS master test that checks `ros::master::getTopics()` contains the three preview topics and does not gain a second `/joint_states` or `/tf` publisher from `remani_real_node`.

- [ ] **Step 6: Commit validation and preview**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/candidate_validator.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/candidate_validator.cpp \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/preview_publisher.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/preview_publisher.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_candidate_validator.cpp \
        remani_planner/src/REMANI-Planner/remani_real/config/remani_real.yaml \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "feat: validate and preview frozen candidates"
```

---

### Task 5: Define the external deployment state machine and output boundaries

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/deployment_state_machine.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/deployment_state_machine.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/motion_output.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_deployment_state_machine.cpp`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`

**Interfaces:**
- Consumes: planner/readiness/assembly/validation events and candidate-bound commands.
- Produces: a pure deployment-state/permission decision and the two narrow output interfaces; ROS composition is deferred until after the Panel contract is fixed.

- [ ] **Step 1: Write the complete transition-table test first**

```cpp
static ReadinessSnapshot readySnapshot() {
  ReadinessSnapshot ready;
  ready.odom = true;
  ready.cr10_joints = true;
  ready.cr10_velocity = true;
  ready.tf = true;
  ready.robot_status = true;
  ready.action_server = true;
  ready.grid_map = true;
  ready.ranger_watchdog = true;
  return ready;
}

TEST(DeploymentStateMachine, RequiresExplicitExecuteAfterValidFinal) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  EXPECT_EQ(State::Ready, fsm.state());
  EXPECT_TRUE(fsm.requestPlan().accepted);
  EXPECT_EQ(State::Planning, fsm.state());
  fsm.onCandidateValidated(7, true);
  EXPECT_EQ(State::Planned, fsm.state());
  EXPECT_TRUE(fsm.requestExecute(7).accepted);
  EXPECT_EQ(State::Executing, fsm.state());
}

TEST(DeploymentStateMachine, RejectsStaleIdAndNewStartWhileExecuting) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onCandidateValidated(8, true);
  EXPECT_FALSE(fsm.requestExecute(7).accepted);
  ASSERT_TRUE(fsm.requestExecute(8).accepted);
  EXPECT_FALSE(fsm.newTransactionAllowed());
}

TEST(DeploymentStateMachine, OrdinaryPlanningFailureReturnsReady) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onPlanningFailure("NO_PATH", false);
  EXPECT_EQ(State::Ready, fsm.state());
}

TEST(DeploymentStateMachine, ProtocolCorruptionEntersError) {
  DeploymentStateMachine fsm;
  fsm.updateReadiness(readySnapshot());
  ASSERT_TRUE(fsm.requestPlan().accepted);
  fsm.onPlanningFailure("SEGMENT_SEQUENCE", true);
  EXPECT_EQ(State::Error, fsm.state());
}
```

- [ ] **Step 2: Implement exact states and button permissions**

```cpp
enum class State : uint8_t {
  NotReady, Ready, Planning, Planned, Executing, Paused, Succeeded, Error
};

struct CommandPermissions {
  bool plan{false};
  bool execute{false};
  bool pause{false};
  bool resume{false};
  bool abort{false};
};

struct ReadinessSnapshot {
  bool odom{false};
  bool cr10_joints{false};
  bool cr10_velocity{false};
  bool tf{false};
  bool robot_status{false};
  bool action_server{false};
  bool grid_map{false};
  bool ranger_watchdog{false};
};
```

Implement the spec transition table exactly. `requestExecute(uint64_t)` checks frozen/current ID equality and a fresh readiness snapshot. No state transition is inferred from wall time outside Executor callbacks.

- [ ] **Step 3: Define the output boundary used by later hardware phases**

```cpp
class RangerCommandChannel {
public:
  virtual ~RangerCommandChannel() = default;
  virtual bool publish(const geometry_msgs::Twist& command) = 0;
  virtual bool hardwareOutputEnabled() const = 0;
};

enum class ArmGoalState { Unavailable, Pending, Accepted, Active,
                          Succeeded, Canceled, Aborted };

class ArmCommandChannel {
public:
  virtual ~ArmCommandChannel() = default;
  virtual bool send(const trajectory_msgs::JointTrajectory& trajectory) = 0;
  virtual bool cancel() = 0;
  virtual bool stop() = 0;
  virtual ArmGoalState state() const = 0;
  virtual bool hardwareOutputEnabled() const = 0;
};
```

- [ ] **Step 4: Build and run the state/permission tests**

```bash
catkin_make -C remani_planner --pkg remani_real
catkin_make -C remani_planner --pkg remani_real --make-args run_tests_remani_real
catkin_test_results remani_planner/build/remani_real/test_results
```

- [ ] **Step 5: Commit the state and interface contract**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/include/remani_real/deployment_state_machine.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/deployment_state_machine.cpp \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/motion_output.hpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_deployment_state_machine.cpp \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt
git commit -m "feat: define real deployment state contract"
```

---

### Task 6: Add the RViz confirmation Panel as a separate plugin package

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real_rviz/CMakeLists.txt`
- Create: `remani_planner/src/REMANI-Planner/remani_real_rviz/package.xml`
- Create: `remani_planner/src/REMANI-Planner/remani_real_rviz/plugin_description.xml`
- Create: `remani_planner/src/REMANI-Planner/remani_real_rviz/include/remani_real_rviz/panel_view_model.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real_rviz/src/remani_real_panel.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real_rviz/src/remani_real_panel.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real_rviz/test/test_panel_view_model.cpp`

**Interfaces:**
- Consumes: `/remani/execution_state` and service responses.
- Produces: `/ee_goal_plan` Empty and calls Execute/Pause/Resume/Abort services; it publishes no hardware command.

- [ ] **Step 1: Write the failing pure view-model test**

```cpp
TEST(PanelViewModel, PlannedEnablesPlanExecuteAbortOnly) {
  remani_real_msgs::ExecutionState msg;
  msg.executor_state = msg.PLANNED;
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
  msg.environment_mode = msg.ENV_STATIC_EMPTY;
  EXPECT_EQ("ENVIRONMENT: STATIC EMPTY / NO ONLINE OBSTACLE SENSING",
            PanelViewModel::from(msg).environment_warning);
}
```

- [ ] **Step 2: Implement state-to-widget mapping with no local inference**

`PanelViewModel::from()` maps every button from `executor_state` plus candidate flags. It copies feedback ages, errors, candidate metrics, execution progress, pause time, start skew, and final errors directly from the message.

- [ ] **Step 3: Implement the Panel widget and ROS calls**

Create buttons `Plan`, `Execute`, `Pause`, `Resume`, `Abort`. Execute uses the currently displayed ID:

```cpp
remani_real_msgs::ExecuteCandidate service;
service.request.candidate_id = latest_state_.candidate_id;
execute_client_.call(service);
```

Plan publishes one `std_msgs::Empty` on `/ee_goal_plan`. Other buttons call their `std_srvs/Trigger` services. Disable the clicked button until a new `ExecutionState` or service response arrives, but never optimistically advance state.

- [ ] **Step 4: Export and build the plugin**

`plugin_description.xml`:

```xml
<library path="lib/libremani_real_rviz">
  <class name="remani_real_rviz/RemaniRealPanel"
         type="remani_real_rviz::RemaniRealPanel"
         base_class_type="rviz::Panel">
    <description>Manual Plan/Execute/Pause/Resume/Abort panel for REMANI real mode.</description>
  </class>
</library>
```

Run:

```bash
catkin_make -C remani_planner --pkg remani_real_rviz
catkin_make -C remani_planner --pkg remani_real_rviz --make-args run_tests_remani_real_rviz
```

- [ ] **Step 5: Commit the Panel**

```bash
git add remani_planner/src/REMANI-Planner/remani_real_rviz/CMakeLists.txt \
        remani_planner/src/REMANI-Planner/remani_real_rviz/package.xml \
        remani_planner/src/REMANI-Planner/remani_real_rviz/plugin_description.xml \
        remani_planner/src/REMANI-Planner/remani_real_rviz/include/remani_real_rviz/panel_view_model.hpp \
        remani_planner/src/REMANI-Planner/remani_real_rviz/src/remani_real_panel.hpp \
        remani_planner/src/REMANI-Planner/remani_real_rviz/src/remani_real_panel.cpp \
        remani_planner/src/REMANI-Planner/remani_real_rviz/test/test_panel_view_model.cpp
git commit -m "feat: add REMANI manual execution RViz panel"
```

---

### Task 7: Gate the complete planner-to-dry-run workflow

**Files:**
- Create: `remani_planner/src/REMANI-Planner/remani_real/include/remani_real/dry_run_motion_output.hpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/dry_run_motion_output.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_dry_run_output.cpp`
- Create: `remani_planner/src/REMANI-Planner/remani_real/launch/remani_real_control_plane.launch`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/fake_real_feedback_node.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/fake_ee_goal_marker.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/fake_candidate_planner.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/real_control_plane.test`
- Create: `remani_planner/src/REMANI-Planner/remani_real/test/test_real_control_plane.py`
- Create: `remani_planner/src/REMANI-Planner/remani_real_rviz/config/remani_ranger_cr10_real.rviz`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt`
- Modify: `remani_planner/src/REMANI-Planner/remani_real/package.xml`

**Interfaces:**
- Consumes: the production phase-1 planner-only launch; the rostest substitutes a deterministic fake planner at the same ROS interface so Gate/Panel behavior is not coupled to optimizer randomness.
- Produces: a testable Plan→PLANNED→explicit Execute→dry-run terminal workflow with zero hardware output.

- [ ] **Step 1: Implement and unit-test the zero-output adapter**

```cpp
class DryRunMotionOutput : public RangerCommandChannel,
                           public ArmCommandChannel {
public:
  bool publish(const geometry_msgs::Twist& command) override;
  bool send(const trajectory_msgs::JointTrajectory& trajectory) override;
  bool cancel() override;
  bool stop() override;
  ArmGoalState state() const override;
  bool hardwareOutputEnabled() const override { return false; }
  const std::vector<geometry_msgs::Twist>& rangerDiagnostics() const;
  const std::vector<trajectory_msgs::JointTrajectory>& armDiagnostics() const;
  std::size_t hardwarePublishCount() const { return 0; }
  std::size_t actionGoalCount() const { return 0; }
  std::size_t writeServiceCount() const { return 0; }
};
```

The failing test builds one six-name/two-point full position+velocity trajectory and one 0.05 m/s Twist, calls both dry interfaces, and asserts diagnostic sizes are one while all three hardware/write counters remain zero. The class contains no `ros::Publisher`, Action client, or service client member.

- [ ] **Step 2: Compose only safe control-plane nodes**

`remani_real_node` owns one State Machine, Assembler, Validator, FrozenCandidate, PreviewPublisher, ActualState cache, and references to one `RangerCommandChannel` and one `ArmCommandChannel`; both point to the same owned `DryRunMotionOutput`. Reject startup unless `mode=real`, `execution_owner=external`, `environment_mode=static_empty`, and `dry_run=true` in this phase.

For this hardware-free phase, Execute records `ros::SteadyTime::now()`, samples `t=steady_now-execute_time`, writes only dry diagnostic intent, and drives `EXECUTING→SUCCEEDED` against a simulated desired-state plant. Pause freezes that parameter time; Resume continues it; Abort clears it. This phase-local clock never updates Planner state and is replaced—not run in parallel—by the explicit shared-T0 `SynchronizedExecutor` in phase 5.

The launch includes planner-only real launch, State Bridge, `robot_state_publisher`, `remani_real_node dry_run:=true`, Marker, and RViz. It does not include Ranger or CR10 hardware nodes. Add explicit planner remaps:

```xml
<remap from="~odom_world" to="/odom"/>
<remap from="~joint_state" to="/remani/cr10_joint_states"/>
```

- [ ] **Step 3: Create deterministic fake feedback**

`fake_real_feedback_node.py` publishes finite `/odom`, shuffled `/remani/cr10_joint_states_raw`, fake healthy `/remani/cr10_status`, and watchdog readiness. It advertises a fake `/cr10_robot/joint_controller/follow_joint_trajectory` ActionServer solely so the read-only readiness client can connect; the server counts and rejects any received goal, and the test requires that count to remain zero. It never advertises `/remani/hardware/ranger/cmd_vel` or a CR10 write service.

`fake_ee_goal_marker.py` caches one fixed reachable `PoseStamped` and publishes it on `/ee_goal` for every `/ee_goal_plan` Empty. `fake_candidate_planner.py` listens to `/ee_goal`, publishes `PlannerStatus::PLANNING`, then emits one deterministic 8D degree-7 START/ADD(trajectory_id=1)/FINAL transaction on `/remani/planner_candidate` and `PlannerStatus::HANDOFF` followed by IDLE. This substitution exists only in `real_control_plane.test`; `remani_real_control_plane.launch` includes the actual PLAN-ONLY planner.

- [ ] **Step 4: Write the integration assertions**

`test_real_control_plane.py` performs:

```text
wait READY
publish /ee_goal_plan
wait START/ADD.../FINAL and PLANNED
assert no motion output before Execute
call Execute(candidate_id - 1) -> rejected
call Execute(candidate_id) -> accepted
wait dry-run SUCCEEDED
```

It checks ROS master state and counters:

```python
self.assertNotIn("/remani/hardware/ranger/cmd_vel", published_topics)
self.assertEqual(0, fake_feedback.cr10_action_goal_count)
self.assertEqual(0, arm_write_service_count)
self.assertGreater(dry_ranger_preview_count, 0)  # /remani/dry_run/ranger_cmd_vel_preview
self.assertGreater(candidate_preview_count, 0)
```

Also inject ADD-before-START, duplicate ADD, skipped ID, timeout, and START during EXECUTING; each must produce the specified READY/ERROR behavior without altering the frozen executing candidate.

- [ ] **Step 5: Run the complete phase test**

```bash
catkin_make -C remani_planner --pkg remani_real_msgs plan_env remani_planner remani_real remani_real_rviz
catkin_make -C remani_planner --pkg remani_real remani_real_rviz --make-args run_tests
catkin_test_results remani_planner/build
```

Expected: all tests pass, hardware output counts are zero, and Execute remains an explicit candidate-bound operation.

- [ ] **Step 6: Commit the dry-run control plane and integration gate**

```bash
git add remani_planner/src/REMANI-Planner/remani_real/launch/remani_real_control_plane.launch \
        remani_planner/src/REMANI-Planner/remani_real/include/remani_real/dry_run_motion_output.hpp \
        remani_planner/src/REMANI-Planner/remani_real/src/dry_run_motion_output.cpp \
        remani_planner/src/REMANI-Planner/remani_real/src/remani_real_node.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/test_dry_run_output.cpp \
        remani_planner/src/REMANI-Planner/remani_real/test/fake_real_feedback_node.py \
        remani_planner/src/REMANI-Planner/remani_real/test/fake_ee_goal_marker.py \
        remani_planner/src/REMANI-Planner/remani_real/test/fake_candidate_planner.py \
        remani_planner/src/REMANI-Planner/remani_real/test/real_control_plane.test \
        remani_planner/src/REMANI-Planner/remani_real/test/test_real_control_plane.py \
        remani_planner/src/REMANI-Planner/remani_real_rviz/config/remani_ranger_cr10_real.rviz \
        remani_planner/src/REMANI-Planner/remani_real/CMakeLists.txt \
        remani_planner/src/REMANI-Planner/remani_real/package.xml
git commit -m "test: gate REMANI real dry run control plane"
```

## Phase Exit Gate

```text
actual raw q names -> fixed six-joint planner topic
actual odom -> one world/base_link TF
START/ADD/FINAL -> immutable Gate-owned candidate_id
FINAL + validation -> PLANNED + isolated preview
explicit Execute(current candidate_id) -> dry EXECUTING
dry_run -> zero Ranger hardware publishes, zero CR10 goals, zero arm write services
Panel -> unified ExecutionState only
```
