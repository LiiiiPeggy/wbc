# Engineering Process

This file preserves important engineering history: why a design was chosen, what alternatives were considered, what failed, and how the final result was verified. Keep current status in `PROGRESS.md` and durable takeaways in `MEMORY.md`.

## REMANI Ranger+CR10 Real-Robot Deployment Design

Date: 2026-09-01

Primary reference: `docs/superpowers/specs/2026-09-01-remani-real-robot-deployment-design.md`

### Background

The repository already had a REMANI Ranger+CR10 simulation path. The real-robot goal is to let RViz generate and preview a whole-body trajectory, then require human confirmation before executing on Ranger + CR10 hardware.

### Source-Grounded Findings

- REMANI simulation currently owns execution internally: successful planning enters `GEN_NEW_TRAJ -> EXEC_TRAJ`, advances trajectory time from `ros::Time::now() - trajectory.start_time`, and publishes ADD-only `PolynomialTraj` messages to `mm_controller`.
- `checkCollisionCallback()` uses the same virtual execution time for future collision checks, local replanning, and emergency handling when not in `WAIT_TARGET`.
- `PolynomialTraj` already had START/FINAL action values, but the planner originally used only ADD. Its `trajectory_id` is a segment index inside one trajectory, not a candidate version.
- Planner arm state handling reads `JointState.position[0..5]` directly, so real CR10 state must be supplied on a separate fixed-order six-axis topic rather than a full RobotModel `/joint_states` array.
- The Ranger driver subscribes to absolute `/cmd_vel`, so real Ranger command isolation must be handled at launch/topic-boundary level.
- The CR10 Action implementation blocks while executing a full trajectory, which prevents reliable preemption under a single-threaded spinner.

### Final Design

The selected design separates planning from real execution:

- `mode=sim` keeps `execution_owner=internal`, preserving existing REMANI simulation semantics.
- `mode=real` requires `execution_owner=external`; REMANI becomes strict PLAN-ONLY and never owns real execution time.
- Real candidates flow through Planner -> Trajectory Gate -> Real Executor. The Executor must consume only Gate-frozen candidates, never raw planner output.
- Real trajectory publication uses START, contiguous ADD segments, and FINAL as a transaction. The Gate assigns monotonic candidate IDs.
- The first real phase uses a static-empty GridMap and `global_plan=true`; dynamic perception and execution-time replanning are out of scope.
- `dry_run=true` exercises planning, validation, sampling, timing, and state machines while blocking hardware motion outputs and mutating arm calls.

### Verification Preserved From Existing Project Evidence

- The design document records source-consistency review blockers as closed.
- The prior project progress snapshot records phase-1 Task 1-5 as independently reviewed, including focused verification for static-empty GridMap and planner-only policy.
- During the shared-memory initialization on 2026-09-06, those tests were not rerun; they remain documented evidence, not fresh verification from this task.

### Remaining Engineering History To Capture Later

- When Task 6 completes, record the reason for its final implementation and the verification evidence here if it involves a meaningful design choice, failed attempt, or root-cause finding.
- When hardware Executor work begins, record hardware safety decisions, rejected alternatives, and validation results here rather than burying them in `PROGRESS.md`.

## Phase 2 Task 1 Candidate Sampling Root Isolation

Date: 2026-09-09

### Problem

Stopped-candidate yaw recovery originally formed `vx² + vy² - 1e-12` in raw piece time and trimmed high-order coefficients with a scale-aware absolute tolerance. On long-duration pieces, dynamically important high-order terms become tiny in raw coefficients and were trimmed away, so the last valid speed interval disappeared and yaw fell back to the constructor start yaw. Tangent/repeated threshold roots were also unreliable under Eigen `PolynomialSolver` in raw time.

### Decision

Recover heading by isolating real roots on the bounded normalized domain `u = t / duration ∈ [0,1]` with long-double coefficient scaling, derivative-recursive critical-interval isolation, and bisection. Roots after the query time are discarded without tolerance expansion. Stopped samples keep angular velocity at 0. Catkin Eigen export uses `DEPENDS EIGEN3`.

### Verification

- Honest RED: detached worktree at `d20a308` with only the new tests failed `RecoversHeadingAcrossLongNormalizedDuration` and `RecoversTangentThresholdHeading` (yaw remained start yaw `0.73`).
- GREEN at `9cc4b1f`: 15/15 sampling tests, XML 0 failures/0 errors, public consumer build succeeded, generated Configs export Eigen includes, no AGX dependency in scoped packages.
- Independent scoped re-review verdict: `ship`; previous Important and Minor findings closed.

## Cross-Host Control-Plane Boundary

Date: 2026-09-09

Phase 2 dry-run work is constrained to the laptop control plane. Remote AGX is treated as a black-box ROS endpoint. Local `catkin_make -C agx` / `source agx/devel/setup.bash`, AGX message compile dependencies, and AGX source edits for Task 2 status telemetry are removed from the control-plane plan. Formal non-dry output remains blocked on separate remote safety and sync capabilities.

## Phase 2 Task 2 State Bridge

Date: 2026-09-09

### Decision

Implement State Bridge entirely in `remani_planner` with project-owned `Cr10Status`, standard JointState/Odometry inputs, name-ordered planning joints, empty-until-valid velocity arrays, display-only RobotModel joints, and TF copied from `/odom`. Do not modify remote AGX drivers.

### Verification

- Commits: `132c7ed`, `dbc90fb`, `158fa6e`.
- GREEN: mapper 2/2, velocity 5/5, sampling 15/15, rostest state_bridge 1/1; catkin_test_results 46/0/0.
- Independent reviews: initial `ship` with Minors; after odom-z and zero-stamp fixes, re-review `ship` with status-snapshot residual deferred-ok for later readiness composition.

## Phase 2 Task 7 Dry-Run Control Plane

Date: 2026-09-13

### Decision

Wire laptop-only `remani_real_node` to State Bridge, Gate assembler/validator/preview, Deployment SM, and `DryRunMotionOutput`. Advertise only `/remani/dry_run/ranger_cmd_vel_preview` for base motion diagnostics; never advertise `/remani/hardware/ranger/cmd_vel` or send CR10 FollowJointTrajectory goals under dry-run. Startup hard-gates `mode=real` + `execution_owner=external` + `environment_mode=static_empty` + `dry_run=true`. Rostest uses fake feedback/planner/marker nodes and asserts stale Execute rejection, SUCCEEDED, zero hardware topic, and zero action goals.

### Fixes During Integration

- Rostest topic enumeration via `rosgraph.Master` failed under rostest wrappers (`NameError` / stale install); switched to `rospy.get_published_topics()`.
- IMPOSSIBLE/ABORT assembler events are `accepted=true` + Invalid; control plane must still invoke `onPlanningFailure` or Planning sticks until Abort.
- `ExecutionState.last_error_code` now prefers FSM `lastErrorCode()` over a stale valid validation report.
- CR10 `status_ok_` requires `enabled` in addition to connected and zero error.
- Independent review P1: Gate callbacks must use the Plan-time captured `active_plan_session_`, and Plan/Abort must `assembler_.invalidate()` so late FINAL cannot PLANNED a stale transaction under a new session. Idle rejects `ADD_WITHOUT_START` / `FINAL_WITHOUT_START` / `START_NOT_ALLOWED` must not kill the new Planning session. Timer polls `pollTimeout()` for `ASSEMBLY_TIMEOUT`.

### Verification

- Commits: `3959cc7` (adapter prerequisite `90b228d`); review fix `8c9d14c` (session/assembler isolation).
- GREEN: assembler 10/10; deployment SM 14/14; rostest `test_real_control_plane` passed; `catkin_test_results remani_real` 124/0/0.
- Exit-gate checklist PASS for zero hardware pubs, zero CR10 goals, explicit Execute.
- Residual deferred: feedback readiness flags do not expire after first sample.

## Phase 4 CR10 Non-blocking Action (2026-09-13/14)

### Problem

`CRRobot::moveHandle` blocked on full-trajectory Hermite sampling with `ros::Rate`/`sleep`, so cancel could not preempt and REMANI could not own a shared T0.

### Resolution

- Pure `TrajectoryGoalValidator` + one-tick `TrajectoryRunner` (Hermite, hold, settle, cancel) in `dobot_v4_bringup`.
- `FollowJointTrajectoryAdapter` + production ServoJ sink; `CRRobot` goal/cancel/timer path returns promptly.
- `ArmTrajectoryBuilder` / `Cr10HardwareChannel` in `remani_real` (Stop/E-Stop via MD5-compatible `remani_real_msgs` srv mirrors; no AGX compile dep on laptop).
- Software gate: `rostest remani_real cr10_only_execution.test` (fake Action server).

### Verification

- Commits through `c8b5fed` on `remani-real-implementation`.
- AGX gtests: validator 6/6, runner 5/5, adapter 3/3; remani_real builder 2/2; CR10-only rostest SUCCESS.
- Executor composition of `Cr10HardwareChannel` deferred to Phase 5.

## Phase 5 Synchronized Integration (2026-09-15)

### Problem

Ranger and CR10 channels existed separately; dry-run Execute still used wall-clock sampling without a shared T0, and there was no unified real launch / `run_remani.sh mode:=real` path.

### Resolution

- Pure `ExecutionClock` + `SynchronizedExecutor` + `RemainingCandidate` + `RuntimeSafetyMonitor` + `CompletionVerifier`.
- `remani_real_node` composes SynchronizedExecutor: dry-run uses `DryRunMotionOutput` for both channels; non-dry uses `RangerHardwareChannel` + `Cr10HardwareChannel`.
- Unified `remani_real.launch` with CI-safe `start_hardware_drivers=false` and optional `remani_real_hardware.launch` on the 18.04 host (canonical `/remani/hardware/ranger/cmd_vel`).
- `run_remani.sh` keeps default `sim`; `mode:=real` selects `remani_real` and rejects `execution_owner:=internal`.
- Startup odom origin gate `START_ODOM_NOT_ZERO` in State Bridge and Executor.

### Verification

- Rostest SUCCESS: `real_launch_contract`, `synchronized_dry_run`, `pause_resume_abort`, `final_completion`, `real_control_plane` (remani_real suite 175/0/0).
- Sim regression: `remani_sim_owner.test` SUCCESS; short `remani_sim.launch` smoke OK under local master.
- AGX pure gtests on laptop: ranger_command 7/7, watchdog 2/2, CR10 validator/runner/adapter 6/5/3. Messenger build skipped without standalone asio (18.04 host).
- Acceptance checklist: `remani_real/docs/synchronized_acceptance.md` (physical stages still unauthorized).
- Commits: `ee96065`, `d474601` on `remani-real-implementation`.

## Robot-Host Scripts (agilex_ws/scripts) for Split Deployment

Date: 2026-09-22

### Decision

Keep `remani_planner` on the laptop and robot drivers in `agilex_ws/` (`src/` packages + `scripts/` helpers; formerly flat `agx/`). Split helpers: `agilex_ws/scripts/` (build/CAN/drivers/sensors/watches) vs `remani_planner/scripts/` (dry-run control plane, Cr10Status stub, execution watches). Builds must use `--source agilex_ws/src`. Robot `run_real_dry_run.sh` only launches `scripts/launch/remani_hardware_drivers.launch`. Do not put motion-publishing or `dry_run:=false` one-liners on either host's helper set.

### Gap recorded

No AGX → `remani_real_msgs/Cr10Status` bridge yet. Temporary laptop stub: `remani_planner/scripts/publish_cr10_status_stub.sh`.
