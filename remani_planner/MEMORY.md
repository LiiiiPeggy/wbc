# Project Memory

This file stores durable, reusable project knowledge. Do not record current tasks, transient blockers, raw logs, or unverified guesses here.

## Repository and Environment

- The checkout's effective repository root is `wbc/` under `/home/gzz/Codes/remani`.
- REMANI project memory lives under `remani_planner/` as `PROGRESS.md`, `MEMORY.md`, and `PROCESS.md` (not at the repository root).
- The project is a ROS1/Noetic mobile-manipulator research and integration repository for Ranger base + CR10 arm whole-body control.
- Major roots are `agx/` for real robot packages, `ocs2_ws/` for OCS2, `remani_planner/` for REMANI-Planner, `robotics-toolbox-python/` for Holistic/RTB experiments, and `scripts/` for local adaptation scripts.
- `agx/` is a nonstandard catkin source root with packages directly beneath it. Package-selective builds require an explicit source root and package whitelist; there is no `agx/src/` directory.
- `remani_planner` package-selective builds may require building `traj_utils` first so `traj_utils/DataDisp.h` is generated; after that prerequisite, the planner node builds successfully.

## Durable Architecture Constraints

- The binding real-robot design is `docs/superpowers/specs/2026-09-01-remani-real-robot-deployment-design.md`. Implementation-plan examples that conflict with it are stale.
- `mode=sim` means `execution_owner=internal`: REMANI keeps its existing `GEN_NEW_TRAJ -> EXEC_TRAJ -> mm_controller` lifecycle and ADD-only trajectory publication.
- `mode=real` means `execution_owner=external`: REMANI is strict PLAN-ONLY. It publishes a candidate transaction and must not run planner wall-clock execution, execution-time replanning, planner EE completion, or `planning/finish` for that candidate. Trajectory Gate and Real Executor own the real execution lifecycle.
- In real/external mode, waypoint input is admitted only after the planner reaches its planning-idle `WAIT_TARGET` state. Accepting a preset target during `INIT` can otherwise enter legacy `planNextWaypoint()` logic that waits for the forbidden `EXEC_TRAJ` state indefinitely. Simulation keeps its existing admission behavior.
- The real data boundary is Planner `/remani/planner_candidate` -> Trajectory Gate -> `/remani/frozen_candidate` -> Real Executor. The Executor must never consume a raw planner candidate.
- In real mode, `PolynomialTraj` actions form a Gate transaction protocol: START begins, contiguous ADD messages append segments, FINAL commits, ABORT invalidates, and IMPOSSIBLE reports planning failure. Simulation retains the original ADD-only meaning.
- `PolynomialTraj.trajectory_id` is a 1-based segment index that restarts for each transaction. The Gate alone assigns monotonic `candidate_id`; Execute requests bind to that Gate version.
- Planner and executor states are independent: planner is `IDLE/PLANNING/HANDOFF`; executor is `NONE/PLANNED/EXECUTING/PAUSED/SUCCEEDED/ERROR`. Planner IDLE while executor EXECUTING is valid.
- State Bridge must publish full `/joint_states` for RobotModel/RViz and a separate, fixed-order six-axis `/remani/cr10_joint_states` for REMANI. The current planner reads `position[0..5]`, so it must not consume the full RobotModel array.
- Real Ranger commands must be isolated at launch level on `/remani/hardware/ranger/cmd_vel`; ordinary `/cmd_vel` must not reach hardware. Publisher-count checking is only a second defense.
- `dry_run=true` forbids publishing hardware motion commands, sending a CR10 FollowJointTrajectory goal, or calling mutating arm services. Read-only readiness and feedback checks remain allowed.
- Real-mode laptop deployment is control-plane only: planner/Gate/State Bridge/dry-run Executor/RViz run on the notebook; `agx/` is a remote black-box ROS endpoint. Do not compile, source, or start `agx/` packages on the laptop for Phase 2 dry-run work.
- Ranger + CR10 hardware attach to an Ubuntu 18.04 x86-64 host where `agx/` previously built and ran. Laptop development hosts (e.g. Ubuntu 20.04/Noetic) may lack standalone `asio.hpp` and must not treat a failed local `ugv_sdk` build as an AGX driver regression; full `ranger_base`/`ugv_sdk`/`dobot_v4_bringup` verification belongs on that 18.04 machine.
- Canonical Ranger hardware command topic is `/remani/hardware/ranger/cmd_vel`; dry-run diagnostics use `/remani/dry_run/ranger_cmd_vel_preview` only. Planner raw transaction topic is `/remani/planner_candidate`; planner state name is `HANDOFF`.
- Formal CR10 readiness/fault uses project-owned `/remani/cr10_status`. Absence/staleness must keep `NOT_READY`; do not infer healthy from connected/enabled or topic presence, and do not add AGX-generated `RobotStatus` compile deps for the laptop control plane.
- Phase-2 `remani_real_node` startup rejects unless `mode=real`, `execution_owner=external`, `environment_mode=static_empty`, and `dry_run=true`. Control services are `/remani/execute|pause|resume|abort`; state is latched on `/remani/execution_state`.
- Assembler `ACTION_WARN_IMPOSSIBLE` / `ACTION_ABORT` return `accepted=true` with `AssemblyState::Invalid`; the control plane must still call `onPlanningFailure` so Planning returns to Ready.
- Dry-run Execute readiness treats CR10 status as OK only when `connected && enabled && error_status==0`.
- Gate callbacks must bind the Plan-time `plan_session_id` (not the live id at callback time). Plan and Abort invalidate the assembler; ignore late `*_WITHOUT_START` / `START_NOT_ALLOWED` so they cannot abort a fresh Planning session.
- State Bridge maps raw `joint1..joint6` by name into fixed `cr10_joint1..cr10_joint6`, publishes RobotModel `/joint_states` with measured=false display defaults, and broadcasts the sole laptop `world -> base_link` TF copied from `/odom` (no second integrator). Velocity estimates stay unpublished (empty array) until three strictly increasing samples pass dt/clamp/LPF checks.
- Real V1 uses configurable `environment/static_empty/{size_x,size_y,size_z,resolution}` defaults `16.0/12.0/3.0/0.05`. It provides a bounded free GridMap/ESDF but does not detect real obstacles.
- V1 Resume continues the original remaining trajectory only when the actual stopped state is within strict tolerance at the Executor-owned pause parameter. Otherwise Resume is rejected and the operator must Abort and replan; no automatic whole-body connector is allowed.
- Real execution succeeds only when CR10 completion, Ranger completion, base tolerance, joint tolerance, and actual EE FK tolerance all pass.

## Verified Implementation and Environment Facts

- Real-hardware validation has so far covered MoveIt end-effector goal motion only; the REMANI whole-body real execution path is not yet validated on hardware.
- CR10 FollowJointTrajectory in `dobot_v4_bringup` is timer-driven and preemptible: validate→accept, one ServoJ per tick via Hermite/`TrajectoryRunner`, cancel→Stop with no further ServoJ, settle to SUCCEEDED/ABORTED/CANCELED. Laptop control plane uses `ArmTrajectoryBuilder` + `Cr10HardwareChannel` (Stop/E-Stop via `remani_real_msgs` MD5-compatible srv mirrors) composed into `remani_real_node` through `SynchronizedExecutor` when `dry_run=false`.
- Unified real launch is `remani_real/launch/remani_real.launch`. Laptop/CI default is `start_hardware_drivers=false` with fakes; robot host sets `start_hardware_drivers:=true` and `hardware_drivers_launch` to `remani_real_hardware.launch`. `run_remani.sh` defaults to `mode=sim`; `mode:=real` launches `remani_real` with `DOBOT_TYPE=cr10` and rejects `execution_owner:=internal`.
- Startup odom must stay near world origin for the first three finite samples (≤1 mm, ≤0.1 deg) or State Bridge / Executor reject with `START_ODOM_NOT_ZERO`.
- Deployment SM: `Succeeded` permits Plan only (not Abort). Abort recovers from Error/Executing/Paused/Planning/Planned.
- For this catkin package, `add_rostest(test/remani_plan_only.test)` generates the focused Make target `run_tests_remani_planner_rostest_test_remani_plan_only.test`; the `.test` suffix is part of the target name.
- On the Ubuntu 18.04 robot host, `ugv_sdk` historically builds with the local `asio` stack. Ubuntu 20.04 laptop hosts need `libasio-dev` (`asio.hpp`); without it, skip hardware messenger/node and keep pure `ranger_command`/`command_watchdog` gtests. Do not substitute Boost.Asio for standalone `asio.hpp` in `ugv_sdk` (handler/`error_code` ABI mismatch).
- Catkin `DEPENDS` for Eigen on this machine must use `EIGEN3` casing for export into generated `*Config.cmake`. External consumers should verify an exported include directory contains `Eigen/Core` rather than hardcoding `/usr/include/eigen3`.
- Focused `run_tests_*_gtest_*` Make targets can return shell exit 0 even when gtest reports FAILED TESTS; always inspect console FAILED TESTS, generated XML, and `catkin_test_results`.
- Candidate yaw recovery for stopped samples must isolate real roots of the speed-threshold polynomial on normalized time `[0,1]`; raw-time coefficient trimming can drop dynamically significant high-order terms on long durations.
