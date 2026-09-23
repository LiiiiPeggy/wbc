# Project Progress

Updated: 2026-09-22

## Current Goal

Deploy REMANI on the Ranger+CR10 real robot while preserving the existing simulation path. Phase 5 software integration is complete on the laptop; formal synchronized hardware remains for physical authorization.

## Current Implementation State

- Active branch: `remani-real-implementation` (primary checkout; worktree no longer used).
- Phase 2–5 software gates green on laptop; Phase 5 tip includes unified launch through `d474601`, plus project memory under `remani_planner/`.
- Robot-host helpers: `agilex_ws/scripts/` (standard catkin `src/` layout). Laptop helpers: `remani_planner/scripts/`.
- Leftover uncommitted (out of scope): `remani_real/package.xml` demo deps, `remani_real_dry_run_demo.launch`.

## Verified Work (2026-09-15)

- Phase 5 commits: `ee96065` unified launch + SynchronizedExecutor wire; `d474601` synchronized dry-run rostests + acceptance doc.
- Rostest SUCCESS: `real_launch_contract`, `synchronized_dry_run`, `pause_resume_abort`, `final_completion`, `real_control_plane` (`catkin_test_results remani_real` → 175/0/0).
- Sim regression: `rostest remani_planner remani_sim_owner.test` SUCCESS; `timeout 20s roslaunch remani_sim.launch robot_model:=ranger_cr10 use_rviz:=false` starts planner + `mm_controller` + fake_mm under local `ROS_MASTER_URI` (exit 124 = timeout OK). Note: env `ROS_MASTER_URI` pointing at unreachable robot host fails smoke.
- AGX laptop gtests (no messenger): `test_ranger_command` 7/7, `test_command_watchdog` 2/2, CR10 validator 6/6, runner 5/5, adapter 3/3. Full `ranger_messenger` build still fails on this host without standalone `asio` (expected; verify on 18.04).

## Current Issues

- Formal non-dry Ranger/CR10/synchronized motion still needs Ubuntu 18.04 authorization (`synchronized_acceptance.md` stages 2–10).
- Real-feedback dry-run lacks a `/dobot_v4_bringup/msg/RobotStatus` → `/remani/cr10_status` converter; temporary laptop stub is `remani_planner/scripts/publish_cr10_status_stub.sh`.
- Plan Task 6 Step 3 full fault-injection rostest matrix not expanded beyond control-plane / sync happy paths.
- AGX full package `run_tests` (including messenger) not runnable on this laptop.

## Next Steps

- On robot: `cd agilex_ws && ./scripts/bringup_can.sh && ./scripts/run_real_dry_run.sh`, then attach laptop dry-run control plane.
- Add formal Cr10Status bridge (no AGX msg compile dep on laptop), then authorized physical stages.
- Optional: expand fault-injection rostests; push branch / open PR.
