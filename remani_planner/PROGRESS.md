# Project Progress

Updated: 2026-09-15

## Current Goal

Deploy REMANI on the Ranger+CR10 real robot while preserving the existing simulation path. Phase 5 software integration is complete on the laptop; formal synchronized hardware remains for physical authorization.

## Current Implementation State

- Active branch: `remani-real-implementation` (primary checkout; worktree no longer used).
- Phase 2–5 software gates green on laptop; Phase 5 tip includes unified launch through `d474601`, plus project memory under `remani_planner/`.
- Leftover uncommitted (out of scope): `remani_real/package.xml` demo deps, `remani_real_dry_run_demo.launch`, `agx/.catkin_workspace`.

## Verified Work (2026-09-15)

- Phase 5 commits: `ee96065` unified launch + SynchronizedExecutor wire; `d474601` synchronized dry-run rostests + acceptance doc.
- Rostest SUCCESS: `real_launch_contract`, `synchronized_dry_run`, `pause_resume_abort`, `final_completion`, `real_control_plane` (`catkin_test_results remani_real` → 175/0/0).
- Sim regression: `rostest remani_planner remani_sim_owner.test` SUCCESS; `timeout 20s roslaunch remani_sim.launch robot_model:=ranger_cr10 use_rviz:=false` starts planner + `mm_controller` + fake_mm under local `ROS_MASTER_URI` (exit 124 = timeout OK). Note: env `ROS_MASTER_URI` pointing at unreachable robot host fails smoke.
- AGX laptop gtests (no messenger): `test_ranger_command` 7/7, `test_command_watchdog` 2/2, CR10 validator 6/6, runner 5/5, adapter 3/3. Full `ranger_messenger` build still fails on this host without standalone `asio` (expected; verify on 18.04).

## Current Issues

- Formal non-dry Ranger/CR10/synchronized motion still needs Ubuntu 18.04 authorization (`synchronized_acceptance.md` stages 2–10).
- Plan Task 6 Step 3 full fault-injection rostest matrix not expanded beyond control-plane / sync happy paths.
- AGX full package `run_tests` (including messenger) not runnable on this laptop.

## Next Steps

- Authorized physical synchronized acceptance on the 18.04 host.
- Optional: expand fault-injection rostests; push branch / open PR.
