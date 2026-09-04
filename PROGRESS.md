# Progress

**Goal:** On branch `topay`, finish Ranger+CR10 visualization/collision fixes per `docs/superpowers/plans/2026-08-31-ranger-cr10-viz-collision-fixes.md` (wheels, CR10 sphere frames, upper-box false negatives).

**Branch / HEAD:** `topay` @ `9f6b0c6` (Tasks 0–2C committed). Working tree has large **uncommitted** Task 3+ changes; do not treat them as merged.

**Committed and verified (docker `topay`):**
- Task 0–1: STL audit + four-wheel visual ground fix (`--test-wheels` PASS).
- Task 2A–2C: planning sphere truth, `/sphere_visual` overlay, CR10 cylinder exact-chain (`test_topay_cr10_colli_frame` PASS).
- Base-box FD gate inside `test_topay_cr10_fk` PASS (corner sphere, yaw 0/0.7).

**Implemented in tree, not committed / not fully re-verified after last edit:**
- `base_obstacle_proxies_` + YAML `box_obstacle` (3×3×2, R=0.18, margin 0.02).
- `getBaseObstaclePts` / `getBaseObstacleGrads`; GridMap + `MomaTrajOpt*` (incl. falm/relax) wired.
- Docs: collision-model diagnosis + consumer audit.
- Discrete+continuous gates merged into `map/test_topay_box_collision` (Cases A–D); planner-side traj test **removed**.
- GridMap standalone tests must use `agent/mode=planner` (not `test`) or they hang without roscore.

**Open:**
- Rebuild/re-run `test_topay_box_collision` (A–D) and remaining Final Regression Gates after merge cleanup.
- Commit Task 3+ (exclude `map.pcd`).
- Task 4 smoke: `roslaunch planner run_ranger_cr10_smoke.launch` — not confirmed here.
- Docker `topay` may be stopped; host `TopAY/build` is often root-owned — prefer container builds with modest `-j`.

**Next:** Verify box gate in docker → commit Task 3 stack → full regression + smoke → update this file.
