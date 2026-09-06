# Progress

**Goal:** Ranger+CR10 viz/collision fixes on `topay` — plan `docs/superpowers/plans/2026-08-31-ranger-cr10-viz-collision-fixes.md`.

**Branch / HEAD:** `topay` @ `9dd76d3`.

**Plan status:** Tasks 0–4 closed for unit/docs/smoke-startup scope.

**Verified (docker `topay`, `-j2`, `roscore` for GridMap gates):**
- All Final Regression unit gates PASS (FK/grads/pose, colli_frame overlay+cylinder, ranger visual, wheels, box layout, box Cases A–D).
- Headless smoke `roslaunch planner run_ranger_cr10_smoke.launch rviz:=false` (~45s): `/moma/box_obstacle/*` loaded; `fake_moma` / `moma_vis` / `planner_node` start as `ranger_cr10`; log shows `Map ready`. Timeout kill was clean. Full interactive RViz planning demo not asserted here.
- `moma_traj_opt_falm.cpp` / `moma_traj_opt_relax.cpp` are alternate sources **not** listed in `planner/CMakeLists.txt`; production smoke uses compiled `moma_traj_opt.cpp` (base-obstacle wired).

**Uncommitted on purpose:** `map.pcd`; optional `.gitignore` IDE exception.

**Next (user):** optional RViz smoke for visual confirm; merge/PR when ready.
