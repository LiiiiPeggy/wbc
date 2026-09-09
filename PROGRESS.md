# Progress

**Goal:** Ranger + CR10 whole-body planning/visualization on branch `topay`.

**Branch / HEAD:** `topay` @ `d436094`.

**Current status:** Soft-opt traj could publish box-penetrating paths; unified hard validator wired into optimize / printConstraints / safeCallback.

**Verified:**
- `test_trajectory_collision_checker` A/B/C PASS.
- `test_base_obstacle_collision` A–D PASS; `test_optimizer_collision_gradient` PASS.
- Headless smoke: `box_obstacle` params load + `Map ready` (timeout kill OK).

**Rulings (unchanged this round):**
- Do not change visual root / box grid / `obstacle_radius` / collision weights.
- Hard safety = `checkWholeBodyTrajectoryCollision` → `GridMap::isWholeBodyCollision` only.

**Open:** `map.pcd` local dirty — do not commit.
