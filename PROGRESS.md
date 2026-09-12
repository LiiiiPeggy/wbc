# Progress

**Goal:** Ranger + CR10 whole-body planning/visualization on branch `topay`.

**Branch / HEAD:** `topay` (local uncommitted: bridge + plan timing + hard-gate 减卡).

**Current status:** Bridge clearance (lintel bottom) **1.5 m**; opening ~2.8–3.5 m. Wall-clock plan timings; hard-gate 减卡.

**Verified:**
- `test_bridge_obstacle_clearance` A/B/C PASS @ clearance=1.5 (2D free; home arm hits lintel; pillar collides).
- Prior: traj checker PASS; ordinary/bridge smoke `Map ready`.

**Rulings (unchanged):**
- Do not change visual root / box grid / `obstacle_radius` / collision weights.
- Traj collision authority = `checkWholeBodyTrajectoryCollision` only (not embedded in printConstraints).

**Open:** `map.pcd` local dirty — do not commit. Commit/push when asked.
