# Progress

**Goal:** Ranger + CR10 whole-body planning/visualization on branch `topay`.

**Branch / HEAD:** `topay` (pushing short-circuit / WallTime / B1+B2 / physical boxes).

**Current status:** Hard-gate runs only when `shouldRunWholeBodyTrajHardGate`; timings use `ros::WallTime`; bridge gate A/B1/B2/C; physical bridge boxes ≠ placement footprint.

**Verified:**
- `test_hard_gate_short_circuit` PASS
- `test_bridge_obstacle_clearance` A / B1 / B2 / C PASS
- `test_trajectory_collision_checker` A/B/C PASS
- `test_base_obstacle_collision` A–D PASS
- `test_optimizer_collision_gradient` PASS
- Smoke 5-goal: ≥2 succ with `[PlanTiming] ... hard=...`; `[SafeCheck]` WARN ~5–7 ms when >5 ms

**Rulings (unchanged):** no visual/box envelope/weight changes.

**Open:** do not commit `map.pcd` / `bin/` / `topay_current_review2.txt`. Commit when asked.
