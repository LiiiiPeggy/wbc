# Progress

**Goal:** Ranger + CR10 whole-body planning/visualization on branch `topay` (TopAY smoke path).

**Branch / HEAD:** `topay` (test-suite consolidation commit).

**Current status:** Plan `docs/superpowers/plans/2026-08-31-ranger-cr10-viz-collision-fixes.md` Tasks 0–4 closed for unit/docs/headless-smoke scope. Follow-up consolidated regression suite is in tree (viz-only gates removed).

**Verified (consolidated suite):**
- Host: `scripts/audit_ranger_geometry.py --test-all` PASS (wheels, frames, box AABB); wrapper `audit_ranger_stl_bounds.py` forwards.
- Docker `topay` (+`roscore`): `test_cr10_collision_proxy` PASS; `test_base_obstacle_collision` (Cases A/B) PASS; `test_optimizer_collision_gradient` PASS (`max_rel_err≈1e-6`, yaw∈{0,0.7}).
- Production traj-opt path remains compiled `moma_traj_opt.cpp` only.
- CAD ground lift + RViz overlay defaults unchanged on prior HEAD (`fefb389`).

**Open / local only:**
- `TopAY/src/simulator/random_map_generator/env/map.pcd` — do not commit.
- Full interactive RViz planning demo success not asserted (headless startup only).

**Next:** merge/PR when ready; optional interactive RViz confirmation on hard maps.
