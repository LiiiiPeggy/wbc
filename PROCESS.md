# Process

Engineering history for shared agent context. Keep entries that explain **why** the current design exists. Do not copy full plans or chat logs here.

Detailed plans/ledgers (optional deeper reading):
- `docs/superpowers/plans/2026-08-31-ranger-cr10-viz-collision-fixes.md`
- `.superpowers/sdd/2026-08-31-ranger-cr10-viz-collision-fixes/progress.md`

---

## 2026-08/09 — Ranger+CR10 visual root, collision overlay, box env envelope

### Background

Smoke on `topay` showed three persistent issues after baseline `f792a3a` (visual root split):

1. Drive-wheel meshes penetrated the ground.
2. Green RViz collision spheres looked “wrong” relative to CAD (planning vs visual frame confusion).
3. Upper cargo-box meshes could intersect mid-height obstacles in RViz while planning reported no collision.

### Attempts and failures

- **Per-mesh Z lift mixed into planning poses** (pre-`f792a3a`): hard to reason about; mixed CAD floor offset into kinematics consumers.
- **Raise only wheel mesh Z** (`-0.258→-0.122`) for ground clearance: wheels cleared ground but chassis/steering relative layout broke (wheels appeared embedded / chassis looked too low). Raising steering alone without the body still left the chassis sunk.
- **Treat green `/sphere` as a CAD bug and bake `+0.275` into planning collision APIs:** rejected — planning must stay at base `z=0`; would pollute GridMap / traj-opt.
- **Put box spheres into arm `collision_proxies_` / self-collision matrix:** rejected — box is an environment envelope, not self-collision geometry; would distort self-collision and link ownership.
- **Fix only GridMap discrete checks:** insufficient — continuous `MomaTrajOpt` ESDF cost also omitted base box, so optimized trajectories could still penetrate.
- **Raise random-map obstacle heights to hide the miss:** forbidden by plan constraints.

### Root causes

| Symptom | Root cause |
|---------|------------|
| Green spheres vs CAD | Planning collision is `T_owner * local_offset` at base `z=0`; CAD uses `applyVisualRoot`. Overlay must use visual owner × `local_offset`, not link origin. |
| Wheel ground penetration | With `Rx≈90°`, clearance is world-frame STL `zmin`, not local-z formulas. |
| Chassis low after wheel-only lift | `mesh_parts` are base-relative (not a live URDF tree). Ground clearance must lift the whole CAD via `visual.base_xyz`, keeping URDF-abs mesh offsets. |
| Box planning false negative | Chassis occupancy is 2D + `z < chassis_height` (~0.15). Box planning Z is higher → no 3D env envelope. |
| “Cannot turn off” blue spheres | Smoke runs both `fake_moma_node` and `moma_vis_node`; each publishes `/sphere_visual`. Disabling one leaves the other (stacked alpha looks darker; one left looks lighter). |

### Final design

1. **Frames:** planning base `z=0`; CAD/`sphere_visual` via `visual.base_xyz` (production ≈ `0.4113` = STL floor `0.275` + ground Δ `0.1363`). Never bake visual root into `getColliPts*` / `getBaseObstaclePts*` / GridMap / traj-opt.
2. **Wheels:** keep URDF-abs wheel/steering in `mesh_parts`; clear ground by raising `visual.base_xyz`.
3. **RViz:** green `/sphere` = planning truth; blue `/sphere_visual` = CAD overlay. Default: planning off; enable only `fake_moma` CAD overlay (disable moma_vis duplicate).
4. **Box env:** separate `base_obstacle_proxies_` + `getBaseObstaclePts` / `getBaseObstacleGrads` (base x/y/yaw only). Wire into GridMap whole-body and compiled `moma_traj_opt.cpp` ESDF path. Do not enter self-collision matrix. GridMap tests live in `map` (no `fake_moma↔map` cycle).
5. **CR10 arm FK:** left unchanged (`mount.relative_t`, joint fixed transforms, arm collision gradient chain for `link_id >= 1`).

### Verification

- Docker `topay`: original Final Regression unit gates PASS (incl. box Cases A–D, wheel ground, overlay/cylinder).
- Headless smoke: params/`box_obstacle` load, nodes start, `Map ready`. Interactive hard-map plan success not claimed.
- Commit trail on `topay` through visual-root lift / RViz dedupe (`b77bd0d` / docs `fefb389`); see plan + SDD ledger for task-level commits.

---

## 2026-09-06 — Test suite consolidation (planner-correctness only)

### Background

Post-Task-4 suite still carried viz-only and duplicated gates (`test_topay_cr10_fk`, `colli_frame`, `ranger_visual`, `box_collision` Case C/D reimplements). Goal: shrink to YAML-driven geometry + production optimizer path.

### Decisions

- Delete viz/sphere/cylinder regression binaries; keep implementations as RViz debug.
- Merge FK/EE + planning `getColliPtsCr10` transform + colli FD → `test_cr10_collision_proxy`.
- GridMap A/B only in `test_base_obstacle_collision`; box AABB coverage → `audit_ranger_geometry.py`.
- Continuous cost/grad → `test_optimizer_collision_gradient` calling linked `MomaTrajOpt::eeCostCallback` (not hand-written smoothL1).
- falm/relax remain source-sync, not CI-linked.

### Pitfall fixed in optimizer gate

yaw=0 at origin + obstacle at sphere center made ESDF analytic/FD disagree (lattice flat / center singularity). Fix: off-grid base xy, `resolution=0.05`, sphere-relative obstacle offset, `fd_eps=5e-4`, pre-size grad vectors, skip `opt.init()` (set `relu_mu` only).

---

## 2026-09-08/09 — Visual/box contract diagnosis (no blind Z lift)

### Symptoms

RViz smoke: Box/LiDAR looked sunk into chassis; Box env collisions felt wrong.

### Root causes (measured)

1. **Dual CAD markers:** both `/fake_moma_node/marker` and `/moma_vis_node/marker` Enabled.
2. **Not missing visual root:** `applyVisualRoot` applies uniformly; Ranger→Box/LiDAR/CR10 relative z invariant.
3. **Keep visual_root=0.4113** for ground contact; do not revert to 0.275 (wheels penetrate) and do not raise further.
4. **Box env under-coverage:** old 3×3×2 grid left ~0.11 m STL-surface holes on the box bottom; densify to 5×5×3 (R=0.18, margin 0.02).

### Final deltas

- `default.rviz`: moma_vis CAD marker OFF.
- `box_obstacle` grid densified; audit STL-vertex coverage + outward gates.
- GridMap Cases A–D tied to physical STL AABB / proxy hit.
