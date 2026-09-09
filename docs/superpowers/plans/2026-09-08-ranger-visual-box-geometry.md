# TopAY Ranger visual / Box geometry contract (2026-09-08)

> **For agentic workers:** Use superpowers:subagent-driven-development. Spec authority: user query 2026-09-08 + Phase A frame contracts in `MEMORY.md` / prior Ranger+CR10 plan.

**Goal:** Diagnose and minimally fix RViz smoke issues where Box/LiDAR appear sunk into Ranger chassis and Box falsely (or inconsistently) collides with env obstacles — by establishing layered **physical / planning / visual / sensor** contracts, not by blindly raising `visual.base_xyz.z`.

**Branch:** `topay` @ HEAD (must re-check before edits).  
**Workspace:** `.superpowers/sdd/2026-09-08-ranger-visual-box-geometry/`

## Observed symptoms (user, latest smoke)

1. Box and LiDAR not correctly lifted — meshes embedded in Ranger chassis.
2. Upper Box collides with environment obstacles (need classify A correct / B over-coverage / C frame mismatch).

## Global Constraints

- Do **not** first assume a numeric visual root; forbid blindly increasing `visual.base_xyz.z` or restoring an old value without geometry evidence.
- Prefer **wheel-specific visual correction** over raising the whole robot for tire ground clearance.
- Do **not** put Box into arm `collision_proxies_` / `collision_matrix` / self-collision; keep `base_obstacle_proxies_` environment-only.
- Do **not** modify CR10 math (`getLinkTransformsCr10`, `getFKPoseCr10`, `getEEGradsCr10`, mount `[0.2462,0,0.1]`) unless physical base contract is proven wrong.
- Do **not** change LiDAR **sensor** extrinsic (`~sim/lidar_*` / URDF `[0.52588,0,0.16587]`) to fix CAD mesh origin.
- Planning Box envelope must share the same physical base-frame height definition as the physical Box; visual offsets only compensate STL mesh origin.
- Production YAML (`robot_ranger_cr10.yaml`) is the only geometry source for audits/tests — no hardcoded grids/fixtures that diverge.
- Keep consolidated gates; do not reintroduce viz-only CI binaries.
- C++/YAML edits: language-appropriate `################################` fence before changed blocks.
- Prefer docker `topay` for C++ builds/tests; never commit `map.pcd`.
- falm/relax: source-sync if touching cost; verify linked `moma_traj_opt.cpp`.

## Architecture (required layering)

```text
physical Ranger base (planning z=0)
  ├── physical Box / LiDAR extrinsic / CR10 mount
  └── Box collision envelope (getBaseObstaclePts*)

visual: T_world_mesh = T_world_base * T_visual_root * T_base_mesh [* T_mesh_visual_correction]
sensor lidar: T_world_lidar = T_world_base_physical * T_base_lidar   (no visual root)
```

---

### Task 0: Baseline + frame/contract diagnosis (read-only + audit output)

**Files:** production YAML, `moma_param*`, `visual_transform_utils`, `moma_sim`/`moma_vis`, URDF, STLs, `default.rviz`, `audit_ranger_geometry.py`.

**Deliverables:**

1. Confirm `branch=topay` and HEAD.
2. Answer in report (evidence-backed):
   - planning base z=0 meaning
   - whether map/RViz z=0 is ground
   - Ranger STL mesh origin
   - why wheel needs ~0.1363
   - whether Box/LiDAR URDF install height is already in `mesh_parts`
   - whether visual root is double-applied or missed for Box/LiDAR
3. Check dual marker publishers (`fake_moma` vs `moma_vis`) and `default.rviz` Enabled flags.
4. Extend `audit_ranger_geometry.py` to dump zero-state production poses for ranger_base, box, lidar_link0, D435, CR10 base, 4 wheels: mesh xyz/rpy, visual root, final world origin, STL world bounds zmin/zmax.
5. Classify Box/LiDAR embed root cause: missing visual root vs mesh local origin vs dual-marker overlay.
6. Commit audit-only improvements if code changes; otherwise report-only.

**Gate:** script prints zero-state bounds table; dual-marker status documented.

---

### Task 1: RViz default CAD marker dedupe

**Files:** `fake_moma/launch/default.rviz` (and related if needed).

**Require:** Only one robot CAD `/marker` set Enabled by default (prefer `fake_moma_node`). Keep publishers; do not delete nodes.

**Gate:** RViz config shows single CAD marker Enabled; other Disabled.

---

### Task 2: Visual geometry fix (minimal, evidence-driven)

Based on Task 0 ruling:

- If Box/LiDAR Marker lacks visual root → fix `meshWorldVisualTransform` / role mapping / `updateMeshMarkers` so all base-mounted meshes use `T_world_base * T_visual_root * T_base_mesh`.
- If Marker poses already correct but STL sunk → mesh-local visual correction only (not global root).
- If chassis/Box/LiDAR/CR10 correct at visual_root≈0.275 and only wheels penetrate ground → **revert** whole-robot lift that was for wheels; apply **wheel-specific** visual correction so `|wheel_world_zmin| ≤ 0.01`.
- Preserve relative transforms `T_ranger_box`, `T_ranger_lidar`, `T_ranger_cr10` (regression in audit).

**Forbidden:** raising whole robot again solely for wheels; changing CR10 FK; changing lidar sensor extrinsic.

**Gate:** audit relative transforms PASS; wheel ground PASS; Box/LiDAR sit above chassis in world bounds.

---

### Task 3: Box physical STL ↔ proxy envelope audit + layout if needed

**Files:** `audit_ranger_geometry.py`, optionally YAML `box_obstacle` **only if** coverage/outward-margin fails.

- Read production `box_obstacle` grids/radius (no hardcode).
- Surface-sample `box_link.STL` at ~1–2 cm; require coverage `min_i ||p-c_i|| ≤ R` with ~0.01–0.02 margin.
- Report max coverage hole and max outward margin.
- Classify user “Box hits obstacle” as A/B/C; only then adjust proxy layout/radius minimally.

**Gate:** coverage PASS; outward-margin gate PASS (document thresholds).

---

### Task 4: Redesign `test_base_obstacle_collision` Cases A–D

Cases must use production Box physical bounds:

- A: safe outside physical Box + margin → false
- B: intersects physical Box STL envelope → true
- C: above physical_box_zmax + margin → false
- D: near but not touching; must not false-positive beyond configured margin

Keep GridMap in `map` package. Optimizer gate still via production `eeCostCallback`.

**Gate:** A–D PASS in docker with roscore.

---

### Task 5: Final regression + smoke + report

Run consolidated suite + headless smoke (`rviz:=false` acceptable for CI evidence; note RViz manual checklist).

Output the 15-point report from the user query. Update `PROGRESS.md` / `MEMORY.md` / `PROCESS.md` as needed.

**Gate:** all listed regression lines PASS or explicitly ruled with evidence.

---

## Final Regression Gates (must)

- CR10 FK/EE-grad; collision proxy transform; colli FD
- Wheel actual-YAML+STL ground
- Box STL ↔ proxy coverage + outward margin
- GridMap A–D
- Optimizer continuous cost/grad
- Ranger→Box/LiDAR/CR10 relative visual transforms
- LiDAR sensor extrinsic unchanged
- Headless smoke startup + box_obstacle load + Map ready
