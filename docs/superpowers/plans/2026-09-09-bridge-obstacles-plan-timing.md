# Bridge Obstacles + Plan Stage Timing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add chassis-passable bridge/arch obstacles (box/arm may hit the lintel), record per-stage plan timings to `TopAY/src/logs/*.log`, and reduce lag from duplicated/over-dense whole-body hard checks.

**Architecture:** Extend `random_map` with a `generateBridge` primitive (two pillars + lintel) and optional third `obs_num` count (default **0**). Add `PlanTimingLogger` writing `.log` lines with **wall-clock** plan latency. Sole trajectory collision authority remains `checkWholeBodyTrajectoryCollision`; `printConstraintsSituations` reports kinematics/limits only (no embedded hard sweep). Dedupe means remove the **extra** dense sweep from printConstraints, not delete the validator. Coarsen `safeCallback` via **YAML** resolution/period.

**Tech Stack:** C++14, ROS Noetic, PCL, Eigen, existing `nmoma_planner::random_map::Box` / `GridMap` / `Planner` / `checkWholeBodyTrajectoryCollision`.

**Spec:** Chat requirements (2026-09-09) + execution adjustments (same day):
1. Bridge = chassis can pass under; cargo box and arm may hit the beam.
2. Stage timings under `TopAY/src/logs/*.log`.
3. 减卡 = remove **duplicate** hard sweep; keep explicit validator on success path.
4. `trajectory_collision_checker` = **only** collision authority (do not bind hard validation into `printConstraintsSituations` long-term).
5. Success path: `optimizeTraj` → `printConstraintsSituations` → `checkWholeBodyTrajectoryCollision` → publish.
6. `safeCallback` resolution/period from YAML (not hard-coded every-N).
7. Parallel planner timings = **wall-clock**, not worker-sum.
8. Bridge count default **0**; dedicated bridge test map / enable flag — do **not** pollute ordinary smoke map.

## Global Constraints

- Do **not** change: `visual.base_xyz`, `box_obstacle.grid`, `obstacle_radius`, collision weights, `base_obstacle_proxies_` layout.
- Never commit `TopAY/src/simulator/random_map_generator/env/map.pcd`.
- Prefer docker container `topay` for TopAY builds/tests.
- Mark C++/YAML edits with `################################` comment fences.
- Runtime logs under `TopAY/src/logs/` must not be committed (`**/logs/` already in root `.gitignore`).
- Keep backward compatible YAML: `obs_num: [walls, floats]` with length 2 still works (bridge count defaults to 0).
- Default maps / ordinary smoke: **bridge count = 0** unless a dedicated bridge map or enable flag is used.
- Hard publish safety: **only** via `checkWholeBodyTrajectoryCollision` on the optimize success path (and parameterized `safeCallback`). `printConstraintsSituations` must **not** own trajectory collision authority.

---

## What “减卡” means (read this)

“减卡” = **reduce lag / stutter**, not remove safety.

After the hard-gate commit (`d436094`), expensive whole-body sweeps can stack:

| Site | What happens today | Cost driver |
|------|--------------------|-------------|
| `printConstraintsSituations` | dense kinematics report **plus** embedded `checkWholeBodyTrajectoryCollision` | ~75 box spheres × N samples (duplicate authority) |
| `planMoma*` success chain | calls `checkWholeBodyTrajectoryCollision` again @ `0.01` | second dense sweep |
| `safeCallback` loop | re-scans **full traj** @ `0.01` every control period | continuous background CPU |

减卡 in this plan (approved):

1. **Dedup:** remove the hard sweep **from** `printConstraintsSituations`. Keep **one** explicit `checkWholeBodyTrajectoryCollision` on the success path after printConstraints. Do **not** delete the validator.
2. **Canonical success path:**
   ```
   optimizeTraj()
   → printConstraintsSituations()          // limits / report only (no traj collision authority)
   → checkWholeBodyTrajectoryCollision() // sole collision authority
   → publish
   ```
3. **Coarsen `safeCallback`:** YAML params for sample resolution and check period (seconds), not hard-coded `kSafeCheckEveryN`.
4. **Time it:** log hard-check and stage **wall-clock** milliseconds.

Safety contract: colliding trajs must fail `checkWholeBodyTrajectoryCollision` and must not publish as success.

---

## File map

| File | Responsibility |
|------|----------------|
| `TopAY/src/simulator/random_map_generator/include/random_map_generator/random_map.hpp` | Declare `generateBridge`, bridge params, extend `init` |
| `TopAY/src/simulator/random_map_generator/src/random_map_generator.cpp` | Implement bridge geometry + emit into `generataRandomCaseAux` |
| `TopAY/src/simulator/random_map_generator/params/map.yaml` | Document `obs_num[2]`; **default bridges=0** |
| `TopAY/src/planner/params/map_ranger_cr10_bridge.yaml` (new) | Dedicated bridge test map (`obs_num` with bridges>0) |
| `TopAY/src/planner/launch/run_ranger_cr10_bridge.launch` (new, optional) | Load bridge map; do not change default smoke |
| `TopAY/src/planner/params/map_ranger_cr10_smoke.yaml` | Leave bridge count **0** / unchanged (no pollution) |
| `TopAY/src/planner/include/planner/plan_timing_logger.h` | Open/append `.log`, write stage lines |
| `TopAY/src/planner/src/plan_timing_logger.cpp` | Implementation |
| `TopAY/src/planner/params/` (planner YAML, e.g. agent section or `planner_node`) | `safe_check_resolution`, `safe_check_period` |
| `TopAY/src/planner/CMakeLists.txt` | Link logger + bridge test |
| `TopAY/src/planner/src/planner.cpp` | Wall-clock stage timers; success path keeps explicit hard check; YAML safeCallback |
| `TopAY/src/planner/include/planner/moma_traj_opt.h` | Remove embedded `checkWholeBodyTrajectoryCollision` from `printConstraintsSituations` (report arm/box distances may remain; authority moves out) |
| `TopAY/src/planner/src/test_bridge_obstacle_clearance.cpp` (new) | Bridge geometry unit gate |
| `TopAY/src/planner/src/test_trajectory_collision_checker.cpp` | Still PASS after dedup |
| `TopAY/src/logs/` | Runtime directory (gitignored); mkdir at runtime |
| `PROGRESS.md` / `MEMORY.md` / `PROCESS.md` | Snapshot after verification |

### Bridge geometry (planning frame, z=0 ground)

Chassis: `height=0.15`, `collision_radius≈0.711`. Box proxies: `grid_z ≈ [0.18, 0.30, 0.42]`.

```
        +------------------+  <- lintel (beam), thickness ~0.08–0.15
        |##################|
   #####|                  |#####
   # P1 #   OPENING        # P2 #  <- pillars from z=0 up to lintel bottom
   #####|                  |#####
   -----+---- ground ------+-----
```

- **Opening width** (clear span between pillars): default range **1.6–2.2 m** (> ~2× footprint need; must allow chassis XY through).
- **Clearance height** (lintel bottom z): default range **0.22–0.38 m** — **> chassis_height (0.15)** so 2D chassis ESDF does **not** stamp the beam as wall; overlaps box Z so box/arm can collide in 3D.
- **Pillars:** ground-rooted boxes; width/depth ~0.15–0.30 m; height = clearance.
- **Lintel:** box sitting on pillars, size spanning outer pillar faces; z from clearance to clearance+thickness.
- Orientation: yaw 0 or π/2 (axis-aligned first; optional small yaw later — YAGNI: start axis-aligned).

Because beam z ≥ clearance > `chassis_height`, GridMap 2D occupancy ignores the beam (only `z < chassis_height`), so JPS/chassis search can go through the arch; whole-body / box hard gate still sees the beam.

---

### Task 1: Bridge generator + map YAML

**Files:**
- Modify: `TopAY/src/simulator/random_map_generator/include/random_map_generator/random_map.hpp`
- Modify: `TopAY/src/simulator/random_map_generator/src/random_map_generator.cpp`
- Modify: `TopAY/src/simulator/random_map_generator/params/map.yaml` (document bridges; **default count 0**)
- Create: `TopAY/src/planner/params/map_ranger_cr10_bridge.yaml` (bridges > 0)
- Optional create: `TopAY/src/planner/launch/run_ranger_cr10_bridge.launch`
- Do **not** raise bridge count in `map_ranger_cr10_smoke.yaml`
- Test: `TopAY/src/planner/src/test_bridge_obstacle_clearance.cpp` (added in Task 2; Task 1 ends with compileable API)

**Interfaces:**
- Consumes: existing `Box`, `generatePCL`, `obs_boxes` overlap checks
- Produces:
  ```cpp
  // In RandomPCGenerator:
  std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>>
  generateBridge(const Eigen::Vector3d& pos,   // center of opening on ground
                 double opening_width,
                 double opening_depth,         // pillar thickness along passage
                 double clearance_height,      // lintel bottom z
                 double lintel_thickness,
                 double pillar_width,
                 double yaw);                  // 0 or M_PI/2 initially

  // obs_num semantics after change:
  // obs_num[0] = walls, obs_num[1] = floats, obs_num[2] = bridges (optional; default 0)
  int bridgeCount() const; // returns obs_num.size() > 2 ? obs_num[2] : 0;
  ```

- [ ] **Step 1: Extend header with bridge params + `generateBridge` declaration**

Add members (ranges loaded from ROS):
```cpp
vector<double> bridge_opening_width_range = {1.6, 2.2};
vector<double> bridge_opening_depth_range = {0.20, 0.40};
vector<double> bridge_clearance_range = {0.22, 0.38};  // lintel bottom
vector<double> bridge_lintel_thickness_range = {0.08, 0.15};
vector<double> bridge_pillar_width_range = {0.15, 0.30};
uniform_real_distribution<double> rand_bridge_opening_width;
// ... same for other ranges
uniform_int_distribution<int> rand_bridge_axis; // 0 => yaw=0, 1 => yaw=pi/2
```

- [ ] **Step 2: Implement `generateBridge`**

```cpp
// Pseudocode — three boxes in local frame then rotate by yaw about z:
// Pillar L: pos = center + R * (-opening_width/2 - pillar_width, -opening_depth/2, 0)
//           size = (pillar_width, opening_depth, clearance_height)
// Pillar R: mirrored +x
// Lintel:   pos = center + R * (-opening_width/2 - pillar_width, -opening_depth/2, clearance_height)
//           size = (opening_width + 2*pillar_width, opening_depth, lintel_thickness)
// Return merged PCL + each Box::toArray()
```

Use existing `Box::generatePCL(resolution)` and `overlap` / `overlap2d` with spawn box + previous obstacles (same pattern as walls in `generataRandomCaseAux`).

- [ ] **Step 3: Wire into `generataRandomCaseAux` after walls/floats**

```cpp
const int n_bridge = (obs_num.size() > 2) ? obs_num[2] : 0;
for (int j = 0; j < n_bridge; ++j) {
  // sample center, dims, yaw in {0, pi/2}
  // build AABB Box covering whole bridge footprint for overlap test
  // on overlap with obs_boxes or spawn_box: j--; continue;
  // append generateBridge cloud + push constituent boxes into obs_boxes
}
```

- [ ] **Step 4: Update `init(ros::NodeHandle&)` to load bridge params**

```cpp
nh.param<std::vector<double>>("/map/bridge_opening_width_range", ...);
// ... all bridge ranges
// distributions from ranges; if a range missing, keep defaults above
```

- [ ] **Step 5: Update YAML (default bridges=0; separate bridge map)**

`map.yaml` (default / ordinary maps — bridges off):
```yaml
map:
  # obs_num: [walls, floats, bridges]  — bridges default 0
  obs_num: [80, 80, 0]
  # or keep length-2 [80, 80] for backward compat
  bridge_opening_width_range: [1.6, 2.2]
  bridge_opening_depth_range: [0.20, 0.40]
  bridge_clearance_range: [0.22, 0.38]
  bridge_lintel_thickness_range: [0.08, 0.15]
  bridge_pillar_width_range: [0.15, 0.30]
  # optional enable flag (if used, only generate when true AND obs_num[2]>0)
  bridge_enable: false
```

New `map_ranger_cr10_bridge.yaml` (dedicated):
```yaml
map:
  obs_num: [20, 5, 4]
  bridge_enable: true
  # ... same bridge ranges ...
```

Ordinary smoke (`map_ranger_cr10_smoke.yaml`): leave `obs_num` length-2 or `[..., 0]`; **do not** enable bridges by default.

- [ ] **Step 6: Build `random_map_generator` + `map` in docker**

Run:
```bash
docker exec topay bash -lc 'source /opt/ros/noetic/setup.bash && source /home/topay/devel/setup.bash && cd /home/topay && timeout 300 catkin_make -j2 --pkg random_map_generator'
```
Expected: build success.

- [ ] **Step 7: Commit**

```bash
git add TopAY/src/simulator/random_map_generator/include/random_map_generator/random_map.hpp \
        TopAY/src/simulator/random_map_generator/src/random_map_generator.cpp \
        TopAY/src/simulator/random_map_generator/params/map.yaml \
        TopAY/src/planner/params/map_ranger_cr10_bridge.yaml
# optional launch file if created
# /usr/bin/git commit -F msgfile
# msg: feat(map): add chassis-passable bridge/arch obstacles (default count 0)
---

### Task 2: Bridge clearance unit gate

**Files:**
- Create: `TopAY/src/planner/src/test_bridge_obstacle_clearance.cpp`
- Modify: `TopAY/src/planner/CMakeLists.txt`
- Test: same binary via `rosrun planner test_bridge_obstacle_clearance`

**Interfaces:**
- Consumes: `RandomPCGenerator::generateBridge`, `GridMap::loadMap` / `isWholeBodyCollision`, production `MomaParam` from `robot_ranger_cr10.yaml`
- Produces: PASS/FAIL Cases A–C below

- [ ] **Step 1: Write failing test skeleton (Cases A–C)**

```cpp
// Case A: chassis-only at bridge center, home joints → isWholeBodyCollision == false
//         (opening free at z < chassis_height; beam above chassis)
// Case B: same XY, but assert at least one base_obstacle sphere has ESDF < radius
//         under the lintel (box hits beam) → isWholeBodyCollision == true
//         OR construct state with box under beam: if Case A free and beam stamped in 3D,
//         home pose under lintel should collide via getBaseObstaclePts
// Case C: chassis state shifted into a pillar footprint → isWholeBodyCollision == true
```

Concrete setup:
```cpp
// Bridge at origin, opening_width=2.0, clearance=0.28, lintel_thickness=0.10, pillar_width=0.20, yaw=0
// Stamp generateBridge cloud into GridMap occ buffers (z < chassis_height → 2D) like other tests
Eigen::VectorXd home = Zero(3+dof);
// Case A: home at (0,0,0) under opening
// Case B: expect collision true at home if lintel overlaps box z (clearance 0.28 overlaps grid_z)
// Case C: home at pillar center world xy
```

If Case A and B conflict for the same home pose: that is the intended design (chassis free in 2D narrative, whole-body true because box). So:
- Case A checks **2D chassis path intent**: `!isCollision2d(center, chassis_radius)` under opening.
- Case B checks **whole-body**: `isWholeBodyCollision(home)` true under opening (box vs lintel).
- Case C: whole-body true at pillar.

- [ ] **Step 2: Run test — expect FAIL (binary missing / cases not green)**

Run:
```bash
docker exec topay bash -lc '... rosrun planner test_bridge_obstacle_clearance'
```
Expected: FAIL or not found until Steps 3–4.

- [ ] **Step 3: Add CMake target + fix generator/stamp helpers until A–C PASS**

Mirror `test_base_obstacle_collision.cpp` map stamping from PCL points of `generateBridge`.

- [ ] **Step 4: Run test — expect PASS**

```
Case A chassis-opening 2D free PASSED
Case B box-vs-lintel whole-body collision PASSED
Case C pillar collision PASSED
```

- [ ] **Step 5: Commit**

```bash
# msg: test(map): gate bridge opening free for chassis and lintel hit for box
```

---

### Task 3: Plan timing logger (`.log` under `TopAY/src/logs`)

**Files:**
- Create: `TopAY/src/planner/include/planner/plan_timing_logger.h`
- Create: `TopAY/src/planner/src/plan_timing_logger.cpp`
- Modify: `TopAY/src/planner/CMakeLists.txt`
- Modify: `TopAY/src/planner/src/planner.cpp` (`planMomaParallel` + goal callback)
- Modify: `TopAY/src/planner/include/planner/planner.h` (member logger)

**Interfaces:**
- Consumes: `ros::package::getPath("planner")` or absolute `TopAY/src/logs` via env/package path
- Produces:
  ```cpp
  class PlanTimingLogger {
  public:
    // Creates TopAY/src/logs/ if needed; opens plan_YYYYMMDD_HHMMSS.log (append-safe)
    void startSession();  // call once in Planner::init
    void logPlan(const std::string& tag,
                 double topo_ms, double mcrrt_ms, double opt_ms,
                 double hard_ms, double total_ms, bool success);
  };
  ```

**Log format (`.log`, one record per plan attempt):**
```
# session_start=2026-09-09T21:00:00Z robot=ranger_cr10
2026-09-09T21:01:12.345Z plan tag=moma_parallel succ=1 topo_ms=12.3 mcrrt_ms=45.6 opt_ms=210.0 hard_ms=35.1 total_ms=310.2
2026-09-09T21:01:40.100Z plan tag=moma_parallel succ=0 topo_ms=10.1 mcrrt_ms=80.0 opt_ms=0.0 hard_ms=0.0 total_ms=95.5
```

Path resolution (prefer in this order):
1. If `planner` package path is `.../TopAY/src/planner`, write to `.../TopAY/src/logs/`
2. Else `ros::package::getPath("planner") + "/../logs"` (sibling of `planner` under `src/`)
3. `mkdir -p` before open; on failure PRINT_RED once and no-op (do not crash planner)

- [ ] **Step 1: Implement logger header/cpp + CMake**

- [ ] **Step 2: Instrument `planMomaParallel` with wall-clock stage timers**

Measure **wall-clock latency of the whole plan call**, not the sum of worker CPU times:

```cpp
ros::Time plan_t0 = ros::Time::now();
ros::Time t0 = plan_t0;
// topo_prm->findTopoPaths / JPS fallback
double topo_ms = (ros::Time::now()-t0).toSec()*1000;

t0 = ros::Time::now();
// parallel workers: wait until first success / all done (existing sync)
// mcrrt_ms + opt_ms may be attributed to the winning path's wall span:
//   from thread-start of selected worker to its finish, OR
//   from end of topo to end of optimize barrier (preferred single wall segment)
double search_opt_wall_ms = (ros::Time::now()-t0).toSec()*1000;
// Optionally split mcrrt_ms / opt_ms only if measured inside the winning worker
// as wall intervals on that thread; NEVER sum across workers.

t0 = ros::Time::now();
bool hard_ok = checkWholeBodyTrajectoryCollision(grid_map, end_traj, 0.01);
double hard_ms = (ros::Time::now()-t0).toSec()*1000;

double total_ms = (ros::Time::now()-plan_t0).toSec()*1000;
timing_logger_.logPlan("moma_parallel", topo_ms, mcrrt_ms, opt_ms, hard_ms, total_ms, succ);
PRINT_GREEN("[PlanTiming] topo=" << topo_ms << " mcrrt=" << mcrrt_ms
            << " opt=" << opt_ms << " hard=" << hard_ms
            << " total=" << total_ms << " ms succ=" << succ);
```

Document in code comment:
```cpp
// ################################
// C++: PlanTiming uses wall-clock latency (parallel barrier), not sum of worker CPUs
// ################################
```

Also keep console print so RViz users see timings without opening the file.

- [ ] **Step 3: Manual verify log file appears**

Run headless smoke / one plan; expect file:
`TopAY/src/logs/plan_*.log` with at least one `plan tag=` line.

- [ ] **Step 4: Commit**

```bash
# msg: feat(planner): write per-stage plan timings to TopAY/src/logs/*.log
```

---

### Task 4: 减卡 — remove duplicate hard sweep; keep explicit validator

**Files:**
- Modify: `TopAY/src/planner/include/planner/moma_traj_opt.h` — remove `checkWholeBodyTrajectoryCollision` call from `printConstraintsSituations` (may keep arm/box **distance report** lines; they must not be the sole reject authority for traj collision — feasible may still use distance thresholds for reporting, but trajectory hard authority is the checker on the success path)
- Modify: `TopAY/src/planner/src/planner.cpp` — keep success path:
  ```
  optimizeTraj()
  && printConstraintsSituations(traj)
  && checkWholeBodyTrajectoryCollision(grid_map, traj, res)
  && traj.is_init
  ```
- Modify: planner YAML + `Planner::init` — load `safe_check_resolution`, `safe_check_period`
- Verify: `test_trajectory_collision_checker`, `test_bridge_obstacle_clearance`

**Interfaces:**
- Consumes: `checkWholeBodyTrajectoryCollision` as sole traj collision authority
- Produces: one dense hard sweep on publish path; parameterized safeCallback; printConstraints without embedded checker call

- [ ] **Step 1: Strip hard authority from `printConstraintsSituations`**

Remove:
```cpp
if (!checkWholeBodyTrajectoryCollision(grid_map, traj, res)) {
  feasible = false;
  ...
}
```
Keep optional min-distance **prints** for arm/box if useful for debugging. Add fence:
```cpp
// ################################
// C++: Trajectory collision authority is checkWholeBodyTrajectoryCollision on optimize success path
// ################################
```
Also drop `#include "planner/trajectory_collision_checker.h"` from `moma_traj_opt.h` if no longer needed.

- [ ] **Step 2: Keep explicit validator on all optimize success sites**

Canonical form (do **not** delete this call):
```cpp
_succ =
    this->traj_opters[idx]->optimizeTraj(...)
    && this->traj_opters[idx]->printConstraintsSituations(traj)
    // ################################
    // C++: Sole whole-body traj hard gate (trajectory_collision_checker)
    // ################################
    && checkWholeBodyTrajectoryCollision(this->grid_map, traj, 0.01)
    && this->traj_opters[idx]->getTraj().is_init;
```

Goal of Task 4: delete the **duplicate** sweep that lived inside printConstraints, not the validator.

- [ ] **Step 3: Parameterize `safeCallback` via YAML**

Planner params (e.g. under `planner_node` / `agent`):
```yaml
agent:
  safe_check_resolution: 0.05   # seconds along traj
  safe_check_period: 0.10       # wall seconds between full traj rechecks
```

```cpp
// members: double safe_check_resolution_{0.05}; double safe_check_period_{0.10};
// safeCallback:
static ros::Time last_check;
if ((ros::Time::now() - last_check).toSec() < tsvr->safe_check_period_) {
  // sleep as today; skip heavy check
} else {
  last_check = ros::Time::now();
  if (!checkWholeBodyTrajectoryCollision(
          tsvr->grid_map, tsvr->end_traj, tsvr->safe_check_resolution_))
    tsvr->is_safe = false;
}
```

No hard-coded `kSafeCheckEveryN`.

- [ ] **Step 4: Re-run regressions**

```bash
rosrun planner test_trajectory_collision_checker   # A/B/C PASS
rosrun planner test_bridge_obstacle_clearance      # A/B/C PASS
rosrun map test_base_obstacle_collision            # still PASS
```

- [ ] **Step 5: Confirm logs show single hard_ms per plan (wall-clock total)**

- [ ] **Step 6: Commit**

```bash
# msg: perf(planner): keep explicit traj hard gate; drop printConstraints duplicate sweep; YAML safeCallback
```

---

### Task 5: Docs + smoke checklist

**Files:**
- Modify: `PROGRESS.md`, `MEMORY.md`, `PROCESS.md`

- [ ] **Step 1: Update PROGRESS** — HEAD, bridge+timing+减卡 status, verified commands

- [ ] **Step 2: Update MEMORY** — durable notes:
  - Bridge clearance > `chassis_height` so 2D search passes; box hits lintel in 3D
  - `obs_num[2]` bridges; default 0; dedicated `map_ranger_cr10_bridge.yaml`
  - Timings in `TopAY/src/logs/plan_*.log` (gitignored); parallel = wall-clock
  - Traj collision authority = `checkWholeBodyTrajectoryCollision` only; success path keeps explicit call after printConstraints
  - `safe_check_resolution` / `safe_check_period` YAML for safeCallback

- [ ] **Step 3: Update PROCESS** — short history (duplicate hard sweep lag; authority split)

- [ ] **Step 4: Headless smoke (ordinary + optional bridge)**

```bash
# ordinary smoke — must NOT require bridges
timeout 45 roslaunch planner run_ranger_cr10_smoke.launch rviz:=false
# bridge map (if launch added)
timeout 45 roslaunch planner run_ranger_cr10_bridge.launch rviz:=false
```
Expect: ordinary `Map ready` with bridges=0; bridge launch shows arches; `TopAY/src/logs/plan_*.log` after a plan.

- [ ] **Step 5: Commit docs**

```bash
# msg: docs: record bridge obstacles, plan timing logs, and hard-check 减卡
```

---

## Self-review

1. **Spec coverage**
   - Chassis-passable / box-may-hit bridge → Task 1–2
   - Stage timings `.log` → Task 3 (wall-clock)
   - 减卡 = drop printConstraints duplicate, keep explicit validator → Task 4
   - YAML safeCallback → Task 4
   - Bridge default 0 + dedicated map → Task 1 / 5
   - Sole authority = `trajectory_collision_checker` → Task 4

2. **Placeholder scan** — no TBD; concrete ranges, signatures, log line format, commands included.

3. **Type consistency** — `generateBridge(...)`, `PlanTimingLogger::logPlan(...)`, success-path checker call, YAML `safe_check_*` match across tasks.

4. **Out of scope** — continuous box sweep; changing collision weights/radius/visual root; committing `map.pcd` or log files; polluting default smoke with bridges.

---

## Suggested verification matrix

| Check | Command / evidence |
|-------|--------------------|
| Bridge unit | `rosrun planner test_bridge_obstacle_clearance` |
| Traj hard gate still OK | `rosrun planner test_trajectory_collision_checker` |
| GridMap box | `rosrun map test_base_obstacle_collision` |
| Timing file | `ls TopAY/src/logs/plan_*.log` + read one line |
| Console timing | `[PlanTiming] topo=...` in rosout |
| Smoke | `Map ready` + optional RViz arches |

---

Plan complete and saved to `docs/superpowers/plans/2026-09-09-bridge-obstacles-plan-timing.md`.

**Two execution options (after you approve the plan):**

1. **Subagent-Driven (recommended)** — fresh subagent per task, review between tasks  
2. **Inline Execution** — execute in this session with executing-plans checkpoints  

Which approach — or what should change in the plan first?
