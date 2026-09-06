# Memory

## Frames
- Planning base stays z=0; CAD/RViz uses `visual.base_xyz ≈ [0,0,0.275]`. Collision planning truth is `T_owner * local_offset`; visual overlay is `applyVisualRoot(base_T, T_owner) * local_offset`. Do not “fix” green spheres by adding 0.275 in planning APIs.
- CR10 `getColliPtsCr10()` matches that contract (not a Tracer-style frame bug). RViz `/sphere` uses `obstacle_radius` (0.10), not `self_radius`.

## Collision model
- Chassis 2D ESDF only sees low-Z obstacles (`z < chassis_height`, typically 0.15). Upper cargo box STL planning z ≈ `[0.073, 0.472]` had no env envelope → mid-height XY-overlap obstacles looked colliding in RViz but `isWholeBodyCollision` returned false.
- Fix: separate `base_obstacle_proxies_` (env only). Do **not** put box spheres into `collision_proxies_` / `collision_matrix` / self-collision. Layout: multi-sphere with margin (production YAML 3×3×2, R=0.18, margin 0.02); at least one sphere with nonzero local x/y for yaw FD.
- Consumers that must see box env: GridMap whole-body + `MomaTrajOpt*` ESDF paths. FALM: add base cost as smooth-L1 penalty without new ALM constraint slots (count is hard-coded).
- Put GridMap box tests in the `map` package. fake_moma ↔ map dependency cycle is forbidden.

## Geometry / wheels
- Drive-wheel STL entries match `^(fr|fl|rl|rr)_wheel_link\.stl$` only (exclude steering wheels). With Rx≈90°, ground clearance must be world-frame zmin, not local-z formulas.
- Wheel z visual-only correction to ≈`-0.122` cleared ground gate (±1 cm). Do not mix penetration depth with clearance semantics.

## Build / test hygiene
- Prefer docker container `topay` (`/home/topay` → TopAY). Host `TopAY/build` is often root-owned.
- `git commit -m` can fail on trailer hooks here; use `/usr/bin/git commit -F <msgfile>`.
- Never commit `TopAY/src/simulator/random_map_generator/env/map.pcd`.
- Standalone tests that call `ros::Time::now()` without a node can crash; CR10 visual/cylinder paths avoid stamping inside the API.
- GridMap `init` with `agent/mode != planner` calls `regenerateMap` and needs live ROS/param server — without roscore, tests hang and spam XmlRpc. Use `mode=planner` + `loadMap` for unit gates; still start `roscore` because timers/subscriptions contact the master.
- Default `catkin_make -j32` plus stuck tests can exhaust RAM/swap; use low `-j`, `timeout`, and kill leftover `test_topay_*` after interrupted sessions.
- `moma_traj_opt_falm.cpp` / `moma_traj_opt_relax.cpp` are not in `planner/CMakeLists.txt`; only `moma_traj_opt.cpp` is linked into `libplanner`. Keep falm/relax source-synced if editing collision costs, but verify the compiled path.
- GridMap destructor must `delete` each `grid_node_map[i]` (allocated with `new GridNode()`), not `delete[]`, or standalone tests segfault on exit.
- Box continuous Case D: place mid-height obstacle on the box side (±y), not on the CR10 arm mount (+x), or discrete mid hits can be arm-only while base ESDF cost stays zero.
