# Memory

Durable project knowledge for all agents. Record only what future work still needs
to avoid repeated failures or incorrect rewrites. Current tasks → `PROGRESS.md`;
design history → `PROCESS.md`.

## Frames
- Planning base stays z=0; CAD/RViz uses `visual.base_xyz` (production Ranger ≈ `[0,0,0.4113]` = STL floor 0.275 + wheel ground Δ 0.1363). Collision planning truth is `T_owner * local_offset`; visual overlay is `applyVisualRoot(base_T, T_owner) * local_offset`. Do not bake visual root into planning APIs.
- CR10 `getColliPtsCr10()` matches that contract (not a Tracer-style frame bug). RViz `/sphere` uses `obstacle_radius` (0.10), not `self_radius`.

## Collision model
- Chassis 2D ESDF only sees low-Z obstacles (`z < chassis_height`, typically 0.15). Upper cargo box STL planning z ≈ `[0.073, 0.472]` had no env envelope → mid-height XY-overlap obstacles looked colliding in RViz but `isWholeBodyCollision` returned false.
- Fix: separate `base_obstacle_proxies_` (env only). Do **not** put box spheres into `collision_proxies_` / `collision_matrix` / self-collision. Layout: multi-sphere with margin (production YAML **5×5×3**, R=0.18, margin 0.02) so `box_link.STL` surface ⊆ envelope; old 3×3×2 left ~0.11 m holes on the box bottom face. At least one sphere with nonzero local x/y for yaw FD.
- Consumers that must see box env: GridMap whole-body + `MomaTrajOpt*` ESDF paths. FALM: add base cost as smooth-L1 penalty without new ALM constraint slots (count is hard-coded).
- Put GridMap box tests in the `map` package. fake_moma ↔ map dependency cycle is forbidden.
- Box planning AABB (production STL, planning frame) ≈ x[-0.606,0.600], y[-0.40,0.41], z[0.073,0.472]. Proxy outward beyond AABB up to ~R is expected; gate limit is `R + margin`.

## Geometry / wheels
- Drive-wheel STL entries match `^(fr|fl|rl|rr)_wheel_link\.stl$` only (exclude steering wheels). With Rx≈90°, ground clearance must be world-frame zmin, not local-z formulas.
- Keep mesh_parts at URDF abs (wheels ≈`-0.2583/-0.25995`, steering `0.0335`). Ground clearance comes from raising `visual.base_xyz`, not per-wheel z — raising only wheels leaves the chassis sunk and breaks steering→wheel offset.
- Do **not** raise `visual.base_xyz` further to “fix” Box/LiDAR look: production applies `applyVisualRoot` uniformly; Ranger→Box/LiDAR/CR10 relative transforms are invariant. Reverting to 0.275 makes wheels penetrate ground.
- LiDAR mesh local zmin≈-0.15 below joint is URDF/STL origin (hangs into chassis volume); do not change `~sim/lidar_*` sensor extrinsic to compensate CAD.
- RViz: green `/sphere` = planning; blue `/sphere_visual` = CAD. Smoke runs both `fake_moma_node` and `moma_vis_node`. Default: only `/fake_moma_node/marker` + `fake_moma` `sphere_visual` ON; **`/moma_vis_node/marker` OFF** (duplicate CAD). Turn both sphere_visual off to hide completely.

## Build / test hygiene
- Prefer docker container `topay` (`/home/topay` → TopAY). Host `TopAY/build` is often root-owned.
- `git commit -m` can fail on trailer hooks here; use `/usr/bin/git commit -F <msgfile>`.
- Never commit `TopAY/src/simulator/random_map_generator/env/map.pcd`.
- Standalone tests that call `ros::Time::now()` without a node can crash; CR10 visual/cylinder paths avoid stamping inside the API.
- GridMap `init` with `agent/mode != planner` calls `regenerateMap` and needs live ROS/param server — without roscore, tests hang and spam XmlRpc. Use `mode=planner` + `loadMap` for unit gates; still start `roscore` because timers/subscriptions contact the master.
- Default `catkin_make -j32` plus stuck tests can exhaust RAM/swap; use low `-j`, `timeout`, and kill leftover test binaries after interrupted sessions.
- `moma_traj_opt_falm.cpp` / `moma_traj_opt_relax.cpp` are not in `planner/CMakeLists.txt`; only `moma_traj_opt.cpp` is linked into `libplanner`. Keep falm/relax source-synced if editing collision costs, but verify the compiled path. Optimizer FD gate must link that production `.cpp` (no reimplemented smoothL1/ESDF).
- GridMap destructor must `delete` each `grid_node_map[i]` (allocated with `new GridNode()`), not `delete[]`, or standalone tests segfault on exit.
- Consolidated gates (replace deleted `test_topay_*`): `scripts/audit_ranger_geometry.py` (wheels/frames/box STL coverage + outward; `--dump-zero-poses`; `audit_ranger_stl_bounds.py` wraps it); `fake_moma/test_cr10_collision_proxy` (FK/EE + planning transform + colli FD); `map/test_base_obstacle_collision` (GridMap A–D); `planner/test_optimizer_collision_gradient` (`eeCostCallback` base x/y/yaw FD). Do not reintroduce sphere_visual/cylinder regression binaries — RViz debug only.
- Optimizer FD on GridMap ESDF: avoid yaw=0 at origin (voxel lattice singularity). Use off-grid base xy, resolution finer than FD step, obstacle offset from sphere center (not coincident), and `fd_eps ≪ resolution`. Skip full `opt.init()` — set `opt_param.relu_mu` only; pre-size grad vectors.
- Box continuous cost: place mid-height obstacle on the box side (±y), not on the CR10 arm mount (+x), or discrete mid hits can be arm-only while base ESDF cost stays zero.
- Docker `topay` has `DISPLAY=:0` and `/tmp/.X11-unix` mounted, but still needs host X auth (`xhost +local:docker` or cookie). Without it RViz dies with `No protocol specified` / xcb. Prefer `rviz:=false` in container, or run RViz on the host against the same `ROS_MASTER_URI`.
