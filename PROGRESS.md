# Progress

**Goal:** Ranger + CR10 whole-body planning/visualization on branch `topay`.

**Branch / HEAD:** `topay` @ `b83bad7`.

**Current status:** Diagnosed RViz Box/LiDAR “embed” + Box env collision; dual CAD marker deduped; box_obstacle densified for STL coverage; GridMap A–D redesigned.

**Verified:**
- `audit_ranger_geometry.py --test-all` PASS (wheels, relative frames, STL coverage hole≈1e-3, outward≤0.13).
- Docker: `test_cr10_collision_proxy`, `test_base_obstacle_collision` A–D, `test_optimizer_collision_gradient` PASS.

**Rulings:**
- Keep `visual.base_xyz.z=0.4113` (ground-correct); do not raise further / do not revert to 0.275 for wheels alone.
- Default RViz: only `/fake_moma_node/marker` CAD ON.

**Open:** `map.pcd` local dirty — do not commit.

**Next:** push when asked; optional interactive RViz re-check.
