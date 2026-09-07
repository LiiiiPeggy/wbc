#!/usr/bin/env python3
# ################################
# Python: Ranger geometry regression (wheels, STL bounds, frames, box coverage)
# ################################
"""Production-YAML geometry gates for Ranger+CR10 (no hardcoded visual/box grid)."""

from __future__ import annotations

import argparse
import re
import struct
import sys
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import numpy as np
import yaml

REPO_ROOT_DEFAULT = Path(__file__).resolve().parents[1]
YAML_DEFAULT = REPO_ROOT_DEFAULT / "TopAY/src/planner/params/robot_ranger_cr10.yaml"
GROUND_Z = 0.0
WHEEL_GROUND_TOL = 0.01


def resolve_package_uri(uri: str, repo_root: Path) -> Path:
    if uri.startswith("package://fake_moma/"):
        rel = uri[len("package://fake_moma/") :]
        return repo_root / "TopAY/src/simulator/fake_moma" / rel
    raise ValueError(f"unsupported uri: {uri}")


def rpy_to_R(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def T_from_xyz_rpy(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(rpy)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def bounds_from_vertices(vertices: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if vertices.size == 0:
        return np.zeros(3), np.zeros(3)
    return vertices.min(axis=0), vertices.max(axis=0)


def update_bounds(vmin: np.ndarray, vmax: np.ndarray, point: Sequence[float]) -> None:
    p = np.asarray(point, dtype=float)
    vmin[:] = np.minimum(vmin, p)
    vmax[:] = np.maximum(vmax, p)


def transform_point(T: np.ndarray, point: Sequence[float]) -> np.ndarray:
    hom = np.array([point[0], point[1], point[2], 1.0], dtype=float)
    return (T @ hom)[:3]


def is_binary_stl(data: bytes) -> bool:
    # ################################
    # Python: Prefer header triangle_count size match over "solid" prefix heuristic
    # ################################
    if len(data) < 84:
        return False
    tri_count = struct.unpack("<I", data[80:84])[0]
    expected = 84 + tri_count * 50
    return expected == len(data) or (tri_count > 0 and len(data) >= expected)


def iter_stl_vertices(path: Path):
    data = path.read_bytes()
    if is_binary_stl(data):
        tri_count = struct.unpack("<I", data[80:84])[0]
        expected = 84 + tri_count * 50
        if len(data) < expected:
            tri_count = max(0, (len(data) - 84) // 50)
        offset = 84
        for _ in range(tri_count):
            if offset + 50 > len(data):
                break
            offset += 12
            tri = struct.unpack("<9f", data[offset : offset + 36])
            offset += 38
            for i in range(0, 9, 3):
                yield float(tri[i]), float(tri[i + 1]), float(tri[i + 2]), "binary"
        return

    text = data.decode("utf-8", errors="ignore")
    for line in text.splitlines():
        line = line.strip()
        if line.lower().startswith("vertex"):
            parts = line.split()
            if len(parts) >= 4:
                yield float(parts[1]), float(parts[2]), float(parts[3]), "ascii"


def stl_stream_world_bounds(path: Path, T_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, str, int]:
    local_vmin = np.array([np.inf, np.inf, np.inf])
    local_vmax = np.array([-np.inf, -np.inf, -np.inf])
    world_vmin = np.array([np.inf, np.inf, np.inf])
    world_vmax = np.array([-np.inf, -np.inf, -np.inf])
    vertex_count = 0
    fmt = "ascii"
    for x, y, z, f in iter_stl_vertices(path):
        fmt = f
        update_bounds(local_vmin, local_vmax, [x, y, z])
        update_bounds(world_vmin, world_vmax, transform_point(T_world, [x, y, z]))
        vertex_count += 1
    if vertex_count == 0:
        local_vmin = np.zeros(3)
        local_vmax = np.zeros(3)
        world_vmin = np.zeros(3)
        world_vmax = np.zeros(3)
    return local_vmin, local_vmax, world_vmin, world_vmax, fmt, vertex_count


def load_moma_profile(yaml_path: Path) -> dict:
    with yaml_path.open("r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    return doc["moma"]


def is_wheel_entry(entry: dict) -> bool:
    # ################################
    # Python: Drive wheels only (exclude *steering*_wheel_link)
    # ################################
    file_name = Path(entry.get("file", "")).name.lower()
    return bool(re.match(r"^(fr|fl|rl|rr)_wheel_link\.stl$", file_name))


def is_box_entry(entry: dict) -> bool:
    return "box_link.stl" in entry.get("file", "").lower()


def is_ranger_base_entry(entry: dict) -> bool:
    return "ranger_base_link.stl" in entry.get("file", "").lower()


def visual_root_T(profile: dict) -> np.ndarray:
    visual = profile.get("visual", {})
    xyz = visual.get("base_xyz", [0.0, 0.0, 0.0])
    rpy = visual.get("base_rpy", [0.0, 0.0, 0.0])
    return T_from_xyz_rpy(xyz, rpy)


def audit_mesh_entry(
    entry: dict,
    repo_root: Path,
    base_T: np.ndarray,
    profile: dict,
    apply_visual: bool,
) -> dict:
    stl_path = resolve_package_uri(entry["file"], repo_root)

    T_mesh = T_from_xyz_rpy(entry.get("xyz", [0, 0, 0]), entry.get("rpy", [0, 0, 0]))
    T_world = base_T @ T_mesh
    if apply_visual:
        # T_world = T_base * T_visual_root * T_mesh_part
        T_world = base_T @ visual_root_T(profile) @ T_mesh

    local_vmin, local_vmax, world_vmin, world_vmax, fmt, vertex_count = stl_stream_world_bounds(
        stl_path, T_world
    )
    ground_clearance = float(world_vmin[2] - GROUND_Z)
    penetration_depth = max(0.0, -ground_clearance)
    recommended_delta_z = -ground_clearance if abs(ground_clearance) > WHEEL_GROUND_TOL else 0.0

    return {
        "file": entry.get("file"),
        "path": str(stl_path),
        "format": fmt,
        "vertex_count": int(vertex_count),
        "local_bounds": (local_vmin, local_vmax),
        "world_bounds": (world_vmin, world_vmax),
        "local_vmin": local_vmin,
        "local_vmax": local_vmax,
        "world_vmin": world_vmin,
        "world_vmax": world_vmax,
        "world_zmin": float(world_vmin[2]),
        "ground_clearance": ground_clearance,
        "penetration_depth": penetration_depth,
        "recommended_delta_z": recommended_delta_z,
        "xyz": entry.get("xyz", [0, 0, 0]),
        "rpy": entry.get("rpy", [0, 0, 0]),
        "apply_visual": apply_visual,
    }


def fmt_vec(v: np.ndarray) -> str:
    return f"[{v[0]:.6f}, {v[1]:.6f}, {v[2]:.6f}]"


def print_mesh_report(title: str, result: dict) -> None:
    print(f"\n=== {title} ===")
    print(f"mesh: {result['file']}")
    print(f"path: {result['path']}")
    print(f"format: {result['format']} | vertex_count: {result['vertex_count']}")
    print(f"mesh xyz: {result['xyz']} | rpy: {result['rpy']} | visual_root_applied: {result['apply_visual']}")
    print(f"local_bounds min: {fmt_vec(result['local_vmin'])} max: {fmt_vec(result['local_vmax'])}")
    print(f"world_bounds min: {fmt_vec(result['world_vmin'])} max: {fmt_vec(result['world_vmax'])}")
    print(
        f"ground_clearance: {result['ground_clearance']:.6f} | "
        f"penetration_depth: {result['penetration_depth']:.6f} | "
        f"recommended_delta_z: {result['recommended_delta_z']:.6f}"
    )


def chassis_footprint_radius(x_len: float, y_len: float) -> float:
    return float(np.sqrt((x_len / 2.0) ** 2 + (y_len / 2.0) ** 2))


def load_box_spheres_from_yaml(profile: dict):
    # ################################
    # Python: Expand production box_obstacle (spheres[] or grid_*) — no hardcoded grids
    # ################################
    box = profile.get("box_obstacle", {})
    if not box or not box.get("enabled", False):
        raise AssertionError("box_obstacle.enabled must be true in production YAML")
    margin = float(box.get("margin", 0.0))
    spheres = []
    if box.get("spheres"):
        for entry in box["spheres"]:
            offset = np.asarray(entry["local_offset"], dtype=float)
            radius = float(entry["obstacle_radius"])
            spheres.append((offset, radius))
    else:
        grid_x = [float(v) for v in box["grid_x"]]
        grid_y = [float(v) for v in box["grid_y"]]
        grid_z = [float(v) for v in box["grid_z"]]
        radius = float(box["obstacle_radius"])
        for x in grid_x:
            for y in grid_y:
                for z in grid_z:
                    spheres.append((np.array([x, y, z], dtype=float), radius))
    if not spheres:
        raise AssertionError("box_obstacle produced zero spheres")
    if not any(abs(c[0]) > 1e-9 or abs(c[1]) > 1e-9 for c, _ in spheres):
        raise AssertionError("box_obstacle requires at least one sphere with nonzero local x/y")
    return spheres, margin


def sample_box_surface_points(local_vmin: np.ndarray, local_vmax: np.ndarray, n_edge: int = 8) -> np.ndarray:
    xs = np.linspace(local_vmin[0], local_vmax[0], n_edge)
    ys = np.linspace(local_vmin[1], local_vmax[1], n_edge)
    zs = np.linspace(local_vmin[2], local_vmax[2], n_edge)
    pts = []
    for y in ys:
        for z in zs:
            pts.append([local_vmin[0], y, z])
            pts.append([local_vmax[0], y, z])
    for x in xs:
        for z in zs:
            pts.append([x, local_vmin[1], z])
            pts.append([x, local_vmax[1], z])
    for x in xs:
        for y in ys:
            pts.append([x, y, local_vmin[2]])
            pts.append([x, y, local_vmax[2]])
    return np.asarray(pts, dtype=float)


def test_box_proxy_coverage_gate(repo_root: Path, yaml_path: Path) -> None:
    profile = load_moma_profile(yaml_path)
    base_T = np.eye(4)
    box_entry = next((e for e in profile.get("mesh_parts", []) if is_box_entry(e)), None)
    if box_entry is None:
        raise AssertionError("box_link.STL entry not found in mesh_parts")
    # Planning-frame AABB (matches base_obstacle_proxies_ frame; no visual root)
    box = audit_mesh_entry(box_entry, repo_root, base_T, profile, apply_visual=False)
    spheres, margin = load_box_spheres_from_yaml(profile)
    # ################################
    # Python: Dense sample of planning AABB surface (n_edge=3 ⇒ corners/edges/face centers)
    # ################################
    surface = sample_box_surface_points(box["world_vmin"], box["world_vmax"], n_edge=3)
    max_deficit = 0.0
    for world_pt in surface:
        nearest = min(np.linalg.norm(world_pt - center) - radius for center, radius in spheres)
        max_deficit = max(max_deficit, nearest - margin)
    if max_deficit > 1e-6:
        raise AssertionError(
            f"box AABB surface not covered by envelope: max_deficit={max_deficit:.6f} "
            f"(need distance <= radius + margin)"
        )
    print(
        f"Box STL coverage PASSED spheres={len(spheres)} margin={margin:.3f} "
        f"aabb_samples={len(surface)} max_deficit={max_deficit:.6e}"
    )


def test_visual_planning_frame_gate(repo_root: Path, yaml_path: Path) -> None:
    profile = load_moma_profile(yaml_path)
    visual = profile.get("visual", {})
    vxyz = np.asarray(visual.get("base_xyz", [0, 0, 0]), dtype=float)
    base_T = np.eye(4)
    base_entry = next((e for e in profile.get("mesh_parts", []) if is_ranger_base_entry(e)), None)
    if base_entry is None:
        raise AssertionError("ranger_base_link.STL not found")
    planning = audit_mesh_entry(base_entry, repo_root, base_T, profile, False)
    visual_r = audit_mesh_entry(base_entry, repo_root, base_T, profile, True)
    dz = visual_r["world_zmin"] - planning["world_zmin"]
    if abs(dz - float(vxyz[2])) > 1e-6:
        raise AssertionError(
            f"visual/planning zmin delta {dz:.6f} != visual.base_xyz.z {vxyz[2]:.6f}"
        )
    print(
        f"Visual/planning frame consistency PASSED "
        f"base_zmin_plan={planning['world_zmin']:.6f} "
        f"base_zmin_vis={visual_r['world_zmin']:.6f} delta={dz:.6f}"
    )


def print_mesh_report(title: str, result: dict) -> None:
    print(f"\n=== {title} ===")
    print(f"mesh: {result['file']}")
    print(f"path: {result['path']}")
    print(f"format: {result['format']} | vertex_count: {result['vertex_count']}")
    print(f"mesh xyz: {result['xyz']} | rpy: {result['rpy']} | visual_root_applied: {result['apply_visual']}")
    print(f"local_bounds min: {fmt_vec(result['local_vmin'])} max: {fmt_vec(result['local_vmax'])}")
    print(f"world_bounds min: {fmt_vec(result['world_vmin'])} max: {fmt_vec(result['world_vmax'])}")
    print(
        f"world_zmin: {result['world_zmin']:.6f} | "
        f"ground_clearance: {result['ground_clearance']:.6f} | "
        f"penetration_depth: {result['penetration_depth']:.6f}"
    )


def chassis_footprint_radius(x_len: float, y_len: float) -> float:
    return float(np.sqrt((x_len / 2.0) ** 2 + (y_len / 2.0) ** 2))


def run_audit(repo_root: Path, yaml_path: Path) -> int:
    profile = load_moma_profile(yaml_path)
    mesh_parts = profile.get("mesh_parts", [])
    chassis = profile.get("chassis", {})
    base_T = np.eye(4)
    visual = profile.get("visual", {})

    print("# Ranger Geometry Audit")
    print(f"repo_root: {repo_root}")
    print(f"yaml: {yaml_path}")
    print(f"visual.base_xyz: {visual.get('base_xyz')} base_rpy: {visual.get('base_rpy')}")
    print(
        f"chassis: length={chassis.get('length')} width={chassis.get('width')} "
        f"height={chassis.get('height')} collision_radius={chassis.get('collision_radius')}"
    )

    for entry in mesh_parts:
        if is_ranger_base_entry(entry):
            print_mesh_report("Ranger base STL (visual)", audit_mesh_entry(entry, repo_root, base_T, profile, True))
        elif is_box_entry(entry):
            print_mesh_report("Box STL (planning)", audit_mesh_entry(entry, repo_root, base_T, profile, False))
            print_mesh_report("Box STL (visual)", audit_mesh_entry(entry, repo_root, base_T, profile, True))
        elif is_wheel_entry(entry):
            print_mesh_report("Wheel STL (visual)", audit_mesh_entry(entry, repo_root, base_T, profile, True))
    return 0


def test_four_wheel_ground_gate(repo_root: Path, yaml_path: Path) -> None:
    profile = load_moma_profile(yaml_path)
    base_T = np.eye(4)
    wheels = [e for e in profile.get("mesh_parts", []) if is_wheel_entry(e)]
    if len(wheels) != 4:
        raise AssertionError(f"expected 4 wheel entries, found {len(wheels)}")

    for entry in wheels:
        result = audit_mesh_entry(entry, repo_root, base_T, profile, True)
        gc = result["ground_clearance"]
        assert -WHEEL_GROUND_TOL <= gc <= WHEEL_GROUND_TOL, (
            f"{entry['file']} ground_clearance={gc:.6f} outside ±{WHEEL_GROUND_TOL}"
        )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Ranger geometry regression from production YAML.")
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT_DEFAULT)
    parser.add_argument("--yaml", type=Path, default=YAML_DEFAULT)
    parser.add_argument("--test-wheels", action="store_true", help="Four-wheel ground gate")
    parser.add_argument("--test-box-layout", action="store_true", help="Alias for --test-box-coverage")
    parser.add_argument("--test-box-coverage", action="store_true", help="Box STL ⊂ proxy envelope")
    parser.add_argument("--test-frames", action="store_true", help="Visual vs planning frame consistency")
    parser.add_argument("--test-all", action="store_true", help="Run all geometry gates")
    args = parser.parse_args(argv)

    yaml_path = args.yaml if args.yaml.is_absolute() else args.repo_root / args.yaml
    run_wheels = args.test_wheels or args.test_all
    run_box = args.test_box_layout or args.test_box_coverage or args.test_all
    run_frames = args.test_frames or args.test_all
    if not (run_wheels or run_box or run_frames):
        return run_audit(args.repo_root, yaml_path)

    if run_wheels:
        test_four_wheel_ground_gate(args.repo_root, yaml_path)
        print("Wheel STL ground gate PASSED")
    if run_frames:
        test_visual_planning_frame_gate(args.repo_root, yaml_path)
    if run_box:
        test_box_proxy_coverage_gate(args.repo_root, yaml_path)
        print("Box STL coverage PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
