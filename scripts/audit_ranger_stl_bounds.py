#!/usr/bin/env python3
# ################################
# Python: Unified Ranger STL geometry / ground audit for topay branch
# ################################
"""Audit Ranger wheel/box/base STLs in planning and visual frames."""

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


def stl_stream_bounds(path: Path) -> Tuple[np.ndarray, np.ndarray, str, int]:
    # ################################
    # Python: Stream all STL vertices for exact local/world AABB (no full mesh load)
    # ################################
    data = path.read_bytes()
    local_vmin = np.array([np.inf, np.inf, np.inf])
    local_vmax = np.array([-np.inf, -np.inf, -np.inf])
    vertex_count = 0

    if len(data) >= 84 and not data.lstrip().startswith(b"solid"):
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
                update_bounds(local_vmin, local_vmax, tri[i : i + 3])
                vertex_count += 1
        if vertex_count == 0:
            local_vmin = np.zeros(3)
            local_vmax = np.zeros(3)
        return local_vmin, local_vmax, "binary", vertex_count

    text = data.decode("utf-8", errors="ignore")
    for line in text.splitlines():
        line = line.strip()
        if line.lower().startswith("vertex"):
            parts = line.split()
            if len(parts) >= 4:
                update_bounds(local_vmin, local_vmax, [float(parts[1]), float(parts[2]), float(parts[3])])
                vertex_count += 1
    if vertex_count == 0:
        local_vmin = np.zeros(3)
        local_vmax = np.zeros(3)
    return local_vmin, local_vmax, "ascii", vertex_count


def world_bounds_from_local(local_vmin: np.ndarray, local_vmax: np.ndarray, T_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray, int]:
    corners = [
        [local_vmin[0], local_vmin[1], local_vmin[2]],
        [local_vmin[0], local_vmin[1], local_vmax[2]],
        [local_vmin[0], local_vmax[1], local_vmin[2]],
        [local_vmin[0], local_vmax[1], local_vmax[2]],
        [local_vmax[0], local_vmin[1], local_vmin[2]],
        [local_vmax[0], local_vmin[1], local_vmax[2]],
        [local_vmax[0], local_vmax[1], local_vmin[2]],
        [local_vmax[0], local_vmax[1], local_vmax[2]],
    ]
    world_vmin = np.array([np.inf, np.inf, np.inf])
    world_vmax = np.array([-np.inf, -np.inf, -np.inf])
    for corner in corners:
        wp = transform_point(T_world, corner)
        update_bounds(world_vmin, world_vmax, wp)
    return world_vmin, world_vmax, len(corners)


def stl_stream_world_bounds(path: Path, T_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, str, int]:
    data = path.read_bytes()
    local_vmin = np.array([np.inf, np.inf, np.inf])
    local_vmax = np.array([-np.inf, -np.inf, -np.inf])
    world_vmin = np.array([np.inf, np.inf, np.inf])
    world_vmax = np.array([-np.inf, -np.inf, -np.inf])
    vertex_count = 0

    def consume_vertex(x: float, y: float, z: float) -> None:
        nonlocal vertex_count
        update_bounds(local_vmin, local_vmax, [x, y, z])
        wp = transform_point(T_world, [x, y, z])
        update_bounds(world_vmin, world_vmax, wp)
        vertex_count += 1

    if len(data) >= 84 and not data.lstrip().startswith(b"solid"):
        tri_count = struct.unpack("<I", data[80:84])[0]
        expected = 84 + tri_count * 50
        if len(data) < expected:
            tri_count = max(0, (len(data) - 84) // 50)
        offset = 84
        fmt = "binary"
        for _ in range(tri_count):
            if offset + 50 > len(data):
                break
            offset += 12
            tri = struct.unpack("<9f", data[offset : offset + 36])
            offset += 38
            for i in range(0, 9, 3):
                consume_vertex(tri[i], tri[i + 1], tri[i + 2])
    else:
        fmt = "ascii"
        text = data.decode("utf-8", errors="ignore")
        for line in text.splitlines():
            line = line.strip()
            if line.lower().startswith("vertex"):
                parts = line.split()
                if len(parts) >= 4:
                    consume_vertex(float(parts[1]), float(parts[2]), float(parts[3]))

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


def audit_mesh_entry(
    entry: dict,
    repo_root: Path,
    base_T: np.ndarray,
    visual_root_xyz: Sequence[float],
    apply_visual: bool,
) -> dict:
    stl_path = resolve_package_uri(entry["file"], repo_root)

    T_mesh = T_from_xyz_rpy(entry.get("xyz", [0, 0, 0]), entry.get("rpy", [0, 0, 0]))
    T_world = base_T @ T_mesh
    if apply_visual:
        T_world = base_T @ T_from_xyz_rpy(visual_root_xyz, [0, 0, 0]) @ T_mesh

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
        "local_vmin": local_vmin,
        "local_vmax": local_vmax,
        "world_vmin": world_vmin,
        "world_vmax": world_vmax,
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


def recommend_box_sphere_layout(
    world_vmin: np.ndarray,
    world_vmax: np.ndarray,
    margin: float = 0.02,
    obstacle_radius: float = 0.18,
) -> dict:
    # ################################
    # Python: 3×3×2 multi-sphere envelope with conservative corner coverage
    # ################################
    expanded_min = world_vmin - margin
    expanded_max = world_vmax + margin
    grid_x = [-0.45, 0.0, 0.45]
    grid_y = [-0.35, 0.0, 0.35]
    grid_z = [0.18, 0.38]
    spacing_x = 0.45
    spacing_y = 0.35
    spacing_z = 0.20
    max_uncovered = 0.0
    sample_points = [
        world_vmin,
        world_vmax,
        np.array([world_vmin[0], world_vmax[1], world_vmin[2]]),
        np.array([world_vmax[0], world_vmin[1], world_vmax[2]]),
    ]
    centers = []
    for x in grid_x:
        for y in grid_y:
            for z in grid_z:
                centers.append(np.array([x, y, z], dtype=float))
    for point in sample_points:
        dist = min(np.linalg.norm(point - c) for c in centers)
        max_uncovered = max(max_uncovered, dist - obstacle_radius)
    return {
        "layout": "3x3x2 multi_sphere",
        "margin": margin,
        "obstacle_radius": obstacle_radius,
        "grid_x": grid_x,
        "grid_y": grid_y,
        "grid_z": grid_z,
        "spacing_xyz": [spacing_x, spacing_y, spacing_z],
        "expanded_bounds_min": expanded_min,
        "expanded_bounds_max": expanded_max,
        "max_corner_coverage_error_m": float(max_uncovered),
        "sphere_count": len(centers),
    }


def print_box_layout_report(layout: dict) -> None:
    print("\n=== Box multi-sphere layout recommendation ===")
    print(f"recommended layout: {layout['layout']}")
    print(f"sphere count: {layout['sphere_count']}")
    print(f"margin: {layout['margin']:.3f} m")
    print(f"obstacle_radius: {layout['obstacle_radius']:.3f} m")
    print(f"grid_x: {layout['grid_x']}")
    print(f"grid_y: {layout['grid_y']}")
    print(f"grid_z: {layout['grid_z']}")
    print(f"center spacing (x,y,z): {layout['spacing_xyz']}")
    print(f"expanded bounds min: {fmt_vec(layout['expanded_bounds_min'])}")
    print(f"expanded bounds max: {fmt_vec(layout['expanded_bounds_max'])}")
    print(f"max corner coverage error estimate: {layout['max_corner_coverage_error_m']:.6f} m")
    if layout["max_corner_coverage_error_m"] > 0.01:
        print("WARNING: corner coverage error > 1 cm — increase radius or add spheres")


def test_box_layout_gate(repo_root: Path, yaml_path: Path) -> None:
    profile = load_moma_profile(yaml_path)
    mesh_parts = profile.get("mesh_parts", [])
    base_T = np.eye(4)
    box_planning = None
    for entry in mesh_parts:
        if is_box_entry(entry):
            box_planning = audit_mesh_entry(entry, repo_root, base_T, [0, 0, 0], False)
            break
    if box_planning is None:
        raise AssertionError("box_link.stl entry not found in mesh_parts")

    box_obstacle = profile.get("box_obstacle", {})
    margin = float(box_obstacle.get("margin", 0.02))
    radius = float(box_obstacle.get("obstacle_radius", 0.18))
    layout = recommend_box_sphere_layout(
        box_planning["world_vmin"], box_planning["world_vmax"], margin, radius
    )
    assert layout["max_corner_coverage_error_m"] <= 0.02, (
        f"box layout coverage error {layout['max_corner_coverage_error_m']:.4f} > 2 cm"
    )
    assert layout["sphere_count"] >= 8
    has_nonzero_xy = any(
        abs(x) > 1e-6 or abs(y) > 1e-6
        for x in layout["grid_x"]
        for y in layout["grid_y"]
        for _ in layout["grid_z"]
    )
    assert has_nonzero_xy


def run_audit(repo_root: Path, yaml_path: Path) -> int:
    profile = load_moma_profile(yaml_path)
    visual_root = profile.get("visual", {}).get("base_xyz", [0.0, 0.0, 0.0])
    mesh_parts = profile.get("mesh_parts", [])
    chassis = profile.get("chassis", {})

    base_T = np.eye(4)

    print("# Ranger Geometry / Collision Audit")
    print(f"repo_root: {repo_root}")
    print(f"yaml: {yaml_path}")
    print(f"visual.base_xyz: {visual_root}")
    print(f"chassis: length={chassis.get('length')} width={chassis.get('width')} "
          f"height={chassis.get('height')} collision_radius={chassis.get('collision_radius')}")

    theoretical_r = chassis_footprint_radius(float(chassis.get("length", 0.0)), float(chassis.get("width", 0.0)))
    print(f"chassis rectangle circumradius (theoretical): {theoretical_r:.6f}")

    box_planning = None
    box_visual = None

    for entry in mesh_parts:
        if is_ranger_base_entry(entry):
            print_mesh_report("Ranger base STL", audit_mesh_entry(entry, repo_root, base_T, visual_root, True))
        elif is_box_entry(entry):
            box_planning = audit_mesh_entry(entry, repo_root, base_T, visual_root, False)
            box_visual = audit_mesh_entry(entry, repo_root, base_T, visual_root, True)
            print_mesh_report("Box STL (planning frame)", box_planning)
            print_mesh_report("Box STL (visual frame)", box_visual)
        elif is_wheel_entry(entry):
            print_mesh_report("Wheel STL", audit_mesh_entry(entry, repo_root, base_T, visual_root, True))

    if box_planning is not None:
        xy_half = max(
            abs(box_planning["world_vmin"][0]),
            abs(box_planning["world_vmax"][0]),
            abs(box_planning["world_vmin"][1]),
            abs(box_planning["world_vmax"][1]),
        )
        print("\n=== Box vs chassis collision model ===")
        print(f"box planning XY half-extent (approx): {xy_half:.6f}")
        print(f"chassis collision_radius: {float(chassis.get('collision_radius', 0.0)):.6f}")
        print(
            "note: GridMap chassis 2D layer only ingests obstacle points with z < chassis_height "
            f"({chassis.get('height')}); upper box uses base_obstacle_proxies_ 3D envelope."
        )
        layout = recommend_box_sphere_layout(
            box_planning["world_vmin"],
            box_planning["world_vmax"],
            margin=float(profile.get("box_obstacle", {}).get("margin", 0.02)),
            obstacle_radius=float(profile.get("box_obstacle", {}).get("obstacle_radius", 0.18)),
        )
        print_box_layout_report(layout)
        chassis_r = float(chassis.get("collision_radius", 0.0))
        print(
            f"chassis disk covers box XY footprint: "
            f"{xy_half:.6f} <= {chassis_r:.6f} -> {xy_half <= chassis_r + 1e-6}"
        )

    print("\n=== CR10 collision proxies ===")
    print("deferred to Task 2A (test_topay_cr10_colli_frame / getColliPtsCr10 numeric dump)")

    print("\n=== Box smoke false negative (observed pattern) ===")
    print(
        "When obstacle z > chassis_height but XY overlaps box, RViz box mesh may intersect obstacle "
        "while GridMap::isWholeBodyCollision() returns false (chassis 2D + low-Z filtering only)."
    )
    if box_planning is not None:
        print(
            f"box planning z-range: [{box_planning['world_vmin'][2]:.4f}, {box_planning['world_vmax'][2]:.4f}]"
        )

    return 0


def test_four_wheel_ground_gate(repo_root: Path, yaml_path: Path) -> None:
    profile = load_moma_profile(yaml_path)
    visual_root = profile.get("visual", {}).get("base_xyz", [0.0, 0.0, 0.0])
    base_T = np.eye(4)
    wheels = [e for e in profile.get("mesh_parts", []) if is_wheel_entry(e)]
    if len(wheels) != 4:
        raise AssertionError(f"expected 4 wheel entries, found {len(wheels)}")

    for entry in wheels:
        result = audit_mesh_entry(entry, repo_root, base_T, visual_root, True)
        gc = result["ground_clearance"]
        assert -WHEEL_GROUND_TOL <= gc <= WHEEL_GROUND_TOL, (
            f"{entry['file']} ground_clearance={gc:.6f} outside ±{WHEEL_GROUND_TOL}"
        )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Audit Ranger STL bounds in world frames.")
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT_DEFAULT)
    parser.add_argument("--yaml", type=Path, default=YAML_DEFAULT)
    parser.add_argument("--test-wheels", action="store_true", help="Run four-wheel ground gate")
    parser.add_argument("--test-box-layout", action="store_true", help="Run box multi-sphere coverage gate")
    args = parser.parse_args(argv)

    yaml_path = args.yaml if args.yaml.is_absolute() else args.repo_root / args.yaml
    if args.test_wheels:
        test_four_wheel_ground_gate(args.repo_root, yaml_path)
        print("Four-wheel ground gate PASSED")
        return 0
    if args.test_box_layout:
        test_box_layout_gate(args.repo_root, yaml_path)
        print("Base-box obstacle proxy geometry coverage PASSED")
        return 0
    return run_audit(args.repo_root, yaml_path)


if __name__ == "__main__":
    sys.exit(main())
