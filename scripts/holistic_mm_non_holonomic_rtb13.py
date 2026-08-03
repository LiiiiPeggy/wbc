#!/usr/bin/env python3
"""Holistic MM (non-holonomic) demo adapted for roboticstoolbox-python 1.3.x.

Source idea: https://jhavl.github.io/holistic/

Usage:
  conda activate holistic

  # 默认目标（与官网类似：身后远处）
  python scripts/holistic_mm_non_holonomic_rtb13.py

  # 预设目标
  python scripts/holistic_mm_non_holonomic_rtb13.py --preset front
  python scripts/holistic_mm_non_holonomic_rtb13.py --preset left --swift

  # 自定义世界系位置 + 姿态（米 / 度）
  python scripts/holistic_mm_non_holonomic_rtb13.py --xyz 1.0 -0.5 0.6 --rpy 180 0 0
  python scripts/holistic_mm_non_holonomic_rtb13.py --xyz 0.8 0.3 0.5 --rpy 180 0 90 --swift
"""

# ################################
# Python: Holistic MM non-holonomic demo begin
# ################################

from __future__ import annotations

import argparse
import math
from typing import Optional, Sequence, Tuple

import numpy as np
import qpsolvers as qp
import roboticstoolbox as rtb
import spatialmath as sm


PRESETS = ("default", "front", "left", "right", "up", "near")


def step_robot(r: rtb.ERobot, Tep: np.ndarray, solver: str = "quadprog"):
    wTe = r.fkine(r.q)
    wTe_A = wTe.A if hasattr(wTe, "A") else np.asarray(wTe)
    eTep = np.linalg.inv(wTe_A) @ Tep

    # Spatial error
    et = float(np.sum(np.abs(eTep[:3, -1])))
    et_safe = max(et, 1e-6)

    # Gain term (lambda) for control minimisation
    Y = 0.01

    # Quadratic component of objective function
    Q = np.eye(r.n + 6)
    Q[: r.n, : r.n] *= Y
    Q[:2, :2] *= 1.0 / et_safe
    Q[r.n :, r.n :] = (1.0 / et_safe) * np.eye(6)

    v, _ = rtb.p_servo(wTe, Tep, 1.5)
    v = np.asarray(v).reshape(6)
    v[3:] *= 1.3

    # Equality constraints
    Aeq = np.c_[r.jacobe(r.q), np.eye(6)]
    beq = v.reshape((6,))

    # Inequality constraints for joint limit avoidance
    Ain = np.zeros((r.n + 6, r.n + 6))
    bin_ = np.zeros(r.n + 6)

    # RTB 1.3+: keyword args (old positional joint_velocity_damper(ps, pi, n) is broken)
    Ain[: r.n, : r.n], bin_[: r.n] = r.joint_velocity_damper(ps=0.1, pi=0.9, n=r.n)

    # Linear objective: manipulability Jacobian
    c = np.concatenate(
        (np.zeros(2), -r.jacobm(start=r.links[4]).reshape((r.n - 2,)), np.zeros(6))
    )

    # Get base to face end-effector
    k_eps = 0.5
    bTe = r.fkine(r.q, include_base=False).A
    theta_e = math.atan2(bTe[1, -1], bTe[0, -1])
    c[0] = -k_eps * theta_e

    qdlim = np.asarray(r.qdlim[: r.n], dtype=float)
    lb = -np.r_[qdlim, 10 * np.ones(6)]
    ub = np.r_[qdlim, 10 * np.ones(6)]

    qd = qp.solve_qp(Q, c, Ain, bin_, Aeq, beq, lb=lb, ub=ub, solver=solver)
    if qd is None:
        raise RuntimeError("QP solver returned None; check qpsolvers/quadprog install")
    qd = qd[: r.n]

    if et > 0.5:
        qd *= 0.7 / et
    else:
        qd *= 1.4

    return et < 0.05, qd, et


def make_preset_goal(robot: rtb.ERobot, preset: str) -> sm.SE3:
    """Build a goal pose relative to the current end-effector / ready pose."""
    # ################################
    # Python: preset goal poses begin
    # ################################
    Te = robot.fkine(robot.q)

    if preset == "default":
        # Official Holistic demo style: behind and lower
        wTep = Te * sm.SE3.Rz(np.pi)
        wTep.A[:3, :3] = np.diag([-1, 1, -1])
        wTep.A[0, -1] -= 4.0
        wTep.A[2, -1] -= 0.25
        return wTep

    # Orientation: tool roughly pointing down (common pick orientation)
    R_down = sm.SE3.RPY([np.pi, 0.0, 0.0], order="xyz")

    offsets = {
        "front": np.array([1.5, 0.0, 0.0]),
        "left": np.array([0.8, 1.2, 0.0]),
        "right": np.array([0.8, -1.2, 0.0]),
        "up": np.array([0.6, 0.0, 0.5]),
        "near": np.array([0.5, 0.2, -0.1]),
    }
    if preset not in offsets:
        raise ValueError(f"Unknown preset '{preset}', choose from {PRESETS}")

    p = Te.t + offsets[preset]
    return sm.SE3.Trans(p) * R_down
    # ################################
    # Python: preset goal poses end
    # ################################


def make_custom_goal(
    xyz: Sequence[float],
    rpy_deg: Optional[Sequence[float]] = None,
) -> sm.SE3:
    """Build a goal pose in the world frame from xyz (m) and rpy (deg)."""
    # ################################
    # Python: custom world-frame goal begin
    # ################################
    if rpy_deg is None:
        rpy_deg = (180.0, 0.0, 0.0)
    rpy = np.deg2rad(np.asarray(rpy_deg, dtype=float))
    return sm.SE3.Trans(xyz) * sm.SE3.RPY(rpy, order="xyz")
    # ################################
    # Python: custom world-frame goal end
    # ################################


def resolve_goal(
    robot: rtb.ERobot,
    preset: str,
    xyz: Optional[Sequence[float]],
    rpy_deg: Optional[Sequence[float]],
) -> sm.SE3:
    if xyz is not None:
        return make_custom_goal(xyz, rpy_deg)
    return make_preset_goal(robot, preset)


def print_goal(wTep: sm.SE3) -> None:
    p = wTep.t
    rpy = sm.base.tr2rpy(wTep.R, order="xyz")
    print(
        "Goal pose: "
        f"xyz=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m, "
        f"rpy=({np.rad2deg(rpy[0]):.1f}, {np.rad2deg(rpy[1]):.1f}, {np.rad2deg(rpy[2]):.1f}) deg"
    )


def integrate_step(robot: rtb.ERobot, qd: np.ndarray, dt: float) -> None:
    robot.q = robot.q + qd * dt
    base_new = robot.fkine(robot._q, end=robot.links[2])
    robot._T = base_new.A
    robot.q[:2] = 0


def run_headless(
    wTep: sm.SE3,
    max_steps: int = 1200,
    dt: float = 0.025,
) -> None:
    frankie = rtb.models.Frankie()
    frankie.q = frankie.qr.copy()
    # If goal was built from a temporary robot earlier, rebuild only when needed;
    # here wTep is already absolute in world.
    print_goal(wTep)

    arrived = False
    et = None
    for i in range(max_steps):
        arrived, frankie.qd, et = step_robot(frankie, wTep.A)
        integrate_step(frankie, frankie.qd, dt)
        if i % 100 == 0:
            print(f"step={i} et={et:.4f}")
        if arrived:
            print(f"ARRIVED step={i} et={et:.4f}")
            break
    else:
        print(f"STOPPED step={max_steps} et={et:.4f}")


def run_swift(wTep: sm.SE3, dt: float = 0.025) -> None:
    import spatialgeometry as sg
    import swift

    env = swift.Swift()
    env.launch(realtime=True)

    ax_goal = sg.Axes(0.1)
    env.add(ax_goal)

    frankie = rtb.models.Frankie()
    frankie.q = frankie.qr.copy()
    env.add(frankie)

    env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])
    print_goal(wTep)
    ax_goal.T = wTep
    env.step()

    arrived = False
    while not arrived:
        arrived, frankie.qd, _ = step_robot(frankie, wTep.A)
        env.step(dt)
        integrate_step(frankie, frankie.qd, dt)

    print("ARRIVED")
    env.hold()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Holistic MM non-holonomic demo with selectable goal pose"
    )
    parser.add_argument("--swift", action="store_true", help="Enable Swift browser viz")
    parser.add_argument(
        "--preset",
        choices=PRESETS,
        default="default",
        help="Preset goal relative to ready EE pose (ignored if --xyz is set)",
    )
    parser.add_argument(
        "--xyz",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Custom goal position in world frame (meters)",
    )
    parser.add_argument(
        "--rpy",
        nargs=3,
        type=float,
        metavar=("ROLL", "PITCH", "YAW"),
        default=None,
        help="Custom goal RPY in degrees (default with --xyz: 180 0 0)",
    )
    parser.add_argument("--max-steps", type=int, default=1200, help="Headless max steps")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # Build goal using a temporary robot at ready config (for presets)
    robot0 = rtb.models.Frankie()
    robot0.q = robot0.qr.copy()
    wTep = resolve_goal(robot0, args.preset, args.xyz, args.rpy)

    if args.swift:
        run_swift(wTep)
    else:
        run_headless(wTep, max_steps=args.max_steps)


if __name__ == "__main__":
    main()

# ################################
# Python: Holistic MM non-holonomic demo end
# ################################
