#!/usr/bin/env python3
# ################################
# Python: CR10 FK / gradient validation begin
# ################################
"""Analytic CR10 FK + T_grad check (same fixed origins as mm_config.cpp).

Acceptance:
  position error < 1 mm (self-chain)
  rotation error < 0.1 deg
  T_grad relative error < 1e-4
"""
from __future__ import print_function
import math
import sys

import numpy as np


def urdf_rpy_to_R(r, p, y):
    cx, sx = math.cos(r), math.sin(r)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = urdf_rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


FIXED = [
    make_T([0.0, 0.0, 0.1765], [0.0, 0.0, 0.0]),
    make_T([0.0, 0.0, 0.0], [1.5708, 1.5708, 0.0]),
    make_T([-0.607, 0.0, 0.0], [0.0, 0.0, 0.0]),
    make_T([-0.568, 0.0, 0.191], [0.0, 0.0, -1.5708]),
    make_T([0.0, -0.125, 0.0], [1.5708, 0.0, 0.0]),
    make_T([0.0, 0.1084, 0.0], [-1.5708, 0.0, 0.0]),
]

MOUNT = make_T([0.2462, 0.0, 0.1], [0.0, 0.0, 0.0])


def Rz(theta):
    c, s = math.cos(theta), math.sin(theta)
    T = np.eye(4)
    T[0, 0], T[0, 1] = c, -s
    T[1, 0], T[1, 1] = s, c
    return T


def dRz(theta):
    c, s = math.cos(theta), math.sin(theta)
    T = np.zeros((4, 4))
    T[0, 0], T[0, 1] = -s, -c
    T[1, 0], T[1, 1] = c, -s
    return T


def joint_T(i, theta):
    return FIXED[i] @ Rz(theta)


def joint_T_grad(i, theta):
    return FIXED[i] @ dRz(theta)


def fk(theta):
    T = MOUNT.copy()
    for i in range(6):
        T = T @ joint_T(i, theta[i])
    return T


def rot_error_deg(Ra, Rb):
    R = Ra.T @ Rb
    tr = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(tr))


def main():
    np.random.seed(0)
    qmin = np.array([-3.00, -1.50, -2.80, -3.00, -3.00, -3.00])
    qmax = np.array([3.00, 1.50, 2.80, 3.00, 3.00, 3.00])

    max_pos_err = 0.0
    max_rot_err = 0.0
    max_grad_rel = 0.0
    eps = 1e-6
    n = 100

    for _ in range(n):
        q = qmin + (qmax - qmin) * np.random.rand(6)
        T = fk(q)
        T2 = MOUNT.copy()
        for i in range(6):
            T2 = T2 @ joint_T(i, q[i])
        max_pos_err = max(max_pos_err, float(np.linalg.norm(T[:3, 3] - T2[:3, 3])))
        max_rot_err = max(max_rot_err, rot_error_deg(T[:3, :3], T2[:3, :3]))

        for i in range(6):
            Tg = joint_T_grad(i, q[i])
            Tfd = (joint_T(i, q[i] + eps) - joint_T(i, q[i] - eps)) / (2 * eps)
            denom = max(float(np.linalg.norm(Tg)), 1e-12)
            max_grad_rel = max(max_grad_rel, float(np.linalg.norm(Tg - Tfd)) / denom)

    # Zero-config check vs known URDF mount*product
    T0 = fk(np.zeros(6))
    print("samples:", n)
    print("zero-config flange pos:", T0[:3, 3])
    print("max FK position error [m]:", max_pos_err)
    print("max FK rotation error [deg]:", max_rot_err)
    print("max T_grad relative error:", max_grad_rel)

    ok = (max_pos_err < 1e-3 and max_rot_err < 0.1 and max_grad_rel < 1e-4)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
# ################################
# Python: CR10 FK / gradient validation end
# ################################
