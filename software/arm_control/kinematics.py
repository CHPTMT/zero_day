"""Forward and inverse kinematics using standard DH parameters.

All public functions take joint angles in degrees and return positions in metres.
"""

from __future__ import annotations

import numpy as np

from .config import ArmConfig


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Homogeneous transform for one standard DH link. Angles in radians."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def joint_frames(cfg: ArmConfig, angles_deg) -> list[np.ndarray]:
    """Base frame followed by the frame after each joint, then the tool frame."""
    angles = np.radians(np.asarray(angles_deg, dtype=float))
    if angles.shape != (cfg.dof,):
        raise ValueError(f"expected {cfg.dof} joint angles, got {angles.shape}")
    T = np.eye(4)
    frames = [T]
    for j, q in zip(cfg.joints, angles):
        T = T @ dh_transform(j.a, np.radians(j.alpha), j.d, q + np.radians(j.theta_offset))
        frames.append(T)
    tool = np.eye(4)
    tool[:3, 3] = cfg.tool_offset
    frames.append(T @ tool)
    return frames


def forward_kinematics(cfg: ArmConfig, angles_deg) -> np.ndarray:
    """4x4 pose of the tool point in the base frame."""
    return joint_frames(cfg, angles_deg)[-1]


def jacobian(cfg: ArmConfig, angles_deg) -> np.ndarray:
    """6xN geometric Jacobian (linear rows first) for revolute joints, per radian."""
    frames = joint_frames(cfg, angles_deg)
    p_tool = frames[-1][:3, 3]
    J = np.zeros((6, cfg.dof))
    for i in range(cfg.dof):
        z = frames[i][:3, 2]
        p = frames[i][:3, 3]
        J[:3, i] = np.cross(z, p_tool - p)
        J[3:, i] = z
    return J


def _orientation_error(R_cur: np.ndarray, R_goal: np.ndarray) -> np.ndarray:
    R_err = R_goal @ R_cur.T
    return 0.5 * np.array(
        [R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]]
    )


def inverse_kinematics(
    cfg: ArmConfig,
    target_position,
    target_rotation: np.ndarray | None = None,
    initial_deg=None,
    tol: float = 1e-4,
    max_iter: int = 300,
    damping: float = 0.05,
) -> tuple[np.ndarray, bool]:
    """Damped least squares IK.

    Returns (angles_deg, converged). Angles are clamped to joint limits every step.
    If target_rotation is None only position is solved, which suits arms with
    fewer than 6 DOF.
    """
    q = np.radians(cfg.home if initial_deg is None else np.asarray(initial_deg, dtype=float))
    lo, hi = np.radians(cfg.lower_limits), np.radians(cfg.upper_limits)
    target = np.asarray(target_position, dtype=float)

    for _ in range(max_iter):
        T = forward_kinematics(cfg, np.degrees(q))
        err = target - T[:3, 3]
        J = jacobian(cfg, np.degrees(q))
        if target_rotation is not None:
            err = np.concatenate([err, _orientation_error(T[:3, :3], target_rotation)])
        else:
            J = J[:3]
        if np.linalg.norm(err) < tol:
            return np.degrees(q), True
        JJt = J @ J.T + (damping**2) * np.eye(J.shape[0])
        q = np.clip(q + J.T @ np.linalg.solve(JJt, err), lo, hi)

    return np.degrees(q), False
