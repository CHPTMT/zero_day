"""Joint-space trajectory generation with trapezoidal velocity profiles."""

from __future__ import annotations

import numpy as np

from .config import ArmConfig


def trapezoidal_profile(distance: float, v_max: float, a_max: float) -> tuple[float, float]:
    """Return (accel_time, total_time) for a rest-to-rest move of |distance|.

    Falls back to a triangular profile when v_max cannot be reached.
    """
    d = abs(distance)
    if d == 0.0:
        return 0.0, 0.0
    t_acc = v_max / a_max
    if a_max * t_acc**2 >= d:  # triangular
        t_acc = np.sqrt(d / a_max)
        return t_acc, 2 * t_acc
    return t_acc, 2 * t_acc + (d - a_max * t_acc**2) / v_max


def _sample(distance: float, t_acc: float, t_total: float, t: np.ndarray) -> np.ndarray:
    """Position along a profile scaled to fit t_total, sampled at times t."""
    if t_total == 0.0:
        return np.zeros_like(t)
    t_cruise = t_total - 2 * t_acc
    v = distance / (t_acc + t_cruise)  # peak velocity that covers distance in t_total
    a = v / t_acc if t_acc > 0 else 0.0
    t = np.clip(t, 0.0, t_total)
    return np.where(
        t < t_acc,
        0.5 * a * t**2,
        np.where(
            t < t_acc + t_cruise,
            0.5 * a * t_acc**2 + v * (t - t_acc),
            distance - 0.5 * a * (t_total - t) ** 2,
        ),
    )


def joint_trajectory(cfg: ArmConfig, start_deg, goal_deg, rate_hz: float | None = None):
    """Synchronised joint move: every joint starts and stops together.

    Returns (times, positions) with positions shaped (samples, dof) in degrees.
    """
    start = np.asarray(start_deg, dtype=float)
    goal = np.asarray(goal_deg, dtype=float)
    bad = cfg.check_limits(goal)
    if bad:
        raise ValueError(f"goal outside joint limits: {', '.join(bad)}")
    rate = rate_hz or cfg.controller.get("control_rate_hz", 100)

    delta = goal - start
    profiles = [
        trapezoidal_profile(dq, j.max_velocity, j.max_acceleration)
        for dq, j in zip(delta, cfg.joints)
    ]
    t_total = max(p[1] for p in profiles)
    if t_total == 0.0:
        return np.array([0.0]), start[None, :]

    # Stretch each joint's own profile in time to the slowest joint's duration.
    # Stretching by k divides velocity by k and acceleration by k^2, so limits still hold.
    times = np.linspace(0.0, t_total, max(2, int(np.ceil(t_total * rate)) + 1))
    columns = []
    for dq, q0, (t_acc, t_move) in zip(delta, start, profiles):
        k = t_total / t_move if t_move > 0 else 1.0
        columns.append(q0 + _sample(dq, t_acc * k, t_total if t_move > 0 else 0.0, times))
    positions = np.stack(columns, axis=1)
    return times, positions
