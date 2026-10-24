"""Load and validate config/arm.yaml."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "arm.yaml"


@dataclass
class JointConfig:
    name: str
    a: float
    alpha: float  # deg
    d: float
    theta_offset: float  # deg
    min: float  # deg
    max: float  # deg
    max_velocity: float  # deg/s
    max_acceleration: float  # deg/s^2
    drive: dict = field(default_factory=dict)
    home: float = 0.0

    def within_limits(self, angle_deg: float) -> bool:
        return self.min <= angle_deg <= self.max


@dataclass
class ArmConfig:
    name: str
    joints: list[JointConfig]
    tool_offset: np.ndarray
    gripper: dict
    controller: dict

    @property
    def dof(self) -> int:
        return len(self.joints)

    @property
    def lower_limits(self) -> np.ndarray:
        return np.array([j.min for j in self.joints])

    @property
    def upper_limits(self) -> np.ndarray:
        return np.array([j.max for j in self.joints])

    @property
    def home(self) -> np.ndarray:
        return np.array([j.home for j in self.joints])

    def check_limits(self, angles_deg) -> list[str]:
        """Return names of joints whose angle is outside its limits."""
        return [j.name for j, q in zip(self.joints, angles_deg) if not j.within_limits(q)]


def load_config(path: str | Path = DEFAULT_CONFIG) -> ArmConfig:
    with open(path) as f:
        raw = yaml.safe_load(f)

    joints = []
    for j in raw["joints"]:
        dh, lim = j["dh"], j["limits"]
        if lim["min"] >= lim["max"]:
            raise ValueError(f"joint {j['name']}: min limit must be below max limit")
        joints.append(
            JointConfig(
                name=j["name"],
                a=float(dh["a"]),
                alpha=float(dh["alpha"]),
                d=float(dh["d"]),
                theta_offset=float(dh.get("theta_offset", 0.0)),
                min=float(lim["min"]),
                max=float(lim["max"]),
                max_velocity=float(lim["max_velocity"]),
                max_acceleration=float(lim["max_acceleration"]),
                drive=j.get("drive", {}),
                home=float(j.get("home", 0.0)),
            )
        )
    if not joints:
        raise ValueError("config must define at least one joint")

    t = raw.get("tool_offset", {})
    return ArmConfig(
        name=raw.get("name", "arm"),
        joints=joints,
        tool_offset=np.array([t.get("x", 0.0), t.get("y", 0.0), t.get("z", 0.0)], dtype=float),
        gripper=raw.get("gripper", {}),
        controller=raw.get("controller", {}),
    )
