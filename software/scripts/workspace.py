"""Sample the reachable workspace and print its bounds. Plots it if matplotlib is installed.

Usage: python scripts/workspace.py [--samples N] [--plot]
"""

import argparse

import numpy as np

from arm_control import forward_kinematics, load_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=5000)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    cfg = load_config()
    rng = np.random.default_rng(0)
    qs = rng.uniform(cfg.lower_limits, cfg.upper_limits, size=(args.samples, cfg.dof))
    pts = np.array([forward_kinematics(cfg, q)[:3, 3] for q in qs])

    print(f"{cfg.name}: {cfg.dof} DOF, {args.samples} samples")
    for axis, lo, hi in zip("xyz", pts.min(axis=0), pts.max(axis=0)):
        print(f"  {axis}: {lo:+.3f} to {hi:+.3f} m")
    print(f"  max reach: {np.linalg.norm(pts, axis=1).max():.3f} m")

    if args.plot:
        import matplotlib.pyplot as plt

        ax = plt.figure().add_subplot(projection="3d")
        ax.scatter(*pts.T, s=1)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        plt.show()


if __name__ == "__main__":
    main()
