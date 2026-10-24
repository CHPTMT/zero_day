import numpy as np
import pytest

from arm_control import joint_trajectory, load_config, trapezoidal_profile


@pytest.fixture(scope="module")
def cfg():
    return load_config()


def test_trapezoidal_and_triangular():
    t_acc, t_total = trapezoidal_profile(90.0, v_max=45.0, a_max=90.0)
    assert t_acc == pytest.approx(0.5)
    assert t_total == pytest.approx(2.5)
    t_acc, t_total = trapezoidal_profile(1.0, v_max=45.0, a_max=90.0)
    assert t_total == pytest.approx(2 * t_acc)


def test_trajectory_hits_endpoints_and_respects_limits(cfg):
    start = cfg.home
    goal = np.clip(start + np.linspace(30, -60, cfg.dof), cfg.lower_limits, cfg.upper_limits)
    times, pos = joint_trajectory(cfg, start, goal)
    np.testing.assert_allclose(pos[0], start, atol=1e-9)
    np.testing.assert_allclose(pos[-1], goal, atol=1e-9)

    dt = np.diff(times)[:, None]
    vel = np.diff(pos, axis=0) / dt
    vmax = np.array([j.max_velocity for j in cfg.joints])
    assert np.all(np.abs(vel) <= vmax * 1.01)


def test_trajectory_rejects_goal_outside_limits(cfg):
    goal = cfg.upper_limits + 1.0
    with pytest.raises(ValueError):
        joint_trajectory(cfg, cfg.home, goal)


def test_zero_length_move(cfg):
    times, pos = joint_trajectory(cfg, cfg.home, cfg.home)
    assert len(times) == 1
