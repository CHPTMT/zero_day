import numpy as np
import pytest

from arm_control import forward_kinematics, inverse_kinematics, jacobian, load_config


@pytest.fixture(scope="module")
def cfg():
    return load_config()


def test_config_loads(cfg):
    assert cfg.dof == len(cfg.joints) > 0
    assert np.all(cfg.lower_limits < cfg.upper_limits)
    assert not cfg.check_limits(cfg.home)


def test_fk_returns_valid_transform(cfg):
    T = forward_kinematics(cfg, cfg.home)
    assert T.shape == (4, 4)
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
    np.testing.assert_allclose(T[3], [0, 0, 0, 1])


def test_fk_rejects_wrong_joint_count(cfg):
    with pytest.raises(ValueError):
        forward_kinematics(cfg, [0.0] * (cfg.dof + 1))


def test_jacobian_matches_finite_difference(cfg):
    q = np.linspace(-20, 20, cfg.dof)
    J = jacobian(cfg, q)
    eps = 1e-6
    for i in range(cfg.dof):
        dq = np.zeros(cfg.dof)
        dq[i] = np.degrees(eps)
        p1 = forward_kinematics(cfg, q + dq)[:3, 3]
        p0 = forward_kinematics(cfg, q - dq)[:3, 3]
        np.testing.assert_allclose(J[:3, i], (p1 - p0) / (2 * eps), atol=1e-6)


def test_ik_round_trip(cfg):
    rng = np.random.default_rng(0)
    for _ in range(10):
        q_true = rng.uniform(cfg.lower_limits * 0.5, cfg.upper_limits * 0.5)
        target = forward_kinematics(cfg, q_true)[:3, 3]
        q_sol, ok = inverse_kinematics(cfg, target, initial_deg=q_true + 5.0)
        assert ok
        np.testing.assert_allclose(forward_kinematics(cfg, q_sol)[:3, 3], target, atol=1e-3)
        assert not cfg.check_limits(q_sol)
