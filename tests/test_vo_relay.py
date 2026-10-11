"""VO relay: frame-to-frame steps must chain back to the same trajectory."""
import math
import random

import pytest

from vo_relay import apply_step, relative_step, wrap


def test_wrap_keeps_angles_in_range():
    for a in (-7.0, -math.pi, 0.0, 3.0, 4.0, 10.0):
        w = wrap(a)
        assert -math.pi <= w <= math.pi
        assert math.isclose(math.cos(w), math.cos(a), abs_tol=1e-9)


def test_step_forward_in_robot_frame():
    # robot at (1, 1) facing north moves 0.1 m north: that is 0.1 m straight ahead
    step = relative_step((1.0, 1.0, math.pi / 2), (1.0, 1.1, math.pi / 2))
    assert step[0] == pytest.approx(0.1)
    assert step[1] == pytest.approx(0.0, abs=1e-9)
    assert step[2] == pytest.approx(0.0, abs=1e-9)


def test_chained_steps_rebuild_trajectory():
    random.seed(1)
    traj = [(0.0, 0.0, 0.0)]
    for _ in range(200):
        x, y, th = traj[-1]
        d, dth = random.uniform(0.0, 0.05), random.uniform(-0.1, 0.1)
        traj.append((x + d * math.cos(th), y + d * math.sin(th), wrap(th + dth)))
    pose = traj[0]
    for a, b in zip(traj, traj[1:]):
        pose = apply_step(pose, relative_step(a, b))
    assert pose[0] == pytest.approx(traj[-1][0], abs=1e-6)
    assert pose[1] == pytest.approx(traj[-1][1], abs=1e-6)
    assert math.isclose(math.cos(pose[2]), math.cos(traj[-1][2]), abs_tol=1e-6)


def test_steps_are_independent_of_vo_start_frame():
    # same motion seen from a VO frame that is shifted and rotated gives the same step
    a, b = (0.0, 0.0, 0.3), (0.05, 0.02, 0.35)

    def shift(p):
        c, s = math.cos(1.0), math.sin(1.0)
        return (5 + c * p[0] - s * p[1], -2 + s * p[0] + c * p[1], wrap(p[2] + 1.0))

    s1, s2 = relative_step(a, b), relative_step(shift(a), shift(b))
    assert s1 == pytest.approx(s2, abs=1e-9)
