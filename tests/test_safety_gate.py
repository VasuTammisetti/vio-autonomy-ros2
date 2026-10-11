"""Safety gate: state thresholds and the direction-aware escape rule.

The gate methods are called on a small stand-in object, so no ROS node or TF is needed.
"""
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist

import safety_gate as sg
from safety_gate import SafetyGate


@pytest.mark.parametrize("dist, state", [
    (None, "AUTONOMOUS"),
    (5.0, "AUTONOMOUS"),
    (sg.WARN_RADIUS + 0.01, "AUTONOMOUS"),
    (sg.WARN_RADIUS - 0.01, "DEGRADED"),
    (sg.STOP_RADIUS - 0.01, "SAFE_STOP"),
    (sg.CRITICAL_RADIUS - 0.01, "HOLD"),
    (0.0, "HOLD"),
])
def test_state_thresholds(dist, state):
    assert SafetyGate.compute_state(None, dist) == state


def test_radii_are_ordered():
    assert sg.CRITICAL_RADIUS < sg.STOP_RADIUS < sg.WARN_RADIUS


def gate(robot_pose, person_xy, v, w):
    cmd = Twist()
    cmd.linear.x, cmd.angular.z = v, w
    return SimpleNamespace(robot_pose=lambda: robot_pose, person_xy=person_xy, last_cmd=cmd)


def escape(robot_pose, person_xy, v, w=0.0):
    return SafetyGate.escape_cmd(gate(robot_pose, person_xy, v, w))


def test_no_driving_towards_person_ahead():
    out = escape((0.0, 0.0, 0.0), (0.7, 0.0), v=0.3)
    assert out.linear.x == 0.0


def test_backing_away_from_person_ahead_is_allowed():
    out = escape((0.0, 0.0, 0.0), (0.7, 0.0), v=-0.2)
    assert out.linear.x == pytest.approx(-0.2 * sg.ESCAPE_SPEED_SCALE)


def test_driving_on_with_person_behind_is_allowed():
    out = escape((0.0, 0.0, 0.0), (-0.7, 0.0), v=0.3)
    assert out.linear.x == pytest.approx(0.3 * sg.ESCAPE_SPEED_SCALE)


def test_driving_past_person_beside_is_allowed():
    out = escape((0.0, 0.0, 0.0), (0.0, 0.7), v=0.3)
    assert out.linear.x == pytest.approx(0.3 * sg.ESCAPE_SPEED_SCALE)


def test_heading_is_taken_into_account():
    # robot faces north (yaw 90 deg), person is north of it: driving forward approaches the person
    out = escape((0.0, 0.0, math.pi / 2), (0.0, 0.7), v=0.3)
    assert out.linear.x == 0.0


def test_turning_on_the_spot_is_always_allowed():
    out = escape((0.0, 0.0, 0.0), (0.7, 0.0), v=0.3, w=0.8)
    assert out.angular.z == pytest.approx(0.8 * sg.ESCAPE_SPEED_SCALE)


def test_no_pose_means_full_stop():
    out = escape(None, (0.7, 0.0), v=-0.2, w=0.5)
    assert out.linear.x == 0.0 and out.angular.z == 0.0
