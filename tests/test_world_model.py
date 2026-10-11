"""World model: constant-velocity Kalman tracker and forecast."""
import math

import numpy as np

from world_model_node import Track


def run_track(vx, vy, steps=40, dt=0.1, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    t = Track(0.0, 0.0, "person", None)
    for k in range(1, steps + 1):
        t.predict(dt)
        zx = vx * k * dt + rng.normal(0.0, noise)
        zy = vy * k * dt + rng.normal(0.0, noise)
        t.update(zx, zy)
    return t


def test_velocity_converges_for_walking_person():
    t = run_track(0.7, 0.0)
    assert abs(t.X[2] - 0.7) < 0.05
    assert abs(t.X[3]) < 0.05


def test_velocity_converges_with_detection_noise():
    t = run_track(0.0, -0.6, steps=80, noise=0.05)
    assert abs(t.X[3] + 0.6) < 0.15


def test_forecast_follows_straight_line():
    t = run_track(0.5, 0.5)
    fc = t.forecast(10, 0.5)
    x_end, y_end = fc[-1][0], fc[-1][1]
    # 5 s ahead at about 0.5 m/s per axis from the last position
    assert math.isclose(x_end, t.X[0] + 2.5, abs_tol=0.3)
    assert math.isclose(y_end, t.X[1] + 2.5, abs_tol=0.3)


def test_forecast_uncertainty_grows_with_horizon():
    t = run_track(0.7, 0.0)
    fc = t.forecast(10, 0.5)
    var_x = [step[2] for step in fc]
    var_y = [step[3] for step in fc]
    assert all(b > a for a, b in zip(var_x, var_x[1:]))
    assert all(b > a for a, b in zip(var_y, var_y[1:]))


def test_forecast_does_not_change_the_track():
    t = run_track(0.7, 0.0)
    before = t.X.copy(), t.P.copy()
    t.forecast(10, 0.5)
    assert np.array_equal(t.X, before[0])
    assert np.array_equal(t.P, before[1])


def test_update_resets_missed_counter():
    t = Track(1.0, 2.0, "cart", None)
    t.missed = 3
    t.predict(0.1)
    t.update(1.0, 2.0)
    assert t.missed == 0
