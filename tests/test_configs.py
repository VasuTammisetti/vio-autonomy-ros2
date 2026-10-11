"""Config regression tests. Each one guards a bug we actually hit."""
import os

import pytest
import yaml


def load(config_dir, name):
    with open(os.path.join(config_dir, name)) as f:
        return yaml.safe_load(f)


def walk(d, prefix=""):
    if isinstance(d, dict):
        for k, v in d.items():
            yield from walk(v, prefix + "/" + str(k))
    else:
        yield prefix, d


@pytest.mark.parametrize("name", ["nav2_params.yaml", "ekf.yaml", "controllers.yaml", "vo.yaml",
                                  "slam_localization.yaml"])
def test_config_has_no_yaml_aliases(config_dir, name):
    # rcl's parameter parser rejects YAML anchors/aliases. A shared dict in nav2_params.yaml made
    # every Nav2 server die at start with "Will not support aliasing" (exit -6).
    with open(os.path.join(config_dir, name)) as f:
        tokens = list(yaml.scan(f))
    bad = [t for t in tokens if isinstance(t, (yaml.AnchorToken, yaml.AliasToken))]
    assert not bad, "%s uses YAML anchors/aliases" % name


def test_ekf_heading_comes_from_gyro_not_wheels(config_dir):
    # Drift test: taking yaw from the wheels let a 6% wheel twist error turn into 21 deg of drift.
    p = load(config_dir, "ekf.yaml")["ekf_filter_node"]["ros__parameters"]
    assert p["two_d_mode"] is True
    assert p["odom0_config"][5] is False, "wheel yaw must not be fused"
    assert p["odom0_differential"] is True
    assert p["imu0_config"][11] is True, "gyro yaw rate must be fused"


def test_ekf_accepts_late_visual_odometry(config_dir):
    # rtabmap VO arrives 150-200 ms late; without lagged-data smoothing the EKF drops it.
    p = load(config_dir, "ekf.yaml")["ekf_filter_node"]["ros__parameters"]
    if "odom1" in p:
        assert p.get("smooth_lagged_data") is True
        assert p.get("history_length", 0.0) >= 0.5


def test_no_wheel_error_left_in_controllers(config_dir):
    # tools/set_wheel_error.py adds a radius multiplier for the drift test; it must never be committed.
    d = load(config_dir, "controllers.yaml")
    for key, value in walk(d):
        if "radius_multiplier" in key:
            assert float(value) == 1.0, "%s = %s" % (key, value)


def test_slam_runs_in_localization_mode(config_dir):
    d = load(config_dir, "slam_localization.yaml")
    p = d["slam_toolbox"]["ros__parameters"]
    assert p["mode"] == "localization"
    assert p["map_file_name"]
