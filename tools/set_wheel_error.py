#!/usr/bin/env python3
"""
set_wheel_error.py -- make the wheel odometry deliberately wrong (or correct again) for the drift test.

Sets left/right_wheel_radius_multiplier of the diff_drive_controller in src/my_robot/config/controllers.yaml.
With 1.10 the controller believes the wheels are 10% bigger than they are, like a mis-calibrated or worn wheel:
it drives 10% slower than commanded but reports the commanded distance, so wheel odometry over-counts by 10%.
The IMU and the camera are not affected, so this shows what each sensor contributes.

Usage (from ~/autonomy, then restart the stack):
    python3 tools/set_wheel_error.py 1.10     # wheels lie by 10%
    python3 tools/set_wheel_error.py 1.0      # back to correct
"""
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
P = os.path.join(ROOT, "src", "my_robot", "config", "controllers.yaml")


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


m = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
d = yaml.safe_load(open(P))
p = d["diff_drive_controller"]["ros__parameters"]
if m == 1.0:
    p.pop("left_wheel_radius_multiplier", None)
    p.pop("right_wheel_radius_multiplier", None)
else:
    p["left_wheel_radius_multiplier"] = m
    p["right_wheel_radius_multiplier"] = m
yaml.dump(d, open(P, "w"), Dumper=NoAliasDumper, sort_keys=False, default_flow_style=None)
print("wheel radius multiplier = %.2f written to %s (restart the stack to apply)" % (m, P))