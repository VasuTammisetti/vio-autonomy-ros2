#!/usr/bin/env python3
"""
set_wheel_yaw.py -- choose whether the EKF takes heading (yaw) from the wheels or not.

  python3 tools/set_wheel_yaw.py off   -> wheels give x, y only. Heading comes from the IMU gyro
                                          (and VO yaw when VO runs).
  python3 tools/set_wheel_yaw.py on    -> wheels give x, y and yaw (the old setting).

Why: the drift test showed that during turns the EKF follows the wheel yaw almost completely.
With wheels 10% wrong, turn 1 came out 8.6 deg short with and without VO, which is what you
get if the gyro is ignored. On a real robot wheels slip most in turns, so the gyro should own
the heading.

Edits src/my_robot/config/ekf.yaml (backup ekf.yaml.prewheelyaw.bak on first run).
Restart the bringup afterwards. Run from ~/autonomy.
"""
import os
import shutil
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EKF = os.path.join(ROOT, "src", "my_robot", "config", "ekf.yaml")


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


if len(sys.argv) != 2 or sys.argv[1] not in ("on", "off"):
    raise SystemExit("usage: python3 tools/set_wheel_yaw.py on|off")
want = sys.argv[1] == "on"

d = yaml.safe_load(open(EKF))
p = d["ekf_filter_node"]["ros__parameters"]
cfg = list(p["odom0_config"])
if cfg[5] == want:
    print("ekf.yaml: wheel yaw already %s, nothing changed" % sys.argv[1])
    sys.exit(0)
bak = EKF + ".prewheelyaw.bak"
if not os.path.exists(bak):
    shutil.copyfile(EKF, bak)
    print("backup:", bak)
cfg[5] = want
p["odom0_config"] = cfg
yaml.dump(d, open(EKF, "w"), Dumper=NoAliasDumper, sort_keys=False, default_flow_style=None)
print("ekf.yaml: wheel yaw %s  (odom0_config = %s)" % (sys.argv[1], cfg[:6]))
print("odom1 (VO) present:", "odom1" in p, " imu0_config vyaw:", p["imu0_config"][11])