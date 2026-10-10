#!/usr/bin/env python3
"""
setup_vio_fusion.py -- step 2 of VIO: fuse visual odometry into the EKF.

ekf.yaml:
  odom1 = /vo/odom_ekf (from tools/vo_relay.py), pose x, y, yaw in differential mode, like the wheels.
  If VO is not running, the EKF simply has no odom1 data and works on wheels + IMU as before.
bringup_all.launch.py:
  new arg use_vo:=true|false (default true): at t=8 s start rtabmap rgbd_odometry + vo_relay.

Backups on first run: *.previo.bak. Safe to run again. Run from ~/autonomy:  python3 tools/setup_vio_fusion.py
"""
import os
import shutil

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EKF = os.path.join(ROOT, "src", "my_robot", "config", "ekf.yaml")
BRINGUP = os.path.join(ROOT, "src", "my_robot", "launch", "bringup_all.launch.py")


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def backup(p):
    b = p + ".previo.bak"
    if not os.path.exists(b):
        shutil.copyfile(p, b)
        print("backup:", b)


# ------------------------------------------------------------------ EKF
d = yaml.safe_load(open(EKF))
p = d["ekf_filter_node"]["ros__parameters"]
if "odom1" in p:
    print("ekf.yaml: already has odom1")
else:
    backup(EKF)
    p["odom1"] = "/vo/odom_ekf"
    p["odom1_config"] = [True, True, False,
                         False, False, True,
                         False, False, False,
                         False, False, False,
                         False, False, False]
    p["odom1_differential"] = True
    p["odom1_relative"] = False
    p["odom1_queue_size"] = 10
    yaml.dump(d, open(EKF, "w"), Dumper=NoAliasDumper, sort_keys=False, default_flow_style=None)
    print("ekf.yaml: odom1 = /vo/odom_ekf (x, y, yaw, differential)")

# ------------------------------------------------------------------ bringup
s = open(BRINGUP).read()
if "use_vo" in s:
    print("bringup_all.launch.py: already has use_vo")
else:
    backup(BRINGUP)
    def rep(old, new, what):
        global s
        if old not in s:
            raise SystemExit("could not find %s in bringup_all.launch.py; nothing written there. Send this to Claude." % what)
        s = s.replace(old, new, 1)
    rep('''    use_mission = LaunchConfiguration("use_mission")''', '''    use_mission = LaunchConfiguration("use_mission")
    use_vo = LaunchConfiguration("use_vo")''', "use_mission")
    rep('''    # --- t=10: Nav2 servers + lifecycle manager ---''', '''    # --- t=8: visual odometry (rtabmap rgbd_odometry) + relay into the EKF ---
    vo = TimerAction(period=8.0, actions=[
        bash("ros2 run rtabmap_odom rgbd_odometry --ros-args "
             f"--params-file {HOME}/src/my_robot/config/vo.yaml "
             "-r rgb/image:=/camera/image -r depth/image:=/camera/depth_image "
             "-r rgb/camera_info:=/camera/camera_info -r odom:=/vo/odom"),
        bash(f"python3 {TOOLS}/vo_relay.py"),
    ], condition=IfCondition(use_vo))

    # --- t=10: Nav2 servers + lifecycle manager ---''', "Nav2 comment")
    rep('''        DeclareLaunchArgument("use_mission", default_value="false"),''', '''        DeclareLaunchArgument("use_mission", default_value="false"),
        DeclareLaunchArgument("use_vo", default_value="true"),''', "use_mission argument")
    rep("gazebo, slam, nav2_servers,", "gazebo, slam, vo, nav2_servers,", "launch list")
    open(BRINGUP, "w").write(s)
    print("bringup_all.launch.py: use_vo:=true starts rgbd_odometry + vo_relay at t=8 s")