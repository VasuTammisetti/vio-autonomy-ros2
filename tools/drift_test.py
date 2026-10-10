#!/usr/bin/env python3
"""
drift_test -- how far does the odometry (EKF) drift on a 3 x 3 m square, with or without visual odometry?

Reference: slam_toolbox LiDAR localization on the saved map. It publishes map -> odom, the correction it
has to apply to the EKF's odom frame to keep the robot where the LiDAR sees it. So the size of map -> odom
IS the EKF's accumulated drift (it starts at zero at the robot's start pose).

The robot drives one 3 x 3 m square (left turns), commanded directly (no Nav2), starting at the hall centre
facing east: (0,0) -> (3,0) -> (3,3) -> (0,3) -> (0,0). That area is free when the agents are off.

Run with:  bringup_all.launch.py use_agents:=false use_predictions:=false use_vo:=true|false
Then:      python3 ~/autonomy/tools/drift_test.py --label wheels110_vo
Writes ~/autonomy/logs/drift_<label>.csv (sim time, drift x/y/yaw) and prints the final drift.
"""
import argparse
import csv
import math
import os

import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter

SIDE = 3.0        # m
SPEED = 0.2       # m/s
TURN = 0.4        # rad/s
LOG_DIR = os.path.expanduser("~/autonomy/logs")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class DriftTest(Node):
    def __init__(self, label):
        super().__init__("drift_test", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        os.makedirs(LOG_DIR, exist_ok=True)
        self.path = os.path.join(LOG_DIR, "drift_%s.csv" % label)
        self.f = open(self.path, "w", newline="")
        self.log = csv.writer(self.f)
        self.log.writerow(["sim_t", "drift_x", "drift_y", "drift_m", "drift_yaw_deg", "phase"])
        self.t0 = None
        self.next_log = 0.0
        self.phase = "start"

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def drift(self):
        try:
            t = self.buf.lookup_transform("map", "odom", rclpy.time.Time())
        except Exception:
            return None
        x, y = t.transform.translation.x, t.transform.translation.y
        return x, y, math.hypot(x, y), math.degrees(yaw_of(t.transform.rotation))

    def record(self, force=False):
        now = self.now()
        if self.t0 is None:
            return
        if force or now - self.t0 >= self.next_log:
            d = self.drift()
            if d:
                self.log.writerow(["%.2f" % (now - self.t0), "%.4f" % d[0], "%.4f" % d[1], "%.4f" % d[2], "%.2f" % d[3], self.phase])
                self.f.flush()
            self.next_log += 0.5

    def send(self, v, w):
        c = TwistStamped()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = "base_link"
        c.twist.linear.x, c.twist.angular.z = float(v), float(w)
        self.pub.publish(c)

    def run_for(self, seconds, v, w, phase):
        self.phase = phase
        t_end = self.now() + seconds
        while rclpy.ok() and self.now() < t_end:
            self.send(v, w)
            self.record()
            rclpy.spin_once(self, timeout_sec=0.02)

    def settle(self, seconds):
        self.run_for(seconds, 0.0, 0.0, "stop")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True, help="name for the log, e.g. wheels110_novo")
    args = ap.parse_args()
    rclpy.init()
    n = DriftTest(args.label)
    print("waiting for /clock and the map -> odom transform (LiDAR localization) ...")
    while rclpy.ok() and (n.now() == 0.0 or n.drift() is None):
        rclpy.spin_once(n, timeout_sec=0.1)
    n.t0 = n.now()
    n.settle(3.0)
    d0 = n.drift()
    print("start: map -> odom = %.3f m, %.2f deg (should be close to 0)" % (d0[2], d0[3]))
    for k in range(4):
        print("side %d of 4 ..." % (k + 1))
        n.run_for(SIDE / SPEED, SPEED, 0.0, "side%d" % (k + 1))
        n.settle(1.0)
        n.run_for((math.pi / 2) / TURN, 0.0, TURN, "turn%d" % (k + 1))
        n.settle(1.0)
    n.settle(5.0)   # let the LiDAR localization settle on the final pose
    n.record(force=True)
    d = n.drift()
    print("\nDRIFT after one 3 x 3 m square (12 m driven), label '%s':" % args.label)
    pos = math.hypot(d[0] - d0[0], d[1] - d0[1])
    print("  position drift  %.3f m  (%.1f %% of distance)" % (pos, 100.0 * pos / (4 * SIDE)))
    print("  heading drift   %.2f deg" % (d[3] - d0[3]))
    print("log written to", n.path)
    n.f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()