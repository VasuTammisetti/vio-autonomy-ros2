#!/usr/bin/env python3
"""
tilt_check -- does the robot tip forward/back enough for the LiDAR to hit the floor?

Drives a few hard starts and stops (like the mapper and Nav2 do) and records the robot's pitch and
roll from the IMU. With the LiDAR 0.17 m above the floor, a tilt of t degrees makes horizontal beams
hit the floor at 0.17 / tan(t) metres:   0.5 deg -> 19 m,  1 deg -> 10 m,  2 deg -> 5 m.
Anything above about 0.4 deg puts floor hits inside the 20 m LiDAR range.

Run with only Gazebo + EKF up, robot standing in open space (the hall centre):
    python3 ~/autonomy/tools/tilt_check.py
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

LIDAR_HEIGHT = 0.17


def pitch_roll(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return pitch, roll


class TiltCheck(Node):
    def __init__(self):
        super().__init__("tilt_check", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.create_subscription(Imu, "/imu", self.on_imu, 100)
        self.pr = None
        self.base = None
        self.samples = []

    def on_imu(self, m):
        self.pr = pitch_roll(m.orientation)
        if self.base is not None:
            self.samples.append((self.pr[0] - self.base[0], self.pr[1] - self.base[1]))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def drive(self, seconds, v, w):
        t_end = self.now() + seconds
        while rclpy.ok() and self.now() < t_end:
            c = TwistStamped()
            c.header.stamp = self.get_clock().now().to_msg()
            c.header.frame_id = "base_link"
            c.twist.linear.x, c.twist.angular.z = float(v), float(w)
            self.pub.publish(c)
            rclpy.spin_once(self, timeout_sec=0.01)

    def phase(self, name, steps):
        self.samples = []
        for (sec, v, w) in steps:
            self.drive(sec, v, w)
        p = max(abs(s[0]) for s in self.samples) if self.samples else 0.0
        r = max(abs(s[1]) for s in self.samples) if self.samples else 0.0
        t = max(p, r)
        hit = LIDAR_HEIGHT / math.tan(t) if t > 1e-4 else float("inf")
        print("  %-28s max pitch %5.2f deg   max roll %5.2f deg   -> floor hits from %s"
              % (name, math.degrees(p), math.degrees(r), "%.1f m" % hit if hit < 100 else "never"))
        return t


def main():
    rclpy.init()
    n = TiltCheck()
    print("waiting for /imu and /clock ...")
    while rclpy.ok() and n.pr is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.drive(2.0, 0.0, 0.0)
    n.base = n.pr   # standing still = zero tilt reference
    print("standing tilt reference: pitch %.2f deg, roll %.2f deg" % (math.degrees(n.base[0]), math.degrees(n.base[1])))
    print("\nTILT TEST (relative to standing still)")
    worst = 0.0
    worst = max(worst, n.phase("standing still", [(2.0, 0.0, 0.0)]))
    worst = max(worst, n.phase("hard start + cruise 0.22", [(4.0, 0.22, 0.0)]))
    worst = max(worst, n.phase("hard stop", [(3.0, 0.0, 0.0)]))
    worst = max(worst, n.phase("hard start backwards", [(3.0, -0.15, 0.0), (2.0, 0.0, 0.0)]))
    worst = max(worst, n.phase("spin start + stop", [(4.0, 0.0, 0.7), (2.0, 0.0, 0.0)]))
    worst = max(worst, n.phase("drive + turn (curve)", [(4.0, 0.2, 0.5), (2.0, 0.0, 0.0)]))
    print("\nworst tilt %.2f deg. %s" % (math.degrees(worst),
          "OK: the LiDAR stays off the floor within 20 m." if math.degrees(worst) < 0.4 else
          "TOO MUCH: the LiDAR hits the floor inside its range, which draws false obstacles."))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()