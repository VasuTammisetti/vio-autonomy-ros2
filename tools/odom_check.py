#!/usr/bin/env python3
"""
odom_check -- measures how well each source tracks the robot's heading during an on-the-spot spin.

The robot spins 2 full turns in place (commanded), stops, then we compare the total rotation from:
  wheels   /diff_drive_controller/odom  (wheel encoders)
  imu      /imu angular velocity, integrated
  ekf      /odometry/filtered           (what SLAM and Nav2 use)
  vo       /vo/odom                     (rtabmap rgbd_odometry, if running)
  lidar    the LiDAR scan before vs after: the shift that best lines the two scans up gives the
           real rotation (1 deg resolution), independent of any odometry
Then it drives 2 m straight and reports the distance from wheels and EKF.

Run with only Gazebo + EKF up (no SLAM, no Nav2, nothing else driving the robot):
    python3 ~/autonomy/tools/odom_check.py
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, LaserScan

SPIN_RATE = 0.5          # rad/s
SPIN_TURNS = 2
FWD_SPEED = 0.2          # m/s
FWD_DIST = 2.0           # m


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Unwrap:
    def __init__(self):
        self.last, self.total = None, 0.0

    def add(self, yaw):
        if self.last is not None:
            d = yaw - self.last
            self.total += math.atan2(math.sin(d), math.cos(d))
        self.last = yaw


class OdomCheck(Node):
    def __init__(self):
        super().__init__("odom_check", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.wheel, self.ekf, self.vo = Unwrap(), Unwrap(), Unwrap()
        self.vo_xy = None
        self.vo_lost = 0
        self.imu_total, self.imu_t = 0.0, None
        self.wheel_xy = self.ekf_xy = None
        self.scan = None
        self.twist_v, self.twist_w = [], []   # wheel-reported speeds while recording
        self.recording = False
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self.on_wheel, 20)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_ekf, 20)
        self.create_subscription(Odometry, "/vo/odom", self.on_vo, 20)
        self.create_subscription(Imu, "/imu", self.on_imu, 50)
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), 10)

    def on_wheel(self, m):
        if self.recording:
            self.twist_v.append(m.twist.twist.linear.x)
            self.twist_w.append(m.twist.twist.angular.z)
        self.wheel.add(yaw_of(m.pose.pose.orientation))
        self.wheel_xy = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_ekf(self, m):
        self.ekf.add(yaw_of(m.pose.pose.orientation))
        self.ekf_xy = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_vo(self, m):
        # rtabmap publishes an all-zero covariance pose with 9999 variance when tracking is lost
        if m.pose.covariance[0] >= 9999:
            self.vo_lost += 1
            return
        self.vo.add(yaw_of(m.pose.pose.orientation))
        self.vo_xy = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.imu_t is not None and 0 < t - self.imu_t < 0.5:
            self.imu_total += m.angular_velocity.z * (t - self.imu_t)
        self.imu_t = t

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, v, w):
        c = TwistStamped()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = "base_link"
        c.twist.linear.x, c.twist.angular.z = float(v), float(w)
        self.pub.publish(c)

    def run_for(self, seconds, v, w):
        t0 = self.now()
        t_end = t0 + seconds
        self.twist_v, self.twist_w = [], []
        while rclpy.ok() and self.now() < t_end:
            self.recording = t0 + 2.0 < self.now() < t_end - 1.0   # skip speed-up and slow-down
            self.send(v, w)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.recording = False
        for _ in range(20):
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def settle(self, seconds):
        t_end = self.now() + seconds
        while rclpy.ok() and self.now() < t_end:
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)


def scan_array(s):
    r = np.array(s.ranges, dtype=float)
    r[~np.isfinite(r)] = s.range_max
    return np.clip(r, 0, s.range_max)


def scan_rotation(a, b, inc):
    """Rotation (rad) of the robot between scans a and b, by best circular shift."""
    n = len(a)
    errs = [np.mean(np.abs(np.roll(b, k) - a)) for k in range(n)]
    k = int(np.argmin(errs))
    if k > n // 2:
        k -= n
    return k * inc, errs[int(np.argmin(errs))]


def main():
    rclpy.init()
    n = OdomCheck()
    print("waiting for /clock, odometry, IMU and scan ...")
    while rclpy.ok() and (n.scan is None or n.wheel.last is None or n.ekf.last is None or n.imu_t is None):
        rclpy.spin_once(n, timeout_sec=0.1)
    n.settle(1.0)
    scan0 = scan_array(n.scan)
    w0, e0, i0 = n.wheel.total, n.ekf.total, n.imu_total
    v0 = n.vo.total
    print("visual odometry (/vo/odom): %s" % ("receiving" if n.vo.last is not None else "not running, VO rows will be skipped"))
    cmd = 2 * math.pi * SPIN_TURNS
    print("spinning %d turns at %.1f rad/s ..." % (SPIN_TURNS, SPIN_RATE))
    n.run_for(cmd / SPIN_RATE, 0.0, SPIN_RATE)
    n.settle(2.0)
    scan1 = scan_array(n.scan)
    wheel, ekf, imu = n.wheel.total - w0, n.ekf.total - e0, n.imu_total - i0
    vo = n.vo.total - v0 if n.vo.last is not None else None
    lid_res, fit = scan_rotation(scan0, scan1, n.scan.angle_increment)
    # LiDAR gives rotation modulo 360 deg: pick the full-turn count closest to the IMU total
    turns = round((imu - lid_res) / (2 * math.pi))
    lidar = lid_res + turns * 2 * math.pi
    d = math.degrees
    print("\nSPIN TEST  (total rotation, degrees; lidar = reference, +/- 1 deg)")
    print("  commanded  %8.1f" % d(cmd))
    print("  lidar      %8.1f   (scan match error %.3f m)" % (d(lidar), fit))
    rows = [("wheels", wheel), ("imu", imu), ("ekf", ekf)] + ([("vo", vo)] if vo is not None else [])
    for name, val in rows:
        print("  %-9s  %8.1f   error vs lidar %+6.1f" % (name, d(val), d(val - lidar)))
    if n.twist_w:
        print("  wheel twist reports %.3f rad/s while turning (commanded %.3f)" % (np.mean(n.twist_w), SPIN_RATE))

    w_xy0, e_xy0, v_xy0 = n.wheel_xy, n.ekf_xy, n.vo_xy
    print("\ndriving %.1f m straight ..." % FWD_DIST)
    n.run_for(FWD_DIST / FWD_SPEED, FWD_SPEED, 0.0)
    n.settle(2.0)
    dist = lambda a, b: math.hypot(b[0] - a[0], b[1] - a[1])
    print("STRAIGHT TEST (metres)")
    print("  commanded  %.2f" % FWD_DIST)
    print("  wheels     %.2f" % dist(w_xy0, n.wheel_xy))
    print("  ekf        %.2f" % dist(e_xy0, n.ekf_xy))
    if v_xy0 is not None and n.vo_xy is not None:
        print("  vo         %.2f" % dist(v_xy0, n.vo_xy))
    if n.vo.last is not None:
        print("VO lost tracking %d times during the test" % n.vo_lost)
    if n.twist_v:
        print("  wheel twist reports %.3f m/s while driving (commanded %.3f)" % (np.mean(n.twist_v), FWD_SPEED))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()