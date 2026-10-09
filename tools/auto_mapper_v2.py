#!/usr/bin/env python3
"""
auto_mapper_v2 -- drive the robot around the room by itself so slam_toolbox (mapping mode) can build a map,
steering away from obstacles with the LiDAR (safe with box stacks and pallets in the room).

Compared with auto_mapper.py:
  - reads /scan and turns away when anything is closer than SAFE_DIST in front
  - parks all moving agents (people, cart; see factory_layout.py) outside the hall before mapping,
    so they are not mapped as permanent obstacles. Start tools/sim_agents.py afterwards to bring them back.
  - ramps speed up and down (no hard stops) and turns gently, which gives SLAM cleaner, better
    overlapping scans
  - counts DURATION in simulation time, so coverage is the same at any Gazebo speed
  - stops cleanly (zero velocity) when done or on Ctrl+C

Run while: Gazebo + EKF are up, and slam_toolbox online_async (mapping mode) is up.
    python3 ~/autonomy/tools/auto_mapper_v2.py
"""
import math
import random
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

from factory_layout import PARK

DURATION = 300.0      # seconds of driving, SIMULATION time (20 x 15 m hall)
FWD = 0.20            # forward speed, m/s
TURN = 0.45           # turn rate when avoiding, rad/s
ACC_LIN = 0.25        # max change of forward speed, m/s per second
ACC_ANG = 0.8         # max change of turn rate, rad/s per second
SAFE_DIST = 0.90      # start turning when something is this close in front (m)
CLEAR_DIST = 1.20     # stop turning once the front is this clear (m)
SIDE_DIST = 0.50      # also turn away when anything this close to the sides (m)
FRONT_HALF = math.radians(50)
WORLD = "my_world"


def sector_min(ranges, angle_min, inc, a0, a1, rmax):
    """Smallest valid range with beam angle in [a0, a1]."""
    best = rmax
    for i, r in enumerate(ranges):
        a = angle_min + i * inc
        if a0 <= a <= a1 and math.isfinite(r) and r > 0.05:
            best = min(best, r)
    return best


def decide(ranges, angle_min, inc, rmax, turning, turn_dir):
    """Pure decision step. Returns (v, w, turning, turn_dir)."""
    front = sector_min(ranges, angle_min, inc, -FRONT_HALF, FRONT_HALF, rmax)
    left = sector_min(ranges, angle_min, inc, FRONT_HALF, math.radians(110), rmax)
    right = sector_min(ranges, angle_min, inc, -math.radians(110), -FRONT_HALF, rmax)
    if not turning and (front < SAFE_DIST or min(left, right) < SIDE_DIST):
        turning, turn_dir = True, (1.0 if left >= right else -1.0)
    if turning and front > CLEAR_DIST and min(left, right) > SIDE_DIST:
        turning = False
    if turning:
        return 0.0, TURN * turn_dir, turning, turn_dir
    return FWD, 0.0, turning, turn_dir


def ramp(current, target, max_step):
    """Move current toward target by at most max_step."""
    return current + max(-max_step, min(max_step, target - current))


def set_pose(name, x, y):
    req = 'name: "%s", position: {x: %.2f, y: %.2f, z: 0.0}' % (name, x, y)
    subprocess.run(["gz", "service", "-s", "/world/%s/set_pose" % WORLD, "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class AutoMapperV2(Node):
    def __init__(self):
        super().__init__("auto_mapper_v2",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.scan = None
        self.turning, self.turn_dir = False, 1.0
        self.wander_w, self.next_wander = 0.0, 0.0
        self.v, self.w = 0.0, 0.0          # commanded speeds after ramping
        self.t0 = None
        self.last = None
        self.next_report = 30.0
        self.done = False
        self.create_timer(0.05, self.tick)   # runs on sim time (use_sim_time is on)
        self.get_logger().info("auto_mapper_v2: waiting for /scan ...")

    def on_scan(self, msg):
        self.scan = msg

    def send(self, v, w):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.pub.publish(cmd)

    def tick(self):
        if self.done or self.scan is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9   # sim seconds
        if self.t0 is None:
            self.t0, self.last = now, now
            self.get_logger().info("scan received, driving for %.0f s of sim time" % DURATION)
        dt = min(max(now - self.last, 0.0), 0.2)
        self.last = now
        elapsed = now - self.t0
        if elapsed > self.next_report:
            self.get_logger().info("%.0f / %.0f s" % (elapsed, DURATION))
            self.next_report += 30.0
        if elapsed > DURATION:
            # ramp down to a stop, then finish
            self.v, self.w = ramp(self.v, 0.0, ACC_LIN * dt), ramp(self.w, 0.0, ACC_ANG * dt)
            self.send(self.v, self.w)
            if abs(self.v) < 1e-3 and abs(self.w) < 1e-3:
                self.done = True
                self.get_logger().info("done. Save the map now.")
            return
        s = self.scan
        v, w, self.turning, self.turn_dir = decide(s.ranges, s.angle_min, s.angle_increment, s.range_max,
                                                   self.turning, self.turn_dir)
        if not self.turning:
            if elapsed > self.next_wander:  # gentle random curves so the robot covers the room
                self.wander_w = random.uniform(-0.3, 0.3)
                self.next_wander = elapsed + random.uniform(4.0, 9.0)
            w = self.wander_w
        self.v = ramp(self.v, v, ACC_LIN * dt)
        self.w = ramp(self.w, w, ACC_ANG * dt)
        self.send(self.v, self.w)


def main():
    print("parking people and cart outside the hall for mapping ...")
    for name, (x, y) in PARK.items():
        set_pose(name, x, y)
    rclpy.init()
    node = AutoMapperV2()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        print("agents stay parked; start tools/sim_agents.py to bring them back")


if __name__ == "__main__":
    main()