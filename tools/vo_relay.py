#!/usr/bin/env python3
"""
vo_relay -- prepares rtabmap's visual odometry (/vo/odom) for the EKF (/vo/odom_ekf).

Why a relay:
  1. Honest uncertainty. rtabmap reports itself as extremely sure (std dev about 0.0003 m), so the EKF would
     follow VO blindly, including its measured -4% distance error. The relay sets a realistic covariance
     (POS_STD, YAW_STD), so the EKF weighs VO against wheels and IMU.
  2. Glitch protection. When VO loses tracking it publishes a "null" pose (covariance 9999); when it recovers,
     its pose can jump. The EKF fuses VO in differential mode (pose changes -> velocity), so a jump would look
     like a huge velocity. The relay keeps its own pose and only adds the frame-to-frame motion that looks
     physically possible (MAX_STEP_M, MAX_STEP_RAD); lost frames and jumps are skipped.

Output: nav_msgs/Odometry on /vo/odom_ekf, frame odom -> base_footprint, 2D (x, y, yaw).
Run:    python3 ~/autonomy/tools/vo_relay.py      (needs rgbd_odometry publishing /vo/odom)
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

POS_STD = 0.02       # m, per-message position uncertainty given to the EKF
YAW_STD = 0.01       # rad
MAX_STEP_M = 0.15    # larger jumps between two VO frames are treated as glitches
MAX_STEP_RAD = 0.3


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def relative_step(prev, cur):
    """Motion from prev to cur (x, y, yaw), expressed in prev's own frame."""
    dx, dy = cur[0] - prev[0], cur[1] - prev[1]
    c, s = math.cos(prev[2]), math.sin(prev[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(cur[2] - prev[2])


def apply_step(pose, step):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * step[0] - s * step[1], pose[1] + s * step[0] + c * step[1], wrap(pose[2] + step[2]))


class VoRelay(Node):
    def __init__(self):
        super().__init__("vo_relay", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Odometry, "/vo/odom_ekf", 20)
        self.create_subscription(Odometry, "/vo/odom", self.on_vo, 20)
        self.prev = None                 # last good VO pose (VO's own frame)
        self.out = (0.0, 0.0, 0.0)       # relay pose that the EKF sees
        self.n_ok, self.n_lost, self.n_jump = 0, 0, 0
        self.create_timer(30.0, self.report)
        self.get_logger().info("vo_relay: /vo/odom -> /vo/odom_ekf (pos std %.3f m, yaw std %.3f rad)" % (POS_STD, YAW_STD))

    def on_vo(self, m):
        if m.pose.covariance[0] >= 9999.0:          # VO lost tracking
            self.n_lost += 1
            self.prev = None                         # restart the chain when tracking returns
            return
        cur = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        if self.prev is not None:
            step = relative_step(self.prev, cur)
            if math.hypot(step[0], step[1]) > MAX_STEP_M or abs(step[2]) > MAX_STEP_RAD:
                self.n_jump += 1                     # glitch: skip this step, keep the chain from here
            else:
                self.out = apply_step(self.out, step)
                self.n_ok += 1
        self.prev = cur
        o = Odometry()
        o.header.stamp = m.header.stamp
        o.header.frame_id = "odom"
        o.child_frame_id = "base_footprint"
        o.pose.pose.position.x, o.pose.pose.position.y = self.out[0], self.out[1]
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.out[2] / 2), math.cos(self.out[2] / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = POS_STD ** 2
        cov[14] = cov[21] = cov[28] = 1e3            # z, roll, pitch: not measured (2D)
        cov[35] = YAW_STD ** 2
        o.pose.covariance = cov
        self.pub.publish(o)

    def report(self):
        self.get_logger().info("last 30 s: %d frames used, %d lost, %d jumps skipped" % (self.n_ok, self.n_lost, self.n_jump))
        self.n_ok, self.n_lost, self.n_jump = 0, 0, 0


def main():
    rclpy.init()
    n = VoRelay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()