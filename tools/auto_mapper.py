#!/usr/bin/env python3
"""
auto_mapper -- drives the robot in a slow expanding spiral / sweep so
slam_toolbox (mapping mode) can build a map of the enlarged room WITHOUT
keyboard teleop. Publishes TwistStamped straight to the controller.

Run while: Gazebo + EKF up, and slam_toolbox online_async (mapping) up.
Drives for DURATION seconds then stops. Watch the map build in RViz.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped

DURATION = 90.0        # seconds of driving
FWD = 0.25             # forward speed m/s
TURN = 0.5             # turn rate rad/s
RATE_HZ = 20.0


class AutoMapper(Node):
    def __init__(self):
        super().__init__("auto_mapper",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.t0 = time.time()
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(f"auto_mapper: driving a sweep for {DURATION}s to build map")

    def tick(self):
        t = time.time() - self.t0
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        if t > DURATION:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.pub.publish(cmd)
            self.get_logger().info("auto_mapper: done. Save the map now.")
            rclpy.shutdown()
            return
        # pattern: drive forward, periodically turn — sweeps the space
        # alternate straight runs and turns to cover the room
        phase = t % 12.0
        if phase < 8.0:
            cmd.twist.linear.x = FWD          # drive straight
            cmd.twist.angular.z = 0.0
        else:
            cmd.twist.linear.x = 0.05         # slow while turning
            cmd.twist.angular.z = TURN        # turn ~90deg over 4s
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = AutoMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()