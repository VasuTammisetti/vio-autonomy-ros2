#!/usr/bin/env python3
"""
cmd_vel bridge for my_robot.

Nav2 (Jazzy, default config) publishes geometry_msgs/Twist on /cmd_vel.
The Jazzy diff_drive_controller expects geometry_msgs/TwistStamped on
/diff_drive_controller/cmd_vel. This node converts and republishes.

use_sim_time is forced True so the stamp is on the /clock timeline --
otherwise the controller sees "future" wall-clock stamps and may drop
the command (same class of bug that made fusion markers vanish in RViz).

If `ros2 topic info /cmd_vel` shows TwistStamped instead of Twist, you do
NOT need this bridge -- just remap Nav2's /cmd_vel to the controller topic.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TwistStamped

OUT_TOPIC = "/diff_drive_controller/cmd_vel"
IN_TOPIC = "/cmd_vel"
FRAME_ID = "base_link"


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__(
            "cmd_vel_bridge",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.pub = self.create_publisher(TwistStamped, OUT_TOPIC, 10)
        self.sub = self.create_subscription(Twist, IN_TOPIC, self.cb, 10)
        self.get_logger().info(
            f"cmd_vel bridge up: {IN_TOPIC} (Twist) -> {OUT_TOPIC} (TwistStamped)"
        )

    def cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = FRAME_ID
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()