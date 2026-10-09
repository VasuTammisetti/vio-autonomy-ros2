#!/usr/bin/env python3
"""
safety_gate -- Step C: the safety gate (confidence + human-in-the-loop).

Sits between Nav2 and the robot on the velocity path:

    Nav2 -> /cmd_vel -> [SAFETY GATE] -> /cmd_vel_safe -> cmd_vel_bridge -> robot

Decides, for every command, whether to pass / slow / stop / hold, based on how
close the nearest tracked person is (from /world_model/tracks) to the robot
(base_footprint in map). Publishes its state so it's visible.

State machine:
  AUTONOMOUS  (person far)            -> pass command through unchanged
  DEGRADED    (person within WARN)    -> pass but cap speed (slow)
  SAFE_STOP   (person within STOP)    -> zero the command (hold position)
  HOLD        (person within CRITICAL)-> zero command. How HOLD ends depends on REQUIRE_APPROVAL:
                                         False (default, fully autonomous): HOLD releases by itself
                                           once the nearest person/cart has been outside the critical
                                           radius for HOLD_CLEAR_TIME seconds.
                                         True (supervised mode): also needs a human approval via
                                           `ros2 service call /safety/approve std_srvs/srv/Trigger`.

"Person" here means the nearest tracked agent (person or cart) from the world model.

Publishes:
  /cmd_vel_safe      (geometry_msgs/Twist)   gated command for the bridge
  /safety/state      (std_msgs/String)       current state name
  /safety/marker     (visualization_msgs/Marker) color-coded status dome in RViz

Service:
  /safety/approve    (std_srvs/srv/Trigger)  releases a HOLD once

use_sim_time forced True (timestamps on /clock).
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

# ---- danger radii (metres, robot-to-person) -------------------------------
WARN_RADIUS = 1.5        # closer than this -> DEGRADED (slow)
STOP_RADIUS = 0.9        # closer than this -> SAFE_STOP
CRITICAL_RADIUS = 0.5    # closer than this -> HOLD (needs human approval)
DEGRADED_SPEED_SCALE = 0.4  # speed multiplier in DEGRADED
REQUIRE_APPROVAL = False    # True = supervised mode: a human must approve leaving HOLD
HOLD_CLEAR_TIME = 1.0       # s the critical zone must stay clear before HOLD releases
# ---------------------------------------------------------------------------


class SafetyGate(Node):
    def __init__(self):
        super().__init__(
            "safety_gate",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.state = "AUTONOMOUS"
        self.tracks = []               # tracked agents (map frame)
        self.person_xy = None          # nearest tracked agent (map frame)
        self.clear_since = None        # sim time the critical zone became clear (HOLD release)
        self.approved = False          # one-shot human approval flag
        self.last_cmd = Twist()

        # TF: robot pose in map
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(MarkerArray, "/world_model/tracks", self.on_tracks, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.state_pub = self.create_publisher(String, "/safety/state", 10)
        self.marker_pub = self.create_publisher(Marker, "/safety/marker", 10)
        self.srv = self.create_service(Trigger, "/safety/approve", self.on_approve)

        self.create_timer(0.1, self.tick)   # 10 Hz state + marker publish
        self.get_logger().info(
            "safety_gate up: /cmd_vel -> [gate] -> /cmd_vel_safe | mode: %s"
            % ("SUPERVISED (approve with: ros2 service call /safety/approve std_srvs/srv/Trigger)"
               if REQUIRE_APPROVAL else "AUTONOMOUS (HOLD releases when the zone is clear)")
        )

    def on_tracks(self, msg: MarkerArray):
        # all tracked agents (CUBE markers in ns 'track'); the nearest one is picked in nearest_distance()
        self.tracks = [(m.pose.position.x, m.pose.position.y) for m in msg.markers if m.ns == "track"]

    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    def nearest_distance(self):
        r = self.robot_xy()
        if r is None or not self.tracks:
            self.person_xy = None
            return None
        self.person_xy = min(self.tracks, key=lambda p: math.hypot(r[0] - p[0], r[1] - p[1]))
        return math.hypot(r[0] - self.person_xy[0], r[1] - self.person_xy[1])

    def on_approve(self, request, response):
        self.approved = True
        response.success = True
        response.message = "Approval received; HOLD will release once person clears critical zone."
        self.get_logger().warn("HUMAN APPROVAL received.")
        return response

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg   # store latest Nav2 command; gating applied in tick()

    def compute_state(self, dist):
        if dist is None:
            return "AUTONOMOUS"            # no person known -> normal
        if dist < CRITICAL_RADIUS:
            return "HOLD"
        if dist < STOP_RADIUS:
            return "SAFE_STOP"
        if dist < WARN_RADIUS:
            return "DEGRADED"
        return "AUTONOMOUS"

    def tick(self):
        dist = self.nearest_distance()
        new_state = self.compute_state(dist)

        # HOLD latch: stay in HOLD until the critical zone has been clear for HOLD_CLEAR_TIME
        # (and, in supervised mode, a human has approved)
        if self.state == "HOLD":
            now = self.get_clock().now().nanoseconds * 1e-9
            clear = dist is None or dist >= CRITICAL_RADIUS
            if not clear:
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = now
            released = clear and now - self.clear_since >= HOLD_CLEAR_TIME
            if released and (self.approved or not REQUIRE_APPROVAL):
                self.approved = False           # consume approval
                self.clear_since = None
                new_state = self.compute_state(dist)
                self.get_logger().info("HOLD released (%s)" % ("approved" if REQUIRE_APPROVAL else "zone clear"))
            else:
                new_state = "HOLD"

        self.state = new_state

        # apply gating to the latest command
        out = Twist()
        if self.state == "AUTONOMOUS":
            out = self.last_cmd
        elif self.state == "DEGRADED":
            out.linear.x = self.last_cmd.linear.x * DEGRADED_SPEED_SCALE
            out.linear.y = self.last_cmd.linear.y * DEGRADED_SPEED_SCALE
            out.angular.z = self.last_cmd.angular.z * DEGRADED_SPEED_SCALE
        else:  # SAFE_STOP or HOLD -> zero
            out = Twist()

        self.cmd_pub.publish(out)
        self.state_pub.publish(String(data=self.state))
        self.publish_marker(dist)

    def publish_marker(self, dist):
        r = self.robot_xy()
        if r is None:
            return
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "safety"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = r[0]
        m.pose.position.y = r[1]
        m.pose.position.z = 0.6
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        # color by state
        colors = {
            "AUTONOMOUS": (0.0, 1.0, 0.0),
            "DEGRADED":   (1.0, 1.0, 0.0),
            "SAFE_STOP":  (1.0, 0.3, 0.0),
            "HOLD":       (1.0, 0.0, 0.0),
        }
        cr, cg, cb = colors.get(self.state, (1.0, 1.0, 1.0))
        m.color.r, m.color.g, m.color.b = cr, cg, cb
        m.color.a = 0.9
        m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()