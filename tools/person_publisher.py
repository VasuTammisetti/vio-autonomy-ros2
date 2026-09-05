#!/usr/bin/env python3
"""person_publisher -- ground-truth detection source for the world model."""
import subprocess
import threading
import random
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray

WORLD = "my_world"
TARGET_NAME = "person_standing"
NOISE_STD = 0.05
PUBLISH_HZ = 10.0


class PersonPublisher(Node):
    def __init__(self):
        super().__init__("person_publisher",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(MarkerArray, "/fused_objects", 10)
        self.lock = threading.Lock()
        self.xy = None
        self.t = threading.Thread(target=self._read_gz, daemon=True)
        self.t.start()
        self.create_timer(1.0 / PUBLISH_HZ, self.publish)
        self.get_logger().info(f"person_publisher up: tracking '{TARGET_NAME}' -> /fused_objects")

    def _read_gz(self):
        cmd = ["gz", "topic", "-e", "-t", f"/world/{WORLD}/pose/info"]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                text=True, bufsize=1)
        in_target = False
        cur_x = None
        for line in proc.stdout:
            s = line.strip()
            if s.startswith("name:"):
                in_target = (TARGET_NAME in s)
            elif in_target and s.startswith("x:"):
                try:
                    cur_x = float(s.split(":", 1)[1])
                except ValueError:
                    cur_x = None
            elif in_target and s.startswith("y:") and cur_x is not None:
                try:
                    y = float(s.split(":", 1)[1])
                    with self.lock:
                        self.xy = (cur_x, y)
                except ValueError:
                    pass
                in_target = False
                cur_x = None

    def publish(self):
        with self.lock:
            xy = self.xy
        if xy is None:
            return
        x = xy[0] + random.gauss(0.0, NOISE_STD)
        y = xy[1] + random.gauss(0.0, NOISE_STD)
        arr = MarkerArray()
        s = Marker()
        s.header.frame_id = "map"
        s.ns = "person"
        s.id = 0
        s.type = Marker.SPHERE
        s.action = Marker.ADD
        s.pose.position.x = float(x)
        s.pose.position.y = float(y)
        s.pose.position.z = 0.3
        s.pose.orientation.w = 1.0
        s.scale.x = s.scale.y = s.scale.z = 0.35
        s.color.b = 1.0
        s.color.a = 0.9
        s.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        arr.markers.append(s)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = PersonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()