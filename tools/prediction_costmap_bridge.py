#!/usr/bin/env python3
"""
prediction_costmap_bridge -- Step B.5: close the perception->planning loop.

Subscribes to /world_model/predictions (the forecast ellipses from the world
model) and republishes those FUTURE positions as a PointCloud2 on
/predicted_obstacles, in the 'map' frame. Nav2's costmap is configured to read
this cloud as an obstacle source, so the planner routes around where the
tracked person is PREDICTED to be -- not just where they are now.

Only the nearer-term, higher-confidence part of the forecast is emitted as
solid obstacles (configurable horizon), so the robot avoids the likely path
without treating the far-future low-confidence tail as a hard wall.

use_sim_time forced True so cloud stamps sit on the /clock timeline (Nav2's
observation buffer rejects stale/future stamps otherwise).
"""
import struct
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# How much of the forecast to treat as obstacles.
# The world model emits FORECAST_STEPS ellipses (default 10 @ 0.5s = 5s horizon);
# we use the first N as solid obstacles (nearer-term = higher confidence).
USE_FIRST_N = 6
POINTS_PER_ELLIPSE = 8   # ring of points around each predicted position
OUT_TOPIC = "/predicted_obstacles"


class PredictionCostmapBridge(Node):
    def __init__(self):
        super().__init__(
            "prediction_costmap_bridge",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.sub = self.create_subscription(
            MarkerArray, "/world_model/predictions", self.on_pred, 10)
        self.pub = self.create_publisher(PointCloud2, OUT_TOPIC, 10)
        self.get_logger().info(
            f"prediction_costmap_bridge up: /world_model/predictions -> {OUT_TOPIC} "
            f"(first {USE_FIRST_N} steps as obstacles)"
        )

    def on_pred(self, msg: MarkerArray):
        import math
        pts = []
        # markers are CYLINDER ellipses named 'pred_cov', ordered by horizon step
        cov_markers = [m for m in msg.markers if m.ns == "pred_cov"]
        for m in cov_markers[:USE_FIRST_N]:
            cx = m.pose.position.x
            cy = m.pose.position.y
            rx = max(m.scale.x * 0.5, 0.1)   # ellipse radii from marker scale
            ry = max(m.scale.y * 0.5, 0.1)
            # centre point
            pts.append((cx, cy, 0.1))
            # ring around the ellipse so the costmap inflates the whole footprint
            for i in range(POINTS_PER_ELLIPSE):
                a = 2.0 * math.pi * i / POINTS_PER_ELLIPSE
                pts.append((cx + rx * math.cos(a), cy + ry * math.sin(a), 0.1))

        self.publish_cloud(pts)

    def publish_cloud(self, pts):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.is_dense = True
        buf = bytearray()
        for (x, y, z) in pts:
            buf += struct.pack("fff", x, y, z)
        cloud.data = bytes(buf)
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = PredictionCostmapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()