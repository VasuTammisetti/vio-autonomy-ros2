#!/usr/bin/env python3
"""
Camera-LiDAR fusion node (2D lidar, angular association).
Runs YOLOv8n on the camera; for each detection, estimates the object's
bearing from the bounding-box center + camera FOV, reads the LiDAR range at
that bearing, transforms to the map frame, and publishes an RViz marker.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from ultralytics import YOLO
import torch

CAMERA_HFOV = 1.047  # radians (matches URDF camera horizontal_fov)


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading YOLOv8n on {self.device}')
        self.model = YOLO('yolov8n.pt')
        self.conf = 0.4

        self.latest_scan = None
        self.img_w = 640

        self.create_subscription(Image, '/image', self.on_image, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fused_objects', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Fusion node ready.')

    def on_scan(self, msg):
        self.latest_scan = msg

    def range_at_bearing(self, bearing):
        # Read the lidar range near a given bearing, searching a small window
        # and returning the median of valid hits (robust to single inf rays)
        s = self.latest_scan
        if s is None:
            return None
        center = int(round((bearing - s.angle_min) / s.angle_increment))
        window = 8  # +/- rays around the bearing (~8 deg each side)
        vals = []
        for i in range(center - window, center + window + 1):
            if 0 <= i < len(s.ranges):
                r = s.ranges[i]
                if not (math.isinf(r) or math.isnan(r)) and s.range_min < r < s.range_max:
                    vals.append(r)
        if not vals:
            return None
        vals.sort()
        return vals[len(vals) // 2]

    def on_image(self, msg):
        if self.latest_scan is None:
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.img_w = msg.width

        results = self.model.predict(frame, device=self.device, conf=self.conf, verbose=False)
        r = results[0]

        markers = MarkerArray()
        mid = 0
        for box in r.boxes:
            cls_id = int(box.cls[0])
            name = self.model.names[cls_id]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = (x1 + x2) / 2.0

            # Image x -> bearing. Center = 0, left = +, right = - (REP-103: +y left)
            norm = (cx - self.img_w / 2.0) / (self.img_w / 2.0)  # -1..1
            bearing = -norm * (CAMERA_HFOV / 2.0)

            rng = self.range_at_bearing(bearing)
            if rng is None:
                continue

            # Position in base_footprint frame (x forward, y left)
            px = rng * math.cos(bearing)
            py = rng * math.sin(bearing)

            pt = PointStamped()
            pt.header.frame_id = 'base_footprint'
            pt.header.stamp = rclpy.time.Time().to_msg()
            pt.point.x, pt.point.y, pt.point.z = px, py, 0.2

            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                pmap = do_transform_point(pt, tf)
            except Exception:
                continue

            # Sphere marker
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'fused'
            m.id = mid; mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = pmap.point
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
            m.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            markers.markers.append(m)

            # Text label
            t = Marker()
            t.header = m.header
            t.ns = 'fused_label'
            t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position = pmap.point
            t.pose.position.z += 0.4
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f'{name} ({rng:.1f}m)'
            t.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            markers.markers.append(t)

        if markers.markers:
            self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
