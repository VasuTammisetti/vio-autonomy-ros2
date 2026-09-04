#!/usr/bin/env python3
"""
YOLOv8n live detection node.
Subscribes to the robot's camera, runs detection on GPU, publishes a
readable detection string and an annotated image with boxes.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
import torch


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading YOLOv8n on device: {self.device}')
        self.model = YOLO('yolov8n.pt')
        self.use_half = self.device == 'cuda'
        self.conf = 0.4

        self.sub = self.create_subscription(Image, '/image', self.on_image, 10)
        self.det_pub = self.create_publisher(String, '/detections', 10)
        self.img_pub = self.create_publisher(Image, '/yolo/annotated', 10)

        self.get_logger().info('YOLO node ready, waiting for images on /image')

    def ros_to_np(self, msg):
        # Manual ROS Image -> numpy BGR (avoids cv_bridge encoding quirks)
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            arr = arr[:, :, ::-1]  # RGB -> BGR
        return np.ascontiguousarray(arr)

    def np_to_ros(self, img, header):
        # Manual numpy BGR -> ROS Image
        out = Image()
        out.header = header
        out.height, out.width = img.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = img.tobytes()
        return out

    def on_image(self, msg):
        frame = self.ros_to_np(msg)

        results = self.model.predict(
            frame, device=self.device, half=self.use_half,
            conf=self.conf, verbose=False
        )
        r = results[0]

        labels = []
        for box in r.boxes:
            cls_id = int(box.cls[0])
            name = self.model.names[cls_id]
            conf = float(box.conf[0])
            labels.append(f'{name}:{conf:.2f}')

        summary = ', '.join(labels) if labels else 'nothing detected'
        self.det_pub.publish(String(data=summary))

        annotated = r.plot()  # returns BGR numpy array with boxes
        self.img_pub.publish(self.np_to_ros(annotated, msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
