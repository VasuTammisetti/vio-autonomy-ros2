#!/usr/bin/env python3
"""
VLM scene-description node.
Holds the latest camera frame. On a std_srvs/Trigger service call, sends that
frame to the LLaVA FastAPI server and publishes the description on /scene_description.
"""
import io
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from PIL import Image as PILImage

VLM_URL = "http://localhost:8000/describe"
PROMPT = "Describe this scene for a robot. Is there a person? What objects and obstacles do you see?"


class VlmNode(Node):
    def __init__(self):
        super().__init__('vlm_node')
        self.latest = None

        self.sub = self.create_subscription(Image, '/image', self.on_image, 10)
        self.pub = self.create_publisher(String, '/scene_description', 10)
        self.srv = self.create_service(Trigger, 'describe_scene', self.on_request)

        self.get_logger().info('VLM node ready. Call: ros2 service call /describe_scene std_srvs/srv/Trigger')

    def on_image(self, msg):
        # Keep only the most recent frame as a PIL image
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            arr = arr[:, :, ::-1]  # BGR -> RGB for PIL
        self.latest = PILImage.fromarray(np.ascontiguousarray(arr), 'RGB')

    def on_request(self, request, response):
        if self.latest is None:
            response.success = False
            response.message = 'No camera frame received yet.'
            return response

        self.get_logger().info('Sending frame to VLM, please wait...')
        buf = io.BytesIO()
        self.latest.save(buf, format='JPEG')
        buf.seek(0)

        try:
            r = requests.post(
                VLM_URL,
                files={'image': ('frame.jpg', buf, 'image/jpeg')},
                data={'prompt': PROMPT},
                timeout=180,
            )
            desc = r.json().get('description', '(no description returned)')
        except Exception as e:
            response.success = False
            response.message = f'VLM request failed: {e}'
            return response

        self.pub.publish(String(data=desc))
        self.get_logger().info(f'VLM: {desc}')
        response.success = True
        response.message = desc
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VlmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
