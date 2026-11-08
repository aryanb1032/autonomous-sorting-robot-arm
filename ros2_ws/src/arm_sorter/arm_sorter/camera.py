"""Request a fresh JPEG snapshot rather than consuming a buffered video stream."""
import urllib.request
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera(Node):
    def __init__(self):
        super().__init__('esp32_camera')
        self.url = self.declare_parameter('capture_url', '').value
        if not self.url.startswith(('http://', 'https://')):
            raise ValueError('Set capture_url to the camera JPEG /capture endpoint')
        self.frame = self.declare_parameter('frame_id', 'camera_optical_frame').value
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.create_timer(0.5, self.capture)
    def capture(self):
        try:
            with urllib.request.urlopen(self.url, timeout=2) as response:
                data = response.read(4 * 1024 * 1024)
            image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Response was not a JPEG image')
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            # Receive time; the ESP32 endpoint does not expose exposure timestamps.
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame
            self.pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f'Capture failed: {exc}')

def main():
    rclpy.init()
    node = Camera()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
