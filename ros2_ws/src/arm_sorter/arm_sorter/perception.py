"""Detection is published only while the coordinator enables observation."""
import json
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from .geometry import measure_polygon, classify_brightness

class Perception(Node):
    def __init__(self):
        super().__init__('box_perception')
        path = self.declare_parameter('config', '').value
        self.cfg = json.loads(Path(path).read_text())
        if not self.cfg.get('calibrated'):
            raise ValueError('Complete camera calibration before publishing metric targets')
        self.model = None
        if self.cfg['detector'] == 'yolo':
            from ultralytics import YOLO
            self.model = YOLO(self.cfg['weights'])
        elif self.cfg['detector'] != 'controlled_background':
            raise ValueError('Unknown detector')
        self.enabled = False
        self.enabled_since = 0
        self.bridge = CvBridge()
        self.pub = self.create_publisher(String, '/sorting/detections', 10)
        self.create_subscription(Bool, '/sorting/observe', self.enable, 10)
        self.create_subscription(Image, '/camera/image_raw', self.frame, qos_profile_sensor_data)
    def enable(self, msg):
        if msg.data and not self.enabled:
            self.enabled_since = self.get_clock().now().nanoseconds
        self.enabled = msg.data
    def frame(self, msg):
        if not self.enabled: return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= self.enabled_since or self.get_clock().now().nanoseconds - stamp > 2e9: return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if list(image.shape[1::-1]) != self.cfg['image_size']:
                raise ValueError('Camera resolution differs from calibration')
            image = cv2.undistort(image, np.array(self.cfg['camera_matrix']), np.array(self.cfg['dist_coeffs']))
            masks = []
            if self.model:
                result = self.model.predict(image, conf=self.cfg['confidence'], device='cpu', verbose=False)[0]
                if result.masks is not None:
                    for polygon, cls in zip(result.masks.xy, result.boxes.cls.tolist()):
                        if result.names[int(cls)] not in self.cfg['allowed_classes']: continue
                        mask = np.zeros(image.shape[:2], np.uint8)
                        cv2.fillPoly(mask, [polygon.astype(np.int32)], 255)
                        masks.append(mask)
            else:
                # Prototype fallback: white/black boxes on a mid-brightness background.
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                for candidate in [(gray < self.cfg['black_max']), (gray > self.cfg['white_min'])]:
                    contours, _ = cv2.findContours(candidate.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for contour in contours:
                        mask = np.zeros(gray.shape, np.uint8)
                        cv2.drawContours(mask, [contour], -1, 255, -1)
                        masks.append(mask)
            found = []
            for mask in masks:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not contours: continue
                contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(contour) < self.cfg['min_area_px']: continue
                x, y, w, h = cv2.boundingRect(contour)
                if x <= 1 or y <= 1 or x+w >= image.shape[1]-1 or y+h >= image.shape[0]-1: continue
                colour = classify_brightness(image, mask, self.cfg['black_max'], self.cfg['white_min'])
                if colour == 'uncertain': continue
                box = measure_polygon(contour, self.cfg['pixel_to_base_homography'])
                xmin, xmax, ymin, ymax = self.cfg['workspace_bounds_m']
                if not (xmin <= box['x'] <= xmax and ymin <= box['y'] <= ymax): continue
                if not self.cfg['width_range_m'][0] <= box['width_m'] <= self.cfg['width_range_m'][1]: continue
                box['colour'] = colour
                found.append(box)
            out = {'stamp_ns': stamp, 'frame_id': 'base_link', 'boxes': found}
            self.pub.publish(String(data=json.dumps(out, allow_nan=False)))
        except Exception as exc:
            self.get_logger().error(f'Frame rejected: {exc}')

def main():
    rclpy.init(); node = Perception()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
