#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloCam(Node):
    def __init__(self):
        super().__init__('yolo_cam')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.sub = self.create_subscription(
            Image,
            '/j100_0000/sensors/camera_0/color/image',
            self.cb,
            10
        )

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)
        out = results[0].plot()
        cv2.imshow("YOLO Camera", out)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
