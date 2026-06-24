#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pupil_apriltags import Detector

class AprilTagCam(Node):
    def __init__(self):
        super().__init__('apriltag_cam')

        self.bridge = CvBridge()

        # 실시간 카메라용: decimate 낮추고, 나중에 프레임 자체도 키워서 검출
        self.detector = Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.5,
            debug=0
        )

        self.sub = self.create_subscription(
            Image,
            '/j100_0000/sensors/camera_0/color/image',
            self.cb,
            10
        )

        self.scale = 4.0
        self.last_count = -1
        self.get_logger().info('AprilTag detector started.')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 태그가 작아서 안 잡히는 경우가 많으니 크게 키워서 검출
        gray_big = cv2.resize(
            gray,
            None,
            fx=self.scale,
            fy=self.scale,
            interpolation=cv2.INTER_CUBIC
        )

        detections = self.detector.detect(gray_big, estimate_tag_pose=False)

        # 원본 프레임에 다시 그리기 위해 좌표를 scale로 나눔
        for det in detections:
            tag_id = int(det.tag_id)
            corners = (det.corners / self.scale).astype(int)
            center = tuple((det.center / self.scale).astype(int))

            for i in range(4):
                p1 = tuple(corners[i])
                p2 = tuple(corners[(i + 1) % 4])
                cv2.line(frame, p1, p2, (0, 255, 0), 2)

            cv2.circle(frame, center, 4, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f'ID {tag_id}',
                (corners[0][0], corners[0][1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        count = len(detections)
        if count != self.last_count:
            self.get_logger().info(f'detections: {count}')
            self.last_count = count

        cv2.putText(
            frame,
            f'detections: {count}',
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 0, 0),
            2
        )

        cv2.imshow('AprilTag Camera', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = AprilTagCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
