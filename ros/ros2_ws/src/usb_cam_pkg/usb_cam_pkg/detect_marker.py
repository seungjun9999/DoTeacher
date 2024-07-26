#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv
import cv2.aruco as aruco

# OpenCV 창 설정
window_name = "ArUco Marker Detection"
cv.namedWindow(window_name, cv.WINDOW_NORMAL)
# cv.resizeWindow(window_name, 1280, 720)  # 창 크기 조절

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

        # 아루코 사전 로드
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.get_logger().info("Image subscriber node has been started.")

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.recognize_marker(cv_image)

            
        except CvBridge.CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def recognize_marker(self, frame):
        # 아루코 마커 탐지
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(frame)

        # 마커가 감지되었을 때
        if markerIds is not None:
            frame = cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            for i in range(len(markerIds)):
                c = markerCorners[i][0]
                center_x = int((c[0][0] + c[2][0]) / 2)
                center_y = int((c[0][1] + c[2][1]) / 2)
                cv.putText(frame, f"ID: {markerIds[i][0]}", (center_x, center_y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Display image
        cv.imshow(window_name, frame)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
