#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

class CamNode(Node):
    def __init__(self):
        super().__init__("usbcam_node")

        self.device_index = 0
        self.cap = cv2.VideoCapture(self.device_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open webcam. Device index: {self.device_index}")
            exit()

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.create_timer(0.03, self.timer_callback) # 30 Hz

        self.get_logger().info("USB camera node has been started.")

    def timer_callback(self):
        self.get_image()

    def get_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Cannot read frame from webcam.")
            return
        
        # Convert the frame to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(image_msg)
        self.get_logger().info("Published image frame.")

    def release(self):
        # Release resources
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Released camera resources.")

def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.release()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
