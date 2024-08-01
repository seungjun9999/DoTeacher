import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10) # 여유 큐 10
        self.timer = self.create_timer(1, self.timer_callback) # 1초에 1번 업데이트하는것. 더 빨라지면 jetson 과부하걸림
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
