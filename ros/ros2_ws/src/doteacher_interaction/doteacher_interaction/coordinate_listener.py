import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

import paho.mqtt.client as mqtt


class CoordinateListener(Node):
    def __init__(self):
        super().__init__('coordinate_listener')
        self.subscription = self.create_subscription(
            Pose,
            '/robot/pose',
            self.position_callback,
            10)
        self.mqtt_client = mqtt.Client()

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_publish = self.on_publish
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect('broker.hivemq.com', 1883, 60)
        self.mqtt_client.subscribe("ros2/explanation_finished")

        self.mqtt_client.loop_start()

        self.target_positions = [
            [1.0, 2.0, 0.0],  # 첫 번째 좌표
            [3.0, 4.0, 0.0],  # 두 번째 좌표
            [5.0, 6.0, 0.0]   # 세 번째 좌표
        ]

        self.published_indices = set()  # 이미 발행된 인덱스를 저장하는 집합

    def position_callback(self, msg):
        current_position = (msg.position.x, msg.position.y, msg.position.z)
        self.get_logger().info(f'Received position: {current_position}')

        index = self.find_index(current_position)
        self.get_logger().info(f'Index found: {index}')
        
        if index != -1 and index not in self.published_indices:
            constructed_message = f'start:{index}'
            self.mqtt_client.publish("raspberry_pi/start_explanation", constructed_message)
            self.get_logger().info(f'Published start signal for position: {current_position}, index: {index}')
            self.published_indices.add(index)

    def find_index(self, location):
        tolerance = 1e-5  # 허용 오차
        for index, pos in enumerate(self.target_positions):
            if all(abs(p - l) < tolerance for p, l in zip(pos, location)):
                return index
        return -1

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info('MQTT Client Connected with result code: %d' % rc)

    def on_publish(self, client, userdata, mid):
        self.get_logger().info('MQTT message published: %d' % mid)

    def on_message(self, client, userdata, msg):
        if msg.topic == "ros2/explanation_finished":
            self.get_logger().info('Explanation finished message received.')


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

