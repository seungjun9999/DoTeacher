import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import paho.mqtt.client as mqtt
import json

class MqttRosBridge(Node):
    def __init__(self):
        super().__init__('mqtt_ros_bridge')
        self.publisher = self.create_publisher(Float32, 'ultrasonic_distance', 30)
        
        # MQTT 클라이언트 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("192.168.137.125", 1883, 60)  # Raspberry Pi의 IP 주소로 변경
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker!")
        client.subscribe("sensor/ultrasonic")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            distance = data.get("distance", None)
            if distance is not None:
                self.get_logger().info(f"Received distance: {distance:.2f} cm")
                ros_msg = Float32()
                ros_msg.data = distance
                self.publisher.publish(ros_msg)
        except ValueError:
            self.get_logger().error("Failed to decode MQTT message")

def main(args=None):
    rclpy.init(args=args)
    node = MqttRosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
