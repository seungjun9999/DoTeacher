import rclpy
from rclpy.node import Node
from doteacher_interfaces.srv import RPiCommand

import paho.mqtt.client as mqtt

class RPiCommandPublisherService(Node):
    def __init__(self):
        super().__init__('rpi_cmd_publisher')
        self.srv = self.create_service(RPiCommand, 'rpi_cmd_service', self.command_callback)
        
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_publish = self.on_publish
        
        self.mqtt_client.connect('broker.hivemq.com', 1883, 60)
        self.mqtt_client.subscribe("ros2/explanation_finished")
        self.mqtt_client.loop_start()

    def command_callback(self, request, response):
        command = request.command
        self.get_logger().info(f'Service request received: {command}')

        if command == 'beep_sound':
            self.cmd_beep_sound()
            response.success = True
            response.message = 'Beep sound command executed.'
        elif command == 'capture_sound':
            self.cmd_capture_sound()
            response.success = True
            response.message = 'Capture sound command executed.'
        elif command.startswith('description'):
            index = int(command.split(':')[1])
            self.cmd_description(index)
            response.success = True
            response.message = f'Description command executed with index {index}.'
        else:
            response.success = False
            response.message = 'Unknown command.'
        
        return response

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info('MQTT Client Connected with result code: %d' % rc)

    def on_publish(self, client, userdata, mid):
        self.get_logger().info('MQTT message published: %d' % mid)

    def on_message(self, client, userdata, msg):
        if msg.topic == "ros2/explanation_finished":
            self.get_logger().info('Explanation finished message received.')

    # 명령 실행 메서드
    def cmd_beep_sound(self):
        self.get_logger().info(f'RPi Command sent: beep sound')
        self.mqtt_client.publish("ros2/nav", "start:99")  # 사진찍을 때 소리 위한 MQTT 메시지 발행
    
    def cmd_capture_sound(self):
        self.get_logger().info(f'RPi Command sent: capture sound')
        self.mqtt_client.publish("ros2/nav", "start:100")  # 사진찍을 때 소리 위한 MQTT 메시지 발행

    def cmd_description(self, index):
        self.get_logger().info(f'RPi Command sent: description {index}')
        self.mqtt_client.publish("raspberry_pi/start_explanation", f"start:{index}")

def main(args=None):
    rclpy.init(args=args)
    node = RPiCommandPublisherService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
