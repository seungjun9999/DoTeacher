import rclpy
from rclpy.node import Node
from doteacher_interfaces.srv import RPiCommand

import paho.mqtt.client as mqtt

class RPiCommandPublisherService(Node):
    def __init__(self):
        super().__init__('rpi_cmd_publisher')
        self.srv = self.create_service(RPiCommand, 'rpi_cmd_service', self.command_callback)
        
        self.FROM_JETSON_TOPIC = "jetson/from"
        self.TO_JETSON_TOPIC = "jetson/to"

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_publish = self.on_publish
        
        self.mqtt_client.connect('broker.hivemq.com', 1883, 60)

        self.mqtt_client.subscribe("jetson/to")
        self.mqtt_client.loop_start()        

    def command_callback(self, request, response):
        command = request.command
        self.get_logger().info(f'Service request received: {command}')

        if command == 'beep':
            self.cmd_beep()
            response.success = True
            response.message = 'Beep sound command executed.'
        elif command == 'ment_start':
            self.cmd_ment_start()
            response.success = True
            response.message = 'Ment Start sound command executed.'
        elif command == 'ment_end':
            self.cmd_ment_end()
            response.success = True
            response.message = 'Ment End sound command executed.'
        elif command.startswith('next'):
            index = int(command.split(':')[1])
            self.cmd_next(index)
            response.success = True
            response.message = f'Next command executed with index {index}.'
        elif command == 'p_sound':
            self.cmd_p_sound()
            response.success = True
            response.message = 'Capture sound command executed.'
        elif command.startswith('desc'):
            index = int(command.split(':')[1])
            self.cmd_desc(index)
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
        if msg.topic == "jetson/to":
            self.get_logger().info(f'{msg}')

    # 명령 실행 메서드
    def cmd_beep(self):
        self.get_logger().info(f'RPi Command sent: beep sound')
        self.mqtt_client.publish("jetson/from", "beep")
    
    def cmd_ment_start(self):
        self.get_logger().info(f'RPi Command sent: intro sound')
        self.mqtt_client.publish("jetson/from", "ment_start")

    def cmd_ment_end(self):
        self.get_logger().info(f'RPi Command sent: intro sound')
        self.mqtt_client.publish("jetson/from", "ment_end")

    def cmd_next(self, index):
        self.get_logger().info(f'RPi Command sent: intro sound')
        self.mqtt_client.publish("jetson/from", f"next:{index}")

    def cmd_p_sound(self):
        self.get_logger().info(f'RPi Command sent: picture sound')
        self.mqtt_client.publish("jetson/from", "p_sound")  # 사진찍을 때 소리 위한 MQTT 메시지 발행

    def cmd_desc(self, index):
        self.get_logger().info(f'RPi Command sent: description {index}')
        self.mqtt_client.publish("jetson/from", f"desc:{index}")


def main(args=None):
    rclpy.init(args=args)
    node = RPiCommandPublisherService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
