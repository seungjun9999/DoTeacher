import rclpy
from rclpy.node import Node
import asyncio
import threading
import websockets
from doteacher_interfaces.srv import RPiCommand, Nav2Index, DetectPoseService
import requests

class ConnectServerNode(Node):
    def __init__(self):
        super().__init__('connect_server_node')

        self.get_logger().info('Websocket Client Node started')

        # ROS2 서비스 클라이언트 생성
        self.cli = self.create_client(RPiCommand, 'rpi_cmd_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('RPi Command Service is available')

        # CommandManager 서비스 클라이언트 생성
        self.command_manager_client = self.create_client(Nav2Index, 'nav2_index')
        while not self.command_manager_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('CommandManager Service not available, waiting again...')
        self.get_logger().info('CommandManager Service is available')

        self.detect_pose_client = self.create_client(DetectPoseService, 'detect_pose_service')
        while not self.detect_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DetectPose Service not available, waiting again...')

        self.server_thread = threading.Thread(target=self.run_asyncio)
        self.server_thread.start()

        self.user_id = -1

        # Demo
        self.seq_index_list = [17, 19, 12]
        self.count = 0

    def run_asyncio(self):
        asyncio.run(self.connect())

    async def listen(self, websocket):
        async for message in websocket:
            self.get_logger().info(f"Received msg from server: {message}")
            if message.split(' ')[0] == "Acknowledgement":
                continue
            else:
                if message.split(',')[0].isdigit():
                    self.user_id = int(message.split(',')[0])
                    self.handle_message('ment_start')
                    asyncio.create_task(self.send_messages(websocket, "ack start"))
                elif message == "next":
                    self.handle_message('next')
                    asyncio.create_task(self.send_messages(websocket, "ack next"))
                elif message == "photo":
                    self.handle_message('beep')
                    asyncio.create_task(self.send_messages(websocket, "ack photo"))
                    self.send_end(self.user_id)

    async def send_messages(self, websocket, message):
        await websocket.send(message)
        self.get_logger().info(f"Sent message to server: {message}")

    async def connect(self):
        uri = "ws://i11d102.p.ssafy.io:8081/ws"
        async with websockets.connect(uri) as websocket:
            listen_task = asyncio.create_task(self.listen(websocket))
            send_task = asyncio.create_task(self.send_messages(websocket, "1"))
            await asyncio.gather(listen_task, send_task)

    # service request
    def handle_message(self, message):
        if message == 'beep':
            self.call_service('beep')
        elif message == 'ment_start':
            self.call_service('ment_start')
        elif message == 'ment_end':
            self.call_service('ment_end')
        elif message == 'next':
            if self.count >= len(self.seq_index_list):
                self.count = 0
                self.send_end(self.user_id)
                self.call_service('ment_end')
            else:
                self.call_service(f'next:{self.seq_index_list[self.count]}')
                self.call_command_manager_service(self.seq_index_list[self.count])
        elif message == 'p_sound':
            self.call_service('p_sound')
            self.call_detect_pose_service()
        elif message.startswith('desc'):
            self.call_service(message)
        else:
            self.get_logger().info('Unknown command received.')

    def call_detect_pose_service(self):
        req = DetectPoseService.Request()
        req.user_id = self.user_id
        future = self.detect_pose_client.call_async(req)
        future.add_done_callback(self.detect_pose_callback)

    def detect_pose_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"DetectPose succeeded: {response.message}")
            else:
                self.get_logger().info(f"DetectPose failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f'DetectPose Service call failed: {e}')

    def call_service(self, command):
        req = RPiCommand.Request()
        req.command = command
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_callback)

    def call_command_manager_service(self, index):
        req = Nav2Index.Request()
        req.index = index
        future = self.command_manager_client.call_async(req)
        future.add_done_callback(self.command_manager_service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: success={response.success}, message='{response.message}'")
            if response.message.startswith("Next"):
                self.count += 1
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def command_manager_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"CommandManager response: success={response.success}, message='{response.message}'")
            if response.success:
                self.call_service(f'desc:{self.seq_index_list[self.count]}')
        except Exception as e:
            self.get_logger().error(f'CommandManager call failed: {e}')

    def send_end(self, user_id):
        url = f'http://i11d102.p.ssafy.io:8081/user/{user_id}/prodDes?userDes=0'

        try:
            response = requests.put(url)
            if response.status_code == 200:
                self.get_logger().info('Successfully sent')
                return True
            else:
                self.get_logger().error(f'Failed to send: {response.status_code}')
                return False
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Send request failed: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = ConnectServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
