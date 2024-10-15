import rclpy
from rclpy.node import Node
import asyncio
import threading
import websockets
from doteacher_interfaces.srv import RPiCommand

class ConnectServerNode(Node):
    def __init__(self):
        super().__init__('connect_server_node')
        
        self.get_logger().info('Websocket Client Node started')
        
        # ROS2 서비스 클라이언트 생성
        self.cli = self.create_client(RPiCommand, 'rpi_cmd_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('RPi Command Service is available')
        
        self.server_thread = threading.Thread(target=self.run_asyncio)
        self.server_thread.start()

    def run_asyncio(self):
        asyncio.run(self.connect())
    
    async def listen(self, websocket):
        async for message in websocket:
            print(f"Received message from server: {message}")
            self.handle_message('beep_sound')

            if message.split(' ')[0] == "ack":
                return

            if message.split(',')[0].isdigit():
                asyncio.create_task(self.send_messages(websocket, "ack start"))
            elif message == "next":
                asyncio.create_task(self.send_messages(websocket, "ack next"))
            elif message == "photo":
                asyncio.create_task(self.send_messages(websocket, "ack photo"))
            else:
                pass

    async def send_messages(self, websocket, message):
        await websocket.send(message)
        print(f"Sent message to server: {message}")
    
    
    async def connect(self):
        #uri = "ws://localhost:8080/ws"
        uri = "ws://i11d102.p.ssafy.io:8081/ws"
        async with websockets.connect(uri) as websocket:
            listen_task = asyncio.create_task(self.listen(websocket))
            send_task = asyncio.create_task(self.send_messages(websocket, "1"))
            await asyncio.gather(listen_task, send_task)
    
    # service request
    def handle_message(self, message):
        # 메시지에 따라 서비스 호출
        if message == 'beep_sound':
            self.call_service('beep_sound')
        elif message == 'capture_sound':
            self.call_service('capture_sound')
        elif message.startswith('description'):
            self.call_service(message)
        else:
            self.get_logger().info('Unknown command received.')
            
    def call_service(self, command):
        # ROS2 서비스 요청 생성 및 전송
        req = RPiCommand.Request()
        req.command = command
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: success={response.success}, message='{response.message}'")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ConnectServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
