import rclpy
from rclpy.node import Node
import asyncio
import threading
import websockets


class ServerBackendOpNode(Node):
    def __init__(self):
        super().__init__('client_backend_op_node')
        
        self.get_logger().info('Websocket Client Node started')
        self.server_thread = threading.Thread(target=self.run_asyncio)
        self.server_thread.start()

    def run_asyncio(self):
        asyncio.run(self.connect())
    
    async def listen(self, websocket):
        async for message in websocket:
            print(f"Received message from server: {message}")

    async def send_messages(self, websocket):
        # init
        message = "1"
        await websocket.send(message)
        print(f"Sent message to server: {message}")
    
    
    async def connect(self):
        #uri = "ws://localhost:8080/ws"
        uri = "ws://i11d102.p.ssafy.io:8081/ws"
        async with websockets.connect(uri) as websocket:
            listen_task = asyncio.create_task(self.listen(websocket))
            send_task = asyncio.create_task(self.send_messages(websocket))
            await asyncio.gather(listen_task, send_task)

def main(args=None):
    rclpy.init(args=args)
    node = ServerBackendOpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
