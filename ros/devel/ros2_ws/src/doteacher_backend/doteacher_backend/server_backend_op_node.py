import rclpy
from rclpy.node import Node
import asyncio
import socket
import threading
import websockets


class ServerBackendOpNode(Node):
    def __init__(self):
        super().__init__('server_backend_op_node')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 5050))  # 모든 인터페이스에서 포트 5050을 바인딩
        self.server_socket.listen(5)  # 최대 5개의 대기 클라이언트
        self.get_logger().info('TCP Server Node started on port 5050')
        
        self.accept_thread = threading.Thread(target=self.accept_connections)
        self.accept_thread.start()

    def accept_connections(self):
        while rclpy.ok():
            client_socket, client_address = self.server_socket.accept()
            self.get_logger().info(f'Connection from {client_address}')
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
            client_thread.start()

    def handle_client(self, client_socket):
        while rclpy.ok():
            try:
                data = client_socket.recv(1024)  # 최대 1024바이트 수신
                if not data:
                    break
                self.get_logger().info(f'Received data: {data.decode("utf-8")}')
            except ConnectionResetError:
                break
        client_socket.close()

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
