import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from collections import deque
from doteacher_interfaces.srv import Nav2Index, DetectedImage

class CommandManager(Node):
    def __init__(self):
        super().__init__('command_manager')
        self.command_queue = deque()
        self.previous_commands = deque(maxlen=10)
        self.current_state = 'idle'
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # DetectPictureService 클라이언트 생성 및 서비스가 준비될 때까지 대기
        self.detect_client = self.create_client(DetectedImage, 'detect_image')
        while not self.detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DetectPictureService not available, waiting...')
        
        self.create_service(Nav2Index, 'nav2_index', self.add_command_service_callback)
        self.check_queue_timer = self.create_timer(1.0, self.check_queue_periodically)

    def check_queue_periodically(self):
        if self.current_state == 'idle' and not self.command_queue:
            self.get_logger().info("Wait for command")
        else:
            self.process_next_command()


    def add_command_service_callback(self, request, response):
        self.add_command(f'navigate_to:{request.index}')
        response.success = True
        response.message = f'Successfully complete.'
        return response

    def add_command(self, command, priority=False):
        if priority:
            self.command_queue.appendleft(command)
        else:
            self.command_queue.append(command)
        self.process_next_command()

    def process_next_command(self):
        if self.current_state == 'idle' and self.command_queue:
            next_command = self.command_queue.popleft()
            self.previous_commands.append(next_command)
            self.execute_command(next_command)

    def execute_command(self, command):
        if command.startswith('navigate_to'):
            index = int(command.split(':')[1])
            self.navigate_to_goal(index)
        self.current_state = 'navigating'

    def navigate_to_goal(self, index):
        goal_msg = NavigateToPose.Goal()
        x, y, yaw = self.index_to_coordinates(index)
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = yaw  # yaw를 quaternion으로 변환하는 로직이 필요
        goal_msg.pose.pose.orientation.w = 1.0  # 실제 상황에 맞게 조정 필요

        self.nav_client.wait_for_server()
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.current_state = 'idle'
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal succeeded!')
            # check service camera
            self.call_detect_service()
            self.current_state = 'idle'
            self.process_next_command()  # 다음 명령 처리

    def call_detect_service(self):
        req = DetectedImage.Request()
        future = self.detect_client.call_async(req)
        future.add_done_callback(self.detect_service_callback)


    def detect_service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Detection succeeded: {response.message}')
            else:
                self.get_logger().info(f'Detection failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


    def index_to_coordinates(self, index):
        coordinates_map = {
            0: (0.0, 0.0, 0.0),  # 난맹첩
            1: (0.0, 0.0, 0.0),  # 아홉번째 파도
            2: (0.0, 0.0, 0.0),  # 부유세계 마타베이의 걸작
            3: (0.0, 0.0, 0.0),  # 겨울 풍경
            4: (0.0, 0.0, 0.0),  # 게르니카
            5: (0.0, 0.0, 0.0),  # 무제
            6: (0.0, 0.0, 0.0),  # 송하음다도
            7: (0.0, 0.0, 0.0),  # 시계의 연속성
            8: (0.0, 0.0, 0.0),  # 밤의 카페 테라스
            9: (0.0, 0.0, 0.0),  # 별이 빛나는 밤
            10: (0.0, 0.0, 0.0), # 서당
            11: (0.0, 0.0, 0.0), # 야묘도추
            12: (0.7, 0.3, 0.2), # 절규
            13: (0.0, 0.0, 0.0), # 단오 풍정
            14: (0.0, 0.0, 0.0), # 아테네 학당
            15: (0.0, 0.0, 0.0), # 영묘도 대련
            16: (0.0, 0.0, 0.0), # 인왕제색도
            17: (0.5, 0.0, -0.2), # 가시 목걸이 자화상
            18: (0.0, 0.0, 0.0), # 미산이곡
            19: (0.8, 0.1, 0.1), # 진주 귀걸이를 한 소녀
        }
        return coordinates_map.get(index, (0.0, 0.0, 0.0))

def main(args=None):
    rclpy.init(args=args)
    command_manager = CommandManager()
    rclpy.spin(command_manager)
    command_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
