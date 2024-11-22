import socket
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl

# 소켓 클라이언트 설정
HOST = '192.168.26.36'  # 서버 IP 주소
PORT = 10001            # 서버 포트 번호

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('dobot_pick_and_place')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self._service_client = self.create_client(SuctionCupControl, '/dobot_suction_cup_service')

    def send_goal(self, joint_values):
        """조인트 위치를 두봇으로 전달"""
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = joint_values
        goal_msg.motion_type = 4  # PTP(Motion Type) joint로 이동은 2번임
        self.get_logger().info(f'Sending goal request: {joint_values}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Received feedback: {feedback.feedback.current_pose}")

def main(args=None):
    rclpy.init(args=args)
    action_client = PickAndPlaceNode()

    # 소켓 클라이언트 초기화
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))
    print(f"Connected to server at {HOST}:{PORT}")

    try:
        while True:
            # 데이터 수신
            data = client_socket.recv(1024)
            if not data:
                break

            # 데이터를 파싱하여 조인트 값으로 변환
            raw_data = data.decode('utf-8').strip()
            print(f"Raw received data: {raw_data}")
            try:
                joint_values = [float(x) for x in raw_data.split(',')]
                print(f"Parsed joint values: {joint_values}")

                # 두봇 조종
                action_client.send_goal(joint_values)
            except ValueError as e:
                print(f"Error parsing joint values: {e}. Raw data: {raw_data}")
    except KeyboardInterrupt:
        print("Client interrupted by user.")
    finally:
        client_socket.close()
        action_client.destroy_node()
        rclpy.shutdown()
        print("Client closed.")


if __name__ == "__main__":
    main()
