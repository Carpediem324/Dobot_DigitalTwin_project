import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket


class JointStateSocketPublisher(Node):
    def __init__(self, host, port):
        super().__init__('joint_state_socket_publisher')

        # 소켓 설정
        self.host = host
        self.port = port
        self.client_socket = None

        self.setup_socket()

        # /dobot_joint_states 토픽 구독
        self.subscription = self.create_subscription(
            JointState,
            '/dobot_joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info(f"Subscribed to /dobot_joint_states and sending data to {host}:{port}")

    def setup_socket(self):
        """소켓 설정 및 연결"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port))
            self.get_logger().info(f"Connected to socket server at {self.host}:{self.port}")
        except socket.error as e:
            self.get_logger().error(f"Failed to connect to socket server: {e}")
            self.client_socket = None

    def joint_state_callback(self, msg: JointState):
        """JointState 메시지를 수신하고 소켓으로 전송"""
        if self.client_socket is None:
            self.get_logger().error("Socket is not connected. Skipping message.")
            return

        try:
            # 조인트 값 추출
            joint_positions = [pos * 35 for pos in msg.position[:3]]  # 앞 3개 축만 사용
            data = ','.join(map(str, joint_positions))
            self.get_logger().info(f"Sending joint data: {data}")
            
            # 소켓으로 데이터 전송
            self.client_socket.sendall(data.encode('utf-8'))
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}. Attempting to reconnect...")
            self.client_socket.close()
            self.client_socket = None
            self.setup_socket()
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def destroy_node(self):
        """소켓 정리 및 노드 종료"""
        if self.client_socket:
            self.client_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # 서버 IP와 포트 설정
    host = '192.168.26.36'
    port = 10000

    node = JointStateSocketPublisher(host, port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
