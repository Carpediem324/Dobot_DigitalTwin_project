import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import socket  # 소켓 라이브러리 추가

import pyrealsense2 as rs

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import torch

class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolov5_node')

        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ubuntu/ssafy_ws/src/live/model/num.pt')
        
        os.system("pip install numpy==1.23.0 setuptools==58.2.0")
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        self.detection_publisher = self.create_publisher(String, 'detection_results', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 소켓 초기화
        self.server_ip = '127.0.0.1'
        self.server_port = 65432
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.server_ip, self.server_port))

        self.detection_buffer = []
        self.buffer_size = 20
        self.detection_threshold = 12

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            self.get_logger().error("Failed to retrieve frames")
            return
        
        color_image = np.asanyarray(color_frame.get_data())

        results = self.yolo_model(color_image)

        self.detection_result = String()

        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = map(int, result[:6])

            label = self.yolo_model.names[class_id]
            self.detection_buffer.append(label)
            if len(self.detection_buffer) > self.buffer_size:
                self.detection_buffer.pop(0)

            if self.detection_buffer.count(label) >= self.detection_threshold:
                # 검출 결과를 소켓을 통해 전송
                message = f'{label} detected {self.detection_threshold} times\n'
                try:
                    self.client_socket.sendall(message.encode())  # 메시지를 소켓으로 전송
                except socket.error as e:
                    self.get_logger().error(f"Failed to send message: {e}")

                # ROS 토픽으로도 결과를 퍼블리시
                self.detection_result.data = message
                self.detection_publisher.publish(self.detection_result)

                # 리스트 초기화
                self.detection_buffer = []
                self.get_logger().info("초기화함")

            # 이미지에 객체 정보 표시
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(color_image, f'No.{label}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_publisher.publish(ros_image_message)

    def destroy_node(self):
        self.pipeline.stop()
        self.client_socket.close()  # 소켓 닫기
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()