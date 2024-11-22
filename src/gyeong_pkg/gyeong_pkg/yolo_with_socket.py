import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import cv2
import numpy as np
import os
import pyrealsense2 as rs

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dobot_msgs.action import PointToPoint

import torch
import socket
import time
#pie ip 
IP = '192.168.110.154'
PORT = 9999
MAXCOUNT = 20
OVERCOST = 16
class RealSenseYoloNode(Node):
    def __init__(self):
        # Parameter MAX buffer is maxparam, when get colordata over overcost send socket
        self.maxparam = MAXCOUNT
        self.overcost = OVERCOST
        super().__init__('realsense_yolov5_node')

        # YOLOv5 model_load
        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ubuntu/magician_ros2_control_system_ws/src/gyeong_pkg/model/panel_sj.pt')
        os.system("pip install numpy==1.23.0 setuptools==58.2.0")

        # RealSense camera setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.detection_publisher = self.create_publisher(String, 'detection_results', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)
        
        self.bridge = CvBridge()

        # Socket setup with retry mechanism
        # my ip
        # self.server_ip = '192.168.110.140'
        # gyeong ip
        self.server_ip = IP
        self.server_port = PORT
        self.socket = None
        self.connect_to_server()

        # Initialize ROI
        self.roi_position = self.select_roi()

        # Initialize detection buffer
        self.recent_detections = []  # List to store recent detections (max 20)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def connect_to_server(self):
        """Try to connect to the server. Retry until successful."""
        while self.socket is None:
            try:
                self.get_logger().info(f"Trying to connect to {self.server_ip}:{self.server_port}...")
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.server_ip, self.server_port))
                self.get_logger().info("Connected to the server successfully.")
            except socket.error as e:
                self.get_logger().error(f"Connection failed: {e}. Retrying in 5 seconds...")
                self.socket = None
                time.sleep(5)

    def select_roi(self):
        """Allow the user to select ROI using the mouse."""
        self.get_logger().info("Opening RealSense camera to select ROI. Press 'Enter' to confirm.")
        while True:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            # Show the image and let the user select ROI
            roi = cv2.selectROI("Select ROI", color_image, fromCenter=False, showCrosshair=True)
            cv2.destroyAllWindows()

            # Return the selected ROI as (x1, y1, x2, y2)
            if roi:
                x, y, w, h = roi
                return int(x), int(y), int(x + w), int(y + h)

    def get_color_name(self, hsv_color):
        """Determine the color name based on HSV values."""
        h, s, v = hsv_color

        # White color: High brightness, low saturation
        if 0 <= s < 40 and 180 <= v <= 255:
            return 'white'

        # Red color: Can exist in two Hue ranges (0-10 and 160-180)
        if ((0 <= h <= 10 or 160 <= h <= 180) and s >= 100 and v >= 100):
            return 'red'

        # Blue color: Hue range 100-130, medium to high saturation and brightness
        if 100 <= h <= 130 and s >= 150 and v >= 100:
            return 'blue'

        # If no color is matched
        return 'unknown'

    def get_color_bgr(self, color_name):
        if color_name == 'white':
            return (255, 255, 255)
        elif color_name == 'red':
            return (0, 0, 255)
        elif color_name == 'blue':
            return (255, 0, 0)
        return (0, 255, 0)  # Default to green for unknown colors

    def get_center_color(self, image):
        height, width = image.shape[:2]
        center_y, center_x = height // 2, width // 2
        sample_size = min(width, height) // 4
        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)
        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size // 2)
        center_region = image[start_y:end_y, start_x:end_x]
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        average_color = np.mean(hsv_region, axis=(0, 1))
        
        return average_color

    def update_recent_detections(self, label, color_name):
        """Add a detection to the buffer and maintain the size limit."""
        if len(self.recent_detections) >= self.maxparam:
            self.recent_detections.pop(0)  # Remove the oldest detection
        self.recent_detections.append((label, color_name))

    def get_majority_color(self):
        """Determine the majority color from the recent detections without resetting the buffer."""
        color_counts = {}
        for _, color_name in self.recent_detections:
            if color_name not in color_counts:
                color_counts[color_name] = 0
            color_counts[color_name] += 1

        # Find the color with the maximum count
        majority_color = max(color_counts, key=color_counts.get, default='unknown')
        majority_count = color_counts.get(majority_color, 0)

        return majority_color, majority_count



    def timer_callback(self):
        # Get RealSense frames
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        roi_x1, roi_y1, roi_x2, roi_y2 = self.roi_position
        cv2.rectangle(color_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)  # ROI rectangle

        roi_image = color_image[roi_y1:roi_y2, roi_x1:roi_x2]
                    
        # YOLOv5 inference
        results = self.yolo_model(roi_image)

        self.detection_result = String()

        # Draw bounding boxes
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = map(int, result[:6])
            
            object_roi = color_image[roi_y1 + y1: roi_y1 + y2, roi_x1 + x1: roi_x1 + x2]
            center_color = self.get_center_color(object_roi)

            color_name = self.get_color_name(center_color)
            label = self.yolo_model.names[class_id]
            self.update_recent_detections(label, color_name)  # Add to buffer

        # Determine majority color without resetting the buffer
        majority_color, count = self.get_majority_color()

        print(f"Majority color: {majority_color}, Count: {count}")

        # Perform action if majority count is sufficient
        if count >= self.overcost:
            try:
                if majority_color == 'red':
                    self.socket.sendall(b'1')
                else:
                    self.socket.sendall(b'0')
            except socket.error as e:
                self.get_logger().error(f"Socket error: {e}. Attempting to reconnect...")
                self.socket.close()
                self.socket = None
                self.connect_to_server()
            
            # Clear the buffer after action
            self.recent_detections.clear()
            print("Buffer cleared after action.")

        # Publish image
        ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_publisher.publish(ros_image_message)


    def destroy_node(self):
        self.pipeline.stop()
        if self.socket:
            self.socket.close()  # Close socket on node destruction
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
