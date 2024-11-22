from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.client import Client
import time

class PTP_MOVE(Node):

    def __init__(self):
        super().__init__('dobot_PTP_client')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self._suction_client = self.create_client(SuctionCupControl, '/dobot_suction_cup_service')

        # Wait for the suction service to be available
        while not self._suction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for suction cup service...')

    def send_suction_command(self, enable):
        # Create the request object for the suction service
        request = SuctionCupControl.Request()
        request.enable_suction = enable

        # Call the service asynchronously
        future = self._suction_client.call_async(request)
        future.add_done_callback(self.suction_response_callback)

    def suction_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Suction cup command executed successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to call suction service: {e}')

    def cancel_done(self, future): 
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.current_pose))

    def send_goal(self, target, mode):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = target
        goal_msg.motion_type = mode
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    action_client = PTP_MOVE()
    while True:
        a = int(input())
        if a == 1:
            action_client.send_suction_command(True)  # 석션컵 켜기
            # 
            action_client.send_goal(target=[203.7, 116.3, -58.2, 0.0], mode=1)
            time.sleep(2)
        elif a == 2:
            #action_client.send_suction_command(False)  # 석션컵 끄기
            action_client.send_goal(target=[200.0, 0.0, 100.0, 0.0], mode=1)
            time.sleep(2)
        elif a == 3:
            action_client.send_goal(target=[191.5, -4.0, 112.0, 0.0], mode=1)
            time.sleep(2)
        elif a == 4:
            action_client.send_suction_command(False)
            #action_client.send_goal(target=[200.0, 0.0, 100.0, 0.0], mode=1)
            time.sleep(2)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
