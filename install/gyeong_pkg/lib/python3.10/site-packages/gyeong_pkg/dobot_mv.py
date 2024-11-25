import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl
import time

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('dobot_pick_and_place')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self._service_client = self.create_client(SuctionCupControl, '/dobot_suction_cup_service')

    def send_goal(self, type, target, mode):
        if type == 'move':
            self.get_logger().info('Waiting for action server...')
            self._action_client.wait_for_server()

            goal_msg = PointToPoint.Goal()
            goal_msg.target_pose = target
            goal_msg.motion_type = mode
            self.get_logger().info('Sending move goal request...')

            # Wait for the result
            future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().info('Move goal rejected')
                return False

            self.get_logger().info('Move goal accepted')

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            self.get_logger().info(f'Move result: {result.result}')
            return True

        elif type == 'gripper':
            while not self._service_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')

            self._request = SuctionCupControl.Request()
            self._request.enable_suction = mode
            future = self._service_client.call_async(self._request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(f"Gripper action completed: {response}")
            return True

        return False


def main(args=None):
    try:
        rclpy.init()

        action_client = PickAndPlaceNode()

        dobot_position = {
            '1': [159.6, 49.8, -57.6, 0.0],
            '2': [226.6, 52.1, -59.7, 0.0],
            '3': [164.6, 87.9, -57.7, 0.0],
            '4': [230.5, 94.2, -59.9, 0.0],
            '5': [169.1, 129.4, -58.6, 0.0],
            '6': [220.2, 137.4, -58.6, 0.0],
            'way': [195.4, 40.2, 68.6, 0.0],
            'place': [191.8, -32.0, -7.6, 0.0],
        }

        # Make task_list
        tasks_list = []

        for i in range(1, 7):  # temp job 한 개만 하기
            key = str(i)
            val = dobot_position[key]
            temp_val = dobot_position[key].copy()
            temp_val[-2] += 80

            tasks_list.append(["move", dobot_position['way'], 1])
            tasks_list.append(["move", temp_val, 1])
            tasks_list.append(["move", val, 1])
            tasks_list.append(["gripper", "open", True])
            tasks_list.append(["move", temp_val, 1])
            tasks_list.append(["move", dobot_position['way'], 1])
            tasks_list.append(["move", dobot_position['place'], 1])
            tasks_list.append(["gripper", "close", False])

        # Set home position
        action_client.send_goal("move", [150.0, 0.0, 100.0, 0.0], 1)  # Home

        for (type, target, mode) in tasks_list:
            success = action_client.send_goal(type, target, mode)
            if not success:
                action_client.get_logger().error(f"Failed to execute task: {type} with target {target}")
                break  # Stop if any task fails

        action_client.send_goal("move", [150.0, 0.0, 100.0, 0.0], 1)  # Return to Home

    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()


if __name__ == '__main__':
    main()
