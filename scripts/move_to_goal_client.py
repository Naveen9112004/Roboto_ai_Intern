#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from gazebo.action import MoveToGoal


class MoveToGoalClient(Node):
    def __init__(self):
        super().__init__('move_to_goal_client')
        self._client = ActionClient(self, MoveToGoal, 'move_to_goal')

    def send_goal(self):
        goal_msg = MoveToGoal.Goal()
        goal_msg.dummy = 0.0   # matches MoveToGoal.action

        self.get_logger().info('Sending goal: start obstacle-avoid motion.')
        self._client.wait_for_server()

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted, robot will keep moving.')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f'Result: success={result.success}, message="{result.message}"')

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'distance_ahead = {fb.distance_ahead:.2f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalClient()
    node.send_goal()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
