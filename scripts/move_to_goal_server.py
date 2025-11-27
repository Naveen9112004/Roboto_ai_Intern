#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from gazebo.action import MoveToGoal


class ObstacleAvoidServer(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_server')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

        self._action_server = ActionServer(
            self,
            MoveToGoal,
            'move_to_goal',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info('MoveToGoal obstacle-avoid server started.')

    def scan_callback(self, msg: LaserScan):
        self.ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested.')
        self.stop_robot()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Goal received: start obstacle-avoid motion.')

        # Parameters
        forward_speed = 0.20
        turn_speed = 0.80
        safe_dist = 0.80          # start turning if object < 0.8 m
        emergency_dist = 0.30     # back off if < 0.3 m
        front_angle = math.radians(60.0)  # +-60 degrees front cone

        rate = self.create_rate(20.0)
        result = MoveToGoal.Result()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled.')
                self.stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled'
                return result

            twist = Twist()

            # Wait until we have a scan
            if not self.ranges:
                self.cmd_pub.publish(twist)
                await rate.sleep()
                continue

            min_front, min_left, min_right = self.compute_regions(front_angle)

            # Behaviour:
            # 1) emergency back + turn
            if min_front < emergency_dist:
                twist.linear.x = -0.15
                twist.angular.z = turn_speed if min_left > min_right else -turn_speed

            # 2) front clear -> go straight
            elif min_front > safe_dist:
                twist.linear.x = forward_speed
                twist.angular.z = 0.0

            # 3) obstacle ahead -> turn to freer side
            else:
                twist.linear.x = 0.0
                twist.angular.z = turn_speed if min_left > min_right else -turn_speed

            self.cmd_pub.publish(twist)

            fb = MoveToGoal.Feedback()
            fb.distance_ahead = float(min_front)
            fb.current_x = 0.0
            fb.current_y = 0.0
            goal_handle.publish_feedback(fb)

            await rate.sleep()

        self.stop_robot()
        goal_handle.abort()
        result.success = False
        result.message = 'Node shutting down'
        return result

    def compute_regions(self, front_angle):
        n = len(self.ranges)
        if n == 0:
            return float('inf'), float('inf'), float('inf')

        angles = [
            self.angle_min + i * self.angle_increment
            for i in range(n)
        ]

        front, left, right = [], [], []

        for a, r in zip(angles, self.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            while a > math.pi:
                a -= 2.0 * math.pi
            while a < -math.pi:
                a += 2.0 * math.pi

            if -front_angle <= a <= front_angle:
                front.append(r)
            elif a > front_angle:
                left.append(r)
            elif a < -front_angle:
                right.append(r)

        def mmin(lst):
            return min(lst) if lst else float('inf')

        return mmin(front), mmin(left), mmin(right)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidServer()
    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
