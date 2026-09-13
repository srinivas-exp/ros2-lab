"""Lesson 3: use position feedback to drive to a destination."""
from enum import Enum
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from turtlesim_msgs.msg import Pose

class Distance:
    def __init__(self, distance, heading):
        self.distance = distance
        self.heading = heading
class Stage(Enum):
    OUTBOUND = 1
    WAITING = 2
    INBOUND = 3
    COMPLETE = 4

class GoalDriver(Node):
    def __init__(self):
        super().__init__('goal_driver')
        self.target_x = 10.0
        self.target_y = 5.0
        self.waypoint_index = 0.0
        self.original_x = None
        self.original_y = None
        self.stage = Stage.OUTBOUND
        self.resume_at = 0.0
        self.WAIT_SECONDS = 5.0
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose, 'turtle1/pose', self.on_pose, 10)
        self.get_logger().info(
            f'Driving to ({self.target_x}, {self.target_y}). Press Ctrl+C to exit.')

    def find_distance(self, cur_x, cur_y, target_x, target_y):
        dx = target_x - cur_x
        dy = target_y - cur_y
        return Distance(math.hypot(dx, dy), math.atan2(dy, dx))

    def on_pose(self, pose):
        if self.stage is Stage.COMPLETE:
            return
        elif self.stage is Stage.WAITING:
            if self.resume_at < time.monotonic():
                self.stage = Stage.INBOUND
            return
        elif self.original_x is None:
            self.original_x = pose.x
            self.original_y = pose.y

        if self.stage is Stage.OUTBOUND:
            goal = self.find_distance(pose.x, pose.y, self.target_x, self.target_y)
        else:
            goal = self.find_distance(pose.x, pose.y, self.original_x, self.original_y)
        command = Twist()

        if self.stage is Stage.OUTBOUND and goal.distance < 0.15:
            self.stage = Stage.WAITING
            self.publisher.publish(command)  # Zero speeds: stop.
            self.resume_at = time.monotonic() + self.WAIT_SECONDS
            self.get_logger().info('Arrived!')
            return

        if self.stage is Stage.INBOUND and goal.distance < 0.15:
            self.stage = Stage.COMPLETE
            self.publisher.publish(command)  # Zero speeds: stop.
            self.get_logger().info('Completed cycle! Press Ctrl+C to exit.')
            return

        error = goal.heading - pose.theta
        # Choose the shortest turn, including across the -pi/pi boundary.
        error = math.atan2(math.sin(error), math.cos(error))

        # Larger heading error gives a faster turn, capped at 2 rad/s.
        command.angular.z = max(-2.0, min(2.0, 4.0 * error))
        # Face the destination before moving forward; slow down near it.
        if abs(error) < 0.3:
            command.linear.x = min(1.5, goal.distance)
        self.publisher.publish(command)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GoalDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
