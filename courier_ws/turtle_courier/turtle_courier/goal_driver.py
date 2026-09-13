"""Lesson 3: use position feedback to drive to a destination."""
from enum import Enum
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from turtlesim_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool

class Distance:
    def __init__(self, distance, heading):
        self.distance = distance
        self.heading = heading
class Stage(Enum):
    OUTBOUND = 1
    WAITING = 2
    COMPLETE = 3

class GoalDriver(Node):
    def __init__(self):
        super().__init__('goal_driver')
        self.declare_parameter('wait_seconds', 15.0)

        # super().Subscriber('/map', OccupancyGrid, self.map_cb)
        self.waypoint_index = 0
        self.waypoints = [
            (2.0, 4.0),
            (7.0, 8.0),
            (1.0, 5.0),
            (1.0, 15.0)
        ]
        self.initialized = False
        self.stage = Stage.OUTBOUND
        self.resume_at = 0.0
        self.paused = False
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose, 'turtle1/pose', self.on_pose, 10)
        self.pause_service = self.create_service(SetBool, 'pause', self.on_pause)
        self.get_logger().info(
            f'Driving to ({self.waypoints[self.waypoint_index]}, {self.waypoints[self.waypoint_index]}). Press Ctrl+C to exit.')

    def valid_waypoint(self, x, y):
        return (
            math.isfinite(x)
            and math.isfinite(y)
            and 0.5 <= x <= 10.5
            and 0.5 <= y <= 10.5
        )

    def on_pause(self, request, response):
        self.paused = request and request.data
        if self.paused:
            self.publisher.publish(Twist())
        response.success = True
        response.message = 'Paused successfully' if self.paused else 'Resumed or not paused'
        return response

    def map_cb(self, msg):
        # self.origin = msg.map_data.
        self.width = msg.info.width * msg.info.resolution
        self.height = msg.info.height * msg.info.resolution

    def find_distance(self, cur_x, cur_y, target_x, target_y):
        dx = target_x - cur_x
        dy = target_y - cur_y
        return Distance(math.hypot(dx, dy), math.atan2(dy, dx))

    def on_pose(self, pose):
        if self.stage is Stage.COMPLETE or self.paused:
            return
        elif self.stage is Stage.WAITING:
            if self.resume_at < time.monotonic():
                self.waypoint_index += 1
                while self.waypoint_index < len(self.waypoints) and not self.valid_waypoint(*self.waypoints[self.waypoint_index]):
                    self.get_logger().info(f'Skipping invalid waypoint {self.waypoints[self.waypoint_index]}')
                    self.waypoint_index += 1
                if self.waypoint_index >= len(self.waypoints):
                    self.stage = Stage.COMPLETE
                    self.get_logger().info('All deliveries completed!')
                else:
                    self.get_logger().info(
                        f'Driving to ({self.waypoints[self.waypoint_index]}, {self.waypoints[self.waypoint_index]}). Press Ctrl+C to exit.')
                    self.stage = Stage.OUTBOUND
            return
        elif not self.initialized:
            self.initialized = True
            self.waypoints.append((pose.x, pose.y))

        goal = self.find_distance(pose.x, pose.y, self.waypoints[self.waypoint_index][0], self.waypoints[self.waypoint_index][1])
        command = Twist()

        if self.stage is Stage.OUTBOUND and goal.distance < 0.15:
            self.stage = Stage.WAITING
            self.publisher.publish(command)  # Zero speeds: stop.
            wait_seconds = self.get_parameter('wait_seconds').value
            self.resume_at = time.monotonic() + wait_seconds
            self.get_logger().info(f'Arrived at {self.waypoints[self.waypoint_index]}!')
            self.get_logger().info(f'Waiting for {wait_seconds}...')
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
