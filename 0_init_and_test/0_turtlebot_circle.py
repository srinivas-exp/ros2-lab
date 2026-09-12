import time
import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist


class CircleDriver(Node):
	def __init__(self):
		super().__init__('circle_driver')

		# Publish Twist messages on the topic the turtle listens to.
		self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

		# Call drive() every 0.1 seconds: 10 commands per second.
		self.timer = self.create_timer(0.1, self.drive)
		self.get_logger().info('Drawing circles. Press Ctrl+C to stop.')
		self.stop_after_sec = 15.0
		self.start_time = time.monotonic()

	def drive(self):
		elapsed = time.monotonic() - self.start_time
		speed = 1.5
		turn_rate = 1.0
		circle_time = 2*math.pi /turn_rate
		direction = 1.0 if int(elapsed/circle_time)%2 == 0 else -1.0

		if elapsed > self.stop_after_sec or elapsed > 2*(2*math.pi /turn_rate):
			self.publisher.publish(Twist())
			self.timer.cancel()
			self.get_logger().info("Stopping after " + str(int(min(self.stop_after_sec, 2*(2*math.pi /turn_rate)))) + " seconds")
			exit(0)
		command = Twist()

		command.linear.x = -speed   # Forward speed in simulator units/second.
		#command.linear.y = 2.5
		command.angular.z = -direction*turn_rate  # Turning speed in radians/second.
		self.publisher.publish(command)

def main():
	# Let Python handle Ctrl+C so we can send a stop before shutting ROS down.
	rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
	node = CircleDriver()
	try:
		rclpy.spin(node)  # Keep processing timer callbacks.
	except KeyboardInterrupt:
		pass
	finally:
		print("shutting down...")
		node.timer.cancel()
		node.publisher.publish(Twist())  # All-zero speeds mean stop.
		time.sleep(0.2)  # Allow the stop message time to leave the process.
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()


