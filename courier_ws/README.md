### commands to generate package
```ps1
# creates a plain package
ros2 pkg create turtle_courier --build-type ament_python --license Apache-2.0 --node-name goal_driver --dependencies rclpy geometry_msgs turtlesim_msgs std_srvs nav_msgs

# Update the turtle_courier/goal_driver.py as needed
# ... in a text editor

# Build the package
python -m colcon build --merge-install --packages-select turtle_courier

# source the local install
.\install\local_setup.ps1

# finally, run the program
ros2 run turtle_courier goal_driver --ros-args -p wait_seconds:=2.0

# Optional: set the wait seconds at each waypoint
ros2 param set /goal_driver wait_seconds 3.0
```