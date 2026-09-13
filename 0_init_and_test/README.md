### Commands to set it up (Windows - Powershell):
```ps1
    # in each terminal - pre configure to allow using ros2
    # change path as needed
    pixi shell
    . D:\ros2\ros2-lyrical-2026-08-07\ros2-windows\local_setup.ps1
    # Run only if failed due to QT plugin issue
    # $env:QT_QPA_PLATFORM_PLUGIN_PATH = 'D:\ros2\ros2-lyrical-2026-08-07\ros2-windows\.pixi\envs\default\Library\lib\qt6\plugins\platforms'

    # start Turtlebbot sim
    ros2 run turtlesim turtlesim_node.exe

    # Listen to velocity changes
    ros2 topic echo /turtle1/cmd_vel

    # 1. Run the python program
     ..\.pixi\envs\default\python.exe .\0_init_and_test\0_turtlebot_circle.py

    # 2. Run the python program
    ..\.pixi\envs\default\python.exe .\0_init_and_test\1_turtlebot_goal.py

    # Optional: Reset Turtlebot location
    ros2 service call /reset std_srvs/srv/Empty "{}"

    # Optional: Rotate the Turtle sim direction (x = 5.5 && y = 5.5 - center, theta = pi/2)
    ros2 service call /turtle1/teleport_absolute turtlesim_msgs/srv/TeleportAbsolute "{x: 5.5, y: 5.5, theta: 1.5708}"
    # Optional: Run the controls using keyboard
    ros2 run turtlesim turtle_teleop_key.exe
```