# Launching Navigation

ros2 launch turtlebot4_navigation localization.launch.py map:=src/dis_task2/map/map.yaml
ros2 launch turtlebot4_navigation nav2.launch.py
ros2 launch turtlebot4_viz view_robot.launch.py