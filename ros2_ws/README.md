# LiDAR
ros2 launch sllidar_ros2 sllidar_a1_launch.py
# RVIZ
rviz2 -d ~/MREN203/ros2_ws/src/sllidar_ros2/rviz/sllidar_ros2.rviz
# MicroROS
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_304C61E62E421B93-if00
# Send test command
ros2 topic pub --once /pico_motor_commands mren_interfaces/msg/PicoMotorCommands "{left_wheel_velocity: 0.0, right_wheel_velocity: 0.0}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1, y: 0, z: 0},angular: {x: 0, y: 0, z: 0}}"
# NAV2
ros2 launch nav2_bringup navigation_launch.py
# SLAM
ros2 launch slam_toolbox online_async_launch.py


# LAUNCH
ros2 launch sllidar_ros2 sllidar_a1_launch.py
ros2 run tf2_ros static_transform_publisher "0.065" "0.05" "0" "0" "0" "1" "0" "base_link" "laser"
ros2 run robot_driver robot_driver
ros2 launch nav2_bringup navigation_launch.py
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args -p base_frame:=base_link --ros-args -p max_laser_range:=5.0
