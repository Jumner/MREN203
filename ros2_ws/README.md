# LiDAR
ros2 launch sllidar_ros2 sllidar_a1_launch.py
# MicroROS
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_304C61E62E421B93-if00
# Send test command
ros2 topic pub --once /pico_motor_commands mren_interfaces/msg/PicoMotorCommands "{left_wheel_velocity: 0.0, right_wheel_velocity: 0.0}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1, y: 0, z: 0},angular: {x: 0, y: 0, z: 0}}"
