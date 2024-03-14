
# LAUNCH
ros2 launch sllidar_ros2 sllidar_a1_launch.py &
ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "1" "0" "base_link" "laser" &
ros2 run robot_driver robot_driver &
ros2 launch nav2_bringup navigation_launch.py &
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args -p base_frame:=base_link --ros-args -p max_laser_range:=5.0 &
