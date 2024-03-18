import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    nav2 = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py'])
    )
    lidar = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('sllidar_ros2'), 'launch'), '/sllidar_a1_launch.py'])
    )

    lidar_transform = Node(package='tf2_ros', executable='static_transform_publisher', arguments=["0.065", "0.05", "0", "0", "0", "1", "0", "base_link", "laser"])

    robot_driver = Node(package='robot_driver', executable='robot_driver')

    slam = Node( package="slam_toolbox", executable="sync_slam_toolbox_node", parameters=[ {"base_frame": "base_link", "max_laser_range": "5.0"}])

    foxglove = Node(package="foxglove_bridge", executable="foxglove_bridge")
    camera = Node(package="py_pubsub", executable="talker")

    return LaunchDescription([nav2, lidar_transform, lidar, robot_driver, slam, foxglove, camera])
