# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import numpy as np
from math import asin, atan2, hypot
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pathlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, PointStamped, Transform, TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import os

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fps = 5
        self.timer = self.create_timer(1/self.fps, self.timer_callback)
        self.point_subscription = self.create_subscription(PointStamped, 'target_point', self.target_callback,10)

        self.transform = Transform()
        self.pose_array = []
        self.frame_count = 0

        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0
        self.max_radius = 8.0
        self.angle_threshhold = np.pi/6.0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if frame is None:
            self.get_logger().warning("failed to read frame")
            return
        # Get current pose from tf2
        try:
            self.transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
        except TransformException as ex:
            self.get_logger().warning(f'Could not get base_link transform: {ex}')
            return

        image_name = f'/home/CBT/MREN203/ros2_ws/src/camera_driver/camera_driver/images/{self.frame_count}.jpg'
        # self.get_logger().info(f'Logging frame: {image_name}')
        cv2.imwrite(image_name, frame) # RIP Storage
        self.get_logger().info(f'Logged frame: {self.frame_count}.jpg')

        # Gather all required pose data
        self.pose_x = self.transform.translation.x
        self.pose_y = self.transform.translation.y
        
        # Retrieve Yaw
        self.pose_theta = 2*asin(self.transform.rotation.z)

        # Put frame in array
        self.pose_array.append((image_name, self.pose_x, self.pose_y, self.pose_theta))

        # Publish on ROS
        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher_.publish(img_msg)
        self.frame_count += 1

    def target_callback(self, target_point):
        target_point = target_point.point
        self.get_logger().info(str(self.pose_array))
        self.get_logger().info(str(target_point))
        target_point = [target_point.x, target_point.y]
        filtered_array = list(map(lambda pose: pose[0], filter(lambda pose: self.is_seen(target_point, pose), self.pose_array)))
        self.get_logger().info(str(filtered_array))

        # Turn filtered_array into video
        output_video_filename = '/home/CBT/MREN203/output/output_video.mp4'
        self.create_video_from_images(filtered_array, output_video_filename)

    def create_video_from_images(self, filtered_array, output_video_filename):
        # Define the path to the image folder

        # Get the dimensions of the first image
        if (len(filtered_array) < 1):
            self.get_logger().warning("No images to create video of")
            return
        first_image = cv2.imread(filtered_array[0])
        height, width, _ = first_image.shape

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_video_filename, fourcc, self.fps, (width, height))

        # Iterate through the filtered array and write images to the video
        for image_filename in filtered_array:
            if os.path.exists(image_filename):
                image = cv2.imread(image_filename)
                out.write(image)
                self.get_logger().info(image_filename)

        # Release the VideoWriter object and close all windows
        out.release()
        self.get_logger().info("Outputed Video")

    def is_seen(self, point, pose):
        # Gather target coordinates
        target_x = point[0]
        target_y = point[1]
        # Gather robot pose coordinates
        robot_x = pose[1]
        robot_y = pose[2]
        # Get distance of point from robot
        distance = hypot(target_x - robot_x, target_y - robot_y)
        # Exclude if the point is outside the maximum radius
        if distance > self.max_radius:
            return False
        # Compute angle between input coordinate and pose
        robot_theta = pose[3]
        angle = atan2(target_y - robot_y, target_x - robot_x)
        # Compute the difference between pose_theta and angle
        angle_difference = abs(robot_theta - angle)
        # Exclude if the point is outside the FOV cone
        if angle_difference > self.angle_threshhold:
            return False
        # Point is within 3m and within the FOV cone, it is seen
        return True

def main(args=None):
    rclpy.init(args=args)

    node = CameraDriver()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
