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
import math
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pathlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, 'pose',self.pose_callback,10)
        self.point_subscription = self.create_subscription(PointStamped, 'target_point', self.target_callback,10)

        self.pose = Pose()
        self.pose_array = []
        self.frame_count = 0

        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if frame is None:
            self.get_logger().warning("failed to read frame")
            return
            
        frame = cv2.flip(frame, 1)
        image_name = f'images/{self.frame_count}.jpg'
        self.get_logger().info(f'Logging frame: {image_name}')
        # cv2.imwrite(image_name, frame) TODO

        # Gather all required pose data
        self.pose_x = self.pose.position.x
        self.pose_y = self.pose.position.y
        w = self.pose.orientation.w
        z = self.pose.orientation.z
        x = self.pose.orientation.x
        y = self.pose.orientation.y
        
        # Retrieve Yaw
        t3 = 2.0 * (x * y * z * w)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        self.pose_theta = yaw

        # Put frame in array
        self.pose_array.append((image_name, self.pose_x, self.pose_y, self.pose_theta))

        # Publish on ROS
        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher_.publish(img_msg)
        self.frame_count += 1

    def pose_callback(self, pose_msg): # Activates whenever a pose is recieved
        self.pose = pose_msg.pose.pose

    def target_callback(self, target_point):
        target_point = target_point.point
        self.get_logger().info(str(self.pose_array))
        self.get_logger().info(str(target_point))
        filtered_array = list(filter(lambda pose: is_seen(target_point, pose), self.pose_array))
        self.get_logger().info(str(filtered_array))


def is_seen(point, pose):
    # If a point is seen by a pose
    return True

def main(args=None):
    rclpy.init(args=args)

    node = CameraDriver()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
