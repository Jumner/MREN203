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
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pathlib

from std_msgs.msg import String




class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if frame is None:
            print("failed to read frame")
            return 
            
        frame = cv2.flip(frame, 1)
        # cv2.imshow('frame'. frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.cap.release()
        #     cv2.destroyAllWindows()
        #     rclpy.shutdown()

        # msg = String()
        # msg.data = 'Hello World: %d' % self.i

        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher_.publish(img_msg)

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image, 
            'camera/image',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, img_msg): # Activates whenever and image is recieved/taken
        pass






def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
