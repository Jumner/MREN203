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

import cv2
import os

cap = cv2.VideoCapture(0)
frame_count = 4
ret, frame = cap.read()
image_name = f'/home/CBT/MREN203/ros2_ws/src/camera_driver/camera_driver/images/{frame_count}.jpg'
print(image_name)
cv2.imwrite(image_name, frame) # RIP Storage
