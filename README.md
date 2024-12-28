# MREN 203

MREN 203 is a course offered by Queen's University as a core design course in the amazing Mechatronics and Robotics Engineering program. I want to make that clear.<br />

This document will go over my personal contributions to the team project.

First I'll talk about our provided resources, then a journey about micro-ROS, and finally a break down of our final ROS2 nodes.

## The Provided Robot

We were provided a [Lynxmotion 4 wheeled robot](https://ca.robotshop.com/products/4wd1-robot-aluminum-kit) some basic sensors, a Raspberry Pi 4B, a Raspberry Pi Pico, and an RPLIDAR A1 2D LiDAR.

They gave us some documentation, but nothing beyond datasheets and some starter code for reading the sensors and driving the motors, which we didn't end up using.

## A Micro Journey

Though the PI could support everything, I did some research and learned about micro-ROS and wanted to set it up for the Pico.
I learned a ton getting it to work and eventually started driving the robot around. Everything changed when I accidentily snapped our SD card. Thankfully all the code was on GitHub, but after reinstalling everything, I could not get micro-ROS to build anymore. It seems something changed in those months that broke compatibility.
In the end, we pivoted back to using the PI for everything.

## camera_driver
If you're unfamiliar with ROS nodes. ROS is a data distribution service (DDS) wrapper which allows programs (called nodes) to interact with eachother over a standard interface. This is implemented with a pub/sub architecture plus some other goodies.

The camera driver node reads images from our webcam and publishes them over ROS. It also gets our pose from Nav2 and when the user clicks on a point on the map, finds all the images which captured that point and stitches them into a video. This node was mostly written by my group but I helped integrate it and get it working towards the end.

## cbt_bringup

This contains the launchfiles to start different configurations of the stack for testing.

## cbt_guidance

Nav2 handles navigation and high level control. That is, we give it a destination, and it tells us the linear and angular velocity that we must achieve to get there.
Guidance handles finding the destination. Essentially it searches the map and global costmap for nearby unexplored areas and when there are none, it returns to the stop where it was dropped off.
The algorithm simply uses a bunch of heuristics to pick a point that is near the robot, near it's current destination, and far from walls.

## robot_driver

the robot driver handles the 2nd half of Nav2. It listens to Nav2 and actuates the given command with a PID controller. It also publishes all the right information to keep the Nav2 stack happy. This was definitely the hardest part as I came into this project with very little ROS knowledge. Most of the struggles were due to lacking documentation or error reporting. Nav2 would just fail with no given reason. A good example is that for SLAM toolbox you need to broadcast the base_link to laser transform and change the frame_id to laser in the driver. This took hours to figure out but could have been minutes with good documentation or error messages. Another example is that you need to publish odometry on /odom AND broadcast it on the odom to base_link transform. To my knowledge this is not documented in primary sources.
