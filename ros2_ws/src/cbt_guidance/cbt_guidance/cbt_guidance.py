import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from math import sin, cos, atan2, asin
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, PoseWithCovarianceStamped

class Guidance(Node):
    def __init__(self):
        super().__init__('cbt_guidance')
        self.subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.map = OccupancyGrid()
        self.pose = Pose()

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def map_callback(self, msg):
        if (self.map.data == msg.data):
            self.map = msg
            return

        x, y, yaw = self.pickPose(msg)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        q = Quaternion()
        q.z = sin(yaw / 2)
        q.w = cos(yaw / 2)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = self.pose.orientation
        
        self.get_logger().info(str(pose))
        # -1 is unknown, 0 is clear, 100 is wall
        self.publisher.publish(pose)

    def pickPose(self, msg):
        poses = []
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        px = 0.0
        py = 0.0
        yaw = 0.0
        for y in range(1, height-1):
            for x in range(1, width-1):
                if msg.data[y * width + x] != -1:
                    continue # Ignore known
                neighbours = 0
                h = msg.data[y * width + (x+1)] + msg.data[y * width + (x-1)]
                v = msg.data[(y+1) * width + x] + msg.data[(y-1) * width + x]
                neighbours += h
                neighbours += v
                if (neighbours == -4 or neighbours >= 0):
                    continue # ignore squares with no known neighbours, squares bordering walls, and isolated unknown squares
                px = msg.info.origin.position.x + x * resolution
                py = msg.info.origin.position.y + y * resolution
                yaw = 0.0
                poses.append((px, py, yaw))
        best_cost = ((poses[0][0] - self.pose.position.x) ** 2 + (poses[0][1] - self.pose.position.y) ** 2) * abs(asin(self.pose.orientation.z) + atan2(poses[0][1] - self.pose.position.y, poses[0][0] - self.pose.position.x))
        while len(poses) > 1:
            cost = ((poses[1][0] - self.pose.position.x) ** 2 + (poses[1][1] - self.pose.position.y) ** 2) * abs(asin(self.pose.orientation.z) + atan2(poses[1][1] - self.pose.position.y, poses[1][0] - self.pose.position.x))
            if cost < best_cost:
                best_cost = cost
                poses.pop(0)
            else:
                poses.pop(1)
        return poses[0]

def main(args=None):
    rclpy.init(args=args)
    node = Guidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
