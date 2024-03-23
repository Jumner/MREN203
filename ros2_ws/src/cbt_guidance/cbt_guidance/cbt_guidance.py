import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from math import atan2, asin
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TransformStamped, Transform
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Guidance(Node):
    def __init__(self):
        super().__init__('cbt_guidance')
        self.cost_subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map = OccupancyGrid()
        self.transform = Transform()
        self.last_goal = (0.0, 0.0, 0.0)
        self.costmap = OccupancyGrid()

    def costmap_callback(self, msg):
        self.costmap = msg

    def map_callback(self, msg):
        self.map = msg
        # Get current pose from tf2
        try:
            self.transform = self.tf_buffer.lookup_transform(
                'base_link',
                'map',
                rclpy.time.Time()).transform
        except TransformException as ex:
            self.get_logger().warning(f'Could not get base_link transform: {ex}')
            return

        x, y, yaw = self.pickPose(msg)

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = self.transform.rotation

        self.get_logger().info(str(goal_pose))
        # -1 is unknown, 0 is clear, 100 is wall
        self.publisher.publish(goal_pose)

    def pickPose(self, map):
        poses = []
        resolution = map.info.resolution
        width = map.info.width
        height = map.info.height
        px = 0.0
        py = 0.0
        yaw = 0.0
        for y in range(1, height-1):
            for x in range(1, width-1):
                if map.data[y * width + x] != -1:
                    continue # Ignore known
                neighbours = 0
                h = map.data[y * width + (x+1)] + map.data[y * width + (x-1)]
                v = map.data[(y+1) * width + x] + map.data[(y-1) * width + x]
                neighbours += h
                neighbours += v
                if (neighbours == -4 or neighbours >= 0):
                    continue # ignore squares with no known neighbours, and isolated unknown squares
                px = map.info.origin.position.x + x * resolution
                py = map.info.origin.position.y + y * resolution
                cmx = int((px - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
                cmy = int((py - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
                costmap_index = cmy * self.costmap.info.width + cmx
                if (costmap_index < 0 or costmap_index >= len(self.costmap.data) or self.costmap.data[costmap_index] >= 90): 
                    continue  # Avoid near walls 100 is wall, 99 is inflated obstacle
                poses.append((px, py, yaw))
        if (len(poses) == 0):  # Mapping done
            return (0, 0, 0)  # Return to origin
        best_cost = ((poses[0][0] - self.last_goal[0]) ** 2 + (poses[0][1] - self.last_goal[1]) ** 2) * abs(2*asin(self.transform.rotation.z) - atan2(poses[0][1] - self.transform.translation.y, poses[0][0] - self.transform.translation.x))
        while len(poses) > 1:
            cost = ((poses[1][0] - self.last_goal[0]) ** 2 + (poses[1][1] - self.last_goal[1]) ** 2) * abs(2*asin(self.transform.rotation.z) - atan2(poses[1][1] - self.transform.translation.y, poses[1][0] - self.transform.translation.x))
            if cost < best_cost:
                best_cost = cost
                poses.pop(0)
            else:
                poses.pop(1)
        self.get_logger().info(f'\nPose angle: {2*asin(self.transform.rotation.z)}\nPoint angle: {atan2(poses[0][1] - self.transform.translation.y, poses[0][0] - self.transform.translation.x)}')
        self.last_goal = poses[0]
        return self.last_goal

def main(args=None):
    rclpy.init(args=args)
    node = Guidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
