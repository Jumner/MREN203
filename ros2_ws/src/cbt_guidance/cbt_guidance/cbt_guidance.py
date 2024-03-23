import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from math import atan2, asin, hypot
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
                'map',
                'base_link',
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

        # -1 is unknown, 0 is clear, 100 is wall
        self.publisher.publish(goal_pose)

    def pickPose(self, map):
        pose = (0, 0, 0) # Default to home
        resolution = map.info.resolution
        width = map.info.width
        height = map.info.height
        best_cost = 10000
        for y in range(1, height-1):
            for x in range(1, width-1):
                if map.data[y * width + x] != -1:
                    continue # Ignore known

                neighbours = map.data[y * width + (x+1)] + map.data[y * width + (x-1)] + map.data[(y+1) * width + x] + map.data[(y-1) * width + x]
                if (neighbours == -4 or neighbours >= 2):
                    continue # ignore squares with no known neighbours, and isolated unknown squares

                px = map.info.origin.position.x + x * resolution
                py = map.info.origin.position.y + y * resolution
                cmx = int((px - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
                cmy = int((py - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
                costmap_index = cmy * self.costmap.info.width + cmx
                if (costmap_index < 0 or costmap_index >= len(self.costmap.data) or self.costmap.data[costmap_index] >= 90): 
                    continue  # Avoid near walls 100 is wall, 99 is inflated obstacle
                
                
                if hypot(px - self.transform.translation.x, py - self.transform.translation.y) < 0.5:
                    continue  # Don't get too close

                # Point is valid, select best cost
                dist = hypot(px - self.last_goal[0], py - self.last_goal[1])
                angle_diff = abs(2*asin(self.transform.rotation.z) - atan2(py - self.transform.translation.y, px - self.transform.translation.x))
                cost = dist * angle_diff
                if cost < best_cost:
                    best_cost = cost
                    pose = (px, py, 0)
        self.get_logger().info(f'Picked pose: {pose} with cost: {best_cost}')
        self.last_goal = pose
        return self.last_goal

def main(args=None):
    rclpy.init(args=args)
    node = Guidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
