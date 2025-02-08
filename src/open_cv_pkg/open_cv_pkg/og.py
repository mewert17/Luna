import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        # Subscriber to non-ground points
        self.sub = self.create_subscription(
            PointCloud2,
            '/non_ground_points',
            self.pointcloud_callback,
            10
        )

        # Publisher for the occupancy grid
        self.occ_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Timer for rate limiting
        self.timer = self.create_timer(2.0, self.publish_occupancy_grid)  # Publish every 1.0 seconds

        # Storage for grid
        self.latest_grid = None

        # Grid parameters
        self.grid_size = 100  # Number of grid cells (100x100)
        self.resolution = 0.1  # Grid cell size in meters (5cm per cell)
        self.map_origin = [-2.5, -2.5]  # Origin (X, Y) in meters (centered at (0,0))
        self.frame_id = "camera_link"  # Frame ID (can be changed to "camera_link" if needed)

        self.get_logger().info('Occupancy Grid Node started.')

    def pointcloud_callback(self, msg):
        """Convert non-ground points into an occupancy grid."""
        # Convert PointCloud2 to NumPy array
        points = np.array([
            (p[0], p[1]) for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        ], dtype=np.float32)

        # Create an empty grid (-1 = unknown)
        grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Convert point coordinates to grid indices
        for x, y in points:
            grid_x = int((x - self.map_origin[0]) / self.resolution)
            grid_y = int((y - self.map_origin[1]) / self.resolution)

            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                grid[grid_y, grid_x] = 100  # Mark as occupied

        # Store the grid for the timer to publish
        self.latest_grid = grid

    def publish_occupancy_grid(self):
        """Publish the latest occupancy grid at a controlled rate."""
        if self.latest_grid is None:
            return  # No grid to publish yet

        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.frame_id

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size
        grid_msg.info.origin.position.x = self.map_origin[0]
        grid_msg.info.origin.position.y = self.map_origin[1]
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.x = 0.0
        grid_msg.info.origin.orientation.y = 0.0
        grid_msg.info.origin.orientation.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # Convert NumPy array to a flat list
        grid_msg.data = self.latest_grid.flatten().tolist()

        self.occ_pub.publish(grid_msg)
        self.get_logger().info("Published Occupancy Grid")

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
