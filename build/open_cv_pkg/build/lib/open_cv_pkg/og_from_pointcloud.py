
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
class OccupancyGridFromPointCloud(Node):
    def __init__(self):
        super().__init__('occupancy_grid_from_pointcloud')
        
        # Publisher to publish the occupancy grid
        self.pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        # Subscribe to the point cloud topic
        self.sub = self.create_subscription(
            PointCloud2, 
            '/camera/camera/depth/color/points',  # Change topic if needed
            self.pointcloud_callback, 
            10
        )
        
        self.get_logger().info('Occupancy Grid Node started.')
        # Define grid parameters
        self.grid_size = 0.1  # The resolution of the grid (meters per cell)
        self.grid_width = 100  # Number of grid cells in width
        self.grid_height = 100  # Number of grid cells in height
        # Initialize the occupancy grid with zeros (empty grid)
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width))  
    def pointcloud_callback(self, msg):
        """Process point cloud data and update the occupancy grid."""
        
        # Extract the point cloud data from the ROS message
        point_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points = np.array(point_list, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        # Log the point cloud data size for debugging
        self.get_logger().info(f"Point cloud data received with {len(points)} points.")
        
        # Filter out points that are too far away or not within the expected depth range
        filtered_points = self.filter_boulders_and_craters(points)
        self.get_logger().info(f"Filtered points: {len(filtered_points)} points.")  # Log filtered points
        if len(filtered_points) == 0:
            self.get_logger().warn("No points left after filtering.")  # Warn if no points passed filtering
        
        # Convert filtered point cloud to 2D grid coordinates
        for point in filtered_points:
            x, y, z = point
            # Convert to grid cell coordinates (assuming the center of the robot is at the origin)
            grid_x = int((x + self.grid_size * self.grid_width / 2) // self.grid_size)
            grid_y = int((y + self.grid_size * self.grid_height / 2) // self.grid_size)
            # Ensure the point falls within the grid's boundaries
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                # Mark the grid cell as occupied (obstacle)
                self.occupancy_grid[grid_y, grid_x] = 100  # Occupied cell
                self.get_logger().info(f"Mapped point ({x}, {y}, {z}) to grid position ({grid_x}, {grid_y})")
        # Log the number of occupied cells
        occupied_cells = np.count_nonzero(self.occupancy_grid)
        self.get_logger().info(f"Number of occupied cells: {occupied_cells}")
        if occupied_cells == 0:
            self.get_logger().warn("No occupied cells detected in the occupancy grid.")
        # Create an OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "camera_link"  # Set the frame of reference (could also use robot base frame)
        
        grid_msg.info.resolution = self.grid_size  # The resolution of each grid cell
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        
        # Flatten the occupancy grid array and convert it to a list of integers
        grid_msg.data = self.occupancy_grid.flatten().astype(int).tolist()
        
        # Publish the occupancy grid
        self.pub.publish(grid_msg)
        self.get_logger().info(f"Published Occupancy Grid with {occupied_cells} occupied cells.")
    def filter_boulders_and_craters(self, points):
        """Filter out irrelevant points, keeping only boulders and craters based on depth and size."""
        
        # Depth threshold values (adjust based on your environment)
        min_depth = 0.1  # Min depth (close objects like boulders/craters)
        max_depth = 2.0  # Max depth (objects far away like walls)
        # Extract the z-axis (depth) values and filter based on depth range
        z_values = points['z']
        filtered_points = points[(z_values >= min_depth) & (z_values <= max_depth)]
        
        # Log the number of points after filtering
        self.get_logger().info(f"Filtered points based on depth: {len(filtered_points)}")
        
        return filtered_points
def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridFromPointCloud()
    
    try:
        rclpy.spin(node)  # Keep the node running and processing incoming data
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Gracefully shut down the node
        rclpy.shutdown()
if __name__ == '__main__':
    main()
