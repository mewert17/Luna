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
        self.grid_size = 0.4  # The resolution of grid cells (meters per cell)
        self.grid_width = 50  # Grid width (number of columns)
        self.grid_height = 50  # Grid height (number of rows)
        # Initialize the occupancy grid with unknown values (50 = Unknown)
        self.occupancy_grid = np.full((self.grid_height, self.grid_width), 50)  # Set all cells as unknown initially
        
        # For limiting debug output
        self.output_frequency = 5  # Limit to every 5 messages
        self.output_count = 0  # Counter to track output frequency

    def pointcloud_callback(self, msg):
        """Process point cloud data and update the occupancy grid."""
        
        # Extract the point cloud data from the ROS message
        point_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points = np.array(point_list, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

        # Debug output limited by frequency
        self.output_count += 1
        if self.output_count % self.output_frequency == 0:
            self.get_logger().info(f"Point cloud data received with {len(points)} points.")
        
        # Apply depth filtering based on the expected object distances (e.g., 0.2m - 2m)
        filtered_points = self.filter_boulders_and_craters(points)
        if self.output_count % self.output_frequency == 0:
            self.get_logger().info(f"Filtered points: {len(filtered_points)} points.")
        
        if len(filtered_points) == 0:
            self.get_logger().warn("No points left after filtering.")  # Warn if no points passed filtering
        
        # Convert filtered point cloud to 2D grid coordinates
        for point in filtered_points:
            x, y, z = point
            # Convert 3D (x, y) to 2D grid coordinates (ignoring z for simplicity)
            grid_x = int((x / self.grid_size) + self.grid_width / 2)
            grid_y = int((y / self.grid_size) + self.grid_height / 2)
            
            # Debug output for mapped grid coordinates (limited frequency)
            if self.output_count % self.output_frequency == 0:
                self.get_logger().debug(f"Processing point ({x}, {y}, {z}) mapped to grid ({grid_x}, {grid_y})")
            
            # Ensure the point falls within the grid's boundaries
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                # Differentiating obstacles based on depth
                if z < 0.5:  # If the object is very close (boulders, etc.)
                    self.occupancy_grid[grid_y, grid_x] = 100  # Boulders (occupied)
                elif 0.5 <= z <= 1.0:  # If the object is farther away (craters, etc.)
                    self.occupancy_grid[grid_y, grid_x] = 75  # Craters (occupied)
                else:  # If the object is further or too far (may represent background or large obstacles)
                    self.occupancy_grid[grid_y, grid_x] = 50  # Unknown or not an obstacle

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
        if self.output_count % self.output_frequency == 0:
            self.get_logger().info(f"Published Occupancy Grid.")

    def filter_boulders_and_craters(self, points):
        """Filter out irrelevant points, keeping only boulders and craters based on depth."""
        
        # Depth threshold values (adjust based on your environment)
        min_depth = 0.1  # Minimum depth (close objects like rocks/boulders)
        max_depth = 2.0  # Maximum depth (objects far away like walls)
        # Extract the z-axis (depth) values and filter based on depth range
        z_values = points['z']
        filtered_points = points[(z_values >= min_depth) & (z_values <= max_depth)]
        
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
