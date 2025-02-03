import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from transforms3d.affines import compose
from transforms3d.quaternions import quat2mat
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class OccupancyGridFromPointCloud(Node):
    def __init__(self):
        super().__init__('occupancy_grid_from_pointcloud')

        # Publisher to publish the occupancy grid
        self.pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Publisher for debug markers
        self.debug_marker_pub = self.create_publisher(MarkerArray, '/debug_markers', 10)

        #publish camera marker
        self.camera_marker_pub = self.create_publisher(Marker, '/camera_marker', 10)

        # Publisher to publish the camera FoV
        self.fov_pub = self.create_publisher(Marker, '/camera_fov', 10)

        # Subscribe to the point cloud topic
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',  # Adjust topic as needed
            self.pointcloud_callback,
            10
        )

        # TF Buffer and Listener
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Occupancy Grid Node started.')

        # Define grid parameters
        self.grid_size = 0.2  # The resolution of grid cells (meters per cell)
        self.grid_width = int(10 / self.grid_size)  # Grid width (number of columns)
        self.grid_height = int(10 / self.grid_size)  # Grid height (number of rows)
        self.occupancy_grid = np.full((self.grid_height, self.grid_width), 0)  # Initialize grid as free

        # Publish the camera FoV once at startup
        self.publish_camera_fov()

        # Create timer to continuously publish FoV
        self.create_timer(1.0, self.publish_camera_fov)  # Publishing every 1 second

        # Debugging control
        self.output_frequency = 5
        self.output_count = 0
    

    def pointcloud_callback(self, msg):
        """Process point cloud data and update the occupancy grid."""
        self.occupancy_grid.fill(0)  # Reset grid to free space

        # Try to get the transformation from camera to base_link (ground-relative frame)
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                "camera_link",  # Target frame
                msg.header.frame_id,  # Source frame
                rclpy.time.Time()
            )

            # Extract translation and rotation
            translation = [
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z
            ]
            quaternion = [
                transform_stamped.transform.rotation.w,
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z
            ]
            rotation_matrix = quat2mat(quaternion)  # Convert quaternion to rotation matrix
            transform_matrix = compose(translation, rotation_matrix, [1, 1, 1])  # Build 4x4 matrix

        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return

        # Extract point cloud data
        point_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points_camera = np.array([[x, y, z, 1] for x, y, z in point_list])  # Homogeneous coordinates

        # Transform points into the base_link frame
        points_world = points_camera @ transform_matrix.T
        transformed_points = points_world[:, :3]  # Extract x, y, z in world frame

        # Debug: Log transformed points
        self.log_counter = getattr(self, 'log_counter', 0) + 1
        if self.log_counter % 10 == 0:
            for point in transformed_points[:10]:
                self.get_logger().info(f"Transformed Point: x={point[0]:.2f}, y={point[1]:.2f}, z={point[2]:.2f}")

        # Filter points within a specific range
        filtered_points = self.filter_points(transformed_points)

        # Map points to the occupancy grid
        self.update_occupancy_grid(filtered_points)

        # Publish the occupancy grid
        grid_msg = self.create_occupancy_grid_msg()
        self.pub.publish(grid_msg)

        # Publish debug markers
        self.publish_debug_markers(filtered_points)

        self.output_count += 1
        if self.output_count % self.output_frequency == 0:
            self.get_logger().info("Published Occupancy Grid.")

    def filter_points(self, points):
        """Filter points within a specified height and depth range."""
        min_height = 0.0  # Ground level
        max_height = 2.0  # Maximum height to consider
        min_depth = 0.1
        max_depth = 2.0

        filtered = [point for point in points if min_depth <= point[2] <= max_depth and min_height <= point[1] <= max_height]
        return filtered

    def update_occupancy_grid(self, points):
        """Map points to the occupancy grid."""
        for x, y, z in points:
            # Convert world-frame coordinates to grid indices
            grid_x = int((x / self.grid_size) + self.grid_width / 2)
            grid_y = int(self.grid_height / 2 - (z / self.grid_size))  # Use z for depth mapping
            rel_y = y - 0.0

            # Check bounds
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                # Mark the cell based on height (y-axis)
                if rel_y < 0.7:
                    self.occupancy_grid[grid_y, grid_x] = 100  # Low obstacle
                elif 0.7 <= rel_y <= 1.0:
                    self.occupancy_grid[grid_y, grid_x] = 50  # Medium obstacle
                else:
                    self.occupancy_grid[grid_y, grid_x] = 20  # Tall obstacle

    def create_occupancy_grid_msg(self):
        """Create the OccupancyGrid message."""
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "camera_link"  # Now relative to base_link
        grid_msg.info.resolution = self.grid_size
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.data = self.occupancy_grid.flatten().astype(int).tolist()
        return grid_msg

    def publish_debug_markers(self, points):
        """Publish debug markers for filtered points."""
        marker_array = MarkerArray()
        for i, (x, y, z) in enumerate(points):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "debug_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.debug_marker_pub.publish(marker_array)
    def publish_camera_marker(self):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_marker"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.g = 1.0
        self.camera_marker_pub.publish(marker)

    def publish_camera_fov(self):
        """Publish the camera FoV marker."""
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_fov"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.x = 55.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 90.0 #0.0  # Add rotation if necessary
        marker.pose.orientation.w = 1.0  # Quaternion for orientation

        fov_horizontal = 69 * (np.pi / 180)  # Horizontal FoV in radians
        fov_vertical = 42 * (np.pi / 180)    # Vertical FoV in radians
        max_depth = 2.0  # Maximum depth for the FOV lines

        # Flipping axes
        center = [0.0, 0.0, 0.0]
        top_left = [max_depth * np.tan(fov_vertical / 2), max_depth * np.tan(fov_horizontal / 2), max_depth]  # Swapped axes
        top_right = [-top_left[0], top_left[1], max_depth]
        bottom_left = [top_left[0], -top_left[1], max_depth]
        bottom_right = [-top_left[0], -top_left[1], max_depth]

        points = [
            center, top_left, center, top_right, center, bottom_left, center, bottom_right,
            top_left, top_right, top_right, bottom_right, bottom_right, bottom_left, bottom_left, top_left
        ]

        for p in points:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = p[2]
            marker.points.append(point)

        marker.scale.x = 0.1  # Line thickness
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.fov_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridFromPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
