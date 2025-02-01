import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from transforms3d.affines import compose
from transforms3d.quaternions import quat2mat
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class OccupancyGridFromPointCloud(Node):
    def __init__(self):
        super().__init__('occupancy_grid_from_pointcloud')

        # Publisher to publish the occupancy grid
        self.pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Publisher to publish the camera marker
        self.marker_pub = self.create_publisher(Marker, '/camera_marker', 10)

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
        self.grid_width = 25  # Grid width (number of columns)
        self.grid_height = 25  # Grid height (number of rows)
        self.occupancy_grid = np.full((self.grid_height, self.grid_width), 50)  # Initialize grid with unknown values

        # Debugging control
        self.output_frequency = 5
        self.output_count = 0

        # Publish the camera FoV once at startup
        self.publish_camera_fov()

    def pointcloud_callback(self, msg):
        """Process point cloud data and update the occupancy grid."""
        try:
            # Transform lookup from camera_depth_optical_frame to source frame
            transform_stamped = self.tf_buffer.lookup_transform(
                "camera_depth_optical_frame",  # Target frame
                msg.header.frame_id,  # Source frame
                rclpy.time.Time()
            )

            # Build the transformation matrix
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
            transform_matrix = compose(translation, rotation_matrix, [1, 1, 1])  # Full 4x4 matrix

        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return

        # Extract point cloud data
        point_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points = np.array(point_list, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

        # Debugging output
        self.output_count += 1
        if self.output_count % self.output_frequency == 0:
            self.get_logger().info(f"Point cloud data received with {len(points)} points.")

        # Apply depth filtering
        filtered_points = self.filter_boulders_and_craters(points)
        if self.output_count % self.output_frequency == 0:
            self.get_logger().info(f"Filtered points: {len(filtered_points)} points.")

        # Map points to the occupancy grid
        for point in filtered_points:
            x, y, z = point
            grid_x = int((y / self.grid_size) + self.grid_width / 2)
            grid_y = int((x / self.grid_size) + self.grid_height / 2)

            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                if z < 0.5:  # Boulders
                    self.occupancy_grid[grid_y, grid_x] = 100
                elif 0.5 <= z <= 1.0:  # Craters
                    self.occupancy_grid[grid_y, grid_x] = 75
                else:
                    self.occupancy_grid[grid_y, grid_x] = 50

        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "camera_link"
        grid_msg.info.resolution = self.grid_size
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.data = self.occupancy_grid.flatten().astype(int).tolist()

        self.pub.publish(grid_msg)
        if self.output_count % self.output_frequency == 0:
            self.get_logger().info("Published Occupancy Grid.")

        self.publish_camera_marker()

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
        self.marker_pub.publish(marker)

    def publish_camera_fov(self):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_fov"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        fov_horizontal = 69 * (np.pi / 180)
        fov_vertical = 42 * (np.pi / 180)
        max_depth = 2.0

        center = [0.0, 0.0, 0.0]
        top_left = [max_depth * np.tan(fov_horizontal / 2), max_depth * np.tan(fov_vertical / 2), max_depth]
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

        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.fov_pub.publish(marker)

    def filter_boulders_and_craters(self, points):
        min_depth = 0.1
        max_depth = 2.0
        z_values = points['z']
        return points[(z_values >= min_depth) & (z_values <= max_depth)]


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
