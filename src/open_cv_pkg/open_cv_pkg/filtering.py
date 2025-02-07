import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from transforms3d.quaternions import quat2mat
from std_msgs.msg import Header
import pcl  # Requires python-pcl


class PointCloudFilteringNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filtering_node')

        # Publishers
        self.transformed_pub = self.create_publisher(PointCloud2, '/transformed_points', 10)
        self.ground_pub = self.create_publisher(PointCloud2, '/ground_points', 10)
        self.non_ground_pub = self.create_publisher(PointCloud2, '/non_ground_points', 10)

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',  # Adjust this to your camera topic
            self.pointcloud_callback,
            10
        )

        # TF Buffer and Listener
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('PointCloud Filtering Node started.')

    def pointcloud_callback(self, msg):
        """Process and filter point cloud data."""
        # Transform points
        transformed_points = self.transform_pointcloud(msg)
        if len(transformed_points) == 0:
            self.get_logger().warn("No transformed points.")
            return

        # Crop and downsample
        cropped_points = self.crop_points(transformed_points, y_min=0.0, y_max=2.0)
        downsampled_points = self.downsample_pointcloud(cropped_points)

        # Segment ground and non-ground points
        ground_points, non_ground_points = self.segment_ground(downsampled_points)

        # Publish results
        self.publish_pointcloud(transformed_points, self.transformed_pub, "camera_link")
        if len(ground_points) > 0:
            self.publish_pointcloud(ground_points, self.ground_pub, "camera_link")
        if len(non_ground_points) > 0:
            self.publish_pointcloud(non_ground_points, self.non_ground_pub, "camera_link")

    def transform_pointcloud(self, msg):
        """Transform the point cloud to the target frame."""
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                "camera_link",  # Make sure this is correct in your TF tree
                msg.header.frame_id,
                rclpy.time.Time()
            )
            translation = np.array([
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z
            ])
            quaternion = [
                transform_stamped.transform.rotation.w,
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z
            ]
            rotation_matrix = quat2mat(quaternion)
        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return np.empty((0, 3), dtype=np.float32)

        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(points) == 0:
            return np.empty((0, 3), dtype=np.float32)

        # Convert structured array to a regular array with only x, y, z fields
        points_array = np.array([(p['x'], p['y'], p['z']) for p in points], dtype=np.float32)

        # Transform the points
        return (points_array @ rotation_matrix.T) + translation

    def crop_points(self, points, y_min, y_max):
        """Crop points to a region of interest."""
        cropped = points[(points[:, 1] >= y_min) & (points[:, 1] <= y_max)]
        self.get_logger().info(f"Cropped to {len(cropped)} points.")
        return cropped

    def downsample_pointcloud(self, points, leaf_size=0.03):
        """Downsample the point cloud using a voxel grid filter."""
        if len(points) == 0:
            return points
        
        # Convert NumPy array to PCL PointCloud
        cloud = pcl.PointCloud(points.astype(np.float32))

        # Apply voxel grid filter
        voxel_filter = cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(leaf_size, leaf_size, leaf_size)
        downsampled_cloud = voxel_filter.filter()

        # Convert back to NumPy before returning
        downsampled_array = np.asarray(downsampled_cloud.to_array())

        self.get_logger().info(f"Downsampled to {len(downsampled_array)} points.")
        return downsampled_array

    def segment_ground(self, points):
        """Segment the ground plane using RANSAC."""
        if len(points) == 0:
            return np.empty((0, 3)), np.empty((0, 3))
        cloud = pcl.PointCloud(points.astype(np.float32))

        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.05)  # Adjust for your sensor's precision

        indices, coefficients = seg.segment()
        if len(indices) == 0:
            self.get_logger().warn("No ground plane found.")
            return np.empty((0, 3)), points

        # Verify ground orientation
        normal = np.array(coefficients[:3])
        if abs(normal[1]) < 0.9:  # Check if normal aligns with Y-axis
            self.get_logger().warn(f"Detected plane normal {normal} is not horizontal.")
            return np.empty((0, 3)), points

        ground_points = points[indices]
        non_ground_points = np.delete(points, indices, axis=0)

        self.get_logger().info(f"Segmented {len(ground_points)} ground points and {len(non_ground_points)} non-ground points.")
        return ground_points, non_ground_points

    def publish_pointcloud(self, points, publisher, frame_id):
        """Publish a point cloud as a PointCloud2 message."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        msg = pc2.create_cloud_xyz32(header, points.tolist())
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilteringNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
