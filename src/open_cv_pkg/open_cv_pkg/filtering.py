import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from transforms3d.quaternions import quat2mat
from std_msgs.msg import Header
import pcl  # Requires python-pcl
import time  # For delays

class PointCloudFilteringNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filtering_node')

        # Publishers
        self.downsampled_pub = self.create_publisher(PointCloud2, '/downsampled_points', 10)
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

        # Rate limiting parameters
        self.last_log_time = time.time()
        self.log_interval = 1.0  # Minimum time between logs (in seconds)

        self.get_logger().info('PointCloud Filtering Node started.')

    def should_log(self):
        """Check if enough time has passed to log a message."""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            return True
        return False

    def pointcloud_callback(self, msg):
        """Process and filter point cloud data."""
        # Read and downsample the point cloud
        raw_points = self.read_pointcloud(msg)
        downsampled_points = self.downsample_pointcloud(raw_points, leaf_size=0.02)

        # Transform the downsampled point cloud
        transformed_points = self.transform_pointcloud(downsampled_points, msg.header.frame_id)

        # Segment ground and non-ground points
        ground_points, non_ground_points = self.segment_ground(transformed_points)

        # Publish results
        self.publish_pointcloud(downsampled_points, self.downsampled_pub, "camera_link")
        self.publish_pointcloud(transformed_points, self.transformed_pub, "camera_link")
        if len(ground_points) > 0:
            self.publish_pointcloud(ground_points, self.ground_pub, "camera_link")
        if len(non_ground_points) > 0:
            self.publish_pointcloud(non_ground_points, self.non_ground_pub, "camera_link")

    def read_pointcloud(self, msg):
        """Convert the raw PointCloud2 message into a NumPy array."""
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        return np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)

    def downsample_pointcloud(self, points, leaf_size=0.02):
        """Downsample the point cloud using a voxel grid filter."""
        if len(points) == 0:
            return points
        
        cloud = pcl.PointCloud(points.astype(np.float32))
        voxel_filter = cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(leaf_size, leaf_size, leaf_size)
        downsampled_cloud = voxel_filter.filter()

        downsampled_array = np.asarray(downsampled_cloud)
        if self.should_log():
            self.get_logger().info(f"Downsampled to {len(downsampled_array)} points (leaf size: {leaf_size}).")
        return downsampled_array

    def transform_pointcloud(self, points, source_frame):
        """Transform the point cloud to the target frame."""
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                "camera_link",  # Ensure this frame exists in your TF tree
                source_frame,
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
            if self.should_log():
                self.get_logger().error(f"Transform lookup failed: {e}")
            return np.empty((0, 3), dtype=np.float32)

        return (points @ rotation_matrix.T) + translation

    def segment_ground(self, points):
        """Segment the ground plane using RANSAC with alignment constraints for any axis."""
        if len(points) == 0:
            return np.empty((0, 3)), np.empty((0, 3))

        cloud = pcl.PointCloud(points.astype(np.float32))
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.03)  # Adjust for sensor noise

        max_attempts = 5

        for attempt in range(max_attempts):
            indices, coefficients = seg.segment()
            if len(indices) == 0:
                if self.should_log():
                    self.get_logger().warn("No ground plane found.")
                return np.empty((0, 3)), points

            # Extract normal vector of the detected plane
            normal = np.array(coefficients[:3])
            alignment = np.linalg.norm(normal)  # General alignment for any axis

            if self.should_log():
                self.get_logger().info(
                    f"Detected plane normal: {normal}, alignment magnitude: {alignment:.3f}"
                )

            if alignment > 0.9:  # Strong alignment with any axis
                ground_points = points[indices]
                non_ground_points = np.delete(points, indices, axis=0)
                if self.should_log():
                    self.get_logger().info(f"Segmented {len(ground_points)} ground points and {len(non_ground_points)} non-ground points.")
                return ground_points, non_ground_points

        if self.should_log():
            self.get_logger().warn("Could not find a valid ground plane after multiple attempts.")
        return np.empty((0, 3)), points

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
