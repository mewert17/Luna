import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from transforms3d.quaternions import quat2mat
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


class PointCloudFilteringNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filtering_node')

        # Publishers for transformed and filtered point clouds
        self.transformed_pub = self.create_publisher(PointCloud2, '/transformed_points', 10)
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_points', 10)

        # Publisher for threshold visualization marker
        self.threshold_marker_pub = self.create_publisher(Marker, '/threshold_marker', 10)

        # Subscribe to the raw point cloud topic
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',  # Adjust this topic to your setup
            self.pointcloud_callback,
            10
        )

        # TF Buffer and Listener
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Frame counter to slow down terminal output
        self.frame_counter = 0
        self.output_frequency = 5  # Log every 5 frames

        self.get_logger().info('PointCloud Filtering Node started.')

        # Initial thresholds
        self.x_min, self.x_max = -5.0, 5.0
        self.y_min, self.y_max = 0.5, 2.0
        self.z_min, self.z_max = 0.0, 10.0

        # Publish the initial threshold visualization
        self.publish_threshold_marker()

    def pointcloud_callback(self, msg):
        """Process and filter point cloud data."""
        try:
            # Retrieve the transform from the camera frame to the target frame
            transform_stamped = self.tf_buffer.lookup_transform(
                "camera_link",  # Replace with the correct target frame
                msg.header.frame_id,  # Source frame (original point cloud frame)
                rclpy.time.Time()  # Use the latest available transform
            )

            # Extract translation and rotation
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
            rotation_matrix = quat2mat(quaternion)  # Convert quaternion to rotation matrix

        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return

        # Extract raw point cloud data
        point_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(point_list) == 0:
            self.get_logger().warn("Received empty point cloud.")
            return

        # Convert structured point cloud to plain array
        points_array = np.array([(p[0], p[1], p[2]) for p in point_list], dtype=np.float32)

        # Transform the points
        transformed_points = (points_array @ rotation_matrix.T) + translation

        # Publish the transformed point cloud
        self.publish_pointcloud(transformed_points, self.transformed_pub, "camera_link")

        # Filter the points within bounds
        filtered_points = self.filter_points(transformed_points)

        # Publish the filtered point cloud
        self.publish_pointcloud(filtered_points, self.filtered_pub, "camera_link")

        # Publish threshold marker
        self.publish_threshold_marker()

        # Slow down terminal output
        self.frame_counter += 1
        if self.frame_counter % self.output_frequency == 0:
            self.get_logger().info(f"Transformed {len(transformed_points)} points, filtered to {len(filtered_points)} points.")

    def filter_points(self, points):
        """
        Filter points within specific bounds.
        """
        filtered = points[
            (points[:, 0] >= self.x_min) & (points[:, 0] <= self.x_max) &
            (points[:, 1] >= self.y_min) & (points[:, 1] <= self.y_max) &
            (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        ]
        return filtered

    def publish_pointcloud(self, points, publisher, frame_id):
        """Publish a point cloud as a PointCloud2 message."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        # Convert points to PointCloud2 message
        point_cloud_msg = pc2.create_cloud_xyz32(header, points.tolist())
        publisher.publish(point_cloud_msg)

    def publish_threshold_marker(self):
        """Publish a marker to visualize the filtering thresholds."""
        # Publish the marker only when frame counter hits the frequency
        self.frame_counter += 1
        if self.frame_counter % self.output_frequency != 0:
            return

        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "threshold_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set position and dimensions of the marker based on thresholds
        marker.pose.position.x = (self.x_min + self.x_max) / 2.0
        marker.pose.position.y = (self.y_min + self.y_max) / 2.0
        marker.pose.position.z = (self.z_min + self.z_max) / 2.0

        marker.scale.x = self.x_max - self.x_min
        marker.scale.y = self.y_max - self.y_min
        marker.scale.z = self.z_max - self.z_min

        # Set color and transparency
        marker.color.a = 0.5  # Transparency
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.threshold_marker_pub.publish(marker)


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
