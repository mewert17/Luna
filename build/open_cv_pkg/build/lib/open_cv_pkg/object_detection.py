import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import pcl

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()

        # Publisher to publish detected boulders (PointCloud2)
        self.pub = self.create_publisher(PointCloud2, '/detected_boulders', 10)

        # Subscribe to point cloud topics
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/points',  # Point cloud from the depth camera
            self.pointcloud_callback,
            10
        )

        self.get_logger().info('Object Detection Node has been started.')

    def pointcloud_callback(self, msg):
        """
        Callback function to process the point cloud data for crater and boulder detection.
        """
        # Extract and process point cloud data
        point_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points_array = np.array(point_list)

        # Debugging: Log point cloud data size
        self.get_logger().info(f"Point cloud data received with {len(points_array)} points.")

        # Log minimum and maximum depth values for debugging
        if len(points_array) > 0:
            min_depth = np.min(points_array[:, 2])
            max_depth = np.max(points_array[:, 2])
            self.get_logger().info(f"Depth Range: min_depth = {min_depth}, max_depth = {max_depth}")
        else:
            self.get_logger().warn("Point cloud data is empty, skipping processing.")
            return

        # Filter noise in point cloud
        filtered_cloud = self.filter_noise(points_array)
        self.get_logger().info(f"Filtered cloud has {len(filtered_cloud)} points.")

        # Detect craters based on depth thresholding
        self.detect_crater(filtered_cloud)

        # Detect boulders based on depth thresholding and size
        self.detect_boulder_by_size(filtered_cloud)

    def detect_crater(self, point_cloud_data):
        """
        Detect craters in the point cloud based on depth thresholds (lower depth values).
        """
        if len(point_cloud_data) == 0:
            self.get_logger().warn("No points to detect craters.")
            return

        min_depth = np.min(point_cloud_data[:, 2])
        crater_threshold = min_depth - 0.05  # Depth threshold for crater detection (e.g., 5 cm below minimum)

        # Find points below the crater threshold
        crater_points = point_cloud_data[point_cloud_data[:, 2] < crater_threshold]
        self.get_logger().info(f"Detected {len(crater_points)} crater points based on depth threshold.")

        if len(crater_points) == 0:
            self.get_logger().info("No craters detected in the specified depth range.")

    def detect_boulder_by_size(self, point_cloud_data):
        """
        Detect boulders in the point cloud based on depth thresholds and size (e.g., detecting objects within a certain depth range).
        """
        if len(point_cloud_data) == 0:
            self.get_logger().warn("No points to detect boulders.")
            return

        # Set a range of depths to detect boulders (e.g., between 30 cm and 1 meter)
        depth_min = 0.1  # Minimum depth (e.g., 10 cm)
        depth_max = 10.0  # Maximum depth (e.g., 1 meter)

        # Filter points within the depth range
        detected_points = point_cloud_data[
            (point_cloud_data[:, 2] >= depth_min) & (point_cloud_data[:, 2] <= depth_max)
        ]

        self.get_logger().info(f"Detected {len(detected_points)} points within the specified depth range.")

        # Optionally, apply size threshold for detecting objects like boulders
        size_threshold = 0.05  # Minimum size for an object to be considered a boulder (e.g., 10 cm)
        if len(detected_points) > size_threshold * 1000:  # Assuming each object is at least 10 cm large
            self.get_logger().info("Boulder detected based on size and depth threshold.")
            self.visualize_boulders(detected_points)
        else:
            self.get_logger().info("No boulders detected in the specified depth range.")

    def visualize_boulders(self, detected_points):
        """
        Visualize the detected boulders by publishing them as a new PointCloud2 message.
        """
        if len(detected_points) == 0:
            self.get_logger().warn("No boulders to visualize.")
            return

        # Create a PointCloud2 message to publish the detected points
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"

        # Convert the numpy array to PointCloud2 message format
        point_cloud = pc2.create_cloud_xyz32(header, detected_points)

        # Publish the point cloud
        self.pub.publish(point_cloud)
        self.get_logger().info(f"Visualizing {len(detected_points)} boulder points in RViz.")

    def filter_noise(self, point_cloud_data):
        """
        Function to filter out noisy data from the point cloud using Statistical Outlier Removal (SOR).
        """
        cloud = pcl.PointCloud()
        cloud.from_array(point_cloud_data.astype(np.float32))

        sor = cloud.make_statistical_outlier_filter()
        sor.set_mean_k(50)
        sor.set_std_dev_mul_thresh(1.0)

        filtered_cloud = sor.filter()
        return filtered_cloud


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    node = ObjectDetectionNode()  # Create the object detection node
    
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Gracefully shut down the node
        rclpy.shutdown()  # Shutdown ROS2
