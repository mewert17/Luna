from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def publish_debug_markers(node, points, frame_id="camera_link"):
    """Publish debug markers for filtered points."""
    marker_array = MarkerArray()
    for i, (x, y, z) in enumerate(points):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = node.get_clock().now().to_msg()
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
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)

    # Publish the marker array
    node.debug_marker_pub.publish(marker_array)


def publish_camera_marker(node, frame_id="camera_link"):
    """Publish a marker for the camera position."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = node.get_clock().now().to_msg()
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
    marker.color.g = 1.0  # Green
    node.camera_marker_pub.publish(marker)


def publish_camera_fov(node, frame_id="camera_link", fov_horizontal=69, fov_vertical=42, max_depth=2.0):
    """Publish the camera FoV marker."""
    import numpy as np

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = "camera_fov"
    marker.id = 1
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    # Calculate the four corners of the camera FoV
    fov_horizontal = np.radians(fov_horizontal)
    fov_vertical = np.radians(fov_vertical)

    center = [0.0, 0.0, 0.0]
    top_left = [max_depth * np.tan(fov_vertical / 2), max_depth * np.tan(fov_horizontal / 2), max_depth]
    top_right = [-top_left[0], top_left[1], max_depth]
    bottom_left = [top_left[0], -top_left[1], max_depth]
    bottom_right = [-top_left[0], -top_left[1], max_depth]

    # Connect the points to form a wireframe FoV
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
    marker.color.r = 1.0  # Red
    marker.color.g = 0.0
    marker.color.b = 0.0
    node.fov_pub.publish(marker)
