#!/usr/bin/env python3
"""
object_publisher_node.py - Converts detected 2D bounding boxes (RectMultiArray) into
world-frame ObjectInfo, reusing the same ground-plane Inverse Perspective Mapping (IPM)
already used for BEV lane tracking (see utils/bev_transformer.py).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from aiformula_interfaces.msg import RectMultiArray, ObjectInfo, ObjectInfoMultiArray

from oit_navigation.utils.bev_transformer import BEVTransformer


class ObjectPublisherNode(Node):
    """Turns 2D image-space object detections into base_link-frame ObjectInfo."""

    def __init__(self):
        super().__init__('object_publisher_node')

        self._declare_parameters()
        self._load_parameters()

        self.bev_transformer = BEVTransformer(
            src_w=self.image_width,
            src_h=self.image_height,
            cam_height=self.camera_height,
            dist_min=self.dist_min,
            dist_max=self.dist_max,
            lateral_max=self.lateral_max,
        )

        self.info_pub = self.create_publisher(ObjectInfoMultiArray, self.object_info_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(RectMultiArray, self.bounding_box_topic, self._rect_callback, 10)

        self.get_logger().info(
            f"ObjectPublisherNode running: {self.bounding_box_topic} -> {self.object_info_topic}"
        )

    def _declare_parameters(self):
        self.declare_parameter('bounding_box_topic', '/aiformula_perception/object_road_detector/rect')
        self.declare_parameter('object_info_topic', '/aiformula_perception/object_publisher/object_info')
        self.declare_parameter('marker_topic', '/aiformula_visualization/object_publisher/unfiltered_object')
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('camera_height', 0.56)
        self.declare_parameter('dist_min', 1.15)
        self.declare_parameter('dist_max', 10.0)
        self.declare_parameter('lateral_max', 5.0)
        # Rect carries no detection score; every box already passed the detector's own
        # confidence threshold, so a constant is the honest value to forward here.
        self.declare_parameter('detection_confidence', 1.0)

    def _load_parameters(self):
        p = self.get_parameter
        self.bounding_box_topic = p('bounding_box_topic').value
        self.object_info_topic = p('object_info_topic').value
        self.marker_topic = p('marker_topic').value
        self.robot_frame_id = p('robot_frame_id').value
        self.image_width = int(p('image_width').value)
        self.image_height = int(p('image_height').value)
        self.camera_height = float(p('camera_height').value)
        self.dist_min = float(p('dist_min').value)
        self.dist_max = float(p('dist_max').value)
        self.lateral_max = float(p('lateral_max').value)
        self.detection_confidence = float(p('detection_confidence').value)

    def _rect_callback(self, msg: RectMultiArray):
        info_array = ObjectInfoMultiArray()
        info_array.header = msg.header
        marker_array = MarkerArray()

        for idx, rect in enumerate(msg.rects):
            bottom_x = rect.x + rect.width / 2.0
            bottom_y = rect.y + rect.height
            x, y = self.bev_transformer.image_px_to_robot_xy(bottom_x, bottom_y)

            if not (self.dist_min <= x <= self.dist_max and abs(y) <= self.lateral_max):
                continue

            left_x, left_y = self.bev_transformer.image_px_to_robot_xy(rect.x, bottom_y)
            right_x, right_y = self.bev_transformer.image_px_to_robot_xy(rect.x + rect.width, bottom_y)
            width_m = float(np.hypot(right_x - left_x, right_y - left_y))

            info = ObjectInfo()
            info.x = x
            info.y = y
            info.width = width_m
            info.id = 0
            info.confidence = self.detection_confidence
            info_array.objects.append(info)

            marker_array.markers.append(self._make_marker(idx, x, y, width_m, msg.header.stamp))

        self.info_pub.publish(info_array)
        self.marker_pub.publish(marker_array)

    def _make_marker(self, idx: int, x: float, y: float, width_m: float, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.robot_frame_id
        marker.header.stamp = stamp
        marker.ns = 'unfiltered_object'
        marker.id = idx
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.2, width_m)
        marker.scale.y = max(0.2, width_m)
        marker.scale.z = 0.5
        marker.color.a = 0.7
        marker.color.r = 1.0
        marker.color.g = 0.3
        marker.color.b = 0.0
        marker.lifetime.nanosec = 200_000_000
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
