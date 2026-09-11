#!/usr/bin/env python3
"""
image_compressor_node.py - Republishes a raw camera image as a JPEG CompressedImage.
Used for spectator/monitoring feeds (e.g. visualization.aiformula_pilot) that don't
need the full-resolution raw stream perception nodes subscribe to.
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage

from oit_navigation.utils.image_util import imgmsg_to_cv2


class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__('image_compressor_node')

        self._declare_parameters()
        self._load_parameters()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(CompressedImage, self.output_topic, qos)
        self.create_subscription(Image, self.input_topic, self._image_callback, qos)

        self.get_logger().info(f"ImageCompressorNode running: {self.input_topic} -> {self.output_topic}")

    def _declare_parameters(self):
        self.declare_parameter('input_topic', '/aiformula_sensing/zed_node/left_image/undistorted')
        self.declare_parameter('output_topic', '/aiformula_visualization/zed/left_image/compressed')
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('resize_width', 0)
        self.declare_parameter('resize_height', 0)

    def _load_parameters(self):
        p = self.get_parameter
        self.input_topic = p('input_topic').value
        self.output_topic = p('output_topic').value
        self.jpeg_quality = int(p('jpeg_quality').value)
        self.resize_width = int(p('resize_width').value)
        self.resize_height = int(p('resize_height').value)

    def _image_callback(self, msg: Image):
        frame = imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is None:
            return

        if self.resize_width > 0 and self.resize_height > 0:
            frame = cv2.resize(frame, (self.resize_width, self.resize_height))

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressorNode()
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
