#!/usr/bin/env python3
"""
video_publisher.py - Sequential Frame-by-Frame Real-Time MP4 Video Stream Publisher
Reads and publishes video frames strictly sequentially without arbitrary seeking/jumping.
Supports both raw sensor_msgs/msg/Image and JPEG sensor_msgs/msg/CompressedImage topics.
"""

import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage

from oit_navigation.utils.image_util import cv2_to_imgmsg
from common_python.workspace_paths import default_workspace_asset, resolve_workspace_asset


class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher')

        self.declare_parameter(
            'video_path',
            default_workspace_asset('mp4', 'shihou_video_2026_08_24_13_51_47.mp4'),
        )
        self.declare_parameter('topic_name', '/aiformula_sensing/zed_node/left_image/undistorted')
        self.declare_parameter('frame_id', 'zed_left_camera_optical_frame')
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('resize_width', 0)
        self.declare_parameter('resize_height', 0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('publish_raw', False)

        self.video_path = str(self.get_parameter('video_path').value)
        self.topic_name = str(self.get_parameter('topic_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.target_fps = max(1.0, float(self.get_parameter('fps').value))
        self.loop = bool(self.get_parameter('loop').value)
        self.resize_width = int(self.get_parameter('resize_width').value)
        self.resize_height = int(self.get_parameter('resize_height').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.publish_raw = bool(self.get_parameter('publish_raw').value)

        # QoS: RELIABLE with depth 1
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.compressed_pub = self.create_publisher(CompressedImage, self.topic_name + '/compressed', qos)
        self.image_pub = self.create_publisher(Image, self.topic_name, qos) if self.publish_raw else None

        self._open_video()

        self.frame_count = 0
        timer_period = 1.0 / self.target_fps
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f"VideoPublisherNode running: {self.video_path} -> {self.topic_name} at {self.target_fps:.1f} FPS (Sequential Playback)"
        )

    def _open_video(self):
        resolved = resolve_workspace_asset(self.video_path)
        if not os.path.exists(resolved):
            self.get_logger().error(f"Video file not found: {self.video_path}")
            self.cap = None
            return
        self.video_path = resolved

        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {self.video_path}")
            self.cap = None
        else:
            self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.video_fps = self.cap.get(cv2.CAP_PROP_FPS) or self.target_fps
            self.get_logger().info(f"Opened video ({self.total_frames} frames, source FPS: {self.video_fps:.1f})")

    def _timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        # Read the next frame strictly sequentially
        ret, frame = self.cap.read()
        if not ret:
            if self.loop:
                self.get_logger().info("Video reached end. Looping to start...")
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
            else:
                self.get_logger().info("Video playback completed.")
                self.timer.cancel()
                return

        if not ret or frame is None:
            return

        if self.resize_width > 0 and self.resize_height > 0:
            frame = cv2.resize(frame, (self.resize_width, self.resize_height))

        try:
            now_msg = self.get_clock().now().to_msg()

            # 1. Publish JPEG Compressed Image (Lightweight: ~4ms)
            ret_enc, enc_buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ret_enc:
                comp_msg = CompressedImage()
                comp_msg.header.stamp = now_msg
                comp_msg.header.frame_id = self.frame_id
                comp_msg.format = 'jpeg'
                comp_msg.data = enc_buf.tobytes()
                self.compressed_pub.publish(comp_msg)

            # 2. Publish Raw Image only if requested (Heavy: ~300ms)
            if self.publish_raw and self.image_pub is not None:
                msg = cv2_to_imgmsg(
                    frame,
                    encoding='bgr8',
                    frame_id=self.frame_id,
                    stamp=now_msg
                )
                self.image_pub.publish(msg)

            self.frame_count += 1
            if self.frame_count % 150 == 0:
                self.get_logger().info(f"Published {self.frame_count} frames to {self.topic_name}/compressed")

        except Exception as e:
            self.get_logger().warning(f"Error publishing frame: {str(e)}")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
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
