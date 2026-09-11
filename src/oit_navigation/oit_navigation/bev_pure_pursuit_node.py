#!/usr/bin/env python3
"""
bev_pure_pursuit_node.py - 2D Vision-Only BEV Lane Tracking & Pure Pursuit Control Node
Performs Inverse Perspective Mapping (IPM), sliding window lane tracking,
clean-side prioritized 3.5m offset target line generation, and 2-Point Anchor Pure Pursuit steering control.
"""

import math
import time
from typing import Optional, List, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from oit_navigation.utils.bev_transformer import BEVTransformer
from oit_navigation.utils.bev_lane_extractor import BEVLaneExtractor
from oit_navigation.utils.pure_pursuit import AdaptivePurePursuit
from oit_navigation.utils.image_util import cv2_to_imgmsg, imgmsg_to_cv2


class BEVPurePursuitNode(Node):
    """Direct 2D Vision-Only BEV 2-Point Anchor & Pure Pursuit Controller Node."""

    def __init__(self):
        super().__init__('bev_pure_pursuit_node')

        self._declare_parameters()
        self._load_parameters()

        # Initialize algorithmic components
        self.bev_transformer = BEVTransformer(
            cam_height=self.cam_height,
            dist_min=self.dist_min,
            dist_max=self.dist_max,
            lateral_max=self.lateral_max,
            bev_w=640,
            bev_h=640,
        )

        self.lane_extractor = BEVLaneExtractor(
            bev_w=640,
            bev_h=640,
            lane_width_m=self.lane_width,
            dist_min=self.dist_min,
            dist_max=self.dist_max,
            lateral_max=self.lateral_max,
            n_windows=14,
            margin_px=45,
            min_recenter_pixels=15,
            ema_alpha=0.60,
        )

        self.pure_pursuit = AdaptivePurePursuit(
            min_lookahead=1.5,
            max_lookahead=self.lookahead_distance,
            lookahead_ratio=0.5,
            angular_gain=self.angular_gain,
            max_angular_vel=self.max_angular_speed,
        )

        # Buffers & States
        self.latest_mask_img: Optional[np.ndarray] = None
        self.last_mask_time: float = 0.0
        self.last_control_time = time.time()
        self.current_omega: float = 0.0

        # QoS profile with depth=1 to prevent message accumulation
        qos_img = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # ROS Publishers (ai_formula standard naming)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # Visualizations for RViz2
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.target_markers_pub = self.create_publisher(MarkerArray, '/aiformula_visualization/lane_target_markers', 10)
        self.lane_markers_pub = self.create_publisher(MarkerArray, '/aiformula_visualization/detected_lane_lines', 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.bev_img_pub = self.create_publisher(Image, '/aiformula_visualization/bev_annotated_image', qos_img)

        # ROS Subscriber (Direct YOLOP mask image)
        self.create_subscription(Image, self.mask_image_topic, self._mask_cb, qos_img)

        # Control Loop Timer (20Hz)
        self.timer = self.create_timer(self.control_period, self._control_loop)

        self.get_logger().info(
            f'AI Formula OIT 2027 2-Point Anchor Controller Initialized: '
            f'Origin Anchor + 5.0m Ahead Anchor, Speed={self.target_speed}m/s'
        )

    def _declare_parameters(self):
        # Topics
        self.declare_parameter('mask_image_topic', '/aiformula_perception/object_road_detector/mask_image')
        self.declare_parameter('cmd_vel_topic', '/aiformula_control/handle_controller/cmd_vel')
        self.declare_parameter('status_topic', '/aiformula_control/lane_tracker/status')

        self.declare_parameter('marker_topic', '/aiformula_visualization/lane_target_marker')
        self.declare_parameter('path_topic', '/aiformula_visualization/target_trajectory')

        # Frame IDs
        self.declare_parameter('robot_frame_id', 'base_link')

        # Camera & BEV Geometry
        self.declare_parameter('camera_height', 0.56)
        self.declare_parameter('dist_min', 1.15)
        self.declare_parameter('dist_max', 10.0)
        self.declare_parameter('lateral_max', 5.0)
        self.declare_parameter('lane_width', 3.5)

        # Control Settings
        self.declare_parameter('lookahead_distance', 5.0)
        self.declare_parameter('target_linear_speed', 1.0)
        self.declare_parameter('angular_gain', 0.85)
        self.declare_parameter('cross_track_gain', 0.95)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_angular_accel', 4.0)

        # Timing
        self.declare_parameter('data_timeout', 0.8)
        self.declare_parameter('control_rate_hz', 20.0)

    def _load_parameters(self):
        p = self.get_parameter
        self.mask_image_topic = p('mask_image_topic').value
        self.cmd_vel_topic = p('cmd_vel_topic').value
        self.status_topic = p('status_topic').value
        self.marker_topic = p('marker_topic').value
        self.path_topic = p('path_topic').value

        self.robot_frame_id = p('robot_frame_id').value

        self.cam_height = float(p('camera_height').value)
        self.dist_min = float(p('dist_min').value)
        self.dist_max = float(p('dist_max').value)
        self.lateral_max = float(p('lateral_max').value)
        self.lane_width = float(p('lane_width').value)

        self.lookahead_distance = float(p('lookahead_distance').value)
        self.target_speed = float(p('target_linear_speed').value)
        self.angular_gain = float(p('angular_gain').value)
        self.cross_track_gain = float(p('cross_track_gain').value)
        self.max_angular_speed = float(p('max_angular_speed').value)
        self.max_angular_accel = float(p('max_angular_accel').value)

        self.data_timeout = float(p('data_timeout').value)
        rate = float(p('control_rate_hz').value)
        self.control_period = 1.0 / max(1.0, rate)

    def _mask_cb(self, msg: Image):
        try:
            cv_img = imgmsg_to_cv2(msg, desired_encoding='mono8')
            if cv_img is not None and cv_img.ndim == 3:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            self.latest_mask_img = cv_img
            self.last_mask_time = time.time()
        except Exception as e:
            self.get_logger().warning(f"Error decoding mask_cb image: {str(e)}")

    def _control_loop(self):
        now = time.time()
        dt = now - self.last_control_time
        self.last_control_time = now

        if self.latest_mask_img is None or (now - self.last_mask_time) > self.data_timeout:
            self._handle_fallback(dt)
            return

        # 1. Warp Camera Mask directly to 2D BEV
        bev_mask = self.bev_transformer.warp_to_bev(self.latest_mask_img, is_binary=True)

        # 2. Extract Left/Right lanes and compute Clean-Anchored 3.5m Target Line
        target_center, left_pts, right_pts, annotated_bev = self.lane_extractor.extract_lane_trajectories(bev_mask)

        # Publish BEV visualization image
        self._publish_bev_image(annotated_bev)

        # Publish 3D Lane Line Strips to RViz
        self._publish_detected_lane_markers(left_pts, right_pts)

        if target_center is None or len(target_center) < 2:
            self._handle_fallback(dt)
            return

        xs = target_center[:, 0]
        ys = target_center[:, 1]

        # 3. Target Lookahead Point at 5.0m Ahead
        far_idx = int(np.argmin(np.abs(xs - self.lookahead_distance)))
        x_f, y_f = float(xs[far_idx]), float(ys[far_idx])
        far_pt = (x_f, y_f)

        # Slope at lookahead point
        if 0 < far_idx < len(xs) - 1:
            slope = (ys[far_idx + 1] - ys[far_idx - 1]) / max(0.1, xs[far_idx + 1] - xs[far_idx - 1])
        else:
            slope = y_f / max(1.0, x_f)

        # 4. Generate Origin-Anchored Target Trajectory (Hermite Spline)
        origin_pt = (0.0, 0.0)
        cross_track_error = float(ys[0])  # Road center offset relative to vehicle

        traj_x = np.linspace(0.0, float(xs[-1]), num=50)
        traj_y = np.zeros_like(traj_x)

        denom = max(1.0, x_f**2)
        a2 = (3.0 * y_f - x_f * slope) / denom
        a3 = (x_f * slope - 2.0 * y_f) / (denom * x_f)

        for i, x_val in enumerate(traj_x):
            if x_val <= x_f:
                traj_y[i] = a3 * (x_val**3) + a2 * (x_val**2)
            else:
                interp_idx = int(np.argmin(np.abs(xs - x_val)))
                traj_y[i] = ys[interp_idx]

        origin_anchored_trajectory = np.column_stack((traj_x, traj_y))

        # Publish Calculated Target Path (Green Line in RViz)
        self._publish_trajectory_path(origin_anchored_trajectory)

        # 5. Pure Pursuit & 2-Point Anchor Steering Calculation:
        l_sq = x_f**2 + y_f**2
        preview_curvature = (2.0 * y_f / l_sq) if l_sq > 1e-4 else 0.0
        omega_preview = self.angular_gain * self.target_speed * preview_curvature

        omega_crosstrack = self.cross_track_gain * math.atan2(cross_track_error, max(0.5, self.target_speed))

        # Combined Steering Command
        target_omega = omega_preview + omega_crosstrack
        target_omega = float(np.clip(target_omega, -self.max_angular_speed, self.max_angular_speed))

        # Apply Angular Acceleration Slew Rate Limiting
        max_delta_w = self.max_angular_accel * dt
        delta_w = np.clip(target_omega - self.current_omega, -max_delta_w, max_delta_w)
        self.current_omega += delta_w

        # Publish Twist Command
        twist = Twist()
        twist.linear.x = self.target_speed
        twist.angular.z = self.current_omega
        self.cmd_pub.publish(twist)

        # Publish Anchor Spheres to RViz & Status
        mode_str = "Both Lanes" if (left_pts is not None and right_pts is not None) else ("Left Anchor" if left_pts is not None else "Right Anchor")
        self._publish_anchor_spheres(origin_pt, far_pt)
        self._publish_status(self.target_speed, self.current_omega, origin_pt, far_pt, mode_str)

    def _handle_fallback(self, dt: float):
        max_delta_w = self.max_angular_accel * dt
        self.current_omega = float(np.clip(0.0 - self.current_omega, -max_delta_w, max_delta_w))
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.current_omega
        self.cmd_pub.publish(twist)

    def _publish_detected_lane_markers(self, left_pts: Optional[np.ndarray], right_pts: Optional[np.ndarray]):
        marker_arr = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        # 1. Left Lane (Cyan LineStrip)
        m_left = Marker()
        m_left.header.frame_id = self.robot_frame_id
        m_left.header.stamp = now_msg
        m_left.ns = "detected_lanes"
        m_left.id = 1
        m_left.type = Marker.LINE_STRIP
        m_left.action = Marker.ADD if (left_pts is not None and len(left_pts) > 0) else Marker.DELETE
        m_left.scale.x = 0.12
        m_left.color.a = 0.95
        m_left.color.r = 0.0
        m_left.color.g = 0.85
        m_left.color.b = 1.0  # Cyan
        if left_pts is not None:
            for pt in left_pts:
                p = Point()
                p.x, p.y, p.z = float(pt[0]), float(pt[1]), 0.02
                m_left.points.append(p)
        marker_arr.markers.append(m_left)

        # 2. Right Lane (Yellow LineStrip)
        m_right = Marker()
        m_right.header.frame_id = self.robot_frame_id
        m_right.header.stamp = now_msg
        m_right.ns = "detected_lanes"
        m_right.id = 2
        m_right.type = Marker.LINE_STRIP
        m_right.action = Marker.ADD if (right_pts is not None and len(right_pts) > 0) else Marker.DELETE
        m_right.scale.x = 0.12
        m_right.color.a = 0.95
        m_right.color.r = 1.0
        m_right.color.g = 0.9
        m_right.color.b = 0.0  # Yellow
        if right_pts is not None:
            for pt in right_pts:
                p = Point()
                p.x, p.y, p.z = float(pt[0]), float(pt[1]), 0.02
                m_right.points.append(p)
        marker_arr.markers.append(m_right)

        self.lane_markers_pub.publish(marker_arr)

    def _publish_anchor_spheres(self, origin_pt: Tuple[float, float], far_pt: Tuple[float, float]):
        now_msg = self.get_clock().now().to_msg()
        marker_arr = MarkerArray()

        # 1. Forward 5.0m Preview Anchor Sphere (Bright Green)
        m_far = Marker()
        m_far.header.frame_id = self.robot_frame_id
        m_far.header.stamp = now_msg
        m_far.ns = 'anchor_spheres'
        m_far.id = 0
        m_far.type = Marker.SPHERE
        m_far.action = Marker.ADD
        m_far.pose.position.x = float(far_pt[0])
        m_far.pose.position.y = float(far_pt[1])
        m_far.pose.position.z = 0.15
        m_far.scale.x = 0.50
        m_far.scale.y = 0.50
        m_far.scale.z = 0.50
        m_far.color.a = 0.95
        m_far.color.r = 0.0
        m_far.color.g = 1.0
        m_far.color.b = 0.2  # Green
        marker_arr.markers.append(m_far)

        # 2. Vehicle Origin Anchor Sphere (Cyan)
        m_origin = Marker()
        m_origin.header.frame_id = self.robot_frame_id
        m_origin.header.stamp = now_msg
        m_origin.ns = 'anchor_spheres'
        m_origin.id = 1
        m_origin.type = Marker.SPHERE
        m_origin.action = Marker.ADD
        m_origin.pose.position.x = 0.0
        m_origin.pose.position.y = 0.0
        m_origin.pose.position.z = 0.15
        m_origin.scale.x = 0.40
        m_origin.scale.y = 0.40
        m_origin.scale.z = 0.40
        m_origin.color.a = 0.95
        m_origin.color.r = 0.0
        m_origin.color.g = 0.9
        m_origin.color.b = 1.0  # Cyan
        marker_arr.markers.append(m_origin)

        self.marker_pub.publish(m_far)
        self.target_markers_pub.publish(marker_arr)

    def _publish_trajectory_path(self, trajectory: np.ndarray):
        path_msg = Path()
        path_msg.header.frame_id = self.robot_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for pt in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def _publish_bev_image(self, annotated_bev: np.ndarray):
        try:
            img_msg = cv2_to_imgmsg(
                annotated_bev,
                encoding='bgr8',
                frame_id=self.robot_frame_id,
                stamp=self.get_clock().now().to_msg()
            )
            self.bev_img_pub.publish(img_msg)
        except Exception:
            pass

    def _publish_status(self, v_cmd: float, w_cmd: float, origin_pt: Tuple[float, float], far_pt: Tuple[float, float], mode: str):
        msg = String()
        msg.data = (
            f"[oit_navigation] Mode={mode} | Speed={v_cmd:.2f}m/s | Omega={w_cmd:+.2f}rad/s "
            f"| Origin_ey={origin_pt[1]:+.2f}m | Far_5m=({far_pt[0]:.2f}, {far_pt[1]:+.2f}m)"
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BEVPurePursuitNode()
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
