#!/usr/bin/env python3
"""
cone_detector_node.py - コーン検出 (models/cone.pt, Ultralytics YOLO) + 地面投影による位置推定ノード.

web_simulator/js/cone_detector.js と同じ処理: コーンは接地物なので, バウンディングボックス下辺の中央を
白線検出 (lane_detector) と同じカメラモデルで地面 z=0 に投影すれば base_link での位置 (= 距離) が直接求まる.
有効範囲 (0.3m < x < 8.0m, |y| < 3.0m) の外は捨てる.

    入力: カメラ画像 (Image / CompressedImage. トピック名が compressed を含めば CompressedImage)
    出力: /aiformula_perception/cone_detector/cones             geometry_msgs/PoseArray (base_link)
                                                                 -> six_lane_planner の cones_topic (回避に使う)
          /aiformula_perception/cone_detector/status            std_msgs/String (JSON: 各コーンの x, y, 距離, 信頼度, bbox)
          /aiformula_visualization/cone_detector/markers        visualization_msgs/MarkerArray (円柱 + 距離)
          /aiformula_visualization/cone_detector/annotated_image sensor_msgs/Image (枠 + 距離)

camera_* パラメータは navigation_params.yaml の lane_detector と同じ値にすること (同じカメラ).
"""

import json
import os
import threading
import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String
from visualization_msgs.msg import MarkerArray

from common_python.workspace_paths import default_workspace_asset, resolve_workspace_asset
from oit_navigation.lane_nav.geometry import CameraModel, project_to_ground
from oit_navigation.utils.image_util import cv2_to_imgmsg, imgmsg_to_cv2
from oit_navigation.utils.viz_markers import cone_markers, marker_array


class ConeDetectorNode(Node):
    def __init__(self):
        super().__init__('cone_detector')
        d = self.declare_parameter
        self.model_path = str(d('model_path', default_workspace_asset('models', 'cone.pt')).value)
        self.device = str(d('device', 'cpu').value)
        self.imgsz = int(d('imgsz', 640).value)
        self.conf_threshold = float(d('conf_threshold', 0.4).value)     # js/cone_detector.js CONF_THRESHOLD
        self.iou_threshold = float(d('iou_threshold', 0.45).value)
        self.max_inference_hz = float(d('max_inference_hz', 10.0).value)
        self.image_topic = str(d('image_topic', '/aiformula_sensing/zed_node/left_image/undistorted').value)
        self.frame_id = str(d('frame_id', 'base_link').value)
        self.cones_topic = str(d('cones_topic', '/aiformula_perception/cone_detector/cones').value)
        self.status_topic = str(d('status_topic', '/aiformula_perception/cone_detector/status').value)
        self.markers_topic = str(d('markers_topic', '/aiformula_visualization/cone_detector/markers').value)
        self.annotated_topic = str(d('annotated_image_topic', '/aiformula_visualization/cone_detector/annotated_image').value)
        self.publish_annotated = bool(d('publish_annotated_image', True).value)
        self.annotated_scale = float(d('annotated_image_scale', 0.5).value)   # rosbag を軽くするため縮小して出す
        self.x_min = float(d('valid_x_min', 0.3).value)
        self.x_max = float(d('valid_x_max', 8.0).value)
        self.y_abs_max = float(d('valid_y_abs_max', 3.0).value)
        self.camera = CameraModel(
            fx=float(d('camera_fx', 763.17).value), fy=float(d('camera_fy', 763.17).value),
            cx=float(d('camera_cx', 960.0).value), cy=float(d('camera_cy', 540.0).value),
            ref_width=int(d('camera_ref_width', 1920).value), ref_height=int(d('camera_ref_height', 1080).value),
            cam_height=float(d('camera_height', 0.56).value), cam_x=float(d('camera_x', 0.055).value),
            pitch_down=np.deg2rad(float(d('camera_pitch_down_deg', 1.8).value)),
        )

        self.model = None
        self._load_model()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.is_compressed = 'compressed' in self.image_topic
        self.create_subscription(CompressedImage if self.is_compressed else Image, self.image_topic, self._image_cb, qos)
        self.cones_pub = self.create_publisher(PoseArray, self.cones_topic, 1)
        self.status_pub = self.create_publisher(String, self.status_topic, 1)
        self.markers_pub = self.create_publisher(MarkerArray, self.markers_topic, 1)
        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 1) if self.publish_annotated else None
        self._busy = threading.Lock()
        self._last_t = 0.0
        self.get_logger().info(
            f'cone_detector: {self.image_topic} -> {self.cones_topic} (model {self.model_path}, conf>={self.conf_threshold}, '
            f'有効範囲 {self.x_min}<x<{self.x_max}m |y|<{self.y_abs_max}m)')

    def _load_model(self):
        path = resolve_workspace_asset(self.model_path)
        if not os.path.exists(path):
            self.get_logger().error(f'コーンのモデルがありません: {self.model_path}')
            return
        try:
            from ultralytics import YOLO
            self.model = YOLO(path)
            self.model_path = path
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'コーンのモデル読込に失敗: {exc}')

    def _decode(self, msg) -> Optional[np.ndarray]:
        if isinstance(msg, CompressedImage):
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        return imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _image_cb(self, msg):
        if self.model is None:
            return
        now = time.monotonic()
        if self.max_inference_hz > 0 and now - self._last_t < 1.0 / self.max_inference_hz:
            return
        if not self._busy.acquire(blocking=False):
            return
        try:
            self._last_t = now
            img = self._decode(msg)
            if img is not None:
                self._process(msg.header, img)
        finally:
            self._busy.release()

    def _process(self, header, img: np.ndarray):
        h, w = img.shape[:2]
        res = self.model.predict(source=img, conf=self.conf_threshold, iou=self.iou_threshold, imgsz=self.imgsz,
                                 device=self.device, verbose=False)
        boxes = res[0].boxes if res and res[0].boxes is not None else []
        cones: List[dict] = []
        drawn = []
        for box in boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())
            gx, gy, ok = project_to_ground(self.camera, np.array([(x1 + x2) / 2]), np.array([y2]), w, h)
            x, y = float(gx[0]), float(gy[0])
            valid = bool(ok[0]) and self.x_min < x < self.x_max and abs(y) < self.y_abs_max
            drawn.append((x1, y1, x2, y2, conf, x, y, valid))
            if valid:
                cones.append({'x': round(x, 3), 'y': round(y, 3), 'distance': round(float(np.hypot(x, y)), 3),
                              'conf': round(conf, 3), 'bbox': [round(v, 1) for v in (x1, y1, x2, y2)]})

        out_header = Header(stamp=header.stamp, frame_id=self.frame_id)
        pa = PoseArray(header=out_header)
        for c in cones:
            p = Pose()
            p.position.x, p.position.y = c['x'], c['y']
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.cones_pub.publish(pa)
        self.status_pub.publish(String(data=json.dumps({'num_cones': len(cones), 'num_boxes': len(drawn), 'cones': cones})))
        self.markers_pub.publish(marker_array(out_header, cone_markers(
            out_header, [(c['x'], c['y']) for c in cones], [c['conf'] for c in cones])))
        if self.annotated_pub is not None:
            vis = img.copy()
            for i, (x1, y1, x2, y2, conf, x, y, valid) in enumerate(sorted(drawn, key=lambda b: b[0])):
                color = (0, 140, 255) if valid else (120, 120, 120)
                cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), color, 3)
                label = (f'{np.hypot(x, y):.1f}m (x={x:.1f} y={y:+.1f}) {conf:.2f}' if valid
                         else f'out of range {conf:.2f}')
                # 近くに並んだコーンのラベルが重ならないよう, 枠の上と下に交互に置く
                ty = max(28, int(y1) - 10) if i % 2 == 0 else min(vis.shape[0] - 8, int(y2) + 36)
                cv2.putText(vis, label, (int(x1), ty), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 6, cv2.LINE_AA)
                cv2.putText(vis, label, (int(x1), ty), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 2, cv2.LINE_AA)
            cv2.putText(vis, f'cones: {len(cones)} (boxes {len(drawn)})', (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 255), 2, cv2.LINE_AA)
            if self.annotated_scale != 1.0:
                vis = cv2.resize(vis, None, fx=self.annotated_scale, fy=self.annotated_scale, interpolation=cv2.INTER_AREA)
            self.annotated_pub.publish(cv2_to_imgmsg(vis, encoding='bgr8', frame_id=header.frame_id, stamp=header.stamp))


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
