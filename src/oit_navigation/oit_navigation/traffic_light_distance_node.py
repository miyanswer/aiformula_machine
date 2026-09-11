#!/usr/bin/env python3
"""
traffic_light_distance_node.py - 信号機検出 & 画面占有率による距離逆算 ROS 2 ノード

概要:
    YOLO モデル (models/traffic_light.pt) でカメラ画像から信号機を検出し、
    バウンディングボックスの「縦の長さ = 画面に占める割合 (占有率)」から
    信号機までの距離 [m] を逆算してトピックに配信する。

距離計算:
    occupancy = bbox_height_px / image_height_px
    distance  = distance_coeff / occupancy
    (distance_coeff は real_height_m と vertical_fov_deg から自動算出、
     または distance_coeff パラメータで直接指定)

配信トピック (base_topic = /aiformula_perception/traffic_light):
    <base>/nearest_distance  std_msgs/Float32  最も近い信号機までの距離 [m]
    <base>/red_distance      std_msgs/Float32  最も近い赤信号までの距離 [m]
    <base>/green_distance    std_msgs/Float32  最も近い青信号までの距離 [m]
    <base>/status            std_msgs/String   検出内容の JSON
    <base>/annotated_image   sensor_msgs/Image 可視化画像 (publish_annotated_image=true 時)
"""

import json
import os
import time
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Header, String

from oit_navigation.utils.image_util import cv2_to_imgmsg, imgmsg_to_cv2
from oit_navigation.utils.traffic_light_distance import TrafficLightDistanceEstimator
from common_python.workspace_paths import default_workspace_asset, resolve_workspace_asset


class TrafficLightDistanceNode(Node):
    """YOLO 信号機検出 & 画面占有率ベースの距離逆算ノード。"""

    def __init__(self):
        super().__init__('traffic_light_distance_node')

        self._declare_parameters()
        self._load_parameters()

        self.estimator = TrafficLightDistanceEstimator(
            real_height_m=self.real_height_m,
            focal_length_y=self.focal_length_y,
            reference_image_height=self.reference_image_height,
            vertical_fov_deg=self.vertical_fov_deg,
            distance_coeff=self.distance_coeff,
            min_occupancy=self.min_occupancy,
            max_valid_distance=self.max_valid_distance,
            smoothing_alpha=self.smoothing_alpha,
        )

        self.model = None
        self._init_yolo_model()

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.is_compressed = self.image_topic.endswith('/compressed') or 'compressed' in self.image_topic
        sub_type = CompressedImage if self.is_compressed else Image
        self.sub_image = self.create_subscription(
            sub_type, self.image_topic, self._image_callback, qos_sensor
        )

        self.pub_nearest_distance = self.create_publisher(Float32, f"{self.base_topic}/nearest_distance", 10)
        self.pub_red_distance = self.create_publisher(Float32, f"{self.base_topic}/red_distance", 10)
        self.pub_green_distance = self.create_publisher(Float32, f"{self.base_topic}/green_distance", 10)
        self.pub_status = self.create_publisher(String, f"{self.base_topic}/status", 10)
        self.pub_annotated = (
            self.create_publisher(Image, f"{self.base_topic}/annotated_image", 1)
            if self.publish_annotated_image else None
        )

        self._frame_count = 0

        self.get_logger().info(
            "TrafficLightDistanceNode initialized:\n"
            f"  - model_path        : {self.model_path}\n"
            f"  - image_topic       : {self.image_topic} (compressed={self.is_compressed})\n"
            f"  - device            : {self.device}\n"
            f"  - conf/iou          : {self.conf_threshold} / {self.iou_threshold}\n"
            f"  - real_height_m     : {self.real_height_m}\n"
            f"  - focal_length_y    : {self.focal_length_y} px @ h={self.reference_image_height}px\n"
            f"  - vertical_fov_deg  : {self.vertical_fov_deg}\n"
            f"  - distance_coeff    : {self.estimator.distance_coeff:.4f} "
            f"(distance = coeff / (bbox_h / image_h))"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        # YOLO
        self.declare_parameter('model_path', default_workspace_asset('models', 'traffic_light.pt'))
        self.declare_parameter('device', 'cpu')            # 'cpu', 'mps', '0' (CUDA)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)

        # Topics
        self.declare_parameter('image_topic', '/aiformula_sensing/zed_node/left_image/undistorted/compressed')
        self.declare_parameter('base_topic', '/aiformula_perception/traffic_light')
        self.declare_parameter('publish_annotated_image', True)

        # 画面占有率 -> 距離 の逆算モデル (優先順: distance_coeff > focal_length_y > vertical_fov_deg)
        self.declare_parameter('real_height_m', 0.32)            # 信号機の実寸縦幅 [m] (1辺32cm)
        self.declare_parameter('focal_length_y', 733.0)         # ZED X 垂直焦点距離 [px] (K[4])
        self.declare_parameter('reference_image_height', 1080)  # focal_length_y の基準画像高さ [px]
        self.declare_parameter('vertical_fov_deg', 0.0)         # 焦点距離不明時の垂直画角 [deg]
        self.declare_parameter('distance_coeff', 0.0)           # >0 で係数 k を直接指定
        self.declare_parameter('min_occupancy', 1.0e-4)         # ゼロ除算防止の最小占有率
        self.declare_parameter('max_valid_distance', 50.0)      # 有効距離上限 [m]
        self.declare_parameter('smoothing_alpha', 0.7)          # EMA 係数 (1.0で無効)

    def _load_parameters(self):
        p = self.get_parameter
        self.model_path = str(p('model_path').value)
        self.device = str(p('device').value)
        self.imgsz = int(p('imgsz').value)
        self.conf_threshold = float(p('conf_threshold').value)
        self.iou_threshold = float(p('iou_threshold').value)

        self.image_topic = str(p('image_topic').value)
        self.base_topic = str(p('base_topic').value).rstrip('/')
        self.publish_annotated_image = bool(p('publish_annotated_image').value)

        self.real_height_m = float(p('real_height_m').value)
        self.focal_length_y = float(p('focal_length_y').value)
        self.reference_image_height = int(p('reference_image_height').value)
        self.vertical_fov_deg = float(p('vertical_fov_deg').value)
        self.distance_coeff = float(p('distance_coeff').value)
        self.min_occupancy = float(p('min_occupancy').value)
        self.max_valid_distance = float(p('max_valid_distance').value)
        self.smoothing_alpha = float(p('smoothing_alpha').value)

    def _init_yolo_model(self):
        resolved = resolve_workspace_asset(self.model_path)
        if not os.path.exists(resolved):
            fallback = resolve_workspace_asset(os.path.join("models", "traffic_light.pt"))
            if os.path.exists(fallback):
                resolved = fallback

        if not os.path.exists(resolved):
            self.get_logger().error(f"YOLO model not found: {self.model_path}")
            self.model = None
            return

        try:
            from ultralytics import YOLO
            self.model = YOLO(resolved)
            self.model_path = resolved
            self.get_logger().info(f"YOLO model loaded ({resolved}), classes: {self.model.names}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load YOLO model: {exc}")
            self.model = None

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------
    def _decode_image(self, msg) -> Optional[np.ndarray]:
        if isinstance(msg, CompressedImage):
            np_arr = np.frombuffer(msg.data, np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _image_callback(self, msg):
        if self.model is None:
            return

        t_start = time.perf_counter()
        cv_image = self._decode_image(msg)
        if cv_image is None:
            return

        image_h = int(cv_image.shape[0])
        self._frame_count += 1

        results = self.model.predict(
            source=cv_image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )

        detections: List[Dict[str, Any]] = []
        nearest: Optional[float] = None
        nearest_red: Optional[float] = None
        nearest_green: Optional[float] = None

        annotated = cv_image.copy() if self.pub_annotated is not None else None

        if results and results[0].boxes is not None:
            for i, box in enumerate(results[0].boxes):
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                cls_name = self.model.names.get(cls_id, f"class_{cls_id}")

                x1, y1, x2, y2 = (float(v) for v in xyxy)
                bbox_h = abs(y2 - y1)
                occupancy = self.estimator.occupancy_ratio(bbox_h, image_h)
                distance = self.estimator.estimate(bbox_h, image_h, track_id=i)

                detections.append({
                    "class_name": cls_name,
                    "class_id": cls_id,
                    "confidence": round(conf, 3),
                    "bbox": [round(v, 1) for v in (x1, y1, x2, y2)],
                    "bbox_height_px": round(bbox_h, 1),
                    "occupancy_ratio": round(occupancy, 5) if occupancy is not None else None,
                    "distance_m": round(distance, 3) if distance is not None else None,
                })

                if distance is not None:
                    if nearest is None or distance < nearest:
                        nearest = distance
                    if "red" in cls_name.lower():
                        if nearest_red is None or distance < nearest_red:
                            nearest_red = distance
                    elif "green" in cls_name.lower():
                        if nearest_green is None or distance < nearest_green:
                            nearest_green = distance

                if annotated is not None:
                    self._draw_detection(annotated, x1, y1, x2, y2, cls_name, conf, bbox_h, distance)

        infer_ms = (time.perf_counter() - t_start) * 1000.0
        self._publish_results(msg.header, detections, nearest, nearest_red, nearest_green)

        if annotated is not None:
            self._draw_hud(annotated, infer_ms, len(detections), nearest)
            self.pub_annotated.publish(
                cv2_to_imgmsg(annotated, encoding='bgr8',
                              frame_id=msg.header.frame_id, stamp=msg.header.stamp)
            )

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"[traffic_light_distance] det={len(detections)} "
                f"nearest={'%.2f m' % nearest if nearest is not None else '---'} "
                f"infer={infer_ms:.1f} ms"
            )

    # ------------------------------------------------------------------
    # Publish / draw
    # ------------------------------------------------------------------
    def _publish_results(self, header: Header, detections, nearest, nearest_red, nearest_green):
        if nearest is not None:
            self.pub_nearest_distance.publish(Float32(data=float(nearest)))
        if nearest_red is not None:
            self.pub_red_distance.publish(Float32(data=float(nearest_red)))
        if nearest_green is not None:
            self.pub_green_distance.publish(Float32(data=float(nearest_green)))

        status = {
            "timestamp": header.stamp.sec + header.stamp.nanosec * 1e-9,
            "num_detections": len(detections),
            "nearest_distance_m": nearest,
            "nearest_red_distance_m": nearest_red,
            "nearest_green_distance_m": nearest_green,
            "detections": detections,
        }
        self.pub_status.publish(String(data=json.dumps(status)))

    @staticmethod
    def _class_color(cls_name: str):
        if "red" in cls_name.lower():
            return (0, 0, 255)
        if "green" in cls_name.lower():
            return (0, 255, 0)
        return (0, 255, 255)

    def _draw_detection(self, img, x1, y1, x2, y2, cls_name, conf, bbox_h, distance):
        ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)
        color = self._class_color(cls_name)
        cv2.rectangle(img, (ix1, iy1), (ix2, iy2), color, 2)

        if distance is not None:
            label = f"{cls_name} {conf:.2f} | D={distance:.2f}m (h={bbox_h:.0f}px)"
        else:
            label = f"{cls_name} {conf:.2f} | h={bbox_h:.0f}px"
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(img, (ix1, max(0, iy1 - lh - 8)), (ix1 + lw + 6, iy1), color, -1)
        cv2.putText(img, label, (ix1 + 3, iy1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    def _draw_hud(self, img, infer_ms, num_det, nearest):
        h, w = img.shape[:2]
        cv2.rectangle(img, (0, 0), (w, 40), (25, 25, 25), -1)
        d_str = f"{nearest:.2f} m" if nearest is not None else "---"
        text = (f"Traffic Light: {num_det} det | Nearest: {d_str} | "
                f"Infer: {infer_ms:.1f} ms | coeff={self.estimator.distance_coeff:.3f}")
        cv2.putText(img, text, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDistanceNode()
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
