#!/usr/bin/env python3
"""
lane_detector.py - 白線検出ノード (backend: YOLOP または UFLD).

    カメラ画像 --backend--> 白線ごとの画像点列 --地面投影--> base_link の点列
        --2 次多項式フィット--> LineTracker で「左境界 / 中央線 / 右境界」に割り当て
        --> aiformula_interfaces/LaneLines (+ RViz 用 Path x3, 注釈画像)

backend:
    yolop (実機の既定): models/honda_shihou_finetuned_best.pth の白線セグメンテーションマスクから
                        lane_nav/mask_lines.py で白線ごとの点列を取り出す. TensorRT 対応 (Jetson).
    ufld              : models/ufld_honda_finetuned_best.pth (UFLD v1) の車線スロットごとの点列.
                        重みは 245MB で git 管理外なので, 使う場合は各自 models/ に置く.

どちらの backend でも, 線の順番/スロット番号は役割に使わない (lane_nav/line_tracker.py 参照).
見えなかった線は追跡中の道幅から補完し, detected=false として出す.

推論は専用スレッドで最新フレームだけを処理する (重い推論でコールバックを詰まらせない).
"""

import json
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64, Header, String

from aiformula_interfaces.msg import LaneLine, LaneLines
from common_python.workspace_paths import default_workspace_asset, resolve_workspace_asset
from oit_navigation.lane_nav import (
    CameraModel, LineTracker, LineTrackerParams, fit_line, ground_to_image, project_to_ground,
)
from oit_navigation.lane_nav.line_tracker import ROLES, TrackedLines
from oit_navigation.lane_nav.mask_lines import MaskLinesParams, extract_mask_lines
from oit_navigation.utils.image_util import cv2_to_imgmsg, imgmsg_to_cv2

ROLE_COLORS_BGR = {"left": (255, 208, 53), "center": (0, 212, 255), "right": (216, 90, 255)}


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__("lane_detector")
        self._declare_and_load_parameters()

        import torch  # torch の import は重いのでここで
        if torch.get_num_threads() > 2:
            torch.set_num_threads(2)
        if self.backend == "ufld":
            from oit_navigation.ufld import UFLDLaneModel
            weight = resolve_workspace_asset(self.ufld_weight_path)
            self.model = UFLDLaneModel(weight, device=self.use_device)
            c = self.model.cfg
            backend_info = (f"UFLD weights={weight} (ResNet{c.backbone}, griding={c.griding_num}, "
                            f"rows={c.cls_num_per_lane}, lanes={c.num_lanes}) device={self.model.device}")
        elif self.backend == "yolop":
            from oit_navigation.yolop_lane_backend import YOLOPLaneModel
            self.model = YOLOPLaneModel(
                self.yolop_weight_path, device=self.use_device, roi_mode=self.roi_mode,
                top_cut_ratio=self.top_cut_ratio, use_tensorrt=self.use_tensorrt,
                tensorrt_engine_path=self.tensorrt_engine_path, log=self.get_logger().info)
            backend_info = f"YOLOP weights={self.model.weight_path} ({self.model.backend_name})"
        else:
            raise ValueError(f"backend は 'yolop' か 'ufld' です: {self.backend}")
        self.tracker = LineTracker(self.tracker_params)
        self._tracker_lock = threading.Lock()  # tracker は推論スレッドと reseed コールバックの両方から触る

        self._lock = threading.Lock()
        self._event = threading.Event()
        self._frame: Optional[Tuple[np.ndarray, Header]] = None
        self._running = True
        self._last_infer = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.is_compressed = "compressed" in self.input_image_topic
        self.create_subscription(CompressedImage if self.is_compressed else Image,
                                 self.input_image_topic, self._image_cb, qos)
        self.lanes_pub = self.create_publisher(LaneLines, self.lane_lines_topic, 1)
        self.path_pubs = {r: self.create_publisher(Path, t, 1) for r, t in (
            ("left", self.lane_line_left_topic), ("center", self.lane_line_center_topic),
            ("right", self.lane_line_right_topic))}
        self.annotated_pub = self.create_publisher(Image, self.annotated_image_topic, 1)
        self.debug_pub = self.create_publisher(String, "~/debug", 1)
        # 6 レーン走行 (six_lane_planner) が白線の役割取り違えを検出したときの車両の横位置
        # (レーン座標 F: 左白線=0, 中央線=3, 右白線=6). これで線の並びを置き直す. 空文字なら購読しない
        if self.lane_reseed_topic:
            self.create_subscription(Float64, self.lane_reseed_topic, self._reseed_cb, 5)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"lane_detector ready: backend={self.backend} {backend_info} "
            f"topic={self.input_image_topic} lane_width={self.tracker_params.lane_width_init}m")

    # ------------------------------------------------------------------ params
    def _declare_and_load_parameters(self):
        d = self.declare_parameter
        self.input_image_topic = d("input_image_topic", "/aiformula_sensing/zed_node/left_image/undistorted").value
        self.backend = str(d("backend", "yolop").value).lower()
        self.yolop_weight_path = d("weight_path", default_workspace_asset("models", "honda_shihou_finetuned_best.pth")).value
        self.ufld_weight_path = d("ufld_weight_path", default_workspace_asset("models", "ufld_honda_finetuned_best.pth")).value
        self.use_device = str(d("use_device", "cpu").value)
        # YOLOP 用
        self.roi_mode = str(d("roi_mode", "mask_top").value)
        self.top_cut_ratio = float(d("top_cut_ratio", 0.45).value)
        self.use_tensorrt = bool(d("use_tensorrt", False).value)
        self.tensorrt_engine_path = str(d("tensorrt_engine_path", "").value)
        self.mask_lines_params = MaskLinesParams(top_ratio=self.top_cut_ratio)
        self.max_inference_hz = float(d("max_inference_hz", 15.0).value)
        self.robot_frame_id = d("robot_frame_id", "base_link").value
        self.lane_lines_topic = d("lane_lines_topic", "/aiformula_perception/lane_detector/lane_lines").value
        self.lane_line_left_topic = d("lane_line_left_topic", "/aiformula_perception/lane_line_publisher/lane_lines/left").value
        self.lane_line_center_topic = d("lane_line_center_topic", "/aiformula_perception/lane_line_publisher/lane_lines/center").value
        self.lane_line_right_topic = d("lane_line_right_topic", "/aiformula_perception/lane_line_publisher/lane_lines/right").value
        self.annotated_image_topic = d("annotated_image_topic", "/aiformula_visualization/lane_detector/annotated_image").value
        self.publish_annotated_image = bool(d("publish_annotated_image", True).value)
        self.fit_x_max = float(d("fit_x_max", 12.0).value)

        # カメラ (ZED X HD1080 SN48442725 / extrinsic.yaml 相当)
        self.camera = CameraModel(
            fx=float(d("camera_fx", 763.17).value), fy=float(d("camera_fy", 763.17).value),
            cx=float(d("camera_cx", 960.0).value), cy=float(d("camera_cy", 540.0).value),
            ref_width=int(d("camera_ref_width", 1920).value), ref_height=int(d("camera_ref_height", 1080).value),
            cam_height=float(d("camera_height", 0.56).value), cam_x=float(d("camera_x", 0.055).value),
            pitch_down=np.deg2rad(float(d("camera_pitch_down_deg", 1.8).value)),
        )
        self.tracker_params = LineTrackerParams(
            x_ref=float(d("tracker_x_ref", 2.0).value),
            lane_width_init=float(d("lane_width", 3.5).value),
            gate_ratio=float(d("tracker_gate_ratio", 0.4).value),
            width_tolerance=float(d("tracker_width_tolerance", 0.3).value),
            max_heading_diff=float(d("tracker_max_heading_diff", 0.3).value),
            # 起動時の車両位置: 中央線からの横ずれ [m] (左正). 0 = 中央線の上から発進
            init_offset=float(d("tracker_init_offset", 0.0).value),
            anchor_tolerance=float(d("tracker_anchor_tolerance", 0.15).value),
            anchor_frames=int(d("tracker_anchor_frames", 3).value),
            anchor_max_x_min=float(d("tracker_anchor_max_x_min", 6.0).value),
            anchor_heading_diff=float(d("tracker_anchor_heading_diff", 0.1).value),
        )
        self.lane_reseed_topic = str(d("lane_reseed_topic", "/aiformula_control/six_lane_planner/lane_reseed").value)

    # ------------------------------------------------------------------ io
    def _image_cb(self, msg):
        if isinstance(msg, CompressedImage):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        else:
            img = imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if img is None:
            self.get_logger().warning("カメラ画像のデコードに失敗 (フレームを破棄)", throttle_duration_sec=5.0)
            return
        with self._lock:
            self._frame = (img, msg.header)
        self._event.set()

    def _reseed_cb(self, msg: Float64):
        with self._tracker_lock:
            self.tracker.seed_lane_position(float(msg.data))

    def _worker_loop(self):
        min_period = 1.0 / self.max_inference_hz if self.max_inference_hz > 0 else 0.0
        while rclpy.ok() and self._running:
            if not self._event.wait(timeout=0.1):
                continue
            self._event.clear()
            wait = self._last_infer + min_period - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            with self._lock:
                item, self._frame = self._frame, None
            if item is None:
                continue
            self._last_infer = time.monotonic()
            try:
                self._process(*item)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"UFLD 処理エラー: {e}", throttle_duration_sec=2.0)

    # ------------------------------------------------------------------ main
    def _process(self, img: np.ndarray, header: Header):
        h, w = img.shape[:2]
        mask = None
        if self.backend == "yolop":
            mask = self.model.infer_mask(img)
            lanes = extract_mask_lines(mask, self.mask_lines_params)
        else:
            lanes = self.model.detect(img)
        fits, ground_pts = [], []
        for lane in lanes:
            if lane is None:
                continue
            x, y, valid = project_to_ground(self.camera, lane["u"], lane["v"], w, h)
            f = fit_line(x[valid], y[valid], x_max_fit=self.fit_x_max)
            if f is not None:
                fits.append(f)
                ground_pts.append((x[valid], y[valid]))
        with self._tracker_lock:
            tracked = self.tracker.update(fits)

        out_header = Header(stamp=header.stamp, frame_id=self.robot_frame_id)
        msg = LaneLines(header=out_header)
        msg.lane_width_left = float(tracked.lane_widths["left"])
        msg.lane_width_right = float(tracked.lane_widths["right"])
        msg.reanchored = bool(tracked.reanchored)
        for role in ROLES:
            setattr(msg, role, self._lane_line_msg(tracked, role, fits, ground_pts))
            self.path_pubs[role].publish(self._path_msg(tracked.lines.get(role), out_header))
        self.lanes_pub.publish(msg)
        self.debug_pub.publish(String(data=json.dumps({
            "backend": self.backend,
            "lines": [None if l is None else len(l["u"]) for l in lanes],
            "detected": tracked.detected, "offsets": {k: round(v, 3) for k, v in tracked.offsets.items()},
        })))
        if self.publish_annotated_image and self.annotated_pub.get_subscription_count() > 0:
            self.annotated_pub.publish(cv2_to_imgmsg(self._annotate(img, lanes, tracked, mask), "bgr8",
                                                     frame_id=header.frame_id, stamp=header.stamp))

    @staticmethod
    def _lane_line_msg(tracked: TrackedLines, role: str, fits, ground_pts) -> LaneLine:
        m = LaneLine()
        f = tracked.lines.get(role)
        if f is None:
            return m
        m.valid = True
        m.detected = bool(tracked.detected.get(role))
        m.c0, m.c1, m.c2 = (float(c) for c in f.coeffs)
        m.x_min, m.x_max = float(f.x_min), float(f.x_max)
        for fit, (px, py) in zip(fits, ground_pts):
            if fit is f:
                m.points_x = [float(v) for v in px]
                m.points_y = [float(v) for v in py]
        return m

    @staticmethod
    def _path_msg(fit, header: Header) -> Path:
        path = Path(header=header)
        if fit is None:
            return path
        x0 = 1.0 if fit.inferred else fit.x_min
        x1 = 8.0 if fit.inferred else fit.x_max
        for x in np.arange(max(x0, 0.8), min(x1, 12.0) + 1e-6, 0.5):
            ps = PoseStamped(header=header)
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(fit.y_at(x))
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def _annotate(self, img, lanes, tracked: TrackedLines, mask=None):
        vis = img.copy()
        if mask is not None:  # YOLOP の白線マスクを薄く重ねる
            vis[mask > 0] = (0.5 * vis[mask > 0] + 0.5 * np.array([0, 255, 0])).astype(np.uint8)
        h, w = vis.shape[:2]
        r = max(2, w // 320)
        for lane in lanes:
            if lane is None:
                continue
            for u, v in zip(lane["u"], lane["v"]):
                cv2.circle(vis, (int(u), int(v)), r, (255, 255, 255), -1)
        for role in ROLES:
            f = tracked.lines.get(role)
            if f is None:
                continue
            xs = np.arange(1.0 if f.inferred else max(f.x_min, 0.8), (8.0 if f.inferred else min(f.x_max, 12.0)), 0.25)
            u, v, ok = ground_to_image(self.camera, xs, f.y_at(xs), w, h)
            pts = np.stack([u[ok], v[ok]], 1).astype(np.int32)
            if len(pts) < 2:
                continue
            color = ROLE_COLORS_BGR[role]
            if tracked.detected.get(role):
                cv2.polylines(vis, [pts], False, color, max(2, w // 240))
            else:  # 補完線は破線
                for i in range(0, len(pts) - 1, 2):
                    cv2.line(vis, tuple(pts[i]), tuple(pts[i + 1]), color, max(2, w // 240))
        for i, role in enumerate(ROLES):
            tag = "det" if tracked.detected.get(role) else ("inf" if tracked.lines.get(role) is not None else "-")
            cv2.putText(vis, f"{role}: {tag}", (10, 30 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        ROLE_COLORS_BGR[role], 2)
        return vis

    def destroy_node(self):
        self._running = False
        self._event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=0.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
