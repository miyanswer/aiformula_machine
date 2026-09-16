#!/usr/bin/env python3
"""
yolop_lane_detector.py - Exact Replication of object_road_detector with Inference Frame Decimation
Replicates the exact decoding, interpolation, and visualization of ai_formula_oit_2027/2026.
Features lightweight frame skipping (decimation) and single-slot non-blocking worker to prevent lag.
"""

from copy import deepcopy
import os
import sys
import threading
import time
from pathlib import Path
from typing import Tuple, Optional

import cv2
import numpy as np
import torch
import torchvision.transforms as transforms

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from aiformula_interfaces.msg import Rect, RectMultiArray

from oit_navigation.utils.image_util import cv2_to_imgmsg, imgmsg_to_cv2
from common_python.workspace_paths import default_workspace_asset, resolve_workspace_asset

# Add internal oit_navigation/yolop to sys.path
_LOCAL_YOLOP_DIR = Path(__file__).resolve().parent / "yolop"
if _LOCAL_YOLOP_DIR.exists() and str(_LOCAL_YOLOP_DIR) not in sys.path:
    sys.path.insert(0, str(_LOCAL_YOLOP_DIR))

try:
    from lib.config import cfg
    from lib.core.general import non_max_suppression, scale_coords
    from lib.models import get_net
    from lib.utils import letterbox_for_img
    from lib.utils.utils import select_device
except ImportError:
    try:
        from oit_navigation.yolop.lib.config import cfg
        from oit_navigation.yolop.lib.core.general import non_max_suppression, scale_coords
        from oit_navigation.yolop.lib.models import get_net
        from oit_navigation.yolop.lib.utils import letterbox_for_img
        from oit_navigation.yolop.lib.utils.utils import select_device
    except ImportError:
        cfg = None
        get_net = None
        letterbox_for_img = None
        select_device = None
        non_max_suppression = None
        scale_coords = None


def draw_lane_lines(image: np.ndarray, ll_seg_mask: np.ndarray,
                    alpha: float = 0.5,
                    color: Tuple[int, int, int] = (0, 255, 0)) -> None:
    """Exact replication of object_road_detector_util.draw_lane_lines."""
    if image is None or ll_seg_mask is None:
        return
    if image.shape[:2] != ll_seg_mask.shape[:2]:
        return

    mask = ll_seg_mask == 1
    if not np.any(mask):
        return

    overlay = np.zeros_like(image, dtype=np.uint8)
    overlay[mask] = color

    blended = cv2.addWeighted(image[mask], 1.0 - alpha, overlay[mask], alpha, gamma=0.0)
    if blended is not None:
        image[mask] = blended


def plot_one_box(image: np.ndarray, bbox_coords: torch.Tensor, color: Tuple[int, int, int] = (255, 0, 0)) -> None:
    """Exact replication of object_road_detector_util.plot_one_box."""
    top_left = (int(bbox_coords[0]), int(bbox_coords[1]))
    bottom_right = (int(bbox_coords[2]), int(bbox_coords[3]))
    thickness = max(int(image.shape[0] * 0.002), 1)
    cv2.rectangle(image, top_left, bottom_right, color, thickness, lineType=cv2.LINE_AA)


def draw_bounding_boxes(image: np.ndarray, scaled_boxes: torch.Tensor) -> None:
    """Exact replication of object_road_detector_util.draw_bounding_boxes.
    `scaled_boxes` must already be in `image`'s pixel space (see scale_detected_boxes)."""
    if scaled_boxes is None or len(scaled_boxes) == 0:
        return
    for bboxes_coord in scaled_boxes:
        plot_one_box(image, bboxes_coord)


def scale_detected_boxes(objects: torch.Tensor, input_image_shape: torch.Size, output_image_shape) -> torch.Tensor:
    """Scales raw detection boxes into `output_image_shape` pixel space.
    Clones first: `scale_coords` mutates its coords argument in place, and `objects` is a
    view into the node's shared/cached detection tensor - mutating it would corrupt the
    cache for every subsequent frame until the next inference cycle overwrites it."""
    return scale_coords(input_image_shape, objects[:, :4].clone(), output_image_shape).round()


class YOLOPLaneDetectorNode(Node):
    """
    High-Performance, Exact-Match YOLOP Lane & Object Detector.
    Features:
    - Decimated / Skip-Frame Inference: Runs heavy YOLOP inference every N frames.
    - Single-slot buffer: Drops old frames automatically without queue backlog.
    - CPU thread management: Avoids CPU starvation.
    """

    def __init__(self):
        super().__init__('yolop_lane_detector')

        # Limit CPU threads to keep CPU responsive for video streaming and RViz
        if torch.get_num_threads() > 2:
            torch.set_num_threads(2)

        self._declare_parameters()
        self._load_parameters()

        # Initialize detector
        self._init_detector()

        # Threading buffers & cache for smooth real-time streaming
        self._infer_msg: Optional[Tuple[np.ndarray, Header]] = None
        self._msg_lock = threading.Lock()
        self._infer_event = threading.Event()
        self._is_inferring = False
        self._latest_mask: Optional[np.ndarray] = None
        self._latest_boxes: Optional[torch.Tensor] = None
        self._latest_input_shape: Optional[torch.Size] = None
        self._frame_count = 0
        self._running = True

        # Sensor data QoS: RELIABLE with depth 1
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Automatically select Image or CompressedImage type based on topic name
        self.is_compressed = self.input_image_topic.endswith('/compressed') or 'compressed' in self.input_image_topic
        sub_type = CompressedImage if self.is_compressed else Image

        self.image_sub = self.create_subscription(
            sub_type,
            self.input_image_topic,
            self._image_callback,
            qos_sensor
        )

        self.lane_mask_image_pub = self.create_publisher(Image, self.mask_image_topic, 1)
        self.annotated_image_pub = self.create_publisher(Image, self.annotated_image_topic, 1)
        self.bbox_pub = self.create_publisher(RectMultiArray, self.bounding_box_topic, 1)

        # Start dedicated inference worker thread
        self._worker_thread = threading.Thread(target=self._inference_worker, daemon=True)
        self._worker_thread.start()

        backend = "TensorRT" if self.trt_runner is not None else "PyTorch"
        self.get_logger().info(
            f"YOLOP Lane Detector Initialized: topic={self.input_image_topic} (compressed={self.is_compressed}), "
            f"device={self.device_str}, backend={backend}, frame_skip={self.frame_skip}, "
            f"weights={self.weight_path}, roi_mode={self.roi_mode}"
        )

    def _declare_parameters(self):
        self.declare_parameter('input_image_topic', '/aiformula_sensing/zed_node/left_image/undistorted')
        self.declare_parameter('mask_image_topic', '/aiformula_perception/object_road_detector/mask_image')
        self.declare_parameter('annotated_image_topic', '/aiformula_visualization/object_road_detector/annotated_image')
        self.declare_parameter('bounding_box_topic', '/aiformula_perception/object_road_detector/rect')

        self.declare_parameter(
            'weight_path',
            default_workspace_asset('models', 'honda_shihou_finetuned_best.pth'),
        )
        self.declare_parameter('use_device', 'cpu')  # 'cpu' or '0' (CUDA)
        self.declare_parameter('confidence_threshold', 0.80)
        self.declare_parameter('iou_threshold', 0.60)
        self.declare_parameter('roi_mode', 'mask_top')  # 'none', 'mask_top', 'crop_bottom'
        self.declare_parameter('top_cut_ratio', 0.45)
        self.declare_parameter('norm_mean', [0.485, 0.456, 0.406])
        self.declare_parameter('norm_std', [0.229, 0.224, 0.225])
        self.declare_parameter('frame_skip', 1)

        # TensorRT (real-hardware only; see export_tensorrt.py). Requires the
        # NVIDIA TensorRT Python bindings, which are not installed in this
        # project's default CPU/dev container - any failure to load falls
        # back to plain PyTorch automatically.
        self.declare_parameter('use_tensorrt', False)
        self.declare_parameter('tensorrt_engine_path', '')  # default: weight_path with .engine extension

    def _load_parameters(self):
        p = self.get_parameter
        self.input_image_topic = p('input_image_topic').value
        self.mask_image_topic = p('mask_image_topic').value
        self.annotated_image_topic = p('annotated_image_topic').value
        self.bounding_box_topic = p('bounding_box_topic').value

        self.weight_path = p('weight_path').value
        self.device_str = str(p('use_device').value)
        self.confidence_threshold = float(p('confidence_threshold').value)
        self.iou_threshold = float(p('iou_threshold').value)
        self.roi_mode = str(p('roi_mode').value)
        self.top_cut_ratio = float(p('top_cut_ratio').value)
        self.norm_mean = list(p('norm_mean').value)
        self.norm_std = list(p('norm_std').value)
        self.frame_skip = max(1, int(p('frame_skip').value))

        self.use_tensorrt = bool(p('use_tensorrt').value)
        self.tensorrt_engine_path = str(p('tensorrt_engine_path').value)

    def _init_detector(self):
        self.trt_runner = None

        # Device selection & preprocessing transform - needed regardless of
        # which inference backend ends up running below.
        if self.device_str == 'cpu':
            self.use_device = torch.device('cpu')
        elif torch.cuda.is_available() and self.device_str != 'cpu':
            self.use_device = torch.device(f'cuda:{self.device_str}' if self.device_str.isdigit() else 'cuda')
        elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            self.use_device = torch.device('mps')
        else:
            self.use_device = torch.device('cpu')

        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=self.norm_mean, std=self.norm_std),
        ])

        if self.use_tensorrt:
            self._init_tensorrt_detector()
            if self.trt_runner is not None:
                # export_tensorrt.py always builds float32 input/output bindings,
                # even for an FP16/INT8 engine (the engine handles internal
                # precision itself) - never half() the input for this path.
                self.use_half_precision = False
                self.detector = None
                return
            self.get_logger().warning("TensorRT unavailable, falling back to PyTorch inference.")

        self.use_half_precision = (self.use_device.type == 'cuda')

        if cfg is None or get_net is None:
            self.get_logger().error("Could not import YOLOP modules!")
            self.detector = None
            return

        # Resolve weights path
        weights = resolve_workspace_asset(self.weight_path)
        if not os.path.exists(weights):
            alt_names = [
                os.path.basename(weights),
                "honda_shihou_finetuned_best.pth",
                "shiho_lane_mask_v2_best.pth",
                "shiho_lane_crop_best.pth",
                "shiho_lane_mask_best.pth",
            ]
            for name in alt_names:
                alt = resolve_workspace_asset(os.path.join("models", name))
                if os.path.exists(alt):
                    weights = alt
                    break

        if not os.path.exists(weights):
            self.get_logger().error(f"YOLOP model weights not found at: {self.weight_path}")
            self.detector = None
            return

        self.get_logger().info(f"Loading YOLOP weights from: {weights}")
        self.detector = get_net(cfg)
        checkpoint = torch.load(weights, map_location=self.use_device)
        state_dict = checkpoint["state_dict"] if isinstance(checkpoint, dict) and "state_dict" in checkpoint else checkpoint
        self.detector.load_state_dict(state_dict)

        if self.use_device.type == 'cuda':
            self.detector = self.detector.to(self.use_device)
            if self.use_half_precision:
                self.detector.half()
        else:
            self.detector = self.detector.to(self.use_device).float()

        self.detector.eval()
        self.get_logger().info("YOLOP Model successfully loaded.")

    def _init_tensorrt_detector(self):
        """Load or automatically compile a TensorRT engine optimized for the current GPU.
        If an engine matching the current GPU model/architecture does not exist yet,
        it builds and caches it on the fly using export_tensorrt."""
        if not torch.cuda.is_available():
            self.get_logger().warning("TensorRT requires CUDA, but CUDA is not available. Falling back to CPU/PyTorch.")
            return

        # Resolve weights path
        weights = resolve_workspace_asset(self.weight_path)
        if not os.path.exists(weights):
            alt_names = [
                os.path.basename(weights),
                "honda_shihou_finetuned_best.pth",
                "shiho_lane_mask_v2_best.pth",
                "shiho_lane_crop_best.pth",
                "shiho_lane_mask_best.pth",
            ]
            for name in alt_names:
                alt = resolve_workspace_asset(os.path.join("models", name))
                if os.path.exists(alt):
                    weights = alt
                    break

        gpu_tag = "cuda0"
        try:
            from oit_navigation.export_tensorrt import get_gpu_device_tag, export_engine_for_current_gpu
            gpu_tag = get_gpu_device_tag(0)
        except Exception:
            export_engine_for_current_gpu = None

        # Determine target engine path
        if self.tensorrt_engine_path:
            engine_path = resolve_workspace_asset(self.tensorrt_engine_path)
        else:
            base_name = os.path.splitext(weights)[0]
            engine_path = f"{base_name}_{gpu_tag}.engine"
            # Fallback check for plain .engine
            if not os.path.exists(engine_path) and os.path.exists(base_name + ".engine"):
                engine_path = base_name + ".engine"

        # Check if engine exists; if not, attempt on-demand compilation
        if not os.path.exists(engine_path):
            self.get_logger().info(f"[TensorRT] No pre-built engine found for GPU '{gpu_tag}'.")
            if export_engine_for_current_gpu is not None and os.path.exists(weights):
                self.get_logger().info(f"[TensorRT] Compiling optimized TensorRT engine for {gpu_tag} -> {engine_path} ...")
                try:
                    export_engine_for_current_gpu(weights_path=weights, engine_path=engine_path, fp16=True)
                    self.get_logger().info(f"[TensorRT] Engine successfully compiled and cached: {engine_path}")
                except Exception as build_err:
                    self.get_logger().warning(
                        f"[TensorRT] Automatic engine compilation skipped/failed ({str(build_err)}). Falling back to PyTorch."
                    )
                    self.trt_runner = None
                    return
            else:
                self.get_logger().warning(f"[TensorRT] Engine not found and compiler unavailable: {engine_path}")
                self.trt_runner = None
                return

        # Load engine
        try:
            from oit_navigation.utils.tensorrt_runtime import TensorRTYOLOPRunner
            self.trt_runner = TensorRTYOLOPRunner(engine_path)
            self.get_logger().info(f"[TensorRT] Engine loaded successfully on GPU ({gpu_tag}): {engine_path}")
        except Exception as e:
            self.get_logger().warning(f"Could not load TensorRT engine ({str(e)}). Falling back to PyTorch.")
            self.trt_runner = None

    def _image_callback(self, msg):
        """
        Runs at full stream FPS (15 FPS).
        - Dispatches heavy inference to worker thread every `frame_skip` frames (e.g. 2 frames = 1 inference).
        - Renders and publishes annotated image and mask immediately using the latest available detection.
        """
        self._frame_count += 1

        # Decode image
        if isinstance(msg, CompressedImage):
            np_arr = np.frombuffer(msg.data, np.uint8)
            undistorted_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            undistorted_image = imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if undistorted_image is None:
            # Silent before: a broken/incomplete frame (e.g. from a rosbridge/
            # websocket relay) just made the mask never update, with nothing in
            # the log to say why. Throttled since a bad frame here can recur
            # every callback if the source keeps sending malformed data.
            self.get_logger().warning(
                "Failed to decode incoming camera frame (cv2.imdecode/imgmsg_to_cv2 "
                "returned None) - dropping this frame.",
                throttle_duration_sec=5.0,
            )
            return

        # Queue for heavy inference on every `frame_skip` frames (or instantly when worker is idle if frame_skip=1)
        if self.frame_skip <= 1 or (self._frame_count % self.frame_skip == 0):
            with self._msg_lock:
                if not self._is_inferring:
                    self._infer_msg = (undistorted_image.copy(), msg.header)
                    self._infer_event.set()

        # Get cached mask and boxes
        with self._msg_lock:
            cur_mask = self._latest_mask
            cur_boxes = self._latest_boxes
            cur_shape = self._latest_input_shape

        # 1. Publish mask image
        if cur_mask is not None:
            vis_mask = (cur_mask * 255).astype(np.uint8)
            mask_msg = cv2_to_imgmsg(vis_mask, encoding='mono8', frame_id=msg.header.frame_id, stamp=msg.header.stamp)
            self.lane_mask_image_pub.publish(mask_msg)

        # 2. Render & publish annotated image at 15 FPS smoothly
        annotated_image = undistorted_image.copy()
        if cur_mask is not None:
            draw_lane_lines(annotated_image, cur_mask)
        if cur_boxes is not None and cur_shape is not None:
            try:
                scaled_boxes = scale_detected_boxes(cur_boxes, cur_shape, annotated_image.shape)
                draw_bounding_boxes(annotated_image, scaled_boxes)
                self._publish_bounding_boxes(scaled_boxes, msg.header)
            except Exception as e:
                # Never let box scaling/publishing block the always-wanted lane overlay below.
                self.get_logger().warning(f"Error processing detection boxes: {str(e)}")

        annotated_msg = cv2_to_imgmsg(annotated_image, encoding='bgr8', frame_id=msg.header.frame_id, stamp=msg.header.stamp)
        self.annotated_image_pub.publish(annotated_msg)

    def _publish_bounding_boxes(self, scaled_boxes: torch.Tensor, header: Header):
        """Publish YOLOP's detected vehicle/obstacle boxes so downstream nodes (e.g.
        object_publisher_node) can turn them into world-frame ObjectInfo without
        re-running detection. `scaled_boxes` must already be in output-image pixel space."""
        rect_array = RectMultiArray()
        rect_array.header = header
        for bboxes_coord in scaled_boxes:
            x1, y1, x2, y2 = (float(v) for v in bboxes_coord[:4])
            rect = Rect()
            rect.x = x1
            rect.y = y1
            rect.width = x2 - x1
            rect.height = y2 - y1
            rect_array.rects.append(rect)
        self.bbox_pub.publish(rect_array)

    def _inference_worker(self):
        """Dedicated event-driven inference loop that runs immediately when a new frame is available."""
        while rclpy.ok() and self._running:
            # Wait for event without polling sleep
            if not self._infer_event.wait(timeout=0.1):
                continue
            self._infer_event.clear()

            item = None
            with self._msg_lock:
                if self._infer_msg is not None:
                    item = self._infer_msg
                    self._infer_msg = None
                    self._is_inferring = True

            if item is None:
                continue

            if self.detector is None and self.trt_runner is None:
                with self._msg_lock:
                    self._is_inferring = False
                time.sleep(0.05)
                continue

            frame, header = item
            try:
                self._run_inference(frame, header)
            except Exception as e:
                self.get_logger().warning(f"Error during inference: {str(e)}")
            finally:
                with self._msg_lock:
                    self._is_inferring = False

    def _run_inference(self, undistorted_image: np.ndarray, header: Header):
        t_start = time.perf_counter()

        h_orig, w_orig = undistorted_image.shape[:2]
        cut_y = int(h_orig * self.top_cut_ratio)

        if self.roi_mode == "crop_bottom":
            proc_image = undistorted_image[cut_y:, :]
        elif self.roi_mode == "mask_top":
            proc_image = undistorted_image.copy()
            proc_image[:cut_y, :] = 0
        else:
            proc_image = undistorted_image

        # Padded resize. auto=False (pad to the full 640x640 square) rather than
        # auto=True (minimal padding to a 32-px multiple, often a non-square
        # rectangle like 640x384 for a 16:9 camera) - export_tensorrt.py's ONNX
        # export always uses a fixed (1,3,640,640) dummy input, so a TensorRT
        # engine only accepts that exact square shape. decode_lane_line_output
        # already uses whatever ratio/padding letterbox_for_img actually
        # returns, so this is safe for the PyTorch path too.
        padded_image, (ratio_to_padded, _), (pad_x_half, pad_y_half) = letterbox_for_img(
            proc_image, new_shape=640, auto=False
        )
        normalized_tensor = self.transform(padded_image).to(self.use_device)
        input_image = normalized_tensor.half() if self.use_half_precision else normalized_tensor.float()
        input_image = input_image.unsqueeze(0)
        input_image_size = input_image.shape[2:]

        # Heavy YOLOP Inference (PyTorch or, on real hardware, a TensorRT engine)
        if self.trt_runner is not None:
            raw_detections, ll_seg_raw_outputs = self.trt_runner.infer(input_image)
            # TensorRTYOLOPRunner.infer() hands back CPU tensors (its buffers are
            # pagelocked host memory). decode_lane_line_output's interpolate and
            # decode_object_output's NMS below are the same either way, but
            # leaving these on CPU forces both onto the CPU too - measured
            # ~10-20x slower than running them on the GPU like the PyTorch path
            # already does, dwarfing the TensorRT forward pass's own speedup.
            raw_detections = raw_detections.to(self.use_device)
            ll_seg_raw_outputs = ll_seg_raw_outputs.to(self.use_device)
        else:
            with torch.no_grad():
                det_out, _da_seg, ll_seg_raw_outputs = self.detector(input_image)
                raw_detections, _features = det_out

        # Exact lane decoding
        sub_mask = self.decode_lane_line_output(
            ll_seg_raw_outputs, *input_image_size, ratio_to_padded, pad_x_half, pad_y_half
        )

        if self.roi_mode == "crop_bottom":
            ll_seg_mask = np.zeros((h_orig, w_orig), dtype=np.uint8)
            h_sub, w_sub = sub_mask.shape[:2]
            ll_seg_mask[cut_y:cut_y+h_sub, :w_sub] = sub_mask
        elif self.roi_mode == "mask_top":
            ll_seg_mask = sub_mask
            ll_seg_mask[:cut_y, :] = 0
        else:
            ll_seg_mask = sub_mask

        bbox_detections = self.decode_object_output(raw_detections)

        t_end = time.perf_counter()
        infer_duration_ms = (t_end - t_start) * 1000.0

        # Update cache atomically
        with self._msg_lock:
            self._latest_mask = ll_seg_mask
            self._latest_boxes = bbox_detections
            self._latest_input_shape = input_image_size

        if self._frame_count % 30 == 0:
            self.get_logger().info(f"[YOLOP Decimated Inference] Latency: {infer_duration_ms:.1f} ms | frame_skip={self.frame_skip}")

    def decode_lane_line_output(
        self,
        ll_seg_raw: torch.Tensor,
        height: int,
        width: int,
        ratio_to_padded: float,
        pad_x_half: float,
        pad_y_half: float
    ) -> np.ndarray:
        """Exact 2027 Bilinear Interpolation + ArgMax Lane Mask Decoding."""
        ROUNDING_ADJUSTMENT = 0.1
        top, bottom = round(pad_y_half - ROUNDING_ADJUSTMENT), round(pad_y_half + ROUNDING_ADJUSTMENT)
        left, right = round(pad_x_half - ROUNDING_ADJUSTMENT), round(pad_x_half + ROUNDING_ADJUSTMENT)
        ll_predict = ll_seg_raw[:, :, top:(height-bottom), left:(width-right)]
        ll_seg_mask_raw = torch.nn.functional.interpolate(
            ll_predict, scale_factor=int(1.0/ratio_to_padded), mode='bilinear'
        )
        _, ll_seg_map = torch.max(ll_seg_mask_raw, dim=1)
        ll_seg_mask = np.array(ll_seg_map.int().squeeze().cpu().numpy(), dtype=np.uint8)
        return ll_seg_mask

    def decode_object_output(self, raw_detections: Optional[torch.Tensor]) -> Optional[torch.Tensor]:
        """Exact 2027 Object NMS Decoding."""
        if raw_detections is None or non_max_suppression is None:
            return None
        batched_detections = non_max_suppression(
            raw_detections,
            conf_thres=self.confidence_threshold,
            iou_thres=self.iou_threshold,
            classes=None,
            agnostic=False,
        )
        return batched_detections[0] if len(batched_detections) > 0 else None

    def destroy_node(self):
        self._running = False
        self._infer_event.set()
        if hasattr(self, '_worker_thread') and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=0.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPLaneDetectorNode()
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
