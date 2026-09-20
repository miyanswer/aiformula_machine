#!/usr/bin/env python3
"""
lane_navigator_node.py - 周回マップ作成 + QP レーシングライン走行ノード.

    入力: aiformula_interfaces/LaneLines (lane_detector)  - 左境界/中央線/右境界 (base_link)
          nav_msgs/Odometry (odom_imu_localizer)          - 自己位置 (odom)
    出力: geometry_msgs/Twist -> twist_mux の "mpc" 入力 (/aiformula_control/extremum_seeking_mpc/cmd_vel)
          std_msgs/String (JSON)  状態 (/aiformula_control/lane_tracker/status)
          nav_msgs/Path  記録した左右境界 / レーシングライン (odom, RViz・Web シミュレータ用)

    1 周目 (MAPPING): 中央白線の上をレーントラッキング走行しつつ, 前方 x_rec の左右境界点を
        直線は疎・カーブは密に odom 座標へ記録する. スタート地点に戻ったら 1 周完了
        (オドメトリのドリフトで判定できないときは ~/finish_mapping サービスで手動終了).
    OPTIMIZING: ループ補正したコースマップから QP で最小曲率ライン (アウト・イン・アウト) を別スレッドで計算.
        その間は中央線走行を続ける. map_save_path が指定されていればマップを JSON で保存.
    2 周目以降 (RACING): レーシングラインを追従. 白線観測をマップに合わせて自己位置を補正.

    map_load_path を指定すると保存済みマップから直接 RACING で始まる (1 周目と同じ位置・向きから発進すること).

アルゴリズム本体は oit_navigation/lane_nav/ (ROS 非依存, web_simulator/js/lane_navigator.js と同一).
"""

import json
import math
import os
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from aiformula_interfaces.msg import LaneLine, LaneLines
from common_python.workspace_paths import resolve_workspace_asset
from oit_navigation.lane_nav import (
    CourseMap, LaneNavigator, LineFit, NavigatorParams, OPTIMIZING, RACING, TrackedLines, optimize_raceline,
)
from oit_navigation.lane_nav.line_tracker import ROLES


def lane_line_to_fit(msg: LaneLine) -> Optional[LineFit]:
    if not msg.valid:
        return None
    return LineFit(np.array([msg.c0, msg.c1, msg.c2]), msg.x_min, msg.x_max,
                   len(msg.points_x), inferred=not msg.detected)


class LaneNavigatorNode(Node):
    def __init__(self):
        super().__init__("lane_navigator")
        self.params = self._load_params()
        self.nav = LaneNavigator(self.params, async_optimize=True)

        self.pose = None
        self.v = 0.0
        self.omega = 0.0
        self.s = 0.0
        self._last_odom_pose = None
        self._pending_lines: Optional[TrackedLines] = None
        self._lock = threading.Lock()
        self._opt_thread: Optional[threading.Thread] = None
        self._last_map_pub = 0.0
        self._map_saved = False

        if self.map_load_path:
            path = resolve_workspace_asset(self.map_load_path)
            with open(path) as f:
                self.nav.load_map(CourseMap.from_json(f.read()))
            self.get_logger().info(f"保存済みマップから開始: {path} ({len(self.nav.course_map.left)} 断面)")

        self.create_subscription(LaneLines, self.lane_lines_topic, self._lines_cb, 5)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 1)
        self.status_pub = self.create_publisher(String, self.status_topic, 1)
        self.left_pub = self.create_publisher(Path, self.left_boundary_topic, 1)
        self.right_pub = self.create_publisher(Path, self.right_boundary_topic, 1)
        self.raceline_pub = self.create_publisher(Path, self.raceline_topic, 1)
        self.create_service(Trigger, "~/finish_mapping", self._finish_mapping_srv)
        self.create_service(Trigger, "~/reset", self._reset_srv)
        self.create_timer(1.0 / self.control_rate, self._control)
        self._last_t: Optional[float] = None
        self.get_logger().info(
            f"lane_navigator: lines={self.lane_lines_topic} odom={self.odom_topic} -> cmd_vel={self.cmd_vel_topic}")

    # ------------------------------------------------------------------ params
    def _load_params(self) -> NavigatorParams:
        d = self.declare_parameter
        self.lane_lines_topic = d("lane_lines_topic", "/aiformula_perception/lane_detector/lane_lines").value
        self.odom_topic = d("odom_topic", "/aiformula_sensing/odom_imu_localizer/odom").value
        self.cmd_vel_topic = d("cmd_vel_topic", "/aiformula_control/extremum_seeking_mpc/cmd_vel").value
        self.status_topic = d("status_topic", "/aiformula_control/lane_tracker/status").value
        self.left_boundary_topic = d("left_boundary_topic", "/aiformula_visualization/lane_navigator/left_boundary").value
        self.right_boundary_topic = d("right_boundary_topic", "/aiformula_visualization/lane_navigator/right_boundary").value
        self.raceline_topic = d("raceline_topic", "/aiformula_visualization/target_trajectory").value
        self.map_frame_id = d("map_frame_id", "odom").value
        self.control_rate = float(d("control_rate", 20.0).value)
        self.map_save_path = str(d("map_save_path", "").value)
        self.map_load_path = str(d("map_load_path", "").value)

        p = NavigatorParams()
        def load(obj, prefix):
            for name in obj.__dataclass_fields__:
                default = getattr(obj, name)
                if isinstance(default, (int, float, bool)) and not isinstance(default, tuple):
                    val = d(f"{prefix}{name}", default).value
                    setattr(obj, name, type(default)(val))
        load(p.recorder, "recorder.")
        load(p.lap, "lap.")
        load(p.raceline, "raceline.")
        load(p.tracker, "tracker.")
        for name in ("heading_window", "lines_timeout", "stop_decel", "map_matching_gain",
                     "map_matching_max_error", "match_x_min", "match_x_max", "match_x_step",
                     "match_max_step_xy", "match_max_step_yaw", "match_min_lines"):
            default = getattr(p, name)
            setattr(p, name, type(default)(d(name, default).value))
        return p

    # ------------------------------------------------------------------ io
    def _lines_cb(self, msg: LaneLines):
        lines = {r: lane_line_to_fit(getattr(msg, r)) for r in ROLES}
        detected = {r: bool(getattr(msg, r).valid and getattr(msg, r).detected) for r in ROLES}
        with self._lock:
            self._pending_lines = TrackedLines(lines, detected, {}, {
                "left": msg.lane_width_left, "right": msg.lane_width_right})

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if self._last_odom_pose is not None:
            self.s += math.hypot(pose[0] - self._last_odom_pose[0], pose[1] - self._last_odom_pose[1])
        self._last_odom_pose = pose
        self.pose = pose
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    # ------------------------------------------------------------------ control
    def _control(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._last_t is None else min(now - self._last_t, 0.5)
        self._last_t = now
        if self.pose is None:
            self.get_logger().warning(f"オドメトリ待ち: {self.odom_topic}", throttle_duration_sec=5.0)
            return
        with self._lock:
            lines, self._pending_lines = self._pending_lines, None

        cmd = self.nav.step(now, dt, self.pose, self.v, self.omega, self.s, lines)
        self._maybe_start_optimization()

        tw = Twist()
        tw.linear.x = float(cmd.v)
        tw.angular.z = float(cmd.omega)
        self.cmd_pub.publish(tw)
        self.status_pub.publish(String(data=json.dumps(self.nav.status(), ensure_ascii=False)))
        if now - self._last_map_pub > 1.0:
            self._last_map_pub = now
            self._publish_map()

    def _maybe_start_optimization(self):
        cmap = self.nav.pending_course_map()
        if cmap is None or (self._opt_thread is not None and self._opt_thread.is_alive()):
            return
        self._save_map(cmap)

        def work():
            try:
                rl = optimize_raceline(cmap.left, cmap.right, self.params.raceline)
                self.nav.finish_optimization(rl)
                self.get_logger().info(
                    f"レーシングライン生成: {len(rl.points)} ウェイポイント, 1 周 {rl.length:.1f} m, "
                    f"ループ補正 {cmap.closure_error:.2f} m, 方位ドリフト推定 {math.degrees(self.nav.yaw_drift):.1f} deg "
                    f"({'地図に反映' if self.params.lap.yaw_drift_correction else '地図には未反映'})")
            except Exception as e:  # noqa: BLE001
                self.nav.fail(f"レーシングライン生成失敗: {e}")
                self.get_logger().error(f"レーシングライン生成失敗: {e}")

        self._opt_thread = threading.Thread(target=work, daemon=True)
        self._opt_thread.start()

    def _save_map(self, cmap: CourseMap):
        if not self.map_save_path or self._map_saved:
            return
        path = os.path.expanduser(self.map_save_path)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            f.write(cmap.to_json())
        self._map_saved = True
        self.get_logger().info(f"コースマップを保存: {path}")

    def _publish_map(self):
        m = self.nav.course_map
        samples = self.nav.recorder.samples
        left = m.left if m is not None else [s.left for s in samples]
        right = m.right if m is not None else [s.right for s in samples]
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map_frame_id)
        self.left_pub.publish(self._path(left, header))
        self.right_pub.publish(self._path(right, header))
        if self.nav.raceline is not None:
            pts = list(self.nav.raceline.points)
            self.raceline_pub.publish(self._path(pts + pts[:1], header))

    @staticmethod
    def _path(points, header) -> Path:
        path = Path(header=header)
        for x, y in points:
            ps = PoseStamped(header=header)
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    # ------------------------------------------------------------------ services
    def _finish_mapping_srv(self, _req, res):
        ok = self.nav.finish_mapping(self.pose, self.s)
        res.success = ok
        res.message = ("1 周目を終了しました. QP 計算中" if ok
                       else f"終了できません (状態 {self.nav.state}, 断面 {len(self.nav.recorder.samples)} 点)")
        return res

    def _reset_srv(self, _req, res):
        self.nav.reset()
        self._map_saved = False
        res.success = True
        res.message = "リセットしました (1 周目から)"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = LaneNavigatorNode()
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
