"""six_lane_planner_node.py の ROS 層のテスト. ROS (rclpy) が無くても動くように, ROS のモジュールを差し替えて
ノードの制御ロジック (速度上限の切替, 遅れ補償, 指令維持距離, 車速推定) を直接呼ぶ.

    python3 -m pytest src/oit_navigation/test/test_six_lane_node.py
"""

import importlib
import os
import sys
import threading
import types
from unittest import mock

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_six_lane import POLICY, core, make_lines  # noqa: E402


@pytest.fixture(scope='module')
def node_mod():
    names = ['rclpy', 'rclpy.node', 'rcl_interfaces', 'rcl_interfaces.msg', 'ament_index_python',
             'ament_index_python.packages', 'can_msgs', 'can_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
             'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'visualization_msgs',
             'visualization_msgs.msg', 'aiformula_interfaces', 'aiformula_interfaces.msg',
             'oit_navigation.utils.traffic_light_stop_ros', 'oit_navigation.utils.debug_panel',
             'oit_navigation.utils.image_util', 'oit_navigation.utils.viz_markers']
    saved = {n: sys.modules.get(n) for n in names}
    for n in names:
        sys.modules[n] = mock.MagicMock()
    sys.modules['rclpy.node'].Node = type('Node', (), {})
    try:
        yield importlib.import_module('oit_navigation.6lane.six_lane_planner_node')
    finally:
        for n, m in saved.items():
            if m is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = m
        sys.modules.pop('oit_navigation.6lane.six_lane_planner_node', None)


class FakeNode:
    """SixLanePlannerNode のメソッドを ROS 無しで呼ぶための最小のインスタンス."""

    def __new__(cls, mod, speed_limit=1.5):
        n = object.__new__(mod.SixLanePlannerNode)
        n.planner = core.SixLanePlanner(core.LanePolicyNet.load(POLICY), core.SixLaneParams())
        n.planner.set_speed_limit(speed_limit)
        n.get_logger = lambda: mock.MagicMock()
        n.speed_est = core.SpeedEstimator(tau=1.0)
        n.gate_lines = core.LatencyGate(0.5)
        n.gate_cones = core.LatencyGate(0.5)
        n.v = n.v_wheel = n.yaw_rate = 0.0
        n._imu_acc_sum, n._imu_acc_n, n._imu_t, n._last_est_t = 0.0, 0, -1e9, None
        n._lock = threading.Lock()
        return n


def test_status_json_has_speed_limit_fields(node_mod):
    pl = core.SixLanePlanner(core.LanePolicyNet.load(POLICY))
    pl.set_speed_limit(2.0)
    st = pl.step(1 / 15, make_lines(), 1.0)
    st.update({'v_wheel': 1.0, 'imu_ok': True, 'latency': 0.05})
    j = node_mod.status_json(st)
    assert j['speed_limit'] == 2.0 and j['lookahead_range'] and j['imu_ok'] is True and j['latency'] == 0.05


def test_speed_limit_topic_and_param_clamp(node_mod):
    n = FakeNode(node_mod)
    n._speed_limit_cb(types.SimpleNamespace(data=99.0))
    assert n.planner.p.v_max == core.SPEED_LIMIT_MAX
    n._speed_limit_cb(types.SimpleNamespace(data=2.0))
    assert n.planner.p.v_max == 2.0
    res = n._on_params([types.SimpleNamespace(name='speed_limit', value=1.2)])
    assert n.planner.p.v_max == 1.2 and res is not None


def test_stamp_helpers(node_mod):
    S = types.SimpleNamespace
    assert node_mod.SixLanePlannerNode._stamp_sec(S(sec=0, nanosec=0)) is None   # stamp 無し
    assert node_mod.SixLanePlannerNode._stamp_sec(S(sec=10, nanosec=500_000_000)) == pytest.approx(10.5)
    n = FakeNode(node_mod)
    assert n._age(None, 5.0) is None and n._age(4.9, 5.0) == pytest.approx(0.1)


def test_speed_estimate_uses_imu_mean_and_falls_back_without_imu(node_mod):
    n = FakeNode(node_mod)
    n.v_wheel = 1.0
    n._last_est_t = 0.0
    n._imu_acc_sum, n._imu_acc_n, n._imu_t = 2.0, 2, 0.0   # 平均 1.0 m/s^2
    n._update_speed(0.02)
    assert n.v > 0.0
    n._update_speed(5.0)                                    # IMU が途絶えた (0.3 s 超) -> 車輪速をそのまま使う
    assert n.v == pytest.approx(1.0)


def test_cones_are_dead_reckoned_between_detections(node_mod):
    n = FakeNode(node_mod)
    n.v = 2.0
    out = n._cones_now([(5.0, 0.5)], stamp=10.0, now=10.1, omega_est=0.0)   # 0.1 s 前の検出: 0.2 m 進んだ
    assert out[0] == pytest.approx((4.8, 0.5))
    assert n._cones_now([], 10.0, 10.1, 0.0) == []
