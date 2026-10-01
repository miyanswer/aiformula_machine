"""コーン回避 (lane_core/cone_avoidance.py = web_simulator/js/cone_avoidance.js) のテスト. ROS 不要.

    python3 -m pytest src/oit_navigation/test/test_cone_avoidance.py
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from oit_navigation.lane_core.cone_avoidance import ReactiveAvoider, cluster_cones  # noqa: E402


def test_reactive_steers_away_and_slows():
    av = ReactiveAvoider()
    # 進路のやや左にあるコーン -> 右へ抜ける (負のバイアス)
    for _ in range(20):
        v, w = av.step(0.0, 0.05, 1.5, 0.0, [(3.0, 0.2)])
    assert av.active and av.locked_sign == -1 and w < -0.2 and v == 1.5
    # 近い (1.8m 未満) と減速
    v, w = av.step(0.0, 0.05, 1.5, 0.0, [(1.5, 0.2)])
    assert v == pytest.approx(0.6)
    # 進路から外れたコーン (|y| >= 1.0) は回避しない, バイアスは滑らかに 0 へ戻る
    b0 = av.bias
    v, w = av.step(0.0, 0.05, 1.5, 0.0, [(3.0, 1.5)])
    assert not av.active and abs(av.bias) < abs(b0) and abs(av.bias - b0) <= 4.0 * 0.05 + 1e-9


def test_clusters_link_cones_within_two_radii():
    cl = cluster_cones([(3.0, 0.0), (4.5, 0.3), (9.0, 0.0)])
    assert sorted(len(c['cones']) for c in cl) == [1, 2]


