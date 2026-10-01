"""ゴールデンデータ (test/golden/six_lane_golden.json) と Python の出力が一致することの確認. ROS 不要.
JS 側の一致は web_simulator/test/parity.html (同じ入力を JS に通す)."""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
import gen_golden as g  # noqa: E402

GOLDEN = json.load(open(g.OUT))


def _same(a, b):
    if isinstance(a, float) or isinstance(b, float):
        return a == pytest.approx(b, abs=1e-9)
    if isinstance(a, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    return a == b


def test_planner_matches_golden():
    for case in GOLDEN['planner']:
        assert _same(g.run_planner(case), case['expected']), f"speed_limit={case['speed_limit']}"


def test_compensation_matches_golden():
    for case in GOLDEN['compensation']:
        assert _same(g.run_compensation(case), case['expected'])


def test_estimator_matches_golden():
    for case in GOLDEN['estimator']:
        assert _same(g.run_estimator(case), case['expected'])


def test_effective_control_matches_golden():
    for case in GOLDEN['effective']:
        assert _same(g.core.effective_control(g.core.SixLaneParams(), case['v']), case['expected'])
