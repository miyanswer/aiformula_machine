#!/usr/bin/env python3
"""6レーン走行の JS 一致テスト用ゴールデンデータを作る (入力と Python の出力). 出力: test/golden/six_lane_golden.json

    python3 src/oit_navigation/test/gen_golden.py

Python の挙動を意図して変えたときだけ作り直す (test_six_lane_golden.py が Python の出力が変わっていないことを確かめる).
web_simulator/test/parity.html が同じ入力を JS (six_lane_planner.js) に通して一致を確かめる:
    python3 web_simulator/serve.py & -> http://localhost:8000/web_simulator/test/parity.html
"""

import importlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
core = importlib.import_module('oit_navigation.6lane.six_lane_core')
from test_six_lane import make_lines, POLICY  # noqa: E402

OUT = os.path.join(HERE, 'golden', 'six_lane_golden.json')


def line_to_json(ln):
    return {'c': list(ln.c), 'x_min': ln.x_min, 'x_max': ln.x_max, 'detected': ln.detected,
            'px': None if ln.px is None else [float(v) for v in ln.px],
            'py': None if ln.py is None else [float(v) for v in ln.py]}


def lines_to_json(lines):
    return {r: None if lines[r] is None else line_to_json(lines[r]) for r in core.ROLES}


def lines_from_json(d):
    return {r: None if d[r] is None else core.LineObs(c=d[r]['c'], x_min=d[r]['x_min'], x_max=d[r]['x_max'],
                                                      detected=d[r]['detected'], px=d[r]['px'], py=d[r]['py'])
            for r in core.ROLES}


STEP_KEYS = ('phase', 'current_lane', 'target_lane', 'v', 'omega', 'F', 'speed_limit', 'speed_scale')


def run_planner(case):
    p = core.SixLaneParams()
    pl = core.SixLanePlanner(core.LanePolicyNet.load(POLICY), p)
    pl.set_speed_limit(case['speed_limit'])
    outs = []
    for s in case['steps']:
        lines = None if s['lines'] is None else lines_from_json(s['lines'])
        st = pl.step(s['dt'], lines, s['v_meas'], [tuple(c) for c in s['cones']])
        outs.append({**{k: st.get(k) for k in STEP_KEYS}, **{'lookahead_range': st.get('lookahead_range'),
                                                       'lookahead': st.get('lookahead'), 'blocked': st.get('blocked')}})
    return outs


def run_compensation(case):
    out = core.compensate_lines(lines_from_json(case['lines']), case['v'], case['omega'], case['age'])
    xs = case['xs']
    return {r: {'y': [out[r].y_at(x) for x in xs], 'x_min': out[r].x_min, 'x_max': out[r].x_max} for r in core.ROLES}


def run_estimator(case):
    est = core.SpeedEstimator(tau=case['tau'], scale=case['scale'])
    return [est.update(s['v_wheel'], s['accel'], s['dt']) for s in case['steps']]


def build():
    rng = np.random.default_rng(3)
    cases = {'planner': [], 'compensation': [], 'estimator': [], 'effective': []}
    for lim in (1.0, 1.5, 3.0):
        steps = []
        for i in range(90):
            kappa = 0.0 if i < 30 else (0.07 if i < 60 else 0.0)
            offset = 0.0 if i < 45 else 0.4
            lines = make_lines(offset=offset, kappa=kappa, noise=0.02, rng=rng) if i != 70 else None
            cones = [[4.0, 0.6]] if 20 <= i < 40 else []
            steps.append({'dt': 1 / 15, 'v_meas': min(lim, 0.05 * i), 'lines': None if lines is None else lines_to_json(lines),
                          'cones': cones})
        case = {'speed_limit': lim, 'steps': steps}
        case['expected'] = run_planner(case)
        cases['planner'].append(case)
    for v, om, age in ((2.0, 0.0, 0.1), (1.5, 0.4, 0.2), (3.0, -0.3, 0.05)):
        case = {'lines': lines_to_json(make_lines(offset=0.3, kappa=0.05)), 'v': v, 'omega': om, 'age': age,
                'xs': [2.0, 4.0, 6.0, 8.0]}
        case['expected'] = run_compensation(case)
        cases['compensation'].append(case)
    steps = [{'v_wheel': float(1.08 * min(2.0, 0.05 * i)), 'accel': 1.0 if i < 40 else 0.0, 'dt': 0.02} for i in range(150)]
    case = {'tau': 1.0, 'scale': 0.95, 'steps': steps}
    case['expected'] = run_estimator(case)
    cases['estimator'].append(case)
    for v in (0.0, 0.3, 0.75, 1.5, 2.2, 3.0, 5.0):
        cases['effective'].append({'v': v, 'expected': core.effective_control(core.SixLaneParams(), v)})
    return cases


if __name__ == '__main__':
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        json.dump(build(), f)
    print('wrote', OUT, os.path.getsize(OUT), 'bytes')
