#!/usr/bin/env python3
"""train_policy.py - 6レーン選択 MLP を教師ルール (アウト・イン・アウト) の模倣で学習する.

    python3 src/oit_navigation/oit_navigation/6lane/train_policy.py

合成コース (直線/左右カーブを乱数でつないだ曲率プロファイル) 上のランダムな地点・速度・
横位置・検出信頼度について, 前方 近/中/遠 の観測曲率 (ノイズつき) を特徴量にし,
six_lane_core.teacher_distribution() のレーン確率をソフトラベルとしてクロスエントロピーで学習.
結果を同じフォルダの six_lane_policy.json に書き出す (実機ノードと web_simulator の共用).
numpy だけで動く (ROS 不要).
"""

import importlib.util
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('six_lane_core', os.path.join(HERE, 'six_lane_core.py'))
core = importlib.util.module_from_spec(_spec)
sys.modules['six_lane_core'] = core  # dataclass が自モジュールを参照できるように
_spec.loader.exec_module(core)

HIDDEN = 24
SEED = 7


def random_course(rng, length=4000.0, ds=0.25):
    """直線とカーブ (7割左) を交互に並べた曲率プロファイル. カーブの出入りはクロソイド的に線形遷移."""
    ks = []
    while len(ks) * ds < length:
        n_straight = int(rng.uniform(3, 40) / ds)
        ks += [rng.normal(0, 0.004)] * n_straight
        k = rng.uniform(0.03, 0.13) * (1 if rng.random() < 0.7 else -1)
        n_ramp = int(rng.uniform(1, 6) / ds)
        n_curve = int(rng.uniform(8, 45) / ds)
        ks += list(np.linspace(0, k, n_ramp)) + [k] * n_curve + list(np.linspace(k, 0, n_ramp))
    return np.array(ks), ds


def make_dataset(p, n, rng):
    ks, ds = random_course(rng)
    st = [int(x / ds) for x in p.stations]
    X, Y = [], []
    for _ in range(n):
        i = rng.integers(0, len(ks) - st[-1] - 1)
        kap = [ks[i + o] + rng.normal(0, 0.006) for o in st]
        v = rng.uniform(0.2, p.v_max * 1.07)
        F = rng.uniform(0.0, 6.0)
        conf = rng.choice([1 / 3, 2 / 3, 1.0], p=[0.15, 0.35, 0.5])
        X.append(core.features(v, kap, F, conf, p))
        Y.append(core.teacher_distribution(v, kap, F, conf, p))
    return np.array(X), np.array(Y)


def train(X, Y, rng, epochs=60, batch=256, lr=3e-3):
    d_in, d_out = X.shape[1], Y.shape[1]
    W1 = rng.normal(0, 1 / np.sqrt(d_in), (HIDDEN, d_in))
    b1 = np.zeros(HIDDEN)
    W2 = rng.normal(0, 1 / np.sqrt(HIDDEN), (d_out, HIDDEN))
    b2 = np.zeros(d_out)
    params = [W1, b1, W2, b2]
    m = [np.zeros_like(q) for q in params]
    v = [np.zeros_like(q) for q in params]
    t = 0
    for ep in range(epochs):
        order = rng.permutation(len(X))
        for s in range(0, len(X), batch):
            idx = order[s:s + batch]
            x, y = X[idx], Y[idx]
            h = np.tanh(x @ W1.T + b1)
            z = h @ W2.T + b2
            z -= z.max(axis=1, keepdims=True)
            pr = np.exp(z)
            pr /= pr.sum(axis=1, keepdims=True)
            gz = (pr - y) / len(idx)
            gW2 = gz.T @ h
            gb2 = gz.sum(0)
            gh = (gz @ W2) * (1 - h * h)
            gW1 = gh.T @ x
            gb1 = gh.sum(0)
            t += 1
            for q, g, mq, vq in zip(params, [gW1, gb1, gW2, gb2], m, v):
                mq *= 0.9
                mq += 0.1 * g
                vq *= 0.999
                vq += 0.001 * g * g
                q -= lr * (mq / (1 - 0.9 ** t)) / (np.sqrt(vq / (1 - 0.999 ** t)) + 1e-8)
        if ep % 10 == 9 or ep == epochs - 1:
            print(f'epoch {ep + 1}: {evaluate(params, X[:5000], Y[:5000])}')
    return params


def evaluate(params, X, Y):
    W1, b1, W2, b2 = params
    pr = np.tanh(X @ W1.T + b1) @ W2.T + b2
    pr = np.exp(pr - pr.max(1, keepdims=True))
    pr /= pr.sum(1, keepdims=True)
    ce = -np.mean(np.sum(Y * np.log(pr + 1e-12), 1))
    ent = -np.mean(np.sum(Y * np.log(Y + 1e-12), 1))
    acc = np.mean(pr.argmax(1) == Y.argmax(1))
    near = np.mean(np.abs(pr.argmax(1) - Y.argmax(1)) <= 1)
    return f'KL={ce - ent:.4f} argmax一致={acc * 100:.1f}% ±1レーン以内={near * 100:.1f}%'


def main():
    p = core.SixLaneParams()
    rng = np.random.default_rng(SEED)
    X, Y = make_dataset(p, 60000, rng)
    Xt, Yt = make_dataset(p, 8000, np.random.default_rng(SEED + 1))
    params = train(X, Y, rng)
    print('検証データ:', evaluate(params, Xt, Yt))
    W1, b1, W2, b2 = params
    out = {
        'description': '6レーン選択MLP (入力6 -> tanh%d -> 6). train_policy.py で教師ルール模倣学習.' % HIDDEN,
        'features': list(core.FEATURE_NAMES),
        'params': {'v_max': p.v_max, 'kappa_scale': p.kappa_scale, 'stations': list(p.stations),
                   'kappa_straight': p.kappa_straight, 'kappa_curve': p.kappa_curve, 'home_lane': p.home_lane},
        'W1': np.round(W1, 6).tolist(), 'b1': np.round(b1, 6).tolist(),
        'W2': np.round(W2, 6).tolist(), 'b2': np.round(b2, 6).tolist(),
    }
    path = os.path.join(HERE, 'six_lane_policy.json')
    with open(path, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print('wrote', path)


if __name__ == '__main__':
    main()
