#!/usr/bin/env python3
"""
debug_panel.py - 走行判断を RViz の Image 表示で確認するためのパネル画像を描く (ROS 非依存).

RViz2 のマーカー (TEXT_VIEW_FACING) は日本語を描けないので, 日本語の判断理由は PIL で画像に描いて
sensor_msgs/Image として出す. 実機で走らせた後に rosbag を再生すれば, そのときの判断がそのまま見える.

    six_lane_panel()  : 6レーン走行 (Web シミュレータ右下の判断パネル + 俯瞰図と同じ内容)
                        俯瞰図 (白線点群・仮想6レーン・現在/目標レーン・コーン・信号) + 6レーンの確率バー + 日本語の説明
    text_panel()      : 周回マップ + QP 走行など, 文字だけのパネル

日本語フォントは下記の候補から探す (font_path パラメータで指定も可). 見つからなければ
"sudo apt install fonts-noto-cjk" を促す英語の注意だけ出し, 日本語は '?' になる.
"""

import os
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    from PIL import Image as PILImage, ImageDraw, ImageFont
except ImportError:  # pragma: no cover - PIL は ultralytics の依存で実機に入っている
    PILImage = None

FONT_CANDIDATES = (
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJKjp-Regular.otf',
    '/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/google-noto-cjk/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/fonts-japanese-gothic.ttf',
    '/usr/share/fonts/truetype/takao-gothic/TakaoPGothic.ttf',
    '/usr/share/fonts/truetype/vlgothic/VL-PGothic-Regular.ttf',
    '/usr/share/fonts/opentype/ipaexfont-gothic/ipaexg.ttf',
    '/System/Library/Fonts/ヒラギノ角ゴシック W4.ttc',
    '/System/Library/Fonts/Hiragino Sans GB.ttc',
)

# BGR
WHITE = (235, 235, 235)
GRAY = (150, 150, 150)
DIM = (90, 90, 90)
BG = (32, 30, 28)
PANEL_BG = (22, 20, 18)
GREEN = (80, 200, 80)
ORANGE = (40, 150, 255)
RED = (60, 60, 230)
YELLOW = (60, 220, 240)
CYAN = (230, 200, 60)
LINE_COLORS = {'left': (255, 170, 80), 'center': (80, 220, 255), 'right': (220, 120, 255)}


class JapaneseText:
    """PIL で日本語テキストを cv2 画像 (BGR) に描く. フォントが無ければ英数字だけ描く."""

    def __init__(self, font_path: str = '', size: int = 16):
        self.path = None
        for cand in ([font_path] if font_path else []) + list(FONT_CANDIDATES):
            if cand and os.path.exists(cand):
                self.path = cand
                break
        self._fonts = {}
        self.size = size

    @property
    def available(self) -> bool:
        return self.path is not None and PILImage is not None

    def _font(self, size: int):
        if size not in self._fonts:
            self._fonts[size] = ImageFont.truetype(self.path, size)
        return self._fonts[size]

    def draw(self, img: np.ndarray, items: Sequence[Tuple[Tuple[int, int], str, Tuple[int, int, int], int]]):
        """items: ((x, y 左上), 文字列, BGR, サイズ) のリスト. img をその場で書き換える."""
        if not items:
            return
        if not self.available:
            for (x, y), text, color, size in items:
                ascii_text = ''.join(c if ord(c) < 128 else '?' for c in text)
                cv2.putText(img, ascii_text, (x, y + size), cv2.FONT_HERSHEY_SIMPLEX, size / 30.0, color, 1, cv2.LINE_AA)
            return
        pil = PILImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        dr = ImageDraw.Draw(pil)
        for (x, y), text, color, size in items:
            dr.text((x, y), text, font=self._font(size), fill=(color[2], color[1], color[0]))
        img[:] = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)

    def wrap(self, text: str, size: int, max_width: int) -> List[str]:
        """max_width [px] に収まるよう折り返す (フォントが無いときは文字数で概算)."""
        if not self.available:
            n = max(10, int(max_width / (size * 0.55)))
            return [text[i:i + n] for i in range(0, len(text), n)] or ['']
        font = self._font(size)
        out, cur = [], ''
        for ch in text:
            if font.getlength(cur + ch) > max_width and cur:
                out.append(cur)
                cur = '　' + ch if not ch.isspace() else '　'
            else:
                cur += ch
        out.append(cur)
        return out


def _text_block(jt: JapaneseText, img, x, y, lines, width, size=15, color=WHITE, gap=4):
    """行を折り返しながら描き, 次の y を返す. lines の要素は str か (str, BGR)."""
    items = []
    for line in lines:
        text, col = (line, color) if isinstance(line, str) else line
        for part in jt.wrap(text, size, width):
            items.append(((x, y), part, col, size))
            y += size + gap
    jt.draw(img, items)
    return y


def _font_warning(jt: JapaneseText, img):
    if not jt.available:
        cv2.putText(img, 'No Japanese font: sudo apt install fonts-noto-cjk', (8, img.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, RED, 1, cv2.LINE_AA)


# ---------------------------------------------------------------------------
# 6レーン走行
# ---------------------------------------------------------------------------
class Bev:
    """俯瞰図の座標変換: base_link (x 前, y 左) -> 画像 (u 右, v 下). 車の少し後ろから前方 x_max まで."""

    def __init__(self, w: int, h: int, x_min=-1.5, x_max=12.0, y_half=5.5):
        self.w, self.h, self.x_min, self.x_max = w, h, x_min, x_max
        self.scale = min(h / (x_max - x_min), w / (2 * y_half))

    def px(self, x, y) -> Tuple[int, int]:
        return int(round(self.w / 2 - y * self.scale)), int(round(self.h - (x - self.x_min) * self.scale))


def six_lane_panel(jt: JapaneseText, lines: Optional[Dict], st: Dict, explain: List[str], lane_y_fn,
                   cones: Sequence[Tuple[float, float]] = (), tl: Optional[Dict] = None,
                   n_lanes: int = 6, bev_size=(360, 420), text_width=460) -> np.ndarray:
    """6レーン走行の判断パネル. lines: 役割 -> LineObs (px/py に点群), lane_y_fn(lines, x, F) -> y."""
    bw, bh = bev_size
    img = np.full((bh + 150, bw + text_width, 3), BG, np.uint8)
    bev_img = img[:bh, :bw]
    bev_img[:] = PANEL_BG
    bev = Bev(bw, bh)
    labels = []

    # 距離目盛り
    for x in range(0, int(bev.x_max) + 1, 2):
        u0, v = bev.px(x, 5.5)
        cv2.line(bev_img, (0, v), (bw, v), (45, 45, 45), 1)
        labels.append(((2, v - 14), f'{x}m', DIM, 11))

    cur, tgt = st.get('current_lane'), st.get('target_lane')
    blocked = set(st.get('blocked') or [])
    if lines:
        xs = np.arange(0.0, 10.01, 0.5)
        # レーンの塗り (目標=橙, 現在=緑, コーンで塞がれた=赤)
        overlay = bev_img.copy()
        for k in range(1, n_lanes + 1):
            color = RED if k in blocked else ORANGE if k == tgt else GREEN if k == cur else None
            if color is None:
                continue
            left = [bev.px(x, lane_y_fn(lines, x, k - 1)) for x in xs]
            right = [bev.px(x, lane_y_fn(lines, x, k)) for x in xs[::-1]]
            cv2.fillPoly(overlay, [np.array(left + right, np.int32)], color)
        cv2.addWeighted(overlay, 0.35, bev_img, 0.65, 0, dst=bev_img)
        # 仮想レーン境界 (白線 = F 0/3/6 は太線, 3 等分線は細線)
        for F in range(0, n_lanes + 1):
            pts = np.array([bev.px(x, lane_y_fn(lines, x, F)) for x in xs], np.int32)
            main = F % 3 == 0
            cv2.polylines(bev_img, [pts], False, WHITE if main else DIM, 2 if main else 1, cv2.LINE_AA)
        for k in range(1, n_lanes + 1):
            u, v = bev.px(7.5, lane_y_fn(lines, 7.5, k - 0.5))
            labels.append(((u - 6, v - 8), str(k), WHITE, 12))
        # 検出点群 (役割ごとの色)
        for role, obs in lines.items():
            if obs is None or getattr(obs, 'px', None) is None:
                continue
            for x, y in zip(obs.px, obs.py):
                if bev.x_min <= x <= bev.x_max:
                    cv2.circle(bev_img, bev.px(x, y), 2, LINE_COLORS.get(role, WHITE), -1)
        la = st.get('lookahead')
        if la:
            cv2.circle(bev_img, bev.px(la[0], la[1]), 5, ORANGE, 2)
    # コーン
    for cx, cy in cones:
        if bev.x_min <= cx <= bev.x_max:
            cv2.circle(bev_img, bev.px(cx, cy), 7, (0, 110, 255), -1)
            u, v = bev.px(cx, cy)
            labels.append(((u + 8, v - 8), f'{np.hypot(cx, cy):.1f}m', (0, 160, 255), 12))
    # 信号 (距離だけ分かるので正面に置く) と停止予定位置
    if tl:
        d_red, d_green = tl.get('red_recent'), tl.get('green_recent')
        for d, col, name in ((d_red, RED, '赤'), (d_green, GREEN, '青')):
            if d is not None and d <= bev.x_max:
                u, v = bev.px(d, 0.0)
                cv2.circle(bev_img, (u, v), 9, col, -1)
                labels.append(((u + 12, v - 9), f'{name} {d:.1f}m', col, 13))
        if tl.get('state') in ('APPROACH', 'STOPPED') and tl.get('distance') is not None:
            xs_stop = tl['distance'] - tl.get('stop_distance', 7.0)
            if bev.x_min <= xs_stop <= bev.x_max:
                _, v = bev.px(xs_stop, 0.0)
                cv2.line(bev_img, (0, v), (bw, v), YELLOW, 2)
                labels.append(((4, v + 2), '停止予定位置', YELLOW, 12))
    # 車体 (base_link = 駆動輪軸)
    body = np.array([bev.px(0.35, 0.35), bev.px(0.35, -0.35), bev.px(-0.9, -0.35), bev.px(-0.9, 0.35)], np.int32)
    cv2.fillPoly(bev_img, [body], (60, 60, 255))
    jt.draw(bev_img, labels)

    # 確率バー (6レーン)
    probs = st.get('probs') or [0.0] * n_lanes
    nn = st.get('nn_probs') or probs
    bar_w = (bw - 20) // n_lanes
    y0, y1 = bh + 20, bh + 122
    bar_labels = [((10, bh + 2), 'レーン確率 (白線 = NN出力, 棒 = コーン補正後)  ●現在 ★目標', GRAY, 12)]
    for i, q in enumerate(probs):
        k = i + 1
        x0 = 10 + i * bar_w
        col = RED if k in blocked else ORANGE if k == tgt else GREEN if k == cur else (110, 110, 110)
        cv2.rectangle(img, (x0 + 3, y0), (x0 + bar_w - 3, y1), (50, 50, 50), 1)
        top = int(y1 - (y1 - y0) * max(0.02, q))
        cv2.rectangle(img, (x0 + 4, top), (x0 + bar_w - 4, y1), col, -1)
        nn_y = int(y1 - (y1 - y0) * nn[i])
        cv2.line(img, (x0 + 4, nn_y), (x0 + bar_w - 4, nn_y), WHITE, 1)
        mark = ('●' if k == cur else '') + ('★' if k == tgt else '')
        bar_labels.append(((x0 + 6, top - 16 if top - 16 > y0 else y0 + 2), f'{q * 100:.0f}%', WHITE, 12))
        bar_labels.append(((x0 + 6, y1 + 4), f'L{k}{mark}', WHITE, 12))
    jt.draw(img, bar_labels)

    # 右側: 日本語の判断説明
    tx = bw + 12
    head = (f"現在 レーン{cur} (F={st.get('F', 0):.2f}) → 目標 レーン{tgt}" if cur else '現在レーン: 不明')
    text = [('6レーン判断 (ニューラルネット)', ORANGE), (head, WHITE)]
    if tl and tl.get('state') not in (None, 'NORMAL'):
        text.append((f"信号: {tl.get('reason', '')}", YELLOW))
    text += [(line, WHITE) for line in explain]
    if tl:
        text.append((f"信号検出: {tl.get('detect_text', 'なし')}", GRAY))
    text.append((f"コーン検出: {len(cones)}個" + (f" (最寄り {min(np.hypot(x, y) for x, y in cones):.1f}m)" if cones else ''), GRAY))
    _text_block(jt, img, tx, 10, text, text_width - 24)
    _font_warning(jt, img)
    return img


# ---------------------------------------------------------------------------
# 文字だけのパネル (周回マップ + QP 走行など)
# ---------------------------------------------------------------------------
def text_panel(jt: JapaneseText, title: str, lines: List, width=620, height=260) -> np.ndarray:
    img = np.full((height, width, 3), BG, np.uint8)
    _text_block(jt, img, 12, 10, [(title, ORANGE)] + list(lines), width - 24)
    _font_warning(jt, img)
    return img


def traffic_light_summary(tl_status: Optional[Dict], red_recent: Optional[float], green_recent: Optional[float]) -> Dict:
    """TrafficLightStop.status() に, 直近に見えた赤/青の距離と表示用の文字を足す."""
    tl = dict(tl_status or {})
    tl['red_recent'], tl['green_recent'] = red_recent, green_recent
    parts = []
    if red_recent is not None:
        parts.append(f'赤 {red_recent:.1f}m')
    if green_recent is not None:
        parts.append(f'青 {green_recent:.1f}m')
    tl['detect_text'] = ' / '.join(parts) if parts else 'なし'
    return tl
