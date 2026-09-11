#!/usr/bin/env python3
"""
traffic_light_distance.py - バウンディングボックスの画面占有率から信号機までの距離を逆算するモジュール

考え方 (画面占有率モデル):
    実寸高さ H_real [m] の物体が距離 D [m] にあるとき、ピンホールカメラの相似則により
        bbox_height_px = f_y * H_real / D
    となる。バウンディングボックスが画面縦を占める割合 (占有率) を
        occupancy = bbox_height_px / image_height_px
    と定義すると、
        D = ( f_y / image_height_px ) * H_real / occupancy
          = distance_coeff / occupancy
    と書ける。f_y はカメラの垂直焦点距離 [pixel]。
    項 f_y / image_height_px は解像度によらず 1 / (2 * tan(VFOV / 2)) に等しい。

    distance_coeff の決め方 (優先順):
      1. distance_coeff を正の値で直接指定 (既知距離での実測キャリブレーション向け)
      2. focal_length_y [px] と reference_image_height [px] から
         distance_coeff = real_height_m * focal_length_y / reference_image_height
         (ZED X の内部パラメータ K[4]=fy をそのまま使える)
      3. vertical_fov_deg から
         distance_coeff = real_height_m / (2 * tan(vertical_fov_deg / 2))

    占有率が上がる (信号機が近い) ほど距離は小さくなる。
"""

import math
from typing import Dict, Optional

# ZED X (2.2mm レンズ / AR0234 センサ, 画素ピッチ 3.0um) の垂直焦点距離 [px]。
# f_px = focal_length_mm / pixel_pitch_mm = 2.2 / 0.003 ~= 733
ZEDX_FOCAL_LENGTH_Y_PX = 733.0
# ZED X の既定グラブ解像度 HD1080 の画像高さ [px] (HD1200 の場合は 1200)
ZEDX_REFERENCE_IMAGE_HEIGHT_PX = 1080


class TrafficLightDistanceEstimator:
    """バウンディングボックスの縦方向占有率から距離 [m] を逆算する推定器。"""

    def __init__(
        self,
        real_height_m: float = 0.32,
        focal_length_y: float = ZEDX_FOCAL_LENGTH_Y_PX,
        reference_image_height: int = ZEDX_REFERENCE_IMAGE_HEIGHT_PX,
        vertical_fov_deg: float = 0.0,
        distance_coeff: float = 0.0,
        min_occupancy: float = 1e-4,
        max_valid_distance: float = 50.0,
        smoothing_alpha: float = 0.7,
    ) -> None:
        """
        Args:
            real_height_m: 信号機の実際の縦寸法 [m] (1辺32cmの正方形規格が既定)。
            focal_length_y: カメラの垂直焦点距離 [pixel]。ZED X は既定で 733 px。
                            camera_info トピックの K[4] を使うとより正確。
            reference_image_height: focal_length_y を測った画像高さ [pixel] (ZED X HD1080=1080)。
            vertical_fov_deg: カメラの垂直画角 [deg]。>0 かつ focal_length_y<=0 のとき係数算出に使用。
            distance_coeff: 距離係数 k [m]。>0 なら最優先で使用する。
            min_occupancy: ゼロ除算・ノイズ防止用の最小占有率。
            max_valid_distance: 異常値クリップ用の最大許容距離 [m]。
            smoothing_alpha: 指数平滑化 (EMA) の重み。1.0 でフィルタ無効、0.0 で更新なし。
        """
        self.real_height_m = float(real_height_m)
        self.focal_length_y = float(focal_length_y)
        self.reference_image_height = int(reference_image_height)
        self.vertical_fov_deg = float(vertical_fov_deg)
        self.min_occupancy = float(min_occupancy)
        self.max_valid_distance = float(max_valid_distance)
        self.smoothing_alpha = float(smoothing_alpha)

        self.distance_coeff = self._resolve_distance_coeff(distance_coeff)

        # トラッキング用の距離キャッシュ {track_id: smoothed_distance}
        self._distance_cache: Dict[int, float] = {}

    def _resolve_distance_coeff(self, distance_coeff: float) -> float:
        """distance_coeff [m] を確定する (優先順: 直接指定 > 焦点距離 > 画角)。"""
        if distance_coeff and distance_coeff > 0.0:
            return float(distance_coeff)

        if self.focal_length_y and self.focal_length_y > 0.0:
            if self.reference_image_height <= 0:
                raise ValueError(
                    f"Invalid reference_image_height={self.reference_image_height}"
                )
            return self.real_height_m * self.focal_length_y / self.reference_image_height

        if self.vertical_fov_deg and self.vertical_fov_deg > 0.0:
            denom = 2.0 * math.tan(math.radians(self.vertical_fov_deg) / 2.0)
            if denom <= 1e-9:
                raise ValueError(f"Invalid vertical_fov_deg={self.vertical_fov_deg}")
            return self.real_height_m / denom

        raise ValueError(
            "distance_coeff を決定できません。distance_coeff / focal_length_y / "
            "vertical_fov_deg のいずれかを指定してください。"
        )

    def occupancy_ratio(self, bbox_height_px: float, image_height_px: int) -> Optional[float]:
        """バウンディングボックス高さと画像高さから縦方向の占有率を求める。"""
        if image_height_px <= 0 or bbox_height_px <= 0.0:
            return None
        return float(bbox_height_px) / float(image_height_px)

    def calculate_distance(self, bbox_height_px: float, image_height_px: int) -> Optional[float]:
        """占有率から距離 D [m] を直接計算する。計算不能なら None。"""
        occupancy = self.occupancy_ratio(bbox_height_px, image_height_px)
        if occupancy is None or occupancy < self.min_occupancy:
            return None

        distance = self.distance_coeff / occupancy

        if distance <= 0.0 or distance > self.max_valid_distance:
            return None
        return float(distance)

    def estimate(
        self,
        bbox_height_px: float,
        image_height_px: int,
        track_id: Optional[int] = None,
    ) -> Optional[float]:
        """
        バウンディングボックス高さ [px] と画像高さ [px] から距離 [m] を推定する。

        track_id を渡すと EMA による指数平滑化を適用する。
        """
        raw_distance = self.calculate_distance(bbox_height_px, image_height_px)
        if raw_distance is None:
            return None

        if track_id is not None and 0.0 < self.smoothing_alpha < 1.0:
            prev = self._distance_cache.get(track_id)
            if prev is not None:
                smoothed = self.smoothing_alpha * raw_distance + (1.0 - self.smoothing_alpha) * prev
            else:
                smoothed = raw_distance
            self._distance_cache[track_id] = smoothed
            return smoothed

        return raw_distance

    def clear_cache(self) -> None:
        """平滑化キャッシュをリセットする。"""
        self._distance_cache.clear()
