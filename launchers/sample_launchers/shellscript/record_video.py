#!/usr/bin/env python3
"""
record_video.py - JPEG の CompressedImage トピックを H.264 の MP4 に保存する (record_rosbag_video.sh から起動).

    python3 record_video.py --out-dir <dir> camera=/aiformula_sensing/zed_node/left_image/undistorted/compressed [...]

名前=トピック ごとに <dir>/<名前>.mp4 と <dir>/<名前>_stamps.csv を作る.
  - JPEG をデコードせず ffmpeg (libx264) にそのまま流し込むので Python 側はほぼ CPU を使わない
  - 届いた時刻をフレームの時刻にする (可変フレームレート). 取りこぼしがあっても再生速度は実時間のまま
  - <名前>_stamps.csv: フレーム番号ごとの header.stamp (ROS 時刻) と受信時刻. data の rosbag と突き合わせる用
  - 断片化 MP4 なので, 強制終了されても途中までは再生できる
Ctrl+C (SIGINT) / SIGTERM で ffmpeg の入力を閉じ, MP4 を書き終えてから終わる.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def ffmpeg_command(ffmpeg: str, out_path: str, crf: int, preset: str):
    return [
        ffmpeg, '-hide_banner', '-loglevel', 'warning', '-y',
        # 入力: 連結した JPEG. 時刻は ffmpeg が読んだ実時間 (= 届いた時刻)
        '-use_wallclock_as_timestamps', '1', '-f', 'image2pipe', '-c:v', 'mjpeg', '-i', 'pipe:0',
        # yuv420p は幅・高さが偶数である必要がある (判断パネルなど奇数サイズの画像向け)
        '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2',
        '-c:v', 'libx264', '-preset', preset, '-crf', str(crf), '-pix_fmt', 'yuv420p',
        '-vsync', '0',  # 届いた時刻のまま (フレームの複製・間引きをしない)
        '-movflags', '+frag_keyframe+empty_moov+default_base_moof',
        out_path,
    ]


class VideoFile:
    """1 本の MP4 (ffmpeg 1 プロセス) と時刻の CSV."""

    def __init__(self, ffmpeg: str, out_dir: str, name: str, crf: int, preset: str):
        self.path = os.path.join(out_dir, f'{name}.mp4')
        self.log = open(os.path.join(out_dir, f'{name}_ffmpeg.log'), 'w')
        # 別セッションで起動: Ctrl+C はこのスクリプトだけが受け, 入力を閉じて ffmpeg にきれいに書き終えさせる
        self.proc = subprocess.Popen(ffmpeg_command(ffmpeg, self.path, crf, preset), stdin=subprocess.PIPE,
                                     stdout=subprocess.DEVNULL, stderr=self.log, start_new_session=True)
        self.stamps = open(os.path.join(out_dir, f'{name}_stamps.csv'), 'w')
        self.stamps.write('frame,stamp_sec,receive_sec\n')
        self.frames = 0

    def write(self, jpeg: bytes, stamp_sec: float) -> bool:
        try:
            self.proc.stdin.write(jpeg)
        except (BrokenPipeError, ValueError):
            return False
        self.stamps.write(f'{self.frames},{stamp_sec:.6f},{time.time():.6f}\n')
        self.frames += 1
        return True

    def close(self, timeout: float = 30.0):
        try:
            self.proc.stdin.close()
        except (BrokenPipeError, ValueError):
            pass
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.stamps.close()
        self.log.close()


class VideoRecorder(Node):
    def __init__(self, targets, out_dir, ffmpeg, crf, preset):
        super().__init__('record_video')
        self.out_dir, self.ffmpeg, self.crf, self.preset = out_dir, ffmpeg, crf, preset
        self.files = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        for name, topic in targets:
            self.create_subscription(CompressedImage, topic, lambda m, n=name: self._cb(n, m), qos)
            self.get_logger().info(f'{topic} -> {os.path.join(out_dir, name + ".mp4")}')
        self.create_timer(5.0, self._report)

    def _cb(self, name, msg: CompressedImage):
        if 'jpeg' not in msg.format.lower() and 'jpg' not in msg.format.lower():
            self.get_logger().warn(f'{name}: JPEG ではない形式 ({msg.format}) は保存できません', once=True)
            return
        f = self.files.get(name)
        if f is None:  # 最初のフレームが届いたら ffmpeg を起動 (届かないトピックは空の MP4 を作らない)
            f = self.files[name] = VideoFile(self.ffmpeg, self.out_dir, name, self.crf, self.preset)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not f.write(bytes(msg.data), stamp):
            self.get_logger().error(f'{name}: ffmpeg が終了しました ({name}_ffmpeg.log を確認)', once=True)

    def _report(self):
        got = ', '.join(f'{n} {f.frames}枚' for n, f in self.files.items()) or 'まだ 1 枚も届いていません'
        self.get_logger().info(f'録画中: {got}')

    def close(self):
        for name, f in self.files.items():
            f.close()
            self.get_logger().info(f'{f.path} ({f.frames} 枚)')


def _raise_interrupt(*_):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--crf', type=int, default=23, help='H.264 の画質 (小さいほど高画質・大きい. 既定 23)')
    parser.add_argument('--preset', default='veryfast', help='libx264 の速度 (既定 veryfast)')
    parser.add_argument('targets', nargs='+', help='名前=トピック')
    args = parser.parse_args()

    ffmpeg = os.environ.get('FFMPEG') or shutil.which('ffmpeg')
    if not ffmpeg:
        print('[record_video] ffmpeg がありません: sudo apt install ffmpeg', file=sys.stderr)
        return 1
    targets = [t.split('=', 1) for t in args.targets]
    os.makedirs(args.out_dir, exist_ok=True)

    rclpy.init(args=None)
    node = VideoRecorder(targets, args.out_dir, ffmpeg, args.crf, args.preset)
    signal.signal(signal.SIGTERM, _raise_interrupt)  # record_rosbag.sh の 2 段目 (SIGTERM) でも書き終えてから終わる
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
