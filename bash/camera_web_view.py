#!/usr/bin/env python3
"""
camera_web_view.py - カメラ映像をブラウザで確認するための軽量 MJPEG ビューア

Jetson イメージ (docker/Dockerfile.jetson) はヘッドレス構成で rviz2 /
rqt_image_view を持たないため、ZED ノードが既に出している JPEG 圧縮トピック
(sensor_msgs/CompressedImage, image_transport の "compressed") をそのまま
multipart MJPEG として HTTP 配信する。再エンコードしないので Jetson の負荷は
ほぼゼロ。追加の依存パッケージも不要 (rclpy + 標準ライブラリのみ)。

    python3 bash/camera_web_view.py [--topic TOPIC ...] [--port 8091]

ブラウザで http://<JetsonのIP>:8091/ を開く。複数 --topic を渡すと並べて表示。
"""
import argparse
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

DEFAULT_TOPICS = [
    "/aiformula_sensing/zed_node/rgb/image_rect_color/compressed",
]


class FrameStore:
    """トピックごとの最新 JPEG フレームと受信数を保持する。"""

    def __init__(self, topics):
        self.cond = threading.Condition()
        self.frames = {t: None for t in topics}
        self.counts = {t: 0 for t in topics}

    def put(self, topic, data):
        with self.cond:
            self.frames[topic] = data
            self.counts[topic] += 1
            self.cond.notify_all()


class ViewerNode(Node):
    def __init__(self, topics, store):
        super().__init__("camera_web_view")
        self._subs = []
        for topic in topics:
            self._subs.append(self.create_subscription(
                CompressedImage, topic,
                lambda msg, t=topic: self._on_image(t, msg),
                qos_profile_sensor_data))
            self.get_logger().info(f"subscribed: {topic}")
        self._store = store

    def _on_image(self, topic, msg):
        if "jpeg" not in msg.format and "jpg" not in msg.format:
            self.get_logger().warn(
                f"{topic}: unsupported format '{msg.format}' (JPEG only)",
                throttle_duration_sec=5.0)
            return
        self._store.put(topic, bytes(msg.data))


def make_handler(topics, store):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                self._index()
            elif self.path.startswith("/stream/"):
                self._stream(self._topic_from_path("/stream/"))
            elif self.path.startswith("/snapshot/"):
                self._snapshot(self._topic_from_path("/snapshot/"))
            else:
                self.send_error(404)

        def _topic_from_path(self, prefix):
            try:
                return topics[int(self.path[len(prefix):].split(".")[0])]
            except (ValueError, IndexError):
                return None

        def _index(self):
            items = "".join(
                f'<figure><img src="/stream/{i}"><figcaption>{t} '
                f'(<a href="/snapshot/{i}.jpg">snapshot</a>)</figcaption></figure>'
                for i, t in enumerate(topics))
            body = (
                "<!doctype html><meta charset=utf-8><title>Camera View</title>"
                "<style>body{background:#111;color:#ddd;font-family:sans-serif;margin:16px}"
                "img{max-width:100%;background:#000}figure{margin:0 0 16px}</style>"
                f"{items}").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _snapshot(self, topic):
            if topic is None:
                return self.send_error(404)
            with store.cond:
                data = store.frames[topic]
            if data is None:
                return self.send_error(503, "no frame received yet")
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _stream(self, topic):
            if topic is None:
                return self.send_error(404)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            last = -1
            try:
                while True:
                    with store.cond:
                        store.cond.wait_for(lambda: store.counts[topic] != last, timeout=5.0)
                        if store.counts[topic] == last:
                            continue
                        last = store.counts[topic]
                        data = store.frames[topic]
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(data)}\r\n\r\n".encode()
                        + data + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", action="append",
                        help="CompressedImage (JPEG) topic. repeatable. "
                             f"default: {DEFAULT_TOPICS[0]}")
    parser.add_argument("--port", type=int, default=8091)
    args, ros_args = parser.parse_known_args()
    topics = args.topic or DEFAULT_TOPICS

    rclpy.init(args=ros_args)
    store = FrameStore(topics)
    node = ViewerNode(topics, store)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(topics, store))
    server.daemon_threads = True
    node.get_logger().info(f"open http://<this-machine-ip>:{args.port}/ in a browser")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
