#!/usr/bin/env python3
"""
verification_gui.py - 検証用launch選択GUI (ブラウザ版)

「どの動画」で「どの検証パイプライン」を回すかをブラウザから選んで
`ros2 launch` を起動/停止するだけの軽量ツール。ROSノードではなく、
標準ライブラリのみ (http.server) で作った小さなWebサーバー。

コンテナ内のXvfb/noVNCにはCJKフォントが無くTkinter/RViz上の日本語表示が
文字化けするため、ホスト側ブラウザで描画することで文字化けを回避する。
noVNC (RViz用, :8080) とは別のポート (既定 :8090) で待受ける。

検証パイプライン (既存launchの組み合わせで表現):
    1. YOLO単体     (信号機検出)         -> traffic_light_video_test.launch.py
    2. YOLOP単体    (白線・走路認識)      -> yolop_video_test.launch.py
    3. YOLOP+PurePursuit (経路生成まで)   -> video_test.launch.py traffic_light:=false
    4. 統合 (YOLO+YOLOP+PurePursuit)      -> video_test.launch.py traffic_light:=true
"""

import glob
import json
import os
import signal
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

DEFAULT_PORT = int(os.environ.get("VERIFICATION_GUI_PORT", "8090"))


def _find_default_mp4_dir() -> str:
    """Find an existing mp4 folder without hardcoding one fixed layout.

    "/aiformula_machine/mp4" only exists inside this project's Docker container
    (see compose.yaml: ".:/aiformula_machine"). Running this GUI directly on
    the host, or from a workspace mounted elsewhere, needs other candidates
    checked too - including the older "/aiformula_ws" mount name.
    """
    candidates = [os.environ.get("AIFORMULA_MP4_DIR")]
    candidates.append("/aiformula_machine/mp4")
    candidates.append("/aiformula_ws/mp4")
    # This file lives at <workspace_root>/src/oit_navigation/oit_navigation/verification_gui.py
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.abspath(os.path.join(here, "..", "..", "..", "mp4")))
    candidates.append(os.path.expanduser("~/aiformula_ws/mp4"))

    for candidate in candidates:
        if candidate and os.path.isdir(candidate):
            return candidate
    return "/aiformula_machine/mp4"


DEFAULT_MP4_DIR = _find_default_mp4_dir()

# (表示ラベル, launchファイル名, 固定引数, デバイス引数名)
PIPELINES = [
    {
        "label": "① YOLO単体 (信号機検出)",
        "launch_file": "traffic_light_video_test.launch.py",
        "fixed_args": {"rviz": "true"},
        "device_arg": "device",
    },
    {
        "label": "② YOLOP単体 (白線・走路認識)",
        "launch_file": "yolop_video_test.launch.py",
        "fixed_args": {"rviz": "true"},
        "device_arg": "use_device",
    },
    {
        "label": "③ YOLOP + PurePursuit (経路生成)",
        "launch_file": "video_test.launch.py",
        "fixed_args": {"traffic_light": "false", "rviz": "true"},
        "device_arg": "use_device",
    },
    {
        "label": "④ 統合 (YOLO + YOLOP + PurePursuit)",
        "launch_file": "video_test.launch.py",
        "fixed_args": {"traffic_light": "true", "rviz": "true"},
        "device_arg": "use_device",
    },
]

DEVICES = ["cpu", "mps", "0"]


class LaunchRunner:
    """`ros2 launch` を1つだけ実行・停止・ログ収集する。"""

    def __init__(self):
        self._lock = threading.Lock()
        self.proc: subprocess.Popen | None = None
        self.label = ""
        self.logs: list[str] = []
        self.returncode = None

    def status(self):
        with self._lock:
            running = self.proc is not None and self.proc.poll() is None
            return {"running": running, "label": self.label, "returncode": self.returncode}

    def logs_since(self, offset: int):
        with self._lock:
            return self.logs[offset:], len(self.logs)

    def start(self, pipeline_index: int, video_path: str, device: str, fps: str, loop: bool):
        with self._lock:
            if self.proc is not None and self.proc.poll() is None:
                raise RuntimeError("既にlaunchが実行中です。先に停止してください。")

            pipeline = PIPELINES[pipeline_index]
            cmd = [
                "ros2", "launch", "oit_navigation", pipeline["launch_file"],
                f"video_path:={video_path}",
                f"fps:={fps}",
                f"loop:={'true' if loop else 'false'}",
                f"{pipeline['device_arg']}:={device}",
            ]
            for key, value in pipeline["fixed_args"].items():
                cmd.append(f"{key}:={value}")

            self.logs = [f"$ {' '.join(cmd)}"]
            self.label = pipeline["label"]
            self.returncode = None

            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
            proc = self.proc

        threading.Thread(target=self._pump_output, args=(proc,), daemon=True).start()
        return {"ok": True, "label": pipeline["label"]}

    def _pump_output(self, proc: subprocess.Popen):
        if proc.stdout is not None:
            for line in proc.stdout:
                with self._lock:
                    self.logs.append(line.rstrip("\n"))
        proc.wait()
        with self._lock:
            self.returncode = proc.returncode
            self.logs.append(f"[launch終了] returncode={proc.returncode}")

    def stop(self):
        with self._lock:
            proc = self.proc
        if proc is None or proc.poll() is not None:
            return {"ok": True, "already_stopped": True}
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            return {"ok": True}

        def _force_kill():
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass

        threading.Timer(10.0, _force_kill).start()
        return {"ok": True}


RUNNER = LaunchRunner()

INDEX_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>OIT Navigation 検証GUI</title>
<style>
  body { font-family: -apple-system, "Hiragino Sans", "Yu Gothic", sans-serif; background:#111827; color:#e5e7eb; margin:0; padding:24px; }
  h1 { font-size:20px; margin:0 0 16px; }
  .panel { background:#1f2937; border-radius:8px; padding:16px; margin-bottom:16px; }
  label { display:block; font-size:13px; color:#9ca3af; margin-bottom:4px; margin-top:10px; }
  select, input[type=text], input[type=number] { width:100%; box-sizing:border-box; padding:6px 8px; border-radius:4px; border:1px solid #374151; background:#111827; color:#e5e7eb; }
  .row { display:flex; gap:12px; flex-wrap:wrap; }
  .row > div { flex:1; min-width:160px; }
  .checkbox-row { display:flex; align-items:center; gap:6px; margin-top:12px; }
  button { padding:8px 16px; border-radius:4px; border:none; cursor:pointer; font-size:14px; margin-right:8px; }
  #startBtn { background:#2563eb; color:white; }
  #startBtn:disabled { background:#374151; cursor:not-allowed; }
  #stopBtn { background:#dc2626; color:white; }
  #stopBtn:disabled { background:#374151; cursor:not-allowed; }
  #reloadBtn { background:#374151; color:#e5e7eb; }
  #status { margin-left:8px; font-size:13px; color:#9ca3af; }
  #log { background:#0b0f19; border-radius:6px; padding:12px; height:420px; overflow-y:auto; white-space:pre-wrap; font-family:ui-monospace, monospace; font-size:12px; line-height:1.5; }
</style>
</head>
<body>
  <h1>OIT Navigation 検証GUI</h1>

  <div class="panel">
    <label for="pipeline">検証パイプライン</label>
    <select id="pipeline"></select>

    <div class="row">
      <div>
        <label for="mp4dir">動画フォルダ</label>
        <input type="text" id="mp4dir" value="{mp4_dir}">
      </div>
      <div style="flex:0 0 auto; align-self:flex-end;">
        <button id="reloadBtn">再読込</button>
      </div>
    </div>

    <label for="video">検証動画</label>
    <select id="video"></select>

    <div class="row">
      <div>
        <label for="device">デバイス</label>
        <select id="device"></select>
      </div>
      <div>
        <label for="fps">FPS</label>
        <input type="number" id="fps" value="15.0" step="0.5">
      </div>
    </div>

    <div class="checkbox-row">
      <input type="checkbox" id="loop" checked>
      <label for="loop" style="margin:0;">ループ再生</label>
    </div>

    <div style="margin-top:16px;">
      <button id="startBtn">起動</button>
      <button id="stopBtn" disabled>停止</button>
      <span id="status">停止中</span>
    </div>
  </div>

  <div class="panel">
    <div id="log"></div>
  </div>

<script>
let logOffset = 0;
let pollTimer = null;

async function loadPipelines() {
  const res = await fetch('/api/pipelines');
  const data = await res.json();
  const sel = document.getElementById('pipeline');
  sel.innerHTML = '';
  data.forEach((p, i) => {
    const opt = document.createElement('option');
    opt.value = i;
    opt.textContent = p.label;
    sel.appendChild(opt);
  });
}

async function loadDevices() {
  const res = await fetch('/api/devices');
  const data = await res.json();
  const sel = document.getElementById('device');
  sel.innerHTML = '';
  data.forEach(d => {
    const opt = document.createElement('option');
    opt.value = d;
    opt.textContent = d;
    sel.appendChild(opt);
  });
}

async function loadVideos() {
  const dir = document.getElementById('mp4dir').value;
  const res = await fetch('/api/videos?dir=' + encodeURIComponent(dir));
  const data = await res.json();
  const sel = document.getElementById('video');
  sel.innerHTML = '';
  data.forEach(v => {
    const opt = document.createElement('option');
    opt.value = v.path;
    opt.textContent = v.name;
    sel.appendChild(opt);
  });
}

function appendLog(lines) {
  const box = document.getElementById('log');
  const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 8;
  lines.forEach(l => { box.textContent += l + "\\n"; });
  if (atBottom) box.scrollTop = box.scrollHeight;
}

async function pollLogsAndStatus() {
  const [logRes, statusRes] = await Promise.all([
    fetch('/api/logs?since=' + logOffset),
    fetch('/api/status'),
  ]);
  const logData = await logRes.json();
  appendLog(logData.lines);
  logOffset = logData.next;

  const st = await statusRes.json();
  const statusEl = document.getElementById('status');
  const startBtn = document.getElementById('startBtn');
  const stopBtn = document.getElementById('stopBtn');
  if (st.running) {
    statusEl.textContent = '実行中: ' + st.label;
    startBtn.disabled = true;
    stopBtn.disabled = false;
  } else {
    statusEl.textContent = '停止中';
    startBtn.disabled = false;
    stopBtn.disabled = true;
  }
}

document.getElementById('reloadBtn').addEventListener('click', loadVideos);

document.getElementById('startBtn').addEventListener('click', async () => {
  const body = {
    pipeline_index: Number(document.getElementById('pipeline').value),
    video_path: document.getElementById('video').value,
    device: document.getElementById('device').value,
    fps: document.getElementById('fps').value,
    loop: document.getElementById('loop').checked,
  };
  if (!body.video_path) { alert('検証動画を選択してください'); return; }
  const res = await fetch('/api/start', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body),
  });
  if (!res.ok) {
    const err = await res.json();
    alert('起動失敗: ' + err.error);
  }
});

document.getElementById('stopBtn').addEventListener('click', async () => {
  await fetch('/api/stop', { method: 'POST' });
});

(async function init() {
  await Promise.all([loadPipelines(), loadDevices(), loadVideos()]);
  pollTimer = setInterval(pollLogsAndStatus, 1000);
})();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # ポーリングAPIでターミナルが埋まらないよう抑制

    def _send_json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)

        if path == "/":
            body = INDEX_HTML.replace("{mp4_dir}", DEFAULT_MP4_DIR).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/api/pipelines":
            self._send_json([{"label": p["label"]} for p in PIPELINES])
            return

        if path == "/api/devices":
            self._send_json(DEVICES)
            return

        if path == "/api/videos":
            mp4_dir = query.get("dir", [DEFAULT_MP4_DIR])[0]
            paths = sorted(glob.glob(os.path.join(mp4_dir, "*.mp4")))
            self._send_json([{"name": os.path.basename(p), "path": p} for p in paths])
            return

        if path == "/api/status":
            self._send_json(RUNNER.status())
            return

        if path == "/api/logs":
            since = int(query.get("since", ["0"])[0])
            lines, next_offset = RUNNER.logs_since(since)
            self._send_json({"lines": lines, "next": next_offset})
            return

        self._send_json({"error": "not found"}, status=404)

    def do_POST(self):
        if self.path == "/api/start":
            try:
                body = self._read_json_body()
                result = RUNNER.start(
                    pipeline_index=int(body["pipeline_index"]),
                    video_path=str(body["video_path"]),
                    device=str(body.get("device", "cpu")),
                    fps=str(body.get("fps", "15.0")),
                    loop=bool(body.get("loop", True)),
                )
                self._send_json(result)
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=400)
            return

        if self.path == "/api/stop":
            self._send_json(RUNNER.stop())
            return

        self._send_json({"error": "not found"}, status=404)


def main():
    server = ThreadingHTTPServer(("0.0.0.0", DEFAULT_PORT), Handler)
    print(f"[verification_gui] http://localhost:{DEFAULT_PORT} をホスト側ブラウザで開いてください")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        RUNNER.stop()
        server.shutdown()


if __name__ == "__main__":
    main()
