#!/usr/bin/env python3
"""Static dev server that sends the COOP/COEP headers onnxruntime-web's
threaded WASM/WebGPU backend requires (SharedArrayBuffer is gated behind
cross-origin isolation). Plain `python3 -m http.server` doesn't send these,
so the "モデル (ONNX)" detection mode's model load fails.

Run from the repository root, same as the plain http.server instructions
this replaces (web_simulator/'s vehicle mesh references reach outside this
directory, so this directory itself can't be the server root):

    python3 web_simulator/serve.py [port]   # default port: 8000
"""
import http.server
import socketserver
import sys


class CrossOriginIsolatedHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        # This server exists to iterate on the files it serves. Without this,
        # browsers heuristically cache the ES modules under js/ and keep
        # running an old copy after an edit -- which surfaces as a confusing
        # "does not provide an export named ..." error for an export that is
        # plainly there on disk, and survives opening a new tab.
        self.send_header("Cache-Control", "no-store, must-revalidate")
        super().end_headers()


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", port), CrossOriginIsolatedHandler) as httpd:
        print(f"Serving on http://localhost:{port}/ (COOP/COEP enabled)")
        httpd.serve_forever()


if __name__ == "__main__":
    main()
