#!/usr/bin/env python3
"""Serve the capture viewer: `plot.html` plus the endpoints it fetches.

    python3 tools/plot_server.py                      # captures in runs/, http://localhost:8765/
    python3 tools/plot_server.py --dir ~/captures     # another folder (repeat for several)

All the work is in `capture.py`; this only routes to it, so another server can import the same
functions and serve the same page.
"""
import argparse
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import capture                                                          # noqa: E402


class Handler(BaseHTTPRequestHandler):
    dirs = capture.DEFAULT_DIRS

    def _send(self, body, code=200, ctype="text/html; charset=utf-8"):
        raw = body if isinstance(body, (bytes, bytearray)) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            with open(os.path.join(HERE, "plot.html")) as f:
                return self._send(f.read())
        if path == "/meta":
            return self._send(json.dumps(capture.serve_meta(self.path, self.dirs)),
                              ctype="application/json")
        if path == "/raw":
            p = capture.pick_capture(self.path, self.dirs)
            return self._send(capture.raw_bytes(p) if p else b"",
                              ctype="application/octet-stream")
        if path == "/data":
            return self._send(json.dumps(capture.serve_data(self.path, self.dirs)),
                              ctype="application/json")
        self._send("<h1>404</h1>", 404)

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--host", default="127.0.0.1",
                    help="address to listen on; 0.0.0.0 to reach it from another machine")
    ap.add_argument("--dir", action="append",
                    help="folder of captures to list (default: runs/); repeat for several")
    a = ap.parse_args()
    if a.dir:
        Handler.dirs = [os.path.expanduser(d) for d in a.dir]
    print(f"serving on http://localhost:{a.port}/  (captures: {', '.join(Handler.dirs)})")
    ThreadingHTTPServer((a.host, a.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
