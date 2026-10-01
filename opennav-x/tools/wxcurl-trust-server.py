#!/usr/bin/env python3
"""Owned loopback TLS endpoint for the actual wxCurlHTTP trust harness."""
import http.server
import json
from pathlib import Path
import ssl
import sys

mode = sys.argv[1]
cert, key, port_file = map(Path, sys.argv[2:5])
redirect_target = sys.argv[5] if len(sys.argv) > 5 else ""
payload = b"SCRUM211 wxCurlHTTP trusted payload\n"


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def _redirect(self, target):
        self.send_response(302)
        self.send_header("Location", target)
        self.end_headers()

    def _serve(self, body):
        self.send_response(200)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        if body:
            self.wfile.write(payload)

    def _handle(self, body):
        if self.path == "/redirect":
            self._redirect(f"https://localhost:{self.server.server_port}/payload")
        elif self.path == "/to-https" and redirect_target:
            self._redirect(redirect_target)
        elif self.path == "/downgrade":
            self._redirect("http://localhost:9/payload")
        elif self.path == "/local-file":
            self._redirect("file:///etc/passwd")
        else:
            self._serve(body)

    def do_GET(self):
        self._handle(True)

    def do_HEAD(self):
        self._handle(False)


server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
if mode == "tls":
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert, key)
    server.socket = context.wrap_socket(server.socket, server_side=True)
elif mode != "plain":
    raise SystemExit("server mode must be tls or plain")
port_file.write_text(json.dumps({"port": server.server_port}), encoding="utf-8")
server.serve_forever()
