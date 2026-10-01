#!/usr/bin/env python3
"""Owned loopback TLS endpoint for the real Downloader trust probe."""
import http.server
import json
from pathlib import Path
import socket
import ssl
import sys

cert, key, port_file = map(Path, sys.argv[1:4])
payload = b"SCRUM211 trusted downloader payload\n"


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def _headers(self, size=len(payload)):
        self.send_response(200)
        self.send_header("Content-Length", str(size))
        self.send_header("Content-Type", "application/octet-stream")
        self.end_headers()

    def do_HEAD(self):
        if self.path == "/redirect":
            self.send_response(302)
            self.send_header("Location", f"https://localhost:{self.server.server_port}/payload")
            self.end_headers()
        elif self.path == "/downgrade":
            self.send_response(302)
            self.send_header("Location", "http://localhost:9/payload")
            self.end_headers()
        elif self.path == "/local-file":
            self.send_response(302)
            self.send_header("Location", "file:///etc/passwd")
            self.end_headers()
        else:
            self._headers(len(payload) + (19 if self.path == "/partial" else 0))

    def do_GET(self):
        if self.path == "/redirect":
            self.send_response(302)
            self.send_header("Location", f"https://localhost:{self.server.server_port}/payload")
            self.end_headers()
            return
        if self.path == "/downgrade":
            self.send_response(302)
            self.send_header("Location", "http://localhost:9/payload")
            self.end_headers()
            return
        if self.path == "/local-file":
            self.send_response(302)
            self.send_header("Location", "file:///etc/passwd")
            self.end_headers()
            return
        if self.path == "/partial":
            self._headers(len(payload) + 19)
            self.wfile.write(payload[:8])
            self.wfile.flush()
            self.connection.shutdown(socket.SHUT_RDWR)
            return
        self._headers()
        self.wfile.write(payload)


server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(cert, key)
server.socket = context.wrap_socket(server.socket, server_side=True)
port_file.write_text(json.dumps({"port": server.server_port}), encoding="utf-8")
server.serve_forever()
