#!/usr/bin/env python3
"""Owned loopback TLS: complete inert archive, optionally incomplete HTTP body."""
import http.server
import json
from pathlib import Path
import socket
import ssl
import sys

cert, key, port_file, archive = map(Path, sys.argv[1:])
payload = archive.read_bytes()


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def do_GET(self):
        if self.path not in ("/valid", "/interrupted"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Length", str(len(payload) +
                         (19 if self.path == "/interrupted" else 0)))
        self.end_headers()
        # Both cases deliver the complete extractable tar. The interrupted
        # response then closes before the advertised additional bytes arrive.
        self.wfile.write(payload)
        self.wfile.flush()
        if self.path == "/interrupted":
            self.connection.shutdown(socket.SHUT_RDWR)


server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(cert, key)
server.socket = context.wrap_socket(server.socket, server_side=True)
port_file.write_text(json.dumps({"port": server.server_port}), encoding="utf-8")
server.serve_forever()
