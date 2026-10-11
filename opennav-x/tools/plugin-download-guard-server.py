#!/usr/bin/env python3
"""Owned loopback fixture: TLS certificates, redirects and extractable partial body."""
import http.server
import json
from pathlib import Path
import socket
import ssl
import sys
from urllib.parse import parse_qs, urlsplit

cert, key, port_file, archive = map(Path, sys.argv[1:])
payload = archive.read_bytes()
requests = port_file.with_suffix(".requests")


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def do_GET(self):
        with requests.open("a", encoding="utf-8") as log:
            log.write(self.path + "\n")
        request = urlsplit(self.path)
        if request.path == "/redirect":
            self.send_response(302)
            self.send_header("Location", parse_qs(request.query)["target"][0])
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        if request.path not in ("/valid", "/interrupted"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Length", str(len(payload) +
                         (19 if request.path == "/interrupted" else 0)))
        self.end_headers()
        # A complete extractable tar is sent even for the interrupted transfer.
        # Ignoring libcurl's failure must never install these bytes.
        self.wfile.write(payload)
        self.wfile.flush()
        if request.path == "/interrupted":
            self.connection.shutdown(socket.SHUT_RDWR)


server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
if str(cert) != "-":
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert, key)
    server.socket = context.wrap_socket(server.socket, server_side=True)
port_file.write_text(json.dumps({"port": server.server_port}), encoding="utf-8")
server.serve_forever()
