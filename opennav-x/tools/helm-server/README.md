# SKAGER Helm Server (development slice)

This Go helper is the first transport slice for Jira Epic SCRUM-379. It is not
yet bundled into the Windows product. It provides:

```text
SKAGER UI / platform capture boundary
        ↓ H.264 RTP
Helm Server (Pion WebRTC + authenticated signalling)
        ↓ DTLS/SRTP + input DataChannel
browser probe / native Helm client
```

Run the local probe:

```text
/home/standard/.local/share/mise/installs/go/1.27.1/bin/go run ./cmd/helm-server \
  -addr 127.0.0.1:43872 -pairing-code 123456
```

Open `http://127.0.0.1:43872/helm/` and enter the pairing code. The optional
`-h264-file` flag is a developer-only Annex-B H.264 publisher for transport
testing; it is not a substitute for capturing the real SKAGER render surface.

Use `-tls-cert` and `-tls-key` for a non-loopback LAN listener. The server
refuses a non-loopback listener without TLS. Do not put pairing codes or tokens
in scripts, release files or logs.

Checks:

```text
/home/standard/.local/share/mise/installs/go/1.27.1/bin/go test ./...
/home/standard/.local/share/mise/installs/go/1.27.1/bin/go test -race ./...
/home/standard/.local/share/mise/installs/go/1.27.1/bin/go vet ./...
```
