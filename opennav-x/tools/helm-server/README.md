# SKAGER Helm Server

`skager-helm[.exe]`: shows the SKAGER window on paired phones and tablets and
takes their touches (Jira SCRUM-379/380/381). SKAGER starts it with
`-control stdio`; it stops when SKAGER closes its stdin. See
[docs/helm](../../docs/helm/README.md) and [protocol.md](../../docs/helm/protocol.md).

| File | Role |
|---|---|
| `pairing.go`, `link.go` | Device store, pairing window, keys and MACs, the QR link and matrix, LAN addresses |
| `server.go`, `session.go` | HTTP endpoints, CORS, rate limits, the WebRTC session, input safety (staleness, release on disconnect) |
| `stream.go` | Changed-tile JPEG stream with flow control |
| `display.go`, `display_windows.go`, `display_x11.go` | Capture and input: GDI + SendInput (tagged), X11 GetImage + XTEST, and the labelled test pattern |
| `control.go` | The stdin/stdout control channel with SKAGER |

```sh
go test ./... && go test -race ./... && go vet ./... && GOOS=windows go vet ./...
go run ./cmd/helm-server -source test -state-dir /tmp/helm -pair     # development, prints the QR code
CGO_ENABLED=0 GOOS=windows go build -ldflags "-s -w -H=windowsgui" -o skager-helm.exe ./cmd/helm-server
```

`-source test` is a synthetic screen labelled TEST PATTERN, for development and
tests only; it is never SKAGER's picture. Pairing keys and device keys are never
logged.

`THIRD_PARTY_NOTICES.txt` lists the licenses of every linked module and is
installed with the server (`opennav/third-party/helm/`). Regenerate it with
`python3 gen-notices.py` after changing `go.mod`.
