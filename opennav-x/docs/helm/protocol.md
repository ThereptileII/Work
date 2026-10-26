# SKAGER Helm protocol

The Helm Server listens on TCP 43872 on the boat network. All JSON. CORS allows
only the app's WebView origins (`http://localhost`, `https://localhost`,
`capacitor://localhost`) and pages served from the PC itself. Requests are rate
limited per address.

| Endpoint | Auth | Purpose |
|---|---|---|
| `GET /helm/v1/info` | none | `{v, server_id, name, pairing, software}` |
| `POST /helm/v1/pair` | pairing secret | Pair a phone; returns its device id |
| `GET /helm/v1/challenge` | none | Single-use nonce, 60 s |
| `POST /helm/v1/connect` | device key | Signed WebRTC offer → signed answer |

Messages that are authenticated are a fixed label and fields separated by `\n`
(names cannot contain control characters). MAC = HMAC-SHA256, base64url.

## Pairing

SKAGER's pairing window holds a random 10-byte secret **S** for five minutes,
one use, five wrong attempts. The QR code is

```
app.skager.helm://pair?v=1&id=<server id>&n=<name>&a=<ip:port,...>&k=<base32(S)>
```

The same key is shown as `ABCD-EFGH-JKLM-NPQR` with the first address.

1. Phone: `client_nonce` (16 random bytes), `proof = MAC(S, "skager-helm pair v1\n" + server_id + "\n" + client_nonce + "\n" + device_name)`.
2. Server checks the proof, creates `device_id` and `server_nonce`, derives
   `K = HKDF-SHA256(ikm=S, salt=client_nonce‖server_nonce, info="skager-helm device v1\n" + server_id + "\n" + device_id, 32)`
   and answers `{device_id, server_id, server_nonce, proof = MAC(K, "skager-helm paired v1\n" + device_id + "\n" + client_nonce + "\n" + server_nonce)}`.
3. The phone derives K and verifies the proof, so it knows it paired with the PC that showed the code.

S and K never cross the network. A recorded pairing gives nothing to guess
offline: S has 80 bits.

## Connecting

1. `GET /challenge` → `challenge`.
2. The phone makes a WebRTC offer (no ICE servers; host candidates on the boat
   network), waits for ICE gathering, and sends
   `{device_id, challenge, client_nonce, sdp, takeover, mac = MAC(K, "skager-helm offer v1\n" + device_id + "\n" + challenge + "\n" + client_nonce + "\n" + (takeover ? 1 : 0) + "\n" + sdp)}`.
3. The server answers `{sdp, mac = MAC(K, "skager-helm answer v1\n" + device_id + "\n" + challenge + "\n" + client_nonce + "\n" + answer_sdp)}`.
   Errors: `401 unpaired` (removed or unknown phone), `401 expired`,
   `409 busy {by}` (another phone; retry with `takeover`).

Both descriptions carry the DTLS certificate fingerprints, so the DTLS session
that carries everything else is authenticated to the paired phone and PC.

## Data channels (created by the phone)

- **`screen`** (ordered, reliable, binary, little endian):
  - `1 begin`: frame u32, width u16, height u16, flags u8 (1 = full frame), tiles u16
  - `2 tile`: frame u32, x u16, y u16, w u16, h u16, JPEG
  - `3 end`: frame u32, capture ms u16, encode ms u16

  The phone draws a frame when it ends and acknowledges it on `control`.
  At most two frames may be unacknowledged.
- **`input`** (ordered, reliable): `{"type":"input","event":{type, x, y, button, delta, key, text, sequence, t}}`.
  - Types: `pointer_down|pointer_move|pointer_up|wheel|key|text`.
  - `x`, `y` are normalized to the frame. `t` is the phone's monotonic ms, used for the staleness check.
  - The server answers each event with `{"type":"input_ack", sequence, accepted, reason}`.
- **`control`** (ordered, reliable, JSON):
  - Phone → PC: `ack {frame}`, `refresh`, `ping {t}`, `bye`.
  - PC → phone:
    - `hello {name, server_id, software, capabilities, pilot}`
    - `stats {fps, kbps, capture_ms, encode_ms, width, height}` every 2 s
    - `screen {available}`
    - `pilot {status}`
    - `pong {t}`
    - `bye {reason, by}`. Reasons: `taken_over`, `revoked`, `ended_on_skager`,
      `server_stopped`, `connection_lost`, `disconnected_by_phone`.

## SKAGER ↔ Helm Server (stdin/stdout)

One JSON object per line; see `tools/helm-server/control.go`.

- **Commands:**
  - `pair`, `pair_cancel`, `devices`
  - `revoke {device}`, `disconnect`
  - `name {name}`, `pilot {status}`, `quit`
- **Events:**
  - `ready {server_id, name, port, addresses}`
  - `pairing {key, link, qr {size, rows}, expires, addresses}`
  - `pairing_closed {reason}`, `paired {device}`
  - `devices {devices, connected}`
  - `session {state, device, reason}`
  - `screen {available}`
  - `input` (just before an injected touch or key)
  - `error {message}`

## Test vectors

`tools/helm-server/vectors_test.go` and `mobile/helm/tests/lib.test.ts` check
the same fixed keys and MACs in Go and TypeScript.
