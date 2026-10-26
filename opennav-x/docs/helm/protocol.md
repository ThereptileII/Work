# Helm LAN protocol (initial slice)

The server exposes three bounded endpoints on the local network:

* `GET /helm/v1/info` — server identity, pairing expiry and capabilities;
* `POST /helm/v1/pair` — consumes a short-lived pairing code and returns a
  revocable bearer session token;
* `GET /helm/v1/signal` — authenticated WebSocket signalling for a WebRTC
  offer/answer exchange.

The WebRTC connection carries:

* an H.264 video track from the SKAGER capture/encoder boundary;
* an unordered, non-retransmitted `input` DataChannel for normalized pointer
  events. Input is acknowledged and rejected after disconnect or validation
  failure.

Coordinates are normalized to `[0,1]` against the displayed frame. The server
maps them to the current SKAGER render surface; clients must not send native
desktop coordinates.

The production path must use TLS for signalling outside a loopback development
session and WebRTC's DTLS/SRTP for media. The current development server
refuses the `TLSRequired` mode until certificate provisioning is connected to
the SKAGER installation boundary.
