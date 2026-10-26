# Production publication and updater-feed evidence — 2026-10-25

The user explicitly authorized publication of the already qualified production candidate.

## Product release

- Release: `skager-production-0.4.0-beta2.7-run40146693249-attempt1-1d71cdd115f4`
- URL: <https://github.com/ThereptileII/Work/releases/tag/skager-production-0.4.0-beta2.7-run40146693249-attempt1-1d71cdd115f4>
- Product commit: `1d71cdd115f438aab373e7795b17e69bd7fc704e`
- Qualification run: <https://github.com/ThereptileII/Work/actions/runs/40153726621>
- Installer SHA-256: `5c0253526256ba95f33a5804d1192e82ef77df07178a06782de9a215e3a909f1`
- Portable recovery SHA-256: `3ea4921839548afc2e2583d6fb1ec455b2483d77e7db64731c31b2001a8d7214`

The release was changed from draft to published without replacing or rebuilding any asset.

## Private updater feed

- Origin: `https://omarchy.tail5878fc.ts.net`
- Repository metadata version: `1`
- Policy target SHA-256: `ed973d70c3f034b939ca1c1685c432cd7cc8cd0e65b1a137492c67c398bea357`
- Bootstrap root SHA-256: `a041ec78891a5e3d8896385ed939b691a32ed0ec11e291fb25de773754515d57`
- Transport preflight: passed; exact root bytes returned over HTTPS.
- Live go-tuf refresh: passed; `releases/beta.json` authenticated and downloaded (2,129 bytes).

The feed artifacts are the exact qualified release bytes. Signing keys remain in the local Secret Service and are not copied into the repository or evidence.

## Remaining acceptance

The final in-program Windows update sequence (startup offer, Update Now, restart and post-update version check) could not be run in this session because the boat PC's `boat` SSH endpoint timed out. This is an open environment gate, not a product-pass claim.
