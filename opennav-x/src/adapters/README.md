# Hardware adapter boundary

Not implemented. Autopilot and radar belong in separate adapters with fresh
observed state, acknowledgements, timeouts and command-rate limits. Requested
state is never treated as observed state. Safe Mode must prevent adapter
creation and command transmission. No autonomous steering in the first release.
