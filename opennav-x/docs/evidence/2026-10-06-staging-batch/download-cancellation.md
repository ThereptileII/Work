# Cancellation racing transport EOF

Staging `47be3ddceb04a76eacea4275c4d69a8c0e43c421`,
[run 37417780924](https://github.com/ThereptileII/Work/actions/runs/37417780924),
failed the existing Linux updater cancellation case before Windows application
compilation. It reported `artifact length differs from signed size` instead of
`context canceled`. The artifact was rejected; this was not an accepted partial
download or signature bypass.

The transfer checked cancellation for transport errors and after successful
length/hash verification, but EOF can coincide with cancellation without
returning a transport error. The short length branch then hid cancellation.
The fix checks the caller's context immediately after copying and before
interpreting size/hash, preserving every rejection and cleanup boundary.

A deterministic reader cancels the context while returning partial bytes with
EOF. The new regression fails against the unchanged candidate implementation
with exactly the observed error; it passes with the fix. The complete local Go
suite passes (verifier, launcher, policy, repository and operator tests).
No existing assertion was relaxed. Fresh native and integrated qualification
remains required for the corrected source.
