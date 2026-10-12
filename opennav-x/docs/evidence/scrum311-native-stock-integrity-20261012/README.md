# SCRUM-311 — native stock mapping custody check

Exact test/runner source: `eda928e2fee715ad3ec01e95db9e984426cac633`.
The Windows/x86 Go component executable was compiled from that source; this is
not an integrated MSVC application build or release qualification.

The defensive hypothesis was that a writable file mapping created before stock
observation could survive writer-handle close and alter bytes after a deny-write
reader was opened. The native temporary-file fixture instead observed
`ERROR_SHARING_VIOLATION` at the initial stock read open while the writable mapping
remained live. No mutation through the mapped view occurred in this run. It did
not reproduce a metadata-validation bypass and no production rehash change was
made. The regression refuses acceptance if a future environment permits the
reader and subsequently validates observed changed bytes.

## Result

All **3 exact native fixtures passed**, with no skips: existing-mapping integrity,
directory rename pin, retained cleanup failure. Discovery and execution allowlists,
exact executable SHA-256, native PowerShell parsing and evidence transfer hashes
passed. See the transfer manifest and original runner receipts/logs.

This operated only on fresh `C:\XNav\ComponentChecks\stock-before-eda928e2`
scratch and per-test temporary files. Installed OpenCPN/profile, services, data
connections and physical outputs were not changed. OpenCPN PID 1852 remained
responsive and no owned test process remained. Raw Windows line endings are
preserved.

## Limits

This result closes the investigated existing-mapping hypothesis on this native
environment. It does not prove continuous arbitrary stock-loader immutability,
service enrollment, selected-version state, authenticated startup, restoration,
or the actual packaged updater/Windows/boat release gates. SCRUM-311 stays open.
