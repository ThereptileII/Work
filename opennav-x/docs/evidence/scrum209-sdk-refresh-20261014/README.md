# Refreshed Windows dependency SDK — 2026-10-14

The explicitly selected producer [38618335277](https://github.com/ThereptileII/Work/actions/runs/38618335277), attempt 1, succeeds at `68c835175bdbc441e57aa1201db8f6ca95a7d1e7` on the expected hosted image `20261012.333.1`. This replaces the older-image SDK selection after the preserved early refusal in application run `38615517743`; no failed SDK or application artifact is promoted.

Original artifact `11924018896` is 117,986,124 bytes, SHA-256 `08f4fa9ca058564822ba9b69ef19aca0cbf3ea84953657d838e054709ab83ca9`. Authenticated workflow/run/attempt/job identity, full archive hash, all 296 ZIP members, 295 payload inventory entries, producer source/recipe inputs, toolchain digest, Win32 PE outputs, source archives and upstream test evidence verify. Native producer logs report OpenSSL **4283**, curl **1569**, zlib **13** passing tests. These are dependency tests, not SKAGER application tests.

All 30 SDK producer inputs and workflow bytes remain identical between the original producer Git revision and consumer `f2b59bde`. Windows checkout CRLF bytes in the original artifact were compared explicitly with their exact Git source; no verifier normalization or compatibility exception was added.

Two interrupted local downloads remain private evidence. The completed download resumed through bounded HTTP 206 ranges requiring exact Content-Range, followed by whole-file size and SHA-256 validation. The local GitHub CLI's binary-output filter was handled only for file-captured archive output; no secrets or binary payload were printed.

The committed audit records **portable custody**, not fresh native consumption. The selected dependency workflow must verify the SDK on the current Windows host before it is relied upon. The application job independently repeats authenticated restoration and native environment/tool reprobes before compilation. Actual installed customer, application package, boat and public-release acceptance remain separate and pending. No new application compilation or boat deployment is claimed here.
