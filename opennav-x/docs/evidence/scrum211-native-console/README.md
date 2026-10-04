# SCRUM-211 native console probe

[Native run 37183988795, attempt 1](https://github.com/ThereptileII/Work/actions/runs/37183988795)
passed at published commit `66db1498c3690d43b31820c722032cc54584f259`.
The six source identities in the original report match local
`7efa23793be9fb8d21053ba189be0debb226ce9d` exactly after Git's LF-to-CRLF
checkout conversion, including `TrustProbeConsole.h`, the fixture and CMake
target. [validation.json](validation.json) retains both hash forms and the
20 original archive-entry identities.

Artifact `11296137043` is retained byte-for-byte as [original.zip](original.zip):
9,732 bytes, SHA256
`0972d9afaefaf2f15f8f5f4d656174d80956dc7b19956e0874d280aa7ee80f37`.
[summary.json](summary.json) is the unmodified report extracted from that ZIP;
the ZIP also retains stdout, stderr, process/window receipts and build logs.
Every retained extracted entry was compared with the original archive.

| Actual case | Result |
| --- | --- |
| Original `wxLogMessage`, without wx initialization or an explicit logger | Stopped before `after-original-log`; the five-second bound captured its own visible `#32770` window titled `Message`, containing `native-original-log`. The owned process was then terminated; exit 1 is that cleanup outcome. |
| Shared console helper with wx initialization and stderr logging | Exit 0 within five seconds. Message and warning appeared on stderr; temporary staging, writing and replacement rename completed. The runner checked exact destination bytes and absence of staging leftovers. |
| Actual assertion with the shared fail-fast handler | Exit 86 within five seconds; stderr contained the assertion detail and no `after-assert` marker. The assertion was not suppressed. |

The logs show native MSVC Win32 `/MD` compilation against the three locked
wxWidgets 3.2.8 Dev, headers and ReleaseDLL archives. The report binds the tested
executable and staged runtime DLLs by hash.

This demonstrates a concrete modal logging failure and the shared console
helper's correction. It does **not** prove that the earlier downloader timeout
reached that log statement, nor qualify TLS trust, GET/HEAD, production packaging,
the navigation application or boat operation. The replacement native candidate
must still pass its actual downloader and private wxCurl trust probes.
