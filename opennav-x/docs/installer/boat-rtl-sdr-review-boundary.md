# Boat RTL-SDR receiver: provenance and read-only boundary

The recorded boat receiver files now have exact maintainer-package matches.
This improves the initial quarantine review's unresolved provenance finding;
it does **not** enable the plugin, qualify a live receiver or close B1-10.
No boat connection, DLL/helper execution, configuration change or upload was
performed by this audit.

## Exact recorded identities

| Installed file | Recorded SHA-256 | Maintainer package |
| --- | --- | --- |
| `rtlsdr_pi.dll` (208,384 bytes) | `404e69931034140fbda0179964d0a29077a46b78a441c05fc68a4ee0616a64e5` | `rtlsdr_pi-1.3.1-ov50-win32.exe` |
| `rtlsdr_pi/bin/rtl_ais.exe` (1,291,776 bytes) | `2ac1bb584e5b0e92ea57f7de35c7a249eaf5297aecd5acd5d28aa0a9cbe4355b` | `AIS-catcher.exe`, AIS-catcher 0.62 x64 |

The launcher DLL was extracted without execution from the
[maintainer's 1.3.1 release](https://github.com/rgleason/rtlsdr_pi/releases/tag/v1.3.1-ov50beta).
The setup SHA-256 is
`aed81ad322c47a6b17ae1bdcf3491daf82a24a7ad60b555f14c33108002f0f61`;
the release tag resolves to source
`b93834de76c73bcf9a695e3777e592651df6f61c`.

All **25** recorded files in the helper `bin` tree, including dependencies,
licenses, web extensions, README and batch file, match the
[official AIS-catcher 0.62 x64 archive](https://github.com/jvde-github/AIS-catcher/releases/tag/v0.62).
Only the main executable's filename differs. Archive SHA-256 is
`4024a609c1f14cb72738581be1faa5e81ee5e83ea3ff7e4674b7f74eda9e33e4`;
the full per-file comparison is retained privately. Source revision is `155bdabf249f835379b3bed0759f4e7657350165`.
These are package-byte matches and associated source revisions, **not** a
bit-reproducible rebuild attestation.

## Launcher behavior and compatibility

The [pinned launcher source](https://github.com/rgleason/rtlsdr_pi/blob/b93834de76c73bcf9a695e3777e592651df6f61c/src/rtlsdr_pi.cpp)
probes helper executables with `-h` in its constructor (lines 90–118), before
reading the Enabled setting. Init reads `/Settings/rtlsdr`; Enabled can start
the selected receiver with saved arguments (732–768). The timer forwards
received `!AIVDM` lines into OpenCPN's input API (262–340). No route, pilot or
actuator-command callback was found in the reviewed launcher. DeInit calls
Stop; its helper termination escalates to `wxSIGKILL` after short waits
(44–55, 151–180, 593–623). That is not proof of graceful receiver shutdown.

PE imports in the matched DLL require `wxbase312u_vc_custom.dll` and
`wxmsw312u_core_vc_custom.dll`. The recorded cold application tree contains
wx 3.2 `vc14x` libraries, not those two dependencies. This is a concrete
compatibility gap; without an actual loader log and the complete effective
Windows DLL search path, it is not a proven historical load failure.

The preserved 21,380-byte recovered INI has neither a `PlugIns/rtlsdr_pi.dll`
section nor `Settings/rtlsdr`. The saved configuration therefore does not
establish an enabled receiver or its intended arguments.

A read-only Windows PnP query at **2026-09-27 07:20 UTC** found no device matching
the usual RTL2832/RTL2838 USB identities (VID `0BDA`, PID `2832`/`2838`) or
RTL-SDR/RTL283/AIS/SDR/Bulk-In names. The raw inventory is private; no helper was
launched and no driver/device setting changed. This does not exclude a receiver
with another identity or a network AIS source, and cannot establish antenna or
reception health. A live target cannot be fabricated to close that remaining gate.

## Helper output and shutdown boundaries

The matched helper is x64 and the launcher is x86; a separate helper process
can use that combination, subject to its own exact DLL/runtime prerequisites.
The [pinned helper entry point](https://github.com/jvde-github/AIS-catcher/blob/155bdabf249f835379b3bed0759f4e7657350165/Application/Main.cpp)
enumerates SDR devices before processing even `-h` (line 310). Help avoids
starting the receive/output loops, but must not be described as avoiding all
hardware discovery. Receive operation configures the radio and reads samples;
[RTL-SDR](https://github.com/jvde-github/AIS-catcher/blob/155bdabf249f835379b3bed0759f4e7657350165/Device/RTLSDR.cpp)
and [HackRF](https://github.com/jvde-github/AIS-catcher/blob/155bdabf249f835379b3bed0759f4e7657350165/Device/HACKRF.cpp)
paths use receive APIs. Receiver settings can include a bias tee, so arbitrary
saved arguments must not be accepted as harmless.

The bundled `start.bat` enables **`-X` external community sharing**, in addition
to loopback UDP and a web viewer. It is unsuitable for a no-upload review.
The renamed installation also no longer contains that batch file's named
`AIS-catcher` executable; its presence is not evidence that it was executed.
The helper supports configurable UDP/TCP/HTTP/MQTT outputs and config files.
A future read-only receiver check must explicitly exclude external destinations,
`-X`, arbitrary config/plugin loading and output paths to marine equipment.
The pinned MSVC build excludes socketCAN/NMEA2000 support; the exact binary
also contains its unsupported-NMEA2000 error. This does not make unrestricted
network output safe.

Normal helper exit stops receiver workers and closes servers; RTL-SDR cancels
its asynchronous read and joins workers. The old plugin's forced termination
path needs separate native lifetime qualification and cannot establish that
normal cleanup occurred.

## Consequences for B1-10 and further work

Current quarantine intentionally prevents this launcher from acquiring AIS.
It cannot explain a bug report recorded **before** that quarantine. The fixed
OpenNav timezone and AIS-health defects have their own deterministic evidence;
absence of live reception remains a separate hardware/integration item.

A further receiver qualification would need current input-only connection and
USB identity evidence, exact helper/runtime hashes, a fixed receive-only
command with no external sharing or marine outputs, bounded normal shutdown,
and actual AIS arrival/source-age observation. The legacy DLL's compatibility
must be resolved separately; copying old wx libraries or simply restoring the
DLL is not justified by this audit. None of those actions is implemented here.
The existing quarantine and cold-launch guards remain unchanged.

Private evidence: `evidence/local/boat-beta2/plugin-audit/rtlsdr-followup/`
contains maintainer release/tag metadata, downloaded archives, extracted
inspection-only files, PE imports, pinned source, complete helper-tree hash
comparison and `provenance.json`. The earlier hash-bound commissioning review
has not been rewritten beneath an existing preparation record.
