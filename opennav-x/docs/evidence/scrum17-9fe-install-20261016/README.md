# SKAGER boat installation — 16 October 2026

Status: package installation is recorded; post-reboot restoration completion and fresh passive commissioning remain pending. This record covers installation and cold inspection, not runtime, physical-control, Production or public-release acceptance. It was reviewed against the retained operation receipts. Personal paths, account identifiers and navigation data are omitted.

## Exact candidate and retained CI evidence

- Source commit: `9feae59ce0ef1f54107c3c51f920085297ea7bc8`.
- Embedded version: `0.4.0-beta2.5`; channel: Staging.
- CI run: [38911440527](https://github.com/ThereptileII/Work/actions/runs/38911440527), attempt `2`; candidate ID: `0.4.0-beta2.5-run38911440527-attempt2-9feae59ce0ef`.
- Retained CI job receipts record `completed/success` for native Windows integration (`117036304896`), retained native Staging qualification (`117061245460`) and Windows x86 updater contracts (`117028247900`), all at this exact commit/run/attempt. These are CI observations, not boat acceptance.
- `QUALIFICATION.json` records Linux, Windows, installer, package and restart qualification as passed; design review as not requested; endurance as skipped; public access as false. These results do not establish boat acceptance.
- Installer SHA-256: `662821c1e3ceb87a915129c03b684bb33dd6486e8fdbee464c31766b1840dfb9`.
- Portable recovery archive SHA-256: `a20bfb2e92680a7fea9294141085d2c2145893f874d71ebb4a7ff0dffcc7b3a3`.

## Recorded installation and cold file observations

`update-once.stdout` reports `status=installed`, `launchPending=true`, and no application or hardware command launched. The subsequent cold observation at **2026-10-16 16:39:33 UTC**, before the reboot, recorded:

- Current generation: `cd6a841a0a574c9f872457e09e82c51f`.
- Installed commit/version equal the candidate above; `activeCommissioning=false`; no observed `opencpn`, `oexserverd` or `skager-start` process.
- Installed state SHA-256: `48aeb6b67a6bd310865686f7eb468672bab8dfd8e7b85be4ee8de911f761fdf2`.
- Ownership SHA-256: `efba47e21c04ea67214f3ddf59a469f55531e37c4e560a46aa57d0ca4fc76185`.
- Stock executable SHA-256 preserved: `7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`.
- Profile INI SHA-256 preserved: `7ffd9e6b59f89e2d656924f8ddaceb68cd3b826b2bba4b0e27308d774900f559`.
- RustDesk, SSH and Tailscale services observed running. Prior generation directories remained present; this is not a newly exercised rollback.

All nine cold runtime file observations equal `expected-runtime-files.json` in relative path, length and hash. These are file identities, not loaded-module or plugin Init observations:

| Package-relative file | Bytes | SHA-256 |
| --- | ---: | --- |
| `app/opencpn.exe` | 16003072 | `6cdaa8a705a3f6745ea1085b4975c2f63928d49bdd0b92d092b89ea28120a164` |
| `app/skager-ocharts-adapter.dll` | 1373184 | `47559be7f910df9f2f460c4756d867ca2d118515dadfdafb1bc731ccddb98c1b` |
| `app/libcurl.dll` | 638464 | `0a39fc592e20366dd2dfcda0d47841bce909e27da21af7a12cafa19bde9e9ef2` |
| `app/libssl-3.dll` | 1067520 | `292674cce1433331ed278837d5328d291b28e4a51f78ea550f82f9b82e3f49e9` |
| `app/libcrypto-3.dll` | 4491776 | `1c6eaa9614cc64920f4f9e4121aabb042d59b2e056306fad6b756a850d9302ce` |
| `app/plugins/chartdldr_pi.dll` | 615424 | `17e6a0abde692ad84f7a35db499d1260f4af060ade0307bd3bc1e1e38386d20a` |
| `app/plugins/dashboard_pi.dll` | 862208 | `ee45e2a2eb0334f5413de0ffa43e3dca04b8fb3f4bd3505527ce2e5a6997db02` |
| `app/plugins/grib_pi.dll` | 1338880 | `77e61b708b97c554c7d4c703c0ad87d8498eaad235d02649977590e95607b7ec` |
| `app/plugins/wmm_pi.dll` | 4570624 | `2808ba211eef1f32518348c660b25ceade5c8b953e540127b4bbdea27587d9d9` |

## Interrupted restoration and unexpected reboot

The temporary stock RTL-SDR hold restoration recorded `restore-intent.json` at **16:39:39 UTC**. The original restore invocation lost its connection without producing a completion receipt. The later desktop observation gives Windows boot time **2026-10-16 16:40:25.500 UTC**. Retained System-log evidence includes event 41 and event 6008, indicating an unclean/unexpected shutdown. No event 1074 appears in the retained query, which requested events 1074/41/6008/6006/6005 from 16:35 UTC through its observation at 16:42:25 UTC. The additional 16:43:12 UTC classification receipt records event 41 `BugcheckCode=0`, an empty result for queried System event 1001, and `desktopPresent=false`. These facts do not identify the reboot cause or establish power loss, a crash, or a clean restart. The event 6008 message's embedded prior-shutdown time is not substituted for the separately observed boot time.

The post-reboot recovery observation at **16:41:13 UTC** found the original stock `rtlsdr_pi.dll` present, 208384 bytes, SHA-256 `404e69931034140fbda0179964d0a29077a46b78a441c05fc68a4ee0616a64e5`. The held copy was absent, the restoration intent remained present, and `restored.json` was absent. This supports that the DLL had been moved back, but does not establish completed restoration validation. The exact timing of the move relative to the reboot is not independently timestamped in this observation.

The resumed restoration safely refused with: “One interactive desktop owned by the current SID is required.” The desktop observation showed the Windows logon screen and no interactive Explorer desktop. No application was observed running in the recorded cold checks before or after the reboot. Remote services were observed running after reboot. No application crash or causal link to SKAGER is established by these receipts.

## Work still pending

The operator has been asked to log back into the Windows desktop. SCRUM-17 comment `11882` records the installed result and open recovery/startup gate. After the required desktop exists, the coordinator must inspect and safely finish the recorded restoration transaction, verify the cold post-reboot state, then obtain the fresh inventory and exact plugin/passive/shutdown/audit bindings. The restored RTL-SDR DLL must receive its fresh reviewed quarantine decision before any passive start.

Fresh XNav passive startup, builtin/plugin/helper loading, real-data observation and normal shutdown/helper exit have not been completed for this installed candidate. The private review generator and draft preparation are not acceptance evidence. No physical pilot command, commissioning qualification, public opening or Production promotion is recorded here.

## Exact retained receipt identities

Paths below are relative to the private candidate evidence directory. Original receipts remain private and may contain personal paths; this sanitized record names only their relative filenames and SHA-256 values. Hashes were calculated from original retained bytes from the retained local evidence.

| Receipt | SHA-256 |
| --- | --- |
| `boat-delivery/update-once.stdout` | `9c33136f7b8872aaf8e7352b71e47a7d443ecef7f73e1d2a0347cb9bb326c648` |
| `boat-delivery/postupdate-health.stdout` | `2e90de129ec7e6c1a8d6d94b6861120dea0029eb6e5740befcf539d06735a47e` |
| `boat-delivery/expected-runtime-files.json` | `7781b81b81a5b3f090149d382730454154736bb6bfa0593dc7ab638001159172` |
| `boat-delivery/restore-recovery-observe.stdout` | `317fa1b12e9d5709dfd0d592e2cfe30f962f22bae469cf486708c5d21f0be98a` |
| `boat-delivery/cold-restore-recovery.stderr` | `a13bff8e7ae750dbb07232a53e7d2d4a49c2b229b77cbf64ec14b74dcba2a010` |
| `boat-delivery/desktop-session-observe.stdout` | `ffae62f372684760d19e5c937b03dfc9ed20f1be5f05a915421e78f7b77057a4` |
| `boat-delivery/reboot-evidence.command.ps1` | `27f2bde886d94faf0cb9fb494c5c9041a7fcad71c173a3c2e06e4c132cc42ce8` |
| `boat-delivery/reboot-evidence.stdout` | `d302191ead4629a488278dea155e2244c9e49858c94f3398ea591698d2490fbe` |
| `staging-release/RELEASE.json` | `56752567fb3bc5b76f3ee7d53eb4be60a059159db77343b975adc13de0f94ad6` |
| `staging-release/QUALIFICATION.json` | `27ee4d714d354183b6ae5404ee431306fbc08aabedb29cc34c5d4519e0b0bf53` |
| `windows-producer/job.json` | `a87b2db7acd0e4a502de86551897951435bbf1ca33ef9043bce755ebd5216c4b` |
| `windows-qualification/job.json` | `d3a7122fc0314a22c2a779959690a4f6639775dc57a41e0486df1e1a30461a8f` |
| `updater-attempt2-original/job.json` | `e7e9ae7b142f01d6e24829ebc33489961e589e2c3cf05ddb71728a6a1e032fe5` |
| `boat-delivery/reboot-classification.stdout` | `43878fa6b4284af9462e80dad1d9c32932125f6b881763004c1ab0f49d51e04e` |
