# Completed contract jobs for frozen 17ab044

Read-only audit of [run 37164360050](https://github.com/ThereptileII/Work/actions/runs/37164360050), attempt **1**, exact remote head **17ab044a1e5222dc71791ac8118454219efe8734**. The run remains in progress at collection; only the two completed contract jobs are claimed here. Fresh run/jobs API reports both completed/success, and both original logs independently show checkout of that exact full SHA.

| Job | Platform | Main CTest passed / failed / skipped | Main time | Additional restart executions |
| --- | --- | --- | --- | --- |
| [111324138073](https://github.com/ThereptileII/Work/actions/runs/37164360050/job/111324138073) | Ubuntu 24.04 | **94 / 0 / 0** | 4.02 s | Same one test, ten passes; 19.31 s |
| [111324138106](https://github.com/ThereptileII/Work/actions/runs/37164360050/job/111324138106) | Windows Server 2022, MSVC Win32 Release | **91 / 0 / 0** | 6.44 s | Same one test, ten passes; 19.98 s |

Counts come from every original CTest result row and its terminal summary. The ten restart executions are not ten distinct cases and do not increase main-suite counts. `restart_lifecycle_contract` also ran once in each main suite. The three Linux-only registrations are `early_startup_product_isolation`, `early_startup_fixtures_isolation`, and `early_startup_msw_excluded_isolation`; the explicit CMake Linux boundary excludes their registration on Windows. They are not skipped Windows CTest tests. Platform-conditional workflow steps are listed separately in `audit.json` (three Linux job skips and two Windows job skips).

Other original contract receipts, kept separate from CTest counts:

- Corresponding-source/OpenSSL/curl package Python suites: 11, 9 and 17 tests, each OK on both platforms.
- Same-job receipt/evidence/reuse/staging Python suites: 15, 14, 13 and 4 tests, each OK on both platforms. These are contract fixtures; they do not authorize cross-job producer reuse.
- Peer-boundary and installer-completion Python suites: 7 and 12 tests, each OK on both platforms. Linux portable navigation-copy suite additionally reports 11 tests, OK.
- Native version-probe stream/refusal/deadline script reports passed in each platform's pwsh; Windows also reports passed in Windows PowerShell 5.1.
- Windows PowerShell 5.1 reports **44 filesystem checks** and **238 shortcut-migration checks** separately on its 64-bit and 32-bit hosts. Output-policy scripts report **48 checks** per invocation, explicitly without invoking an installer operation. These are assertion/check counts, not new test-case or installed-product counts.

The selected workflow and CMakeLists bytes match mapped local candidate `0a52a6cfe3bd3b4a1e6253d609bd9dc046016bd4` exactly; hashes are recorded. This evidence worktree starts at `1f86d993e6777c8b459cff14b0fc3059d01b242d`. The publication's larger source mapping is existing root evidence, not independently re-created by this bounded audit.

`api.json` contains selected run/jobs fields without account metadata. `audit.json` retains derived counts, source hashes, platform skips and log identities; the excerpts retain every individual main CTest name/result and original-log line reference. The numbered `*-excerpts.log` files retain exact selected primary lines; full decoded original logs remain ignored under `.local/{linux,windows}-original.log`. Their UTF-8 hashes are respectively `64ba2292fbb2191b8cafa4371fb864f4496407196dff78740848ecbd5a5c9f10` (92,938 bytes) and `d5d031fb7b6553e784e400068dde496d83db43983c663991e888efb1d4c9e001` (130,992 bytes). These hash the connector-returned decoded logs, not an unprovided raw ZIP. Both extend through job cleanup. The connector rejected individual `/actions/jobs/…` fetches; the successful run-jobs API plus same-run checkout logs supply the job binding. An exact-name `contracts` artifact query returned none; these jobs have no upload step, so no separate result artifact is claimed.

No tests or jobs were rerun, and no application build, CI mutation or boat operation was performed. **Integrated application, fixture-free package/installer, native AIS runtime and physical boat qualification remain separate gates.** These completed contracts do not qualify the still-running whole candidate or make a package eligible.
