# SCRUM-278: native parent-context proof passed

[Run 37155593528](https://github.com/ThereptileII/Work/actions/runs/37155593528),
attempt 1, job `111298215075`, passed on Windows Server 2022 with PowerShell 7.6.6.
The dedicated run began at 21:34:48 UTC and completed at 21:35:50 UTC on
2026-10-03. It was already terminal at the first audit query; no repeated polling,
re-execution, producer build or full workflow dispatch was performed by this audit.

Source: local `48c2f8b8d5d4103bcbaacc45a18c4e6a238d4c52` maps to executed remote
`4ddf1f383e495150551946dd36103cd77cea85eb`, tree
`82ccd6efcd61cbb6bce07bbbd3786c14729149f7`. All seven source input receipts match
that exact local source, including normal Windows checkout line endings; the
eighth input is the original verified NASM acquisition receipt. The retained
producer setup matches the exact reviewed source span and its existing hash.

Original artifact `11285546611` is 14,720 bytes, SHA-256
`8ffb32dfee1f54b8131e9e206d5ed59ffcce6bfc9f8f903e29456514b8b97593`.
This matches GitHub's artifact digest. All 25 ZIP entries passed CRC validation.
The original ZIP, extracted bytes, original job log, GitHub metadata and independent
hash/source audit are retained here. One initial plain HTTP retrieval returned
403 without bytes; the same artifact reference downloaded successfully with a
browser User-Agent. There was no second GitHub artifact download or native retry.

The actual maintained helper captured one fresh test receipt, refused omission of
only the shared Gettext/native-Perl prefixes, then accepted restoration against the
same receipt. Independent byte comparison confirms the sole negative difference
is `environment.PATHSha256`. Real Strawberry Perl, pinned local NASM 3.02, system
`tar.exe`/`cmd.exe`, PowerShell, Visual Studio and all other facts remained identical.
Captured, negative-observed and Gettext receipt hashes match their retained bytes;
the restored comparison did not refresh them. Gettext was obtained through the
existing bounded Chocolatey path and subsequently verified without replacement.

This is **native parent-context replay proof only**. It does not establish x86
child acceptance, dependency builds, AIS configure/link/TLS/lifecycle, application,
installer, boat or release acceptance. The earlier full-run failure is unchanged.
The original complete qualification gates remain mandatory.
