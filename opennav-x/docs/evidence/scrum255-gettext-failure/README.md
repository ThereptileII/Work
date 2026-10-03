# Windows prerequisite failure, not application crash

[Run 37080314681, job 111080247315](https://github.com/ThereptileII/Work/actions/runs/37080314681/job/111080247315)
failed for published `9fcd3db54ee6913cc144ecfc09ec2152078a6eff` before application
configuration/compilation. At 00:12:49 UTC, Chocolatey's Poedit 3.9.1 package
request received HTTP 504. Pinned `win_deps.bat` continued after the failed
installation. At 01:09:09, our existing mandatory gettext check finally refused
the missing tool, after the maintained dependency work had run.

The downloaded artifact is 20,009,975 bytes, SHA-256
`e081a53c6638e20a5b852659d88fc5c8857ec9f564faaf30b507fb77e9fe6485`;
all 5,787 entries pass ZIP CRC checks. `receipt.json` records the archived log
identity, and the extracts preserve the acquisition and final refusal evidence.
The full ZIP is retained locally and in GitHub Actions artifact 11260087803.

curl reported 1,569 applicable tests passing and 503 upstream environment/feature
skips. That dependency result does not qualify the skipped application,
installer, rendering or endurance gates. No SKAGER application crash is observed
by this failure.

SCRUM-255 requires early usable msgfmt/msgmerge verification and bounded recovery
of the existing Poedit acquisition route before costly dependencies. The failure
remains recorded; there is no blind full-job rerun. The separate seventeen-unit
chart preflight for newer `9a4231f` passed and does not substitute for this job.
