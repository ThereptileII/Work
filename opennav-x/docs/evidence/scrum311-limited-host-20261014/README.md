# Genuine limited-primary host probe — 2026-10-14

[Run 38615961316](https://github.com/ThereptileII/Work/actions/runs/38615961316)
used exact helper `51ef8373d43ab2a32d2f6df712f72f188f1dcb67` in a disposable
GitHub-hosted Windows account. The original artifact's source/name, SHA-256,
831-byte length and every ZIP CRC were verified; the sanitized receipt is retained
here. Ephemeral account/SID/session authentication identifiers are deliberately
omitted. The full original report remains private evidence.

The original linked FULL token was actually an impersonation token at
SecurityIdentification (level 1), supplying the missing discriminating fact for
the earlier conversion refusal. This run did not convert or modify that token.
It used the genuine LIMITED PRIMARY logon token for the fixed read-only noargs
capability child. Native child identity, authentication context, image and original
process/thread handles passed the checks before its single ResumeThread, which
returned the expected previous suspension count of one.

The child did not exit within the fixed 60-second bound. The exact outcome is
`original-capability-child-timeout`, not successful desktop or customer acceptance.
The result does not establish the cause of the timeout. Cleanup preserved uncertain
process/account/profile state for destruction with the disposable job; it did not
kill or retry the child or change desktop ACLs, token levels, privileges or UAC.
No SKAGER product, installed service, boat profile or physical equipment was used.

The helper change passed 39 focused portable probe cases and 31 existing customer
launch cases before native dispatch; independent review found no concrete source
blocker. Those results do not supersede this native refusal. Actual ordinary-user
customer/UAC/installed lifecycle qualification still needs a usable authorized
Windows session. This result does not qualify that environment or close SCRUM-311.
The separate full product candidate remains `9b12442d` in run38615517743; this later
helper neither rebuilt nor relabelled that candidate.
