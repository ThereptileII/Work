# SCRUM-255: native path proof passed; actual banner rejected

[Run 37087034276 / job 111099369055](https://github.com/ThereptileII/Work/actions/runs/37087034276/job/111099369055)
used remote `5fec1915c1245f18e47dc65e088aaf9cf1fc842e`, mapped to local
`891c5379e7f2a1614d113eed59cc7666930d7ed1`. Artifact **11261091575** was
independently downloaded: **9,264 bytes**, SHA-256
`93961274757fe34404bb5717e1cfd3a62b6e48150798f0d0290318684ff09dc1`.
All **23 ZIP entries** pass CRC; original entries are retained under `raw/`.
Four source input identities match the exact local revision with only Git CRLF
conversion. Seven separate native stdout/stderr pairs match recorded hashes.

All **23 native contracts passed, no skips**. Actual diagnostics now prove the
previous hypothesis: TEMP contains `C:\Users\RUNNER~1`, while its canonical name
contains `runneradmin`; the normal directory attribute is 16, not reparse. The
explicit GetShortPathNameW file alias refers to the same file as its long name
and has normal file attributes 32. The real Windows directory-junction ancestor
refusal also passed. The path correction is therefore verified natively.

Poedit **3.9.1 installed successfully on attempt 1**, using the reviewed
Chocolatey community provider; its retained log records a matched installer
hash and deployment to `C:\Program Files\Poedit`. All three bounded package
commands returned zero. Attempts 2 and 3 reported already installed. The valid
actual `msgfmt --version` command also returned zero each time, but printed:

```
msgfmt.exe (GNU gettext-tools) 0.26
```

The helper expected only `msgfmt` before the GNU label and rejected the literal
Windows `.exe` suffix. This is a helper validation bug, not a package/provider
failure. `msgmerge` and functional catalog proof were not reached; the whole
native job correctly remains **failed**.

The narrow correction accepts the exact current tool stem with an optional
literal `.exe` suffix. GNU label, numeric version, zero exit, known directory,
regular-file/reparse checks and identity validation are unchanged. A captured
banner regression passes; different tool names, repeated `.exe` and invented
suffixes remain rejected. Local correction: **25 tests, 23 passed, two native-only
cases skipped**. The original contracts are retained. Root will publish this
exact delta to the same short proof; no retry, full build or boat action was
performed during this audit. Native Gettext/catalog and full candidate acceptance
remain open.
