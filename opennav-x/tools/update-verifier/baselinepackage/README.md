# Initial protected baseline handoff

`skager-baseline-package` runs on Windows against the exact portable recovery
ZIP and its extracted tree. The tree must contain every original ZIP member,
including `FILE_SHA256.json`, `installer/package.json`, `app/`, `docs/`, and the
packaged recovery profile. It reads that packaged profile only to compare the
source archive; it never reads or writes the user's OpenCPN profile.

The operator must supply the seven SHA-256 values from an independent approval
record. In particular, do not derive these values from `FILE_SHA256.json`,
`ownership.json`, the source ZIP, or this command's output while invoking it.
The public trust JSON must have no surrounding whitespace; its approved hash
is over those exact bytes, including any whitespace *inside* the JSON value.

```powershell
skager-baseline-package.exe `
  --source-archive C:\Approved\SKAGER-Beta2-Portable-Recovery.zip `
  --baseline-tree C:\Approved\SKAGER-Beta2-Portable-Recovery `
  --trust-config C:\Approved\trust.json `
  --stock-path 'C:\Program Files (x86)\OpenCPN\opencpn.exe' `
  --output-directory C:\ProtectedAcquisition `
  --client-sid S-1-5-21-123-456-789-500 `
  --generation 0123456789abcdef0123456789abcdef `
  --version 0.4.2-beta.1 `
  --commit 0123456789abcdef0123456789abcdef01234567 `
  --approved-source-sha256 <independently-approved-recovery-zip-sha256> `
  --approved-package-sha256 <independently-approved-installer-package-sha256> `
  --approved-build-sha256 <independently-approved-product-build-sha256> `
  --approved-executable-sha256 <independently-approved-app-executable-sha256> `
  --approved-inventory-sha256 <independently-approved-inventory-sha256> `
  --approved-trust-sha256 <independently-approved-trust-json-sha256> `
  --approved-stock-sha256 <independently-approved-supported-stock-sha256>
```

The command creates `approved-installed-baseline.zip` as an exact byte copy and
`client-registration-request.json` in the existing output directory. It refuses
to overwrite either file and retains partial output after an error. The caller
must protect that directory independently before invoking the native provisioner.
The printed request SHA-256 is for a **separate operator approval step** before
passing the request and its approved hash to `skager-update-provisioner`.
Creation, validation, or an output hash alone does not authorize enrollment,
shortcut activation, commissioning, or application launch.
