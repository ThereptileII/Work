# Windows artifact signing interface

SCRUM-25 preparation only: this interface is not enabled in a release workflow.
No publisher certificate, private key, timestamp service or trusted tool pin is
provided by the repository. No actual signing qualification has been performed.
Public publication remains closed and Production promotion requires its existing
explicit authorization and acceptance gates.

## Operator inputs and isolated output

`tools/sign-release-artifact.ps1` requires native Windows and explicit inputs:

- `-InputFile`: absolute local path to an unsigned PE `.exe` or `.dll` (up to 2 GiB).
- `-InputSha256`: independently reviewed lowercase SHA256 of those unsigned bytes.
- `-OutputDirectory`: a new directory under an existing local parent. Existing
  output directories and receipts are rejected, including preparation outputs.
- `-CertificateThumbprint`: exact 40-hex certificate identifier in **CurrentUser/My**.
- `-PublisherSubject`: exact expected certificate subject, including organization.
- `-SignTool`: explicit absolute local `signtool.exe` path.
- `-SignToolSha256`: independently reviewed lowercase SHA256 pin for that SDK tool.
- `-TimestampUrl`: selected HTTPS RFC3161 endpoint on port 443, with a DNS hostname
  and no credentials, query or fragment. There is no default endpoint.
- `-Mode Prepare` (default) or `-Mode Sign` (explicit private-key operation).

Preparation verifies the input, tool signature and certificate prerequisites,
then makes an unchanged private copy and a receipt with `outcome: prepared`.
It does not sign, contact the TSA or establish release acceptance. Invoke Sign
with another new output directory only when certificate use is authorized.
The original input is never passed to SignTool. A read handle denies concurrent
writes/deletes to the original and pinned tool for the operation. Paths cannot
traverse reparse points, UNC/device paths or alternate data streams. The new
directory is created atomically with an inheritance-protected DACL for the
current user and SYSTEM before artifact bytes are written. Same-user malicious code and administrators
are outside this isolation boundary.

The tool selects one existing, currently valid certificate by exact thumbprint
and subject, requires a private key and an explicit Code Signing EKU, and opens
the store read-only. It does not import/export certificates, provision trust,
accept PFX/password arguments, auto-select certificates or append signatures.
The SHA1 thumbprint is only the certificate selector; artifact and RFC3161
timestamp digest arguments are SHA256. The pinned SDK executable must also have
a valid Microsoft Corporation signature and pass WinVerifyTrust.

## Verification and failure behavior

Sign mode has one attempt, with at most 120 seconds per SignTool invocation.
It signs only the new copy, then verifies all embedded signatures using default
Authenticode policy and requires timestamp verification without warnings. Every
nonzero SignTool exit code fails. It additionally requires
Get-AuthenticodeSignature to report a valid embedded Authenticode signature,
the exact selected publisher certificate, and a timestamp signer certificate.
WinVerifyTrust must return exactly zero with chain revocation checks except the
root, MD2/MD4 disabled, and cached revocation retrieval only. A machine without
usable cached revocation information fails closed; do not disable revocation
to make it pass. SignTool verification may perform network retrieval beforehand.
The supplied TSA URL is constrained; this interface does not independently
control SignTool's HTTP redirects or provision a timestamp trust root.

Only after all checks does `signing-receipt.json` contain `outcome: signed` and
the signed file SHA256. It records the input/tool pins, publisher, certificate
thumbprint, algorithms, selected TSA URL and timestamp signer thumbprint.
Keep the receipt with the exact signed bytes; it is build evidence, not an
independent trust authority. A failed operation preserves the unsigned original
and may leave a private failed copy plus `outcome: failed`. Never package a
failed or merely prepared copy. There is no automatic retry, output replacement,
upload, public publication or release modification.

## Required release ordering

1. Build exact committed source and qualify the intended signing identity/tool.
2. Sign inner PE files before computing package inventory/ownership hashes.
   Reconcile any binary hash records such as updater `build.json` with the final
   signed bytes in the authorized packaging pipeline. The current unsigned
   launcher packaging recipe is not automatically made signing-aware by this tool.
3. Build Setup from those final inner bytes; sign the new Setup copy before
   sealing its SHA256 into release policy and TUF targets metadata.
4. Qualify and retain those exact signed artifact bytes and associated source.
   Promotion copies accepted bytes. It must never re-sign an accepted package,
   refresh a timestamp or silently regenerate metadata against changed bytes.

`tools/test-sign-release-policy.ps1` checks inert policy failures, original-byte
preservation, no-overwrite receipts and exact WinTrust interop compilation. It
does not create or use a certificate. Native Windows signing with the selected
SDK tool, DACL inspection, certificate/key availability, TSA failure, revoked or
untrusted chains, timestamp validation and unchanged-input failure evidence
remain required before enabling an authorized signing workflow.

## Primary references

- [Microsoft SignTool reference](https://learn.microsoft.com/en-us/windows/win32/seccrypto/signtool):
  exact certificate selection, SHA256 digest arguments, RFC3161 timestamping,
  verification policy and nonzero warning/failure results.
- [Get-AuthenticodeSignature](https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.security/get-authenticodesignature?view=powershell-5.1):
  signature inspection; catalog preference is why embedded verification is also required.
- [WinVerifyTrust](https://learn.microsoft.com/en-us/windows/win32/api/wintrust/nf-wintrust-winverifytrust)
  and [WINTRUST_DATA](https://learn.microsoft.com/en-us/windows/win32/api/wintrust/ns-wintrust-wintrust_data):
  exact-zero success and explicit verification/revocation options.
