# SCRUM-277: empty CA database for native trust fixtures

Run [37136712793](https://github.com/ThereptileII/Work/actions/runs/37136712793)
failed after the real production application and both native trust probes built.
OpenSSL `ca` could not parse `expired-index.txt` while creating the expired test
certificate. Original evidence is preserved separately in commit
`da19a51ccaf1d48c4d8a8f8988273dad9b5c3cbe`, under
`docs/evidence/scrum272-native-154-expired-fixture`; its downloaded artifact is
`11281990645`, SHA256
`1fa9660cbf99b68893cd96b49c6fee22d92dd9f4a4d0d39f3660f970cba43f3b`.

`Set-Content -Value ''` produced a blank database row. The correction writes an
actual zero-byte database using `File.WriteAllBytes`. Serial, policy, subjects,
SANs, certificate dates and all trust assertions remain identical. The original
trust-store script's disposable-CI restriction remains unchanged.

The dedicated `skager-certificate-fixtures.yml` workflow runs only the new
PowerShell fixture preflight on `windows-2022` (five-minute timeout). It extracts
exactly `Run`, `OpenSsl`, `New-Ca` and `New-Leaf` through the actual script's AST.
It never executes that script's top-level statements or trust/server functions.
Seven checks demonstrate the original CRLF index rejection, corrected empty
pre-issuance database, issued database row, valid localhost verification and
specific expired-certificate rejection with the unchanged January 2020 dates.

The runner's actual PATH-resolved `openssl.exe` path, hash, full version and build
information are recorded. Its version may differ from maintained OpenSSL 3.5.9;
this fixture-only gate does not qualify the maintained producer or full native
TLS path. It builds no application/dependency and imports no CA into a trust
store. Generated private keys stay in an owned temporary directory which is
removed; only public certificates, source identities, logs and a result receipt
are uploaded. Failure evidence is uploaded with `always()`.

Local validation: both PowerShell files parse; the four actual AST-extracted
helpers ran on Linux under PowerShell with real OpenSSL 3.6.4. The original
newline index failed; corrected valid/expired issuance, zero-byte `.old` database
and specific verification error 10 passed. This supporting check is not native
Windows acceptance. The committed native preflight remains pending publication.
No trust assertion, CI restriction or application/runtime code was changed.

Publish this exact source to branch `skager-certificate-fixtures` with the normal
verified `opennav-x` source mapping and workflow at repository `.github/workflows`.
The workflow invokes `tools/test-downloader-certificate-fixtures.ps1` and retains
`native-certificate-fixtures-<sha>`. Do not start another full candidate until the
native fixture gate passes and its original artifact is independently audited.
