# SCRUM-277: native certificate fixture repair passed

[Run 37145287262](https://github.com/ThereptileII/Work/actions/runs/37145287262),
job `111267837693`, passed on 2026-10-03 at remote
`a191102c6f150f47980bdb1c774878483c203b99`, mapped to exact local source
`827cf9b40f57a7159614727d9ae616d3fc42c75e`.

The original eight-entry artifact `11281538973` is retained verbatim as
`original-artifact.zip`: 8,315 bytes, SHA256
`fc2f6fa6b4fa44fa87c067fa5373a8d31f43874336776e2dad6562facfc88040`.
Its eight CRCs and all three source/preflight/workflow identities were independently
verified against precise Windows CRLF checkout bytes of that revision.

All seven checks passed using the four actual AST-extracted functions. The
original index was exactly `0D0A` and OpenSSL refused it. The corrected actual
issuer started with a zero-byte database, issued a certificate row and produced
valid and expired localhost certificates. The expired certificate retains the
January 1–2, 2020 validity interval and is rejected specifically with error 10.
The retained public certificates independently reproduce valid acceptance and
expiry rejection with their explicit retained CA; no trust store is modified.

The actual native provider was `C:\Program Files\OpenSSL\bin\openssl.exe`,
OpenSSL **3.6.4**, VC-WIN64A, SHA256
`670bf9086a4f553dac14646091713614fc5ec47b94f4725023b1cf6af7838fe0`.
It is the first of the four recorded PATH candidates. This differs from the
maintained application dependency **3.5.9**: the result qualifies the certificate
fixture repair only. It does not qualify dependency producers, native Downloader/
wxCurl TLS cases, application/runtime, package or boat acceptance.

No trust-store function or server was loaded. The report confirms owned temporary
fixture cleanup; only public certificates and logs/receipt are retained. Original
full-candidate and first preflight failures remain separate preserved evidence.
No full candidate was dispatched by this audit.
