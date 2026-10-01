# curl Windows certificate-test source boundary

The locked curl 8.22.0 archive remains byte-identical to its source lock. Its
`tests/certs/genserv.pl` searches PATH for `openssl` without a Windows
extension, while the reviewed OpenSSL 3.5.9 installation supplies
`bin/openssl.exe`. The build-tree patch makes a Windows-only executable-name
selection and routes OpenSSL child output to duplicated file handles instead
of pipes. This preserves the upstream certificate and key-generation commands
while avoiding the observed x509 output-pipe deadlock. The patch helper accepts
only the locked original generator SHA-256
`d737cbe77e23e275b4fcfcec36e62d49d1d59d9d9fd0013a428b7143ee75c982`
and produces SHA-256
`4c176ec6a1556f519d6c0c02d17c40caadb9a542894fedc9f2055b7a48ce9ab3`.
Unexpected or partially changed source is refused before curl configuration.

The OpenSSL producer records the installed `bin/openssl.exe` as a hashed Win32
output. The curl producer requires that exact file and version, runs the
patched upstream generator with `test test-localhost.prm` in an empty temporary
directory, and checks the generated CA, `test-localhost` certificate and key with the
pinned executable. Its build manifest records the patch source hashes, helper
hash, executable hash/version and successful probe. Package validation checks
those records against the current producer manifest and file. The normal curl
`build-certs` target and complete upstream test target remain required.

These are source and local validation gates. Native Windows reproduction,
full integrated tests, packaging and boat acceptance must be recorded against
the exact later candidate before claiming the repair qualified.

The corrected argument follows the locked `tests/certs/Makefile.inc` certificate
list. A bounded Linux run of the patched locked source generated and verified
those outputs with host OpenSSL 3.6.4 in under one second. The earlier
`test localhost` argument also returned success locally but printed a
missing-certificate warning and created misleading `localhost.*` files; that
argument error alone does not establish the cause of the Windows probe timeout.
