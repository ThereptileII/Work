#!/usr/bin/env bash
# Signs an unsigned Google Play bundle from CI (skager-*-unsigned.aab) with the app's upload key.
# The upload key never leaves the owner's computer; Google re-signs with the app signing key.
#   sign_play_bundle.sh <unsigned.aab> <upload.jks> <password-file> [alias]
set -euo pipefail
in=$1 keystore=$2 password=$3 alias=${4:-upload}
out=${in%-unsigned.aab}.aab
[ "$in" != "$out" ] || { echo "expected a *-unsigned.aab" >&2; exit 2; }
cp "$in" "$out"
jarsigner -sigalg SHA256withRSA -digestalg SHA-256 -keystore "$keystore" -storepass:file "$password" "$out" "$alias" >/dev/null
# Not -strict: an upload key is self-signed by design, which -strict reports as an error.
jarsigner -verify "$out" | grep -q "jar verified."
sha256sum "$out"
