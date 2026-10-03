#!/usr/bin/env bash
set -euo pipefail
prep=$(cd -- "$(dirname -- "$0")" && pwd)
: "${SKAGER_CAPTURE_PYTHON:?Set the verified Pillow-capable Python path}"
exec bwrap --die-with-parent --unshare-user --unshare-pid --ro-bind / / --proc /proc --dev /dev --tmpfs /tmp \
 --bind "$prep" "$prep" --ro-bind "$prep/fixture" "$prep/fixture" \
 --setenv SKAGER_PRIVATE_CACHE "$prep/inputs" --setenv GIT_OPTIONAL_LOCKS 0 \
 --setenv SKAGER_CAPTURE_SCRATCH "$prep/output" --chdir "$prep/output" \
 "$SKAGER_CAPTURE_PYTHON" "$prep/capture-s64.py" "$@"
