#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
if [[ -x .local/sysroot/usr/bin/cmake ]]; then
  source tools/local-env.sh
fi
mkdir -p evidence/local
python tools/verify-upstream.py | tee evidence/local/upstream-provenance.json
args=(-G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  -DOCPN_BUILD_TEST=ON -DOCPN_BUNDLE_DOCS=OFF -DOCPN_BUNDLE_GSHHS=OFF
  -DOCPN_BUNDLE_TCDATA=OFF -DCMAKE_INSTALL_PREFIX="$root/build/pristine-install")
if [[ -x .local/sysroot/usr/bin/wx-config ]]; then
  args+=(-DwxWidgets_CONFIG_EXECUTABLE="$root/tools/wx-config-local"
    -DOCPN_USE_WEBVIEW=OFF)
fi
cmake -S upstream/OpenCPN -B build/pristine-linux "${args[@]}" \
  2>&1 | tee evidence/local/linux-configure.log
cmake --build build/pristine-linux --parallel "${OPENNAV_BUILD_JOBS:-2}" \
  2>&1 | tee evidence/local/linux-build.log
cmake --install build/pristine-linux 2>&1 | tee evidence/local/linux-install.log
dbus-run-session -- ctest --test-dir build/pristine-linux --output-on-failure \
  --timeout 90 --output-junit "$root/evidence/local/linux-tests.xml" \
  2>&1 | tee evidence/local/linux-tests.log
python tools/verify-upstream.py > evidence/local/upstream-provenance-after.json
