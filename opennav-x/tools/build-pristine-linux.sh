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
  -DOCPN_BUILD_TEST=ON -DOCPN_BUNDLE_DOCS=OFF -DOCPN_BUNDLE_GSHHS=ON
  -DOCPN_BUNDLE_TCDATA=ON -DCMAKE_INSTALL_PREFIX="$root/build/pristine-install")
if [[ -x .local/sysroot/usr/bin/wx-config ]]; then
  args+=(-DwxWidgets_CONFIG_EXECUTABLE="$root/tools/wx-config-local"
    -DCMAKE_PROJECT_INCLUDE="$root/tools/arch-gcc-compat.cmake"
    -DOCPN_USE_WEBVIEW=OFF)
fi
cmake -S upstream/OpenCPN -B build/pristine-linux "${args[@]}" \
  2>&1 | tee evidence/local/linux-configure.log
cmake --build build/pristine-linux --parallel "${OPENNAV_BUILD_JOBS:-2}" \
  2>&1 | tee evidence/local/linux-build.log
cmake --install build/pristine-linux 2>&1 | tee evidence/local/linux-install.log
test_status=0
dbus-run-session -- ctest --test-dir build/pristine-linux/test --output-on-failure \
  --no-tests=error -E '^tests$' --timeout 90 --output-junit "$root/evidence/local/linux-tests.xml" \
  2>&1 | tee evidence/local/linux-tests.log || test_status=$?
if [[ "$test_status" -ne 0 && "$test_status" -ne 8 ]]; then exit "$test_status"; fi
python tools/check-pristine-results.py evidence/local/linux-tests.xml
python tools/verify-upstream.py > evidence/local/upstream-provenance-after.json
