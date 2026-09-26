#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
if [[ -x .local/sysroot/usr/bin/cmake ]]; then source tools/local-env.sh; fi
mkdir -p evidence/local
python tools/prepare-integration.py
args=(-G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  -DOCPN_BUILD_TEST=ON -DOCPN_BUNDLE_DOCS=OFF -DOCPN_BUNDLE_GSHHS=ON
  -DOCPN_BUNDLE_TCDATA=ON -DOPENNAV_ROOT="$root"
  -DOPENNAV_ENABLE_ROUTE_SCENARIO=ON -DXNAV_ENABLE_TEST_FIXTURES=ON
  -DCMAKE_INSTALL_PREFIX="$root/build/xnav-install")
if [[ -x .local/sysroot/usr/bin/wx-config ]]; then
  args+=(-DwxWidgets_CONFIG_EXECUTABLE="$root/tools/wx-config-local"
    -DCMAKE_PROJECT_INCLUDE="$root/tools/arch-gcc-compat.cmake"
    -DOCPN_USE_WEBVIEW=OFF)
fi
cmake -S build/integration-source -B build/xnav-linux "${args[@]}" \
  2>&1 | tee evidence/local/xnav-linux-configure.log
cmake --build build/xnav-linux --parallel "${OPENNAV_BUILD_JOBS:-2}" \
  2>&1 | tee evidence/local/xnav-linux-build.log
cmake --install build/xnav-linux 2>&1 | tee evidence/local/xnav-linux-install.log
dbus-run-session -- ctest --test-dir build/xnav-linux/test --output-on-failure \
  --no-tests=error -E '^tests$' --timeout 90 --output-junit "$root/evidence/local/xnav-linux-tests.xml" \
  2>&1 | tee evidence/local/xnav-linux-tests.log
