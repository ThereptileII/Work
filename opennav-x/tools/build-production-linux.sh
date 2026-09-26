#!/usr/bin/env bash
# Separate fixture-free product; never package the CI scenario executable.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
if [[ -x .local/sysroot/usr/bin/cmake ]]; then source tools/local-env.sh; fi
mkdir -p evidence/local
python3 tools/prepare-integration.py
args=(-G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  -DOCPN_BUILD_TEST=ON -DOCPN_BUNDLE_DOCS=OFF -DOCPN_BUNDLE_GSHHS=ON
  -DOCPN_BUNDLE_TCDATA=ON -DOPENNAV_ROOT="$root"
  -DOPENNAV_ENABLE_ROUTE_SCENARIO=OFF -DXNAV_ENABLE_TEST_FIXTURES=OFF
  -DCMAKE_INSTALL_PREFIX="$root/build/production-install")
if [[ -x .local/sysroot/usr/bin/wx-config ]]; then
  args+=(-DwxWidgets_CONFIG_EXECUTABLE="$root/tools/wx-config-local"
    -DCMAKE_PROJECT_INCLUDE="$root/tools/arch-gcc-compat.cmake" -DOCPN_USE_WEBVIEW=OFF)
fi
cmake -S build/integration-source -B build/production-linux "${args[@]}" 2>&1 | tee evidence/local/production-linux-configure.log
cmake --build build/production-linux --parallel "${OPENNAV_BUILD_JOBS:-2}" 2>&1 | tee evidence/local/production-linux-build.log
cmake --install build/production-linux 2>&1 | tee evidence/local/production-linux-install.log
if [[ "${1:-}" == "--build-only" ]]; then exit 0; fi
dbus-run-session -- ctest --test-dir build/production-linux/test --output-on-failure --no-tests=error -E '^tests$' --timeout 90 --output-junit "$root/evidence/local/production-linux-tests.xml" 2>&1 | tee evidence/local/production-linux-tests.log
python3 tools/smoke-installer-selftest.py --app build/production-install/bin/opencpn
