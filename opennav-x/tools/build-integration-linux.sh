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
  -DXNAV_ENABLE_PILOT_LOOPBACK_TESTS=ON
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
if [[ "${SKAGER_DESIGN_VALIDATION:-false}" == "true" ]]; then
xvfb-run -a build/xnav-linux/chart_name_text_test evidence/local/chart-names-xnav.png
xvfb-run -a build/xnav-linux/chart_light_label_test evidence/local/chart-lights-xnav.png
xvfb-run -a build/xnav-linux/skager_wordmark_test evidence/local/skager-wordmark-xnav.png
xvfb-run -a build/xnav-linux/chart_route_label_test evidence/local/chart-route-labels-xnav.png
xvfb-run -a build/xnav-linux/onboard_ais_body_test evidence/local/onboard-ais-xnav.png
fi
dbus-run-session -- ctest --test-dir build/xnav-linux/test --output-on-failure \
  --no-tests=error -E '^tests$' --timeout 90 --output-junit "$root/evidence/local/xnav-linux-tests.xml" \
  2>&1 | tee evidence/local/xnav-linux-tests.log
