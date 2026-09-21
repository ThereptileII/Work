#!/usr/bin/env bash
# Source this only when using the optional, project-local Arch toolchain.
opennav_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="$opennav_root/.local/sysroot/usr/bin:$PATH"
export LD_LIBRARY_PATH="$opennav_root/.local/sysroot/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_PREFIX_PATH="$opennav_root/.local/sysroot/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PKG_CONFIG_PATH="$opennav_root/.local/sysroot/usr/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"

# Upstream date/time regressions require Swedish and US locales. These optional
# generated locales stay within the project instead of modifying the host.
if [[ -d "$opennav_root/.local/locales" ]]; then
  export LOCPATH="$opennav_root/.local/locales${LOCPATH:+:$LOCPATH}"
fi
