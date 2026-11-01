#!/usr/bin/env bash
# SKAGER for Linux as a Debian package (Raspberry Pi OS and Debian; built on the target
# release and architecture by .github/workflows/skager-raspberry-pi.yml).
# Installs into /opt/skager, so it never touches an OpenCPN package; adds the `skager`
# command and a menu entry. Dependencies come from the binaries (dpkg-shlibdeps).
#   SKAGER_INSTALL_PREFIX=/opt/skager tools/build-production-linux.sh --build-only
#   tools/package-linux-deb.sh <output-dir>
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
out="$(mkdir -p "$1" && cd "$1" && pwd)"
prefix=/opt/skager
version=$(sed -n 's/.*Version\[\] = "\([^"]*\)".*/\1/p' "$root/src/application/Version.h")
commit=$(git -C "$root" rev-parse --short=8 HEAD)
arch=$(dpkg --print-architecture)
# Debian orders "~" before anything, so 0.4.0~beta2.10 < 0.4.0.
deb_version="$(echo "$version" | sed 's/-/~/')+git${commit}"
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
pkg="$stage/skager"
DESTDIR="$pkg" cmake --install "$root/build/production-linux" >/dev/null
test -x "$pkg$prefix/bin/opencpn"

mkdir -p "$pkg/usr/bin" "$pkg/usr/share/applications" "$pkg/usr/share/icons/hicolor/scalable/apps"
cat > "$pkg/usr/bin/skager" <<SH
#!/bin/sh
# SKAGER (OpenCPN with the SKAGER interface). X11 keeps SKAGER Helm's screen capture working.
export GDK_BACKEND=\${GDK_BACKEND:-x11}
exec $prefix/bin/opencpn --xnav "\$@"
SH
chmod 755 "$pkg/usr/bin/skager"
cat > "$pkg/usr/share/applications/skager.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=SKAGER
Comment=Navigation, instruments and boat systems
Exec=skager
Icon=skager
Terminal=false
Categories=Utility;Geography;
StartupWMClass=opencpn
DESKTOP
cp "$root/companion/resources/icon.svg" "$pkg/usr/share/icons/hicolor/scalable/apps/skager.svg"

# Shared-library dependencies of every ELF file, resolved against the system packages;
# SKAGER's own libraries in the package are found through -l.
mkdir -p "$stage/debian"
printf 'Source: skager\n\nPackage: skager\nArchitecture: %s\n' "$arch" > "$stage/debian/control"
mapfile -t elves < <(find "$pkg$prefix" -type f -exec sh -c 'head -c 4 "$1" | grep -q "ELF"' _ {} \; -print)
libdirs=$(find "$pkg$prefix" -name '*.so*' -printf '-l%h\n' | sort -u | tr '\n' ' ')
depends=$(cd "$stage" && dpkg-shlibdeps -O --ignore-missing-info $libdirs "${elves[@]}" 2>/dev/null | sed -n 's/^shlibs:Depends=//p')
test -n "$depends"

mkdir -p "$pkg/DEBIAN"
size=$(du -sk "$pkg" | cut -f1)
cat > "$pkg/DEBIAN/control" <<CONTROL
Package: skager
Version: $deb_version
Architecture: $arch
Maintainer: SKAGER <orders@auth.skager.app>
Installed-Size: $size
Depends: $depends
Section: misc
Priority: optional
Homepage: https://skager.app
Description: SKAGER marine navigation
 SKAGER: the chart, instruments, passage planning, anchor watch and boat systems
 in one interface, built on OpenCPN. Start it from the menu or with the
 command skager.
CONTROL
name="skager_${version}-${commit}_${arch}.deb"
dpkg-deb --root-owner-group --build "$pkg" "$out/$name" >/dev/null
(cd "$out" && sha256sum "$name" | tee "$name.sha256")
