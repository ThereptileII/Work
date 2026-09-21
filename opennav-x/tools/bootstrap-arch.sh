#!/usr/bin/env bash
# Optional rootless development dependencies. Uses the current pacman sync DB.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
mkdir -p .local/packages .local/sysroot evidence/local
pacman -Sp --print-format '%n %v %h %l' \
  cmake ninja wxwidgets-gtk3 wxwidgets-common glew lsb-release \
  xorg-server-xvfb xdotool xorg-xauth > .local/packages/manifest.txt
while read -r package version checksum url; do
  [[ "$url" == https://* && "$checksum" =~ ^[0-9a-f]{64}$ ]] || exit 1
  file=".local/packages/${url##*/}"
  if [[ ! -f "$file" ]]; then
    curl --fail --location --silent --show-error "$url" -o "$file"
  fi
  printf '%s  %s\n' "$checksum" "$file" | sha256sum --check --status
  bsdtar -xf "$file" -C .local/sysroot
done < .local/packages/manifest.txt
cp .local/packages/manifest.txt evidence/local/linux-dependency-manifest.txt
sha256sum .local/packages/*.pkg.tar.zst > evidence/local/linux-dependency-sha256.txt
printf 'Dependencies verified against pacman metadata and extracted locally. Source tools/local-env.sh to use them.\n'
