#!/usr/bin/env bash
# Rebuild .debs of this laptop's installed Gazebo Harmonic packages into tools/remote/debs/.
# The OSRF apt pool keeps only the newest release (gz-sim 8.15, physics 7.8 by Sep 2026),
# so the laptop's gz-sim 8.10 / physics 7.6 cannot be fetched; the image installs these.
set -euo pipefail

OUT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/debs"
mkdir -p "$OUT"

PKGS=$(dpkg-query -W -f='${Package}\n' | grep -E \
  '^(gz-|libgz-|python3-gz-|libsdformat14|sdformat14-sdf|python3-sdformat14|ros-humble-ros-gzharmonic)' \
  | sort -u)

for p in $PKGS; do
  ver=$(dpkg-query -W -f='${Version}' "$p")
  arch=$(dpkg-query -W -f='${Architecture}' "$p")
  deb="$OUT/${p}_${ver//:/%3a}_${arch}.deb"
  [ -f "$deb" ] && continue
  root=$(mktemp -d)
  mkdir -p "$root/DEBIAN"
  # Control file without the local-install fields.
  dpkg-query -s "$p" | awk '
    /^[A-Za-z-]+:/ { skip = ($1 == "Status:" || $1 == "Conffiles:" || $1 == "Config-Version:") }
    !skip' > "$root/DEBIAN/control"
  dpkg-query -W -f='${Conffiles}\n' "$p" | awk 'NF {print $1}' > "$root/DEBIAN/conffiles"
  [ -s "$root/DEBIAN/conffiles" ] || rm "$root/DEBIAN/conffiles"
  for s in preinst postinst prerm postrm triggers shlibs symbols; do
    for f in "/var/lib/dpkg/info/$p.$s" "/var/lib/dpkg/info/$p:$arch.$s"; do
      [ -f "$f" ] && cp "$f" "$root/DEBIAN/$s"
    done
  done
  dpkg -L "$p" | while read -r f; do
    if [ -L "$f" ] || [ -f "$f" ]; then cp -a --parents "$f" "$root/"; fi
  done
  fakeroot dpkg-deb --root-owner-group -Zgzip -b "$root" "$deb" >/dev/null
  rm -rf "$root"
  echo "repacked $p $ver"
done
ls "$OUT" | wc -l | xargs echo "debs in $OUT:"
