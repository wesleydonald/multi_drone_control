#!/usr/bin/env bash
# Mirror the laptop working tree to drones:~/mdc (build outputs and data stay per machine).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LAB="${LAB_HOST:-drones@drones}"
DEST="${LAB_DIR:-mdc}"

# --delete never touches excluded paths, so the lab's own build/, install/ and results/ survive.
rsync -az --delete \
  --exclude=/build/ --exclude=/install/ --exclude=/log/ --exclude=/logs/ \
  --exclude=/results/ --exclude=/results_archive/ --exclude='c_generated_code*' \
  --exclude=__pycache__ --exclude=.git \
  -e "ssh -o BatchMode=yes" "$REPO/" "$LAB:$DEST/"
echo "synced $REPO -> $LAB:$DEST"
