#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <scenario> <sim|irl> [--notes TEXT] [--force]" >&2
  echo "Example: $0 c1f3a sim" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/experiment_capture.py" start "$@"
