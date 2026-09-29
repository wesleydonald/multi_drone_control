#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Usage:
  tools/sim_test/run_m1_case.sh --list
  tools/sim_test/run_m1_case.sh <case> [extra run_sim_test.py arguments]

Examples:
  tools/sim_test/run_m1_case.sh static_two
  tools/sim_test/run_m1_case.sh narrow_gap
  tools/sim_test/run_m1_case.sh figure8 --no-plots
EOF
}

if [[ $# -eq 0 ]]; then
  usage >&2
  exit 2
fi

if [[ "$1" == "--list" ]]; then
  exec "${SCRIPT_DIR}/run.sh" cases
fi
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

CASE="$1"
shift
exec "${SCRIPT_DIR}/run.sh" run --scenario c1f6 --case "${CASE}" --gui --manual-control "$@"
