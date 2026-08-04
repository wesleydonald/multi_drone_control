#!/usr/bin/env bash
# gate.sh — run before every commit that touches control code.
#
# Ordered cheapest-first so it fails fast. The whole thing is a couple of minutes,
# which is the point: a gate that is slow does not get run.
#
#   ./tools/gate.sh            # everything
#   ./tools/gate.sh --quick    # skip the offline dissipative harness (the slow part)
#
# Each stage exists because something got through without it:
#
#   1. workspace check  — ~/thesis used to shadow this repo's interfaces/utility_objects
#   2. unit tests       — envelope checker, RViz config structure
#   3. IMPORT CHECK     — a missing `String` import killed all three trackers at launch.
#                         ast.parse and colcon build BOTH pass on that; only an actual
#                         import catches it
#   4. offline gate     — dissipative tests A-K, the detach/attach correctness guard
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

QUICK=0
[ "${1:-}" = "--quick" ] && QUICK=1

FAILED=()
step() { echo; echo "══ $* ══"; }

step "1/4  workspace"
if ./tools/clean_slate.sh --check-only 2>&1 | grep -q "WRONG WORKSPACE"; then
  echo "!! wrong workspace active — run: source install/setup.bash"
  FAILED+=("workspace")
else
  echo "   ok"
fi

step "2/4  unit tests"
if python3 -m pytest src/utility_objects/test/test_safety.py \
                    src/controller_quad_load/test/test_rviz_config.py -q 2>&1 | tail -3; then
  :
else
  FAILED+=("pytest")
fi

step "3/4  import check (every console-script module)"
if python3 tools/import_check.py 2>&1 | tail -3; then
  :
else
  FAILED+=("imports")
fi

if [ "$QUICK" -eq 0 ]; then
  step "4/4  offline dissipative harness (tests A-K)"
  if python3 -m controller_dissipative.verify_dissipative 2>&1 | tail -2; then
    :
  else
    FAILED+=("verify_dissipative")
  fi
else
  step "4/4  offline dissipative harness — SKIPPED (--quick)"
fi

echo
if [ ${#FAILED[@]} -eq 0 ]; then
  echo "══ GATE PASSED ══"
  exit 0
fi
echo "══ GATE FAILED: ${FAILED[*]} ══"
exit 1
