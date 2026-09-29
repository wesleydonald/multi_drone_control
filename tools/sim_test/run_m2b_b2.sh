#!/usr/bin/env bash
# One-line M2B B2 end-to-end commissioning wrapper.
# Usage: bash tools/sim_test/run_m2b_b2.sh obstacle true
#        bash tools/sim_test/run_m2b_b2.sh clear true
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CASE="${1:-obstacle}"
GUI="${2:-true}"

if [[ "$CASE" != "obstacle" && "$CASE" != "clear" ]]; then
  echo "ERROR: first argument must be obstacle|clear."
  exit 2
fi

export M2B_COMMISSIONING_STAGE=b2
export M2B_B2_CPP_TRANSIT_ENABLED=true
if [[ "$CASE" == "obstacle" ]]; then
  export M2B_B2_STATIC_OBSTACLE_ENABLED=true
else
  export M2B_B2_STATIC_OBSTACLE_ENABLED=false
fi

echo "M2B B2 case=$CASE - C++ transit to exact B1 capture hover, then unchanged full B1 attachment."
exec bash "$SCRIPT_DIR/run_m2b_b1.sh" success "$GUI"
