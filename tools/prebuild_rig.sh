#!/usr/bin/env bash
# prebuild_rig.sh — fill the rig's own solver folder (the MDC_ACADOS_ROOT rig_flight.sh exports) at the rig's drone mass,
# so twin runs on this laptop never leave the rig a planner built at another mass (Wesley 22 Oct). Run before a lab day.
#
#   tools/prebuild_rig.sh            # planner n = 2, 3, 4 and the tracker
#   tools/prebuild_rig.sh 3 4        # just those fleet sizes
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
export MDC_ACADOS_ROOT="${MDC_RIG_ACADOS_ROOT:-$HOME/mdc_rig_generated}"
mkdir -p "$MDC_ACADOS_ROOT"
cd "$REPO"
python3 tools/prebuild_planner.py --drone-mass "${RIG_DRONE_MASS:-0.583}" "$@"
python3 -c 'from tracker import tracker_node as c; c._solver_is_fresh() or c.generate_ocp_controller()'
echo "prebuild_rig: $MDC_ACADOS_ROOT ready (drone mass ${RIG_DRONE_MASS:-0.583})"
