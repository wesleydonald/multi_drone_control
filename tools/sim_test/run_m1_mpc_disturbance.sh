#!/usr/bin/env bash
# M1 pickup-bias MPC commissioning wrapper.
# GUI stays ON. ARM / TAKEOFF / DISARM remain manual operator actions.
# Every stage delegates to the established supervised C1F.6 shell runner.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Keep the one-command commissioning workflow robust after Ctrl-C / failed launches.
# The cleanup helper refuses to interfere with a genuinely live supervised run.
bash "${SCRIPT_DIR}/cleanup_m1_stale.sh"

STAGE="${1:-}"
FX_N="-0.05"
FY_N="0.00"

usage() {
  cat <<'USAGE'
Usage:
  bash tools/sim_test/run_m1_mpc_disturbance.sh <stage>

Pickup-bias stages:
  pickup-legacy   P1 legacy integral, Fx=-0.05 N, Fy=0.00 N
  pickup-shadow   P2 disturbance observer shadow mode, same force
  pickup-active   P3 active disturbance-aware MPC, same force

Regression stages:
  legacy-zero        Existing nominal legacy integral, zero injected XY force
  landing-approach   Landing regression from APPROACH_ABOVE_PICKUP
  landing-loaded     Loaded landing regression from LIFT_OBJECT

All stages:
  - Gazebo GUI ON
  - RViz ON
  - manual ARM / TAKEOFF / DISARM authority
  - supervised evidence capture ON
  - baseline M1 environment case

Pickup-bias force window:
  - applied to x3::X3/base_link only at SETTLE_ABOVE_PICKUP
  - held through DESCEND_TO_PICKUP / MAGNET_ATTACH_WAIT
  - cleared at LIFT_OBJECT

STOP RULE:
  If this small fixture produces persistent/continuous drift instead of a bounded
  pickup offset, stop the disturbance-fixture experiment rather than tuning the
  controller around an unrepresentative simulation behaviour.
USAGE
}

if [[ -z "$STAGE" || "$STAGE" == "-h" || "$STAGE" == "--help" ]]; then
  usage
  [[ -n "$STAGE" ]] && exit 0 || exit 2
fi

MODE=""
FX="0.0"
FY="0.0"
LANDING_PHASE=""
LABEL=""

case "$STAGE" in
  pickup-legacy)
    LABEL="P1 - PICKUP BIAS / LEGACY INTEGRAL"
    MODE="legacy_integral"
    FX="$FX_N"
    FY="$FY_N"
    ;;
  pickup-shadow)
    LABEL="P2 - PICKUP BIAS / DISTURBANCE OBSERVER SHADOW"
    MODE="lateral_disturbance_shadow"
    FX="$FX_N"
    FY="$FY_N"
    ;;
  pickup-active)
    LABEL="P3 - PICKUP BIAS / ACTIVE DISTURBANCE-AWARE MPC"
    MODE="lateral_disturbance"
    FX="$FX_N"
    FY="$FY_N"
    ;;
  legacy-zero)
    LABEL="NOMINAL LEGACY INTEGRAL / ZERO INJECTED BIAS"
    MODE="legacy_integral"
    ;;
  landing-approach)
    LABEL="LANDING REGRESSION - NORMAL AIRBORNE APPROACH"
    MODE="legacy_integral"
    LANDING_PHASE="APPROACH_ABOVE_PICKUP"
    ;;
  landing-loaded)
    LABEL="LANDING REGRESSION - LOADED LIFT"
    MODE="legacy_integral"
    LANDING_PHASE="LIFT_OBJECT"
    ;;
  *)
    echo "ERROR: unknown stage '$STAGE'." >&2
    usage >&2
    exit 2
    ;;
esac

echo "============================================================================="
echo "M1 PICKUP-BIAS MPC COMMISSIONING"
echo "Stage:        $LABEL"
echo "XY mode:      $MODE"
echo "World force:  Fx=$FX N, Fy=$FY N"
echo "Force window: SETTLE_ABOVE_PICKUP -> clear at LIFT_OBJECT"
echo "Force target: x3::X3/base_link only"
echo "Display:      Gazebo GUI + RViz"
echo "Authority:    MANUAL ARM / TAKEOFF / DISARM"
if [[ -n "$LANDING_PHASE" ]]; then
  echo "Landing:      normal /join_planner/land_now request from $LANDING_PHASE"
else
  echo "Landing:      normal mission semantics"
fi
echo "============================================================================="
echo "In RViz: ARM when ready, then TAKEOFF. Do not use automatic arming."
echo "STOP if the small pickup-bias fixture causes persistent drift instead of a bounded offset."
echo

ARGS=(
  baseline
  --xy-bias-mode "$MODE"
  --disturbance-force-x-n "$FX"
  --disturbance-force-y-n "$FY"
)
if [[ -n "$LANDING_PHASE" ]]; then
  ARGS+=(--landing-check-from "$LANDING_PHASE")
fi

exec "${SCRIPT_DIR}/run_m1_case.sh" "${ARGS[@]}"
