#!/usr/bin/env bash
# gate.sh — run before every commit that touches control code.
#
# Ordered cheapest-first so it fails fast. The whole thing is a couple of minutes,
# which is the point: a gate that is slow does not get run.
#
#   ./tools/gate.sh            # everything
#   ./tools/gate.sh --quick    # skip the two slow stages (5 and 6)
#
# Also wired as the pre-push hook (tools/hooks/pre-push). `git push --no-verify`
# bypasses it when you genuinely mean to.
#
# Each stage exists because something got through without it:
#
#   1. workspace check  — ~/thesis used to shadow this repo's interfaces/utility_objects
#   2. unit tests       — envelope checker, RViz config structure, metrics and the
#                         analysis pipeline (tools/test)
#   3. IMPORT CHECK     — a missing `String` import killed all three trackers at launch.
#                         ast.parse and colcon build BOTH pass on that; only an actual
#                         import catches it
#   4. world geometry   — world SDF vs launch disagreement on cable_len/attach_radius
#   5. offline gate     — dissipative tests A-K, the detach/attach correctness guard
#   6. SIL SMOKE        — the only stage that runs the real nodes over real topics.
#                         Everything above it is offline and cannot see a launch file
#                         that no longer comes up. Held to configs/gate_thresholds.yaml
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

QUICK=0
[ "${1:-}" = "--quick" ] && QUICK=1

FAILED=()
step() { echo; echo "══ $* ══"; }

step "1/6  workspace"
if ./tools/clean_slate.sh --check-only 2>&1 | grep -q "WRONG WORKSPACE"; then
  echo "!! wrong workspace active — run: source install/setup.bash"
  FAILED+=("workspace")
else
  echo "   ok"
fi

step "2/6  unit tests"
if python3 -m pytest src/utility_objects/test/test_safety.py \
                    src/controller_quad_load/test/test_rviz_config.py \
                    src/controller_quad_load/test/test_planner_reference.py \
                    src/controller_quad_load/test/test_config_tools.py \
                    src/controller_dissipative/test/test_attach_network.py \
                    src/controller_load_mpc/test/test_creep_controller.py \
                    src/controller_quad_load/test/test_velocity_loop.py \
                    src/controller_load_mpc/test/test_load_geometry_params.py \
                    src/drone_magnet/test/test_handover_policy.py \
                    tools/test -q 2>&1 | tail -3; then
  :
else
  FAILED+=("pytest")
fi

step "3/6  import check (every console-script module)"
if python3 tools/import_check.py 2>&1 | tail -3; then
  :
else
  FAILED+=("imports")
fi

step "4/6  world geometry vs controller config"
# A cable_len / attach_radius / load_mass mismatch makes every drone's tension
# feedforward wrong, and presents as "the controller cannot fly" rather than as a
# config bug. This project has lost a week to exactly that.
GEO_BAD=0
for pair in \
  "simulation_assets/three_rigid_ground.sdf:mpc_quad_load_launch.py" \
  "simulation_assets/four_rigid_ground.sdf:dissipative_launch.py" \
  "simulation_assets/three_attach.sdf:three_attach_launch.py" \
  "simulation_assets/three_rigid_ground.sdf:dissipative_only_launch.py" ; do
  world="${pair%%:*}"; launch="src/controller_quad_load/launch/${pair##*:}"
  [ -f "$world" ] && [ -f "$launch" ] || continue
  if ! python3 tools/check_geometry.py "$world" --launch "$launch" >/dev/null 2>&1; then
    echo "!! geometry mismatch: $(basename "$world") vs $(basename "$launch")"
    python3 tools/check_geometry.py "$world" --launch "$launch" 2>&1 | grep -E "MISMATCH|WARN"
    GEO_BAD=1
  else
    echo "   ok   $(basename "$world")"
  fi
done
# The rod/seg2/rigid weld-variant worlds are generated from three_attach.sdf; a
# hand edit to the base that skips the generator leaves them silently stale.
if python3 tools/make_weld_variants.py --check >/dev/null 2>&1; then
  echo "   ok   weld variants up to date"
else
  echo "!! weld variants stale: rerun tools/make_weld_variants.py"
  GEO_BAD=1
fi
[ "$GEO_BAD" -eq 0 ] || FAILED+=("geometry")

if [ "$QUICK" -eq 0 ]; then
  step "5/6  offline dissipative harness (tests A-K)"
  if python3 -m controller_dissipative.verify_dissipative 2>&1 | tail -2; then
    :
  else
    FAILED+=("verify_dissipative")
  fi
else
  step "5/6  offline dissipative harness — SKIPPED (--quick)"
fi

if [ "$QUICK" -eq 0 ]; then
  step "6/6  SIL smoke suite (real nodes, numerical plant)"
  # The one stage that runs the ACTUAL controller nodes over ROS. Stages 2-5 are all
  # offline: they cannot catch a launch file that no longer comes up, a node that
  # crashes on a parameter it now reads, or a topic that got renamed on one side.
  # ~25 s, and it flies a payload.
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "!! ros2 is not on PATH — the SIL smoke suite could NOT RUN."
    echo "   source ~/ros2_humble/install/setup.bash && source install/setup.bash"
    FAILED+=("sil_smoke (not run)")
  else
    SMOKE_LOG="$(mktemp)"
    ./tools/sil_bench.py configs/sil/carry_hover_n3.yaml >"$SMOKE_LOG" 2>&1
    SMOKE_DIR="$(grep -m1 -oP '(?<=out:\s{4})\S+' "$SMOKE_LOG" || true)"
    grep -E "^\s+(ran|exit:)" "$SMOKE_LOG" || true
    if [ -z "$SMOKE_DIR" ] || [ ! -d "$SMOKE_DIR" ]; then
      echo "!! the bench produced no run directory; see $SMOKE_LOG"
      tail -15 "$SMOKE_LOG"
      FAILED+=("sil_smoke")
    # Thresholds, not just "it exited 0": the smoke scenario has no acceptance criteria
    # of its own, so without this the stage would pass on a run that took off and then
    # dropped the load. Bars are in configs/gate_thresholds.yaml, set from a measured
    # baseline (R0047-R0050).
    elif ./tools/check_thresholds.py "$SMOKE_DIR" --profile sil_smoke; then
      rm -f "$SMOKE_LOG"
    else
      echo "   full log: $SMOKE_LOG"
      FAILED+=("sil_smoke thresholds")
    fi
  fi
else
  step "6/6  SIL smoke suite — SKIPPED (--quick)"
fi

echo
if [ ${#FAILED[@]} -eq 0 ]; then
  echo "══ GATE PASSED ══"
  exit 0
fi
echo "══ GATE FAILED: ${FAILED[*]} ══"
exit 1
