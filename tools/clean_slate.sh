#!/usr/bin/env bash
# clean_slate.sh — return the machine to a known-clean state before a launch.
#
# Replaces the old ./cleanup.sh, which had a duplicated block pasted into it and
# failed `bash -n` at line 32 — meaning it had never actually run. Two things it
# must do that the old one did not:
#
#   1. Clear Fast-DDS shared memory. Back-to-back launches leak nodes and exhaust
#      /dev/shm, which produces pose-timeout ESTOPs and cross-talk between drones
#      that looks exactly like controller divergence. Hours have been lost to this.
#   2. Exit non-zero when the machine is NOT clean, so tools/run_experiment.py can
#      refuse to start a run rather than silently producing corrupt results.
#
# A leftover planner from a previous run publishes to the same
# /drone_*/reference_trajectory topic as the new one. Two planners fighting over
# one tracker corrupts every number in the run, and nothing in the logs says so.
#
#   ./tools/clean_slate.sh                  # kill + clear + verify
#   ./tools/clean_slate.sh --check-only     # verify only, change nothing
#   ./tools/clean_slate.sh --publishers 3   # after launching: assert 1 publisher/drone
#   ./tools/clean_slate.sh --rig            # between rig flights: T2 only. Spares T1 (mocap + radios,
#                                           # fleet_viz, RViz with the DISARM panel) and the T4 bag, and
#                                           # leaves the Fast-DDS shared memory of those live processes
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CHECK_ONLY=0
RIG=0
N_DRONES=0
for arg in "$@"; do
  case "$arg" in
    --check-only) CHECK_ONLY=1 ;;
    --rig) RIG=1 ;;
    --publishers) shift; N_DRONES="${1:-0}" ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
  esac
  shift || true
done

SELF=$$

# Match INSTALLED node paths (".../install/<pkg>/lib/..."), never a bare package
# name: a bare name also matches the shell running this script, so the script
# kills itself. That trap is why the original used a '/lib' suffix.
PATTERNS=(
  'install/mpc_planner/lib'
  'install/tracker/lib'
  'install/fleet_manager/lib'
  'install/dissipative_planner/lib'
  'install/controller_mpc_payload/lib'
  'install/drone_magnet/lib'
  'install/simulation_communication/lib'
  'install/drone_communication/lib'
  'install/drone_visualisation/lib'
  'parameter_bridge'
  'ros_gz_bridge'
  'gz sim'
  'ruby.*gz'
  'rviz2'
  # 2026-09-26: 15 orphaned robot_state_publishers (5 old io launches) ran for 9 h
  # under every headless run; the partner stack's packages and its bag recorder
  'robot_state_publisher/lib'
  'install/tejen_mission/lib'
  'install/tejen_mpc/lib'
  'install/tejen_dynamic_planner/lib'
  'ros2 bag record'
  # 2026-09-29: survives a stopped M2 driver (python, not an installed node) at ~33 % CPU
  'sim_test/m2d_commissioning_observer.py'
)
if [ "$RIG" -eq 1 ]; then
  # the rig's T1/T4 processes stay up between flights (the sheet: only T2 is relaunched)
  RIG_SPARE='^(install/drone_communication/lib|install/drone_visualisation/lib|rviz2|ros2 bag record|robot_state_publisher/lib)$'
  kept=()
  for pat in "${PATTERNS[@]}"; do
    [[ "$pat" =~ $RIG_SPARE ]] || kept+=("$pat")
  done
  PATTERNS=("${kept[@]}")
fi
# processes spared by --rig that live in a package whose other nodes are killed (T1's fleet_viz: RViz TF)
KEEP_RE='^$'
[ "$RIG" -eq 1 ] && KEEP_RE='simulation_communication/fleet_viz'

# ── Post-launch publisher check (a different job: run AFTER the stack is up) ──
if [ "$N_DRONES" -gt 0 ]; then
  echo "── verifying exactly one reference publisher per drone ──"
  bad=0
  for ((i=0; i<N_DRONES; i++)); do
    count=$(ros2 topic info "/drone_${i}/reference_trajectory" 2>/dev/null \
            | awk -F': ' '/Publisher count/{print $2}')
    count="${count:-0}"
    if [ "$count" = "1" ]; then
      printf "   drone %-2s publishers=%s  OK\n" "$i" "$count"
    else
      printf "!! drone %-2s publishers=%s  (expected 1)\n" "$i" "$count" >&2
      bad=1
    fi
  done
  [ "$bad" -eq 0 ] || { echo "!! duplicate or missing planner — results would be corrupt." >&2; exit 1; }
  echo "   all good."
  exit 0
fi

# ── Kill ─────────────────────────────────────────────────────────────────────
if [ "$CHECK_ONLY" -eq 0 ]; then
  echo "── killing stale sim / planner / controller processes ──"
  for pat in "${PATTERNS[@]}"; do
    while read -r pid _; do
      [ -n "$pid" ] && [ "$pid" != "$SELF" ] && kill -9 "$pid" 2>/dev/null
    done < <(pgrep -af "$pat" 2>/dev/null | grep -v "clean_slate" | grep -Ev "$KEEP_RE")
  done
  sleep 1

  # ── Fast-DDS shared memory ────────────────────────────────────────────────
  # Safe once the processes above are dead: these are recreated on demand.
  # Not with --rig: the spared T1/T4 processes still use theirs.
  if [ "$RIG" -eq 1 ]; then
  echo "── --rig: Fast-DDS shared memory left alone (T1/T4 are live) ──"
  else
  echo "── clearing Fast-DDS shared memory ──"
  n_shm=$(ls /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  echo "   removed $n_shm segment(s)"

  # ── Verify the segments actually went, BEFORE restarting the daemon ───────
  # Ordering is load-bearing. The daemon recreates fastrtps_* the instant it
  # starts, so checking after the restart counted the daemon's OWN fresh
  # segments as leaked ones and this script exited 1 every single time --
  # unconditionally, on a perfectly clean machine, blaming "another user".
  # sil_bench.py ignores the exit code so it never noticed; run_experiment.py
  # honours it and refused to run at all.
  shm_left=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
  if [ "$shm_left" -gt 0 ]; then
    echo "!! $shm_left Fast-DDS segment(s) survived removal (owned by another user?)" >&2
    exit 1
  fi
  fi

  # ── ROS daemon ────────────────────────────────────────────────────────────
  # The daemon caches discovery, and after processes are killed underneath it,
  # it can go stale: `ros2 node list` then returns NOTHING while topics are
  # perfectly healthy, and `ros2 param set` reports "Node not found". That looks
  # like a broken node and is not. Restarting it costs a second.
  echo "── restarting the ROS daemon ──"
  ros2 daemon stop  >/dev/null 2>&1
  sleep 0.5
  ros2 daemon start >/dev/null 2>&1
  echo "   done"
fi

# ── Verify the right WORKSPACE is active ─────────────────────────────────────
# ~/.bashrc sources a second, older ROS workspace (~/thesis, project
# drone_cage_control) which installs packages that overlap this one --
# interfaces, utility_objects, drone_communication, drone_visualisation,
# controller_ukf. Every new shell therefore gets THOSE first unless this
# workspace's install/setup.bash is sourced afterwards.
#
# The dangerous one is `interfaces`: picking up the wrong ELRSCommand or
# MotionCaptureState definition gives subtle, silent mismatches rather than an
# error. The second most dangerous is utility_objects, where edits appear to have
# no effect at all. Catch it here rather than mid-flight.
echo "── verifying the active workspace ──"
WS_BAD=0
for pkg in utility_objects interfaces; do
  resolved=$(python3 -c "
import importlib.util as u
s = u.find_spec('$pkg')
print(s.origin if s and s.origin else 'NOT FOUND')
" 2>/dev/null)
  case "$resolved" in
    "$REPO"/*) printf "   %-16s -> this workspace  OK\n" "$pkg" ;;
    "NOT FOUND"|"") printf "!! %-16s -> NOT FOUND (workspace not sourced?)\n" "$pkg" >&2; WS_BAD=1 ;;
    *) printf "!! %-16s -> %s\n" "$pkg" "$resolved" >&2; WS_BAD=1 ;;
  esac
done
if [ "$WS_BAD" -ne 0 ]; then
  echo "!! WRONG WORKSPACE ACTIVE. Fix with:" >&2
  echo "     cd $REPO && source install/setup.bash" >&2
  echo "   (~/.bashrc sources ~/thesis/install/setup.bash, which shadows this repo.)" >&2
  exit 1
fi

# ── Verify ───────────────────────────────────────────────────────────────────
echo "── verifying clean slate ──"
REMAIN=""
for pat in "${PATTERNS[@]}"; do
  hit=$(pgrep -af "$pat" 2>/dev/null | grep -v "clean_slate" | grep -v "^$SELF " | grep -Ev "$KEEP_RE")
  [ -n "$hit" ] && REMAIN="$REMAIN$hit"$'\n'
done

if [ -n "$REMAIN" ]; then
  echo "!! still running:" >&2
  echo "$REMAIN" >&2
  echo "!! NOT clean — a run started now would be corrupt." >&2
  exit 1
fi

echo "   clean — no stale processes, no leaked DDS segments."
echo
echo "   After launching, confirm one publisher per drone with:"
echo "     ./tools/clean_slate.sh --publishers <num_drones>"
