#!/usr/bin/env bash
# rig_flight.sh — one rig flight in one terminal (replaces the pasted T2 + T4 lines).
#
#   tools/rig_flight.sh F <launch file> [launch args...]
#   tools/rig_flight.sh h1 real_control_launch.py num_drones:=3 target_z:=1.0
#
# Clears stale control nodes (clean_slate --rig, T1 spared), sets MDC_RUN_DIR so the
# CSV logs land in results/rig/<date>/F_logs, records the bag with every topic the
# read-back needs, and tees the launch output to results/rig/<date>/F.log. Ctrl-C stops
# the launch and then the bag (so its metadata is written). On 1 Oct the pasted bag
# line was cut short and the bags held only mocap.
set -uo pipefail

if [ $# -lt 2 ]; then sed -n '2,10p' "$0"; exit 2; fi
F="$1"; LAUNCH="$2"; shift 2

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO" || exit 1
D="results/rig/$(date +%Y-%m-%d)"
mkdir -p "$D"
if [ -e "$D/$F.log" ] || [ -e "$D/$F" ]; then
  echo "rig_flight: $D/$F already exists; pick a new flight name" >&2; exit 1
fi

N=4
for a in "$@"; do case "$a" in num_drones:=*) N="${a#num_drones:=}" ;; esac; done

tools/clean_slate.sh --rig || { echo "rig_flight: clean_slate failed" >&2; exit 1; }
export MDC_RUN_DIR="$PWD/$D/${F}_logs"

topics=(/payload/motion_capture_state /fleet/command /fleet/abort /fleet/status /fleet/landed
        /fleet/manager_status /rosout)
for ((i = 0; i < N; i++)); do
  topics+=("/drone_$i/motion_capture_state" "/drone_$i/ELRSCommand" "/drone_$i/telemetry"
           "/drone_$i/magnet" "/drone_$i/fc_arm_state")
done
ros2 bag record -o "$D/$F" "${topics[@]}" > "$D/${F}_bag.txt" 2>&1 &
BAG=$!
stop_bag() { kill -INT "$BAG" 2>/dev/null; wait "$BAG" 2>/dev/null; echo "rig_flight: bag $D/$F closed"; }
trap stop_bag EXIT

echo "rig_flight: F=$F  logs $MDC_RUN_DIR  bag $D/$F (${#topics[@]} topics)"
ros2 launch controller_quad_load "$LAUNCH" "$@" 2>&1 | tee "$D/$F.log"
