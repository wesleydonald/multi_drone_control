#!/usr/bin/env bash
# rig_flight.sh — one rig flight in one terminal (replaces the pasted T2 + T4 lines).
#
#   tools/rig_flight.sh <launch file> [launch args...] <name>     (name last: edit only the end)
#   tools/rig_flight.sh real_control_launch.py num_drones:=3 target_z:=1.0 h1
#   tools/rig_flight.sh <name> <launch file> [launch args...]     (the old order still works)
#
# A name that already exists today gets _2, _3, ... appended (printed), so the same line can be
# re-run after a failed ARM. RIG_FLIGHT_DRY_RUN=1 prints what it would do and stops.
# Clears stale control nodes (clean_slate --rig, T1 spared), sets MDC_RUN_DIR so the CSV logs
# land in results/rig/<date>/<name>_logs, writes <name>_code.txt (git HEAD + status), records
# the bag with every topic the read-back needs, and tees the launch output to <name>.log.
# Ctrl-C stops the launch and then the bag (so its metadata is written).
set -uo pipefail

usage() { sed -n '2,13p' "$0"; exit 2; }
[ $# -ge 2 ] || usage
if [[ "$1" == *.py ]]; then
  LAUNCH="$1"; F="${!#}"
  [[ "$F" == *:=* || "$F" == *.py ]] && { echo "rig_flight: put the run name last" >&2; usage; }
  set -- "${@:2:$#-2}"
elif [[ "$2" == *.py ]]; then
  F="$1"; LAUNCH="$2"; shift 2
else
  usage
fi
[[ "$F" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "rig_flight: bad run name '$F'" >&2; exit 2; }

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO" || exit 1
D="${RIG_FLIGHT_DIR:-results/rig/$(date +%Y-%m-%d)}"
base="$F"; k=2
while [ -e "$D/$F.log" ] || [ -e "$D/$F" ] || [ -e "$D/${F}_logs" ]; do F="${base}_$k"; k=$((k + 1)); done
[ "$F" != "$base" ] && echo "rig_flight: $base already exists today: this run is $F"

N=4
for a in "$@"; do case "$a" in num_drones:=*) N="${a#num_drones:=}" ;; esac; done

if [ "${RIG_FLIGHT_DRY_RUN:-0}" = 1 ]; then
  echo "rig_flight: DRY RUN F=$F LAUNCH=$LAUNCH N=$N DIR=$D ARGS=$*"; exit 0
fi
mkdir -p "$D"
{ git rev-parse HEAD; git status --short; } > "$D/${F}_code.txt" 2>&1

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
ros2 launch bringup "$LAUNCH" "$@" 2>&1 | tee "$D/$F.log"
