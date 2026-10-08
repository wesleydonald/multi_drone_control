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
# Ctrl-C stops the launch and then the bag (so its metadata is written). Then tools/rig_summary.py
# draws figures/<name>_summary.png in the background, ~20 s (RIG_FLIGHT_SUMMARY=0 skips it).
# Refuses unless ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID are set; mode:=attach refuses while T1 is up.
set -uo pipefail

usage() { sed -n '2,15p' "$0"; exit 2; }
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

N=""; MODE=""; REAL_IO=true
for a in "$@"; do case "$a" in
  num_drones:=*) N="${a#num_drones:=}" ;; mode:=*) MODE="${a#mode:=}" ;; real_io:=false) REAL_IO=false ;;
esac; done
# mode:=attach counts only the tethered drones (default 3); the newcomer is one more radio
if [ "$MODE" = attach ]; then N=$(( ${N:-3} + 1 )); else N=${N:-4}; fi

if [ "${RIG_FLIGHT_DRY_RUN:-0}" = 1 ]; then
  echo "rig_flight: DRY RUN F=$F LAUNCH=$LAUNCH N=$N DIR=$D ARGS=$*"; exit 0
fi
# one DDS domain for T1, T2, T3 and RViz, local only (Wesley 8 Oct: everything on this laptop;
# a laptop elsewhere on the lab network must not reach the fleet). Set both in ~/.bashrc.
if [ "${ROS_LOCALHOST_ONLY:-0}" != 1 ] || [ -z "${ROS_DOMAIN_ID:-}" ]; then
  echo "rig_flight: set ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID in ~/.bashrc (one place, so T1, T3 and RViz" >&2
  echo "            share it), e.g.  export ROS_DOMAIN_ID=23; export ROS_LOCALHOST_ONLY=1  then open new terminals" >&2
  exit 1
fi
# mode:=attach starts its own radios and mocap: a T1 still up would put two writers on every radio
if [ "$MODE" = attach ] && [ "$REAL_IO" = true ] && pgrep -f "drone_communication/.*elrs_interface|real_io_launch" >/dev/null; then
  echo "rig_flight: mode:=attach starts its own radios and mocap: stop T1 (rio) first" >&2
  exit 1
fi
mkdir -p "$D"
{ git rev-parse HEAD; git status --short; } > "$D/${F}_code.txt" 2>&1

tools/clean_slate.sh --rig || { echo "rig_flight: clean_slate failed" >&2; exit 1; }
export MDC_RUN_DIR="$PWD/$D/${F}_logs"

topics=(/payload/motion_capture_state /fleet/command /fleet/abort /fleet/status /fleet/landed
        /fleet/manager_status /rosout /fleet/detach /fleet/drop /fleet/payload_dropped
        /magnet_tip_pose /attach_target/pose /magnet/command /magnet/object_attached
        /magnet/attachment_state /partner/release /join_planner/handoff_ready)
for ((i = 0; i < N; i++)); do
  topics+=("/drone_$i/motion_capture_state" "/drone_$i/ELRSCommand" "/drone_$i/telemetry"
           "/drone_$i/magnet" "/drone_$i/fc_arm_state" "/drone_$i/mux_state" "/drone_$i/ELRSCommand_diss")
done
ros2 bag record -o "$D/$F" "${topics[@]}" > "$D/${F}_bag.txt" 2>&1 &
BAG=$!
stop_bag() { kill -INT "$BAG" 2>/dev/null; wait "$BAG" 2>/dev/null; echo "rig_flight: bag $D/$F closed"; }
summary() {
  [ "${RIG_FLIGHT_SUMMARY:-1}" = 1 ] && [ -d "$MDC_RUN_DIR/logs/tracker" ] || return 0
  # its own session: the next flight's Ctrl-C in this terminal must not stop it
  setsid nohup python3 tools/rig_summary.py "$D/$F" > "$D/${F}_summary.txt" 2>&1 < /dev/null &
  echo "rig_flight: summary in ~20 s -> $D/figures/${F}_summary.png"
}
trap 'stop_bag; summary' EXIT

echo "rig_flight: F=$F  logs $MDC_RUN_DIR  bag $D/$F (${#topics[@]} topics)"
ros2 launch bringup "$LAUNCH" "$@" 2>&1 | tee "$D/$F.log"
