#!/usr/bin/env bash
# load_run.sh <n busy cores> <tag>: the morning line on fm9_5 with n CPU busy loops running (a stand-in for the rig stack)
cd "$(dirname "$0")"
PIDS=""
for i in $(seq 1 $1); do python3 -c "while True: pass" & PIDS="$PIDS $!"; done
trap 'kill $PIDS 2>/dev/null' EXIT
BK=${BKL:-_r02z38} ./run_flight.sh ${LF:-fm9_5} ${LO:-135.0} ${LX:--0.6209} ${LY:-1.6300} ${LZ:-2.6934} ${LR:-50.0} $2
