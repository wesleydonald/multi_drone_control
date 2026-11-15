#!/usr/bin/env bash
# bridge_after.sh: the runs_disable conditions (fm9h8_14, goal over its own start, recorded error, shifted start, the
# default parameter set, the fixture's transfer generator, ring_bridge's outputs recorded) on the live tree after the
# ring_bridge fix and with the logging backend (21:02): 1 idle + 5 with 8 busy cores; runs_bridge_after/<slot>/
cd "$(dirname "$0")"
mkdir -p runs_bridge_after
one() { local slot=$1 load=$2
  OUTROOT=$PWD/runs_bridge_after RECORD=1 ERR=1 GOAL=own LOAD=$load START=shift timeout 240 ./run_exit.sh fm9h8_14 "_$slot" > /dev/null 2>&1
  rm -rf runs_bridge_after/$slot; mv runs_bridge_after/fm9h8_14_$slot runs_bridge_after/$slot
  echo "$(date +%T) $slot: $(grep -o 'phase RETURN_TO_START -> AT_START\|handoff preparing -> failed.*' runs_bridge_after/$slot/events.txt | head -1) | $(grep -o 'Backend disabled at ROS [0-9.]*: [A-Z_]*' runs_bridge_after/$slot/backend.log | head -2 | tr '\n' ' ') $(grep -c DISABLE_DIAG runs_bridge_after/$slot/backend.log) DISABLE_DIAG"
}
one idle1 0
for k in 1 2 3 4 5; do one attempt$k 8; done
