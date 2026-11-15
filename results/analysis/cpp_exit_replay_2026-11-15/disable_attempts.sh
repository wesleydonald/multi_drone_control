#!/usr/bin/env bash
# disable_attempts.sh: the retained post-handoff disable (fm9h8_14, goal over its own start, 8 busy cores) re-run 5 times
# on the unmodified backend, ring_bridge's outputs recorded (RECORD=1); runs_disable/attempt<k>/
cd "$(dirname "$0")"
mkdir -p runs_disable
# the mission code frozen (snapshot 19:32; the coordinator is editing src/tejen_mission meanwhile)
export MDC_PY_SNAPSHOT=$PWD/runs_disable/mission_snapshot MDC_TEST_DIR=$PWD/runs_disable/mission_snapshot/test
for k in 1 2 3 4 5; do
  OUTROOT=$PWD/runs_disable RECORD=1 ERR=1 GOAL=own LOAD=8 START=shift timeout 240 ./run_exit.sh fm9h8_14 "_attempt$k" > /dev/null 2>&1
  rm -rf runs_disable/attempt$k; mv runs_disable/fm9h8_14_attempt$k runs_disable/attempt$k
  echo "$(date +%T) attempt$k: $(grep -o 'phase RETURN_TO_START -> AT_START\|handoff preparing -> failed.*' runs_disable/attempt$k/events.txt | head -1) | $(grep -o 'Backend disabled at ROS [0-9.]*: [A-Z_]*' runs_disable/attempt$k/backend.log | head -2 | tr '\n' ' ')"
done
