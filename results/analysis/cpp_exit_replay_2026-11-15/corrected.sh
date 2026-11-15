#!/usr/bin/env bash
# corrected.sh: the bounded replay of the corrected mission (live src/tejen_mission), 8 runs, no repeats:
# hover fm9h_2, circle fm9hc_10, figure-eights fm9h8_13 and fm9h8_14, each idle and with 8 busy cores; recorded error,
# goal over the mat; parameters re-dumped with TOP=3.2 (params_corrected/); the node's transfer generator config
cd "$(dirname "$0")"
mkdir -p runs_corrected
for f in fm9h_2 fm9hc_10 fm9h8_13 fm9h8_14; do for load in 0 8; do
  tag="_e_mat_$([ $load = 0 ] && echo idle || echo load8)"
  OUTROOT=$PWD/runs_corrected PARAMDIR=$PWD/params_corrected RIG_TRANSFER=1 ERR=1 GOAL=mat LOAD=$load START=shift \
    timeout 240 ./run_exit.sh $f "$tag" > /dev/null 2>&1
  echo "$(date +%T) $f$tag: $(grep -o 'phase RETURN_TO_START -> AT_START\|phase AT_START -> LANDING\|handoff preparing -> failed.*\|EXCEPTION.\{0,100\}' runs_corrected/$f$tag/events.txt | tr '\n' ' ')"
done; done
