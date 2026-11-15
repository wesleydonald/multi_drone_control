#!/usr/bin/env bash
# batch.sh: the replay selection, one run at a time (laptop). Tags: _<e|p>_<mat|own>_<idle|load8>[_drop]
cd "$(dirname "$0")"
P1="fm9h_2 fm9hc_10 fm9hc_11 fm9h8_13 fm9h8_14"
P2="fm9h8_5 fm9hc fm9hc_8"
one() { local f=$1 err=$2 goal=$3 load=$4 start=$5
  local tag="_$([ "$err" = 1 ] && echo e || echo p)_${goal}_$([ "$load" = 0 ] && echo idle || echo load$load)$([ "$start" = drop ] && echo _drop)"
  [ -f "runs/$f$tag/events.txt" ] && grep -q END "runs/$f$tag/events.txt" && return
  ERR=$err GOAL=$goal LOAD=$load START=$start timeout 240 ./run_exit.sh "$f" "$tag" > /dev/null 2>&1
  echo "$(date +%T) $f$tag: $(grep -c . runs/$f$tag/events.txt 2>/dev/null) events; $(grep -o 'phase RETURN_TO_START -> AT_START\|handoff preparing -> failed.*\|EXCEPTION.\{0,80\}' runs/$f$tag/events.txt | head -2 | tr '\n' ' ')"
}
for f in $P1 $P2; do one $f 1 mat 0 shift; one $f 0 mat 0 shift; one $f 1 mat 8 shift; one $f 0 mat 8 shift; done
for f in $P1 $P2; do one $f 1 own 0 shift; one $f 1 own 8 shift; done
for f in $P1 $P2; do one $f 1 mat 0 drop; done
python3 analyze.py > /dev/null && echo "summary.csv written"
