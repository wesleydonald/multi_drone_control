#!/usr/bin/env bash
# rerun.sh: clean repeats (_r2) of the runs that overlapped another job on the laptop, and an instrumented repeat of
# the one idle failure that ran before the acceptance logging existed
cd "$(dirname "$0")"
r() { local f=$1 tag=$2; shift 2; env "$@" timeout 240 ./run_exit.sh "$f" "${tag}_r2" > /dev/null 2>&1
  echo "$(date +%T) $f${tag}_r2: $(grep -o 'phase RETURN_TO_START -> AT_START\|handoff preparing -> failed.*\|EXCEPTION.\{0,80\}' runs/$f${tag}_r2/events.txt | head -2 | tr '\n' ' ')"; }
r fm9h8_14 _e_mat_idle ERR=1 GOAL=mat LOAD=0 START=shift
r fm9h_2 _e_own_idle ERR=1 GOAL=own LOAD=0 START=shift
r fm9h8_5 _e_own_idle ERR=1 GOAL=own LOAD=0 START=shift
r fm9hc_11 _e_mat_idle ERR=1 GOAL=mat LOAD=0 START=shift
r fm9h8_5 _e_mat_idle_drop ERR=1 GOAL=mat LOAD=0 START=drop
r fm9hc _e_mat_idle_drop ERR=1 GOAL=mat LOAD=0 START=drop
r fm9hc_8 _e_mat_idle_drop ERR=1 GOAL=mat LOAD=0 START=drop
r fm9hc_10 _e_mat_load8 ERR=1 GOAL=mat LOAD=8 START=shift
r fm9h8_5 _e_own_load8 ERR=1 GOAL=own LOAD=8 START=shift
