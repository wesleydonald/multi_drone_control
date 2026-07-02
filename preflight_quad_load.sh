#!/usr/bin/env bash
# preflight_quad_load.sh — kill stale sim/ROS processes and verify a clean slate
# before launching the cable-load stack. Run this BEFORE every launch: a leftover
# planner from a previous run publishes to the same /drone_*/reference_trajectory
# topic and silently corrupts every result (two planners fighting the tracker).
set -u

echo "── killing stale sim / planner / controller processes ──"
pkill -9 -f 'controller_load_mpc/lib'    2>/dev/null
pkill -9 -f 'controller_quad_load/lib'   2>/dev/null
pkill -9 -f 'controller_mpc_multi/lib'   2>/dev/null
pkill -9 -f 'parameter_bridge'           2>/dev/null
pkill -9 -f 'gz sim'                     2>/dev/null
pkill -9 -f 'ruby.*gz'                   2>/dev/null
sleep 1

REMAIN=$(ps -eo pid,cmd | grep -iE 'controller_(load_mpc|quad_load|mpc_multi)/lib|/planner |gz sim' | grep -v grep)
if [ -n "$REMAIN" ]; then
  echo "!! still running (kill manually):"; echo "$REMAIN"
else
  echo "   clean — no stale processes."
fi

echo "── after launch, verify exactly ONE publisher per drone: ──"
echo "   ros2 topic info /drone_0/reference_trajectory   # 'Publisher count: 1'"
