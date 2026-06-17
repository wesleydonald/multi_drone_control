#!/bin/bash
PID_FILE=/tmp/cables_world_pids.txt
if [ -f "$PID_FILE" ]; then
    while read pid; do
        kill "$pid" 2>/dev/null
    done < "$PID_FILE"
    rm -f "$PID_FILE"
fi
pkill -f "gz sim world_multi_cables" 2>/dev/null
pkill -f "rigid_cables_launch" 2>/dev/null
echo "Stopped cables world session."
