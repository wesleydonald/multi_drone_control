#!/bin/bash
# Archive-tag, then remove the three finished worktrees and their branches (Wesley runs this; 23 Oct).
set -euo pipefail
cd /home/wesley/multi_drone_control
git tag -f archive/time-cascade-1d174bf    1d174bf
git tag -f archive/safety-defaults-7ed29c9 7ed29c9
git tag -f archive/tejen-sync2-6483d90     6483d90
mkdir -p results/legacy
[ -d /home/wesley/mdc_wt/tejen_sync2/logs ] && mv /home/wesley/mdc_wt/tejen_sync2/logs results/legacy/tejen_sync2_logs
git worktree remove --force /home/wesley/mdc_lag
git worktree remove --force /home/wesley/mdc_safety
git worktree remove --force /home/wesley/mdc_wt/tejen_sync2
git branch -D time-cascade safety-defaults tejen-sync2
git tag -l 'archive/*' | grep -E 'time-cascade|safety-defaults|tejen-sync2'
df -h / | tail -1
