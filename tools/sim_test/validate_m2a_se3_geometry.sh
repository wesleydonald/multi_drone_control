#!/usr/bin/env bash
# Pure M2A SE(3) geometry validation. No ROS/Gazebo runtime is launched.
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

export PYTHONPATH="$REPO_ROOT/src/tejen_mission${PYTHONPATH:+:$PYTHONPATH}"

echo "== M2A pure SE(3) attachment geometry tests =="
python3 -m pytest -q \
  src/tejen_mission/test/test_m2a_attachment_geometry_se3.py

echo
echo "== Existing pure M2A detector + ring geometry regressions =="
python3 -m pytest -q \
  src/tejen_mission/test/test_m2a_attachment_detection.py \
  src/tejen_mission/test/test_ring_net_geometry_contract.py

echo
echo "PASS: pure M2A SE(3) geometry validation completed."
