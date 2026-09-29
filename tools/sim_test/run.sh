#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f "${HOME}/ros2_humble/install/setup.bash" ]]; then source "${HOME}/ros2_humble/install/setup.bash"; else source /opt/ros/humble/setup.bash; fi
fi
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "run.sh: workspace install/setup.bash is missing; build the workspace first" >&2
  exit 2
fi
source "${REPO_ROOT}/install/setup.bash"
set -u

cd "${REPO_ROOT}"
exec python3 "${SCRIPT_DIR}/run_sim_test.py" "$@"
