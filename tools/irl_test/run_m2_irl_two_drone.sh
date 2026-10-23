#!/usr/bin/env bash
# This runner starts I/O or mission processes only. ARM, TAKEOFF and LAND stay manual.
# Generated ROS/colcon setup scripts dereference optional variables, so source them
# before enabling nounset.
set +u
source /opt/ros/humble/setup.bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -f "${repo_root}/install/setup.bash" ]]; then
  source "${repo_root}/install/setup.bash"
fi
set -euo pipefail

mode="${1:-}"
rviz="${3:-false}"
passive_diagnostics="${4:-false}"
completion_mode="${5:-airborne_hold}"
if [[ "${completion_mode}" != "airborne_hold" && "${completion_mode}" != "attach_park" ]]; then
  echo "completion mode must be airborne_hold or attach_park" >&2
  exit 2
fi
if [[ "${mode}" != "io" && "${mode}" != "mission" ]]; then
  echo "usage: $0 {io|mission} [config.yaml] [rviz:true|false] [passive_diagnostics:true|false] [airborne_hold|attach_park]" >&2
  exit 2
fi
if [[ "${rviz}" != "true" && "${rviz}" != "false" ]]; then
  echo "rviz must be true or false" >&2
  exit 2
fi
if [[ "${passive_diagnostics}" != "true" && "${passive_diagnostics}" != "false" ]]; then
  echo "passive diagnostics must be true or false" >&2
  exit 2
fi
if [[ "${mode}" == "io" && "${rviz}" == "true" ]]; then
  echo "rviz is available only in mission mode" >&2
  exit 2
fi
config="${2:-${repo_root}/src/tejen_mission/config/m2_irl_two_drone.yaml}"
stamp="$(date +%Y%m%d_%H%M%S)"
log_dir="${repo_root}/logs/m2_irl/${mode}_${stamp}"
mkdir -p "${log_dir}"
exec ros2 launch tejen_mission m2_irl_two_drone.launch.py \
  mode:="${mode}" config_file:="${config}" log_dir:="${log_dir}" rviz:="${rviz}" \
  lateral_disturbance_diagnostics_enabled:="${passive_diagnostics}" completion_mode:="${completion_mode}"
