#!/usr/bin/env bash
# Four-drone configuration of the shared IRL launch. Operator commands stay manual.
set +u
source /opt/ros/humble/setup.bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -f "${repo_root}/install/setup.bash" ]]; then
  source "${repo_root}/install/setup.bash"
fi
set -euo pipefail

mode="${1:-}"
if [[ "${mode}" != "io" && "${mode}" != "mission" ]]; then
  echo "usage: $0 {io|mission} [config.yaml] [true|false]" >&2
  exit 2
fi
rviz_default=false
if [[ "${mode}" == "mission" ]]; then rviz_default=true; fi
rviz="${3:-${rviz_default}}"
if [[ "${rviz}" != "true" && "${rviz}" != "false" ]]; then
  echo "rviz must be true or false" >&2
  exit 2
fi
if [[ "${mode}" == "io" && "${rviz}" == "true" ]]; then
  echo "rviz is available only in mission mode" >&2
  exit 2
fi
config="${2:-${repo_root}/src/tejen_mission/config/m2_irl_four_drone.yaml}"
stamp="$(date +%Y%m%d_%H%M%S)"
log_dir="${repo_root}/logs/m2_irl/${mode}_${stamp}"
mkdir -p "${log_dir}"
exec ros2 launch tejen_mission m2_irl_two_drone.launch.py \
  expected_vehicle_count:=4 mode:="${mode}" config_file:="${config}" \
  log_dir:="${log_dir}" rviz:="${rviz}"
