#!/usr/bin/env bash
# One/two standalone real obstacles. No automatic ARM/TAKEOFF, no ego mission.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$task_root"
source /opt/ros/humble/setup.bash
source install/setup.bash
action=run
if [[ "${1:-}" == check ]]; then action=check; shift; fi
motion="${1:-line}"
vehicle="${2:-drone_2}"
speed="${3:-1.0}"
xy_integral="${4:-true}"
horizontal_drag="${5:-0.0}"
if [[ "$xy_integral" != true && "$xy_integral" != false ]]; then
  echo 'Fourth argument must be true or false (XY integral).' >&2
  exit 2
fi
profile="${THESIS_C_PROFILE:-$task_root/src/thesis_c_experiments/config/experiments.yaml}"
run_dir="$task_root/logs/independent_drones/obstacle_${motion}_$(date +%Y%m%d_%H%M%S)"
python3 - "$profile" "$motion" "$vehicle" "$speed" "$run_dir" "$xy_integral" "$horizontal_drag" <<'PY'
import json,sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from thesis_c_experiments.obstacle_tracking import build_trial
share=lambda name:Path(get_package_share_directory(name))
data=build_trial(sys.argv[1],share('tejen_mission')/'config/m2_irl_four_drone.yaml',
    share('wesley_drone_control')/'config/independent_hover.yaml',sys.argv[2],sys.argv[3],float(sys.argv[4]),sys.argv[5],
    xy_integral_enabled=sys.argv[6]=='true',horizontal_drag_per_s=float(sys.argv[7]))
print('Bounded XY integral:', sys.argv[6])
print('Fixed horizontal drag (1/s):', sys.argv[7])
print(json.dumps({k:data[k] for k in ('vehicle','vehicle_trials','motion','start_position','period_s','solver_variant','landing_footprint')},indent=2))
if not data['landing_footprint']['inside_workspace']:
 print('WARNING: existing LAND brake reference can leave the configured workspace. Review this before flight.')
print('CHECK ONLY: no sockets, ports or flight commands started.')
PY
if [[ "$action" == check ]]; then exit 0; fi
if ss -H -lun | grep -Eq '(^|[[:space:]])[^[:space:]]*:1511([[:space:]]|$)'; then
  echo 'UDP1511 already owned. Stop the old flight stack before this standalone test.' >&2
  exit 2
fi
mkdir -p "$run_dir"
git rev-parse HEAD > "$run_dir/git_commit.txt"
git status --short > "$run_dir/git_status.txt"
export ROS_LOG_DIR="$run_dir/ros"
exec ros2 launch thesis_c_experiments obstacle_tracking.launch.py motion:="$motion" vehicle:="$vehicle" \
  speed:="$speed" profile:="$profile" run_dir:="$run_dir" xy_integral_enabled:="$xy_integral" \
  horizontal_drag_per_s:="$horizontal_drag"
