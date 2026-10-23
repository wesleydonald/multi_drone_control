#!/usr/bin/env bash
# Opt-in M1 physical ground-ring trial; never arms or takes off automatically.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="${1:-check}"
GUI="${2:-true}"
TARGET_ALIGNMENT_MODE="${3:-legacy}"
INTEGRAL_PROFILE="${4:-baseline}"
RING_DESCENT_MODE="${5:-legacy}"
PASSIVE_DIAGNOSTICS="${6:-false}"
if [[ "$MODE" != check && "$MODE" != run ]] || [[ "$GUI" != true && "$GUI" != false ]] ||
   [[ "$TARGET_ALIGNMENT_MODE" != legacy && "$TARGET_ALIGNMENT_MODE" != measured_tip_trim ]] ||
   [[ "$INTEGRAL_PROFILE" != baseline && "$INTEGRAL_PROFILE" != integral_045 ]] ||
   [[ "$RING_DESCENT_MODE" != legacy && "$RING_DESCENT_MODE" != measured_tip_tiered ]] ||
   [[ "$PASSIVE_DIAGNOSTICS" != true && "$PASSIVE_DIAGNOSTICS" != false ]]; then
  echo "Usage: bash tools/irl_test/run_m1_ground_ring.sh [check|run] [true|false] [legacy|measured_tip_trim] [baseline|integral_045] [legacy|measured_tip_tiered] [passive_diagnostics:true|false]" >&2
  exit 2
fi
if [[ "$RING_DESCENT_MODE" == measured_tip_tiered && "$TARGET_ALIGNMENT_MODE" != measured_tip_trim ]]; then
  echo "Tiered ring descent requires measured_tip_trim (third argument)." >&2
  exit 2
fi
cd "$ROOT"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
echo "Local target alignment: $TARGET_ALIGNMENT_MODE"
echo "Ground-ring descent: $RING_DESCENT_MODE"
echo "Passive disturbance diagnostics: $PASSIVE_DIAGNOSTICS (control mode unchanged)"
CONTROLLER_TUNING_FILE="$ROOT/src/tejen_mission/config/irl_commissioning.yaml"
if [[ "$INTEGRAL_PROFILE" == integral_045 ]]; then
  CONTROLLER_TUNING_FILE="$ROOT/src/tejen_mission/config/m1_integral_clamp_trial.yaml"
fi
echo "Controller tuning: $INTEGRAL_PROFILE ($CONTROLLER_TUNING_FILE)"
python3 - "$ROOT" "$CONTROLLER_TUNING_FILE" <<'PY'
from pathlib import Path
import sys
import yaml

root = Path(sys.argv[1])
config = root/'src/tejen_mission/config'
base = yaml.safe_load((config/'irl_commissioning.yaml').read_text())
trial = yaml.safe_load((config/'m1_ground_ring.yaml').read_text())
tuning_file = Path(sys.argv[2])
def p(data, node):
    return data[node]['ros__parameters']
errors = []
if tuning_file != config/'irl_commissioning.yaml':
    try:
        controller_tuning = p(yaml.safe_load(tuning_file.read_text()), 'tejen_mpc')
        if controller_tuning != {'xy_bias_mode': 'legacy_integral', 'xy_integral_limit': .45}:
            errors.append('integral_045 overlay must select only legacy_integral and a 0.45 m*s cap')
    except (OSError, TypeError, KeyError, yaml.YAMLError) as exc:
        errors.append(f'controller tuning overlay invalid: {exc}')
try:
    from tejen_mission.measured_tip_alignment import TrackingTipAlignment, TipAlignmentConfig
    tuning = p(yaml.safe_load((config/'m1_measured_tip_tracking.yaml').read_text()), 'online_join_planner')
    TrackingTipAlignment(TipAlignmentConfig(**{k.removeprefix('target_alignment_'):v
                                              for k,v in tuning.items()}))
except (OSError, TypeError, ValueError, ImportError) as exc:
    errors.append(f'local target alignment profile invalid (rebuild if needed): {exc}')
mocap = dict(p(base, 'motion_capture_publisher_irl'), **p(trial, 'motion_capture_publisher_irl'))
planner = dict(p(base, 'online_join_planner'), **p(trial, 'online_join_planner'))
world = dict(p(base, 'fake_cooperative_transport_world'), **p(trial, 'fake_cooperative_transport_world'))
for key, expected in (('pickup_rigid_body_id','6'), ('quad_rigid_body_id','12'),
                      ('magnet_rigid_body_id','22'), ('ring_rigid_body_id','8')):
    if str(mocap.get(key)) != expected:
        errors.append(f'{key} must be {expected}')
if p(base, 'm1_irl_operator').get('lab_config_ready') is not True:
    errors.append('irl_commissioning.yaml lab_config_ready must be true')
if mocap.get('ring_pose_topic') != world.get('measured_ring_pose_topic'):
    errors.append('mocap and ring source topics differ')
if world.get('trajectory_type') != 'measured_stationary':
    errors.append('ring source must be measured_stationary')
if world.get('payload_pose_topic') != planner.get('payload_pose_topic'):
    errors.append('ring centre topic differs between source and planner')
if world.get('attachment_pose_topic') != planner.get('attachment_pose_topic'):
    errors.append('plate-0 topic differs between source and planner')
if planner.get('ground_ring_attach_trial') is not True:
    errors.append('ground-ring trial mode must be true')
if planner.get('use_measured_magnet_tip') is not True:
    errors.append('measured magnet tip must be enabled')
if planner.get('drop_point_source') != 'payload':
    errors.append('drop point must follow live ring centre')
if planner.get('enable_attach_ready_timeout_landing') is not False:
    errors.append('ATTACH_READY auto-LAND must be disabled')
if float(world.get('ring_plate_pitch_diameter_m', 0)) != .4475:
    errors.append('plate pitch diameter must be 0.4475 m')
if int(world.get('ring_attachment_plate_index', -1)) != 0:
    errors.append('attachment target must be plate 0')
if float(world.get('ring_plate_diameter_m', 0)) != .06:
    errors.append('plate diameter must be 0.06 m')
if float(world.get('ring_collision_outer_diameter_m', 0)) != .51:
    errors.append('visual ring OD must be 0.51 m')
if float(planner.get('final_attach_offset_z', 0)) != .0125:
    errors.append('terminal marker offset must be 0.0125 m')
if errors:
    print('M1 GROUND-RING CONFIG: STOP')
    for error in errors:
        print(' -', error)
    raise SystemExit(3)
print('M1 GROUND-RING CONFIG: PASS (pickup6, body12, magnet22, ring8)')
print('Live props-off placement check is still required before ARM.')
PY
if [[ "$MODE" == check ]]; then
  echo "CHECK ONLY: no ROS stack started."
  exit 0
fi
echo "Starting M1 ground-ring stack. No automatic ARM, TAKEOFF or LAND."
exec ros2 launch tejen_mission m1_irl_ground_ring.launch.py gui:="$GUI" target_alignment_mode:="$TARGET_ALIGNMENT_MODE" controller_tuning_file:="$CONTROLLER_TUNING_FILE" ring_descent_mode:="$RING_DESCENT_MODE" lateral_disturbance_diagnostics_enabled:="$PASSIVE_DIAGNOSTICS"
