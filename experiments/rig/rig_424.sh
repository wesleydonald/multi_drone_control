#!/usr/bin/env bash
# the repo root, from this script's place (experiments/rig/)
REPO="${MDC_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
# 424 (4 -> 2 -> 4 in hover, the supervisor's item 3; Wesley 13 Nov): the square. QUAD1 and QUAD2 (drone_0/1) carry as the
# pair on plates 0/6 (typed 0,180); QUAD3 (drone_2) starts welded on plate 3 (marker 23, the second funnel) and QUAD4
# (drone_3) on plate 9 (marker 24, its funnel). DETACH BOTH: one set detach releases QUAD3 and QUAD4 together (no T on
# the way down); the pair carries alone; each returner is flown back by its own instance of his stack (QUAD4: /tejen,
# plate 9; QUAD3: /tejen_d2, plate 3), his TAKEOFF BOTH pressed together; our planner folds them in one at a time
# (the second waits for the first's blend). Hover only. The chain's rules (min_survivors 2, the 200 deg gap rule,
# int_k_xy 0) are in attach_424_pair.yaml; each partner's record (tip body, W, X, Y, rod, contact radius, taut length) is
# in partners_424.yaml: QUAD3's is measured on the day (the sheet's setup) and nothing flies while one is unset.
# Motive: 8,11,12,13,14,23,24 only (python3 tools/mocap_stream_check.py --expect 8,11,12,13,14,23,24), NOT recording.
# Run:  bash ~/rig_424.sh       (dry run: RIG_FLIGHT_DRY_RUN=1 bash ~/rig_424.sh)
# CNMAX: our contact window over the plate (0.01 m; CNMAX=0.04 restores the old one). BAND=mid / wide: his descent bands.
# Q3KT: his MPC's thrust ratio for QUAD3 (22 as QUAD4's until Te-Jen says; his kT estimate adapts it in the air).
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
cd $REPO && source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23 MDC_LEAN_RCLPY=1 MDC_PLANNER_CPUS=0-7
export PYTHONPATH=$REPO/tools/lean_rclpy:$PYTHONPATH
export MDC_RIG_ACADOS_ROOT=$HOME/mdc_rig_generated_m2a_0p86   # the 0.86 kg solvers (n = 2, 3 and 4)
C=$REPO/configs/rig
PF=$C/attach_424_pair.yaml
PARTNERS=$C/partners_424.yaml
MXY=0.05; AXY=0.05   # his descent bands: start within 5 cm, pause outside 5 cm (the default since 12 Nov)
[ "${BAND:-}" = wide ] && { MXY=0.08; AXY=0.10; }   # BAND=wide: the bands as flown 4-5 Nov
[ "${BAND:-}" = mid ] && { MXY=0.06; AXY=0.06; }    # BAND=mid: the step between

# the records the launch will fly, on screen before it starts; an unset value (QUAD3's before the setup) stops here
python3 - "$PARTNERS" <<'PY' || { [ "${RIG_FLIGHT_DRY_RUN:-0}" = 1 ] || exit 1; }
import sys, yaml
keys = ('plate', 'tip_body', 'weld_point_z', 'attach_x_offset', 'attach_y_offset', 'attach_cable_len', 'weld_radius',
        'proof_taut_len')
ok = True
for p in (yaml.safe_load(open(sys.argv[1])) or {}).get('partners') or []:
    unset = [k for k in keys if p.get(k) is None]
    print(f"rig_424: QUAD{int(p['drone']) + 1} (drone {p['drone']}): " + ', '.join(f'{k} {p.get(k)}' for k in keys),
          flush=True)
    if unset:
        ok = False
        print(f"rig_424: QUAD{int(p['drone']) + 1}: {', '.join(unset)} not measured: the sheet's setup, then "
              f"{sys.argv[1]}", file=sys.stderr, flush=True)
sys.exit(0 if ok else 1)
PY

ARGS=(
  mode:=attach num_drones:=2 attach_azimuths_deg:=0,180 target_z:=1.0 lift_ramp_vel:=0.15 throttle_max:=0.95
  enable_approach_mpc:=false params_file:=$PF partners_file:=$PARTNERS
  partner:=true partner_contact:=true partner_nodes:=true
  tip_skip_repeats:=true comotion_d_ok:=0.02 comotion_speed_window_s:=0.5 comotion_taut_margin:=0.05
  comotion_normal_min:=-0.03 comotion_normal_max:=${CNMAX:-0.01} comotion_normal_ok:=0.04 comotion_sep_normal:=0.04
  comotion_speed:=0.06 comotion_speed_ok:=0.06 comotion_proof_dwell_s:=0.5 comotion_inconclusive_s:=6.0
  comotion_lost:=0.06 comotion_sep_xy:=0.06
  magnet_operator_topic:=/drone_3/magnet_operator
  partner_attached:=true detach_unload_s:=3.0
  load_traj:=hover
)
# his two instances: the same settings, each its own plate and thrust ratio; his planners see only the pair as carriers
HIS=(
  mpc_cable_length:=1.0 trim_speed_gate:=0.20 carry_clearance:=0.25
  carrier_azimuths_deg:=[0.0,180.0] commit_backdate:=0.04 tracking_check:=off
  attach_cpp_transit:=false attach_transit_clearance:=0.30 minimum_search_z_m:=2.20 approach_offset_z:=0.30
  attach_twist:=zero match_xy:=$MXY match_dwell:=0.5 climb_speed:=0.3
  match_speed:=0.20 descent_abort_speed:=0.35 descent_hold:=true
  geometry:=rig_m2a mission_mode:=floor_join descent_debounce:=1.0 descent_abort_xy:=$AXY contact_give_up_s:=120
  contact_dz:=-0.03
)
HIS_Q4=(mpc_thrust_ratio:=22 plate:=9 "${HIS[@]}")
HIS_Q3=(mpc_thrust_ratio:=${Q3KT:-22} plate:=3 "${HIS[@]}")
tools/rig_flight.sh real_control_launch.py "${ARGS[@]}" partner_args:="${HIS_Q4[*]}" partner_args_d2:="${HIS_Q3[*]}" \
  "${1:-r424}"
