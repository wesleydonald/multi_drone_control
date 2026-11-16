#!/usr/bin/env bash
# the repo root, from this script's place (experiments/rig/)
REPO="${MDC_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
# S4 line 2 (nro, x3: nro, nro_2, nro_3; the paper's moving attach): the stop-and-attach on the orbit. The triangle (QUAD1-3
# on 3/11/7, QUAD4 welded on 9, the funnel on 9 if fitted) lifts, holds 5 s, circles (0.3 m, 0.125 m/s); DETACH on the
# circle; his TAKEOFF pauses it; his re-attach on 9; the circle resumes. As RIG-1105-nr9o_2. MD: his MATCH dwell, 0.5
# (Wesley 5 Nov ~21:48: no 1.0 try). W/X/Y: the funnel's, as nrtf. contact_rig_orbit: Wesley's word.
# Run:  bash ~/rig_nro.sh       (dry run: RIG_FLIGHT_DRY_RUN=1 bash ~/rig_nro.sh)
# CNMAX: our contact window over the plate (Wesley 13 Nov: 0.01 m, was 0.04; CNMAX=0.04 restores it)
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
cd $REPO && source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23 MDC_LEAN_RCLPY=1 MDC_PLANNER_CPUS=0-7
export PYTHONPATH=$REPO/tools/lean_rclpy:$PYTHONPATH
export MDC_RIG_ACADOS_ROOT=$HOME/mdc_rig_generated_m2a_0p86
C=$REPO/configs/rig
W=0.0491; L=1.034; K=0.025; X=0.0; Y=0.2293   # QUAD4 on the M2A ring, body 24 as redefined 4 Nov (decisions 4 Nov ~17:50)
MXY=0.05; AXY=0.05   # his descent bands: start within 5 cm, pause outside 5 cm (the default since 12 Nov: 3 of 3 caught on 11 Nov, v4 and the supervisor's funnel)
[ "${BAND:-}" = wide ] && { MXY=0.08; AXY=0.10; }   # BAND=wide: the bands as flown 4-5 Nov (BAND=cone is now the default)
[ "${BAND:-}" = mid ] && { MXY=0.06; AXY=0.06; }    # BAND=mid: the step between (Wesley 11 Nov: test the tighter band without getting stuck)
P=$(python3 -c "print(round($L-0.003,3))"); Q=$(python3 -c "print(round($L-0.04,3))")
PF=$C/attach_m1_1m_m2a_orbit_det15.yaml
[ "${PAUSE:-}" = off ] && PF=$C/attach_m1_1m_m2a_orbit_det15_nopause.yaml   # PAUSE=off: the circle does not stop for the re-attach (Wesley 12 Nov); attach_twist zero acts only on a still ring (ring_bridge), so a moving ring sends its measured twist
QR=${QROD:-}   # QROD=0.010: m added to QUAD4's hand-over cable reading (the bounce comparison, 11 Nov; default off)
if [ -n "$QR" ]; then   # a launch reads one params file: a copy of it with the correction appended
  D=$REPO/results/rig/$(date +%F); mkdir -p "$D"; PFQ=$D/params_qrod_$(date +%H%M%S).yaml
  cat "$PF" > "$PFQ"; echo "partner_rod_add_m: $QR" >> "$PFQ"; PF=$PFQ
fi
MD=0.5

ARGS=(
  mode:=attach target_z:=1.0 lift_ramp_vel:=0.15 throttle_max:=0.95 enable_approach_mpc:=false
  mocap_magnet_tip_body_id:=24 params_file:=$PF
  attach_azimuths_deg:=90,330,210
  partner:=true partner_contact:=true partner_nodes:=true
  weld_radius:=$K proof_taut_len:=$P weld_point_z:=$W attach_cable_len:=$Q attach_x_offset:=$X attach_y_offset:=$Y
  tip_skip_repeats:=true comotion_d_ok:=0.02 comotion_speed_window_s:=0.5 comotion_taut_margin:=0.05
  comotion_normal_min:=-0.03 comotion_normal_max:=${CNMAX:-0.01} comotion_normal_ok:=0.04 comotion_sep_normal:=0.04
  comotion_speed:=0.06 comotion_speed_ok:=0.06 comotion_proof_dwell_s:=0.5 comotion_inconclusive_s:=6.0
  comotion_lost:=0.06 comotion_sep_xy:=0.06
  magnet_operator_topic:=/drone_3/magnet_operator
  partner_attached:=true detach_unload_s:=3.0
  load_traj:=orbit traj_radius:=0.3 traj_speed:=0.125 contact_rig_orbit:=true
)
HIS=(
  mpc_thrust_ratio:=22 mpc_cable_length:=1.0 trim_speed_gate:=0.20 carry_clearance:=0.25 plate:=9
  carrier_azimuths_deg:=[90.0,330.0,210.0] commit_backdate:=0.04 tracking_check:=off
  attach_cpp_transit:=false attach_transit_clearance:=0.30 minimum_search_z_m:=2.20 approach_offset_z:=0.30
  attach_twist:=zero match_xy:=$MXY match_dwell:=$MD climb_speed:=0.3
  match_speed:=0.20 descent_abort_speed:=0.35 descent_hold:=true
  geometry:=rig_m2a mission_mode:=floor_join descent_debounce:=1.0 descent_abort_xy:=$AXY contact_give_up_s:=120
  contact_dz:=-0.03
)
tools/rig_flight.sh real_control_launch.py "${ARGS[@]}" partner_args:="${HIS[*]}" "${1:-nro}"
