#!/usr/bin/env bash
# the repo root, from this script's place (experiments/rig/)
REPO="${MDC_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
# nr8 (the 13 Nov trials; rig_nro.sh on the figure-eight): the stop-and-attach on the figure-eight. The triangle (QUAD1-3 on
# 3/11/7, QUAD4 welded on 9, the funnel on 9 if fitted) lifts to 1.0 m, holds 5 s, flies the rig's figure-eight (0.75 m,
# 0.125 m/s, without end since 15 Nov (traj_laps 0 in the params file), contact_rig_fig8 typed); DETACH on the figure-eight; his TAKEOFF pauses it; his re-attach on 9; the
# figure-eight resumes on four; LAND. Anything but a weld on the figure-eight: LAND at once (it freezes the path). MD: his
# MATCH dwell, 0.5. W/X/Y: the funnel's, as nro. contact_rig_fig8: Wesley's word (12 Nov).
# Run:  bash ~/rig_nr8.sh       (dry run: RIG_FLIGHT_DRY_RUN=1 bash ~/rig_nr8.sh)
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
PF=$C/attach_m1_1m_m2a_fig8_det15.yaml   # his TAKEOFF pauses the figure-eight (as the circle's); without end (traj_laps 0)
[ "${PAUSE:-}" = off ] && PF=$C/attach_m1_1m_m2a_fig8_det15_nopause.yaml   # PAUSE=off: the figure-eight does not stop for the re-attach (as nro's PAUSE=off); attach_twist zero acts only on a still ring (ring_bridge), so a moving ring sends its measured twist
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
  load_traj:=fig_8 fig8_size:=0.75 traj_speed:=0.125 contact_rig_fig8:=true
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
tools/rig_flight.sh real_control_launch.py "${ARGS[@]}" partner_args:="${HIS[*]}" "${1:-nr8}"
