#!/usr/bin/env bash
# the repo root, from this script's place (experiments/rig/)
REPO="${MDC_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
# fm9c (until one completes; Wesley 10 Nov: THE FULL MISSION, on the circle): QUAD1-3 on 3/11/7, QUAD4's magnet caught on
# plate 9 (in the funnel), the ring at 1.2 m; lift, 5 s hold, the 0.3 m circle at 0.125 m/s. DETACH on the circle (the
# circle goes on with three); his TAKEOFF (held until PARTNER RELEASE); his pickup of the small object and the drop into
# the CIRCLING ring; when his mission starts the re-attach (TRANSIT_TO_REATTACH or the approach) the circle slows to rest
# over 3 s by itself (attach_pause_at reattach, attach_m1_1m_m2a_orbit_reattach_det15.yaml); his re-attach on 9, our
# co-motion proof, the fold-in; the circle speeds back up with four; LAND. Otherwise exactly fm9o (the tilt drop 55 deg,
# the loss check 15 cm, drop_on_arrival_xy 0.08). contact_rig_orbit: Wesley's word (as nro).
# Run:  bash ~/rig_fm9c.sh      (dry run: RIG_FLIGHT_DRY_RUN=1 bash ~/rig_fm9c.sh)
# CNMAX: our contact window over the plate (Wesley 13 Nov: 0.01 m, was 0.04; CNMAX=0.04 restores it)
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
cd $REPO && source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23 MDC_LEAN_RCLPY=1 MDC_PLANNER_CPUS=0-7
export PYTHONPATH=$REPO/tools/lean_rclpy:$PYTHONPATH
export MDC_RIG_ACADOS_ROOT=$HOME/mdc_rig_generated_m2a_0p86
C=$REPO/configs/rig
W=0.0491; L=1.034; K=0.025; X=0.0; Y=0.2293   # QUAD4 on the M2A ring, body 24 as redefined 4 Nov (decisions 4 Nov ~17:50)
DOA=; DH=0.48   # the drop, hover and circle (Wesley 11 Nov): his settle drop: over the drop point at 0.48, match it, settle, lower, release
[ "${DROP:-}" = arrival ] && { DOA=drop_on_arrival:=true; DH=0.28; }   # DROP=arrival: released at the transit height once the magnet is within 8 cm
PF=$C/attach_m1_1m_m2a_orbit_reattach_det15_stand.yaml   # the ring stops only for his re-attach; + take_back_avoid_pickup_m 0.6 (13 Nov: a take-back never comes down on the stand)
[ "${PAUSE:-}" = pickup ] && PF=$C/attach_m1_1m_m2a_orbit_pickreattach_det15_stand.yaml   # PAUSE=pickup: also at rest for his pickup until confirmed (Wesley 11 Nov)
REL=${RELEASE:-0.03}   # the release: the magnet tip 3 cm above the drop point (RIG-1111-fm9hc_8: in the net, nothing detached); RELEASE=0.14 is his
MXY=0.05; AXY=0.05   # his descent bands: start within 5 cm, pause outside 5 cm (the default since 12 Nov: 3 of 3 caught on 11 Nov, v4 and the supervisor's funnel)
[ "${BAND:-}" = wide ] && { MXY=0.08; AXY=0.10; }   # BAND=wide: the bands as flown 4-5 Nov (BAND=cone is now the default)
[ "${BAND:-}" = mid ] && { MXY=0.06; AXY=0.06; }    # BAND=mid: the step between (Wesley 11 Nov: test the tighter band without getting stuck)
P=$(python3 -c "print(round($L-0.003,3))"); Q=$(python3 -c "print(round($L-0.04,3))")
QR=${QROD:-}   # QROD=0.010: m added to QUAD4's hand-over cable reading (the bounce comparison, 11 Nov; default off)
if [ -n "$QR" ]; then   # a launch reads one params file: a copy of it with the correction appended
  D=$REPO/results/rig/$(date +%F); mkdir -p "$D"; PFQ=$D/params_qrod_$(date +%H%M%S).yaml
  cat "$PF" > "$PFQ"; echo "partner_rod_add_m: $QR" >> "$PFQ"; PF=$PFQ
fi
PC=0.011; PT=0.050; PS=0.105; PX=-0.0045; PY=-0.0107   # the small object WITH the supervisor's funnel on it, body 6 redefined (11 Nov 16:25 and 16:36, two seats a quarter turn apart: contact height 11.4 / 11.0 mm, the magnet seats (-13.1, -9.0) / (-4.9, -10.7) mm from body 6's pivot in the object's axes, the mean typed; 105 mm across; the 50 mm is his obstacle box only)
PB=${PBOX:-1.9}; [ "$PB" = 0 ] && PB=   # the pickup box, 1.9 m from the cage centre in x and y (Wesley 11 Nov 20:19): an object lying further out is not gone after, he gives up and we take QUAD4 back (RIG-1111-fm9c_6: his retry followed the fallen object to y 2.16 m). PBOX=0: off
AR=${ARAD:-0.04}   # his inferred pickup attach only within 4 cm of the seat (the default since 12 Nov, Wesley; Te-Jen agrees): his 0.12 read 'attached' on the funnel rim, then 'lost', latched (RIG-1112-fm9c_3). ARAD=0.12: his
PR=${PRESS:-}   # PRESS=0.02: at the early pickup contact he holds QUAD4 2 cm lower, so the cable goes slack and the magnet settles flat (Wesley 12 Nov)
[ "${OBJ:-}" = bare ] && { PC=0.038; PT=0.022; PS=0.07; PX=0; PY=0; }   # OBJ=bare: the small object without its funnel (4 Nov numbers)
PH=${PHOLD:-0.04}; [ "$PH" = 0 ] && PH=   # the pickup hold (Wesley 11 Nov): his last 8 cm of descent wait while the magnet is more than 4 cm off the object; PHOLD=0 = off
LC=${LIFTCAP:-1}; [ "$LC" = 0 ] && LC=   # lift_cap (13 Nov, the object on Thomas' stand): his lift counts from the object, so from a 1.2 m stand it would end QUAD4 at ~3.9 m; capped at the planned lift height wherever the object lies (a floor pickup unchanged); LIFTCAP=0 = off
[ "${OBJ:-}" = big ] && { PC=0.018; PT=0.010; PS=0.10; PX=0; PY=0; }   # OBJ=big: the bigger object (100 x 10 mm, ~0.15 kg; 5 Nov values). Not for the net: it brought the ring down (5 Nov) and the fly-away of 11 Nov started with it

ARGS=(
  mode:=attach target_z:=1.2 lift_ramp_vel:=0.15 throttle_max:=0.95 enable_approach_mpc:=false
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
  attach_twist:=zero match_xy:=$MXY match_dwell:=0.5 climb_speed:=0.3
  match_speed:=0.20 descent_abort_speed:=0.35 descent_hold:=true
  cpp_authority:=false pickup_xy:=0.06 pickup_speed:=0.20 ${PH:+pickup_low_hold_xy:=$PH}
  pickup_contact_height:=$PC pickup_object_thickness:=$PT pickup_object_size:=$PS pickup_point_x:=$PX pickup_point_y:=$PY ${PB:+pickup_box:=$PB} ${AR:+pickup_attach_radius:=$AR} ${PR:+pickup_press:=$PR}
  pickup_proof_drift:=0.05 pickup_proof_speed:=0.08 pickup_proof_loss:=0.08 ${LC:+lift_cap:=true}
  $DOA drop_on_arrival_xy:=0.08 lift_full:=true mpc_throttle_max:=0.8 drop_height:=$DH ${REL:+drop_release:=$REL} max_body_z:=2.8 line_up:=true
  mpc_battery_comp:=0.022
  geometry:=rig_m2a mission_mode:=pickup_delivery descent_debounce:=1.0 descent_abort_xy:=$AXY contact_give_up_s:=120
  contact_dz:=-0.03
)
tools/rig_flight.sh real_control_launch.py "${ARGS[@]}" partner_args:="${HIS[*]}" "${1:-fm9c}"
