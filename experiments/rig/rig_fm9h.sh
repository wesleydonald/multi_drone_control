#!/usr/bin/env bash
# the repo root, from this script's place (experiments/rig/)
REPO="${MDC_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
# fm9h (fm9's fallback when the pickup struggles in the cage's downwash; Wesley 6 Nov): the object on QUAD4's magnet from
# the start (his mission_mode delivery_rejoin). QUAD1-3 on 3/11/7, the ring at 1.2 m; QUAD4 on its mat on plate 9's line
# holding the small object (attach_magnet_initial ON keeps its coil on from boot). NO detach: one magnet cannot hold the
# object and plate 9 at once, so QUAD4 starts off the ring. His TAKEOFF once the ring is steady at the top: the loaded
# climb, the drop into the net (drop_height 0.28, max_body_z 2.8, as k1z), his re-attach on 9 (descent 3 cm below the
# plate), our co-motion proof, the fold-in; LAND. Otherwise rig_fm9.sh's values. Never flown on the rig (k1_7, 30 Oct,
# was desk-checked only, with end_after_drop).
# Run:  bash ~/rig_fm9h.sh      (dry run: RIG_FLIGHT_DRY_RUN=1 bash ~/rig_fm9h.sh)
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
REL=${RELEASE:-0.03}   # the release: the magnet tip 3 cm above the drop point (RIG-1111-fm9hc_8: in the net, nothing detached); RELEASE=0.14 is his
MXY=0.05; AXY=0.05   # his descent bands: start within 5 cm, pause outside 5 cm (the default since 12 Nov: 3 of 3 caught on 11 Nov, v4 and the supervisor's funnel)
[ "${BAND:-}" = wide ] && { MXY=0.08; AXY=0.10; }   # BAND=wide: the bands as flown 4-5 Nov (BAND=cone is now the default)
[ "${BAND:-}" = mid ] && { MXY=0.06; AXY=0.06; }    # BAND=mid: the step between (Wesley 11 Nov: test the tighter band without getting stuck)
P=$(python3 -c "print(round($L-0.003,3))"); Q=$(python3 -c "print(round($L-0.04,3))")
PC=0.011; PT=0.050; PS=0.105; PX=-0.0045; PY=-0.0107   # the small object WITH the supervisor's funnel on it, body 6 redefined (11 Nov 16:25 and 16:36, two seats a quarter turn apart: contact height 11.4 / 11.0 mm, the magnet seats (-13.1, -9.0) / (-4.9, -10.7) mm from body 6's pivot in the object's axes, the mean typed; 105 mm across; the 50 mm is his obstacle box only)
PB=${PBOX:-1.9}; [ "$PB" = 0 ] && PB=   # the pickup box, 1.9 m from the cage centre in x and y (Wesley 11 Nov 20:19): an object lying further out is not gone after, he gives up and we take QUAD4 back (RIG-1111-fm9c_6: his retry followed the fallen object to y 2.16 m). PBOX=0: off
AR=${ARAD:-}   # ARAD=0.04: his inferred pickup attach only within 4 cm of the seat (his 0.12 read 'attached' mid-descent, then 'lost', latched: RIG-1112-fm9c_3)
PR=${PRESS:-}   # PRESS=0.02: at the early pickup contact he holds QUAD4 2 cm lower, so the cable goes slack and the magnet settles flat (Wesley 12 Nov)
LU=${LINEUP:-true}   # his transits line up on plate 9's line from beyond 1.2 m (4 Nov); LINEUP=gap: the drop transit lines up on the NEAREST carrier gap (12 Nov, for starts at 1 and 5 o'clock; from beyond 1.2 m plate 9's line passes ~9 cm from carrier 11 or 7)
CPA=cpp_authority:=false   # his Python transfer flies the loaded transit (RIG-1029-sq3/-sq4_5: his C++ hand-over and search floor never let it start)
CG=   # his C++ collision geometry, with CPP=on only (measured on the rig 12 Nov; SWING deg, SPLINE, TRACK m, WSXY/WSZ m)
MBZ=2.8; RCL=; [ "${RET:-}" = cpp ] && { MBZ=3.0; RCL="return_climb:=transit"; }   # RET=cpp (13 Nov): his C++ return needs the climb back to the transit height first (twin R4406/R4414), ~2.96 m planned, so max_body_z 3.0 (Wesley: at most 3.8)
WR=${WREF-0.2}; [ "$WR" = 0 ] && WR=   # with CPP=on: his world refresh coalesced to 0.2 s (13 Nov: at real time, without it his planner reached the drop point in 0 of 4 replays of 12 Nov); WREF=0 = off. TOP: his search box top, body z (default 2.9: real-time replays over a still ring 8 of 8 at <= 2.89 m; Wesley 13 Nov: at most 3.8 m)
RX=${RELXY-0.06}; [ "$RX" = 0 ] && RX=   # 13 Nov (Te-Jen, RIG-1113-fm9hc): the release waits for the magnet within 6 cm of the drop point, at most RELWAIT (3) s, then releases anyway; RELXY=0 = off. With CPP=on his hand-over settle bound is PREPSPD (0.20 m/s; his 0.12 reset 3 times)
[ "${EXIT:-}" = moving ] && TOP=${TOP:-3.2}   # EXIT=moving: his loaded search top within the 3.2 m body limit
[ "${CPP:-}" = on ] && { CPA=; LU=; CG="max_swing_angle_rad:=$(python3 -c "import math; print(round(math.radians(${SWING:-5}), 5))") spline_time_factor:=${SPLINE:-4} cooperative_tracking_half_m:=${TRACK:-0.06} swing_envelope:=${ENV:-outer_octagon} target_time_iterations:=${TTI:-30} prepare_settle_speed:=${PREPSPD:-0.20} prepare_settle_accel:=${PREPACC:-0.8} cpp_max_z:=${TOP:-3.3} cpp_splice:=${SPLA:-0.30}${WR:+ cpp_world_refresh:=$WR}${WSXY:+ workspace_xy:=$WSXY workspace_z_max:=${WSZ:-2.9}}"; }   # ENV (12 Nov night): his convex collision shapes; TTI his rendezvous-time steps (6 left the moving drop unconverged at spline 4); RET=cpp|direct: back to the start after the drop, no re-attach (the supervisor's drop-off); his default legacy_square boxes put the drop point inside carriers 7/11 (decisions FINDING 22:26)   # his workspace only with WSXY typed (12 Nov R4366-R4373: on the whole assembly +-1.8 left no plan from a start 1.65-1.9 m out)                               # CPP=on (Te-Jen 12 Nov): his C++ obstacle-avoiding planner flies the loaded transit (carrier_model tether, the default), no line-up of ours
[ "${CPP:-}" = on ] && { DH=${CPPDH:-0.71}; MBZ=3.0; }   # 13 Nov (Te-Jen): his C++ goal over the drop point at 3.00 m (drop_height 0.71: with the rig magnet offset 1.034 m, 0.72 gave 3.004 m and was refused under max_body_z 3.0), the Python descent lowers it into the ring
# EXIT=moving (Te-Jen 15 Nov; frozen at 969e1b41; needs CPP=on RET=cpp): his C++ exit from the moving prefix after
# the drop: the climb to the 2.9 m hold, his C++ return, AT_START, LANDING there 2 s later; body limit 3.2 m.
# Manual LAND during the exit = the escape (the direct return, then LANDING).
XH=; [ "${EXIT:-}" = moving ] && { { [ "${CPP:-}" = on ] && [ "${RET:-}" = cpp ]; } || { echo "EXIT=moving needs CPP=on RET=cpp"; exit 1; }; MBZ=3.2; XH="exit_handoff:=moving"; }
# (his C++ re-attach transit, pickup_cpp_reattach, is wired only in his launch's twin path, not the rig's: the hop from the drop point to plate 9 stays his Python transfer)
[ "${OBJ:-}" = bare ] && { PC=0.038; PT=0.022; PS=0.07; PX=0; PY=0; }   # OBJ=bare: the small object without its funnel (4 Nov numbers)

ARGS=(
  mode:=attach target_z:=1.2 lift_ramp_vel:=0.15 throttle_max:=0.95 enable_approach_mpc:=false
  mocap_magnet_tip_body_id:=24 params_file:=$C/attach_m1_1m_m2a_drop55_det15.yaml
  attach_azimuths_deg:=90,330,210 attach_magnet_initial:=ON
  partner:=true partner_contact:=true partner_nodes:=true
  weld_radius:=$K proof_taut_len:=$P weld_point_z:=$W attach_cable_len:=$Q attach_x_offset:=$X attach_y_offset:=$Y
  tip_skip_repeats:=true comotion_d_ok:=0.02 comotion_speed_window_s:=0.5 comotion_taut_margin:=0.05
  comotion_normal_min:=-0.03 comotion_normal_max:=${CNMAX:-0.01} comotion_normal_ok:=0.04 comotion_sep_normal:=0.04
  comotion_speed:=0.06 comotion_speed_ok:=0.06 comotion_proof_dwell_s:=0.5 comotion_inconclusive_s:=6.0
  comotion_lost:=0.06 comotion_sep_xy:=0.06
  magnet_operator_topic:=/drone_3/magnet_operator
  load_traj:=hover
)
HIS=(
  mpc_thrust_ratio:=22 mpc_cable_length:=1.0 trim_speed_gate:=0.20 carry_clearance:=0.25 plate:=9
  carrier_azimuths_deg:=[90.0,330.0,210.0] commit_backdate:=0.04 tracking_check:=off
  attach_cpp_transit:=false attach_transit_clearance:=0.30 minimum_search_z_m:=2.20 approach_offset_z:=0.30
  attach_twist:=zero match_xy:=$MXY match_dwell:=0.5 climb_speed:=0.3
  match_speed:=0.20 descent_abort_speed:=0.35 descent_hold:=true
  $CPA pickup_xy:=0.06 pickup_speed:=0.20
  pickup_contact_height:=$PC pickup_object_thickness:=$PT pickup_object_size:=$PS pickup_point_x:=$PX pickup_point_y:=$PY ${PB:+pickup_box:=$PB} ${AR:+pickup_attach_radius:=$AR} ${PR:+pickup_press:=$PR}
  pickup_proof_drift:=0.05 pickup_proof_speed:=0.08 pickup_proof_loss:=0.08
  $DOA drop_on_arrival_xy:=0.08 lift_full:=true mpc_throttle_max:=0.8 drop_height:=$DH ${REL:+drop_release:=$REL} ${RX:+drop_release_xy:=$RX drop_release_wait:=${RELWAIT:-3}} max_body_z:=$MBZ ${RET:+return_after_drop:=$RET} $XH $RCL ${LU:+line_up:=$LU} $CG
  mpc_battery_comp:=0.022
  geometry:=rig_m2a mission_mode:=delivery_rejoin descent_debounce:=1.0 descent_abort_xy:=$AXY contact_give_up_s:=120
  contact_dz:=-0.03
)
tools/rig_flight.sh real_control_launch.py "${ARGS[@]}" partner_args:="${HIS[*]}" "${1:-fm9h}"
