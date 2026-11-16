"""Tejen's M1 pickup/drop/rejoin mission flown by drone 3 of our three_attach fleet.

Start AFTER our side is up and the ring is flying:
  ros2 launch bringup sim_control_launch.py mode:=attach partner:=true ...
  ros2 launch tejen_mission partner_mission.launch.py

His nodes live in /tejen (MPC, supervisor, pendulum, magnet manager) so their relative
topics do not meet ours. Boundary:
  * his MPC   -> /drone_3/ELRSCommand_tejen (our mux forwards it until the ring weld)
  * his state <- /drone_3/motion_capture_state (our mocap emulator)
  * his object magnet -> gz /pickup/attach|detach (second DetachableJoint on the partner model)
  * magnets: his planner and object manager use /tejen/magnet/command; /magnet/command is
    our ring magnet. At his ATTACH_READY (/join_planner/handoff_ready) our mux hands drone
    3 to our tracker, which flies the last descent (his ready point was ~0.18 m above the
    plate before the 10 Oct sync, 0.12 m of it from his label-marker aliasing bug), our
    dissipative node commands the ring magnet ON, and the weld folds it in
  * the ring and carriers <- ring_bridge (our ring's mocap + the load planner's trajectory)

contact_attach:=true (rig_twin only; the combined M1 contact attach, docs/design/m1_contact_contract.md):
his planner descends to the contact target and proves the catch by a small lift (LIFT_PROOF);
our /magnet/object_attached ends it in RING_ATTACHED, our /drone_3/take_back or his give-up in
TAKEN_BACK. No handoff_ready. His pickup descent ends on the ball weld (pickup_hold_on_attach) and
his retry is on (pickup_retry_enabled): a pickup not on the ball after his wait rises and
re-approaches the ball, and his third failure is a give-up.
confirm_pickup_release:=true: his supervisor arms only once the
ball weld made at spawn reads free (repeated /pickup/detach), instead of one gz publish.

real:=true (the rig, combined M1; default false = everything above, unchanged): no Gazebo node, the wall
clock, his IRL values (irl_commissioning.yaml blocks copied in with every topic written out; never the file
as a whole: its root topics include our /magnet/object_attached), geometry 'rig', contact_attach required and
contact_target_z typed (the 09:00 calibration's weld_point_z: the marker height with the face on plate 9), and
ring_cable_len typed (the carriers' cable_len; our attach launch passes it with QUAD4's record: his drop is the taut
length, his MPC's cable the drop - 0.044).
His mocap inputs come from our mocap node through partner_rig_state; his ball manager is his IRL one (no
radio output: QUAD4's one coil goes through our magnet_gate). ARM and TAKEOFF stay manual
(/tejen/drone_command), as in his IRL launches; supervisor:=true adds his supervisor, whose stops request a
LAND (land_now), never a disarm. Usually included by real_control_launch.py mode:=attach partner_nodes:=true.

Transit height and collision avoidance (Wesley 21 Oct): unless typed, pickup_lift_height, drop_approach_clearance,
minimum_search_z_m and attach_transit_clearance put the lowest point QUAD4 carries (the object, or the magnet on its
way to the plate) carry_clearance (default half the carriers' cable) above the carriers' bodies, so his C++ planner
has to route it past them (carry_heights). carrier_model:=tether (default) gives his planner the carriers' rods
(his attached-tether model, fed by ring_bridge) and his small carrier boxes; column is the old tall carrier boxes.
attach_cpp_transit:=true (default) flies floor_join's leg to the plate on his C++ planner (TRANSIT_TO_REATTACH).

instance:=d<d> drone_id:=<d> (4 -> 2 -> 4, Wesley 13 Nov): a second instance of his unchanged stack flying our drone
<d>, e.g. instance:=d2 drone_id:=2 plate:=3 carrier_azimuths_deg:=[0.0,180.0]. Every node runs in /tejen_d<d> on
today's names renamed by Instance.t: his /tejen/x -> /tejen_d<d>/x, his root /join_planner/*, /dynamic_planner/*,
/fake_* -> /tejen_d<d>/..., our /drone_3/x -> /drone_<d>/x and our root /magnet/*, /magnet_tip_pose, /partner/release
-> /drone_<d>/...; the ring and object (/payload/*, /pickup/*) are shared. His YAML files reach it as dicts, his backend
is built here (not through his launch) and his MPC follows its own planner. In the twin it is a carrier released to
his stack: floor_join only, no gz object or x3_drone3 pendulum; partner_rig_state and his IRL ball manager as on the
rig, the marker from the emulator's /drone_<d>/magnet_tip_state, his supervisor on the instance's names (start gate
start_gate_topic:=/drone_<d>/partner/release). No instance is today's QUAD4 instance.
"""
import math
import os
import tempfile
from datetime import datetime
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from mpc_planner.geometry import cw_to_ccw_deg, tejen_plate

NS = 'tejen'
DRONE = 3
DRONE_MODEL = 'x3_drone3'
OBJECT_POSE = '/model/payload_model/pose'

# geometry:=legacy is the old partner worlds (our x3_drone3 partner rod, linear plant); geometry:=rig_twin is
# the rig-twin attach world (three_rigid_ground_rig_attach_pickup.sdf, the combined M1 subtest, Wesley 10 Oct):
# plates at r 0.225, 0.55 m rods at 60 deg, QUAD4 0.55 kg with a 0.55 m rod from a pivot 4 cm below the body
# (body to tip 0.59), flown through the rig thrust map, so his MPC takes his rig values (thrust ratio 22 with
# feedback, irl_commissioning.yaml) instead of the linear plant's 82.7.
# plate and carrier_azimuths_deg are typed as everywhere since 15 Oct (Wesley 14 Oct): our plate number, clockwise
# from +x, and clockwise degrees (30 x plate). His nodes count counter-clockwise; _his_ring converts. legacy:
# plate 3 (his 9) = -y = our old weld point (attach_y_offset -0.25), carriers at 30/270/150 cw. rig_twin: plate 9 (his
# 3) = +y (attach_y_offset 0.225), carriers at 90/330/210 cw (plates 3/11/7). contact_attach_z_m: his contact
# target above the ring origin, the tip sphere's centre on the plate (plate top 0.02 + 0.012 radius).
# ring_plate_diameter: his ring model's steel plates (60 mm on the M2A ring; the 28 Oct payload's slots ~100 x 85 mm).
GEOMETRY = {
    'legacy': {
        'ring_radius': 0.25, 'ring_cable_len': 0.5, 'ring_cable_elev_deg': 45.0,
        'ring_pitch_diameter': 0.50, 'ring_collision_outer_diameter': 0.56, 'ring_plate_diameter': 0.06,
        'magnet_drop_below_quad': 0.49, 'mpc_cable_length': 0.49,
        'thrust_ratio': 82.7, 'thrust_ratio_feedback': False, 'vehicle_mass': 0.643,
        'plate': '3', 'carrier_azimuths_deg': '[30.0, 270.0, 150.0]', 'contact_attach_z_m': None,
        'object_bottom_below_tip': 0.16, 'object_rest_z': 0.08,
    },
    'rig_twin': {
        'ring_radius': 0.225, 'ring_cable_len': 0.55, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.45, 'ring_collision_outer_diameter': 0.51, 'ring_plate_diameter': 0.06,
        'magnet_drop_below_quad': 0.59, 'mpc_cable_length': 0.55,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '9', 'carrier_azimuths_deg': '[90.0, 330.0, 210.0]', 'contact_attach_z_m': 0.032,
        'object_bottom_below_tip': 0.16, 'object_rest_z': 0.08,
    },
    # rig_twin with 1 m cables (the carriers' rods and QUAD4's arm; plan 2026-10-15 A1): world
    # three_rigid_ground_rig_attach_pickup_L100.sdf (tools/make_attach_worlds_L100.py)
    'rig_twin_L100': {
        'ring_radius': 0.225, 'ring_cable_len': 1.0, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.45, 'ring_collision_outer_diameter': 0.51, 'ring_plate_diameter': 0.06,
        'magnet_drop_below_quad': 1.04, 'mpc_cable_length': 1.0,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '9', 'carrier_azimuths_deg': '[90.0, 330.0, 210.0]', 'contact_attach_z_m': 0.032,
        'object_bottom_below_tip': 0.16, 'object_rest_z': 0.08,
    },
    # rig_twin_L100 on the 28 Oct payload (RIG-1028-payload): magnets at r 0.204, plate faces 0.025 above the ring
    # origin in the _p115 worlds (tools/make_attach_layout_worlds_p115.py), so the contact target is 0.037 (+ the 0.012
    # tip sphere, as 0.032 above); outer r 0.255 as the M2A ring
    'rig_twin_L100_p115': {
        'ring_radius': 0.204, 'ring_cable_len': 1.0, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.408, 'ring_collision_outer_diameter': 0.51, 'ring_plate_diameter': 0.09,
        'magnet_drop_below_quad': 1.04, 'mpc_cable_length': 1.0,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '9', 'carrier_azimuths_deg': '[90.0, 330.0, 210.0]', 'contact_attach_z_m': 0.037,
        # the rig's pickup object (Wesley 29 Oct; RIG-1021-q1pick): a 100 x 100 x 25 mm (estimated) 0.15 kg magnet plate
        # in the _p115 worlds (tools/make_attach_worlds.py plate_model): centre 0.0125 up at rest, its bottom at most
        # 0.0525 under the tip inside the weld radius
        'object_bottom_below_tip': 0.0525, 'object_rest_z': 0.0125, 'pickup_object': 'plate',
    },
    # rig_twin_L100 (the 0.86 kg M2A ring) with the _p115 worlds' pickup plate (Q011, Wesley 9 Nov): world
    # sq_rigid_ground_rig_attach_pickup_L100_plate.sdf (tools/make_m2a_plate_world.py); the ring as rig_twin_L100, the
    # object as rig_twin_L100_p115
    'rig_twin_L100_plate': {
        'ring_radius': 0.225, 'ring_cable_len': 1.0, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.45, 'ring_collision_outer_diameter': 0.51, 'ring_plate_diameter': 0.06,
        'magnet_drop_below_quad': 1.04, 'mpc_cable_length': 1.0,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '9', 'carrier_azimuths_deg': '[90.0, 330.0, 210.0]', 'contact_attach_z_m': 0.032,
        'object_bottom_below_tip': 0.0525, 'object_rest_z': 0.0125, 'pickup_object': 'plate',
    },
    # real:=true only. The rig layout as rig_twin, his IRL values for QUAD4: body origin to magnet marker 0.575
    # (irl_commissioning.yaml, also his M2 QUAD4 geometry), his MPC's own cable length and kT (his IRL launches
    # set neither: the code defaults 0.531 and 22, with his UKF and kT feedback). The contact target is typed
    # (contact_target_z), not a geometry constant. On the rig the carriers' cable is typed (ring_cable_len, from our
    # launch's cable_len) and his MPC's cable follows QUAD4's typed drop (MPC_CABLE_BELOW_DROP): the 0.55 / 0.531
    # here are the 0.55 m era's. 'rig' is the 28 Oct payload (magnets at r 0.204); geometry:=rig_m2a the M2A ring.
    'rig': {
        'ring_radius': 0.204, 'ring_cable_len': 0.55, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.408, 'ring_collision_outer_diameter': 0.51, 'ring_plate_diameter': 0.09,
        'magnet_drop_below_quad': 0.575, 'mpc_cable_length': 0.531,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '9', 'carrier_azimuths_deg': '[90.0, 330.0, 210.0]', 'contact_attach_z_m': None,
        'object_bottom_below_tip': 0.05, 'object_rest_z': 0.03,
    },
    'rig_m2a': {
        'ring_radius': 0.225, 'ring_cable_len': 0.55, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.45, 'ring_collision_outer_diameter': 0.51, 'ring_plate_diameter': 0.06,
        'magnet_drop_below_quad': 0.575, 'mpc_cable_length': 0.531,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '9', 'carrier_azimuths_deg': '[90.0, 330.0, 210.0]', 'contact_attach_z_m': None,
        'object_bottom_below_tip': 0.05, 'object_rest_z': 0.03,
    },
}
RIG_GEOMETRIES = ('rig', 'rig_m2a')     # real:=true only
# real:=true: the rig's values where the sim launch has its own defaults (a typed value wins)
RIG = {
    'contact_dz': -0.01,          # Wesley 10 Oct: his descent aims the magnet face 1 cm below the plate
    'proof_rise_max': 0.03,       # card rev 5: the rig's lift-proof cap (the twin's is 0.01)
    'anchor_offset_z': -0.030,    # his IRL mocap: QUAD4's body origin to its tether anchor
}
# his calibration on QUAD2 (m1_irl_pickup.launch.py): body to marker 0.575, his controller's pendulum 0.531
MPC_CABLE_BELOW_DROP = 0.044
# the transit height (Wesley 21 Oct): the lowest point QUAD4 carries sits carry_clearance above the carriers' bodies
CARRIER_BODY_ABOVE_PIVOT = 0.04    # pivot_offset_z -0.04: the rod pivot 4 cm below a carrier's body centre
MAGNET_BOTTOM_BELOW_TIP = 0.05     # his ego model's magnet disk: centre 0.025 below the cable end, half-height 0.025
DROP_OFFSET_Z = 0.15               # his planner's drop_offset_z: the drop point above the ring origin
SEARCH_FLOOR_MARGIN = 0.05         # his C++ search floor this far under the transit height
# the low route (Wesley 29 Oct: "I never want the incoming quad to fly that high"): QUAD4 carries the object's bottom
# (or the bare magnet) this far above the ring's plates, as the attach's 0.30 m approach, never over the carriers
LOW_ROUTE_CLEARANCE = 0.30
QUAD4_MAX_BODY_Z = 2.6             # rig: the launch refuses a transit that puts QUAD4's body higher (transit:=over lifts it)
DROP_CLEAR_HEIGHT = 0.45           # his planner's drop_clear_height default (tip above the drop point after the drop)
DROP_RELEASE_CLEARANCE = 0.14      # his planner's drop_release_clearance default (tip above the drop point at the release)
# the low route after the drop: a 3 cm climb from the release (R2898/R2899: his MPC overshot 0.18-0.19 m on losing the
# 0.15 kg plate, to 2.62 m from a 2.43 m climb), and that overshoot counted in the ceiling check
DROP_CLEAR_ABOVE_RELEASE = 0.03
DROP_UNLOAD_OVERSHOOT_M = 0.20
# exit_handoff:=moving (15 Nov, docs/rig/cpp_exit_impl_contract_2026-11-15.md 3.10; typed only, return_after_drop:=cpp):
# his C++ exit seeded from the moving prefix his planner publishes, at the trial line's settled starting values
EXIT_PREFIX_TOPIC = '/dynamic_planner/transfer_prefix'
EXIT_HOLD_Z = 2.9                   # QUAD4's body at the clearance hold
EXIT_MAX_BODY_Z = 3.2               # the trial line's general body limit: max_body_z may be typed up to it
EXIT_PLANNER = {'c1f2_exit_moving_handoff': True, 'exit_prefix_topic': EXIT_PREFIX_TOPIC,
                'exit_ring_commitment_topic': '/fake_payload/committed_trajectory', 'exit_prefix_horizon_s': 5.0,
                'exit_prefix_refresh_s': 1.0, 'exit_blend_s': 1.0, 'exit_protected_s': 1.5,
                'exit_handoff_timeout_s': 7.0, 'exit_hold_z_m': EXIT_HOLD_Z, 'at_start_land_after_s': 2.0}
# his twin supervisor on a return_after_drop line: success is landing at the start once it was reached
RETURN_SUPERVISOR = {'success_phase': 'LANDED_DISARMED', 'success_after_phase': 'AT_START', 'require_handoff_ready': False}
# his backend's exit-only context: 5 deg swing, the +-1.8 m assembly workspace up to the body's top face at 3.2 m
EXIT_BACKEND = {'moving_prefix_topic': EXIT_PREFIX_TOPIC, 'exit_max_swing_angle_rad': math.radians(5.0),
                'exit_workspace_min': [-1.8, -1.8, 0.0], 'exit_workspace_max': [1.8, 1.8, 3.26],
                'exit_max_search_z_m': EXIT_MAX_BODY_Z, 'exit_protected_s': EXIT_PLANNER['exit_protected_s']}
# his geometry-aware pickup on the twin's plate (in place of the spanner_8mm.yaml profile): the target 0.0225 over its
# centre (its top face 0.0125, less his 0.002 overtravel, plus the twin tip's 0.012 sphere)
PLATE_PROFILE = {'payload_name': 'plate_100mm', 'payload_collision_size_x': 0.100, 'payload_collision_size_y': 0.100,
                 'payload_collision_size_z': 0.025, 'payload_pickup_point_x': 0.0, 'payload_pickup_point_y': 0.0,
                 'payload_pickup_point_z': 0.0125, 'magnet_marker_to_contact_face_z': -0.012}


def transit_route(context, floor_join):
    """transit:=low|over; untyped: low for pickup_delivery, over for floor_join (the rig sheet types the attach's low
    heights itself)."""
    typed = LaunchConfiguration('transit').perform(context).strip()
    return typed or ('over' if floor_join else 'low')


def planned_body_heights(*, floor_join, ring_z, attach_z, rod, object_rest_z, lift, drop_approach, drop_clear,
                         attach_clearance, search_floor):
    """QUAD4's planned body heights (m) for each leg of its mission."""
    drop = ring_z + DROP_OFFSET_Z
    legs = {'search floor': search_floor, 'transit to the plate': ring_z + attach_z + attach_clearance + rod}
    if not floor_join:
        legs.update({'lift with the object': object_rest_z + lift + rod, 'over the drop point': drop + drop_approach + rod,
                     'clear of the drop (+ the unload overshoot)': drop + drop_clear + rod + DROP_UNLOAD_OVERSHOOT_M})
    return legs


def check_transit_ceiling(legs, *, refuse, log, ceiling=QUAD4_MAX_BODY_Z, who='QUAD4'):
    """Log the partner's (who) planned heights; over the ceiling (QUAD4_MAX_BODY_Z unless max_body_z is typed, for
    every partner) refuse the launch (rig) or warn (twin)."""
    text = ', '.join(f'{k} {v:.2f}' for k, v in legs.items())
    high = {k: v for k, v in legs.items() if v > ceiling + 1e-6}
    if high and refuse:
        raise RuntimeError(f'partner_mission: {who} would fly above {ceiling} m ('
                           + ', '.join(f'{k} {v:.2f} m' for k, v in high.items())
                           + '): type the low heights (transit:=low) or transit:=over to allow it')
    log(f'{who} planned body heights (m): {text}' + (f'; ABOVE {ceiling} m' if high else ''))


def _drop_height(context):
    """Typed only: drop_height (m, the object's bottom above the ring origin, about the net), else None."""
    v = LaunchConfiguration('drop_height').perform(context).strip()
    return float(v) if v else None


def _drop_release(context):
    """Typed only (Wesley 11 Nov, RIG-1111-k1zo: the drop into the circling ring pulled a carrier's magnet off):
    drop_release (m, the magnet tip above the drop point at his settle drop's release), else None (his 0.14)."""
    v = LaunchConfiguration('drop_release').perform(context).strip()
    return float(v) if v else None


def _release_typed(context):
    """His release clearance: drop_release if typed, else his default."""
    r = _drop_release(context)
    return DROP_RELEASE_CLEARANCE if r is None else r


def _ceiling(context):
    """QUAD4's ceiling: max_body_z if typed (Wesley 4 Nov: the pickup line up to ~2.8 m), else QUAD4_MAX_BODY_Z;
    never above 3.0 m (EXIT_MAX_BODY_Z on the exit_handoff:=moving trial line)."""
    v = LaunchConfiguration('max_body_z').perform(context).strip()
    c = float(v) if v else QUAD4_MAX_BODY_Z
    top = EXIT_MAX_BODY_Z if _exit_moving(context) else 3.0
    if not QUAD4_MAX_BODY_Z <= c <= top:
        raise RuntimeError(f'partner_mission: max_body_z {c} outside [{QUAD4_MAX_BODY_Z}, {top}]')
    return c


def _exit_moving(context):
    """exit_handoff:=moving, typed only and refused unless return_after_drop:=cpp."""
    v = context.launch_configurations.get('exit_handoff', '').strip()
    if v not in ('', 'moving'):
        raise RuntimeError(f'partner_mission: exit_handoff:={v}: only moving')
    if not v:
        return False
    if context.launch_configurations.get('return_after_drop', '').strip() != 'cpp':
        raise RuntimeError('partner_mission: exit_handoff:=moving needs return_after_drop:=cpp')
    return True


def _exit_drop_clear(ring_z, rod):
    """The drop zone's climb (tip above the drop point) that holds QUAD4's body at EXIT_HOLD_Z over a ring at ring_z: the
    launch's height checks see the hold (his planner climbs to exit_hold_z_m from the drop point's measured height)."""
    return EXIT_HOLD_Z - (ring_z + DROP_OFFSET_Z) - rod


def _exit_handoff(context):
    return dict(EXIT_PLANNER) if _exit_moving(context) else {}


def _release_clear(context, drop_clear, drop_approach):
    """With drop_on_arrival the release is at the transit height and the climb after it never goes lower: the
    after-drop leg (plus the unload overshoot) starts there."""
    on = LaunchConfiguration('drop_on_arrival').perform(context).strip() == 'true'
    return max(drop_clear, drop_approach) if on else drop_clear


def carry_heights(G, *, ring_z, rod, attach_z, clearance=None, route='over', drop_height=None):
    """His heights for a transit: pickup_lift_height (the tip above the object at rest), drop_approach_clearance
    (tip above the drop point), attach_transit_clearance (tip above the plate) and the C++ search floor (body) for
    the loaded and the unloaded transit. route 'over': the object's bottom (or, on the way to the plate, the
    magnet's) clearance (default half the carriers' cable) above the carriers' bodies; 'low': the object's bottom
    (or the tip) LOW_ROUTE_CLEARANCE above the ring's plates, in between the carriers as the attach comes in."""
    if route == 'low':
        plate = ring_z + attach_z
        loaded_tip = plate + LOW_ROUTE_CLEARANCE + G['object_bottom_below_tip']
        if drop_height is not None:
            # drop_height (Wesley 4 Nov, his supervisor: drop 0.5-0.6 m above the net, which sits about level with the
            # ring's bottom): the carried object's bottom this far above the ring origin, on the loaded legs only
            loaded_tip = ring_z + drop_height + G['object_bottom_below_tip']
        empty_tip = plate + LOW_ROUTE_CLEARANCE
        return {'pickup_lift_height': loaded_tip - G['object_rest_z'],
                'drop_approach_clearance': loaded_tip - (ring_z + DROP_OFFSET_Z),
                'attach_transit_clearance': LOW_ROUTE_CLEARANCE,
                'minimum_search_z_m': loaded_tip + rod - SEARCH_FLOOR_MARGIN,
                'minimum_search_z_m_empty': empty_tip + rod - SEARCH_FLOOR_MARGIN}
    carriers = ring_z + G['ring_cable_len'] * math.sin(math.radians(G['ring_cable_elev_deg'])) \
        + CARRIER_BODY_ABOVE_PIVOT
    clearance = 0.5 * G['ring_cable_len'] if clearance is None else clearance
    loaded_tip = carriers + clearance + G['object_bottom_below_tip']
    empty_tip = carriers + clearance + MAGNET_BOTTOM_BELOW_TIP
    return {'pickup_lift_height': loaded_tip - G['object_rest_z'],
            'drop_approach_clearance': loaded_tip - (ring_z + DROP_OFFSET_Z),
            'attach_transit_clearance': empty_tip - (ring_z + attach_z),
            'minimum_search_z_m': loaded_tip + rod - SEARCH_FLOOR_MARGIN,
            'minimum_search_z_m_empty': empty_tip + rod - SEARCH_FLOOR_MARGIN}


def _cpp_reattach_heights(height, *, ring_z, attach_z, rod, typed):
    """pickup_cpp_reattach (Wesley 28 Oct, "fix the avoidance"): his backend keeps ONE search floor in pickup_delivery,
    the loaded leg's, so the hand-over at the end of CLEAR_DROP_ZONE and the C++ transit's end must sit above it:
    the drop-zone climb ends at the drop approach's height, the transit end 0.05 m over the floor. Typed values win."""
    floor = height('minimum_search_z_m')
    clear = height('drop_approach_clearance')
    out = {'drop_clear_height': clear}
    if 'attach_transit_clearance' not in typed:
        out['attach_transit_clearance'] = max(height('attach_transit_clearance'), floor + 0.05 - rod - (ring_z + attach_z))
    check_search_floor(floor, what='drop-zone hand-over', start_body_z=ring_z + DROP_OFFSET_Z + clear + rod)
    check_search_floor(floor, what='reattach transit end',
                       start_body_z=ring_z + attach_z + out.get('attach_transit_clearance',
                                                                height('attach_transit_clearance')) + rod)
    return out


def check_search_floor(floor, *, start_body_z, what, margin=0.0):
    """His backend waits below its search floor (WAITING_ABOVE_SEARCH_FLOOR) for ever: refuse a floor above where the
    transit starts (an old typed minimum_search_z_m with the new transit heights, say), less margin."""
    if floor > start_body_z - margin + 1e-6:
        raise RuntimeError(f'partner_mission: minimum_search_z_m {floor:.2f} is within {margin:.2f} m of (or above) the '
                           f'{what} ({start_body_z:.2f} m body): his C++ transit would never start; type a lower floor')


def _backend_margins(context):
    """Typed only: his ego swing envelope (max_swing_angle_rad) and the carriers' tracking inflation
    (cooperative_tracking_half_m, x y z); untyped, his config files' values stand."""
    out = {}
    swing = LaunchConfiguration('max_swing_angle_rad').perform(context).strip()
    if swing:
        out['max_swing_angle_rad'] = float(swing)
    track = LaunchConfiguration('cooperative_tracking_half_m').perform(context).strip()
    if track:
        out.update({f'cooperative_tracking_half_{a}_m': float(track) for a in 'xyz'})
    # spline_time_factor (typed only; Te-Jen 12 Nov: a slower transit keeps the real swing inside a smaller swing bound)
    # cooperative_body_half_m (typed only; Te-Jen's overnight sweep limit 0.09): the carriers' body box, x and y (his tether
    # model's 0.105 otherwise)
    body = LaunchConfiguration('cooperative_body_half_m').perform(context).strip()
    if body:
        out.update({'cooperative_body_half_x_m': float(body), 'cooperative_body_half_y_m': float(body)})
    spline = LaunchConfiguration('spline_time_factor').perform(context).strip()
    if spline:
        out['spline_time_factor'] = float(spline)
    # swing_envelope (typed only, 12 Nov): his swing_envelope_shape; outer_octagon pairs his exact convex components where
    # legacy_square (his default) boxes each one, which closes the gap between carriers 7 and 11 over the drop point
    # return_after_drop:=cpp (12 Nov): his backend predicts the return's goal (the start hover) still while his planner
    # says so on this topic, instead of moving it with the basket's committed future
    if LaunchConfiguration('return_after_drop').perform(context).strip() == 'cpp':
        out['static_target_topic'] = '/dynamic_planner/static_target'
    if _exit_moving(context):
        # the line's general body limit (max_body_z, 3.2 m on the trial) bounds his searches too: the exit's, and the
        # loaded approach's (review 15 Nov: cpp_max_z 3.3 left the loaded search above it); assembly bounds unchanged
        out.update({**EXIT_BACKEND, 'exit_max_search_z_m': min(EXIT_MAX_BODY_Z, _ceiling(context))})
    # cpp_max_z (typed only, 13 Nov): the top of his local search box, QUAD4's body z (his start/goal + 1 m otherwise;
    # he climbed to 3.2-3.6 m on a moving ring in the twin, the rig keeps QUAD4 under 2.8-3.0 m)
    # temporal_refinement:=true (typed only, 13 Nov; Te-Jen's Codex: his existing, offline-tested opt-in): his collision
    # certificates subdivide each planning interval in common time, so the carriers' swept hulls stop closing the gap
    if LaunchConfiguration('temporal_refinement').perform(context).strip() == 'true':
        out['temporal_refinement_enabled'] = True
    # cpp_world_refresh (typed only, 13 Nov; real-time replay of 12 Nov: ~214 world versions/s discarded every outer_octagon
    # candidate in his final recheck): his backend coalesces the carriers' state/plate/ring-pose refreshes to this period (s)
    # cpp_splice (typed only, 13 Nov; real-time replays under 8 busy cores: his replans run 90-150 ms, so at his 0.1 s splice
    # lookahead 83 % arrived late and one run stalled; 0.15 s: 1 % late, 4 of 4 captured): his fixed_splice_lookahead_s
    splice = LaunchConfiguration('cpp_splice').perform(context).strip()
    if splice:
        out['fixed_splice_lookahead_s'] = float(splice)
    refresh = LaunchConfiguration('cpp_world_refresh').perform(context).strip()
    if refresh:
        out['cooperative_world_refresh_period_s'] = float(refresh)
    top = LaunchConfiguration('cpp_max_z').perform(context).strip()
    if _exit_moving(context):
        ceiling = _ceiling(context)
        if top and float(top) > ceiling + 1e-9:
            raise RuntimeError(f'partner_mission: cpp_max_z {float(top)} is above max_body_z {ceiling}, the '
                               'exit_handoff:=moving line\'s general body limit (the loaded approach included)')
        out['maximum_search_z_m'] = float(top) if top else ceiling
    elif top:
        out['maximum_search_z_m'] = float(top)
    envelope = LaunchConfiguration('swing_envelope').perform(context).strip()
    if envelope:
        out['swing_envelope_shape'] = envelope
    # target_time_iterations (typed only, 12 Nov): his target_time_fixed_point_iterations (6). His rendezvous time is a
    # fixed point that contracts by about spline_time_factor x target speed / QUAD4's speed per step; at spline 4 six steps
    # left 28-40 % of the circle's replans TARGET_TIME_FIXED_POINT_NOT_CONVERGED (R4377, R4382; 7 % at his 3, R4362)
    iterations = LaunchConfiguration('target_time_iterations').perform(context).strip()
    if iterations:
        out['target_time_fixed_point_iterations'] = int(iterations)
    # workspace_xy / workspace_z_max (typed only; Te-Jen 12 Nov, the lab's limits): his workspace bounds on the whole
    # assembly, x and y within +-workspace_xy, z from 0 to workspace_z_max (his default: off)
    ws_xy = LaunchConfiguration('workspace_xy').perform(context).strip()
    ws_z = LaunchConfiguration('workspace_z_max').perform(context).strip()
    if ws_xy or ws_z:
        if not (ws_xy and ws_z):
            raise RuntimeError('partner_mission: type workspace_xy and workspace_z_max together')
        xy, zmax = float(ws_xy), float(ws_z)
        out.update({'workspace_bounds_enabled': True, 'workspace_min': [-xy, -xy, 0.0], 'workspace_max': [xy, xy, zmax]})
    # tracking_check:=off: his IRL policy (c1f_irl_m1_virtual_ring.yaml): a carrier off its published path no longer
    # disables his backend for good; his collision inflation by the tracking half-extents stays
    if LaunchConfiguration('tracking_check').perform(context).strip() == 'off':
        out['cooperative_tracking_safety_enabled'] = False
    return out


def _backend_override_params(G, *, rod, floor_join, carrier_model, margins=None, count=3):
    """His backend's overrides after our file: QUAD4's own cable length, no wait for an object on the empty
    (floor_join) transit, and with carrier_model tether his attached-tether model of the carriers' rods (their plates,
    one per listed carrier, and the ring pose from ring_bridge) with his small carrier boxes; then any typed margins
    (_backend_margins)."""
    params = {'suspended_cable_length_m': float(rod),
              'require_payload_attached_for_enable': not floor_join}
    if carrier_model == 'tether':
        params.update({
            'cooperative_body_half_x_m': 0.105, 'cooperative_body_half_y_m': 0.105, 'cooperative_body_half_z_m': 0.060,
            'cooperative_attached_tether_enabled': True,
            'cooperative_attached_plate_topics': [f'/fake_obstacles/drone_{i}/attached_plate' for i in range(count)],
            'cooperative_ring_pose_topic': '/fake_payload/pose_array',
            'cooperative_ring_pose_index': 0,
            'cooperative_ring_plate_pitch_diameter_m': float(G['ring_pitch_diameter']),
            'cooperative_ring_angle_zero_rad': 0.0,
            'cooperative_tether_anchor_z_m': -CARRIER_BODY_ABOVE_PIVOT,
        })
    params.update(margins or {})        # typed values last (12 Nov: a typed carrier body box must win over the tether model's)
    return params


def _backend_overrides(base, G, *, rod, floor_join, carrier_model, margins=None, count=3):
    """Our override file, then _backend_override_params written to a file keyed by his node name. Returns the comma
    list for his backend launch."""
    params = _backend_override_params(G, rod=rod, floor_join=floor_join, carrier_model=carrier_model, margins=margins,
                                      count=count)
    out = tempfile.NamedTemporaryFile('w', prefix='partner_backend_', suffix='.yaml', delete=False)
    yaml.safe_dump({BACKEND_NODE: {'ros__parameters': params}}, out)
    out.close()
    return f'{base},{out.name}'


def _throttle_max(context):
    """Typed only (rig and twin): mpc_throttle_max raises his OCP's 0.60 throttle bound at run time (Te-Jen's review of
    RIG-1104-k1m_3: carrying the object QUAD4 sat at 0.60 for 17 s and sank 0.4 m under his reference)."""
    v = LaunchConfiguration('mpc_throttle_max').perform(context).strip()
    b = LaunchConfiguration('mpc_battery_comp').perform(context).strip()
    # mpc_battery_comp (typed only): his radio throttle + this per volt below 23.5 V (the rig's identified 0.022)
    return {**({'throttle_state_max': float(v)} if v else {}), **({'battery_comp_slope': float(b)} if b else {})}


def _return_after_drop(context):
    """Typed only (rig and twin; 12 Nov, the supervisor's drop-off, start -> ring -> start): after the drop zone is clear
    his planner flies QUAD4 back to its start hover and holds it there until his LAND; 'cpp' on his C++ planner, 'direct'
    on his Python transfer (the backup)."""
    v = LaunchConfiguration('return_after_drop').perform(context).strip()
    if not v:
        return {}
    if LaunchConfiguration('end_after_drop').perform(context).strip() == 'true':
        raise RuntimeError('partner_mission: return_after_drop and end_after_drop exclude each other')
    if v == 'cpp' and LaunchConfiguration('cpp_authority').perform(context).strip() == 'false':
        raise RuntimeError('partner_mission: return_after_drop:=cpp needs his C++ planner (not cpp_authority:=false)')
    return {'return_after_drop': v}


def _cpp_python_fallback(context):
    """Typed only (13 Nov, RIG-1113-fm9h8_3): his C++ authority lost this long (s) in the drop transit -> his Python
    transfer flies on (online_join_planner cpp_python_fallback_s)."""
    v = LaunchConfiguration('cpp_python_fallback').perform(context).strip()
    return {'cpp_python_fallback_s': float(v)} if v else {}


def _prepare_and_release(context):
    """Typed only (13 Nov, Te-Jen at the rig): prepare_settle_speed -> his hand-over settle speed bound
    (c1f2_prepare_settle_speed_mps; RIG-1113-fm9hc: QUAD4's hover 0.13-0.22 m/s reset it 3 times at 0.12);
    drop_release_xy / drop_release_wait -> the release waits for the magnet within that of the drop point, at most that
    long (drop_release_xy_m, drop_release_wait_s)."""
    out = {}
    for arg, key in (('prepare_settle_speed', 'c1f2_prepare_settle_speed_mps'), ('prepare_settle_accel', 'c1f2_prepare_settle_acceleration_mps2'),
                     ('drop_release_xy', 'drop_release_xy_m'),
                     ('drop_release_wait', 'drop_release_wait_s')):
        v = LaunchConfiguration(arg).perform(context).strip()
        if v:
            out[key] = float(v)
    return out


def _takeoff_gate(context):
    """Typed only (our combined rig launch types it for a start-welded QUAD4): his TAKEOFF waits for this topic."""
    topic = LaunchConfiguration('takeoff_gate_topic').perform(context).strip()
    return {'takeoff_gate_topic': topic} if topic else {}


def _drop_on_arrival(context):
    """Typed only (rig and twin): drop_on_arrival:=true releases at the transit height once the magnet is within
    drop_on_arrival_xy ('' = 0.12 m) of the drop point for 0.1 s, skipping his settle and descent."""
    if LaunchConfiguration('drop_on_arrival').perform(context).strip() != 'true':
        return {}
    xy = LaunchConfiguration('drop_on_arrival_xy').perform(context).strip()
    return {'drop_on_arrival': True, **({'drop_on_arrival_xy_m': float(xy)} if xy else {})}


def line_up_gap_offsets(plate, carriers_cw):
    """The carrier gaps' bisectors as clockwise offsets (deg) from the plate's line, from our plate number and the
    carriers' clockwise azimuths: plate 9 with carriers [90, 330, 210] -> [0, 120, 240] (plates 9, 1, 5)."""
    az = sorted(float(a) % 360.0 for a in carriers_cw)
    gaps = [((a + ((b - a) % 360.0) / 2.0) % 360.0) for a, b in zip(az, az[1:] + az[:1])]
    return sorted(round((g - 30.0 * int(plate)) % 360.0, 6) for g in gaps)


def _lift_full(context, G=None):
    """Typed only (rig and twin): lift_full:=true ends his loaded lift only at its full height (up, then over);
    line_up:=true sends his transits to the ring along the plate's line first (line_up_radius, line_up_lateral);
    line_up:=gap (12 Nov) does the same, but the transit to the drop point lines up on the nearest carrier gap."""
    v = lambda name: LaunchConfiguration(name).perform(context).strip()  # noqa: E731
    out = {'lift_full_before_transit': True} if v('lift_full') == 'true' else {}
    if v('line_up') == 'gap':
        G = G or {}
        out['line_up_gap_offsets_deg'] = line_up_gap_offsets(
            v('plate') or G.get('plate', '9'),
            yaml.safe_load(v('carrier_azimuths_deg') or G.get('carrier_azimuths_deg', '[90.0, 330.0, 210.0]')))
    if v('line_up') in ('true', 'gap'):
        out['line_up'] = True
        out.update({k: float(v(a)) for a, k in (('line_up_radius', 'line_up_radius_m'),
                                                ('line_up_lateral', 'line_up_lateral_m')) if v(a)})
    return out


def _lift_cap(context, planned_tip_z):
    """Typed only (rig and twin; 12 Nov, the object raised on a stand): lift_cap:=true ends his loaded lift with the tip
    no higher than planned_tip_z (the planned loaded tip: object_rest_z + pickup_lift_height), wherever the object lay;
    untyped, his lift counts pickup_lift_height from the object, so an object on a 1.2 m stand puts QUAD4 1.2 m above the
    plan (~3.9 m on the rig), which the ceiling check above does not see."""
    if LaunchConfiguration('lift_cap').perform(context).strip() != 'true':
        return {}
    return {'pickup_lift_max_tip_z': round(float(planned_tip_z), 4)}


# his planner's pickup values from his last successful IRL pickup (m1_ground_ring.yaml, 7 Oct 17:53); the
# rest of that file is his ground-ring trial, which contact_attach refuses
GROUND_RING_PICKUP_KEYS = ('min_reference_z', 'pickup_overtravel', 'pickup_max_overtravel',
                           'pickup_support_surface_overtravel_allowance_m')
TYPED_KEY = 'partner_mission_typed_args'
# the topics the contact attach adds to the diagnosis bag
CONTACT_TOPICS = ['/magnet/contact', '/magnet/object_attached', '/magnet/attach_proof',
                  '/join_planner/give_up', f'/drone_{DRONE}/take_back', f'/drone_{DRONE}/mux_state']
PICKUP_STATE = '/pickup/detachable_joint_state'
RELEASE_GATE = '/partner/release'    # our planner's release of a start-welded QUAD4, in the air after its step-out

# A second instance of his stack (4 -> 2 -> 4, Wesley 13 Nov; mdc_424 CONTRACT.md): instance:=d<d> drone_id:=<d> puts
# every node and topic of his under /tejen_d<d> and our drone's own topics under /drone_<d>/; instance '' is today's
# QUAD4 instance, untouched. The other instance's names are today's names through Instance.t (the contract's rule).
SHARED_TOPICS = ('/payload/', '/pickup/')     # one ring, one object: every instance reads the same bodies
OUR_TOPICS = ('/magnet/', '/magnet_tip_pose', '/partner/')     # ours for QUAD4 at the root: /drone_<d> in front
# his nodes' topic parameters with absolute code defaults: an instance not at the root types every one of them, since
# one left at its default would join the other instance's topic (online_join_planner.py, transfer_backend_node.cpp,
# tejen_mpc/main.py, fake_cooperative_transport_world.py + ring_bridge.py; test_partner_mission_instances checks the lists)
PLANNER_TOPICS = {
    'drone_state_topic': '/motion_capture_state', 'attachment_pose_topic': '/fake_attachment_point/pose',
    'attachment_twist_topic': '/fake_attachment_point/twist', 'payload_pose_topic': '/fake_payload/pose',
    'payload_twist_topic': '/fake_payload/twist', 'pickup_object_pose_topic': '/model/payload_model/pose',
    'object_attached_topic': '/magnet/object_attached', 'magnet_tip_pose_topic': '/magnet_tip_pose',
    'reference_topic': '/join_planner/reference', 'marker_topic': '/join_planner/markers',
    'state_topic': '/join_planner/state', 'phase_topic': '/join_planner/phase',
    'handoff_ready_topic': '/join_planner/handoff_ready',
    'obstacle_diagnostics_topic': '/join_planner/obstacle_diagnostics', 'magnet_command_topic': '/magnet/command',
    'external_landing_topic': '/join_planner/land_now', 'operator_advance_override_topic': '/join_planner/advance_override',
    'c1f1_shadow_target_topic': '/dynamic_planner/transfer_target',
    'c1f1_shadow_enable_topic': '/dynamic_planner/transfer_enable',
    'c1f1_shadow_reference_topic': '/dynamic_planner/transfer_reference',
    'c1f1_shadow_status_topic': '/dynamic_planner/transfer_status',
    'c1f2_target_velocity_topic': '/dynamic_planner/transfer_target_velocity',
    'c1f2_authority_topic': '/dynamic_planner/transfer_authority',
    'c1f2_authority_ack_topic': '/dynamic_planner/transfer_authority_ack',
    'c1f2_recovery_request_topic': '/dynamic_planner/reference_recovery_request',
    'c1f2_recovery_status_topic': '/dynamic_planner/reference_recovery_status',
    'c1f2_recovery_continuation_topic': '/dynamic_planner/transfer_reference_continuation',
    'exit_prefix_topic': EXIT_PREFIX_TOPIC, 'exit_ring_commitment_topic': '/fake_payload/committed_trajectory',
    'm2d_mission_permission_topic': '/m2d/mission_permission', 'm2d_assigned_plate_topic': '/m2d/assigned_plate_id',
    'm2b_joint_truth_topic': '/m2a/sim/joint_detached_truth', 'm2b_bootstrap_release_topic': '/m2b/bootstrap/release',
    'm2b_raw_detach_request_topic': '/m2b/sim/raw_detach_request',
    'm2b_arm_permission_topic': '/join_planner/arm_permission', 'm2b_mpc_mode_topic': '/join_planner/mpc_mode',
    'm2b_attachment_diagnostics_topic': '/m2a/attachment/diagnostics',
    'm2b_proof_requested_topic': '/m2a/attachment/proof_requested', 'm2b_detector_reset_topic': '/m2a/attachment/reset',
    'm2b_attempt_topic': '/m2b/attempt_number', 'm2b_latch_stable_topic': '/m2b/latch_stable',
    'm2b_latch_stability_topic': '/m2b/latch_stability', 'm2b_pendulum_topic': '/pendulum_swing_state',
    'm2b_ring_pose_topic': '/model/payload_model/pose', 'moving_obstacle_state_topic': '/fake_moving_obstacle/state',
    'contact_topic': '/magnet/contact', 'ring_attached_topic': '/magnet/object_attached',
    'give_up_topic': '/join_planner/give_up', 'take_back_topic': '/drone_3/take_back',
    'mux_state_topic': '/drone_3/mux_state', 'proof_lift_complete_topic': '/join_planner/proof_lift_complete',
}
# his planner's topics written in its code, not parameters: remapped for an instance not at the root
PLANNER_FIXED_TOPICS = ('/join_planner/force_drop', '/dynamic_planner/static_target',
                        '/experiments/navigation_ready', '/experiments/start_navigation')
BACKEND_TOPICS = {
    'state_topic': '/motion_capture_state', 'target_topic': '/dynamic_planner/transfer_target',
    'target_velocity_topic': '/dynamic_planner/transfer_target_velocity', 'object_attached_topic': '/magnet/object_attached',
    'enable_topic': '/dynamic_planner/transfer_enable', 'authority_topic': '/dynamic_planner/transfer_authority',
    'authority_ack_topic': '/dynamic_planner/transfer_authority_ack',
    'recovery_request_topic': '/dynamic_planner/reference_recovery_request',
    'recovery_status_topic': '/dynamic_planner/reference_recovery_status',
    'shadow_reference_topic': '/dynamic_planner/transfer_reference',
    'recovery_continuation_topic': '/dynamic_planner/transfer_reference_continuation',
    'diagnostics_topic': '/dynamic_planner/transfer_status', 'marker_topic': '/dynamic_planner/markers',
    'cooperative_ring_pose_topic': '/model/payload_model/pose',
    'moving_basket_committed_trajectory_topic': '/fake_payload/committed_trajectory',
}
MPC_TOPICS = {'external_arm_permission_topic': '/join_planner/arm_permission',
              'payload_mpc_mode_topic': '/join_planner/mpc_mode', 'external_reference_topic': '/join_planner/reference'}
RING_TOPICS = {
    'payload_pose_topic': '/fake_payload/pose', 'payload_twist_topic': '/fake_payload/twist',
    'attachment_pose_topic': '/fake_attachment_point/pose', 'attachment_twist_topic': '/fake_attachment_point/twist',
    'obstacle_pose_array_topic': '/fake_obstacles/poses', 'obstacle_state_topic_prefix': '/fake_obstacles/drone',
    'marker_topic': '/fake_world/markers', 'committed_trajectory_topic_prefix': '/fake_obstacles/drone',
    'payload_committed_trajectory_topic': '/fake_payload/committed_trajectory',
    'measured_ring_pose_topic': '/irl/ring/mocap_pose', 'ring_pose_array_topic': '/fake_payload/pose_array',
    'attached_plate_topic_prefix': '/fake_obstacles/drone',
}
BALL_TOPICS = {'magnet_command_topic': '/magnet/command', 'object_attached_topic': '/magnet/object_attached',
               'attachment_state_topic': '/magnet/attachment_state', 'magnet_tip_pose_topic': '/magnet_tip_pose',
               'pickup_object_pose_topic': '/irl/pickup_object/pose', 'elrs_magnet_command_topic': '/magnet/ELRSCommand'}
BALL_FIXED_TOPICS = ('/join_planner/phase',)    # his IRL manager's, only with its preloaded takeoff override
SUPERVISOR_FIXED_TOPICS = ('/join_planner/phase', '/join_planner/handoff_ready', '/dynamic_planner/transfer_status',
                           '/join_planner/land_now', '/join_planner/give_up')    # his twin supervisor's
RING_STATE = '/payload/motion_capture_state'
BACKEND_NODE = 'dynamic_planner_transfer_backend'


def _is_topic_key(key):
    return key.endswith(('_topic', '_topics', '_topic_prefix'))


class Instance:
    """One instance of his stack: '' is today's (QUAD4, drone 3, /tejen, his topics at their root names); 'd<d>' runs
    every node in /tejen_d<d> on its own names. node() builds today's node for '' and that node renamed otherwise."""

    def __init__(self, name='', drone=DRONE):
        self.name, self.drone = name, int(drone)
        self.ns = f'{NS}_{name}' if name else NS
        self.vehicle_id = f'drone_{self.drone}' if name else 'drone_0'    # his label; 'drone_0' as his M1 launches
        self.quad = f'QUAD{self.drone + 1}'      # ours: drone_<d> is QUAD<d+1>

    @property
    def legacy(self):
        return not self.name

    def t(self, topic):
        """Today's (QUAD4's) name of a topic -> this instance's: ours /drone_3/x -> /drone_<d>/x and /magnet/x,
        /magnet_tip_pose, /partner/x -> /drone_<d>/...; his /tejen/x -> /tejen_d<d>/x and every other absolute name of
        his under /tejen_d<d>; the shared ring and object, relative names and names already this instance's unchanged."""
        if self.legacy or not isinstance(topic, str) or not topic.startswith('/') or topic.startswith(SHARED_TOPICS):
            return topic
        if topic.startswith((f'/{self.ns}/', f'/drone_{self.drone}/')):
            return topic
        if topic.startswith(f'/drone_{DRONE}/'):
            return f'/drone_{self.drone}/' + topic[len(f'/drone_{DRONE}/'):]
        if topic.startswith(OUR_TOPICS):
            return f'/drone_{self.drone}' + topic
        if topic.startswith(f'/{NS}/'):
            return f'/{self.ns}/' + topic[len(NS) + 2:]
        return f'/{self.ns}' + topic

    def topics(self, value):
        return [self.t(v) for v in value] if isinstance(value, (list, tuple)) else self.t(value)

    def node(self, *, topics=None, fixed=(), extra=None, **kw):
        """Node(**kw) for today's instance. Otherwise the same node in this instance's namespace with ONE parameter dict:
        his files loaded as dicts (rclpy and rclcpp match a file's node-name key against the full node name, so a
        namespaced node would silently drop the file), every topic parameter renamed, including those left at an
        absolute code default (topics), the code's fixed topics remapped (fixed), then extra."""
        if self.legacy:
            return Node(**kw)
        merged = {}
        for entry in kw.get('parameters') or []:
            if isinstance(entry, dict):
                merged.update(entry)
                continue
            data = yaml.safe_load(Path(entry).read_text()) or {}
            if kw['name'] not in data:
                raise RuntimeError(f'partner_mission instance {self.name}: {entry} has no {kw["name"]} block')
            merged.update(data[kw['name']]['ros__parameters'])
        for key in {*(topics or {}), *(k for k in merged if _is_topic_key(k))}:
            merged[key] = self.topics(merged.get(key, (topics or {}).get(key)))
        merged.update(extra or {})
        kw.update(namespace=self.ns, parameters=[merged],
                  remappings=[(a, self.t(b)) for a, b in kw.get('remappings') or []] + [(f, self.t(f)) for f in fixed])
        return Node(**kw)


def _instance(context):
    """instance:=d<d> drone_id:=<d> (both, or neither: today's QUAD4 instance)."""
    name = LaunchConfiguration('instance').perform(context).strip()
    drone = LaunchConfiguration('drone_id').perform(context).strip()
    if not name:
        if drone not in ('', str(DRONE)):
            raise RuntimeError(f'partner_mission: drone_id:={drone} needs instance:=d{drone} (no instance is QUAD4, '
                               f'drone {DRONE})')
        return Instance()
    if not drone.isdigit() or name != f'd{drone}' or int(drone) == DRONE:
        raise RuntimeError(f'partner_mission: instance:={name} drone_id:={drone}: a second instance is instance:=d<d> '
                           f'drone_id:=<d> for a drone other than {DRONE} (QUAD4 is the instance without a name)')
    return Instance(name, int(drone))


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('1', 'true', 'yes', 'on')


def _his_ring(plate, carriers):
    """(his plate index, his carrier azimuths) from the typed plate (our number, clockwise from +x: 9 is +y) and
    carrier_azimuths_deg (clockwise degrees, '[90.0, 330.0, 210.0]'): his nodes count counter-clockwise."""
    return tejen_plate(int(plate)), cw_to_ccw_deg([float(a) for a in yaml.safe_load(carriers)])


def generate_launch_description() -> LaunchDescription:
    # _record_typed first: the rig mode tells a typed value from a sim default by it
    return LaunchDescription([OpaqueFunction(function=_record_typed)] + _arguments()
                             + [OpaqueFunction(function=_nodes)])


def _record_typed(context):
    """Note the arguments set before the declarations ran (the command line or an including launch)."""
    context.launch_configurations[TYPED_KEY] = ','.join(
        sorted(k for k in context.launch_configurations if k != TYPED_KEY))
    return []


def _nodes(context):
    inst = _instance(context)
    if _flag(context, 'real'):
        return _rig_nodes(context)
    geometry = LaunchConfiguration('geometry').perform(context)
    if geometry not in GEOMETRY or geometry in RIG_GEOMETRIES:
        raise RuntimeError(f'partner_mission: geometry:={geometry!r}, expected one of '
                           f'{sorted(g for g in GEOMETRY if g not in RIG_GEOMETRIES)} '
                           f'({"/".join(RIG_GEOMETRIES)} are real:=true)')
    G = GEOMETRY[geometry]
    contact = _flag(context, 'contact_attach')
    if contact and G['contact_attach_z_m'] is None:
        raise RuntimeError(f'partner_mission: contact_attach:=true has no contact target for geometry:={geometry}')
    confirm_release = _flag(context, 'confirm_pickup_release')
    # delivery_rejoin (14 Oct) starts holding the object: the ball's spawn weld stays, nothing frees it at start
    held = LaunchConfiguration('mission_mode').perform(context).strip() == 'delivery_rejoin'
    confirm_release = confirm_release and not held
    # his MPC arms where QUAD4 is (it re-latches its kT estimator's start height every disarmed tick) and learns kT
    # only 0.20 m above it: released in the air (our welded start, gate /partner/release) his mission never climbs
    # that far before the pickup, kT stays at its nominal 22 and he flies ~11 cm under his reference (R1503)
    airborne = LaunchConfiguration('airborne_start').perform(context).strip().lower()
    airborne = (LaunchConfiguration('start_gate_topic').perform(context).strip() == inst.t(RELEASE_GATE)
                if airborne in ('', 'auto') else airborne in ('1', 'true', 'yes', 'on'))
    learn_in_air = airborne and G['thrust_ratio_feedback']
    plate, azimuths = _his_ring(LaunchConfiguration('plate').perform(context) or G['plate'],
                                LaunchConfiguration('carrier_azimuths_deg').perform(context) or G['carrier_azimuths_deg'])
    share = Path(get_package_share_directory('tejen_mission'))
    dyn_share = Path(get_package_share_directory('tejen_dynamic_planner'))
    state = f'/drone_{DRONE}/motion_capture_state'
    L = LaunchConfiguration
    typed = set(context.launch_configurations.get(TYPED_KEY, '').split(','))
    if not inst.legacy:
        _check_instance(context, inst, typed, plate, azimuths)
        _check_twin_instance(context, inst)
    floor_join = L('mission_mode').perform(context).strip() == 'floor_join'
    carrier_model = L('carrier_model').perform(context)
    clearance = L('carry_clearance').perform(context).strip()
    plate_object = G.get('pickup_object') == 'plate'
    route = transit_route(context, floor_join)
    H = carry_heights(G, ring_z=float(L('ring_z').perform(context)), rod=G['magnet_drop_below_quad'],
                      attach_z=G['contact_attach_z_m'] or 0.025, clearance=float(clearance) if clearance else None,
                      route=route, drop_height=_drop_height(context))
    if not floor_join:
        H['minimum_search_z_m_empty'] = H['minimum_search_z_m']
    # a typed value wins (the m1c_*/m1x_* configs type the old high transit: 2.3 / 2.7 / 1.35)
    height = lambda name, key=None: float(L(name).perform(context)) if name in typed else H[key or name]  # noqa: E731
    attach_z = G['contact_attach_z_m'] or 0.025
    cpp_reattach = _flag(context, 'pickup_cpp_reattach') and not floor_join
    # the low route climbs 3 cm after the drop (his 0.45 m, or back to the transit height, overshoots past the cap)
    extra = ({'drop_clear_height': _release_typed(context) + DROP_CLEAR_ABOVE_RELEASE}
             if route == 'low' and not floor_join else {})
    if _drop_release(context) is not None:
        extra['drop_release_clearance'] = _drop_release(context)
    if LaunchConfiguration('pickup_box').perform(context).strip():      # typed only, as on the rig path
        extra['pickup_box_m'] = float(LaunchConfiguration('pickup_box').perform(context))
    if LaunchConfiguration('return_climb').perform(context).strip() == 'transit':
        # return_climb:=transit (13 Nov, twin R4406: the C++ hand-over in the usual 3 cm climb sank onto his search floor):
        # climb back to the transit height first, as pickup_cpp_reattach's hand-over does
        extra['drop_clear_height'] = height('drop_approach_clearance')
    if _exit_moving(context):
        extra['drop_clear_height'] = _exit_drop_clear(float(L('ring_z').perform(context)), G['magnet_drop_below_quad'])
    if LaunchConfiguration('return_after_drop').perform(context).strip():
        # the return leaves from the drop zone's usual climb (the rig's 2.8 m ceiling refuses a climb back to the
        # transit height, 2.96 m with the unload overshoot); its C++ hand-over must sit above his search floor
        check_search_floor(height('minimum_search_z_m'), what='return hand-over',
                           start_body_z=float(L('ring_z').perform(context)) + DROP_OFFSET_Z
                           + extra.get('drop_clear_height', DROP_CLEAR_HEIGHT) + G['magnet_drop_below_quad'])
    if cpp_reattach:      # its heights on top; the typed release and pickup box kept (Te-Jen's review 11 Nov)
        extra = {**extra, **_cpp_reattach_heights(height, ring_z=float(L('ring_z').perform(context)), attach_z=attach_z,
                                                  rod=G['magnet_drop_below_quad'], typed=typed)}
        if _drop_release(context) is not None:
            extra['drop_release_clearance'] = _drop_release(context)
    check_search_floor(height('minimum_search_z_m', 'minimum_search_z_m_empty'), what='climb end' if floor_join
                       else 'lift end', start_body_z=(float(L('ring_z').perform(context)) + attach_z
                                                      + height('attach_transit_clearance') if floor_join
                                                      else G['object_rest_z'] + height('pickup_lift_height'))
                       + G['magnet_drop_below_quad'])
    check_transit_ceiling(planned_body_heights(
        floor_join=floor_join, ring_z=float(L('ring_z').perform(context)), attach_z=attach_z,
        rod=G['magnet_drop_below_quad'], object_rest_z=G['object_rest_z'], lift=height('pickup_lift_height'),
        drop_approach=height('drop_approach_clearance'),
        drop_clear=_release_clear(context, extra.get('drop_clear_height', DROP_CLEAR_HEIGHT), height('drop_approach_clearance')),
        attach_clearance=extra.get('attach_transit_clearance', height('attach_transit_clearance')),
        search_floor=height('minimum_search_z_m', 'minimum_search_z_m_empty')),
        refuse=False, log=lambda msg: print(f'partner_mission ({route} transit): {msg}', flush=True),
        ceiling=_ceiling(context), who=inst.quad)

    object_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='pickup_object_pose_bridge',
        arguments=[f'{OBJECT_POSE}@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'])

    ring = inst.node(
        topics=RING_TOPICS,
        package='tejen_mission', executable='ring_bridge', name='fake_cooperative_transport_world',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'bridge_carrier_azimuths_deg': azimuths,
            'bridge_attach_radius_m': G['ring_radius'],
            'bridge_cable_len_m': G['ring_cable_len'],
            'bridge_cable_elev_deg': G['ring_cable_elev_deg'],
            'payload_committed_trajectory_topic': '/fake_payload/committed_trajectory',
            'ring_plate_count': 12,
            'ring_plate_pitch_diameter_m': G['ring_pitch_diameter'],
            'ring_plate_diameter_m': G['ring_plate_diameter'],
            'ring_attachment_plate_index': plate,     # GEOMETRY: his numbering vs ours
            'ring_collision_outer_diameter_m': G['ring_collision_outer_diameter'],
            'ring_collision_top_offset_m': 0.0,
            'ring_collision_bottom_offset_m': -0.27,
            'ring_structure_height_m': 0.07,
            'enable_fake_obstacles': True,
            'fake_drone_count': len(azimuths),     # cosmetic: the azimuth list sets the carriers
            'fake_drone_radius': 0.12,
            'fake_drone_safety_radius': 0.35,
            'publish_shared_trajectories': True,
            'publish_attached_plates': carrier_model == 'tether',
            # rolling commitments: our ring's path is only known from its live
            # trajectory state (his fake ring published one analytic 180 s plan)
            'shared_trajectory_duration_s': 30.0,
            'shared_trajectory_control_interval_s': 0.50,
            'shared_trajectory_refresh_period_s': 1.0,
            # the contact target, rotated by the full ring attitude like our weld point
            **({'bridge_attach_z_m': G['contact_attach_z_m'], 'full_pose_attachment': True} if contact else {}),
            # the plate height low-passed (Hz), typed only, as on the rig (twin check of RIG-1015-a3's switch)
            **({'attach_z_lowpass_hz': float(LaunchConfiguration('attach_z_lowpass').perform(context))}
               if LaunchConfiguration('attach_z_lowpass').perform(context).strip() else {}),
            **({'attach_twist': LaunchConfiguration('attach_twist').perform(context).strip()}
               if LaunchConfiguration('attach_twist').perform(context).strip() else {}),
            # the shared commitments start this long before now (s; '' = 0, his timing): R2854/R2855's start_skew
            **({'shared_trajectory_backdate_s': float(LaunchConfiguration('commit_backdate').perform(context))}
               if LaunchConfiguration('commit_backdate').perform(context).strip() else {}),
        }])

    pendulum = Node(
        package='tejen_mission', executable='pendulum_state_publisher',
        name='pendulum_state_publisher', namespace=NS, output='screen',
        parameters=[{
            'use_sim_time': True,
            'x3_pose_topic': f'/model/{DRONE_MODEL}/pose',
            'magnet_index': 0, 'attachment_index': 5, 'drone_index': 7,
            'x3_link_poses_are_relative': True,
            'object_pose_topic': OBJECT_POSE,
            'publish_payload_world_state': True,
            'publish_rate_hz': 100.0,    # his MPC runs at 30 Hz; 500 Hz cost sim speed
        }])

    magnet = Node(
        package='tejen_mission', executable='magnet_attachment_manager',
        name='magnet_attachment_manager', namespace=NS, output='screen',
        parameters=[{
            'use_sim_time': True,
            'enable_elrs_magnet_output': False,
            'attachment_mode': 'fixed_joint',
            # one-shot gz CLI publishes drop: the object stayed welded after DROP (R0678) and
            # missed the net; the ROS topics go through the bridges below as well
            'command_backend': 'both',
            'magnet_command_topic': f'/{NS}/magnet/command',   # his object magnet; /magnet/command is our ring magnet (ours from the handoff)
            'magnet_tip_pose_topic': f'/{NS}/magnet_tip_pose',
            'magnet_tip_pose_msg_type': 'pose_stamped',
            'object_pose_topic': OBJECT_POSE,
            'object_pose_index': 1,
            'use_fallback_object_pose': False,
            # near-contact, like our ring weld: 0.18 welded the object mid-descent 18 cm off
            # the tip, and the spanner then hung on that lever (R0626); covers his 3 cm
            # attach clearance plus ~5 cm xy error
            # contact weld (Wesley: a magnet touching a steel ball): tip within 0.10 m of the
            # 0.08 m ball's centre = touching its top. 0.18 welded mid-descent 18 cm off
            # the tip, which is why the pickup looked wrong (R0625-R0633)
            # the plate (pickup_object plate): the tip on its top face sits 0.0245 over its centre; 0.04 allows
            # 3.2 cm of xy there (0.10 welded it 9.5 cm under the magnet)
            'attach_radius': 0.04 if plate_object else 0.10,
            'attach_speed_threshold': 1.0,
            'attach_dwell_time_s': 0.0,
            # the object only: never /payload/* (that is the ring weld)
            'object_attached_topic': f'/{NS}/object_attached',
            'attachment_state_topic': f'/{NS}/attachment_state',
            'gz_attach_topic': '/pickup/attach', 'gz_detach_topic': '/pickup/detach',
            'ros_attach_topic': '/pickup/attach', 'ros_detach_topic': '/pickup/detach',
        }])

    planner = inst.node(
        topics=PLANNER_TOPICS, fixed=PLANNER_FIXED_TOPICS, extra={'vehicle_id': inst.vehicle_id},
        package='tejen_mission', executable='online_join_planner',
        name='online_join_planner', output='screen',
        parameters=[
            str(share / 'config' / 'payloads' / 'spanner_8mm.yaml'),
            str(share / 'config' / 'visual_astar_visual_only.yaml'),
            str(share / 'config' / 'c1f2b_cpp_authority.yaml'),
            {
                'use_sim_time': True,
                'vehicle_id': 'drone_0',
                'assigned_attachment_id': 'attachment_0',
                'mission_mode': L('mission_mode').perform(context),
                'drone_state_topic': state,
                'drone_command_topic': f'/{NS}/drone_command',
                'pickup_object_pose_topic': OBJECT_POSE,
                'pickup_object_index': 1,
                'object_attached_topic': f'/{NS}/object_attached',
                'magnet_command_topic': f'/{NS}/magnet/command',   # his object magnet; /magnet/command is our ring magnet (ours from the handoff)
                'use_measured_magnet_tip': True,
                'magnet_tip_pose_topic': f'/{NS}/magnet_tip_pose',
                'use_geometry_aware_pickup': True,
                'auto_descend': True,
                'pickup_xy_tolerance': 0.10,
                'pickup_tip_speed_tolerance': 0.60,
                'pickup_approach_clearance': 0.35,
                # tip target = ball centre + 0.09: 1 cm above the top of the 0.08 m ball
                # (0.03 put the target inside the ball; the rod was pushed aside); the plate: 1 cm over its top face
                'pickup_attach_clearance': 0.0225 if plate_object else 0.09,
                **(PLATE_PROFILE if plate_object else {}),
                # a HIGH transit (Wesley 2026-09-26: near-collisions): lift the quad to
                # ~1.9 m at the pickup, fly the C++ transit above a 1.8 m floor so the
                # 0.49 m rod + ball clear the carriers (tops ~1.1 m), and only descend
                # inside the ring's clear centre (R0629/R0636 swept the rod into carrier 0)
                'pickup_lift_height': height('pickup_lift_height'),
                'drop_approach_clearance': height('drop_approach_clearance'),
                'attach_cpp_transit': _flag(context, 'attach_cpp_transit'),
                'attach_transit_clearance': height('attach_transit_clearance'),
                'pickup_cpp_reattach': cpp_reattach,
                **extra,
                # typed only (his 0.30 otherwise): the tip above the plate where his approach ends
                **({'approach_offset_z': float(L('approach_offset_z').perform(context))}
                   if L('approach_offset_z').perform(context).strip() else {}),
                'pickup_lift_speed': 0.12,
                'pickup_z_tolerance': 0.20,
                'max_reference_speed': 2.0,
                'lift_complete_object_clearance': 0.08,
                'match_xy_threshold': 0.18,
                'match_xy_velocity_threshold': 0.20,
                'reattach_transit_xy_threshold': 0.40,
                'attach_ready_target_lead_time': 0.30,
                'use_static_pickup_object': False,
                'drop_point_source': 'payload',
                'payload_pose_topic': '/fake_payload/pose',
                'payload_twist_topic': '/fake_payload/twist',
                'reference_nominal_speed': 0.25,
                'static_obstacle_count': 0,
                # no auto-land from ATTACH_READY: after our weld its magnet OFF would
                # release drone 3 from the ring
                'attach_ready_land_after_s': 0.0,
                'log_every_n_updates': 6,     # 30 Hz -> 5 Hz CSV (sim speed)
                # our x3_drone3 rod: body centre to tip 0.49 (his airframe 0.50 below a
                # 0.05 pivot); his 0.60 reference floor held our tip 8 cm above the
                # pickup target (R0627)
                'magnet_drop_below_quad': G['magnet_drop_below_quad'],
                'min_reference_z': 0.52,
                # the backend publishes at 5 Hz (sim); a 0.20 s freshness limit is one
                # period, and one late tick parked the drone on its terminal hold for the
                # rest of the run (R0622, R0628)
                'c1f1_shadow_reference_timeout_s': 0.6,
                # his hold before the C++ grant wanders 8-15 cm in our loaded sim; the
                # 0.15 m bound cancelled every grant in R0630 (R0609 alike). The C++ plan
                # starts from the re-latched measured state either way.
                'c1f2_prepare_max_lift_drift_m': 0.30,
                'c1f2_prepare_settle_speed_mps': 0.12,
                **({'takeoff_speed': float(LaunchConfiguration('climb_speed').perform(context))}
                   if LaunchConfiguration('climb_speed').perform(context).strip() else {}),
                # the rig path's drop-off switches, typed only, so the twin flies the rig sheet's k1 line (30 Oct)
                **({'c1f2_cpp_authority_enabled': False}
                   if LaunchConfiguration('cpp_authority').perform(context).strip() == 'false' else {}),
                **({'end_after_drop': True}
                   if LaunchConfiguration('end_after_drop').perform(context).strip() == 'true' else {}),
                **_return_after_drop(context),
                **_exit_handoff(context),
                **_cpp_python_fallback(context),
                **_prepare_and_release(context),
                **_drop_on_arrival(context), **_takeoff_gate(context),
                **_lift_full(context, G),
                **_lift_cap(context, G['object_rest_z'] + height('pickup_lift_height')),
                # the descent ends on the contact target (ring_bridge's attachment point), held
                # with no lead; the planner's contact-mode gates are his ground-ring values
                # (the card's neighbours move one of them: contact_lead / contact_dz / contact_dx).
                # His pickup: the descent ends on the ball weld (his object flag is the weld here, not a
                # proximity flag as on the rig) and holds the tip there; his bounded retry: not on the ball
                # after his contact wait -> magnet OFF, rise, re-approach the ball where it now is; his third
                # failure gives up and we take QUAD4 back
                **({'contact_attach': True,
                    # the ring's own pose: the bottom hold judges the tip in the plate frame (Tejen 15 Oct)
                    'ring_pose_topic': '/payload/motion_capture_state',
                    # card 2026-10-11_quad4_tilt_cut (rig-port critic F6): land here if the fleet cannot take it back
                    **({'taken_back_self_land': True} if _flag(context, 'taken_back_self_land') else {}),
                    'final_attach_offset_z': ParameterValue(L('contact_dz'), value_type=float),
                    'approach_offset_x': ParameterValue(L('contact_dx'), value_type=float),
                    'approach_offset_y': ParameterValue(L('contact_dy'), value_type=float),
                    'attach_ready_target_lead_time': ParameterValue(L('contact_lead'), value_type=float),
                    'pickup_hold_on_attach': True,
                    'pickup_retry_enabled': True,
                    'pickup_retry_clearance_m': 0.10,
                    # typed only (his twin values otherwise): the rig's match gate in the twin (plan 2026-10-15 N6)
                    **{k: float(L(arg).perform(context)) for arg, k in (
                        ('match_xy', 'contact_match_xy_threshold'), ('match_speed', 'contact_match_xy_velocity_threshold'),
                        ('match_dwell', 'dwell_time_s'), ('descent_debounce', 'contact_abort_xy_debounce_s'),
                        ('descent_abort_xy', 'contact_abort_xy_threshold'), ('descent_gross_xy', 'contact_descent_gross_xy_m'),
                        ('descent_abort_speed', 'contact_abort_xy_velocity_threshold'),
                        # 11 Nov (R4255): the twin ignored a typed contact_give_up_s and gave up at his 30 s
                        ('contact_give_up_s', 'contact_give_up_s'))
                        if L(arg).perform(context).strip()},
                    **({'contact_descent_hold': L('descent_hold').perform(context).strip().lower() in ('1', 'true', 'yes')}
                       if L('descent_hold').perform(context).strip() else {})}
                   if contact else {}),
            },
        ])

    # a C++ leg after the drop flies with no object: his backend must not wait for one (the moving exit keeps the wait:
    # his exit context skips the object checks for its own transaction)
    overrides = dict(G=G, rod=G['magnet_drop_below_quad'],
                     floor_join=floor_join or cpp_reattach
                     or (LaunchConfiguration('return_after_drop').perform(context).strip() == 'cpp'
                         and not _exit_moving(context)),
                     carrier_model=carrier_model, margins=_backend_margins(context), count=len(azimuths))
    if inst.legacy:
        backend = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(dyn_share / 'launch' / 'c1f6_moving_rendezvous_backend.launch.py')),
            launch_arguments={'planner_rate_hz': '5.0',
                              'minimum_search_z_m': str(height('minimum_search_z_m', 'minimum_search_z_m_empty')),
                              'cooperative_preview_count': str(len(azimuths)),     # one per listed carrier
                              # drone 3's state and HIS object flag (/magnet/object_attached is
                              # our ring weld): R0609's backend waited on both forever
                              # the shared trajectories (ring_bridge) are stamped in sim time
                              'use_sim_time': 'true',
                              'extra_params_file': _backend_overrides(
                                  str(share / 'config' / 'partner_backend_overrides.yaml'), **overrides),
                              'state_topic': state,
                              'object_attached_topic': f'/{NS}/object_attached'}.items())]
    else:
        backend = _instance_backend(inst, share, dyn_share, base_override='partner_backend_overrides.yaml',
                                    overrides=overrides, use_sim_time=True,
                                    search_floor=height('minimum_search_z_m', 'minimum_search_z_m_empty'),
                                    state=state, object_attached=f'/{NS}/object_attached')

    controller = inst.node(
        topics=MPC_TOPICS, extra={'logging_instance': inst.vehicle_id},
        package='tejen_mpc', executable='main', name='tejen_mpc', namespace=NS, output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_external_reference': True,
            'enable_thrust_ratio_ukf': True,
            # linear sim plant, x3 0.643 kg, motorConstant 0.62e-6: 4*0.62e-6*4631^2/0.643
            'thrust_ratio': G['thrust_ratio'],
            'enable_thrust_ratio_feedback': G['thrust_ratio_feedback'],
            **_throttle_max(context),
            'xy_bias_mode': 'legacy_integral',
            # a cached solver: a fresh acados build at launch starved drone 3's mocap (R0600)
            'acados_cache_dir': os.path.expanduser('~/.cache/multi_drone_control/tejen_payload_mpc'),
            'cable_length': G['mpc_cable_length'],
            'pose_timeout_s': 1.0,       # sim at ~0.25 real time (R0620 self-disarm)
            # periodic ARM state heartbeat: a single startup message can be lost, and his
            # supervisor then never arms (R0638, R0641 'arming_feedback_received': False)
            'arming_state_feedback_period_s': 0.5,        # body centre to tip on x3_drone3_magnet_rod_partner
            # carried-object gain schedule (G1b; 0 = off; default 0.1 kg = payload_model.sdf): kT*m_d/(m_d+m_obj) while the object is on
            'object_mass_kg': ParameterValue(L('object_mass_kg'), value_type=float),
            'object_vehicle_mass_kg': G['vehicle_mass'],
            'object_attached_topic': f'/{NS}/object_attached',
            # his gate is re-checked every tick, not only at the start: 0.0 paused the learning below the arm
            # height, i.e. through his whole pickup (R1505); -10 m never gates
            **({'thrust_ratio_ukf_min_height_above_start': -10.0} if learn_in_air else {}),
            'mpc_profile': ParameterValue(L('mpc_profile'), value_type=str),
            'mpc_command_execution': ParameterValue(L('mpc_command_execution'), value_type=str),
        }],
        remappings=[('motion_capture_state', state),
                    ('ELRSCommand', f'/drone_{DRONE}/ELRSCommand_tejen'),
                    ('telemetry', f'/drone_{DRONE}/telemetry')])

    supervisor = inst.node(
        fixed=SUPERVISOR_FIXED_TOPICS,
        package='tejen_mission', executable='simulation_test_supervisor',
        name='simulation_test_supervisor', namespace=NS, output='screen',
        parameters=[{
            'use_sim_time': True,
            'control_mode': 'automatic',
            # his nodes start with the fleet on the ground (a start mid-flight starved our
            # trackers of CPU, R0604); his mission waits for the ring to fly its path
            'start_gate_topic': L('start_gate_topic') if inst.legacy else L('start_gate_topic').perform(context),
            'start_gate_min_traj_t': ParameterValue(L('start_gate_min_traj_t'), value_type=float),
            'result_path': ParameterValue(_instance_path(context, inst, 'result_path', typed), value_type=str),
            'startup_timeout_s': 300.0,   # counted from the start gate (R0638: 60 s ran out)
            'arm_timeout_s': 20.0,
            'manual_start_timeout_s': 300.0,
            # wall-clock in his supervisor: the full stack runs at ~0.25 real time
            'mission_timeout_s': 3000.0,
            'success_dwell_s': 2.0,
            # wall-clock limits; the stack runs at ~0.25 real time and the backend's 1 Hz
            # (sim) status arrives every ~4 s wall (R0613 READINESS_LOST at 3.0 s)
            'freshness_timeout_s': 5.0,
            # supervisor_status_timeout_s (typed only, one twin check): the C++ exit's 7 s timeout with his backend
            # frozen (SIGSTOP) needs his backend status to go quiet ~50 s wall (Te-Jen and Codex 15 Nov)
            'planner_status_timeout_s': float(L('supervisor_status_timeout_s').perform(context) or 15.0),
            # our ring weld is the success, not his phase: a supervisor stop disarms his
            # MPC, which would drop drone 3 if it came before the weld
            'success_phase': 'NONE_PARTNER_WELD_OWNS_SUCCESS',
            'failure_phase': 'LANDED_DISARMED',
            'require_handoff_ready': True,
            # return_after_drop (no re-attach): the line ends LANDED_DISARMED at its start, after AT_START; a landing
            # before AT_START, or a disarm in flight, still fails (Te-Jen and Codex 15 Nov: R4536/R4537's
            # EARLY_TERMINAL_DISARM was this end, not an early one)
            **(RETURN_SUPERVISOR if L('return_after_drop').perform(context).strip() else {}),
            'landing_request_phase': '',
            **({'pickup_release_state_topic': PICKUP_STATE} if confirm_release else {}),
        }],
        # an instance without an object: his readiness input 'payload' is the ring (a fresh state is all it checks)
        remappings=[('/motion_capture_state', state),
                    ('/pendulum_swing_state', f'/{NS}/pendulum_swing_state'),
                    ('/payload_world_state', f'/{NS}/payload_world_state' if inst.legacy else RING_STATE)])
    # no shutdown-on-supervisor-exit: his MPC must keep flying until our weld takes over

    pickup_bridges = [Node(
        package='ros_gz_bridge', executable='parameter_bridge', name=f'pickup_{w}_bridge',
        arguments=[f'/pickup/{w}@std_msgs/msg/Empty]gz.msgs.Empty']) for w in ('attach', 'detach')]

    # gz DetachableJoint may weld the object to the tip at spawn: free it before the mission.
    # confirm_pickup_release: the supervisor repeats /pickup/detach until the joint reads free.
    if confirm_release:
        release = Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='pickup_state_bridge',
            arguments=[f'{PICKUP_STATE}@std_msgs/msg/String[gz.msgs.StringMsg'])
    else:
        release = ExecuteProcess(
            cmd=['gz', 'topic', '-t', '/pickup/detach', '-m', 'gz.msgs.Empty', '-p', ''],
            output='screen')

    # diagnosis record: his view of the drone, his reference and his commands
    bag_topics = [state, f'/{NS}/pendulum_swing_state', f'/{NS}/magnet_tip_pose',
                  '/join_planner/reference', '/join_planner/phase', f'/drone_{DRONE}/ELRSCommand_tejen',
                  '/fake_payload/pose', '/magnet/command', f'/{NS}/magnet/command', '/join_planner/handoff_ready',
                  f'/{NS}/object_attached', OBJECT_POSE,
                  *(CONTACT_TOPICS if contact else []), *([PICKUP_STATE] if confirm_release else [])]
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', _instance_path(context, inst, 'bag_path', typed), '--use-sim-time',
             *(bag_topics if inst.legacy else [inst.t(t) for t in bag_topics if t != OBJECT_POSE])],
        output='screen', condition=IfCondition(L('record')))

    if not inst.legacy:
        # a carrier released to his stack: no gz object (bridges, his gz magnet manager) and no x3_drone3 pendulum;
        # his inputs from its own mocap as on the rig (body, and the magnet marker the twin's emulator publishes)
        return [bag, ring, *_instance_rig_inputs(context, inst, share, typed, use_sim_time=True,
                                                  anchor_z=-CARRIER_BODY_ABOVE_PIVOT,
                                                  tip_topic=f'/drone_{inst.drone}/magnet_tip_state',
                                                  tip_msg_type='motion_capture_state'),
                planner, *backend, controller, supervisor]
    return [bag, *([] if held else [release]), object_bridge, *pickup_bridges, ring, pendulum, magnet, planner, *backend,
            controller, supervisor]


def _irl_block(share, node, name='irl_commissioning.yaml'):
    """One node's ros__parameters from his config file, as a dict (so only that node's values are used)."""
    data = yaml.safe_load((share / 'config' / name).read_text())
    return dict(data[node]['ros__parameters'])


def _check_instance(context, inst, typed, plate, azimuths):
    """A second instance types its own plate and the carriers that stay, never lists a carrier on its own plate (it
    would sit inside a phantom obstacle), and waits for its own drone's release, never QUAD4's."""
    for name in ('plate', 'carrier_azimuths_deg'):
        if name not in typed:
            raise RuntimeError(f'partner_mission instance:={inst.name}: type {name} (the defaults are QUAD4\'s layout)')
    if any(int(round(a / 30.0)) % 12 == plate for a in azimuths):
        raise RuntimeError(f'partner_mission instance:={inst.name}: carrier_azimuths_deg lists a carrier on its own plate '
                           f'(his plate {plate}); list only the carriers that stay')
    gate = LaunchConfiguration('takeoff_gate_topic').perform(context).strip()
    if gate and not gate.startswith(f'/drone_{inst.drone}/'):
        raise RuntimeError(f'partner_mission instance:={inst.name}: takeoff_gate_topic:={gate} is not drone '
                           f'{inst.drone}\'s (its release is /drone_{inst.drone}/partner/release)')


def _check_twin_instance(context, inst):
    """The twin path's second instance is a carrier released to his stack: no object (floor_join, no pickup weld to
    confirm) and a start gate of its own (its release) or the shared ring's, never QUAD4's release."""
    if LaunchConfiguration('mission_mode').perform(context).strip() != 'floor_join':
        raise RuntimeError(f'partner_mission instance:={inst.name} (twin): mission_mode:=floor_join (the twin has no '
                           f'object for {inst.quad})')
    if _flag(context, 'confirm_pickup_release'):
        raise RuntimeError(f'partner_mission instance:={inst.name} (twin): no confirm_pickup_release (no object)')
    gate = LaunchConfiguration('start_gate_topic').perform(context).strip()
    if gate and not gate.startswith((f'/drone_{inst.drone}/', *SHARED_TOPICS)):
        raise RuntimeError(f'partner_mission instance:={inst.name}: start_gate_topic:={gate} is not drone '
                           f'{inst.drone}\'s (its release is /drone_{inst.drone}/partner/release)')


def _instance_rig_inputs(context, inst, share, typed, *, use_sim_time, anchor_z, tip_topic, tip_msg_type='',
                         ball_params=None):
    """partner_rig_state (his IRL publisher's outputs from our mocap: the marker, his pendulum, the ball) and his IRL
    ball manager (attached inferred from his magnet command, the marker and the ball; no radio: the coil is our
    magnet_gate's, from his magnet command), under the instance's names. The rig, and a twin instance without the
    x3 pendulum or the gz object. tip_topic / tip_msg_type typed win (e.g. /drone_<d>/magnet_tip_state,
    motion_capture_state: our mocap's extra body); otherwise the given ones (renamed for a second instance)."""
    typed_tip = LaunchConfiguration('tip_topic').perform(context).strip() if 'tip_topic' in typed else ''
    tip_type = (LaunchConfiguration('tip_msg_type').perform(context).strip() if 'tip_msg_type' in typed else '') \
        or tip_msg_type
    rig_state = inst.node(
        extra={'ns': f'/{inst.ns}', 'label': inst.quad},
        package='tejen_mission', executable='partner_rig_state', name='partner_rig_state', namespace=NS,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'ns': f'/{NS}', 'body_topic': f'/drone_{DRONE}/motion_capture_state',
                     'tip_topic': typed_tip or tip_topic,
                     **({'tip_msg_type': tip_type} if tip_type else {}),
                     'ball_topic': '/pickup/motion_capture_state',
                     'anchor_offset_z': anchor_z,
                     'max_pair_age_s': 0.20}])
    ball = inst.node(
        topics=BALL_TOPICS, fixed=BALL_FIXED_TOPICS,
        package='tejen_mission', executable='magnet_attachment_manager_irl',
        name='magnet_attachment_manager_irl', namespace=NS, output='screen',
        parameters=[{**_irl_block(share, 'magnet_attachment_manager_irl'),
                     **(ball_params or {}),
                     'use_sim_time': use_sim_time,
                     'magnet_command_topic': f'/{NS}/magnet/command',
                     'object_attached_topic': f'/{NS}/object_attached',
                     'attachment_state_topic': f'/{NS}/attachment_state',
                     'magnet_tip_pose_topic': f'/{NS}/magnet_tip_pose',
                     'pickup_object_pose_topic': f'/{NS}/pickup_object/pose',
                     'pickup_object_index': 0,
                     'enable_elrs_magnet_output': False,
                     'elrs_magnet_command_topic': f'/{NS}/magnet/ELRSCommand'}])
    return [rig_state, ball]


def _instance_path(context, inst, name, typed):
    """bag_path / result_path: the argument itself for QUAD4's instance; another instance's own file unless typed."""
    if inst.legacy:
        return LaunchConfiguration(name)
    path = LaunchConfiguration(name).perform(context).strip()
    if name in typed:
        return path
    stem, ext = os.path.splitext(path)
    return f'{stem}_{inst.name}{ext}'


def _instance_backend(inst, share, dyn_share, *, base_override, overrides, use_sim_time, search_floor, state,
                      object_attached):
    """His backend for an instance not at the root, as his c1f6_moving_rendezvous_backend.launch.py builds it (his file,
    our override file base_override, _backend_override_params, his launch's own values) but through Instance.node: one
    dict, every topic the instance's, his run directory logs/tejen_dynamic_planner/c1f6_rendezvous_<time> with its name."""
    count = overrides['count']
    root = Path(get_package_prefix('tejen_dynamic_planner')).resolve().parents[1]
    run_dir = (root / 'logs' / 'tejen_dynamic_planner'
               / f"c1f6_rendezvous_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{inst.name}")
    node = inst.node(
        topics=BACKEND_TOPICS,
        extra={'vehicle_id': inst.vehicle_id, 'csv_path': str(run_dir / 'backend.csv'),
               'replan_csv_path': str(run_dir / 'replans.csv')},
        package='tejen_dynamic_planner', executable='dynamic_planner_transfer_backend', name=BACKEND_NODE,
        output='screen',
        parameters=[str(dyn_share / 'config' / 'c1f6_moving_rendezvous.yaml'),
                    str(share / 'config' / base_override),
                    _backend_override_params(**overrides),
                    {'planner_rate_hz': 5.0, 'minimum_search_z_m': float(search_floor),
                     'cooperative_drone_state_topics': [f'/fake_obstacles/drone_{i}/state' for i in range(count)],
                     'cooperative_committed_trajectory_topics': [
                         f'/fake_obstacles/drone_{i}/committed_trajectory' for i in range(count)],
                     'use_sim_time': use_sim_time, 'state_topic': state, 'object_attached_topic': object_attached}])
    return [LogInfo(msg=f'C1F.6 C++ log directory ({inst.ns}): {run_dir}'),
            OpaqueFunction(function=lambda _ctx: run_dir.mkdir(parents=True, exist_ok=True)),
            node]


def _rig_nodes(context):
    """real:=true: his M1 on the rig around our real carried ring (see the module docstring)."""
    typed = set(context.launch_configurations.get(TYPED_KEY, '').split(','))
    inst = _instance(context)

    def val(name, default):
        """A typed launch argument, else the rig default (the sim launch's own default is not the rig's)."""
        return LaunchConfiguration(name).perform(context).strip() if name in typed else default

    if LaunchConfiguration('supervisor_status_timeout_s').perform(context).strip():
        raise RuntimeError('partner_mission real:=true: supervisor_status_timeout_s is a twin check only')
    geometry = val('geometry', 'rig')
    if geometry not in RIG_GEOMETRIES:
        raise RuntimeError(f'partner_mission real:=true flies geometry:=rig (the 28 Oct payload) or rig_m2a (the M2A '
                           f'ring), got {geometry!r}')
    # the carriers' cable is ours (bringup passes cable_len): a stale one misplaces his carrier obstacles and transit
    # heights (the 0.55 m table on 1 m cables: carriers 0.23 m further out and 0.39 m higher than he models them)
    if not val('ring_cable_len', ''):
        raise RuntimeError('partner_mission real:=true: type ring_cable_len:=<the carriers\' cable_len> (our attach '
                           'launch passes it); the rig table\'s 0.55 is the 0.55 m era\'s')
    G = {**GEOMETRY[geometry], 'ring_cable_len': float(val('ring_cable_len', '')),
         'ring_cable_elev_deg': float(val('ring_cable_elev_deg', '') or GEOMETRY[geometry]['ring_cable_elev_deg'])}
    if not _flag(context, 'contact_attach'):
        raise RuntimeError('partner_mission real:=true is the combined M1 contact attach: contact_attach:=true '
                           '(the ATTACH_READY hand-back is not for the rig, decisions 10 Oct)')
    # QUAD4's own numbers (critic F8): his IRL defaults are QUAD2's (body 12 / marker 22), so the rig refuses them
    # untyped; they come from Wednesday's tip record and Thursday's R10a, with weld_point_z and proof_taut_len
    for name, what in (('magnet_drop_below_quad', f"{inst.quad}'s body to magnet marker (m, the 09:00 tip record)"),
                       ('mpc_thrust_ratio', f"his kT on {inst.quad} (Thursday's R10a)")):
        if name not in typed or not LaunchConfiguration(name).perform(context).strip():
            raise RuntimeError(f'partner_mission real:=true: type {name}:=<{what}>; his IRL default is QUAD2\'s')
    if _flag(context, 'supervisor'):
        raise RuntimeError('partner_mission real:=true: supervisor:=true is refused on the combined rig until its '
                           'give-up stop has been through the critic (critic F9); ARM and TAKEOFF stay manual')
    target_z = LaunchConfiguration('contact_target_z').perform(context).strip()
    xy_txt = LaunchConfiguration('contact_target_xy').perform(context).strip()
    target_xy = [float(v) for v in xy_txt.replace(',', ' ').split()] if xy_txt else []
    if target_xy and len(target_xy) != 2:
        raise RuntimeError(f'contact_target_xy {xy_txt!r}: two numbers, x,y in the ring frame (m)')
    if not target_z:
        raise RuntimeError('partner_mission real:=true: type contact_target_z:=<weld_point_z> (the 09:00 '
                           'calibration: the magnet marker above the ring-origin plane with the face on plate 9)')
    plate, azimuths = _his_ring(val('plate', G['plate']), val('carrier_azimuths_deg', G['carrier_azimuths_deg']))
    if not inst.legacy:
        _check_instance(context, inst, typed, plate, azimuths)
    share = Path(get_package_share_directory('tejen_mission'))
    dyn_share = Path(get_package_share_directory('tejen_dynamic_planner'))
    state = f'/drone_{DRONE}/motion_capture_state'
    tip, ball_pose, ball_attached = f'/{NS}/magnet_tip_pose', f'/{NS}/pickup_object/pose', f'/{NS}/object_attached'
    f = lambda name, default: float(val(name, default))  # noqa: E731
    floor_join = val('mission_mode', 'pickup_delivery') == 'floor_join'
    carrier_model = LaunchConfiguration('carrier_model').perform(context)
    rod = f('magnet_drop_below_quad', G['magnet_drop_below_quad'])
    clearance = LaunchConfiguration('carry_clearance').perform(context).strip()
    route = transit_route(context, floor_join)
    H = carry_heights(G, ring_z=float(LaunchConfiguration('ring_z').perform(context)), rod=rod,
                      attach_z=float(target_z), clearance=float(clearance) if clearance else None, route=route,
                      drop_height=_drop_height(context))
    search_floor = f('minimum_search_z_m', H['minimum_search_z_m_empty' if floor_join else 'minimum_search_z_m'])
    # the low route climbs 3 cm after the drop (his 0.45 m, or back to the transit height, overshoots past the cap)
    drop_clear = (_release_typed(context) + DROP_CLEAR_ABOVE_RELEASE
                  if route == 'low' and not floor_join else DROP_CLEAR_HEIGHT)
    if val('return_climb', '') == 'transit':   # as on the sim path: back to the transit height before the hand-over
        drop_clear = f('drop_approach_clearance', H['drop_approach_clearance'])
    if _exit_moving(context):
        drop_clear = _exit_drop_clear(float(LaunchConfiguration('ring_z').perform(context)), rod)
    if val('return_after_drop', ''):     # the return's C++ hand-over at the drop zone's climb, above his search floor
        check_search_floor(search_floor, what='return hand-over',
                           start_body_z=float(LaunchConfiguration('ring_z').perform(context)) + DROP_OFFSET_Z
                           + drop_clear + rod)
    # rig: never higher than QUAD4_MAX_BODY_Z (Wesley 29 Oct, after RIG-1029-sq2_5's 3.1 m climb and the 3.3 m
    # preview of the drop-off): refused, unless transit:=over is typed
    check_transit_ceiling(planned_body_heights(
        floor_join=floor_join, ring_z=float(LaunchConfiguration('ring_z').perform(context)), attach_z=float(target_z),
        rod=rod, object_rest_z=G['object_rest_z'], lift=f('pickup_lift_height', H['pickup_lift_height']),
        drop_approach=f('drop_approach_clearance', H['drop_approach_clearance']),
        drop_clear=_release_clear(context, drop_clear, f('drop_approach_clearance', H['drop_approach_clearance'])),
        attach_clearance=f('attach_transit_clearance', H['attach_transit_clearance']), search_floor=search_floor),
        refuse=route != 'over' or 'transit' not in typed,
        log=lambda msg: print(f'partner_mission rig ({route} transit): {msg}', flush=True), ceiling=_ceiling(context),
        who=inst.quad)
    # pickup_contact_height (typed only; m1_irl_pickup.launch.py's measurement, Wesley 21 Oct, RIG-1021-q1pick: the
    # magnet marker above the object's marker with the magnet face resting on it): his contact point on the real
    # object instead of his commissioning puck's (86 x 86 x 22 mm, 0.012 over its marker)
    object_point = {}
    backend_object = {}
    if ({'pickup_point_x', 'pickup_point_y'} & set(typed)) and 'pickup_contact_height' not in typed:
        raise RuntimeError('partner_mission: pickup_point_x / pickup_point_y act only with pickup_contact_height typed '
                           '(the object\'s measured contact point); type all three')
    if 'pickup_contact_height' in typed:
        face_z = float(_irl_block(share, 'online_join_planner')['magnet_marker_to_contact_face_z'])
        point_z = f('pickup_contact_height', 0.0) + face_z
        size, thick = f('pickup_object_size', 0.10), f('pickup_object_thickness', 0.025)
        # his backend's object box from the same typed sizes (12 Nov: our rig file's 0.043 m half was an 86 mm object; the
        # object with its funnel is 105 x 50 mm), hanging from the magnet's face
        backend_object = {'payload_half_x_m': size / 2.0, 'payload_half_y_m': size / 2.0, 'payload_half_z_m': thick / 2.0,
                          'payload_center_from_magnet_z_m': face_z - thick / 2.0}
        # pickup_point_x / _y (typed only, 11 Nov: the object with a funnel on it, its Motive pivot 1.6 cm from where the
        # funnel seats the magnet): the contact point's offset from the object's marker, in the object's own axes
        px, py = f('pickup_point_x', 0.0), f('pickup_point_y', 0.0)
        object_point = {'magnet_attachment_manager_irl': {'pickup_point_offset_x': px, 'pickup_point_offset_y': py,
                                                          'pickup_point_offset_z': point_z},
                        'online_join_planner': {'payload_pickup_point_x': px, 'payload_pickup_point_y': py,
                                                'payload_pickup_point_z': point_z, 'payload_collision_size_x': size,
                                                'payload_collision_size_y': size, 'payload_collision_size_z': thick}}
        print(f'partner_mission rig: the object\'s contact point {point_z:+.4f} m over its marker '
              f'(contact height {point_z - face_z:.4f}, face {face_z}), {px:+.4f} / {py:+.4f} m sideways in its axes; '
              f'{size} x {size} x {thick} m', flush=True)
    # rig: a typed floor at least 0.10 m under the nominal climb end (RIG-1029-sq4_5: the ring rode 3 cm low and QUAD4
    # held at 2.33 m under a 2.36 m floor, his backend WAITING_ABOVE_SEARCH_FLOOR for the whole flight)
    check_search_floor(search_floor, what='climb end' if floor_join else 'lift end', start_body_z=(
        float(LaunchConfiguration('ring_z').perform(context)) + float(target_z)
        + f('attach_transit_clearance', H['attach_transit_clearance']) if floor_join
        else G['object_rest_z'] + f('pickup_lift_height', H['pickup_lift_height'])) + rod,
        margin=0.10 if 'minimum_search_z_m' in typed else 0.0)

    ring = inst.node(
        topics=RING_TOPICS,
        package='tejen_mission', executable='ring_bridge', name='fake_cooperative_transport_world',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'bridge_carrier_azimuths_deg': azimuths,
            'bridge_attach_radius_m': G['ring_radius'],
            'bridge_cable_len_m': G['ring_cable_len'],
            'bridge_cable_elev_deg': G['ring_cable_elev_deg'],
            'payload_committed_trajectory_topic': '/fake_payload/committed_trajectory',
            'ring_plate_count': 12,
            'ring_plate_pitch_diameter_m': G['ring_pitch_diameter'],
            'ring_plate_diameter_m': G['ring_plate_diameter'],
            'ring_attachment_plate_index': plate,
            'ring_collision_outer_diameter_m': G['ring_collision_outer_diameter'],
            'ring_collision_top_offset_m': 0.0,
            'ring_collision_bottom_offset_m': -0.27,
            'ring_structure_height_m': 0.07,
            'enable_fake_obstacles': True,
            'fake_drone_count': len(azimuths),     # cosmetic: the azimuth list sets the carriers
            'fake_drone_radius': 0.12,
            'fake_drone_safety_radius': 0.35,
            'publish_shared_trajectories': True,
            'shared_trajectory_duration_s': 30.0,
            'shared_trajectory_control_interval_s': 0.50,
            'shared_trajectory_refresh_period_s': 1.0,
            'publish_attached_plates': carrier_model == 'tether',
            # his contact target = our weld point: the calibrated marker height, the full ring attitude
            'bridge_attach_z_m': float(target_z),
            # one plate point for both stacks (plan 2026-10-14 WP3): our weld point's x, y in the ring frame
            **({'bridge_attach_xy_m': target_xy} if target_xy else {}),
            'full_pose_attachment': True,
            # 14 Oct load cut (Wesley): no 20 s trail MarkerArray at 50 Hz on the rig laptop
            'trail_seconds': 0.0,
            # the plate height his reference follows, low-passed (Hz; '' / 0 = raw, the default): a1 15 Oct
            **({'attach_z_lowpass_hz': f('attach_z_lowpass', 0.0)} if 'attach_z_lowpass' in typed else {}),
            # a zero attachment twist ('' = raw finite difference; card 2026-10-16_plate_height_filter, critic F4)
            **({'attach_twist': val('attach_twist', '')} if 'attach_twist' in typed else {}),
            **({'shared_trajectory_backdate_s': f('commit_backdate', 0.0)} if 'commit_backdate' in typed else {}),
        }])

    # his inputs from our mocap node (the only Motive owner): his IRL publisher's outputs, under /tejen, and his ball
    rig_state, ball = _instance_rig_inputs(
        context, inst, share, typed, use_sim_time=False, anchor_z=f('anchor_offset_z', RIG['anchor_offset_z']),
        tip_topic='/magnet_tip_pose',
        ball_params={**object_point.get('magnet_attachment_manager_irl', {}),
                     # pickup_attach_radius (typed only; 12 Nov, RIG-1112-fm9c_3): his inferred attach within this distance
                     # of the object's contact point (his file 0.12). At 0.12 the swinging magnet read "attached" on the
                     # way down, then "lost" (latched until a magnet OFF), and the real seat was never confirmed
                     **({'attach_radius_m': f('pickup_attach_radius', 0.12)} if 'pickup_attach_radius' in typed else {})})

    ground_ring = _irl_block(share, 'online_join_planner', 'm1_ground_ring.yaml')
    planner = inst.node(
        topics=PLANNER_TOPICS, fixed=PLANNER_FIXED_TOPICS, extra={'vehicle_id': inst.vehicle_id},
        package='tejen_mission', executable='online_join_planner', name='online_join_planner', output='screen',
        # his IRL order (m1_irl_*_ring.launch.py): later wins; ours last, every topic written out
        parameters=[
            str(share / 'config' / 'visual_astar_visual_only.yaml'),
            str(share / 'config' / 'c1f2b_cpp_authority.yaml'),
            _irl_block(share, 'online_join_planner'),
            str(share / 'config' / 'm1_measured_tip_pickup.yaml'),
            {k: ground_ring[k] for k in GROUND_RING_PICKUP_KEYS},
            str(share / 'config' / 'm1_measured_tip_tracking.yaml'),
            object_point.get('online_join_planner', {}),
            {
                'use_sim_time': False,
                'vehicle_id': 'drone_0',
                'assigned_attachment_id': 'attachment_0',
                'mission_mode': val('mission_mode', 'pickup_delivery'),
                'pickup_alignment_mode': 'measured_tip_tiered',
                'drone_state_topic': state,
                'drone_command_topic': f'/{NS}/drone_command',
                'pickup_object_pose_topic': ball_pose,
                'pickup_object_index': 0,
                'object_attached_topic': ball_attached,
                'magnet_command_topic': f'/{NS}/magnet/command',
                'use_measured_magnet_tip': True,
                'magnet_tip_pose_topic': tip,
                'payload_pose_topic': '/fake_payload/pose',
                'payload_twist_topic': '/fake_payload/twist',
                'attachment_pose_topic': '/fake_attachment_point/pose',
                'attachment_twist_topic': '/fake_attachment_point/twist',
                'external_landing_topic': '/join_planner/land_now',
                'drop_point_source': 'payload',
                'static_obstacle_count': 0,
                'pickup_lift_height': f('pickup_lift_height', H['pickup_lift_height']),
                'drop_approach_clearance': f('drop_approach_clearance', H['drop_approach_clearance']),
                **({'drop_clear_height': drop_clear}
                   if (route == 'low' and not floor_join) or val('return_after_drop', '') else {}),
                **({'drop_release_clearance': _drop_release(context)} if _drop_release(context) is not None else {}),
                'attach_cpp_transit': _flag(context, 'attach_cpp_transit'),
                'attach_transit_clearance': f('attach_transit_clearance', H['attach_transit_clearance']),
                **({'approach_offset_z': f('approach_offset_z', 0.30)} if 'approach_offset_z' in typed else {}),
                # the twin's hand-over bounds (RIG-1029-sq3: on its 1 m cable QUAD4 hovered at 0.08-0.09 m/s at the climb
                # end, just over his 0.08, and the C++ transit was never granted)
                'c1f2_prepare_max_lift_drift_m': 0.30,
                'c1f2_prepare_settle_speed_mps': 0.12,
                **({'takeoff_speed': f('climb_speed', 0.15)} if 'climb_speed' in typed else {}),
                # cpp_authority:=false: his loaded drop leg on his Python transfer, level at the lift height (RIG-1029-sq3,
                # -sq4_5: his C++ hand-over and search floor kept the transit from starting on the rig)
                **({'c1f2_cpp_authority_enabled': False} if val('cpp_authority', '') == 'false' else {}),
                # end_after_drop:=true: after the drop QUAD4 holds clear of the drop zone until LAND (no reattach)
                **({'end_after_drop': True} if val('end_after_drop', '') == 'true' else {}),
                **_return_after_drop(context),
                **_exit_handoff(context),
                **_cpp_python_fallback(context),
                **_prepare_and_release(context),
                # drop_on_arrival:=true (Wesley 4 Nov, k1m_2-4: the swinging object never passed his 0.18 m held 0.5 s):
                # release at the transit height once the magnet is inside drop_on_arrival_xy of the drop point
                **_drop_on_arrival(context), **_takeoff_gate(context),
                **_lift_full(context, G),
                **_lift_cap(context, G['object_rest_z'] + f('pickup_lift_height', H['pickup_lift_height'])),
                # pickup_xy / pickup_speed (typed only; his m1_measured_tip_pickup.yaml 0.025 m / 0.07 m/s): RIG-1029-k1_4, the
                # magnet on the 1 m cable sat 4.1 cm (median) off the object and inside 2.5 cm only 17 % of the time, so his
                # tiered capture never fired (his low and high tiers are already 0.10 / 0.12 on the rig, below)
                **({'pickup_alignment_capture_xy_m': f('pickup_xy', 0.025)} if 'pickup_xy' in typed else {}),
                **({'pickup_alignment_max_relative_speed_mps': f('pickup_speed', 0.07)} if 'pickup_speed' in typed else {}),
                # pickup_low_hold_xy (typed only; Wesley 11 Nov, RIG-1111-fm9c*): in the last 8 cm the descent holds its
                # height while the tip is this far off the object (7 of 7 arrivals 4.7-9.7 cm off failed, 4 of 4 within
                # ~2 cm lifted); no dwell, and the crossing into LOW keeps the 0.10 m above
                **({'pickup_alignment_low_hold_xy_m': f('pickup_low_hold_xy', 0.0)} if 'pickup_low_hold_xy' in typed else {}),
                # pickup_box (typed only; 11 Nov, RIG-1111-fm9c_6): the object further than this from the cage centre in
                # x or y is not gone after (give-up, our take-back)
                **({'pickup_box_m': f('pickup_box', 0.0)} if 'pickup_box' in typed else {}),
                # pickup_press (typed only; 12 Nov, Wesley): the early contact's hold this much lower (tether slack)
                **({'pickup_candidate_press_m': f('pickup_press', 0.0)} if 'pickup_press' in typed else {}),
                # pickup_proof_drift / _speed / _loss (typed only; Wesley 4 Nov, RIG-1104-k1m: the object lifted 1.3 m on the magnet
                # but the magnet-object gap moved ~2 cm as it left the floor, past his 0.015 m drift, so the lift never proved)
                **({'pickup_proof_maximum_contact_drift_m': f('pickup_proof_drift', 0.015)} if 'pickup_proof_drift' in typed else {}),
                **({'pickup_proof_relative_speed_mps': f('pickup_proof_speed', 0.03)} if 'pickup_proof_speed' in typed else {}),
                **({'pickup_proof_loss_contact_drift_m': f('pickup_proof_loss', 0.045)} if 'pickup_proof_loss' in typed else {}),
                'magnet_drop_below_quad': f('magnet_drop_below_quad', G['magnet_drop_below_quad']),
                # no auto-land from ATTACH_READY (its magnet OFF would drop QUAD4 off the ring)
                'attach_ready_land_after_s': 0.0,
                'contact_attach': True,
                'ring_pose_topic': '/payload/motion_capture_state',     # the hold's plate frame (Tejen 15 Oct)
                'final_attach_offset_z': f('contact_dz', RIG['contact_dz']),
                'approach_offset_x': f('contact_dx', 0.0),
                'approach_offset_y': f('contact_dy', 0.0),
                'attach_ready_target_lead_time': f('contact_lead', 0.0),
                'proof_rise_max': f('proof_rise_max', RIG['proof_rise_max']),
                # the drop ends on the measured object leaving the marker; stuck = give-up (critic F9)
                'drop_check_puck': True,
                # 0 = trust an unseen object released (no puck pose in the net); set from Wednesday's mocap check
                'drop_puck_wait_s': f('drop_puck_wait_s', 2.0),
                'contact_topic': '/magnet/contact',
                'ring_attached_topic': '/magnet/object_attached',
                'give_up_topic': '/join_planner/give_up',
                'take_back_topic': f'/drone_{DRONE}/take_back',
                'mux_state_topic': f'/drone_{DRONE}/mux_state',
                # 14 Oct load cuts (Wesley): the visual-only A* (display; the mission reference is
                # unchanged) off, and the CSV every 6th update (30 -> 5 Hz) as in the sim
                'visual_astar_enabled': False,
                'log_every_n_updates': 6,
                # 14 Oct, Tejen's tuning after RIG-1014-t5_4 (Wesley): the tip swung a median 3-4 cm round the puck
                # (QUAD4's wash off the floor), inside the 2.5 cm low tube 14-35 % of the time, so the descent stalled;
                # a wider low tube and a longer dwell at the bottom (the final capture gate stays 2.5 cm)
                'pickup_alignment_low_xy_m': 0.10,    # 14 Oct t5_5 (Wesley): 0.045 still stalled HIGH once
                'pickup_alignment_high_xy_m': 0.12,
                'pickup_alignment_low_dwell_s': 1.0,
                # 14 Oct props-off check (RIG-1014-puck, Wesley): the puck catches anywhere on its face, but t5_4/t5_5
                # lifted within 0.3 s of the proximity "attached" (12 cm), so the magnet barely touched or never did:
                # sit 1.5 s at the bottom, lift only with the tip over the puck, and his 13 Oct overtravel
                'pickup_lift_xy_dwell_s': 1.5,
                'pickup_lift_xy_tolerance': 0.04,
                'pickup_overtravel': 0.036,
                'pickup_max_overtravel': 0.036,
                'pickup_support_surface_overtravel_allowance_m': 0.024,
                # 14 Oct RIG-1014-t6f (Wesley): the slack tether's tip swings a median 3.8 cm / 0.105 m/s over plate 9,
                # so the 2.5 cm / 0.08 m/s match gate never held 1 s; the descent abort is widened past the new gate.
                # The contact itself still has to land inside our weld radius.
                # typed match_xy widens it (Wesley 28 Oct, the replay of the stalled rig MATCH windows: 0.08 opens 3 of 5)
                'contact_match_xy_threshold': f('match_xy', 0.05) if 'match_xy' in typed else 0.05,
                'contact_match_xy_velocity_threshold': f('match_speed', 0.15) if 'match_speed' in typed else 0.15,
                # his gates hold for dwell_time_s unbroken (his 1.0 s): a3_2 (15 Oct) sat 42 s in MATCH, both conditions
                # true 36 % of the time, never a clean second; replayed, 0.5 s passes at 1.1 s. Typed only.
                **({'dwell_time_s': f('match_dwell', 1.0)} if 'match_dwell' in typed else {}),
                # descent_abort_xy (typed only): plan_rig_fix_2026-10-30 W2a, the replay of sq5_2's swing (a full descent
                # survives 49 % of start ticks at 0.08 m with a 1 s debounce, 85 % at 0.10 m)
                'contact_abort_xy_threshold': f('descent_abort_xy', 0.08) if 'descent_abort_xy' in typed else 0.08,
                # descent_abort_speed (typed only; Tejen's review 3 Nov: 0.35 trial): the gross-motion bound of the descent
                'contact_abort_xy_velocity_threshold': f('descent_abort_speed', 0.25) if 'descent_abort_speed' in typed else 0.25,
                # descent_hold / descent_gross_xy (typed only; Tejen's review 3 Nov, priority 1): past the position band the
                # descent holds its height and resumes inside it; it restarts only past descent_gross_xy or the speed abort
                **({'contact_descent_hold': val('descent_hold', '').strip().lower() in ('1', 'true', 'yes')}
                   if 'descent_hold' in typed else {}),
                **({'contact_descent_gross_xy_m': f('descent_gross_xy', 0.20)} if 'descent_gross_xy' in typed else {}),
                # descent_debounce (typed only; DECISIONS_1355 section 2): RIG-1029-sq5_2's 6 of 6 descents left on one tick
                **({'contact_abort_xy_debounce_s': f('descent_debounce', 0.0)} if 'descent_debounce' in typed else {}),
                # plan 2026-10-14 WP3: hold at the bottom for the contact (gross errors and his quiet-miss rule leave),
                # three tries in the give-up time, and his measured-tip trim instead of hand-set contact_dx/dy
                'contact_bottom_hold': True,
                'contact_give_up_s': f('contact_give_up_s', 45.0),
                'target_alignment_mode': val('target_alignment_mode', 'measured_tip_trim'),
                # his trim learns only while the tip moves slower than this (his 0.08 m/s): over plate 9 on 14 Oct the
                # swinging tip read 0.16-0.20 m/s, so in replay it learned <= 1 cm in 7 s of MATCH; 0.20 learned the
                # 4-5 cm offset in 2-4 s. Typed only (the sheet's choice), his value otherwise.
                **({'target_alignment_max_relative_speed_mps': f('trim_speed_gate', 0.08)}
                   if 'trim_speed_gate' in typed else {}),
            },
        ])

    # the C++ return flies with no object: his backend must not wait for one (the moving exit keeps the wait: his exit
    # context skips the object checks for its own transaction)
    overrides = dict(G=G, rod=rod, floor_join=floor_join or (val('return_after_drop', '') == 'cpp'
                                                             and not _exit_moving(context)),
                     carrier_model=carrier_model, margins={**backend_object, **_backend_margins(context)},
                     count=len(azimuths))
    if inst.legacy:
        backend = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(dyn_share / 'launch' / 'c1f6_moving_rendezvous_backend.launch.py')),
            launch_arguments={'planner_rate_hz': '5.0',
                              'minimum_search_z_m': str(search_floor),
                              'cooperative_preview_count': str(len(azimuths)),     # one per listed carrier
                              'use_sim_time': 'false',
                              'extra_params_file': _backend_overrides(
                                  str(share / 'config' / 'partner_backend_overrides_rig.yaml'), **overrides),
                              'state_topic': state,
                              'object_attached_topic': ball_attached}.items())]
    else:
        backend = _instance_backend(inst, share, dyn_share, base_override='partner_backend_overrides_rig.yaml',
                                    overrides=overrides, use_sim_time=False, search_floor=search_floor,
                                    state=state, object_attached=ball_attached)

    # an instance not at the root follows its own planner's reference and mode, and logs under its vehicle id
    controller = inst.node(
        topics=MPC_TOPICS, extra={'logging_instance': inst.vehicle_id},
        package='tejen_mpc', executable='main', name='tejen_mpc', namespace=NS, output='screen',
        parameters=[{
            **_irl_block(share, 'tejen_mpc'),
            'use_sim_time': False,
            'use_external_reference': True,
            'thrust_ratio': f('mpc_thrust_ratio', G['thrust_ratio']),
            'enable_thrust_ratio_feedback': G['thrust_ratio_feedback'],
            **_throttle_max(context),
            'cable_length': f('mpc_cable_length', round(rod - MPC_CABLE_BELOW_DROP, 4)),
            'acados_cache_dir': os.path.expanduser('~/.cache/multi_drone_control/tejen_payload_mpc'),
            'pose_timeout_s': 0.25,
            'arming_state_feedback_period_s': 0.5,
            # his thrust-ratio feedback owns the model gain on the rig, as in his IRL flights
            'object_mass_kg': 0.0,
            'object_vehicle_mass_kg': G['vehicle_mass'],
            'object_attached_topic': ball_attached,
            'mpc_profile': LaunchConfiguration('mpc_profile').perform(context),
            'mpc_command_execution': LaunchConfiguration('mpc_command_execution').perform(context),
        }],
        remappings=[('motion_capture_state', state),
                    ('ELRSCommand', f'/drone_{DRONE}/ELRSCommand_tejen'),
                    ('telemetry', f'/drone_{DRONE}/telemetry')])

    out = [ring, rig_state, ball, planner, *backend, controller]
    if _flag(context, 'supervisor'):
        out.append(inst.node(
            package='tejen_mission', executable='simulation_test_supervisor',
            name='simulation_test_supervisor', namespace=NS, output='screen',
            parameters=[{
                'use_sim_time': False,
                'control_mode': 'manual',          # ARM and TAKEOFF stay with the operator
                'stop_action': 'give_up',          # H5 / F9: a stale input gives up (our take-back), never disarms
                'result_path': ParameterValue(_instance_path(context, inst, 'result_path', typed), value_type=str),
                'startup_timeout_s': 600.0,
                'manual_start_timeout_s': 600.0,
                'mission_timeout_s': 900.0,
                'success_dwell_s': 2.0,
                'freshness_timeout_s': 0.5,
                'planner_status_timeout_s': 3.0,
                'success_phase': 'NONE_PARTNER_WELD_OWNS_SUCCESS',
                'failure_phase': 'LANDED_DISARMED',
                'require_handoff_ready': True,
                'landing_request_phase': '',
            }],
            remappings=[('/motion_capture_state', state),
                        ('/pendulum_swing_state', f'/{NS}/pendulum_swing_state'),
                        ('/payload_world_state', f'/{NS}/payload_world_state')]))
    out.append(ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', _instance_path(context, inst, 'bag_path', typed),
             *map(inst.t, [state, f'/{NS}/pendulum_swing_state', tip,
                           '/join_planner/reference', '/join_planner/phase', f'/drone_{DRONE}/ELRSCommand_tejen',
                           '/fake_payload/pose', '/fake_attachment_point/pose', '/magnet/command', f'/{NS}/magnet/command',
                           ball_attached, ball_pose, '/pickup/motion_capture_state', f'/drone_{DRONE}/magnet',
                           f'/drone_{DRONE}/magnet_ours', f'/drone_{DRONE}/magnet_gate', '/join_planner/land_now',
                           *CONTACT_TOPICS])],
        output='screen', condition=IfCondition(LaunchConfiguration('record'))))
    return out


def _arguments():
    return [
        # a second instance of his stack for another returning drone: instance:=d<d> drone_id:=<d>, e.g.
        # instance:=d2 drone_id:=2 plate:=3; '' = today's QUAD4 instance (/tejen, drone 3, his topics at the root)
        DeclareLaunchArgument('instance', default_value=''),
        DeclareLaunchArgument('drone_id', default_value=''),
        DeclareLaunchArgument('tip_topic', default_value=''),      # partner_rig_state's marker topic ('' = the instance's)
        DeclareLaunchArgument('tip_msg_type', default_value='', choices=['', 'pose_stamped', 'motion_capture_state']),
        DeclareLaunchArgument('geometry', default_value='legacy', choices=sorted(GEOMETRY)),
        DeclareLaunchArgument('plate', default_value=''),   # our plate number (9 = +y; his 3); '' = the geometry's
        DeclareLaunchArgument('object_mass_kg', default_value='0.1'),   # the 0.1 kg object (payload_model.sdf)
        # clockwise degrees; '' = the geometry's; the all-attached start: '[210.0, 90.0, 330.0]', plate 9, gate
        # /partner/release
        DeclareLaunchArgument('carrier_azimuths_deg', default_value=''),
        DeclareLaunchArgument('contact_attach', default_value='false'),
        # 14 Oct (Wesley): floor_join takes off from the mat and goes straight to the ring rejoin (no pickup, no drop);
        # delivery_rejoin takes off already holding the object (magnet ON from boot), drops it, then rejoins
        DeclareLaunchArgument('mission_mode', default_value='pickup_delivery',
                              choices=['pickup_delivery', 'floor_join', 'delivery_rejoin']),
        DeclareLaunchArgument('contact_lead', default_value='0.0'),   # contact_attach: his hold's reference lead (s)
        DeclareLaunchArgument('contact_dz', default_value='0.0'),     # contact_attach: tip target above the contact point (m)
        DeclareLaunchArgument('contact_dx', default_value='0.0'),     # contact_attach: tip target offset along x (m)
        DeclareLaunchArgument('contact_dy', default_value='0.0'),     # contact_attach: tip target offset along y (m)
        DeclareLaunchArgument('drop_puck_wait_s', default_value='2.0'),   # rig drop check: s to wait for a puck pose; 0 = trust the release
        DeclareLaunchArgument('confirm_pickup_release', default_value='false'),
        # contact_attach: no fleet switch 1 s after TAKEN_BACK -> out 1.2 m from the ring, land, disarm (card
        # 2026-10-11_quad4_tilt_cut); false = TAKEN_BACK holds where it is, as before
        DeclareLaunchArgument('taken_back_self_land', default_value='false'),
        # the transit heights: '' = carry_heights (the carried object's bottom carry_clearance above the carriers);
        # typed values win (the old high transit: 2.3 / 2.7 / 1.35)
        DeclareLaunchArgument('pickup_lift_height', default_value=''),
        DeclareLaunchArgument('drop_approach_clearance', default_value=''),
        DeclareLaunchArgument('minimum_search_z_m', default_value=''),
        DeclareLaunchArgument('attach_transit_clearance', default_value=''),   # floor_join: tip above the plate
        DeclareLaunchArgument('approach_offset_z', default_value=''),   # m, tip above the plate at his approach ('' = his 0.30)
        # his C++ collision margins, typed only ('' = his files: swing 0.17 rad twin / 0.2618 rig, tracking 0.05 m)
        DeclareLaunchArgument('max_swing_angle_rad', default_value=''),
        DeclareLaunchArgument('tracking_check', default_value='', choices=['', 'off']),   # 'off': his tracking latch off, as his IRL configs
        DeclareLaunchArgument('cooperative_tracking_half_m', default_value=''),
        DeclareLaunchArgument('cooperative_body_half_m', default_value=''),   # the carriers' body box x, y ('' = his 0.105)
        DeclareLaunchArgument('spline_time_factor', default_value=''),   # his transit's time scale ('' = his 3.0)
        DeclareLaunchArgument('swing_envelope', default_value='', choices=['', 'legacy_square', 'outer_octagon']),
        DeclareLaunchArgument('target_time_iterations', default_value=''),   # his rendezvous-time fixed point steps ('' = his 6)
        DeclareLaunchArgument('workspace_xy', default_value=''),         # his workspace bounds, x and y +- this (with z)
        DeclareLaunchArgument('workspace_z_max', default_value=''),      # ... z from 0 to this ('' = his bounds off)
        DeclareLaunchArgument('carry_clearance', default_value=''),   # m above the carriers' bodies; '' = cable/2
        # low: in between the carriers, 0.30 m over the plates (Wesley 29 Oct); over: above the carriers (21-23 Oct);
        # '' = low for pickup_delivery, over for floor_join. The rig refuses a body above 2.6 m unless over is typed
        DeclareLaunchArgument('transit', default_value='', choices=['', 'low', 'over']),
        DeclareLaunchArgument('ring_z', default_value='1.0'),          # the carried ring's height (target_z)
        # tether: his attached-tether model of the carriers' rods; column: the old tall carrier boxes
        DeclareLaunchArgument('carrier_model', default_value='tether', choices=['tether', 'column']),
        # floor_join's leg to the plate on his C++ planner (collision avoidance); false = his Python transfer
        DeclareLaunchArgument('attach_cpp_transit', default_value='true'),
        # pickup_delivery's flight from the drop zone to the plate on his C++ planner too (default off; Wesley 28 Oct)
        DeclareLaunchArgument('pickup_cpp_reattach', default_value='false'),
        DeclareLaunchArgument('start_gate_topic', default_value='/payload/trajectory_state'),
        DeclareLaunchArgument('start_gate_min_traj_t', default_value='0.0'),   # 13 Nov: the twin's TAKEOFF once the path clock passes it
        # his MPC learns kT from where it arms (no climb first); 'auto' = true when the gate is /partner/release.
        # Applied only where his kT feedback flies (rig_twin)
        DeclareLaunchArgument('airborne_start', default_value='auto', choices=['auto', 'true', 'false']),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_path', default_value=os.path.join(
            os.environ.get('MDC_RUN_DIR', '/tmp'), 'partner_bag')),
        DeclareLaunchArgument('result_path', default_value='/tmp/partner_mission_result.json'),
        # real:=true only (the rig; the sim graph above does not read these)
        DeclareLaunchArgument('real', default_value='false'),
        DeclareLaunchArgument('contact_target_z', default_value=''),     # m, the calibrated weld_point_z
        DeclareLaunchArgument('contact_target_xy', default_value=''),    # 'x,y' m in the ring frame: our weld point ('' = his nominal plate)
        DeclareLaunchArgument('contact_give_up_s', default_value=''),    # rig: s after the first descent ('' = 45)
        DeclareLaunchArgument('target_alignment_mode', default_value=''),  # rig: '' = measured_tip_trim
        DeclareLaunchArgument('trim_speed_gate', default_value=''),      # rig: m/s, his trim's learning speed ('' = his 0.08)
        DeclareLaunchArgument('attach_z_lowpass', default_value=''),     # rig: Hz, his target's plate height low-passed ('' = raw)
        DeclareLaunchArgument('pickup_xy', default_value=''),      # rig: m, his pickup capture window ('' = 0.025)
        DeclareLaunchArgument('pickup_speed', default_value=''),   # rig: m/s, his pickup relative-speed limit ('' = 0.07)
        DeclareLaunchArgument('pickup_press', default_value=''),   # rig: m, hold the early pickup contact this much lower ('' = 0)
        DeclareLaunchArgument('pickup_attach_radius', default_value=''),   # rig: m, his inferred pickup attach radius ('' = his 0.12)
        DeclareLaunchArgument('pickup_box', default_value=''),   # rig: m, no pickup of an object further out than this in x or y ('' = off)
        DeclareLaunchArgument('pickup_low_hold_xy', default_value=''),   # rig: m, hold the last 8 cm of the pickup descent while the tip is this far off ('' = off)
        # rig, typed only: his contact point on the real object (tools/pickup_geometry.py, the magnet face on its centre)
        DeclareLaunchArgument('pickup_contact_height', default_value=''),   # m, magnet marker over the object's marker
        DeclareLaunchArgument('pickup_object_thickness', default_value=''),   # m ('' = 0.025, RIG-1021-q1pick)
        DeclareLaunchArgument('pickup_object_size', default_value=''),   # m, its side ('' = 0.10)
        DeclareLaunchArgument('pickup_point_x', default_value=''),   # m, the contact point's offset from the object's marker, object axes ('' = 0)
        DeclareLaunchArgument('pickup_point_y', default_value=''),
        DeclareLaunchArgument('end_after_drop', default_value='', choices=['', 'true']),
        DeclareLaunchArgument('prepare_settle_accel', default_value=''),   # m/s^2, his hand-over settle accel bound ('' = 0.50)
        DeclareLaunchArgument('prepare_settle_speed', default_value=''),   # m/s, his hand-over settle bound ('' = 0.12)
        DeclareLaunchArgument('drop_release_xy', default_value=''),   # m, the release waits for the magnet this close
        DeclareLaunchArgument('drop_release_wait', default_value=''),   # s, ...but never longer than this ('' = 3)
        DeclareLaunchArgument('cpp_python_fallback', default_value=''),   # s; his C++ lost this long in the drop transit -> Python
        DeclareLaunchArgument('return_after_drop', default_value='', choices=['', 'cpp', 'direct']),   # back to the start after the drop
        DeclareLaunchArgument('exit_handoff', default_value='', choices=['', 'moving']),   # cpp: his C++ exit from the moving prefix
        DeclareLaunchArgument('supervisor_status_timeout_s', default_value=''),   # s, twin checks only ('' = 15)
        DeclareLaunchArgument('return_climb', default_value='', choices=['', 'transit']),
        DeclareLaunchArgument('cpp_max_z', default_value=''),
        DeclareLaunchArgument('cpp_world_refresh', default_value=''),
        DeclareLaunchArgument('cpp_splice', default_value=''),   # his fixed splice lookahead (s); '' = his   # his world refresh period (s); '' = every message
        DeclareLaunchArgument('temporal_refinement', default_value='', choices=['', 'true']),   # his time-subdivided collision checks   # the top of his search box, body z (m); '' = his +1 m   # transit: climb back to the transit height after the drop   # rig: hold after the drop until LAND
        DeclareLaunchArgument('takeoff_gate_topic', default_value=''),   # his TAKEOFF held until this topic speaks
        DeclareLaunchArgument('drop_on_arrival', default_value='', choices=['', 'true']),   # release at the transit height
        DeclareLaunchArgument('line_up', default_value='', choices=['', 'true', 'gap']),   # to the ring along the plate's line
        DeclareLaunchArgument('line_up_radius', default_value=''),    # m, the staging point's distance ('' = 1.2)
        DeclareLaunchArgument('line_up_lateral', default_value=''),   # m, lined up within this of the line ('' = 0.25)
        DeclareLaunchArgument('lift_full', default_value='', choices=['', 'true']),   # his lift to its full height first
        DeclareLaunchArgument('lift_cap', default_value='', choices=['', 'true']),   # his lift's tip no higher than planned (a raised object)
        DeclareLaunchArgument('drop_release', default_value=''),  # m: his settle drop's release, the tip above the drop point ('' = his 0.14)
        DeclareLaunchArgument('drop_height', default_value=''),   # m: the carried object's bottom above the ring origin (~ the net)
        DeclareLaunchArgument('max_body_z', default_value=''),    # m: QUAD4's ceiling ('' = 2.6; at most 3.0)
        DeclareLaunchArgument('mpc_battery_comp', default_value=''),   # his radio throttle per volt below 23.5 V ('' = none)
        DeclareLaunchArgument('mpc_throttle_max', default_value=''),   # his OCP's throttle bound ('' = his 0.60; 0.60-0.95)
        DeclareLaunchArgument('drop_on_arrival_xy', default_value=''),   # m: the magnet's radius over the drop point ('' = 0.12)
        DeclareLaunchArgument('cpp_authority', default_value='', choices=['', 'false']),   # rig: 'false' = his Python transfers only
        DeclareLaunchArgument('climb_speed', default_value=''),   # m/s, his TAKEOFF climb ('' = his 0.15)
        DeclareLaunchArgument('match_dwell', default_value=''),          # rig: s, his gates' unbroken hold ('' = his 1.0)
        DeclareLaunchArgument('descent_debounce', default_value=''),   # s: his contact descent's 8 cm restart held this long ('' = one tick)
        DeclareLaunchArgument('descent_abort_xy', default_value=''),   # m: his contact descent's position restart ('' = rig 0.08, twin his)
        DeclareLaunchArgument('pickup_proof_drift', default_value=''),   # m: his pickup proof's magnet-object drift, rig ('' = 0.015)
        DeclareLaunchArgument('pickup_proof_speed', default_value=''),   # m/s: its relative speed ('' = 0.03)
        DeclareLaunchArgument('pickup_proof_loss', default_value=''),    # m: its loss drift, above the drift ('' = 0.045)
        DeclareLaunchArgument('descent_abort_speed', default_value=''),   # m/s: his contact descent's speed abort, rig ('' = 0.25)
        DeclareLaunchArgument('descent_hold', default_value=''),   # true: past the band his contact descent holds its height ('' = his restart)
        DeclareLaunchArgument('descent_gross_xy', default_value=''),   # m: with descent_hold, a restart only past this ('' = 0.20)
        DeclareLaunchArgument('match_xy', default_value=''),             # twin: m, his contact match xy gate ('' = his)
        DeclareLaunchArgument('attach_twist', default_value='', choices=['', 'zero']),   # 'zero': the bridge's twist zero in hover ('' = raw)
        DeclareLaunchArgument('commit_backdate', default_value=''),   # s: the bridge's shared commitments start this long before now ('' = his timing)
        DeclareLaunchArgument('match_speed', default_value=''),          # rig: m/s, his contact match speed gate ('' = 0.15)
        DeclareLaunchArgument('magnet_drop_below_quad', default_value=''),   # QUAD4's, typed (refused untyped)
        DeclareLaunchArgument('proof_rise_max', default_value=''),       # '' = the rig's 0.03
        DeclareLaunchArgument('mpc_thrust_ratio', default_value=''),     # QUAD4's, typed (refused untyped)
        DeclareLaunchArgument('mpc_cable_length', default_value=''),     # '' = magnet_drop_below_quad - 0.044
        DeclareLaunchArgument('ring_cable_len', default_value=''),       # the carriers' cable (m; typed by our launch)
        DeclareLaunchArgument('ring_cable_elev_deg', default_value=''),  # their elevation ('' = 60)
        DeclareLaunchArgument('anchor_offset_z', default_value=''),      # '' = his IRL -0.030
        DeclareLaunchArgument('supervisor', default_value='false'),      # his supervisor (manual, stops land)
        # his MPC version, sim and rig (Wesley 14 Oct, twin first): oct10 = his 11a0683f controller, flown to
        # 14 Oct; oct14 = his d00934c1 weights, attitude-rate bounds +-3 and partial-step command execution
        DeclareLaunchArgument('mpc_profile', default_value='oct10', choices=['oct10', 'oct14']),
        # his MPC's send path: 'profile' = the profile's; 'partial_step' = the elapsed-interval command under oct10 (Tejen 29 Oct)
        DeclareLaunchArgument('mpc_command_execution', default_value='profile', choices=['profile', 'partial_step', 'endpoint']),
    ]
