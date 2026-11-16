"""His M1 IRL stack (m1_irl_virtual_ring.launch.py, irl_commissioning.yaml) for a pickup-only flight on any quad,
cable and object (multi_drone_control, Wesley 21 Oct: QUAD1, a 1 m cable, the 0.15 kg 10 x 10 cm object, ID6).

Every node loads his irl_commissioning.yaml, then an overlay written here from the typed arguments: the quad's and
its magnet marker's Motive ids, its radio, the body-to-marker drop and his controller's cable length, the pickup
contact height and the object's size. The planner holds the finished lift with the magnet ON
(commissioning_hold_after_lift): no transit, no drop. LAND with /join_planner/land_now. ARM and TAKEOFF stay manual.

The typed geometry is measured on the day, not guessed: magnet_drop_below_quad (body origin to magnet marker, the
cable hanging taut) and contact_height (magnet marker above the object's marker with the magnet face on the object,
props off) are refused untyped. measure:=true starts only his mocap publisher (no radio, no controller, nothing can
arm) for tools/pickup_geometry.py to read them.
"""
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# his calibration on QUAD2: body to marker 0.575, his controller's pendulum 0.531 (0.044 shorter)
MPC_CABLE_BELOW_DROP = 0.044


def _overlay(context, measure=False):
    v = lambda name: LaunchConfiguration(name).perform(context).strip()  # noqa: E731
    for name in ('magnet_body_id',) if measure else ('magnet_body_id', 'magnet_drop_below_quad', 'contact_height',
                                                      'object_thickness'):
        if not v(name):
            raise RuntimeError(f'm1_irl_pickup: type {name}:=<value> (measured on the day; see the module docstring)')
    share = Path(get_package_share_directory('tejen_mission'))
    base = yaml.safe_load((share / 'config' / 'irl_commissioning.yaml').read_text())
    face_z = float(base['online_join_planner']['ros__parameters']['magnet_marker_to_contact_face_z'])
    drop = float(v('magnet_drop_below_quad') or 'nan')
    contact = float(v('contact_height') or 'nan')
    size, thick = float(v('object_size')), float(v('object_thickness') or 'nan')
    pickup_point_z = contact + face_z          # the object's contact point above its marker
    mpc_cable = float(v('mpc_cable_length')) if v('mpc_cable_length') else drop - MPC_CABLE_BELOW_DROP
    ids = {'quad_rigid_body_id': v('quad_body_id'), 'magnet_rigid_body_id': v('magnet_body_id'),
           'pickup_rigid_body_id': v('pickup_body_id')}
    overlay = {
        'm1_irl_operator': {'ros__parameters': {
            'pickup_object_mass_kg': float(v('object_mass_kg')), 'pickup_object_diameter_m': size,
            'pickup_object_thickness_m': thick,
            'rigid_body_id_note': ' '.join(f'{k.split("_")[0]}={i}' for k, i in ids.items())}},
        'motion_capture_publisher_irl': {'ros__parameters': {
            **ids, 'attachment_offset_z': float(v('anchor_offset_z'))}},
        'magnet_attachment_manager_irl': {'ros__parameters': {'pickup_point_offset_z': pickup_point_z}},
        'online_join_planner': {'ros__parameters': {
            'magnet_drop_below_quad': drop,
            'payload_pickup_point_z': pickup_point_z,
            'payload_collision_size_x': size, 'payload_collision_size_y': size, 'payload_collision_size_z': thick,
            'commissioning_hold_after_lift': True}},
        'tejen_mpc': {'ros__parameters': {'cable_length': mpc_cable}},
        'elrs_interface_irl': {'ros__parameters': {'serial_port': v('serial_port')}},
        'dynamic_planner_transfer_backend': {'ros__parameters': {
            'suspended_cable_length_m': drop,
            'payload_half_x_m': size / 2, 'payload_half_y_m': size / 2, 'payload_half_z_m': thick / 2,
            'payload_center_from_magnet_z_m': face_z - thick / 2}},
    }
    out = tempfile.NamedTemporaryFile('w', prefix='m1_irl_pickup_', suffix='.yaml', delete=False)
    yaml.safe_dump(overlay, out)
    out.close()
    return overlay, out.name


def _nodes(context):
    if LaunchConfiguration('measure').perform(context).strip() == 'true':
        _, overlay_path = _overlay(context, measure=True)
        config = str(Path(get_package_share_directory('tejen_mission')) / 'config' / 'irl_commissioning.yaml')
        return [LogInfo(msg='[m1_irl_pickup] MEASURE ONLY: his mocap publisher, no radio, no controller'),
                Node(package='tejen_mission', executable='motion_capture_publisher_irl',
                     name='motion_capture_publisher_irl', output='screen', parameters=[config, overlay_path])]
    overlay, overlay_path = _overlay(context)
    drone_share = Path(get_package_share_directory('tejen_mission'))
    dynamic_share = Path(get_package_share_directory('tejen_dynamic_planner'))
    visualisation_share = Path(get_package_share_directory('drone_visualisation'))
    config = str(drone_share / 'config' / 'irl_commissioning.yaml')
    lab = [config, overlay_path]

    def node(package, executable, params, **kw):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=params, **kw)

    backend_config = str(dynamic_share / 'config' / 'c1f_irl_m1_virtual_ring.yaml')
    p = overlay['online_join_planner']['ros__parameters']
    m = overlay['motion_capture_publisher_irl']['ros__parameters']
    return [
        LogInfo(msg=f"[m1_irl_pickup] quad {m['quad_rigid_body_id']}, magnet {m['magnet_rigid_body_id']}, object "
                    f"{m['pickup_rigid_body_id']}; radio {overlay['elrs_interface_irl']['ros__parameters']['serial_port']}; "
                    f"body to marker {p['magnet_drop_below_quad']:.3f} m, his cable "
                    f"{overlay['tejen_mpc']['ros__parameters']['cable_length']:.3f} m; contact point "
                    f"{p['payload_pickup_point_z']:.4f} m above the object marker; HOLD AFTER LIFT (no transit, no drop)"),
        node('tejen_mission', 'motion_capture_publisher_irl', lab),
        node('tejen_mission', 'magnet_attachment_manager_irl', lab),
        TimerAction(period=0.5, actions=[node('tejen_dynamic_planner', 'dynamic_planner_transfer_backend', [
            backend_config, *lab, {'planner_rate_hz': 5.0, 'minimum_search_z_m': 0.20,
                                   'stopped_continuation_enabled': True}])]),
        TimerAction(period=1.0, actions=[node('tejen_mission', 'fake_cooperative_transport_world', lab)]),
        TimerAction(period=1.5, actions=[node('tejen_mission', 'online_join_planner', [
            str(drone_share / 'config' / 'visual_astar_visual_only.yaml'),
            str(drone_share / 'config' / 'c1f2b_cpp_authority.yaml'),
            *lab,
            str(drone_share / 'config' / 'm1_measured_tip_pickup.yaml'),
            str(drone_share / 'config' / 'm1_measured_tip_tracking.yaml'),
            overlay_path,                      # after his pickup overlays: ours is the last word
            {'pickup_alignment_mode': LaunchConfiguration('pickup_alignment_mode').perform(context),
             'm1_gentle_takeoff_enabled': True, 'c1f2_stopped_continuation_enabled': True,
             'target_alignment_mode': 'legacy'}])]),
        TimerAction(period=2.0, actions=[
            Node(package='tejen_mpc', executable='main', name='tejen_mpc', output='screen',
                 parameters=[*lab, {'external_reference_validate_timing': True}]),
            node('tejen_mission', 'elrs_interface_irl', lab)]),
        TimerAction(period=2.5, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(visualisation_share / 'launch' / 'view_frame.launch.py')))]
            if LaunchConfiguration('gui').perform(context) == 'true' else []),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('quad_body_id', default_value='11'),          # QUAD1
        DeclareLaunchArgument('magnet_body_id', default_value=''),          # its magnet marker's Motive id (typed)
        DeclareLaunchArgument('pickup_body_id', default_value='6'),
        DeclareLaunchArgument('serial_port', default_value='/dev/QUAD1'),
        DeclareLaunchArgument('magnet_drop_below_quad', default_value=''),  # m, body origin to marker, hanging (typed)
        DeclareLaunchArgument('mpc_cable_length', default_value=''),        # '' = the drop - 0.044 (his calibration)
        DeclareLaunchArgument('anchor_offset_z', default_value='-0.04'),    # the tether's anchor below the body origin
        DeclareLaunchArgument('contact_height', default_value=''),          # m, marker above the object marker at contact (typed)
        DeclareLaunchArgument('object_size', default_value='0.10'),         # m, the object's square side
        DeclareLaunchArgument('object_thickness', default_value=''),        # m (typed)
        DeclareLaunchArgument('object_mass_kg', default_value='0.15'),
        DeclareLaunchArgument('pickup_alignment_mode', default_value='measured_tip_tiered',
                              choices=['legacy', 'measured_tip_tiered']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('measure', default_value='false'),           # true: his mocap publisher only
        OpaqueFunction(function=_nodes),
    ])
