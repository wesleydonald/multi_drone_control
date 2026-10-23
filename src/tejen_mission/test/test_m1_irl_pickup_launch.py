"""m1_irl_pickup.launch.py (Wesley 21 Oct: QUAD1, a 1 m cable, the 0.15 kg object): his IRL stack and config, with an
overlay for the quad, its cable and the object; the planner holds after the lift; measured values refused untyped."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import TimerAction
from launch_ros.actions import Node

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'm1_irl_pickup.launch.py'
TYPED = {'magnet_body_id': '21', 'magnet_drop_below_quad': '1.040', 'contact_height': '0.041',
         'object_thickness': '0.025'}


def _module():
    spec = importlib.util.spec_from_file_location('m1_irl_pickup', LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _context(**typed):
    m = _module()
    ctx = LaunchContext()
    for a in m.generate_launch_description().entities:
        if hasattr(a, 'default_value') and a.default_value is not None:
            ctx.launch_configurations[a.name] = a.default_value[0].text if a.default_value else ''
    ctx.launch_configurations.update(typed)
    return m, ctx


def test_the_overlay_carries_the_quad_the_cable_and_the_object():
    m, ctx = _context(**TYPED)
    overlay, path = m._overlay(ctx)
    assert yaml.safe_load(Path(path).read_text()) == overlay
    mocap = overlay['motion_capture_publisher_irl']['ros__parameters']
    assert (mocap['quad_rigid_body_id'], mocap['magnet_rigid_body_id'], mocap['pickup_rigid_body_id']) == ('11', '21', '6')
    p = overlay['online_join_planner']['ros__parameters']
    assert p['magnet_drop_below_quad'] == 1.04 and p['commissioning_hold_after_lift'] is True
    assert p['payload_pickup_point_z'] == pytest.approx(0.041 - 0.0225)            # contact minus his face offset
    assert overlay['magnet_attachment_manager_irl']['ros__parameters']['pickup_point_offset_z'] == \
        p['payload_pickup_point_z']                                                  # his runner's agreement check
    assert overlay['tejen_mpc']['ros__parameters']['cable_length'] == pytest.approx(1.04 - 0.044)
    assert overlay['elrs_interface_irl']['ros__parameters']['serial_port'] == '/dev/QUAD1'
    assert (p['payload_collision_size_x'], p['payload_collision_size_z']) == (0.10, 0.025)


@pytest.mark.parametrize('missing', sorted(TYPED))
def test_the_measured_values_are_refused_untyped(missing):
    m, ctx = _context(**{k: v for k, v in TYPED.items() if k != missing})
    with pytest.raises(RuntimeError, match=missing):
        m._overlay(ctx)


def test_every_node_loads_his_config_then_the_overlay():
    m, ctx = _context(**TYPED)
    actions = m._nodes(ctx)
    nodes = [a for a in actions if isinstance(a, Node)]
    for t in [a for a in actions if isinstance(a, TimerAction)]:
        nodes += [a for a in t.actions if isinstance(a, Node)]
    names = {n.node_executable for n in nodes}
    assert {'motion_capture_publisher_irl', 'magnet_attachment_manager_irl', 'online_join_planner', 'main',
            'elrs_interface_irl', 'dynamic_planner_transfer_backend', 'fake_cooperative_transport_world'} <= names


def test_measure_mode_starts_only_his_mocap_publisher():
    m, ctx = _context(magnet_body_id='21', measure='true')
    nodes = [a for a in m._nodes(ctx) if isinstance(a, Node)]
    assert [n.node_executable for n in nodes] == ['motion_capture_publisher_irl']
