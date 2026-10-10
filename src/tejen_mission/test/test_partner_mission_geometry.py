"""partner_mission.launch.py (ours): geometry:=legacy keeps every old-world value; geometry:=rig_twin gives his
mission the rig-twin attach world (the combined M1 subtest, Wesley 10 Oct)."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'partner_mission.launch.py'


def _module():
    spec = importlib.util.spec_from_file_location('partner_mission_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _text(key):
    """A normalised parameter name (a tuple of TextSubstitutions) as a str."""
    return key if isinstance(key, str) else ''.join(getattr(t, 'text', str(t)) for t in key)


def _params(geometry):
    mod = _module()
    ctx = LaunchContext()
    for a in mod._arguments():
        if isinstance(a, DeclareLaunchArgument):
            ctx.launch_configurations[a.name] = a.default_value[0].text if a.default_value else ''
    if geometry is not None:
        ctx.launch_configurations['geometry'] = geometry
    out = {}
    for node in mod._nodes(ctx):
        name = getattr(node, '_Node__node_name', None)
        for p in getattr(node, '_Node__parameters', None) or []:
            if isinstance(p, dict) and name:
                out.setdefault(name, {}).update({_text(k): v for k, v in p.items()})
    return out


def test_default_is_the_old_world():
    p = _params(None)
    ring, mpc = p['fake_cooperative_transport_world'], p['tejen_mpc']
    assert (ring['bridge_attach_radius_m'], ring['bridge_cable_len_m'], ring['bridge_cable_elev_deg']) == (0.25, 0.5, 45.0)
    assert (ring['ring_plate_pitch_diameter_m'], ring['ring_collision_outer_diameter_m']) == (0.50, 0.56)
    assert p['online_join_planner']['magnet_drop_below_quad'] == 0.49
    assert (mpc['thrust_ratio'], mpc['enable_thrust_ratio_feedback'], mpc['cable_length'],
            mpc['object_vehicle_mass_kg']) == (82.7, False, 0.49, 0.643)


def test_rig_twin_values():
    p = _params('rig_twin')
    ring, mpc = p['fake_cooperative_transport_world'], p['tejen_mpc']
    assert (ring['bridge_attach_radius_m'], ring['bridge_cable_len_m'], ring['bridge_cable_elev_deg']) == (0.225, 0.55, 60.0)
    assert (ring['ring_plate_pitch_diameter_m'], ring['ring_collision_outer_diameter_m']) == (0.45, 0.51)
    assert p['online_join_planner']['magnet_drop_below_quad'] == 0.59
    assert (mpc['thrust_ratio'], mpc['enable_thrust_ratio_feedback'], mpc['cable_length'],
            mpc['object_vehicle_mass_kg']) == (22.0, True, 0.55, 0.55)


def test_unknown_geometry_is_refused():
    with pytest.raises(RuntimeError, match='geometry'):
        _params('rig')
