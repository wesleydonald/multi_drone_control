"""partner_mission.launch.py (ours): mpc_profile picks his MPC version for QUAD4 on the twin and on the rig
(Wesley 14 Oct, "Port his HEAD, switch"): oct10 (default) his 11a0683f controller, oct14 his d00934c1."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'partner_mission.launch.py'
RIG_TYPED = {'real': 'true', 'contact_attach': 'true', 'contact_target_z': '0.031',
             'magnet_drop_below_quad': '0.575', 'mpc_thrust_ratio': '22.0', 'ring_cable_len': '0.55'}


def _module():
    spec = importlib.util.spec_from_file_location('partner_mission_launch_profile', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _params(typed):
    """Each node's parameters as the launch passes them, with `typed` set before the declarations."""
    mod = _module()
    ctx = LaunchContext()
    ctx.launch_configurations.update(typed)
    mod._record_typed(ctx)
    for a in mod._arguments():
        if isinstance(a, DeclareLaunchArgument) and a.name not in ctx.launch_configurations:
            ctx.launch_configurations[a.name] = a.default_value[0].text if a.default_value else ''
    out = {}
    for e in mod._nodes(ctx):
        if not isinstance(e, Node):
            continue
        params = {}
        for p in evaluate_parameters(ctx, e._Node__parameters or []):
            if isinstance(p, dict):
                params.update(p)
        out[e._Node__node_name] = params
    return out


@pytest.mark.parametrize('path', [{}, {'geometry': 'rig_twin'}, RIG_TYPED], ids=['legacy', 'rig_twin', 'rig'])
def test_default_is_oct10_and_only_his_mpc_takes_it(path):
    p = _params(path)
    assert p['tejen_mpc']['mpc_profile'] == 'oct10'
    assert [name for name, params in p.items() if 'mpc_profile' in params] == ['tejen_mpc']


@pytest.mark.parametrize('path', [{}, {'geometry': 'rig_twin'}, RIG_TYPED], ids=['legacy', 'rig_twin', 'rig'])
def test_oct14_reaches_his_mpc_on_the_twin_and_on_the_rig(path):
    p = _params({**path, 'mpc_profile': 'oct14'})
    assert p['tejen_mpc']['mpc_profile'] == 'oct14'
    assert isinstance(p['tejen_mpc']['mpc_profile'], str)


def test_the_argument_takes_the_two_profiles_only():
    arg = next(a for a in _module()._arguments() if a.name == 'mpc_profile')
    assert arg.choices == ['oct10', 'oct14']
    assert arg.default_value[0].text == 'oct10'
    for value in ('oct10', 'oct14'):
        ctx = LaunchContext()
        ctx.launch_configurations['mpc_profile'] = value
        arg.execute(ctx)
    ctx = LaunchContext()
    ctx.launch_configurations['mpc_profile'] = 'oct12'
    with pytest.raises(Exception, match='oct12'):
        arg.execute(ctx)
