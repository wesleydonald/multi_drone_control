"""
run_experiment's pre-run refusals for the rig twin (card 2026-10-01, critic phase 2):
launch args count only when a launch really declares or includes them, the plant thrust
map follows the world in both launch sections, and the controller geometry agrees with
the world unless a key is exempted by name. The sim launch hands the tracker the rig's
thrust map. Nothing is started.
"""
import importlib.util
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import run_experiment as R  # noqa: E402

CONFIGS = os.path.join(REPO, 'configs', 'experiments')
TWINS = sorted(f for f in os.listdir(CONFIGS) if f.startswith('rig_twin_'))


def _launch_pkg(tmp_path, files):
    d = tmp_path / 'src' / 'pkg' / 'launch'
    d.mkdir(parents=True)
    for name, text in files.items():
        (d / name).write_text(text)
    return tmp_path


DECLARES_X = "from launch.actions import DeclareLaunchArgument\nA = [DeclareLaunchArgument('x')]\n"


def test_a_launch_named_in_a_comment_declares_nothing(tmp_path, monkeypatch):
    repo = _launch_pkg(tmp_path, {
        'a_launch.py': '"""RUN ORDER: b_launch.py first."""\n# see b_launch.py\nA = 1\n',
        'b_launch.py': DECLARES_X})
    monkeypatch.setattr(R, 'REPO', str(repo))
    assert 'x' not in R.declared_launch_args('pkg', 'a_launch.py')


def test_an_included_launch_declares_its_args(tmp_path, monkeypatch):
    repo = _launch_pkg(tmp_path, {
        'a_launch.py': ("from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
                        "S = PythonLaunchDescriptionSource(str(share / 'launch' / 'b_launch.py'))\n"),
        'b_launch.py': DECLARES_X})
    monkeypatch.setattr(R, 'REPO', str(repo))
    assert 'x' in R.declared_launch_args('pkg', 'a_launch.py')


def test_the_sim_launch_declares_the_tracker_thrust_map():
    names = R.declared_launch_args('bringup', 'sim_control_launch.py')
    assert {'thrust_offset', 'thrust_offset_v_slope', 'throttle_max'} <= names
    import launch_args                     # thrust_v_ref: a params_file knob, not an argument
    assert not launch_args.unknown('sim_control_launch.py', {'thrust_v_ref': 23.5})


def _cfg(tmp_path, name, **edit):
    raw = yaml.safe_load(open(os.path.join(CONFIGS, name)))
    for k, v in edit.items():
        sec, _, key = k.partition('__')
        if key:
            raw[sec]['args'][key] = v
        elif v is None:
            raw.pop(sec, None)
        else:
            raw[sec] = v
    p = tmp_path / name
    p.write_text(yaml.safe_dump(raw))
    return str(p)


@pytest.mark.parametrize('name', TWINS)
def test_every_rig_twin_config_passes(name):
    cfg = R.checked_config(os.path.join(CONFIGS, name))
    assert R.sim_thrust_maps(cfg) == ('rig', 'rig')
    assert float(cfg.launch_args['payload_rest_z']) == pytest.approx(0.02)


def test_rig_map_on_one_launch_only_is_refused(tmp_path):
    with pytest.raises(SystemExit, match='both launch sections'):
        R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml', io_launch__sim_thrust_map='linear'))


def test_rig_world_on_the_linear_map_is_refused(tmp_path):
    p = _cfg(tmp_path, 'rig_twin_hover_fixed.yaml', launch__sim_thrust_map='linear',
             io_launch__sim_thrust_map='linear')
    with pytest.raises(SystemExit, match='rig world'):
        R.checked_config(p)


def test_legacy_world_on_the_rig_map_is_refused(tmp_path):
    p = _cfg(tmp_path, 'ocp_hover_ground_ring086.yaml')
    raw = yaml.safe_load(open(p))
    raw['launch']['args']['sim_thrust_map'] = 'rig'
    raw.setdefault('io_launch', {'file': 'sim_io_launch.py', 'args': {}})
    raw['io_launch'].setdefault('args', {})['sim_thrust_map'] = 'rig'
    open(p, 'w').write(yaml.safe_dump(raw))
    with pytest.raises(SystemExit, match='legacy world'):
        R.checked_config(p)


def test_a_world_in_old_worlds_flies_the_legacy_profile(tmp_path):
    """The runner infers legacy from the world path: legacy:=true on the control launch and the
    linear plant on the I/O launch, unless the config types them."""
    cfg = R.checked_config(os.path.join(CONFIGS, 'ocp_hover_ground_creep_defaults.yaml'))
    assert cfg.world.startswith('old_worlds/') and cfg.legacy
    assert 'legacy:=true' in cfg.launch_argv(str(tmp_path / 'p.yaml'))
    assert 'sim_thrust_map:=linear' in cfg.io_launch_argv()
    assert R.sim_thrust_maps(cfg) == ('linear', 'linear')
    twin = R.checked_config(os.path.join(CONFIGS, 'rig_twin_hover_fixed.yaml'))
    assert not twin.legacy and not [a for a in twin.launch_argv(str(tmp_path / 'q.yaml'))
                                    if a.startswith('legacy')]
    m2 = R.ExperimentConfig.from_yaml(os.path.join(CONFIGS, 'm2_bench.yaml'))
    assert m2.legacy and 'legacy:=true' in m2.launch_argv(str(tmp_path / 'r.yaml'))
    assert not [a for a in m2.io_launch_argv() if a.startswith('sim_thrust_map')]


def _bare(tmp_path, world, args=None, io=None, n=3, mode='mpc'):
    p = tmp_path / 'bare.yaml'
    p.write_text(yaml.safe_dump({
        'name': 'bare', 'world': world,
        'launch': {'package': 'bringup', 'file': 'sim_control_launch.py', 'args': {'mode': mode, **(args or {})}},
        'io_launch': {'file': 'sim_io_launch.py', 'args': dict(io or {})},
        'fleet': {'num_drones': n, 'n_total': n},
        'events': [{'t': 3.0, 'do': 'ARM'}]}))
    return str(p)


def test_a_bare_twin_config_passes_on_the_defaults(tmp_path):
    """World, fleet and events only: the sim defaults are the twin's (1 m cables since 22 Oct), so nothing is typed;
    a 0.55 m world needs its cable typed."""
    cfg = R.checked_config(_bare(tmp_path, 'three_rigid_ground_rig_L100.sdf'))
    assert R.sim_thrust_maps(cfg) == ('rig', 'rig') and not cfg.legacy
    with pytest.raises(SystemExit, match='cable_len: world 0.55'):
        R.checked_config(_bare(tmp_path, 'three_rigid_ground_rig.sdf'))
    R.checked_config(_bare(tmp_path, 'three_rigid_ground_rig.sdf', {'cable_len': 0.55}))


def test_the_newcomers_rod_must_be_its_worlds_arm(tmp_path):
    """attach_cable_len is QUAD4's rod from its pivot (Wesley 22 Oct): untyped it is the carriers' cable_len, the 1 m
    arm; the body-to-tip typed in its place (+4 cm, every attach config before 22 Oct) is refused unless exempted."""
    w = 'three_rigid_ground_rig_attach_L100.sdf'
    a = {'num_drones': 3, 'legacy': False}
    R.checked_config(_bare(tmp_path, w, a, mode='attach'))
    with pytest.raises(SystemExit, match='attach_cable_len: world 1, controller 1.04'):
        R.checked_config(_bare(tmp_path, w, {**a, 'attach_cable_len': 1.04}, mode='attach'))
    raw = yaml.safe_load(open(_bare(tmp_path, w, {**a, 'attach_cable_len': 1.04}, mode='attach')))
    raw['geometry_exempt'] = {'attach_cable_len': 'as flown'}
    p = tmp_path / 'exempt.yaml'
    p.write_text(yaml.safe_dump(raw))
    R.checked_config(str(p))


def test_the_newcomers_rod_is_compared_body_to_tip(tmp_path):
    """A legacy newcomer's arm is 0.50 from a pivot 1 cm above its body and the legacy planner's pivot is the body: its
    rod is 0.49, and a typed 0.50 is refused (left untyped, the legacy profile's 0.50 as flown is only reported); the
    twin's body-to-tip 0.59 typed as its rod is refused too."""
    w = 'old_worlds/three_attach_rod.sdf'
    a = {'num_drones': 3, 'legacy': True, 'cable_len': 0.5}
    R.checked_config(_bare(tmp_path, w, {**a, 'attach_cable_len': 0.49}, mode='attach'))
    R.checked_config(_bare(tmp_path, w, a, mode='attach'))
    with pytest.raises(SystemExit, match='attach_cable_len: world 0.49, controller 0.5'):
        R.checked_config(_bare(tmp_path, w, {**a, 'attach_cable_len': 0.5}, mode='attach'))
    tw, t = 'three_rigid_ground_rig_attach.sdf', {'num_drones': 3, 'legacy': False, 'cable_len': 0.55}
    R.checked_config(_bare(tmp_path, tw, {**t, 'attach_cable_len': 0.55}, mode='attach'))
    with pytest.raises(SystemExit, match='attach_cable_len: world 0.55, controller 0.59'):
        R.checked_config(_bare(tmp_path, tw, {**t, 'attach_cable_len': 0.59}, mode='attach'))


def test_the_twin_markers_take_the_worlds_rod_length(tmp_path):
    """io tether_tips places each magnet marker tether_len along its rod: one that is not the world's rods is refused
    (the launch default is 1.0 since 22 Oct)."""
    tips = {'tether_tips': True}
    R.checked_config(_bare(tmp_path, 'three_rigid_ground_rig_L100.sdf', io=tips))
    with pytest.raises(SystemExit, match='tether_len 0.55 but the rods'):
        R.checked_config(_bare(tmp_path, 'three_rigid_ground_rig_L100.sdf', io={**tips, 'tether_len': 0.55}))
    with pytest.raises(SystemExit, match='tether_len 1 but the rods'):
        R.checked_config(_bare(tmp_path, 'three_rigid_ground_rig.sdf', {'cable_len': 0.55}, io=tips))
    R.checked_config(_bare(tmp_path, 'three_rigid_ground_rig.sdf', {'cable_len': 0.55},
                           io={**tips, 'tether_len': 0.55}))


def test_the_network_flies_the_twin_and_a_legacy_only_mode_is_refused(tmp_path, monkeypatch):
    """mode network flies the rig twin (built 22 Oct, card 2026-10-23_paper_method_compare); a mode listed legacy-only
    is still refused on a twin world."""
    R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml', launch__mode='network'))
    from bringup import profiles
    monkeypatch.setattr(profiles, 'LEGACY_ONLY', ('network',))
    with pytest.raises((SystemExit, RuntimeError), match='legacy:=true'):
        R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml', launch__mode='network'))


def test_a_typed_params_file_that_does_not_exist_is_refused(tmp_path):
    """sim-C review (22 Oct): every C config named configs/twin/ before it existed, and --check passed."""
    pf = tmp_path / 'knobs.yaml'
    pf.write_text('traj_hold_s: 5.0\n')
    R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml', launch__params_file=str(pf)))
    with pytest.raises(SystemExit, match='launch params_file not found: .*nowhere.yaml'):
        R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml',
                              launch__params_file=str(tmp_path / 'nowhere.yaml')))


def test_model_f1_pivot_needs_its_named_exemption(tmp_path):
    with pytest.raises(SystemExit, match='pivot_offset_z'):
        R.checked_config(_cfg(tmp_path, 'rig_twin_model_f1.yaml', geometry_exempt=None))


def test_a_wrench_on_a_world_without_the_push_tool_is_refused(tmp_path):
    R.checked_config(os.path.join(CONFIGS, 'app_pushz_none.yaml'))
    with pytest.raises(SystemExit, match='WRENCH event but three_rigid_ground_rig.sdf has no ApplyLinkWrench'):
        R.checked_config(_cfg(tmp_path, 'app_pushz_none.yaml', world='three_rigid_ground_rig.sdf'))


def test_a_wrong_rod_length_is_refused_without_mis_seed(tmp_path):
    with pytest.raises(SystemExit, match='cable_len'):
        R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml', launch__cable_len=0.5))
    R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml', launch__cable_len=0.5, mis_seed=True))


def test_a_typed_ring_inertia_must_agree_with_the_world(tmp_path, capsys):
    """load_ixx / load_izz (card 2026-10-23_scalability): untyped or 0 the planner takes params.py's per-kg ring and
    the runner only warns, as before; typed, they are refused when they disagree with the world's ring."""
    R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml', launch__load_ixx=0, launch__load_izz=0))
    assert 'load_ixx' in capsys.readouterr().err
    with pytest.raises(SystemExit, match='load_ixx'):
        R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml', launch__load_ixx=0.11, launch__load_izz=0.22))
    R.checked_config(_cfg(tmp_path, 'rig_twin_hover_fixed.yaml', launch__load_ixx=2.227e-2, launch__load_izz=4.431e-2))


# ── the sim launch, evaluated offline ────────────────────────────────────────

def _controller_params(**args):
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    path = os.path.join(REPO, 'src/bringup/launch/sim_control_launch.py')
    spec = importlib.util.spec_from_file_location('launch_mpc_cfgcheck', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations.update({'mode': 'mpc', **{k: str(v) for k, v in args.items()}})
    out = {}
    for e in mod.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            e.execute(ctx)
        elif isinstance(e, OpaqueFunction):
            for n in e.execute(ctx) or []:
                if isinstance(n, Node) and n.node_executable == 'tracker':
                    ps = {}
                    for d in evaluate_parameters(ctx, n._Node__parameters):
                        ps.update(d)
                    out[ps['drone_id']] = ps
    return out


def test_sim_launch_defaults_fly_the_rig_tracker_map():
    p = _controller_params()[2]
    assert (p['thrust_ratio'], p['thrust_offset'], p['thrust_offset_v_slope'], p['throttle_max']) \
        == (35.2, 0.185, 0.022, 0.8)


def test_legacy_sim_launch_leaves_the_tracker_linear():
    p = _controller_params(num_drones=2, legacy='true')[0]
    assert (p['thrust_offset'], p['thrust_offset_v_slope'], p['throttle_max']) == (0.0, 0.0, 0.6)


def test_sim_launch_passes_the_rig_tracker_map():
    p = _controller_params(num_drones=4, sim_thrust_map='rig', thrust_ratio='35.2',
                           thrust_offset='0.185', thrust_offset_v_slope='0.022',
                           throttle_max='0.8', sim_thrust_offset='0.181,0.185,0.187,0.187')[3]
    assert (p['thrust_ratio'], p['thrust_offset'], p['thrust_offset_v_slope'], p['throttle_max']) \
        == (35.2, 0.185, 0.022, 0.8)


def test_sim_launch_refuses_a_rig_plant_with_the_linear_tracker():
    with pytest.raises(RuntimeError, match='explicit thrust_ratio'):
        _controller_params(num_drones=4, legacy='true', sim_thrust_map='rig')


def test_sim_launch_refuses_a_wrong_offset_count():
    with pytest.raises(RuntimeError, match='one value or 4'):
        _controller_params(num_drones=4, sim_thrust_offset='0.18,0.19')
