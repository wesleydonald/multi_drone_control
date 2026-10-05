"""
Every sim Betaflight bridge runs its rate loop on the gyro by default (Wesley, 2026-09-28).

In imu mode a bridge sends no motor command until a gyro sample arrives, so each launch
that starts one must also bridge that drone's IMU to the bridge's imu_topic, and the
runner must refuse a world without the Imu system before Gazebo starts. The launches are
evaluated offline: nothing is started.
"""
import importlib.util
import os
import sys

import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,  # noqa: E402
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.utilities import perform_substitutions  # noqa: E402
from launch_ros.actions import Node  # noqa: E402
from launch_ros.utilities import evaluate_parameters  # noqa: E402

import yaml  # noqa: E402

BRIDGES = ('payload_betaflight_comm', 'tejen_betaflight_communication')
SIM_CONTROL = 'src/bringup/launch/sim_control_launch.py'
# the sim control launch's modes; the others are separate launch files
MODES = ('attach', 'dissipative', 'network', 'mpc')
LAUNCH = {
    'm2_bench_io': 'src/bringup/launch/sim_m2_bench_launch.py',
    'tejen_linear': 'src/simulation_communication/launch/tejen_betaflight_linear_simulation_launch.py',
    'c1e': 'src/tejen_dynamic_planner/launch/c1e_commissioning_sim.launch.py',
}


def _s(ctx, x):
    if x is None or isinstance(x, str):
        return x
    if hasattr(x, 'perform'):
        return x.perform(ctx)
    return perform_substitutions(ctx, list(x))


def _collect(entities, ctx, out):
    for e in entities:
        if getattr(e, 'condition', None) is not None and not e.condition.evaluate(ctx):
            continue
        if isinstance(e, DeclareLaunchArgument):
            e.execute(ctx)
        elif isinstance(e, Node):
            out.append(e)
        elif isinstance(e, TimerAction):
            _collect(e.actions, ctx, out)
        elif isinstance(e, GroupAction):
            _collect(e.get_sub_entities(), ctx, out)
        elif isinstance(e, OpaqueFunction):
            _collect(e.execute(ctx) or [], ctx, out)
        elif isinstance(e, IncludeLaunchDescription):
            for k, v in e.launch_arguments:
                ctx.launch_configurations[_s(ctx, k)] = _s(ctx, v)
            _collect(e.launch_description_source.get_launch_description(ctx).entities, ctx, out)
        elif isinstance(e, ExecuteProcess):
            out.append(e)


def evaluate(name, **args):
    """(bridges as {imu_topic: rate_source}, imu bridges as {ros topic: gz topic}, gz cmds).
    name: a sim_control_launch.py mode, or a key of LAUNCH."""
    args = {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in args.items()}
    if name in MODES:
        import launch_args
        path = os.path.join(REPO, SIM_CONTROL)
        args = {**args, 'mode': name}
        if launch_args.split('sim_control_launch.py', args)[1]:
            import tempfile
            pf = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False).name
            args = dict(a.split(':=', 1) for a in launch_args.argv('sim_control_launch.py', args, pf))
    else:
        path = os.path.join(REPO, LAUNCH[name])
    spec = importlib.util.spec_from_file_location(f'launch_{name}', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations.update(args)
    out = []
    _collect(mod.generate_launch_description().entities, ctx, out)
    bridges, imu, gz = {}, {}, []
    for e in out:
        if not isinstance(e, Node):
            gz.append(' '.join(_s(ctx, c) for c in e.cmd))
        elif e.node_executable in BRIDGES:
            p = {}
            for d in evaluate_parameters(ctx, e._Node__parameters or []):
                p.update(d)
            topic = p.get('imu_topic', '<node default>')
            ns = _s(ctx, e._Node__node_namespace)
            if not topic.startswith('/') and ns:
                topic = f'/{ns}/{topic}'
            bridges[topic] = p.get('rate_source', '<node default>')
        elif e.node_executable == 'parameter_bridge':
            remap = {_s(ctx, k): _s(ctx, v) for k, v in (e._Node__remappings or [])}
            for a in (_s(ctx, a) for a in (e._Node__arguments or [])):
                if 'sensor_msgs/msg/Imu' in a:
                    src = a.split('@')[0]
                    imu[remap.get(src, src)] = src
    return bridges, imu, gz


def _config(name):
    path = os.path.join(REPO, 'configs', 'experiments', f'{name}.yaml')
    if not os.path.exists(path):
        pytest.skip('configs/ (gitignored) not in this checkout')
    return yaml.safe_load(open(path))


@pytest.fixture(autouse=True)
def _installed():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('simulation_communication')
    except Exception:
        pytest.skip('source install/setup.bash')


@pytest.mark.parametrize('config,n', [('partner_attached_orbit', 4), ('detach_ocp_n4', 4),
                                      ('diss_circle_n3', 3), ('ocp_hover_ground_creep', 3),
                                      ('m1_rejoin_hover', 4)])
def test_our_launches_fly_the_gyro_and_bridge_every_imu(config, n):
    cfg = _config(config)
    args = dict(cfg['launch']['args'])
    bridges, imu, _ = evaluate(args.pop('mode'), **args)
    assert bridges == {f'/drone_{i}/imu': 'imu' for i in range(n)}     # incl. drone 3 in M1
    assert set(imu) == set(bridges)


@pytest.mark.parametrize('mode', MODES)
def test_our_launches_keep_pose_selectable(mode):
    bridges, _, _ = evaluate(mode, num_drones=3, rate_source='pose')
    assert bridges and set(bridges.values()) == {'pose'}


def test_m2_bench_defaults_to_the_gyro_on_the_imu_world():
    cfg = _config('m2_bench')
    bridges, imu, gz = evaluate('m2_bench_io', **cfg['io_launch']['args'])
    assert bridges == {f'/drone_{i}/imu': 'imu' for i in range(4)} and set(imu) == set(bridges)
    assert cfg['world'] == 'tejen/bench_m2/m2_bench_world_imu.sdf' and gz == []
    _, _, gz = evaluate('m2_bench_io')                              # start_gz: picks the world
    assert gz and gz[0].endswith('m2_bench_world_imu.sdf')
    bridges, imu, gz = evaluate('m2_bench_io', rate_source='pose')
    assert set(bridges.values()) == {'pose'} and not imu and gz[0].endswith('m2_bench_world.sdf')


def test_m2_bench_refuses_the_gyro_on_a_world_without_the_imu_system(tmp_path):
    src = os.path.join(REPO, 'simulation_assets', 'tejen', 'bench_m2', 'm2_bench_world.sdf')
    if not os.path.exists(src):
        pytest.skip('bench world not generated')
    world = tmp_path / 'bench_copy.sdf'
    world.write_text(open(src).read())
    with pytest.raises(RuntimeError, match='no Imu system'):
        evaluate('m2_bench_io', world=str(world))


@pytest.mark.parametrize('launch', ['tejen_linear', 'c1e'])
def test_tejen_single_x3_launches_bridge_the_gyro(launch):
    gz = '/world/quadcopter/model/x3/link/X3/base_link/sensor/imu_sensor/imu'
    bridges, imu, _ = evaluate(launch)
    assert bridges == {'/imu': 'imu'} and imu == {'/imu': gz}
    bridges, imu, _ = evaluate(launch, rate_source='pose')
    assert bridges == {'/imu': 'pose'} and not imu


# ── tools/run_experiment.py: refuse the world before Gazebo starts ────────────────────

def _cfg(tmp_path, world, launch_args=None, io_args=None):
    from experiment.config import ExperimentConfig
    doc = {'name': 'unit', 'world': world,
           'launch': {'package': 'bringup', 'file': 'sim_control_launch.py',
                      'args': dict({'mode': 'dissipative', 'num_drones': 4}, **(launch_args or {}))},
           'io_launch': {'file': 'sim_m2_bench_launch.py', 'args': io_args or {}},
           'fleet': {'num_drones': 4, 'n_total': 4}, 'timing': {'duration_s': 60.0},
           'events': [{'t': 3.0, 'do': 'ARM'}, {'t': 5.0, 'do': 'TAKEOFF'}]}
    p = tmp_path / 'exp.yaml'
    p.write_text(yaml.safe_dump(doc, sort_keys=False))
    return ExperimentConfig.from_yaml(str(p)), str(p)


def test_runner_rate_source_is_imu_unless_every_setting_is_pose(tmp_path):
    import run_experiment as rx
    w = 'three_rigid_ground.sdf'
    assert rx.rate_source(_cfg(tmp_path, w)[0]) == 'imu'
    assert rx.rate_source(_cfg(tmp_path, w, io_args={'rate_source': 'pose'})[0]) == 'pose'
    assert rx.rate_source(_cfg(tmp_path, w, {'rate_source': 'pose'}, {'rate_source': 'imu'})[0]) == 'imu'


def test_runner_refuses_a_world_without_the_imu_system(tmp_path):
    import run_experiment as rx
    if not os.path.exists(os.path.join(REPO, 'simulation_assets', 'tejen', 'bench_m2',
                                       'm2_bench_world.sdf')):
        pytest.skip('bench world not generated')
    cfg, path = _cfg(tmp_path, 'tejen/bench_m2/m2_bench_world.sdf')
    with pytest.raises(SystemExit, match='no Imu system'):
        rx.require_imu_world(cfg, path)
    cfg, path = _cfg(tmp_path, 'tejen/bench_m2/m2_bench_world_imu.sdf')
    rx.require_imu_world(cfg, path)
