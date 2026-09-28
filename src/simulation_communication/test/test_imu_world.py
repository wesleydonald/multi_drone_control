"""World guard for rate_source imu (the sim bridge default since 2026-09-28): a world
without the Imu system never publishes a gyro sample and its bridges never fly."""
import glob
import os

import pytest
from simulation_communication.imu_world import imu_report, launch_guard, require_imu_world

REPO = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
ASSETS = os.path.join(REPO, 'simulation_assets')
MOTOR = '<plugin filename="gz-sim-multicopter-motor-model-system" name="m"/>'
IMU_SYS = '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>'
SENSOR = '<sensor name="imu_sensor" type="imu"/>'


def _config_worlds():
    yaml = pytest.importorskip('yaml')
    worlds = set()
    for path in glob.glob(os.path.join(REPO, 'configs', 'experiments', '*.yaml')):
        cfg = yaml.safe_load(open(path)) or {}
        sections = [(cfg.get(k) or {}).get('args') or {} for k in ('launch', 'io_launch')]
        told = [str(a['rate_source']) for a in sections if 'rate_source' in a]
        if 'world' in cfg and not (told and all(v == 'pose' for v in told)):
            worlds.add(cfg['world'])
    if not worlds:
        pytest.skip('no configs/experiments (gitignored) in this checkout')
    return sorted(worlds)


def test_every_imu_experiment_world_publishes_every_gyro():
    for world in _config_worlds():
        path = world if os.path.isabs(world) else os.path.join(ASSETS, world)
        assert require_imu_world(path), world


@pytest.mark.parametrize('world', [
    'tejen/world_m2b_single_attachment.sdf',     # m2b_b0/b1 default, the M2C/M2D template
    'tejen/world_drone_env_detach.sdf',          # c1f6
    'tejen/world_drone_env_detach_c1e.sdf',      # c1e
    'tejen/stage1_torque_test.sdf',              # tools/sim_test/stage1_torque_test.py
    'tejen/bench_m2/m2_bench_world_imu.sdf',     # m2_bench_io_launch default (imu)
])
def test_tejen_default_worlds_have_the_imu_system(world):
    path = os.path.join(ASSETS, world)
    if not os.path.exists(path):
        pytest.skip(f'{world} not generated')
    assert require_imu_world(path)


def test_the_pose_bench_world_is_refused():
    path = os.path.join(ASSETS, 'tejen', 'bench_m2', 'm2_bench_world.sdf')
    if not os.path.exists(path):
        pytest.skip('bench world not generated')
    with pytest.raises(RuntimeError, match='no Imu system'):
        require_imu_world(path)


def _world(tmp_path, body, model):
    (tmp_path / 'x3.sdf').write_text(f'<sdf><model name="x3">{model}</model></sdf>')
    w = tmp_path / 'w.sdf'
    w.write_text(f'<?xml version="1.0"?><sdf><world name="quadcopter">{body}'
                 '<include><uri>x3.sdf</uri><name>x3_0</name></include></world></sdf>')
    return str(w)


def test_system_on_the_world_or_in_the_model_both_count(tmp_path):
    assert imu_report(_world(tmp_path, IMU_SYS, MOTOR + SENSOR))[0]
    assert imu_report(_world(tmp_path, '', MOTOR + SENSOR + IMU_SYS))[0]
    # gz accepts '--' inside comments, expat does not
    assert imu_report(_world(tmp_path, '<!-- a -- b -->' + IMU_SYS, MOTOR + SENSOR))[0]


def test_a_commented_out_system_does_not_count(tmp_path):
    with pytest.raises(RuntimeError, match='no Imu system'):
        require_imu_world(_world(tmp_path, f'<!-- {IMU_SYS} -->', MOTOR + SENSOR))


def test_a_flying_model_without_an_imu_sensor_is_refused(tmp_path):
    with pytest.raises(RuntimeError, match='x3'):
        require_imu_world(_world(tmp_path, IMU_SYS, MOTOR))


def test_launch_guard_checks_only_the_imu_path(tmp_path):
    pytest.importorskip('launch')
    from launch import LaunchContext
    bad = _world(tmp_path, '', MOTOR + SENSOR)
    guard = launch_guard(world_config='world_path')
    ctx = LaunchContext()
    ctx.launch_configurations['world_path'] = bad
    with pytest.raises(RuntimeError, match='no Imu system'):   # default rate_source: imu
        guard.execute(ctx)
    ctx.launch_configurations['rate_source'] = 'pose'
    assert guard.execute(ctx) == []
