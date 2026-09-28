"""x0_rate_source (card 2026-09-28_mocap_diff_window v3): the gyro x0 body rate and its launch arg.

The rate logic is a pure function; the launch is evaluated offline (nothing is started).
"""
import importlib.util
import os

import numpy as np
import pytest

from controller_quad_load.x0_rate import X0_RATE_MAX_AGE_S, x0_body_rate

MOCAP_W = np.array([9.0, 9.0, 9.0])


def test_mean_of_the_samples_since_the_last_tick():
    s = np.array([0.1, -0.2, 0.3]) + np.array([0.3, 0.0, 0.1])
    w, held, fb = x0_body_rate(s, 2, None, 10.0, 10.01, MOCAP_W)
    assert not fb and np.allclose(w, [0.2, -0.1, 0.2]) and np.allclose(held, w)


def test_no_new_sample_holds_the_last_mean_up_to_the_age_limit():
    held = np.array([0.1, 0.2, 0.3])
    w, h, fb = x0_body_rate(np.zeros(3), 0, held, 10.0, 10.0 + X0_RATE_MAX_AGE_S - 1e-6, MOCAP_W)
    assert not fb and np.allclose(w, held) and h is held


def test_stale_gyro_falls_back_to_mocap_and_keeps_the_held_mean():
    held = np.array([0.1, 0.2, 0.3])
    w, h, fb = x0_body_rate(np.zeros(3), 0, held, 10.0, 10.0 + X0_RATE_MAX_AGE_S + 1e-3, MOCAP_W)
    assert fb and np.allclose(w, MOCAP_W) and h is held


def test_fresh_samples_with_an_old_stamp_still_fall_back():
    w, h, fb = x0_body_rate(np.array([1.0, 1.0, 1.0]), 1, None, 9.0, 10.0, MOCAP_W)
    assert fb and np.allclose(w, MOCAP_W) and np.allclose(h, [1.0, 1.0, 1.0])


def test_no_gyro_yet_falls_back():
    for stamp in (None, 10.0):
        w, h, fb = x0_body_rate(np.zeros(3), 0, None, stamp, 10.0, MOCAP_W)
        assert fb and np.allclose(w, MOCAP_W) and h is None


def test_stamp_ahead_of_the_node_clock_is_fresh():
    w, _, fb = x0_body_rate(np.array([0.5, 0.0, 0.0]), 1, None, 10.02, 10.0, MOCAP_W)
    assert not fb and np.allclose(w, [0.5, 0.0, 0.0])


# ── three_attach_launch.py: x0_rate_source_drones ──────────────────────────────────────

pytest.importorskip('launch_ros')
from launch import LaunchContext  # noqa: E402
from launch.actions import (DeclareLaunchArgument, GroupAction,  # noqa: E402
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.utilities import perform_substitutions  # noqa: E402
from launch_ros.actions import Node  # noqa: E402
from launch_ros.utilities import evaluate_parameters  # noqa: E402

LAUNCH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch',
                      'three_attach_launch.py')
# the M1 partner demo launch args (configs/experiments/partner_attached_orbit_clock0.yaml)
M1 = {'num_drones': 3, 'reserved_attach': 1, 'start_taut': True, 'load_traj': 'orbit',
      'traj_speed': 0.125, 'traj_radius': 0.5, 'control_mode': 'velocity_after_handover',
      'reconfig_mode': 'ocp', 'partner': True, 'partner_attached': True}


@pytest.fixture(autouse=True)
def _installed():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('simulation_communication')
    except Exception:
        pytest.skip('source install/setup.bash')


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
            pass


def trackers(**args):
    """{controller node name: its x0_rate_source param, None when not set}."""
    spec = importlib.util.spec_from_file_location('three_attach_x0', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations.update({k: str(v).lower() if isinstance(v, bool) else str(v)
                                      for k, v in args.items()})
    out = []
    _collect(mod.generate_launch_description().entities, ctx, out)
    res = {}
    for e in out:
        if (e.node_executable == 'controller'
                and _s(ctx, e._Node__package) == 'controller_quad_load'):
            p = {}
            for d in evaluate_parameters(ctx, e._Node__parameters or []):
                p.update(d)
            res[_s(ctx, e._Node__node_name)] = p.get('x0_rate_source')
    return res


def test_default_no_tracker_gets_the_gyro():
    got = trackers(**M1)
    assert set(got) == {f'controller_{i}' for i in range(4)}
    assert set(got.values()) == {None}


def test_the_arg_reaches_only_the_listed_trackers():
    got = trackers(**M1, x0_rate_source_drones='0,2')
    assert got == {'controller_0': 'imu', 'controller_1': None, 'controller_2': 'imu',
                   'controller_3': None}


def test_b1_config_puts_the_gyro_on_the_carriers_only():
    import yaml
    path = os.path.join(os.path.dirname(LAUNCH), '..', '..', '..', 'configs', 'experiments',
                        'partner_attached_orbit_clock0_x0imu.yaml')
    if not os.path.exists(path):
        pytest.skip('configs/ not in this checkout')
    got = trackers(**yaml.safe_load(open(path))['launch']['args'])
    assert got == {'controller_0': 'imu', 'controller_1': 'imu', 'controller_2': 'imu',
                   'controller_3': None}


def test_only_tethered_drones():
    with pytest.raises(RuntimeError, match='only tethered drones'):
        trackers(**M1, x0_rate_source_drones='3')


def test_real_refuses_it():
    with pytest.raises(RuntimeError, match='real:=true refuses x0_rate_source_drones'):
        trackers(num_drones=3, real=True, thrust_ratio=24.0, x0_rate_source_drones='0,1,2')
