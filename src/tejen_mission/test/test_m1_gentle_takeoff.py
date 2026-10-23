"""M1 IRL takeoff adapter checks without constructing ROS nodes or solvers."""
import ast
from pathlib import Path
from types import MethodType, SimpleNamespace as NS
from typing import Optional

import numpy as np
import pytest

from tejen_mission.mission_definitions import pickup_delivery_mission
from tejen_mission.mission_types import MissionPhase as P, TargetSource
from tejen_mission.online_join_planner import RateProfile
from tejen_mission.reference_generators import (
    TargetState, TrajectoryReference, smooth_rate_controlled_manoeuvre, stationary_regulation,
)

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / 'tejen_mission/online_join_planner.py'


def method(name):
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OnlineJoinPlanner')
    fn = next((n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name), None)
    assert fn is not None, f'missing takeoff adapter: {name}'
    scope = dict(np=np, MissionPhase=P, TargetSource=TargetSource, Optional=Optional, RateProfile=RateProfile,
                 TargetState=TargetState, TrajectoryReference=TrajectoryReference,
                 smooth_rate_controlled_manoeuvre=smooth_rate_controlled_manoeuvre,
                 stationary_regulation=stationary_regulation, String=NS)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])), str(SOURCE), 'exec'), scope)
    return scope[name]


def planner(enabled=True, phase=P.TAKEOFF):
    obj = NS(m1_gentle_takeoff_enabled=enabled, phase=phase, is_m2b=False,
             drone_position=np.array([.2, -.3, .06]), drone_velocity=np.zeros(3),
             takeoff_start_position=np.array([.2, -.3, .06]), ground_ring_exit=None,
             c1f2_cpp_authority_enabled=False, min_reference_z=.58,
             takeoff_height=.50, takeoff_speed=.15,
             takeoff_reference_acceleration_limit_mps2=.50, dt=1/30,
             horizon_samples=61, max_reference_speed=2., phase_elapsed=lambda: 0.)
    obj.rate_profile = MethodType(method('rate_profile'), obj)
    obj.current_phase_spec = lambda: pickup_delivery_mission().spec(obj.phase)
    if 'def takeoff_reference_min_z(' in SOURCE.read_text():
        obj.takeoff_reference_min_z = MethodType(method('takeoff_reference_min_z'), obj)
    return obj


@pytest.mark.xfail(reason='multi_drone_control 15 Oct (WP8): WAIT_FOR_TAKEOFF holds the latched ground hold '
                   '(wait_takeoff_hold), not the live pose; his stub has no such method', strict=True)
def test_irl_wait_and_takeoff_follow_measured_rest_not_pickup_floor():
    obj = planner(phase=P.WAIT_FOR_TAKEOFF)
    np.testing.assert_allclose(method('build_reference')(obj).positions[0], obj.drone_position)
    obj.phase = P.TAKEOFF
    for elapsed in (0., .5, 1., 4.):
        obj.phase_elapsed = lambda: elapsed
        actual = method('build_reference')(obj)
        expected, _, _ = smooth_rate_controlled_manoeuvre(
            TargetState(obj.takeoff_start_position, np.zeros(3), np.zeros(3)),
            np.zeros(3), np.array([0., 0., .50]), rate=.15, acceleration_limit=.50,
            elapsed=elapsed, dt=obj.dt, horizon_samples=61, min_reference_z=.06,
            max_reference_speed=2.)
        for field in ('positions', 'velocities', 'accelerations'):
            np.testing.assert_allclose(getattr(actual, field), getattr(expected, field))
    assert obj.min_reference_z == .58  # never mutate the pickup/transfer floor
    obj.phase = P.DESCEND_TO_PICKUP
    assert obj.takeoff_reference_min_z() == .58


def test_default_m1_and_m2_takeoff_floor_behaviour_is_unchanged():
    for phase in (P.TAKEOFF, P.M2_VERTICAL_TAKEOFF):
        obj = planner(enabled=False, phase=phase)
        obj.current_phase_spec = lambda: NS()
        ref = method('build_reference')(obj)
        assert ref.positions[0, 2] == .58


def test_irl_takeoff_mode_restores_normal_costs_without_flight_commands():
    sent = []
    obj = NS(m1_takeoff_mpc_mode_publisher=NS(publish=lambda msg: sent.append(msg.data)))
    publish = method('publish_m1_takeoff_mpc_mode')
    for obj.phase in (P.WAIT_FOR_TAKEOFF, P.TAKEOFF, P.APPROACH_ABOVE_PICKUP,
                      P.DESCEND_TO_PICKUP, P.LIFT_OBJECT, P.LANDING):
        publish(obj)
    assert sent == ['VERTICAL_TAKEOFF', 'VERTICAL_TAKEOFF'] + ['FREE_SWING'] * 4
    obj.m1_takeoff_mpc_mode_publisher = None
    publish(obj)
    assert len(sent) == 6


def test_both_irl_launchers_enable_takeoff_adapter_with_generic_default_off():
    for name in ('m1_irl_ground_ring.launch.py', 'm1_irl_virtual_ring.launch.py'):
        assert "'m1_gentle_takeoff_enabled': True" in (PACKAGE / 'launch' / name).read_text()
    source = SOURCE.read_text()
    assert 'declare_parameter("m1_gentle_takeoff_enabled", False)' in source
    init = source.split('def __init__', 1)[1].split('\n    def ', 1)[0]
    assert 'self.m1_takeoff_mpc_mode_publisher = self.create_publisher(' in init
    tick = source.split('def timer_callback(self)', 1)[1]
    assert tick.index('self.publish_m1_takeoff_mpc_mode()') < tick.index('if not self.inputs_ready():')
    assert tick.index('self.publish_m1_takeoff_mpc_mode()', tick.index('self.update_state_machine()')) < tick.index('self.publish_reference(reference)')
