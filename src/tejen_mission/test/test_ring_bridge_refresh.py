"""ring_bridge's commitment refresh (15 Nov, Te-Jen and Codex's closeout): one LoadTrajectory per set of constructor
arguments, so the figure-eight's arc-length table is built once, not at each of the refresh's control points (that
refresh blocked the node's 20 Hz ring pose and carrier states for 0.13 s idle, 0.23-0.33 s under load). The
commitments are bit-identical to those built with a fresh object per call."""
import pytest
import rclpy

import tejen_mission.ring_bridge as rb
from mpc_planner.load_trajectory import LoadTrajectory
from tejen_mission.fake_cooperative_transport_world import RigidBodyState

FIG8 = {'kind': 'fig_8', 'speed': 0.125, 'distance': 1.0, 'radius': 0.75, 'laps': 0, 'hold_s': 0.0,
        'running': True, 'traj_t': 20.0, 'hover_x': 0.0, 'hover_y': 0.0, 'z': 1.2}
PARAMS = ['--ros-args', '-p', 'publish_shared_trajectories:=true', '-p', 'shared_trajectory_duration_s:=30.0',
          '-p', 'shared_trajectory_control_interval_s:=0.5', '-p', 'shared_trajectory_refresh_period_s:=1.0',
          '-p', 'publish_attached_plates:=true', '-p', 'enable_fake_obstacles:=true']


@pytest.fixture
def bridge():
    rclpy.init(args=PARAMS)
    node = rb.RingBridge()
    node._ring = RigidBodyState(x=0.1, y=0.0, z=1.2, yaw=0.0, vx=0.1, vy=0.0, vz=0.0, yaw_rate=0.0)
    node._ring_q = (1.0, 0.0, 0.0, 0.0)
    clock = FrozenClock(node.get_clock().now())
    node.get_clock = lambda: clock
    yield node
    node.destroy_node()
    rclpy.shutdown()


class FrozenClock:
    """The node clock held still, so two builds see the same now (the commitments depend on it)."""

    def __init__(self, now):
        self.at = now

    def now(self):
        return self.at


def commitments(node, state, valid_from):
    node._traj_state = dict(state)
    node._traj_rx_s = node.get_clock().now().nanoseconds * 1e-9
    splines = node.build_shared_obstacle_splines(valid_from) + [node.build_shared_payload_spline(valid_from)]
    return [(s.knots, s.control_points, s.valid_from_s, s.valid_until_s) for s in splines]


def test_one_trajectory_per_refresh_and_the_same_commitments(bridge, monkeypatch):
    built = []

    class Counted(LoadTrajectory):
        def __init__(self, *a, **k):
            built.append(a)
            super().__init__(*a, **k)
    monkeypatch.setattr(rb, 'LoadTrajectory', Counted)
    t0 = bridge.start_time_s + 5.0
    cached = commitments(bridge, FIG8, t0)
    assert len(built) == 1                                       # 252 control points, one table
    bridge._trajectory = lambda s: LoadTrajectory(s['kind'], s['speed'], s.get('distance', 1.0), s['radius'],
                                                  laps=s.get('laps', 1))
    assert commitments(bridge, FIG8, t0) == cached               # bit-identical to a fresh object per point


def test_a_new_trajectory_state_builds_a_new_trajectory(bridge):
    t0 = bridge.start_time_s + 5.0
    commitments(bridge, FIG8, t0)
    first = bridge._traj_obj
    commitments(bridge, {**FIG8, 'traj_t': 21.0}, t0)            # the clock moves: the same trajectory
    assert bridge._traj_obj is first
    circle = {**FIG8, 'kind': 'orbit', 'radius': 0.3, 'laps': 1}
    commitments(bridge, circle, t0)
    assert bridge._traj_obj is not first and bridge._traj_obj.kind == 'orbit'
