"""rejoin_departed (twin only; plan 2026-10-21 code design B, card 2026-10-23_rejoin_departed): a carrier that leaves
holds at a staging point in the ring frame instead of landing, and /fleet/attach <d> flies it back to its own plate on
the newcomer's approach, re-welds its tether (/drone_d/attach) once its rod's end sits on the plate point, and grows
the OCP with the plate point and rod length it left with. LAND or a drop take it out and down alone. Off: departed
carriers as before, and /fleet/attach of one is refused (it used to fold the carrier in at its body)."""
import time
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.approach import CLEARANCE_M  # noqa: E402
from dissipative_planner.dissipative_node import (  # noqa: E402
    DEPARTED_CLEAR_R, DROP_LAND_WAIT_S, POSITION_LIMIT_M, REJOIN_JOINT_S, REJOIN_OUT_EXTRA_S, REJOIN_OUT_MARGIN_M,
    REJOIN_STAGE_OUT_M, REJOIN_TIMEOUT_S, STAGE_LIMIT_MARGIN_M, STAGE_TRACK_M, TAKE_BACK_ACCEL,
    DissipativeController as D, rejoin_weld_gate)
from mpc_planner.geometry import attach_points, rot_z  # noqa: E402
from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402
from utility_objects.position_limit import POS_LIMIT_LAND_MARGIN_M  # noqa: E402

HZ = 10.0
L, PIVOT, Z_RING, Z_ATT = 1.0, -0.04, 1.0, 0.02
BTT = L - PIVOT                          # body to the rod's end
LIM = POSITION_LIMIT_M - STAGE_LIMIT_MARGIN_M - STAGE_TRACK_M
OUT_R = DEPARTED_CLEAR_R + REJOIN_OUT_MARGIN_M     # a staged or rejoining drone's way out
METHODS = ('_resize_and_release', '_detach_plan', '_detach_ocp', '_gap_flies', '_release_magnet', '_stage_departed',
           '_rejoin_limit', '_stage_lim', '_stage_inside', '_stage_hold', '_stage_point', '_rejoin_cap', '_plate_world',
           '_rejoin_busy', '_rejoin_tick', '_rejoin_refused',
           '_start_rejoin', '_rejoin_tip_cb', '_rejoin_joint_cb', '_rejoin_approach_tick', '_rejoin_send_attach',
           '_rejoin_fold', '_rejoin_blend_tick', '_rejoin_retreat', '_rejoin_out', '_do_attach', '_attach_ocp',
           '_drop_departed', '_publish_departed_refs', '_reserved_index', '_weld_refused', '_set_on_load',
           '_capture_weld_rho', '_path_clear_point', '_pos', '_departed_down', '_drop_and_land',
           '_release_all_magnets', '_cancel_unload_quietly', '_fleet_attach_cb', '_fleet_detach_cb', '_drop_due',
           '_drop_tick', '_land_departed_wait_s')


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = warning = error = info


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data if hasattr(m, 'data') else m)


class _Refs:
    def __init__(self, n):
        self._t_nom = [0.86 * 9.81 / n / np.sin(np.radians(60.0))] * n
        self._s_nom, self.balanced_blend, self.blends, self.active = None, None, [], False

    def start_blend(self, s, t, T):
        self.blends.append((list(t), T))
        self.seeds = [np.array(v, float) for v in s]

    def blend_active(self):
        return self.active


def _body(rho):
    """A carrier hanging at the planned 60 deg on its 1 m rod, its pivot 4 cm under the body."""
    u = rho[:2] / np.linalg.norm(rho[:2])
    return np.array([0.0, 0.0, Z_RING]) + rho + np.r_[0.5 * u, L * np.sin(np.radians(60.0)) - PIVOT]


def _node(rejoin=True, kind='hover', hover_xy=(0.0, 0.0), n=4):
    """A 1 m square (plates at 0, 90, 180, 270 deg ccw) hovering at 1 m; the OCP built for 2..4."""
    log = _Log()
    rho = attach_points(n, 0.225, Z_ATT, [90.0 * k for k in range(n)])
    f = SimpleNamespace(
        _rejoin_departed=rejoin, _rejoin={}, _rejoin_tip_t={}, _rejoin_joint={}, _rejoin_joint_t={},
        _rejoin_blend=None, _rejoin_landing=set(), _pos_limit_m=0.0, _unload_s=0.0, n=n, _n_carry0=n, reserved_attach=0,
        n_net=n, slot2drone=list(range(n)), rho=list(rho), cable_len=L, cable_len_i=[L] * n, attach_radius=0.225,
        attach_z=Z_ATT, cable_elev_deg=60.0, pivot_offset=np.array([0.0, 0.0, PIVOT]), psi0=0.0,
        drone_pos=[_body(r) for r in rho], drone_vel=[np.zeros(3) for _ in range(n)], detached=[False] * n,
        load_state=np.r_[hover_xy[0], hover_xy[1], Z_RING, 1.0, 0.0, 0.0, 0.0, np.zeros(6)],
        hover_xy=tuple(hover_xy), traj=LoadTrajectory(kind, 0.2, 1.0, 1.0), traj_t=0.0, _traj_running=False,
        phase='planner', takeoff_seen=True, lift_z0=0.1, lift_progress=0.9, target_z=Z_RING, descending=False,
        _land_to_ground=False, _dropped=None, _drop_on_loss=True, _drop_t=None, _drop_centre=None, _landed=False,
        _unload=None, _min_survivors=2, _drop_gap_deg=200.0, _reconfig_mode='ocp', _reconfig_hold_left=0.0,
        _reconfig_hold_s=1.5, _attach_traj_hold_s=10.0, _attach_moving=True, _hold_mode='timed', _hold_max_s=30.0,
        _attach_t_start_new=0.1, _attach_datum_shift=False, _attach_blend_balanced=True, _attach_cable_len=0.59,
        _start_attached=False, _ocp_attached={}, _weld_time={}, attach_pending=[], attach_pos=[], _approach={},
        _newcomer_out={}, _newcomer_lifecycle=False, _newcomer_fault_land=False, _newcomer_armed={},
        _partner_release=False, _release_state={}, _detach_step_out=0.5, _detach_magnet=False, _magnet_pubs=[],
        _departed_hold={}, _departed_step={}, _departed_next={}, _departed_td={}, _departed_clear={},
        _departed_landing=set(), _departed_wait={}, _land_alone=set(), _departed_land=False, _departed_land_vel=0.0,
        land_vel=0.2, _departed_down_z=0.15, _departed_gi=False, _rest_z={}, _departed_sent=set(), _departed_pubs={},
        _detect_count={}, _detect_ok={}, _detect_refused=set(), dyn=SimpleNamespace(g=9.81, rho=list(rho), m=0.86),
        N=20, dt=0.1, refs=_Refs(n), solver=SimpleNamespace(last_X=np.zeros((3, 3))), _magnet_cmd_pub=_Pub(),
        _dropped_pub=_Pub(), landed_pub=_Pub(), _drop_takes_back=False, _taken_back=False, _partner_contact=False,
        _load_t=time.monotonic(), _drone_t={i: time.monotonic() for i in range(n)})
    f._solvers = {m: None for m in range(2, n + 1)}
    f.detach_pub = [_Pub() for _ in range(n)]
    f._attach_pubs = [_Pub() for _ in range(n)]
    f.get_logger = lambda: log
    f.log = log
    f.now = [100.0]
    f.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(f.now[0] * 1e9)))
    for name in METHODS:
        setattr(f, name, MethodType(getattr(D, name), f))
    f._prime_solver = lambda: None
    f._log_resize_inputs = lambda *a: None
    f.published = {}
    f._publish_ref = lambda d, nodes: f.published.setdefault(d, []).append(
        (np.array(nodes[0][0], float), np.array(nodes[0][1], float)))
    f.resizes = []

    def resize(m, ids, rho=None, cable_lengths=None):
        f.resizes.append((m, list(ids), [np.array(r, float) for r in rho], list(cable_lengths or [])))
        f.n, f.slot2drone, f.rho = m, list(ids), [np.array(r, float) for r in rho]
        if cable_lengths is not None:
            f.cable_len_i = list(cable_lengths)
        f.dyn = SimpleNamespace(g=9.81, rho=list(f.rho), m=0.86)
        f._s_nom = [np.array([-r[0], -r[1], -0.39]) / np.linalg.norm([r[0], r[1], 0.39]) for r in f.rho]
        f.refs = _Refs(m)
        return True
    f.resize_fleet = resize
    return f


def _tip_msg(p, stamp):
    sec = int(stamp)
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=float(p[0]), y=float(p[1]), z=float(p[2]))),
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int((stamp - sec) * 1e9))))


def _see_tip(f, d, stamp, off=(0.0, 0.0, 0.0)):
    """d's rod hanging straight under its body (the twin's rod-end marker), offset by off."""
    f._load_t = time.monotonic()
    f._drone_t[d] = time.monotonic()
    f._rejoin_tip_cb(d, _tip_msg(f.drone_pos[d] - [0.0, 0.0, BTT] + np.asarray(off, float), stamp))


def _fly(f, d, ticks, off=(0.0, 0.0, 0.0), per_tick=6, track=True, joint=True):
    """Planner ticks with d tracking its reference perfectly; its rod's end seen at 60 Hz (never bit-identical twice);
    the twin's joint reports attached after each /drone_d/attach (joint=True). The reconfiguration hold counts down as
    _plan_inner does."""
    for _ in range(ticks):
        sent = len(f._attach_pubs[d].msgs)
        if f._rejoin or f._rejoin_blend is not None:
            f._rejoin_tick()
        f._publish_departed_refs()
        if track and f.published.get(d):
            f.drone_pos[d] = f.published[d][-1][0].copy()
        f.now[0] += 1.0 / HZ
        for k in range(per_tick):
            _see_tip(f, d, f.now[0] + k / (per_tick * HZ), np.asarray(off, float) + [0.0, 0.0, 1e-7 * (k + 1)])
        if joint and len(f._attach_pubs[d].msgs) > sent:
            f._rejoin_joint_cb(d, SimpleNamespace(data='attached'))
        if f._reconfig_hold_left > 0.0:
            f._reconfig_hold_left -= 1.0 / HZ


def _fly_until(f, d, done, max_ticks=600, **kw):
    """_fly one tick at a time until done(f); False if it never is."""
    for _ in range(max_ticks):
        _fly(f, d, 1, **kw)
        if done(f):
            return True
    return False


def _leave(f, d=1):
    """d is released and the fleet resized (an announced detach; an unannounced one resizes the same way)."""
    f.detached[d] = False
    f._detach_ocp(d)
    assert f.detached[d] and d not in f.slot2drone


def _staged(d=1, **kw):
    f = _node(**kw)
    _leave(f, d)
    _fly(f, d, 60)                      # stepped out to its staging point
    return f


# ── staging ───────────────────────────────────────────────────────────────────────────────────────────────────────

def test_a_leaver_keeps_its_plate_and_rod_and_stages_on_its_bearing_instead_of_landing():
    f = _node()
    rho1, body1 = f.rho[1].copy(), f.drone_pos[1].copy()
    f.cable_len_i[1] = 0.987                                     # its measured length at the hand-over
    _leave(f, 1)
    rj = f._rejoin[1]
    assert np.allclose(rj['rho'], rho1) and rj['len'] == 0.987 and rj['btt'] == pytest.approx(0.987 - PIVOT)
    assert rj['mode'] == 'stage' and 1 not in f._departed_step and 1 not in f._departed_next   # no world step-out
    assert np.allclose(f._departed_hold[1], body1)
    r_stage = 0.225 + 0.987 * np.cos(np.radians(60.0)) + REJOIN_STAGE_OUT_M
    _fly(f, 1, 80)
    q = f.published[1][-1][0]
    assert np.allclose(q, [0.0, r_stage, body1[2]], atol=0.01)               # plate 1's bearing (+y), its height
    assert np.allclose(f._departed_hold[1], q) and 1 not in f._departed_landing
    assert any('stages' in m and 'plate' in m for m in f.log.lines)
    # bounded: the reference's speed never above the step's 0.15 m/s, its acceleration 0.25 m/s^2
    v = np.array([p[1] for p in f.published[1]])
    assert np.max(np.linalg.norm(v, axis=1)) <= 0.15 + 1e-6
    assert np.max(np.linalg.norm(np.diff(v, axis=0), axis=1)) <= 0.25 / HZ + 1e-6


def test_a_release_kick_does_not_carry_the_staging_reference_up():
    """A loaded release kicks the leaver upward before it is seen: its reference starts at its measured position moving
    with the ring frame, not at that velocity (+0.6 m/s braked at 0.25 m/s^2 would carry it 0.7 m up)."""
    f = _node(kind='fig_8', hover_xy=(0.0, -0.5))
    f.traj_t, f._traj_running = 7.0, True
    f.drone_vel[1] = np.array([0.0, 0.1, 0.6])
    z0 = f.drone_pos[1][2]
    _leave(f, 1)
    assert np.allclose(f._rejoin[1]['v'], np.r_[f.traj.offset_at(7.0)[2:4], 0.0])
    _fly(f, 1, 40, track=False)
    assert max(p[0][2] for p in f.published[1]) <= z0 + 1e-6


def test_switch_off_keeps_the_world_fixed_step_out_and_nothing_is_kept():
    f = _node(rejoin=False)
    _leave(f, 1)
    assert f._rejoin == {} and 1 in f._departed_step
    assert np.allclose(f._departed_step[1][:2], f.drone_pos[1][:2] + 0.5 * np.array([0.0, 1.0]), atol=1e-6)


def test_the_staging_point_moves_with_the_planned_ring_and_turns_with_its_yaw():
    f = _node(kind='fig_8', hover_xy=(0.0, -0.5))                # (inside the position limit all the way round)
    _leave(f, 1)                                                  # plate 1, +y
    rj = f._rejoin[1]
    f.traj_t, f._traj_running = 7.0, True
    dx, dy, vx, vy = f.traj.offset_at(7.0)
    goal, v = f._stage_point(rj)
    assert np.allclose(goal, [dx, -0.5 + dy + rj['r_stage'], rj['z']]) and np.allclose(v[:2], [vx, vy])
    assert np.hypot(vx, vy) > 0.1
    f._traj_running = False                                      # a held clock: a still frame
    assert np.allclose(f._stage_point(rj)[1], 0.0)
    g = _node(kind='line_spin', hover_xy=(-0.5, 0.0))
    _leave(g, 0)                                                  # plate 0, +x
    rj = g._rejoin[0]
    g.traj_t, g._traj_running = 9.0, True
    yaw, rate = g.traj.yaw_at(9.0)
    assert yaw > 0.5 and rate > 0.1
    goal, v = g._stage_point(rj)
    dx, _, vx, _ = g.traj.offset_at(9.0)
    assert np.allclose(goal[:2] - [-0.5 + dx, 0.0], rj['r_stage'] * np.array([np.cos(yaw), np.sin(yaw)]))
    assert np.allclose(v[:2] - [vx, 0.0], rj['r_stage'] * rate * np.array([-np.sin(yaw), np.cos(yaw)]))


def test_the_staging_point_stays_inside_the_position_limit():
    f = _node(hover_xy=(1.0, 0.0))                               # the ring at the figure-8's +x end
    _leave(f, 0)
    goal, _ = f._stage_point(f._rejoin[0])
    assert goal[0] == pytest.approx(LIM) and abs(goal[1]) < 1e-9  # pos_limit_m off: Wesley's 1.75 m
    f.traj, f.traj_t, f._traj_running = LoadTrajectory('line_x', 0.2, 1.0, 1.0), 2.0, True
    goal, v = f._stage_point(f._rejoin[0])
    assert goal[0] == pytest.approx(LIM) and v[0] == 0.0         # clamped: no velocity into the limit
    f._pos_limit_m = 1.5                                          # the typed limit when it is set
    assert f._stage_point(f._rejoin[0])[0][0] == pytest.approx(1.5 - STAGE_LIMIT_MARGIN_M - STAGE_TRACK_M)


def test_staged_drones_are_flown_by_the_rejoin_until_land_then_step_out_and_land_alone():
    f = _staged(1)
    n_before = len(f.published[1])
    f._publish_departed_refs()
    assert len(f.published[1]) == n_before                       # not twice in a tick
    q = f._departed_hold[1].copy()
    f._land_to_ground = True
    f._rejoin_tick()
    assert 1 not in f._rejoin and 1 in f._land_alone and 1 not in f._departed_next   # no climb: not over the ring
    assert np.allclose(f._departed_step[1], [0.0, OUT_R, q[2]])                          # out past the clear radius
    assert any('staged drone 2 (/drone_1) leaves (LAND)' in m for m in f.log.lines)
    assert f._land_departed_wait_s() == pytest.approx(15.0 + REJOIN_OUT_EXTRA_S)        # the landed wait grows (M1)
    for _ in range(200):
        f._publish_departed_refs()
    assert 1 in f._departed_landing and f._departed_hold[1][2] < q[2]                  # out first, then down


# ── the latent bug, and the ATTACH path ───────────────────────────────────────────────────────────────────────────

def test_switch_off_an_attach_of_a_departed_carrier_is_refused_not_folded_in_at_its_body():
    f = _node(rejoin=False)
    _leave(f, 1)
    f.resizes.clear()
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert f.resizes == [] and f.detached[1] and 1 not in f.slot2drone
    assert any('refused: rejoin_departed is off' in m for m in f.log.lines)


def test_the_rejoin_flies_back_welds_on_the_plate_and_grows_the_ocp_with_what_it_left_with():
    f = _node()
    f.cable_len_i[1] = 0.987                                     # its measured rod at the hand-over (typed 1.0)
    _leave(f, 1)
    _fly(f, 1, 60)
    f.cable_len_i = [L] * 3
    f._detect_refused.add(1)
    f._detect_count[1], f._detect_ok[1] = 3, 0.2
    left = f._rejoin[1]
    rho1, len1 = left['rho'].copy(), left['len']
    f.resizes.clear()
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert f._rejoin[1]['mode'] == 'approach' and f._reconfig_hold_left == pytest.approx(1.0)  # renewed each tick
    assert any('REJOIN START drone 2 (/drone_1)' in m for m in f.log.lines)
    assert _fly_until(f, 1, lambda f: f._rejoin[1]['mode'] == 'weld', 200)
    assert len(f._attach_pubs[1].msgs) == 1 and f.resizes == []      # /drone_1/attach; no fold-in before the joint
    assert any('REJOIN ON PLATE drone 2 (/drone_1)' in m for m in f.log.lines)
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 2)
    assert len(f._attach_pubs[1].msgs) == 1                                  # one /drone_1/attach
    assert 1 not in f._departed_hold and not f.detached[1]
    m, ids, rho_all, lens = f.resizes[-1]
    assert m == 4 and ids[-1] == 1
    assert np.allclose(rho_all[-1], rho1) and len1 == 0.987            # the plate it left, not its body
    assert lens[-1] == pytest.approx(0.987, abs=1e-6)                   # the rod it left with, not the typed 1.0
    assert 1 not in f._detect_refused and 1 not in f._detect_count and 1 not in f._detect_ok
    assert f._ocp_attached[1] and 1 in f._weld_time
    assert f._reconfig_hold_left == pytest.approx(10.0 - 1.0 / HZ)         # the path paused through the blend
    assert f.refs.blends[-1][1] == 10.0 and f.refs.blends[-1][0][-1] == pytest.approx(0.1)   # from 0.1 N, over 10 s
    assert any('REJOIN WELD drone 2 (/drone_1)' in m for m in f.log.lines)
    assert any('] ATTACH drone 1 (OCP resize, rejoin)' in m for m in f.log.lines)   # the runner's FOLD_IN


def test_the_approach_puts_the_rods_end_on_the_plate_point():
    f = _staged(1)
    plate = f._plate_world(f._rejoin[1])
    f._fleet_attach_cb(SimpleNamespace(data=1))
    prof = f._rejoin[1]['prof']
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 200)
    assert np.allclose(f.drone_pos[1], plate + [0.0, 0.0, BTT], atol=1e-3)
    assert prof.phase == 'descend'
    z = [p[0][2] for p in f.published[1]]
    assert max(z) == pytest.approx(plate[2] + BTT + CLEARANCE_M, abs=0.06)  # transit at the approach's clearance


def test_no_weld_while_the_rods_end_is_off_the_plate_then_it_backs_out_at_the_timeout():
    f = _staged(1)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    _fly(f, 1, int(REJOIN_TIMEOUT_S * HZ) - 5, off=(0.0, 0.025, 0.0))   # the end 2.5 cm off: no weld
    assert f._rejoin[1]['mode'] == 'approach' and f._attach_pubs[1].msgs == []
    assert f._reconfig_hold_left > 0.0                                   # held all the while
    rj = f._rejoin[1]
    assert _fly_until(f, 1, lambda f: rj['mode'] == 'retreat', 10, off=(0.0, 0.025, 0.0))
    assert f._reconfig_hold_left > 0.0 and f._attach_pubs[1].msgs == []    # held while it climbs off the plate
    assert any('REJOIN ABORT drone 2 (/drone_1)' in m and '2.5 cm' in m for m in f.log.lines)
    plate_z = f._plate_world(rj)[2]
    assert _fly_until(f, 1, lambda f: rj['mode'] == 'stage', 40)
    assert f.drone_pos[1][2] >= plate_z + BTT + CLEARANCE_M - 0.03         # straight up first
    assert f._reconfig_hold_left > 0.0
    _fly(f, 1, 11)
    assert f._reconfig_hold_left <= 0.0                                   # then the path resumes
    _fly(f, 1, 120)
    assert np.allclose(f.drone_pos[1][:2], [0.0, rj['r_stage']], atol=0.02)   # back on its staging point
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert f._rejoin[1]['mode'] == 'approach'                            # and it may try again


def test_the_weld_waits_for_a_running_blend():
    f = _staged(1)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    f.refs.active = True
    _fly(f, 1, 150)
    assert 1 in f._rejoin and f._attach_pubs[1].msgs == []
    f.refs.active = False
    _fly(f, 1, 2)                                                        # the attach, then the joint's answer
    assert 1 not in f._rejoin and len(f._attach_pubs[1].msgs) == 1


@pytest.mark.parametrize('why, setup', [
    ('not up yet', lambda f: setattr(f, 'lift_progress', 0.5)),
    ('LAND', lambda f: setattr(f, '_land_to_ground', True)),
    ('descent', lambda f: setattr(f, 'descending', True)),
    ('dropped', lambda f: setattr(f, '_dropped', 'test')),
    ('unloading', lambda f: setattr(f, '_unload', {'d': 2})),
    ('no OCP', lambda f: f._solvers.pop(4)),
    ('not seen', lambda f: f._rejoin_tip_t.clear()),
    ('path is running', lambda f: setattr(f, '_traj_running', True)),
])
def test_a_rejoin_is_refused_when_it_cannot_fly(why, setup):
    f = _staged(1)
    setup(f)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert f._rejoin[1]['mode'] == 'stage' and f._reconfig_hold_left <= 0.0
    assert any('REJOIN REFUSED drone 2 (/drone_1)' in m and why in m for m in f.log.lines)


def test_one_rejoin_at_a_time_and_none_for_a_carrier_on_the_ring():
    f = _node()
    _leave(f, 1)
    _leave(f, 3)
    _fly(f, 1, 60)
    _see_tip(f, 3, f.now[0])
    f._fleet_attach_cb(SimpleNamespace(data=1))
    f._fleet_attach_cb(SimpleNamespace(data=3))
    assert f._rejoin[1]['mode'] == 'approach' and f._rejoin[3]['mode'] == 'stage'
    assert any('REJOIN REFUSED drone 4 (/drone_3)' in m and 'one at a time' in m for m in f.log.lines)
    n = len(f.resizes)
    f._fleet_attach_cb(SimpleNamespace(data=0))                         # on the ring: nothing happens
    assert len(f.resizes) == n and 0 not in f._rejoin


# ── LAND and a drop during a rejoin ───────────────────────────────────────────────────────────────────────────────

def _over_the_plate():
    f = _staged(1)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    _fly(f, 1, 120, off=(0.03, 0.0, 0.0))                                # over its plate, no weld
    assert f._rejoin[1]['mode'] == 'approach' and f._rejoin[1]['phase'] == 'descend'
    return f


def test_land_during_a_rejoin_climbs_out_along_its_bearing_and_lands_alone():
    f = _over_the_plate()
    plate = f._plate_world(f._rejoin[1])
    r_stage = f._rejoin[1]['r_stage']
    f._land_to_ground = True
    f._rejoin_tick()
    assert 1 not in f._rejoin and 1 in f._land_alone
    up, out = f._departed_step[1], f._departed_next[1]
    z_up = plate[2] + BTT + CLEARANCE_M
    assert up[2] == pytest.approx(z_up) and np.allclose(up[:2], f._departed_hold[1][:2])
    assert np.allclose(out, [0.0, max(r_stage, OUT_R), z_up])
    assert any('REJOIN ABORT drone 2 (/drone_1) (LAND)' in m for m in f.log.lines)
    for _ in range(200):
        f._publish_departed_refs()
    assert 1 in f._departed_landing and f._departed_hold[1][2] < z_up      # out first, then down


def test_a_drop_during_a_rejoin_takes_it_up_and_out_before_it_lands():
    f = _over_the_plate()
    f._drop_and_land('test')
    assert 1 not in f._rejoin and 1 in f._land_alone and 1 in f._departed_next
    assert np.linalg.norm(f._departed_next[1][:2]) == pytest.approx(OUT_R)
    assert any('REJOIN ABORT drone 2 (/drone_1) (drop)' in m for m in f.log.lines)


def test_a_staged_drone_steps_out_to_the_clear_radius_after_a_drop_then_lands():
    """Its staging radius (1.025 m on the 1 m square) is inside the clear radius (1.2 m) that keeps a freed drone's rod
    off the fallen ring (R0567)."""
    f = _staged(1)
    q = f._departed_hold[1].copy()
    assert np.hypot(*q[:2]) < DEPARTED_CLEAR_R - 0.1
    f._drop_and_land('test')
    assert 1 in f._land_alone and np.allclose(f._departed_step[1], [0.0, OUT_R, q[2]])
    for _ in range(200):
        f._publish_departed_refs()
    assert 1 in f._departed_landing and np.hypot(*f._departed_hold[1][:2]) == pytest.approx(OUT_R, abs=1e-6)


# ── the weld gate ─────────────────────────────────────────────────────────────────────────────────────────────────

def _samples(offsets, hz=60.0, t0=10.0):
    return [(0.0, t0 + k / hz, np.asarray(e, float)) for k, e in enumerate(offsets)]


def test_weld_gate_needs_0p3_s_within_1p5_cm_and_slow():
    still = [(0.004, 0.0, -0.003)] * 19                                   # 0.3 s at 60 Hz, 5 mm off
    ok, off, speed = rejoin_weld_gate(_samples(still))
    assert ok and off == pytest.approx(0.005) and speed == pytest.approx(0.0)
    assert not rejoin_weld_gate(_samples(still[:18]))[0]                 # 0.283 s: not yet
    gap = _samples(still) + [(0.0, 10.0 + 18 / 60.0 + 0.06, np.zeros(3))]
    assert not rejoin_weld_gate(gap)[0]                                  # a 60 ms gap restarts the run
    off_plate = _samples([(0.0, 0.0, 0.0)] * 10 + [(0.02, 0.0, 0.0)] + [(0.0, 0.0, 0.0)] * 15)
    assert not rejoin_weld_gate(off_plate)[0]                            # 2 cm once: 0.23 s since then
    passing = _samples([(0.0, 0.0, 0.012 - 0.0008 * k) for k in range(30)])   # descending through at 5 cm/s
    ok, _, speed = rejoin_weld_gate(passing)
    assert not ok and speed == pytest.approx(0.048, abs=1e-3)
    sparse = _samples(still[:7], hz=20.0)                                # 0.3 s but 7 samples
    assert not rejoin_weld_gate(sparse)[0]
    assert rejoin_weld_gate([]) == (False, None, None)


def test_rod_end_samples_count_only_while_rejoining_and_believable():
    f = _staged(1)
    _see_tip(f, 1, 1.0)
    assert len(f._rejoin[1]['tips']) == 0 and 1 in f._rejoin_tip_t          # seen, not evidence
    f._fleet_attach_cb(SimpleNamespace(data=1))
    _see_tip(f, 1, 2.0)
    assert len(f._rejoin[1]['tips']) == 1
    f._rejoin_tip_cb(1, _tip_msg(f.drone_pos[1] - [0.0, 0.0, 0.8], 2.1))  # 24 cm short of the rod: not its end
    f._load_t = time.monotonic() - 1.0
    f._rejoin_tip_cb(1, _tip_msg(f.drone_pos[1] - [0.0, 0.0, BTT], 2.2))  # a stale ring pose: no evidence
    assert len(f._rejoin[1]['tips']) == 1


def test_the_joint_state_is_logged_on_change_only():
    f = _node()
    for s in ('attached', 'detached', 'detached', 'attached'):
        f._rejoin_joint_cb(1, SimpleNamespace(data=s))
    assert sum('tether joint of drone 2' in m for m in f.log.lines) == 3


# ── the critic's findings (22 Oct) ────────────────────────────────────────────────────────────────────────────────

def _on_plate(f, d=1, **kw):
    """d staged, ATTACHed and flown until its /drone_d/attach is out (mode 'weld')."""
    f._fleet_attach_cb(SimpleNamespace(data=d))
    assert _fly_until(f, d, lambda f: f._rejoin[d]['mode'] == 'weld', 200, **kw)
    return f._rejoin[d]


def test_no_fold_in_until_the_joint_reports_attached_else_it_lets_go_and_backs_out():
    """Critic B2: an attach the twin's joint never confirms must not grow the OCP (the T alone would hold the ring
    while the OCP flew the square)."""
    f = _staged(1)
    f.resizes.clear()
    rj = _on_plate(f, joint=False)
    _fly(f, 1, int(REJOIN_JOINT_S * HZ), joint=False)
    assert rj['mode'] == 'weld' and f.resizes == []
    _fly(f, 1, 2, joint=False)
    assert rj['mode'] == 'retreat' and f.resizes == [] and len(f.detach_pub[1].msgs) == 2   # the release's, its own
    assert any('REJOIN ABORT drone 2 (/drone_1)' in m and 'did not report attached' in m for m in f.log.lines)
    assert f.detached[1] and 1 in f._departed_hold and f._reconfig_hold_left > 0.0


def test_the_attach_goes_out_only_once_the_n_plus_1_ocp_is_there():
    """Critic, ordering: an attach and a detach go out on two bridges with no order between them, so the attach is sent
    only when the fold-in can follow."""
    f = _staged(1)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    rj = f._rejoin[1]
    assert _fly_until(f, 1, lambda f: rj['phase'] == 'descend', 200)
    f._solvers.pop(4)
    assert _fly_until(f, 1, lambda f: rj['mode'] == 'retreat', 60)
    assert f._attach_pubs[1].msgs == [] and len(f.detach_pub[1].msgs) == 1           # the release's only
    assert any('REJOIN ABORT drone 2 (/drone_1) (no OCP built for n=4)' in m for m in f.log.lines)


def test_a_late_weld_with_no_rejoin_waiting_is_let_go():
    f = _staged(1)
    f._rejoin_joint_cb(1, SimpleNamespace(data='attached'))
    assert len(f.detach_pub[1].msgs) == 2                               # the release's, then this one
    g = _node()
    g._rejoin_joint_cb(1, SimpleNamespace(data='attached'))            # on the ring: its joint, as spawned
    assert g.detach_pub[1].msgs == []


def test_land_while_the_attach_is_out_lets_the_joint_go_before_it_climbs():
    f = _staged(1)
    _on_plate(f, joint=False)
    f._land_to_ground = True
    f._rejoin_tick()
    assert 1 not in f._rejoin and len(f.detach_pub[1].msgs) == 2 and 1 in f._land_alone


def test_the_rejoiner_counts_as_absent_for_the_drop_rule_and_no_detach_until_its_blend_ends():
    """Critic M2: through its 10 s blend the rejoiner carries ~0.1 N. A loss then is judged on the others; an announced
    DETACH waits for the rejoin and the blend."""
    f = _staged(1)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    f._fleet_detach_cb(SimpleNamespace(data=0))
    assert 0 in f.slot2drone and any('detach 0 refused: drone 2 (/drone_1) is rejoining' in m for m in f.log.lines)
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 200)
    f.refs.active = True                                                 # the fold-in blend runs
    assert f._rejoin_blend['d'] == 1 and f.slot2drone == [0, 2, 3, 1]
    # (ccw: drone 0 at 0 deg, the rejoiner 1 at 90, 2 at 180, 3 at 270) 3 lost: 0 and 2 are an opposite pair
    assert f._drop_due(3) is None
    assert 'gap' in f._drop_due(0)                                       # 2 and 3 adjacent: the rejoiner holds nothing
    f._rejoin_blend = None
    assert f._drop_due(0) is None                                        # after the blend: the T on 1, 2, 3 flies
    f._rejoin_blend = {'d': 1, 'rj': {}, 'max_off': 0.0}
    f._fleet_detach_cb(SimpleNamespace(data=3))
    assert 3 in f.slot2drone and any('is folding back in (its blend)' in m for m in f.log.lines)
    f.refs.active = False
    _fly(f, 1, 1)
    assert f._rejoin_blend is None and any('REJOIN BLEND OVER drone 2 (/drone_1)' in m for m in f.log.lines)


def test_the_rod_ends_distance_from_its_plate_is_kept_through_the_blend():
    f = _staged(1)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 200)
    f.refs.active = True
    _fly(f, 1, 5, off=(0.0, 0.0, 0.012))
    f.refs.active = False
    _fly(f, 1, 1)
    line = next(m for m in f.log.lines if 'REJOIN BLEND OVER' in m)
    assert 'at most 1.2 cm from its plate point' in line


def test_a_drop_with_a_rejoiner_coming_down_waits_longer_for_landed():
    """Critic M1: up, out and down from over its plate takes ~14 s more than a departed drone's landing."""
    f = _over_the_plate()
    assert f._land_departed_wait_s() == pytest.approx(15.0)
    f._drop_and_land('test')
    f._drop_t = f.now[0]
    f.now[0] += DROP_LAND_WAIT_S + 1.0
    f._drop_tick()
    assert not f._landed                                                 # still in the air: no /fleet/landed
    f.now[0] += REJOIN_OUT_EXTRA_S
    f._drop_tick()
    assert f._landed and f.landed_pub.msgs == [True]


def test_land_mid_rejoin_near_the_limit_lands_inside_it_and_clear_of_the_ring():
    """The ring at +x near the limit (a figure-8's end), its +x carrier rejoining: the out point stays inside the
    limit's landing margin (1.75 m while pos_limit_m is off) and at least the clear radius from the ring."""
    f = _node(hover_xy=(1.0, 0.0))
    _leave(f, 0)
    _fly(f, 0, 60)
    f._fleet_attach_cb(SimpleNamespace(data=0))
    _fly(f, 0, 120, off=(0.03, 0.0, 0.0))
    assert f._rejoin[0]['phase'] == 'descend'
    f._land_to_ground = True
    f._rejoin_tick()
    out = f._departed_next[0]
    bound = POSITION_LIMIT_M - POS_LIMIT_LAND_MARGIN_M
    assert max(abs(out[0]), abs(out[1])) <= bound + 1e-9 and out[0] == pytest.approx(bound)
    assert np.hypot(out[0] - 1.0, out[1]) >= DEPARTED_CLEAR_R - 1e-6
    g = _node(hover_xy=(1.0, 0.0))
    g._pos_limit_m = 1.5                                                  # a typed limit wins
    _leave(g, 0)
    _fly(g, 0, 60)
    g._land_to_ground = True
    g._rejoin_tick()
    assert max(abs(v) for v in g._departed_step[0][:2]) <= 1.5 - POS_LIMIT_LAND_MARGIN_M + 1e-9


def test_the_tip_loop_trims_a_rod_end_that_hangs_too_high():
    """Critic B1: the departed hold sat 0.7-1.6 cm above its reference 3 s after the release (R2312-R2316), against a
    1.5 cm weld band. The measured end, 2 cm above its plate point here, is brought down by the tip loop and welds."""
    f = _staged(1)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    rj = f._rejoin[1]
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 400, off=(0.0, 0.0, 0.02))
    assert rj['prof'].tip.c < -0.005                                     # the body came down
    assert any('end hold' in m for m in f.log.lines)


def test_the_fold_in_seed_turns_with_the_planned_yaw():
    """Review: a rejoin into a path held mid-turn seeds its rod in the turned nominal frame."""
    f = _node(kind='line_spin', hover_xy=(-0.5, 0.0))
    _leave(f, 1)
    _fly(f, 1, 60)
    f.traj_t = 9.0                                                        # held there (the clock does not run)
    yaw = f.traj.yaw_at(9.0)[0]
    assert yaw > 0.5
    f._fleet_attach_cb(SimpleNamespace(data=1))
    rj = f._rejoin[1]
    assert _fly_until(f, 1, lambda f: rj['phase'] == 'descend' and abs(
        f.drone_pos[1][2] - f._plate_world(rj)[2] - BTT) < 1e-3, 300)
    f.drone_pos[1] = f._plate_world(rj) + [0.01, 0.0, BTT]               # the body 1 cm off: its rod not vertical
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 40, track=False, off=(-0.01, 0.0, 0.0))
    rim = np.asarray(f.load_state[0:3]) + f.rho[-1]
    s_new = (rim - f.drone_pos[1]) / np.linalg.norm(rim - f.drone_pos[1])
    assert np.allclose(f.refs.seeds[-1], rot_z(-yaw) @ s_new) and not np.allclose(f.refs.seeds[-1], s_new)


def test_the_fold_in_rod_runs_from_the_plate_point_to_the_drones_pivot():
    """Critic, the weld rule: the joint welds the end where it is, so the rod the ring now hangs on is |pivot - plate
    point|, up to the weld band off the rod it left with."""
    f = _staged(1)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    rj = f._rejoin[1]
    assert _fly_until(f, 1, lambda f: rj['phase'] == 'descend' and abs(
        f.drone_pos[1][2] - f._plate_world(rj)[2] - BTT) < 1e-3, 300)
    f.drone_pos[1] = f._plate_world(rj) + [0.0, 0.0, BTT - 0.01]         # the body (and its end) 1 cm low
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 40, track=False)
    assert f.resizes[-1][3][-1] == pytest.approx(L - 0.01, abs=1e-6)


def test_the_rejoin_blend_is_its_own_10_s_on_the_detach_graph():
    """E1b (R2466/R2467): the detach graph passes no attach_traj_hold_s (0 at the node), so the fold-in blended over
    the 1.5 s reconfiguration hold; the rejoin now blends over rejoin_blend_s (10 s), whatever attach_traj_hold_s is."""
    f = _node()
    f._attach_traj_hold_s = 0.0
    _leave(f, 1)
    _fly(f, 1, 60)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 400)
    assert f.refs.blends[-1][1] == 10.0


# ── E2-E4's findings (23 Oct) ─────────────────────────────────────────────────────────────────────────────────────

def _t_blend(f, rho, length, push, ticks=100):
    """The twin's T through the rider's 10 s blend: its rigid rod puts plate 1 wherever its body is (the pair on the
    axis through the ring's centre holds no pitch), the plan starts from its body (node 0 pinned) and swings its rod
    out 30 deg about the measured plate, and its body goes where the published reference goes plus `push` up per tick
    for 1.5 s (the fold-in's thrust step). Returns the published node-0 heights and plate 1's lift over a level ring."""
    plate0 = np.asarray(f.load_state[0:3], float) + np.asarray(rho, float)
    u = plate0[:2] / np.linalg.norm(plate0[:2])
    w = np.radians(30.0) / ticks
    body = np.asarray(f.drone_pos[1], float).copy()
    zs, lifts = [], []
    for j in range(ticks):
        h = float(np.hypot(*(body[:2] - plate0[:2])))
        plate = np.r_[plate0[:2], body[2] + PIVOT - np.sqrt(length ** 2 - h ** 2)]
        lifts.append(plate[2] - plate0[2])
        el0 = np.arctan2(body[2] + PIVOT - plate[2], h)
        nodes = []
        for k in range(f.N + 1):
            el = el0 - w * k
            p = np.r_[plate[:2] + length * np.cos(el) * u, plate[2] + length * np.sin(el) - PIVOT]
            v = np.r_[length * np.sin(el) * u, -length * np.cos(el)] * (w / f.dt)
            nodes.append((p, v, np.zeros(3), np.zeros(3)))
        pub = f._rejoin_cap(1, nodes)
        zs.append(float(pub[0][0][2]))
        body = np.asarray(pub[1][0], float) + [0.0, 0.0, push if j < 15 else 0.0]
    return np.array(zs), np.array(lifts), body


def test_the_rejoiners_reference_never_rises_through_its_blend_onto_a_t():
    """E2 (R2584): the T's rider (plates 0, 90, 180; the pair on 0 and 180) folds back in. Its plan starts from its
    measured body, and only the rider holds the ring's pitch about the pair's axis, so the fold-in's thrust step lifted
    it and plate 1 with it while the plan followed (+4.6 cm, the ring 10.6 deg). Through the blend its reference now
    never rises from its weld height and no node sits above where a level ring puts it on its rod."""
    f = _node(n=3)
    _leave(f, 1)
    assert f.slot2drone == [0, 2]                                        # the opposite pair flies on
    _fly(f, 1, 60)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 200)
    f.refs.active = True
    b = f._rejoin_blend
    z_weld, rho, length = f.drone_pos[1][2], b['rj']['rho'].copy(), b['len']
    assert b['d'] == 1 and b['cap'] == pytest.approx(z_weld) and length == pytest.approx(L, abs=1e-6)
    zs, lifts, body = _t_blend(f, rho, length, push=0.006)
    assert np.all(np.diff(zs) <= 1e-12) and zs.max() <= z_weld + 1e-9      # never rises, never above the weld
    assert lifts.max() <= 0.006 + 1e-6                                     # plate 1 held level (one tick's push)
    assert np.hypot(*(body[:2] - rho[:2])) == pytest.approx(L * np.cos(np.radians(60.0)), abs=0.01)  # swung out
    nodes = [(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))]
    assert f._rejoin_cap(0, nodes) is nodes                                # the others' references untouched
    f.refs.active = False
    _fly(f, 1, 1)
    assert f._rejoin_blend is None and f._rejoin_cap(1, nodes) is nodes    # and its own once the blend is over
    g = _node(n=3)                                                         # without the cap: R2584's rise
    _leave(g, 1)
    _fly(g, 1, 60)
    g._fleet_attach_cb(SimpleNamespace(data=1))
    assert _fly_until(g, 1, lambda g: 1 not in g._rejoin, 200)
    g._rejoin_blend = None
    zs, lifts, _ = _t_blend(g, rho, length, push=0.006)
    assert zs.max() - zs[0] > 0.03 and lifts.max() > 0.03


def test_land_mid_descent_lands_it_from_its_out_point_past_the_clear_radius():
    """E3 (R2585): the out step's reference stopped 1.9 cm short of its goal (a departed drone's step counts as
    reached within 2 cm) and it touched down 1.179 m from the ring centre against the 1.2 m clear radius. It now aims
    REJOIN_OUT_MARGIN_M past the clear radius and comes down from the out point itself."""
    f = _over_the_plate()
    f._land_to_ground = True
    f._rejoin_tick()
    out = f._departed_next[1].copy()
    assert np.hypot(*out[:2]) == pytest.approx(OUT_R) and OUT_R > DEPARTED_CLEAR_R
    for _ in range(400):
        f._publish_departed_refs()
        if 1 in f._departed_landing:
            break
    assert 1 in f._departed_landing
    assert np.allclose(f._departed_hold[1][:2], out[:2], atol=1e-9)        # the out point, not 2 cm short of it
    z0 = f._departed_hold[1][2]
    for _ in range(100):
        f._publish_departed_refs()
    assert f._departed_hold[1][2] < z0 - 0.5 and np.allclose(f._departed_hold[1][:2], out[:2], atol=1e-9)


@pytest.mark.parametrize('pos_limit', [0.0, 1.5])
def test_the_staged_reference_stops_on_the_staging_bound_through_a_figure_8(pos_limit):
    """E4 (R2586): staged through the T's figure-8, the +x carrier's ramp ran into the 1.6 m clamp at 0.19 m/s, braked
    past it to 1.656 m, and its 2 s horizon, extrapolated at that speed, led the drone to 1.709 m. The ramp now brakes
    onto the staging bound (STAGE_TRACK_M inside the card's 1.6 m, or inside pos_limit_m's) and no published point, nor
    its velocity, passes it."""
    f = _node(kind='fig_8')
    f.traj = LoadTrajectory('fig_8', 0.2, 1.0, 0.9)
    f._pos_limit_m = pos_limit
    lim = (pos_limit if pos_limit > 0.0 else POSITION_LIMIT_M) - STAGE_LIMIT_MARGIN_M - STAGE_TRACK_M
    _leave(f, 0)
    _fly(f, 0, 60)
    rec = []
    f._publish_ref = lambda d, nodes: rec.append([(np.array(p, float), np.array(v, float)) for p, v, *_ in nodes])
    f._traj_running = True
    for _ in range(600):
        f.traj_t += 0.1
        f._rejoin_tick()
    p = np.array([[n[0] for n in ns] for ns in rec])
    v = np.array([[n[1] for n in ns] for ns in rec])
    assert np.abs(p[:, :, :2]).max() <= lim + 1e-9                         # every published point inside
    on = np.abs(p[:, :, :2]) >= lim - 1e-9
    assert np.all(v[:, :, :2][on] * np.sign(p[:, :, :2][on]) <= 1e-12)     # and no velocity out of it there
    assert p[:, 0, 0].max() >= lim - 1e-3                                  # the reference reached the bound,
    assert v[:, 0, 0].max() > 0.15                                         # running at it as in R2586
    assert np.linalg.norm(np.diff(v[:, 0, :], axis=0), axis=1).max() <= 2.0 * TAKE_BACK_ACCEL / HZ + 1e-9


# ── the critic on the 23 Oct fixes ────────────────────────────────────────────────────────────────────────────────────

def _folded(n=4, **kw):
    """A carrier left, staged and rejoined: its fold-in blend running (the detach graph: no attach_traj_hold_s)."""
    f = _node(n=n)
    f._attach_traj_hold_s = 0.0
    f.__dict__.update(kw)
    _leave(f, 1)
    _fly(f, 1, 60)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 400)
    f.refs.active = True
    return f


def test_a_rejoin_holds_the_path_through_its_whole_blend_and_quad4s_hold_is_as_before():
    """Critic M2: the rejoin held the path max(attach_traj_hold_s, reconfig_hold_s), 1.5 s on the detach graph, while
    its blend and _rejoin_cap ran rejoin_blend_s, so the ring moved on a resumed path under the cap. A rejoin now holds
    it rejoin_blend_s, and to the tick while the cap can act. QUAD4's fold-in holds as before."""
    from test_partner_contact import _fold_in_node
    f = _folded(_rejoin_blend_s=7.0)
    assert f.refs.blends[-1][1] == 7.0
    assert f._reconfig_hold_left == pytest.approx(7.0 - 1.0 / HZ)            # _fly counted the fold's tick
    assert any('trajectory held 7.0 s (the rejoin blend)' in m for m in f.log.lines)
    for _ in range(int(7.0 * HZ) + 20):            # a blend outlasting the hold's count: the path stays held
        f._rejoin_tick()
        assert f._reconfig_hold_left > 0.0
        f._reconfig_hold_left -= 1.0 / HZ
    f.refs.active = False
    f._rejoin_tick()
    assert f._rejoin_blend is None and f._reconfig_hold_left <= 1e-9          # the blend over: the path resumes
    for moving, held in ((True, 1.5), (False, 10.0)):
        g = _fold_in_node()
        g._partner_contact, g._attach_moving, g._rejoin_blend_s = False, moving, 7.0
        assert g._attach_ocp(3) is True
        assert g._reconfig_hold_left == held
        assert any(f'trajectory held {held:.1f} s; ring gap' in m for m in g.log.lines)


def test_the_blend_line_says_how_often_the_cap_bound_and_by_how_much_at_the_last():
    """Critic M1: REJOIN BLEND OVER logs the ticks the cap bound and how far above it the plan sat at the last."""
    f = _folded()
    body = np.asarray(f.drone_pos[1], float)
    zero = np.zeros(3)
    for dz in (0.02, 0.05, -0.01):                 # the last one sits under the cap: no bind
        f._rejoin_cap(1, [(body + [0.0, 0.0, dz], zero, zero, zero)] * 3)
    assert f._rejoin_blend['bind_n'] == 2 and f._rejoin_blend['bind_m'] == pytest.approx(0.05, abs=1e-9)
    f.refs.active = False
    _fly(f, 1, 1)
    line = next(m for m in f.log.lines if 'REJOIN BLEND OVER' in m)
    assert line.endswith('; the cap bound 2 ticks, the plan 5.0 cm above it at the last')
    g = _folded()
    g.refs.active = False
    _fly(g, 1, 1)
    assert next(m for m in g.log.lines if 'REJOIN BLEND OVER' in m).endswith('; the cap never bound')
