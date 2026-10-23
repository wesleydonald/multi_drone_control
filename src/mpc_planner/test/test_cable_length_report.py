"""The CABLE LENGTH pre-flight line (1 m cables, 22 Oct): a drone resting further from its plate than cable_len allows
is a planner typed for shorter cables; the twin's rods are rigid, so there a drone resting nearer is one too (a 1 m
planner on a 0.55 m world), while on the rig a slack cable rests short as a matter of course. The line shares the one
latched status message with SLOT OFFSET."""
import types

import numpy as np

from mpc_planner.geometry import attach_points
from mpc_planner.planner_node import LoadPlanner


class _Log:
    def __init__(self):
        self.lines = []

    def warn(self, text):
        self.lines.append(text)

    info = error = warn


def _planner(cable_len, rests, twin, off_deg=0.0):
    """Three drones on the rig ring, each resting rests[i] from its plate (the rod pivot's distance), drone 1 turned
    off_deg about the ring from its slot."""
    rho = attach_points(3, 0.225, 0.02)
    az = [np.arctan2(r[1], r[0]) + (np.radians(off_deg) if i == 0 else 0.0) for i, r in enumerate(rho)]
    pos = [np.array([np.cos(a), np.sin(a), 0.0]) * (0.225 + d) for a, d in zip(az, rests)]
    log, published = _Log(), []
    p = types.SimpleNamespace(
        n=3, cable_len=cable_len, slot2drone=[0, 1, 2], rho=rho, drone_pos=pos, psi0=0.0,
        load_state=np.r_[0.0, 0.0, 0.02, 1.0, 0.0, 0.0, 0.0, np.zeros(6)],
        get_parameter=lambda name: types.SimpleNamespace(value=twin),
        get_logger=lambda: log, _status_pub=types.SimpleNamespace(publish=lambda m: published.append(m.data)))
    p._drone_at = lambda i: pos[p.slot2drone[i]]
    p._rim_dist = lambda i: rests[i]
    p._cable_length_report = lambda: LoadPlanner._cable_length_report(p)
    return p, log, published


def test_the_rig_warns_only_when_a_drone_rests_beyond_its_cable():
    p, _, _ = _planner(1.0, [0.55, 0.6, 1.02], twin=False)        # slack 1 m cables resting short: as normal
    assert LoadPlanner._cable_length_report(p) == []
    p, _, _ = _planner(0.55, [1.0, 0.98, 0.56], twin=False)       # a 0.55 m planner on 1 m cables
    lines = LoadPlanner._cable_length_report(p)
    assert [ln.split(' rests')[0] for ln in lines] == ['drone 1', 'drone 2']
    assert 'rests 1.00 m from its plate against cable_len 0.55' in lines[0]


def test_the_twin_warns_both_ways():
    p, _, _ = _planner(1.0, [0.55, 0.55, 0.55], twin=True)        # a 1 m planner on a 0.55 m world
    assert len(LoadPlanner._cable_length_report(p)) == 3
    p, _, _ = _planner(1.0, [1.0, 0.99, 1.01], twin=True)
    assert LoadPlanner._cable_length_report(p) == []


def test_slot_offset_and_cable_length_share_one_status_line():
    p, log, published = _planner(1.0, [0.55, 0.55, 0.55], twin=True, off_deg=40.0)
    LoadPlanner._check_slot_offsets(p)
    assert len(published) == 1
    slot, cable = published[0].split(' | ')
    assert slot.startswith('SLOT OFFSET: drone 1 sits') and cable.startswith('CABLE LENGTH: drone 1 rests 0.55 m')
    assert cable.count('; ') == 2 and any('[planner] CABLE LENGTH' in ln for ln in log.lines)
    p, _, published = _planner(1.0, [0.55, 1.0, 1.0], twin=True)
    LoadPlanner._check_slot_offsets(p)
    assert published == ['CABLE LENGTH: drone 1 rests 0.55 m from its plate against cable_len 1.00: a planner for '
                         'other cables? Type cable_len for the cables flown']
    p, _, published = _planner(1.0, [1.0, 1.0, 1.0], twin=True)
    LoadPlanner._check_slot_offsets(p)
    assert published == []
