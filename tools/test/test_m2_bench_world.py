"""tools/sim_test/make_m2_bench_world.py: the M2 hand-over bench geometry (GOALS step 3).

Generated into a temp dir from his shipped SDFs, without Gazebo: every rod tip sits on
its plate, the ring is the runner's dynamic 0.86 kg patch at its floor rest height, and
each X3 hangs on a detachable hanger.
"""
import os
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'sim_test'))

import make_m2_bench_world as bench  # noqa: E402


@pytest.fixture(scope='module')
def built(tmp_path_factory):
    out = str(tmp_path_factory.mktemp('bench_m2'))
    world, man = bench.build(out, bench.HOLD_T0019, bench.HOLD_T0019_SOURCE, gz_check=False)
    return out, world, man


def test_rod_tips_on_their_plates(built):
    _, _, man = built
    for k, c in man['checks'].items():
        assert abs(c['gap_m']) < 1e-4 and c['plate_xy_err_m'] < 1e-4, k
        assert abs(c['rod_len_m'] - 0.45) < 1e-6 and c['pivot_err_m'] < 1e-6, k
    assert [man['checks'][f'drone_{i}']['plate'] for i in range(4)] == [3, 0, 6, 9]


def test_bodies_at_the_logged_hold(built):
    _, _, man = built
    # T0019 attachment.csv at t=103.84 (the switch is at 104.50, within 3 mm): bodies at (-0.128, 0.351), (0.351, 0.128), (-0.350, -0.127),
    # (0.131, -0.358), z 0.602-0.605 with the magnets welded 6-7 mm above the plates
    logged = [(-0.128, 0.351), (0.351, 0.128), (-0.350, -0.127), (0.131, -0.358)]
    for i, (x, y) in enumerate(logged):
        b = man['checks'][f'drone_{i}']['body']
        assert abs(b[0] - x) < 0.006 and abs(b[1] - y) < 0.006
        assert 0.590 < b[2] < 0.605


def test_ring_dynamic_at_rest_and_hangers_present(built):
    out, world, _ = built
    ring = ET.parse(os.path.join(out, 'm2_bench_ring.sdf')).getroot().find('model')
    assert ring.findtext('static').strip() == 'false'
    assert float(ring.find("link[@name='payload_link']/inertial/mass").text) == 0.86
    assert len(ring.findall("plugin[@name='gz::sim::systems::DetachableJoint']")) == 4
    w = ET.parse(world).getroot().find('world')
    ring_inc = [i for i in w.findall('include') if i.findtext('uri').endswith('m2_bench_ring.sdf')]
    assert len(ring_inc) == 1 and float(ring_inc[0].findtext('pose').split()[2]) == 0.1
    for i in range(4):
        h = w.find(f"model[@name='bench_hanger_{i}']")
        # welded to the world, not static: dartsim keeps a detached base_link in the
        # parent's skeleton, and a static one never moves
        assert h is not None and h.findtext('static') is None
        j = h.find("joint[@name='hanger_to_world']")
        assert j.get('type') == 'fixed' and j.findtext('parent') == 'world'
        assert j.findtext('child') == 'hanger_link'
        p = h.find('plugin')
        assert p.findtext('child_model') == f'x3_{i}' and p.findtext('child_link') == 'X3/base_link'
        assert p.findtext('detach_topic') == f'/bench/hanger_{i}/detach'
    assert all(os.path.isabs(i.findtext('uri')) for i in w.findall('include'))


def test_gap_limit_is_enforced(tmp_path):
    with pytest.raises(SystemExit, match='above plate'):
        bench.build(str(tmp_path), bench.HOLD_T0019, 'test', weld_gap=0.006, gz_check=False)
