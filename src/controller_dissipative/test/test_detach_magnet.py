"""detach_magnet (rig): the dissipative node latches the TETHERS' magnets, ON at ARM and OFF
at their detach. A reserved newcomer's latch is the magnet manager's (ladder P10), so ARM
must not turn on a newcomer that should stay OFF (drone 3 in R8a)."""
from types import SimpleNamespace

import pytest

pytest.importorskip('acados_template')
from controller_dissipative.dissipative_node import DissipativeController as D  # noqa: E402


class _Pub:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []

    def publish(self, m):
        self.msgs.append(m.data)


class _Log:
    def info(self, *a, **k):
        pass

    warn = error = info


def _node(n_carry0, detach_magnet=True):
    f = SimpleNamespace(_detach_magnet=detach_magnet, _n_carry0=n_carry0,
                        _magnet_pubs=[_Pub(f'/drone_{i}/magnet') for i in range(n_carry0)]
                        if detach_magnet else [])
    f.get_logger = lambda: _Log()
    return f


def test_arm_latches_only_the_tethers_on():
    f = _node(3)                       # M1: tethers 0-2, newcomer 3 (reserved)
    D._arm_magnets(f)
    assert [p.topic for p in f._magnet_pubs] == [f'/drone_{i}/magnet' for i in range(3)]
    assert [p.msgs for p in f._magnet_pubs] == [['ON']] * 3


def test_detach_releases_a_tether_and_never_the_newcomer():
    f = _node(3)
    D._release_magnet(f, 1)
    D._release_magnet(f, 3)            # newcomer: released through /magnet/command instead
    assert [p.msgs for p in f._magnet_pubs] == [[], ['OFF'], []]


def test_off_by_default_publishes_nothing():
    f = _node(4, detach_magnet=False)
    D._arm_magnets(f)
    D._release_magnet(f, 0)
    assert f._magnet_pubs == []
