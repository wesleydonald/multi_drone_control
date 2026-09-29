"""tools/headroom.py against numbers already on record."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
pytest.importorskip('controller_load_mpc.geometry')
import headroom  # noqa: E402


def test_reproduces_the_sim_twin_r0726():
    # 3 on 1/5/9, 0.64 kg, sim kT 83.1: logged HOLD throttle 0.181
    rows = headroom.predict([0.64] * 3, [30, 150, 270], 0.86, kt=[83.1] * 3)
    assert all(abs(u - 0.181) < 0.004 for _, _, u in rows)


def test_uneven_ring_tensions_match_the_planner_log():
    # the dissipative node logged tensions [2.65, 4.64, 2.65, 1.99] for azimuths [150, -90, 30, 90]
    rows = headroom.predict([0.64] * 4, [150, -90, 30, 90], 0.86, kt=[83.1] * 4)
    assert [round(t, 2) for t, _, _ in rows] == [2.65, 4.64, 2.65, 1.99]


def test_free_hover_scaling_is_the_closed_form_for_an_even_ring():
    m, uf = 0.8, 0.45
    r = 0.86 / 3 / m
    rows = headroom.predict([m] * 3, [30, 150, 270], 0.86, u_free=[uf] * 3)
    assert rows[0][2] == pytest.approx(uf * ((1 + r) ** 2 + r ** 2) ** 0.5, rel=1e-6)
