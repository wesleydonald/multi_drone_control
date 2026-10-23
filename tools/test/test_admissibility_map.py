"""Checks on the static admissibility map (tools/admissibility_map.py): the support polygon and the shares."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import admissibility_map as am  # noqa: E402


def test_even_ring_has_zero_margin_after_any_loss():
    rows = am.removal_rows('0/3/6/9', (0, 90, 180, 270), 0.225, 60.0)
    assert all(r['largest_gap_deg'] == 180.0 for r in rows)
    assert all(not r['level_ok'] for r in rows)


def test_1359_margins_match_the_support_polygon():
    # after the 90 deg leaver the survivors sit at 30/150/270: an equilateral triangle, margin r cos(60) = 112.5 mm
    rows = {r['leaver_deg']: r for r in am.removal_rows('1/3/5/9', (30, 90, 150, 270), 0.225, 60.0)}
    assert abs(rows[90]['margin_mm'] - 112.5) < 0.2
    assert rows[90]['largest_gap_deg'] == 120.0
    # losing 270 leaves 30/90/150: a 240 deg gap, the centre outside
    assert rows[270]['largest_gap_deg'] == 240.0 and rows[270]['margin_mm'] < 0


def test_shares_sum_to_the_ring_weight_and_steps_balance():
    _, sh = am.shares((30, 90, 150, 270), 0.225, 60.0)
    assert abs(sh.sum() - 1.0) < 1e-6
    for r in am.removal_rows('1/3/5/9', (30, 90, 150, 270), 0.225, 60.0):
        if r['level_ok']:
            # the survivors take on exactly the leaver's share where a level hover exists
            assert abs(r['survivor_step_sum'] - r['leaver_share_before']) < 2e-3
        else:
            # no level hover: the solve floors tensions at t_min and the shares no longer balance
            assert r['margin_mm'] <= 0


def test_no_four_plate_layout_can_lose_an_arbitrary_carrier():
    # the worst single removal merges two adjacent gaps; all four gaps sum to 360 deg, so some merged pair >= 180
    ranking = am.all_layouts(0.225)
    assert all(r['worst_removal_margin_mm'] <= 0 for r in ranking)
    assert max(r['n_admissible_leavers'] for r in ranking) <= 2


def test_newcomer_on_the_even_triple():
    # the newcomer can always leave again (back to the even triple, margin r cos 60 = 112.5 mm); at a gap centre it
    # is the only admissible leaver, next to a carrier (30 deg away) that carrier may leave as well
    rows = {r['newcomer_deg']: r for r in am.newcomer_rows('0/4/8', (0, 120, 240), 0.225, 60.0)}
    for r in rows.values():
        assert abs(r['newcomer_leaves_again_margin_mm'] - 112.5) < 0.2
        assert r['n_admissible_leavers'] in (1, 2)
    assert all(rows[a]['n_admissible_leavers'] == 1 for a in (60, 180, 300))
    assert all(rows[a]['n_admissible_leavers'] == 2 for a in (30, 90, 150, 210, 270, 330))
