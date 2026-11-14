"""tools/x1_metrics.py's table and metrics.spread_text: the supervisors' format (mean +- SD for n >= 3, both values for
n = 2, the value for n = 1), and an arm's percent against its cell's no-resize arm."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import metrics as M  # noqa: E402
import x1_metrics  # noqa: E402


def test_spread_text_by_n():
    assert M.spread_text([]) == '-'
    assert M.spread_text([None, float('nan')]) == '-'
    assert M.spread_text([1.0]) == '1.00'
    assert M.spread_text([3.0, 1.0]) == '1.00 - 3.00'
    assert M.spread_text([1.0, 2.0, 3.0], 1) == '2.0 +- 1.0'


def _row(arm, settled):
    r = {k: None for k, _, _ in x1_metrics.TABLE}
    r.update(cell='sqT', arm=arm, settled_tilt=settled)
    return r


def test_table_gives_each_arm_against_no_resize():
    txt = x1_metrics.table([_row('ours', 1.0), _row('ours', 1.2), _row('nors', 2.0), _row('nors', 2.0)])
    ours = next(ln for ln in txt.splitlines() if ln.startswith('| ours'))
    nors = next(ln for ln in txt.splitlines() if ln.startswith('| nors'))
    assert '1.00 - 1.20 (-45 %)' in ours
    assert '2.00 - 2.00' in nors and '%' not in nors


def _arm(cell, arm, rise, dip, settled=0.1, t6=3.0, peak=None, loss=False, run='R0'):
    delay = x1_metrics.ARM_DELAY[arm]
    return dict(run=run, cell=cell, arm=arm, tilt_rise=rise, dip_mm=dip, settled_tilt=settled, t_back_t6=t6,
                t_level=2.0, abs_peak=rise if peak is None else peak, dropped=loss, loss=loss, unannounced_lines=1,
                detect_s=None if delay > 100 else 0.3, resize_s=None if delay > 100 else 0.3 + delay,
                n_after=None if delay > 100 else 3, late_resize_lines=int(0 < delay < 100), resize_delay_s=delay,
                void_late=False, failed_solves=0)


def _cell(cell, ours, nors, d05=None, d10=None, d20=None, **kw):
    rows = []
    for arm, vals in (('ours', ours), ('nors', nors), ('d05', d05), ('d10', d10), ('d20', d20)):
        for k, v in enumerate(vals or []):
            rows.append(_arm(cell, arm, *v, run=f'{arm}{k}', **kw))
    return rows


def test_bars_b1_falsified_inside_the_bands_and_confirmed_by_the_dip():
    flat = _cell('h4', ours=[(6.8, 32.0), (6.9, 32.7)], nors=[(6.5, 31.0), (7.0, 32.7)])
    worse = _cell('h8', ours=[(1.6, 13.0), (1.6, 15.0)], nors=[(1.6, 21.0), (1.7, 20.5)])
    b = x1_metrics.bars(flat + worse)
    assert b['cells']['h4']['B1']['verdict'] == 'FALSIFIED'
    assert b['cells']['h8']['B1']['verdict'] == 'CONFIRMED'       # both repeats dip more than ours_max + 5 mm
    assert b['B1_claim'] == 'CONFIRMED for h8'
    b = x1_metrics.bars(flat + _cell('h8', ours=[(1.6, 15.0)] * 2, nors=[(1.6, 15.5)] * 2))
    assert b['B1_claim'] == 'FALSIFIED'


def test_bars_b2_order_and_the_late_arm():
    late = [(7.0, 31.0)] * 2
    rows = _cell('h4', ours=[(6.0, 30.0)] * 2, d05=late, d10=late, d20=[(7.2, 31.0)] * 2, nors=[(7.3, 31.0)] * 2)
    assert x1_metrics.bars(rows)['cells']['h4']['B2']['verdict'] == 'CONFIRMED'
    inverted = _cell('h4', ours=[(6.0, 30.0)] * 2, d05=[(8.0, 31.0)] * 2, d10=late, d20=[(7.2, 31.0)] * 2,
                     nors=[(7.3, 31.0)] * 2)
    b2 = x1_metrics.bars(inverted)['cells']['h4']['B2']
    assert b2['verdict'] == 'FALSIFIED' and abs(b2['largest_inversion'] - 1.0) < 1e-9


def test_bars_b4_reads_t6_in_hover_and_u5_in_the_t():
    rows = _cell('h4p115', ours=[(9.4, 45.0)] * 2, nors=[(9.3, 45.0)] * 2, t6=5.5)
    b4 = x1_metrics.bars(rows)['cells']['h4p115']['B4']
    assert b4['verdict'] == 'MISS' and b4['verdict_frozen_t_level'] == 'HOLDS'
    rows = _cell('sqTnoxy', ours=[(56.0, 97.0, 1.8)] * 2, nors=[(57.0, 108.0, 2.2)] * 2, t6=4.0)
    c = x1_metrics.bars(rows)['cells']['sqTnoxy']
    assert c['U1']['verdict'] == 'CONFIRMED' and c['U5']['verdict'] == 'HOLDS'


def test_bars_leave_a_void_run_out():
    rows = _cell('h4', ours=[(6.8, 32.0), (6.9, 32.7)], nors=[(6.5, 31.0), (7.0, 32.7)])
    rows[0]['void_late'] = True
    rows[2]['n_after'] = 3                                   # a no-resize arm that resized
    b = x1_metrics.bars(rows)
    assert set(b['void']) == {rows[0]['run'], rows[2]['run']}
    assert b['cells']['h4']['runs'] == {'ours': ['ours1'], 'nors': ['nors1']}
