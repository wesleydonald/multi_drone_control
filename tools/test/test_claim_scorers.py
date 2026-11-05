"""The claim scorers of the final-tree batch (docs/claim_batch_2026-11-07.md, section 'Scoring'): each reads a claim's
cell, arm or case from the run's name, old or new, and scores the card's bars from its rows; where a registered run is
on this machine, the scorer reproduces its registered number (read-only, skipped elsewhere)."""
import glob
import os
import sys

import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS)
import rejoin_metrics  # noqa: E402
import s41_metrics  # noqa: E402
import x2_metrics  # noqa: E402
import x3_metrics  # noqa: E402
import x3b_metrics  # noqa: E402
import x8_metrics  # noqa: E402

REPO = os.path.dirname(TOOLS)


def _run(rid):
    hits = glob.glob(os.path.join(REPO, 'results', '2*', rid + '_*'))
    return hits[0] if hits and os.path.isfile(os.path.join(hits[0], 'logs', 'run.csv')) else None


def needs(rid):
    return pytest.mark.skipif(_run(rid) is None, reason=f'{rid} not on this machine')


def test_x3_case_from_old_and_new_names():
    pairs = {'s8_hover_R50_L100_it5': 'hover', 'x3_s8_hover_it5': 'hover', 'x3_s4_hover_it5': 's4_hover',
             's8_fig8_h5_v200_R50_L100_it5': 'fig8', 'x3_s8_fig8laps_it5': 'fig8laps',
             's8_orbit_r30_v200_R50_L100_it5': 'orbit', 'x3_s8_adjpair_it5_tip': 'adjpair',
             's8_chain4b2b_orbit_r30_v200_R50_L100_it5_tip': 'chain4b2b_orbit', 'x3_s8_chain4b2b_it5_tip': 'chain4b2b',
             'x3_s8_chain4back_fig8laps_it5_tip': 'chain4back_fig8laps', 's8_chain4back_pairs_R50_L100_it5_tip': 'pairs',
             'x3_s8_chain4back_relblend_it5_tip': 'relblend', 'x3_s8_chain4task_it5_tip': 'task',
             'x3_s8_fig8rel2_it5_tip': 'fig8rel2', 'x3_s8_relay_it5_tip': 'relay',
             'x3_s8_chain2back_it5_tip_noxy': 'chain2back', 'x3_s8_chain4back_it5_tip': 'chain4back'}
    for name, case in pairs.items():
        assert x3_metrics.case_of(f'/r/R9999_sim_gz_{name}') == case, name
    for path in open(os.path.join(REPO, 'configs', 'batches', 'claim1107_2_x3.txt')).read().split():
        assert x3_metrics.case_of('R9999_sim_gz_' + os.path.basename(path)[:-5]) in pairs.values(), path


def test_x3_p_form_and_n_bars():
    p = x3_metrics.p_form([(17.85, 1.19), (17.54, 1.26)])
    assert abs(p['bar_mm'] - 22.11) < 0.01 and abs(p['bar_tilt_deg'] - 2.225) < 1e-6

    class R:
        def __init__(self, mm, deg):
            self.rid, self.sm = 'R1', {'n1': {'pos_rmse_m': mm / 1e3, 'tilt_peak_deg': deg}}
    nb = x3_metrics.n_bars([R(1.35, 0.226), R(1.33, 0.185)])
    assert abs(nb['rmse_mm'] - 5.68) < 1e-9 and abs(nb['tilt_deg'] - 1.411) < 1e-9
    assert x3_metrics.n_bars([R(20.0, 4.0)])['rmse_mm'] == 25.0     # never above 25 mm


def test_x3b_cell_and_the_filter_read_back(tmp_path):
    assert x3b_metrics.cell_of('/r/R3572_sim_gz_x3b_s8_chain4back_rt_tip') == 'x3b_s8_chain4back_rt'
    assert x3b_metrics.cell_of('/r/R3584_sim_gz_x3_s4_hover_it5') == 'x3_s4_hover_it5'
    (tmp_path / 'params').mkdir()
    assert x3b_metrics.mocap_filters(str(tmp_path), {}) == ['slope']               # nothing typed: the default
    assert x3b_metrics.mocap_filters(str(tmp_path), {'lab_env': {'SIM_MOCAP_VEL_FILTER': 'ema'}}) == ['ema']
    for i, f in enumerate(('slope', 'slope', 'ema')):
        (tmp_path / 'params' / f'mocap_{i}.yaml').write_text(f'/mocap_{i}:\n  ros__parameters:\n    ring_velocity_filter: '
                                                             f'ema\n    velocity_filter: {f}\n')
    assert x3b_metrics.mocap_filters(str(tmp_path), {}) == ['ema', 'slope']


def test_x2_variant_and_x8_arm_from_the_name():
    assert x2_metrics.variant_of('/r/R3588_sim_gz_x2_full_fig8_soft') == 'A'
    assert x2_metrics.variant_of('/r/R3590_sim_gz_x2_full_fig8_unann') == 'B'
    assert x2_metrics.variant_of('/r/R3586_sim_gz_x2_full_fig8_base') == 'base'
    assert x8_metrics.cell_arm('R1_sim_gz_x8_rim_ours_noxy') == ('rim', 'ours')
    assert x8_metrics.cell_arm('R1_sim_gz_x8_hub_portu') == ('hub', 'portu')
    assert rejoin_metrics.case_of('R2603_sim_gz_rj3_pairrelease_rejoin_L100_leanneg') == 'T, leaning away'
    assert rejoin_metrics.case_of('R2602_sim_gz_rj3_pairrelease_rejoin_L100_lean') == 'T, leaning toward'


def _x8(run, cell, arm, E, loss=(), ts=5.0):
    return {'run': run, 'cell': cell, 'arm': arm, 'E_mm': E, 'loss': list(loss), 'ts': ts, 'ts_cens': False,
            'validity': {'reasons': []}}


def test_x8_bars():
    rows = [_x8('R1', 'hub', 'ours', 315.0), _x8('R2', 'hub', 'ours', 312.0), _x8('R3', 'hub', 'port', 365.0),
            _x8('R4', 'hub', 'port', 370.0), _x8('R5', 'rim', 'ours', 244.0), _x8('R6', 'rim', 'ours', 236.0),
            _x8('R7', 'rim', 'port', 250.0), _x8('R8', 'rim', 'port', 240.0, ts=12.0)]
    b = x8_metrics.bars(rows)
    assert b['L1']['verdict'] == 'CONFIRMED' and abs(b['L1']['bar_mm'] - 335.0) < 1e-9
    assert b['L2']['verdict'] == 'INCONCLUSIVE'          # the port's mean over ours', neither 20 mm over ours_max
    rows[7]['E_mm'] = 230.0
    assert x8_metrics.bars(rows)['L2']['verdict'] == 'FALSIFIED'
    assert b['L3']['verdict'] == 'HOLDS'
    rows[0]['ts'] = 11.0
    assert x8_metrics.bars(rows)['L3']['verdict'] == 'MISS'


@needs('R3592')
def test_x8_reproduces_the_reviewers_hub_excursion():
    o = x8_metrics.score(_run('R3592'))
    assert (o['cell'], o['arm']) == ('hub', 'ours')
    assert abs(o['E_mm'] - 315.06) < 0.01 and abs(o['ts'] - 5.664) < 1e-3 and not o['loss'] and not o['validity']['reasons']


@needs('R2596')
def test_rejoin_reproduces_the_square_row():
    o = rejoin_metrics.score(_run('R2596'))
    r = o['rejoins'][0]
    assert abs(r['plate_s'] - 23.87) < 0.01 and abs(r['fold_peak_deg'] - 1.55) < 0.01 and r['rod_end_cm'] == 1.6
    assert abs(o['lean_deg'] - 0.09) < 0.01 and o['holds']


@needs('R3507')
def test_s41_reproduces_the_release_flown_in_both():
    o = s41_metrics.score(_run('R3507'))
    assert o['kind'] == 'release' and abs(o['tilt_rise_deg'] - 6.59) < 0.01 and abs(o['dip_mm'] - 30.6) < 0.05
    assert abs(o['reaction']['back_s'] - 3.34) < 0.01 and abs(o['t6_back_s'] - 3.53) < 0.01
