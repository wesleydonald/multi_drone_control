"""tools/thesis_runsets.py: the 0.55 m run lists are the ones the thesis generators and the registry cite, and the 1 m
set resolves only finished, non-void r1m_ runs."""
import csv
import os
import sys

import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import thesis_runsets as RS  # noqa: E402

REGISTRY = os.path.join(RS.REPO, 'results', 'registry.csv')


def _old(key):
    return RS.runset(key, 'old')


def test_the_detach_lists_are_the_generators():
    tl = pytest.importorskip('thesis_lever_figures')
    import thesis_freeze_tables as tf
    a = _old('tier_a')
    arm = lambda k: [a[f'{k}_r{r}'][0] for r in (1, 2, 3)]  # noqa: E731
    assert (arm('A1'), arm('A2'), arm('A5'), arm('A6')) == (tl.A1, tl.A2, tl.A5, tl.A6)
    b = _old('tier_b')
    for g, runs in ((135, 'G135'), (150, 'G150'), (165, 'G165'), (180, 'G180')):
        assert [b[f'{runs}_r1'][0], b[f'{runs}_r2'][0]] == tl.TIER_B[g]
    assert [b['G150P_r1'][0], b['G150P_r2'][0]] == tl.G150P and b['G180D_r1'] == tl.G180D
    assert _old('ledger_w4')['W4'] == tf.W4_TWIN and _old('ledger_w5')['W5'] == tf.W5_TWIN
    assert tf.NO_SWITCH_CONTROL[0] == (arm('A5'), arm('A1'))


def test_the_appendix_figures_first_runs_are_the_tables():
    sf = pytest.importorskip('thesis_sim_figures')
    app = RS.APP_RUNS
    first = lambda c: app[c].split()[0]  # noqa: E731
    assert sf.X0_RUNS == dict(hold=(first('app_x0_hold_eps025'), first('app_x0_hold_eps0')),
                              orbit=(first('app_x0_orbit_eps025'), first('app_x0_orbit_eps0')))
    assert sf.LAND_RUNS == dict(before=first('app_land_straight'), after=first('app_x0_hold_eps0'))
    assert sf.DETACH_RUN == _old('tier_a')['A1_r1'][0]
    assert _old('flight')['flight'] == [first('app_rob_none_int')]


@pytest.mark.skipif(not os.path.isfile(REGISTRY), reason='no results/registry.csv')
def test_every_cited_run_is_a_registry_row_of_its_config():
    rows = {r[0]: r for r in csv.reader(open(REGISTRY)) if r and r[0].startswith('R')}
    for key, t in RS.TABLES.items():
        for c in t['cells']:
            for rid in c['old']:
                assert rid in rows, (key, rid)
                assert rows[rid][2].split(' ')[0] == RS.FLOWN_AS.get(c['config'], c['config']), (key, rid)
                assert 'VOID' not in rows[rid][9].upper(), (key, rid)


def test_repeats_and_sources():
    src = RS.sources()
    assert src['w4_da_release_hover'] == 3 and src['ta_a1_r1'] == 1 and src['traj3_spin_v300'] == 2
    for t in RS.TABLES.values():
        for c in t['cells']:
            assert c['repeats'] >= len(c['old']) or t['scorer'] in ('departed', 'traj', '')


def test_every_table_has_its_1m_worlds():
    # Tier B's four layouts and the push world at 1 m: Wesley 23 Oct (decisions.md, DECIDED 2026-10-23 (8))
    assert {k: RS.status(k) for k in RS.TABLES} == {k: ('refly', []) for k in RS.TABLES}


def test_status_names_the_missing_worlds(tmp_path, monkeypatch):
    # an assets folder without Tier B's and the push world's 1 m siblings, as before 23 Oct
    gone = {'four_rigid_ground_rig_3969_L100.sdf', 'four_rigid_ground_rig_gap135_L100.sdf',
            'four_rigid_ground_rig_gap165_L100.sdf', 'four_rigid_ground_rig_gap180_L100.sdf',
            'three_rigid_ground_rig_push_L100.sdf'}
    for w in os.listdir(RS.ASSETS):
        if w.endswith('.sdf') and w not in gone:
            (tmp_path / w).touch()
    monkeypatch.setattr(RS, 'ASSETS', str(tmp_path))
    assert RS.status('T7t') == ('refly', [])
    st, miss = RS.status('tier_b')
    assert st == 'waits for worlds' and miss == sorted(gone - {'three_rigid_ground_rig_push_L100.sdf'})
    assert RS.status('disturbance') == ('partial', ['three_rigid_ground_rig_push_L100.sdf'])
    assert RS.status('gain')[0] == 'waits for worlds'


def test_the_1m_set_takes_finished_non_void_r1m_runs(tmp_path, monkeypatch):
    day = tmp_path / '2026-10-28'
    for rid, cfg, done in (('R9001', 'r1m_ta_a1_r1', True), ('R9002', 'r1m_ta_a1_r1', True),
                           ('R9003', 'r1m_ta_a1_r1', False), ('R9004', 'ta_a1_r1', True)):
        d = day / f'{rid}_sim_gz_{cfg}'
        d.mkdir(parents=True)
        if done:
            (d / 'metrics.json').write_text('{}')
    (tmp_path / 'registry.csv').write_text('run,date,config,card,variable,baseline,metric,value,repeat_of,verdict,'
                                           'reviewer,notes\nR9002,2026-10-28,r1m_ta_a1_r1,,,,,,,VOID (pose),,\n')
    monkeypatch.setattr(RS, 'RESULTS', str(tmp_path))
    new = RS.runset('tier_a', '1m')
    assert new['A1_r1'] == ['R9001'] and new['A2_r1'] == []


def test_the_listing_runs():
    assert RS.main([]) == 0 and RS.main(['T7t', '--json']) == 0
