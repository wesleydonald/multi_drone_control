#!/usr/bin/env python3
"""tools/thesis_runsets.py -- the runs behind every thesis twin table: the 0.55 m runs it was built from (as cited in
its card and generator) and its 1 m re-fly (thesis R, card docs/experiments/2026-10-23_refly_1m.md), found in the
results by the generated config's name, r1m_<source>.

    python3 tools/thesis_runsets.py                 # every table: cells, 0.55 m runs, 1 m configs and runs found
    python3 tools/thesis_runsets.py T7t tier_a      # some tables
    python3 tools/thesis_runsets.py --json          # the same, machine-readable

A source is re-flown only where its world has a 1 m sibling (<world>_L100.sdf) in simulation_assets/: generating a
world is Wesley's word, so a table whose world is missing is listed as waiting for it, never flown on a stand-in.
"""
import argparse
import csv
import glob
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.environ.get('MDC_RESULTS_ROOT', os.path.join(REPO, 'results'))
ASSETS = os.path.join(REPO, 'simulation_assets')
PREFIX = 'r1m_'
CARD = 'docs/experiments/2026-10-23_refly_1m.md'

CARDS = {'carry': 'docs/experiments/2026-10-16_carry_speed_envelope.md',
         'tier_a': 'docs/experiments/2026-10-12_tier_a_crossover.md (rev 3)',
         'tier_b': 'docs/experiments/2026-10-16_tier_b_support_margin.md',
         'departed': 'docs/experiments/2026-10-09_departed_disarm_alone.md (detach freeze re-fly)',
         'drop': 'docs/experiments/2026-10-08_drop_and_land.md (v3, the v2 bars)',
         'appendix': 'docs/experiments/2026-10-16_appendix_refly.md'}

SHAPES = ('circle', 'fig8', 'linex', 'orbit', 'spin')
SPEEDS = ('125', '200', '300')
CARRY_RUNS = {'circle_v125': 'R1847 R1848', 'circle_v200': 'R1857 R1858', 'circle_v300': 'R1859 R1860',
              'fig8_v125': 'R1849 R1850', 'fig8_v200': 'R1861 R1862', 'fig8_v300': 'R1863 R1864',
              'linex_v125': 'R1851 R1852', 'linex_v200': 'R1865 R1866', 'linex_v300': 'R1867 R1868',
              'orbit_v125': 'R1853 R1854', 'orbit_v200': 'R1869 R1870', 'orbit_v300': 'R1871 R1872',
              'spin_v125': 'R1855 R1856', 'spin_v200': 'R1873 R1874', 'spin_v300': 'R1875 R1876'}
TIER_A_RUNS = {'a1': 'R1887 R1891 R1895', 'a2': 'R1888 R1892 R1896', 'a5': 'R1889 R1893 R1897',
               'a6': 'R1890 R1894 R1898'}
TIER_B_RUNS = {'g135': 'R1902 R1907', 'g150': 'R1899 R1904', 'g150p': 'R1900 R1905', 'g165': 'R1903 R1908',
               'g180': 'R1901 R1906'}
APP_RUNS = {'app_x0_hold_eps0': 'R2023 R2035', 'app_x0_hold_eps025': 'R2024 R2036',
            'app_x0_orbit_eps0': 'R2025 R2037', 'app_x0_orbit_eps025': 'R2026 R2038',
            'app_pushz_none': 'R2027 R2039', 'app_pushz_ref': 'R2028 R2040', 'app_pushz_model': 'R2029 R2041',
            'app_push04_none': 'R2030 R2042', 'app_push04_model': 'R2031 R2043',
            'app_mapneg_ref': 'R2032 R2044', 'app_mapneg_model': 'R2033 R2045', 'app_rob_none_int': 'R2034 R2046',
            'app_pushz_model_ki08': 'R2047 R2062', 'app_pushz_model_settle': 'R2048 R2063',
            'app_land_straight': 'R2049 R2064',
            'app_rob_m+10_z0': 'R2050 R2065', 'app_rob_m+20_z0': 'R2051 R2066', 'app_rob_m-20_z0': 'R2052 R2067',
            'app_rob_rod+5_z0': 'R2053 R2068', 'app_rob_rod+5nomeas_z0': 'R2054 R2069',
            'app_rob_r+2_z0': 'R2055 R2070', 'app_rob_m+10_int': 'R2056 R2071', 'app_rob_m+20_int': 'R2057 R2072',
            'app_rob_m-20_int': 'R2058 R2073', 'app_rob_rod+5_int': 'R2059 R2074',
            'app_rob_rod+5nomeas_int': 'R2060 R2075', 'app_rob_r+2_int': 'R2061 R2076'}
ROB_ARMS = ('m+10', 'm+20', 'm-20', 'rod+5', 'rod+5nomeas', 'r+2')
# a cell whose cited 0.55 m run flew under another config of the same bytes (name and description apart)
FLOWN_AS = {'w4_da_release_hover': 'ta_a1_r1'}


def _cell(name, config, repeats, old):
    return {'cell': name, 'config': config, 'repeats': repeats, 'old': old.split() if isinstance(old, str) else old}


def _carry_cells(speeds=SPEEDS):
    return [_cell(f'{s}_v{v}', f'traj3_{s}_v{v}', 2, CARRY_RUNS[f'{s}_v{v}']) for v in speeds for s in SHAPES]


def _app(*configs):
    return [_cell(c[len('app_'):], c, 2, APP_RUNS[c]) for c in configs]


# One entry per thesis table (or group of tables scored together), in the thesis's priority order. 'scorer' names the
# scorer in tools/refly_score.py; 'old' are the 0.55 m runs the table cites, per cell.
TABLES = {
    'ledger_w4': dict(
        thesis='T13 W4 twin column; the 1 m W4 ledger band input (push answer 9); disarm_lanes(b) hover lane; '
               'tab:faults "freed UAV disarmed alone"',
        card=CARDS['departed'], scorer='departed',
        cells=[_cell('W4', 'w4_da_release_hover', 3, 'R1887')],
        note='the 0.55 m W4 column is R1887 (ta_a1_r1, the same config bytes; n = 1 on the freeze tree); the 0.55 m '
             'band [1.75, 14.51] came from R1459/R1460/R1626 and is not a 1 m bar'),
    'tier_a': dict(
        thesis='T9 (appendix), F10b, F12a, fig:detach (A1 r1), T10 twin rows: pairs T3 (A2 vs A1), T8 (A6 vs A2), '
               'T9 (A5 vs A1, no sign), K1',
        card=CARDS['tier_a'], scorer='tier_a',
        cells=[_cell(f'{a.upper()}_r{r}', f'ta_{a}_r{r}', 1, TIER_A_RUNS[a].split()[r - 1])
               for r in (1, 2, 3) for a in ('a1', 'a2', 'a5', 'a6')]),
    'ledger_w5': dict(
        thesis='T13 W5 twin column; disarm_lanes(b) circle lane; T10 twin rows; tab:faults',
        card=CARDS['departed'], scorer='departed',
        cells=[_cell('W5', 'w4_gi_g2_release_orbit', 2, 'R1913 R1914')],
        note='the 0.55 m band [1.00, 13.20] (R1457/R1627) is not a 1 m bar; no rig circle flies at 1 m'),
    'drop': dict(
        thesis='tab:faults twin column (drop and land), T10 twin rows',
        card=CARDS['drop'], scorer='drop',
        cells=[_cell('D1', 'dl_d1_second_loss', 2, 'R1915 R1916'), _cell('D4', 'dl_d4_gap240', 2, 'R1917 R1918')]),
    't15': dict(
        thesis='T9 row T15 (the 60 deg circle, planned against measured pull)',
        card=CARDS['tier_a'], scorer='t15',
        cells=[_cell('A7', 'ta_a7_circle60', 2, 'R1909 R1911'), _cell('A8', 'ta_a8_circle60', 2, 'R1910 R1912')]),
    'tier_b': dict(
        thesis='F12b, T9 rows B1-B5, tab:faults rows "tilt over 30 deg" (G150p) and "drop rule off" (G180)',
        card=CARDS['tier_b'], scorer='',
        cells=[_cell(f'{g.upper()}_r{r}', f'tb_{g}_r{r}', 1, TIER_B_RUNS[g].split()[r - 1])
               for r in (1, 2) for g in ('g135', 'g150', 'g150p', 'g165', 'g180')]
        + [_cell('G180D_r1', 'tb_g180drop_r1', 1, 'R1919')]),
    'T7t': dict(
        thesis='tab:T7t (carry speed envelope, main text); 30 of tab:land\'s landings',
        card=CARDS['carry'], scorer='carry', cells=_carry_cells()),
    'traj': dict(
        thesis='tab:traj and fig:traj (appendix; the hover row and the five shapes at 0.125 m/s)',
        card=CARDS['appendix'], scorer='traj',
        cells=[_cell('hover', 'app_rob_none_int', 2, APP_RUNS['app_rob_none_int'])] + _carry_cells(('125',)),
        note='no runs of its own: the hover row is tab:robust\'s, the shapes T7t\'s'),
    'x0': dict(
        thesis='tab:x0 (claims C-X1, C-X2)', card=CARDS['appendix'], scorer='x0',
        cells=_app('app_x0_hold_eps0', 'app_x0_hold_eps025', 'app_x0_orbit_eps0', 'app_x0_orbit_eps025')),
    'disturbance': dict(
        thesis='tab:disturbance (main text; C-D1..C-D5), fig:disturbance',
        card=CARDS['appendix'], scorer='mapneg',
        cells=_app('app_pushz_none', 'app_pushz_ref', 'app_pushz_model', 'app_push04_none', 'app_push04_model',
                   'app_mapneg_ref', 'app_mapneg_model'),
        note='the push rows fly three_rigid_ground_rig_push_L100.sdf (Wesley 23 Oct)'),
    'gain': dict(
        thesis='tab:gain (C-G1, C-G2)', card=CARDS['appendix'], scorer='',
        cells=_app('app_pushz_model', 'app_pushz_model_ki08', 'app_pushz_model_settle')),
    'robust': dict(
        thesis='tab:robust (C-R1..C-R4)', card=CARDS['appendix'], scorer='robust',
        cells=_app('app_x0_hold_eps0', 'app_rob_none_int',
                   *[f'app_rob_{a}_{k}' for k in ('z0', 'int') for a in ROB_ARMS])),
    'land': dict(
        thesis='tab:land (C-L1, C-L2)', card=CARDS['appendix'], scorer='land',
        cells=_app(*[c for c in APP_RUNS if c != 'app_land_straight']) + _carry_cells()
        + _app('app_land_straight'),
        note='C-L1 pools every three-carrier landing but the straight descent (L-S, C-L2)'),
    'flight': dict(
        thesis='fig:flight (one flight from the floor)', card=CARDS['appendix'], scorer='',
        cells=[_cell('flight', 'app_rob_none_int', 1, 'R2034')]),
}

# Kept as labelled 0.55 m history (Wesley 22 Oct, push answer 10), not re-flown.
HISTORY = [
    ('8 Oct twin-rig pairs', 'T13 lower rows, fig:twin_rig_parity, fig:rig_detach, fig:rig_circle_detach, T8b twin '
                             'column (R1207/R1212, R1140/R1145 or R1712/R1713, R1206/R1211, R1208/R1213)'),
    ('freed-UAV landing against the rig', 'fig:departed_landing (rig r303_4 against twin R1257/R1306)'),
    ('the earlier attach sequence', 'fig:land_phases (R1413-R1446), fig:tip_feedback, disarm_lanes(a)'),
    ('superseded', 'tab:T7t1m (traj3_L100: T7t itself becomes 1 m), tab:bands1m (the 22 Oct bands replace it)'),
    ('legacy or never flown', 'tab:methods, tab:detach_ab, T11, T12, F14'),
]


def world_1m(world):
    """The 1 m sibling of a 0.55 m world if it exists, else None (a missing world is Wesley's word to generate)."""
    w = os.path.basename(world)
    if w.endswith('_L100.sdf'):
        return None
    sib = w[:-len('.sdf')] + '_L100.sdf'
    return sib if os.path.isfile(os.path.join(ASSETS, sib)) else None


def source_world(config):
    import yaml
    with open(os.path.join(REPO, 'configs', 'experiments', config + '.yaml')) as fh:
        return yaml.safe_load(fh)['world']


def missing_world(config):
    """The 1 m world a source would need, if it does not exist; '' when it does."""
    w = source_world(config)
    return '' if world_1m(w) else w[:-len('.sdf')] + '_L100.sdf'


def sources():
    """{source config: repeats} over every table (a config shared by tables flies once, at its largest n)."""
    out = {}
    for t in TABLES.values():
        for c in t['cells']:
            out[c['config']] = max(out.get(c['config'], 0), c['repeats'])
    return out


def _void_runs():
    path = os.path.join(RESULTS, 'registry.csv')
    if not os.path.isfile(path):
        return set()
    with open(path, newline='') as fh:
        return {r[0] for r in csv.reader(fh) if len(r) > 9 and 'VOID' in r[9].upper()}


def runs_of(config, voids=None):
    """Finished, non-void runs of a config (results/<date>/R####_sim_gz_<config> with metrics.json), in id order."""
    voids = _void_runs() if voids is None else voids
    out = []
    for d in glob.glob(os.path.join(RESULTS, '2*', f'R*_sim_gz_{config}')):
        rid = os.path.basename(d).split('_')[0]
        if os.path.isfile(os.path.join(d, 'metrics.json')) and rid not in voids:
            out.append(rid)
    return sorted(out)


def runset(key, which='old'):
    """{cell: [run ids]}: 'old' the 0.55 m runs as cited, '1m' the r1m_ re-fly found in the results."""
    voids = _void_runs() if which == '1m' else None
    out = {}
    for c in TABLES[key]['cells']:
        out[c['cell']] = list(c['old']) if which == 'old' else runs_of(PREFIX + c['config'], voids)
    return out


def status(key):
    """'refly' (every cell's world exists at 1 m), 'partial' or 'waits for worlds', with the missing worlds."""
    miss = sorted({m for m in (missing_world(c['config']) for c in TABLES[key]['cells']) if m})
    n = sum(bool(missing_world(c['config'])) for c in TABLES[key]['cells'])
    return ('refly' if not n else 'partial' if n < len(TABLES[key]['cells']) else 'waits for worlds'), miss


def summary(keys):
    out = []
    for k in keys:
        t = TABLES[k]
        st, miss = status(k)
        new = runset(k, '1m')
        out.append(dict(table=k, thesis=t['thesis'], card=t['card'], scorer=t['scorer'], status=st,
                        missing_worlds=miss, note=t.get('note', ''),
                        cells=[dict(c, r1m=PREFIX + c['config'], world_missing=missing_world(c['config']),
                                    new=new[c['cell']]) for c in t['cells']]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('tables', nargs='*', help=f'table keys ({", ".join(TABLES)}); default all')
    ap.add_argument('--json', action='store_true')
    o = ap.parse_args(argv)
    bad = [k for k in o.tables if k not in TABLES]
    if bad:
        ap.error(f'unknown table(s) {bad}; known: {", ".join(TABLES)}')
    rows = summary(o.tables or list(TABLES))
    if o.json:
        print(json.dumps(dict(tables=rows, history=HISTORY), indent=1))
        return 0
    for t in rows:
        miss = f' (needs {", ".join(t["missing_worlds"])})' if t['missing_worlds'] else ''
        print(f"== {t['table']}: {t['status']}{miss}\n   {t['thesis']}\n   card {t['card']}")
        if t['note']:
            print(f"   note: {t['note']}")
        for c in t['cells']:
            print(f"   {c['cell']:16s} {c['config']:26s} x{c['repeats']}  0.55 m {' '.join(c['old']):12s}  "
                  f"1 m {' '.join(c['new']) or ('-' if not c['world_missing'] else 'no world')}")
    print('\n== kept as 0.55 m history (not re-flown)')
    for what, items in HISTORY:
        print(f'   {what}: {items}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
