"""tools/make_catch_offset_configs.py (sim U, card docs/experiments/2026-10-23_catch_offset.md): the configs on disk
are the generated ones, and each comparison of the card differs in its one variable only."""
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import make_catch_offset_configs as G  # noqa: E402


def _flat(cfg):
    """{dotted key: value} without the name and description."""
    out = {}

    def walk(p, v):
        if isinstance(v, dict):
            for k, x in v.items():
                walk(f'{p}.{k}' if p else k, x)
        else:
            out[p] = v
    walk('', {k: v for k, v in cfg.items() if k not in ('name', 'description')})
    return out


def _diff(a, b):
    fa, fb = _flat(a), _flat(b)
    return sorted(k for k in set(fa) | set(fb) if fa.get(k) != fb.get(k))


def _cfgs():
    return {c['name']: c for c in (G.config(*v) for v in G.variants())}


def test_check_passes_and_every_file_reads_back_as_generated():
    assert G.main(['--check']) == 0
    for name, cfg in _cfgs().items():
        with open(os.path.join(G.OUT, f'{name}.yaml')) as fh:
            assert yaml.safe_load(fh) == cfg


def test_the_grid_neighbours_and_extension():
    names = set(_cfgs())
    grid = {f'u_{a}_{o}' for a in ('meas', 'slot')
            for o in ['d00', 't116'] + [f'{d}{cm:02d}' for d in ('t', 'ri') for cm in (2, 4, 6, 8, 12, 16)]}
    assert grid <= names and len(grid) == 28
    assert names - grid == {'u_meas_ta08', 'u_meas_ro08', 'u_slot_ro08', 'u_meas_t20', 'u_slot_t20', 'u_meas_ri20',
                            'u_slot_ri20'}


def test_one_variable_per_comparison():
    c = _cfgs()
    for name in c:
        if name.startswith('u_meas_') and name.replace('u_meas_', 'u_slot_') in c:
            assert _diff(c[name], c[name.replace('u_meas_', 'u_slot_')]) == ['launch.args.attach_point_source']
    for arm in ('meas', 'slot'):
        base = c[f'u_{arm}_d00']
        for name, cfg in c.items():
            if not name.startswith(f'u_{arm}_') or name.endswith('d00'):
                continue
            knob = 'catch_offset_t' if name.split('_')[-1][0] == 't' else 'catch_offset_r'
            assert _diff(base, cfg) == [f'launch.args.{knob}'], name


def test_offsets_signs_and_the_flown_figure_8():
    c = _cfgs()
    a = lambda n: c[n]['launch']['args']  # noqa: E731
    assert (a('u_meas_t04')['catch_offset_t'], a('u_meas_ta08')['catch_offset_t']) == (0.04, -0.08)
    assert (a('u_slot_ri16')['catch_offset_r'], a('u_slot_ro08')['catch_offset_r']) == (-0.16, 0.08)
    assert a('u_meas_t116')['catch_offset_t'] == a('u_slot_t116')['catch_offset_t'] == 0.11647   # plate 10's centre
    assert math.isclose(2.0 * math.asin(0.11647 / 0.45), math.radians(30.0), abs_tol=1e-4)
    for cfg in c.values():
        args = cfg['launch']['args']
        assert (args['load_traj'], args['fig8_size'], args['traj_speed']) == ('fig_8', 1.0, 0.2)
        assert (args['weld_radius'], args['weld_point_z'], args['weld_speed_gate']) == (0.025, 0.04, 0.05)
        assert 'attach_cable_len' not in args and 'traj_radius' not in args
