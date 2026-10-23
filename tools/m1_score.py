#!/usr/bin/env python3
"""tools/m1_score.py -- one registry row per combined-M1 run, printed (never written: the caller appends it with LF).

    tools/m1_score.py claim    RUN... --rev 10 [--arm TEXT --verdict TEXT --notes TEXT ...]
    tools/m1_score.py full     RUN... --rev 10                 # welded start (m1_full_*), keyed on REWELD
    tools/m1_score.py takeback RUN... --rev 10 [--no-tb5]      # the TB6 rows (the §5 take-back bars, TB4)
    tools/m1_score.py tilt     RUN... --rev 4                  # the tilt-cut card's rows
    tools/m1_score.py tb14     R1479-R1578                     # the §6.4 landing tip-over count
    tools/m1_score.py json     RUN...                          # every metric, one JSON line per run

RUN is a run directory (results/<date>/R####_*) or a run id (looked up under MDC_RESULTS_ROOT, this tree's results/
and the main tree's). Options of the row families:
  --json            print each run's JSON line before its row
  --check           compare the row's value column with the registry's row for the run ('<id>-corr' read in
                    preference to its original), field by field
  --tree TEXT       the tree in the config column (default: the manifest's 'branch <branch> <sha7>')
  --rev / --card    the card revision / the whole card column
  --arm, --baseline, --verdict, --notes, --date   the other columns (date default: the run's results/<date>/)

Metrics: tools/m1_metrics.py (definitions there). The claim and full rows keep the 10 Oct value fields in their order
(so earlier rows compare field by field) and append the rev 10 fields: his flight's body, rod-segment and ball
clearance; the take-back's rod-segment and body clearance, attitude steps, ring tilt and time to QUAD4 down; the
overflight in both windows; the ring tilt over his rejoin when nothing attached. A TB14 run's LAND reads
'tip-over (TB14)'.
"""
import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import m1_metrics as M  # noqa: E402

CLAIM_CARD = 'docs/experiments/2026-10-10_combined_m1_contact.md'
TILT_CARD = 'docs/experiments/2026-10-11_quad4_tilt_cut.md'
WORLDS = {'three_rigid_ground_rig_attach_pickup.sdf': 'rig-twin pickup world',
          'three_rigid_ground_rig_attach_welded_pickup.sdf': 'rig-twin welded pickup world',
          'three_rigid_ground_rig_attach_pickup_push.sdf': 'rig-twin pickup push world',
          'three_rigid_ground_rig_attach_welded_pickup_push.sdf': 'rig-twin welded pickup push world'}

CLAIM_METRIC = ('descend->contact (s) / contacts / contact->proof (s) / proof->mux (s) / throttle step (% hover) / '
                'QUAD4 tilt at the proof (deg) / ring tilt peak contact..proof+8 s (deg) / dip (m) / closest QUAD4 '
                'body to a carrier body, rod (m) / HOLDING lines / OFF or DETACH after the attach / trajectory held / '
                'give-up, take-back (s) / LAND')
FULL_METRIC = ('stages / contact->proof (s) / ring tilt peak contact..proof+8 s, detach window (deg) / dip (m) / '
               'QUAD4 tilt at the proof (deg) / throttle before->after the switch / closest QUAD4 body, rod to a '
               'carrier (m) / LAND')
TAIL_METRIC = ('his flight: body, rod-segment, ball clearance (m) / take-back..down: rod-segment, body clearance (m) / '
               'attitude step at take-back, decouple (deg) / ring tilt max take-back..down (deg) / take-back->QUAD4 '
               'down (s) / overflight his flight, take-back (s); min tip-to-carrier gap (m) / ring tilt peak over his '
               'rejoin, no attach (deg)')
TB_METRIC = ('zone / give-up, decouple phase (deg in the lap) / rod-segment, body clearance take-back..down (m) / TB5 '
             'prediction rod, body (m) / attitude step at take-back, decouple (deg) / ring tilt max take-back..down '
             '(deg) / overflight (s), min tip-to-carrier gap (m) / take-back->QUAD4 down (s) / contacts / attach '
             'declared / fleet stop / LAND')
TILT_METRIC = ('injection->TILT_FAULT (s) / TILT_FAULT->MUX:cut (s) / MUX:cut->d3 disarmed (s) / his LANDED_DISARMED / '
               'FAULT_LAND count / LAND / injection / injection->FAULT_LAND (s) / injection->d3 disarmed (s) / '
               'tip-over in the LAND unwind (tilt rev 4 9.2)')


def f(v, n=2):
    """'-' for nothing; floats at n decimals; anything else as text (the 10 Oct scorers' format)."""
    if v is None:
        return '-'
    return f'{v:.{n}f}' if isinstance(v, float) else str(v)


def q(s):
    return '"' + str(s).replace('"', "'").replace('\n', ' ') + '"'


def q_min(s):
    s = str(s).replace('"', "'").replace('\n', ' ')
    return q(s) if ',' in s else s


# ── value columns ────────────────────────────────────────────────────────────────────────────────────────────────

def tail(r, c, fl, tb, both_over=True):
    gaps = [g for g in (fl.get('overflight_gap_min'), tb.get('overflight_gap_min')) if g is not None]
    return [f"{f(fl.get('body_min'), 3)}, {f(fl.get('rod_min'), 3)}, {f(fl.get('ball_min'), 3)}",
            f"{f(tb.get('rod_min'), 3)}, {f(tb.get('body_min'), 3)}" if tb.get('take_back') is not None else '-',
            f"{f(tb.get('att_step_take_back'))}, {f(tb.get('att_step_decouple'))}" if tb.get('take_back') is not None
            else '-',
            f(tb.get('ring_tilt_max')),
            f(tb.get('down_s'), 1),
            f"{f(fl.get('overflight_s'))}, {f(tb.get('overflight_s'))}; {f(min(gaps) if gaps else None, 3)}",
            f(c.get('ring_tilt_rejoin_deg'))]


def claim_value(r, tb5=False):
    c, a, fl = M.contact(r), M.authority(r), M.flight(r)
    tb, t14 = M.take_back(r, tb5=tb5), M.tb14(r)
    val = [f(c.get('descend_to_contact_s')), str(c.get('tries')), f(c.get('contact_to_proof_s')),
           f(c.get('proof_to_mux_s'), 3), f(c.get('thr_step_pct_hover'), 1), f(c.get('q4_tilt_at_confirm_deg')),
           f(c.get('ring_tilt_peak_contact_to_confirm8_deg')), f(c.get('dip_m'), 4),
           f"{f(fl['legacy_body'])}, {f(fl['legacy_rod'])}", str(a['holding_lines']),
           str(a['off_or_detach_after_attach']), str(a['trajectory_held']),
           f"give-up {f(a['give_up'])}, take-back {f(a['take_back'])}", M.land_text(r, t14)]
    data = {'contact': c, 'authority': a, 'flight': fl, 'take_back': tb, 'tb14': t14}
    return val + tail(r, c, fl, tb), CLAIM_METRIC + ' / ' + TAIL_METRIC, data


def full_value(r, tb5=False):
    c, s, fl = M.contact(r), M.stages(r), M.flight(r)
    tb, t14 = M.take_back(r, tb5=tb5), M.tb14(r)
    thr = (f"{c['thr_before']:.3f} -> {c['thr_after']:.3f}" if c.get('thr_before') is not None else '-')
    val = [M.stage_text(s), f(c.get('contact_to_proof_s')),
           f"{f(c.get('ring_tilt_peak_contact_to_confirm8_deg'))}, {f(s.get('ring_tilt_detach_deg'))}",
           f(c.get('dip_m'), 4), f(c.get('q4_tilt_at_confirm_deg')), thr,
           f"{f(fl['legacy_body'])}, {f(fl['legacy_rod'])}", M.land_text(r, t14)]
    data = {'contact': c, 'stages': s, 'authority': M.authority(r), 'flight': fl, 'take_back': tb, 'tb14': t14}
    return val + tail(r, c, fl, tb), FULL_METRIC + ' / ' + TAIL_METRIC, data


def takeback_value(r, tb5=True):
    d, t14 = M.take_back(r, tb5=tb5), M.tb14(r)
    tb5_txt = '-'
    if d.get('tb5_rod_min') is not None:
        tb5_txt = (f"{f(d['tb5_rod_min'], 3)}, {f(d['tb5_body_min'], 3)} (gap {f(d.get('tb5_gap_rod'), 3)}, "
                   f"{f(d.get('tb5_gap_body'), 3)}; fit {f(d.get('tb5_fit_rms'), 3)}; ring after decouple >= "
                   f"{f(d.get('tb5_after_decouple_ring_xy_min'), 2)})")
    val = [d.get('zone') or '-', f(d.get('take_back_phase_deg'), 1) + ', ' + f(d.get('decouple_phase_deg'), 1),
           f(d.get('rod_min'), 3) + ', ' + f(d.get('body_min'), 3), tb5_txt,
           f(d.get('att_step_take_back')) + ', ' + f(d.get('att_step_decouple')), f(d.get('ring_tilt_max')),
           f(d.get('overflight_s')) + ', ' + f(d.get('overflight_gap_min'), 3), f(d.get('down_s'), 1),
           str(d.get('contacts')), 'yes' if d.get('attach') is not None else 'no',
           'yes' if d.get('emergency') else 'no', M.land_text(r, t14)]
    return val, TB_METRIC, {'take_back': d, 'tb14': t14}


def tilt_value(r, tb5=False):
    d, t14 = M.tilt(r), M.tb14(r)
    f3 = lambda v: f(v, 3)  # noqa: E731
    inj = f"{d['injection']} {d['injection_t']:.2f} s ({d['injection_arg']})" if d['injection'] else '-'
    val = [f3(d['inj_to_tilt_fault_s']), f3(d['tilt_fault_to_cut_s']), f3(d['cut_to_d3_disarm_s']),
           'yes' if d['landed_disarmed'] else 'no', str(d['fault_land_count']), d['land'],
           inj, f3(d['inj_to_fault_land_s']), f3(d['inj_to_d3_disarm_s']), 'yes' if t14['unwind_tipover'] else 'no']
    return val, TILT_METRIC, {'tilt': d, 'tb14': t14}


FAMILIES = {'claim': claim_value, 'full': full_value, 'takeback': takeback_value, 'tilt': tilt_value}


# ── the row ──────────────────────────────────────────────────────────────────────────────────────────────────────

def tree_text(r):
    m = r.manifest
    if not m.get('git_sha'):
        return None
    t = f"branch {m.get('git_branch', '?')} {m['git_sha'][:7]}"
    if m.get('dirty_files'):
        t += f" + {m['dirty_files']} dirty files (diff {str(m.get('diff_sha256', ''))[:8]})"
    return t


def config_text(r, tree):
    m = r.manifest
    kind = 'Gazebo' if m.get('harness', 'run_experiment.py') == 'run_experiment.py' else m.get('harness')
    host = 'LAB PC' if m.get('lab') else 'laptop'
    world = WORLDS.get(m.get('world', ''), (m.get('world') or 'world ?').replace('.sdf', ''))
    return f'{r.config} ({kind}, {host}, {world}; {tree})'


def card_text(family, rev, card):
    if card:
        return card
    if rev is None:
        raise SystemExit('a row needs --rev (or --card)')
    return {'claim': f'{CLAIM_CARD} (claim rev {rev})',
            'full': f'{CLAIM_CARD} (claim rev {rev}, full M1 beside the table)',
            'takeback': f'{CLAIM_CARD} (rev {rev} TB6)',
            'tilt': f'{TILT_CARD} (rev {rev})'}[family]


def previous_same_config(r, registry):
    """The newest registry run before this one with the same config (the full M1's baseline is its predecessor)."""
    n = int(r.id[1:]) if r.id[1:].isdigit() else None
    best = None
    for rid, row in registry.items():
        if not rid[1:].isdigit() or n is None or int(rid[1:]) >= n:
            continue
        if row.get('config', '').split(' (')[0].strip() == r.config and (best is None or int(rid[1:]) > int(best[1:])):
            best = rid
    return best


def baseline_text(family, r, registry):
    if family == 'claim':
        return 'R1479-R1490'                 # the card's §4 baseline (E0 / E1)
    if family == 'takeback':
        return 'R1565'                       # §6.2: the arm that failed
    if family == 'tilt':
        return 'T0'
    return (previous_same_config(r, registry) if registry else None) or '-'


def row(family, r, val, metric, a, registry=None):
    tree = a.tree or tree_text(r)
    if tree is None:
        raise SystemExit(f'{r.id}: no git state in the manifest; pass --tree')
    fields = [r.id, a.date or r.date, q(config_text(r, tree)), q(card_text(family, a.rev, a.card)),
              q(a.arm or r.config), q_min(a.baseline or baseline_text(family, r, registry)), q(metric),
              q(' / '.join(val)), '', q(a.verdict or ''), '', q(a.notes or '')]
    return ','.join(fields)


# ── --check: the value column against the registry ───────────────────────────────────────────────────────────────

NUM = re.compile(r'-?\d+\.\d+|-?\d+')


def compare(val, reg_value):
    """[(field, registry text, tool text, verdict)] over the registry's value fields: 'same' (text), 'same numbers'
    (the registry's numbers all appear in the tool's field, e.g. a hand-written stage list), else 'DIFF'."""
    reg = [x.strip() for x in reg_value.split(' / ')]
    out = []
    for i, rv in enumerate(reg):
        tv = val[i] if i < len(val) else ''
        if rv == tv:
            v = 'same'
        else:
            rn, tn = NUM.findall(rv), NUM.findall(tv)
            v = 'same numbers' if rn and all(x in tn for x in rn) else 'DIFF'
        out.append((i, rv, tv, v))
    return out


def check_lines(r, val, registry):
    reg = registry.get(r.id)
    if reg is None:
        return [f'{r.id}: no registry row'], 0
    names = [x.strip() for x in reg.get('metric', '').split(' / ')]
    lines, bad = [], 0
    src = f"{r.id}{'-corr' if reg['_corr'] else ''}"
    for i, rv, tv, v in compare(val, reg.get('value', '')):
        name = names[i] if i < len(names) else f'field {i + 1}'
        lines.append(f'{src:11s} {name[:58]:58s} registry {rv[:48]:48s} tool {tv[:60]:60s} {v}')
        bad += v == 'DIFF'
    return lines, bad


# ── TB14 over a range ────────────────────────────────────────────────────────────────────────────────────────────

def expand(ids):
    out = []
    for x in ids:
        m = re.fullmatch(r'R(\d+)-R?(\d+)', x)
        if m:
            out += [f'R{i}' for i in range(int(m.group(1)), int(m.group(2)) + 1)]
        else:
            out.append(x)
    return out


def tb14_count(ids, out=sys.stdout):
    """Per run welded at LAND: the §6.4 parts and the verdict; then the counts, subtest runs and full M1 apart."""
    tally = {'subtest': [0, 0, []], 'full M1': [0, 0, []]}
    for rid in expand(ids):
        try:
            r = M.Run(rid)
            d = M.tb14(r)
        except (SystemExit, FileNotFoundError, KeyError, ValueError, IndexError):
            continue                                    # no run, or a void run with no logs
        if not d['welded_at_land']:
            continue
        fam = 'full M1' if r.config.startswith('m1_full') else 'subtest'
        tally[fam][1] += 1
        if d['tb14']:
            tally[fam][0] += 1
            tally[fam][2].append(r.id)
        print(f"{r.id} {r.config[:28]:28s} abort {f(d.get('abort_after_land_s'), 1):>5s} s after LAND  tip down "
              f"{f(d.get('tip_down_t'))}  60 deg {f(d.get('tilt60_t'))} (rise {f(d.get('tilt_rise_s'))} s, body "
              f"{f(d.get('body_z_at_60'), 3)} m)  TB14 {'yes' if d['tb14'] else 'no'}  (F10 literal "
              f"{'yes' if d['tb14_f10'] else 'no'}; tip min {f(d.get('tip_z_min_after_land'), 3)} m)", file=out)
    for fam, (k, n, ids_) in tally.items():
        print(f"TB14 {fam}: {k} of {n} welded at LAND" + (f": {', '.join(ids_)}" if ids_ else ''), file=out)
    return tally


# ── main ─────────────────────────────────────────────────────────────────────────────────────────────────────────

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0], epilog=__doc__.split('\n\n', 1)[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('family', choices=sorted(FAMILIES) + ['tb14', 'json'])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json', action='store_true', help="print each run's JSON line before its row")
    ap.add_argument('--check', action='store_true', help='compare the value column with the registry row')
    ap.add_argument('--registry', default=None, help='registry.csv (default: the live one, see m1_metrics)')
    ap.add_argument('--tree')
    ap.add_argument('--rev')
    ap.add_argument('--card')
    ap.add_argument('--arm')
    ap.add_argument('--baseline')
    ap.add_argument('--verdict')
    ap.add_argument('--notes')
    ap.add_argument('--date')
    ap.add_argument('--no-tb5', action='store_true', help='takeback: skip the TB5 replay (needs the ROS environment)')
    a = ap.parse_args(argv)

    if a.family == 'tb14':
        tb14_count(a.runs)
        return 0
    if a.family == 'json':
        for rid in a.runs:
            r = M.Run(rid)
            print(json.dumps({'run': r.id, 'date': r.date, 'config': r.config, 'contact': M.contact(r),
                              'authority': M.authority(r), 'stages': M.stages(r), 'flight': M.flight(r),
                              'take_back': M.take_back(r, tb5=not a.no_tb5), 'tilt': M.tilt(r), 'tb14': M.tb14(r)},
                             default=str))
        return 0

    registry = None
    if a.check or (a.family == 'full' and not a.baseline):
        p = a.registry or M.registry_path()
        registry = M.read_registry(p) if p else {}
    bad = 0
    for rid in a.runs:
        r = M.Run(rid)
        kw = {'tb5': not a.no_tb5} if a.family == 'takeback' else {}
        val, metric, data = FAMILIES[a.family](r, **kw)
        if a.json:
            print(json.dumps({'run': r.id, 'date': r.date, 'config': r.config, **data}, default=str))
        if a.check:
            lines, b = check_lines(r, val, registry)
            print('\n'.join(lines))
            bad += b
        else:
            print(row(a.family, r, val, metric, a, registry))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
