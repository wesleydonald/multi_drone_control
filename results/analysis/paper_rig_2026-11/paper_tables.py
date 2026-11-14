#!/usr/bin/env python3
"""The paper's real-world tables from paper_rig.py's CSVs: rows = metrics, columns = conditions, each cell
"mean (min-max), n" (rates "k/n"), in Markdown and LaTeX (booktabs), plus a long-form CSV.

    python3 results/analysis/paper_rig_2026-11/paper_tables.py          # -> tables.md, tables.tex, tables.csv

Which rows count (Wesley decides; change MAIN below or exclude a flight in conditions.csv):
- the main tables: 1 m cables and the planner's ring config at 0.86 kg (the M2A ring flown since 4 Nov); a flight the
  registry calls VOID or conditions.csv excludes is left out; every flight left out is listed with its reason.
- the 8 Oct unannounced releases (0.55 m links, the code of that day) go in a context table: the method's check
  against the registry's numbers, not the paper's setup.
- columns: detach by (unannounced | planned) x (static | moving) x formation; attach by (static | moving) x formation x
  funnel, with the attach from the mat and the re-attach after a pickup apart; the application by static | moving.
"""
import csv
import math
import os
import re
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = {'cable_m': 1.0, 'load_kg_config': 0.86}

DETACH_ROWS = [
    # (key, label, decimals, applies to: None = all, 'unannounced' / 'planned', 'static' / 'moving')
    ('success', 'Success rate', None, None),
    ('detect_delay_s', 'Detection delay after the magnet OFF (s)', 2, 'unannounced'),
    ('resize_s', 'Planner resize after the OFF (s)', 2, None),
    ('separation_s', 'Magnet off its plate after the OFF (s)', 2, None),
    ('unload_s', 'Unload before the OFF (s)', 1, 'planned'),
    ('tilt_pre_deg', 'Ring tilt before (deg)', 1, None),
    ('swing_deg', 'Ring swing: tilt change from before, peak (deg)', 1, None),
    ('tilt_peak_deg', 'Ring tilt peak (deg)', 1, None),
    ('settle_s', 'Time to settle back to the tilt before (s)', 1, None),
    ('settle_new_s', 'Time to settle at the new tilt (s)', 1, None),
    ('tilt_shift_deg', 'Tilt after minus before (deg)', 1, None),
    ('dip_mm', 'Height dip (mm)', 0, None),
    ('xy_excursion_cm', 'Horizontal excursion (cm)', 1, 'static'),
    ('sp_pre_rms_cm', 'Set-point deviation before, RMS (cm)', 1, None),
    ('sp_rms_cm', 'Set-point deviation 10 s after, RMS (cm)', 1, None),
    ('path_rms_cm', 'Path deviation 10 s after, RMS (cm)', 1, 'moving'),
]
ATTACH_ROWS = [
    ('success', 'Success rate (proven, folded in, held 15 s)', None, None),
    ('success_desc', 'Success rate, flights with a descent', None, None),
    ('descents', 'Descents per flight', 1, None),
    ('descents_to_catch', 'Descents to the catch', 1, None),
    ('d_at_height', 'Descents reaching the plate height', None, None),
    ('d_contact', 'Descents with a contact', None, None),
    ('miss', 'Miss at plate height, per descent (cm; median)', 1, None),
    ('start_to_proven_s', 'Release to proven attach (s)', 1, None),
    ('start_to_foldin_s', 'Release to fold-in (s)', 1, None),
    ('contact_to_foldin_s', 'Contact to fold-in (s)', 1, None),
    ('tilt_pre_deg', 'Ring tilt before the fold-in (deg)', 1, None),
    ('tilt_after_deg', 'Ring tilt after the fold-in (deg)', 1, None),
    ('tilt_peak_deg', 'Ring tilt peak (deg)', 1, None),
    ('settle_new_s', 'Time to settle at the new tilt (s)', 1, None),
    ('dip_mm', 'Height dip at the fold-in (mm)', 0, None),
    ('sp_rms_cm', 'Set-point deviation 10 s after, RMS (cm)', 1, None),
    ('path_rms_cm', 'Path deviation 10 s after, RMS (cm)', 1, 'moving'),
]
PHASES = ['lift', 'detach', 'pickup', 'transit', 'drop', 're-attach', 'land']


def load(name):
    fn = os.path.join(HERE, name)
    if not os.path.exists(fn):
        return []
    with open(fn, newline='') as f:
        return list(csv.DictReader(f))


def num(r, k):
    try:
        return float(r.get(k, ''))
    except (TypeError, ValueError):
        return math.nan


def is_main(r):
    return (abs(num(r, 'cable_m') - MAIN['cable_m']) < 0.05 and abs(num(r, 'load_kg_config') - MAIN['load_kg_config']) < 0.01)


def left_out(r):
    if r.get('void') == 'yes':
        return 'VOID in the registry'
    if r.get('exclude'):
        return 'excluded in conditions.csv: ' + (r.get('cond_note') or r['exclude'])
    if not is_main(r):
        return f"not the main setup (links {r.get('cable_m')} m, ring config {r.get('load_kg_config')} kg)"
    return ''


def cell(vals, nd, median=False):
    v = [x for x in vals if math.isfinite(x)]
    if not v:
        return '-', None
    c = st.median(v) if median else sum(v) / len(v)
    c, v = c + 0.0 if round(c, nd) != 0 else 0.0, [x if round(x, nd) != 0 else 0.0 for x in v]
    if len(v) == 1:
        return f'{c:.{nd}f}, n=1', (c, c, c, 1)
    return f'{c:.{nd}f} ({min(v):.{nd}f} to {max(v):.{nd}f}), n={len(v)}', (c, min(v), max(v), len(v))


def rate(k, n):
    return (f'{k}/{n}' if n else '-'), (k / n if n else None, None, None, n)


def det_col(r):
    kind = 'Unannounced' if 'unannounced' in r['event'] else 'Planned'
    return f"{kind}, {r['motion']}, {r['formation']}"


def att_col(r):
    if r['flight_kind'] == 'attach from the mat':
        lead = 'From the mat'
    elif r['flight_kind'] == 'application':
        lead = 'After the pickup'
    else:
        lead = 'Re-attach'
    f = {'yes': 'funnel', 'no': 'no funnel'}.get(r['funnel'], 'funnel ?')
    usual = {'triangle': '9', 'square': '8'}.get(r['formation'])
    plate = '' if (r.get('plate') or usual) == usual else f", plate {r.get('plate')}"
    return f"{lead}, {r['motion']}, {r['formation']}, {f}{plate}"


def order(cols, first):
    return sorted(cols, key=lambda c: (0 if c in first else 1, first.index(c) if c in first else 0, c))


def detach_table(ev, context=False):
    rows = [r for r in ev if r['event'].startswith('detach') and not left_out(r)] if not context else \
        [r for r in ev if r['event'].startswith('detach') and abs(num(r, 'cable_m') - 0.55) < 0.05 and not r.get('void')]
    need = [] if context else ['Unannounced, static, triangle', 'Unannounced, moving, triangle']
    cols = order(set(det_col(r) for r in rows) | set(need), need + ['Planned, static, triangle',
                                                                     'Planned, moving, triangle'])
    out = {'flights': {}}
    for c in cols:
        rs = [r for r in rows if det_col(r) == c]
        out['flights'][c] = (str(len(rs)), (len(rs), None, None, len(rs)))
        for key, label, nd, only in DETACH_ROWS:
            if only in ('unannounced', 'planned') and only not in c.lower():
                out.setdefault(key, {})[c] = ('', None)
                continue
            if only in ('static', 'moving') and f', {only},' not in c:
                out.setdefault(key, {})[c] = ('', None)
                continue
            if key == 'success':
                out.setdefault(key, {})[c] = rate(sum(1 for r in rs if r['success'] == 'yes'), len(rs))
            elif key in ('settle_s', 'settle_new_s'):
                have = [r for r in rs if r.get('pre_s') and num(r, 'pre_s') >= 1.0 and
                        (key == 'settle_s' or num(r, 'window_s') >= 10.0)]
                txt, val = cell([num(r, key) for r in have], nd)
                k = sum(1 for r in have if math.isfinite(num(r, key)))
                out.setdefault(key, {})[c] = ((txt + f'; settled {k}/{len(have)}') if have else '-', val)
            else:
                out.setdefault(key, {})[c] = cell([num(r, key) for r in rs], nd)
    labels = [('flights', 'Flights')] + [(k, lab) for k, lab, _, _ in DETACH_ROWS]
    return cols, labels, out, rows


def attach_table(ev, descents):
    rows = [r for r in ev if r['event'] == 'attach' and not left_out(r)]
    need = ['Re-attach, static, triangle, no funnel', 'Re-attach, static, triangle, funnel',
            'Re-attach, moving, triangle, no funnel', 'Re-attach, moving, triangle, funnel']
    cols = order(set(att_col(r) for r in rows) | set(need), need + [
        'Re-attach, static, triangle, no funnel, plate 1', 'Re-attach, static, square, no funnel',
        'After the pickup, static, triangle, no funnel', 'From the mat, static, triangle, no funnel'])
    dmap = {}
    for d in descents:
        if d['kind'] in ('re-attach', 'attach from the mat'):
            dmap.setdefault((d['day'], d['flight']), []).append(d)
    out = {'flights': {}}
    for c in cols:
        rs = [r for r in rows if att_col(r) == c]
        out['flights'][c] = (str(len(rs)), (len(rs), None, None, len(rs)))
        ds = []
        for r in rs:
            dd = sorted(dmap.get((r['day'], r['flight']), []), key=lambda d: float(d['descent_start_s']))
            n = int(num(r, 'descents')) if math.isfinite(num(r, 'descents')) else 0
            ds += dd[:n] if r['success'] == 'yes' else dd[:n] or dd
        for key, label, nd, only in ATTACH_ROWS:
            if only in ('static', 'moving') and f', {only},' not in c:
                out.setdefault(key, {})[c] = ('', None)
                continue
            if key == 'success':
                v = rate(sum(1 for r in rs if r['success'] == 'yes'), len(rs))
            elif key == 'success_desc':
                rd = [r for r in rs if num(r, 'descents') > 0]
                v = rate(sum(1 for r in rd if r['success'] == 'yes'), len(rd))
            elif key == 'descents_to_catch':
                v = cell([num(r, 'descents') for r in rs if r['success'] == 'yes'], nd)
            elif key == 'd_at_height':
                v = rate(sum(1 for d in ds if d['eval'] == 'plate height'), len(ds))
            elif key == 'd_contact':
                v = rate(sum(1 for d in ds if d.get('contact_log_xy_mm', '') != ''), len(ds))
            elif key == 'miss':
                v = cell([num(d, 'tip_xy_cm') for d in ds if d['eval'] == 'plate height'], nd, median=True)
            elif key in ('settle_new_s',):
                have = [r for r in rs if r['success'] == 'yes' and num(r, 'pre_s') >= 1.0 and num(r, 'window_s') >= 10.0]
                txt, val = cell([num(r, key) for r in have], nd)
                k = sum(1 for r in have if math.isfinite(num(r, key)))
                v = ((txt + f'; settled {k}/{len(have)}') if have else '-', val)
            else:
                v = cell([num(r, key) for r in rs], nd)
            out.setdefault(key, {})[c] = v
    labels = [('flights', 'Flights')] + [(k, lab) for k, lab, _, _ in ATTACH_ROWS]
    return cols, labels, out, rows


def app_table(ph):
    rows = [r for r in ph if not left_out(r)]
    lead_of = {'application': 'Application', 'pickup and drop': 'Pickup and drop only',
               'delivery and rejoin': 'Delivery and rejoin (object held)'}
    ev_of = {v: k for k, v in lead_of.items()}
    cols = order(set(f"{lead_of[r['event']]}, {r['motion']}" for r in rows) | {'Application, static', 'Application, moving'},
                 ['Application, static', 'Application, moving', 'Pickup and drop only, static',
                  'Pickup and drop only, moving'])
    out = {'flights': {}, 'completed': {}, 'pickup_attempts': {}, 'in_net': {}, 'total_s': {}, 'failed_in': {}}
    for c in cols:
        lead, motion = c.rsplit(', ', 1)
        ev = ev_of[lead]
        rs = [r for r in rows if r['event'] == ev and r['motion'] == motion]
        out['flights'][c] = (str(len(rs)), (len(rs), None, None, len(rs)))
        out['completed'][c] = rate(sum(1 for r in rs if r['completed'] == 'yes'), len(rs))
        for p in PHASES:
            if (ev != 'application' and p == 'detach') or (ev == 'pickup and drop' and p == 're-attach') or \
                    (ev == 'delivery and rejoin' and p == 'pickup'):
                out.setdefault(p, {})[c] = ('', None)
                continue
            started = [r for r in rs if r.get(f'{p}_start_s', '') != '']
            done = [num(r, f'{p}_s') for r in started if math.isfinite(num(r, f'{p}_s'))]
            txt, val = cell(done, 1)
            out.setdefault(p, {})[c] = (f'{len(done)}/{len(started)} done; ' + txt if started else '-', val)
        out['pickup_attempts'][c] = cell([num(r, 'pickup_attempts') for r in rs if r.get('pickup_start_s')], 1) \
            if ev != 'delivery and rejoin' else ('', None)
        dr = [r for r in rs if r.get('drop_outcome')]
        out['in_net'][c] = rate(sum(1 for r in dr if r['drop_outcome'] == 'hit the net'), len(dr))
        out['total_s'][c] = cell([num(r, 'total_s') for r in rs if r['completed'] == 'yes'], 1)
        fails = {}
        for r in rs:
            if r['completed'] != 'yes':
                fails[r['failed_in']] = fails.get(r['failed_in'], 0) + 1
        out['failed_in'][c] = (', '.join(f'{k} {v}' for k, v in sorted(fails.items())) or '-', None)
    labels = [('flights', 'Flights'), ('completed', 'Missions completed (landed, nothing lost)')] + \
        [(p, f'{p.capitalize()} phase (s)') for p in PHASES] + \
        [('pickup_attempts', 'Pickup attempts'), ('in_net', 'Object into the net'),
         ('total_s', 'Mission time, completed (s)'), ('failed_in', 'Where the others stopped')]
    return cols, labels, out, rows


def md(title, cols, labels, out):
    head = '| Metric | ' + ' | '.join(cols) + ' |'
    lines = [f'### {title}', '', head, '|' + '---|' * (len(cols) + 1)]
    for key, lab in labels:
        if all(out.get(key, {}).get(c, ('', None))[0] in ('', '-') for c in cols) and key != 'flights':
            continue
        lines.append(f'| {lab} | ' + ' | '.join(out.get(key, {}).get(c, ('', None))[0] or ' ' for c in cols) + ' |')
    return '\n'.join(lines) + '\n'


def tex_escape(s, is_cell=False):
    s = s.replace('\\', r'\textbackslash{}').replace('%', r'\%').replace('&', r'\&').replace('_', r'\_')
    if not is_cell:
        return s
    if s.strip() == '-':
        return '--'
    s = re.sub(r'(?<![\w.])-(?=\d)', '\u2212', s)         # a minus sign, not a dash
    s = s.replace(' to \u2212', '\u0000').replace(' to ', '--').replace('\u0000', ' to \u2212').replace('\u2212', '$-$')
    return s.replace('n=', '$n{=}$')


def tex(label, caption, cols, labels, out):
    spec = 'l' + 'c' * len(cols)
    lines = [r'\begin{table}[H]', r'\centering', r'\caption{' + caption + '}', r'\label{' + label + '}',
             r'\resizebox{\textwidth}{!}{%', r'\begin{tabular}{' + spec + '}', r'\toprule',
             'Metric & ' + ' & '.join(r'\shortstack{' + tex_escape(c).replace(', ', r'\\') + '}' for c in cols) + r' \\',
             r'\midrule']
    for key, lab in labels:
        if all(out.get(key, {}).get(c, ('', None))[0] in ('', '-') for c in cols) and key != 'flights':
            continue
        lines.append(tex_escape(lab) + ' & ' + ' & '.join(tex_escape(out.get(key, {}).get(c, ('', None))[0] or '', True)
                                                         for c in cols) + r' \\')
    lines += [r'\bottomrule', r'\end{tabular}}', r'\end{table}', '']
    return '\n'.join(lines)


def main():
    ev = load('events.csv')
    ph = load('phases.csv')
    descents = load(os.path.join('joins', 'joins.csv'))
    D = detach_table(ev)
    A = attach_table(ev, descents)
    P = app_table(ph)
    C = detach_table(ev, context=True)
    parts_md, parts_tex, long = [], [], []
    for name, label, cap, (cols, labels, out, rows) in (
            ('Detach performance (real world)', 'tab:detach_performance',
             'Detachment on the real system: the unannounced release (the magnet switched off, the planner not told) and '
             'our planned detachment.', D),
            ('Attach performance (real world)', 'tab:attach_performance',
             'Attachment on the real system: our planned detachment, then the returning quadrotor attaches on its plate.',
             A),
            ('Application (real world): the full mission in one flight', 'tab:application',
             'The full mission on the real system: detachment, pickup, drop into the net, attachment, landing.', P),
            ('Context: the unannounced releases of 8 Oct (0.55 m links, that day\'s code)', 'tab:detach_context_0p55',
             'Unannounced releases of 8 Oct on 0.55 m links.', C)):
        parts_md.append(md(name, cols, labels, out))
        parts_tex.append(tex(label, cap, cols, labels, out))
        for key, lab in labels:
            for c in cols:
                txt, val = out.get(key, {}).get(c, ('', None))
                if not txt:
                    continue
                m, lo, hi, n = val if val else (None, None, None, None)
                long.append({'table': label, 'metric': lab, 'key': key, 'condition': c, 'cell': txt, 'mean': m,
                             'min': lo, 'max': hi, 'n': n})
    # every flight left out, and why
    seen, outs = set(), []
    for r in ev + ph:
        why = left_out(r)
        key = (r['day'], r['flight'])
        if why and key not in seen:
            seen.add(key)
            outs.append(f"- {r['day']}/{r['flight']} ({r['flight_kind']}): {why}")
    flights = load('flights.csv')
    for f in flights:
        key = (f['day'], f['flight'])
        if key not in seen and f.get('events') in ('0', ''):
            seen.add(key)
            outs.append(f"- {f['day']}/{f['flight']} ({f['flight_kind']}): no detach, attach or mission event"
                        + (' (VOID in the registry)' if f.get('void') == 'yes' else ''))
    text = '\n'.join(parts_md) + '\n### Flights left out of the main tables\n\n' + ('\n'.join(outs) or '- none') + '\n'
    with open(os.path.join(HERE, 'tables.md'), 'w') as f:
        f.write(text)
    with open(os.path.join(HERE, 'tables.tex'), 'w') as f:
        f.write('% made by paper_tables.py from events.csv / phases.csv (paper_rig.py); needs booktabs, graphicx, float\n')
        f.write('\n'.join(parts_tex))
    with open(os.path.join(HERE, 'tables.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['table', 'metric', 'key', 'condition', 'cell', 'mean', 'min', 'max', 'n'],
                           lineterminator='\n')
        w.writeheader()
        for r in long:
            w.writerow({k: ('' if v is None else (f'{v:.4g}' if isinstance(v, float) else v)) for k, v in r.items()})
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
