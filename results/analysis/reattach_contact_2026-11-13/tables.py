#!/usr/bin/env python3
"""Markdown tables for summary.md from events.csv, flights.csv and verify.csv. usage: tables.py > tables.md"""
import csv, os

HERE = os.path.dirname(os.path.abspath(__file__))
ev = list(csv.DictReader(open(os.path.join(HERE, 'events.csv'))))
fl = list(csv.DictReader(open(os.path.join(HERE, 'flights.csv'))))
ve = list(csv.DictReader(open(os.path.join(HERE, 'verify.csv'))))


def short(s):
    if not s:
        return ''
    if s.startswith('same'):
        return 'same'
    if s.startswith('later by'):
        return s.split(' (')[0]
    if s.startswith('later,'):
        return 'waits past the real contact'
    if s.startswith('no'):
        return 'no (not by +2 s)'
    return s


print('### Every contact event\n')
print('| flight | try | gate typed (mm) | xy (mm) | normal at contact (mm) | c0 normal (mm) | dwell normal min..max (mm) '
      '| min normal next 2 s (mm) | outcome | held | his phase at contact | his LIFT_PROOF after | 0.04 | 0.02 | 0.01 '
      '(delay, normal) |')
print('|' + '---|' * 15)
for e in ev:
    d01 = short(e['fires_0.01'])
    if e['delay_0.01_s'] and not e['fires_0.01'].startswith('same'):
        d01 += f" ({e['delay_0.01_s']} s, {e['normal_0.01_mm']} mm)"
    d02 = short(e['fires_0.02'])
    if e['delay_0.02_s'] and not e['fires_0.02'].startswith('same'):
        d02 += f" ({e['delay_0.02_s']} s)"
    held = e['held'].split(':')[0].split(' (')[0] if e['held'] else ''
    print(f"| {e['flight']} | {e['try']} | {e['gate_typed_mm']} | {e['xy_log_mm']} | {float(e['normal_log_mm']):+.1f} | "
          f"{e['c0_normal_mm']} | {e['dwell_normal_min_mm']}..{e['dwell_normal_max_mm']} | {e['post2s_normal_min_mm']} | "
          f"{e['outcome']} | {held} | {e['phase_at_contact']} ({e['attach_ready_to_contact_s']}) | "
          f"{e['lift_proof_after_contact_s']} s | {short(e['fires_0.04'])} | {d02} | {d01} |")

print('\n### QUAD4 against his reference at each contact\n')
print('| flight | try | his phase at contact | body - ref z (mm) | mean over 0.5 s (mm) | body vz (m/s) | tip normal (mm) '
      '| tip normal if body on ref (mm) | tip - that (mm) | ref still descending? | his LIFT_PROOF tip from target (m) |')
print('|' + '---|' * 11)
for e in ev:
    desc = 'yes' if float(e['ref_bottom_reached_before_contact_s']) < 0.05 else f"no, bottom {e['ref_bottom_reached_before_contact_s']} s before"
    print(f"| {e['flight']} | {e['try']} | {e['phase_at_contact']} | {e['body_minus_ref_mm']} | {e['body_minus_ref_mean_0p5s_mm']} | "
          f"{e['body_vz_mps']} | {float(e['normal_log_mm']):+.1f} | {e['ref_tip_normal_mm']} | "
          f"{e['tip_normal_minus_ref_tip_normal_mm']} | {desc} | {e['his_tip_from_target_m']} |")

print('\n### Flights with the magnet ON over a descent and no contact\n')
print('| flight | kind | gate typed | his descents | lowest tip normal inside the xy gate (mm) | contacts at 0.04 / 0.02 / 0.01 |')
print('|---|---|---|---|---|---|')
for f in fl:
    if f['contacts_logged'] != '0':
        continue
    print(f"| {f['flight']} | {f['kind']} | {f['gate_typed']} | {f['descents_logged']} | {f['min_normal_inside_xy_mm']} | "
          f"{f.get('contacts_at_0.04', '')} / {f.get('contacts_at_0.02', '')} / {f.get('contacts_at_0.01', '')} "
          f"{('(0.04: ' + f['first_at_0.04'] + ')') if f.get('first_at_0.04') else ''} |")

print('\n### Replay check at the typed gate\n')
dts = [abs(float(v['dt_s'])) for v in ve if v['dt_s']]
miss = [v for v in ve if not v['dt_s']]
print(f'{len(dts)} of {len(ve)} logged contacts reproduced; |dt| median {1000 * sorted(dts)[len(dts) // 2]:.0f} ms, '
      f'max {1000 * max(dts):.0f} ms; unmatched: {len(miss)}')
geo = [(f['flight'], f['geom_check_normal_mm'], f['geom_check_xy_mm']) for f in fl]
print('plate-frame geometry vs the logged /magnet/attach_proof (median |diff| normal, xy in mm): '
      + ', '.join(f'{a} {b}/{c}' for a, b, c in geo))
