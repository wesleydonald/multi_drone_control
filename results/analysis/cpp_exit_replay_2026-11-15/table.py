"""table.py: summary.csv -> compact text tables (printed): the hand-over timeline, plans and joins, geometry."""
import csv
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
rows = list(csv.DictReader(open(os.path.join(HERE, 'summary.csv'))))
rec = json.load(open(os.path.join(HERE, 'flights', 'recsteps.json'))) if os.path.exists(
    os.path.join(HERE, 'flights', 'recsteps.json')) else {}


def f(x, nd=2):
    try:
        return f'{float(x):.{nd}f}'
    except (TypeError, ValueError):
        return '-'


def d(a, b):
    try:
        return f'{float(a) - float(b):.2f}'
    except (TypeError, ValueError):
        return '-'


print('run | t0 | prefix1 | seedtraj acc | prepared | grant | ack | RETURN (h) | AT_START (h) | LANDING | fail')
for r in rows:
    t0 = r['t0']
    print(f"{r['run']} | {f(t0)} | {d(r['prefix1'], t0)} | {d(r['seed_accepted'], t0)} | {d(r['prepared'], t0)} | "
          f"{d(r['grant'], t0)} | {d(r['ack'], t0)} | {f(r['h_return'])} | {f(r['h_at_start'])} | {f(r['landing'])} | "
          f"{r['fail_reason'] or '-'} {r['reject_reasons']}")
print()
print('run | pre att/acc/inst/late | st att/acc/late/inv | post att/acc/inst/late | pre statuses | post statuses | '
      'refreshes | runtime max pre/post ms | splice margin min pre/post ms')
for r in rows:
    print(f"{r['run']} | {r['pre_attempted']}/{r['pre_accepted']}/{r['pre_installed']}/{r['pre_late']} | "
          f"{r['st_attempted']}/{r['st_accepted']}/{r['st_late']}/{r['st_invalidated']} | "
          f"{r['post_attempted']}/{r['post_accepted']}/{r['post_installed']}/{r['post_late']} | {r['pre_statuses']} | "
          f"{r['post_statuses']} | {r['n_refresh']} | {r['pre_runtime_ms_max']}/{r['post_runtime_ms_max']} | "
          f"{r['pre_splice_margin_ms_min']}/{r['post_splice_margin_ms_min']}")
print()
print('run | join latch | refresh max | grant | switch | cpp ticks max | AT_START | live ticks p95 | prefix ticks max | '
      'recorded p95 / max')
for r in rows:
    fl = r['run'].split('_e_')[0].split('_p_')[0]
    rr = rec.get(fl, {})
    print(f"{r['run']} | {r['join_latch']} | {r['join_refresh_max']} | {r['join_grant']} | {r['join_switch']} | "
          f"{r['join_cpp_ticks_max']} | {r['join_at_start']} | {r['tick_live_p95']} | {r['tick_prefix_max']} | "
          f"{rr.get('p95')} / {rr.get('max')}")
print()
print('run | exit z max | cpp cxy max | box gap | cable-tether | cable-basket | exceptions')
for r in rows:
    print(f"{r['run']} | {r['exit_zmax']} | {r['cpp_cxy']} | {r['exit_box_gap']} | {r['exit_tether']} | "
          f"{r['exit_basket']} | {r['exceptions']} {r['exception_text'][:80]}")
