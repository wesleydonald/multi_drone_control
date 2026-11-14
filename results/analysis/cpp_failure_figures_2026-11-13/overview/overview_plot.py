"""Overview figure of every twin run of Te-Jen's C++ planner (runs_table.csv): settings group vs outcome and transit time."""
import csv, collections
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
rows = list(csv.DictReader(open('runs_table.csv')))
CRASH = {'R4438'}
def outcome(r):
    if r['run'] in CRASH: return 'QUAD4 crashed in its climb (before C++)'
    if '_land' in r['name'] and r['reached_drop'] == 'no': return 'our LAND test (deliberate abort)'
    if r['cpp_started'] == 'no': return 'C++ never started (no plan from the start)'
    if r['reached_drop'] == 'yes': return 'reached the drop point'
    return 'stalled / never reached (to the wall cap)'
COL = {'reached the drop point': '#1b9e77', 'stalled / never reached (to the wall cap)': '#d95f02',
       'C++ never started (no plan from the start)': '#7570b3', 'our LAND test (deliberate abort)': '#999999',
       'QUAD4 crashed in its climb (before C++)': '#000000'}
MK = {'hover': 'o', 'circle': 's', 'fig8': '^'}
def group(r):
    moving = not (r['traj'] == 'hover' or r['paused'] == 'yes' or (r['ring_speed_at_transit'] and float(r['ring_speed_at_transit']) < 0.02))
    shape = 'boxes (legacy_square, default)' if r['shape'].startswith('legacy') else 'convex (outer_octagon)'
    bits = [shape, 'MOVING ring' if moving else 'still ring']
    if r['workspace'] != 'none': bits.append(f'workspace +-{r["workspace"]}')
    if r['spline'] != 'default': bits.append(f'spline {r["spline"]}')
    if r['cap_m'] != 'none': bits.append(f'cap {r["cap_m"]} m')
    bits.append('splice ' + ('0.20' if r['splice_s'].startswith('0.2') else '0.10'))
    return ', '.join(bits)
g = collections.defaultdict(list)
for r in rows: g[group(r)].append(r)
order = sorted(g, key=lambda k: (k.startswith('convex'), 'still' in k, k))
fig, ax = plt.subplots(figsize=(13, 0.55 * len(order) + 2.2))
NEVER = max(float(r['transit_s']) for r in rows if r['transit_s']) + 30
for y, k in enumerate(order):
    for j, r in enumerate(g[k]):
        o = outcome(r)
        x = float(r['transit_s']) if r['transit_s'] else NEVER + (j % 6) * 2.2
        ax.scatter(x, y + (j % 3 - 1) * 0.18, c=COL[o], marker=MK.get(r['traj'], 'o'), s=55, edgecolors='k', linewidths=0.4, zorder=3)
    n = len(g[k]); ok = sum(outcome(r) == 'reached the drop point' for r in g[k])
    tests = sum('LAND test' in outcome(r) or 'crashed' in outcome(r) for r in g[k])
    ax.text(NEVER + 17, y, f'{ok}/{n - tests} reached' + (f' (+{tests} test/crash)' if tests else ''), va='center', fontsize=8)
ax.axvline(NEVER - 5, color='k', lw=0.8, ls='--')
ax.text(NEVER + 6, -0.65, 'never reached /\nnot applicable', fontsize=8, ha='center', va='bottom')
ax.set_yticks(range(len(order))); ax.set_yticklabels(order, fontsize=8)
ax.set_xlim(0, NEVER + 45); ax.set_ylim(-0.7, len(order) - 0.1)
ax.set_xlabel('his C++ transit time to the drop point (s, simulation time)')
ax.set_title('Every simulated drop-off with his C++ planner, 12-13 Nov (91 runs): settings vs what happened\n'
             'the simulation ran at 0.11-0.16 of real time, so moving-ring results here are slow-time results', fontsize=10)
from matplotlib.lines import Line2D
h = [Line2D([], [], marker='o', ls='', color=c, markeredgecolor='k', label=o) for o, c in COL.items()]
h += [Line2D([], [], marker=m, ls='', color='w', markeredgecolor='k', label=t) for t, m in MK.items()]
ax.legend(handles=h, loc='upper center', bbox_to_anchor=(0.5, -0.07), ncol=4, fontsize=8, frameon=True)
ax.grid(axis='x', alpha=0.3)
fig.tight_layout()
fig.savefig('overview_all_cpp_twin_runs.png', dpi=150, bbox_inches='tight')
print('ok', len(order), 'groups')
