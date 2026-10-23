"""Resolve every launch line of a rig sheet with tools/launch_dump (nothing started) and report refusals and key params."""
import os
import re
import shlex
import sys

REPO = '/home/wesley/multi_drone_control'
sys.path.insert(0, os.path.join(REPO, 'tools'))
import launch_dump  # noqa: E402

sheet = open(sys.argv[1]).read()
blocks = re.findall(r'```bash\n(.*?)```', sheet, re.S)
var = {'C': os.path.join(REPO, 'configs', 'rig'), 'HOME': os.path.expanduser('~')}
lines = []
for b in blocks:
    for raw in b.splitlines():
        line = raw.split('  #')[0].strip()
        if not line or line.startswith('#'):
            continue
        m = re.match(r'^(\w+)="(.*)"$', line)
        if m:
            var[m.group(1)] = m.group(2)
            continue
        if 'rig_flight.sh' in line or line.startswith('ros2 launch bringup') or line.startswith('$R'):
            lines.append(line)


def expand(s):
    for _ in range(3):
        s = re.sub(r'\$(\w+)', lambda mm: var.get(mm.group(1), mm.group(0)), s)
    return s


bad = 0
for line in lines:
    toks = shlex.split(expand(line))
    if 'rig_flight.sh' in toks[0]:
        toks = toks[1:]
        name = toks[-1] if not toks[-1].endswith('.py') and ':=' not in toks[-1] else ''
        if name:
            toks = toks[:-1]
        launch, args = toks[0], toks[1:]
    elif toks[:3] == ['ros2', 'launch', 'bringup']:
        launch, args, name = toks[3], toks[4:], '(T1)'
    else:
        continue
    kv = dict(a.split(':=', 1) for a in args if ':=' in a)
    try:
        g = launch_dump.dump(launch, kv)
        nodes = g['entries']
        pl = [n for n in nodes if isinstance(n, dict) and n.get('executable') in ('mpc_planner', 'dissipative_planner')]
        info = ''
        if pl:
            p = pl[0].get('parameters', {})
            def v(k):
                x = p.get(k)
                return x[1] if isinstance(x, (list, tuple)) else x
            info = (f"traj={v('load_traj')} fig8={v('fig8_size')} dist={v('traj_distance')} cable={v('cable_len')} "
                    f"lim={v('pos_limit_m')} prefix={pl[0].get('prefix', [k for k in pl[0] if 'prefix' in k])!r}")
        print(f'ok    {name:10s} {launch:28s} {info}')
    except BaseException as e:
        bad += 1
        print(f'FAIL  {name:10s} {launch:28s} {type(e).__name__}: {str(e)[:200]}')
print(f'{len(lines)} lines, {bad} refused')
