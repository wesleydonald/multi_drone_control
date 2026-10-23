"""Resolve the Thu 29 sheet's $A lines with test_real_mode_launches.evaluate (handles scoped groups); W, L stood in."""
import json, os, re, shlex, sys
REPO = '/home/wesley/multi_drone_control'
sys.path.insert(0, os.path.join(REPO, 'src', 'bringup', 'test'))
import test_real_mode_launches as T  # noqa: E402
W, L = sys.argv[2], sys.argv[3]
sheet = open(sys.argv[1]).read()
var = {'C': os.path.join(REPO, 'configs', 'rig'), 'W': W, 'L': L, 'K': '0.024', 'X': '0.013', 'Y': '0.210',
       'P': str(round(float(L) - 0.003, 3)), 'Q': str(round(float(L) - 0.04, 3))}
for name in 'AMBH':
    var[name] = re.search(rf'^{name}="(.*?)"', sheet, re.M).group(1)
def expand(s):
    for _ in range(4):
        s = re.sub(r'\$(\w+)', lambda mm: var.get(mm.group(1), mm.group(0)), s)
    return s
for line in [l.split('  #')[0].strip() for l in sheet.splitlines() if re.match(r'^\s*\$A ', l)]:
    toks = shlex.split(expand(line))[1:]
    name = toks[-1] if ':=' not in toks[-1] else ''
    toks = toks[:-1] if name else toks
    kv = dict(a.split(':=', 1) for a in toks[1:] if ':=' in a)
    try:
        nodes = T.evaluate(toks[0], **kv)
        ex = sorted({n['executable'] for n in nodes})
        partner = [n for n in nodes if 'tejen' in (n['executable'] or '') or 'partner' in (n['name'] or '') or 'mission' in (n['executable'] or '')]
        cc = sorted({str(n['params'].get('carry_clearance')) for n in nodes if 'carry_clearance' in n['params']})
        cl = sorted({str(n['params'].get('cable_len')) for n in nodes if 'cable_len' in n['params']})
        print(f"ok    {name:4s} nodes={len(nodes)} carry_clearance={cc} cable_len={cl} partner-ish={len(partner)}")
    except BaseException as e:
        print(f'FAIL  {name:4s} {type(e).__name__}: {str(e)[:300]}')
