"""X11 manifest: every simulation config the full draft cites, by the section citing it, with its registry status."""
import csv, os, re
TH = os.path.expanduser('~/thesis_full_draft'); REPO = os.path.expanduser('~/multi_drone_control')
def inputs(path, seen):
    if path in seen or not os.path.exists(path): return
    seen.append(path)
    for m in re.finditer(r'^[^%\n]*\\(?:input|include)\{([^}]+)\}', open(path).read(), re.M):
        p = m.group(1); inputs(os.path.join(TH, p if p.endswith('.tex') else p + '.tex'), seen)
files = []; inputs(os.path.join(TH, 'main.tex'), files)
reg = {}
for row in csv.reader(open(os.path.join(REPO, 'results/registry.csv'), newline='')):
    if row and re.fullmatch(r'R\d{4}', row[0]): reg[row[0]] = row
HEAD = re.compile(r'\\(?:sub)*section\*?\{([^}]*)\}')
cfg = {}   # config -> {'sections': set, 'runs': set}
for p in files:
    txt = open(p).read(); heads = [(m.start(), m.group(1)) for m in HEAD.finditer(txt)]
    for m in re.finditer(r'\bR(\d{4})(?:\s*(?:-|--|–|to)\s*R?(\d{4}))?', txt):
        sec = next((h for pos, h in reversed(heads) if pos < m.start()), os.path.basename(p))
        lo = int(m.group(1)); hi = int(m.group(2)) if m.group(2) and 0 <= int(m.group(2)) - lo < 80 else lo
        for i in range(lo, hi + 1):
            r = f'R{i:04d}'; row = reg.get(r)
            if not row: continue
            c = cfg.setdefault(row[2], {'sections': [], 'runs': set(), 'date': row[1], 'expl': False})
            if sec not in c['sections']: c['sections'].append(sec)
            c['runs'].add(r); c['expl'] |= 'exploratory' in row[3].lower()
def group(c, v):
    if v['date'] < '2026-10-21': return 'C'
    return 'B' if v['expl'] else 'A'
out = ['# X11 manifest: the simulation configs the full draft cites (generated 30 Oct 2026)', '',
       'From the run ids in the files `~/thesis_full_draft/main.tex` inputs (commit c943094), mapped through',
       '`results/registry.csv`; the section is the heading above each citation. Script:',
       '`tools/x11_manifest.py` (re-run after the draft changes). Re-fly each A config x2 on the final tree (after the',
       'freeze), with the slope estimate; B and C need a decision first.', '']
names = {'A': 'A. Claims cited (re-fly x2)', 'B': 'B. Exploratory runs cited (re-fly only where a table claims them)',
         'C': 'C. Before 21 Oct (older tree and cables; re-fly, cite as the earlier setup, or drop)'}
for g in 'ABC':
    rows = sorted((v['sections'][0], c, v) for c, v in cfg.items() if group(c, v) == g)
    out += [f'## {names[g]} ({len(rows)} configs)', '', '| Section | Config | Runs cited | Config file |', '|---|---|---|---|']
    for sec, c, v in rows:
        ok = os.path.exists(os.path.join(REPO, 'configs/experiments', c + '.yaml'))
        rs = sorted(v['runs']); out.append(f'| {"; ".join(v["sections"])} | `{c}` | {", ".join(rs)} | {"ok" if ok else "MISSING"} |')
    out.append('')
    with open(os.path.join(REPO, f'configs/batches/x11_{g}.txt'), 'w') as f:
        f.write(''.join(f'configs/experiments/{c}.yaml\n' for _, c, _ in rows))
open(os.path.join(REPO, 'docs/analysis/x11_refly_manifest.md'), 'w').write('\n'.join(out))
print({g: sum(1 for c, v in cfg.items() if group(c, v) == g) for g in 'ABC'})
