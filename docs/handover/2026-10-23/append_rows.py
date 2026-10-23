"""Append registry rows (LF, csv.writer) after a backup; cmp the old bytes. Usage: python3 append_rows.py rows.json"""
import csv, json, os, shutil, subprocess, sys
REG = '/home/wesley/multi_drone_control/results/registry.csv'
S = os.path.dirname(os.path.abspath(__file__))
rows = json.load(open(sys.argv[1]))
n = 71
while os.path.exists(os.path.join(S, f'registry.bak{n}')):
    n += 1
bak = os.path.join(S, f'registry.bak{n}')
shutil.copy2(REG, bak)
size = os.path.getsize(bak)
with open(REG, 'rb') as fh:
    assert fh.read()[-1:] == b'\n'
with open(REG, 'a', newline='') as fh:
    w = csv.writer(fh, lineterminator='\n')
    for r in rows:
        assert isinstance(r, list) and len(r) == 12 and all(isinstance(v, str) for v in r), r
        w.writerow(r)
ok = subprocess.run(['cmp', '-n', str(size), bak, REG]).returncode == 0
print(f'backup {bak}; appended {len(rows)}; old bytes unchanged: {ok}')
