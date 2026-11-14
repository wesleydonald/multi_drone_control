#!/usr/bin/env python3
"""Page-break check for the thesis (Wesley 11 Nov: additions must not spill pages): compare two builds page by page
(first and last text line of each page) and the free space at the bottom of each page.
Usage: cmp_pages.py OLD.pdf NEW.pdf [first_page last_page]"""
import re
import subprocess
import sys


def pages(p):
    n = int([l for l in subprocess.run(['pdfinfo', p], capture_output=True, text=True).stdout.splitlines()
             if l.startswith('Pages')][0].split()[1])
    out = []
    for i in range(1, n + 1):
        t = [x for x in subprocess.run(['pdftotext', '-f', str(i), '-l', str(i), p, '-'], capture_output=True,
                                       text=True).stdout.splitlines() if x.strip()]
        out.append((t[0][:45] if t else '', t[-2][:45] if len(t) > 1 else ''))
    return out


def slack(p):
    h = subprocess.run(['pdftotext', '-bbox-layout', p, '-'], capture_output=True, text=True).stdout
    res = []
    for pg in re.split(r'<page ', h)[1:]:
        ys = [float(m) for m in re.findall(r'<line xMin="[\d.]+" yMin="[\d.]+" xMax="[\d.]+" yMax="([\d.]+)"', pg)]
        ys = [y for y in ys if y < 760]
        res.append(round(746 - max(ys)) if ys else None)
    return res


old, new = sys.argv[1], sys.argv[2]
lo, hi = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) > 4 else (10, 999)
o, nw = pages(old), pages(new)
print('pages old', len(o), 'new', len(nw))
for i, (a, b) in enumerate(zip(o, nw), 1):
    if a != b and lo <= i <= hi:
        print(i, 'OLD', a, '\n   NEW', b)
so, sn = slack(old), slack(new)
print('free space (pt) old:', ' '.join(f'{i}:{v}' for i, v in enumerate(so, 1) if lo <= i <= hi and v is not None))
print('free space (pt) new:', ' '.join(f'{i}:{v}' for i, v in enumerate(sn, 1) if lo <= i <= hi and v is not None))
