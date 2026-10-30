#!/usr/bin/env python3
"""Title-case \\section/\\subsection/\\subsubsection headings: each word capitalised except minor words (not first)."""
import re, sys, glob
MINOR = {'a', 'an', 'the', 'and', 'but', 'or', 'nor', 'for', 'so', 'yet', 'as', 'at', 'by', 'in', 'of', 'off', 'on',
         'per', 'to', 'up', 'via', 'with', 'from', 'into', 'over', 'than', 'vs', 'versus', 'et', 'al.', 'al'}
HEAD = re.compile(r'(\\(?:sub){0,2}section\*?\{)((?:[^{}]|\{[^{}]*\})*)(\})')

def cap_word(w, first):
    if not w or w.startswith('\\') or w.startswith('$') or any(c.isdigit() for c in w[:1]):
        return w
    core = w.lstrip('(`\'"')
    lead = w[:len(w) - len(core)]
    if not core:
        return w
    if core.isupper() and len(core) > 1:          # acronyms stay
        return w
    if not first and core.lower() in MINOR:
        return lead + core.lower()
    return lead + core[0].upper() + core[1:]

def title(text):
    out, first = [], True
    # split on spaces but keep math and braces intact
    for tok in re.split(r'(\s+|\$[^$]*\$|\\[A-Za-z]+\{[^}]*\})', text):
        if not tok or tok.isspace() or tok.startswith('$') or tok.startswith('\\'):
            out.append(tok)
            if tok and not tok.isspace():
                first = False
            continue
        parts = re.split(r'(--|-)', tok)             # hyphenated: capitalise each part, except after a prefix (Re-plan)
        PREFIX = {'re', 'pre', 'co', 'non', 'anti', 'sub', 'multi', 'inter', 'intra', 'un'}
        new = []
        for i, p in enumerate(parts):
            if p in ('-', '--'):
                new.append(p)
            elif i == 0:
                new.append(cap_word(p, first))
            elif parts[0].lower() in PREFIX:
                new.append(p)
            else:
                new.append(cap_word(p, True))
        parts = new
        out.append(''.join(parts)); first = False
    return ''.join(out)

apply = '--apply' in sys.argv
files = [f for f in sys.argv[1:] if not f.startswith('--')]
for f in files:
    s = open(f).read(); changed = []
    def rep(m):
        new = title(m.group(2))
        if new != m.group(2):
            changed.append((m.group(2), new))
        return m.group(1) + new + m.group(3)
    s2 = HEAD.sub(rep, s)
    for a, b in changed:
        print(f'{f.split("Source/")[-1]}: {a}  ->  {b}')
    if apply and s2 != s:
        open(f, 'w').write(s2)
