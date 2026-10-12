#!/usr/bin/env python3
"""tools/figure_runs.py -- the thesis figures' run ids, read from configs/thesis_figures.yaml (section 9.4).

    from figure_runs import figure, resolve
    spec = figure('F_releases')          # the registry entry: runs, rows, caption, pending
    d = resolve('R1644')                 # the run directory of a twin run (results/<date>/R1644_*)
    d = resolve('results/rig/2026-10-08/r305_run')   # a rig flight's run directory

A figure script holds no run ids: Wednesday's flights go into the yaml and the script is re-run. Run directories are
looked up under MDC_RESULTS_ROOT first (the main tree's results, read only), then under this tree's results/, so a
converted rig flight that lives only in a worktree is found too.
"""
import glob
import os

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REGISTRY = os.path.join(REPO, 'configs', 'thesis_figures.yaml')


def roots():
    out = []
    env = os.environ.get('MDC_RESULTS_ROOT')
    if env:
        out.append(os.path.abspath(env))
    local = os.path.join(REPO, 'results')
    if local not in out:
        out.append(local)
    return out


def figure(fig_id):
    """The registry entry for one figure id."""
    for f in yaml.safe_load(open(REGISTRY))['figures']:
        if f.get('id') == fig_id:
            return f
    raise KeyError(f'{fig_id}: not in {REGISTRY}')


def resolve(tag):
    """A run directory (the one holding logs/run.csv) from R####, results/rig/<date>/<name>_run or rig:<date>/<name>."""
    if tag.startswith('rig:'):
        tag = os.path.join('results', 'rig', tag[4:] + '_run')
    for root in roots():
        if tag[:1] == 'R' and tag[1:5].isdigit() and '/' not in tag:
            hits = sorted(glob.glob(os.path.join(root, '2*', tag + '_*')))
            hits = [h for h in hits if os.path.isfile(os.path.join(h, 'logs', 'run.csv'))]
            if hits:
                return hits[0]
        else:
            rel = tag[len('results/'):] if tag.startswith('results/') else tag
            d = os.path.join(root, rel)
            if os.path.isfile(os.path.join(d, 'logs', 'run.csv')):
                return d
    raise FileNotFoundError(f'{tag}: no run directory with logs/run.csv under {roots()}')


def resolve_dir(rel):
    """Any directory under results/ (a flight's _logs, a bag), by the same root order."""
    rel = rel[len('results/'):] if rel.startswith('results/') else rel
    for root in roots():
        d = os.path.join(root, rel)
        if os.path.isdir(d):
            return d
    raise FileNotFoundError(f'{rel}: not under {roots()}')
