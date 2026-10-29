#!/usr/bin/env python3
"""Tejen's completion-wakeup fix (1cfc7a90) in the Thu 29 floor-join rehearsal, the twin.

Message: before the fix his planner installed almost none of its accepted plans and QUAD4 waited 74 s to forever before
its transit; with it nearly every plan installs and the wait is under half a second.
Data: R2695 / R2738 (before), R2819 / R2820 (after): registry rows (replans.csv tallies).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402

RUNS = (('before', 'R2695', 551, 2, 73.8), ('before', 'R2738', 1422, 0, None),
        ('after', 'R2819', 34, 33, 0.2), ('after', 'R2820', 35, 34, 0.4))
NEVER_S = 300.0


def main():
    S.setup()
    fig = S.figure('full', 2.3)
    a, b = fig.subplots(1, 2)
    xs = range(len(RUNS))
    for x, (arm, _, acc, inst, _) in zip(xs, RUNS):
        col = S.RED if arm == 'before' else S.BLUE
        a.bar(x, 100.0 * inst / acc, width=0.6, color=col)
        a.annotate(f'{inst} of {acc}', xy=(x, 100.0 * inst / acc), xytext=(0, 3), textcoords='offset points',
                   ha='center', va='bottom', fontsize=8, color=col)
    a.set_ylabel(r'Accepted plans installed (\%)')
    a.set_ylim(0, 110)
    for x, (arm, _, _, _, wait) in zip(xs, RUNS):
        col = S.RED if arm == 'before' else S.BLUE
        b.bar(x, wait if wait is not None else NEVER_S, width=0.6, color=col,
              hatch=None if wait is not None else '////', edgecolor='white' if wait is None else col)
        b.annotate(f'{wait:g} s' if wait is not None else 'none in 300 s', xy=(x, wait or NEVER_S), xytext=(0, 3),
                   textcoords='offset points', ha='center', va='bottom', fontsize=8, color=col)
    b.set_yscale('log')
    b.set_ylim(0.1, 1500)
    b.set_ylabel('Settled to first install (s)')
    for ax in (a, b):
        ax.set_xticks(list(xs))
        ax.set_xticklabels(['before', 'before', 'with the fix', 'with the fix'], fontsize=8)
    S.panel(a, 'a')
    S.panel(b, 'b')
    S.save(fig, 'fig_install_fix')


if __name__ == '__main__':
    main()
