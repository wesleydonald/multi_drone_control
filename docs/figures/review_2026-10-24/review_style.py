"""The shared look of the 23 Oct review figures: the thesis preset (tools/plot_style.thesis(): Computer Modern,
print sizes) plus one red/blue pair for two-way comparisons, one set of line weights and the save step.

Blue is the arm the thesis argues for (our planner, the measured catch point, the fix); red is the one it is
compared with. Single-run figures stay black and grey.
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
TOOLS = os.path.join(REPO, 'tools')
sys.path.insert(0, TOOLS)

import plot_style as ps  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

BLUE, RED = '#2166ac', '#b2182b'
BLUE_LIGHT, RED_LIGHT = '#92c5de', '#f4a582'
RED_MID = '#d6604d'          # text on a RED_LIGHT line
INK, GREY, LIGHT = '0.1', '0.45', '0.75'
FILL = '0.93'
LW_HERO, LW_CTX, LW_REF = 1.2, 0.7, 0.7     # dense time series stay at LW_CTX even as the hero
MS = 3.8                                   # one marker size for every run marker
REF = dict(color=GREY, lw=LW_REF, ls=(0, (4, 3)), zorder=0)

WIDTH = {'full': ps.TEXTWIDTH_IN, 'half': (ps.TEXTWIDTH_IN - ps.HALF_GAP_IN) / 2}
AUDIT = os.path.join(REPO, '.claude', 'skills', 'thesis-figure', 'scripts', 'figure_audit.py')
SCRATCH = os.environ.get('REVIEW_SCRATCH', '/tmp/claude-1000/-home-wesley-multi-drone-control/'
                         'f2a54328-3214-4ac3-989e-f5a0c5208168/scratchpad/audit')


def setup():
    ps.thesis()
    plt.rcParams.update({'lines.linewidth': LW_CTX, 'legend.handlelength': 1.6, 'legend.borderaxespad': 0.2,
                         'legend.columnspacing': 1.2, 'legend.handletextpad': 0.5})


def figure(width='full', height=2.4):
    return plt.figure(figsize=(WIDTH.get(width, width), height))


def ref_line(ax, y, text=None, x=1.0, ha='right', va='bottom', dy=1.5, horizontal=True):
    """A dashed grey reference (a bar or a budget), labelled in grey at one end."""
    if horizontal:
        ax.axhline(y, **REF)
        if text:
            dx = {'right': -2, 'left': 2}.get(ha, 0)        # clear of the box's side (full box, Wesley 30 Oct)
            ax.annotate(text, xy=(x, y), xycoords=('axes fraction', 'data'), xytext=(dx, dy if va == 'bottom' else -dy),
                        textcoords='offset points', ha=ha, va=va, fontsize=8, color=GREY)
    else:
        ax.axvline(y, **REF)


def panel(ax, letter, text=None):
    """'(a)' where plot_style.panel_label puts it, and an optional short name of the panel after it."""
    ax.annotate(f'({letter})' + (rf'\ \ {text}' if text else ''), xy=(0, 1), xycoords='axes fraction', xytext=(0, 4),
                textcoords='offset points', ha='left', va='bottom', fontsize=9)


def save(fig, name, width='full'):
    """PDF + 300 dpi PNG beside this module; the audit runs on a scratch copy so its previews stay out of here."""
    stem = os.path.join(HERE, name)
    ps.save_print(fig, stem)
    os.makedirs(SCRATCH, exist_ok=True)
    tmp = os.path.join(SCRATCH, name + '.pdf')
    shutil.copy(stem + '.pdf', tmp)
    w = WIDTH.get(width, width)
    out = subprocess.run([sys.executable, AUDIT, tmp, '--width', f'{w:.3f}'], capture_output=True, text=True)
    print(out.stdout.strip())
    if out.returncode:
        print(out.stderr.strip())
    return stem
