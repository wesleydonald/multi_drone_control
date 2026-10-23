"""The 23 Oct review look (../review_2026-10-23/review_style.py, reused as is) with the save step pointed at this
folder, plus what this set adds (Wesley 23 Oct: less black, colour the data):
  * one colour per fleet size in every figure: 2, 3 and 4 carriers are matplotlib's plasma at 0.74, 0.50 and 0.20
    (orange, rose, purple): ordered in lightness (L* 71, 51, 30), so they survive greyscale print and the
    deuteranopia/protanopia previews; none is the comparison red or blue;
  * red and blue (review_style.RED / BLUE) stay for two-way comparisons (planned against measured, the released carrier
    against the survivors); grey only for references and context; black for the ring only beside drone traces;
  * rig against twin: filled against open.
"""
import os
import shutil
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), 'review_2026-10-23'))
sys.path.insert(0, _HERE)

from review_style import *  # noqa: E402,F401,F403  -- the shared look: colours, weights, setup(), figure(), panel()
import review_style as _R  # noqa: E402

HERE = _HERE            # after the star import, which brings review_style's own HERE (its folder) with it
ps = _R.ps
FLEET_COLOUR = {2: '#f79143', 3: '#cc4778', 4: '#6a00a8'}       # plasma(0.74), plasma(0.50), plasma(0.20)
FLEET = {n: dict(color=c, lw=1.0) for n, c in FLEET_COLOUR.items()}


def tint(colour, a=0.25):
    """colour mixed with white (a = how much colour is left): box fills and wide bars."""
    import matplotlib.colors as mc
    r, g, b = mc.to_rgb(colour)
    return (1 - a * (1 - r), 1 - a * (1 - g), 1 - a * (1 - b))


def fleet_panel(ax, letter, n, text=None):
    """'(a)  n carriers' above the panel, the fleet size in its colour (the figure's key), as review_style.panel."""
    ax.annotate(f'({letter})', xy=(0, 1), xycoords='axes fraction', xytext=(0, 4), textcoords='offset points',
                ha='left', va='bottom', fontsize=9)
    ax.annotate(text or f'{n} carriers', xy=(0, 1), xycoords='axes fraction', xytext=(17, 4),
                textcoords='offset points', ha='left', va='bottom', fontsize=9, color=FLEET_COLOUR[n])


def g_ticks(*axes, which='xy'):
    """Tick labels without trailing zeros (0.5, 1 rather than 0.50, 1.00), in math mode for a true minus sign."""
    from matplotlib.ticker import FuncFormatter
    fmt = FuncFormatter(lambda v, _: f'${v:g}$')
    for ax in axes:
        if 'x' in which:
            ax.xaxis.set_major_formatter(fmt)
        if 'y' in which:
            ax.yaxis.set_major_formatter(fmt)


def save(fig, name, width='full'):
    """PDF + 300 dpi PNG in this folder; the audit (the skill's figure_audit.py) runs on a scratch copy."""
    stem = os.path.join(HERE, name)
    ps.save_print(fig, stem)
    os.makedirs(_R.SCRATCH, exist_ok=True)
    tmp = os.path.join(_R.SCRATCH, name + '.pdf')
    shutil.copy(stem + '.pdf', tmp)
    w = _R.WIDTH.get(width, width)
    out = subprocess.run([sys.executable, _R.AUDIT, tmp, '--width', f'{w:.3f}'], capture_output=True, text=True)
    print(out.stdout.strip())
    if out.returncode:
        print(out.stderr.strip())
    return stem
