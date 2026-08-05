"""
tools/plot_style.py — the one figure style (THESIS_PLAN §9.4, §6.2)

Every figure in the thesis is drawn through this module, so the four months of results
that follow are publication-ready by default rather than retro-fitted at the end.

Two rules it exists to enforce:

  * ONE COLOUR PER DRONE ID, and it is the SAME colour RViz uses. The palette is
    IMPORTED from `controller_quad_load/rviz_config.py` rather than copied, because a
    copied palette drifts silently and then drone 2 is green on screen and red in the
    figure you are describing in the same sentence.
  * ONE VISUAL GRAMMAR: desired/reference is always dashed and pale, actual is always
    solid, events are always vertical dotted lines, the steady window is always the
    shaded band. Consistency across figures is what makes a reader able to skim them.

Vector output (PDF) is the deliverable; a PNG is written alongside for quick viewing
and for pasting into a message. Fonts are embedded as TrueType (type 42) so the PDF
survives a journal's font check.
"""
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'controller_quad_load'))

import matplotlib                                     # noqa: E402
matplotlib.use('Agg')                                 # headless: batches have no display
import matplotlib.pyplot as plt                       # noqa: E402

from controller_quad_load.rviz_config import (        # noqa: E402
    DRONE_COLOURS as _RVIZ_DRONE_COLOURS, PAYLOAD_COLOUR as _RVIZ_PAYLOAD_COLOUR)


def _hex(rviz_colour):
    """RViz's "r; g; b" (0-255) -> matplotlib "#rrggbb"."""
    r, g, b = (int(v) for v in rviz_colour.split(';'))
    return f'#{r:02x}{g:02x}{b:02x}'


DRONE_HEX = [_hex(c) for c, _ in _RVIZ_DRONE_COLOURS]
PAYLOAD_HEX = _hex(_RVIZ_PAYLOAD_COLOUR)

# Gold-on-white is unreadable, and the payload is on almost every figure. RViz needs the
# bright value against a dark viewport; print needs a dark one. Same identity, adjusted
# for the medium -- this is the only deliberate divergence from the RViz palette.
PAYLOAD_LINE = '#b8860b'

# Runs, for cross-run overlays. DELIBERATELY NOT the drone palette: on a compare_runs
# figure colour means "which run", and reusing the per-drone colours there would make
# the same blue mean drone 0 on one page and R0034 on the next.
RUN_HEX = ['#000000', '#e41a1c', '#377eb8', '#4daf4a',
           '#984ea3', '#ff7f00', '#a65628', '#999999']

# The grammar. Passed as **kwargs so a caller cannot half-apply it.
DESIRED = dict(linestyle='--', linewidth=1.4, alpha=0.85)
ACTUAL = dict(linestyle='-', linewidth=1.6)
REFERENCE = dict(linestyle=':', linewidth=1.2, alpha=0.9)

EVENT_HEX = {'ARM': '#999999', 'TAKEOFF': '#999999', 'LAND': '#999999',
             'MAGNET': '#377eb8', 'WELD': '#4daf4a', 'DETACH': '#ff7f00',
             'FLEET_ABORT': '#e41a1c', 'DISARM': '#e41a1c'}


def drone_colour(i):
    """Matplotlib colour for drone `i`, matching RViz. Wraps past the palette."""
    return DRONE_HEX[i % len(DRONE_HEX)]


def run_colour(k):
    return RUN_HEX[k % len(RUN_HEX)]


def event_colour(name):
    return EVENT_HEX.get(str(name).upper(), '#666666')


def use():
    """Apply the thesis rcParams. Idempotent; call it before making any figure."""
    plt.rcParams.update({
        'figure.dpi': 110,
        'savefig.dpi': 200,
        'savefig.bbox': 'tight',
        'font.family': 'sans-serif',
        'font.size': 9,
        'axes.titlesize': 10,
        'axes.labelsize': 9,
        'axes.grid': True,
        'axes.axisbelow': True,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'grid.alpha': 0.25,
        'grid.linewidth': 0.5,
        'legend.fontsize': 8,
        'legend.frameon': False,
        'lines.solid_capstyle': 'round',
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        # Embed real fonts rather than rasterising or subsetting to type 3.
        'pdf.fonttype': 42,
        'svg.fonttype': 'none',
    })


# ── the shared annotations ───────────────────────────────────────────────────

def mark_events(ax, events, skip=(), label=True, t_max=None):
    """Vertical line per event, coloured by kind, labelled once at the top.

    `events` is metrics.read_events()'s {name: sim_time}. Every time axis in the
    pipeline shares one origin with events.csv, so these land in the right place
    without a per-figure offset."""
    if not events:
        return
    for name, t in sorted(events.items(), key=lambda kv: kv[1]):
        if name in skip or not math.isfinite(t):
            continue
        if t_max is not None and t > t_max:
            continue
        ax.axvline(t, color=event_colour(name), linestyle=':', linewidth=1.0,
                   alpha=0.8, zorder=0)
        if label:
            ax.annotate(name, xy=(t, 1.0), xycoords=('data', 'axes fraction'),
                        xytext=(2, -2), textcoords='offset points',
                        fontsize=6.5, rotation=90, va='top', ha='left',
                        color=event_colour(name))


def shade_steady(ax, t_start, t_end, label='steady window'):
    """The §9.2 steady window, shaded. Every settled number is read from inside it, so
    it belongs on the figure that number is defended with."""
    if t_start is None or not math.isfinite(t_start) or t_end is None:
        return
    ax.axvspan(t_start, t_end, color='#4daf4a', alpha=0.07, zorder=0, label=label)


def stamp(fig, run_label, extra=None):
    """Run id in the figure itself (§4.1: you can always get from a figure back to the
    raw data). Bottom-right so it never collides with a title."""
    txt = run_label if not extra else f'{run_label}   {extra}'
    fig.text(0.995, 0.005, txt, ha='right', va='bottom', fontsize=6.5,
             color='#666666')


def save(fig, out_dir, name, formats=('pdf', 'png')):
    """Write `name.<fmt>` into `out_dir` and close the figure. Returns the paths."""
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    for fmt in formats:
        p = os.path.join(out_dir, f'{name}.{fmt}')
        fig.savefig(p, format=fmt)
        paths.append(p)
    plt.close(fig)
    return paths


def busy_legend(ax, **kw):
    """A legend for a panel whose traces reach the top of the axes. Opaque, because a
    frameless legend over dense data is unreadable and this pipeline draws several."""
    kw.setdefault('loc', 'upper left')
    return ax.legend(frameon=True, facecolor='white', edgecolor='none',
                     framealpha=0.85, **kw)


def note(ax, text):
    """Say plainly, on the axes, that a panel has no data -- an empty panel otherwise
    reads as "nothing happened" when it means "this run does not record that"."""
    ax.text(0.5, 0.5, text, ha='center', va='center', transform=ax.transAxes,
            fontsize=8, color='#888888', wrap=True)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
