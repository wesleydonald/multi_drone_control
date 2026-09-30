"""
tools/plot_style.py — the one figure style (THESIS_PLAN §9.4, §6.2)

Every figure in the thesis is drawn through this module. It enforces two things:

  * ONE COLOUR PER DRONE ID, the same colour RViz uses. The palette is IMPORTED from
    `controller_quad_load/rviz_config.py`, never copied -- a copy drifts silently, and
    then drone 2 is green on screen and red in the figure describing it.
  * ONE VISUAL GRAMMAR: reference dashed, actual solid, events vertical dotted, steady
    window shaded.

PDF is the deliverable; a PNG is written alongside for quick viewing. Fonts embed as
TrueType (type 42) so the PDF survives a journal's font check.
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

# The one deliberate divergence from the RViz palette: RViz's gold reads against a dark
# viewport, not against white paper.
PAYLOAD_LINE = '#b8860b'

# For cross-run overlays, and deliberately NOT the drone palette: there colour means
# "which run", and sharing would make the same blue mean drone 0 on the facing page.
RUN_HEX = ['#000000', '#e41a1c', '#377eb8', '#4daf4a',
           '#984ea3', '#ff7f00', '#a65628', '#999999']

# Passed as **kwargs so a caller cannot half-apply the grammar.
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


# ── print preset for the thesis (the dashboard style above is for reading runs) ──────
# thesis/main.tex: article 12 pt, A4, 1 in margins -> \textwidth 452.97 pt; body font
# Computer Modern. Figures are built at the size they print, so 9 pt here is 9 pt on paper.
TEXTWIDTH_IN = 452.97 / 72.27
HALF_GAP_IN = 0.2


def size(width='full', aspect=0.62, height=None):
    """(w, h) in inches: width 'full', 'half' or a fraction of \\textwidth."""
    w = {'full': TEXTWIDTH_IN, 'half': (TEXTWIDTH_IN - HALF_GAP_IN) / 2}.get(width)
    if w is None:
        w = float(width) * TEXTWIDTH_IN
    return (w, height if height is not None else w * aspect)


def thesis():
    """rcParams for a figure that goes into the thesis. Text is typeset by LaTeX in
    Computer Modern so it matches the body; sizes are print sizes."""
    plt.rcParams.update(plt.rcParamsDefault)
    plt.rcParams.update({
        'text.usetex': True,
        'text.latex.preamble': r'\usepackage{amsmath}\usepackage{siunitx}',
        'font.family': 'serif',
        'font.size': 9,
        'axes.labelsize': 9,
        'axes.titlesize': 9,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'legend.fontsize': 8,
        'legend.frameon': False,
        'legend.handlelength': 1.8,
        'axes.linewidth': 0.6,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'axes.grid': False,
        'axes.labelpad': 3,
        'xtick.direction': 'out',
        'ytick.direction': 'out',
        'xtick.major.width': 0.6,
        'ytick.major.width': 0.6,
        'xtick.major.size': 3,
        'ytick.major.size': 3,
        'xtick.major.pad': 2,
        'ytick.major.pad': 2,
        'lines.linewidth': 1.0,
        'lines.markersize': 3,
        'patch.linewidth': 0.6,
        'figure.constrained_layout.use': True,
        'figure.constrained_layout.h_pad': 0.02,
        'figure.constrained_layout.w_pad': 0.02,
        'savefig.bbox': None,
        'savefig.dpi': 300,
        'pdf.fonttype': 42,
    })


def panel_label(ax, letter, dx_pt=0, dy_pt=4):
    """'(a)' just above the top-left corner of the axes, the same place on every panel."""
    ax.annotate(f'({letter})', xy=(0, 1), xycoords='axes fraction', xytext=(dx_pt, dy_pt),
                textcoords='offset points', ha='left', va='bottom', fontsize=9)


def save_print(fig, stem):
    """PDF (for LaTeX) and a 300 dpi PNG (for looking) at exactly the figure's size."""
    os.makedirs(os.path.dirname(os.path.abspath(stem)), exist_ok=True)
    paths = [stem + '.pdf', stem + '.png']
    for p in paths:
        fig.savefig(p)
    plt.close(fig)
    return paths
