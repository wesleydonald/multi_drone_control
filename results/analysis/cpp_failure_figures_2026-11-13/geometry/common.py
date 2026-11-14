"""Shared paths, styles and loaders for the geometry figures (read-only on the data)."""
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HOME = os.path.expanduser('~')
STALL = f'{HOME}/Downloads/tejen_hover_stall_2026-11-12'
RT = f'{HOME}/multi_drone_control/results/analysis/cpp_realtime_2026-11-13'
RES = f'{HOME}/multi_drone_control/results'
OUT = os.path.dirname(os.path.abspath(__file__))

R4374 = f'{STALL}/R4374_hover_s9_cpp_sw5_nows'
R4383 = f'{RES}/2026-11-12/R4383_sim_gz_t13_hover_s9_cpp_sw5_nows_oct_twin/logs'
R4392 = f'{RES}/2026-11-12/R4392_sim_gz_t13_hover_s9_cpp_sw5_nows_oct_tti30_twin/logs'

PLATE_R = 0.225
C_BLOCK = '#c0392b'     # blocked by his obstacle set
C_FREE = '#ffffff'
C_UNK = '#e6e6e6'
C_CARRIER = '#1f4e9c'   # carriers and their tethers
C_Q4 = '#e67e22'        # QUAD4 and its cable / object
C_DROP = '#1e8449'      # drop point
C_LEG = '#c0392b'
C_OCT = '#2471a3'

plt.rcParams.update({'figure.facecolor': 'white', 'axes.facecolor': 'white', 'savefig.facecolor': 'white',
                     'font.size': 10, 'axes.titlesize': 10.5, 'axes.grid': False, 'legend.fontsize': 8.5,
                     'legend.framealpha': 0.95})


def load_run(path):
    """run.csv -> dict of float arrays (numpy genfromtxt; empty cells become nan)."""
    d = np.genfromtxt(path, delimiter=',', names=True, invalid_raise=False)
    return {k: d[k] for k in d.dtype.names}


def events(path):
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                out.append((float(row['sim_time']), row['event'], row['arg']))
            except ValueError:
                pass
    return out


def at(run, t):
    i = int(np.argmin(np.abs(run['t'] - t)))
    return {k: v[i] for k, v in run.items()}


def plates(s):
    """Plate centres (twin): ring centre + PLATE_R towards each carrier, at the ring height."""
    c = np.array([s['payload_x'], s['payload_y'], s['payload_z']])
    out = []
    for i in range(3):
        b = np.array([s[f'd{i}_x'], s[f'd{i}_y'], s[f'd{i}_z']])
        u = (b[:2] - c[:2]) / np.linalg.norm(b[:2] - c[:2])
        out.append(np.array([c[0] + PLATE_R * u[0], c[1] + PLATE_R * u[1], c[2]]))
    return out


def seg_dist(p1, q1, p2, q2):
    """Closest distance between segments p1-q1 and p2-q2 (dense sampling, fine for a figure)."""
    s = np.linspace(0, 1, 401)
    a = p1[None, :] + s[:, None] * (q1 - p1)[None, :]
    b = p2[None, :] + s[:, None] * (q2 - p2)[None, :]
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    i, j = np.unravel_index(np.argmin(d), d.shape)
    return d[i, j], a[i], b[j]


def save(fig, name):
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('wrote', p)
