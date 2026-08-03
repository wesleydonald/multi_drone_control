#!/usr/bin/env python3
"""
plot_run.py — plot desired vs actual trajectories from a controller run.

Reads the per-drone log.csv files written by DataLogger (one dir per drone:
<logdir>/<traj>_drone<id>_<YYYYmmdd_HHMMSS>/log.csv) and plots the desired
(ref_x/y/z) against the actual (pose_x/y/z) for every drone in the run.

Usage:
    python3 plot_run.py                 # latest run, controller_mpc_multi logs
    python3 plot_run.py --show          # also open an interactive window
    python3 plot_run.py --run 20260701_135906    # a specific run timestamp
    python3 plot_run.py --logdir c_generated_code_quad_load/logs/controller_quad_load
    python3 plot_run.py --list          # list available runs and exit

The figure (XY path + x/y/z-vs-time + position-error-vs-time) is saved as
run_<timestamp>.png inside the newest drone's log directory, and its path is
printed.  Runs with no ref_* columns (logs from before that was added) still
plot the actual trajectory, with a warning.
"""
import argparse
import csv
import glob
import os
import re
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")  # switched to a GUI backend below if --show
import matplotlib.pyplot as plt

_REPO = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOGDIR = os.path.join(
    _REPO, "c_generated_code_quad_load", "logs", "controller_quad_load")
DEFAULT_OUTDIR = os.path.join(_REPO, "images")

DIR_RE = re.compile(r"^(?P<traj>.+)_drone(?P<id>\d+)_(?P<ts>\d{8}_\d{6})$")


def parse_dir(path):
    m = DIR_RE.match(os.path.basename(path))
    if not m:
        return None
    return {"path": path, "traj": m["traj"],
            "id": int(m["id"]), "ts": m["ts"]}


def find_runs(logdir):
    """Return {ts: [drone_dir_info, ...]} grouped by run timestamp."""
    runs = defaultdict(list)
    for p in glob.glob(os.path.join(logdir, "*_drone*_*")):
        if not os.path.isdir(p):
            continue
        info = parse_dir(p)
        if info:
            runs[info["ts"]].append(info)
    return runs


def _as_float(v):
    """Cell -> float, with non-numeric cells becoming NaN.

    Not every logged column is a number: the controller logs a few human-readable
    status strings (kt_status is 'waiting_for_takeoff', 'updated', 'frozen', ...).
    Those are worth keeping in the CSV for reading a run back, so parse them to NaN
    rather than refusing to load the file. Plots that ask for a numeric column are
    unaffected; anything plotting a status column gets a gap, which is honest."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def load_csv(csv_path):
    with open(csv_path) as f:
        reader = csv.reader(f)
        headers = next(reader)
        cols = {h: i for i, h in enumerate(headers)}
        n = len(headers)
        rows = []
        for row in reader:
            if not row:
                continue
            # Tolerate a short final row: a run killed with ctrl-c can leave the
            # last line half-written, which would otherwise make the array ragged.
            if len(row) != n:
                if len(row) < n:
                    continue
                row = row[:n]
            rows.append([_as_float(v) for v in row])
        data = np.array(rows)
    return cols, data


def get(cols, data, name):
    return data[:, cols[name]] if name in cols else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--logdir", default=DEFAULT_LOGDIR,
                    help="base logs dir (default: controller_quad_load)")
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR,
                    help="where to save the PNG (default: multi_drone_control/images)")
    ap.add_argument("--run", default=None,
                    help="run timestamp YYYYmmdd_HHMMSS (default: latest)")
    ap.add_argument("--window", type=float, default=120.0,
                    help="seconds: group drone dirs within this of the run ts")
    ap.add_argument("--show", action="store_true", help="open interactive window")
    ap.add_argument("--list", action="store_true", help="list runs and exit")
    args = ap.parse_args()

    runs = find_runs(args.logdir)
    if not runs:
        raise SystemExit(f"No drone log dirs found under {args.logdir}")

    if args.list:
        for ts in sorted(runs):
            ids = sorted(d["id"] for d in runs[ts])
            print(f"{ts}  drones={ids}  ({runs[ts][0]['traj']})")
        return

    # Choose the run. Drones in one run may start a few seconds apart, so group
    # every drone dir whose ts is within --window of the anchor ts.
    anchor = args.run or max(runs)
    if anchor not in runs:
        raise SystemExit(f"Run {anchor} not found. Use --list to see options.")

    def ts_to_sec(ts):
        from datetime import datetime
        return datetime.strptime(ts, "%Y%m%d_%H%M%S").timestamp()

    anchor_sec = ts_to_sec(anchor)
    dirs = []
    seen_ids = {}
    for ts in runs:
        if abs(ts_to_sec(ts) - anchor_sec) <= args.window:
            for d in runs[ts]:
                # keep the dir closest to the anchor per drone id
                if (d["id"] not in seen_ids
                        or abs(ts_to_sec(ts) - anchor_sec)
                        < abs(ts_to_sec(seen_ids[d["id"]]["ts"]) - anchor_sec)):
                    seen_ids[d["id"]] = d
    dirs = sorted(seen_ids.values(), key=lambda d: d["id"])

    print(f"Run {anchor}: plotting drones {[d['id'] for d in dirs]}")

    if args.show:
        matplotlib.use("TkAgg", force=True)
        globals()["plt"] = __import__("matplotlib.pyplot", fromlist=["x"])

    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d proj)
    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")

    colors = plt.cm.tab10.colors
    any_ref = False
    payload_plotted = False

    for d in dirs:
        csv_path = os.path.join(d["path"], "log.csv")
        if not os.path.exists(csv_path):
            print(f"  ! missing {csv_path}")
            continue
        cols, data = load_csv(csv_path)
        if data.size == 0:
            print(f"  ! empty {csv_path}")
            continue
        c = colors[d["id"] % len(colors)]
        px, py, pz = (get(cols, data, n) for n in ("pose_x", "pose_y", "pose_z"))
        rx, ry, rz = (get(cols, data, n) for n in ("ref_x", "ref_y", "ref_z"))
        lbl = f"drone{d['id']}"

        # actual trajectory (solid) + start/end markers
        ax.plot(px, py, pz, "-", color=c, lw=1.8, label=f"{lbl} actual")
        ax.scatter(px[0], py[0], pz[0], color=c, marker="o", s=30)   # start
        ax.scatter(px[-1], py[-1], pz[-1], color=c, marker="X", s=45)  # end

        if rx is not None:
            any_ref = True
            # desired trajectory (dotted, same color)
            ax.plot(rx, ry, rz, ":", color=c, lw=2.0, label=f"{lbl} desired")

        # Payload track (logged by drone 0 only): overlay actual (solid) vs
        # desired (dotted) in black so it stands out from the drones.
        if not payload_plotted:
            lx, ly, lz = (get(cols, data, n)
                          for n in ("payload_x", "payload_y", "payload_z"))
            if lx is not None:
                m = ~(np.isnan(lx) | np.isnan(ly) | np.isnan(lz))
                if m.any():
                    ax.plot(lx[m], ly[m], lz[m], "-", color="k", lw=2.6,
                            label="payload actual")
                    ax.scatter(lx[m][0], ly[m][0], lz[m][0], color="k",
                               marker="o", s=45)
                    ax.scatter(lx[m][-1], ly[m][-1], lz[m][-1], color="k",
                               marker="X", s=60)
                    qx, qy, qz = (get(cols, data, n) for n in
                                  ("payload_ref_x", "payload_ref_y", "payload_ref_z"))
                    if qx is not None:
                        mq = ~(np.isnan(qx) | np.isnan(qy) | np.isnan(qz))
                        if mq.any():
                            any_ref = True
                            ax.plot(qx[mq], qy[mq], qz[mq], ":", color="k",
                                    lw=2.6, label="payload desired")
                    payload_plotted = True

    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    title = f"Run {anchor} — actual (solid) vs desired (dotted); o=start, X=end"
    if not any_ref:
        title += "\n(no ref_* columns in these logs — actual only)"
    ax.set_title(title, fontsize=11)
    ax.legend(fontsize=8)
    _equalize_3d(ax)

    os.makedirs(args.outdir, exist_ok=True)
    out = os.path.join(args.outdir, f"run_{anchor}.png")
    fig.savefig(out, dpi=120)
    print(f"Saved {out}")
    if args.show:
        plt.show()


def _equalize_3d(ax):
    """Give the 3D axes an equal aspect ratio so paths aren't distorted."""
    xlim, ylim, zlim = ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()
    spans = [xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]]
    r = max(spans) / 2.0
    cx, cy, cz = (np.mean(xlim), np.mean(ylim), np.mean(zlim))
    ax.set_xlim3d(cx - r, cx + r)
    ax.set_ylim3d(cy - r, cy + r)
    ax.set_zlim3d(cz - r, cz + r)


if __name__ == "__main__":
    main()
