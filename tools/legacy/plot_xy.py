#!/usr/bin/env python3
"""
plot_xy.py - xy tracking + commanded attitude, for diagnosing takeoff xy drift.

The z loop can track well while xy drifts if the per-drone MPC is commanding
roll/pitch it shouldn't. This plots, vs time:
  1. x: desired (dotted) vs actual (solid), per drone
  2. y: desired vs actual, per drone
  3. xy error magnitude, per drone
  4. commanded roll (u0) and pitch (u1), per drone   <- the suspect

If the xy error grows at takeoff *at the same time* the roll/pitch commands
spike, the attitude loop is the problem (feedforward tilt or the cable term).

Usage:
    python3 plot_xy.py                # latest run, quad_load logs -> images/
    python3 plot_xy.py --tmax 8       # zoom the takeoff
    python3 plot_xy.py --run <ts> --list --show --outdir <dir>
"""
import argparse, csv, glob, os, re
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_REPO = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOGDIR = os.path.join(
    _REPO, "c_generated_code_quad_load", "logs", "controller_quad_load")
DEFAULT_OUTDIR = os.path.join(_REPO, "images")
DIR_RE = re.compile(r"^(?P<traj>.+)_drone(?P<id>\d+)_(?P<ts>\d{8}_\d{6})$")


def find_runs(logdir):
    runs = defaultdict(list)
    for p in glob.glob(os.path.join(logdir, "*_drone*_*")):
        m = DIR_RE.match(os.path.basename(p))
        if m and os.path.isdir(p):
            runs[m["ts"]].append({"path": p, "id": int(m["id"]), "ts": m["ts"],
                                  "traj": m["traj"]})
    return runs


def load_csv(path):
    with open(path) as f:
        r = csv.reader(f)
        cols = {h: i for i, h in enumerate(next(r))}
        data = np.array([[float(v) for v in row] for row in r if row])
    return cols, data


def col(cols, data, name):
    return data[:, cols[name]] if name in cols and data.size else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--logdir", default=DEFAULT_LOGDIR)
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR)
    ap.add_argument("--run", default=None)
    ap.add_argument("--window", type=float, default=120.0)
    ap.add_argument("--tmax", type=float, default=None,
                    help="only plot the first TMAX seconds (zoom the takeoff)")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    runs = find_runs(args.logdir)
    if not runs:
        raise SystemExit(f"No drone log dirs under {args.logdir}")
    if args.list:
        for ts in sorted(runs):
            print(f"{ts}  drones={sorted(d['id'] for d in runs[ts])}")
        return

    from datetime import datetime
    ts_sec = lambda ts: datetime.strptime(ts, "%Y%m%d_%H%M%S").timestamp()
    anchor = args.run or max(runs)
    if anchor not in runs:
        raise SystemExit(f"Run {anchor} not found. --list to see options.")
    seen = {}
    for ts in runs:
        if abs(ts_sec(ts) - ts_sec(anchor)) <= args.window:
            for d in runs[ts]:
                if (d["id"] not in seen or abs(ts_sec(ts) - ts_sec(anchor))
                        < abs(ts_sec(seen[d["id"]]["ts"]) - ts_sec(anchor))):
                    seen[d["id"]] = d
    dirs = sorted(seen.values(), key=lambda d: d["id"])
    print(f"Run {anchor}: drones {[d['id'] for d in dirs]}")

    if args.show:
        matplotlib.use("TkAgg", force=True)
        globals()["plt"] = __import__("matplotlib.pyplot", fromlist=["x"])

    fig, ax = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    colors = plt.cm.tab10.colors
    for d in dirs:
        p = os.path.join(d["path"], "log.csv")
        if not os.path.exists(p):
            continue
        cols, data = load_csv(p)
        if data.size == 0:
            continue
        t = col(cols, data, "sim_time"); t = t - t[0]
        m = np.ones_like(t, bool) if args.tmax is None else (t <= args.tmax)
        c = colors[d["id"] % len(colors)]
        px, py = col(cols, data, "pose_x"), col(cols, data, "pose_y")
        rx, ry = col(cols, data, "ref_x"), col(cols, data, "ref_y")
        u0, u1 = col(cols, data, "u0"), col(cols, data, "u1")

        ax[0, 0].plot(t[m], px[m], "-", color=c, lw=1.5, label=f"drone{d['id']} actual")
        if rx is not None:
            ax[0, 0].plot(t[m], rx[m], ":", color=c, lw=1.8, label=f"drone{d['id']} desired")
        ax[0, 1].plot(t[m], py[m], "-", color=c, lw=1.5, label=f"drone{d['id']} actual")
        if ry is not None:
            ax[0, 1].plot(t[m], ry[m], ":", color=c, lw=1.8, label=f"drone{d['id']} desired")
        if rx is not None and ry is not None:
            exy = np.hypot(rx - px, ry - py)
            ax[1, 0].plot(t[m], exy[m], "-", color=c, lw=1.5, label=f"drone{d['id']}")
        if u0 is not None:
            ax[1, 1].plot(t[m], u0[m], "-", color=c, lw=1.3, label=f"d{d['id']} roll")
        if u1 is not None:
            ax[1, 1].plot(t[m], u1[m], "--", color=c, lw=1.3, label=f"d{d['id']} pitch")

    ax[0, 0].set_title("x: desired (dotted) vs actual (solid)"); ax[0, 0].set_ylabel("x [m]")
    ax[0, 1].set_title("y: desired vs actual"); ax[0, 1].set_ylabel("y [m]")
    ax[1, 0].set_title("xy error magnitude"); ax[1, 0].set_ylabel("|xy err| [m]")
    ax[1, 0].set_xlabel("t [s]"); ax[1, 0].axhline(0, color="gray", lw=0.6)
    ax[1, 1].set_title("commanded roll (solid) / pitch (dashed)")
    ax[1, 1].set_ylabel("cmd"); ax[1, 1].set_xlabel("t [s]"); ax[1, 1].axhline(0, color="gray", lw=0.6)
    for a in ax.flat:
        a.grid(alpha=0.3); a.legend(fontsize=7)
    fig.suptitle(f"xy tracking + attitude - run {anchor}", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    os.makedirs(args.outdir, exist_ok=True)
    out = os.path.join(args.outdir, f"xy_{anchor}.png")
    fig.savefig(out, dpi=120)
    print(f"Saved {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
