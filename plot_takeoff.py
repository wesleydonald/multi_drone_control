#!/usr/bin/env python3
"""
plot_takeoff.py - time-series view of a run, aimed at diagnosing takeoff.

Reads the per-drone log.csv files (one dir per drone under the logdir) and plots,
vs time:
  1. each drone's height: desired (ref_z, dotted) vs actual (pose_z, solid)
  2. payload height: desired vs actual (logged by drone 0)
  3. each drone's throttle command (u2)
  4. each drone's z error (ref_z - pose_z)

Usage:
    python3 plot_takeoff.py                 # latest run in the quad_load logs
    python3 plot_takeoff.py --show          # also open a window
    python3 plot_takeoff.py --run 20260715_155650
    python3 plot_takeoff.py --logdir <dir>
    python3 plot_takeoff.py --list
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
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR,
                    help="where to save the PNG (default: multi_drone_control/images)")
    ap.add_argument("--run", default=None, help="run ts YYYYmmdd_HHMMSS (default: latest)")
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
            ids = sorted(d["id"] for d in runs[ts])
            print(f"{ts}  drones={ids}  ({runs[ts][0]['traj']})")
        return

    from datetime import datetime
    def ts_to_sec(ts):
        return datetime.strptime(ts, "%Y%m%d_%H%M%S").timestamp()
    anchor = args.run or max(runs)
    if anchor not in runs:
        raise SystemExit(f"Run {anchor} not found. --list to see options.")
    anchor_sec = ts_to_sec(anchor)
    seen = {}
    for ts in runs:
        if abs(ts_to_sec(ts) - anchor_sec) <= args.window:
            for d in runs[ts]:
                if (d["id"] not in seen or abs(ts_to_sec(ts) - anchor_sec)
                        < abs(ts_to_sec(seen[d["id"]]["ts"]) - anchor_sec)):
                    seen[d["id"]] = d
    dirs = sorted(seen.values(), key=lambda d: d["id"])
    print(f"Run {anchor}: drones {[d['id'] for d in dirs]}")

    if args.show:
        matplotlib.use("TkAgg", force=True)
        globals()["plt"] = __import__("matplotlib.pyplot", fromlist=["x"])

    fig, ax = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    colors = plt.cm.tab10.colors
    payload_done = False

    for d in dirs:
        p = os.path.join(d["path"], "log.csv")
        if not os.path.exists(p):
            continue
        cols, data = load_csv(p)
        if data.size == 0:
            continue
        t = col(cols, data, "sim_time")
        t = t - t[0]
        m = np.ones_like(t, bool) if args.tmax is None else (t <= args.tmax)
        c = colors[d["id"] % len(colors)]
        pz, rz = col(cols, data, "pose_z"), col(cols, data, "ref_z")
        u2 = col(cols, data, "u2")

        # (0,0) drone height desired vs actual
        ax[0, 0].plot(t[m], pz[m], "-", color=c, lw=1.6, label=f"drone{d['id']} actual")
        if rz is not None:
            ax[0, 0].plot(t[m], rz[m], ":", color=c, lw=1.8, label=f"drone{d['id']} desired")
        # (1,0) drone z error
        if rz is not None:
            ax[1, 0].plot(t[m], (rz - pz)[m], "-", color=c, lw=1.4, label=f"drone{d['id']}")
        # (1,1) throttle
        if u2 is not None:
            ax[1, 1].plot(t[m], u2[m], "-", color=c, lw=1.4, label=f"drone{d['id']}")

        # (0,1) payload height desired vs actual (drone 0 only)
        if not payload_done:
            lz, qz = col(cols, data, "payload_z"), col(cols, data, "payload_ref_z")
            if lz is not None:
                good = ~np.isnan(lz)
                mm = m & good
                ax[0, 1].plot(t[mm], lz[mm], "-", color="k", lw=1.8, label="payload actual")
                if qz is not None:
                    mq = m & ~np.isnan(qz)
                    ax[0, 1].plot(t[mq], qz[mq], ":", color="k", lw=1.8, label="payload desired")
                payload_done = True

    ax[0, 0].set_title("Drone height: desired (dotted) vs actual (solid)")
    ax[0, 0].set_ylabel("z [m]")
    ax[0, 1].set_title("Payload height: desired vs actual")
    ax[0, 1].set_ylabel("z [m]")
    ax[1, 0].set_title("Drone z error (desired - actual)")
    ax[1, 0].set_ylabel("z error [m]"); ax[1, 0].set_xlabel("t [s]")
    ax[1, 0].axhline(0, color="gray", lw=0.6)
    ax[1, 1].set_title("Throttle command (u2)")
    ax[1, 1].set_ylabel("throttle [0-1]"); ax[1, 1].set_xlabel("t [s]")
    for a in ax.flat:
        a.grid(alpha=0.3); a.legend(fontsize=7)
    fig.suptitle(f"Takeoff diagnostics - run {anchor}", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    os.makedirs(args.outdir, exist_ok=True)
    out = os.path.join(args.outdir, f"takeoff_{anchor}.png")
    fig.savefig(out, dpi=120)
    print(f"Saved {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
