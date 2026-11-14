#!/usr/bin/env python3
"""Figures 5-7: the 29 Oct rig flights where Te-Jen's C++ transfer backend flew with the carriers.

Sources (read only): results/rig/2026-10-29/<run>.log (his node lines: online_join_planner and
dynamic_planner_transfer_backend status blocks, ROS epoch stamps) and <run>_run/logs/run.csv
(50 Hz grid from the bag's mocap; t = 0 at the manifest's t0_epoch, ~0.1 s after his TAKEOFF).
QUAD4 is d3 in run.csv.
"""
import json
import re
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DAY = Path("/home/wesley/multi_drone_control/results/rig/2026-10-29")
OUT = Path(__file__).resolve().parent
plt.rcParams.update({"font.size": 10.5, "axes.titlesize": 11.5, "axes.labelsize": 10.5,
                     "legend.fontsize": 9.5, "figure.dpi": 150})

TS = re.compile(r"^\[(?P<node>[\w\-]+)\] \[(?P<lvl>\w+)\] \[(?P<t>\d+\.\d+)\] \[(?P<name>[\w\.]+)\]: (?P<msg>.*)$")
BK = re.compile(r"^\[dynamic_planner_transfer_backend-\d+\] (?P<msg>.*)$")


def load(run):
    man = json.load(open(DAY / f"{run}_run/manifest.json"))
    t0 = man["t0_epoch"]
    d = pd.read_csv(DAY / f"{run}_run/logs/run.csv")
    ev = pd.read_csv(DAY / f"{run}_run/logs/events.csv")
    planner, blocks = [], []
    cur = None
    for line in open(DAY / f"{run}.log", errors="replace"):
        line = line.rstrip("\n")
        m = TS.match(line)
        if m:
            t = float(m["t"]) - t0
            if m["name"] == "online_join_planner" or (
                    m["name"] == "dynamic_planner_transfer_backend" and m["lvl"] in ("ERROR", "WARN")):
                planner.append((t, m["lvl"], m["msg"]))
            if m["name"] == "dynamic_planner_transfer_backend" and "LIVE-TARGET TRANSFER BACKEND" in m["msg"]:
                cur = {"t": t}
                blocks.append(cur)
            continue
        b = BK.match(line)
        if b and cur is not None:
            msg = b["msg"]
            if msg.startswith("latest_status:"):
                cur["status"] = msg.split(":", 1)[1].strip()
            elif msg.startswith("enable_requested/enabled:"):
                a, c = msg.split(":", 1)[1].split("/")
                cur["enable_req"], cur["enabled"] = a.strip() == "true", c.strip() == "true"
            elif msg.startswith("replans accepted/failed/late:"):
                cur["acc"], cur["fail"], cur["late"] = [int(x) for x in msg.split(":", 1)[1].split("/")]
            elif msg.startswith("prepared/fresh_pending/authority_granted"):
                v = msg.split(":", 1)[1].split("/")
                cur["prepared"], cur["granted"] = v[0].strip() == "true", v[2].strip() == "true"
            elif msg.startswith("target_received:"):
                mm = re.search(r"target=\[([^\]]+)\]", msg)
                if mm:
                    cur["target"] = [float(x) for x in mm.group(1).split()]
    bk = pd.DataFrame(blocks)
    return d, ev, planner, bk, t0


def speed(d):
    return np.sqrt(d.d3_vx ** 2 + d.d3_vy ** 2 + d.d3_vz ** 2)


def first(planner, needle):
    for t, _, msg in planner:
        if needle in msg:
            return t
    return None


def status_strip(ax, bk, colors, t_end):
    """Backend latest_status as coloured bands (one per 1 s status block)."""
    ts = bk.t.values
    for i, row in bk.iterrows():
        t1 = ts[i + 1] if i + 1 < len(ts) else min(row.t + 1.0, t_end)
        st = row.get("status", "")
        if not isinstance(st, str):
            st = ""
        c = colors.get(st, colors.get("*", "0.85"))
        ax.axvspan(row.t, t1, color=c, lw=0)
    ax.set_yticks([])


# ---------------------------------------------------------------- fig 5: sq3
def fig5():
    run = "sq3"
    d, ev, planner, bk, t0 = load(run)
    t = d.t.values
    sp = speed(d).values
    sp_s = pd.Series(sp).rolling(15, center=True, min_periods=1).mean().values  # 0.3 s centred mean
    t_climb = first(planner, "climb complete; latched")
    t_tb = first(planner, "TAKEN_BACK")
    i_c = int(np.searchsorted(t, t_climb))
    p_lift = d[["d3_x", "d3_y", "d3_z"]].values[i_c]
    drift = np.linalg.norm(d[["d3_x", "d3_y", "d3_z"]].values - p_lift, axis=1)

    rx = re.compile(r"drift=([\d.]+) m, speed=([\d.]+) m/s")
    settled = [(tt, *map(float, rx.search(m).groups())) for tt, _, m in planner if "stationary handoff state settled" in m]
    left = [(tt, *map(float, rx.search(m).groups())) for tt, _, m in planner if "left the prepare bounds" in m]

    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True,
                            gridspec_kw={"height_ratios": [3, 1.6, 1.6, 0.55]})
    ax = axs[0]
    win = (t > t_climb - 4) & (t < t_tb + 2)
    ax.plot(t[win], sp[win], color="0.65", lw=0.8, label="QUAD4 speed |v|, mocap 50 Hz (run.csv)")
    ax.plot(t[win], sp_s[win], color="tab:blue", lw=1.6, label="same, 0.3 s centred mean")
    ax.axhline(0.08, color="k", ls="--", lw=1.2, label="settle / stay bound 0.08 m/s (his rig default)")
    ax.scatter([s[0] for s in settled], [s[2] for s in settled], marker="^", s=55, color="tab:green", zorder=5,
               label=f"'stationary handoff state settled' ({len(settled)}x; his speed)")
    ax.scatter([s[0] for s in left], [s[2] for s in left], marker="v", s=55, color="tab:red", zorder=5,
               label=f"'left the prepare bounds' ({len(left)}x; his speed)")
    for s in settled:
        nxt = [l[0] for l in left if l[0] > s[0]]
        nxt_s = [x[0] for x in settled if x[0] > s[0]]
        end = min(nxt[0] if nxt else t_tb, nxt_s[0] if nxt_s else t_tb)
        ax.axvspan(s[0], end, color="tab:green", alpha=0.10, lw=0)
    ax.set_ylim(0, 0.25)
    ax.set_ylabel("speed (m/s)")
    ax.legend(loc="upper right", ncol=2, fontsize=8.8, framealpha=0.95)
    ax.set_title("sq3 (29 Oct rig): the hand-over to his C++ transit was never granted.\n"
                 "QUAD4's hover speed noise sits on the 0.08 m/s bound, so every 'settled' (green band) "
                 "is lost within 0.1-2 s.\nBehind it the backend never searched anyway: QUAD4 hovered at 2.14-2.20 m, "
                 "under the typed floor 2.36 m\n(status WAITING_ABOVE_SEARCH_FLOOR, 0 replans; panels 3 and 4)", loc="left")

    ax = axs[1]
    ax.plot(t[win], drift[win], color="tab:purple", lw=1.4, label="distance from the latched climb-end position")
    ax.axhline(0.15, color="k", ls="--", lw=1.2, label="drift bound 0.15 m")
    ax.scatter([s[0] for s in settled + left], [s[1] for s in settled + left], s=14, color="k", zorder=5,
               label="his logged drift at the events")
    ax.set_ylabel("drift (m)")
    ax.set_ylim(0, 0.2)
    ax.legend(loc="upper left", fontsize=8.8, ncol=3)

    ax = axs[2]
    ax.plot(t[win], d.d3_z.values[win], color="tab:blue", lw=1.4, label="QUAD4 body z (mocap)")
    ax.axhline(2.36, color="tab:red", ls="--", lw=1.2, label="typed search floor minimum_search_z_m 2.36 m")
    ax.set_ylabel("z (m)")
    ax.set_ylim(1.9, 2.5)
    ax.legend(loc="lower right", fontsize=8.8, ncol=2)

    ax = axs[3]
    colors = {"WAITING_ABOVE_SEARCH_FLOOR": "tab:orange", "*": "0.85"}
    status_strip(ax, bk[bk.t > t_climb - 4].reset_index(drop=True), colors, t_tb)
    ax.text(0.5, 0.5, "backend latest_status: orange = WAITING_ABOVE_SEARCH_FLOOR (0 replans), grey = other",
            transform=ax.transAxes, va="center", ha="center", fontsize=9,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.85))
    ax.set_xlabel(f"time since his TAKEOFF command (s)")
    for a in axs:
        a.axvline(t_climb, color="0.3", lw=1)
        a.axvline(t_tb, color="0.3", lw=1)
    axs[0].text(t_climb, 0.245, " climb complete", va="top", fontsize=9)
    axs[2].text(t_tb, 2.47, "TAKEN_BACK ", va="top", ha="right", fontsize=9)
    axs[0].set_xlim(t_climb - 4, t_tb + 2)
    fig.tight_layout()
    fig.savefig(OUT / "fig5_sq3_handover_speed.png", dpi=150)
    plt.close(fig)

    # numbers for the README
    hold = (t > t_climb) & (t < t_tb)
    durs = []
    for s in settled:
        nxt = [l[0] for l in left if l[0] > s[0]]
        nxt_s = [x[0] for x in settled if x[0] > s[0]]
        end = min(nxt[0] if nxt else t_tb, nxt_s[0] if nxt_s else t_tb)
        durs.append(end - s[0])
    print("fig5: climb end %.2f, taken back %.2f; settled %d, left %d; settled durations %s" % (
        t_climb, t_tb, len(settled), len(left), np.round(durs, 2)))
    print("fig5: left speeds", [s[2] for s in left], "accels in log only")
    print("fig5: hold |v| median %.3f p90 %.3f; frac > 0.08 raw %.2f smooth %.2f; drift max %.3f; z %.3f-%.3f" % (
        np.median(sp[hold]), np.percentile(sp[hold], 90), np.mean(sp[hold] > 0.08), np.mean(sp_s[hold] > 0.08),
        drift[hold].max(), d.d3_z.values[hold].min(), d.d3_z.values[hold].max()))
    print("fig5: backend statuses", bk[(bk.t > t_climb) & (bk.t < t_tb)].status.value_counts().to_dict(),
          "replans max", bk[["acc", "fail", "late"]].max().to_dict(),
          "enabled", bk[(bk.t > t_climb) & (bk.t < t_tb)].enabled.value_counts().to_dict())


# ---------------------------------------------------------------- fig 6: sq4_5
def fig6():
    run = "sq4_5"
    d, ev, planner, bk, t0 = load(run)
    t = d.t.values
    t_climb = first(planner, "climb complete; latched")
    t_settle = first(planner, "stationary handoff state settled")
    t_tb = first(planner, "TAKEN_BACK")
    t_land = float(ev[ev.event == "LAND"].sim_time.iloc[0])
    sp = speed(d).values

    rx = re.compile(r"drift=([\d.]+) m, speed=([\d.]+) m/s")
    settled = [(tt, *map(float, rx.search(m).groups())) for tt, _, m in planner if "stationary handoff state settled" in m]
    left = [(tt, *map(float, rx.search(m).groups())) for tt, _, m in planner if "left the prepare bounds" in m]
    above = (t > t_settle) & (t < t_tb) & (d.d3_z.values > 2.36)

    fig, axs = plt.subplots(3, 1, figsize=(10, 8.0), sharex=True,
                            gridspec_kw={"height_ratios": [3, 1.3, 0.55]})
    ax = axs[0]
    ax.plot(t, d.d3_z, color="tab:blue", lw=1.5, label="QUAD4 body z (mocap)")
    ax.axhline(2.36, color="tab:red", ls="--", lw=1.3, label="typed search floor 2.36 m (his backend plans only above it)")
    tz = [(r.t, r.target[2]) for _, r in bk.iterrows() if isinstance(r.get("target"), list)]
    if tz:
        ax.plot(*zip(*tz), ".", color="tab:green", ms=4, label="capture target z sent to the backend (76 % of them under the floor too)")
    ax.plot(t, d.payload_z, color="0.4", lw=1.0, label="ring centre z (mocap)")
    ax.set_ylabel("z (m)")
    ax.set_ylim(0, 2.75)
    ax.legend(loc="lower left", fontsize=9, bbox_to_anchor=(0.0, 0.02))
    ax.set_title("sq4_5 (29 Oct rig): the hand-over settled, then his backend waited 'above the search floor' "
                 "and never planned.\nQUAD4 held its climb end at 2.30-2.37 m, under the typed floor 2.36 m "
                 "for 98 % of the hold: 0 replans in 25 s", loc="left")
    ins = ax.inset_axes([0.36, 0.40, 0.45, 0.36])
    w = (t > t_climb - 1) & (t < t_tb)
    ins.plot(t[w], d.d3_z.values[w], color="tab:blue", lw=1.2)
    ins.axhline(2.36, color="tab:red", ls="--", lw=1.2)
    tt_above = t[above]
    if len(tt_above):
        ins.axvspan(tt_above[0], tt_above[-1], color="tab:green", alpha=0.25, lw=0)
        ins.annotate(f"above the floor {tt_above[0]:.1f}-{tt_above[-1]:.1f} s:\nstatus left WAITING for one\n"
                     "block, no plan, back under", xy=(tt_above[-1], 2.365), xytext=(tt_above[-1] + 0.8, 2.262),
                     fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.8))
    ins.set_ylim(2.25, 2.40)
    ins.set_xlim(t_climb - 1, t_tb)
    ins.set_title("zoom: the hold after the climb (z, m)", fontsize=9)
    ins.tick_params(labelsize=8)

    ax = axs[1]
    ax.plot(t, sp, color="0.6", lw=0.7, label="QUAD4 speed |v| (mocap)")
    ax.plot(t, pd.Series(sp).rolling(15, center=True, min_periods=1).mean(), color="tab:blue", lw=1.3,
            label="0.3 s mean")
    ax.axhline(0.12, color="k", ls="--", lw=1, label="hand-over speed bound 0.12 m/s (typed for this run)")
    ax.scatter([s_[0] for s_ in settled], [s_[2] for s_ in settled], marker="^", s=45, color="tab:green", zorder=5,
               label="settled")
    ax.scatter([s_[0] for s_ in left], [s_[2] for s_ in left], marker="v", s=45, color="tab:red", zorder=5,
               label="left the bounds")
    ax.set_ylim(0, 0.75)
    ax.set_ylabel("speed (m/s)")
    ax.legend(loc="upper center", fontsize=8.5, ncol=3)
    ax.text(0.30, 0.40, "backend replans accepted / failed / late: 0 / 0 / 0 for the whole flight",
            transform=ax.transAxes, fontsize=9.5, bbox=dict(facecolor="white", edgecolor="0.6"))

    ax = axs[2]
    colors = {"WAITING_ABOVE_SEARCH_FLOOR": "tab:orange", "*": "0.82"}
    status_strip(ax, bk, colors, t[-1])
    ax.set_xlabel("time since his TAKEOFF command (s)")
    for a in axs:
        for tt in (t_climb, t_settle, t_tb):
            a.axvline(tt, color="0.3", lw=1)
    axs[0].text(t_climb, 2.72, "climb\ncomplete ", ha="right", va="top", fontsize=9)
    axs[0].text(t_settle, 2.72, " hand-over settled,\n C++ preparation enabled", va="top", fontsize=9)
    axs[0].text(t_tb, 2.72, "TAKEN_BACK\n(operator LAND) ", ha="right", va="top", fontsize=9)
    ax.text(0.5, 0.5, "backend latest_status: orange = WAITING_ABOVE_SEARCH_FLOOR, grey = other",
            transform=ax.transAxes, ha="center", va="center", fontsize=9,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.85))
    axs[0].set_xlim(0, t[-1])
    fig.tight_layout()
    fig.savefig(OUT / "fig6_sq4_5_search_floor.png", dpi=150)
    plt.close(fig)

    hold = (t > t_settle) & (t < t_tb)
    wa = bk[(bk.status == "WAITING_ABOVE_SEARCH_FLOOR")]
    print("fig6: climb end %.2f settled %.2f TB %.2f LAND %.2f" % (t_climb, t_settle, t_tb, t_land))
    print("fig6: z in hold %.3f-%.3f median %.3f; WAITING blocks %d from %.1f to %.1f; replans max %s" % (
        d.d3_z.values[hold].min(), d.d3_z.values[hold].max(), np.median(d.d3_z.values[hold]), len(wa),
        wa.t.min(), wa.t.max(), bk[["acc", "fail", "late"]].max().to_dict()))
    print("fig6: statuses", bk.status.value_counts().to_dict())
    print("fig6: ring z hold median %.3f" % np.median(d.payload_z.values[hold]))
    print("fig6: frac z>2.36 in hold %.3f" % np.mean(d.d3_z.values[hold] > 2.36))
    tzz = np.array([r.target[2] for _, r in bk.iterrows() if isinstance(r.get("target"), list) and t_settle < r.t < t_tb])
    print("fig6: target z in hold min %.3f median %.3f max %.3f, frac under floor %.2f" % (tzz.min(), np.median(tzz), tzz.max(), np.mean(tzz < 2.36)))
    print("fig6: settled", [(round(a_, 2), b_, c_) for a_, b_, c_ in settled], "left", [(round(a_, 2), b_, c_) for a_, b_, c_ in left])


# ---------------------------------------------------------------- fig 7: sq2_5
def fig7():
    run = "sq2_5"
    d, ev, planner, bk, t0 = load(run)
    t = d.t.values
    phases = [(tt, m.split("Planner phase: ")[1].split(". Reason")[0], m.split("Reason: ")[-1])
              for tt, _, m in planner if "Planner phase:" in m]
    for p in phases:
        print("fig7 phase %.2f %s | %s" % p)
    t_cpp = first(planner, "authority source switched stationary Python hold -> C++")
    t_cap = first(planner, "authority source switched C++ -> Python")
    t_abort = [tt for tt, _, m in planner if "descent aborted" in m]
    t_match = [tt for tt, _, m in planner if "-> MATCH_VELOCITY" in m]
    t_desc = [tt for tt, _, m in planner if "-> DESCEND_TO_ATTACHMENT" in m]
    t_tb = first(planner, "TAKEN_BACK")
    t_climb = first(planner, "climb complete; latched")

    t_fb = first(planner, "execution certificate invalid")
    fb_msg = [m for _, _, m in planner if "execution certificate invalid" in m][0]
    print("fig7: fallback at %.2f: %s" % (t_fb, fb_msg))

    fig = plt.figure(figsize=(12, 9.4))
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.7], height_ratios=[1.5, 1, 0.9])
    axt = fig.add_subplot(gs[:, 0])
    fly = (t >= t_cpp) & (t <= t_fb)
    brk = (t > t_fb) & (t <= t_cap)
    pre = (t < t_cpp) & (t > t_climb - 3)
    post = (t > t_cap) & (t < t_tb)
    axt.plot(d.d3_x[pre], d.d3_y[pre], color="0.6", lw=1.2, label="QUAD4 climb-end hold (Python)")
    axt.plot(d.d3_x[fly], d.d3_y[fly], color="tab:blue", lw=2.2, label="C++ transit, flying")
    axt.plot(d.d3_x[brk], d.d3_y[brk], color="tab:purple", lw=1.6, label="C++ brake fallback, held 12 s")
    axt.plot(d.d3_x[post], d.d3_y[post], color="tab:orange", lw=1.0, label="Python approach / MATCH / descent")
    i_c = int(np.searchsorted(t, t_cap))
    for k, name in enumerate(["d0", "d1", "d2"]):
        x, y = d[f"{name}_x"].values[i_c], d[f"{name}_y"].values[i_c]
        axt.plot(x, y, "s", color="k", ms=8, label="carriers (at capture)" if k == 0 else None)
    px, py = d.payload_x.values[i_c], d.payload_y.values[i_c]
    th = np.linspace(0, 2 * np.pi, 100)
    axt.plot(px + 0.225 * np.cos(th), py + 0.225 * np.sin(th), color="k", lw=1.2, label="ring (plates r 0.225 m)")
    tg = [r.target for _, r in bk.iterrows() if isinstance(r.get("target"), list)]
    if tg:
        tg = np.array(tg)
        axt.plot(tg[:, 0], tg[:, 1], "x", color="tab:green", ms=7, mew=2, label="capture target xy (plate 9)")
    i_f = int(np.searchsorted(t, t_fb))
    axt.plot(d.d3_x.values[i_f], d.d3_y.values[i_f], "*", color="tab:red", ms=13, label="tracking-tube violation")
    axt.plot(d.d3_x.values[i_c], d.d3_y.values[i_c], "o", color="tab:red", ms=6, mfc="none", mew=2,
             label="capture region entered")
    axt.set_aspect("equal")
    axt.set_xlabel("x (m)")
    axt.set_ylabel("y (m)")
    axt.set_xlim(-1.0, 1.0)
    axt.set_ylim(-0.9, 2.1)
    axt.grid(alpha=0.3)
    h_, l_ = axt.get_legend_handles_labels()
    fig.legend(h_, l_, loc="lower left", bbox_to_anchor=(0.01, 0.005), fontsize=8.8, ncol=5, frameon=True)
    axt.set_title("top view", loc="left")

    def marks(a, txt=False):
        a.axvspan(t_cpp, t_fb, color="tab:blue", alpha=0.12, lw=0)
        a.axvspan(t_fb, t_cap, color="tab:purple", alpha=0.12, lw=0)
        for tt in t_match:
            a.axvline(tt, color="tab:green", lw=1, ls="--")
        for tt in t_desc:
            a.axvline(tt, color="tab:purple", lw=1.2)
        for tt in t_abort:
            a.axvline(tt, color="tab:red", lw=1.4)
        a.axvline(t_tb, color="0.3", lw=1)

    axz = fig.add_subplot(gs[0, 1])
    marks(axz)
    axz.plot(t, d.d3_z, color="tab:blue", lw=1.5, label="QUAD4 body z")
    axz.plot(t, d.payload_z, color="0.4", lw=1.0, label="ring centre z")
    # Not logged: the launch's formula at 413ec0e (ring 1.0 + 1.0 sin60 + 0.04, carry_clearance 0.25, magnet 0.05,
    # rod 1.041, margin 0.05) gives the unloaded search floor 3.197 m.
    axz.axhline(3.197, color="tab:red", ls="--", lw=1.1, label="search floor ~3.20 m (launch formula; not logged)")
    axz.set_ylabel("z (m)")
    axz.set_ylim(0, 3.75)
    ins = axz.inset_axes([0.12, 0.37, 0.36, 0.23])
    wz = (t > t_cpp - 2) & (t < t_cap + 1)
    ins.axvspan(t_cpp, t_fb, color="tab:blue", alpha=0.12, lw=0)
    ins.axvspan(t_fb, t_cap, color="tab:purple", alpha=0.12, lw=0)
    ins.plot(t[wz], d.d3_z.values[wz], color="tab:blue", lw=1.2)
    ins.axhline(3.197, color="tab:red", ls="--", lw=1.1)
    ins.plot(t_fb, d.d3_z.values[np.searchsorted(t, t_fb)], "*", color="tab:red", ms=10)
    ins.set_ylim(3.14, 3.26)
    ins.set_xlim(t_cpp - 2, t_cap + 1)
    ins.tick_params(labelsize=7.5)
    ins.set_title("zoom: body z vs the floor; star = violation (3.194 m)", fontsize=8)
    axz.legend(loc="lower left", fontsize=8.8)
    y0 = 3.7
    axz.text(t_cpp, y0, " C++ has\n authority", va="top", fontsize=8.8)
    axz.text(t_fb, y0, " brake fallback;\n restart blocked", va="top", fontsize=8.8)
    axz.text(t_cap, y0, " capture region:\n Python takes over", va="top", fontsize=8.8)
    axz.text(t_tb, y0, "TAKEN_BACK ", ha="right", va="top", fontsize=8.8)
    axz.annotate(f"MATCH {t_desc[0] - t_match[0]:.1f} s, descent\naborted after {t_abort[0] - t_desc[0]:.1f} s "
                 "(lost alignment),\nMATCH again until take-back", xy=(t_abort[0], 2.3), xytext=(t_cap + 1.5, 1.35),
                 fontsize=8.5, arrowprops=dict(arrowstyle="->", lw=0.8))

    axh = fig.add_subplot(gs[1, 1], sharex=axz)
    marks(axh)
    hsp = np.hypot(d.d3_vx, d.d3_vy)
    dxy = np.hypot(d.d3_x - px, d.d3_y - py)
    axh.plot(t, pd.Series(hsp).rolling(15, center=True, min_periods=1).mean(), color="tab:blue", lw=1.3,
             label="QUAD4 horizontal speed (0.3 s mean, m/s)")
    axh.plot(t, dxy, color="tab:red", lw=1.4, label="QUAD4 xy distance to ring centre (m)")
    axh.set_ylim(0, 2.0)
    axh.set_ylabel("m/s  |  m")
    axh.legend(loc="upper right", fontsize=8.5)

    axr = fig.add_subplot(gs[2, 1], sharex=axz)
    marks(axr)
    axr.plot(bk.t, bk.acc, color="tab:green", lw=1.5, label="replans accepted")
    axr.plot(bk.t, bk.fail, color="tab:red", lw=1.5, label="failed")
    axr.plot(bk.t, bk.late, color="tab:orange", lw=1.2, label="late")
    axr.set_ylabel("backend replans\n(cumulative)")
    axr.set_xlabel("time since his TAKEOFF command (s)")
    axr.legend(loc="center right", fontsize=8.5)
    axz.set_xlim(t_climb - 3, t_tb + 3)
    plt.setp(axz.get_xticklabels(), visible=False)
    plt.setp(axh.get_xticklabels(), visible=False)
    fig.suptitle(f"sq2_5 (29 Oct rig): his C++ transit flew {t_fb - t_cpp:.0f} s (1.82 -> 0.37 m from the ring centre, at 3.2 m). "
                 "Then a tracking-tube violation\n(x error 5.3 cm > 5 cm bound) installed a brake fallback whose restart "
                 "was blocked (its stop point under the backend's search floor):\n"
                 f"QUAD4 hovered {t_cap - t_fb:.0f} s with 0 replans until Python's capture-region check took over; "
                 "the Python descents then failed", x=0.01, ha="left", fontsize=11)
    fig.tight_layout(rect=(0, 0.075, 1, 0.925))
    fig.savefig(OUT / "fig7_sq2_5_transit.png", dpi=150)
    plt.close(fig)

    cpp = (t >= t_cpp) & (t <= t_cap)
    print("fig7: C++ on %.2f, capture %.2f (%.1f s), climb end %.2f, TB %.2f" % (t_cpp, t_cap, t_cap - t_cpp, t_climb, t_tb))
    print("fig7: z during C++ %.3f-%.3f; ring z %.3f; xy dist at C++ on %.2f, at capture %.2f" % (
        d.d3_z.values[cpp].min(), d.d3_z.values[cpp].max(), np.median(d.payload_z.values[cpp]),
        dxy.values[np.searchsorted(t, t_cpp)], dxy.values[i_c]))
    print("fig7: horizontal speed during C++ max %.3f, time moving >0.05: %.1f s" % (
        hsp.values[cpp].max(), np.sum(hsp.values[cpp] > 0.05) * 0.02))
    print("fig7: replans max", bk[["acc", "fail", "late"]].max().to_dict(), "statuses", bk.status.value_counts().to_dict())
    print("fig7: matches", np.round(t_match, 2), "descents", np.round(t_desc, 2), "aborts", np.round(t_abort, 2))
    print("fig7: z at fallback %.3f, z during brake hold min %.3f median %.3f max %.3f" % (
        d.d3_z.values[np.searchsorted(t, t_fb)], d.d3_z.values[brk].min(), np.median(d.d3_z.values[brk]), d.d3_z.values[brk].max()))
    print("fig7: z at capture %.3f, at first descend %.3f" % (d.d3_z.values[i_c], d.d3_z.values[np.searchsorted(t, t_desc[0])]))


if __name__ == "__main__":
    fig5()
    fig6()
    fig7()
