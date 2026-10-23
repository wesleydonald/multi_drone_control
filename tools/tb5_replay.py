#!/usr/bin/env python3
"""tools/tb5_replay.py -- TB5, the take-back's desk replay (card 2026-10-10_combined_m1_contact rev 10 §6.1 with
the critic's round 4 response, F2 and F4). No Gazebo.

    tools/tb5_replay.py                       # every case below, a table on stdout
    tools/tb5_replay.py --json out.json       # and every number in a file

The formation is KINEMATIC: the planned ring point c(tau) = hover + the trajectory's offset, and each carrier (and the
ring) at c(tau - lag) + a fixed offset, both fitted on the cleanest lap of the same run before his give-up (fit
residual <= 2 cm). QUAD4 flies its reference exactly, its rod hanging straight down. The take-back's reference comes
from the planner's own code (DissipativeController's methods on a stand-in node, as the unit tests run them): this
tree's, or c6f2032's (git show de91ba2) for the reproduction of R1565.

Cases:
  - repro: c6f2032's take-back on R1565, on the LOGGED carriers and on the kinematic formation (the logged failure:
    carrier 2 within ~0.28 m at ~119.4 s);
  - the logged give-ups of R1565, R1554, R1566, R1549 and R1543 (this tree);
  - R1565's give-up swept through the lap in 10 deg steps (QUAD4 kept on plate 9 relative to the planned point);
  - F1's three latches on R1565: his DESCEND_TO_PICKUP at the ring's nearest phase, his LIFT_OBJECT climb with the
    ball, his TRANSIT_TO_REATTACH descent; each as a give-up (the clock running) and at LAND (the clock frozen).

Reported per case (the §5 take-back bars, frozen 11 Oct): the minimum rod-segment clearance (QUAD4's rod, body to tip,
to the ball's far side while carried, against every carrier body and carrier rod; >= 0.15 m confirms, < 0.10 m
falsifies), the minimum body clearance (>= 0.35 / < 0.30 m), the closest ring centre in xy after the decouple (F2
bar >= 0.9 m), the overflight time (a carrier's 0.2 m rotor disc under QUAD4's body or rod in xy) and the minimum
vertical gap from QUAD4's tip to a carrier body during it, the decouple point against the swept disc, and the ring's
yaw change over the fitted lap. Acceptance: rod-segment >= 0.15 m and body >= 0.35 m in every case, the F2 bar, and
QUAD4 down within 60 s of the take-back (F5).
"""
import argparse
import glob
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
from types import MethodType, SimpleNamespace

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in ('dissipative_planner', 'mpc_planner'):
    sys.path.insert(0, os.path.join(REPO, 'src', pkg))

from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402
from run_dir import manifest_azimuths_ccw  # noqa: E402

HZ = 10.0
SUB = 5                      # geometry sub-samples per planner tick
ROTOR_R = 0.2                # m, a carrier's rotor disc (§5)
BALL_DIAM = 0.16             # m, his twin ball (tejen payload_model.sdf, r 0.08)
BASE_REV = 'de91ba2'         # m1-tiltcut4: c6f2032's take-back
RUNS = ('R1565', 'R1554', 'R1566', 'R1549', 'R1543')
METHODS = ('_take_back', '_publish_departed_refs', '_pos', '_reserved_index', '_reserved_plate_dir',
           '_path_clear_point', '_departed_down', '_drop_departed', '_take_back_plan', '_take_back_tick',
           '_take_back_next', '_ring_moves', '_ring_plan', '_guard_setup', '_guard_read', '_guard_check',
           '_weld_rho_of', '_guard_why')


def results_roots():
    """results/ of this tree and of the main tree (a worktree's holds only the registry)."""
    common = subprocess.run(['git', '-C', REPO, 'rev-parse', '--git-common-dir'], capture_output=True, text=True)
    main = os.path.dirname(os.path.abspath(os.path.join(REPO, common.stdout.strip())))
    return [os.path.join(REPO, 'results'), os.path.join(main, 'results')]


def run_dir(rid):
    hits = sorted(h for root in results_roots() for h in glob.glob(os.path.join(root, '*', f'{rid}_*')))
    if not hits:
        raise SystemExit(f'{rid}: no run directory')
    return hits[-1]


def planner_module(rev=None):
    """dissipative_node of this tree, or of a git revision (loaded under its own module name)."""
    if rev is None:
        from dissipative_planner import dissipative_node
        return dissipative_node
    src = subprocess.run(['git', '-C', REPO, 'show', f'{rev}:src/dissipative_planner/dissipative_planner/'
                          'dissipative_node.py'], capture_output=True, text=True, check=True).stdout
    path = os.path.join(tempfile.mkdtemp(), f'dissipative_node_{rev}.py')
    open(path, 'w').write(src)
    spec = importlib.util.spec_from_file_location(f'dissipative_node_{rev}', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ── the run and its kinematic formation ───────────────────────────────────────────────────────────────────────────

class Run:
    def __init__(self, rid):
        self.rid = rid
        d = run_dir(rid)
        self.run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
        self.ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
        self.pl = pd.read_csv(sorted(glob.glob(os.path.join(d, 'logs', 'dissipative_planner', '*', 'log.csv')))[-1])
        full = json.load(open(os.path.join(d, 'manifest.json')))
        man = full['launch_args']
        self.az = manifest_azimuths_ccw(man.get('attach_azimuths_deg'), full) or [270.0, 30.0, 150.0]
        self.n = int(man.get('num_drones') or 3)
        self.attach_r = float(man.get('attach_radius') or 0.225)
        self.attach_z = float(man.get('attach_z') or 0.02)
        # QUAD4's rod from its pivot as the planner took it (untyped: the carriers' cable_len; TB5/TB6 flew 0.55 m)
        self.btt = (float(man.get('attach_cable_len') or man.get('cable_len') or 0.55)
                    - float(man.get('pivot_offset_z') or -0.04))
        self.plate9 = np.array([float(man.get('attach_x_offset') or 0.0), float(man.get('attach_y_offset') or 0.225),
                                self.attach_z])
        self.traj = LoadTrajectory.from_params(man, kind='orbit', speed=0.125, distance=0.0)
        self.lap = 2.0 * np.pi * self.traj.radius / self.traj.speed
        t = self.run.t.to_numpy()
        # the planner's clock against the runner's: fit the offset on the measured ring
        m = (self.pl.sim_time > self.pl.sim_time.iloc[-1] * 0.3).to_numpy()

        def err(o):
            return float(np.mean((np.interp(self.pl.sim_time[m] - o, t, self.run.payload_x) - self.pl.load_x[m]) ** 2
                                 + (np.interp(self.pl.sim_time[m] - o, t, self.run.payload_y)
                                    - self.pl.load_y[m]) ** 2))
        # coarse then fine: the offset was 5.4 s on R1565 and over 10 s on TB6's runs (the planner starts later)
        o0 = min((err(o), o) for o in np.arange(-60.0, 60.0, 0.1))[1]
        best = min((err(o), o) for o in np.arange(o0 - 0.15, o0 + 0.15, 0.01))
        self.clock_off = best[1]
        self.tau = np.interp(t + self.clock_off, self.pl.sim_time, self.pl.traj_t)
        ref = self.run[['payload_ref_x', 'payload_ref_y']].to_numpy()
        on = self.tau > 1.0
        off = np.array([self.traj.offset_at(x)[:2] for x in self.tau[on]])
        self.hover = (ref[on] - off).mean(0)
        # the trajectory's offset on a grid, for the fits (np.interp in place of one offset_at call per sample)
        self._tg = np.arange(-5.0, float(self.tau.max()) + 2.0 * self.lap + 120.0, 0.01)
        self._cg = np.array([self.traj.offset_at(x)[:2] for x in self._tg])
        self.t_give = self.event('GIVE_UP')
        self.fit_formation()

    def event(self, name, after=-1.0):
        e = self.ev[(self.ev.event == name) & (self.ev.sim_time > after)]
        return float(e.sim_time.iloc[0]) if len(e) else None

    def c(self, tau):
        dx, dy, vx, vy = self.traj.offset_at(tau)
        return self.hover + np.array([dx, dy]), np.array([vx, vy])

    def fit_formation(self):
        """Each carrier and the ring as c(tau - lag) + offset, on the cleanest whole lap before the give-up."""
        t = self.run.t.to_numpy()
        t_orbit = float(t[np.argmax(self.tau > 0.0)]) + 4.0 + 1.0         # after the spin-up ramp
        t_last = (self.t_give or t[-1]) - 0.5
        bodies = {i: self.run[[f'd{i}_x', f'd{i}_y', f'd{i}_z']].to_numpy() for i in range(self.n)}
        bodies['ring'] = self.run[['payload_x', 'payload_y', 'payload_z']].to_numpy()
        best = None
        for a in np.arange(t_orbit, t_last - self.lap + 1e-6, 2.5):
            w = (t >= a) & (t <= a + self.lap)
            fits, worst = {}, 0.0
            for k, p in bodies.items():
                f = self._fit_one(t[w], self.tau[w], p[w])
                fits[k] = f
                worst = max(worst, f['rms'])
            if best is None or worst < best[0]:
                best = (worst, a, fits)
        if best is None:
            raise SystemExit(f'{self.rid}: no whole lap before the give-up')
        self.fit_rms, self.fit_t0, self.fits = best
        w = (t >= self.fit_t0) & (t <= self.fit_t0 + self.lap)
        q = self.run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy()[w]
        yaw = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))
        self.ring_yaw_change = float(np.ptp(np.unwrap(np.radians(yaw)))) * 180.0 / np.pi

    def _fit_one(self, t, tau, p):
        best = None
        for lag in np.arange(0.0, 1.0, 0.01):
            c = self.hover + np.c_[np.interp(tau - lag, self._tg, self._cg[:, 0]),
                                   np.interp(tau - lag, self._tg, self._cg[:, 1])]
            o = (p[:, :2] - c).mean(0)
            r = np.linalg.norm(p[:, :2] - c - o, axis=1)
            rms = float(np.sqrt(np.mean(r ** 2)))
            if best is None or rms < best['rms']:
                best = {'lag': float(lag), 'off': o, 'z': float(p[:, 2].mean()), 'rms': rms, 'max': float(r.max())}
        return best

    def at(self, tau):
        """(ring centre xyz, [carrier body xyz]) of the kinematic formation at trajectory time tau."""
        def one(k):
            f = self.fits[k]
            return np.r_[self.c(tau - f['lag'])[0] + f['off'], f['z']]
        return one('ring'), [one(i) for i in range(self.n)]

    def logged(self, t):
        """The same from run.csv at runner time t (the c6f2032 reproduction)."""
        i = min(int(np.searchsorted(self.run.t.to_numpy(), t)), len(self.run) - 1)
        r = self.run.iloc[i]
        return (np.array([r.payload_x, r.payload_y, r.payload_z]),
                [np.array([r[f'd{k}_x'], r[f'd{k}_y'], r[f'd{k}_z']]) for k in range(self.n)])

    def quad4(self, t):
        i = min(int(np.searchsorted(self.run.t.to_numpy(), t)), len(self.run) - 1)
        r = self.run.iloc[i]
        k = f'd{self.n}_'
        return (np.array([r[k + 'x'], r[k + 'y'], r[k + 'z']]), np.array([r[k + 'vx'], r[k + 'vy'], r[k + 'vz']]))

    def ball_on(self, t):
        up, drop = self.event('PICKUP'), self.event('DROP')
        return up is not None and up <= t and (drop is None or t < drop)

    def tau_at(self, t):
        return float(np.interp(t, self.run.t, self.tau))


# ── the stand-in planner node ─────────────────────────────────────────────────────────────────────────────────────

class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = warning = error = info


class _Pub:
    def publish(self, m):
        pass


def stand_in(mod, run, tau, p0, v0, ball):
    """The planner's take-back state at a latch: the kinematic formation at tau, QUAD4 at p0 moving at v0."""
    ring, carriers = run.at(tau)
    log = _Log()
    f = SimpleNamespace(
        _partner_contact=True, _newcomer_lifecycle=True, _newcomer_fault_land=True, _newcomer_armed={},
        _n_carry0=run.n, reserved_attach=1, n=run.n, attach_pending=[True], attach_pos=[np.asarray(p0, float)],
        _attach_vel=[np.asarray(v0, float)], _attach_pos_t=[None], _approach={}, _newcomer_out={}, _land_alone=set(),
        _off_resend_t=None, _departed_hold={}, _departed_step={}, _departed_next={}, _departed_td={},
        _departed_clear={}, _departed_landing=set(), _departed_wait={}, _departed_land=False, _departed_land_vel=0.0,
        land_vel=0.2, _departed_down_z=0.15, _land_to_ground=False, _dropped=None, _landed=False, takeoff_seen=True,
        phase='planner', load_state=np.r_[ring, 1.0, 0.0, 0.0, 0.0, np.zeros(6)], drone_pos=list(carriers),
        dyn=SimpleNamespace(g=9.81), N=20, dt=0.1, drone_mass=0.55, attach_radius=run.attach_r,
        _attach_body_to_tip=run.btt, _magnet_cmd_pub=_Pub(), _take_back_pub=_Pub(), _fleet_cmd_pub=_Pub(),
        traj=run.traj, hover_xy=tuple(run.hover), _departed_gi=False, _rest_z={run.n: 0.1}, _departed_sent=set(),
        _departed_pubs={}, _attach_target=None,
        net=SimpleNamespace(rho=[None] * run.n + [run.plate9]), _his_phase='DESCEND_TO_ATTACHMENT', _his_took_off=True,
        _mux_state='partner', _joint_free=True, _ball_on=ball, _ball_mass=0.1, _span_t=0.0, _taken_back=False,
        traj_t=float(tau), _comove={}, _traj_running=True, _release_guard=False, _tip_now=None, _tilt_faulted=set(),
        _press_t={}, _at_rest_t={})
    f.get_logger = lambda: log
    f.log = log
    f.now = [0.0]
    f.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(f.now[0] * 1e9)))
    for name in METHODS:
        if hasattr(mod.DissipativeController, name):
            setattr(f, name, MethodType(getattr(mod.DissipativeController, name), f))
    f.refs = []
    f._publish_ref = lambda d, nodes: f.refs.append(np.array(nodes[0][0], float))
    return f


# ── geometry ──────────────────────────────────────────────────────────────────────────────────────────────────────

def seg_seg(p1, q1, p2, q2):
    """Closest distance between segments p1-q1 and p2-q2."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a < 1e-12 and e < 1e-12:
        return float(np.linalg.norm(r))
    if a < 1e-12:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e < 1e-12:
            t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = np.clip((b * f - c * e) / den, 0.0, 1.0) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    return float(np.linalg.norm(p1 + d1 * s - p2 - d2 * t))


def geometry(run, body, ring, carriers, ball):
    """(rod-segment clearance, body clearance, [(carrier, xy distance, tip-to-carrier vertical gap)])."""
    tip = body - np.array([0.0, 0.0, run.btt + (BALL_DIAM if ball else 0.0)])
    tip[2] = max(tip[2], 0.0)
    rod = bod = np.inf
    over = []
    for i, cb in enumerate(carriers):
        a = np.radians(run.az[i])
        plate = ring + np.array([run.attach_r * np.cos(a), run.attach_r * np.sin(a), run.attach_z])
        rod = min(rod, seg_seg(body, tip, cb, cb), seg_seg(body, tip, cb, plate))
        bod = min(bod, float(np.linalg.norm(body - cb)))
        dxy = float(np.linalg.norm(body[:2] - cb[:2]))
        if dxy < ROTOR_R and cb[2] < body[2]:
            over.append((i, dxy, float(tip[2] - cb[2])))
    return rod, bod, over


# ── one replay ────────────────────────────────────────────────────────────────────────────────────────────────────

def replay(run, mod, tau0, p0, v0, ball, why='his give-up', logged_t0=None, t_max=90.0):
    """Fly the take-back from a latch at trajectory time tau0. logged_t0: use run.csv's carriers from that runner
    time (the c6f2032 reproduction) instead of the kinematic formation."""
    f = stand_in(mod, run, tau0, p0, v0, ball)
    land = why == 'LAND'
    f._take_back(0, why)
    if land:
        f._land_to_ground = True
    d = run.n
    rows = []
    tau = tau0
    body_prev = np.asarray(p0, float)
    t_dc = None
    zone = f._comove[d]['zone'] if d in getattr(f, '_comove', {}) else None
    k = 0
    while k < int(t_max * HZ):
        f.refs.clear()
        f._traj_running = not land
        if f._departed_hold:
            f._publish_departed_refs()
        if logged_t0 is not None:
            ring, carriers = run.logged(logged_t0 + k / HZ)
        else:
            ring, carriers = run.at(tau)
        body = f.refs[-1] if f.refs else body_prev
        for s in range(SUB):                                   # between ticks: QUAD4 and the formation move linearly
            b = body_prev + (body - body_prev) * (s + 1) / SUB
            rod, bod, over = geometry(run, b, ring, carriers, ball)
            rows.append({'t': k / HZ + (s + 1) / (SUB * HZ), 'rod': rod, 'body': bod, 'over': over,
                         'ring_xy': float(np.linalg.norm(b[:2] - ring[:2])), 'decoupled': t_dc is not None,
                         'z': float(b[2])})
        body_prev = body
        if t_dc is None and any('decoupled' in ln for ln in f.log.lines):
            t_dc = (k + 1) / HZ
            dc_point = body.copy()
        if any(' is down' in ln for ln in f.log.lines):
            break
        if not land:
            tau += 1.0 / HZ
            f.traj_t = tau
        ring, carriers = run.at(tau) if logged_t0 is None else run.logged(logged_t0 + (k + 1) / HZ)
        f.load_state[0:3] = ring
        f.drone_pos[:run.n] = carriers
        f.attach_pos[0] = body
        f.now[0] += 1.0 / HZ
        k += 1
    rod = min(r['rod'] for r in rows)
    bod = min(r['body'] for r in rows)
    i_rod = int(np.argmin([r['rod'] for r in rows]))
    i_bod = int(np.argmin([r['body'] for r in rows]))
    after = [r['ring_xy'] for r in rows if r['decoupled']]
    over = [r for r in rows if r['over']]
    centre = run.hover + np.array([0.0, run.traj.radius])
    out = {'zone': zone, 'rod_min': round(rod, 3), 'rod_at': round(rows[i_rod]['t'], 1),
           'body_min': round(bod, 3), 'body_at': round(rows[i_bod]['t'], 1),
           'after_decouple_ring_xy_min': round(min(after), 3) if after else None,
           'overflight_s': round(len(over) / (SUB * HZ), 2),
           'overflight_gap_min': round(min(g for r in over for _, _, g in r['over']), 3) if over else None,
           'decouple_t': t_dc, 'down_t': round(k / HZ, 1), 'down': any(' is down' in ln for ln in f.log.lines),
           'take_back': next((ln for ln in f.log.lines if 'TAKE-BACK' in ln), '')}
    if t_dc is not None:
        out['decouple_r_path'] = round(float(np.linalg.norm(dc_point[:2] - centre)), 3)
        out['decouple_z'] = round(float(dc_point[2]), 3)
    out['swept_disc_r'] = round(run.traj.radius + 0.5 + ROTOR_R, 2)
    return out


def logged_clearance(run):
    """The same geometry on the logged QUAD4 and carriers from the give-up to the end of the data: the failure the
    reproduction must match (R1565: carrier 2 within ~0.28 m at ~119.4 s)."""
    best = {'body': (np.inf, None, None), 'rod': (np.inf, None)}
    for t in np.arange(run.t_give, float(run.run.t.iloc[-1]), 0.02):
        body, _ = run.quad4(t)
        ring, car = run.logged(t)
        rod, bod, _ = geometry(run, body, ring, car, run.ball_on(t))
        if bod < best['body'][0]:
            best['body'] = (round(bod, 3), round(float(t), 2),
                            int(np.argmin([np.linalg.norm(body - c) for c in car])) + 1)
        if rod < best['rod'][0]:
            best['rod'] = (round(rod, 3), round(float(t), 2))
    return best


def latch_from_log(run, t):
    p, v = run.quad4(t)
    return run.tau_at(t), p, v, run.ball_on(t)


def shifted(run, tau, p, v, phase_deg):
    """QUAD4's latch moved phase_deg along the lap, kept where it was relative to the planned ring point."""
    tau2 = tau + phase_deg / 360.0 * run.lap
    c, vc = run.c(tau)
    c2, vc2 = run.c(tau2)
    return tau2, np.r_[c2 + (p[:2] - c), p[2]], np.r_[vc2 + (v[:2] - vc), v[2]]


def nearest_phase(run, tau, p, v):
    """The phase shift (deg) that brings the ring centre nearest QUAD4."""
    best = min(((float(np.linalg.norm(run.at(tau + ph / 360.0 * run.lap)[0][:2] - p[:2])), ph)
                for ph in range(0, 360, 5)))
    return best[1]


def bars(r):
    rod = 'confirms' if r['rod_min'] >= 0.15 else ('FALSIFIES' if r['rod_min'] < 0.10 else 'between')
    body = 'confirms' if r['body_min'] >= 0.35 else ('FALSIFIES' if r['body_min'] < 0.30 else 'between')
    dc = r['after_decouple_ring_xy_min']
    f2 = '-' if dc is None else ('ok' if dc >= 0.9 else 'FAIL')
    return rod, body, f2


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--json')
    ap.add_argument('--runs', nargs='*', default=list(RUNS))
    a = ap.parse_args(argv)
    new, old = planner_module(), planner_module(BASE_REV)
    runs = {rid: Run(rid) for rid in a.runs}
    res = {'fits': {}, 'cases': {}}
    for rid, run in runs.items():
        res['fits'][rid] = {'lap_t0': round(run.fit_t0, 1), 'rms_worst': round(run.fit_rms, 4),
                            'max_worst': round(max(f['max'] for f in run.fits.values()), 4),
                            'lags': {str(k): round(f['lag'], 2) for k, f in run.fits.items()},
                            'ring_yaw_change_deg': round(run.ring_yaw_change, 2), 'clock_off': round(run.clock_off, 2)}
    cases = res['cases']
    r65 = runs.get('R1565')
    if r65 is not None:
        res['logged_R1565'] = logged_clearance(r65)
        tau, p, v, ball = latch_from_log(r65, r65.t_give)
        cases['repro c6f2032 R1565 logged carriers'] = replay(r65, old, tau, p, v, ball, logged_t0=r65.t_give,
                                                              t_max=r65.run.t.iloc[-1] - r65.t_give - 0.1)
        cases['repro c6f2032 R1565 kinematic'] = replay(r65, old, tau, p, v, ball)
    for rid, run in runs.items():
        tau, p, v, ball = latch_from_log(run, run.t_give)
        cases[f'{rid} give-up {run.t_give:.1f} s'] = replay(run, new, tau, p, v, ball)
    if r65 is not None:
        tau, p, v, ball = latch_from_log(r65, r65.t_give)
        for ph in range(0, 360, 10):
            t2, p2, v2 = shifted(r65, tau, p, v, ph)
            cases[f'R1565 sweep {ph:3d} deg'] = replay(r65, new, t2, p2, v2, ball)
        for name, phase, frac in (('DESCEND_TO_PICKUP', 'PHASE:DESCEND_TO_PICKUP', 0.8),
                                  ('LIFT_OBJECT climb', 'PHASE:LIFT_OBJECT', None),
                                  ('TRANSIT_TO_REATTACH descent', 'PHASE:TRANSIT_TO_REATTACH', None)):
            t0 = r65.event(phase)
            if t0 is None:
                continue
            # his rejoin descent runs from TRANSIT_TO_REATTACH through APPROACH_ABOVE_TARGET and MATCH_VELOCITY
            t1 = r65.event('PHASE:DESCEND_TO_ATTACHMENT', t0) if name.startswith('TRANSIT') else \
                float(r65.ev.sim_time[r65.ev.sim_time > t0 + 0.01].iloc[0])
            if frac is not None:
                t = t0 + frac * (t1 - t0)
            else:                                               # mid-climb / mid-descent by height
                w = (r65.run.t >= t0) & (r65.run.t <= t1)
                z = r65.run[f'd{r65.n}_z'][w].to_numpy()
                tt = r65.run.t[w].to_numpy()
                t = float(tt[np.argmin(np.abs(z - 0.5 * (z.min() + z.max())))])
            tau, p, v, ball = latch_from_log(r65, t)
            if name == 'DESCEND_TO_PICKUP':
                ph = nearest_phase(r65, tau, p, v)
                tau, p, v = tau + ph / 360.0 * r65.lap, p, v
                name += f' (ring nearest, +{ph} deg)'
            for why in ('his give-up', 'LAND'):
                cases[f'F1 {name} at {t:.1f} s, {why}'] = replay(r65, new, tau, p, v, ball, why=why)
    if a.json:
        json.dump(res, open(a.json, 'w'), indent=1, default=float)
    print('fits (worst body: rms / max m; lags s; ring yaw change over the lap deg):')
    for rid, f in res['fits'].items():
        print(f'  {rid}: lap from {f["lap_t0"]} s, rms {f["rms_worst"]:.3f} max {f["max_worst"]:.3f}, '
              f'yaw {f["ring_yaw_change_deg"]:.2f}, lags {f["lags"]}')
    if 'logged_R1565' in res:
        lg = res['logged_R1565']
        print(f'R1565 as flown (logged QUAD4): body min {lg["body"][0]} m at {lg["body"][1]} s (carrier '
              f'{lg["body"][2]}), rod min {lg["rod"][0]} m at {lg["rod"][1]} s; the reproductions below put QUAD4 '
              f'on c6f2032\'s reference (times from the give-up {runs["R1565"].t_give:.1f} s)')
    print()
    print('| case | zone | rod min (m) @t | body min (m) @t | rod bar | body bar | ring xy after decouple (F2 >= 0.9) |'
          ' overflight s / min gap (m) | decouple t, r path, z | down |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    ok = True
    for name, r in cases.items():
        rod, body, f2 = bars(r)
        if not name.startswith('repro'):           # + F5's bound: down within 60 s of the take-back
            ok &= rod == 'confirms' and body == 'confirms' and f2 != 'FAIL' and r['down'] and r['down_t'] <= 60.0
        dc = '-' if r['decouple_t'] is None else f'{r["decouple_t"]:.1f} s, {r["decouple_r_path"]}, {r["decouple_z"]}'
        print(f'| {name} | {r["zone"]} | {r["rod_min"]} @{r["rod_at"]} | {r["body_min"]} @{r["body_at"]} | {rod} |'
              f' {body} | {r["after_decouple_ring_xy_min"]} {f2} | {r["overflight_s"]} / {r["overflight_gap_min"]} |'
              f' {dc} | {"yes" if r["down"] else "NO"} {r["down_t"]} s |')
    print()
    print('TB5', 'PASSES' if ok else 'FAILS',
          '(every case but the reproductions: rod >= 0.15, body >= 0.35, F2 >= 0.9, down within 60 s)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
