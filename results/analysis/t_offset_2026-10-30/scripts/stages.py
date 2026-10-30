import glob, json, os, re, sys
import numpy as np, pandas as pd
sys.path.insert(0, os.path.dirname(__file__))
from load import quat_R
ROOT = '/home/wesley/multi_drone_control'
def arg(txt, k, default=None):
    m = re.search(k + r':=([^\s\']+)', txt)
    return m.group(1) if m else default
class R:
    def __init__(self, tag):
        self.dir = glob.glob(f'{ROOT}/results/2*/{tag}_*')[0]
        L = self.dir + '/logs'
        self.df = pd.read_csv(L + '/run.csv').drop_duplicates('t').reset_index(drop=True)
        self.ev = pd.read_csv(L + '/events.csv')
        self.off = float(json.load(open(self.dir + '/manifest.json')).get('t_ready_sim', 0))
        self.pl = pd.read_csv(glob.glob(L + '/dissipative_planner/*/log.csv')[0], low_memory=False)
        self.pl['t'] = self.pl.sim_time - self.off
        self.tr = {}
        for f in glob.glob(L + '/tracker/drone*/log.csv'):
            k = int(re.search(r'drone(\d+)_', f).group(1)); d = pd.read_csv(f, low_memory=False); d['t'] = d.sim_time - self.off; self.tr[k] = d
        txt = open(L + '/launch.log', errors='replace').read()
        self.az = [float(v) for v in arg(txt, 'attach_azimuths_deg').split(',')]
        self.r = float(arg(txt, 'attach_radius', 0.225)); self.az_z = float(arg(txt, 'attach_z', 0.02))
        self.m = float(arg(txt, 'load_mass', 0.86)); self.md = float(arg(txt, 'drone_mass', 0.55))
        Rm = quat_R(self.df.payload_qw, self.df.payload_qx, self.df.payload_qy, self.df.payload_qz)
        self.df['nx'], self.df['ny'] = Rm[:, 0, 2], Rm[:, 1, 2]
    def stage(self, a, b):
        d = self.df[(self.df.t >= a) & (self.df.t < b)]
        p = self.pl[(self.pl.t >= a) & (self.pl.t < b)]
        p = p[p.phase == 'planner']
        out = dict(tilt=float(np.degrees(np.arcsin(np.hypot(d.nx.mean(), d.ny.mean())))),
                   tilt_mag=float(d.payload_tilt_deg.mean()),
                   ndir=float(np.degrees(np.arctan2(d.ny.mean(), d.nx.mean()))), n=int(p.n.mode()[0]))
        ids = [int(v) for v in str(p.slot2drone.mode()[0]).split()]
        out['ids'] = ids
        out['T'] = {k: float(pd.to_numeric(p[f't{s}']).mean()) for s, k in enumerate(ids)}
        out['Tz'] = {k: float(-pd.to_numeric(p[f'tsz{s}']).mean()) for s, k in enumerate(ids)}
        F = {}
        for k in ids:
            tr = self.tr[k]; tr = tr[(tr.t >= a) & (tr.t < b)]
            Rd = quat_R(tr.pose_qw, tr.pose_qx, tr.pose_qy, tr.pose_qz)
            est = np.array([0, 0, 9.81]) - (tr.kt_mpc.values * (tr.thr_out.values - tr.thr_off.values))[:, None] * Rd[:, :, 2]
            F[k] = self.md * est.mean(0)
        out['Fdrone'] = F
        out['int_f'] = p[['int_fx', 'int_fy', 'int_fz']].apply(pd.to_numeric, errors='coerce').mean().values
        return out
    def plate(self, k):
        return int(round(self.az[k] / 30.0)) % 12
