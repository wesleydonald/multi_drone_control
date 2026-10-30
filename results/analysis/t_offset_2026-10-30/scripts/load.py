import glob, json, os, re
import numpy as np, pandas as pd
ROOT = '/home/wesley/multi_drone_control'
RUNS = {'ours': (2965, 2972), 'nors': (2966, 2973), 'd05': (2967, 2974), 'd10': (2968, 2975),
        'd20': (2969, 2976), 'plan': (2970, 2977), 'nplan': (2971, 2978)}
DELAY = {'ours': 0.0, 'd05': 0.5, 'd10': 1.0, 'd20': 2.0, 'nors': None, 'plan': 0.0, 'nplan': None}

def rdir(r):
    h = glob.glob(f'{ROOT}/results/2*/R{r}_*')
    return h[0]

def quat_R(qw, qx, qy, qz):
    qw, qx, qy, qz = [np.asarray(v, float) for v in (qw, qx, qy, qz)]
    R = np.empty(qw.shape + (3, 3))
    R[..., 0, 0] = 1 - 2 * (qy**2 + qz**2); R[..., 0, 1] = 2 * (qx*qy - qz*qw); R[..., 0, 2] = 2 * (qx*qz + qy*qw)
    R[..., 1, 0] = 2 * (qx*qy + qz*qw); R[..., 1, 1] = 1 - 2 * (qx**2 + qz**2); R[..., 1, 2] = 2 * (qy*qz - qx*qw)
    R[..., 2, 0] = 2 * (qx*qz - qy*qw); R[..., 2, 1] = 2 * (qy*qz + qx*qw); R[..., 2, 2] = 1 - 2 * (qx**2 + qy**2)
    return R

class Run:
    def __init__(self, r):
        self.id = r
        self.dir = rdir(r)
        self.name = os.path.basename(self.dir)
        self.arm = self.name.split('_')[-1]
        L = os.path.join(self.dir, 'logs')
        self.df = pd.read_csv(os.path.join(L, 'run.csv')).drop_duplicates('t').reset_index(drop=True)
        self.ev = pd.read_csv(os.path.join(L, 'events.csv'))
        self.man = json.load(open(os.path.join(self.dir, 'manifest.json')))
        self.off = float(self.man.get('t_ready_sim', 0.0))
        self.te = float(self.ev[self.ev.event == 'RELEASE'].sim_time.iloc[0])
        land = self.ev[self.ev.event == 'LAND'].sim_time
        self.tland = float(land.iloc[0]) if len(land) else float(self.df.t.max())
        pf = glob.glob(os.path.join(L, 'dissipative_planner', '*', 'log.csv'))
        self.pl = pd.read_csv(pf[0], low_memory=False)
        self.pl['t'] = self.pl.sim_time - self.off
        self.tr = {}
        for f in glob.glob(os.path.join(L, 'tracker', 'drone*', 'log.csv')):
            k = int(re.search(r'drone(\d+)_', f).group(1))
            d = pd.read_csv(f, low_memory=False)
            d['t'] = d.sim_time - self.off
            self.tr[k] = d
        after = self.pl[(self.pl.t > self.te) & (self.pl.n == 3)]
        self.tres = float(after.t.iloc[0]) if len(after) else None
        self.launch = open(os.path.join(L, 'launch.log'), errors='replace').read()
        # ring tilt vector from run.csv quaternion
        R = quat_R(self.df.payload_qw, self.df.payload_qx, self.df.payload_qy, self.df.payload_qz)
        nrm = R[..., :, 2]
        self.df['nx'], self.df['ny'], self.df['nz'] = nrm[:, 0], nrm[:, 1], nrm[:, 2]
        self.R = R
    def at(self, t0, t1, src=None):
        d = self.df if src is None else src
        return d[(d.t >= t0) & (d.t < t1)]

def load_all():
    return {r: Run(r) for arm, rr in RUNS.items() for r in rr}
