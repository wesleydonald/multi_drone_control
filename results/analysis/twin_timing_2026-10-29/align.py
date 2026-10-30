"""Map his MPC log's wall-clock timestamps onto a twin run's sim time: run.csv's wall is seconds since the runner
started, so find that start (wall0) from QUAD4's height (his pose_z against run.csv's d3_z), coarse by the first
crossing of 0.5 m, then refined by least squares over +-1 s."""
import numpy as np


def sim_time_of(run_df, mpc_df):
    rw = run_df[['wall', 't', 'd3_z']].dropna()
    w, t, z = rw.wall.values, rw.t.values, rw.d3_z.values
    ts, pz = mpc_df.timestamp.values, mpc_df.pose_z.values
    i_run, i_mpc = np.argmax(z > 0.5), np.argmax(pz > 0.5)
    w0 = ts[i_mpc] - w[i_run]

    def cost(c):
        tt = np.interp(ts - c, w, t)
        m = (ts - c > w[0]) & (ts - c < w[-1])
        return np.nanmean((np.interp(tt[m], t, z) - pz[m]) ** 2) if m.sum() > 50 else np.inf
    grid = w0 + np.arange(-1.0, 1.0, 0.01)
    best = grid[int(np.argmin([cost(c) for c in grid]))]
    return (lambda e: np.interp(np.asarray(e) - best, w, t)), best, float(np.sqrt(cost(best)))
