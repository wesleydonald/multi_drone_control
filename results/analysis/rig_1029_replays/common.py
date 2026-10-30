"""Shared loaders for the 29 Oct rig replays (results/rig/2026-10-29): our planner and tracker CSVs, the bag topics,
and Tejen's join-planner CSV. Times are epoch seconds throughout (the logs and the bag share the laptop clock)."""
import glob
import os

import numpy as np
import pandas as pd

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
DAY = os.environ.get("RIG_DAY", os.path.join(REPO, "results", "rig", "2026-10-29"))
OUT = os.path.dirname(os.path.abspath(__file__))
G = 9.81


def planner(run):
    f = sorted(glob.glob(os.path.join(DAY, f'{run}_logs', 'logs', 'dissipative_planner', '*', 'log.csv')))
    return pd.read_csv(f[0])


def trackers(run):
    """{drone index: DataFrame} from the run's tracker logs."""
    out = {}
    for f in sorted(glob.glob(os.path.join(DAY, f'{run}_logs', 'logs', 'tracker', 'drone*', 'log.csv'))):
        i = int(os.path.basename(os.path.dirname(f)).split('_')[0][5:])
        out[i] = pd.read_csv(f)
    return out


def read_bag(run, topics):
    """{topic: list of (t_bag_s, msg)} for the listed topics."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=os.path.join(DAY, run), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in topics if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    out = {t: [] for t in topics}
    while r.has_next():
        tp, raw, t = r.read_next()
        out[tp].append((t * 1e-9, deserialize_message(raw, cls[tp])))
    return out


def pose_array(msgs):
    """[t, x, y, z, qx, qy, qz, qw, vx, vy, vz] per message (MotionCaptureState or PoseStamped)."""
    rows = []
    for t, m in msgs:
        p, q = m.pose.position, m.pose.orientation
        tw = getattr(m, 'twist', None)
        v = (tw.linear.x, tw.linear.y, tw.linear.z) if tw is not None else (np.nan,) * 3
        rows.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w, *v])
    return np.array(rows, float)


def join_planner_csv(stamp):
    """Tejen's join-planner CSV (5 Hz) by its file stamp, e.g. '2026-10-29_10-28-45'."""
    f = os.path.join(REPO, 'logs', 'join_planner', f'join_planner_clean_{stamp}_drone_0.csv')
    return pd.read_csv(f, low_memory=False)


def hold(arr, tq):
    """Sample-and-hold rows of arr (column 0 = time) at the query times."""
    return arr[np.clip(np.searchsorted(arr[:, 0], tq, side='right') - 1, 0, None)]
