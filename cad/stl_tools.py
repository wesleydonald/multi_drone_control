"""Small numpy helpers for Wesley's STL parts (no mesh library needed): load, bounding box, horizontal slices as loops."""
import struct
import numpy as np


def load_stl(path):
    """Triangles (n, 3, 3) in the file's units; binary or ASCII."""
    with open(path, 'rb') as f:
        head = f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        data = f.read()
    if len(data) == n * 50:
        rec = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n)
        return rec['v'].astype(float)
    txt = (head + struct.pack('<I', n) + data).decode('ascii', 'ignore')
    v = [list(map(float, l.split()[1:4])) for l in txt.splitlines() if l.strip().startswith('vertex')]
    return np.array(v, float).reshape(-1, 3, 3)


def slice_loops(tri, axis, c, tol=1e-6):
    """Closed loops where the plane coordinate[axis] == c cuts the mesh: a list of (k, 2) arrays in the other two axes."""
    other = [i for i in range(3) if i != axis]
    segs = []
    for t in tri:
        d = t[:, axis] - c
        pts = []
        for i in range(3):
            a, b = t[i], t[(i + 1) % 3]
            da, db = d[i], d[(i + 1) % 3]
            if (da > tol and db < -tol) or (da < -tol and db > tol):
                s = da / (da - db)
                pts.append(a + s * (b - a))
        if len(pts) == 2:
            segs.append((pts[0][other], pts[1][other]))
    if not segs:
        return []
    key = lambda p: (round(p[0], 3), round(p[1], 3))
    adj = {}
    for a, b in segs:
        adj.setdefault(key(a), []).append(key(b))
        adj.setdefault(key(b), []).append(key(a))
    seen, loops = set(), []
    for start in list(adj):
        if start in seen:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        while True:
            nxt = [q for q in adj[cur] if q != prev and q not in seen]
            if not nxt:
                break
            prev, cur = cur, nxt[0]
            seen.add(cur)
            loop.append(cur)
        if len(loop) > 2:
            loops.append(np.array(loop))
    return loops


def loop_props(lp):
    """(centroid, area (signed), equivalent diameter, bbox size) of a closed polygon."""
    x, y = lp[:, 0], lp[:, 1]
    a = 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)
    c = lp.mean(axis=0)
    return c, a, 2 * np.sqrt(abs(a) / np.pi), lp.max(axis=0) - lp.min(axis=0)
