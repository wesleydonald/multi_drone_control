#!/usr/bin/env python3
"""Contact events on every rig descent onto a plate under proof_rules=comotion, and an offline replay of the contact
gate at other normal_max values.

Inputs: data/<day>_<flight>.npz/.json (extract.py), the flight's .log and launches.csv row in results/rig/<day>/.
The replay runs the flight code's own ComotionProof (src/drone_magnet/drone_magnet/comotion_proof.py, unchanged since
14 Oct) on the bag's /magnet_tip_pose and /payload/motion_capture_state, as magnet_attachment_manager feeds it: one
step per tip pose (bag receive time as the node clock), the plate point = ring pose + the typed offset rotated by the
ring's attitude, fresh when the ring pose is at most 0.1 s old, the body when at most 0.1 s old, his lift-done flag
from /join_planner/proof_lift_complete, the magnet ON/OFF from the logged proof state, and the abort when his phase
leaves the contact phases during a proof.

usage: analyze.py            (writes events.csv, replay.csv, flights.csv, verify.csv, windows/*.csv)"""
import csv, json, math, os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'drone_magnet'))
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof  # noqa: E402

RIG = os.path.join(REPO, 'results', 'rig')
CONTACT_PHASES = ('MATCH_VELOCITY', 'DESCEND_TO_ATTACHMENT', 'ATTACH_READY', 'LIFT_PROOF', 'RING_ATTACHED')
GATES = (0.04, 0.02, 0.01)
FRESH_S = 0.1
POST_S = 2.0
# QUAD4's command-to-response delay on the rig is 45-65 ms (5 Nov): the logged tip is still the no-lift tip for ~0.1 s
# after the real contact, so a counterfactual contact up to that late is still read from unchanged data
VALID_AFTER_S = 0.1

# (day, flight, kind): every comotion-rules flight whose proof left OFF (QUAD4's magnet ON over a descent)
FLIGHTS = [
    ('2026-10-15', 'a1', 'attach'), ('2026-10-15', 'a2_2', 'attach'), ('2026-10-15', 'a3_2', 'attach'),
    ('2026-10-29', 'sq2_5', 'attach'), ('2026-10-29', 'sq5_2', 'attach'),
    ('2026-11-04', 'nr9', 're-attach'), ('2026-11-04', 'nr9m', 're-attach'), ('2026-11-04', 'sq6b', 'attach'),
    ('2026-11-04', 'sq6m', 'attach'), ('2026-11-04', 'nr1_2', 're-attach'), ('2026-11-04', 'nr9m_4', 're-attach'),
    ('2026-11-04', 'nr9m_5', 're-attach'),
    ('2026-11-05', 'nr1r', 're-attach'), ('2026-11-05', 'nr1r_3', 're-attach'), ('2026-11-05', 'nr9m', 're-attach'),
    ('2026-11-05', 'nr9o_2', 're-attach'), ('2026-11-05', 'sq8_2', 're-attach'), ('2026-11-05', 'fm9y_2', 're-attach'),
    ('2026-11-11', 'nrtfc_2', 're-attach'), ('2026-11-11', 'nrtfc_3', 're-attach'),
    ('2026-11-11', 'nrtfc_5', 're-attach'),
    ('2026-11-12', 'fm9_5', 're-attach'), ('2026-11-12', 'fm9hc_2', 're-attach'), ('2026-11-12', 'fm9hc_4', 're-attach'),
    ('2026-11-12', 'fm9hc_5', 're-attach'), ('2026-11-12', 'nro', 're-attach'),
    ('2026-11-13', 'fm9c_2', 're-attach'),
]
# whether a proven catch stayed on its plate (registry rows / results feed; checked against the tip below)
HELD_NOTE = {
    ('2026-11-04', 'sq6m'): 'held: FLEET RESIZED n=4, LAND (RIG-1104-sq6m)',
    ('2026-11-05', 'sq8_2'): 'held until the false-loss ring drop in LAND (RIG-1105-sq8_2)',
    ('2026-11-11', 'nrtfc_2'): 'held to the landing (results feed, RIG-1111-nrtfc_2)',
    ('2026-11-11', 'nrtfc_3'): 'NOT held: let go ~4 s after the fold-in, loose magnet lead (RIG-1111-nrtfc_4 notes)',
    ('2026-11-11', 'nrtfc_5'): 'held through 12 s on four and the LAND (RIG-1111-nrtfc_5)',
    ('2026-11-12', 'fm9_5'): 'held: RING_ATTACHED, n=4, LAND (RIG-1112-fm9_5)',
    ('2026-11-12', 'fm9hc_2'): 'held: the circle on with four, LAND (RIG-1112-fm9hc_2)',
    ('2026-11-12', 'fm9hc_4'): 'held: the circle on with four, LAND (RIG-1112-fm9hc_4)',
    ('2026-11-12', 'fm9hc_5'): 'held: the circle on with four, LAND (RIG-1112-fm9hc_5)',
}

TS = re.compile(r'\[(\d{10}\.\d+)\]')


def log_lines(day, flight):
    with open(os.path.join(RIG, day, flight + '.log'), errors='replace') as f:
        return f.read().splitlines()


def launch_line(day, flight):
    path = os.path.join(RIG, day, 'launches.csv')
    row = None
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            if r['name'] == flight:
                row = r
    return row['launch'] if row else ''


def params(day, flight, lines):
    """The proof config and the plate point the manager flew with."""
    cfg = {}
    for ln in lines:
        if 'proof_rules=comotion:' in ln:
            s = ln.split('proof_rules=comotion:')[1]
            m = re.search(r'contact xy <= ([\d.]+) m, normal ([-+\d.]+)\.\.([-+\d.]+) m, ([\d.]+) m/s for ([\d.]+) s; '
                          r'proven ([\d.]+) s after the lift: drift <= ([\d.]+) m, taut >= free hang - ([\d.]+) m '
                          r'\(fallback ([\d.]+)\)', s)
            k = ('contact_xy', 'normal_min', 'normal_max', 'speed', 'contact_dwell_s', 'proof_dwell_s', 'd_ok',
                 'taut_margin', 'taut_fallback')
            cfg.update({kk: float(v) for kk, v in zip(k, m.groups())})
            m = re.search(r'lost at ([\d.]+) m for ([\d.]+) s', s)
            cfg['lost'], cfg['lost_dwell_s'] = float(m.group(1)), float(m.group(2))
            break
    launch = launch_line(day, flight)
    top = re.sub(r'partner_args:=\S+', '', launch)       # his args are one token in the csv
    for k, v in re.findall(r'\bcomotion_(\w+):=([-\d.]+)', top):
        if k not in cfg:
            cfg[k] = float(v)
    # the rig wiring (graph_attach._contact_manager_params): his lift signal only; the taut fallback is in the log
    cfg['lift_fallback_s'] = 0.0
    if cfg.get('taut_fallback'):
        cfg['taut_fallback'] = cfg['taut_fallback'] - 0.003    # the log prints proof_taut_len + 0.003
    g = dict(re.findall(r'\b(attach_x_offset|attach_y_offset|weld_point_z|tip_skip_repeats|proof_ma_s):=(\S+)', top))
    ox, oy = float(g.get('attach_x_offset', 0.0)), float(g.get('attach_y_offset', 0.0))
    for ln in lines:
        m = re.search(r"reattach_plate (\d+): QUAD4's plate point \(([-+\d.]+), ([-+\d.]+)\)", ln)
        if m:
            ox, oy = float(m.group(2)), float(m.group(3))
            break
    oz = float(g.get('weld_point_z', 'nan'))
    if math.isnan(oz):
        for ln in lines:
            m = re.search(r'at weld_point_z ([-+\d.]+)', ln)
            if m:
                oz = float(m.group(1))
                break
    return cfg, (ox, oy, oz), g.get('tip_skip_repeats', 'false').lower() == 'true', float(g.get('proof_ma_s', FRESH_S))


def quat_rot_mats(q):
    """q: (n, 4) as x, y, z, w -> (n, 3, 3)."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w * w + x * x + y * y + z * z)
    n[n == 0] = 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


class Flight:
    def __init__(self, day, flight, kind):
        self.day, self.flight, self.kind = day, flight, kind
        self.key = f'{day}_{flight}'
        d = np.load(os.path.join(HERE, 'data', self.key + '.npz'))
        with open(os.path.join(HERE, 'data', self.key + '.json')) as f:
            j = json.load(f)
        self.lines = log_lines(day, flight)
        self.cfg, self.off, self.skip_repeats, self.fresh_s = params(day, flight, self.lines)
        self.proof = j['proof']
        self.changes = j['changes']
        tip, ring, q4, ref = d['tip'], d['ring'], d['q4'], d['ref']
        self.ref = ref
        self.q4 = q4
        # the samples the node steps on
        if self.skip_repeats:
            keep = np.ones(len(tip), bool)
            keep[1:] = np.any(tip[1:, 1:] != tip[:-1, 1:], axis=1)
            tip = tip[keep]
        self.t = tip[:, 0]
        self.tip = tip[:, 1:4]
        iring = np.searchsorted(ring[:, 0], self.t, side='right') - 1
        ok = iring >= 0
        iring = np.clip(iring, 0, len(ring) - 1)
        rot = quat_rot_mats(ring[:, 4:8])
        self.rot = rot[iring]
        off = np.array(self.off)
        self.plate = ring[iring, 1:4] + self.rot @ off
        self.fresh = ok & (self.t - ring[iring, 0] <= self.fresh_s + 1e-9)
        ib = np.searchsorted(q4[:, 0], self.t, side='right') - 1
        okb = (ib >= 0)
        ib = np.clip(ib, 0, len(q4) - 1)
        self.body = q4[ib, 1:4]
        self.body_ok = okb & (self.t - q4[ib, 0] <= self.fresh_s + 1e-9)
        self.rel = np.einsum('nji,nj->ni', self.rot, self.tip - self.plate)    # R^T (tip - plate)
        self.xy = np.hypot(self.rel[:, 0], self.rel[:, 1])
        self.normal = self.rel[:, 2]
        # proof-state records (10 Hz) -> magnet ON / OFF times
        self.ptime = np.array([p['t'] for p in self.proof])
        self.pstate = [p['state'] for p in self.proof]
        self.mag = []      # (t, on)
        prev = 'off'
        for p in self.proof:
            s = p['state']
            if (s == 'off') != (prev == 'off'):
                self.mag.append((p['t'] - 0.05, s != 'off'))
            prev = s
        self.lift = [(t, v) for t, topic, v in self.changes if topic == '/join_planner/proof_lift_complete']
        self.phase = [(t, v) for t, topic, v in self.changes if topic == '/join_planner/phase']
        self.log_events = self._log_events()

    def _log_events(self):
        ev = []
        for ln in self.lines:
            m = TS.search(ln)
            if not m:
                continue
            t = float(m.group(1))
            if '[magnet_attachment_manager]' in ln:
                c = re.search(r'CONTACT \(try (\d+)\): xy ([\d.]+) mm, normal ([-+\d.]+) mm, rel_speed=([\d.]+)', ln)
                if c:
                    h = re.search(r'free-hang tether ([\d.]+) m', ln)
                    ev.append({'t': t, 'ev': 'contact', 'try': int(c.group(1)), 'xy_mm': float(c.group(2)),
                               'normal_mm': float(c.group(3)), 'rel_speed': float(c.group(4)),
                               'l_hang': float(h.group(1)) if h else math.nan})
                f = re.search(r'PROOF FAILED \(try (\d+), (\w+)\)', ln)
                if f:
                    ev.append({'t': t, 'ev': 'failed', 'try': int(f.group(1)), 'reason': f.group(2)})
                if 'ATTACH: proven' in ln:
                    ev.append({'t': t, 'ev': 'proven'})
                if 'CONTACT RESET' in ln:
                    ev.append({'t': t, 'ev': 'reset'})
            elif '[online_join_planner]' in ln:
                p = re.search(r'Planner phase: (\w+) -> (\w+)\. Reason: (.*)$', ln)
                if p:
                    ev.append({'t': t, 'ev': 'phase', 'from': p.group(1), 'to': p.group(2), 'reason': p.group(3)})
                lp = re.search(r'LIFT_PROOF: tip measured ([-+\d.]+) m from the contact target', ln)
                if lp:
                    ev.append({'t': t, 'ev': 'lift_line', 'tip_from_target': float(lp.group(1)), 'text': ln.split(
                        '[online_join_planner]: ')[1]})
        return ev

    def value_at(self, series, t, default):
        v = default
        for ts, x in series:
            if ts <= t:
                v = x
            else:
                break
        return v

    def replay(self, normal_max=None, t_from=None, t_to=None, first_contact_only=False):
        """Run ComotionProof over the samples. Returns its events [(t, ev, normal, xy, info)]."""
        kw = {k: v for k, v in self.cfg.items() if k in ComotionConfig.__dataclass_fields__}
        if normal_max is not None:
            kw['normal_max'] = normal_max
        p = ComotionProof(ComotionConfig(**kw))
        out = []
        mag_i, lift_i, ph_i = 0, 0, 0
        lift_v, phase_v = False, ''
        started = t_from is None
        if t_from is not None:
            # start in WATCH at t_from (the previous proof's end or the magnet ON), with the free hang of the
            # last 2 s before it
            p.magnet(True)
            sel = (self.t >= t_from - 2.0) & (self.t < t_from) & (self.normal > p.config.hang_normal) & self.body_ok
            for t, tip, body in zip(self.t[sel], self.tip[sel], self.body[sel]):
                p._hang.append((t, float(np.linalg.norm(tip - body))))
            started = True
            while mag_i < len(self.mag) and self.mag[mag_i][0] <= t_from:
                mag_i += 1
        for i in range(len(self.t)):
            t = self.t[i]
            if t_from is not None and t < t_from:
                continue
            if t_to is not None and t > t_to:
                break
            while mag_i < len(self.mag) and self.mag[mag_i][0] <= t:
                if t_from is None:
                    e = p.magnet(self.mag[mag_i][1])
                    if e == 'reset':
                        out.append((self.mag[mag_i][0], 'reset', math.nan, math.nan, ''))
                elif not self.mag[mag_i][1]:
                    e = p.magnet(False)
                    if e == 'reset':
                        out.append((self.mag[mag_i][0], 'reset', math.nan, math.nan, ''))
                    if p.state == 'off':
                        return out
                mag_i += 1
            while lift_i < len(self.lift) and self.lift[lift_i][0] <= t:
                lift_v = bool(self.lift[lift_i][1])
                lift_i += 1
            while ph_i < len(self.phase) and self.phase[ph_i][0] <= t:
                phase_v = str(self.phase[ph_i][1]).strip().upper()
                ph_i += 1
            if p.state == 'prove' and phase_v and phase_v not in CONTACT_PHASES:
                e = p.abort(f'aborted ({phase_v})', t)
                out.append((t, 'failed', p.normal, p.xy, f'aborted ({phase_v})'))
            body = self.body[i] if self.body_ok[i] else None
            e = p.step(t, self.tip[i], self.plate[i], body=body, fresh=bool(self.fresh[i]), rotation=self.rot[i],
                       lift_done=lift_v)
            if e is None:
                e = p.tick(t)
            if e:
                info = p.reason if e == 'failed' else (f'speed {p.rel_speed:.3f}' if e == 'contact' else '')
                out.append((t, e, p.normal, p.xy, info))
                if first_contact_only and e == 'contact':
                    return out
        return out

    def window(self, t0, t1):
        s = (self.t >= t0) & (self.t <= t1)
        return s


def fmt(x, nd=1, scale=1000.0):
    return '' if x is None or (isinstance(x, float) and not math.isfinite(x)) else f'{scale * x:+.{nd}f}'


def main():
    os.makedirs(os.path.join(HERE, 'windows'), exist_ok=True)
    ev_rows, rep_rows, fl_rows, ver_rows = [], [], [], []
    only = set(sys.argv[1:])
    for day, flight, kind in FLIGHTS:
        if only and f'{day}_{flight}' not in only:
            continue
        F = Flight(day, flight, kind)
        cfg = F.cfg
        logc = [e for e in F.log_events if e['ev'] == 'contact']
        logend = [e for e in F.log_events if e['ev'] in ('failed', 'proven', 'reset')]
        phases = [e for e in F.log_events if e['ev'] == 'phase']
        # 1. the replay at the typed gate reproduces the logged contacts?
        typed = F.replay()
        tc = [e for e in typed if e[1] == 'contact']
        for k in range(max(len(tc), len(logc))):
            a = logc[k] if k < len(logc) else None
            b = tc[k] if k < len(tc) else None
            ver_rows.append({'flight': F.key, 'try': k + 1, 't_log': f"{a['t']:.3f}" if a else '',
                             't_replay': f'{b[0]:.3f}' if b else '',
                             'dt_s': f"{b[0] - a['t']:+.3f}" if a and b else '',
                             'normal_log_mm': a['normal_mm'] if a else '', 'normal_replay_mm': fmt(b[2]) if b else '',
                             'xy_log_mm': a['xy_mm'] if a else '', 'xy_replay_mm': fmt(b[3]) if b else ''})
        # the logged proof stream vs the replayed geometry (normal_m, xy_m at the record times while not off)
        pr = [(p['t'], p.get('normal_m'), p.get('xy_m')) for p in F.proof if p['state'] != 'off'
              and p.get('normal_m') is not None]
        dn = []
        for t, n, x in pr:
            i = np.searchsorted(F.t, t, side='right') - 1
            if i >= 0 and t - F.t[i] < 0.05:
                dn.append((F.normal[i] - n, F.xy[i] - x))
        dn = np.array(dn) if dn else np.zeros((0, 2))
        # 2. descents with the magnet ON and the contact gates over the whole flight (no-contact flights)
        n_desc = sum(1 for e in phases if e['to'] == 'DESCEND_TO_ATTACHMENT')
        near = (F.xy <= cfg['contact_xy']) & F.fresh
        fl = {'flight': F.key, 'kind': kind, 'contacts_logged': len(logc), 'descents_logged': n_desc,
              'gate_typed': f"xy<={1000 * cfg['contact_xy']:.0f} mm, normal {1000 * cfg['normal_min']:+.0f}.."
                            f"{1000 * cfg['normal_max']:+.0f} mm, {cfg['speed']:.3f} m/s, {cfg['contact_dwell_s']:.2f} s",
              'plate_point': '({:+.4f}, {:+.4f}, {:+.4f})'.format(*F.off), 'tip_skip_repeats': F.skip_repeats,
              'tip_rate_hz': f'{(len(F.t) - 1) / (F.t[-1] - F.t[0]):.0f}',
              'geom_check_normal_mm': f'{1000 * np.median(np.abs(dn[:, 0])):.2f}' if len(dn) else '',
              'geom_check_xy_mm': f'{1000 * np.median(np.abs(dn[:, 1])):.2f}' if len(dn) else '',
              'min_normal_inside_xy_mm': fmt(float(F.normal[near].min())) if near.any() else ''}
        if not logc:
            # a narrower gate cannot fire where the wider did not (the dwell window is the same or shorter); the
            # replay at 0.04 checks that and shows what the wider gate would have done on the Oct / 4 Nov gates
            for g in GATES:
                r = [e for e in F.replay(normal_max=g) if e[1] == 'contact']
                fl[f'contacts_at_{g:.2f}'] = len(r)
                fl[f'first_at_{g:.2f}'] = (f'{r[0][0] - F.t[0]:.1f} s into the bag, normal {fmt(r[0][2])} mm, '
                                           f'xy {fmt(r[0][3])} mm') if r else ''
        fl_rows.append(fl)
        # 3. per contact event
        for k, c in enumerate(logc):
            tk = c['t']
            # the replay's matching contact sample
            b = min(tc, key=lambda e: abs(e[0] - tk)) if tc else None
            t_rep = b[0] if b and abs(b[0] - tk) < 0.1 else tk
            i_c = int(np.searchsorted(F.t, t_rep, side='right') - 1)
            # when this try's WATCH began: the previous proof's end, or the magnet ON before the contact
            prev_end = [e['t'] for e in logend if e['t'] < tk]
            mag_on = [t for t, on_ in F.mag if on_ and t < tk]
            t_start = max(prev_end[-1] if prev_end else -1e18, mag_on[-1] if mag_on else -1e18)
            end = [e for e in logend if e['t'] > tk]
            outcome = (f"{end[0]['ev']}" + (f" ({[x for x in F.log_events if x['ev'] == 'failed' and x['t'] == end[0]['t']][0]['reason']})"
                                            if end and end[0]['ev'] == 'failed' else '')
                       + f" +{end[0]['t'] - tk:.2f} s") if end else 'none logged'
            dw = F.window(t_rep - cfg['contact_dwell_s'] - 1e-6, t_rep)
            tail = F.window(t_rep - 0.1, t_rep)
            post = F.window(t_rep + 1e-6, t_rep + POST_S) & F.fresh
            post_pr = [p.get('normal_m') for p in F.proof if tk < p['t'] <= tk + POST_S and p.get('normal_m') is not None]
            # the last contiguous run at or below 1 cm before the contact
            j = i_c
            while j > 0 and F.normal[j] <= 0.01 and F.fresh[j]:
                j -= 1
            below_1cm = F.t[i_c] - F.t[j + 1] if F.normal[i_c] <= 0.01 else 0.0
            # his phases around the contact
            ph_before = [e for e in phases if tk - 30.0 <= e['t'] <= tk]
            ph_after = [e for e in phases if tk < e['t'] <= tk + POST_S + 3.0]
            ready = [e for e in ph_before if e['to'] == 'ATTACH_READY']
            desc = [e for e in ph_before if e['to'] == 'DESCEND_TO_ATTACHMENT']
            lift = [e for e in ph_after if e['to'] == 'LIFT_PROOF']
            lift_line = [e for e in F.log_events if e['ev'] == 'lift_line' and tk <= e['t'] <= tk + 1.0]
            # the tip and the body against his reference
            ir = int(np.searchsorted(F.ref[:, 0], tk, side='right') - 1)
            iq = int(np.searchsorted(F.q4[:, 0], tk, side='right') - 1)
            ref_z, body_z = F.ref[ir, 3], F.q4[iq, 3]
            plate_z = F.plate[i_c, 2]
            lh = c['l_hang']
            # the reference's bottom in this descent: its lowest z from the descent start to the contact
            t_d0 = desc[-1]['t'] if desc else tk - 10.0
            rs = (F.ref[:, 0] >= t_d0) & (F.ref[:, 0] <= tk)
            rz_min = F.ref[rs, 3].min() if rs.any() else math.nan
            t_lp = lift[0]['t'] if lift else tk
            ir2 = int(np.searchsorted(F.ref[:, 0], t_lp, side='right') - 1)
            t_rz_min = F.ref[rs, 0][np.argmin(F.ref[rs, 3])] if rs.any() else math.nan
            qs = (F.q4[:, 0] >= tk - 0.5) & (F.q4[:, 0] <= tk)
            vz = np.polyfit(F.q4[qs, 0] - tk, F.q4[qs, 3], 1)[0] if qs.sum() > 3 else math.nan
            rs5 = (F.ref[:, 0] >= tk - 0.5) & (F.ref[:, 0] <= tk)
            ref_minus_body = []
            for tt, rz in zip(F.ref[rs5, 0], F.ref[rs5, 3]):
                ii = int(np.searchsorted(F.q4[:, 0], tt, side='right') - 1)
                ref_minus_body.append(F.q4[ii, 3] - rz)
            # a proven catch: when (if ever) the tip left its plate afterwards (|rel| > 30 mm held 0.3 s), and the
            # plate's height then (a ring on the floor is the landing)
            left = ''
            if end and end[0]['ev'] == 'proven':
                tp = end[0]['t']
                sel = np.nonzero((F.t > tp) & F.fresh)[0]
                far = np.linalg.norm(F.rel[sel], axis=1) > 0.03
                t0 = None
                for ii, fa in zip(sel, far):
                    if fa:
                        t0 = F.t[ii] if t0 is None else t0
                        if F.t[ii] - t0 >= 0.3:
                            jj = int(np.searchsorted(F.t, t0))
                            left = f'{t0 - tp:.1f} s after the proof (plate z {F.plate[jj, 2]:.2f} m)'
                            break
                    else:
                        t0 = None
                if not left:
                    left = f'never, to the bag end {F.t[-1] - tp:.0f} s later (plate z then {F.plate[-1, 2]:.2f} m)'
            # the body's time lag behind his reference: how long before the contact the reference was at the height
            # the body had at the contact (descending reference; '' when the body is below the reference)
            rr = (F.ref[:, 0] >= tk - 5.0) & (F.ref[:, 0] <= tk)
            above = np.nonzero(F.ref[rr, 3] >= body_z)[0]
            time_lag = (f'{tk - F.ref[rr, 0][above[-1]]:.2f}' if len(above) and body_z > ref_z else '')
            ref_rate = (np.polyfit(F.ref[rs5, 0] - tk, F.ref[rs5, 3], 1)[0] if rs5.sum() > 3 else math.nan)
            # when his lift reached QUAD4: the body's vertical speed (0.2 s fit) first above 0 after the contact
            turn = ''
            for tt in np.arange(tk, tk + 3.0, 0.02):
                qq = (F.q4[:, 0] >= tt - 0.1) & (F.q4[:, 0] <= tt + 0.1)
                if qq.sum() > 5 and np.polyfit(F.q4[qq, 0] - tt, F.q4[qq, 3], 1)[0] > 0.0:
                    turn = f'{tt - tk:.2f}'
                    break
            row = {'flight': F.key, 'kind': kind, 'try': c['try'], 't_contact': f'{tk:.3f}',
                   't_in_bag_s': f'{tk - F.t[0]:.1f}',
                   'gate_typed_mm': f"{1000 * cfg['normal_min']:+.0f}..{1000 * cfg['normal_max']:+.0f}",
                   'xy_typed_mm': f"{1000 * cfg['contact_xy']:.0f}",
                   'normal_log_mm': c['normal_mm'], 'xy_log_mm': c['xy_mm'], 'rel_speed_log': c['rel_speed'],
                   'normal_replay_mm': fmt(F.normal[i_c]), 'c0_normal_mm': fmt(float(F.normal[tail].mean())),
                   'dwell_normal_max_mm': fmt(float(F.normal[dw].max())),
                   'tip_normal_rate_mps': f'{np.polyfit(F.t[dw] - t_rep, F.normal[dw], 1)[0]:+.3f}' if dw.sum() > 3 else '',
                   'dwell_normal_min_mm': fmt(float(F.normal[dw].min())),
                   'run_at_or_below_10mm_s': f'{below_1cm:.2f}',
                   'post2s_normal_min_mm': fmt(float(F.normal[post].min())) if post.any() else '',
                   'post2s_proofstream_normal_min_mm': fmt(min(post_pr)) if post_pr else '',
                   'outcome': outcome,
                   'held': HELD_NOTE.get((day, flight), '') if outcome.startswith('proven') else '',
                   'tip_left_plate': left,
                   'descent_start_to_contact_s': f"{tk - desc[-1]['t']:.2f}" if desc else '',
                   'attach_ready_to_contact_s': (f"{tk - ready[-1]['t']:.2f}" if ready and (not desc or ready[-1]['t'] > desc[-1]['t'])
                                                 else 'contact before ATTACH_READY'),
                   'phase_at_contact': (ph_before[-1]['to'] if ph_before else ''),
                   'his_phases_before': '; '.join(f"{e['to']} {e['t'] - tk:+.2f} s" for e in ph_before[-4:]),
                   'his_phases_after': '; '.join(f"{e['to']} {e['t'] - tk:+.2f} s ({e['reason'][:60]})"
                                                for e in ph_after[:3]),
                   'lift_proof_after_contact_s': f"{lift[0]['t'] - tk:.3f}" if lift else 'none',
                   'his_tip_from_target_m': f"{lift_line[0]['tip_from_target']:+.3f}" if lift_line else '',
                   'ref_z': f'{ref_z:.3f}', 'body_z': f'{body_z:.3f}',
                   'body_minus_ref_mm': fmt(body_z - ref_z),
                   'body_minus_ref_mean_0p5s_mm': fmt(float(np.mean(ref_minus_body))) if ref_minus_body else '',
                   'body_vz_mps': f'{vz:+.3f}',
                   'ref_vz_mps': f'{ref_rate:+.3f}',
                   'body_time_lag_s': time_lag,
                   'body_moving_up_after_contact_s': turn,
                   'ref_tip_normal_mm': fmt(ref_z - lh - plate_z) if math.isfinite(lh) else '',
                   'tip_normal_minus_ref_tip_normal_mm': fmt(F.normal[i_c] - (ref_z - lh - plate_z)) if math.isfinite(lh) else '',
                   'ref_bottom_z': f'{rz_min:.3f}', 'ref_bottom_reached_before_contact_s': f'{tk - t_rz_min:.2f}',
                   'ref_bottom_tip_normal_mm': fmt(rz_min - lh - plate_z) if math.isfinite(lh) else '',
                   'ref_final_z': f'{F.ref[ir2, 3]:.3f}',
                   'ref_final_tip_normal_mm': fmt(F.ref[ir2, 3] - lh - plate_z) if math.isfinite(lh) else '',
                   'l_hang': lh, 'watch_from_s_before': f'{tk - t_start:.1f}' if t_start > -1e17 else ''}
            # 4. the counterfactual gates from this try's WATCH start
            for g in GATES:
                r = F.replay(normal_max=g, t_from=t_start if t_start > -1e17 else None, t_to=tk + POST_S,
                             first_contact_only=True)
                rc = [e for e in r if e[1] == 'contact']
                key = f'{g:.2f}'
                if not rc:
                    row[f'fires_{key}'] = 'no (not by +2 s)'
                    row[f'delay_{key}_s'] = ''
                    row[f'normal_{key}_mm'] = ''
                else:
                    dt = rc[0][0] - t_rep
                    if abs(dt) < 1e-6:
                        row[f'fires_{key}'] = 'same sample'
                    elif dt < 0:
                        row[f'fires_{key}'] = 'earlier'
                    elif dt <= VALID_AFTER_S:
                        row[f'fires_{key}'] = f'later by {1000 * dt:.0f} ms (before his lift can move the tip)'
                    else:
                        row[f'fires_{key}'] = 'later, after the real contact (his lift had begun: not replayable)'
                    row[f'delay_{key}_s'] = f'{dt:+.3f}'
                    row[f'normal_{key}_mm'] = fmt(rc[0][2])
                rep_rows.append({'flight': F.key, 'try': c['try'], 'gate_mm': f'{1000 * g:+.0f}',
                                 'fires': row[f'fires_{key}'], 'delay_s': row[f'delay_{key}_s'],
                                 'normal_mm': row[f'normal_{key}_mm'], 'watch_start': f'{t_start:.3f}'})
            ev_rows.append(row)
            # the window for plots: -6 s .. +3 s
            s = F.window(tk - 6.0, tk + 3.0)
            with open(os.path.join(HERE, 'windows', f'{F.key}_try{c["try"]}.csv'), 'w', newline='') as f:
                w = csv.writer(f, lineterminator='\n')
                w.writerow(('t_rel', 'normal_mm', 'xy_mm', 'fresh', 'tip_z', 'plate_z', 'body_z'))
                for ii in np.nonzero(s)[0]:
                    w.writerow((f'{F.t[ii] - tk:.4f}', f'{1000 * F.normal[ii]:.2f}', f'{1000 * F.xy[ii]:.2f}',
                                int(F.fresh[ii]), f'{F.tip[ii, 2]:.4f}', f'{F.plate[ii, 2]:.4f}',
                                f'{F.body[ii, 2]:.4f}'))
            with open(os.path.join(HERE, 'windows', f'{F.key}_try{c["try"]}_ref.csv'), 'w', newline='') as f:
                w = csv.writer(f, lineterminator='\n')
                w.writerow(('t_rel', 'ref_z', 'body_z'))
                rs = (F.ref[:, 0] >= tk - 6.0) & (F.ref[:, 0] <= tk + 3.0)
                for tt, rz in zip(F.ref[rs, 0], F.ref[rs, 3]):
                    ii = int(np.searchsorted(F.q4[:, 0], tt, side='right') - 1)
                    w.writerow((f'{tt - tk:.4f}', f'{rz:.4f}', f'{F.q4[ii, 3]:.4f}'))
        print(F.key, 'contacts', len(logc), 'replayed', len(tc), flush=True)

    def write(name, rows):
        keys = []
        for r in rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
        with open(os.path.join(HERE, name), 'w', newline='') as f:
            w = csv.DictWriter(f, keys, lineterminator='\n')
            w.writeheader()
            w.writerows(rows)
    write('events.csv', ev_rows)
    write('replay.csv', rep_rows)
    write('flights.csv', fl_rows)
    write('verify.csv', ver_rows)


if __name__ == '__main__':
    main()
