"""tools/experiment/relay.py -- the endurance relay's rota (card docs/experiments/2026-10-30_x4_endurance_relay.md).

Twin only, off unless a config has a `relay:` section. The section is expanded, when the config loads, into the events
that fly the relay on the planner's relay_task (dissipative_node: a carrier's TASK waits for its leave, a leave for a
full ring, the path held while the leaver is inside the lap's keep-out, parked at its last waypoint until an ATTACH
flies it back). At lap end k (`lap k done`, the lap-end hold) the runner sends, 0.3-1.0 s into the hold:

    TASK <leaver> <its route>      queued: flown when it leaves
    ATTACH <last leaver>           parked at its pad: back to its staging point with the path held, then its rejoin
    DETACH <leaver>                the announced soft detach (detach_unload_s), once the ring is full again

so the ring carries N-1 through each lap and N through the swap. The leaver is the carrier whose pack is oldest (the
pack clock runs from TAKEOFF, is reset at the end of a pad dwell, and is not reset by an errand); an errand leave
(`errand_at` / `errand_every`) sends the freshest carrier on pickup -> delivery instead. Pads: per carrier, the
nearest point clear of the lap's keep-out (the planner's: its staging radius about every point of the lap) by
`pad_margin_m`, inside the staging bound, reached on a straight route the planner accepts.

The projection (`RelayPlan.table`, printed by run_experiment.py --check) flies the rota on a timing model from the
8-quad runs (R2776-R2779): rejoin ATTACH -> weld ~21 s, the 10 s fold-in blend, the unload, the 0.2 m/s task legs. It
is a plan, not a result: the scorer (tools/x4_metrics.py) reads the run's own times. B is a budget, not a battery
model.
"""
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# the planner's values (dissipative_node / approach); tools/test/test_relay_rota.py pins them
STAGE_OUT_M = 0.3          # REJOIN_STAGE_OUT_M
REACHED_M = 0.02           # TAKE_BACK_REACHED_M
POSITION_LIMIT_M = 1.75    # the staging bound's limit while pos_limit_m is off
STAGE_INSET_M = 0.18       # STAGE_LIMIT_MARGIN_M + STAGE_TRACK_M
TASK_VEL = 0.2             # APPROACH_VEL: every task leg
TASK_ACCEL = 0.25          # TAKE_BACK_ACCEL
SAMPLE_M = 0.02            # RELAY_SAMPLE_M
PAD_INSET_M = 0.02         # a pad stays this far inside the staging bound (off its clamp)

# the timing model (s): measured on the 8-quad tree (registry R2776-R2779) unless said
MODEL = {
    'path_start_s': 25.0,     # TAKEOFF -> the path's first lap (lift, settle, traj_hold_s 5; R2779 24.9)
    'rejoin_s': 21.0,         # ATTACH at its staging point -> REJOIN WELD (R2779 19.6-24.1, R2776 17.3-19.1)
    'blend_s': 10.0,          # the fold-in blend (rejoin_blend_s), the path held
    'release_s': 1.0,         # the unload's gate wait and the resize after detach_unload_s
    'runner_s': 0.5,          # the runner's ATTACH after 'lap k done'
    'tail_s': 1.0,            # the relay hold's last second once the leaver is clear
    'land_s': 20.0,           # LAND -> every drone down
}
KEYS = {'leaves', 'first_lap', 'final_laps', 'order', 'dwell_s', 'errand_at', 'errand_every', 'errand', 'pad_margin_m',
        'pads', 'budget_s', 'budget_check', 'ring_xy', 'model', 'wait_timeout_s', 'land_at_budget'}


def _traj_cls():
    try:
        from mpc_planner.load_trajectory import LoadTrajectory
    except ImportError:
        sys.path.insert(0, os.path.join(REPO, 'src', 'mpc_planner'))
        from mpc_planner.load_trajectory import LoadTrajectory
    return LoadTrajectory


def leg_s(dist):
    """Seconds for a task leg of `dist` m from rest to rest at TASK_VEL, TASK_ACCEL."""
    d = max(float(dist), 0.0)
    if d >= TASK_VEL ** 2 / TASK_ACCEL:
        return d / TASK_VEL + TASK_VEL / TASK_ACCEL
    return 2.0 * math.sqrt(d / TASK_ACCEL)


def route_s(points):
    return sum(leg_s(np.linalg.norm(np.asarray(b[:2], float) - np.asarray(a[:2], float)))
               for a, b in zip(points[:-1], points[1:]))


def sample_route(path, step=SAMPLE_M):
    out = [np.asarray(path[0], float)[:2]]
    for a, b in zip(path[:-1], path[1:]):
        a, b = np.asarray(a, float)[:2], np.asarray(b, float)[:2]
        n = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / step)))
        out += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    return np.array(out)


def min_dist(points, pts):
    """Per point (xy): the distance to the nearest of pts (xy)."""
    s, p = np.asarray(points, float)[:, None, :2], np.asarray(pts, float)[None, :, :2]
    return np.min(np.linalg.norm(s - p, axis=2), axis=1)


_PADS = {}                 # Geometry.pad's results by geometry, drone, margin and grid


class Geometry:
    """The ring, its plates and the lap, as the planner sees them in the lap-end hold (the ring at its start point)."""

    def __init__(self, knobs, ring_xy=(0.0, 0.0)):
        def num(k, default):
            v = knobs.get(k)
            return default if v in (None, '') else float(v)
        self.n = int(knobs['num_drones'])
        az = str(knobs.get('attach_azimuths_deg') or '').strip()
        if az and az.lower() not in ('even', 'none', 'auto'):
            typed = [float(v) for v in az.replace(' ', ',').split(',') if v]
            self.az = [(-c) % 360.0 for c in typed]                 # typed clockwise -> the planner's ccw
        else:
            self.az = [360.0 * k / self.n for k in range(self.n)]
        if len(self.az) != self.n:
            raise ValueError(f'relay: {len(self.az)} azimuths for {self.n} drones')
        self.R = num('attach_radius', 0.225)
        self.L = num('cable_len', 1.0)
        self.elev = num('cable_elev_deg', 60.0)
        self.z = (num('target_z', 1.0) + num('attach_z', 0.02) + self.L * math.sin(math.radians(self.elev))
                  - num('pivot_offset_z', -0.04))
        self.r_stage = self.R + self.L * math.cos(math.radians(self.elev)) + STAGE_OUT_M
        self.keep = self.r_stage - REACHED_M
        limit = num('pos_limit_m', 0.0)
        self.lim = (limit if limit > 0.0 else POSITION_LIMIT_M) - STAGE_INSET_M
        kind = str(knobs.get('load_traj', 'hover')).strip()
        LT = _traj_cls()
        laps = int(num('traj_laps', 1))
        self.traj = LT(kind, num('traj_speed', 0.1), num('traj_distance', 1.0),
                       num('fig8_size', 0.0) or num('traj_radius', 0.5), laps=laps)
        self.ring = np.asarray(ring_xy, float)
        T = self.traj.lap_s()
        if T is None:
            raise ValueError(f'relay: load_traj {kind} has no lap (the relay flies a lapped path)')
        self.lap_s = T
        ts = np.arange(0.0, T + 0.1, 0.1)
        self.lap = np.array([self.traj.offset_at(t)[:2] for t in ts]) + self.ring

    def stage(self, d):
        a = math.radians(self.az[d])
        return np.array([self.ring[0] + self.r_stage * math.cos(a), self.ring[1] + self.r_stage * math.sin(a), self.z])

    def plate_body(self, d):
        a = math.radians(self.az[d])
        r = self.R + self.L * math.cos(math.radians(self.elev))
        return np.array([self.ring[0] + r * math.cos(a), self.ring[1] + r * math.sin(a), self.z])

    def clear(self, p, margin=0.0):
        p = np.asarray(p, float)
        return (max(abs(p[0]), abs(p[1])) <= self.lim + 1e-9
                and float(min_dist([p], self.lap)[0]) >= self.keep + margin)

    def route_refused(self, d, wps):
        """The planner's _relay_route_refused for carrier d's waypoints [(x, y, z, dwell)], or None."""
        stage = self.stage(d)
        pts = [np.asarray(w[:3], float) for w in wps]
        for k, (p, w) in enumerate(zip(pts, wps), 1):
            if max(abs(p[0]), abs(p[1])) > self.lim + 1e-9:
                return f'waypoint {k} outside the staging bound +-{self.lim:.2f} m'
            if float(min_dist([p], self.lap)[0]) < self.keep:
                return f'waypoint {k} ({p[0]:.2f}, {p[1]:.2f}) inside the lap keep-out'
        via = [p for p, w in zip(pts[:-1], wps[:-1]) if float(w[3]) <= 0.0]
        out, back = [stage] + pts, [pts[-1]] + via[::-1] + [stage]
        for path in (out, back):
            if float(np.min(min_dist(sample_route(path), [self.ring]))) < self.keep:
                return 'the route comes inside the keep-out of the ring where it is held'
        inside = min_dist(sample_route(out), self.lap) < self.keep
        first = int(np.argmin(inside)) if not inside.all() else len(inside)
        if inside[first:].any():
            return 'the route out comes back inside the lap keep-out'
        return None

    def exit_s(self, d, wps):
        """Seconds from the release on its plate until the leaver is clear of the lap keep-out (the path held)."""
        stage = self.stage(d)
        out = [stage] + [np.asarray(w[:3], float) for w in wps]
        s = sample_route(out)
        inside = min_dist(s, self.lap) < self.keep
        first = int(np.argmin(inside)) if not inside.all() else len(inside) - 1
        run = float(np.sum(np.linalg.norm(np.diff(s[:first + 1], axis=0), axis=1))) if first > 0 else 0.0
        return leg_s(np.linalg.norm(stage[:2] - self.plate_body(d)[:2])) + (leg_s(run) if run > 0 else 0.0)

    def signature(self):
        return (tuple(self.az), self.R, self.L, self.elev, round(self.z, 9), self.lim, tuple(self.ring),
                self.traj.kind, self.traj.speed, self.traj.radius)

    def pad(self, d, margin, step=0.01):
        """The nearest point to d's staging point clear of the lap keep-out by margin, on a route the planner takes."""
        key = (self.signature(), d, margin, step)
        if key not in _PADS:
            _PADS[key] = self._pad(d, margin, step)
        return _PADS[key].copy()

    def _pad(self, d, margin, step):
        s = self.stage(d)
        g = np.arange(-self.lim + PAD_INSET_M, self.lim - PAD_INSET_M + 1e-9, step)
        X, Y = np.meshgrid(g, g)
        cand = np.c_[X.ravel(), Y.ravel()]
        ok = min_dist(cand, self.lap) >= self.keep + margin
        cand = cand[ok]
        cand = cand[np.argsort(np.linalg.norm(cand - s[:2], axis=1))]
        for c in cand[:4000]:
            p = (float(c[0]), float(c[1]), self.z, 1.0)
            if self.route_refused(d, [p]) is None:
                return np.array([round(p[0], 3), round(p[1], 3), round(self.z, 3)])
        raise ValueError(f'relay: no pad for drone {d} clear of the lap by {margin} m inside +-{self.lim:.2f} m')


class Leave:
    def __init__(self, k, d, kind, wps, returner):
        self.lap, self.d, self.kind, self.wps, self.returner = k, d, kind, wps, returner
        self.t = {}                           # the model's times (s after TAKEOFF)

    def task_text(self):
        return f'{self.d} ' + '; '.join(f'{w[0]:g},{w[1]:g},{w[2]:g},{w[3]:g}' for w in self.wps)


class RelayPlan:
    """The expanded rota: `leaves`, `events` (dicts t/do/arg), `laps` (traj_laps it needs), the model's `packs`
    (drone -> [pack intervals s]) and `mission_s` (TAKEOFF -> every drone down)."""

    def __init__(self, spec, knobs, t0):
        bad = set(spec) - KEYS
        if bad:
            raise ValueError(f'relay: unknown keys {sorted(bad)} (known: {sorted(KEYS)})')
        self.spec = spec
        self.n_leaves = int(spec.get('leaves', 0))
        self.first = int(spec.get('first_lap', 1))
        self.final = int(spec.get('final_laps', 1))
        self.dwell = float(spec.get('dwell_s', 8.0))
        self.margin = float(spec.get('pad_margin_m', 0.05))
        self.budget = float(spec.get('budget_s', 300.0))
        self.check = str(spec.get('budget_check', 'report'))
        self.timeout = float(spec.get('wait_timeout_s', 240.0))
        if self.check not in ('report', 'refuse'):
            raise ValueError("relay: budget_check is 'report' or 'refuse'")
        if self.n_leaves < 0 or self.first < 1 or self.final < 1 or self.dwell < 0.0:
            raise ValueError('relay: leaves >= 0, first_lap >= 1, final_laps >= 1, dwell_s >= 0')
        self.model = dict(MODEL, **(spec.get('model') or {}))
        self.lap_hold = float(knobs.get('traj_lap_hold_s') or 0.0)
        self.unload = float(knobs.get('detach_unload_s') or 0.0)
        self.geo = Geometry(dict(knobs, traj_laps=2), spec.get('ring_xy', (0.0, 0.0)))
        self.land_at_budget = bool(spec.get('land_at_budget', False))
        if self.land_at_budget:
            # the fixed team: the whole laps that bring every drone down by B, then LAND in the last lap's end
            if self.n_leaves:
                raise ValueError('relay: land_at_budget is the fixed team (leaves: 0)')
            T, m = self.geo.lap_s, self.model
            self.laps = int((self.budget - m['path_start_s'] - 2.0 - m['land_s'] + self.lap_hold) // (T + self.lap_hold))
            if self.laps < 1:
                raise ValueError(f'relay: no whole lap fits in B {self.budget:g} s')
        else:
            self.laps = self.first + self.n_leaves + self.final if self.n_leaves else self.first + self.final
        n = self.geo.n
        self.order = [int(d) for d in spec['order']] if spec.get('order') else None
        if self.order and sorted(set(self.order)) != sorted(self.order):
            raise ValueError('relay: order repeats a drone')
        given = spec.get('pads') or {}
        self.pads = {d: (np.asarray(given[d], float) if d in given else self.geo.pad(d, self.margin))
                     for d in range(n)}
        for d, p in self.pads.items():
            why = self.geo.route_refused(d, [(*p[:3], self.dwell)])
            if why:
                raise ValueError(f'relay: drone {d} pad {list(p)}: {why}')
        self.errand_at = set(int(i) for i in (spec.get('errand_at') or []))
        every = int(spec.get('errand_every', 0) or 0)
        if every:
            self.errand_at |= {i for i in range(1, self.n_leaves + 1) if i % every == 0}
        self.errand = spec.get('errand') or {}
        if self.errand_at and not {'pickup', 'delivery'} <= set(self.errand):
            raise ValueError('relay: an errand needs errand: {pickup: [x, y, z, dwell], delivery: [x, y, z, dwell]}')
        self._plan(t0)

    # ── the rota and its projection ──────────────────────────────────────────────────────────────────────────────
    def _errand_wps(self, d):
        p = self.pads[d]
        return [(*p[:3], 0.0), tuple(float(v) for v in self.errand['pickup']),
                tuple(float(v) for v in self.errand['delivery'])]

    def _choose(self, i, on, last, reset):
        """Leave i's (drone, kind, waypoints): the oldest pack on the ring, the freshest for an errand (never the one
        that has just come back); an errand only for a carrier whose route the planner takes."""
        cands = [d for d in on if d != last] or list(on)
        if i in self.errand_at:
            ok = [d for d in cands if self.geo.route_refused(d, self._errand_wps(d)) is None]
            if not ok:
                raise ValueError(f'relay: errand {i}: no carrier on the ring has a route the planner takes '
                                 f'(pickup {self.errand["pickup"]}, delivery {self.errand["delivery"]})')
            d = max(ok, key=lambda x: (reset[x], -x))
            return d, 'errand', self._errand_wps(d)
        if self.order:
            d = self.order[(i - 1) % len(self.order)]
            if d not in cands:
                raise ValueError(f'relay: leave {i}: drone {d} of `order` is not on the ring')
        else:
            d = min(cands, key=lambda x: (reset[x], x))
        return d, 'pad', [(*self.pads[d][:3], self.dwell)]

    def _plan(self, t0):
        m, geo, T = self.model, self.geo, self.geo.lap_s
        hold = self.lap_hold
        reset = {d: 0.0 for d in range(geo.n)}              # pack clock zero (s after TAKEOFF)
        self.packs = {d: [] for d in range(geo.n)}
        on, away = set(range(geo.n)), None                   # away: (leave, its park time)
        self.leaves, t, last = [], m['path_start_s'], None
        for k in range(1, self.laps + 1):
            t += T                                           # lap k's end
            if k == self.laps:
                break
            i = k - self.first + 1                           # the leave at this lap end (1-based)
            th = t + m['runner_s']
            t_free, back = th, None
            if away is not None:
                lv, t_park = away
                t_att = max(th, t_park)
                back = t_att + route_s([lv.wps[-1][:3]] + [w[:3] for w in lv.wps[:-1] if w[3] <= 0.0][::-1]
                                       + [geo.stage(lv.d)])
                t_weld = back + m['rejoin_s']
                t_free = t_weld + m['blend_s']
                lv.t.update(attach=t_att, back=back, weld=t_weld, blend_over=t_free)
                on.add(lv.d)
                last, away = lv.d, None
            end = t + hold
            if 1 <= i <= self.n_leaves:
                d, kind, wps = self._choose(i, on, last, reset)
                lv = Leave(k, d, kind, wps, last if back is not None else None)
                t_rel = t_free + self.unload + m['release_s']
                t_out = t_rel + geo.exit_s(d, wps)
                path = [geo.stage(d)] + [w[:3] for w in wps]
                t_wp, legs = t_rel + leg_s(np.linalg.norm(geo.stage(d)[:2] - geo.plate_body(d)[:2])), []
                for a, b, w in zip(path[:-1], path[1:], wps):
                    t_wp += route_s([a, b])
                    legs.append(t_wp)
                    t_wp += float(w[3])
                lv.t.update(release=t_rel, clear=t_out, arrive=legs, park=t_wp)
                if kind == 'pad':
                    self.packs[d].append(legs[-1] - reset[d])
                    reset[d] = legs[-1] + float(wps[-1][3])
                on.discard(d)
                away = (lv, t_wp)
                self.leaves.append(lv)
                end = max(end, t_out + m['tail_s'])
            elif back is not None:
                end = max(end, t_free)
            t = end
        self.t_land = t + 2.0
        self.mission_s = self.t_land + m['land_s']
        for d in range(geo.n):
            self.packs[d].append(self.mission_s - reset[d])
        self.max_pack = max(max(v) for v in self.packs.values())
        self.events = self._events(t0)

    def _events(self, t0):
        ev, t = [], float(t0)
        by_lap = {lv.lap: lv for lv in self.leaves}
        back_at = {lv.lap + 1: lv.d for lv in self.leaves}
        for k in range(1, self.laps):
            if k not in by_lap and k not in back_at:
                continue
            ev.append({'t': round(t, 3), 'do': 'WAIT_LOG', 'arg': f'{self.timeout:g} \\] lap {k} done at t='})
            lv = by_lap.get(k)
            if lv is not None:
                ev.append({'t': round(t + 0.3, 3), 'do': 'TASK', 'arg': lv.task_text()})
            if k in back_at:
                ev.append({'t': round(t + 0.5, 3), 'do': 'ATTACH', 'arg': back_at[k]})
            if lv is not None:
                ev.append({'t': round(t + 1.0, 3), 'do': 'DETACH', 'arg': lv.d})
            t += 1.1
        ev.append({'t': round(t, 3), 'do': 'WAIT_TRAJ', 'arg': f'lap {self.laps} {self.timeout + 60.0:g}'})
        ev.append({'t': round(t + 2.0, 3), 'do': 'LAND'})
        return ev

    # ── what --check prints ─────────────────────────────────────────────────────────────────────────────────────
    def over_budget(self):
        return {d: max(v) for d, v in self.packs.items() if max(v) > self.budget + 1e-9}

    def table(self):
        g = self.geo
        rows = [f'relay: {self.n_leaves} leaves at lap ends {self.first}-{self.first + self.n_leaves - 1}, '
                f'traj_laps {self.laps}, lap {g.lap_s:.1f} s; staging radius {g.r_stage:.2f} m, keep-out '
                f'{g.keep:.2f} m, bound +-{g.lim:.2f} m',
                '  pads: ' + ', '.join(f'{d}:({p[0]:.2f},{p[1]:.2f})' for d, p in self.pads.items()),
                '  leave  lap  drone  kind    back  release  clear   park   (model, s after TAKEOFF)']
        for i, lv in enumerate(self.leaves, 1):
            rows.append(f'  {i:5d} {lv.lap:4d} {lv.d:6d}  {lv.kind:6s} {str(lv.returner):>5s} '
                        f'{lv.t["release"]:8.1f} {lv.t["clear"]:6.1f} {lv.t["park"]:6.1f}')
        over = self.over_budget()
        rows.append(f'  mission {self.mission_s:.0f} s ({self.mission_s / 60.0:.1f} min) = '
                    f'{self.mission_s / self.budget:.2f} B; longest pack {self.max_pack:.0f} s '
                    f'({self.max_pack / 60.0:.1f} min) against B {self.budget:.0f} s'
                    + (f'; OVER B: ' + ', '.join(f'drone {d} {v:.0f} s' for d, v in sorted(over.items()))
                       if over else '; every pack inside B'))
        return '\n'.join(rows)


def expand(spec, knobs, t0):
    """A config's relay: section -> RelayPlan (its events start at t0, after the config's own)."""
    plan = RelayPlan(dict(spec or {}), knobs, t0)
    if plan.check == 'refuse' and plan.over_budget():
        raise ValueError('relay: the rota keeps no pack inside B (budget_check refuse):\n' + plan.table())
    return plan
