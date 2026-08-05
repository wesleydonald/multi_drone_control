"""
creep_controller.py
-------------------
Phase-1 takeoff, factored out of the planner node. Before the OCP takes over the
drones must bring the cables from slack/near-horizontal up to a liftable taut
configuration WITHOUT snapping them taut or dragging the grounded payload. This
controller owns that whole sub-behavior: the two creep reference generators (a
straight-up "soft" creep for slack cables, an arc sweep for rigid rods), the creep
state, and the handover decision that tells the node when the cables are ready.

The node calls step() each tick while in the creep phase; it publishes this tick's
per-drone references (via the injected publish_ref callback) and returns
(handover, reason) -- handover=True meaning the OCP should take over now.
"""
import numpy as np

from .geometry import quat_to_rot_np

CREEP_VEL      = 0.10          # m/s rise
CREEP_LEAD     = 0.10          # m z lead held while grounded, to initiate climb
LIFTOFF_MARGIN = 0.05          # m above spawn before the ramp starts
LIFTOFF_TIMEOUT_S  = 4.0       # s after TAKEOFF to start the sweep regardless (see
                               # _arc_creep: a rigid rod can make the margin above
                               # unreachable, which deadlocked the whole phase)
TAUT_SWITCH_GATE = 0.95        # hand over once every cable is at least this taut

# Rigid ground-start handover. The measured elevation is allowed to lag the swept
# reference: the arc only asymptotes onto the target and the drones track just
# behind it, so demanding the exact target means handover never fires. Latching a
# few degrees low is cheap since tension comes from the measured geometry
# (40 deg needs 2.03 N/drone vs 1.85 N at 45).
HANDOVER_ELEV_TOL  = 8.0       # deg the measurement may lag. The drones track ~6 deg
                               # behind the swept reference, so a 5 deg tol never latched
                               # on the settle path -- it burned the full timeout every
                               # takeoff. 8 deg latches promptly once the sweep completes.
HANDOVER_SETTLE_S  = 1.0       # s within tolerance before latching
HANDOVER_TIMEOUT_S = 3.0       # s after the sweep ends, latch regardless (the OCP
                               # is primed warm through creep, so a shallower-than-
                               # target latch still hands over smoothly)


class CreepController:
    def __init__(self, n, rho, cable_len, N, dt, g, handover_elev_deg, hz,
                 drone_at, publish_ref, logger):
        self.n = n
        self.rho = rho
        self.cable_len = cable_len
        self.N = N
        self.dt = dt                       # horizon node spacing (s)
        self.g = g                         # gravity magnitude (level-hover thrust)
        self.handover_elev_deg = handover_elev_deg
        self.hz = hz                       # planner control rate (Hz)
        self._drone_at = drone_at          # slot i -> measured drone position
        self._publish_ref = publish_ref    # (i, nodes) -> publish drone i's ref
        self._log = logger
        # soft creep state
        self.creep_anchor = None           # latched spawn xy/z per drone
        self.creep_climb = 0.0             # accumulated creep climb height (m)
        # rigid arc creep state
        self.arc_anchor = None             # per-drone (attach, radial)
        self.arc_theta0 = None             # per-drone spawn elevation (rad)
        self.arc_theta = 0.0               # swept elevation reference (rad)
        self._arc_wait = 0.0               # s since the arc sweep finished
        self._arc_hold = 0.0               # s measured elev held in tolerance
        self.lifted_off = False            # gate the creep climb on real liftoff
        self._liftoff_wait = 0.0           # s since TAKEOFF without liftoff
        self._diag_ctr = 0

    def sin_elev(self, i, load_state):
        """sin of the cable's elevation above horizontal, from the measured
        geometry. This is the quantity that sets tension (t = mg/(n sin_elev)), so
        it -- not cable length -- is what decides whether the configuration can
        actually lift the load."""
        ls = load_state
        R = quat_to_rot_np(ls[3:7])
        attach = ls[0:3] + R @ self.rho[i]
        d = attach - self._drone_at(i)
        nrm = float(np.linalg.norm(d))
        if nrm < 1e-6:
            return 0.0
        return float(-d[2] / nrm)          # drone above attach -> positive

    def step(self, load_state, drone_pos, gates, takeoff_seen=True):
        """Publish this tick's creep references and return (handover, reason).
        handover=True means the cables are ready for the OCP to take over."""
        if self.handover_elev_deg > 0.0:
            self._arc_creep(load_state, drone_pos, takeoff_seen)
        else:
            self._soft_creep(load_state, drone_pos, gates)
        return self._handover_check(load_state, gates)

    def _handover_check(self, load_state, gates):
        if self.handover_elev_deg > 0.0:
            # Rigid ground start: wait for the rod to rotate up to a liftable angle
            # (the length gate would hand over at ~0 deg). Only test once the swept
            # reference is done; before that the drones are still on their way up.
            ref_done = self.arc_theta >= np.deg2rad(self.handover_elev_deg) - 1e-9
            sins = [self.sin_elev(i, load_state) for i in range(self.n)]
            lo = np.degrees(np.arcsin(np.clip(min(sins), -1.0, 1.0)))
            if ref_done:
                self._arc_wait += 1.0 / self.hz
                want = np.deg2rad(self.handover_elev_deg - HANDOVER_ELEV_TOL)
                if min(sins) >= np.sin(want):
                    self._arc_hold += 1.0 / self.hz
                else:
                    self._arc_hold = 0.0
                if self._arc_hold >= HANDOVER_SETTLE_S:
                    return True, (f'elevation {lo:.1f}deg reached '
                                  f'(target {self.handover_elev_deg:.0f}, '
                                  f'tol {HANDOVER_ELEV_TOL:.0f})')
                elif self._arc_wait >= HANDOVER_TIMEOUT_S:
                    # Never strand the fleet mid-creep. Tension comes from the
                    # measured geometry, so a shallower angle still works, heavier.
                    self._log.warn(
                        f'[planner] arc creep timed out {self._arc_wait:.1f}s '
                        f'after the sweep ended; measured elevation only '
                        f'{lo:.1f}deg vs target {self.handover_elev_deg:.0f}. '
                        f'Latching anyway.')
                    return True, f'elevation timeout at {lo:.1f}deg'
            return False, None
        if all(g >= TAUT_SWITCH_GATE for (g, _d) in gates):
            return True, 'cables taut'
        return False, None

    def _arc_creep(self, load_state, drone_pos, takeoff_seen=True):
        """Phase-1 takeoff for a RIGID rod starting near-horizontal.

        A rigid rod fixes |drone - attach|. So commanding the drones straight up
        while holding their spawn xy (the soft-cable creep) cannot rotate the rod
        at all -- the horizontal leg stays put, the vertical leg is then pinned by
        the rod length, and the elevation never leaves its spawn value. The drones
        just try to drag the payload up at an angle needing ~13 N per drone.

        Instead sweep each drone along the ARC the rod actually permits, pivoting
        about its (grounded) attach point: elevation ramps from the spawn angle up
        to handover_elev_deg while the radius shrinks to match. The payload stays
        on the ground throughout, so there is no tension to fight, and at handover
        the geometry is the same liftable ~45 deg the elevated worlds spawn with.
        """
        if self.arc_anchor is None:
            ls = load_state
            R = quat_to_rot_np(ls[3:7])
            self.arc_anchor, self.arc_theta0 = [], []
            for i in range(self.n):
                attach = ls[0:3] + R @ self.rho[i]
                d = drone_pos[i] - attach
                horiz = np.array([d[0], d[1], 0.0])
                hn = float(np.linalg.norm(horiz))
                radial = horiz / hn if hn > 1e-6 else np.array([1.0, 0.0, 0.0])
                self.arc_anchor.append((attach.copy(), radial))
                self.arc_theta0.append(float(np.arctan2(d[2], hn)))
            self.arc_theta = float(np.mean(self.arc_theta0))
            self._log.info(
                f'[planner] rigid arc creep: sweeping {np.degrees(self.arc_theta):.1f}'
                f' -> {self.handover_elev_deg:.1f} deg about the grounded attach '
                f'points (rod {self.cable_len:.2f} m)')

        target = np.deg2rad(self.handover_elev_deg)
        dtheta = CREEP_VEL / max(self.cable_len, 1e-6) / self.hz
        if not self.lifted_off:
            for i in range(self.n):
                z_spawn = (self.arc_anchor[i][0][2]
                           + self.cable_len * np.sin(self.arc_theta0[i]))
                if drone_pos[i][2] > z_spawn + LIFTOFF_MARGIN:
                    self.lifted_off = True
                    break
            # LIFTOFF TIMEOUT -- same principle as HANDOVER_TIMEOUT_S below: never
            # strand the fleet in a phase it cannot leave.
            #
            # This gate deadlocks on a rigid rod, and measurably does. LIFTOFF_MARGIN
            # is 0.05 m of z, which on a 0.5 m rod pivoting about a grounded attach
            # point means reaching ~11.5 deg of elevation -- but the reference held
            # here before liftoff is a PURE VERTICAL lead (CREEP_LEAD), which is the
            # exact thing this class's own docstring says "cannot rotate the rod at
            # all". The drones push up against the rod, top out at 9-10 deg (~0.031 m,
            # measured), and never trip the gate. arc_theta then never advances,
            # ref_done never becomes true, the handover timeout below never even arms,
            # and the fleet sits at its spawn angle until someone kills the run.
            #
            # Measured over 8 identical Gazebo runs (R0026-R0033, 2026-08-05): 2 stalled
            # here forever and 3 more only escaped after ~40 s, because whether the
            # drones cross 0.05 m against the rod is a coin flip. Starting the sweep is
            # what physically PERMITS the climb (it adds the inward radial component the
            # rod needs), so on timeout the right move is to start sweeping, not to wait
            # longer. The timer only runs once the fleet has been told to take off --
            # before that the drones are idle on the ground and not rising is correct.
            if takeoff_seen and not self.lifted_off:
                self._liftoff_wait += 1.0 / self.hz
                if self._liftoff_wait >= LIFTOFF_TIMEOUT_S:
                    zs = [drone_pos[i][2] - (self.arc_anchor[i][0][2] + self.cable_len
                                             * np.sin(self.arc_theta0[i]))
                          for i in range(self.n)]
                    self._log.warn(
                        f'[planner] liftoff gate timed out {self._liftoff_wait:.1f}s '
                        f'after TAKEOFF; best rise {max(zs):+.3f} m vs the '
                        f'{LIFTOFF_MARGIN:.2f} m margin (a rigid rod resists the pure '
                        f'vertical lead). Starting the arc sweep anyway — the sweep is '
                        f'what lets them climb.')
                    self.lifted_off = True
        else:
            self.arc_theta = min(target, self.arc_theta + dtheta)

        th = self.arc_theta
        thd = dtheta * self.hz if th < target else 0.0
        for i in range(self.n):
            attach, radial = self.arc_anchor[i]
            L = self.cable_len
            nodes = []
            for k in range(self.N + 1):
                a = min(target, th + thd * self.dt * k)
                p_i = attach + L * (np.cos(a) * radial
                                    + np.array([0.0, 0.0, np.sin(a)]))
                w = thd if a < target else 0.0
                v_i = L * w * (-np.sin(a) * radial
                               + np.array([0.0, 0.0, np.cos(a)]))
                if not self.lifted_off:
                    p_i = p_i + np.array([0.0, 0.0, CREEP_LEAD])
                    v_i = np.zeros(3)
                # level hover thrust, no cable term: the load is still grounded
                nodes.append((p_i, v_i, (0.0, 0.0, self.g), (0.0, 0.0, 0.0)))
            self._publish_ref(i, nodes)

        self._diag_ctr += 1
        if self._diag_ctr % int(max(self.hz, 1)) == 0:
            meas = '  '.join(
                f"d{i}:{np.degrees(np.arcsin(np.clip(self.sin_elev(i, load_state),-1,1))):.1f}"
                for i in range(self.n))
            self._log.info(
                f"[planner arc-creep] ref={np.degrees(th):.1f}deg "
                f"target={self.handover_elev_deg:.0f}deg "
                f"(latch >= {self.handover_elev_deg - HANDOVER_ELEV_TOL:.0f}deg "
                f"for {HANDOVER_SETTLE_S:.0f}s; held {self._arc_hold:.1f}s)  "
                f"measured: {meas}")

    def _soft_creep(self, load_state, drone_pos, gates):
        """Phase-1 takeoff: command each drone to rise straight up at CREEP_VEL,
        level attitude, no cable force.

        The xy reference is ANCHORED to each drone's takeoff position (not its live
        position) and the z reference is a time accumulator, so the reference is a
        fixed point in the air the drone must hold. This gives a real horizontal
        position-hold that resists the inward pull of the tautening cable --
        anchoring to the live position instead lets the drone drift inward and wind
        the attitude up until it tips over."""
        if self.creep_anchor is None:                 # latch spawn xy/z once
            self.creep_anchor = [drone_pos[i].astype(float).copy()
                                 for i in range(self.n)]
        # Don't accumulate climb until the drones leave the ground: the planner runs
        # while they are still disarmed, so the reference would run metres above them
        # and they would rocket up when armed. Hold a small constant lead while
        # grounded so takeoff still initiates.
        if not self.lifted_off:
            self.creep_climb = CREEP_LEAD
            if any(drone_pos[i][2] > self.creep_anchor[i][2] + LIFTOFF_MARGIN
                   for i in range(self.n)):
                self.lifted_off = True
        else:
            self.creep_climb += CREEP_VEL / self.hz

        for i in range(self.n):
            ax, ay, az = self.creep_anchor[i]
            nodes = []
            for k in range(self.N + 1):
                z = az + self.creep_climb + CREEP_VEL * self.dt * k
                # level hover thrust, no cable term: cables still slack
                nodes.append(((ax, ay, z), (0.0, 0.0, CREEP_VEL),
                              (0.0, 0.0, self.g), (0.0, 0.0, 0.0)))
            self._publish_ref(i, nodes)

        self._diag_ctr += 1
        if self._diag_ctr % int(max(self.hz, 1)) == 0:
            if self.handover_elev_deg > 0.0:
                s = '  '.join(
                    f"d{i}:elev={np.degrees(np.arcsin(np.clip(self.sin_elev(i, load_state),-1,1))):.1f}deg"
                    for i in range(self.n))
                s += f"  (handover at {self.handover_elev_deg:.0f}deg)"
            else:
                s = '  '.join(f"d{i}:dist={d:.2f} gate={g:.2f}"
                              for i, (g, d) in enumerate(gates))
            self._log.info(f"[planner creep] vz={CREEP_VEL:.2f}  {s}")
