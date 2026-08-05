"""
dissipative_network.py
-----------------------
Decentralized dissipative virtual-node spring-damper network for a cable-suspended
load -- a faithful port of the method in Quan et al., "Self-Organizing Aerial Swarm
Robotics: A Table-Mechanics-Inspired Approach" (MATLAB in matlab_dissipative_method/).
It replaces the centralized load-cable OCP as the flight-reference generator: a robot
can detach mid-flight (n -> n-1) and the remaining fleet re-settles with the load
suspended, with no solver switch and no hand-designed redistribution transient -- a
member leaving is the network's native operating mode.

The paper's structure (matlab_dissipative_method/Failure/{init,caldata}.m), adapted:
  * NODES  [anchor, robot_1..n, payload]. The paper couples them by springs (adjacency
    W, stiffness K, rest lengths L0) and a graph-Laplacian relative-velocity damper
    (the dissipation), drives the payload node toward the desired trajectory, and puts
    gravity on the payload.
  * ROBOT REFERENCE (caldata.m:46): p_ref_i = p_payload + cable_len * unit(node_i -
    p_payload). THE NODE POSITION IS THE REFERENCE -- the cable direction is set by
    where the virtual node settled, and the drone sits one cable-length out along it.
  * DETACH (init.m:74-79): zero the node's rows/cols in W and M; the rest re-settle;
    the detached drone's reference is frozen (it flies away).

Adaptation to THIS system (rigid rods, load pinned by mocap, pinned "table top"):
  * The PAYLOAD node is pinned to the DESIRED load position p_des (the cone apex). The
    real Gazebo load closes the loop physically through the rigid rods; building the
    reference cone on p_des (not the measured, lagging, possibly-grounded load) is what
    makes the reference LIFT the load instead of collapsing onto a sphere around it.
  * The ANCHOR node is pinned on the vertical axis a height cable_len*sin(elev) above
    p_des. It is the paper's fixed "table-top" -- it and the payload together pin every
    robot node onto the OUTWARD cone rim (radius cable_len*cos(elev) from the axis), so
    the reference can never tilt inward.
  * ROBOT nodes are dynamic. Per node, springs to the payload (rest cable_len), the
    anchor (rest cable_len*cos(elev) = cone radius), and its two ring neighbours (rest =
    the chord of the EVEN m-gon for the currently-attached count m, so survivors re-space
    to equal azimuth gaps after a detach), plus
    graph-Laplacian relative-velocity damping to those same neighbours and to the pinned
    anchor/payload (i.e. absolute damping, since they are still) -- the dissipation that
    absorbs the detach transient. With every spring at its rest length the desired cone
    is an exact force-free equilibrium; the damper makes it the stable one.
  * TENSION feedforward for the tracker (the paper's tracker does not need one): from
    the node's own elevation phi, t_i = m_load g / (n' sin phi_i) shared over the
    currently-attached robots (rises 4->3->2 as drones leave). a_cable pulls in-and-down
    along the cable; a_ff = (0,0,g) - a_cable is the loaded-hover specific thrust, whose
    horizontal part points OUTWARD -- the tracker tilts each drone out to hold station.

Pure numpy + the shared geometry helpers (imported read-only from controller_load_mpc)
-- no ROS, no solver -- so it runs and is self-tested standalone (see __main__), and the
verification harness (verify_dissipative.py) drives it offline before Gazebo.
"""
import numpy as np

from controller_load_mpc.geometry import quat_to_rot_np, attach_points


class DissipativeParams:
    """Spring-damper coefficients. Defaults tuned (see __main__ self-test and
    verify_dissipative.py) for the rigid-short geometry: cable_len=0.5, load_mass=0.4,
    drone_mass=0.6, at a 10 Hz control tick with `substeps` sub-integrations. All SI.

    Stiffnesses set the framework; `c` is the graph-Laplacian damping (the dissipation).
    node_mass sets the network timescale. With semi-implicit Euler at h=dt/substeps the
    scheme is stable while sqrt(k/m)*h << 2 (here ~0.09) and settles in ~1 s with c near
    half of critical (2*sqrt(k*m))."""
    def __init__(self, k_pay=40.0, k_anchor=40.0, k_ring=20.0, c=6.0,
                 node_mass=0.5, substeps=10, elev_deg=45.0, k_slot=18.0,
                 balanced_tensions=False, T_handout=12.0):
        self.k_pay = k_pay          # N/m spring to the payload node (rest cable_len)
        self.k_anchor = k_anchor    # N/m spring to the anchor node (rest cone radius)
        self.k_ring = k_ring        # N/m spring to each ring neighbour (rest chord)
        self.k_slot = k_slot        # N/m spring to the phased even-azimuth slot (removes
        #                             the formation's free-rotation degeneracy; 0 disables)
        self.c = c                  # N.s/m graph-Laplacian relative-velocity damping
        self.node_mass = node_mass  # kg virtual node mass (sets network timescale)
        self.substeps = int(substeps)  # integrator sub-steps per control tick
        self.elev_deg = elev_deg    # design cable elevation above horizontal
        # UNEQUAL (moment-balanced) force sharing. When the attach points are NOT
        # symmetric -- e.g. a mid-flight newcomer welds at an off-centre point, so the
        # fleet is 3 tethers at 120deg + 1 at an interstitial azimuth -- equal tension
        # t_i = mg/(n sinphi) leaves a net moment and the rigid load tilts. With this on,
        # reference() instead solves per-drone tensions from a 6-DOF wrench balance at the
        # ACTUAL attach geometry (see _solve_tensions), so uneven azimuths hold the load
        # LEVEL. This is what lets the fleet visibly reconfigure after an off-centre attach
        # (a centre weld still wants the central lifter -- its moment arm is ~0). Off by
        # default so the verified equal-share/central-lifter paths are unchanged.
        self.balanced_tensions = bool(balanced_tensions)
        # SOFT HAND-OUT time constant (s). A mid-flight newcomer is welded as a CENTRAL
        # lifter (elevation 90deg, moment arm 0 -- pure vertical support that the feedforward-
        # trusting, no-integrator tracker holds exactly) and then CONTINUOUSLY handed out to
        # its off-centre ring slot (design elevation, captured azimuth) over T_handout seconds.
        # Every intermediate is a near-equilibrium cone, so the ACTUAL cable force stays ~equal
        # to the reference feedforward throughout -- the tracker never under-thrusts, and the
        # abrupt central->ring jump that ran the load away (tilt 12->90deg) is removed. Only the
        # newcomer's own handout scalar ramps; the existing tethers stay at full ring. Set the
        # ramp on per node via attach(k, ..., handout=True).
        self.T_handout = float(T_handout)


class DissipativeNetwork:
    """Virtual-node spring-damper network. One instance per fleet; step() once per
    control tick with the measured load pose and the desired load position, then read
    each drone's reference via reference(i)."""

    def __init__(self, n, rho, cable_len, drone_mass, load_mass, g,
                 params: DissipativeParams = None):
        self.n = int(n)
        self.rho = [np.asarray(r, float) for r in rho]   # attach ring, load frame
        self.cable_len = float(cable_len)
        self.drone_mass = float(drone_mass)
        self.load_mass = float(load_mass)
        self.g = float(g)
        self.p = params or DissipativeParams()

        elev = np.radians(self.p.elev_deg)
        self._cos_e = float(np.cos(elev))
        self._sin_e = float(np.sin(elev))
        # anchor height above the payload (on the vertical axis) and the cone radius.
        self._anchor_h = self.cable_len * self._sin_e
        self._cone_r = self.cable_len * self._cos_e
        # ring rest length is recomputed from the CURRENTLY-attached count each step
        # (see _ring_rest_for) so that after a detach the remaining drones redistribute
        # to EVEN azimuth gaps -- the reduced m-gon becomes the spring rest equilibrium.

        # robot node state (world frame); seeded at takeoff/handover from measurements.
        self.q = np.zeros((self.n, 3))
        self.qd = np.zeros((self.n, 3))
        self.attached = [True] * self.n
        # CENTRAL lifters hang straight up from the load centre (no ring azimuth), so a
        # mid-flight newcomer welded near the payload centre adds pure vertical lift instead of
        # being pulled to a side ring slot it cannot hold (which levers the load over). A central
        # node is excluded from the ring/slot springs and pinned to the vertical axis; it still
        # carries its 1/n share via the tension feedforward (at elevation 90deg). Set via
        # attach(k, ..., central=True). Ring members keep the outward-cone behaviour.
        self.central = [False] * self.n
        # per-node cable rest length (payload-spring rest + reference projection). Defaults
        # to the shared cable_len; a mid-flight ATTACH newcomer that hangs on a different
        # cable (e.g. a swung-electromagnet pendulum) can override its own entry via
        # attach(k, ..., cable_len_k=...). The cone geometry (anchor height / radius / slot)
        # stays on the shared cable_len -- only the newcomer's cable distance & reference
        # projection use its own length.
        self.cable_len_i = np.full(self.n, self.cable_len, dtype=float)
        # SOFT HAND-OUT continuation scalar per node, s in [0,1]: 1 = full RING member (the
        # legacy behaviour -- design elevation, full captured moment arm), 0 = CENTRAL lifter
        # (elevation 90deg, zero moment arm). Existing tethers stay at 1. A newcomer attached
        # with handout=True is seeded at 0 and ramped to 1 at _handout_rate = 1/T_handout, so
        # its cable elevation eases from vertical to its slot and its moment arm grows from 0
        # to the captured rho -- a quasi-static central->ring transition that keeps the actual
        # force ~equal to the feedforward the tracker trusts. See _handout_geom / step().
        self.handout = np.ones(self.n, dtype=float)
        self._handout_rate = np.zeros(self.n, dtype=float)
        # per-node SETTLE elevation (deg). The fleet uses the shared design elev_deg; a welded
        # newcomer can be given a STEEPER target so it settles close to where it welds (high &
        # near-vertical over its ring point) instead of transiting far out-and-down to the 45deg
        # rim. A steeper cable is also mostly vertical -> less horizontal shove for the tension
        # solve to balance -> the newcomer joins as a stable ring member with minimal transit.
        self.elev_target = np.full(self.n, float(self.p.elev_deg), dtype=float)
        self._seeded = False

    # ── setup / topology ──────────────────────────────────────────────────
    def seed(self, drone_positions):
        """Bumpless handover: place every node at its measured drone position with zero
        velocity, so the first step() makes no jump."""
        for i in range(self.n):
            self.q[i] = np.asarray(drone_positions[i], float)
        self.qd[:] = 0.0
        self._seeded = True

    def detach(self, k):
        """Drop node k (paper init.m:74-79: zero its rows/cols in W and M). Its springs
        and damping edges vanish, the ring re-closes, the phased even-azimuth slots
        recompute for the reduced fleet, and the remaining nodes re-settle -- their tension
        feedforward rises by itself. Idempotent."""
        if 0 <= k < self.n:
            self.attached[k] = False

    def attach(self, k, measured_pos, cable_len_k=None, central=False, handout=False,
               elev_deg_k=None):
        """Add node k to the network -- the exact reverse of detach. Because step() gates
        EVERY spring/slot/damping edge on self.attached[i], a node that was inert simply
        rejoins: flip its flag true and seed it bumplessly at the measured drone position
        with zero velocity (like seed() :109), so the first step() makes no jump. The ring
        rest (_ring_rest_for), the phased even-azimuth slots, and the tension feedforward
        (n_attached) all recompute for the INCREASED count, so the fleet re-spaces into an
        even (m+1)-gon and every drone's load share drops by itself. If the newcomer hangs
        on a different-length cable (a swung-electromagnet pendulum), pass cable_len_k to
        set its per-node cable rest.

        handout=True enables the SOFT HAND-OUT: the newcomer joins as a CENTRAL lifter
        (handout scalar 0 -- vertical, zero moment arm) and is CONTINUOUSLY handed out to its
        ring slot over T_handout seconds, so its actual cable force tracks the feedforward the
        no-integrator tracker trusts and the load never runs away. handout=False (default)
        keeps the legacy instant full-ring join. central=True (a pure centre weld) overrides
        both -- it stays the frozen vertical lifter. Idempotent."""
        if not (0 <= k < self.n):
            return
        self.attached[k] = True
        self.central[k] = bool(central)
        self.q[k] = np.asarray(measured_pos, float)
        self.qd[k] = 0.0
        if cable_len_k is not None:
            self.cable_len_i[k] = float(cable_len_k)
        # steeper settle target reduces the newcomer's out-and-down transit from its high,
        # near-vertical weld pose (default = the shared fleet elevation).
        self.elev_target[k] = float(elev_deg_k) if elev_deg_k is not None else self.p.elev_deg
        if handout and not central:
            self.handout[k] = 0.0                              # start as a central lifter
            self._handout_rate[k] = 1.0 / max(self.p.T_handout, 1e-3)
        else:
            self.handout[k] = 1.0                              # legacy instant full-ring join
            self._handout_rate[k] = 0.0

    def set_attach_rho(self, k, rho_k):
        """Relocate node k's FIXED body attach point (load-frame). Used at a mid-flight
        weld to make the network model where the newcomer ACTUALLY attached (captured from
        the measured geometry) instead of a nominal even-ring azimuth -- so the
        moment-balanced tension solve (balanced_tensions) computes the RIGHT wrench and the
        real load stays level. No-op if the network is running equal force sharing (rho
        only feeds the azimuth/cone geometry then)."""
        if 0 <= k < self.n:
            self.rho[k] = np.asarray(rho_k, float)

    def n_attached(self):
        return sum(1 for a in self.attached if a)

    def _ring_rest_for(self, m):
        """Chord between adjacent slots of an EVEN m-gon on the cone rim, for the
        CURRENTLY-attached count m. Using this (not the fixed full-fleet chord) as the
        ring-spring rest length makes the reduced fleet's EVEN spacing the force-free
        equilibrium: after a detach the survivors spread to equal azimuth gaps instead
        of collapsing into the gap the departed drone left."""
        m = max(float(m), 2.0)   # float so the eased n_eff gives a smooth rest length
        return 2.0 * self._cone_r * float(np.sin(np.pi / m))

    def _ring_members(self):
        """Attached nodes that sit on the outward ring (excludes central lifters, which hang
        on the vertical axis and take no azimuth slot)."""
        return [j for j in range(self.n) if self.attached[j] and not self.central[j]]

    def _ring_neighbours(self, i):
        """The two azimuth neighbours of node i among the CURRENTLY attached RING nodes.
        Skips detached and central nodes so the ring re-closes across the gap as members
        leave. Nodes sit at azimuth 2*pi*j/n, so index order is azimuth order."""
        if self.central[i]:
            return []
        att = [j for j in self._ring_members() if j != i]
        if not att:
            return []
        order = sorted(att + [i])
        pos = order.index(i)
        if len(order) == 1:
            return []
        if len(order) == 2:
            return [order[(pos + 1) % 2]]
        return [order[(pos - 1) % len(order)], order[(pos + 1) % len(order)]]

    def _handout_geom(self, i):
        """Blended cone geometry for node i under the soft hand-out. Returns (cos_phi,
        sin_phi) at the interpolated cable elevation phi = (1-s)*90deg + s*elev_deg, where
        s = handout[i]. At s=0 (just welded) phi=90deg -> (0,1): a vertical CENTRAL lifter
        (zero cone radius, all support vertical). At s=1 (handed out) phi=elev_deg: the full
        outward RING slot. Everything in step()/reference() built on these eases continuously
        between the two, so every intermediate is a near-equilibrium cone."""
        s = float(np.clip(self.handout[i], 0.0, 1.0))
        phi = (1.0 - s) * (np.pi / 2.0) + s * np.radians(self.elev_target[i])
        return float(np.cos(phi)), float(np.sin(phi))

    def _azimuth(self, i, R):
        """Outward horizontal unit bearing of slot i in world (attach ring rotated by
        the measured load yaw)."""
        azi = (R @ self.rho[i])[:2]
        na = float(np.linalg.norm(azi))
        return azi / na if na > 1e-9 else np.array([1.0, 0.0])

    @staticmethod
    def _yaw_rot(load_quat):
        """Rotation matrix from the load's YAW ONLY (roll/pitch dropped). The formation is
        defined against GRAVITY, so a load TILT must not distort where the drones are told to
        hover. Using the full measured rotation instead lets a small tilt tip the attach-
        direction vectors out of horizontal, shrink some nodes' cone slots inward, and pull
        the load further over -- a self-amplifying tilt (seen post-attach: drones bunch above
        the payload, tilt plateaus ~60deg). Yaw-only breaks that feedback."""
        w, x, y, z = (float(load_quat[0]), float(load_quat[1]),
                      float(load_quat[2]), float(load_quat[3]))
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        c, s = np.cos(yaw), np.sin(yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def _frame_rot(self, load_quat):
        """Frame the formation is built in. Balanced (unequal-force) mode uses YAW ONLY so a
        load tilt cannot distort the references (see _yaw_rot); the verified level/detach
        modes keep the full measured rotation unchanged (a level load makes them identical)."""
        if self.p.balanced_tensions:
            return self._yaw_rot(load_quat)
        return quat_to_rot_np(load_quat)

    # ── network evolution ──────────────────────────────────────────────────
    def _lean(self, load_accel):
        """(R_lean, up_eff, |g_eff|) for a load commanded to accelerate at load_accel.

        A load accelerating at a must have its cables counter an EFFECTIVE gravity
        g_eff = g - a, so the whole reference cone tilts onto it and the tension scales
        with |g_eff|. This is the flatness relation the OCP planner already uses
        (reference_builder.yref_at); without it the network builds a cone that is always
        symmetric about the VERTICAL through p_des, whose net horizontal pull on the load
        is zero -- so the load can only accelerate by first falling behind far enough for
        the geometry error to supply the force. On a circle that shows up as cutting the
        corner (measured -18.5% radius at traj_speed 0.4, r 0.5).

        load_accel None or zero returns (I, +z, g): every existing hover/detach/attach
        path is bit-for-bit unchanged."""
        if load_accel is None:
            return np.eye(3), np.array([0.0, 0.0, 1.0]), self.g
        a = np.asarray(load_accel, float)
        if float(np.linalg.norm(a)) < 1e-9:
            return np.eye(3), np.array([0.0, 0.0, 1.0]), self.g
        up = np.array([a[0], a[1], self.g])          # -g_eff, the cone's new "up"
        mag = float(np.linalg.norm(up))
        up = up / mag
        # Rodrigues rotation taking +z onto up (small tilt; atan(|a|/g)).
        z = np.array([0.0, 0.0, 1.0])
        v = np.cross(z, up)
        s = float(np.linalg.norm(v))
        if s < 1e-9:
            return np.eye(3), up, mag
        c = float(np.dot(z, up))
        vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
        R_lean = np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))
        return R_lean, up, mag

    def step(self, load_pos, load_quat, load_vel, p_des_load, dt, load_accel=None):
        """Advance the network one control tick of length dt (integrated in substeps).
        The payload node is pinned to p_des_load and the anchor to the axis above it, so
        the whole cone translates up as the lift target rises; robot nodes relax onto the
        cone rim under the springs and dissipate their transient through the damper.
        load_quat orients the ring in the load's yaw; load_pos is unused here (the loop
        closes physically through the rigid rods -- reference() reads it only for the
        measured attach point). Read references via reference(i)."""
        p_des = np.asarray(p_des_load, float)
        R = self._frame_rot(load_quat)

        # advance the SOFT HAND-OUT of any newcomer: its handout scalar creeps 0 -> 1 at
        # 1/T_handout per second, easing it from central lifter to full ring member. Done here
        # (once per control tick, not per substep) so the geometry is fixed within a tick.
        for i in range(self.n):
            if self.attached[i] and self._handout_rate[i] > 0.0:
                self.handout[i] = min(1.0, self.handout[i] + self._handout_rate[i] * dt)

        # Tilt the whole cone onto effective gravity, so the formation LEADS the load
        # by exactly the geometry the commanded acceleration needs (see _lean).
        R_lean, _up_eff, _g_eff = self._lean(load_accel)
        payload = p_des
        anchor = p_des + R_lean @ np.array([0.0, 0.0, self._anchor_h])
        # per-slot outward cone target (the force-free equilibrium of the springs).
        azi = [self._azimuth(i, R) for i in range(self.n)]
        # ring rest for the CURRENTLY-attached RING fleet (central lifters excluded), eased by
        # the handout scalars so a newcomer only counts toward the even m-gon as it hands out
        # (n_eff climbs 3 -> 4 smoothly) -- the ring members re-space without a step change.
        ring_att = self._ring_members()
        n_eff = sum(float(self.handout[j]) for j in ring_att)
        ring_rest = self._ring_rest_for(n_eff)

        # phased EVEN azimuth slots: the attached nodes are assigned equal 2*pi/m gaps,
        # and the whole m-gon is phased to the payload's FIXED attach directions (rho) by
        # the circular mean of the attached homes. This pins absolute azimuth (removing
        # the free-rotation degeneracy that destabilises n=2) while keeping spacing even
        # and repositioning minimal. A rho-anchored slot cannot spin with the nodes, so a
        # rotation of the formation is restored. k_slot=0 disables it (pure ring behaviour).
        slot = {}
        att = ring_att
        m = len(att)
        if self.p.balanced_tensions and m >= 1:
            # FIXED-GEOMETRY mode (unequal force sharing). The attach points are physically
            # fixed (tethers + an off-centre weld) and CANNOT be respaced to an even m-gon --
            # forcing that drives every node's reference toward an unreachable ring, the fleet
            # chases it and drifts, and a node can be sent to the OPPOSITE side of its own weld,
            # levering the load over. Instead pin each node to the cone point at ITS OWN fixed
            # attach azimuth (rho direction); the moment-balanced tensions (not respacing) hold
            # the load level. The ring-neighbour chord springs are also dropped below. The cone
            # ELEVATION per node is the handout blend (90deg -> elev_deg), so a just-welded
            # newcomer's slot sits straight overhead and slides out to its azimuth as it hands
            # out -- a continuous central->ring transition.
            for i in att:
                cos_i, sin_i = self._handout_geom(i)
                d = np.array([azi[i][0] * cos_i, azi[i][1] * cos_i, sin_i])
                slot[i] = payload + self.cable_len * (R_lean @ d)
        elif self.p.k_slot > 0.0 and m >= 1:
            # WEIGHTED even azimuth slots. Each ring member occupies an angular width
            # proportional to its hand-out weight, so a just-welded newcomer (handout~0) takes
            # ~no width and the existing members keep their current even (m-1)-gon; as the
            # newcomer hands out (weight 0->1) the slots morph CONTINUOUSLY to the even m-gon.
            # This removes the 120->90deg one-tick azimuth step that slammed the existing
            # drones at the weld. Spacing stays keyed on the SAME eased count as ring_rest
            # (adjacent full nodes are 2*pi/n_eff apart), so the ring and slot springs agree
            # throughout the hand-out instead of fighting. With all handout==1 this reduces
            # EXACTLY to the plain even m-gon (frac_r=(r+0.5)/m, ref absorbs the half-slot
            # offset), so detach/steady behaviour is unchanged.
            w = [max(float(self.handout[i]), 0.0) for i in att]
            total = sum(w)
            if total > 1e-6:
                home = [float(np.arctan2(azi[i][1], azi[i][0])) for i in att]
                frac, acc = [], 0.0
                for r in range(m):
                    frac.append((acc + 0.5 * w[r]) / total)   # angular fraction of slot CENTRE
                    acc += w[r]
                # phase the pattern to the members' actual bearings, weighted by presence so a
                # barely-present newcomer does not drag the phase while it is still overhead.
                implied = [home[r] - 2.0 * np.pi * frac[r] for r in range(m)]
                cw = np.asarray(w)
                ref = float(np.arctan2(float(np.sum(cw * np.sin(implied))),
                                       float(np.sum(cw * np.cos(implied)))))
                for r, i in enumerate(att):
                    th = ref + 2.0 * np.pi * frac[r]
                    cos_i, sin_i = self._handout_geom(i)   # =cos_e/sin_e unless handing out
                    d = np.array([cos_i * np.cos(th), cos_i * np.sin(th), sin_i])
                    slot[i] = payload + self.cable_len * (R_lean @ d)

        h = dt / max(self.p.substeps, 1)
        for _ in range(max(self.p.substeps, 1)):
            acc = np.zeros((self.n, 3))
            for i in range(self.n):
                if not self.attached[i]:
                    continue
                qi = self.q[i]
                f = np.zeros(3)
                # spring to the payload node, rest length cable_len (sets cable dist).
                f += self._spring(qi, payload, self.p.k_pay, self.cable_len_i[i])
                if self.central[i]:
                    # CENTRAL lifter: pin to the vertical axis directly above the load centre
                    # (rest 0 to the point one cable_len straight up). No cone/ring/slot springs,
                    # so it hovers over the centre and adds pure vertical lift -- it cannot be
                    # pulled to a side azimuth and lever the load over.
                    v_target = payload + R_lean @ np.array(
                        [0.0, 0.0, self.cable_len_i[i]])
                    f += self._spring(qi, v_target, self.p.k_anchor, 0.0)
                    f += -self.p.c * self.qd[i] * 2.0
                    acc[i] = f / self.p.node_mass
                    continue
                # spring to the anchor node. Under a soft hand-out the anchor point and the
                # cone-radius rest are the handout blend: at handout=0 the anchor sits one
                # cable_len straight up with rest 0 (the node is pinned vertical -- a central
                # lifter), easing to the design rim (height _anchor_h, rest _cone_r) at
                # handout=1. Equals the fixed anchor/_cone_r for a full ring member.
                cos_i, sin_i = self._handout_geom(i)
                anchor_i = payload + R_lean @ np.array(
                    [0.0, 0.0, self.cable_len * sin_i])
                f += self._spring(qi, anchor_i, self.p.k_anchor, self.cable_len * cos_i)
                # spring to the phased even-azimuth slot (rest 0), pinning absolute azimuth.
                # Scaled by handout so a just-welded newcomer feels no side pull (it is
                # central); the azimuth pin fades in as it hands out.
                if i in slot:
                    f += self._spring(qi, slot[i], self.p.k_slot * self.handout[i], 0.0)
                # graph-Laplacian damping toward the pinned anchor & payload (both still,
                # so this is absolute damping) -- 2 edges.
                f += -self.p.c * (self.qd[i] - 0.0) * 2.0
                # ring-neighbour springs (rest = current-fleet chord) + damping (the
                # dissipation that couples the robots and absorbs the detach transient).
                # Skipped in balanced (fixed-geometry) mode: the even-chord rest would pull the
                # fixed attach points toward an unreachable even m-gon (the drift/lever failure).
                # Each edge is scaled by BOTH endpoints' handout, so a mid-hand-out newcomer
                # couples in gradually (no ring yank at the instant of weld).
                if not self.p.balanced_tensions:
                    for j in self._ring_neighbours(i):
                        gij = self.handout[i] * self.handout[j]
                        f += gij * self._spring(qi, self.q[j], self.p.k_ring, ring_rest)
                        f += -gij * self.p.c * (self.qd[i] - self.qd[j])
                acc[i] = f / self.p.node_mass
            # semi-implicit Euler (velocity then position) -- stable for these springs.
            for i in range(self.n):
                if not self.attached[i]:
                    continue
                self.qd[i] += h * acc[i]
                self.q[i] += h * self.qd[i]

    @staticmethod
    def _spring(a, b, k, rest):
        """Hooke force on node at `a` from a spring to fixed/other point `b` with
        stiffness k and rest length `rest`: pulls toward b when stretched, pushes away
        when compressed. Returns a length-3 array."""
        d = b - a
        dist = float(np.linalg.norm(d))
        if dist < 1e-9:
            return np.zeros(3)
        u = d / dist
        return k * (dist - rest) * u

    # ── unequal (moment-balanced) force sharing ─────────────────────────────
    def _attach_frame(self, i, R, p_des):
        """Attach point (world) and up-outward cable unit vector for node i, built on the
        FIXED body attach point rho_i (not the load centre). r_i = R@rho_i is the moment
        arm about the load COM; a_i = p_des + r_i is where the cable meets the load; the
        drone sits one cable-length out along u_i = unit(node - a_i). A central lifter is
        pinned vertical (u = +z) so it adds no horizontal disturbance / moment.

        The moment arm uses the TRUE captured attach point rho_i even mid-hand-out: a
        vertical lift at an OFF-CENTRE weld still applies a real moment (rho_i x F), so the
        wrench solve must see it to make the OTHER cables compensate and keep the load level.
        (Scaling the arm to 0 during hand-out was a bug -- it hid the newcomer's real off-
        centre moment, so nothing cancelled it and the load tilted.) The soft hand-out eases
        only the cable DIRECTION (the reference elevation, via _handout_geom), not the arm."""
        r_i = R @ self.rho[i]
        a_i = p_des + r_i
        if self.central[i]:
            return a_i, r_i, np.array([0.0, 0.0, 1.0])
        d = self.q[i] - a_i
        nd = float(np.linalg.norm(d))
        u = d / nd if nd > 1e-9 else np.array([0.0, 0.0, 1.0])
        return a_i, r_i, u

    def _solve_tensions(self, R, p_des, load_accel=None):
        """Per-drone cable tensions that balance the load's 6-DOF wrench at the ACTUAL
        (possibly asymmetric) attach geometry -- the unequal force sharing.

        Each cable i pulls the load toward the drone with force T_i*u_i at attach point
        r_i (rel COM), contributing wrench column [u_i; r_i x u_i]. We want the fleet to
        supply the load's weight (+ desired accel) with ZERO net moment (level):
            A T = w,  w = [ m_load*(a_des + g z) ; 0 ],  A[:,i] = [u_i ; r_i x u_i].
        Least-squares (min residual for m<6 cables); tensions clamped >=0 (cables can only
        pull) and then rescaled so the vertical support exactly equals the load weight --
        the load-bearing term the tracker must not get wrong. Returns {node_index: T}."""
        idx = [i for i in range(self.n) if self.attached[i]]
        if not idx:
            return {}
        A = np.zeros((6, len(idx)))
        for c, i in enumerate(idx):
            _, r_i, u = self._attach_frame(i, R, p_des)
            A[0:3, c] = u
            A[3:6, c] = np.cross(r_i, u)
        a_des = np.zeros(3) if load_accel is None else np.asarray(load_accel, float)
        F = self.load_mass * (a_des + np.array([0.0, 0.0, self.g]))
        w = np.concatenate([F, np.zeros(3)])
        # With m<6 cables the 6-DOF wrench is over-determined, so we weight the rows: the
        # load MUST stay supported (Fz) and LEVEL (Mx,My), so those dominate; a small net
        # horizontal force (Fx,Fy) is left for the position tracker to trim, and yaw torque
        # (Mz, barely controllable with near-vertical cables) is de-emphasised. Weighted
        # least squares distributes the unavoidable residual onto the least-critical axes.
        W = np.array([3.0, 3.0, 40.0, 40.0, 40.0, 1.0])
        # The tension split is statically INDETERMINATE (any 3 of 4 up-out cables can hold the
        # load), so plain min-residual can zero a redundant cable -- the newcomer would carry
        # nothing and never actually share the load. Regularise toward EQUAL sharing (each
        # drone's design-elevation share t0 = m_load g / (m sin_elev)) with a small weight lam:
        # it breaks the degeneracy so every drone carries a fair share, while the wrench rows
        # (far heavier) still keep the load supported & level. Standard cable-robot tension
        # distribution.
        m = len(idx)
        t0 = self.load_mass * self.g / (max(m, 1) * max(self._sin_e, 1e-3))
        lam = 1.0
        A_aug = np.vstack([A * W[:, None], np.sqrt(lam) * np.eye(m)])
        w_aug = np.concatenate([w * W, np.sqrt(lam) * t0 * np.ones(m)])
        T, *_ = np.linalg.lstsq(A_aug, w_aug, rcond=None)
        T = np.clip(T, 0.0, None)
        return {i: float(T[c]) for c, i in enumerate(idx)}

    # ── reference extraction ────────────────────────────────────────────────
    def reference(self, i, load_quat, p_des_load, taut_gate=1.0, load_accel=None):
        """Per-drone reference (p_ref, v_ref, a_ff, a_cable), each a length-3 numpy
        array -- the tuple the wire-format publisher expects.

        p_ref is the paper's caldata.m:46 map: project the virtual node onto the taut
        sphere of the payload node, p_ref = p_payload + cable_len * unit(node - payload).
        Because the node is pinned to the outward cone rim, unit(node - payload) always
        points up-and-outward, so p_ref (and the a_ff below) can never tilt inward.

        The tension feedforward uses the NODE's elevation (consistent with p_ref, and
        never shallow because the node sits on the design cone), shared over the currently
        attached drones so it rises as members detach. a_cable pulls in-and-down;
        a_ff = (0,0,g) - a_cable is the loaded-hover specific thrust (outward, > g)."""
        p_des = np.asarray(p_des_load, float)
        v_ref = self.qd[i].copy()
        # Effective gravity for a load commanded to accelerate (see _lean). a_des is
        # also added to every drone's thrust feedforward: the whole formation
        # translates with the load, so it must accelerate with it too. All three
        # reduce exactly to the old expressions when load_accel is None/zero.
        _R, up_eff, g_eff = self._lean(load_accel)
        a_des = (np.zeros(3) if load_accel is None
                 else np.asarray(load_accel, float))

        if self.p.balanced_tensions:
            # UNEQUAL force sharing: reference and cable frame are built on the node's OWN
            # fixed attach point (a_i = p_des + R@rho_i), and the tension comes from the
            # 6-DOF wrench balance over the whole fleet -- so an asymmetric attach set holds
            # the load LEVEL with uneven per-drone tensions instead of tilting it.
            R = self._frame_rot(load_quat)
            a_i, _, u = self._attach_frame(i, R, p_des)
            p_ref = a_i + self.cable_len_i[i] * u
            t_i = self._solve_tensions(R, p_des, load_accel).get(
                i, self.load_mass * self.g / max(self.n_attached(), 1))
            a_cable = taut_gate * (t_i / self.drone_mass) * (-u)
            a_ff = np.array([0.0, 0.0, self.g]) + a_des - a_cable
            return p_ref, v_ref, a_ff, a_cable

        u = self.q[i] - p_des
        dist = float(np.linalg.norm(u))
        u = u / dist if dist > 1e-9 else np.array([0.0, 0.0, 1.0])
        # a central lifter always references straight up over the load centre (elevation 90deg),
        # so its cable feed-forward is purely vertical -- no inward/side pull to tilt the load.
        if self.central[i]:
            u = up_eff
        p_ref = p_des + self.cable_len_i[i] * u
        # tension from the node elevation: t = m_load g / (n' sin phi), phi = asin(u_z).
        # n' is the TRUE attached count (not eased): the survivors must pick up the
        # departed drone's load share IMMEDIATELY or the load sags. Only the ring-rest
        # REPOSITIONING is eased (see step()); the load-bearing feedforward is not.
        sin_phi = float(np.clip(float(u @ up_eff), 0.05, 1.0))
        n_att = max(self.n_attached(), 1)
        t_i = self.load_mass * g_eff / (n_att * sin_phi)
        # cable pulls the drone toward the payload (inward, down) = along -u, gated by
        # how taut the real rod measures right now.
        a_cable = taut_gate * (t_i / self.drone_mass) * (-u)
        a_ff = np.array([0.0, 0.0, self.g]) + a_des - a_cable
        return p_ref, v_ref, a_ff, a_cable

    def horizon_references(self, load_quat, p_des_seq, dt, taut_gates=None,
                           load_accel_seq=None):
        """Per-node references over a whole horizon: [drone][node] -> (p, v, a_ff, a_c).

        Rolls a COPY of the network forward along the supplied future load targets
        (p_des_seq[k] is the target at horizon node k, k=0 being now) and reads the
        references off each predicted state. Node 0 is the live state -- the caller has
        already step()ed it for this tick -- so only k>=1 are predicted.

        This replaces repeating node 0 across the horizon. That repeat is exact only
        while the formation's geometry relative to the load is constant; on a curved
        path it is not, because the virtual nodes trail the moving cone and that trailing
        direction rotates with the velocity. Measured on a circle at traj_speed 0.6:
        a_cable swings by 68% of its own magnitude over one 2 s horizon, and the drone's
        offset from the load drifts 0.35 m -- so a repeated node 0 hands the tracker a
        prediction that is badly wrong by the end of its lookahead.

        The network state is restored before returning, so this is side-effect free and
        the caller's live q/qd/handout are untouched. With a constant p_des_seq (hover,
        LAND) every node is identical and the result matches the old repeat exactly.
        """
        q0, qd0, ho0 = self.q.copy(), self.qd.copy(), self.handout.copy()
        gates = taut_gates if taut_gates is not None else [1.0] * self.n
        # Fail by NAME, not by IndexError. A caller sizing this list to the tethered
        # count instead of the network count crashed the planner mid-attach, and the
        # traceback pointed at `gates[i]` inside a comprehension rather than at the
        # caller that got the length wrong (see dissipative_node._network_plan).
        if len(gates) != self.n:
            raise ValueError(
                f'taut_gates has {len(gates)} entries but the network has {self.n} '
                f'nodes. Size it to the NETWORK (n_net), not the tethered count.')
        out = [[] for _ in range(self.n)]
        try:
            for k, p_des in enumerate(p_des_seq):
                a_k = None if load_accel_seq is None else load_accel_seq[k]
                if k > 0:
                    # load_pos is unused by step(); the loop closes through p_des.
                    self.step(p_des, load_quat, np.zeros(3), p_des, dt, load_accel=a_k)
                for i in range(self.n):
                    out[i].append(self.reference(i, load_quat, p_des,
                                                 taut_gate=gates[i], load_accel=a_k))
        finally:
            self.q, self.qd, self.handout = q0, qd0, ho0
        return out

    def fly_away_reference(self, i, clearance=0.6):
        """Reference for a just-DETACHED drone: rise straight up from its frozen node
        position by `clearance` metres and hold, level attitude, NO cable term
        (a_cable=0 is the free-flight dynamics the tracker already handles). The seam
        where a collaborator's controller can take over."""
        p_ref = self.q[i].copy()
        p_ref[2] += clearance
        v_ref = np.zeros(3)
        a_ff = np.array([0.0, 0.0, self.g])
        a_cable = np.zeros(3)
        return p_ref, v_ref, a_ff, a_cable

    def net_wrench(self, load_quat, p_des_load, load_accel=None):
        """Total force and moment (about the load COM) the balanced-tension feedforward
        applies to the load, and the residual against the desired (weight, zero-moment)
        wrench. Used by the verify harness to assert the load stays LEVEL (moment~0) and
        supported. Returns (F_total, M_total, residual_force, residual_moment)."""
        R = self._frame_rot(load_quat)
        p_des = np.asarray(p_des_load, float)
        Tsol = self._solve_tensions(R, p_des, load_accel)
        F = np.zeros(3)
        M = np.zeros(3)
        for i, T in Tsol.items():
            _, r_i, u = self._attach_frame(i, R, p_des)
            F += T * u
            M += np.cross(r_i, T * u)
        a_des = np.zeros(3) if load_accel is None else np.asarray(load_accel, float)
        F_want = self.load_mass * (a_des + np.array([0.0, 0.0, self.g]))
        return F, M, F - F_want, M

    # ── introspection (used by the verification harness) ────────────────────
    def cone_target(self, i, load_quat, p_des_load):
        """The force-free equilibrium position of node i (its outward cone slot). The
        network relaxes q[i] onto this; the harness compares against it."""
        p_des = np.asarray(p_des_load, float)
        R = self._frame_rot(load_quat)
        azi = self._azimuth(i, R)
        cone_dir = np.array([azi[0] * self._cos_e, azi[1] * self._cos_e, self._sin_e])
        return p_des + self.cable_len * cone_dir


# ─────────────────────────────────────────────────────────────────────────────
# Fast, plot-free self-test: settle-from-perturbation onto the OUTWARD cone,
# node-based reference, tension symmetry, a_ff outward guard, 4->3 detach re-settle.
# Run:  python3 -m controller_dissipative.dissipative_network
# ─────────────────────────────────────────────────────────────────────────────
def _self_test():
    g = 9.81
    cable_len = 0.5
    load_mass = 0.4
    drone_mass = 0.6
    n = 4
    rho = attach_points(n, 0.08, 0.025)
    net = DissipativeNetwork(n, rho, cable_len, drone_mass, load_mass, g)

    load_quat = np.array([1.0, 0.0, 0.0, 0.0])   # level load
    load_vel = np.zeros(3)
    p_des = np.array([0.0, 0.0, 0.6])            # desired load (cone apex)

    # seed the nodes at a PERTURBED formation (all shoved +x by 0.15 m off their cone
    # slots) and check the network relaxes them back onto the symmetric outward cone.
    seed = [net.cone_target(i, load_quat, p_des) + np.array([0.15, 0.0, 0.0])
            for i in range(n)]
    net.seed(seed)

    dt = 0.1
    for _ in range(300):                         # 30 s
        net.step([0, 0, 0.6], load_quat, load_vel, p_des, dt)

    # every node should have relaxed onto its cone slot; tensions near-equal.
    errs, tens = [], []
    for i in range(n):
        errs.append(float(np.linalg.norm(net.q[i] - net.cone_target(i, load_quat, p_des))))
        _, _, _, ac = net.reference(i, load_quat, p_des)
        tens.append(float(np.linalg.norm(ac)) * drone_mass)
    errs = np.array(errs); tens = np.array(tens)
    assert np.all(errs < 0.03), f"nodes did not relax onto the cone: {errs}"
    assert (tens.max() - tens.min()) < 0.15 * tens.mean(), \
        f"tensions not symmetric after settle: {tens}"
    assert np.all(np.isfinite(net.q)), "network diverged (nan/inf)"
    exp4 = load_mass * g / (4 * np.sin(np.radians(45)))
    print(f"[self-test] n=4 relax-onto-cone OK: err={np.round(errs,3)} "
          f"tension/drone={np.round(tens,3)} N (expect ~{exp4:.2f})")

    # OUTWARD guard (the inward-tilt bug): the reference position AND the specific-thrust
    # feedforward must both point outward of the payload for every drone.
    for i in range(n):
        p_ref, _, a_ff, a_cable = net.reference(i, load_quat, p_des)
        assert np.allclose(a_ff, np.array([0.0, 0.0, g]) - a_cable), \
            f"a_ff must equal (0,0,g)-a_cable (node {i})"
        outward = net._azimuth(i, quat_to_rot_np(load_quat))
        assert float((p_ref[:2] - p_des[:2]) @ outward) > 0.0, \
            f"p_ref must be outward of the payload (node {i}): {p_ref}"
        assert float(a_ff[:2] @ outward) > 0.0, \
            f"a_ff horizontal must point outward (node {i}): {a_ff}"
    print("[self-test] node-based reference & a_ff tilt OUTWARD OK")

    # LIFT: raise the desired load height; the reference must translate up ~1:1 and stay
    # on the cone (a vertical lift, not an inward collapse). Relax at the new target.
    p_hi = p_des + np.array([0.0, 0.0, 0.3])
    for _ in range(300):
        net.step([0, 0, 0.9], load_quat, load_vel, p_hi, dt)
    p_lo_ref, _, _, _ = net.reference(0, load_quat, p_des)
    p_hi_ref, _, _, _ = net.reference(0, load_quat, p_hi)
    u = (p_hi_ref - p_hi) / cable_len
    elev = np.degrees(np.arcsin(np.clip(u[2], -1.0, 1.0)))
    assert 40.0 < elev < 50.0, f"cable elevation not ~45deg after lift: {elev:.0f}"
    print(f"[self-test] lift tracking OK: cone apex followed p_des, cable elev "
          f"{elev:.0f}deg")

    # 4->3 detach: drop node 2, re-settle, tension per remaining drone must rise.
    t_before = np.mean([float(np.linalg.norm(net.reference(i, load_quat, p_hi)[3]))
                        * drone_mass for i in range(n) if net.attached[i]])
    net.detach(2)
    for _ in range(300):
        net.step([0, 0, 0.9], load_quat, load_vel, p_hi, dt)
    tens3 = [float(np.linalg.norm(net.reference(i, load_quat, p_hi)[3])) * drone_mass
             for i in range(n) if net.attached[i]]
    tens3 = np.array(tens3)
    assert np.all(np.isfinite(net.q)), "network diverged after detach"
    assert tens3.mean() > t_before, \
        f"tension per drone did not rise after 4->3: {t_before:.3f} -> {tens3.mean():.3f}"
    exp3 = load_mass * g / (3 * np.sin(np.radians(45)))
    print(f"[self-test] 4->3 detach OK: tension/drone {t_before:.3f} -> "
          f"{tens3.mean():.3f} N (expect ~{exp3:.2f}), remaining nodes finite & re-settled")

    # EVEN redistribution: the 3 survivors must sit at ~120 deg azimuth gaps (not merely
    # close the gap left by the departed drone -- the ring-rest-for-m fix).
    az = sorted(np.degrees(np.arctan2(net.q[i][1] - p_hi[1], net.q[i][0] - p_hi[0]))
                % 360.0 for i in range(n) if net.attached[i])
    gaps = np.diff(az + [az[0] + 360.0])
    assert np.all(np.abs(gaps - 120.0) < 15.0), \
        f"survivors not evenly redistributed after 4->3: azimuth gaps {np.round(gaps,1)} deg"
    print(f"[self-test] even redistribution OK: 3 survivors at azimuth gaps "
          f"{np.round(gaps,1)} deg (expect ~120)")

    # 3->4 ATTACH (reverse of detach): a newcomer flies in and node 2 rejoins. Seed it at
    # an approach position off its cone slot, re-settle, and confirm the load share per
    # drone DROPS back toward the 4-drone value and the fleet re-spaces to ~90 deg gaps.
    t_before_att = np.mean([float(np.linalg.norm(net.reference(i, load_quat, p_hi)[3]))
                            * drone_mass for i in range(n) if net.attached[i]])
    approach = net.cone_target(2, load_quat, p_hi) + np.array([0.12, -0.10, 0.08])
    net.attach(2, approach)
    for _ in range(300):
        net.step([0, 0, 0.9], load_quat, load_vel, p_hi, dt)
    assert np.all(np.isfinite(net.q)), "network diverged after attach"
    assert net.n_attached() == 4, f"attach did not restore n=4: {net.n_attached()}"
    tens4 = np.array([float(np.linalg.norm(net.reference(i, load_quat, p_hi)[3])) *
                      drone_mass for i in range(n)])
    assert tens4.mean() < t_before_att, \
        f"load share per drone did not drop after 3->4: {t_before_att:.3f} -> {tens4.mean():.3f}"
    err2 = float(np.linalg.norm(net.q[2] - net.cone_target(2, load_quat, p_hi)))
    assert err2 < 0.03, f"newcomer did not settle onto its cone slot: {err2:.3f} m"
    az4 = sorted(np.degrees(np.arctan2(net.q[i][1] - p_hi[1], net.q[i][0] - p_hi[0]))
                 % 360.0 for i in range(n))
    gaps4 = np.diff(az4 + [az4[0] + 360.0])
    assert np.all(np.abs(gaps4 - 90.0) < 15.0), \
        f"fleet not even after 3->4 attach: azimuth gaps {np.round(gaps4,1)} deg"
    print(f"[self-test] 3->4 attach OK: tension/drone {t_before_att:.3f} -> "
          f"{tens4.mean():.3f} N (expect ~{exp4:.2f}), newcomer settled, gaps "
          f"{np.round(gaps4,1)} deg (expect ~90)")

    # ROTATIONAL STABILITY (the n=2 degeneracy fix): at n=2 the ring fixes the two nodes'
    # SEPARATION but not the formation's absolute rotation -- without the phased-slot spring
    # the pair can spin freely (what destabilised n=2 in sim). Settle a 2-node net, apply a
    # rigid +30 deg azimuth spin, and confirm the slots restore it.
    net2 = DissipativeNetwork(2, attach_points(2, 0.08, 0.025), cable_len, drone_mass,
                              load_mass, g)
    net2.seed([net2.cone_target(i, load_quat, p_des) for i in range(2)])
    for _ in range(200):
        net2.step([0, 0, 0.6], load_quat, load_vel, p_des, dt)
    settled = net2.q.copy()
    a = np.radians(30.0)
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    for i in range(2):
        net2.q[i] = p_des + Rz @ (net2.q[i] - p_des)
    net2.qd[:] = 0.0
    for _ in range(300):
        net2.step([0, 0, 0.6], load_quat, load_vel, p_des, dt)
    ret = max(float(np.linalg.norm(net2.q[i] - settled[i])) for i in range(2))
    assert ret < 0.05, f"n=2 did not restore after a +30deg azimuth spin: {ret:.3f} m " \
                       f"(rotational degeneracy -- k_slot too weak?)"
    print(f"[self-test] n=2 rotational stability OK: restored to {ret*1000:.0f} mm after "
          f"a +30deg spin (free-rotation degeneracy removed)")

    # detached node fly-away reference: no cable term, rises.
    _, _, _, ac = net.fly_away_reference(2)
    assert np.allclose(ac, 0.0), "fly-away reference must have a_cable=0"
    print("[self-test] fly-away reference OK (a_cable=0)")

    # UNEQUAL FORCE SHARING (moment-balanced): 3 tethers at 120deg + a 4th welded at an
    # OFF-CENTRE interstitial attach point. With equal sharing the asymmetric set leaves a
    # net moment (the tilt bug); the wrench solve must hold the load LEVEL (moment ~0) with
    # UNEQUAL tensions, while still supporting the full weight.
    rho_asym = attach_points(3, 0.08, 0.025)                    # 3 at 120deg, radius 0.08
    rho_asym.append(np.array([0.08 * np.cos(np.radians(60.0)),  # 4th tucked between two
                              0.08 * np.sin(np.radians(60.0)), 0.025]))
    pb = DissipativeParams(balanced_tensions=True)
    netb = DissipativeNetwork(4, rho_asym, cable_len, drone_mass, load_mass, g, pb)
    netb.seed([netb.cone_target(i, load_quat, p_des) for i in range(4)])
    for _ in range(300):
        netb.step([0, 0, 0.6], load_quat, load_vel, p_des, dt)
    F, M, rF, rM = netb.net_wrench(load_quat, p_des)
    tb = np.array([netb._solve_tensions(quat_to_rot_np(load_quat), p_des)[i]
                   for i in range(4)])
    assert np.all(np.isfinite(netb.q)), "balanced network diverged"
    assert abs(rF[2]) < 0.01 * F[2], \
        f"weight not supported: Fz residual {rF[2]:.4f} N of {F[2]:.2f} N"
    assert np.linalg.norm(rM) < 0.06, \
        f"moment not balanced (load would tilt): |M|={np.linalg.norm(rM):.3f} N.m"
    assert np.linalg.norm(rF[:2]) < 0.5, \
        f"net horizontal force too large: {rF[:2]}"
    assert (tb.max() - tb.min()) > 0.02 * tb.mean(), \
        f"tensions should be UNEQUAL for asymmetric geometry: {np.round(tb,3)}"
    assert np.all(tb >= 0.0), f"cable tensions must be non-negative: {np.round(tb,3)}"
    print(f"[self-test] unequal force sharing OK: tensions {np.round(tb,3)} N "
          f"(unequal), |moment|={np.linalg.norm(rM):.4f} N.m (~0 -> level), "
          f"Fz residual {rF[2]:.1e} N")
    print("[self-test] ALL PASSED")


if __name__ == '__main__':
    _self_test()
