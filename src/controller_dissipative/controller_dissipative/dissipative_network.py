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
                 node_mass=0.5, substeps=10, elev_deg=45.0, k_slot=18.0):
        self.k_pay = k_pay          # N/m spring to the payload node (rest cable_len)
        self.k_anchor = k_anchor    # N/m spring to the anchor node (rest cone radius)
        self.k_ring = k_ring        # N/m spring to each ring neighbour (rest chord)
        self.k_slot = k_slot        # N/m spring to the phased even-azimuth slot (removes
        #                             the formation's free-rotation degeneracy; 0 disables)
        self.c = c                  # N.s/m graph-Laplacian relative-velocity damping
        self.node_mass = node_mass  # kg virtual node mass (sets network timescale)
        self.substeps = int(substeps)  # integrator sub-steps per control tick
        self.elev_deg = elev_deg    # design cable elevation above horizontal


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

    def _ring_neighbours(self, i):
        """The two azimuth neighbours of node i among the CURRENTLY attached nodes.
        Skips detached nodes so the ring re-closes across the gap as members leave.
        Nodes sit at azimuth 2*pi*j/n, so index order is azimuth order."""
        att = [j for j in range(self.n) if self.attached[j] and j != i]
        if not att:
            return []
        order = sorted(att + [i])
        pos = order.index(i)
        if len(order) == 1:
            return []
        if len(order) == 2:
            return [order[(pos + 1) % 2]]
        return [order[(pos - 1) % len(order)], order[(pos + 1) % len(order)]]

    def _azimuth(self, i, R):
        """Outward horizontal unit bearing of slot i in world (attach ring rotated by
        the measured load yaw)."""
        azi = (R @ self.rho[i])[:2]
        na = float(np.linalg.norm(azi))
        return azi / na if na > 1e-9 else np.array([1.0, 0.0])

    # ── network evolution ──────────────────────────────────────────────────
    def step(self, load_pos, load_quat, load_vel, p_des_load, dt):
        """Advance the network one control tick of length dt (integrated in substeps).
        The payload node is pinned to p_des_load and the anchor to the axis above it, so
        the whole cone translates up as the lift target rises; robot nodes relax onto the
        cone rim under the springs and dissipate their transient through the damper.
        load_quat orients the ring in the load's yaw; load_pos is unused here (the loop
        closes physically through the rigid rods -- reference() reads it only for the
        measured attach point). Read references via reference(i)."""
        p_des = np.asarray(p_des_load, float)
        R = quat_to_rot_np(load_quat)

        payload = p_des
        anchor = p_des + np.array([0.0, 0.0, self._anchor_h])
        # per-slot outward cone target (the force-free equilibrium of the springs).
        azi = [self._azimuth(i, R) for i in range(self.n)]
        # ring rest for the CURRENTLY-attached fleet, so survivors re-space evenly.
        ring_rest = self._ring_rest_for(self.n_attached())

        # phased EVEN azimuth slots: the attached nodes are assigned equal 2*pi/m gaps,
        # and the whole m-gon is phased to the payload's FIXED attach directions (rho) by
        # the circular mean of the attached homes. This pins absolute azimuth (removing
        # the free-rotation degeneracy that destabilises n=2) while keeping spacing even
        # and repositioning minimal. A rho-anchored slot cannot spin with the nodes, so a
        # rotation of the formation is restored. k_slot=0 disables it (pure ring behaviour).
        slot = {}
        att = [i for i in range(self.n) if self.attached[i]]
        m = len(att)
        if self.p.k_slot > 0.0 and m >= 1:
            home = [float(np.arctan2(azi[i][1], azi[i][0])) for i in att]
            implied = [home[r] - 2.0 * np.pi * r / m for r in range(m)]
            ref = float(np.arctan2(np.mean(np.sin(implied)), np.mean(np.cos(implied))))
            for r, i in enumerate(att):
                th = ref + 2.0 * np.pi * r / m
                d = np.array([self._cos_e * np.cos(th), self._cos_e * np.sin(th),
                              self._sin_e])
                slot[i] = payload + self.cable_len * d

        h = dt / max(self.p.substeps, 1)
        for _ in range(max(self.p.substeps, 1)):
            acc = np.zeros((self.n, 3))
            for i in range(self.n):
                if not self.attached[i]:
                    continue
                qi = self.q[i]
                f = np.zeros(3)
                # spring to the payload node, rest length cable_len (sets cable dist).
                f += self._spring(qi, payload, self.p.k_pay, self.cable_len)
                # spring to the anchor node, rest length cone radius (sets height/radius,
                # pinning the node to the OUTWARD rim -- horizontal at equilibrium).
                f += self._spring(qi, anchor, self.p.k_anchor, self._cone_r)
                # spring to the phased even-azimuth slot (rest 0), pinning absolute azimuth.
                if i in slot:
                    f += self._spring(qi, slot[i], self.p.k_slot, 0.0)
                # graph-Laplacian damping toward the pinned anchor & payload (both still,
                # so this is absolute damping) -- 2 edges.
                f += -self.p.c * (self.qd[i] - 0.0) * 2.0
                # ring-neighbour springs (rest = current-fleet chord) + damping (the
                # dissipation that couples the robots and absorbs the detach transient).
                for j in self._ring_neighbours(i):
                    f += self._spring(qi, self.q[j], self.p.k_ring, ring_rest)
                    f += -self.p.c * (self.qd[i] - self.qd[j])
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

    # ── reference extraction ────────────────────────────────────────────────
    def reference(self, i, load_quat, p_des_load, taut_gate=1.0):
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
        u = self.q[i] - p_des
        dist = float(np.linalg.norm(u))
        u = u / dist if dist > 1e-9 else np.array([0.0, 0.0, 1.0])
        p_ref = p_des + self.cable_len * u
        v_ref = self.qd[i].copy()
        # tension from the node elevation: t = m_load g / (n' sin phi), phi = asin(u_z).
        # n' is the TRUE attached count (not eased): the survivors must pick up the
        # departed drone's load share IMMEDIATELY or the load sags. Only the ring-rest
        # REPOSITIONING is eased (see step()); the load-bearing feedforward is not.
        sin_phi = float(np.clip(u[2], 0.05, 1.0))
        n_att = max(self.n_attached(), 1)
        t_i = self.load_mass * self.g / (n_att * sin_phi)
        # cable pulls the drone toward the payload (inward, down) = along -u, gated by
        # how taut the real rod measures right now.
        a_cable = taut_gate * (t_i / self.drone_mass) * (-u)
        a_ff = np.array([0.0, 0.0, self.g]) - a_cable
        return p_ref, v_ref, a_ff, a_cable

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

    # ── introspection (used by the verification harness) ────────────────────
    def cone_target(self, i, load_quat, p_des_load):
        """The force-free equilibrium position of node i (its outward cone slot). The
        network relaxes q[i] onto this; the harness compares against it."""
        p_des = np.asarray(p_des_load, float)
        R = quat_to_rot_np(load_quat)
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
    print("[self-test] ALL PASSED")


if __name__ == '__main__':
    _self_test()
