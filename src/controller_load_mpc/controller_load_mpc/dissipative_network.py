"""
dissipative_network.py
-----------------------
Decentralized dissipative virtual-node spring-damper network for a cable-suspended
load, the reference generator behind the `dissipative` planner. It replaces the
centralized load-cable OCP (planner_ocp.py) for the flight phase with a network whose
NATIVE operating mode is members leaving/joining -- so a drone can detach mid-flight
without switching solvers or hand-designing a redistribution transient.

Model (table-mechanics-inspired, all world frame):
  * The PAYLOAD node is PINNED to the measured load pose (the real Gazebo body), so
    the network never has to model the load's own dynamics and the loop closes through
    the true plant. Detach-time redistribution flows through the measured load motion.
  * Each ROBOT gets a virtual point-mass node. Per node, three forces:
      - a STATION spring to its nominal formation anchor g_i (drone station-keeping),
      - a CABLE-LENGTH spring pulling it to exactly cable_len from the measured attach
        point (keeps the rod taut, direction free),
      - LAPLACIAN DAMPING (the dissipation): each node is damped toward the payload
        velocity and toward its ring neighbours' velocity. This is the term the MPC
        lacks; it absorbs the detach transient.
  * The per-drone reference is the node projected onto the taut sphere of the MEASURED
    payload (p_ref = attach + cable_len * unit(node - attach)), so it is ALWAYS a valid
    taut configuration regardless of network transients.
  * The cable-tension feedforward is the analytic hover tension shared over the CURRENTLY
    attached drones, t_i = m_load g / (n' sin phi_i): it rises by itself as drones come
    off (4 -> 3 -> 2), which is exactly the redistribution feedforward the trackers need.

detach(k) simply flips node k out of the attached set: its springs and damping edges
vanish, n' drops, and the remaining nodes re-settle under the same forces. There is no
special-cased transient.

Pure numpy + the shared geometry helpers -- no ROS, no solver -- so it runs and is
self-tested standalone (see __main__).
"""
import numpy as np

from .geometry import quat_to_rot_np, attach_points, nominal_cable_dirs


class DissipativeParams:
    """Spring-damper coefficients. Defaults tuned (see __main__ self-test) for the
    rigid-short geometry: cable_len=0.5, load_mass=0.4, drone_mass=0.6. Retune K_cable
    / c for a different scale. All SI."""
    def __init__(self, k_station=8.0, k_cable=40.0, c=6.0, c_ring=2.0,
                 node_mass=0.5, substeps=10, elev_deg=45.0):
        self.k_station = k_station      # N/m pull toward the formation anchor
        self.k_cable = k_cable          # N/m pull toward cable_len off the attach point
        self.c = c                      # N.s/m damping toward the payload velocity
        self.c_ring = c_ring            # N.s/m damping toward ring-neighbour velocity
        self.node_mass = node_mass      # kg virtual node mass (sets network timescale)
        self.substeps = int(substeps)   # integrator sub-steps per control tick
        self.elev_deg = elev_deg        # design cable elevation for the anchors


class DissipativeNetwork:
    """Virtual-node spring-damper network. One instance per fleet; step() once per
    control tick with the measured load pose and the desired load position."""

    def __init__(self, n, rho, cable_len, drone_mass, load_mass, g,
                 params: DissipativeParams = None):
        self.n = int(n)
        self.rho = [np.asarray(r, float) for r in rho]   # attach ring, load frame
        self.cable_len = float(cable_len)
        self.drone_mass = float(drone_mass)
        self.load_mass = float(load_mass)
        self.g = float(g)
        self.p = params or DissipativeParams()
        # nominal down-and-inward cable directions at the design elevation, one per
        # attach point (world == load frame at zero yaw; re-rotated by load yaw in step).
        self._nom_dir = nominal_cable_dirs(self.rho, self.p.elev_deg)
        # robot node state (world frame); seeded at handover from measured positions.
        self.q = np.zeros((self.n, 3))
        self.qd = np.zeros((self.n, 3))
        self.attached = [True] * self.n
        self._seeded = False

    # ── setup / topology ──────────────────────────────────────────────────
    def seed(self, drone_positions):
        """Bumpless handover: place every node at its measured drone position with
        zero velocity, so the first step() makes no jump (analogous to priming the
        OCP warm start on the live config)."""
        for i in range(self.n):
            self.q[i] = np.asarray(drone_positions[i], float)
        self.qd[:] = 0.0
        self._seeded = True

    def detach(self, k):
        """Drop node k from the network: its springs and damping edges vanish and the
        currently-attached count n' falls, so the remaining nodes re-settle and their
        tension feedforward rises. Idempotent."""
        if 0 <= k < self.n:
            self.attached[k] = False

    def n_attached(self):
        return sum(1 for a in self.attached if a)

    def _ring_neighbours(self, i):
        """The two azimuth neighbours of node i among the CURRENTLY attached nodes
        (the damping ring). Skips detached nodes so the ring stays closed as members
        leave."""
        att = [j for j in range(self.n) if self.attached[j] and j != i]
        if not att:
            return []
        # nodes are placed at azimuth 2*pi*j/n, so index order is azimuth order.
        order = sorted(att + [i])
        pos = order.index(i)
        return [order[(pos - 1) % len(order)], order[(pos + 1) % len(order)]]

    # ── network evolution ──────────────────────────────────────────────────
    def step(self, load_pos, load_quat, load_vel, p_des_load, dt):
        """Advance the network one control tick of length dt (integrated in substeps).
        load_pos/quat/vel: measured load pose+twist (the pinned payload node).
        p_des_load: desired load position [x,y,z] (hover+lift, or trajectory) -- the
        formation anchors ride on this so the whole taut formation translates with it.
        Returns nothing; read references via reference(i)."""
        load_pos = np.asarray(load_pos, float)
        load_vel = np.asarray(load_vel, float)
        p_des = np.asarray(p_des_load, float)
        R = quat_to_rot_np(load_quat)
        # DESIRED-frame geometry. Both the cable-length spring and the station anchor
        # ride on the desired load pose, not the measured (grounded) one -- otherwise
        # the cable spring pins the node to a sphere around the grounded load while the
        # anchor climbs, dragging the node to the TOP of that sphere (cable direction
        # goes vertical) so the drones cluster in over the load. In the desired frame
        # the node settles up-and-outward at the design elevation and the formation
        # simply translates up as the lift target rises. Measured load feeds back only
        # through the velocity damping below (and physically through the rigid rod).
        attach = [p_des + R @ self.rho[i] for i in range(self.n)]
        # anchors: desired attach point + cable_len up-and-outward (−nominal dir).
        anchor = [p_des + R @ (self.rho[i] - self.cable_len * self._nom_dir[i])
                  for i in range(self.n)]

        h = dt / max(self.p.substeps, 1)
        for _ in range(max(self.p.substeps, 1)):
            acc = np.zeros((self.n, 3))
            for i in range(self.n):
                if not self.attached[i]:
                    continue
                # station spring toward the formation anchor
                f = self.p.k_station * (anchor[i] - self.q[i])
                # cable-length spring: pull the node to exactly cable_len from attach
                dvec = self.q[i] - attach[i]
                dist = float(np.linalg.norm(dvec))
                if dist > 1e-9:
                    u = dvec / dist
                    f += -self.p.k_cable * (dist - self.cable_len) * u
                # Laplacian damping toward the (pinned) payload velocity ...
                f += -self.p.c * (self.qd[i] - load_vel)
                # ... and toward the ring neighbours' velocity (inter-robot dissipation)
                for j in self._ring_neighbours(i):
                    f += -self.p.c_ring * (self.qd[i] - self.qd[j])
                acc[i] = f / self.p.node_mass
            # semi-implicit Euler (update velocity then position) -- stable for stiff
            # springs at these coefficients.
            for i in range(self.n):
                if not self.attached[i]:
                    continue
                self.qd[i] += h * acc[i]
                self.q[i] += h * self.qd[i]

    # ── reference extraction ────────────────────────────────────────────────
    def reference(self, i, load_pos, load_quat, p_des_load, taut_gate=1.0):
        """Per-drone reference derived from node i, as (p_ref, v_ref, a_ff, a_cable),
        each a length-3 numpy array -- the tuple _publish_ref expects.

        The cable DIRECTION u comes from the network node relative to the MEASURED
        attach point (formation shaping + dissipation), but the reference is BASED at
        the DESIRED load attach point (p_des_load). Anchoring p_ref to the measured
        (grounded) load could never command a lift -- the drone would just slide around
        a fixed sphere while the load stays put. Basing it on the desired load makes the
        rising lift target actually raise the drone target; the rigid rod then drags the
        load up. At hover p_des == measured, so hold/detach behaviour is unchanged.
        a_cable is the analytic hover tension shared over the currently-attached drones
        (rises as drones detach), gated to 0 while the cable is slack (taut_gate)."""
        p_des_load = np.asarray(p_des_load, float)
        R = quat_to_rot_np(load_quat)
        # DESIRED-frame direction + base (consistent with step()): u points up-and-out
        # at the design elevation, so the drone is commanded up-and-outward from the
        # desired load -- never straight up over it. -u (down-and-in) is the cable pull
        # the tracker feeds forward, which tilts the drone OUTWARD to hold it.
        attach_des = p_des_load + R @ self.rho[i]
        dvec = self.q[i] - attach_des
        dist = float(np.linalg.norm(dvec))
        u = dvec / dist if dist > 1e-9 else np.array([0.0, 0.0, 1.0])
        p_ref = attach_des + self.cable_len * u
        v_ref = self.qd[i].copy()
        # analytic per-drone tension shared over the attached set: t = m_load g /
        # (n' sin phi). sin phi from the cable elevation (u points from attach up to
        # the drone, so u_z is sin phi).
        n_att = max(self.n_attached(), 1)
        sin_phi = float(np.clip(u[2], 0.05, 1.0))
        t_i = self.load_mass * self.g / (n_att * sin_phi)
        # a_cable = t*s/m: the cable pulls the drone toward the attach point (inward
        # and down), i.e. along -u. Gated by tautness.
        a_cable = taut_gate * (t_i / self.drone_mass) * (-u)
        # LOADED-hover specific-thrust feedforward: counter gravity AND the cable pull,
        # so the throttle+attitude FF is the equilibrium (up-and-out, magnitude > g).
        # A bare [0,0,g] FF only supports the drone's own weight -- the whole load share
        # then has to come from the tracker's feedback, which lifts weakly and laggily.
        a_ff = np.array([0.0, 0.0, self.g]) - a_cable
        return p_ref, v_ref, a_ff, a_cable

    def fly_away_reference(self, i, clearance=0.6):
        """Reference for a just-DETACHED drone: rise straight up from its current node
        position by `clearance` metres and hold, level attitude, NO cable term
        (a_cable=0 is the free-flight dynamics the tracker already handles). The seam
        where a collaborator's controller can take over."""
        p_ref = self.q[i].copy()
        p_ref[2] += clearance
        v_ref = np.zeros(3)
        a_ff = np.array([0.0, 0.0, self.g])
        a_cable = np.zeros(3)
        return p_ref, v_ref, a_ff, a_cable


# ─────────────────────────────────────────────────────────────────────────────
# Standalone self-test: settle-from-perturbation, tension symmetry, detach re-settle.
# Mirrors the __main__ checks in controller_quad_load/dynamics.py. Run:
#     python3 -m controller_load_mpc.dissipative_network
# (or execute this file directly with the package importable).
# ─────────────────────────────────────────────────────────────────────────────
def _self_test():
    g = 9.81
    cable_len = 0.5
    load_mass = 0.4
    drone_mass = 0.6
    n = 4
    rho = attach_points(n, 0.08, 0.025)
    net = DissipativeNetwork(n, rho, cable_len, drone_mass, load_mass, g)

    # measured load: level, at the origin, still.
    load_pos = np.array([0.0, 0.0, 0.6])
    load_quat = np.array([1.0, 0.0, 0.0, 0.0])
    load_vel = np.zeros(3)
    p_des = load_pos.copy()

    # seed nodes at a PERTURBED formation (all shoved +x by 0.15 m) and check the
    # network pulls them back to a symmetric taut equilibrium.
    nom = nominal_cable_dirs(rho, 45.0)
    seed = [load_pos - cable_len * nom[i] + np.array([0.15, 0.0, 0.0])
            for i in range(n)]
    net.seed(seed)

    dt = 0.1
    for _ in range(300):                       # 30 s
        net.step(load_pos, load_quat, load_vel, p_des, dt)
    # every node should sit ~cable_len from its attach point, and the four tensions
    # should be near-equal (symmetric hold).
    tens, dists = [], []
    for i in range(n):
        R = quat_to_rot_np(load_quat)
        attach = load_pos + R @ rho[i]
        d = float(np.linalg.norm(net.q[i] - attach))
        dists.append(d)
        _, _, _, ac = net.reference(i, load_pos, load_quat, p_des)
        tens.append(float(np.linalg.norm(ac)) * drone_mass)
    dists = np.array(dists); tens = np.array(tens)
    assert np.all(np.abs(dists - cable_len) < 0.05), \
        f"nodes not at cable_len after settle: {dists}"
    assert (tens.max() - tens.min()) < 0.15 * tens.mean(), \
        f"tensions not symmetric after settle: {tens}"
    assert np.all(np.isfinite(net.q)), "network diverged (nan/inf)"
    exp4 = load_mass * g / (4 * np.sin(np.radians(45)))
    print(f"[self-test] n=4 settle OK: dists={np.round(dists,3)} "
          f"tension/drone={np.round(tens,3)} N (expect ~{exp4:.2f})")

    # LIFT: with the desired load raised and the network re-settled, the drone
    # reference must rise ~1:1 AND stay up-and-outward (~45deg), not collapse
    # vertically over the load. Bug 1 was p_ref anchored to the MEASURED grounded
    # load (lift target never reached the drones); bug 2 was the cable direction
    # migrating vertical (drones commanded straight up -> tilting inward).
    p_hold, _, _, _ = net.reference(0, load_pos, load_quat, load_pos)   # settled @0.6
    net_hi = DissipativeNetwork(n, rho, cable_len, drone_mass, load_mass, g)
    hi = load_pos + np.array([0.0, 0.0, 0.3])
    net_hi.seed([hi - cable_len * nom[i] for i in range(n)])
    for _ in range(200):
        net_hi.step(load_pos, load_quat, load_vel, hi, dt)
    p_lift, _, _, _ = net_hi.reference(0, load_pos, load_quat, hi)
    u_lift = (p_lift - (hi + net_hi.rho[0])) / cable_len
    elev = np.degrees(np.arcsin(np.clip(u_lift[2], -1.0, 1.0)))
    assert p_lift[2] - p_hold[2] > 0.25, \
        f"reference did not track the lift target: {p_hold[2]:.3f} -> {p_lift[2]:.3f}"
    assert 30.0 < elev < 60.0, \
        f"cable not ~45deg up-and-out (inward collapse?): elev={elev:.0f}deg"
    print(f"[self-test] lift tracking OK: p_des +0.3 m -> p_ref z "
          f"{p_hold[2]:.3f} -> {p_lift[2]:.3f}, cable elev {elev:.0f}deg")

    # analytic redistribution: detach one, tension per remaining drone must rise 4->3.
    t4 = tens.mean()
    net.detach(2)
    for _ in range(300):
        net.step(load_pos, load_quat, load_vel, p_des, dt)
    tens3 = []
    for i in range(n):
        if not net.attached[i]:
            continue
        _, _, _, ac = net.reference(i, load_pos, load_quat, p_des)
        tens3.append(float(np.linalg.norm(ac)) * drone_mass)
    tens3 = np.array(tens3)
    assert np.all(np.isfinite(net.q)), "network diverged after detach"
    assert tens3.mean() > t4, \
        f"tension per drone did not rise after 4->3: {t4:.3f} -> {tens3.mean():.3f}"
    exp3 = load_mass * g / (3 * np.sin(np.radians(45)))
    print(f"[self-test] 4->3 detach OK: tension/drone {t4:.3f} -> {tens3.mean():.3f} N "
          f"(expect ~{exp3:.2f}), remaining nodes finite & re-settled")

    # detached node fly-away reference: no cable term, rises.
    p_ref, _, _, ac = net.fly_away_reference(2)
    assert np.allclose(ac, 0.0), "fly-away reference must have a_cable=0"
    print("[self-test] fly-away reference OK (a_cable=0)")
    print("[self-test] ALL PASSED")


if __name__ == '__main__':
    _self_test()
