from .geometry import parse_azimuths_deg
"""
params.py
---------
ROS parameter declarations for the load planner, factored out of the node so
__init__ reads as wiring, not a wall of configuration. PlannerConfig declares every
planner ROS parameter (with its per-world meaning) and resolves it; the node copies
the results onto itself, so the rest of the planner keeps reading plain self.<name>.
"""

# Defaults, overridable per world via ROS params. Must match the world SDF.
N_DRONES      = 3
CABLE_LEN     = 0.5            # matches every world in simulation_assets/ (rod
                               # cylinder length). Was 0.6 — the cable length of
                               # three_soft.sdf, a world that no longer exists — so
                               # any node run WITHOUT a launch silently got a 20%
                               # cable-length error. Verified by
                               # tools/check_geometry.py, which is in the gate.
ATTACH_RADIUS = 0.25   # magnet-plate ring of the 500 mm M2A ring payload (2026-09-23)
ATTACH_Z      = 0.025          # attach height above load CoG, load frame
ATTACH_AZIMUTHS_DEG = ''       # '' = even ring; e.g. '0,90,180' = 3/12/9 o'clock (rig)
LOAD_MASS     = 0.86           # official rig mass (Wesley, 2026-09-23)
DRONE_MASS    = 0.64           # sim x3 airframe over ALL its links (base 0.6 + four 0.01
                               # rotors, simulation_assets/models/x3_drone*.sdf); was 0.6
                               # until 2026-09-24 (both drones measured 3.6 % under the
                               # nominal gain). The rig flies drone_mass:=<weighed with pack>.
# M2A ring (outer 280 mm, inner 220 mm, 30 mm thick; generate_rigid_world.py) about its CoG:
# Ixx = m(3(R^2+r^2)+h^2)/12, Izz = m(R^2+r^2)/2.
# The planner's OCP bakes these into the compiled solver (planner_solver._ocp_signature)
# and tools/check_geometry.py checks them against the world SDF.
LOAD_IXX      = 2.733e-02      # = load_inertia(LOAD_MASS); literals so tools/check_geometry.py can read them
LOAD_IZZ      = 5.452e-02
RING_IXX_PER_KG = LOAD_IXX / LOAD_MASS   # the same annulus per kg, so a load_mass arg carries its inertia
RING_IZZ_PER_KG = LOAD_IZZ / LOAD_MASS


def load_inertia(mass):
    """[Ixx, Iyy, Izz] of the M2A ring at `mass` kg (tools/sil/plant.py uses the same
    per-kg values)."""
    return [RING_IXX_PER_KG * mass, RING_IXX_PER_KG * mass, RING_IZZ_PER_KG * mass]
TARGET_Z      = 0.6            # load hover height
LIFT_RAMP_VEL = 0.05           # m/s load lift rate after handover
LAND_VEL      = 0.20           # m/s descent rate, faster than the gentle lift
# Rod pivot in the DRONE BODY frame (m). The OCP's drone is the rod end; the rig's rod
# hangs from a joint 4 cm below the drone centre ([0, 0, -0.04]), the sim's from the centre.
PIVOT_OFFSET  = (0.0, 0.0, 0.0)
SOLVE_BUDGET_S = 0.0           # s wall cap on a solve's SQP iterations; 0 = off (sim), rig 0.06
MAX_SHIFT_PUBLISHES = 3        # failed solves bridged by the shifted last horizon, then stale


class PlannerConfig:
    """Declares and resolves every load-planner ROS parameter. Attributes mirror the
    node fields the rest of the planner reads; the node copies them onto itself."""

    def __init__(self, node):
        p = node.declare_parameter
        # Geometry and mode params, overriding the module defaults per world.
        # Must match the world SDF and the fleet manager's num_drones: the
        # planner sizes the attach ring and divides load tension by this, so a
        # mismatch mis-scales every drone's feedforward.
        self.n = int(p('num_drones', N_DRONES).value)
        self.cable_len = float(p('cable_len', CABLE_LEN).value)
        self.attach_radius = float(p('attach_radius', ATTACH_RADIUS).value)
        self.attach_z = float(p('attach_z', ATTACH_Z).value)
        # Where the rim attachments actually are (deg, load frame), or '' for the even
        # ring. Sized by num_drones; the world SDF must agree (tools/check_geometry.py).
        self.attach_azimuths = parse_azimuths_deg(
            str(p('attach_azimuths_deg', ATTACH_AZIMUTHS_DEG).value))
        # Must match the payload mass in the world SDF: cable tension is sized
        # off this, so a mismatch scales every drone's tension FF.
        self.load_mass = float(p('load_mass', LOAD_MASS).value)
        self.load_inertia = load_inertia(self.load_mass)
        # Per-drone mass (with pack): the OCP's tension -> drone-acceleration relation
        # and the static cable share the trackers' kt_trim uses both divide by it.
        self.drone_mass = float(p('drone_mass', DRONE_MASS).value)
        # Hand over on cable ELEVATION (deg) instead of cable length. A rigid rod
        # is always exactly cable_len, so the length gate reads 1.0 from spawn and
        # hands over at ~0 deg, where tension mg/(n sin_elev) is effectively
        # infinite. Set for ground-start rigid worlds; 45 matches the elevated
        # ones. 0 = off, use the length gate (correct for soft cables).
        self.handover_elev_deg = float(p('handover_elev_deg', 0.0).value)
        # Seconds to hold the latched config after handover before lifting.
        # Without it two transients land in the same cycle: the position
        # reference steps to the drones' actual pose (so the error the MPC was
        # fighting vanishes and it must unwind), and the lift starts pulling the
        # payload off the ground. Ground starts want ~2 s; 0 = off.
        self.handover_settle_s = float(p('handover_settle_s', 0.0).value)
        # creep sweep / rise rate before the handover (m/s); 0.10 since the creep was written
        self.creep_vel = float(p('creep_vel', 0.10).value)
        # Floor start: after the settle, ramp every rod's pull 0 -> 1 together over this
        # many seconds with the drones held, THEN start the height ramp (rig 2026-09-30:
        # the pull arriving as a step launched the ring and pulled magnets off).
        # 0 = the old 1 s ease during the ramp. Air starts (start_taut) never pretension.
        self.pretension_s = float(p('pretension_s', 3.0).value)
        # measure_rod_len trust band: rods within +-tol of the typed cable_len and within
        # spread of each other. The rig's drone-centre-to-plate reads 0.53-0.57 at every
        # hand-over against the creep's typed 0.47 (2026-09-30), so the rig widens both.
        self.rod_tol_frac = float(p('rod_tol_frac', 0.15).value)
        self.rod_spread_m = float(p('rod_spread_m', 0.03).value)
        # Cables already taut at spawn (elevated world): skip the creep phase.
        self.start_taut = bool(p('start_taut', False).value)
        # Load target rises from the handover height at lift_ramp_vel to
        # target_z. Params, so lift_ramp_vel:=0.0 gives a hold test.
        self.target_z = float(p('target_z', TARGET_Z).value)
        self.lift_ramp_vel = float(p('lift_ramp_vel', LIFT_RAMP_VEL).value)
        # Bounded load-height integral on the height target (card 2026-09-24_planner_offset):
        # 0 = off; 0.2 = tau 5 s. z_i_max bounds it (0.3 m since 4 Oct: an unknown 0.1 kg object
        # needs about 0.14 m at the cascade's 7.1 N/m; 0.15 left it at 92 % of the bound).
        self.z_ki = float(p('z_ki', 0.0).value)
        self.z_i_max = float(p('z_i_max', 0.3).value)
        # the integral only runs while the miss is under this (a bigger miss was taken to be a
        # transient); the rig's capped pull left 0.3-0.4 m misses it never touched (2026-09-30)
        self.z_i_gate = float(p('z_i_gate', 0.25).value)
        self.z_taut_gate = float(p('z_taut_gate', 0.99).value)   # ~0.9 on the rig with a typed rod length
        self.pivot_offset = [float(v) for v in p('pivot_offset', list(PIVOT_OFFSET)).value]
        if len(self.pivot_offset) != 3:
            raise ValueError(f'pivot_offset needs 3 values, got {self.pivot_offset}')
        # off in sim: a wall-clock cut makes the iteration count depend on host load
        self.solve_budget_s = float(p('solve_budget_s', SOLVE_BUDGET_S).value)
        # failed solves are bridged by the shifted last horizon for this many node periods
        # after the last good one, then nothing is published, so the trackers' reference
        # watchdog still trips (0..4 at 10 Hz, checked by HorizonFallback)
        self.max_shift_publishes = int(p('max_shift_publishes', MAX_SHIFT_PUBLISHES).value)
        # Auto slot assignment. OFF: OCP slot i is physical drone i, so the drones
        # must spawn in the nominal ring order (drone 0 at +x, CCW). ON: at the first
        # solve each physical drone is matched to the nearest nominal azimuth slot
        # around the load, so you can place the drones anywhere in the ring
        # (~cable_len out) in ANY order and the planner figures out the labelling. It
        # only relabels the drone<->slot I/O; the OCP is unchanged (the attach ring
        # is symmetric), so no recompile. Real-world default.
        self.auto_slot_assign = bool(p('auto_slot_assign', False).value)
        # Load reference trajectory. Once the lift tops out at target_z the load
        # reference translates laterally; the OCP tracks it via the reference builder.
        #   'hover'  no lateral motion, lift then hold.
        #   'line_x' continuous shuttle 0 -> traj_distance -> 0 until LAND.
        #            Sinusoidal, so traj_speed is the peak (mid-stroke) speed.
        #   'circle' horizontal circle of traj_radius at traj_speed.
        # Keep traj_speed slow: lateral accel is not fed forward, so fast motion
        # would need cable tilt this open-loop translation cannot model.
        self.load_traj = str(p('load_traj', 'hover').value)
        self.traj_speed = float(p('traj_speed', 0.1).value)      # m/s lateral
        self.traj_distance = float(p('traj_distance', 1.0).value)  # m (line_x)
        self.traj_radius = float(p('traj_radius', 0.5).value)    # m (circle)
        # Separate from lift_ramp_vel so a slow takeoff doesn't force a slow land.
        self.land_vel = float(p('land_vel', LAND_VEL).value)

    def log(self, logger):
        logger.info(
            f'[planner] geometry: n={self.n} cable_len={self.cable_len:.3f} '
            f'attach_radius={self.attach_radius:.3f} attach_z={self.attach_z:.3f} '
            f'load_mass={self.load_mass:.3f} '
            f'pivot_offset={[round(v, 3) for v in self.pivot_offset]} '
            f'start_taut={self.start_taut} target_z={self.target_z:.3f} '
            f'lift_ramp_vel={self.lift_ramp_vel:.3f} load_traj={self.load_traj} '
            f'traj_speed={self.traj_speed:.3f} traj_distance={self.traj_distance:.3f} '
            f'traj_radius={self.traj_radius:.3f}')
