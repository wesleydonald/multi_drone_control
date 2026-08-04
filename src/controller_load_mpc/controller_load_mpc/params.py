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
ATTACH_RADIUS = 0.08
ATTACH_Z      = 0.025          # attach height above load CoG, load frame
LOAD_MASS     = 0.4
TARGET_Z      = 0.6            # load hover height
LIFT_RAMP_VEL = 0.05           # m/s load lift rate after handover
LAND_VEL      = 0.20           # m/s descent rate, faster than the gentle lift


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
        # Must match the payload mass in the world SDF: cable tension is sized
        # off this, so a mismatch scales every drone's tension FF.
        self.load_mass = float(p('load_mass', LOAD_MASS).value)
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
        # Cables already taut at spawn (elevated world): skip the creep phase.
        self.start_taut = bool(p('start_taut', False).value)
        # Load target rises from the handover height at lift_ramp_vel to
        # target_z. Params, so lift_ramp_vel:=0.0 gives a hold test.
        self.target_z = float(p('target_z', TARGET_Z).value)
        self.lift_ramp_vel = float(p('lift_ramp_vel', LIFT_RAMP_VEL).value)
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
            f'start_taut={self.start_taut} target_z={self.target_z:.3f} '
            f'lift_ramp_vel={self.lift_ramp_vel:.3f} load_traj={self.load_traj} '
            f'traj_speed={self.traj_speed:.3f} traj_distance={self.traj_distance:.3f} '
            f'traj_radius={self.traj_radius:.3f}')
