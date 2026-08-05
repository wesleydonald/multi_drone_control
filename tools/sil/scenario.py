"""
tools/sil/scenario.py
---------------------
A SIL scenario: which real launch to bring up, what the plant starts as, what happens
when, and what the run has to show to pass.

One scenario = one YAML file in configs/sil/. The launch args live in the scenario, but
every CONTROLLER parameter still comes from the launch file itself -- the scenario only
overrides launch ARGUMENTS, exactly as a human typing `ros2 launch ... foo:=bar` would.
That keeps the launch file the single authority (architecture principle 1) and means a
launch-file change lands in the bench automatically.

Geometry is stated twice on purpose -- once as launch args for the controllers, once for
the plant -- and `check_geometry()` fails the run at startup if the two disagree. That is
the same class of bug tools/check_geometry.py exists for, and it has cost this project a
week before.
"""
from dataclasses import dataclass, field

import numpy as np
import yaml


@dataclass
class Event:
    t: float                  # sim seconds after the bench becomes ready
    do: str                   # ARM | TAKEOFF | WELD | LAND | DISARM | DETACH
    arg: int | None = None    # drone index, for DETACH


@dataclass
class Acceptance:
    """What the run must show. Empty = no assertion, the run is exploratory."""
    drone: int = 3
    window_s: float = 20.0            # measured from the WELD event
    expect: str = 'stable'            # 'diverge' | 'stable'
    min_track_err_m: float = 0.5      # diverge: error must exceed this
    max_track_err_m: float = 0.15     # stable: error must stay under this
    min_cable_accel: float = 15.0     # diverge: peak |aCm| must exceed this
    max_cable_accel: float = 5.0      # stable: peak |aCm| must stay under this
    min_tilt_deg: float = 45.0        # diverge: payload tilt must exceed this
    max_tilt_deg: float = 40.0        # stable: payload tilt must stay under this


@dataclass
class Scenario:
    name: str
    description: str = ''
    launch_package: str = 'controller_quad_load'
    launch_file: str = 'three_attach_launch.py'
    launch_args: dict = field(default_factory=dict)

    n_tethered: int = 3
    n_total: int = 4

    cable_len: float = 0.5
    attach_radius: float = 0.08
    attach_z: float = 0.025
    load_mass: float = 0.4
    drone_mass: float = 0.6
    thrust_c: float = 88.6
    magnet_arm_len: float = 0.5

    load_z0: float = 0.45
    elev_deg: float = 45.0
    weld_x: float = 0.0
    weld_y: float = 0.0
    weld_z_offset: float = 0.05
    # Take-off stands under the tethered drones, at their spawn height.
    #
    # NOT cosmetic. Before TAKEOFF the trackers publish an armed-idle command (throttle
    # 0) and the solver's u_state is pinned to that applied idle, so throttle ramps up
    # from zero under its own u_dot bound. On the ground that is a correct soft start;
    # suspended in mid-air it is a free fall, and the first bench run duly collapsed the
    # formation (payload on the floor, tilt 163 deg) before the trackers had even
    # finished compiling. The elevated taut worlds this scenario represents put the
    # drones on stands for exactly this reason, and AIRBORNE_MARGIN /
    # TAKEOFF_SPOOL_FLOOR in controller_mpc.py are both written for that break-off.
    #
    # The newcomer never gets a stand: it is a free flyer that has already taken off and
    # flown its approach.
    stands: bool = True

    dt: float = 0.02
    substeps: int = 20
    duration_s: float = 60.0
    step_timeout_s: float = 0.5
    ready_timeout_s: float = 240.0

    events: list = field(default_factory=list)
    acceptance: Acceptance | None = None

    # ── loading ──────────────────────────────────────────────────────────────

    @staticmethod
    def from_yaml(path):
        with open(path) as fh:
            raw = yaml.safe_load(fh) or {}
        s = Scenario(name=raw.get('name', 'unnamed'))
        s.description = raw.get('description', '')
        lch = raw.get('launch', {})
        s.launch_package = lch.get('package', s.launch_package)
        s.launch_file = lch.get('file', s.launch_file)
        s.launch_args = dict(lch.get('args', {}))
        fleet = raw.get('fleet', {})
        s.n_tethered = int(fleet.get('n_tethered', s.n_tethered))
        s.n_total = int(fleet.get('n_total', s.n_tethered))
        for k, v in (raw.get('geometry') or {}).items():
            if not hasattr(s, k):
                raise ValueError(f"{path}: unknown geometry key '{k}'")
            setattr(s, k, float(v))
        init = raw.get('initial') or {}
        s.load_z0 = float(init.get('load_z', s.load_z0))
        s.elev_deg = float(init.get('elev_deg', s.elev_deg))
        s.weld_x = float(init.get('weld_x', s.weld_x))
        s.weld_y = float(init.get('weld_y', s.weld_y))
        s.weld_z_offset = float(init.get('weld_z_offset', s.weld_z_offset))
        s.stands = bool(init.get('stands', s.stands))
        tim = raw.get('timing') or {}
        s.dt = float(tim.get('dt', s.dt))
        s.substeps = int(tim.get('substeps', s.substeps))
        s.duration_s = float(tim.get('duration_s', s.duration_s))
        s.step_timeout_s = float(tim.get('step_timeout_s', s.step_timeout_s))
        s.ready_timeout_s = float(tim.get('ready_timeout_s', s.ready_timeout_s))
        s.events = [Event(float(e['t']), str(e['do']).upper(), e.get('arg'))
                    for e in (raw.get('events') or [])]
        if raw.get('acceptance'):
            s.acceptance = Acceptance(**raw['acceptance'])
        s.check_geometry()
        return s

    # ── validation ───────────────────────────────────────────────────────────

    def check_geometry(self):
        """Fail loudly at startup on plant/launch geometry disagreement (principle 5).

        A cable_len or load_mass mismatch makes every tension feedforward wrong and
        presents as 'the controller cannot fly', not as a config error."""
        pairs = [('cable_len', self.cable_len), ('load_mass', self.load_mass),
                 ('attach_x_offset', self.weld_x), ('attach_y_offset', self.weld_y)]
        bad = []
        for arg, mine in pairs:
            if arg in self.launch_args:
                theirs = float(self.launch_args[arg])
                if abs(theirs - mine) > 1e-9:
                    bad.append(f'{arg}: launch {theirs} vs plant {mine}')
        if int(self.launch_args.get('num_drones', self.n_tethered)) != self.n_tethered:
            bad.append(f"num_drones: launch {self.launch_args['num_drones']} vs "
                       f'plant n_tethered {self.n_tethered}')
        reserved = int(self.launch_args.get('reserved_attach', 0))
        if self.n_tethered + reserved != self.n_total:
            bad.append(f'n_tethered {self.n_tethered} + reserved_attach {reserved} '
                       f'!= n_total {self.n_total}')
        if bad:
            raise ValueError('scenario geometry disagrees with its launch args:\n  '
                             + '\n  '.join(bad))

    # ── derived quantities ───────────────────────────────────────────────────

    def attach_rho(self):
        """The n_tethered cable attach points in the payload body frame.

        Uses the same ring the controllers use -- controller_load_mpc.geometry
        .attach_points(n_tethered, ...). Note it is built for the TETHERED count, not
        n_total: the newcomer does not get a nominal ring point, it welds where its
        magnet lands (see weld_point)."""
        return [np.array([self.attach_radius * np.cos(2 * np.pi * k / self.n_tethered),
                          self.attach_radius * np.sin(2 * np.pi * k / self.n_tethered),
                          self.attach_z]) for k in range(self.n_tethered)]

    def initial_state(self):
        """(drone positions, load position) for a taut AIRBORNE start.

        The attach scenarios start airborne and taut rather than on the ground, for the
        same reason verify_dissipative does: the ground break-off is the centralized
        OCP's job and is a separate question from reconfiguration. Set start_taut:=true
        in the launch args to match, or the planner will look for a creep phase that
        this initial condition has already passed."""
        load = np.array([0.0, 0.0, self.load_z0])
        e = np.radians(self.elev_deg)
        pos = []
        for rho in self.attach_rho():
            az = rho[:2] / max(float(np.linalg.norm(rho[:2])), 1e-9)
            d = np.array([az[0] * np.cos(e), az[1] * np.cos(e), np.sin(e)])
            pos.append(load + rho + self.cable_len * d)
        # the newcomer hovers with its magnet tip on the weld target: the tip hangs
        # magnet_arm_len below the body, and the target is the payload centre plus the
        # off-centre weld offset plus attach_target_publisher's z_offset.
        for _ in range(self.n_tethered, self.n_total):
            pos.append(load + np.array([self.weld_x, self.weld_y,
                                        self.weld_z_offset + self.magnet_arm_len]))
        return pos, load

    def weld_point_body(self, plant, i):
        """Payload-frame attach point for the newcomer's weld, taken from where its
        magnet tip actually is at the weld instant -- which is what the Gazebo
        DetachableJoint does (it welds the tip link wherever it is)."""
        tip = plant.p[i] - np.array([0.0, 0.0, self.magnet_arm_len])
        return plant.RL.T @ (tip - plant.xL)

    def launch_argv(self):
        out = []
        for k, v in self.launch_args.items():
            if isinstance(v, bool):
                v = 'true' if v else 'false'
            out.append(f'{k}:={v}')
        return out
