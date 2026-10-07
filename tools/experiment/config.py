"""
tools/experiment/config.py — the experiment config object (THESIS_PLAN §9.1).

One YAML per experiment: world, launch + args, scripted SIM-TIME events, duration and
success criteria. Kept ROS-free and Gazebo-free so it can be unit-tested, which is the
only way the validation below is worth anything.

WHY THE VALIDATION IS AGGRESSIVE. §9.1 lists its requirements as "each earned by a past
failure", and the failures were not exotic: a launch arg that silently did not plumb
through, two planners publishing to one tracker, sweeps compared across unequal
durations. A config that is wrong in one of those ways still runs happily for a minute
and produces a directory full of numbers, which is worse than not running at all. So
anything checkable is checked before Gazebo is started.
"""
import os

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Events the runner knows how to fire. Kept explicit rather than "publish any topic":
# a typo'd topic name in a YAML is a run that looks fine and does nothing at the moment
# that mattered, and that has cost whole sessions here before.
EVENT_KINDS = {
    'ARM',        # /fleet/command "ARM"
    'TAKEOFF',    # /fleet/command "TAKEOFF"
    'LAND',       # /fleet/command "LAND"
    'ESTOP',      # /fleet/command "ESTOP": the manager's emergency path (/fleet/abort + direct
                  # disarm), the string fleet_manager_node.py parses; abort arms need forbid_abort false
    'DISARM',     # /fleet/command "DISARM": the operator's kill switch (panel/spacebar); latches the
                  # muxes like ESTOP (Wesley 2026-09-28) through the service disarm path
    'MAGNET',     # /magnet/command "ON"/"OFF"  -> starts the approach + weld
    'ATTACH',     # /fleet/attach <drone id>    -> tell the network directly
    'DETACH',     # /fleet/detach <drone id>
    'RELEASE',    # /drone_<id>/detach only: the joint lets go and the planner is NOT told
                  # (an unannounced detach, a magnet failing mid-mission; 7 Oct)
    'DROP',       # /fleet/drop "<arg>": ask the planner to drop the ring and land every drone, as a
                  # tracker's ring-tilt rule does (card 2026-10-08_drop_and_land)
    'WAIT_WELD',  # not published: block until /magnet/object_attached goes True
    'WAIT_LIFT',  # not published: block until the payload is above arg m (default 0.5), then
                  # shift every later event by the wait (a creep lift varies by ~10 s, R0553);
                  # arg 'z timeout_s' gives up after timeout_s (event LIFT_TIMEOUT)
    'WAIT_REWELD',  # not published: block until the magnet welds AGAIN after a release
                    # (a drone that starts welded, leaves and rejoins), then shift later events
    'LAUNCH',     # not published: start a second launch mid-run, arg "pkg file [k:=v ...]"
                  # (the partner's mission once the ring flies); stopped with the stack
    'HANDOFF',    # /join_planner/handoff_ready True: our tracker takes the partner's last
                  # descent (stands in for his ATTACH_READY when his mission is not run)
    'WAIT_PARTNER_RELEASE',  # not published: block until the dissipative node logs PARTNER
                             # RELEASE (/rosout), then shift later events
    'HANGER_RELEASE',  # std_msgs/Empty on /bench/hanger_{i}/detach for every drone; arg
                       # overrides the topic template
    'GZ_PUB',     # gz.msgs.Empty on the gz topic in arg, sent 3x (gz CLI one-shots drop):
                  # e.g. /pickup/detach, which the partner mission sends at its start
    'FLEET_HANDOVER',  # /fleet/handover True for 1.5 s: the elrs_mux (partner_m2) hands each
                       # drone to our tracker on its first flying command (M2 bench)
    'WRENCH',     # persistent force on a Gazebo link, arg "fx fy fz [scoped link]" (N; default
                  # lift_system::payload::body); it replaces the previous WRENCH, "0 0 0" removes
                  # it. Needs a world with the ApplyLinkWrench system (*_push.sdf)
    'WAIT_THRUST',     # not published: block until every drone's ELRSCommand on the arg
                       # template (default cmd_ready_topic) is armed above idle throttle,
                       # then shift later events (M2 bench: hangers release only under thrust)
}

def wrench_arg(arg):
    """WRENCH arg -> (force (fx, fy, fz) in N, scoped link name)."""
    parts = str(arg or '').split()
    if len(parts) not in (3, 4):
        raise ValueError(f'WRENCH needs "fx fy fz [link]", got {arg!r}')
    return tuple(float(v) for v in parts[:3]), (parts[3] if len(parts) == 4
                                                else 'lift_system::payload::body')


def wait_lift_arg(arg):
    """WAIT_LIFT arg -> (payload z threshold, timeout SIM s or None): 'z' or 'z timeout'.
    The timeout lets a run whose ring never reaches the height (the model-f1 twin held
    0.3-0.4 m under a 0.5 target) still fly the rest of its schedule and land."""
    if arg is None:
        return 0.5, None
    parts = str(arg).split()
    if not 1 <= len(parts) <= 2:
        raise ValueError(f"WAIT_LIFT arg {arg!r}: expected 'z' or 'z timeout_s'")
    z = float(parts[0])
    timeout = float(parts[1]) if len(parts) == 2 else None
    if timeout is not None and timeout <= 0:
        raise ValueError(f'WAIT_LIFT timeout must be > 0, got {timeout}')
    return z, timeout


# SIM seconds an ESTOP run must keep recording after /fleet/abort (drones staying down)
ESTOP_STAY_DOWN_S = 5.0

# The partner mission's pickup object, where three_attach_partner_attached.sdf puts it.
PARTNER_BALL_START = (1.2, -1.4, 0.08)


class Event:
    """One scripted event. `t` is SIM seconds after the run clock starts."""

    def __init__(self, t, do, arg=None):
        self.t = float(t)
        self.do = str(do).upper()
        self.arg = arg
        if self.do not in EVENT_KINDS:
            raise ValueError(
                f"unknown event '{self.do}'; known: {sorted(EVENT_KINDS)}")
        if self.t < 0:
            raise ValueError(f'event {self.do} has negative time {self.t}')
        # YAML 1.1 parses an unquoted ON as the BOOLEAN True (likewise OFF/NO/YES), so
        # `{do: MAGNET, arg: ON}` arrives here as True and would be published to
        # /magnet/command as the string "TRUE". The magnet would never switch on, the
        # approach would never weld, and the run would complete looking entirely normal.
        # Normalise it rather than relying on everyone remembering to quote it.
        if self.do == 'MAGNET':
            if isinstance(self.arg, bool):
                self.arg = 'ON' if self.arg else 'OFF'
            elif self.arg is None:
                self.arg = 'ON'
            else:
                self.arg = str(self.arg).upper()
            if self.arg not in ('ON', 'OFF'):
                raise ValueError(f"MAGNET arg must be ON or OFF, got {self.arg!r}")
        if self.do == 'LAUNCH' and len(str(self.arg or '').split()) < 2:
            raise ValueError('LAUNCH needs "package launch_file [k:=v ...]" as its arg')
        if self.do == 'WAIT_LIFT':
            wait_lift_arg(self.arg)
        if self.do == 'WAIT_THRUST' and self.arg is not None and '{i}' not in str(self.arg):
            raise ValueError(f"WAIT_THRUST arg {self.arg!r} has no '{{i}}' for the drone id")
        if self.do == 'WRENCH':
            wrench_arg(self.arg)
        if self.do in ('ATTACH', 'DETACH', 'RELEASE'):
            if self.arg is None:
                raise ValueError(f'{self.do} needs a drone id as its arg')
            self.arg = int(self.arg)

    def __repr__(self):
        a = f' {self.arg}' if self.arg is not None else ''
        return f'<{self.do}{a} @ {self.t:g}s>'


class Criteria:
    """Post-run pass/fail. Every field is optional; absent means "not asserted".

    Deliberately NOT the same object as the SIL bench's `Acceptance`. That one encodes
    a specific claim about one drone across a weld; this is a general "did the run do
    something legal" gate, and conflating them would make it tempting to quietly reuse
    SIL thresholds as if a Gazebo run had been held to them.
    """

    def __init__(self, **kw):
        self.max_payload_tilt_deg = kw.pop('max_payload_tilt_deg', None)
        self.max_drone_tilt_deg = kw.pop('max_drone_tilt_deg', None)
        self.max_track_err_m = kw.pop('max_track_err_m', None)
        self.min_payload_z = kw.pop('min_payload_z', None)
        # Peak, not floor. min_payload_z only says the load never fell through the
        # world; it is satisfied by a fleet that never took off at all. R0073 had all
        # three drones disarm on a pose timeout at ARM, sat on the ground for 75 s, and
        # scored PASS on every criterion it had. Require the load to have actually
        # risen, or the run is not evidence about flight.
        self.min_peak_payload_z = kw.pop('min_peak_payload_z', None)
        self.max_payload_z = kw.pop('max_payload_z', None)
        self.require_weld = bool(kw.pop('require_weld', False))
        self.forbid_abort = bool(kw.pop('forbid_abort', True))
        self.window_from = str(kw.pop('window_from', 'start')).lower()
        self.window_s = kw.pop('window_s', None)
        # Tethered hold (plan 2026-10, W6), scored from the planner's tick log by
        # metrics.tethered_run_metrics over the last hold_window_s before LAND.
        self.max_planner_solve_fail = kw.pop('max_planner_solve_fail', None)
        self.max_hold_z_err_m = kw.pop('max_hold_z_err_m', None)
        self.hold_window_s = float(kw.pop('hold_window_s', 20.0))
        self.max_heave_pp_m = kw.pop('max_heave_pp_m', None)
        self.max_payload_vz_mps = kw.pop('max_payload_vz_mps', None)
        self.min_carried_fraction = kw.pop('min_carried_fraction', None)
        self.max_ref_age_frac = kw.pop('max_ref_age_frac', None)
        self.max_hold_tilt_mean_deg = kw.pop('max_hold_tilt_mean_deg', None)
        self.max_z_bias_abs = kw.pop('max_z_bias_abs', None)     # height integral, over the hold
        # every check is computed and printed but none fails the run (a reproduction)
        self.report_only = bool(kw.pop('report_only', False))
        if kw:
            raise ValueError(f'unknown criteria keys: {sorted(kw)}')
        if self.window_from not in ('start', 'weld'):
            raise ValueError("criteria window_from must be 'start' or 'weld'")


class ExperimentConfig:
    def __init__(self, name):
        self.name = name
        self.description = ''
        self.world = 'old_worlds/three_attach.sdf'
        self.launch_package = 'bringup'
        self.launch_file = 'sim_control_launch.py'
        self.launch_args = {}
        # The SIM-INTERFACE launch, started FIRST and separately.
        #
        # This is not optional plumbing: sim_io_launch.py owns the /clock
        # bridge, the pose bridges and the mocap emulators. Start only the control
        # launch and every node sits on use_sim_time with no clock, nothing publishes,
        # and the run times out having produced zero rows -- which is exactly what the
        # first Gazebo run through this tool did. The GUI is suppressed with rviz:=false.
        self.io_launch_file = 'sim_io_launch.py'
        self.io_launch_args = {}
        self.num_drones = 3
        self.n_total = 4
        # Timing. All SIM seconds -- see the note in run_experiment.py about why no
        # wall-clock deadline is allowed to end a run.
        self.duration_s = 60.0
        self.ready_timeout_s = 300.0     # WALL seconds: acados compile happens here
        self.launch_expect_topics = []   # topics a LAUNCH event's nodes must publish
        self.dds_udp_only = False        # Fast DDS UDP-only profile for every process
        self.bg_param_dump = True        # background read-back of the non-evidence nodes
        self.gz_record = False           # gz --record-path <run>/gz_record (GUI playback)
        self.stop_after_landed_s = 0.0   # end the run this long after /fleet/landed (0 = off)
        self.stop_launch_on_handoff = False  # SIGINT every LAUNCH-event launch at the partner's handoff
        self.pose_timeout_s = 2.0        # SIM seconds without mocap = dead run
        self.settle_s = 3.0              # sim time to let /clock stabilise before t=0
        # SIM seconds to keep recording after /fleet/abort, then stop. An abort disarms
        # every drone, so everything past it is a fleet lying on the floor -- in the 45
        # deg runs that was 34 s of sim, ~115 s of WALL time at Gazebo's 0.29x realtime
        # factor, per run. The grace still captures the fall, which is worth having.
        # 0 disables early stopping and runs the full duration_s.
        self.stop_after_abort_s = 12.0
        self.events = []
        self.criteria = None
        # True when the launch is DELIBERATELY told a different mass/geometry than the
        # world (a robustness arm); otherwise run_experiment refuses a mismatch.
        self.mis_seed = False
        # {key: reason}: geometry values a config deliberately flies unlike its world
        self.geometry_exempt = {}
        # the topic whose first message says drone i's tracker is commanding
        self.cmd_ready_topic = '/drone_{i}/ELRSCommand'
        # expected start of the partner's pickup object (/model/payload_model/pose[1]);
        # None = not logged. Defaults on when the partner mission is launched.
        self.ball_start = None

    # ── loading ──────────────────────────────────────────────────────────────

    @staticmethod
    def from_yaml(path):
        with open(path) as fh:
            raw = yaml.safe_load(fh) or {}
        c = ExperimentConfig(raw.get('name') or
                             os.path.splitext(os.path.basename(path))[0])
        c.description = raw.get('description', '')
        c.mis_seed = bool(raw.get('mis_seed', False))
        c.geometry_exempt = dict(raw.get('geometry_exempt') or {})
        c.world = raw.get('world', c.world)
        lch = raw.get('launch') or {}
        c.launch_package = lch.get('package', c.launch_package)
        c.launch_file = lch.get('file', c.launch_file)
        c.launch_args = dict(lch.get('args') or {})
        io = raw.get('io_launch') or {}
        c.io_launch_file = io.get('file', c.io_launch_file)
        c.io_launch_args = dict(io.get('args') or {})
        fleet = raw.get('fleet') or {}
        c.num_drones = int(fleet.get('num_drones', c.num_drones))
        c.n_total = int(fleet.get('n_total', c.num_drones))
        tim = raw.get('timing') or {}
        c.launch_expect_topics = list((raw.get('timing') or {}).get('launch_expect_topics') or [])
        c.dds_udp_only = bool((raw.get('timing') or {}).get('dds_udp_only', False))
        c.bg_param_dump = bool((raw.get('timing') or {}).get('bg_param_dump', True))
        c.gz_record = bool((raw.get('timing') or {}).get('gz_record', False))
        c.stop_after_landed_s = float((raw.get('timing') or {}).get('stop_after_landed_s', 0.0))
        c.stop_launch_on_handoff = bool((raw.get('timing') or {}).get('stop_launch_on_handoff', False))
        for k in ('duration_s', 'ready_timeout_s', 'pose_timeout_s', 'settle_s',
                  'stop_after_abort_s'):
            if k in tim:
                setattr(c, k, float(tim[k]))
        c.events = [Event(e['t'], e['do'], e.get('arg'))
                    for e in (raw.get('events') or [])]
        c.cmd_ready_topic = str(raw.get('cmd_ready_topic', c.cmd_ready_topic))
        partner = any(e.do == 'LAUNCH' and 'partner_mission' in str(e.arg) for e in c.events)
        ball = raw.get('ball_start', PARTNER_BALL_START if partner else None)
        c.ball_start = None if ball is None else tuple(float(v) for v in ball)
        if raw.get('criteria'):
            c.criteria = Criteria(**raw['criteria'])
        c.validate()
        return c

    # ── validation ───────────────────────────────────────────────────────────

    def validate(self):
        bad = []
        # num_drones must agree with the launch arg, or the runner watches the wrong
        # number of mocap topics and its pose watchdog never fires.
        if 'num_drones' in self.launch_args:
            if int(self.launch_args['num_drones']) != self.num_drones:
                bad.append(
                    f"fleet.num_drones={self.num_drones} but launch arg "
                    f"num_drones={self.launch_args['num_drones']}")
        reserved = int(self.launch_args.get('reserved_attach', 0))
        if self.n_total != self.num_drones + reserved:
            bad.append(
                f'fleet.n_total={self.n_total} but num_drones + reserved_attach = '
                f'{self.num_drones} + {reserved} = {self.num_drones + reserved}')
        # Events must fit inside the run, or a config silently never welds.
        for e in self.events:
            if e.t > self.duration_s:
                bad.append(f'event {e!r} is after duration_s={self.duration_s}')
        order = [e.do for e in sorted(self.events, key=lambda e: e.t)]
        if 'TAKEOFF' in order and 'ARM' in order:
            if order.index('ARM') > order.index('TAKEOFF'):
                bad.append('TAKEOFF is scheduled before ARM')
        if 'MAGNET' in order and 'TAKEOFF' not in order:
            bad.append('MAGNET (approach + weld) with no TAKEOFF: nothing is flying')
        if '{i}' not in self.cmd_ready_topic:
            bad.append(f"cmd_ready_topic {self.cmd_ready_topic!r} has no '{{i}}' for the drone id")
        if self.ball_start is not None and len(self.ball_start) != 3:
            bad.append(f'ball_start must be [x, y, z], got {list(self.ball_start)}')
        if any(e.do in ('ESTOP', 'DISARM') for e in self.events):
            # an ESTOP run aborts by design: forbid_abort would fail it, and an early stop
            # shorter than the stay-down window would cut the evidence (card 2026-09-28 F4)
            if self.criteria is None or self.criteria.forbid_abort:
                bad.append('ESTOP/DISARM scheduled but criteria.forbid_abort is not false')
            if 0.0 < self.stop_after_abort_s < ESTOP_STAY_DOWN_S:
                bad.append(f'ESTOP scheduled but timing.stop_after_abort_s='
                           f'{self.stop_after_abort_s:g} < {ESTOP_STAY_DOWN_S:g} s stay-down window')
        if self.criteria and self.criteria.window_from == 'weld':
            if not any(e.do in ('MAGNET', 'ATTACH', 'WAIT_WELD') for e in self.events):
                bad.append("criteria window_from='weld' but no weld event is scheduled")
        if bad:
            raise ValueError(f'{self.name}: invalid experiment config:\n  '
                             + '\n  '.join(bad))
        return True

    # ── launch plumbing ──────────────────────────────────────────────────────

    @staticmethod
    def _argv(d):
        """`name:=value` list for `ros2 launch`, booleans lowercased.

        Python's str(True) is 'True' and ROS launch wants 'true'; passing the former
        makes a bool arg fall through to its default with no error, which is exactly
        how a run silently flies a configuration nobody chose."""
        out = []
        for k, v in d.items():
            if isinstance(v, bool):
                v = 'true' if v else 'false'
            out.append(f'{k}:={v}')
        return out

    @property
    def legacy(self):
        """A world under simulation_assets/old_worlds/ or tejen/ flies the legacy sim profile
        (sim_legacy.yaml, linear plant); every other world is a rig twin."""
        w = os.path.normpath(self.world)
        if os.path.isabs(w):
            w = os.path.relpath(w, os.path.join(REPO, 'simulation_assets'))
        return w.split(os.sep)[0] in ('old_worlds', 'tejen')

    def control_args(self):
        """launch.args as flown: legacy:=true goes along for a legacy world unless typed."""
        args = dict(self.launch_args)
        if self.legacy and self.launch_file == 'sim_control_launch.py':
            args.setdefault('legacy', True)
        return args

    def io_args(self):
        """io_launch.args as flown (without the fleet-derived ones of io_launch_argv)."""
        args = dict(self.io_launch_args)
        if self.legacy and self.io_launch_file == 'sim_io_launch.py':
            args.setdefault('sim_thrust_map', 'linear')
        return args

    def launch_argv(self, params_path=None):
        """Control-launch args; knobs that are not launch arguments go to params_path."""
        import launch_args
        return launch_args.argv(self.launch_file, self.control_args(), params_path)

    def io_launch_argv(self):
        """Args for the sim-interface launch, with the defaults it needs derived from
        the experiment rather than restated in every YAML: the drone count must match,
        `attach` must be on whenever a newcomer exists (it owns that drone's
        visualisation + mocap), and the GUI is always off for a batch run."""
        args = {'num_drones': self.num_drones,
                'attach': self.n_total > self.num_drones,
                'rviz': False}
        args.update(self.io_args())
        return self._argv(args)

    def summary(self):
        ev = ', '.join(f'{e.do}@{e.t:g}' for e in sorted(self.events, key=lambda e: e.t))
        return (f'{self.name}: world={self.world} launch={self.launch_file} '
                f'drones={self.num_drones}(+{self.n_total - self.num_drones}) '
                f'duration={self.duration_s:g}s sim\n    events: {ev or "(none)"}')
