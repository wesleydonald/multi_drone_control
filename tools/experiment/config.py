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

# Events the runner knows how to fire. Kept explicit rather than "publish any topic":
# a typo'd topic name in a YAML is a run that looks fine and does nothing at the moment
# that mattered, and that has cost whole sessions here before.
EVENT_KINDS = {
    'ARM',        # /fleet/command "ARM"
    'TAKEOFF',    # /fleet/command "TAKEOFF"
    'LAND',       # /fleet/command "LAND"
    'MAGNET',     # /magnet/command "ON"/"OFF"  -> starts the approach + weld
    'ATTACH',     # /fleet/attach <drone id>    -> tell the network directly
    'DETACH',     # /fleet/detach <drone id>
    'WAIT_WELD',  # not published: block until /magnet/object_attached goes True
}


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
        if self.do in ('ATTACH', 'DETACH'):
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
        if kw:
            raise ValueError(f'unknown criteria keys: {sorted(kw)}')
        if self.window_from not in ('start', 'weld'):
            raise ValueError("criteria window_from must be 'start' or 'weld'")


class ExperimentConfig:
    def __init__(self, name):
        self.name = name
        self.description = ''
        self.world = 'three_attach.sdf'
        self.launch_package = 'controller_quad_load'
        self.launch_file = 'three_attach_launch.py'
        self.launch_args = {}
        # The SIM-INTERFACE launch, started FIRST and separately.
        #
        # This is not optional plumbing: rviz_quad_load_launch.py owns the /clock
        # bridge, the pose bridges and the mocap emulators. Start only the control
        # launch and every node sits on use_sim_time with no clock, nothing publishes,
        # and the run times out having produced zero rows -- which is exactly what the
        # first Gazebo run through this tool did. The GUI is suppressed with rviz:=false.
        self.io_launch_file = 'rviz_quad_load_launch.py'
        self.io_launch_args = {}
        self.num_drones = 3
        self.n_total = 4
        # Timing. All SIM seconds -- see the note in run_experiment.py about why no
        # wall-clock deadline is allowed to end a run.
        self.duration_s = 60.0
        self.ready_timeout_s = 300.0     # WALL seconds: acados compile happens here
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

    # ── loading ──────────────────────────────────────────────────────────────

    @staticmethod
    def from_yaml(path):
        with open(path) as fh:
            raw = yaml.safe_load(fh) or {}
        c = ExperimentConfig(raw.get('name') or
                             os.path.splitext(os.path.basename(path))[0])
        c.description = raw.get('description', '')
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
        for k in ('duration_s', 'ready_timeout_s', 'pose_timeout_s', 'settle_s',
                  'stop_after_abort_s'):
            if k in tim:
                setattr(c, k, float(tim[k]))
        c.events = [Event(e['t'], e['do'], e.get('arg'))
                    for e in (raw.get('events') or [])]
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

    def launch_argv(self):
        return self._argv(self.launch_args)

    def io_launch_argv(self):
        """Args for the sim-interface launch, with the defaults it needs derived from
        the experiment rather than restated in every YAML: the drone count must match,
        `attach` must be on whenever a newcomer exists (it owns that drone's
        visualisation + mocap), and the GUI is always off for a batch run."""
        args = {'num_drones': self.num_drones,
                'attach': self.n_total > self.num_drones,
                'rviz': False}
        args.update(self.io_launch_args)
        return self._argv(args)

    def summary(self):
        ev = ', '.join(f'{e.do}@{e.t:g}' for e in sorted(self.events, key=lambda e: e.t))
        return (f'{self.name}: world={self.world} launch={self.launch_file} '
                f'drones={self.num_drones}(+{self.n_total - self.num_drones}) '
                f'duration={self.duration_s:g}s sim\n    events: {ev or "(none)"}')
