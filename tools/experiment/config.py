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
import math
import re
import os

import yaml

from . import record as _record

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
    'ATTACH',     # /fleet/attach <drone id>    -> tell the network directly; a carrier's id (below
                  # num_drones) is a departed carrier's rejoin (rejoin_departed, card 2026-10-23_rejoin_departed)
    'DETACH',     # /fleet/detach <drone id>
    'RELEASE',    # /drone_<id>/detach only: the joint lets go and the planner is NOT told
                  # (an unannounced detach, a magnet failing mid-mission; 7 Oct)
    'DROP',       # /fleet/drop "<arg>": ask the planner to drop the ring and land every drone, as a
                  # tracker's ring-tilt rule does (card 2026-10-08_drop_and_land)
    'HIS_LAND',   # /join_planner/land_now True: his planner's own LAND, as the console's his LAND button
    'WAIT_WELD',  # not published: block until /magnet/object_attached goes True
    'WAIT_LIFT',  # not published: block until the payload is above arg m (default 0.5), then
                  # shift every later event by the wait (a creep lift varies by ~10 s, R0553);
                  # arg 'z timeout_s' gives up after timeout_s (event LIFT_TIMEOUT)
    'WAIT_REWELD',  # not published: block until the magnet welds AGAIN after a release
                    # (a drone that starts welded, leaves and rejoins), then shift later events;
                    # arg 'timeout_s' gives up after timeout_s (event REWELD_TIMEOUT) and goes on
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
    'PARTNER_GO',  # '{"running": true}' on /runner/partner_go for 3 s: his supervisor's start gate when his launch
                   # has start_gate_topic:=/runner/partner_go (his takeoff late, card 2026-10-10 rev 10 F3)
    'FLEET_HANDOVER',  # /fleet/handover True for 1.5 s: the elrs_mux (partner_m2) hands each
                       # drone to our tracker on its first flying command (M2 bench)
    'WRENCH',     # persistent wrench on a Gazebo link, arg "fx fy fz [tx ty tz] [scoped link]" (N, N m, world
                  # frame; default lift_system::payload::body); it replaces the previous WRENCH on that link,
                  # "0 0 0" (or "0 0 0 0 0 0") removes it. Needs a world with the ApplyLinkWrench system
                  # (*_push.sdf)
    'WAIT_THRUST',     # not published: block until every drone's ELRSCommand on the arg
                       # template (default cmd_ready_topic) is armed above idle throttle,
                       # then shift later events (M2 bench: hangers release only under thrust)
    'MOCAP_LOSS',  # std_msgs/Float64 on /drone_<id>/mocap_dropout: the sim mocap emulator stops that
                   # drone's state for arg "id [seconds]" (default 600): a lost marker set, the tracker's
                   # own pose-timeout fault (card 2026-10-09_newcomer_fault_scope)
    'TIP_FREEZE',  # std_msgs/Float64 on /magnet_tip_publisher/freeze: the sim tip publisher repeats its last
                   # output for arg seconds, as a frozen mocap body would (card 2026-10-09_approach_tip_feedback)
    'MOCAP_TILT',  # Float64MultiArray [deg, s, ramp_s] on /drone_<id>/mocap_tilt: the sim mocap emulator rotates that
                   # drone's published attitude about its body x, in one step (a mis-solve) or over ramp_s (a body
                   # tipping), then holds deg for s seconds (default 1), as the trackers and the partner's MPC see it
                   # (card 2026-10-11_quad4_tilt_cut); arg "id deg [s [ramp_s]]"; id -1 = the ring, about its body y
                   # (card 2026-10-21_tip_detect rev 3)
    'WAIT_PICKUP_FREE',  # not published: block until his ball weld (/pickup/detachable_joint_state, made at
                         # spawn on QUAD4's tip) has read 'detached' for 1 s, then shift later events; arg
                         # 'timeout_s': on expiry the run fails and stops (the full M1: QUAD4's tip is also
                         # welded to the ring, so the creep must not start with the ball on it)
    'WAIT_LOG',   # not published: block until the dissipative planner has logged a line matching
                  # the arg's regex (any time this run), then time later events from that line; arg
                  # '[timeout_s] regex', e.g. '60 approach: drone 3 descend' (event LOGGED or
                  # LOG_TIMEOUT). Times an event into a planner phase that has no topic of its own.
    'TASK',       # /fleet/task "d x,y,z,dwell; ...": staged carrier d flies to each waypoint (world m), holds there
                  # dwell s, then back to its staging point (rejoin_task, card 2026-10-23_scalability case 6)
    'WAIT_TRAJ',  # not published: block until the planner's trajectory clock (/payload/trajectory_state) has reached
                  # the arg, then time later events from that sample; arg 't_s [timeout_s]' or 'lap laps [timeout_s]'
                  # ('lap 1.5': half way round the second lap; card 2026-10-23_scalability case 7) (event TRAJ_REACHED
                  # or TRAJ_TIMEOUT)
}

# rejoin_task's waypoints, as the planner bounds them: inside its staging bound (the 1.75 m limit less 0.18 m,
# dissipative_node._stage_lim) and high enough that the rod's end clears the floor
TASK_LIMIT_INSET_M = 0.18
TASK_ROD_CLEAR_M = 0.35
# the paths that fly laps (mpc_planner.load_trajectory.LAPPED)
LAPPED_KINDS = ('circle', 'spin', 'fig_8', 'line_spin')

# The twin's rig-fidelity switches, read from the environment by the sim nodes at start (cards
# 2026-10-14_tracker_timing, 2026-10-22_velocity_filter). run_experiment.py sets a config's `lab_env:` for every
# process of the run, on the laptop and in the lab container alike, and records it in the manifest.
LAB_ENV = {'SIM_CMD_DELAY_S': float, 'SIM_CMD_DELAY_FROM_S': float, 'SIM_MOCAP_DELAY_S': float,
           'SIM_MOCAP_DELAY_FROM_S': float, 'SIM_MOCAP_VEL_FILTER': ('none', 'ema', 'slope'),
           'SIM_MOCAP_VEL_RING': ('ema', 'slope'), 'MDC_MOCAP_VEL_DRONES': str,
           'TRACKER_CMD_NEXT_TICK': ('0', '1'), 'TRACKER_DELAY_COMP_S': float,
           # per-channel command delay (card 2026-10-23_pair_mode_timing)
           'SIM_CMD_DELAY_THR_S': float, 'SIM_CMD_DELAY_ATT_S': float}


def lab_env(raw, path='config'):
    """{name: string value} from a config's `lab_env:` mapping, refused on an unknown name or a bad value."""
    env = {}
    for k, v in (raw or {}).items():
        kind = LAB_ENV.get(k)
        if kind is None:
            raise ValueError(f'{path}: lab_env {k} is not a known switch ({", ".join(sorted(LAB_ENV))})')
        v = str(v).strip()
        if kind is float:
            try:
                x = float(v)
            except ValueError:
                raise ValueError(f'{path}: lab_env {k}={v!r} is not a number') from None
            # every numeric switch is a time or a gain: a negative or non-finite one is a typo
            if not math.isfinite(x) or x < 0.0:
                raise ValueError(f'{path}: lab_env {k}={v!r}, want a finite number >= 0')
        elif isinstance(kind, tuple) and v not in kind:
            raise ValueError(f'{path}: lab_env {k}={v!r}, want one of {kind}')
        env[k] = v
    return env


def params_file_path(path):
    """The file a config's params_file names, or None if there is none. A laptop path read in a lab slot's copy of
    the tree is looked up under this tree's configs/."""
    if not path:
        return None
    p = os.path.expanduser(str(path))
    if not os.path.isabs(p):
        p = os.path.join(REPO, p)
    if not os.path.exists(p) and '/configs/' in p:
        p = os.path.join(REPO, 'configs', p.split('/configs/', 1)[1])
    return p if os.path.isfile(p) else None


def params_file_knobs(path):
    """{knob: value} of a config's params_file (sections flattened), or {} where it cannot be read."""
    p = params_file_path(path)
    if p is None:
        return {}
    try:
        with open(p) as fh:
            raw = yaml.safe_load(fh) or {}
    except OSError:
        return {}
    out = {}
    for k, v in raw.items():
        if k == 'modes':
            continue
        out.update(v if isinstance(v, dict) else {k: v})
    return out


def wrench_arg(arg):
    """WRENCH arg -> (force (fx, fy, fz) in N, torque (tx, ty, tz) in N m, scoped link name):
    'fx fy fz [tx ty tz] [link]' (torque 0 and the ring's body by default)."""
    parts = str(arg or '').split()
    if len(parts) not in (3, 4, 6, 7):
        raise ValueError(f'WRENCH needs "fx fy fz [tx ty tz] [link]", got {arg!r}')
    nums = parts[:6] if len(parts) >= 6 else parts[:3]
    vals = tuple(float(v) for v in nums)
    link = parts[len(nums)] if len(parts) > len(nums) else 'lift_system::payload::body'
    return vals[:3], (vals[3:] if len(vals) == 6 else (0.0, 0.0, 0.0)), link


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


def wait_reweld_arg(arg, what='WAIT_REWELD'):
    """WAIT_REWELD / WAIT_WELD arg -> timeout SIM s or None: '' or 'timeout_s'. On expiry the schedule
    goes on (card 2026-10-10_combined_m1_contact: no attach, then LAND)."""
    if arg is None or str(arg).strip() == '':
        return None
    parts = str(arg).split()
    if len(parts) != 1:
        raise ValueError(f"{what} arg {arg!r}: expected 'timeout_s'")
    timeout = float(parts[0])
    if timeout <= 0:
        raise ValueError(f'{what} timeout must be > 0, got {timeout}')
    return timeout


def mocap_tilt_arg(arg):
    """MOCAP_TILT arg -> (drone id, deg, seconds, ramp_s): 'id deg [seconds [ramp_s]]' (default 1 s, a step)."""
    parts = str(arg if arg is not None else '').split()
    if not 2 <= len(parts) <= 4:
        raise ValueError(f"MOCAP_TILT arg {arg!r}: expected 'drone_id deg [seconds [ramp_s]]'")
    deg = float(parts[1])
    secs = float(parts[2]) if len(parts) >= 3 else 1.0
    ramp = float(parts[3]) if len(parts) == 4 else 0.0
    if not 0.0 < abs(deg) <= 180.0 or secs <= 0.0 or ramp < 0.0:
        raise ValueError(f'MOCAP_TILT: deg in (0, 180], seconds > 0 and ramp_s >= 0, got {deg}, {secs}, {ramp}')
    return int(parts[0]), deg, secs, ramp


def mocap_loss_arg(arg):
    """MOCAP_LOSS arg -> (drone id, seconds): 'id [seconds]' (default 600 s, the rest of the run)."""
    parts = str(arg if arg is not None else '').split()
    if not 1 <= len(parts) <= 2:
        raise ValueError(f"MOCAP_LOSS arg {arg!r}: expected 'drone_id [seconds]'")
    secs = float(parts[1]) if len(parts) == 2 else 600.0
    if secs <= 0:
        raise ValueError(f'MOCAP_LOSS seconds must be > 0, got {secs}')
    return int(parts[0]), secs


def wait_log_arg(arg):
    """WAIT_LOG arg -> (compiled regex, timeout SIM s or None): '[timeout_s] regex'."""
    text = str(arg or '').strip()
    if not text:
        raise ValueError('WAIT_LOG needs a regex as its arg')
    head, _, rest = text.partition(' ')
    try:
        timeout = float(head)
    except ValueError:
        timeout, rest = None, text
    if timeout is not None and (timeout <= 0 or not rest.strip()):
        raise ValueError(f"WAIT_LOG arg {arg!r}: expected '[timeout_s] regex' with timeout > 0")
    return re.compile(rest.strip()), timeout


def wait_traj_arg(arg):
    """WAIT_TRAJ arg -> (target, in laps, timeout SIM s or None): 't_s [timeout_s]' (the trajectory clock) or
    'lap laps [timeout_s]' (that many laps of the path)."""
    parts = str(arg if arg is not None else '').split()
    laps = bool(parts) and parts[0].lower() == 'lap'
    try:
        vals = [float(v) for v in (parts[1:] if laps else parts)]
    except ValueError:
        vals = []
    if not 1 <= len(vals) <= 2 or not all(math.isfinite(v) and v > 0.0 for v in vals):
        raise ValueError(f"WAIT_TRAJ arg {arg!r}: expected 't_s [timeout_s]' or 'lap laps [timeout_s]', each > 0")
    return vals[0], laps, (vals[1] if len(vals) == 2 else None)


def task_arg(arg):
    """TASK arg -> (drone id, [(x, y, z, dwell s), ...]): 'd x,y,z,dwell; x,y,z,dwell; ...' (world m), as the planner
    reads /fleet/task (dissipative_node.parse_task)."""
    head, _, rest = str(arg if arg is not None else '').strip().partition(' ')
    try:
        d = int(head)
        wps = [tuple(float(v) for v in part.split(',')) for part in rest.split(';') if part.strip()]
    except ValueError:
        d, wps = -1, []
    if d < 0 or not wps or any(len(w) != 4 or not all(math.isfinite(v) for v in w) or w[3] < 0.0 for w in wps):
        raise ValueError(f"TASK arg {arg!r}: expected 'drone_id x,y,z,dwell[; x,y,z,dwell ...]', finite, dwell >= 0")
    return d, wps


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
        if self.do in ('WAIT_REWELD', 'WAIT_WELD', 'WAIT_PICKUP_FREE'):
            wait_reweld_arg(self.arg, self.do)
        if self.do == 'WAIT_LOG':
            wait_log_arg(self.arg)
        if self.do == 'WAIT_TRAJ':
            wait_traj_arg(self.arg)
        if self.do == 'TASK':
            task_arg(self.arg)
        if self.do == 'WAIT_THRUST' and self.arg is not None and '{i}' not in str(self.arg):
            raise ValueError(f"WAIT_THRUST arg {self.arg!r} has no '{{i}}' for the drone id")
        if self.do == 'WRENCH':
            wrench_arg(self.arg)
        if self.do == 'MOCAP_LOSS':
            mocap_loss_arg(self.arg)
        if self.do == 'MOCAP_TILT':
            mocap_tilt_arg(self.arg)
        if self.do == 'TIP_FREEZE':
            if self.arg is None or float(self.arg) <= 0.0:
                raise ValueError(f'TIP_FREEZE needs seconds > 0, got {self.arg!r}')
            self.arg = float(self.arg)
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
        # card 2026-10-11_quad4_tilt_cut (harness only): any ARMED drone above this tilt for 3 consecutive rows, with
        # a MOCAP_TILT's own window excluded for that drone (its run.csv attitude is the fake one); and every drone
        # disarmed this long after LANDED (R1503: QUAD4 armed upside down after LANDED)
        self.max_armed_tilt_deg = kw.pop('max_armed_tilt_deg', None)
        self.disarmed_after_landed_s = kw.pop('disarmed_after_landed_s', None)
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
        self.record = None               # video: experiment/record.py (`record:` or --record)
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
        c.env = lab_env(raw.get('lab_env'), path)
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
        c.record = _record.parse(raw.get('record'))
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
        knobs = {**params_file_knobs(self.launch_args.get('params_file')), **self.launch_args}
        if str(knobs.get('load_traj', '')).strip() == 'fig_8' and knobs.get('fig8_size') in (None, ''):
            # Wesley 22 Oct: fig8_size sizes the figure-8 (profile 0.9), traj_radius only the circle, spin and orbit
            bad.append('load_traj fig_8 without fig8_size: since 22 Oct the figure-8 is sized by fig8_size (profile '
                       '0.9), no longer traj_radius; type the size it flies (before 22 Oct it flew traj_radius, '
                       '0.5 untyped)')
        bad += self._rejoin_problems(knobs)
        bad += self._traj_problems(knobs)
        if bad:
            raise ValueError(f'{self.name}: invalid experiment config:\n  '
                             + '\n  '.join(bad))
        return True

    def _rejoin_problems(self, knobs):
        """An ATTACH of a carrier (an id below num_drones) is a departed carrier's rejoin: the planner refuses it
        without rejoin_departed, and the rejoin flies only with its prerequisites typed (card
        2026-10-23_rejoin_departed). knobs: launch.args over the typed params_file."""
        def on(v):
            return str(v).strip().lower() in ('1', 'true', 'yes')
        carriers = sorted({e.arg for e in self.events if e.do == 'ATTACH' and e.arg < self.num_drones})
        tasks = [task_arg(e.arg) for e in self.events if e.do == 'TASK']
        if tasks and not (on(knobs.get('rejoin_task', '')) and on(knobs.get('rejoin_departed', ''))):
            return ['TASK: a staged carrier flies a task only with rejoin_departed: true and rejoin_task: true']
        if not on(knobs.get('rejoin_departed', '')):
            return ([f'ATTACH of carrier(s) {carriers}: a departed carrier rejoins only with rejoin_departed: true']
                    if carriers else [])
        bad = []
        if str(knobs.get('departed_land', '')).strip().lower() not in ('false', '0', 'no'):
            bad.append('rejoin_departed needs departed_land: false (a staged carrier holds, it does not land)')
        if str(knobs.get('reconfig_mode', 'ocp')).strip().lower() != 'ocp':
            bad.append('rejoin_departed needs reconfig_mode ocp')
        if self.launch_file != 'sim_control_launch.py' or str(self.launch_args.get('mode', '')) != 'dissipative':
            bad.append('rejoin_departed flies only sim_control_launch.py mode: dissipative')
        if not on(self.io_launch_args.get('tether_tips', False)):
            bad.append("rejoin_departed needs io_launch tether_tips: true (the weld reads each rod's end)")
        # rejoin_task (card 2026-10-23_scalability case 6): a carrier's id, every waypoint inside the planner's staging
        # bound and high enough that the rod's end clears the floor
        bound = float(knobs.get('pos_limit_m') or 1.75) - TASK_LIMIT_INSET_M
        rod = float(knobs.get('cable_len') or self.io_launch_args.get('tether_len') or 1.0)
        for d, wps in tasks:
            if d >= self.num_drones:
                bad.append(f'TASK of drone {d}: only a carrier (an id below num_drones {self.num_drones}) stages')
            for x, y, z, _ in wps:
                if max(abs(x), abs(y)) > bound + 1e-9:
                    bad.append(f'TASK of drone {d}: waypoint ({x:g}, {y:g}) outside the staging bound +-{bound:.2f} m')
                if z < rod + TASK_ROD_CLEAR_M - 1e-9:
                    bad.append(f'TASK of drone {d}: waypoint z {z:g} m under the rod + {TASK_ROD_CLEAR_M:g} m '
                               f'({rod + TASK_ROD_CLEAR_M:.2f} m): its end would near the floor')
        return bad

    def _traj_problems(self, knobs):
        """traj_laps on a lapped path; traj_lap_hold_s with traj_laps over 1 in the twin's mode:=dissipative with
        reconfig_mode ocp; WAIT_TRAJ on a path whose clock runs, 'lap x' within its laps (card 2026-10-23_scalability
        case 7). knobs: launch.args over the typed params_file."""
        kind = str(knobs.get('load_traj', '')).strip()
        laps = knobs.get('traj_laps')
        laps = 1 if laps in (None, '') else float(laps)
        bad = []
        if laps != 1 and (laps != int(laps) or laps < 1 or kind not in LAPPED_KINDS):
            bad.append(f'traj_laps {laps:g} on load_traj {kind or "hover"}: a whole number of laps of '
                       f'{", ".join(LAPPED_KINDS)}')
        if float(knobs.get('traj_lap_hold_s') or 0.0) > 0.0:
            if laps < 2:
                bad.append('traj_lap_hold_s holds the path at the lap ends but the last: it needs traj_laps over 1')
            if (self.launch_file != 'sim_control_launch.py' or str(self.launch_args.get('mode', '')) != 'dissipative'
                    or str(knobs.get('reconfig_mode', 'ocp')).strip().lower() != 'ocp'):
                bad.append('traj_lap_hold_s flies only sim_control_launch.py mode: dissipative with reconfig_mode ocp')
        for e in self.events:
            if e.do != 'WAIT_TRAJ':
                continue
            x, in_laps, _ = wait_traj_arg(e.arg)
            if kind in ('', 'hover'):
                bad.append(f'{e!r}: load_traj {kind or "hover"} has no trajectory clock to wait on')
            elif in_laps and kind not in LAPPED_KINDS:
                bad.append(f'{e!r}: a lap target needs a lapped path ({", ".join(LAPPED_KINDS)})')
            elif in_laps and x > laps + 1e-9:
                bad.append(f'{e!r}: the path flies {laps:g} lap(s), so lap {x:g} never comes')
        return bad

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
