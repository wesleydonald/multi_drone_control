"""The rig side of the graphs M1 and M2 fly (real_control_launch.py mode:=attach | m2).

Wesley 2026-09-28: the real M1/M2 launches are the sim attach and dissipative graphs
(real_control_launch.py mode:=attach | m2), not re-synced copies, so the rig flies the parameters the
sim claim flew. What changes on the rig is here, shared by both launches:

  * kT is the airframe's measured number, typed; 'auto' (the sim plant's gain) is refused.
    takeoff_thrust_ratio 'auto' (the sim's stand pop) becomes 0 = thrust_ratio, the
    ground takeoff every real launch flies.
  * the watchdogs are the node defaults, which are the rig values (pose 0.25 s, reference
    staleness 1.0 s); the sim launches loosen them for Gazebo under load. A tighter typed
    value is kept, a looser one is refused.
  * payload_rest_z 0.05 (the ring rests on the floor; every real launch and the node
    default) and z_taut_gate 0.9 (decisions.md 2026-09-25: 0.99 sim / 0.9 real) replace
    the sim defaults; a typed value is kept.
  * the creep floor start and the OCP resize (decisions.md 2026-09-28, Q7): start_taut
    false, handover_elev_deg 45, handover_settle_s 1.0 (2.0 until 2026-10-03), creep_vel 0.2,
    reconfig_mode ocp replace the sim defaults; a typed value is kept.
  * the rig thrust map and geometry of bringup/config/real.yaml reach the trackers and the
    planner (map_and_geometry; Wesley 2026-10-03), as in the rig carry.
  * "typed" is an argument set before the launch file's declarations ran (the command
    line, or an including launch), recorded by record_typed_args; a typed value equal to
    the sim default is kept too.
  * the rig I/O is real_io_launch.py: the mocap node, one elrs_interface per drone on
    /dev/QUAD<i+1> (drone i = quad i+1 = Motive body 11+i), fleet_viz from mocap, RViz.
  * one magnet path per drone (ladder P10): the radio's String latch /drone_<i>/magnet,
    defined from boot by magnet_initial (ON or OFF, never '' = whatever the forwarded
    stream carries). Tethers: the dissipative node (detach_magnet: ON at ARM, OFF at their
    detach). The newcomer: the magnet manager (ON for the capture, OFF on release). No
    mux merges a magnet channel, so a forwarded stream's channel 6 is passed untouched
    and the latch overwrites it in every packet.
"""
import os

from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions

RIG_WATCHDOGS = {'pose_timeout_s': 0.25, 'safety_ref_timeout_s': 1.0}
RIG_DEFAULTS = {'payload_rest_z': 0.05, 'z_taut_gate': 0.9,
                'start_taut': 'false', 'handover_elev_deg': 45.0, 'handover_settle_s': 1.0,
                'creep_vel': 0.2, 'reconfig_mode': 'ocp'}
_TYPED_KEY = 'real_mode_typed_args'


def record_typed_args(context):
    """Note what was typed (an OpaqueFunction placed before the launch file's declarations)."""
    context.launch_configurations[_TYPED_KEY] = ','.join(
        sorted(k for k in context.launch_configurations if k != _TYPED_KEY))
    return []


def truthy(context, name):
    """Return whether a launch argument reads as true."""
    return LaunchConfiguration(name).perform(context).strip().lower() in ('1', 'true', 'yes')


def declared_defaults(context, declarations):
    """{name: default string} of a launch file's DeclareLaunchArgument list."""
    return {a.name: perform_substitutions(context, a.default_value)
            for a in declarations if a.default_value is not None}


def rig_thrust_ratio(thrust_ratio, takeoff_thrust_ratio):
    """(kT, takeoff kT, note) for the rig: a typed positive number, never 'auto'."""
    spec = str(thrust_ratio).strip()
    try:
        kt = float(spec)
    except ValueError:
        kt = float('nan')
    if not kt > 0.0:
        raise RuntimeError(
            f"the rig needs thrust_ratio typed as the airframe's measured kT "
            f"(e.g. thrust_ratio:=24.0), got {spec!r}: 'auto' is the sim plant's gain")
    tspec = str(takeoff_thrust_ratio).strip().lower()
    kt_to = 0.0 if tspec == 'auto' else float(tspec)
    return kt, kt_to, f'kT typed {kt:.2f} (rig); takeoff ' + (
        f'{kt_to:.2f}' if kt_to > 0.0 else '= kT (ground takeoff)')


def apply_rig_values(context, sim_defaults):
    """Put the rig values into the launch configuration in place of the sim defaults.

    Every node parameter reads these through LaunchConfiguration, so the node graph is the
    sim one with only these numbers changed. Returns a log line.
    """
    if _TYPED_KEY not in context.launch_configurations:
        raise RuntimeError('real_mode: the launch file must run record_typed_args before '
                           'its DeclareLaunchArgument list')
    typed = set(context.launch_configurations[_TYPED_KEY].split(','))
    notes = []
    for name, rig in {**RIG_WATCHDOGS, **RIG_DEFAULTS}.items():
        if name not in sim_defaults:
            continue
        value = context.launch_configurations[name] if name in typed else rig
        if isinstance(rig, float):
            value = float(value)
        if name in RIG_WATCHDOGS and value > rig:
            raise RuntimeError(
                f'rig: {name}:={value:g} is looser than the rig watchdog ({rig:g} s); '
                f'the loose values are for Gazebo under CPU load. Drop it or type <= {rig:g}.')
        context.launch_configurations[name] = str(value)
        notes.append(f'{name} {value:g}' if isinstance(value, float) else f'{name} {value}')
    return '[launch] REAL: ' + ', '.join(notes)


# knobs the rig carry passes and the sim graphs did not (Wesley 2026-10-03: every rig mode)
MAP_TRACKER = ('thrust_offset', 'thrust_offset_v_slope', 'thrust_v_ref', 'throttle_max')
MAP_PLANNER = ('attach_radius', 'pretension_s', 'ff_cap_release_s', 'rod_tol_frac',
               'rod_spread_m', 'z_i_gate')
# planner knobs passed only when set ('' = the node default), so a default launch is unchanged
PLANNER_OPTIONAL = {'land_unwind': 'bool', 'dist_est': 'str', 'dist_tau': 'float',
                    'dist_rod_mass': 'float', 'dist_rod_com': 'float',
                    'rod_mass': 'float', 'rod_com': 'float',
                    'offset_free': 'str', 'of_tau': 'float', 'of_window': 'float',
                    'pin_cable_rates': 'bool',
                    'int_mode': 'str', 'int_k_xy': 'float', 'int_k_z': 'float',
                    'int_settle_s': 'float', 'land_ff_ramp': 'bool'}
# the trackers' thrust map, which the disturbance estimate reads delivered thrust through
DIST_MAP = ('thrust_ratio', 'thrust_offset', 'thrust_offset_v_slope', 'thrust_v_ref')


def planner_options(context, resizes=False):
    """Return [{knob: value}] of the set PLANNER_OPTIONAL knobs, or [].

    Appended to a planner's parameters list. With dist_est set, the trackers' thrust map goes
    along (a numeric thrust_ratio is required). `resizes`: the graph can attach or detach a
    drone mid-flight (int_mode model is refused there). int_mode auto (the default since 5 Oct)
    flies model wherever model is allowed and reference elsewhere.
    """
    opts = {}
    for k, kind in PLANNER_OPTIONAL.items():
        v = LaunchConfiguration(k).perform(context).strip()
        if v:
            opts[k] = (truthy(context, k) if kind == 'bool' else float(v) if kind == 'float'
                       else v.lower())
    if opts.get('offset_free') == 'on' and truthy(context, 'kt_trim'):
        raise RuntimeError('offset_free on with kt_trim on: two adaptive loops on one vertical residual')
    if opts.get('int_mode') in ('model', 'auto'):
        z_ki = LaunchConfiguration('z_ki').perform(context).strip()
        dist = opts.get('dist_est', '')
        why = ('kt_trim on (two adaptive loops on one vertical residual)'
               if truthy(context, 'kt_trim')
               else 'a fleet that can attach or detach mid-flight' if resizes
               else 'z_ki 0 (it is the gain)' if not z_ki or float(z_ki) <= 0.0
               else 'offset_free on' if opts.get('offset_free') == 'on'
               else 'dist_est ' + dist if dist in ('ring', 'full') else '')
        if why and opts['int_mode'] == 'model':
            raise RuntimeError(f'int_mode model refused with {why} (card 2026-10-04_z_int_model)')
        opts['int_mode'] = 'reference' if why else 'model'
    if opts.get('dist_est', 'off') != 'off':
        for k in DIST_MAP:
            v = LaunchConfiguration(k).perform(context).strip()
            try:
                opts[k] = float(v)
            except ValueError:
                raise RuntimeError(f'dist_est needs a numeric {k} (the trackers\' map), got {v!r}')
    return [opts] if opts else []


def map_and_geometry(context, n, real):
    """(tracker, sim plant, planner) parameters of the thrust-map and geometry knobs.

    Rig: every value of the profile (the rig carry's map, pivot, pretension, cap release,
    rod trust band, solve budget). Sim: only the knobs that are set ('' = not passed, the
    node's default), so a rig-twin rehearsal can type them and nothing else changes."""
    def get(k):
        return LaunchConfiguration(k).perform(context).strip()
    tracker = {k: float(get(k)) for k in MAP_TRACKER if get(k)}
    planner = {k: float(get(k)) for k in MAP_PLANNER if get(k)}
    if get('pivot_offset_z'):
        planner['pivot_offset'] = [0.0, 0.0, float(get('pivot_offset_z'))]
    if real:
        planner['solve_budget_s'] = float(get('solve_budget_s'))
        return tracker, {}, planner
    plant = {}
    if get('sim_thrust_map'):
        offsets = [float(x) for x in get('sim_thrust_offset').split(',') if x.strip()] or [0.185]
        if len(offsets) not in (1, n):
            raise RuntimeError(f'sim_thrust_offset needs one value or {n}, got {len(offsets)}')
        plant = {'thrust_map': get('sim_thrust_map'),
                 'thrust_offset': [offsets[min(i, len(offsets) - 1)] for i in range(n)],
                 'pack_v0': float(get('sim_pack_v0') or 24.4)}
        # 'auto' is the linear x3 plant's gain; a rig plant needs the rig's measured map typed in
        if get('sim_thrust_map').lower() == 'rig' and (
                get('thrust_ratio').lower() == 'auto' or tracker.get('thrust_offset', 0.0) <= 0.0):
            raise RuntimeError('sim_thrust_map:=rig needs an explicit thrust_ratio and thrust_offset '
                               '> 0 (the rig launch values: 35.2, 0.185)')
    return tracker, plant, planner


def rig_magnet_latches(context, defaults):
    """Return each drone's radio latch at boot (ON or OFF).

    Its role default (defaults[i]) unless drone<i>_magnet_initial is typed. '' is refused:
    it would leave the magnet to whatever channel 6 the forwarded stream carries until the
    first latch command.
    """
    out = []
    for i, role in enumerate(defaults):
        typed = context.launch_configurations.get(f'drone{i}_magnet_initial')
        value = str(role if typed is None else typed).strip().upper()
        if value not in ('ON', 'OFF'):
            raise RuntimeError(
                f'rig: drone {i} magnet latch at boot must be ON or OFF, got '
                f'{value!r} (drone{i}_magnet_initial / magnet_initial / attach_magnet_initial)')
        out.append(value)
    return out


def rig_io(launch_dir, n_radio, magnet_initial, attach=False, detach=False):
    """real_io_launch.py for drones 0..n_radio-1, scoped so its arguments do not leak.

    magnet_initial: one value per drone for its radio latch (rig_magnet_latches).
    Each drone's serial port is the launch argument drone<i>_serial (default
    /dev/QUAD<i+1>).
    """
    actions = []
    args = {'num_drones': str(n_radio),
            'rviz': LaunchConfiguration('rviz'),
            'magnet_channel': LaunchConfiguration('magnet_channel'),
            'attach': 'true' if attach else 'false',
            'detach': 'true' if (attach or detach) else 'false'}
    for i in range(n_radio):
        name = f'drone{i}_serial'
        actions.append(DeclareLaunchArgument(
            name, default_value=f'/dev/QUAD{i + 1}',
            description=f'drone {i} ELRS TX serial device (drone i = quad i+1)'))
        args[name] = LaunchConfiguration(name)
        args[f'drone{i}_magnet_initial'] = magnet_initial[i]
    actions.append(GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'real_io_launch.py')),
        launch_arguments=list(args.items()))], scoped=True, forwarding=True))
    return actions
