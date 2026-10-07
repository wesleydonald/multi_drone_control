"""Launch defaults for sim_control_launch.py and real_control_launch.py (plan 2026-10-03 §3b).

Every control knob has its default in bringup/config, layered in this order (later wins):

    common.yaml  ->  sim.yaml | real.yaml  ->  that file's modes:<mode>  ->  params_file:=<yaml>
    ->  the typed launch arguments

sim.yaml is the rig twin. legacy:=true (sim only) reads sim_legacy.yaml instead: the worlds in
simulation_assets/old_worlds/, and the only profile with the attach and network modes.

Only the knobs in KEPT are launch arguments (set in three or more run configs or commands,
or structural); any other knob goes in a params_file (flat `knob: value`, or grouped like
the profiles), so a typo cannot be accepted silently. apply() writes the merged values into
the launch configuration before a graph builder runs, so the builders read every knob
through LaunchConfiguration exactly as the per-mode launch files did before 3 Oct 2026.
"""
import os
import re

import yaml

MODES = {'sim': ('mpc', 'free_hover', 'dissipative', 'network', 'attach'),
         'real': ('mpc', 'free_hover', 'dissipative', 'attach', 'm2')}
DEFAULT_MODE = 'mpc'
# sim modes with no rig-twin world or thrust-map plumbing yet (docs/redo_on_twin.md)
LEGACY_ONLY = ('attach', 'network')

# launch arguments: knobs set in >= 3 configs or commands, then the structural ones
KEPT_COMMON = (
    'num_drones', 'cable_len', 'load_mass', 'drone_mass', 'attach_azimuths_deg', 'attach_radius',
    'attach_z', 'pivot_offset_z', 'measure_rod_len', 'rod_tol_frac', 'rod_spread_m',
    'auto_slot_assign', 'start_taut', 'handover_elev_deg', 'handover_settle_s', 'creep_vel',
    'pretension_s', 'ff_cap_release_s', 'lift_ramp_vel', 'target_z', 'z_taut_gate',
    'payload_rest_z', 'takeoff_spool_s', 'airborne_start', 'cable_elev_deg', 'z_ki', 'z_i_max',
    'z_i_gate', 'z_ki_in_orbit', 'load_traj', 'traj_speed', 'traj_radius', 'terminal_vel_ref',
    'thrust_ratio', 'takeoff_thrust_ratio', 'thrust_offset', 'thrust_offset_v_slope',
    'throttle_max', 'kt_trim', 'control_mode', 'vel_ki', 'pose_timeout_s',
    'safety_ref_timeout_s', 'reconfig_mode', 'reconfig_hold_s', 'diss_ki_load',
    'reserved_attach', 'enable_approach', 'enable_approach_mpc', 'attach_cable_len',
    'attach_central', 'attach_handout', 'attach_t_handout', 'attach_elev_deg', 'attach_x_offset',
    'attach_y_offset', 'attach_traj_hold_s', 'attach_traj_hold_mode', 'attach_t_start_new',
    'attach_blend_balanced', 'attach_datum_shift', 'attach_moving', 'diss_balanced_tensions',
    'weld_radius', 'weld_velocity_clock', 'enable_obstacle_avoidance', 'partner',
    'partner_attached', 'partner_m2', 'partner_m2_part')
KEPT_SIM = ('legacy', 'sil', 'sim_interface', 'ff_cap_force', 'sim_thrust_map', 'sim_thrust_offset',
            'sim_pack_v0')
KEPT_REAL = ('real_io', 'rviz', 'magnet_channel', 'magnet_initial', 'attach_magnet_initial',
             'require_fc_armed', 'detach_magnet', 'hover_z')
# typed per drone by the rig modes (bringup.real_mode.rig_io, rig_magnet_latches)
PER_DRONE = re.compile(r'drone\d+_(serial|magnet_initial)$')
# decided by the side or the mode, never typed
INTERNAL = ('real', 'reference')


def config_dir():
    """bringup/config: the source tree under a symlink install, else the installed share."""
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'config')
    if os.path.isdir(here):
        return here
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('bringup'), 'config')


def _load(path):
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def flatten(d):
    """{knob: value} of a profile or params file: nested sections are labels only."""
    out = {}
    for k, v in (d or {}).items():
        if k == 'modes':
            continue
        if isinstance(v, dict):
            out.update(flatten(v))
        else:
            out[k] = v
    return out


def as_launch_string(v):
    """A YAML value as the string a launch argument would carry."""
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if v is None:
        return ''
    if isinstance(v, float):
        return repr(v)
    return str(v)


def side_file(side, legacy=False):
    return 'sim_legacy.yaml' if side == 'sim' and legacy else f'{side}.yaml'


def side_knobs(side, cdir=None):
    """Every knob any mode of this side knows (sim: the twin and the legacy profile)."""
    cdir = cdir or config_dir()
    out = set(flatten(_load(os.path.join(cdir, 'common.yaml'))))
    for legacy in ((False, True) if side == 'sim' else (False,)):
        s = _load(os.path.join(cdir, side_file(side, legacy)))
        out |= set(flatten(s))
        for m in (s.get('modes') or {}).values():
            out |= set(flatten(m))
    if side == 'sim':
        out.add('legacy')
    return out


def kept(side):
    """The launch arguments of <side>_control_launch.py, after `mode` and `params_file`."""
    return tuple(KEPT_COMMON) + (KEPT_SIM if side == 'sim' else KEPT_REAL)


def profile(side, mode, params_file='', cdir=None, legacy=False):
    """Merged {knob: launch string} for one side and mode, with a params_file on top.
    legacy: the sim side reads sim_legacy.yaml (the old_worlds/ plant)."""
    if side not in MODES:
        raise RuntimeError(f'side must be sim or real, got {side!r}')
    if mode not in MODES[side]:
        raise RuntimeError(f'{side}_control_launch.py: mode must be one of '
                           f'{", ".join(MODES[side])}, got {mode!r}')
    legacy = side == 'sim' and bool(legacy)
    if side == 'sim' and mode in LEGACY_ONLY and not legacy:
        raise RuntimeError(f'sim_control_launch.py: mode {mode} has no rig-twin attach world yet: '
                           f'legacy:=true (a world in simulation_assets/old_worlds/)')
    cdir = cdir or config_dir()
    s = _load(os.path.join(cdir, side_file(side, legacy)))
    vals = flatten(_load(os.path.join(cdir, 'common.yaml')))
    vals.update(flatten(s))
    vals.update(flatten((s.get('modes') or {}).get(mode)))
    if params_file:
        extra = flatten(_load(params_file))
        unknown = sorted(set(extra) - side_knobs(side, cdir))
        if unknown:
            raise RuntimeError(f'params_file {params_file}: unknown knob(s) {", ".join(unknown)} '
                               f'for {side}_control_launch.py')
        vals.update(extra)
    if side == 'sim':
        vals['legacy'] = legacy
    return {k: as_launch_string(v) for k, v in vals.items()}


def declarations(side):
    """DeclareLaunchArgument list: mode, params_file and the kept knobs (carry-mode defaults)."""
    from launch.actions import DeclareLaunchArgument
    base = profile(side, DEFAULT_MODE)
    out = [DeclareLaunchArgument('mode', default_value=DEFAULT_MODE,
                                 description=' | '.join(MODES[side])),
           DeclareLaunchArgument('params_file', default_value='',
                                 description='YAML of knobs that are not launch arguments')]
    for k in kept(side):
        out.append(DeclareLaunchArgument(
            k, default_value=base.get(k, ''),
            description=f'default for mode {DEFAULT_MODE}; other modes: bringup/config/{side}.yaml'))
    return out


def split(side, args):
    """(launch arguments, params_file knobs) of a {knob: value} dict for <side>_control_launch.py."""
    keep = set(kept(side)) | {'mode', 'params_file'}
    largs = {k: v for k, v in args.items() if k in keep or PER_DRONE.match(k)}
    return largs, {k: v for k, v in args.items() if k not in largs}


def apply(context, side):
    """Fill the launch configuration from the profile; return (mode, profile values).

    Typed arguments win. A typed knob that is not a launch argument of this side is refused
    (put it in params_file). Needs bringup.real_mode.record_typed_args to have run before
    the declarations, which is how a typed value is told from a declared default."""
    from bringup.real_mode import _TYPED_KEY
    from launch.substitutions import LaunchConfiguration
    lc = context.launch_configurations
    if _TYPED_KEY not in lc:
        raise RuntimeError('profiles.apply: record_typed_args must run before the declarations')
    typed = {k for k in lc[_TYPED_KEY].split(',') if k}
    allowed = set(kept(side)) | {'mode', 'params_file'}
    bad = sorted(k for k in typed if k not in allowed and not PER_DRONE.match(k))
    if bad:
        raise RuntimeError(f'{side}_control_launch.py: {", ".join(bad)} not a launch argument here; '
                           f'set it in params_file:=<yaml> (knobs: bringup/config/{side}.yaml)')
    mode = LaunchConfiguration('mode').perform(context).strip()
    legacy = side == 'sim' and LaunchConfiguration('legacy').perform(context).strip().lower() == 'true'
    vals = profile(side, mode, LaunchConfiguration('params_file').perform(context).strip(),
                   legacy=legacy)
    for k, v in vals.items():
        if k not in typed:
            lc[k] = v
    lc['real'] = 'true' if side == 'real' else 'false'
    lc['reference'] = 'free_hover' if mode == 'free_hover' else 'planner'
    if side == 'real':
        lc['sil'] = 'false'
    return mode, vals
