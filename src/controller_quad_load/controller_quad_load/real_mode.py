"""The rig side of the sim launches M1 and M2 fly (`real:=true`).

Wesley 2026-09-28: the real M1/M2 launches are a mode on three_attach_launch.py and
dissipative_launch.py, not re-synced real_* files, so the rig flies the parameters the
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
RIG_DEFAULTS = {'payload_rest_z': 0.05, 'z_taut_gate': 0.9}


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
            f"real:=true needs thrust_ratio typed as the airframe's measured kT "
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
    notes = []
    for name, rig in {**RIG_WATCHDOGS, **RIG_DEFAULTS}.items():
        if name not in sim_defaults:
            continue
        typed = float(context.launch_configurations.get(name, sim_defaults[name]))
        value = rig if typed == float(sim_defaults[name]) else typed
        if name in RIG_WATCHDOGS and value > rig:
            raise RuntimeError(
                f'real:=true: {name}:={typed:g} is looser than the rig watchdog ({rig:g} s); '
                f'the loose values are for Gazebo under CPU load. Drop it or type <= {rig:g}.')
        context.launch_configurations[name] = str(value)
        notes.append(f'{name} {value:g}')
    return '[launch] REAL: ' + ', '.join(notes)


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
                f'real:=true: drone {i} magnet latch at boot must be ON or OFF, got '
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
