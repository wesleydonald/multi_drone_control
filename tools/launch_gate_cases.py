"""The equivalence gate's cases (plan noble-noodling-lollipop §3b): every launch command the
consolidation must reproduce, as the PRE-consolidation launch file and its arguments.

    python3 tools/launch_gate_cases.py OUTDIR      # freeze old-launch dumps (before the change)

Frozen on 3 Oct 2026 with the rename-only launches; tools/launch_gate.py replays each case
on the consolidated launches and compares the dumps. Kept so a later launch edit can be
checked against the same commands.
"""
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

AZ4 = '330,270,210,90'          # typed clockwise (plates 11/9/7/3)
RIG_IO_0930 = {'num_drones': 3, 'drone0_serial': '/dev/QUAD1', 'drone1_serial': '/dev/QUAD2',
               'drone2_serial': '/dev/QUAD4', 'mocap_drone_body_ids': '11,12,14',
               'magnet_channel': 6, 'magnet_initial': 'ON'}
RIG_IO_WED = {'num_drones': 4, 'drone0_serial': '/dev/QUAD1', 'drone1_serial': '/dev/QUAD2',
              'drone2_serial': '/dev/QUAD3', 'drone3_serial': '/dev/QUAD4',
              'mocap_drone_body_ids': '11,12,13,14', 'magnet_channel': 6, 'magnet_initial': 'ON',
              'detach': True}
M2_DRIVER = ['num_drones:=4', 'sim_interface:=false', 'partner_m2:=true', 'cable_len:=0.515',
             'attach_azimuths_deg:=270.0,0.0,180.0,90.0', 'attach_z:=-0.005', 'start_taut:=false',
             'handover_elev_deg:=45.0', 'cable_elev_deg:=45.0', 'handover_settle_s:=1.0',
             'creep_vel:=0.2', 'load_mass:=0.86', 'load_traj:=hover', 'target_z:=0.6',
             'reconfig_mode:=ocp', 'pose_timeout_s:=3.0', 'safety_ref_timeout_s:=3.0',
             'takeoff_spool_s:=0.0', 'airborne_start:=true', 'auto_slot_assign:=false']
DESK_M2 = ['num_drones:=4', 'sim_interface:=false', 'partner_m2:=true', 'real:=false',
           'cable_len:=0.515', 'attach_azimuths_deg:=330.0,240.0,150.0,60.0', 'start_taut:=false',
           'load_mass:=0.86', 'load_traj:=hover', 'target_z:=0.6', 'reconfig_mode:=ocp',
           'takeoff_spool_s:=0.0', 'airborne_start:=true', 'auto_slot_assign:=false']
DESK_R0A_T2 = ('num_drones:=3 load_mass:=0.86 drone_mass:=0.64 cable_len:=0.47 attach_radius:=0.25 '
               'attach_z:=0.0 attach_azimuths_deg:=330,210,90 thrust_ratio:=24.0 kt_trim:=false '
               'z_ki:=0.4 start_taut:=false handover_elev_deg:=45.0 handover_settle_s:=1.0 '
               'creep_vel:=0.2 target_z:=0.6 lift_ramp_vel:=0.0 load_traj:=hover').split()


def _argv(items):
    out = {}
    for it in items:
        k, v = it.split(':=', 1)
        out[k] = v
    return out


def command_cases():
    """(id, old launch file, args) for the rig, Wednesday, driver, desk and bare cases."""
    c = []
    # 1 Oct rig (docs/archive/rig_2026-10-01_commands.md; typed args read back from params.json)
    c.append(('rig1001_io', 'real_io_launch.py', RIG_IO_0930))
    for tag, a in (('h1', {'target_z': 0.5, 'z_ki': 0.0}), ('h2', {'target_z': 0.8}),
                   ('h5', {'target_z': 1.0}),
                   ('c1', {'target_z': 1.0, 'load_traj': 'orbit', 'traj_speed': 0.125,
                           'traj_radius': 0.5}),
                   ('f8', {'target_z': 1.0, 'load_traj': 'fig_8', 'traj_speed': 0.2})):
        c.append((f'rig1001_{tag}', 'real_control_launch.py', {'num_drones': 3, **a}))
    # Wednesday 7 Oct (plan §5 sheet draft): four drones on plates 1/3/5/9 at 1.0 m
    c.append(('wed_io', 'real_io_launch.py', RIG_IO_WED))
    base = {'num_drones': 4, 'attach_azimuths_deg': AZ4, 'target_z': 1.0}
    for tag, a in (('f1', {'z_ki': 0.0}), ('f5', {}),
                   ('f6', {'load_traj': 'orbit', 'traj_radius': 0.5, 'traj_speed': 0.125}),
                   ('f7', {'load_traj': 'fig_8', 'traj_radius': 1.0, 'traj_speed': 0.25})):
        c.append((f'wed_{tag}', 'real_control_launch.py', {**base, **a}))
    c.append(('wed_f8_detach', 'real_dissipative_launch.py', dict(base)))
    c.append(('wed_f8_detach_typed', 'real_dissipative_launch.py',
              {**base, 'reconfig_mode': 'ocp', 'detach_magnet': True}))
    c.append(('wed_hover_free', 'real_hover_launch.py', {'num_drones': 4, 'hover_z': 0.8}))
    # drivers and desk scripts
    for part in ('all', 'muxes', 'controllers'):
        c.append((f'm2driver_{part}', 'dissipative_launch.py',
                  {**_argv(M2_DRIVER), 'partner_m2_part': part}))
        c.append((f'deskm2_{part}', 'dissipative_launch.py', {**_argv(DESK_M2), 'partner_m2_part': part}))
    c.append(('deskr0a_t1', 'real_io_launch.py', _argv(
        'num_drones:=3 drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 drone2_serial:=/dev/QUAD3 '
        'magnet_channel:=6 magnet_initial:=ON rviz:=false'.split())))
    c.append(('deskr0a_t2', 'real_control_launch.py', _argv(DESK_R0A_T2)))
    # every launch and mode with its defaults
    for lf in ('mpc_quad_load_launch.py', 'dissipative_launch.py', 'dissipative_only_launch.py',
               'three_attach_launch.py', 'real_control_launch.py', 'real_dissipative_launch.py',
               'real_hover_launch.py', 'real_io_launch.py', 'rviz_quad_load_launch.py',
               'm2_bench_io_launch.py'):
        c.append((f'bare_{lf[:-10]}', lf, {}))
    c.append(('bare_free_hover_sim', 'mpc_quad_load_launch.py', {'reference': 'free_hover'}))
    c.append(('bare_attach_sil', 'three_attach_launch.py', {'sil': True}))
    c.append(('bare_attach_real', 'three_attach_launch.py',
              {'real': True, 'thrust_ratio': 24.0, 'enable_approach_mpc': False}))
    c.append(('bare_attach_real_partner', 'three_attach_launch.py',
              {'real': True, 'thrust_ratio': 24.0, 'partner': True}))
    c.append(('bare_m2_real', 'dissipative_launch.py',
              {'real': True, 'thrust_ratio': 24.0, 'partner_m2': True}))
    c.append(('bare_m2_real_plain', 'dissipative_launch.py',
              {'real': True, 'thrust_ratio': 24.0, 'num_drones': 3}))
    for n in (2, 3, 4):
        c.append((f'n{n}_mpc', 'mpc_quad_load_launch.py', {'num_drones': n}))
        c.append((f'n{n}_real', 'real_control_launch.py', {'num_drones': n}))
    # the launch tests' fixed argument sets
    sys.path.insert(0, os.path.join(REPO, 'src', 'bringup', 'test'))
    import test_real_mode_launches as T
    c.append(('test_partner_orbit', 'three_attach_launch.py', T.PARTNER_ATTACHED_ORBIT))
    c.append(('test_m2_driver', 'dissipative_launch.py', T.M2_DRIVER))
    c.append(('test_partner_orbit_real', 'three_attach_launch.py',
              {**{k: v for k, v in T.PARTNER_ATTACHED_ORBIT.items()
                  if k not in ('pose_timeout_s', 'safety_ref_timeout_s')},
               'real': True, 'thrust_ratio': 24.0}))
    c.append(('test_r8a', 'three_attach_launch.py', {'real': True, 'thrust_ratio': 24.0, **T.R8A}))
    for lf, line in T.Q7_RIG_LINE.items():
        c.append((f'test_q7_{lf[:-10]}', lf, {'real': True, 'thrust_ratio': 24.0, **line}))
    return c


def config_cases():
    """(id, old launch file, args, config path, kind) for every run config, as the harness
    would launch it before the consolidation."""
    from experiment.config import ExperimentConfig
    from sil.scenario import Scenario
    out = []
    for p in sorted(glob.glob(os.path.join(REPO, 'configs', 'experiments', '*.yaml'))):
        cfg = ExperimentConfig.from_yaml(p)
        name = os.path.splitext(os.path.basename(p))[0]
        out.append((f'cfg_{name}', cfg.launch_file, _argv(cfg.launch_argv()), p, 'control'))
        out.append((f'cfgio_{name}', cfg.io_launch_file, _argv(cfg.io_launch_argv()), p, 'io'))
    for p in sorted(glob.glob(os.path.join(REPO, 'configs', 'sil', '*.yaml'))):
        s = Scenario.from_yaml(p)
        name = os.path.splitext(os.path.basename(p))[0]
        out.append((f'sil_{name}', s.launch_file, {'sil': 'true', **_argv(s.launch_argv())}, p, 'sil'))
    return out


def main(outdir):
    import launch_dump
    os.makedirs(os.path.join(outdir, 'old'), exist_ok=True)
    cases = [{'id': i, 'launch': lf, 'args': {k: str(v).lower() if isinstance(v, bool) else str(v)
                                             for k, v in a.items()}}
             for i, lf, a in command_cases()]
    cases += [{'id': i, 'launch': lf, 'args': a, 'config': os.path.relpath(p, REPO), 'kind': kind}
              for i, lf, a, p, kind in config_cases()]
    bad = 0
    for c in cases:
        try:
            g = launch_dump.dump(c['launch'], c['args'])
            c['old_entries'] = len(g['entries'])
        except Exception as e:                          # noqa: BLE001
            g = {'error': f'{type(e).__name__}: {e}'}
            c['old_error'] = g['error']
            bad += 1
        json.dump(g, open(os.path.join(outdir, 'old', c['id'] + '.json'), 'w'), indent=1, sort_keys=True)
    json.dump(cases, open(os.path.join(outdir, 'cases.json'), 'w'), indent=1)
    print(f'{len(cases)} cases, {bad} raise (recorded: a refusal must stay a refusal)')


if __name__ == '__main__':
    main(sys.argv[1])
