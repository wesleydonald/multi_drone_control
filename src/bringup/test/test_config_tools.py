"""Tests for the configuration tools (finding F9).

`check_geometry.py` is a validator, and a validator that reports a mismatch which is
not real is worse than no validator — it gets ignored, and then it is not there when
it matters. The first version of it did exactly that: it read `attach_z` in the
lift_system frame instead of relative to the payload CoG, so every elevated world
(payload at 0.6 m) looked like `attach_z = 0.625` against a config value of 0.025.
These tests pin the frame convention down.

    python3 -m pytest src/bringup/test/test_config_tools.py -v
"""
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tools'))

from check_geometry import world_geometry, planner_defaults, launch_defaults  # noqa: E402
from param_diff import launch_args, profile_args                              # noqa: E402

WORLDS = REPO / 'simulation_assets'
LEGACY_WORLDS = WORLDS / 'old_worlds'
LAUNCHES = REPO / 'src/bringup/launch'
PROFILES = [f'{side}/{mode}' for side, modes in
            (('sim', ('mpc', 'free_hover', 'dissipative', 'network', 'attach')),
             ('sim_legacy', ('mpc', 'free_hover', 'dissipative', 'network', 'attach')),
             ('real', ('mpc', 'free_hover', 'dissipative', 'attach', 'm2'))) for mode in modes]


def every_default():
    """(name, {arg: default}) of every control-launch profile and every other launch file."""
    out = [(p, profile_args(p)) for p in PROFILES]
    out += [(lf.name, launch_args(lf)) for lf in LAUNCHES.glob('*launch.py')
            if lf.name not in ('sim_control_launch.py', 'real_control_launch.py')]
    return out


def ground_worlds():
    return [LEGACY_WORLDS / n for n in
            ('two_rigid_ground.sdf', 'three_rigid_ground.sdf', 'four_rigid_ground.sdf')]


# ── geometry extraction ──────────────────────────────────────────────────────

@pytest.mark.parametrize('sdf', ground_worlds(), ids=lambda p: p.name)
def test_ground_worlds_match_the_planner_defaults(sdf):
    """The legacy worlds must agree with params.py (still the legacy geometry; the twin's
    is typed by sim.yaml), or every drone's tension feedforward is sized wrongly."""
    g = world_geometry(sdf)
    d = planner_defaults()
    for k in ('cable_len', 'attach_radius', 'attach_z', 'load_mass'):
        assert abs(g[k] - d[k]) < 1e-3, (
            f"{sdf.name}: {k} world={g[k]:.4f} vs params.py={d[k]:.4f}")


def test_attach_z_is_relative_to_the_payload_not_the_world():
    """The bug this file exists for. `four_rigid_2026-09` hangs its payload at z=0.6 and
    `four_rigid_ground` at z=0.025; attach_z is defined above the load CoG, so both
    must report the SAME value. Reading the raw pose gives 0.625 vs 0.05."""
    elevated = LEGACY_WORLDS / 'four_rigid_2026-09.sdf'
    grounded = LEGACY_WORLDS / 'four_rigid_ground.sdf'
    a = world_geometry(elevated)['attach_z']
    b = world_geometry(grounded)['attach_z']
    assert abs(a - b) < 1e-3, (
        f"attach_z differs between an elevated and a grounded world ({a:.4f} vs "
        f"{b:.4f}) — it is being read in the wrong frame")


def test_drone_count_is_extracted():
    assert world_geometry(WORLDS / 'three_rigid_ground_rig.sdf')['num_drones'] == 3
    assert world_geometry(LEGACY_WORLDS / 'four_rigid_ground.sdf')['num_drones'] == 4


def test_non_uniform_attach_ring_is_flagged():
    """The randomly-generated worlds have unequal attach radii, which the planner's
    single `attach_radius` cannot represent. That must not pass silently."""
    rnd = LEGACY_WORLDS / 'three_random_seed1.sdf'
    assert '_warn_ring' in world_geometry(rnd)


# ── launch argument parsing ──────────────────────────────────────────────────

def test_launch_args_are_parsed_without_executing_the_launch():
    a = launch_args(LAUNCHES / 'sim_io_launch.py')
    assert 'num_drones' in a and 'clock_hz' in a
    assert profile_args('sim/mpc')['thrust_ratio'] == '35.2'
    assert profile_args('sim_legacy/mpc')['thrust_ratio'] == 'auto'


THRUST_MAP = ('thrust_ratio', 'takeoff_thrust_ratio', 'thrust_offset', 'thrust_offset_v_slope',
              'thrust_v_ref', 'throttle_max', 'kt_trim')
RIG_GEOMETRY = ('cable_len', 'pivot_offset_z', 'rod_tol_frac',
                'rod_spread_m', 'lift_ramp_vel', 'z_taut_gate', 'z_i_gate', 'reconfig_mode',
                'auto_network_handover')


@pytest.mark.parametrize('mode', ['mpc', 'dissipative'])
def test_the_twin_flies_the_rig_thrust_map_and_geometry(mode):
    """The sim default is the rig twin (5 Oct 2026): the identified affine map (gain 35.2
    above the 0.185 offset, 30 Sep 2026) and the rig's geometry, never a derived kT. 1 m cables
    on both since 22 Oct; the drone mass differs by decision: the twin flies its model's 0.55 kg
    (airframe, rod and magnet), the rig 0.583 with its 1 m cable (Wesley 22 Oct)."""
    sim, real = profile_args(f'sim/{mode}'), profile_args(f'real/{mode}')
    for k in THRUST_MAP + RIG_GEOMETRY:
        assert sim[k] == real[k], f'{k}: sim {sim[k]} vs rig {real[k]}'
    assert float(real['thrust_ratio']) == 35.2
    assert (sim['sim_thrust_map'], sim['sim_pack_v0'], sim['legacy']) == ('rig', '23.8', 'false')
    assert (sim['cable_len'], sim['drone_mass'], real['drone_mass']) == ('1.0', '0.55', '0.583')


def test_the_rig_flies_the_new_payload_and_the_twin_keeps_its_ring():
    """Wesley 28 Oct: the rig's default payload is the 1.15 kg ring (RIG-1028-payload: magnets at r 0.204, plates
    ~2.5 cm above the pivot, which rests at 1.3 cm); the twin's default worlds keep the M2A ring (0.86 kg, r 0.225,
    0.02 / 0.02) and its _p115 worlds type the new payload's values."""
    for mode in ('mpc', 'dissipative', 'attach', 'm2'):
        real = profile_args(f'real/{mode}')
        assert (real['load_mass'], real['attach_radius'], real['attach_z'], real['payload_rest_z']) == \
            ('1.15', '0.204', '0.028', '0.013'), mode
    sim = profile_args('sim/dissipative')
    assert (sim['load_mass'], sim['attach_radius'], sim['attach_z'], sim['payload_rest_z']) == \
        ('0.86', '0.225', '0.02', '0.02')


def test_the_figure8_has_its_own_size_knob():
    """Wesley 22 Oct: fig8_size sizes the figure-8 (0.9 in the twin: x +-0.9, y +-0.45 m; the rig's paths 0.75, inside
    the position limit), traj_radius the circle, spin and orbit; the legacy profile leaves it '' (traj_radius, as before
    22 Oct)."""
    for prof, size, dist in (('sim/mpc', '0.9', '1.0'), ('sim/dissipative', '0.9', '1.0'), ('sim/attach', '0.9', '1.0'),
                             ('real/mpc', '0.75', '0.75'), ('real/dissipative', '0.75', '0.75'),
                             ('real/attach', '0.75', '0.75')):
        a = profile_args(prof)
        assert (a['fig8_size'], a['traj_radius'], a['traj_distance']) == (size, '0.5', dist), prof
    assert profile_args('sim_legacy/mpc')['fig8_size'] == ''


def test_the_position_limit_is_on_for_the_rig_only():
    """pos_limit_m: 1.75 m on every rig profile (Wesley 22 Oct, after its critic and the LIM runs), off in the twin and
    legacy profiles (their claim tables fly paths past it)."""
    for name_, a in every_default():
        if 'pos_limit_m' in a:
            want = '1.75' if name_.startswith('real') else ''
            assert a['pos_limit_m'] == want, f'{name_}: pos_limit_m {a["pos_limit_m"]!r}'


def test_the_legacy_profile_is_the_sim_default_of_4_oct():
    """legacy:=true flies what sim.yaml held before the twin became the default: the x3
    plant of the worlds in old_worlds/ ('auto' is the secant gain of Gazebo's quadratic motor
    model at hover, thrust_model.py), so the runs flown there stay reproducible."""
    was = {'mpc': dict(num_drones='2', cable_len='0.5', drone_mass='0.64', attach_radius='', fig8_size='',
                       attach_z='', pivot_offset_z='0.0', ff_cap_force='0.0',
                       lift_ramp_vel='0.22', z_taut_gate='0.99', payload_rest_z='-0.1',
                       z_i_gate='', traj_speed='0.6', thrust_ratio='auto',
                       takeoff_thrust_ratio='auto', thrust_offset='0.0',
                       thrust_offset_v_slope='0.0', throttle_max='0.6', kt_trim='true',
                       reconfig_mode='network', auto_network_handover='true',
                       sim_thrust_map='linear', sim_thrust_offset='0.185', sim_pack_v0='24.4',
                       legacy='true'),
           'dissipative': dict(num_drones='4', attach_z='0.025', pivot_offset_z='',
                               throttle_max='', thrust_offset='', sim_thrust_map='',
                               sim_pack_v0=''),
           'network': dict(num_drones='3', sim_thrust_map='linear'),
           # 330/90/210 counter-clockwise then, typed clockwise since 15 Oct
           'attach': dict(num_drones='3', attach_azimuths_deg='30,270,150', start_taut='true',
                          handover_settle_s='0.75', creep_vel='0.1',
                          control_mode='velocity_after_handover', vel_ki='0.0',
                          pose_timeout_s='1.0', safety_ref_timeout_s='2.0', diss_ki_load='1.0')}
    for mode, want in was.items():
        got = profile_args(f'sim_legacy/{mode}')
        assert {k: got[k] for k in want} == want, mode


def test_the_twin_network_flies_the_rig_map_and_the_approved_network_values():
    """Card 2026-10-23_paper_method_compare (Wesley 22 Oct, push answers 21-23): the network flies the twin, with the
    rig's map and geometry as the twin dissipative mode, the automatic hand-over to the velocity loop (vel_ki 0), the
    trim 1.0, the planned 60 deg, no preview; every switch the critic has not seen is off."""
    sim, real = profile_args('sim/network'), profile_args('real/dissipative')
    for k in THRUST_MAP + RIG_GEOMETRY:
        if k not in ('reconfig_mode', 'auto_network_handover'):
            assert sim[k] == real[k], f'{k}: sim {sim[k]} vs rig {real[k]}'
    want = dict(num_drones='4', reconfig_mode='network', auto_network_handover='true',
                control_mode='velocity_after_handover', vel_ki='0.0', diss_ki_load='1.0', diss_elev_deg='60.0',
                cable_elev_deg='60.0', net_horizon_preview='false', handover_blend_s='0.0', legacy='false',
                net_twin_geometry='false', net_damp_relative='false', net_hold_carry='false', net_traj_lean='false',
                sim_thrust_map='rig')
    assert {k: sim[k] for k in want} == want
    for mode in ('mpc', 'dissipative', 'attach'):          # the switches are off in every other profile
        for side in ('sim', 'sim_legacy', 'real'):
            p = profile_args(f'{side}/{mode}')
            assert (p['net_damp_relative'], p['net_twin_geometry'], p['net_hold_carry']) == ('false',) * 3
    from bringup.profiles import MODES
    assert 'network' not in MODES['real']


def test_the_legacy_network_and_the_twin_attach_are_unchanged():
    """legacy:=true keeps the old network profile; attach flies the twin attach worlds since 8 Oct."""
    assert profile_args('sim_legacy/network')['legacy'] == 'true'
    assert profile_args('sim_legacy/network')['control_mode'] == 'mpc'
    a = profile_args('sim/attach')
    assert a['attach_azimuths_deg'] == '90,330,210' and a['legacy'] == 'false'     # typed clockwise: plates 3/11/7
    assert profile_args('sim_legacy/attach')['legacy'] == 'true'


def test_the_adaptive_kt_machinery_stays_gone():
    """kT is FIXED (supervisor-approved 2026-08-05, reaffirmed 2026-08-05 after the
    quadratic-inversion finding below). Two mechanisms used to move it underneath the
    tracker — a per-drone UKF and the thrust_quad_c airborne schedule — and both were
    removed because neither could be reasoned about from the value the launch file sets:
    `thrust_ratio:=20` provably changed nothing. If any of these names comes back in a
    launch, the number below stops meaning what it says.

    `thrust_quad_c` is on this list BY DECISION, not by oversight. Re-linearising kT on
    the last throttle is the exact algebraic inverse of Gazebo's quadratic motor model
    and it demonstrably removes the sim attach capsize (SIL R0022/R0023 vs R0015/R0016),
    but Wesley's decision is that the tracker flies one fixed number and nothing else.
    The sim/plant mismatch is to be fixed in the SIMULATOR, not by varying kT."""
    gone = ('thrust_quad_c', 'adaptive_thrust_ratio', 'adaptive_thrust_feedback',
            'thrust_ratio_estimator', 'kt_seed', 'kt_max_deviation',
            'kt_freeze_after_s', 'ukf_q_kt', 'ukf_rate_hz')
    for name_, a in every_default():
        for name in gone:
            assert name not in a, f'{name_} re-declares {name}'


def test_the_battery_derate_ships_disabled_everywhere():
    """The linear kT-vs-voltage derate is built but UNCALIBRATED — nobody has measured
    the sag fraction on this rig. Shipping it on would change every flight on the
    strength of a guessed number."""
    for name_, a in every_default():
        if 'kt_batt_sag_frac' in a:
            assert float(a['kt_batt_sag_frac']) == 0.0, (
                f'{name_} enables the battery derate by default')
            assert float(a['kt_batt_v_full']) > float(a['kt_batt_v_empty']), (
                f'{name_} has an inverted battery voltage range')


def test_terminal_vel_ref_defaults_to_the_historical_behaviour_everywhere():
    """T1 is an experiment. If a launch ever ships it enabled by default, every run
    silently includes an unmeasured change."""
    for name_, a in every_default():
        if 'terminal_vel_ref' in a:
            assert a['terminal_vel_ref'] == 'false', f"{name_} enables T1 by default"
