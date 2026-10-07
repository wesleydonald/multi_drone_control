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
            (('sim', ('mpc', 'free_hover', 'dissipative')),
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
RIG_GEOMETRY = ('cable_len', 'drone_mass', 'attach_radius', 'pivot_offset_z', 'rod_tol_frac',
                'rod_spread_m', 'lift_ramp_vel', 'z_taut_gate', 'z_i_gate', 'reconfig_mode',
                'auto_network_handover')


@pytest.mark.parametrize('mode', ['mpc', 'dissipative'])
def test_the_twin_flies_the_rig_thrust_map_and_geometry(mode):
    """The sim default is the rig twin (5 Oct 2026): the identified affine map (gain 35.2
    above the 0.185 offset, 30 Sep 2026) and the rig's geometry, never a derived kT."""
    sim, real = profile_args(f'sim/{mode}'), profile_args(f'real/{mode}')
    for k in THRUST_MAP + RIG_GEOMETRY:
        assert sim[k] == real[k], f'{k}: sim {sim[k]} vs rig {real[k]}'
    assert float(real['thrust_ratio']) == 35.2
    assert (sim['sim_thrust_map'], sim['sim_pack_v0'], sim['legacy']) == ('rig', '23.8', 'false')


def test_the_legacy_profile_is_the_sim_default_of_4_oct():
    """legacy:=true flies what sim.yaml held before the twin became the default: the x3
    plant of the worlds in old_worlds/ ('auto' is the secant gain of Gazebo's quadratic motor
    model at hover, thrust_model.py), so the runs flown there stay reproducible."""
    was = {'mpc': dict(num_drones='2', cable_len='0.5', drone_mass='0.64', attach_radius='',
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
           'attach': dict(num_drones='3', attach_azimuths_deg='330,90,210', start_taut='true',
                          handover_settle_s='0.75', creep_vel='0.1',
                          control_mode='velocity_after_handover', vel_ki='0.0',
                          pose_timeout_s='1.0', safety_ref_timeout_s='2.0', diss_ki_load='1.0')}
    for mode, want in was.items():
        got = profile_args(f'sim_legacy/{mode}')
        assert {k: got[k] for k in want} == want, mode


@pytest.mark.parametrize('mode', ['attach', 'network'])
def test_attach_and_network_are_refused_on_the_twin(mode):
    """No rig-twin attach world, and those graphs do not pass the rig thrust map yet."""
    with pytest.raises(RuntimeError, match='legacy:=true'):
        profile_args(f'sim/{mode}')
    assert profile_args(f'sim_legacy/{mode}')['legacy'] == 'true'


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
