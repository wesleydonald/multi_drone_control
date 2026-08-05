"""Tests for the configuration tools (finding F9).

`check_geometry.py` is a validator, and a validator that reports a mismatch which is
not real is worse than no validator — it gets ignored, and then it is not there when
it matters. The first version of it did exactly that: it read `attach_z` in the
lift_system frame instead of relative to the payload CoG, so every elevated world
(payload at 0.6 m) looked like `attach_z = 0.625` against a config value of 0.025.
These tests pin the frame convention down.

    python3 -m pytest src/controller_quad_load/test/test_config_tools.py -v
"""
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tools'))

from check_geometry import world_geometry, planner_defaults, launch_defaults  # noqa: E402
from param_diff import launch_args                                            # noqa: E402

WORLDS = REPO / 'simulation_assets'
LAUNCHES = REPO / 'src/controller_quad_load/launch'


def ground_worlds():
    return [WORLDS / n for n in
            ('two_rigid_ground.sdf', 'three_rigid_ground.sdf', 'four_rigid_ground.sdf')
            if (WORLDS / n).exists()]


# ── geometry extraction ──────────────────────────────────────────────────────

@pytest.mark.parametrize('sdf', ground_worlds(), ids=lambda p: p.name)
def test_ground_worlds_match_the_planner_defaults(sdf):
    """The worlds actually flown must agree with the config the planner uses, or
    every drone's tension feedforward is sized wrongly."""
    g = world_geometry(sdf)
    d = planner_defaults()
    for k in ('cable_len', 'attach_radius', 'attach_z', 'load_mass'):
        assert abs(g[k] - d[k]) < 1e-3, (
            f"{sdf.name}: {k} world={g[k]:.4f} vs params.py={d[k]:.4f}")


def test_attach_z_is_relative_to_the_payload_not_the_world():
    """The bug this file exists for. `four_rigid` hangs its payload at z=0.6 and
    `four_rigid_ground` at z=0.025; attach_z is defined above the load CoG, so both
    must report the SAME value. Reading the raw pose gives 0.625 vs 0.05."""
    elevated = WORLDS / 'four_rigid.sdf'
    grounded = WORLDS / 'four_rigid_ground.sdf'
    if not (elevated.exists() and grounded.exists()):
        pytest.skip('worlds not present')
    a = world_geometry(elevated)['attach_z']
    b = world_geometry(grounded)['attach_z']
    assert abs(a - b) < 1e-3, (
        f"attach_z differs between an elevated and a grounded world ({a:.4f} vs "
        f"{b:.4f}) — it is being read in the wrong frame")


def test_drone_count_is_extracted():
    assert world_geometry(WORLDS / 'three_rigid_ground.sdf')['num_drones'] == 3
    assert world_geometry(WORLDS / 'four_rigid_ground.sdf')['num_drones'] == 4


def test_non_uniform_attach_ring_is_flagged():
    """The randomly-generated worlds have unequal attach radii, which the planner's
    single `attach_radius` cannot represent. That must not pass silently."""
    rnd = WORLDS / 'three_random_seed1.sdf'
    if not rnd.exists():
        pytest.skip('random world not present')
    assert '_warn_ring' in world_geometry(rnd)


# ── launch argument parsing ──────────────────────────────────────────────────

def test_launch_args_are_parsed_without_executing_the_launch():
    a = launch_args(LAUNCHES / 'mpc_quad_load_launch.py')
    assert 'thrust_ratio' in a and 'load_mass' in a
    assert a['thrust_ratio'] == '32.9'


def test_sim_and_real_thrust_settings_still_differ():
    """thrust_ratio 32.9 in sim vs 24.0 on hardware. If these ever converge by
    accident, one of them is wrong — Gazebo's motor model is QUADRATIC
    (a = 88.6*u^2), so a linear kT there must be the secant gain at hover
    (88.6 * the 0.371 measured hover throttle = 32.9); the real airframe
    measures 24 at full battery health."""
    sim = launch_args(LAUNCHES / 'mpc_quad_load_launch.py')
    real = launch_args(LAUNCHES / 'real_control_launch.py')
    assert sim['thrust_ratio'] != real['thrust_ratio']
    assert float(real['thrust_ratio']) == 24.0


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
    for lf in LAUNCHES.glob('*launch.py'):
        a = launch_args(lf)
        for name in gone:
            assert name not in a, f'{lf.name} re-declares {name}'


def test_the_battery_derate_ships_disabled_everywhere():
    """The linear kT-vs-voltage derate is built but UNCALIBRATED — nobody has measured
    the sag fraction on this rig. Shipping it on would change every flight on the
    strength of a guessed number."""
    for lf in LAUNCHES.glob('*launch.py'):
        a = launch_args(lf)
        if 'kt_batt_sag_frac' in a:
            assert float(a['kt_batt_sag_frac']) == 0.0, (
                f'{lf.name} enables the battery derate by default')
            assert float(a['kt_batt_v_full']) > float(a['kt_batt_v_empty']), (
                f'{lf.name} has an inverted battery voltage range')


def test_terminal_vel_ref_defaults_to_the_historical_behaviour_everywhere():
    """T1 is an experiment. If a launch ever ships it enabled by default, every run
    silently includes an unmeasured change."""
    for lf in LAUNCHES.glob('*launch.py'):
        a = launch_args(lf)
        if 'terminal_vel_ref' in a:
            assert a['terminal_vel_ref'] == 'false', f"{lf.name} enables T1 by default"
