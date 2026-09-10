"""The derived sim kT must (a) reproduce the two MEASURED operating points and (b) stay
tied to the airframe SDF it is derived from."""
import re
from pathlib import Path

import pytest

from controller_quad_load.thrust_model import (thrust_c, secant_kt, hover_throttle,
                                               resolve_thrust_ratio, TAKEOFF_POP_FRAC)

REPO = Path(__file__).resolve().parents[3]
DRONE_SDF = REPO / 'simulation_assets' / 'models' / 'x3_drone0.sdf'


def test_thrust_c_matches_the_launch_comments_and_the_sil_plant():
    assert thrust_c() == pytest.approx(88.6, abs=0.1)


def test_thrust_c_is_derived_from_the_drone_sdf():
    if not DRONE_SDF.exists():
        pytest.skip('drone model not present')
    s = DRONE_SDF.read_text()
    mass = float(re.search(r'<mass>([\d.]+)</mass>', s).group(1))     # base link first
    mc = float(re.search(r'<motorConstant>([\d.e+-]+)</motorConstant>', s).group(1))
    w = float(re.search(r'<maxRotVelocity>([\d.]+)</maxRotVelocity>', s).group(1))
    assert thrust_c(mc, w, mass) == pytest.approx(thrust_c(), rel=1e-6)


def test_secant_reproduces_the_measured_0p4kg_operating_point():
    # SIL R0004/R0013: u_hover 0.371 at load 0.4 kg, 3 drones -> the 32.9 the launches used
    assert hover_throttle(0.4, 3) == pytest.approx(0.371, abs=0.003)
    assert secant_kt(0.4, 3) == pytest.approx(32.9, abs=0.3)


def test_secant_reproduces_the_measured_0p6kg_operating_point():
    # SIL R0228-R0230 (2026-09-10): u_hover 0.391 at 0.6 kg / 3 drones; kT 34.6 put the
    # load at 0.632 vs 0.782 with 32.9 (target 0.60)
    assert hover_throttle(0.6, 3) == pytest.approx(0.391, abs=0.003)
    assert secant_kt(0.6, 3) == pytest.approx(34.6, abs=0.3)


def test_kt_rises_with_load_and_falls_with_fleet_size():
    assert secant_kt(0.6, 3) > secant_kt(0.4, 3) > secant_kt(0.0, 3)
    assert secant_kt(0.6, 4) < secant_kt(0.6, 3)


def test_resolve_auto_and_explicit():
    kt, kt_to, note = resolve_thrust_ratio('auto', 'auto', 0.6, 3)
    assert kt == pytest.approx(34.6, abs=0.3) and kt_to == pytest.approx(kt * TAKEOFF_POP_FRAC)
    assert 'auto' in note
    kt, kt_to, note = resolve_thrust_ratio('32.9', '30.0', 0.6, 3)
    assert (kt, kt_to) == (32.9, 30.0) and 'explicit' in note
