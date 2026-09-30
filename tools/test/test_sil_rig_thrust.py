"""SIL plant under thrust_map 'rig': the rig's affine law and pack sag, the same module the
Gazebo bridge runs (simulation_communication/rig_thrust.py), and the linear default left alone."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sil.plant import G, Link, QuadParams, SilPlant, rig_thrust  # noqa: E402
from sil.scenario import Scenario  # noqa: E402
from sil.standin import ApproachStandin  # noqa: E402


def _free(qp, pack=None):
    plant = SilPlant([qp], [Link(np.zeros(3), 0.5, attached=False)], pack=pack)
    plant.reset([[0.0, 0.0, 1.0]], [0.0, 0.0, -10.0])
    return plant


def _fly(plant, u, seconds, dt=0.001):
    plant.set_command(0, 0.0, 0.0, 2 * u - 1, 0.0, True)
    for _ in range(int(round(seconds / dt))):
        plant.step(dt)


def test_linear_default_unchanged():
    qp = QuadParams()
    plant = _free(qp)
    assert qp.thrust_map == 'linear' and plant.packs == [None]
    _fly(plant, 0.5, 0.01)
    assert plant.imu(0)[2] == pytest.approx(qp.thrust_c * 0.5)
    assert np.isnan(plant.pack_v(0))


def test_rig_hover_throttle_holds_at_v_ref():
    qp = QuadParams(mass=0.55, thrust_map='rig', thrust_offset=0.185)
    # no sag, no drain: the pack sits at V_REF, so this is the law alone
    plant = _free(qp, pack={'v0': rig_thrust.V_REF, 'r_sag': 0.0, 'drain': 0.0})
    u = 0.185 + 0.507 * 0.55
    _fly(plant, u, 2.0)
    assert plant.imu(0)[2] == pytest.approx(G, rel=0.01)
    assert abs(plant.p[0][2] - 1.0) < 0.01


def test_rig_zero_thrust_below_offset():
    plant = _free(QuadParams(mass=0.55, thrust_map='rig'))
    _fly(plant, 0.15, 0.01)
    assert plant.imu(0)[2] == 0.0


def test_rig_pack_sags_and_drains():
    qp = QuadParams(mass=0.55, thrust_map='rig')
    plant = _free(qp, pack={'v0': 24.4})
    _fly(plant, 0.46, 20.0, dt=0.005)
    v = plant.pack_v(0)
    assert v < 24.4 - 0.46 * 0.8                  # sag after 4 tau
    assert plant.packs[0].ocv() == pytest.approx(24.4 - 0.022 * 0.46 * 20.0, rel=1e-3)
    assert plant.pack_v_measured(0) == pytest.approx(round(v, 1))
    # thrust follows the sagged pack: less than at the starting voltage
    assert plant.imu(0)[2] * 0.55 == pytest.approx(rig_thrust.rig_thrust(0.46, v), rel=1e-3)


def test_throttle_for_accel_inverts_both_maps():
    for qp in (QuadParams(), QuadParams(mass=0.55, thrust_map='rig', thrust_offset=0.187)):
        plant = _free(qp)
        u = plant.throttle_for_accel(0, G)
        _fly(plant, u, 0.001)
        assert plant.imu(0)[2] == pytest.approx(G, rel=1e-3)


def test_standin_uses_the_plant_inverse():
    plant = _free(QuadParams(mass=0.55, thrust_map='rig'))
    s = plant.drone_state(0)
    ch = ApproachStandin().channels(s[0:3], s[7:10], s[3:7], s[0:3],
                                    throttle_fn=lambda a: plant.throttle_for_accel(0, a))
    assert (ch[2] + 1) / 2 == pytest.approx(plant.throttle_for_accel(0, G))


def test_unknown_map_rejected():
    with pytest.raises(ValueError):
        _free(QuadParams(thrust_map='quadratic'))


def test_scenario_plant_section(tmp_path):
    p = tmp_path / 's.yaml'
    p.write_text('name: x\nfleet: {n_tethered: 3, n_total: 3}\n'
                 'plant: {thrust_map: rig, thrust_offset: 0.181, pack_v0: 22.8}\n')
    with pytest.raises(ValueError, match='drone_mass'):
        Scenario.from_yaml(str(p))
    p.write_text(p.read_text() + 'geometry: {drone_mass: 0.55}\n')
    s = Scenario.from_yaml(str(p))
    assert (s.thrust_map, s.thrust_offset, s.pack_v0, s.drone_mass) == ('rig', 0.181, 22.8, 0.55)
    p.write_text('name: x\nfleet: {n_tethered: 3, n_total: 3}\n')
    assert Scenario.from_yaml(str(p)).thrust_map == 'linear'
