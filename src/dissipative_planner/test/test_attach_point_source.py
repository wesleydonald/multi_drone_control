"""Sim U (card docs/experiments/2026-10-23_catch_offset.md): the newcomer folds in at its measured catch point (the
default, as flown: the body less a vertical arm) or, twin only, at the typed slot (attach_point_source slot)."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D, attach_point_slot  # noqa: E402
from mpc_planner.geometry import apex_direction  # noqa: E402
from test_partner_contact import _fold_in_node  # noqa: E402

SLOT = (0.0, 0.225)
BTT = 1.04                                  # QUAD4-like newcomer at 1 m: rod 1.0 from a pivot 4 cm under the body
CATCH = np.array([0.12, 0.225, 0.02])       # 12 cm off the slot along +x, 2 cm over the ring's origin plane


def _fold(slot_xy=None, stub_has_slot=True, catch=CATCH):
    f = _fold_in_node()
    f._partner_contact = False                     # our own twin approach
    f._attach_body_to_tip, f._attach_cable_len = BTT, 1.0
    f.attach_pos[0] = catch + np.array([0.0, 0.0, 1.0 + BTT])
    if stub_has_slot:
        f._slot_xy = None if slot_xy is None else np.asarray(slot_xy, float)
    assert f._attach_ocp(3) is True
    return f


def test_measured_default_folds_in_where_it_caught_as_before():
    f = _fold()
    assert np.allclose(f.rho[-1], CATCH)
    g = _fold_in_node()                            # bit-identical to the capture, as before the switch
    g._attach_body_to_tip = BTT
    assert np.array_equal(f.rho[-1], g._capture_weld_rho(CATCH + np.array([0.0, 0.0, 1.0 + BTT]), 3, clamp=False,
                                                         keep_z=True))
    assert f.cable_len_i[-1] == 1.0
    g = _fold(stub_has_slot=False)                 # a node built before the switch: the same fold-in
    assert np.allclose(g.rho[-1], f.rho[-1]) and np.allclose(g._s_nom[-1], f._s_nom[-1])
    assert g.blends == f.blends
    assert any(ln.startswith('[dissipative] ATTACH drone 3 (OCP resize)') and ln.endswith('measured catch point')
               for ln in f.log.lines)


def test_slot_folds_in_at_the_slot_through_the_incumbents_apex():
    f = _fold(SLOT)
    rho = np.array([SLOT[0], SLOT[1], 0.02])
    assert np.allclose(f.rho[-1], rho)
    assert np.allclose(f._s_nom[-1], apex_direction(f.rho[:-1], f._s_nom[:-1], rho))
    assert not np.allclose(f._s_nom[-1], _fold()._s_nom[-1])
    (ln,) = [ln for ln in f.log.lines if '] ATTACH drone 3 (OCP resize)' in ln]       # the runner's FOLD_IN
    assert ln.endswith('point (+0.0000,+0.2250,+0.0200) m, slot assumed')


def test_slot_keeps_the_captured_height():
    """Critic M2: the slot arm moves the point's xy only; z is the tip's, so the arms differ in the model point's xy."""
    catch = CATCH + np.array([0.0, 0.0, 0.025])                  # the tip at weld_point_z 0.045, not attach_z 0.02
    f, g = _fold(SLOT, catch=catch), _fold(catch=catch)
    assert np.allclose(f.rho[-1], [SLOT[0], SLOT[1], catch[2]])
    assert f.rho[-1][2] == g.rho[-1][2] and f.rho[-1][2] != f.attach_z


def test_the_switch_parses_and_refuses():
    assert attach_point_slot('measured', [float('nan')] * 2) is None
    assert attach_point_slot(' Slot ', SLOT).tolist() == list(SLOT)
    with pytest.raises(ValueError, match='measured or slot'):
        attach_point_slot('tip', SLOT)
    with pytest.raises(ValueError, match='attach_x_offset'):
        attach_point_slot('slot', [float('nan'), 0.225])


def test_resize_inputs_log_the_attach_points():
    lines = []
    f = SimpleNamespace(
        solver=SimpleNamespace(last_X=None), n=4, load_state=np.r_[0.0, 0.0, 1.0, 1.0, np.zeros(9)],
        hover_xy=(0.0, 0.0), traj_t=0.0, drone_pos=[np.zeros(3)] * 4, drone_vel=[np.zeros(3)] * 4,
        refs=SimpleNamespace(_t_nom=[1.0] * 4), rho=[np.array([0.1, 0.2, 0.02])] * 3 + [CATCH],
        get_logger=lambda: SimpleNamespace(info=lines.append, warn=lines.append))
    D._log_resize_inputs(f, 'attach', 3, [1.0] * 3, [np.zeros(3)] * 4, [1.0] * 4)
    (ln,) = lines
    j = json.loads(ln.split('RESIZE_INPUTS ', 1)[1])
    assert np.allclose(j['rho'][-1], CATCH) and len(j['rho']) == 4
