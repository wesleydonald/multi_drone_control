"""partner_rod_add_m (11 Nov, docs/analysis/quad4_bounce_2026-11-11.md): a typed length added to a start-welded QUAD4's
hand-over rod reading, only when that reading was applied; off (0) changes nothing."""
from types import SimpleNamespace

from dissipative_planner.dissipative_node import DissipativeController
from mpc_planner.planner_node import LoadPlanner


def _node(monkeypatch, add, measured, slot2drone=(1, 0, 2, 3)):
    f = object.__new__(DissipativeController)
    lines, geo = [], []
    log = SimpleNamespace(warn=lines.append, info=lines.append)
    f.get_logger = lambda: log
    f.lines, f.geo = lines, geo
    f.n, f._n_carry0, f.slot2drone = len(slot2drone), 3, list(slot2drone)
    f.cable_len_i = [1.0, 1.0, 1.0, 0.994][:len(slot2drone)]
    f._partner_rod_add = add
    f.solver = SimpleNamespace(set_geometry=lambda cable_lengths=None, rho=None: geo.append(list(cable_lengths)))

    def base(self):
        if measured is not None:
            self.cable_len_i = list(measured)
    monkeypatch.setattr(LoadPlanner, '_apply_measured_rod_lengths', base)
    return f


def test_the_typed_length_is_added_to_quad4s_reading_only(monkeypatch):
    f = _node(monkeypatch, 0.010, [1.011, 1.011, 1.018, 1.021])
    DissipativeController._apply_measured_rod_lengths(f)
    assert f.cable_len_i[:3] == [1.011, 1.011, 1.018] and abs(f.cable_len_i[3] - 1.031) < 1e-9
    assert len(f.geo) == 1 and abs(f.geo[0][3] - 1.031) < 1e-9
    assert any('partner_rod_add_m' in ln and '+10 mm' in ln for ln in f.lines)


def test_off_or_a_refused_reading_or_no_welded_quad4_changes_nothing(monkeypatch):
    f = _node(monkeypatch, 0.0, [1.011, 1.011, 1.018, 1.021])
    DissipativeController._apply_measured_rod_lengths(f)
    assert f.cable_len_i == [1.011, 1.011, 1.018, 1.021] and not f.geo
    f = _node(monkeypatch, 0.010, None)                     # the hand-over reading was refused: the typed lengths stay
    DissipativeController._apply_measured_rod_lengths(f)
    assert f.cable_len_i == [1.0, 1.0, 1.0, 0.994] and not f.geo
    f = _node(monkeypatch, 0.010, [1.011, 1.011, 1.018], slot2drone=(1, 0, 2))   # the triangle alone: no QUAD4 in the fleet
    DissipativeController._apply_measured_rod_lengths(f)
    assert f.cable_len_i == [1.011, 1.011, 1.018] and not f.geo
