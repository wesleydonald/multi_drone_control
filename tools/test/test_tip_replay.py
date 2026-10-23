"""tools/tip_replay.py's geometry and verdict (Wesley 22 Oct): the cable length, plate height and angles not typed are
the flight's own, from its planner params.json; a marker that never arms is no pass (exit 2). At 1 m the old 0.55 m
default left every marker unarmed and gate 3 printed WOULD-FIRES 0, exit 0, having tested nothing."""
import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import tip_replay as tr  # noqa: E402

Q0 = np.array([1.0, 0.0, 0.0, 0.0])
PARAMS = {'cable_len': 1.0, 'attach_z': 0.0175, 'attach_azimuths_deg': '30,210', 'num_drones': 2, 'target_z': 1.0,
          'pivot_offset': '[0.0, 0.0, -0.04]'}


def _flight(tmp_path, params=PARAMS):
    bag = tmp_path / 'abS1'
    bag.mkdir()
    d = tmp_path / 'abS1_logs' / 'logs' / 'dissipative_planner' / '20261028_091500'
    d.mkdir(parents=True)
    (d / 'params.json').write_text(json.dumps(params))
    return str(bag)


def _events(az_cw=(330.0, 150.0), cable=1.0, attach_z=0.0175):
    """The ring lifts to 1.0 and holds 3 s; each marker on its plate, each body at its rod's end at 60 deg; a little
    jitter on the ring and the markers, as mocap has (a bit-identical pose reads as a lost body)."""
    ev, t, btt = [], 0.0, cable + 0.04
    for k in range(400):
        t += 0.01
        z = min(1.0, 0.0275 + 0.01 * k)
        ev.append((t, 'ring', -1, np.array([0.0, 0.0, z + 1e-6 * (k % 2)]), (Q0, np.zeros(3))))   # never frozen
        for d, cw in enumerate(az_cw):
            u = np.array([math.cos(math.radians(-cw)), math.sin(math.radians(-cw)), 0.0])
            plate = np.array([0.0, 0.0, z + attach_z]) + 0.225 * u
            ev.append((t, 'drone', d, plate + btt * (0.5 * u + np.array([0.0, 0.0, math.sqrt(3.0) / 2.0])), None))
            ev.append((t, 'tip', d, plate + 1e-5 * ((k + d) % 3), None))
    return ev


def test_the_flights_params_give_the_untyped_geometry(tmp_path):
    bag = _flight(tmp_path)
    prm, src = tr.flight_params(bag)
    assert prm['cable_len'] == 1.0 and src.endswith('20261028_091500')
    a = tr.main.__globals__['argparse'].Namespace(bag=bag, azimuths=None, attach_z=None, cable_len=None, target_z=None)
    geo = tr.geometry(a, prm)
    assert (geo['cable_len'], geo['attach_z'], geo['azimuths'], geo['target_z'], geo['pivot_z']) == (
        1.0, 0.0175, [330.0, 150.0], 1.0, -0.04)
    a.cable_len, a.azimuths = 0.55, '0,180'                                     # typed values win
    assert (tr.geometry(a, prm)['cable_len'], tr.geometry(a, prm)['azimuths']) == (0.55, [0.0, 180.0])
    assert tr.flight_params(str(tmp_path / 'nothing')) == (None, None)
    with pytest.raises(SystemExit, match='--cable-len'):
        tr.geometry(tr.main.__globals__['argparse'].Namespace(bag='x', azimuths='0,180', attach_z=0.0175,
                                                               cable_len=None, target_z=None), None)


def _verdict(monkeypatch, tmp_path, *argv):
    monkeypatch.setattr(tr, 'read', lambda bag: _events())
    with pytest.raises(SystemExit) as e:
        tr.main([_flight(tmp_path), *argv])
    return e.value.code


def test_a_marker_that_never_arms_is_no_pass(monkeypatch, tmp_path, capsys):
    assert _verdict(monkeypatch, tmp_path) == 0                                  # the flight's 1 m: both armed
    out = capsys.readouterr().out
    assert 'drone 0: armed' in out and 'drone 1: armed' in out


def test_the_wrong_cable_length_leaves_every_marker_unarmed_and_says_so(monkeypatch, tmp_path, capsys):
    assert _verdict(monkeypatch, tmp_path, '--cable-len', '0.55') == 2           # the 0.55 m default on 1 m cables
    assert 'NOT A PASS' in capsys.readouterr().out
