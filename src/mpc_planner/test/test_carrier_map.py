"""The RViz panel's carrier map (22 Oct rig: a release meant for the T's rider went to a pair quad, because the planner
matches drones to plates by position): each carrier's drone index, plate (clockwise, as typed) and planned pull."""
import json

import numpy as np

from mpc_planner.geometry import attach_points
from mpc_planner.planner_node import carrier_map_json


def test_the_t_on_plates_2_11_5_names_the_rider_by_plate_and_pull():
    typed_cw = [60.0, 330.0, 150.0]                     # plates 2, 11, 5 (Part C on 22 Oct)
    ang = [np.radians(-a) for a in typed_cw]            # the maths inside run counter-clockwise
    rho = [np.array([0.225 * np.cos(a), 0.225 * np.sin(a), 0.0175]) for a in ang]
    m = json.loads(carrier_map_json([1, 0, 2], rho, [0.12, 5.25, 5.15]))['carriers']
    assert [(c['drone'], c['plate']) for c in m] == [(1, 2), (0, 11), (2, 5)]
    assert min(m, key=lambda c: c['pull_n'])['drone'] == 1


def test_before_slots_or_references_there_is_no_pull():
    rho = attach_points(3, 0.225, 0.02)
    assert carrier_map_json(None, rho, None) is None
    m = json.loads(carrier_map_json([0, 1, 2], rho, None))['carriers']
    assert all(c['pull_n'] is None for c in m)
    assert [c['plate'] for c in m] == [0, 8, 4]         # attach_points' counter-clockwise 0/120/240 = plates 0/8/4
