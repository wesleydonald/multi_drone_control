"""measure_rod_len guard: the taut measurement replaces the typed cable_len only when it
is believable -- every rod within +-15 % of the typed value and the rods agreeing with
each other -- so a slack or mis-assigned rod cannot shrink the geometry (critic, card
2026-09-23_geometry_autocal.md)."""
import pytest
from controller_load_mpc.planner_node import measured_rod_lengths


def test_a_4cm_short_rig_rod_is_accepted():
    lens, why = measured_rod_lengths(0.50, [0.46, 0.462, 0.458])
    assert why is None and lens == [0.46, 0.462, 0.458]


def test_equal_rods_equal_to_typed_are_a_no_op():
    lens, why = measured_rod_lengths(0.50, [0.50, 0.50, 0.50])
    assert why is None and lens == [0.50, 0.50, 0.50]


def test_one_slack_or_misassigned_rod_keeps_the_typed_value():
    lens, why = measured_rod_lengths(0.50, [0.50, 0.50, 0.41])      # 18 % short
    assert lens is None and 'rod 2' in why


def test_disagreeing_rods_keep_the_typed_value():
    lens, why = measured_rod_lengths(0.50, [0.46, 0.50, 0.48])      # 4 cm spread
    assert lens is None and 'disagree' in why


def test_no_drones():
    assert measured_rod_lengths(0.50, [])[0] is None
